import json
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover - nav2 may be installed after base package build
    NavigateToPose = None


class MissionPhase(Enum):
    IDLE = "idle"
    SEARCHING = "searching"
    APPROACHING = "approaching"
    PICKING = "picking"
    NAVIGATING = "navigating"
    DROPPING = "dropping"
    DONE = "done"
    FAULT = "fault"


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__("mission_manager_node")
        self.declare_parameter("task_topic", "/mission/task")
        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("target_poses_topic", "/perception/target_poses")
        self.declare_parameter("manual_cmd_topic", "/manual_cmd")
        self.declare_parameter("fork_cmd_topic", "/fork/cmd")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("navigate_action", "/navigate_to_pose")
        self.declare_parameter("default_task_json", "{}")
        self.declare_parameter("search_commands", "rotate_left,rotate_left,stop")
        self.declare_parameter("approach_command", "forward")
        self.declare_parameter("approach_cycles", 3)
        self.declare_parameter("search_step_sec", 0.7)
        self.declare_parameter("approach_step_sec", 0.5)
        self.declare_parameter("fork_action_sec", 1.0)
        self.declare_parameter("target_timeout_sec", 8.0)
        self.declare_parameter("use_nav2_action", True)
        self.declare_parameter("publish_manual_cmd", True)

        self.task_topic = str(self.get_parameter("task_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.search_commands = [
            item.strip() for item in str(self.get_parameter("search_commands").value).split(",")
            if item.strip()
        ]
        self.approach_command = str(self.get_parameter("approach_command").value)
        self.approach_cycles = int(self.get_parameter("approach_cycles").value)
        self.search_step_sec = float(self.get_parameter("search_step_sec").value)
        self.approach_step_sec = float(self.get_parameter("approach_step_sec").value)
        self.fork_action_sec = float(self.get_parameter("fork_action_sec").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.use_nav2_action = bool(self.get_parameter("use_nav2_action").value)
        self.publish_manual_cmd = bool(self.get_parameter("publish_manual_cmd").value)

        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.manual_pub = self.create_publisher(
            String, str(self.get_parameter("manual_cmd_topic").value), 10
        )
        self.fork_pub = self.create_publisher(
            String, str(self.get_parameter("fork_cmd_topic").value), 10
        )
        self.goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_pose_topic").value), 10
        )
        self.create_subscription(String, self.task_topic, self.on_task, 10)
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("target_poses_topic").value),
            self.on_target_poses,
            10,
        )
        self.create_service(Trigger, "/mission/start", self.on_start)
        self.create_service(Trigger, "/mission/cancel", self.on_cancel)

        self.nav_client = None
        if NavigateToPose is not None:
            self.nav_client = ActionClient(
                self,
                NavigateToPose,
                str(self.get_parameter("navigate_action").value),
            )

        self.phase = MissionPhase.IDLE
        self.task = {}
        self.phase_started = self.get_clock().now()
        self.last_target_time = None
        self.search_index = 0
        self.approach_count = 0
        self.nav_goal_sent = False
        self.nav_goal_handle = None
        self.timer = self.create_timer(0.1, self.on_timer)
        self.publish_state("idle: waiting for /mission/start or /mission/task")

    def on_start(self, request, response):
        del request
        default_task = str(self.get_parameter("default_task_json").value)
        try:
            task = json.loads(default_task) if default_task.strip() else {}
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = f"default_task_json invalid: {exc}"
            return response
        self.start_task(task)
        response.success = True
        response.message = "mission started"
        return response

    def on_cancel(self, request, response):
        del request
        self.stop_all()
        self.phase = MissionPhase.IDLE
        self.publish_state("cancelled: mission stopped")
        response.success = True
        response.message = "mission cancelled"
        return response

    def on_task(self, msg):
        try:
            task = json.loads(msg.data) if msg.data.strip() else {}
        except json.JSONDecodeError as exc:
            self.fault(f"invalid mission task JSON: {exc}")
            return
        self.start_task(task)

    def start_task(self, task):
        self.task = task
        self.search_index = 0
        self.approach_count = 0
        self.nav_goal_sent = False
        self.nav_goal_handle = None
        self.phase = MissionPhase.SEARCHING
        self.phase_started = self.get_clock().now()
        self.publish_state(f"searching: task={json.dumps(task, ensure_ascii=False)}")

    def on_target_poses(self, msg):
        if not msg.poses:
            return
        self.last_target_time = self.get_clock().now()
        if self.phase == MissionPhase.SEARCHING:
            self.phase = MissionPhase.APPROACHING
            self.phase_started = self.get_clock().now()
            self.approach_count = 0
            self.publish_state("approaching: target pose acquired")

    def publish_state(self, text):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)
        self.get_logger().info(text)

    def publish_string(self, pub, text):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def send_manual(self, command):
        if self.publish_manual_cmd:
            self.publish_string(self.manual_pub, command)

    def send_fork(self, command):
        self.publish_string(self.fork_pub, command)

    def stop_all(self):
        self.send_manual("stop")
        self.send_fork("stop")

    def elapsed(self):
        return (self.get_clock().now() - self.phase_started).nanoseconds * 1e-9

    def target_expired(self):
        if self.last_target_time is None:
            return True
        age = (self.get_clock().now() - self.last_target_time).nanoseconds * 1e-9
        return age > self.target_timeout_sec

    def fault(self, text):
        self.stop_all()
        self.phase = MissionPhase.FAULT
        self.publish_state(f"fault: {text}")

    def delivery_goal(self):
        goal = self.task.get("delivery_pose") or self.task.get("goal_pose")
        if not isinstance(goal, dict):
            return None
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = str(goal.get("frame_id", "map"))
        pose.pose.position.x = float(goal.get("x", 0.0))
        pose.pose.position.y = float(goal.get("y", 0.0))
        pose.pose.position.z = float(goal.get("z", 0.0))
        pose.pose.orientation.z = float(goal.get("qz", 0.0))
        pose.pose.orientation.w = float(goal.get("qw", 1.0))
        return pose

    def send_nav_goal(self, pose):
        self.goal_pub.publish(pose)
        if not self.use_nav2_action:
            return False
        if self.nav_client is None:
            self.publish_state("navigating: nav2_msgs unavailable; published /goal_pose only")
            return False
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.publish_state("navigating: Nav2 action unavailable; published /goal_pose only")
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_nav_goal_response)
        return True

    def on_nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.fault("Nav2 goal rejected")
            return
        self.nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        if self.phase != MissionPhase.NAVIGATING:
            return
        _ = future.result()
        self.phase = MissionPhase.DROPPING
        self.phase_started = self.get_clock().now()
        self.send_fork("down")
        self.publish_state("dropping: Nav2 reached delivery goal")

    def on_timer(self):
        if self.phase in (MissionPhase.IDLE, MissionPhase.DONE, MissionPhase.FAULT):
            return

        if self.phase == MissionPhase.SEARCHING:
            if self.elapsed() < self.search_step_sec:
                return
            self.phase_started = self.get_clock().now()
            command = self.search_commands[self.search_index % len(self.search_commands)]
            self.search_index += 1
            self.send_manual(command)
            self.publish_state(f"searching: {command}")
            return

        if self.phase == MissionPhase.APPROACHING:
            if self.target_expired():
                self.phase = MissionPhase.SEARCHING
                self.phase_started = self.get_clock().now()
                self.publish_state("searching: target expired")
                return
            if self.elapsed() < self.approach_step_sec:
                return
            self.phase_started = self.get_clock().now()
            if self.approach_count >= self.approach_cycles:
                self.stop_all()
                self.phase = MissionPhase.PICKING
                self.phase_started = self.get_clock().now()
                self.send_fork("up")
                self.publish_state("picking: fork up")
                return
            self.approach_count += 1
            self.send_manual(self.approach_command)
            self.publish_state(f"approaching: {self.approach_command} {self.approach_count}/{self.approach_cycles}")
            return

        if self.phase == MissionPhase.PICKING:
            if self.elapsed() < self.fork_action_sec:
                return
            self.send_fork("stop")
            goal = self.delivery_goal()
            if goal is None:
                self.phase = MissionPhase.DONE
                self.publish_state("done: picked target; no delivery goal configured")
                return
            self.phase = MissionPhase.NAVIGATING
            self.phase_started = self.get_clock().now()
            self.nav_goal_sent = True
            action_sent = self.send_nav_goal(goal)
            suffix = "Nav2 action sent" if action_sent else "/goal_pose published"
            self.publish_state(f"navigating: {suffix}")
            return

        if self.phase == MissionPhase.NAVIGATING:
            if self.nav_goal_handle is None and self.elapsed() > 2.0:
                self.phase = MissionPhase.DROPPING
                self.phase_started = self.get_clock().now()
                self.send_fork("down")
                self.publish_state("dropping: action unavailable fallback")
            return

        if self.phase == MissionPhase.DROPPING:
            if self.elapsed() < self.fork_action_sec:
                return
            self.send_fork("stop")
            self.phase = MissionPhase.DONE
            self.publish_state("done: delivery sequence complete")


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_all()
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
