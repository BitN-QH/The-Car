import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def parse_route(route_text):
    route = []
    for item in route_text.split(","):
        command = item.strip().lower()
        if command:
            route.append(command)
    return route


class SegmentedNavNode(Node):
    def __init__(self):
        super().__init__("segmented_nav_node")
        self.declare_parameter(
            "route",
            "forward,left,rotate_right,forward,stop",
        )
        self.declare_parameter("manual_cmd_topic", "/manual_cmd")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("fallback_global_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("segment_duration_sec", 0.35)
        self.declare_parameter("settle_duration_sec", 0.8)
        self.declare_parameter("max_pose_jump_m", 0.8)
        self.declare_parameter("max_yaw_jump_rad", 1.2)
        self.declare_parameter("autostart", True)

        route_text = str(self.get_parameter("route").value)
        self.route = parse_route(route_text)
        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        self.global_frame = str(self.get_parameter("global_frame").value)
        self.fallback_global_frame = str(self.get_parameter("fallback_global_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.segment_duration_sec = float(self.get_parameter("segment_duration_sec").value)
        self.settle_duration_sec = float(self.get_parameter("settle_duration_sec").value)
        self.max_pose_jump_m = float(self.get_parameter("max_pose_jump_m").value)
        self.max_yaw_jump_rad = float(self.get_parameter("max_yaw_jump_rad").value)
        self.autostart = bool(self.get_parameter("autostart").value)

        self.cmd_pub = self.create_publisher(String, self.manual_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, "/mission/state", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.index = 0
        self.phase = "idle"
        self.phase_started = self.get_clock().now()
        self.last_pose = None
        self.active_command = ""
        self.timer = self.create_timer(0.1, self.on_timer)

        if self.autostart:
            self.phase = "settle"
            self.publish_status("starting segmented route")
        self.get_logger().info(f"segmented_nav_node route={self.route}")

    def publish_manual(self, command):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def lookup_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            frame = self.global_frame
        except TransformException:
            transform = self.tf_buffer.lookup_transform(
                self.fallback_global_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            frame = self.fallback_global_frame

        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        t = transform.transform.translation
        return frame, float(t.x), float(t.y), yaw

    def pose_jump_detected(self, pose):
        if self.last_pose is None:
            return False
        _, x0, y0, yaw0 = self.last_pose
        _, x1, y1, yaw1 = pose
        dist = math.hypot(x1 - x0, y1 - y0)
        dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        return dist > self.max_pose_jump_m or abs(dyaw) > self.max_yaw_jump_rad

    def stop_for_fault(self, reason):
        self.publish_manual("stop")
        self.phase = "fault"
        self.publish_status(f"fault: {reason}")
        self.get_logger().error(reason)

    def on_timer(self):
        now = self.get_clock().now()

        if self.phase in ("done", "fault", "idle"):
            return

        try:
            pose = self.lookup_pose()
        except TransformException as exc:
            self.stop_for_fault(f"TF unavailable: {exc}")
            return

        if self.pose_jump_detected(pose):
            self.stop_for_fault("pose jump detected; stopping segmented route")
            return
        self.last_pose = pose

        elapsed = (now - self.phase_started).nanoseconds * 1e-9

        if self.phase == "settle":
            if elapsed < self.settle_duration_sec:
                return
            if self.index >= len(self.route):
                self.publish_manual("stop")
                self.phase = "done"
                self.publish_status("done")
                return
            self.active_command = self.route[self.index]
            self.index += 1
            self.publish_manual(self.active_command)
            self.phase = "move"
            self.phase_started = now
            self.publish_status(f"move: {self.active_command}")
            return

        if self.phase == "move" and elapsed >= self.segment_duration_sec:
            self.publish_manual("stop")
            self.phase = "settle"
            self.phase_started = now
            self.publish_status(f"stop after: {self.active_command}")


def main(args=None):
    rclpy.init(args=args)
    node = SegmentedNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_manual("stop")
    finally:
        node.publish_manual("stop")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
