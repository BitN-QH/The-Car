import json
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionState(Enum):
    WAITING = "waiting"
    APPROACHING = "approaching"
    ROTATING = "rotating"
    RETURNING = "returning"
    DONE = "done"


COMMAND_TO_MANUAL = {
    "向前": "forward",
    "向后": "back",
    "向左": "left",
    "向右": "right",
    "原地旋转": "rotate_left",
}


class PersonFollowMissionNode(Node):
    """Terminal-simulated person approach mission driven by YOLO detections."""

    def __init__(self):
        super().__init__("person_follow_mission_node")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("manual_cmd_topic", "/manual_cmd")
        self.declare_parameter("terminal_cmd_topic", "/mission/terminal_cmd")
        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("target_class_name", "person")
        self.declare_parameter("score_threshold", 0.35)
        self.declare_parameter("control_period_sec", 0.25)
        self.declare_parameter("center_deadband_ratio", 0.12)
        self.declare_parameter("require_centered_to_advance", True)
        self.declare_parameter("close_height_ratio", 0.72)
        self.declare_parameter("close_area_ratio", 0.30)
        self.declare_parameter("target_height_ratio", 0.62)
        self.declare_parameter("target_area_ratio", 0.22)
        self.declare_parameter("size_deadband_ratio", 0.08)
        self.declare_parameter("close_distance_m", 0.75)
        self.declare_parameter("rotate_180_sec", 3.2)
        self.declare_parameter("min_return_sec", 1.0)
        self.declare_parameter("max_return_sec", 8.0)
        self.declare_parameter("lost_timeout_sec", 1.0)
        self.declare_parameter("publish_manual_cmd", True)
        self.declare_parameter("repeat_same_terminal_cmd", False)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        self.terminal_cmd_topic = str(self.get_parameter("terminal_cmd_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value)
        self.score_threshold = float(self.get_parameter("score_threshold").value)
        self.control_period_sec = float(self.get_parameter("control_period_sec").value)
        self.center_deadband_ratio = float(self.get_parameter("center_deadband_ratio").value)
        self.require_centered_to_advance = bool(self.get_parameter("require_centered_to_advance").value)
        self.close_height_ratio = float(self.get_parameter("close_height_ratio").value)
        self.close_area_ratio = float(self.get_parameter("close_area_ratio").value)
        self.target_height_ratio = float(self.get_parameter("target_height_ratio").value)
        self.target_area_ratio = float(self.get_parameter("target_area_ratio").value)
        self.size_deadband_ratio = float(self.get_parameter("size_deadband_ratio").value)
        self.close_distance_m = float(self.get_parameter("close_distance_m").value)
        self.rotate_180_sec = float(self.get_parameter("rotate_180_sec").value)
        self.min_return_sec = float(self.get_parameter("min_return_sec").value)
        self.max_return_sec = float(self.get_parameter("max_return_sec").value)
        self.lost_timeout_sec = float(self.get_parameter("lost_timeout_sec").value)
        self.publish_manual_cmd_enabled = bool(self.get_parameter("publish_manual_cmd").value)
        self.repeat_same_terminal_cmd = bool(self.get_parameter("repeat_same_terminal_cmd").value)

        self.manual_pub = self.create_publisher(String, self.manual_cmd_topic, 10)
        self.terminal_pub = self.create_publisher(String, self.terminal_cmd_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.det_sub = self.create_subscription(String, self.detections_topic, self.on_detections, 10)
        self.timer = self.create_timer(max(self.control_period_sec, 0.05), self.on_timer)

        self.state = MissionState.WAITING
        self.last_detection = None
        self.last_detection_time = None
        self.state_started_at = self.get_clock().now()
        self.approach_forward_sec = 0.0
        self.return_duration_sec = 0.0
        self.last_terminal_command = ""

        self.publish_state("等待 YOLO 识别 person")
        self.get_logger().info(
            "person_follow_mission_node ready: center person first, then size controls 向前/向后, "
            "near -> 原地旋转 180 -> 向前返回起点; STM32 serial is not used"
        )

    def on_detections(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"invalid detections JSON: {exc}")
            return

        detection = self.best_person_detection(payload)
        if detection is None:
            return
        self.last_detection = detection
        self.last_detection_time = self.get_clock().now()
        if self.state == MissionState.WAITING:
            self.transition(MissionState.APPROACHING, "识别到 person，开始接近")

    def best_person_detection(self, payload):
        width = float(payload.get("image_width", 0) or 0)
        height = float(payload.get("image_height", 0) or 0)
        if width <= 0.0 or height <= 0.0:
            return None

        best = None
        best_score = -1.0
        for detection in payload.get("detections", []):
            class_name = str(detection.get("class_name", ""))
            class_id = detection.get("class_id")
            if class_name != self.target_class_name and class_id != 0:
                continue
            score = float(detection.get("score", 0.0) or 0.0)
            if score < self.score_threshold:
                continue
            bbox = detection.get("bbox") or {}
            if not bbox:
                continue
            bbox_w = float(bbox.get("w", 0.0) or 0.0)
            bbox_h = float(bbox.get("h", 0.0) or 0.0)
            area_ratio = max(0.0, bbox_w) * max(0.0, bbox_h) / (width * height)
            rank = score + area_ratio
            if rank > best_score:
                best_score = rank
                best = {
                    "cx_ratio": float(bbox.get("cx", width * 0.5) or width * 0.5) / width,
                    "height_ratio": max(0.0, bbox_h) / height,
                    "area_ratio": area_ratio,
                    "depth_m": detection.get("depth_m"),
                    "score": score,
                }
        return best

    def on_timer(self):
        now = self.get_clock().now()

        if self.state == MissionState.WAITING:
            return
        if self.state == MissionState.DONE:
            return

        if self.state == MissionState.APPROACHING:
            if self.detection_is_stale(now):
                self.publish_state("person 暂时丢失，等待重新识别")
                return
            command = self.centering_command(self.last_detection)
            if self.require_centered_to_advance and command is not None:
                self.send_command(command)
                return
            if self.is_close(self.last_detection):
                self.return_duration_sec = min(
                    self.max_return_sec,
                    max(self.min_return_sec, self.approach_forward_sec),
                )
                self.transition(
                    MissionState.ROTATING,
                    f"person 已很近，原地旋转 180 度，随后返回 {self.return_duration_sec:.1f}s",
                )
                return
            command = self.size_command(self.last_detection)
            self.approach_forward_sec += self.control_period_sec
            self.send_command(command)
            return

        elapsed = (now - self.state_started_at).nanoseconds * 1e-9
        if self.state == MissionState.ROTATING:
            if elapsed < self.rotate_180_sec:
                self.send_command("原地旋转")
            else:
                self.transition(MissionState.RETURNING, "旋转完成，沿反向向前返回起点")
            return

        if self.state == MissionState.RETURNING:
            if elapsed < self.return_duration_sec:
                self.send_command("向前")
            else:
                self.send_stop()
                self.transition(MissionState.DONE, "已返回起点，任务完成")

    def detection_is_stale(self, now):
        if self.last_detection_time is None:
            return True
        age = (now - self.last_detection_time).nanoseconds * 1e-9
        return age > self.lost_timeout_sec

    def is_close(self, detection):
        depth = detection.get("depth_m")
        if depth is not None:
            try:
                if float(depth) <= self.close_distance_m:
                    return True
            except (TypeError, ValueError):
                pass
        return (
            detection["height_ratio"] >= self.close_height_ratio
            or detection["area_ratio"] >= self.close_area_ratio
        )

    def centering_command(self, detection):
        offset = detection["cx_ratio"] - 0.5
        if offset < -self.center_deadband_ratio:
            return "向右"
        if offset > self.center_deadband_ratio:
            return "向左"
        return None

    def size_command(self, detection):
        height_error = detection["height_ratio"] - self.target_height_ratio
        area_error = detection["area_ratio"] - self.target_area_ratio
        if height_error > self.size_deadband_ratio or area_error > self.size_deadband_ratio:
            return "向后"
        return "向前"

    def transition(self, state, text):
        self.state = state
        self.state_started_at = self.get_clock().now()
        self.publish_state(text)

    def send_command(self, command):
        if self.repeat_same_terminal_cmd or command != self.last_terminal_command:
            self.get_logger().info(f"终端模拟小车指令: {command}")
            terminal_msg = String()
            terminal_msg.data = command
            self.terminal_pub.publish(terminal_msg)
            self.last_terminal_command = command

        if self.publish_manual_cmd_enabled:
            manual_command = COMMAND_TO_MANUAL.get(command)
            if manual_command:
                manual_msg = String()
                manual_msg.data = manual_command
                self.manual_pub.publish(manual_msg)

    def send_stop(self):
        if self.publish_manual_cmd_enabled:
            manual_msg = String()
            manual_msg.data = "stop"
            self.manual_pub.publish(manual_msg)
        self.last_terminal_command = ""

    def publish_state(self, text):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_stop()
    finally:
        node.send_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
