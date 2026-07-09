import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSanitizerNode(Node):
    def __init__(self):
        super().__init__("scan_sanitizer_node")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_slam")
        self.declare_parameter("range_min", 0.2)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("force_full_circle", True)
        self.declare_parameter("reverse_ranges", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.force_full_circle = bool(self.get_parameter("force_full_circle").value)
        self.reverse_ranges = bool(self.get_parameter("reverse_ranges").value)

        self.pub = self.create_publisher(LaserScan, output_topic, 10)
        self.sub = self.create_subscription(LaserScan, input_topic, self.on_scan, 10)
        self.get_logger().info(f"sanitizing {input_topic} -> {output_topic}")

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(self.range_min, float(msg.range_min))
        out.range_max = min(self.range_max, float(msg.range_max))

        ranges = list(msg.ranges)
        intensities = list(msg.intensities)
        if self.reverse_ranges:
            ranges.reverse()
            intensities.reverse()

        sanitized = []
        for value in ranges:
            value = float(value)
            if not math.isfinite(value) or value < out.range_min or value > out.range_max:
                sanitized.append(float("inf"))
            else:
                sanitized.append(value)
        out.ranges = sanitized
        out.intensities = intensities if len(intensities) == len(sanitized) else []

        count = len(out.ranges)
        if count > 1:
            out.angle_min = 0.0 if self.force_full_circle else float(msg.angle_min)
            out.angle_max = 2.0 * math.pi if self.force_full_circle else float(msg.angle_max)
            out.angle_increment = (out.angle_max - out.angle_min) / float(count - 1)
        else:
            out.angle_min = float(msg.angle_min)
            out.angle_max = float(msg.angle_max)
            out.angle_increment = float(msg.angle_increment)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSanitizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
