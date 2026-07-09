import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


MOTION_COMMANDS = {
    "forward": (1.0, 0.0, 0.0),
    "back": (-1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "rotate_left": (0.0, 0.0, 1.0),
    "rotate_right": (0.0, 0.0, -1.0),
    "stop": (0.0, 0.0, 0.0),
}

FORK_COMMANDS = {"up", "down"}


class ManualCommandNode(Node):
    def __init__(self):
        super().__init__("manual_command_node")
        self.declare_parameter("manual_cmd_topic", "/manual_cmd")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("fork_cmd_topic", "/fork/cmd")

        manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        fork_cmd_topic = str(self.get_parameter("fork_cmd_topic").value)

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.fork_pub = self.create_publisher(String, fork_cmd_topic, 10)
        self.sub = self.create_subscription(String, manual_cmd_topic, self.on_manual_cmd, 10)
        self.get_logger().info(
            "manual_command_node ready: forward/back/left/right/rotate_left/"
            "rotate_right/stop/up/down"
        )

    def on_manual_cmd(self, msg):
        command = msg.data.strip().lower().replace("-", "_").replace(" ", "_")
        if command in MOTION_COMMANDS:
            vx, vy, wz = MOTION_COMMANDS[command]
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz
            self.cmd_vel_pub.publish(twist)
            return

        if command in FORK_COMMANDS:
            fork = String()
            fork.data = command
            self.fork_pub.publish(fork)
            return

        self.get_logger().warn(f"unsupported manual command: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = ManualCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
