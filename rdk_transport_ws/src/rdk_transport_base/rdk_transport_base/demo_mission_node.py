import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DemoMissionNode(Node):
    def __init__(self):
        super().__init__("demo_mission_node")
        self.declare_parameter(
            "steps",
            [
                "forward",
                "left",
                "rotate_right",
                "forward",
                "fork_up",
                "back",
                "fork_down",
                "stop",
            ],
        )
        self.declare_parameter("step_interval_sec", 1.5)
        self.declare_parameter("manual_cmd_topic", "/manual_cmd")
        self.declare_parameter("fork_cmd_topic", "/fork/cmd")

        self.steps = list(self.get_parameter("steps").value)
        self.step_interval = float(self.get_parameter("step_interval_sec").value)
        self.manual_pub = self.create_publisher(
            String,
            str(self.get_parameter("manual_cmd_topic").value),
            10,
        )
        self.fork_pub = self.create_publisher(
            String,
            str(self.get_parameter("fork_cmd_topic").value),
            10,
        )
        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.index = 0
        self.timer = self.create_timer(self.step_interval, self.on_timer)
        self.publish_state("demo mission started")

    def publish_state(self, text):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def publish_manual(self, command):
        msg = String()
        msg.data = command
        self.manual_pub.publish(msg)

    def publish_fork(self, command):
        msg = String()
        msg.data = command
        self.fork_pub.publish(msg)

    def on_timer(self):
        if self.index >= len(self.steps):
            self.publish_manual("stop")
            self.publish_state("demo mission done")
            self.timer.cancel()
            return
        step = self.steps[self.index]
        self.index += 1
        if step == "fork_up":
            self.publish_fork("up")
        elif step == "fork_down":
            self.publish_fork("down")
        else:
            self.publish_manual(step)
        self.publish_state(f"demo step: {step}")


def main(args=None):
    rclpy.init(args=args)
    node = DemoMissionNode()
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
