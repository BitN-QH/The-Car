import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def rotate_yaw(yaw, x, y):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


class H30ImuOdomNode(Node):
    def __init__(self):
        super().__init__("h30_imu_odom_node")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("odom_topic", "/imu/odom")
        self.declare_parameter("odom_frame_id", "imu_odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("use_orientation_yaw", True)
        self.declare_parameter("calibration_samples", 200)
        self.declare_parameter("accel_deadband", 0.18)
        self.declare_parameter("motion_accel_threshold", 0.22)
        self.declare_parameter("stationary_accel_threshold", 0.25)
        self.declare_parameter("stationary_gyro_threshold", 0.08)
        self.declare_parameter("zero_velocity_after_sec", 0.25)
        self.declare_parameter("velocity_decay", 0.90)
        self.declare_parameter("max_dt", 0.05)
        self.declare_parameter("max_velocity", 0.25)
        self.declare_parameter("max_position_radius", 8.0)

        imu_topic = str(self.get_parameter("imu_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.use_orientation_yaw = bool(self.get_parameter("use_orientation_yaw").value)
        self.calibration_samples = int(self.get_parameter("calibration_samples").value)
        self.accel_deadband = float(self.get_parameter("accel_deadband").value)
        self.motion_accel_threshold = float(self.get_parameter("motion_accel_threshold").value)
        self.stationary_accel_threshold = float(self.get_parameter("stationary_accel_threshold").value)
        self.stationary_gyro_threshold = float(self.get_parameter("stationary_gyro_threshold").value)
        self.zero_velocity_after_sec = float(self.get_parameter("zero_velocity_after_sec").value)
        self.velocity_decay = float(self.get_parameter("velocity_decay").value)
        self.max_dt = float(self.get_parameter("max_dt").value)
        self.max_velocity = float(self.get_parameter("max_velocity").value)
        self.max_position_radius = float(self.get_parameter("max_position_radius").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_time = None
        self.stationary_time = 0.0

        self.bias_samples = []
        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_ready = self.calibration_samples <= 0

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.on_imu, 50)
        self.get_logger().warn(
            "h30_imu_odom_node is experimental: IMU-only distance drifts. "
            "Use it for short, low-speed mapping segments only."
        )

    def calibrated_accel(self, msg):
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        if not self.bias_ready:
            self.bias_samples.append((ax, ay))
            if len(self.bias_samples) >= self.calibration_samples:
                self.bias_x = sum(v[0] for v in self.bias_samples) / len(self.bias_samples)
                self.bias_y = sum(v[1] for v in self.bias_samples) / len(self.bias_samples)
                self.bias_ready = True
                self.get_logger().info(
                    f"IMU accel bias calibrated: x={self.bias_x:.4f}, y={self.bias_y:.4f}"
                )
            return None
        ax -= self.bias_x
        ay -= self.bias_y
        if abs(ax) < self.accel_deadband:
            ax = 0.0
        if abs(ay) < self.accel_deadband:
            ay = 0.0
        return ax, ay

    def on_imu(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        if self.last_time is None:
            self.last_time = stamp
            return

        dt = (stamp - self.last_time).nanoseconds * 1e-9
        self.last_time = stamp
        if dt <= 0.0 or dt > self.max_dt:
            return

        orientation_valid = (
            self.use_orientation_yaw
            and len(msg.orientation_covariance) >= 1
            and msg.orientation_covariance[0] != -1.0
        )
        if orientation_valid:
            self.yaw = quaternion_to_yaw(msg.orientation)
        else:
            gz = float(msg.angular_velocity.z)
            self.yaw = math.atan2(math.sin(self.yaw + gz * dt), math.cos(self.yaw + gz * dt))

        calibrated = self.calibrated_accel(msg)
        if calibrated is None:
            self.publish_odom(msg.header.stamp)
            return

        ax_body, ay_body = calibrated
        gz = float(msg.angular_velocity.z)

        accel_norm = math.hypot(ax_body, ay_body)
        stationary = accel_norm < self.stationary_accel_threshold and abs(gz) < self.stationary_gyro_threshold
        self.stationary_time = self.stationary_time + dt if stationary else 0.0

        if self.stationary_time >= self.zero_velocity_after_sec:
            self.vx = 0.0
            self.vy = 0.0
        else:
            if accel_norm >= self.motion_accel_threshold:
                ax_world, ay_world = rotate_yaw(self.yaw, ax_body, ay_body)
            else:
                ax_world = 0.0
                ay_world = 0.0
            self.vx = (self.vx + ax_world * dt) * self.velocity_decay
            self.vy = (self.vy + ay_world * dt) * self.velocity_decay
            speed = math.hypot(self.vx, self.vy)
            if speed > self.max_velocity:
                scale = self.max_velocity / speed
                self.vx *= scale
                self.vy *= scale

        self.x += self.vx * dt
        self.y += self.vy * dt
        if math.hypot(self.x, self.y) > self.max_position_radius:
            self.get_logger().warn(
                "IMU-only odom drift exceeded max_position_radius; resetting x/y/v"
            )
            self.x = 0.0
            self.y = 0.0
            self.vx = 0.0
            self.vy = 0.0
        self.publish_odom(msg.header.stamp)

    def publish_odom(self, stamp):
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = 0.0
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.50
        odom.twist.covariance[7] = 0.50
        odom.twist.covariance[35] = 0.20
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = H30ImuOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
