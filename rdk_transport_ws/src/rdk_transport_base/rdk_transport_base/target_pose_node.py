import json
import math

import rclpy
from geometry_msgs.msg import PoseArray, Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def rotate_vector(q, x, y, z):
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class TargetPoseNode(Node):
    def __init__(self):
        super().__init__("target_pose_node")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("target_poses_topic", "/perception/target_poses")
        self.declare_parameter("depth_topic", "/depth/image")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("default_target_distance_m", 1.0)
        self.declare_parameter("horizontal_fov_deg", 60.0)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("depth_window_px", 5)
        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 6.0)

        detections_topic = str(self.get_parameter("detections_topic").value)
        target_poses_topic = str(self.get_parameter("target_poses_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.default_distance = float(self.get_parameter("default_target_distance_m").value)
        self.horizontal_fov = math.radians(float(self.get_parameter("horizontal_fov_deg").value))
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.depth_window_px = int(self.get_parameter("depth_window_px").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.pose_pub = self.create_publisher(PoseArray, target_poses_topic, 10)
        self.det_sub = self.create_subscription(String, detections_topic, self.on_detections, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.on_depth, 10)
        self.info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.on_camera_info, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.camera_info = None
        self.depth_image = None
        self.get_logger().info(
            "target_pose_node ready; uses RGB-D depth when available and monocular bearing fallback otherwise"
        )

    def on_camera_info(self, msg):
        if len(msg.k) >= 6 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera_info = msg

    def on_depth(self, msg):
        if msg.width > 0 and msg.height > 0:
            self.depth_image = msg

    def depth_at(self, u, v):
        msg = self.depth_image
        if msg is None:
            return None
        if not (0 <= u < msg.width and 0 <= v < msg.height):
            return None

        encoding = msg.encoding.lower()
        if encoding not in ("16uc1", "mono16", "32fc1"):
            return None
        bytes_per_pixel = 4 if encoding == "32fc1" else 2
        if msg.step < msg.width * bytes_per_pixel:
            return None

        import struct

        depths = []
        radius = max(0, self.depth_window_px // 2)
        for y in range(max(0, v - radius), min(msg.height, v + radius + 1)):
            for x in range(max(0, u - radius), min(msg.width, u + radius + 1)):
                offset = y * msg.step + x * bytes_per_pixel
                raw = msg.data[offset:offset + bytes_per_pixel]
                if len(raw) != bytes_per_pixel:
                    continue
                if encoding == "32fc1":
                    value = struct.unpack("<f", raw)[0]
                    depth = float(value)
                else:
                    value = struct.unpack("<H", raw)[0]
                    depth = float(value) * self.depth_scale
                if math.isfinite(depth) and self.min_depth_m <= depth <= self.max_depth_m:
                    depths.append(depth)
        if not depths:
            return None
        depths.sort()
        return depths[len(depths) // 2]

    def project_from_depth(self, u, v):
        depth = self.depth_at(u, v)
        if depth is None or self.camera_info is None:
            return None
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx0 = float(self.camera_info.k[2])
        cy0 = float(self.camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        x = depth
        y = -(float(u) - cx0) * depth / fx
        z = -(float(v) - cy0) * depth / fy
        return x, y, z

    def project_fallback(self, u, width):
        bearing = ((u / width) - 0.5) * self.horizontal_fov
        return (
            self.default_distance * math.cos(bearing),
            self.default_distance * math.sin(bearing),
            0.0,
        )

    def on_detections(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"invalid detections JSON: {exc}")
            return

        detections = payload.get("detections", [])
        width = float(payload.get("image_width", 0) or 0)
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = self.target_frame

        transform = None
        if self.target_frame != self.camera_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    self.camera_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as exc:
                if detections:
                    self.get_logger().warn(f"camera TF unavailable; publishing camera-frame fallback: {exc}")
                pose_array.header.frame_id = self.camera_frame

        for detection in detections:
            bbox = detection.get("bbox", {})
            if width <= 0 or not bbox:
                continue
            cx = float(bbox.get("cx", bbox.get("x", width * 0.5)))
            cy = float(bbox.get("cy", 0.0))
            px, py, pz = self.project_from_depth(int(round(cx)), int(round(cy))) or self.project_fallback(cx, width)
            if transform is not None:
                tr = transform.transform.translation
                px, py, pz = rotate_vector(transform.transform.rotation, px, py, pz)
                px += tr.x
                py += tr.y
                pz += tr.z
            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            pose.position.z = pz
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.pose_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
