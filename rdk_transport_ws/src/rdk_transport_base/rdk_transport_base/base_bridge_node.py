import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:  # pragma: no cover - depends on target image packages
    serial = None


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def bool_param(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def int_param(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def float_param(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_int_list(value, length, default):
    if isinstance(value, str):
        parts = [item.strip() for item in value.split(",") if item.strip()]
    else:
        try:
            parts = list(value)
        except TypeError:
            parts = []
    out = []
    for item in parts[:length]:
        try:
            out.append(int(item))
        except (TypeError, ValueError):
            out.append(default[len(out)])
    while len(out) < length:
        out.append(default[len(out)])
    return out


def solve_3x3(matrix, vector):
    rows = [
        [float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(vector[0])],
        [float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(vector[1])],
        [float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(vector[2])],
    ]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) < 1e-9:
            return None
        if pivot != col:
            rows[col], rows[pivot] = rows[pivot], rows[col]
        scale = rows[col][col]
        for idx in range(col, 4):
            rows[col][idx] /= scale
        for row in range(3):
            if row == col:
                continue
            factor = rows[row][col]
            for idx in range(col, 4):
                rows[row][idx] -= factor * rows[col][idx]
    return rows[0][3], rows[1][3], rows[2][3]


class BaseBridgeNode(Node):
    def __init__(self):
        super().__init__("base_bridge_node")

        self.declare_parameter("mock_mode", True)
        self.declare_parameter("allow_protected_serial", False)
        self.declare_parameter("serial_port", "")
        self.declare_parameter("serial_baudrate", 115200)
        self.declare_parameter("serial_write_commands", False)
        self.declare_parameter("serial_min_interval_sec", 0.3)
        self.declare_parameter("serial_line_ending", "\n")
        self.declare_parameter("serial_heartbeat_sec", 1.0)
        self.declare_parameter("serial_heartbeat_command", "")
        self.declare_parameter("cmd_axis_threshold", 0.05)
        self.declare_parameter("fork_cmd_topic", "/fork/cmd")
        self.declare_parameter("fork_status_topic", "/fork/status")
        self.declare_parameter("tx_preview_topic", "/stm32/tx_preview")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_rate_hz", 30.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("disabled_encoder_index", -1)
        self.declare_parameter("encoder_ticks_per_rev", 1000.0)
        self.declare_parameter("wheel_radius_m", 0.05)
        self.declare_parameter("wheel_base_x_m", 0.20)
        self.declare_parameter("wheel_base_y_m", 0.18)
        self.declare_parameter("encoder_signs", "1,1,1,1")
        self.declare_parameter("encoder_timeout_sec", 0.3)
        self.declare_parameter("encoder_max_delta_ticks", 10000)
        self.declare_parameter("encoder_max_speed_xy", 0.30)
        self.declare_parameter("encoder_max_wz", 0.80)

        self.mock_mode = bool_param(self.get_parameter("mock_mode").value)
        self.allow_protected_serial = bool_param(self.get_parameter("allow_protected_serial").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.serial_baudrate = int(self.get_parameter("serial_baudrate").value)
        self.serial_write_commands = bool_param(self.get_parameter("serial_write_commands").value)
        self.serial_min_interval_sec = float(self.get_parameter("serial_min_interval_sec").value)
        self.serial_line_ending = str(self.get_parameter("serial_line_ending").value)
        self.serial_heartbeat_sec = float(self.get_parameter("serial_heartbeat_sec").value)
        self.serial_heartbeat_command = str(self.get_parameter("serial_heartbeat_command").value).strip()
        self.cmd_axis_threshold = float(self.get_parameter("cmd_axis_threshold").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.publish_tf = bool_param(self.get_parameter("publish_tf").value)
        self.odom_rate_hz = float(self.get_parameter("odom_rate_hz").value)
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self.disabled_encoder_index = int_param(self.get_parameter("disabled_encoder_index").value, -1)
        self.encoder_ticks_per_rev = float_param(self.get_parameter("encoder_ticks_per_rev").value, 1000.0)
        self.wheel_radius_m = float_param(self.get_parameter("wheel_radius_m").value, 0.05)
        self.wheel_base_x_m = float_param(self.get_parameter("wheel_base_x_m").value, 0.20)
        self.wheel_base_y_m = float_param(self.get_parameter("wheel_base_y_m").value, 0.18)
        self.encoder_signs = parse_int_list(
            self.get_parameter("encoder_signs").value,
            4,
            [1, 1, 1, 1],
        )
        self.encoder_timeout_sec = float_param(self.get_parameter("encoder_timeout_sec").value, 0.3)
        self.encoder_max_delta_ticks = int_param(self.get_parameter("encoder_max_delta_ticks").value, 10000)
        self.encoder_max_speed_xy = float_param(self.get_parameter("encoder_max_speed_xy").value, 0.30)
        self.encoder_max_wz = float_param(self.get_parameter("encoder_max_wz").value, 0.80)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        fork_cmd_topic = str(self.get_parameter("fork_cmd_topic").value)
        fork_status_topic = str(self.get_parameter("fork_status_topic").value)
        tx_preview_topic = str(self.get_parameter("tx_preview_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.last_serial_write_time = self.get_clock().now()
        self.last_heartbeat_time = self.get_clock().now()
        self.last_serial_command = ""
        self.cmd_vel_timeout = True
        self.timeout_stop_sent = False
        self.serial_handle = None
        self.serial_connected = False
        self.serial_tx_count = 0
        self.serial_suppressed_count = 0
        self.serial_last_error = ""
        self.serial_rx_buffer = ""
        self.encoder_rx_count = 0
        self.encoder_parse_errors = 0
        self.encoder_filtered_count = 0
        self.encoder_valid_count = 0
        self.encoder_degraded = False
        self.encoder_last_seq = None
        self.last_encoder_time = None
        self.encoder_velocity = (0.0, 0.0, 0.0)
        self.last_encoder_counts = [0, 0, 0, 0]

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.tx_preview_pub = self.create_publisher(String, tx_preview_topic, 10)
        self.fork_status_pub = self.create_publisher(String, fork_status_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, cmd_vel_topic, self.on_cmd_vel, 10)
        self.fork_sub = self.create_subscription(String, fork_cmd_topic, self.on_fork_cmd, 10)

        period = 1.0 / max(self.odom_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)
        self.open_serial_if_needed()

        self.get_logger().info(
            f"base_bridge_node started in mock_mode={self.mock_mode}; "
            f"publishing {odom_topic} at {self.odom_rate_hz:.1f} Hz"
        )

    def protected_serial_requested(self):
        protected_markers = (
            "FT232R_USB_UART_A50285BI",
            "usb-FTDI_FT232R_USB_UART_A50285BI",
            "USB_Single_Serial_5A37017495",
            "usb-1a86_USB_Single_Serial_5A37017495",
            "/dev/ttyUSB0",
            "/dev/ttyACM0",
        )
        return any(marker in self.serial_port for marker in protected_markers)

    def open_serial_if_needed(self):
        if self.mock_mode:
            return
        if not self.serial_write_commands:
            self.serial_last_error = "serial_write_commands=false; refusing to open serial port"
            self.get_logger().warn(self.serial_last_error)
            return
        if not self.serial_port:
            self.serial_last_error = "serial_port is empty; refusing to open serial port"
            self.get_logger().error(self.serial_last_error)
            return
        if self.protected_serial_requested() and not self.allow_protected_serial:
            self.serial_last_error = (
                "refusing protected serial port; choose the STM32 by-id path or set "
                "allow_protected_serial=true only after confirming it is not lidar/IMU"
            )
            self.get_logger().error(f"{self.serial_last_error}: {self.serial_port}")
            return
        if serial is None:
            self.serial_last_error = "pyserial is not available"
            self.get_logger().error(self.serial_last_error)
            return

        try:
            self.serial_handle = serial.Serial(
                port=self.serial_port,
                baudrate=self.serial_baudrate,
                timeout=0.0,
                write_timeout=0.05,
            )
            self.serial_connected = True
            self.serial_last_error = ""
            self.get_logger().info(f"opened serial port {self.serial_port} at {self.serial_baudrate}")
        except Exception as exc:
            self.serial_connected = False
            self.serial_last_error = str(exc)
            self.get_logger().error(f"failed to open serial port {self.serial_port}: {exc}")

    def on_cmd_vel(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.timeout_stop_sent = False
        self.write_velocity_if_enabled(msg)

    def publish_fork_status(self, status_text):
        msg = String()
        msg.data = status_text
        self.fork_status_pub.publish(msg)

    def send_stm32_command(self, command, force=False):
        command = command.strip()
        if not command:
            return
        now = self.get_clock().now()
        age = (now - self.last_serial_write_time).nanoseconds * 1e-9
        if not force and age < self.serial_min_interval_sec:
            self.serial_suppressed_count += 1
            return
        if not force and command == self.last_serial_command:
            return
        preview = String()
        preview.data = command
        self.tx_preview_pub.publish(preview)
        self.last_serial_write_time = now
        self.last_serial_command = command
        if self.mock_mode or not self.serial_write_commands:
            return
        if not self.serial_connected or self.serial_handle is None:
            return
        line = (command + self.serial_line_ending).encode("ascii")
        try:
            self.serial_handle.write(line)
            self.serial_tx_count += 1
            self.serial_last_error = ""
        except Exception as exc:
            self.serial_connected = False
            self.serial_last_error = str(exc)
            self.get_logger().error(f"serial write failed: {exc}")
            try:
                self.serial_handle.close()
            except Exception:
                pass
            self.serial_handle = None

    def discrete_motion_command(self, msg):
        axes = [
            ("x", float(msg.linear.x)),
            ("y", float(msg.linear.y)),
            ("z", float(msg.angular.z)),
        ]
        active = [(name, value) for name, value in axes if abs(value) >= self.cmd_axis_threshold]
        if not active:
            return "(0,0,0)"

        name, value = max(active, key=lambda item: abs(item[1]))
        direction = 1 if value > 0.0 else -1
        if name == "x":
            return f"({direction},0,0)"
        if name == "y":
            return f"(0,{direction},0)"
        return f"(0,0,{direction})"

    def write_velocity_if_enabled(self, msg):
        self.send_stm32_command(self.discrete_motion_command(msg))

    def on_fork_cmd(self, msg):
        command = msg.data.strip().lower()
        if command not in ("up", "down", "stop"):
            self.serial_last_error = f"invalid fork command: {msg.data}"
            self.get_logger().warn(self.serial_last_error)
            return
        self.send_stm32_command("(0,0,0)" if command == "stop" else command)
        if command == "up":
            self.publish_fork_status("moving_up")
        elif command == "down":
            self.publish_fork_status("moving_down")
        else:
            self.publish_fork_status("stopped")

    def read_serial_if_available(self):
        if self.mock_mode or self.serial_handle is None:
            return
        try:
            waiting = int(getattr(self.serial_handle, "in_waiting", 0) or 0)
            if waiting <= 0:
                return
            chunk = self.serial_handle.read(min(waiting, 512))
        except Exception as exc:
            self.serial_connected = False
            self.serial_last_error = str(exc)
            self.get_logger().error(f"serial read failed: {exc}")
            return
        if not chunk:
            return
        self.serial_rx_buffer += chunk.decode("ascii", "ignore")
        if len(self.serial_rx_buffer) > 2048:
            self.serial_rx_buffer = self.serial_rx_buffer[-2048:]
        while "\n" in self.serial_rx_buffer:
            line, self.serial_rx_buffer = self.serial_rx_buffer.split("\n", 1)
            self.handle_serial_line(line.strip())

    def handle_serial_line(self, line):
        if not line:
            return
        if not line.startswith("ENC,"):
            return
        fields = line.split(",")
        if len(fields) != 11:
            self.encoder_parse_errors += 1
            return
        try:
            seq = int(fields[1])
            dt_ms = int(fields[2])
            counts = [int(item) for item in fields[3:7]]
            deltas = [int(item) for item in fields[7:11]]
        except ValueError:
            self.encoder_parse_errors += 1
            return
        self.apply_encoder_frame(seq, dt_ms, counts, deltas)

    def apply_encoder_frame(self, seq, dt_ms, counts, deltas):
        if dt_ms <= 0 or self.encoder_ticks_per_rev <= 0.0 or self.wheel_radius_m <= 0.0:
            self.encoder_parse_errors += 1
            return
        dt = float(dt_ms) * 0.001
        rows = [
            (1.0, -1.0, -(self.wheel_base_x_m + self.wheel_base_y_m)),
            (1.0, 1.0, self.wheel_base_x_m + self.wheel_base_y_m),
            (1.0, 1.0, -(self.wheel_base_x_m + self.wheel_base_y_m)),
            (1.0, -1.0, self.wheel_base_x_m + self.wheel_base_y_m),
        ]
        meters_per_tick = (2.0 * math.pi * self.wheel_radius_m) / self.encoder_ticks_per_rev
        active_rows = []
        active_values = []
        filtered = 0
        for idx, delta in enumerate(deltas):
            if idx == self.disabled_encoder_index:
                filtered += 1
                continue
            if abs(delta) > self.encoder_max_delta_ticks:
                filtered += 1
                continue
            active_rows.append(rows[idx])
            active_values.append(float(delta) * float(self.encoder_signs[idx]) * meters_per_tick / dt)

        self.encoder_rx_count += 1
        self.encoder_filtered_count += filtered
        self.encoder_valid_count = len(active_rows)
        self.encoder_degraded = len(active_rows) < 4
        self.encoder_last_seq = seq
        self.last_encoder_counts = counts
        self.last_encoder_time = self.get_clock().now()

        if len(active_rows) < 3:
            self.encoder_velocity = (0.0, 0.0, 0.0)
            return

        normal = [[0.0, 0.0, 0.0] for _ in range(3)]
        rhs = [0.0, 0.0, 0.0]
        for row, value in zip(active_rows, active_values):
            for i in range(3):
                rhs[i] += row[i] * value
                for j in range(3):
                    normal[i][j] += row[i] * row[j]
        solved = solve_3x3(normal, rhs)
        if solved is None:
            self.encoder_parse_errors += 1
            self.encoder_velocity = (0.0, 0.0, 0.0)
            return
        vx, vy, wz = solved
        speed_xy = math.hypot(vx, vy)
        if speed_xy > self.encoder_max_speed_xy:
            scale = self.encoder_max_speed_xy / speed_xy
            vx *= scale
            vy *= scale
        if abs(wz) > self.encoder_max_wz:
            wz = math.copysign(self.encoder_max_wz, wz)
        self.encoder_velocity = (vx, vy, wz)

    def maybe_send_heartbeat(self, now):
        if self.serial_heartbeat_sec <= 0.0:
            return
        elapsed = (now - self.last_heartbeat_time).nanoseconds * 1e-9
        if elapsed < self.serial_heartbeat_sec:
            return
        self.last_heartbeat_time = now
        if self.serial_heartbeat_command:
            self.send_stm32_command(self.serial_heartbeat_command, force=True)

    def on_timer(self):
        now = self.get_clock().now()
        self.maybe_send_heartbeat(now)
        self.read_serial_if_available()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        self.cmd_vel_timeout = age > self.cmd_vel_timeout_sec
        if self.cmd_vel_timeout and not self.timeout_stop_sent:
            self.send_stm32_command("(0,0,0)")
            self.timeout_stop_sent = True

        encoder_age = None
        if self.last_encoder_time is not None:
            encoder_age = (now - self.last_encoder_time).nanoseconds * 1e-9

        if not self.mock_mode:
            if (
                self.last_encoder_time is not None
                and encoder_age is not None
                and encoder_age <= self.encoder_timeout_sec
                and self.encoder_valid_count >= 3
            ):
                vx, vy, wz = self.encoder_velocity
            else:
                vx = 0.0
                vy = 0.0
                wz = 0.0
        else:
            if self.cmd_vel_timeout:
                vx = 0.0
                vy = 0.0
                wz = 0.0
                if not self.timeout_stop_sent:
                    self.send_stm32_command("(0,0,0)")
                    self.timeout_stop_sent = True
            else:
                vx = float(self.last_cmd.linear.x)
                vy = float(self.last_cmd.linear.y)
                wz = float(self.last_cmd.angular.z)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = odom.header.stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = "rdk_transport_base/base_bridge_node"
        status.hardware_id = "mock_base" if self.mock_mode else "serial_base"
        encoder_age = -1.0
        if self.last_encoder_time is not None:
            encoder_age = (self.get_clock().now() - self.last_encoder_time).nanoseconds * 1e-9
        encoder_stale = (
            not self.mock_mode
            and self.last_encoder_time is not None
            and encoder_age > self.encoder_timeout_sec
        )
        encoder_invalid = (
            not self.mock_mode
            and self.last_encoder_time is not None
            and self.encoder_valid_count < 3
        )
        if not self.mock_mode and not self.serial_connected:
            status.level = DiagnosticStatus.ERROR
            status.message = self.serial_last_error or "serial disconnected"
        elif not self.mock_mode and self.last_encoder_time is None:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for encoder frames"
        elif encoder_invalid:
            status.level = DiagnosticStatus.WARN
            status.message = "fewer than 3 valid encoders; odom velocity forced to zero"
        elif encoder_stale:
            status.level = DiagnosticStatus.WARN
            status.message = "encoder timeout; odom velocity forced to zero"
        elif self.cmd_vel_timeout:
            status.level = DiagnosticStatus.WARN
            status.message = "cmd_vel timeout; publishing zero velocity"
        elif self.encoder_degraded:
            status.level = DiagnosticStatus.WARN
            status.message = "encoder degraded mode"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "OK"
        status.values = [
            KeyValue(key="mock_mode", value=str(self.mock_mode).lower()),
            KeyValue(key="allow_protected_serial", value=str(self.allow_protected_serial).lower()),
            KeyValue(key="cmd_vel_timeout", value=str(self.cmd_vel_timeout).lower()),
            KeyValue(key="odom_rate_hz", value=f"{self.odom_rate_hz:.1f}"),
            KeyValue(key="odom_frame_id", value=self.odom_frame_id),
            KeyValue(key="base_frame_id", value=self.base_frame_id),
            KeyValue(key="serial_write_commands", value=str(self.serial_write_commands).lower()),
            KeyValue(key="serial_port", value=self.serial_port),
            KeyValue(key="serial_connected", value=str(self.serial_connected).lower()),
            KeyValue(key="serial_heartbeat_sec", value=f"{self.serial_heartbeat_sec:.2f}"),
            KeyValue(key="serial_heartbeat_command", value=self.serial_heartbeat_command),
            KeyValue(key="serial_tx_count", value=str(self.serial_tx_count)),
            KeyValue(key="serial_suppressed_count", value=str(self.serial_suppressed_count)),
            KeyValue(key="serial_last_error", value=self.serial_last_error),
            KeyValue(key="last_serial_command", value=self.last_serial_command),
            KeyValue(key="disabled_encoder_index", value=str(self.disabled_encoder_index)),
            KeyValue(key="encoder_signs", value=",".join(str(item) for item in self.encoder_signs)),
            KeyValue(key="encoder_rx_count", value=str(self.encoder_rx_count)),
            KeyValue(key="encoder_parse_errors", value=str(self.encoder_parse_errors)),
            KeyValue(key="encoder_filtered_count", value=str(self.encoder_filtered_count)),
            KeyValue(key="encoder_valid_count", value=str(self.encoder_valid_count)),
            KeyValue(key="encoder_degraded", value=str(self.encoder_degraded).lower()),
            KeyValue(key="encoder_last_seq", value=str(self.encoder_last_seq)),
            KeyValue(key="last_encoder_age_sec", value=f"{encoder_age:.3f}"),
            KeyValue(key="last_encoder_counts", value=",".join(str(item) for item in self.last_encoder_counts)),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self.diag_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = BaseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(node, "serial_handle", None) is not None:
            node.serial_handle.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
