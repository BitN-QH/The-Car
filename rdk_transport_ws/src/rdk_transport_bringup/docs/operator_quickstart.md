# RDK Transport Bringup Quickstart

## Build

```bash
cd /home/sunrise/rdk_transport_ws
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Start

```bash
bash /home/sunrise/rdk_transport_ws/src/rdk_transport_bringup/scripts/start_bringup.sh
```

To disable a sensor during debugging:

```bash
bash /home/sunrise/rdk_transport_ws/src/rdk_transport_bringup/scripts/start_bringup.sh launch_usb_camera:=false
```

## Topic Checks

```bash
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
source /home/sunrise/lslidar_ws/install/setup.bash
source /home/sunrise/h30_imu_ws/install/setup.bash
source /home/sunrise/rdk_transport_ws/install/setup.bash

ros2 topic hz /scan
ros2 topic hz /imu/data_raw
ros2 topic hz /image
ros2 topic hz /odom
ros2 topic echo /diagnostics --once
ros2 run tf2_tools view_frames
```

## Mock Base Test

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
ros2 topic echo /odom --once
```

If `/cmd_vel` is not refreshed for 0.5 seconds, the mock base publishes zero velocity.

The exact STM32 command that would be sent is visible on:

```bash
ros2 topic echo /stm32/tx_preview
```

Manual command helper:

```bash
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: forward}"
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: left}"
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: rotate_right}"
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: stop}"
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: up}"
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: down}"
```

## Serial Base Preview

Serial mode is implemented but intentionally locked by default. It sends ASCII
commands only and does not expect data from STM32.

Do not point it at the N10 or H30 serial devices. Those are currently:

- N10 lidar: `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0`
- H30 IMU: `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A37017495-if00`

Future STM32 usage:

```bash
ros2 launch rdk_transport_base base_serial.launch.py \
  serial_port:=/dev/serial/by-id/<STM32_DEVICE_ID> \
  serial_baudrate:=115200 \
  serial_write_commands:=true \
  serial_min_interval_sec:=0.3
```

Motion commands are derived from `/cmd_vel` and sent as `(x,y,z)`, where only one
field may be non-zero and non-zero values are `1` or `-1`. Fork commands are sent
with:

```bash
ros2 topic pub --once /fork/cmd std_msgs/msg/String "{data: up}"
ros2 topic pub --once /fork/cmd std_msgs/msg/String "{data: down}"
```

## RViz

Use `odom` as the Fixed Frame. Add `TF`, `LaserScan` on `/scan`, and `Image` on `/image`.

## Progressive SLAM / Navigation Demo

For the staged route from SLAM mapping to safe segmented navigation, see:

```text
/home/sunrise/rdk_transport_ws/src/rdk_transport_bringup/docs/progressive_demo_plan.md
```

## Optional Wheelbot Reference Content

Wheelbot_RDK-main content is kept optional. See:

```text
/home/sunrise/rdk_transport_ws/src/rdk_transport_bringup/docs/wheelbot_deployment_notes.md
```
