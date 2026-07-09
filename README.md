# RDK X5 Multimodal Omnidirectional Transport Robot

An embodied intelligent transport robot based on RDK X5, ROS 2 Humble and TROS Humble. The system combines multimodal perception, SLAM, YOLO detection, VLM-based semantic understanding, Nav2 autonomous navigation and a web-based remote operation console for warehouse and workshop logistics.

## Project Overview

Traditional forklifts and AGVs often have limited intelligence, weak dynamic obstacle avoidance and inconvenient remote operation in flexible logistics scenarios. This project designs and implements an omnidirectional transport robot using a mecanum-wheel chassis and an edge-side perception and control architecture.

The RDK X5 runs the main robot software stack, including chassis communication, sensor access, mapping, localization, navigation, local obstacle avoidance, target detection, task execution and web monitoring. The robot can be used for pallet transport, point-to-point navigation, target search, semantic area navigation, scene inspection, status monitoring and remote task dispatch.

## Key Features

- Omnidirectional mecanum-wheel chassis control
- ROS 2 / TROS Humble robot software stack
- SLAM mapping, localization and Nav2 autonomous navigation
- YOLO-based camera perception and object detection
- VLM-oriented semantic task understanding interface
- Dynamic obstacle avoidance and safety monitoring
- Web console for video display, detection results and manual motion control
- Serial command bridge for chassis motion commands
- Open-source deployment scripts, frontend pages and ROS packages

## System Architecture

```text
Natural language task / Web command
             |
             v
Web console and task interface
             |
             v
RDK X5 edge control and perception stack
   |         |          |          |
   |         |          |          |
Chassis    SLAM       YOLO       Nav2
control    mapping    detection   navigation
   |         |          |          |
   +---------+----------+----------+
             |
             v
Omnidirectional transport robot
```

## Repository Structure

```text
.
|-- rdk_transport_ws/
|   |-- src/
|   |   |-- rdk_transport_base/           # Chassis, IMU odometry, task and web-control nodes
|   |   |-- rdk_transport_bringup/        # Launch files, sensor, SLAM and Nav2 configs
|   |   |-- rdk_transport_description/    # URDF and RViz resources
|   |   `-- rdk_transport_perception_cpp/ # YOLO perception bridge
|   |-- scripts/                          # Runtime scripts
|   `-- third_party/wheelbot_lslidar/     # LiDAR driver snapshot used by this project
|-- frontend/
|   |-- qianrushi_qianduan/               # Project frontend page
|   `-- yolo_web_monitor/                 # YOLO web monitor page
|-- 3D/                                   # 3D printable mechanical parts
|-- docs/                                 # Deployment and open-source notes
|-- tools/                                # Helper scripts
|-- Car_2026-06-29.epro                   # Electrical project file
`-- Car.zip                               # Related project archive
```

## Hardware and Software

- Board: RDK X5
- OS: Ubuntu 22.04 ARM64
- Middleware: ROS 2 Humble / TROS Humble
- Chassis: mecanum-wheel omnidirectional mobile base
- IMU: H30 USB serial IMU
- LiDAR: N10 / FTDI serial LiDAR
- Camera: USB camera, for example `/dev/video0` or `/dev/video2`
- Chassis serial port: CH340 USB serial adapter

Example device paths used during development:

```text
IMU:     /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A37017495-if00
Chassis: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
LiDAR:   /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0
```

## Build

Run on the RDK X5 board:

```bash
cd /home/sunrise/rdk_transport_ws
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the Robot Stack

Start the basic bringup stack:

```bash
bash /home/sunrise/rdk_transport_ws/src/rdk_transport_bringup/scripts/start_bringup.sh
```

Start YOLO perception:

```bash
cd /home/sunrise/rdk_transport_ws
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rdk_transport_perception_cpp yolov8_cpp_perception.launch.py \
  usb_video_device:=/dev/video0 image_width:=640 image_height:=480 \
  score_threshold:=0.35 launch_target_pose:=false
```

Start the YOLO web monitor and motion-control page:

```bash
ros2 launch rdk_transport_base yolo_web_monitor.launch.py \
  port:=8080 compressed_image_topic:=/image \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  serial_baudrate:=115200
```

Open the web console from another computer:

```text
http://<RDK_IP>:8080/
```

## Web Motion Commands

The web console provides seven control buttons. Each button sends one serial command to the chassis:

```text
(1,0,0)    forward
(-1,0,0)   backward
(0,1,0)    left
(0,-1,0)   right
(0,0,1)    rotate left
(0,0,-1)   rotate right
(0,0,0)    stop
```

The serial protocol draft is available at:

```text
rdk_transport_ws/src/rdk_transport_bringup/docs/stm32_uart_protocol_draft.md
```

## NodeHub Submission

NodeHub can read this repository as the project source. The Chinese project page should use `README_cn.md`, and the English project page should use `README.md`.

Suggested NodeHub information:

- Project name: RDK X5 Multimodal Omnidirectional Transport Robot
- Platform: RDK X5
- Categories: robot application, autonomous navigation, multimodal perception, warehouse logistics
- Repository: `https://github.com/BitN-QH/The-Car`
- License: MIT

More submission notes are provided in `docs/NODEHUB_SUBMISSION.md`.

## Notes

- Model weights, build outputs, logs, rosbag files and generated maps are not included.
- The YOLO model should be prepared on the RDK/TROS environment. A typical path is `/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin`.
- `third_party/wheelbot_lslidar` is included as the LiDAR driver snapshot used during integration. Please review the upstream license before redistribution in production.

## License

This project is released under the MIT License. See `LICENSE` for details.
