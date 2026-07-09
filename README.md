# RDK X5 Multimodal Transport Robot

基于 RDK X5、ROS2 Humble / TROS Humble 的多模态感知与全向搬运机器人示例工程。仓库包含底盘串口控制、H30 IMU、N10 激光雷达、SLAM/Nav2 配置、YOLO 检测桥接、Web 可视化与前端控制页面。

## 目录结构

```text
.
├── rdk_transport_ws/
│   ├── src/
│   │   ├── rdk_transport_base/          # 底盘、任务管理、Web 控制、IMU 里程计等节点
│   │   ├── rdk_transport_bringup/       # 一键启动、传感器、SLAM、Nav2 launch/config/scripts
│   │   ├── rdk_transport_description/   # URDF/RViz
│   │   └── rdk_transport_perception_cpp/# YOLO 检测 JSON 桥接
│   ├── scripts/                         # 运行脚本
│   └── third_party/wheelbot_lslidar/     # N10/雷达 ROS2 驱动源码快照
├── frontend/
│   ├── qianrushi_qianduan/              # 独立前端页面
│   └── yolo_web_monitor/                # 从 ROS Web 节点提取的页面参考
├── tools/
│   └── rdk_web_stack.sh                 # 板端 Web/YOLO/VLM 栈脚本
└── docs/
```

## 硬件环境

- RDK X5，Ubuntu 22.04 ARM64
- ROS2 Humble / TROS Humble
- H30 IMU：`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A37017495-if00`
- 小车/底盘 CH340 串口：`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- N10 雷达 FTDI 串口：`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0`
- 摄像头示例：`/dev/video0` 或 `/dev/video2`

## 构建

在 RDK X5 板端：

```bash
cd /home/sunrise/rdk_transport_ws
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 常用启动

基础栈：

```bash
bash /home/sunrise/rdk_transport_ws/src/rdk_transport_bringup/scripts/start_bringup.sh
```

YOLO + Web 控制台：

```bash
cd /home/sunrise/rdk_transport_ws
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rdk_transport_perception_cpp yolov8_cpp_perception.launch.py \
  usb_video_device:=/dev/video0 image_width:=640 image_height:=480 \
  score_threshold:=0.35 launch_target_pose:=false

ros2 launch rdk_transport_base yolo_web_monitor.launch.py \
  port:=8080 compressed_image_topic:=/image \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  serial_baudrate:=115200
```

浏览器访问：

```text
http://<RDK_IP>:8080/
```

## Web 控制指令

Web 页面 7 个运动按钮会向小车串口发送：

```text
(1,0,0)
(-1,0,0)
(0,1,0)
(0,-1,0)
(0,0,1)
(0,0,-1)
(0,0,0)
```

串口协议详见 `rdk_transport_ws/src/rdk_transport_bringup/docs/stm32_uart_protocol_draft.md`。

## 说明

- 本仓库不包含模型权重、构建产物、日志、地图运行输出和录包文件。
- YOLO 默认模型路径位于板端 `/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin`，需按 RDK/TROS 环境自行准备。
- `third_party/wheelbot_lslidar` 为项目集成所需的雷达驱动源码快照，请在正式发布前核对上游许可证。
