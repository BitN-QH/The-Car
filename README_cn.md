# 基于 RDK X5 多模态感知与自主导航的具身智能全向搬运机器人

本项目面向智能仓储托盘搬运、车间物料转运等场景，设计并实现了一套基于 RDK X5、ROS 2 Humble / TROS Humble 的具身智能全向搬运机器人系统。系统融合 SLAM、YOLO、VLM、Nav2 等关键技术，实现环境建图定位、目标识别、语义理解、自主导航、动态避障、远程运维和自然语言任务执行。

## 项目简介

传统叉车及 AGV 在仓储物流场景中常存在智能化程度不足、动态避障能力有限、远程运维不便等问题。本项目以麦克纳姆轮全向底盘为移动平台，采用“边端实时控制 + 智能感知协同”的总体架构，在 RDK X5 上运行底盘控制、定位建图、路径规划、局部避障、目标检测、任务调度和 Web 可视化控制等模块。

系统可支持托盘搬运、到点导航、目标定位、状态监测、异常告警、远程任务下发、语义区域导航和场景巡检等功能，具备较强的自主性、实时性和工程应用价值。

## 核心功能

- 麦克纳姆轮全向移动底盘控制
- ROS 2 / TROS Humble 机器人软件栈
- SLAM 建图、定位与 Nav2 自主导航
- 基于 YOLO 的摄像头目标检测
- 面向 VLM 的语义理解与任务分析接口
- 动态避障、安全监测与异常告警
- Web 页面显示摄像头画面、YOLO 检测结果和控制按钮
- 串口桥接底盘运动指令
- 开源前端页面、ROS 节点、配置文件和部署脚本

## 系统架构

```text
自然语言任务 / Web 控制指令
             |
             v
Web 控制台与任务接口
             |
             v
RDK X5 边端控制与感知系统
   |         |          |          |
   |         |          |          |
底盘控制   SLAM建图   YOLO检测   Nav2导航
   |         |          |          |
   +---------+----------+----------+
             |
             v
具身智能全向搬运机器人
```

## 仓库目录

```text
.
|-- rdk_transport_ws/
|   |-- src/
|   |   |-- rdk_transport_base/           # 底盘、IMU里程计、任务管理、Web控制等节点
|   |   |-- rdk_transport_bringup/        # 传感器、SLAM、Nav2 启动与配置
|   |   |-- rdk_transport_description/    # URDF 与 RViz 资源
|   |   `-- rdk_transport_perception_cpp/ # YOLO 感知桥接
|   |-- scripts/                          # 运行脚本
|   `-- third_party/wheelbot_lslidar/     # 本项目集成使用的雷达驱动快照
|-- frontend/
|   |-- qianrushi_qianduan/               # 项目前端展示页面
|   `-- yolo_web_monitor/                 # YOLO Web 监控与控制页面
|-- 3D/                                   # 3D 打印结构件
|-- docs/                                 # 部署说明与开源清单
|-- tools/                                # 辅助脚本
|-- Car_2026-06-29.epro                   # 电气工程文件
`-- Car.zip                               # 相关工程归档
```

## 硬件与软件环境

- 主控：RDK X5
- 系统：Ubuntu 22.04 ARM64
- 中间件：ROS 2 Humble / TROS Humble
- 底盘：麦克纳姆轮全向移动底盘
- IMU：H30 USB 串口陀螺仪
- 雷达：N10 / FTDI 串口雷达
- 摄像头：USB 摄像头，例如 `/dev/video0` 或 `/dev/video2`
- 底盘串口：CH340 USB 串口

开发时使用过的典型设备路径如下：

```text
IMU:  /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A37017495-if00
底盘: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
雷达: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0
```

## 编译方法

在 RDK X5 板端执行：

```bash
cd /home/sunrise/rdk_transport_ws
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 常用启动

启动基础机器人栈：

```bash
bash /home/sunrise/rdk_transport_ws/src/rdk_transport_bringup/scripts/start_bringup.sh
```

启动 YOLO 感知：

```bash
cd /home/sunrise/rdk_transport_ws
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch rdk_transport_perception_cpp yolov8_cpp_perception.launch.py \
  usb_video_device:=/dev/video0 image_width:=640 image_height:=480 \
  score_threshold:=0.35 launch_target_pose:=false
```

启动 YOLO Web 监控与运动控制页面：

```bash
ros2 launch rdk_transport_base yolo_web_monitor.launch.py \
  port:=8080 compressed_image_topic:=/image \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  serial_baudrate:=115200
```

在电脑浏览器访问：

```text
http://<RDK_IP>:8080/
```

## Web 控制指令

Web 页面提供 7 个运动控制按钮，每个按钮向小车串口发送一个字符串指令：

```text
(1,0,0)    前进
(-1,0,0)   后退
(0,1,0)    左移
(0,-1,0)   右移
(0,0,1)    左旋
(0,0,-1)   右旋
(0,0,0)    停止
```

串口协议草案位于：

```text
rdk_transport_ws/src/rdk_transport_bringup/docs/stm32_uart_protocol_draft.md
```

## NodeHub 提交信息

NodeHub 可直接读取本仓库作为作品代码仓库。中文页面请使用根目录 `README_cn.md`，英文页面请使用根目录 `README.md`。

建议填写：

- 项目名称：基于 RDK X5 多模态感知与自主导航的具身智能全向搬运机器人
- 项目简介：基于RDK X5构建全向搬运机器人，融合SLAM、YOLO、VLM与Nav2，实现建图定位、目标识别、自主导航、避障、远程运维与自然语言任务，适用于仓储托盘搬运、车间转运，具备自主性、实时性和应用价值
- 运行平台：RDK X5
- 分类标签：机器人应用、自主导航、多模态感知、智能仓储、全向移动
- 代码仓库：`https://github.com/BitN-QH/The-Car`
- 开源协议：MIT

更多提交辅助信息见 `docs/NODEHUB_SUBMISSION.md`。

## 说明

- 本仓库不包含模型权重、构建产物、运行日志、rosbag 文件和生成地图。
- YOLO 模型需在 RDK/TROS 环境中自行准备，典型路径为 `/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin`。
- `third_party/wheelbot_lslidar` 为项目集成使用的雷达驱动源码快照，正式发布前请核对上游许可证。

## 开源协议

本项目采用 MIT License 开源，详见 `LICENSE`。
