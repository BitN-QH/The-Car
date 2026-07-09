# Deployment Notes

## Copy to RDK X5

```bash
scp -r rdk_transport_ws sunrise@10.220.190.26:/home/sunrise/
```

或者在板端直接保持路径：

```text
/home/sunrise/rdk_transport_ws
```

## Build

```bash
cd /home/sunrise/rdk_transport_ws
source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Start Web Control

```bash
bash /home/sunrise/rdk_transport_ws/scripts/rdk_web_stack.sh start
```

如果只启动 YOLO 和 Web 控制：

```bash
ros2 launch rdk_transport_perception_cpp yolov8_cpp_perception.launch.py \
  usb_video_device:=/dev/video0 image_width:=640 image_height:=480 \
  score_threshold:=0.35 launch_target_pose:=false

ros2 launch rdk_transport_base yolo_web_monitor.launch.py \
  port:=8080 compressed_image_topic:=/image \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  serial_baudrate:=115200
```

## Check

```bash
ros2 topic list
ros2 topic echo /perception/detections --once
curl http://127.0.0.1:8080/api/state
curl -X POST http://127.0.0.1:8080/api/drive -d '(0,0,0)'
```

## USB Role

如果 USB2 Type-C 口接扩展坞但无法发现设备，检查：

```bash
cat /sys/class/usb_role/35300000.usb-role-switch/role
```

临时切换 host：

```bash
echo sunrise | sudo -S sh -c 'echo host > /sys/class/usb_role/35300000.usb-role-switch/role'
```
