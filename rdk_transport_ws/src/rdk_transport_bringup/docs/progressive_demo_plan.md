# Progressive Demo Implementation

This workspace implements a safe staged route for the RDK X5 transport robot.
STM32 serial writes remain disabled until the final hardware stage.

## Task 1: SLAM Mapping

Start the existing bringup first. Disable hardware nodes if they are already running.

```bash
bash /home/sunrise/rdk_transport_ws/src/rdk_transport_bringup/scripts/start_bringup.sh \
  launch_lidar:=false launch_imu:=false launch_usb_camera:=false
```

Start SLAM on the RDK X5:

```bash
ros2 launch rdk_transport_bringup slam_mapping.launch.py
```

Experimental H30 IMU odometry mapping:

```bash
ros2 launch rdk_transport_bringup slam_mapping_imu.launch.py
```

Keep the robot still for about one second after startup so
`h30_imu_odom_node` can estimate acceleration bias. This mode is only suitable
for short, low-speed mapping segments because IMU-only distance integration
drifts without wheel encoders.

Checks:

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /scan
ros2 topic hz /odom
```

## Task 2: Save the Map

```bash
bash /home/sunrise/rdk_transport_ws/src/rdk_transport_bringup/scripts/save_demo_map.sh
```

Expected files:

```text
/home/sunrise/rdk_transport_ws/maps/rdk_x5_demo_map.yaml
/home/sunrise/rdk_transport_ws/maps/rdk_x5_demo_map.pgm
```

If `/slam_toolbox/serialize_map` exists, the script also tries to save the
posegraph used by slam_toolbox localization.

## Task 3: Localization

```bash
ros2 launch rdk_transport_bringup localization.launch.py
```

Checks:

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
```

## Task 4: Segmented Navigation

This node sends short `/manual_cmd` segments and forces `stop` after each move.

```bash
ros2 topic echo /stm32/tx_preview
ros2 launch rdk_transport_base segmented_nav.launch.py \
  route:=forward,left,rotate_right,forward,stop
```

The default route should produce preview commands such as `(1,0,0)` and
`(0,0,0)`. If TF is unavailable or jumps too far, the node stops and publishes a
fault on `/mission/state`.

## Task 5: YOLO Interface Stub

The current node is a safe interface stub. It republishes `/image` to
`/perception/debug_image` and publishes empty detection JSON on
`/perception/detections`. Replace the internals with hobot_dnn inference after a
real model is available.

```bash
ros2 launch rdk_transport_base perception_stub.launch.py
ros2 topic echo /perception/detections
```

Detection JSON format:

```json
{
  "image_width": 640,
  "image_height": 480,
  "detections": [
    {
      "class": "pallet",
      "confidence": 0.9,
      "bbox": {"cx": 320, "cy": 240, "w": 120, "h": 80},
      "depth_m": 1.2
    }
  ]
}
```

## Task 6: Target Pose

`target_pose_node` consumes the JSON above and publishes
`/perception/target_poses` as `geometry_msgs/PoseArray`. With monocular USB
camera only, it uses the box center bearing and a default distance. With RGB-D,
add `depth_m` per detection.

Manual test:

```bash
ros2 topic pub --once /perception/detections std_msgs/msg/String \
  "{data: '{\"image_width\":640,\"image_height\":480,\"detections\":[{\"class\":\"pallet\",\"confidence\":0.9,\"bbox\":{\"cx\":320,\"cy\":240,\"w\":120,\"h\":80},\"depth_m\":1.2}]}' }"
ros2 topic echo /perception/target_poses --once
```

## Task 7: Safe Mission Demo

```bash
ros2 topic echo /stm32/tx_preview
ros2 launch rdk_transport_base demo_mission.launch.py
```

This is a software-only mission skeleton. It sends manual movement commands and
fork `up/down` commands, then stops.

## Task 8: STM32 Serial

Only after the software route is verified:

```bash
ros2 launch rdk_transport_base base_serial.launch.py \
  serial_port:=/dev/serial/by-id/<STM32_DEVICE_ID> \
  serial_baudrate:=115200 \
  serial_write_commands:=true \
  serial_min_interval_sec:=0.3
```

Do not use the N10 or H30 serial IDs for STM32.
