#!/usr/bin/env bash
set -eo pipefail

source /opt/tros/humble/setup.bash 2>/dev/null || true
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/sunrise/lslidar_ws/install/setup.bash 2>/dev/null || true
source /home/sunrise/h30_imu_ws/install/setup.bash 2>/dev/null || true
source /home/sunrise/rdk_transport_ws/install/setup.bash 2>/dev/null || true

echo "=== serial devices ==="
ls -l /dev/serial/by-id 2>/dev/null || true

echo "=== required ROS packages ==="
for pkg in \
  rdk_transport_base rdk_transport_bringup rdk_transport_description rdk_transport_perception_cpp \
  lslidar_driver slam_toolbox hobot_usb_cam dnn_node_example nav2_bringup nav2_planner \
  nav2_controller nav2_bt_navigator nav2_map_server robot_localization; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "OK      $pkg"
  else
    echo "MISSING $pkg"
  fi
done

echo "=== active topics ==="
ros2 topic list 2>/dev/null | sort || true

echo "=== topic hz quick check ==="
for topic in /scan /scan_slam /image /perception/detections /odom /odometry/filtered /map /diagnostics; do
  echo "--- $topic"
  timeout 4 ros2 topic hz "$topic" 2>&1 | sed -n '1,6p' || true
done

echo "=== diagnostics sample ==="
timeout 5 ros2 topic echo /diagnostics --once 2>&1 | sed -n '1,120p' || true

echo "=== process summary ==="
ps -eo pid,ppid,pcpu,pmem,comm,args | grep -E 'ros2|slam_toolbox|lslidar|hobot|dnn_node|rdk_transport|rviz|nav2|amcl|map_server|controller_server|planner_server|robot_localization' | grep -v grep | sed -n '1,160p' || true
