#!/usr/bin/env bash
set -euo pipefail

source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash

if [ -f /home/sunrise/lslidar_ws/install/setup.bash ]; then
  source /home/sunrise/lslidar_ws/install/setup.bash
fi

if [ -f /home/sunrise/h30_imu_ws/install/setup.bash ]; then
  source /home/sunrise/h30_imu_ws/install/setup.bash
fi

source /home/sunrise/rdk_transport_ws/install/setup.bash

exec ros2 launch rdk_transport_bringup bringup_all.launch.py "$@"
