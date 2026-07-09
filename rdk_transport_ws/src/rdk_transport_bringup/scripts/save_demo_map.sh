#!/usr/bin/env bash
set -euo pipefail

source /opt/tros/humble/setup.bash
source /opt/ros/humble/setup.bash
source /home/sunrise/rdk_transport_ws/install/setup.bash

MAP_DIR="${1:-/home/sunrise/rdk_transport_ws/maps}"
MAP_NAME="${2:-rdk_x5_demo_map}"
MAP_PATH="${MAP_DIR}/${MAP_NAME}"

mkdir -p "${MAP_DIR}"

ros2 run nav2_map_server map_saver_cli -f "${MAP_PATH}"

if ros2 service list | grep -qx "/slam_toolbox/serialize_map"; then
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${MAP_PATH}'}" || true
fi

echo "Saved map files under ${MAP_DIR}"
