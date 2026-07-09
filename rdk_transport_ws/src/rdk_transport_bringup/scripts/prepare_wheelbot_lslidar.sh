#!/usr/bin/env bash
set -euo pipefail

WS=${RDK_TRANSPORT_WS:-/home/sunrise/rdk_transport_ws}
THIRD_PARTY="${WS}/third_party/wheelbot_lslidar"
SRC="${WS}/src"

mkdir -p "${SRC}"

if [ ! -d "${THIRD_PARTY}/lslidar_msgs" ] || [ ! -d "${THIRD_PARTY}/lslidar_driver" ]; then
  echo "Missing optional Wheelbot lidar packages under ${THIRD_PARTY}" >&2
  exit 1
fi

for pkg in lslidar_msgs lslidar_driver; do
  target="${SRC}/${pkg}"
  source_dir="${THIRD_PARTY}/${pkg}"
  if [ -e "${target}" ] && [ ! -L "${target}" ]; then
    echo "Refusing to replace existing non-symlink package: ${target}" >&2
    exit 1
  fi
  ln -sfn "${source_dir}" "${target}"
  echo "Linked ${target} -> ${source_dir}"
done

echo "Ready for explicit validation:"
echo "  colcon build --packages-select lslidar_msgs lslidar_driver --symlink-install"
echo "  ros2 launch lslidar_driver rdk_n10_ftdi.launch.py"
