#!/usr/bin/env bash
set -euo pipefail

SERIAL_PORT="${SERIAL_PORT:-}"
SERIAL_BAUDRATE="${SERIAL_BAUDRATE:-115200}"
ALLOW_PROTECTED_SERIAL="${ALLOW_PROTECTED_SERIAL:-false}"

if [[ -z "${SERIAL_PORT}" ]]; then
  echo "Set SERIAL_PORT to the STM32 /dev/serial/by-id path." >&2
  echo "Do not use the known lidar or IMU devices:" >&2
  echo "  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0" >&2
  echo "  /dev/serial/by-id/usb-1a86_USB_Single_Serial_5A37017495-if00" >&2
  exit 2
fi

source /opt/tros/humble/setup.bash 2>/dev/null || true
source /opt/ros/humble/setup.bash
source /home/sunrise/rdk_transport_ws/install/setup.bash

exec ros2 launch rdk_transport_base base_serial.launch.py \
  serial_port:="${SERIAL_PORT}" \
  serial_baudrate:="${SERIAL_BAUDRATE}" \
  serial_write_commands:=true \
  allow_protected_serial:="${ALLOW_PROTECTED_SERIAL}" \
  serial_min_interval_sec:=0.2
