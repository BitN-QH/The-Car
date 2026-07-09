#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="${WORKSPACE:-/home/sunrise/rdk_transport_ws}"
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
LOG_DIR="${LOG_DIR:-/tmp/rdk_web_stack}"
SERIAL_PORT="${SERIAL_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}"
SERIAL_BAUDRATE="${SERIAL_BAUDRATE:-115200}"
SUDO_PASSWORD="${SUDO_PASSWORD:-sunrise}"
HTTP_PORT="${HTTP_PORT:-8080}"
HTTP_HOST="${HTTP_HOST:-0.0.0.0}"
CAMERA_DEVICE="${CAMERA_DEVICE:-/dev/video0}"
IMAGE_WIDTH="${IMAGE_WIDTH:-320}"
IMAGE_HEIGHT="${IMAGE_HEIGHT:-240}"
YOLO_MODEL="${YOLO_MODEL:-/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin}"
YOLO_SCORE_THRESHOLD="${YOLO_SCORE_THRESHOLD:-0.35}"
VLM_MODEL_DIR="${VLM_MODEL_DIR:-$WORKSPACE/models/vlm/smolvlm2_500m}"
VLM_LAUNCH="${VLM_LAUNCH:-$WORKSPACE/vlm_launch/vlm_smolvlm2_500m_usb_30fps.launch.py}"
OPENCLAW_TOKEN_FILE="${OPENCLAW_TOKEN_FILE:-/home/sunrise/.rdk_openclaw_hook_token}"

mkdir -p "$LOG_DIR"

sudo_write() {
  local value="$1"
  local target="$2"
  printf '%s\n' "$SUDO_PASSWORD" | sudo -S sh -c "printf '%s\n' '$value' > '$target'"
}

source_ros() {
  set +u
  if [ -f /opt/tros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/tros/humble/setup.bash
  else
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi
  if [ -f "$WORKSPACE/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WORKSPACE/install/setup.bash"
  fi
  set -u
}

run_bg() {
  local name="$1"
  shift
  local log="$LOG_DIR/$name.log"
  echo "[start] $name -> $log"
  nohup bash -lc "$*" >"$log" 2>&1 &
  echo $! >"$LOG_DIR/$name.pid"
}

stop_patterns() {
  local patterns=(
    "yolo_web_monitor.launch.py"
    "yolo_web_monitor_node"
    "vlm_smolvlm2_500m_usb_30fps.launch.py"
    "hobot_llamacpp"
    "hobot_usb_cam"
    "hobot_codec_republish"
    "dnn_node_example/example"
    "dnn_node_example example"
    "ai_detections_json_bridge"
  )
  for pattern in "${patterns[@]}"; do
    pkill -f "$pattern" 2>/dev/null || true
  done
}

stop_stack() {
  echo "[stop] stopping YOLO, VLM, and web monitor processes"
  stop_patterns
  sleep 2
  stop_patterns
  sleep 1
}

reset_serial() {
  echo "[serial] checking $SERIAL_PORT"
  if stty -F "$SERIAL_PORT" -a >/dev/null 2>&1; then
    echo "[serial] OK"
    return 0
  fi

  echo "[serial] not healthy; trying to reset the USB branch for CH340"
  local devpath=""
  if [ -e /dev/ttyUSB0 ]; then
    devpath="$(udevadm info -q path -n /dev/ttyUSB0 2>/dev/null || true)"
  elif [ -e "$SERIAL_PORT" ]; then
    devpath="$(udevadm info -q path -n "$SERIAL_PORT" 2>/dev/null || true)"
  fi

  local usb_dev=""
  if [ -n "$devpath" ]; then
    usb_dev="$(echo "$devpath" | grep -oE '1-[0-9.]+' | tail -1 || true)"
  fi
  if [ -z "$usb_dev" ]; then
    usb_dev="$(for d in /sys/bus/usb/devices/1-*; do
      [ -f "$d/idVendor" ] || continue
      if [ "$(cat "$d/idVendor" 2>/dev/null)" = "1a86" ] && [ "$(cat "$d/idProduct" 2>/dev/null)" = "7523" ]; then
        basename "$d"
      fi
    done | tail -1)"
  fi
  if [ -n "$usb_dev" ] && [ -e "/sys/bus/usb/devices/$usb_dev/authorized" ]; then
    echo "[serial] reset USB device $usb_dev"
    sudo_write 0 "/sys/bus/usb/devices/$usb_dev/authorized" || true
    sleep 2
    sudo_write 1 "/sys/bus/usb/devices/$usb_dev/authorized" || true
    sleep 4
  fi

  if stty -F "$SERIAL_PORT" -a >/dev/null 2>&1; then
    echo "[serial] OK after reset"
  else
    echo "[serial] still unavailable: $SERIAL_PORT"
  fi
}

start_vlm() {
  if [ ! -d "$VLM_MODEL_DIR" ]; then
    echo "[warn] VLM model dir missing: $VLM_MODEL_DIR"
    return 1
  fi
  if [ ! -f "$VLM_LAUNCH" ]; then
    echo "[warn] VLM launch missing: $VLM_LAUNCH"
    return 1
  fi
  run_bg "vlm" "cd '$VLM_MODEL_DIR' && source '$SCRIPT_PATH' __source_ros_only && exec ros2 launch '$VLM_LAUNCH' device:='$CAMERA_DEVICE' image_width:='$IMAGE_WIDTH' image_height:='$IMAGE_HEIGHT' prompt_topic:=/prompt_text text_topic:=/tts_text"
}

start_yolo() {
  run_bg "yolo_dnn" "source '$SCRIPT_PATH' __source_ros_only && cd \"\$(ros2 pkg prefix dnn_node_example)/lib/dnn_node_example\" && exec ros2 run dnn_node_example example --ros-args --log-level warn -p config_file:=config/yolov8workconfig.json -p dump_render_img:=0 -p feed_type:=1 -p is_shared_mem_sub:=1 -p sharedmem_img_topic_name:=/hbmem_img -p msg_pub_topic_name:=/hobot_dnn_detection"
  run_bg "yolo_bridge" "source '$SCRIPT_PATH' __source_ros_only && exec ros2 run rdk_transport_perception_cpp ai_detections_json_bridge --ros-args -r __node:=ai_detections_json_bridge -p input_topic:=/hobot_dnn_detection -p output_topic:=/perception/detections -p model_path:='$YOLO_MODEL' -p score_threshold:=$YOLO_SCORE_THRESHOLD -p image_width:=$IMAGE_WIDTH -p image_height:=$IMAGE_HEIGHT"
}

start_web_monitor() {
  run_bg "web_monitor" "cd '$WORKSPACE' && source '$SCRIPT_PATH' __source_ros_only && exec ros2 launch rdk_transport_base yolo_web_monitor.launch.py port:='$HTTP_PORT' host:='$HTTP_HOST' compressed_image_topic:=/image aux_video_device:=none aux_compressed_image_topic:=/image_aux serial_port:='$SERIAL_PORT' serial_baudrate:='$SERIAL_BAUDRATE' model_http_token_file:='$OPENCLAW_TOKEN_FILE'"
}

start_stack() {
  stop_stack
  reset_serial || true
  echo "[start] starting VLM camera pipeline"
  start_vlm
  echo "[start] waiting for camera/shared-memory topics"
  sleep 8
  echo "[start] starting YOLO detection pipeline"
  start_yolo
  sleep 2
  echo "[start] starting 8080 web monitor"
  start_web_monitor
  sleep 3
  status_stack
}

status_stack() {
  echo "=== processes ==="
  ps -ef | grep -E 'yolo_web_monitor|hobot_llamacpp|hobot_usb_cam|hobot_codec_republish|dnn_node_example|ai_detections_json_bridge|openclaw' | grep -v grep || true
  echo
  echo "=== ports ==="
  ss -ltnp 2>/dev/null | grep -E ":($HTTP_PORT|18789|9090)\b" || true
  echo
  echo "=== serial ==="
  ls -l "$SERIAL_PORT" /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
  stty -F "$SERIAL_PORT" -a >/dev/null 2>&1 && echo "serial OK: $SERIAL_PORT" || echo "serial unavailable: $SERIAL_PORT"
  echo
  echo "=== ros topics ==="
  source_ros
  ros2 topic info /prompt_text 2>/dev/null || true
  ros2 topic info /tts_text 2>/dev/null || true
  ros2 topic info /perception/detections 2>/dev/null || true
}

show_logs() {
  echo "Logs are in $LOG_DIR"
  ls -lh "$LOG_DIR"/*.log 2>/dev/null || true
  echo
  for log in "$LOG_DIR"/*.log; do
    [ -f "$log" ] || continue
    echo "=== tail: $log ==="
    tail -n 40 "$log" || true
  done
}

case "${1:-start}" in
  __source_ros_only)
    source_ros
    ;;
  start)
    start_stack
    ;;
  stop)
    stop_stack
    ;;
  restart)
    start_stack
    ;;
  status)
    status_stack
    ;;
  logs)
    show_logs
    ;;
  reset-serial)
    reset_serial
    ;;
  *)
    echo "Usage: $0 {start|stop|restart|status|logs|reset-serial}"
    exit 2
    ;;
esac
