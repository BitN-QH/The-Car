#!/usr/bin/env bash
set -euo pipefail

USB_VIDEO_DEVICE="${USB_VIDEO_DEVICE:-/dev/video0}"
IMAGE_WIDTH="${IMAGE_WIDTH:-640}"
IMAGE_HEIGHT="${IMAGE_HEIGHT:-480}"
SCORE_THRESHOLD="${SCORE_THRESHOLD:-0.35}"
LAUNCH_TARGET_POSE="${LAUNCH_TARGET_POSE:-true}"
ENABLE_WEB="${ENABLE_WEB:-false}"
LOG_DIR="${LOG_DIR:-/tmp/rdk_cpp_yolo_run}"

mkdir -p "${LOG_DIR}"

cleanup_perception() {
python3 - <<'PY'
from pathlib import Path
import os
import signal
import time

needles = [
    'yolov8_cpp_perception.launch.py',
    'perception_stub.launch.py',
    'hobot_usb_cam',
    'hobot_codec',
    'hobot_codec_rep',
    'example',
    'ai_detections_json_bridge',
    'target_pose_node',
    'yolo_detector_node',
    'websocket.launch.py',
    'websocket',
]
matched = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        comm = (proc / 'comm').read_text(errors='replace').strip()
        cmd = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode('utf-8', 'replace').strip()
    except Exception:
        continue
    nginx_web = comm == 'nginx' and '/opt/tros/humble/lib/websocket/webservice' in cmd
    if nginx_web or comm in needles or any(needle in cmd for needle in needles):
        matched.append((int(proc.name), comm, cmd))

for pid, comm, cmd in matched:
    print(f'{pid}\t{comm}\t{cmd}')
for pid, comm, cmd in matched:
    try:
        os.kill(pid, signal.SIGTERM)
        print(f'SIGTERM {pid} {comm}')
    except ProcessLookupError:
        pass
    except Exception as exc:
        print(f'TERM_FAILED {pid} {comm}: {exc}')

time.sleep(1.5)
for pid, comm, cmd in matched:
    if Path(f'/proc/{pid}').exists():
        try:
            os.kill(pid, signal.SIGKILL)
            print(f'SIGKILL {pid} {comm}')
        except Exception as exc:
            print(f'KILL_FAILED {pid} {comm}: {exc}')
PY
}

set +u
source /opt/tros/humble/setup.bash
source /home/sunrise/rdk_transport_ws/install/setup.bash
set -u

echo "[start_yolo_cpp] stopping old perception processes"
cleanup_perception

echo "[start_yolo_cpp] starting C++ YOLOv8n perception"
cd "${LOG_DIR}"
nohup ros2 launch rdk_transport_perception_cpp yolov8_cpp_perception.launch.py \
  usb_video_device:="${USB_VIDEO_DEVICE}" \
  image_width:="${IMAGE_WIDTH}" \
  image_height:="${IMAGE_HEIGHT}" \
  score_threshold:="${SCORE_THRESHOLD}" \
  launch_target_pose:="${LAUNCH_TARGET_POSE}" \
  > "${LOG_DIR}/yolov8_cpp_perception.log" 2>&1 &
echo "$!" > "${LOG_DIR}/yolov8_cpp_perception.pid"
echo "[start_yolo_cpp] launch pid: $(cat "${LOG_DIR}/yolov8_cpp_perception.pid")"

if [[ "${ENABLE_WEB}" == "true" ]]; then
  echo "[start_yolo_cpp] starting websocket display on http://10.220.190.26:8000"
  nohup ros2 launch websocket websocket.launch.py \
    websocket_image_topic:=/image \
    websocket_image_type:=mjpeg \
    websocket_smart_topic:=/hobot_dnn_detection \
    > "${LOG_DIR}/websocket.log" 2>&1 &
  echo "$!" > "${LOG_DIR}/websocket.pid"
  echo "[start_yolo_cpp] websocket pid: $(cat "${LOG_DIR}/websocket.pid")"
fi

echo "[start_yolo_cpp] logs: ${LOG_DIR}/yolov8_cpp_perception.log"
