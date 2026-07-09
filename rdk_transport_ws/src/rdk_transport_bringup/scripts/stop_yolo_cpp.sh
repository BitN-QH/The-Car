#!/usr/bin/env bash
set -euo pipefail

python3 - <<'PY'
from pathlib import Path
import os
import signal
import time

needles = [
    'yolov8_cpp_perception.launch.py',
    'hobot_usb_cam',
    'hobot_codec',
    'hobot_codec_rep',
    'example',
    'ai_detections_json_bridge',
    'target_pose_node',
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
