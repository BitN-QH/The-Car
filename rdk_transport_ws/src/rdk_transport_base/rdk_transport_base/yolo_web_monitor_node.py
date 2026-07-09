import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RDK Robot Control</title>
  <style>
    :root {
      color-scheme: light;
      --ink: #17202a;
      --muted: #64748b;
      --line: #d7dee8;
      --panel: #ffffff;
      --bg: #eef3f8;
      --accent: #0f766e;
      --accent-2: #2563eb;
      --warn: #b45309;
      --danger: #dc2626;
    }
    * { box-sizing: border-box; }
    body { margin: 0; font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: var(--bg); color: var(--ink); }
    main { width: min(1180px, calc(100% - 32px)); margin: 24px auto; display: grid; gap: 16px; }
    header { display: flex; align-items: end; justify-content: space-between; gap: 16px; flex-wrap: wrap; }
    h1 { margin: 0; font-size: 28px; letter-spacing: 0; }
    h2 { margin: 0 0 12px; font-size: 18px; }
    .sub { margin-top: 4px; color: var(--muted); font-size: 14px; }
    .grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; }
    .panel { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; padding: 14px; box-shadow: 0 1px 2px rgba(15, 23, 42, 0.05); }
    .metric-label { color: var(--muted); font-size: 13px; }
    .metric-value { margin-top: 6px; font-size: 22px; font-weight: 700; overflow-wrap: anywhere; }
    .command { color: var(--accent); }
    .warn { color: var(--warn); }
    .danger { color: var(--danger); }
    .stage { display: grid; grid-template-columns: minmax(0, 1.45fr) minmax(320px, 0.95fr); gap: 16px; align-items: start; }
    .video-wrap { position: relative; background: #0f172a; border-radius: 8px; overflow: hidden; aspect-ratio: 4 / 3; min-height: 240px; }
    #video, #overlay { position: absolute; inset: 0; width: 100%; height: 100%; transform: scaleY(-1); transform-origin: center; }
    #video { object-fit: contain; }
    #overlay { pointer-events: none; }
    table { width: 100%; border-collapse: collapse; font-size: 14px; }
    th, td { padding: 9px 8px; border-bottom: 1px solid var(--line); text-align: left; vertical-align: top; }
    th { color: var(--muted); font-weight: 600; }
    textarea { width: 100%; min-height: 120px; resize: vertical; border: 1px solid var(--line); border-radius: 6px; padding: 10px; font: 13px ui-monospace, SFMono-Regular, Consolas, monospace; }
    button, a.button { border: 0; border-radius: 6px; padding: 9px 12px; background: var(--accent); color: white; font-weight: 700; cursor: pointer; text-decoration: none; display: inline-flex; align-items: center; justify-content: center; min-height: 36px; }
    button.secondary { background: #334155; }
    button.motion { background: var(--accent-2); min-width: 84px; min-height: 48px; }
    button.danger { background: var(--danger); color: white; }
    .actions { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 10px; }
    .pad { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; margin-top: 10px; }
    .pad button { width: 100%; }
    .pad .wide { grid-column: 1 / -1; }
    code, pre { display: block; white-space: pre-wrap; overflow-wrap: anywhere; background: #f8fafc; border: 1px solid var(--line); border-radius: 6px; padding: 10px; max-height: 260px; overflow: auto; font-size: 12px; }
    @media (max-width: 860px) { .grid { grid-template-columns: repeat(2, minmax(0, 1fr)); } .stage { grid-template-columns: 1fr; } }
    @media (max-width: 560px) { main { width: min(100% - 20px, 1180px); margin: 14px auto; } .grid { grid-template-columns: 1fr; } h1 { font-size: 22px; } }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>RDK Robot Control</h1>
        <div class="sub">Live camera, YOLO overlay, and serial motion control</div>
      </div>
      <div class="sub" id="updated">waiting</div>
    </header>

    <section class="grid">
      <div class="panel"><div class="metric-label">YOLO</div><div class="metric-value" id="yoloStatus">waiting</div></div>
      <div class="panel"><div class="metric-label">Persons</div><div class="metric-value" id="personCount">0</div></div>
      <div class="panel"><div class="metric-label">Mission</div><div class="metric-value" id="missionState">-</div></div>
      <div class="panel"><div class="metric-label">Last command</div><div class="metric-value command" id="lastCommand">-</div></div>
    </section>

    <section class="stage">
      <div class="panel">
        <h2>Camera</h2>
        <div class="video-wrap"><img id="video" src="/stream.mjpg" alt="camera stream"><canvas id="overlay"></canvas></div>
      </div>
      <div class="panel">
        <h2>Motion</h2>
        <div class="pad">
          <button class="motion" onclick="drive('(0,0,1)')">左旋</button>
          <button class="motion" onclick="drive('(1,0,0)')">前进</button>
          <button class="motion" onclick="drive('(0,0,-1)')">右旋</button>
          <button class="motion" onclick="drive('(0,1,0)')">左移</button>
          <button class="danger" onclick="drive('(0,0,0)')">停止</button>
          <button class="motion" onclick="drive('(0,-1,0)')">右移</button>
          <button class="motion wide" onclick="drive('(-1,0,0)')">后退</button>
          <button class="motion" onclick="drive('up')">up</button>
          <button class="motion" onclick="drive('down')">down</button>
        </div>
        <div class="actions">
          <button class="danger" onclick="drive('(0,0,0)')">Stop robot</button>
          <a class="button secondary" href="/api/logs">Download logs</a>
        </div>
        <table style="margin-top:12px">
          <tbody>
            <tr><th>Fork</th><td id="forkStatus">-</td></tr>
            <tr><th>Best person</th><td id="bestPerson">-</td></tr>
            <tr><th>Offset</th><td id="personOffset">-</td></tr>
            <tr><th>Centered</th><td id="personCentered">-</td></tr>
            <tr><th>Close</th><td id="personClose">-</td></tr>
          </tbody>
        </table>
      </div>
    </section>

    <section class="panel">
      <h2>Detections</h2>
      <table>
        <thead><tr><th>Class</th><th>Score</th><th>Center x</th><th>Width</th><th>Height</th><th>Area</th></tr></thead>
        <tbody id="detections"></tbody>
      </table>
    </section>

    <section class="stage">
      <div class="panel"><h2>Diagnostics</h2><pre id="diagnostics">[]</pre></div>
      <div class="panel"><h2>Raw detection JSON</h2><code id="raw">{}</code></div>
    </section>
  </main>
  <script>
    const fmtTime = (ts) => ts ? new Date(ts * 1000).toLocaleTimeString() : "waiting";
    const fixed = (v, n = 2) => Number.isFinite(v) ? v.toFixed(n) : "-";
    let latestState = {};
    async function postAction(path, body = "") {
      const response = await fetch(path, {method: "POST", body});
      if (!response.ok) alert(await response.text());
    }
    async function drive(command) {
      const response = await fetch("/api/drive", {method: "POST", body: command});
      if (!response.ok) {
        alert(await response.text());
      }
    }
    function drawOverlay(state) {
      const img = document.getElementById("video");
      const canvas = document.getElementById("overlay");
      const rect = img.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.round(rect.width * dpr));
      canvas.height = Math.max(1, Math.round(rect.height * dpr));
      canvas.style.width = rect.width + "px";
      canvas.style.height = rect.height + "px";
      const ctx = canvas.getContext("2d");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, rect.width, rect.height);
      const raw = state.raw || {};
      const iw = Number(raw.image_width || state.image_width || 640);
      const ih = Number(raw.image_height || state.image_height || 480);
      const scale = Math.min(rect.width / iw, rect.height / ih);
      const ox = (rect.width - iw * scale) / 2;
      const oy = (rect.height - ih * scale) / 2;
      const deadband = Number(state.center_deadband_ratio || 0.12);
      ctx.fillStyle = "rgba(15,118,110,0.14)";
      ctx.fillRect(ox + iw * (0.5 - deadband) * scale, oy, iw * deadband * 2 * scale, ih * scale);
      ctx.strokeStyle = "rgba(255,255,255,0.75)";
      ctx.beginPath(); ctx.moveTo(ox + iw * 0.5 * scale, oy); ctx.lineTo(ox + iw * 0.5 * scale, oy + ih * scale); ctx.stroke();
      for (const d of state.detections || []) {
        const b = d.bbox || {};
        const x1 = Number(b.x1 ?? (Number(b.cx || 0) - Number(b.w || 0) / 2));
        const y1 = Number(b.y1 ?? (Number(b.cy || 0) - Number(b.h || 0) / 2));
        const w = Number(b.w || 0);
        const h = Number(b.h || 0);
        const isPerson = d.class_name === "person" || d.class_id === 0;
        ctx.strokeStyle = isPerson ? "#22c55e" : "#38bdf8";
        ctx.fillStyle = isPerson ? "rgba(34,197,94,0.20)" : "rgba(56,189,248,0.18)";
        ctx.lineWidth = isPerson ? 3 : 2;
        ctx.fillRect(ox + x1 * scale, oy + y1 * scale, w * scale, h * scale);
        ctx.strokeRect(ox + x1 * scale, oy + y1 * scale, w * scale, h * scale);
      }
    }
    function update(state) {
      latestState = state;
      document.getElementById("updated").textContent = "updated " + fmtTime(state.updated_at);
      document.getElementById("missionState").textContent = state.mission_state || "-";
      document.getElementById("lastCommand").textContent = state.last_command || "-";
      document.getElementById("forkStatus").textContent = state.fork_status || "-";
      const detections = state.detections || [];
      const persons = detections.filter(d => d.class_name === "person" || d.class_id === 0);
      document.getElementById("personCount").textContent = persons.length;
      const best = state.best_person || null;
      document.getElementById("bestPerson").textContent = best ? `score ${fixed(Number(best.score), 2)}, cx ${fixed(Number(best.cx_ratio) * 100, 1)}%` : "-";
      document.getElementById("personOffset").textContent = best ? fixed(Number(best.offset_ratio) * 100, 1) + "%" : "-";
      document.getElementById("personCentered").textContent = best ? (best.centered ? "yes" : "no") : "-";
      document.getElementById("personClose").textContent = best ? (best.close ? "yes" : "no") : "-";
      const stale = !state.updated_at || (Date.now() / 1000 - state.updated_at) > 2.5;
      const yolo = document.getElementById("yoloStatus");
      yolo.textContent = stale ? "waiting" : "online";
      yolo.className = "metric-value " + (stale ? "warn" : "");
      document.getElementById("detections").innerHTML = detections.map(d => {
        const b = d.bbox || {};
        return `<tr><td>${d.class_name ?? d.class_id ?? "-"}</td><td>${fixed(Number(d.score), 2)}</td><td>${fixed(Number(b.cx), 0)}</td><td>${fixed(Number(b.w), 0)}</td><td>${fixed(Number(b.h), 0)}</td><td>${fixed(Number(d.area_ratio), 3)}</td></tr>`;
      }).join("") || '<tr><td colspan="6">No detections in current frame</td></tr>';
      document.getElementById("diagnostics").textContent = JSON.stringify(state.diagnostics || [], null, 2);
      document.getElementById("raw").textContent = JSON.stringify(state.raw || {}, null, 2);
      drawOverlay(state);
    }
    const events = new EventSource("/events");
    events.onmessage = (ev) => update(JSON.parse(ev.data));
    events.onerror = () => {
      document.getElementById("yoloStatus").textContent = "disconnected";
      document.getElementById("yoloStatus").className = "metric-value warn";
    };
    window.addEventListener("resize", () => drawOverlay(latestState));
  </script>
</body>
</html>
"""


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "updated_at": 0.0,
            "mission_state": "",
            "last_command": "",
            "fork_status": "",
            "diagnostics": [],
            "detections": [],
            "raw": {},
            "image_frame_updated_at": 0.0,
            "aux_image_frame_updated_at": 0.0,
            "yolo_source": "primary",
            "center_deadband_ratio": 0.12,
            "best_person": None,
            "vlm": {
                "last_prompt": "",
                "last_response": "",
                "last_token": "",
                "prompt_time": 0.0,
                "response_time": 0.0,
                "waiting": False,
            },
            "model": {
                "last_prompt": "",
                "last_response": "",
                "prompt_time": 0.0,
                "response_time": 0.0,
                "waiting": False,
                "returncode": None,
            },
            "events": [],
        }
        self.latest_jpeg = b""
        self.latest_aux_jpeg = b""
        self.image_condition = threading.Condition(self.lock)
        self.aux_image_condition = threading.Condition(self.lock)

    def append_event_locked(self, text):
        self.data["events"].append({"time": time.time(), "text": text})
        self.data["events"] = self.data["events"][-300:]

    def update(self, **kwargs):
        with self.lock:
            self.data.update(kwargs)

    def event(self, text):
        with self.lock:
            self.append_event_locked(text)

    def snapshot(self):
        with self.lock:
            return dict(self.data)

    def update_image(self, data, source="primary"):
        if source == "aux":
            with self.aux_image_condition:
                self.latest_aux_jpeg = bytes(data)
                self.data["aux_image_frame_updated_at"] = time.time()
                self.aux_image_condition.notify_all()
            return
        with self.image_condition:
            self.latest_jpeg = bytes(data)
            self.data["image_frame_updated_at"] = time.time()
            self.image_condition.notify_all()

    def wait_for_image(self, last_seen_at, timeout=1.0, source="primary"):
        if source == "aux":
            end = time.time() + timeout
            with self.aux_image_condition:
                while self.data["aux_image_frame_updated_at"] <= last_seen_at:
                    remaining = end - time.time()
                    if remaining <= 0:
                        break
                    self.aux_image_condition.wait(remaining)
                return self.data["aux_image_frame_updated_at"], self.latest_aux_jpeg
        end = time.time() + timeout
        with self.image_condition:
            while self.data["image_frame_updated_at"] <= last_seen_at:
                remaining = end - time.time()
                if remaining <= 0:
                    break
                self.image_condition.wait(remaining)
            return self.data["image_frame_updated_at"], self.latest_jpeg


class MonitorHandler(BaseHTTPRequestHandler):
    state = None
    node = None

    def log_message(self, fmt, *args):
        return

    def send_cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Max-Age", "86400")

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_cors_headers()
        self.end_headers()

    def write_error(self, code, message):
        body = json.dumps({"ok": False, "error": str(message)}, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_cors_headers()
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.write_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if self.path == "/api/state":
            self.write_json(self.state.snapshot())
            return
        if self.path == "/api/vlm/state":
            self.write_json(self.state.snapshot().get("vlm", {}))
            return
        if self.path == "/api/model/state":
            self.write_json(self.state.snapshot().get("model", {}))
            return
        if self.path == "/api/logs":
            events = self.state.snapshot().get("events", [])
            lines = [json.dumps(item, ensure_ascii=False) for item in events]
            self.write_bytes(("\n".join(lines) + "\n").encode("utf-8"), "application/x-ndjson; charset=utf-8")
            return
        if self.path == "/events":
            self.send_response(200)
            self.send_cors_headers()
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            last_payload = None
            while True:
                payload = json.dumps(self.state.snapshot(), ensure_ascii=False)
                if payload != last_payload:
                    try:
                        self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        return
                    last_payload = payload
                time.sleep(0.25)
        if self.path in ("/stream.mjpg", "/stream_aux.mjpg"):
            self.send_response(200)
            self.send_cors_headers()
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            source = "aux" if self.path == "/stream_aux.mjpg" else "primary"
            last_seen_at = 0.0
            while True:
                last_seen_at, frame = self.state.wait_for_image(last_seen_at, timeout=2.0, source=source)
                if not frame:
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
        self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        body = self.rfile.read(length).decode("utf-8", "replace")
        if self.path == "/api/task":
            try:
                json.loads(body)
            except json.JSONDecodeError as exc:
                self.write_error(400, f"invalid JSON: {exc}")
                return
            self.node.publish_task(body)
            self.write_json({"ok": True})
            return
        if self.path == "/api/cancel":
            self.node.publish_cancel()
            self.write_json({"ok": True})
            return
        if self.path == "/api/stop":
            self.node.publish_stop()
            self.write_json({"ok": True})
            return
        if self.path == "/api/drive":
            command = body.strip()
            try:
                self.node.send_drive_command(command)
            except ValueError as exc:
                self.write_error(400, str(exc))
                return
            except (OSError, subprocess.CalledProcessError) as exc:
                self.write_error(500, f"serial write failed: {exc}")
                return
            self.write_json({"ok": True, "command": command})
            return
        if self.path == "/api/vlm/prompt":
            prompt = body.strip()
            if not prompt:
                self.write_error(400, "prompt is empty")
                return
            self.node.publish_vlm_prompt(prompt)
            self.write_json({"ok": True, "prompt": prompt})
            return
        if self.path == "/api/model/prompt":
            prompt = body.strip()
            if not prompt:
                self.write_error(400, "prompt is empty")
                return
            try:
                result = self.node.run_model_prompt(prompt)
            except subprocess.TimeoutExpired:
                self.write_error(504, "model request timed out")
                return
            except TimeoutError as exc:
                self.write_error(504, str(exc))
                return
            except Exception as exc:
                self.write_error(500, f"model request failed: {exc}")
                return
            status = 200 if result.get("ok") else 502
            body = json.dumps(result, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_cors_headers()
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path == "/api/yolo/source":
            source = body.strip().lower()
            try:
                selected = self.node.set_yolo_source(source)
            except ValueError as exc:
                self.write_error(400, str(exc))
                return
            except subprocess.CalledProcessError as exc:
                detail = ""
                if exc.stderr:
                    detail = exc.stderr.decode("utf-8", "replace") if isinstance(exc.stderr, bytes) else str(exc.stderr)
                elif exc.output:
                    detail = exc.output.decode("utf-8", "replace") if isinstance(exc.output, bytes) else str(exc.output)
                detail = detail.strip() or str(exc)
                self.write_error(409, detail)
                return
            except Exception as exc:
                self.write_error(500, f"failed to switch YOLO source: {exc}")
                return
            self.write_json({"ok": True, "source": selected})
            return
        if self.path == "/api/manual":
            data = parse_qs(body).get("data", ["stop"])[0]
            self.node.publish_manual(data)
            self.write_json({"ok": True})
            return
        self.send_error(404)

    def write_bytes(self, body, content_type):
        self.send_response(200)
        self.send_cors_headers()
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def write_json(self, payload):
        self.write_bytes(json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")


class YoloWebMonitorNode(Node):
    CLASS_DISPLAY_NAMES = {
        "book": "Delivery box",
        "microwave": "express box",
    }

    DRIVE_COMMANDS = {
        "(1,0,0)",
        "(-1,0,0)",
        "(0,1,0)",
        "(0,-1,0)",
        "(0,0,1)",
        "(0,0,-1)",
        "(0,0,0)",
        "up",
        "down",
    }

    def __init__(self):
        super().__init__("yolo_web_monitor_node")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("terminal_cmd_topic", "/mission/terminal_cmd")
        self.declare_parameter("state_topic", "/mission/state")
        self.declare_parameter("fork_status_topic", "/fork/status")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("task_topic", "/mission/task")
        self.declare_parameter("manual_cmd_topic", "/manual_cmd")
        self.declare_parameter("vlm_prompt_topic", "/prompt_text")
        self.declare_parameter("vlm_response_topic", "/tts_text")
        self.declare_parameter("model_cli", "/root/.npm-global/bin/openclaw")
        self.declare_parameter("model_http_url", "http://127.0.0.1:18789/hooks/agent")
        self.declare_parameter("model_http_token", "")
        self.declare_parameter("model_http_token_file", "")
        self.declare_parameter("model_http_fallback_cli", True)
        self.declare_parameter("model_session_key", "rdk-web-forklift")
        self.declare_parameter("model_timeout", 180)
        self.declare_parameter("model_transcript_sessions_json", "/root/.openclaw/agents/main/sessions/sessions.json")
        self.declare_parameter("model_sudo_password", "sunrise")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("compressed_image_topic", "/image")
        self.declare_parameter("aux_image_topic", "")
        self.declare_parameter("aux_compressed_image_topic", "/image_aux")
        self.declare_parameter("aux_video_device", "/dev/video2")
        self.declare_parameter("aux_video_width", 320)
        self.declare_parameter("aux_video_height", 240)
        self.declare_parameter("aux_video_fps", 30)
        self.declare_parameter("aux_raw_publish_topic", "/image_aux_raw")
        self.declare_parameter("serial_port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
        self.declare_parameter("serial_baudrate", 115200)
        self.declare_parameter("yolo_source_script", "/tmp/select_yolo_source.sh")
        self.declare_parameter("center_deadband_ratio", 0.12)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)

        self.shared_state = SharedState()
        self.center_deadband_ratio = float(self.get_parameter("center_deadband_ratio").value)
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.serial_baudrate = int(self.get_parameter("serial_baudrate").value)
        self.yolo_source_script = str(self.get_parameter("yolo_source_script").value)
        self.model_cli = str(self.get_parameter("model_cli").value).strip()
        self.model_http_url = str(self.get_parameter("model_http_url").value).strip()
        self.model_http_token = str(self.get_parameter("model_http_token").value).strip()
        self.model_http_token_file = str(self.get_parameter("model_http_token_file").value).strip()
        if not self.model_http_token and self.model_http_token_file:
            try:
                with open(self.model_http_token_file, "r", encoding="utf-8") as token_file:
                    self.model_http_token = token_file.read().strip()
            except OSError as exc:
                self.get_logger().warn(f"failed to read model_http_token_file: {exc}")
        self.model_http_fallback_cli = bool(self.get_parameter("model_http_fallback_cli").value)
        self.model_session_key = str(self.get_parameter("model_session_key").value).strip() or "rdk-web-forklift"
        self.model_timeout = int(self.get_parameter("model_timeout").value)
        self.model_transcript_sessions_json = str(self.get_parameter("model_transcript_sessions_json").value).strip()
        self.model_sudo_password = str(self.get_parameter("model_sudo_password").value)
        self.model_lock = threading.Lock()
        self.aux_video_device = str(self.get_parameter("aux_video_device").value).strip()
        if self.aux_video_device.lower() in ("none", "null", "off", "disabled"):
            self.aux_video_device = ""
        self.aux_video_width = int(self.get_parameter("aux_video_width").value)
        self.aux_video_height = int(self.get_parameter("aux_video_height").value)
        self.aux_video_fps = int(self.get_parameter("aux_video_fps").value)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)

        self.task_pub = self.create_publisher(String, str(self.get_parameter("task_topic").value), 10)
        self.manual_pub = self.create_publisher(String, str(self.get_parameter("manual_cmd_topic").value), 10)
        self.state_pub = self.create_publisher(String, str(self.get_parameter("state_topic").value), 10)
        self.vlm_prompt_pub = self.create_publisher(String, str(self.get_parameter("vlm_prompt_topic").value), 10)
        aux_raw_publish_topic = str(self.get_parameter("aux_raw_publish_topic").value).strip()
        aux_compressed_image_topic = str(self.get_parameter("aux_compressed_image_topic").value).strip()
        self.aux_raw_pub = self.create_publisher(Image, aux_raw_publish_topic, 10) if aux_raw_publish_topic else None
        self.aux_compressed_pub = (
            self.create_publisher(CompressedImage, aux_compressed_image_topic, 10)
            if aux_compressed_image_topic
            else None
        )
        self.cancel_client = self.create_client(Trigger, "/mission/cancel")
        self.create_subscription(String, str(self.get_parameter("detections_topic").value), self.on_detections, 10)
        self.create_subscription(String, str(self.get_parameter("terminal_cmd_topic").value), self.on_command, 10)
        self.create_subscription(String, str(self.get_parameter("state_topic").value), self.on_state, 10)
        self.create_subscription(String, str(self.get_parameter("fork_status_topic").value), self.on_fork_status, 10)
        self.create_subscription(String, str(self.get_parameter("vlm_response_topic").value), self.on_vlm_response, 10)
        self.create_subscription(DiagnosticArray, str(self.get_parameter("diagnostics_topic").value), self.on_diagnostics, 10)
        image_topic = str(self.get_parameter("image_topic").value)
        compressed_image_topic = str(self.get_parameter("compressed_image_topic").value).strip()
        aux_image_topic = str(self.get_parameter("aux_image_topic").value)
        aux_compressed_image_topic = str(self.get_parameter("aux_compressed_image_topic").value).strip()
        if image_topic:
            self.create_subscription(Image, image_topic, self.on_raw_image, 10)
        if compressed_image_topic:
            self.create_subscription(CompressedImage, compressed_image_topic, self.on_compressed_image, 10)
        if aux_image_topic:
            self.create_subscription(Image, aux_image_topic, self.on_aux_raw_image, 10)
        self.shared_state.update(center_deadband_ratio=self.center_deadband_ratio, yolo_source="primary")
        self._stop_aux_capture = threading.Event()
        self.aux_capture_thread = None
        if self.aux_video_device:
            self.aux_capture_thread = threading.Thread(target=self.capture_aux_camera_loop, daemon=True)
            self.aux_capture_thread.start()

        handler = type("YoloMonitorHandler", (MonitorHandler,), {})
        handler.state = self.shared_state
        handler.node = self
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(
            f"RDK robot control web monitor ready: http://{host}:{port}, serial={self.serial_port}"
        )

    def capture_aux_camera_loop(self):
        failure_logged_at = 0.0
        while not self._stop_aux_capture.is_set():
            cap = cv2.VideoCapture(self.aux_video_device, cv2.CAP_V4L2)
            if not cap.isOpened():
                now = time.time()
                if now - failure_logged_at > 5.0:
                    self.get_logger().warn(f"failed to open aux camera: {self.aux_video_device}")
                    failure_logged_at = now
                time.sleep(1.0)
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.aux_video_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.aux_video_height)
            cap.set(cv2.CAP_PROP_FPS, self.aux_video_fps)
            frame_delay = 1.0 / max(1, self.aux_video_fps)
            self.get_logger().info(f"aux camera opened: {self.aux_video_device}")
            failed_reads = 0
            while not self._stop_aux_capture.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    failed_reads += 1
                    if failed_reads >= 5:
                        now = time.time()
                        if now - failure_logged_at > 5.0:
                            self.get_logger().warn(f"aux camera read failed: {self.aux_video_device}")
                            failure_logged_at = now
                        break
                    time.sleep(0.1)
                    continue
                failed_reads = 0
                if frame.shape[1] != self.aux_video_width or frame.shape[0] != self.aux_video_height:
                    frame = cv2.resize(frame, (self.aux_video_width, self.aux_video_height))
                ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
                if ok:
                    jpeg = encoded.tobytes()
                    self.shared_state.update_image(jpeg, source="aux")
                    if self.aux_compressed_pub is not None:
                        compressed = CompressedImage()
                        compressed.header.stamp = self.get_clock().now().to_msg()
                        compressed.header.frame_id = "aux_camera"
                        compressed.format = "jpeg"
                        compressed.data = jpeg
                        self.aux_compressed_pub.publish(compressed)
                if self.aux_raw_pub is not None:
                    msg = Image()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "aux_camera"
                    msg.height = frame.shape[0]
                    msg.width = frame.shape[1]
                    msg.encoding = "bgr8"
                    msg.is_bigendian = False
                    msg.step = frame.shape[1] * 3
                    msg.data = frame.tobytes()
                    self.aux_raw_pub.publish(msg)
                time.sleep(frame_delay)
            cap.release()
            time.sleep(0.5)

    def send_drive_command(self, command):
        command = command.strip()
        if command not in self.DRIVE_COMMANDS:
            raise ValueError(f"unsupported drive command: {command}")
        if not self.serial_port:
            raise OSError("serial_port is empty")
        with open(self.serial_port, "wb", buffering=0) as serial_file:
            subprocess.run(
                [
                    "stty", "-F", self.serial_port, str(self.serial_baudrate),
                    "cs8", "-cstopb", "-parenb", "-ixon", "-ixoff", "-crtscts",
                    "raw", "-echo",
                ],
                check=True,
            )
            serial_file.write((command + "\n").encode("ascii"))
        self.shared_state.update(last_command=command)
        self.shared_state.event(f"drive: {command}")

    def set_yolo_source(self, source):
        aliases = {
            "0": "primary",
            "1": "aux",
            "main": "primary",
            "primary": "primary",
            "aux": "aux",
            "second": "aux",
        }
        selected = aliases.get(source)
        if selected is None:
            raise ValueError("unsupported YOLO source; use primary or aux")
        if self.yolo_source_script:
            subprocess.run([self.yolo_source_script, selected], check=True, capture_output=True, text=True)
        self.shared_state.update(yolo_source=selected)
        self.shared_state.event(f"yolo source: {selected}")
        return selected

    def publish_task(self, payload):
        msg = String()
        msg.data = payload
        self.task_pub.publish(msg)
        self.shared_state.event(f"task: {payload}")

    def publish_manual(self, command):
        msg = String()
        msg.data = command
        self.manual_pub.publish(msg)
        self.shared_state.event(f"manual: {command}")

    def publish_vlm_prompt(self, prompt):
        msg = String()
        msg.data = self.normalize_vlm_prompt(prompt)
        self.vlm_prompt_pub.publish(msg)
        now = time.time()
        self.shared_state.update(vlm={
            "last_prompt": prompt,
            "last_response": "",
            "last_token": "",
            "prompt_time": now,
            "response_time": 0.0,
            "waiting": True,
        })
        self.shared_state.event(f"vlm prompt: {prompt}")

    def run_model_prompt(self, prompt):
        with self.model_lock:
            prompt = str(prompt or "").strip()
            now = time.time()
            self.shared_state.update(model={
                "last_prompt": prompt,
                "last_response": "",
                "prompt_time": now,
                "response_time": 0.0,
                "waiting": True,
                "returncode": None,
            })
            self.shared_state.event(f"model prompt: {prompt}")
            full_prompt = self.build_model_prompt(prompt)
            http_error = None
            if self.model_http_url:
                try:
                    return self.run_model_prompt_openclaw_chat(prompt, full_prompt, now)
                except Exception as exc:
                    http_error = exc
                    self.shared_state.event(f"model openclaw chat failed: {exc}")
                    if not self.model_http_fallback_cli:
                        raise
            if not self.model_cli:
                if http_error is not None:
                    raise RuntimeError(f"model HTTP failed and model_cli is empty: {http_error}")
                raise RuntimeError("model_cli is empty")
            return self.run_model_prompt_cli(prompt, full_prompt, now)

    def build_model_prompt(self, prompt):
        system_prompt = (
            "You are the large-model control assistant for an RDK X5 omnidirectional forklift robot. "
            "Reply in English only. Understand the user's forklift task request and, when appropriate, "
            "explain the intended robot action clearly. If an action is unsafe or ambiguous, ask for clarification. "
            "Do not mention internal product names."
        )
        return f"{system_prompt}\n\nUser request:\n{prompt}\n"

    def run_model_prompt_openclaw_chat(self, prompt, full_prompt, prompt_time):
        session_key = self.openclaw_scoped_session_key()
        before_file, before_size = self.get_openclaw_session_file_and_size(session_key)
        status, response_body = self.submit_openclaw_hook(full_prompt)
        try:
            payload = json.loads(response_body)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"OpenClaw returned non-JSON response: {response_body[:200]}") from exc
        if not (200 <= status < 300) or payload.get("ok") is not True:
            raise RuntimeError(payload.get("error") or f"OpenClaw HTTP {status}")
        run_id = str(payload.get("runId") or "")
        response_text = self.wait_for_openclaw_assistant_response(session_key, before_file, before_size)
        response_time = time.time()
        result = {
            "ok": True,
            "prompt": prompt,
            "response": response_text,
            "returncode": 0,
            "transport": "openclaw-hooks-transcript",
            "session_key": self.model_session_key,
            "run_id": run_id,
        }
        self.shared_state.update(model={
            "last_prompt": prompt,
            "last_response": response_text,
            "prompt_time": prompt_time,
            "response_time": response_time,
            "waiting": False,
            "returncode": 0,
        })
        self.shared_state.event(f"model transcript response: {response_text[:160]}")
        return result

    def openclaw_scoped_session_key(self):
        if self.model_session_key.startswith("agent:"):
            return self.model_session_key
        return f"agent:main:{self.model_session_key}"

    def submit_openclaw_hook(self, full_prompt):
        payload = {
            "message": full_prompt,
            "wakeMode": "now",
            "deliver": False,
            "timeoutSeconds": max(10, self.model_timeout),
        }
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        headers = {"Content-Type": "application/json; charset=utf-8"}
        if self.model_http_token:
            headers["Authorization"] = f"Bearer {self.model_http_token}"
            headers["x-openclaw-token"] = self.model_http_token
        request = Request(self.model_http_url, data=body, headers=headers, method="POST")
        try:
            with urlopen(request, timeout=15) as response:
                return response.status, response.read().decode("utf-8", "replace")
        except HTTPError as exc:
            detail = exc.read().decode("utf-8", "replace")
            raise RuntimeError(f"OpenClaw HTTP {exc.code}: {detail.strip() or exc.reason}") from exc
        except URLError as exc:
            raise RuntimeError(f"OpenClaw HTTP unavailable: {exc.reason}") from exc

    def wait_for_openclaw_assistant_response(self, session_key, before_file, before_size):
        deadline = time.time() + max(10, self.model_timeout)
        last_error = ""
        while time.time() < deadline:
            try:
                session_file, current_size = self.get_openclaw_session_file_and_size(session_key)
                if session_file:
                    start_offset = before_size if session_file == before_file else 0
                    if current_size > start_offset:
                        transcript = self.read_privileged_bytes(session_file)
                        response = self.extract_assistant_text_from_transcript(transcript[start_offset:])
                        if response:
                            return response
            except Exception as exc:
                last_error = str(exc)
            time.sleep(0.5)
        detail = "OpenClaw accepted the task but no assistant response was written before timeout"
        if last_error:
            detail = f"{detail}; last transcript read error: {last_error}"
        raise TimeoutError(detail)

    def get_openclaw_session_file_and_size(self, session_key):
        if not self.model_transcript_sessions_json:
            return "", 0
        sessions = json.loads(self.read_privileged_bytes(self.model_transcript_sessions_json).decode("utf-8", "replace"))
        info = sessions.get(session_key) or sessions.get(self.model_session_key) or {}
        session_file = str(info.get("sessionFile") or "")
        if not session_file:
            return "", 0
        return session_file, self.stat_privileged_size(session_file)

    def stat_privileged_size(self, path):
        try:
            return int(self.run_privileged_python(
                "import os, sys\nprint(os.path.getsize(sys.argv[1]) if os.path.exists(sys.argv[1]) else 0)\n",
                [path],
            ).decode("utf-8", "replace").strip() or "0")
        except Exception:
            try:
                import os
                return os.path.getsize(path)
            except OSError:
                return 0

    def read_privileged_bytes(self, path):
        try:
            with open(path, "rb") as file:
                return file.read()
        except OSError:
            return self.run_privileged_python(
                "import pathlib, sys\nsys.stdout.buffer.write(pathlib.Path(sys.argv[1]).read_bytes())\n",
                [path],
            )

    def run_privileged_python(self, script, args):
        completed = subprocess.run(
            ["sudo", "-S", "python3", "-c", script, *args],
            input=(self.model_sudo_password + "\n").encode("utf-8"),
            check=False,
            capture_output=True,
            timeout=10,
        )
        if completed.returncode != 0:
            error = completed.stderr.decode("utf-8", "replace").strip()
            raise OSError(error or f"sudo python returned {completed.returncode}")
        return completed.stdout

    def extract_assistant_text_from_transcript(self, transcript_bytes):
        text = transcript_bytes.decode("utf-8", "replace")
        assistant_text = ""
        for line in text.splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            message = item.get("message") if isinstance(item, dict) else None
            if not isinstance(message, dict) or message.get("role") != "assistant":
                continue
            extracted = self.extract_message_content_text(message.get("content"))
            if extracted:
                assistant_text = extracted
        return assistant_text

    def extract_message_content_text(self, content):
        if isinstance(content, str):
            return content.strip()
        if not isinstance(content, list):
            return ""
        parts = []
        for item in content:
            if isinstance(item, dict):
                value = item.get("text") or item.get("content")
                if isinstance(value, str) and value.strip():
                    parts.append(value.strip())
            elif isinstance(item, str) and item.strip():
                parts.append(item.strip())
        return "\n".join(parts).strip()

    def run_model_prompt_http(self, prompt, full_prompt, prompt_time):
        payload = {
            "message": full_prompt,
            "sessionKey": self.model_session_key,
            "wakeMode": "now",
            "deliver": False,
            "timeoutSeconds": max(10, self.model_timeout),
        }
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        headers = {"Content-Type": "application/json; charset=utf-8"}
        if self.model_http_token:
            headers["Authorization"] = f"Bearer {self.model_http_token}"
            headers["x-openclaw-token"] = self.model_http_token
        request = Request(self.model_http_url, data=body, headers=headers, method="POST")
        try:
            with urlopen(request, timeout=max(15, self.model_timeout + 10)) as response:
                response_body = response.read().decode("utf-8", "replace")
                status = response.status
        except HTTPError as exc:
            detail = exc.read().decode("utf-8", "replace")
            raise RuntimeError(f"OpenClaw HTTP {exc.code}: {detail.strip() or exc.reason}") from exc
        except URLError as exc:
            raise RuntimeError(f"OpenClaw HTTP unavailable: {exc.reason}") from exc
        response_text = self.extract_openclaw_http_response(response_body)
        response_time = time.time()
        result = {
            "ok": 200 <= status < 300 and bool(response_text),
            "prompt": prompt,
            "response": response_text,
            "returncode": 0 if 200 <= status < 300 else status,
            "transport": "openclaw-http",
        }
        self.shared_state.update(model={
            "last_prompt": prompt,
            "last_response": response_text,
            "prompt_time": prompt_time,
            "response_time": response_time,
            "waiting": False,
            "returncode": result["returncode"],
        })
        self.shared_state.event(f"model http response: {response_text[:160]}")
        return result

    def run_model_prompt_cli(self, prompt, full_prompt, prompt_time):
        cmd = [
            "sudo",
            "-S",
            self.model_cli,
            "agent",
            "--local",
            "--json",
            "--session-key",
            self.model_session_key,
            "--message",
            full_prompt,
            "--timeout",
            str(max(10, self.model_timeout)),
        ]
        completed = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
            input="sunrise\n",
            timeout=max(15, self.model_timeout + 10),
        )
        response_text = self.extract_model_response(completed.stdout)
        if not response_text and completed.stderr:
            response_text = completed.stderr.strip()
        response_time = time.time()
        result = {
            "ok": completed.returncode == 0 and bool(response_text),
            "prompt": prompt,
            "response": response_text,
            "returncode": completed.returncode,
            "transport": "openclaw-cli",
        }
        self.shared_state.update(model={
            "last_prompt": prompt,
            "last_response": response_text,
            "prompt_time": prompt_time,
            "response_time": response_time,
            "waiting": False,
            "returncode": completed.returncode,
        })
        self.shared_state.event(f"model response: {response_text[:160]}")
        return result

    def extract_openclaw_http_response(self, output):
        text = str(output or "").strip()
        if not text:
            return ""
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return text
        if isinstance(payload, dict) and payload.get("ok") is True and payload.get("runId"):
            return f"OpenClaw accepted the forklift task on port 18789. runId: {payload.get('runId')}"
        extracted = self.find_text_in_payload(payload, (
            "response",
            "text",
            "message",
            "content",
            "output",
            "finalAssistantVisibleText",
            "finalAssistantRawText",
        ))
        return extracted.strip() if extracted else text

    def find_text_in_payload(self, payload, keys):
        if isinstance(payload, dict):
            for key in keys:
                value = payload.get(key)
                if isinstance(value, str) and value.strip():
                    return value
            for value in payload.values():
                found = self.find_text_in_payload(value, keys)
                if found:
                    return found
        elif isinstance(payload, list):
            for item in payload:
                found = self.find_text_in_payload(item, keys)
                if found:
                    return found
        return ""

    def extract_model_response(self, output):
        text = str(output or "").strip()
        if not text:
            return ""
        start = text.find("{")
        decoder = json.JSONDecoder()
        while start >= 0:
            try:
                payload, _ = decoder.raw_decode(text[start:])
                meta = payload.get("meta", {}) if isinstance(payload, dict) else {}
                for key in ("finalAssistantVisibleText", "finalAssistantRawText"):
                    value = meta.get(key)
                    if isinstance(value, str) and value.strip():
                        return value.strip()
                break
            except json.JSONDecodeError:
                start = text.find("{", start + 1)
        lines = [line.strip() for line in text.splitlines() if line.strip()]
        return lines[-1] if lines else ""

    def normalize_vlm_prompt(self, prompt):
        prompt = str(prompt or "").strip()
        has_cjk = any("\u4e00" <= ch <= "\u9fff" for ch in prompt)
        if has_cjk:
            lowered = prompt.lower()
            if "你是谁" in prompt or "介绍" in prompt:
                user_request = "Who are you? Introduce yourself as the robot vision-language assistant."
            elif "你好" in prompt:
                user_request = "Say hello and briefly introduce yourself as the robot vision-language assistant."
            elif any(word in prompt for word in ("看到", "画面", "图像", "描述", "识别")):
                user_request = "Describe the current camera image briefly and mention people, cargo, pallets, shelves, or obstacles if visible."
            else:
                user_request = "Describe the current camera image briefly and give one practical robot-assistant observation."
            return (
                "Answer in English only. Do not output Chinese characters. "
                "Use one short sentence. "
                f"User request: {user_request}"
            )
        return (
            "Answer in English only. Do not output Chinese characters. Be concise and focus on the current camera image, "
            f"robot safety, people, cargo, pallets, shelves, and obstacles. User request: {prompt}"
        )

    def clean_vlm_token(self, token):
        token = str(token or "").replace("\ufffd", "")
        return "".join(ch for ch in token if not ("\u4e00" <= ch <= "\u9fff"))

    def append_vlm_token(self, current, token):
        token = self.clean_vlm_token(token)
        if not token:
            return current
        if not current:
            return token.lstrip()
        if token[0].isspace() or current[-1].isspace():
            return current + token
        if token[0] in ",.;:!?)]}，。！？；：、）】》":
            return current + token
        if current[-1] in "([{（【《":
            return current + token
        has_cjk = any("\u4e00" <= ch <= "\u9fff" for ch in current[-1:] + token[:1])
        if has_cjk:
            return current + token
        return current + " " + token

    def publish_stop(self):
        try:
            self.send_drive_command("(0,0,0)")
        except (ValueError, OSError) as exc:
            self.shared_state.event(f"stop serial failed: {exc}")
        self.publish_manual("stop")
        self.shared_state.event("stop requested")

    def publish_cancel(self):
        if self.cancel_client.service_is_ready():
            self.cancel_client.call_async(Trigger.Request())
        else:
            msg = String()
            msg.data = "cancel requested from web; /mission/cancel unavailable"
            self.state_pub.publish(msg)
        self.publish_manual("stop")
        self.shared_state.event("cancel requested")

    def on_compressed_image(self, msg):
        self.shared_state.update_image(msg.data)

    def on_aux_compressed_image(self, msg):
        self.shared_state.update_image(msg.data, source="aux")

    def on_raw_image(self, msg):
        jpeg = self.encode_image(msg)
        if jpeg:
            self.shared_state.update_image(jpeg)

    def on_aux_raw_image(self, msg):
        jpeg = self.encode_image(msg)
        if jpeg:
            self.shared_state.update_image(jpeg, source="aux")

    def encode_image(self, msg):
        try:
            encoding = msg.encoding.lower()
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if encoding in ("bgr8", "rgb8", "8uc3"):
                image = data.reshape((msg.height, msg.step // 3, 3))[:, :msg.width, :]
                if encoding == "rgb8":
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif encoding in ("mono8", "8uc1"):
                image = data.reshape((msg.height, msg.step))[:, :msg.width]
            elif encoding in ("yuyv", "yuyv422", "yuy2"):
                image = data.reshape((msg.height, msg.step // 2, 2))[:, :msg.width, :]
                image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
            elif encoding in ("uyvy", "uyvy422"):
                image = data.reshape((msg.height, msg.step // 2, 2))[:, :msg.width, :]
                image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
            elif encoding == "nv12":
                y_size = msg.width * msg.height
                yuv = data[: y_size + y_size // 2].reshape((msg.height * 3 // 2, msg.width))
                image = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
            else:
                self.get_logger().warn(f"unsupported image encoding for web stream: {msg.encoding}")
                return b""
            ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
            return encoded.tobytes() if ok else b""
        except Exception as exc:
            self.get_logger().warn(f"failed to encode image for web stream: {exc}")
            return b""

    def on_fork_status(self, msg):
        self.shared_state.update(fork_status=msg.data)
        self.shared_state.event(f"fork: {msg.data}")

    def on_diagnostics(self, msg):
        diagnostics = []
        for status in msg.status:
            diagnostics.append({
                "name": status.name,
                "level": int.from_bytes(status.level, "little") if isinstance(status.level, bytes) else int(status.level),
                "message": status.message,
                "hardware_id": status.hardware_id,
                "values": {item.key: item.value for item in status.values},
            })
        self.shared_state.update(diagnostics=diagnostics)

    def on_detections(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"invalid detections JSON: {exc}")
            return

        width = float(payload.get("image_width", 0) or 0)
        height = float(payload.get("image_height", 0) or 0)
        image_area = width * height
        detections = []
        best_person = None
        best_rank = -1.0
        for detection in payload.get("detections", []):
            item = dict(detection)
            class_name = item.get("class_name")
            if isinstance(class_name, str):
                item["class_name"] = self.CLASS_DISPLAY_NAMES.get(class_name, class_name)
            bbox = dict(item.get("bbox") or {})
            area_ratio = 0.0
            if image_area > 0.0:
                area_ratio = max(0.0, float(bbox.get("w", 0.0) or 0.0)) * max(
                    0.0, float(bbox.get("h", 0.0) or 0.0)
                ) / image_area
            item["bbox"] = bbox
            item["area_ratio"] = area_ratio
            detections.append(item)
            is_person = item.get("class_name") == "person" or item.get("class_id") == 0
            if is_person:
                score = float(item.get("score", 0.0) or 0.0)
                rank = score + area_ratio
                if rank > best_rank and width > 0.0:
                    cx_ratio = float(bbox.get("cx", width * 0.5) or width * 0.5) / width
                    offset_ratio = cx_ratio - 0.5
                    centered = abs(offset_ratio) <= self.center_deadband_ratio
                    close = area_ratio >= 0.30 or (
                        height > 0.0 and float(bbox.get("h", 0.0) or 0.0) / height >= 0.72
                    )
                    best_rank = rank
                    best_person = {
                        "score": score,
                        "cx_ratio": cx_ratio,
                        "offset_ratio": offset_ratio,
                        "centered": centered,
                        "close": close,
                    }

        self.shared_state.update(
            updated_at=time.time(),
            detections=detections,
            raw=payload,
            best_person=best_person,
        )

    def on_command(self, msg):
        self.shared_state.update(last_command=msg.data)
        self.shared_state.event(f"command: {msg.data}")

    def on_state(self, msg):
        self.shared_state.update(mission_state=msg.data)
        self.shared_state.event(f"state: {msg.data}")

    def on_vlm_response(self, msg):
        snapshot = self.shared_state.snapshot().get("vlm", {})
        clean_token = self.clean_vlm_token(msg.data)
        response = self.append_vlm_token(snapshot.get("last_response", ""), clean_token)
        snapshot.update({
            "last_response": response,
            "last_token": clean_token,
            "response_time": time.time(),
            "waiting": False,
        })
        self.shared_state.update(vlm=snapshot)
        if clean_token:
            self.shared_state.event(f"vlm token: {clean_token}")

    def destroy_node(self):
        self._stop_aux_capture.set()
        if self.aux_capture_thread is not None:
            self.aux_capture_thread.join(timeout=1.0)
        self.httpd.shutdown()
        self.httpd.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloWebMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
