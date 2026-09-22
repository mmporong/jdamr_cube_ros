"""웹 텔레옵 — 브라우저와 휴대전화에서 데드맨 속도 명령을 낸다.

같은 LAN 에서 http://<파이IP>:8080 접속:
  노트북: W/S 전후 · A/D 회전 · 스페이스 정지 (키를 누르는 동안만 주행)
  핸드폰: 화면 버튼 홀드 (Wi-Fi 를 로봇과 같은 AP 에 붙일 것)

3중 데드맨 사슬 — 어디가 끊겨도 로봇은 선다:
  브라우저 100ms 킵얼라이브 → 이 노드 350ms 신선도 검사(초과 시 0 발행)
  → C++ 드라이버 400ms stale 정지 → 펌웨어 워치독 400ms

기본 출력은 기존 단독 운전과 호환되는 ``cmd_vel``이다. 안전 수동 매핑에서는
``output_topic:=cmd_vel_nav``로 실행해 velocity smoother와 Collision Monitor를
반드시 통과시킨다.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time

from geometry_msgs.msg import Twist

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Empty, String

DEFAULT_PORT = 8080
DEFAULT_FRESH_SEC = 0.35  # 이 시간 안에 킵얼라이브 없으면 0 발행
PUB_HZ = 15.0
DEFAULT_MAX_LIN = 0.15    # [m/s]  서보 상한(0.17)보다 약간 낮게
DEFAULT_MAX_ANG = 1.2     # [rad/s]

PAGE = """<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>JD-AMR 수동 매핑 조종기</title>
<style>
  :root { color-scheme:light; --ink:#10243e; --muted:#607089; --line:#d8e1ed;
          --panel:#fff; --wash:#f4f7fb; --drive:#1769e0; --stop:#c83232;
          --ok:#137a4f; }
  * { box-sizing:border-box; }
  body { font-family:"Noto Sans KR","Pretendard",Arial,sans-serif;
         background:var(--wash); color:var(--ink); margin:0; min-height:100vh;
         display:grid; place-items:center; padding:18px; touch-action:none; }
  main { width:min(100%,430px); background:var(--panel); border:1px solid var(--line);
         border-radius:18px; box-shadow:0 16px 40px rgba(24,49,83,.10);
         overflow:hidden; }
  header { padding:18px 20px 14px; border-bottom:1px solid var(--line); }
  h1 { margin:0; font-size:1.18rem; letter-spacing:-.02em; }
  #profile { margin:5px 0 0; color:var(--muted); font-size:.82rem; }
  #sync { margin-top:13px; display:flex; align-items:center; gap:8px;
          font-weight:700; font-size:.9rem; }
  #lamp { width:10px; height:10px; border-radius:50%; background:#9aa7b8; }
  #sync.ok #lamp { background:var(--ok); box-shadow:0 0 0 4px #dff4ea; }
  #sync.bad #lamp { background:var(--stop); box-shadow:0 0 0 4px #fde8e8; }
  #preflight { margin:9px 0 0; color:var(--muted); font-size:.78rem;
               line-height:1.45; }
  #grid { display:grid; grid-template-columns:repeat(3, 92px);
          grid-template-rows:repeat(3, 84px); gap:10px; justify-content:center;
          padding:22px 16px 14px; }
  button { border:1px solid #bfcddd; border-radius:14px; background:#f8fafc;
           color:var(--ink); font-size:1.65rem; font-weight:800; touch-action:none;
           -webkit-user-select:none; user-select:none; cursor:pointer; }
  button:focus-visible { outline:3px solid #8ab7f5; outline-offset:2px; }
  button.on { background:var(--drive); border-color:var(--drive); color:#fff; }
  button:disabled { opacity:.42; cursor:not-allowed; }
  #stop { background:var(--stop); border-color:var(--stop); color:#fff;
          font-size:.95rem; letter-spacing:.02em; }
  .speed { display:grid; grid-template-columns:auto 1fr auto; gap:12px;
           align-items:center; padding:4px 22px 18px; color:var(--muted);
           font-size:.85rem; }
  #spd { width:100%; accent-color:var(--drive); }
  #speedValue { color:var(--ink); font-weight:800; min-width:38px; text-align:right; }
  .telemetry { border-top:1px solid var(--line); display:grid;
               grid-template-columns:1fr 1fr; }
  .metric { padding:13px 16px; border-bottom:1px solid var(--line); }
  .metric:nth-child(odd) { border-right:1px solid var(--line); }
  .label { color:var(--muted); font-size:.72rem; margin-bottom:4px; }
  .value { font-size:.91rem; font-weight:750; font-variant-numeric:tabular-nums; }
  .hint { margin:0; padding:13px 18px 16px; color:var(--muted); font-size:.78rem;
          line-height:1.55; }
  @media (max-width:380px) {
    #grid { grid-template-columns:repeat(3,78px); grid-template-rows:repeat(3,74px); }
  }
  @media (prefers-reduced-motion:no-preference) {
    button { transition:background-color .08s, color .08s, transform .08s; }
    button.on { transform:scale(.97); }
  }
</style></head><body>
<main>
<header>
  <h1>수동 지도 생성 조종기</h1>
  <p id="profile">제어 상태 확인 중</p>
  <div id="sync"><span id="lamp"></span><span id="syncText">파이 응답 확인 중</span></div>
  <p id="preflight">출발 조건을 확인하고 있습니다.</p>
</header>
<div id="grid">
  <span></span><button id="w" aria-label="전진">▲</button><span></span>
  <button id="a" aria-label="좌회전">◀</button>
  <button id="stop">즉시 정지</button>
  <button id="d" aria-label="우회전">▶</button>
  <span></span><button id="s" aria-label="후진">▼</button><span></span>
</div>
<label class="speed"><span>속도 제한</span>
  <input type="range" id="spd" min="20" max="100" value="50">
  <span id="speedValue">50%</span>
</label>
<section class="telemetry">
  <div class="metric"><div class="label">입력 명령</div>
    <div class="value" id="requested">정지</div></div>
  <div class="metric"><div class="label">파이 수락 명령</div>
    <div class="value" id="accepted">확인 중</div></div>
  <div class="metric"><div class="label">명령 경로</div><div class="value" id="topic">확인 중</div></div>
  <div class="metric"><div class="label">최근 응답</div><div class="value" id="latency">—</div></div>
</section>
<p class="hint">W/A/S/D 또는 방향 버튼을 누르는 동안만 움직입니다. 버튼을 놓거나 창이 가려지거나 연결이 끊기면 0속도로 전환됩니다.</p>
</main>
<script>
const held = new Set();
const clientId = (globalThis.crypto && crypto.randomUUID)
  ? crypto.randomUUID() : `${Date.now()}-${Math.random()}`;
let sequence = 0;
let pendingRequest = null;
let latestAck = 0;
let lastReplyAt = 0;
let driveReady = false;
function scale() { return document.getElementById("spd").value / 100; }
function target() {
  let vx = 0, wz = 0;
  if (held.has("w")) vx += 1;
  if (held.has("s")) vx -= 1;
  if (held.has("a")) wz += 1;
  if (held.has("d")) wz -= 1;
  return { vx: vx * scale(), wz: wz * scale() };
}
function paint() {
  for (const k of ["w","a","s","d"])
    document.getElementById(k).classList.toggle("on", held.has(k));
  const t = target();
  document.getElementById("requested").textContent =
    (t.vx || t.wz) ? `전후 ${t.vx.toFixed(2)} · 회전 ${t.wz.toFixed(2)}` : "정지";
}
function send() {
  if (pendingRequest) pendingRequest.abort();
  const controller = new AbortController();
  pendingRequest = controller;
  const command = {...target(), client_id: clientId, sequence: ++sequence};
  const startedAt = performance.now();
  fetch("/cmd", {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body:JSON.stringify(command),
    cache:"no-store",
    signal:controller.signal,
  }).then(async response => {
    const state = await response.json();
    if (!response.ok) {
      renderState(state);
      const direction = (state.reason || "").split(":")[1];
      const labels = {forward:"전진", backward:"후진", left:"좌회전", right:"우회전"};
      setSync(false, direction ? `${labels[direction]} 방향 차단` : "출발 조건 차단");
      return;
    }
    latestAck = Math.max(latestAck, state.accepted_sequence || 0);
    lastReplyAt = Date.now();
    const elapsed = Math.round(performance.now() - startedAt);
    document.getElementById("latency").textContent = `${elapsed} ms`;
    renderState(state);
  }).catch(error => {
    if (error.name !== "AbortError")
      setSync(false, "명령 응답 지연 · 자동 정지");
  }).finally(() => {
    if (pendingRequest === controller) pendingRequest = null;
  });
}
function sendStopBurst() {
  held.clear(); paint(); send();
  setTimeout(send, 70);
  setTimeout(send, 160);
}
function setSync(ok, message) {
  const sync = document.getElementById("sync");
  sync.className = ok ? "ok" : "bad";
  document.getElementById("syncText").textContent = message;
}
function renderState(state) {
  document.getElementById("profile").textContent =
    state.profile_label || "JD-AMR 수동 제어";
  document.getElementById("topic").textContent = state.output_topic || "—";
  const moving = Math.abs(state.linear_x || 0) > 0.0001
    || Math.abs(state.angular_z || 0) > 0.0001;
  document.getElementById("accepted").textContent = moving
    ? `${(state.linear_x || 0).toFixed(3)} m/s · ${(state.angular_z || 0).toFixed(3)} rad/s`
    : "정지";
  const fresh = state.command_fresh === true;
  const synced = latestAck >= sequence || !held.size;
  driveReady = state.drive_ready === true;
  const directions = state.preflight?.checks?.directions || {};
  const directionKeys = {w:"forward", s:"backward", a:"left", d:"right"};
  for (const [key, direction] of Object.entries(directionKeys))
    document.getElementById(key).disabled =
      !driveReady || directions[direction]?.clear !== true;
  const blockers = state.preflight?.blockers || [];
  const blockedDirections = Object.entries(directions)
    .filter(([, value]) => value?.clear === false)
    .map(([name]) => ({forward:"전진", backward:"후진", left:"좌회전", right:"우회전"})[name]);
  document.getElementById("preflight").textContent = driveReady
    ? (blockedDirections.length
        ? `출발 조건 PASS · 라이다 차단: ${blockedDirections.join(", ")}`
        : "출발 조건 PASS · 모든 방향 주행 가능")
    : (blockers[0] || "출발 조건 확인 중 · 방향 입력 차단");
  if (!driveReady) setSync(false, "출발 차단 · 원인 확인 중");
  else if (fresh && synced)
    setSync(true, held.size ? "입력과 파이 명령 동기화" : "연결됨 · 정지 확인");
  else if (!held.size && !moving) setSync(true, "연결됨 · 정지 확인");
  else setSync(false, "명령 동기화 확인 중");
}
async function pollState() {
  try {
    const response = await fetch("/state", {cache:"no-store"});
    const state = await response.json();
    lastReplyAt = Date.now();
    latestAck = Math.max(latestAck, state.accepted_sequence || 0);
    renderState(state);
  } catch (_) {
    setSync(false, "파이 연결 끊김 · 자동 정지");
  }
}
function press(k){ if(driveReady && !held.has(k)){ held.add(k); paint(); send(); } }
function release(k){ if(held.delete(k)){ paint(); send(); } }
setInterval(() => { if (held.size) send(); }, 100);   // 킵얼라이브
document.addEventListener("keydown", e => {
  const k = e.key.toLowerCase();
  if ("wasd".includes(k) && !e.repeat) press(k);
  if (e.key === " ") {
    sendStopBurst();
    fetch("/abort", { method:"POST" }).catch(()=>{});
  }
});
document.addEventListener("keyup", e => {
  const k = e.key.toLowerCase();
  if ("wasd".includes(k)) release(k);
});
for (const k of ["w","a","s","d"]) {
  const b = document.getElementById(k);
  b.addEventListener("pointerdown", e => {
    e.preventDefault();
    b.setPointerCapture(e.pointerId);
    press(k);
  });
  for (const ev of ["pointerup","pointercancel","lostpointercapture"])
    b.addEventListener(ev, () => release(k));
}
document.getElementById("stop").addEventListener("pointerdown", () => {
  sendStopBurst();
  fetch("/abort", { method:"POST" }).catch(()=>{});   // 자동 프로브도 중단
  setSync(true, "즉시 정지 요청 전송");
});
document.getElementById("spd").addEventListener("input", e => {
  document.getElementById("speedValue").textContent = `${e.target.value}%`;
  paint();
  if (held.size) send();
});
window.addEventListener("blur", sendStopBurst);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) sendStopBurst();
});
window.addEventListener("pagehide", () => {
  const command = {vx:0, wz:0, client_id:clientId, sequence:++sequence};
  navigator.sendBeacon("/cmd", JSON.stringify(command));
});
window.addEventListener("contextmenu", e => e.preventDefault());
setInterval(() => {
  if (Date.now() - lastReplyAt > 900)
    setSync(false, "파이 응답 지연 · 자동 정지");
}, 300);
setInterval(pollState, 250);
paint(); pollState();
</script></body></html>"""


class CommandOrder:
    """Reject delayed commands that arrive after a newer browser request."""

    def __init__(self, freshness=DEFAULT_FRESH_SEC):
        """Initialize one-browser ordering with a takeover timeout."""
        self.client_id = ''
        self.sequence = -1
        self.accepted_at = 0.0
        self.freshness = freshness

    def accept(self, client_id, sequence, now):
        """Accept increasing sequence numbers from one active browser."""
        if not client_id:
            return True
        if client_id != self.client_id:
            if now - self.accepted_at < self.freshness:
                return False
            self.client_id = client_id
            self.sequence = -1
        if sequence <= self.sequence:
            return False
        self.sequence = sequence
        self.accepted_at = now
        return True


def scaled_command(vx, wz, max_linear, max_angular):
    """Validate normalized browser input and return bounded SI commands."""
    vx = float(vx)
    wz = float(wz)
    if not math.isfinite(vx) or not math.isfinite(wz):
        raise ValueError('velocity command must be finite')
    return (
        max(-1.0, min(1.0, vx)) * max_linear,
        max(-1.0, min(1.0, wz)) * max_angular,
    )


class TeleopNode(Node):
    """Publish bounded browser commands and expose synchronized state."""

    def __init__(self):
        """Create the configured velocity publisher and deadman timer."""
        super().__init__('web_teleop')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('fresh_sec', DEFAULT_FRESH_SEC)
        self.declare_parameter('max_linear', DEFAULT_MAX_LIN)
        self.declare_parameter('max_angular', DEFAULT_MAX_ANG)
        self.declare_parameter('profile_label', '단독 수동 주행')
        self.declare_parameter('require_preflight', False)
        self.declare_parameter(
            'preflight_topic', '/operator_mapping/preflight')
        self.declare_parameter('preflight_stale_sec', 1.5)
        self.output_topic = str(
            self.get_parameter('output_topic').value)
        self.port = int(self.get_parameter('port').value)
        self.fresh_sec = float(self.get_parameter('fresh_sec').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.profile_label = str(
            self.get_parameter('profile_label').value)
        self.require_preflight = bool(
            self.get_parameter('require_preflight').value)
        self.preflight_stale_sec = float(
            self.get_parameter('preflight_stale_sec').value)
        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        # 자동 프로브(motion_probe) 중단 신호. STOP 은 cmd_vel 0 만으로는 부족하다 —
        # 프로브가 15Hz 로 지령을 계속 밀면 우리 0 을 덮어쓰기 때문에, 프로브 자신에게
        # 멈추라고 알려야 한다.
        self.abort_pub = self.create_publisher(Empty, 'probe_abort', 10)
        self._lock = threading.RLock()
        self._vx = 0.0
        self._wz = 0.0
        self._stamp = 0.0
        self._command_order = CommandOrder(self.fresh_sec)
        self._preflight = None
        self._preflight_stamp = 0.0
        if self.require_preflight:
            self.create_subscription(
                String,
                str(self.get_parameter('preflight_topic').value),
                self._preflight_status,
                10,
            )
        self._publish_guard = self.create_guard_condition(
            self._publish_latest)
        self.create_timer(1.0 / PUB_HZ, self._tick)

    def abort(self):
        """Publish an immediate stop and abort any motion probe."""
        self._set_command(0.0, 0.0)
        self.abort_pub.publish(Empty())

    def _preflight_status(self, message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(document, dict):
            return
        with self._lock:
            self._preflight = document
            self._preflight_stamp = time.monotonic()

    def _drive_ready(self, now=None):
        if not self.require_preflight:
            return True
        now = time.monotonic() if now is None else now
        with self._lock:
            status = self._preflight
            age = now - self._preflight_stamp
        return (
            isinstance(status, dict)
            and status.get('ready') is True
            and 0.0 <= age <= self.preflight_stale_sec
        )

    def _direction_ready(self, vx, wz):
        if not self.require_preflight:
            return True, ''
        with self._lock:
            status = self._preflight
        directions = (
            status.get('checks', {}).get('directions', {})
            if isinstance(status, dict) else {})
        required = []
        if vx > 1e-9:
            required.append('forward')
        elif vx < -1e-9:
            required.append('backward')
        if wz > 1e-9:
            required.append('left')
        elif wz < -1e-9:
            required.append('right')
        for direction in required:
            if directions.get(direction, {}).get('clear') is not True:
                return False, direction
        return True, ''

    def _set_command(self, vx, wz):
        linear_x, angular_z = scaled_command(
            vx, wz, self.max_linear, self.max_angular)
        with self._lock:
            self._vx = linear_x
            self._wz = angular_z
            self._stamp = time.monotonic()
        self._publish_guard.trigger()

    def set_cmd(self, vx, wz, client_id='', sequence=0):
        """Accept only a newer command from the active browser client."""
        now = time.monotonic()
        requested_motion = abs(float(vx)) > 1e-9 or abs(float(wz)) > 1e-9
        if requested_motion and not self._drive_ready(now):
            return False, 'preflight_not_ready'
        direction_ready, blocked_direction = self._direction_ready(
            float(vx), float(wz))
        if requested_motion and not direction_ready:
            return False, f'direction_blocked:{blocked_direction}'
        with self._lock:
            if not self._command_order.accept(client_id, sequence, now):
                return False, 'stale_or_inactive_client'
            self._set_command(vx, wz)
        return True, ''

    def snapshot(self):
        """Return the command state acknowledged by the ROS-side node."""
        now = time.monotonic()
        with self._lock:
            age = now - self._stamp if self._stamp else None
            fresh = age is not None and age < self.fresh_sec
            vx = self._vx if fresh else 0.0
            wz = self._wz if fresh else 0.0
            accepted_sequence = self._command_order.sequence
            preflight = self._preflight
            preflight_age = (
                None if not self._preflight_stamp
                else now - self._preflight_stamp)
        drive_ready = self._drive_ready(now)
        if self.require_preflight and not drive_ready:
            if isinstance(preflight, dict):
                preflight = dict(preflight)
                blockers = list(preflight.get('blockers') or [])
                if (preflight_age is None
                        or preflight_age > self.preflight_stale_sec):
                    blockers.insert(0, '출발 점검 상태 수신 지연')
                preflight['blockers'] = blockers
            else:
                preflight = {
                    'ready': False,
                    'blockers': ['출발 점검 대기'],
                }
        return {
            'accepted_sequence': accepted_sequence,
            'angular_z': wz,
            'command_age_ms': None if age is None else round(age * 1000.0),
            'command_fresh': fresh,
            'drive_ready': drive_ready,
            'linear_x': vx,
            'output_topic': self.output_topic,
            'preflight': preflight,
            'profile_label': self.profile_label,
            'subscriber_count': self.pub.get_subscription_count(),
        }

    def _publish_latest(self):
        with self._lock:
            vx, wz = self._vx, self._wz
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

    def _tick(self):
        with self._lock:
            age = time.monotonic() - self._stamp
            vx, wz = self._vx, self._wz
        msg = Twist()
        if age < self.fresh_sec:
            msg.linear.x = vx
            msg.angular.z = wz
            self.pub.publish(msg)
        elif age < self.fresh_sec + 1.0:
            self.pub.publish(msg)   # 1초간 0 을 쏴 정지 확정 (데드맨 2단)
        # 이후 침묵 — 터미널 텔레옵·Nav2 와 cmd_vel 을 나눠 쓸 수 있게


def main():
    """Run the ROS node and its lightweight HTTP controller server."""
    rclpy.init()
    node = TeleopNode()

    class Handler(BaseHTTPRequestHandler):
        """Serve the controller UI, state and command endpoints."""

        def log_message(self, *a):        # 접근 로그 소음 차단
            """Suppress per-request access log noise."""
            pass

        def do_GET(self):
            """Return the controller page or current ROS command state."""
            if self.path == '/favicon.ico':
                self.send_response(204)
                self.end_headers()
                return
            if self.path == '/state':
                body = json.dumps(node.snapshot()).encode()
                content_type = 'application/json; charset=utf-8'
            elif self.path == '/' or self.path.startswith('/?'):
                body = PAGE.encode()
                content_type = 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store, max-age=0')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            """Accept a sequenced command or an immediate abort request."""
            if self.path == '/abort':
                node.abort()
                body = json.dumps(node.snapshot()).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != '/cmd':
                self.send_error(404)
                return
            try:
                n = int(self.headers.get('Content-Length', 0))
                d = json.loads(self.rfile.read(n) or b'{}')
                accepted, reason = node.set_cmd(
                    d.get('vx', 0), d.get('wz', 0),
                    str(d.get('client_id', '')),
                    int(d.get('sequence', 0)))
                state = node.snapshot()
                state['accepted'] = accepted
                if not accepted:
                    state['reason'] = reason
                body = json.dumps(state).encode()
                self.send_response(200 if accepted else 409)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except Exception:
                self.send_response(400)
                self.end_headers()

    srv = ThreadingHTTPServer(('0.0.0.0', node.port), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(
        f'웹 텔레옵: http://<파이IP>:{node.port} '
        f'→ {node.output_topic} (WASD/터치)')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
