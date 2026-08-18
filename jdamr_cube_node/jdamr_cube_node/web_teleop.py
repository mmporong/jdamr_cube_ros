"""웹 텔레옵 — 브라우저(WASD)와 핸드폰(터치)에서 cmd_vel 을 낸다.

같은 LAN 에서 http://<파이IP>:8080 접속:
  노트북: W/S 전후 · A/D 회전 · 스페이스 정지 (키를 누르는 동안만 주행)
  핸드폰: 화면 버튼 홀드 (Wi-Fi 를 로봇과 같은 AP 에 붙일 것)

3중 데드맨 사슬 — 어디가 끊겨도 로봇은 선다:
  브라우저 100ms 킵얼라이브 → 이 노드 350ms 신선도 검사(초과 시 0 발행)
  → C++ 드라이버 400ms stale 정지 → 펌웨어 워치독 400ms

Nav2 도입 후에는 cmd_vel 경합이 생기므로 twist_mux 로 중재 예정.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty

PORT = 8080
FRESH_SEC = 0.35          # 이 시간 안에 킵얼라이브 없으면 0 발행
PUB_HZ = 15.0
MAX_LIN = 0.15            # [m/s]  서보 상한(0.17)보다 약간 낮게
MAX_ANG = 1.2             # [rad/s]

PAGE = """<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>JD-AMR 조종</title>
<style>
  body { font-family: sans-serif; background:#111; color:#eee; margin:0;
         display:flex; flex-direction:column; align-items:center; gap:12px;
         padding:16px; touch-action:none; }
  h2 { margin:4px 0 0; font-size:1.1rem; color:#8dd; }
  #grid { display:grid; grid-template-columns:repeat(3, 84px);
          grid-template-rows:repeat(3, 84px); gap:10px; margin-top:8px; }
  button { font-size:1.6rem; border:0; border-radius:14px; background:#2a2f3a;
           color:#eee; touch-action:none; -webkit-user-select:none; user-select:none; }
  button:active, button.on { background:#2e7d4f; }
  #stop { background:#7d2e2e; font-size:1.1rem; }
  #spd { width:240px; }
  #stat { font-size:.85rem; color:#999; min-height:1.2em; }
  .hint { font-size:.8rem; color:#777; }
</style></head><body>
<h2>JD-AMR 조종기</h2>
<div id="stat">대기</div>
<div id="grid">
  <span></span><button id="w">▲</button><span></span>
  <button id="a">◀</button><button id="stop">STOP</button><button id="d">▶</button>
  <span></span><button id="s">▼</button><span></span>
</div>
<label>속도 <input type="range" id="spd" min="20" max="100" value="60"></label>
<div class="hint">PC: W/A/S/D 키 (누르는 동안 주행) · 폰: 버튼 홀드<br>STOP·스페이스는 자동주행(프로브)도 중단시켜요</div>
<script>
const held = new Set();
let lastSend = 0;
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
  document.getElementById("stat").textContent =
    (t.vx || t.wz) ? `v=${t.vx.toFixed(2)} w=${t.wz.toFixed(2)}` : "정지";
}
function send() {
  lastSend = Date.now();
  fetch("/cmd", { method:"POST", body: JSON.stringify(target()) }).catch(()=>{});
}
function press(k){ if(!held.has(k)){ held.add(k); paint(); send(); } }
function release(k){ if(held.delete(k)){ paint(); send(); } }
setInterval(() => { if (held.size) send(); }, 100);   // 킵얼라이브
document.addEventListener("keydown", e => {
  const k = e.key.toLowerCase();
  if ("wasd".includes(k) && !e.repeat) press(k);
  if (e.key === " ") {
    held.clear(); paint(); send();
    fetch("/abort", { method:"POST" }).catch(()=>{});
  }
});
document.addEventListener("keyup", e => {
  const k = e.key.toLowerCase();
  if ("wasd".includes(k)) release(k);
});
for (const k of ["w","a","s","d"]) {
  const b = document.getElementById(k);
  b.addEventListener("pointerdown", e => { e.preventDefault(); press(k); });
  for (const ev of ["pointerup","pointerleave","pointercancel"])
    b.addEventListener(ev, () => release(k));
}
document.getElementById("stop").addEventListener("pointerdown", () => {
  held.clear(); paint(); send();
  fetch("/abort", { method:"POST" }).catch(()=>{});   // 자동 프로브도 중단
  document.getElementById("stat").textContent = "정지 · 자동주행 중단 요청";
});
window.addEventListener("blur", () => { held.clear(); paint(); send(); });
</script></body></html>"""


class TeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # 자동 프로브(motion_probe) 중단 신호. STOP 은 cmd_vel 0 만으로는 부족하다 —
        # 프로브가 15Hz 로 지령을 계속 밀면 우리 0 을 덮어쓰기 때문에, 프로브 자신에게
        # 멈추라고 알려야 한다.
        self.abort_pub = self.create_publisher(Empty, 'probe_abort', 10)
        self._lock = threading.Lock()
        self._vx = 0.0
        self._wz = 0.0
        self._stamp = 0.0
        self.create_timer(1.0 / PUB_HZ, self._tick)

    def abort(self):
        self.abort_pub.publish(Empty())

    def set_cmd(self, vx, wz):
        with self._lock:
            self._vx = max(-1.0, min(1.0, float(vx))) * MAX_LIN
            self._wz = max(-1.0, min(1.0, float(wz))) * MAX_ANG
            self._stamp = time.monotonic()

    def _tick(self):
        with self._lock:
            age = time.monotonic() - self._stamp
            vx, wz = self._vx, self._wz
        msg = Twist()
        if age < FRESH_SEC:
            msg.linear.x = vx
            msg.angular.z = wz
            self.pub.publish(msg)
        elif age < FRESH_SEC + 1.0:
            self.pub.publish(msg)   # 1초간 0 을 쏴 정지 확정 (데드맨 2단)
        # 이후 침묵 — 터미널 텔레옵·Nav2 와 cmd_vel 을 나눠 쓸 수 있게


def main():
    rclpy.init()
    node = TeleopNode()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):        # 접근 로그 소음 차단
            pass

        def do_GET(self):
            body = PAGE.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path == '/abort':
                node.abort()
                self.send_response(204)
                self.end_headers()
                return
            try:
                n = int(self.headers.get('Content-Length', 0))
                d = json.loads(self.rfile.read(n) or b'{}')
                node.set_cmd(d.get('vx', 0), d.get('wz', 0))
                self.send_response(204)
                self.end_headers()
            except Exception:
                self.send_response(400)
                self.end_headers()

    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f'웹 텔레옵: http://<파이IP>:{PORT} (WASD/터치)')
    try:
        rclpy.spin(node)
    finally:
        srv.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
