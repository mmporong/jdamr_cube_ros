"""캡스톤 조작 패널 — 색을 고르고 버튼을 눌러 파지·배치를 실행한다.

원래 과제가 "버튼을 눌러 특정 색 큐브를 집어 지정한 곳에 둔다"이므로,
그 조작부를 여기서 담당한다. 실행은 기존 파이프라인(pick_node)을 그대로
띄우는 방식이라 GUI가 로직을 중복 구현하지 않는다.

실행 (ROS 환경 source 불필요 — 내부에서 처리):
  python3 capstone_gui.py

주의: 시뮬이 먼저 떠 있어야 한다. DDS 격리(LOCALHOST/GZ_PARTITION)는
강의실 LAN 오염을 막기 위한 필수 설정이라 여기서도 그대로 넘긴다.
"""
import os
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk

# 카메라 표시는 있으면 좋고 없으면 마는 기능이다. ROS 환경 없이 패널만 띄우는
# 경우에도 조작·로그는 그대로 동작해야 하므로 임포트 실패를 삼킨다.
try:
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    CAMS_OK = True
except Exception:
    CAMS_OK = False

TOOLS = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(TOOLS, 'logs', 'gui_run.log')

# 월드의 큐브는 green / blue / orange 세 개다. 종전 '빨강(red)'은 HSV H 0~6으로
# 잡는데 그 색 큐브가 월드에 없어 눌러도 아무것도 못 찾았다(잠복 불일치).
COLORS = [('파랑', 'blue', '#2a5db0'), ('주황', 'orange', '#b0662a'), ('초록', 'green', '#2a8f3a')]
PLACES = [('옆 바닥에 놓기', 'side'), ('휴지통에 버리기', 'trash')]

ENV = {
    'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',   # 강의실 LAN DDS 오염 차단
    'GZ_PARTITION': 'lim-capstone',
}
SETUP = ('source /opt/ros/jazzy/setup.bash && '
         'source ~/jdamr_cube_ws/install/setup.bash && ')


CAM_TOPICS = [('전방 (RGB-D)', '/rgbd_camera/image'),
              ('손목', '/wrist_camera/image_raw')]
CAM_W, CAM_H = 400, 300       # 패널에 넣을 표시 크기 (로그보다 이쪽이 중요하다)


class CamFeed(Node):
    """카메라 두 대를 구독해 최신 프레임만 들고 있는다.

    화면에 붙이는 용도라 큐를 쌓지 않는다(depth=1) — 밀리면 최신만 보면 된다.
    """

    def __init__(self):
        super().__init__('capstone_gui_cams')
        self.frames = {}
        for _, topic in CAM_TOPICS:
            self.create_subscription(
                Image, topic, lambda m, t=topic: self._on(t, m), 1)

    def _on(self, topic, m):
        try:
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            if m.encoding in ('rgb8', 'rgba8'):
                a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
            self.frames[topic] = cv2.resize(a[:, :, :3], (CAM_W, CAM_H))
        except Exception:
            pass


class Panel:
    def __init__(self, root):
        self.proc = None
        self.q = queue.Queue()
        root.title('캡스톤 조작 패널')
        root.geometry('1080x720')

        # 왼쪽=조작·로그, 오른쪽=카메라. 로그와 화면을 한 창에서 같이 보려는 배치다
        # (종전에는 카메라가 Gazebo 창에 도킹돼 있어 로그와 따로 봐야 했다).
        body = ttk.Frame(root)
        body.pack(fill='both', expand=True)
        left = ttk.Frame(body)
        left.pack(side='left', fill='both', expand=True)
        right = ttk.Frame(body, padding=(0, 8, 8, 8))
        right.pack(side='right', fill='y')

        top = ttk.Frame(left, padding=12)
        top.pack(fill='x')

        ttk.Label(top, text='집을 색', font=('', 11, 'bold')).grid(row=0, column=0, sticky='w')
        self.color = tk.StringVar(value='green')
        cf = ttk.Frame(top)
        cf.grid(row=0, column=1, sticky='w', padx=8)
        for i, (label, value, _) in enumerate(COLORS):
            ttk.Radiobutton(cf, text=label, value=value, variable=self.color).grid(row=0, column=i, padx=6)

        ttk.Label(top, text='놓을 곳', font=('', 11, 'bold')).grid(row=1, column=0, sticky='w', pady=(8, 0))
        self.place = tk.StringVar(value='trash')
        pf = ttk.Frame(top)
        pf.grid(row=1, column=1, sticky='w', padx=8, pady=(8, 0))
        for i, (label, value) in enumerate(PLACES):
            ttk.Radiobutton(pf, text=label, value=value, variable=self.place).grid(row=0, column=i, padx=6)

        ttk.Label(top, text='접근 방식', font=('', 11, 'bold')).grid(row=2, column=0, sticky='w', pady=(8, 0))
        self.skip = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text='주행 생략 (큐브가 이미 팔 앞에 있을 때)',
                        variable=self.skip).grid(row=2, column=1, sticky='w', padx=8, pady=(8, 0))

        btns = ttk.Frame(left, padding=(12, 4))
        btns.pack(fill='x')
        self.run_btn = tk.Button(btns, text='▶  실행', command=self.start,
                                 bg='#2a8f3a', fg='white', font=('', 12, 'bold'),
                                 width=12, height=2)
        self.run_btn.pack(side='left')
        self.stop_btn = tk.Button(btns, text='■  중지', command=self.stop,
                                  bg='#b03a2a', fg='white', font=('', 12, 'bold'),
                                  width=12, height=2, state='disabled')
        self.stop_btn.pack(side='left', padx=8)
        self.status = ttk.Label(btns, text='대기 중', font=('', 11))
        self.status.pack(side='left', padx=16)

        ttk.Label(left, text='실행 로그', padding=(12, 4)).pack(anchor='w')
        wrap = ttk.Frame(left, padding=(12, 0, 12, 12))
        wrap.pack(fill='both', expand=True)
        # 로그는 최근 몇 줄만 보이면 된다 — 전체 기록은 logs/gui_run.log에 쌓인다.
        self.log = tk.Text(wrap, height=9, bg='#1e1e1e', fg='#d4d4d4', font=('monospace', 9))
        sb = ttk.Scrollbar(wrap, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        self.log.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        for tag, color in (('ok', '#5fd75f'), ('err', '#ff6b6b'), ('info', '#6cb6ff')):
            self.log.tag_config(tag, foreground=color)

        # 카메라 패널
        self.cam_labels = {}
        self._photos = {}          # PhotoImage 참조 유지 (놓치면 화면이 검게 뜬다)
        for name, topic in CAM_TOPICS:
            ttk.Label(right, text=name, font=('', 10, 'bold')).pack(anchor='w', pady=(4, 2))
            # 빈 PhotoImage를 미리 넣어 크기를 픽셀로 고정한다. Label에 width/height를
            # 직접 주면 이미지가 없는 동안 그 값이 '글자 수/줄 수'로 해석돼(400자 x 300줄)
            # 레이아웃이 터지고 왼쪽 조작부가 화면 밖으로 밀려난다 — 실제로 그래서
            # [실행] 버튼을 누를 수 없었다.
            blank = tk.PhotoImage(width=CAM_W, height=CAM_H)
            lb = tk.Label(right, bg='#101010', image=blank)
            lb.image = blank            # 참조 유지
            lb.pack()
            self.cam_labels[topic] = lb
        self.cam_note = ttk.Label(right, text='카메라 연결 중...', foreground='#888')
        self.cam_note.pack(anchor='w', pady=(6, 0))

        self.cams = None
        if CAMS_OK:
            threading.Thread(target=self._cam_spin, daemon=True).start()

        self.root = root
        root.after(120, self._drain)
        root.after(300, self._cam_draw)

    # ---- 카메라 ----
    def shutdown_cams(self):
        try:
            if self.cams is not None:
                self.cams.destroy_node()
            if CAMS_OK and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def _cam_spin(self):
        """별도 스레드에서 ROS를 돌린다. Tk는 메인 스레드만 만질 수 있으므로
        여기서는 프레임만 받아 두고, 그리기는 _cam_draw가 메인 스레드에서 한다."""
        try:
            os.environ.update(ENV)
            os.environ.pop('ROS_DOMAIN_ID', None)
            rclpy.init()
            self.cams = CamFeed()
            rclpy.spin(self.cams)
        except Exception as e:
            self.q.put((f'카메라 구독 실패: {e}', 'err'))

    def _cam_draw(self):
        n = 0
        if self.cams is not None:
            for topic, lb in self.cam_labels.items():
                f = self.cams.frames.get(topic)
                if f is None:
                    continue
                try:
                    ok, buf = cv2.imencode('.png', f)
                    if not ok:
                        continue
                    # Tk 8.6은 PNG를 직접 읽는다 — PIL 의존을 피하려고 이 경로를 쓴다
                    ph = tk.PhotoImage(data=buf.tobytes())
                    lb.configure(image=ph)      # 크기는 이미지가 정한다
                    self._photos[topic] = ph
                    n += 1
                except Exception:
                    pass
        if not CAMS_OK:
            self.cam_note.configure(text='카메라 표시 불가 (ROS 환경 없이 실행됨)')
        else:
            self.cam_note.configure(
                text=f'카메라 {n}/{len(CAM_TOPICS)} 수신' if n else '카메라 대기 중 — 시뮬이 떠 있나요?')
        self.root.after(200, self._cam_draw)     # 5Hz면 눈으로 보기 충분하다

    # ---- 실행 ----
    def start(self):
        if self.proc and self.proc.poll() is None:
            return
        args = [f'target_color:={self.color.get()}',
                f'place_target:={self.place.get()}',
                'detector:=hsv', 'speed_scale:=1.0']
        if self.skip.get():
            args += ['skip_approach:=true', 'floor:=true']
        params = ' '.join(f'-p {a}' for a in args)
        cmd = SETUP + f'exec ros2 run capstone_pick pick --ros-args {params}'

        env = dict(os.environ, **ENV)
        env.pop('ROS_DOMAIN_ID', None)
        os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
        # 이어 쓴다. 실행마다 덮어쓰면 직전 회차가 사라져 "3번 돌렸는데 왜 달랐나"를
        # 비교할 수 없다. 회차 머리글로 구분한다.
        self.logfile = open(LOG_PATH, 'a')
        self.logfile.write(
            f'\n===== {time.strftime("%m-%d %H:%M:%S")} '
            f'{self.color.get()} → {self.place.get()}'
            f'{" (주행 생략)" if self.skip.get() else ""} =====\n')
        # 남아 있는 노드를 먼저 정리한다. 'ros2 run'은 다시 python 노드를 띄워
        # 프로세스 트리가 끊기기 때문에, 앞선 실행이 죽지 않고 쌓이면 여러 노드가
        # 같은 로봇에 동시에 명령을 보낸다(실측: 5개 누적 — 회전이 되다 안 되다 함).
        killed = self._kill_stale()
        if killed:
            self._put(f'남아 있던 노드 {killed}개 정리', 'info')
        self._reset_scene()
        # start_new_session: 자식까지 한 번에 끊을 수 있게 프로세스 그룹을 만든다
        self.proc = subprocess.Popen(['bash', '-c', cmd], env=env, text=True,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     bufsize=1, start_new_session=True)
        self._set_running(True)
        self.log.delete('1.0', 'end')
        self._put(f'실행: {self.color.get()} → {self.place.get()}'
                  f'{" (주행 생략)" if self.skip.get() else ""}', 'info')
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        for line in self.proc.stdout:
            self.logfile.write(line)
            self.logfile.flush()
            s = line.rstrip()
            if not s:
                continue
            tag = 'err' if ('ERROR' in s or 'FAIL' in s) else (
                'ok' if ('SUCCESS' in s or 'HOLDING' in s) else '')
            self.q.put((s, tag))
        rc = self.proc.wait()
        self.q.put((f'--- 종료 (코드 {rc}) ---', 'ok' if rc == 0 else 'err'))
        self.q.put((None, None))

    # 초기 배치 — 실행할 때마다 여기로 되돌린다. 앞선 시행에서 로봇이 이동하거나
    # 큐브가 밀려나 있으면 다음 시행이 전혀 다른 조건에서 시작해 결과를 비교할 수 없다.
    # room.world의 초기 pose와 같은 값이다(둘이 어긋나면 첫 실행만 다르게 시작한다).
    # 큐브 3개는 로봇 앞 0.52m 반경에 방위각만 달리해 둔다 — 색을 고르면 조금 돌아
    # 조금 전진하면 닿는다. 휴지통은 0.64m / 76도.
    HOME = {
        'jdamr_cube': 'position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}',
        'pick_object_green': 'position: {x: 0.45, y: 0.26, z: 0.02}',    # +30도
        'pick_object_orange': 'position: {x: 0.52, y: 0.00, z: 0.02}',   #   0도
        'pick_object_blue': 'position: {x: 0.45, y: -0.26, z: 0.02}',    # -30도
    }

    def _reset_scene(self):
        """로봇과 큐브를 초기 배치로 되돌린다."""
        env = dict(os.environ, **ENV)
        ok = 0
        for name, pose in self.HOME.items():
            r = subprocess.run(
                ['gz', 'service', '-s', '/world/room/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '4000', '--req', f'name: "{name}", {pose}'],
                capture_output=True, text=True, env=env)
            ok += ('true' in r.stdout.lower())
        self._put(f'초기화: 로봇·큐브 {ok}/{len(self.HOME)}개 배치 복원', 'info')

    def _kill_stale(self):
        """남아 있는 capstone_pick 노드를 정리하고 정리한 개수를 돌려준다."""
        out = subprocess.run(['ps', '-eo', 'pid,args', '--no-headers'],
                             capture_output=True, text=True).stdout
        mine = self.proc.pid if self.proc else -1
        pids = [int(l.split()[0]) for l in out.splitlines()
                if 'capstone_pick' in l and 'capstone_gui' not in l
                and int(l.split()[0]) != mine]
        for p in pids:
            try:
                os.kill(p, signal.SIGTERM)
            except ProcessLookupError:
                pass
        return len(pids)

    def stop(self):
        """프로세스 그룹째 끊는다.

        terminate()만으로는 bash→ros2 run→python 트리의 말단 노드가 살아남아
        계속 로봇을 조종한다(실측: 중지를 눌러도 회전 탐색이 이어짐).
        """
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                self.proc.terminate()
            self._put('중지 요청 — 프로세스 그룹 종료', 'info')
            self.root.after(1500, self._force_stop)

    def _force_stop(self):
        """SIGTERM으로 안 죽은 잔여 노드를 확실히 끊는다."""
        n = self._kill_stale()
        if self.proc and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        if n:
            self._put(f'잔여 노드 {n}개 강제 종료', 'info')

    # ---- UI 갱신 ----
    def _put(self, text, tag=''):
        self.q.put((text, tag))

    def _drain(self):
        try:
            while True:
                text, tag = self.q.get_nowait()
                if text is None:
                    self._set_running(False)
                    continue
                # 진행 상황을 한눈에: 성공/실패는 상태줄에도 띄운다
                if 'PICK_SUCCESS' in text:
                    self.status.config(text='성공', foreground='#2a8f3a')
                elif 'PICK_FAIL' in text:
                    self.status.config(text='실패', foreground='#b03a2a')
                self.log.insert('end', text + '\n', tag)
                self.log.see('end')
        except queue.Empty:
            pass
        self.root.after(120, self._drain)

    def _set_running(self, on):
        self.run_btn.config(state='disabled' if on else 'normal')
        self.stop_btn.config(state='normal' if on else 'disabled')
        if on:
            self.status.config(text='실행 중…', foreground='#6cb6ff')
        elif self.status.cget('text') == '실행 중…':
            self.status.config(text='대기 중', foreground='black')


if __name__ == '__main__':
    root = tk.Tk()
    panel = Panel(root)

    def _close():
        # 창을 닫으면 돌던 파이프라인과 ROS 구독까지 같이 정리한다.
        # 남겨 두면 헤드리스로 계속 돌며 CPU를 먹는다(팬 폭주의 원인).
        panel.stop()
        panel.shutdown_cams()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', _close)
    root.mainloop()
