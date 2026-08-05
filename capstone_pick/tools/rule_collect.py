"""규칙 기반 픽을 시연자로 쓰는 모방학습 데이터 수집기.

docs/10의 원래 계획("규칙 기반 파이프라인이 시연자 역할을 한다")을 구현한다.
남의 로봇 기록을 우리 몸에 맞추는 대신, 우리가 통제하는 자율 파지가 시연을
양산하므로 성공률이 높고 실패 원인이 좌표로 설명된다.

관측은 로봇 탑재 카메라(전방 RGB-D + 손목) — 규칙 기반 픽이 실제로 쓰는 것과
같고 실물 이식도 가능하다. 액션은 다음 틱의 관절 상태(state-as-action, BC 관례).

수집 루프는 원시 메시지만 잡고 디코드는 저장 단계에서 한다 — 루프 내 디코드가
sim 클록을 굶겨 에피소드가 절단됐던 사고(2026-08-05) 재발 방지.

--- 아래는 이 파일을 처음 읽을 때의 안내 ---

모방학습(imitation learning)은 "사람이 시연한 것을 흉내 내게" 학습시키는 방식이다.
그런데 사람이 조종해 시연을 모으는 건 느리고 품질이 들쭉날쭉하다. 여기서는
사람 대신 이미 동작하는 규칙 기반 파지 노드(pick_node.py)를 시연자로 쓴다.
같은 일을 수백 번 반복시켜도 지치지 않고, 실패하면 왜 실패했는지가 좌표로 남는다.

흐름은 단순하다. 에피소드 하나가 아래 다섯 걸음이고, 이걸 --episodes 횟수만큼 돈다.

  main()
   └ run_episode()  에피소드 1회
      ① stage()          큐브를 랜덤 위치에 새로 놓는다 (매번 조금씩 다른 상황)
      ② pick 노드 실행    규칙 기반 파지가 알아서 집는다 (이게 '시연')
      ③ 20Hz로 기록       그동안 카메라 2대 + 관절 각도를 계속 담는다
      ④ 성공 판정         큐브가 실제로 움직였나 (물리 좌표로 확인)
      ⑤ 저장              성공한 것만 디스크에 남긴다

학습 데이터의 모양(에피소드 하나당):
    front/000000.jpg …   전방 RGB-D 카메라 프레임 (T장)
    wrist/000000.jpg …   손목 카메라 프레임 (T장)
    state.npy            (T, 6) 각 시점의 관절 각도 5개 + 그리퍼 1개
    action.npy           (T, 6) 그 시점에 '취해야 할 행동'
    meta.json            성공 여부, 큐브 이동거리, 색, 시작·끝 좌표

여기서 핵심 개념이 state-as-action이다. 로봇에게 보낸 명령을 따로 기록하지 않고,
"다음 순간의 관절 각도"를 그 시점의 정답 행동으로 삼는다(action[t] = state[t+1]).
행동복제(BC, Behavior Cloning)의 관례이고, 명령과 실제 도달 사이의 오차가 이미
반영된 값이라 정책이 실제로 재현 가능한 목표가 된다.

사용: python3 rule_collect.py --episodes 20 [--color blue]
"""
import argparse
import json
import math
import os
import random
import re
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

TOOLS = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.join(TOOLS, 'logs', 'rule_collect_static')
POSE_RE = re.compile(r'\[([-\d.eE+ ]+)\]')   # `gz model -p` 출력에서 [x y z] 꼴을 뽑는다
# 기록할 관절. 그리퍼(arm_gripper)는 여기 없고 아래에서 따로 붙여 6개가 된다.
AJ = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll']
# 관측 카메라 둘. 전방은 '판이 어떻게 생겼나', 손목은 '손끝이 어디 있나'를 본다.
# 사람이 물건을 집을 때 눈으로 위치를 잡고 손을 보며 미세 조정하는 것과 같은 구성이다.
CAMS = {'front': '/rgbd_camera/image', 'wrist': '/wrist_camera/image_raw'}
FPS = 20.0   # 기록 주기. 시뮬 시각 기준이라 실행이 느려져도 프레임 간격은 일정하다.
COLOR_RGBA = {'blue': '0.1 0.2 0.9 1', 'red': '0.9 0.1 0.1 1', 'green': '0.1 0.8 0.1 1'}


def sh(cmd):
    """셸 명령을 돌리고 표준출력만 문자열로 돌려준다."""
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout


def svc(s_, t_, r_):
    """Gazebo 서비스를 호출한다 (모델 생성·삭제·위치 지정).

    시뮬레이터에게 "큐브를 지워라", "여기에 새로 만들어라" 같은 요청을 보내는 통로다.
    ROS 토픽이 아니라 Gazebo 자체 서비스라 `gz service` 명령으로 부른다.
    인자는 서비스 이름 / 요청 타입 / 요청 내용(문자열)이다.
    """
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


def gz_xyz(model):
    """시뮬레이터에게 물어본 모델의 실제 위치 [x, y, z]. 실패하면 None.

    이건 로봇이 '보는' 값이 아니라 시뮬레이터만 아는 참값(ground truth)이다.
    학습 입력으로 쓰면 실물에서 재현할 수 없으므로 절대 쓰지 않고,
    "큐브가 진짜로 움직였나"를 채점하는 데만 쓴다 — 시험 문제가 아니라 답안지다.

    출력 파싱이 3회 재시도인 이유: 모델을 막 만든 직후에는 조회가 빈 응답을
    돌려주는 경우가 있다.
    """
    for _ in range(3):
        g = POSE_RE.findall(sh(f'gz model -m {model} -p'))
        if len(g) >= 2:
            return [float(v) for v in g[-2].split()]
        time.sleep(0.2)
    return None


def stage(color, x, y, yaw):
    """로봇을 원점으로, 큐브를 앞쪽 랜덤 위치 바닥에 배치.

    에피소드마다 상황을 새로 만드는 함수다. 매번 같은 자리에 두면 정책이
    "그 자리로 가는 법"만 외우고 위치가 조금만 달라져도 못 하게 된다.
    x·y·yaw를 조금씩 흔들어 놓는 이유가 그것이다.

    순서가 중요하다: 이전 큐브를 먼저 지우고(remove) → 로봇을 원점으로 되돌리고
    (set_pose) → 사라진 것을 확인한 뒤 → 새 큐브를 만든다(create). 지우기를
    건너뛰면 이전 에피소드의 큐브가 남아 정책이 어느 쪽을 봐야 할지 모르게 된다.

    큐브는 SDF(시뮬레이터가 읽는 XML)를 문자열로 만들어 넘긴다. 안에 든 물성치가
    학습 결과를 좌우한다 — 특히 마찰(mu 3.0)이 낮으면 죠 사이에서 미끄러져
    "성공적으로 집었는데 들다가 떨어지는" 시연이 쌓인다.
    """
    name = f'pick_{color}'
    for n in ('pick_object', 'pick_object_green', 'pick_object_blue', 'pick_object_orange',
              'pick_table', 'pick_blue', 'pick_red', 'pick_green', 'demo_cube', 'demo_stand'):
        svc('/world/room/remove', 'gz.msgs.Entity', f'name: "{n}" type: MODEL')
    svc('/world/room/set_pose', 'gz.msgs.Pose',
        'name: "jdamr_cube" position {x: 0.3 y: 0 z: 0.03} orientation {w: 1}')
    for _ in range(10):
        if name not in sh('gz model --list'):
            break
        time.sleep(0.5)
    c = (f'<sdf version="1.6"><model name="{name}">'
         f'<pose>{x} {y} 0.015 0 0 {yaw}</pose>'
         '<link name="link"><inertial><mass>0.04</mass>'
         '<inertia><ixx>4e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>4e-6</iyy><iyz>0</iyz><izz>4e-6</izz></inertia></inertial>'
         '<collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
         '<surface><friction><ode><mu>3.0</mu><mu2>3.0</mu2></ode>'
         '<torsional><coefficient>1.0</coefficient><use_patch_radius>true</use_patch_radius>'
         '<patch_radius>0.01</patch_radius></torsional></friction></surface></collision>'
         '<visual name="v"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
         f'<material><ambient>{COLOR_RGBA[color]}</ambient><diffuse>{COLOR_RGBA[color]}</diffuse></material>'
         '</visual></link></model></sdf>')
    r = svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"')
    if 'true' not in r:
        time.sleep(1)
        svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"')
    time.sleep(1.5)
    return name


class Rec(Node):
    """관측을 받아 두기만 하는 ROS2 노드 — 명령은 보내지 않는 순수 기록자.

    ROS2에서 노드는 데이터를 주고받는 프로그램 단위다. 여기서는 세 종류를 구독만 한다.
    구독(subscription)은 "이 토픽에 메시지가 오면 이 함수를 불러 달라"는 등록이고,
    메시지는 우리가 요청하는 게 아니라 알아서 도착한다.

    self.st   관절 이름 → 각도. 항상 최신값 하나만 들고 있는다.
    self.imgs 카메라 이름 → 원시 이미지 메시지. 마찬가지로 최신 하나만.

    최신값만 두는 게 핵심이다. 큐에 쌓아 두면 기록 시점과 어긋난 과거 프레임이
    섞이고 메모리도 먹는다. 대신 아래 run_episode가 원하는 순간에 지금 값을
    꺼내 간다 — 콜백이 밀어 넣고 루프가 꺼내 가는 구조다.
    """

    def __init__(self):
        super().__init__('rule_collect')
        # 시뮬 시각을 쓰겠다는 선언. 이게 없으면 노드가 벽시계(실제 시각)를 읽는다.
        # 시뮬은 물리 계산이 밀리면 실제보다 느리게 흐르므로, 벽시계로 20Hz를 세면
        # 실제로는 5Hz만 기록되는 식으로 어긋난다. 시뮬 시각으로 세야 프레임 간격이
        # 시뮬 안에서 일정해지고, 나중에 재생·학습할 때 속도가 맞는다.
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.st = {}
        self.imgs = {}
        # 관절 상태: 이름 배열과 각도 배열이 따로 오므로 zip으로 묶어 사전에 넣는다.
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.st.update(dict(zip(m.name, m.position))), 10)
        for key, topic in CAMS.items():
            self.create_subscription(
                Image, topic,
                # 즉시 실행 함수로 key를 가둔다. 그냥 `lambda m: ...imgs[key]...`로 쓰면
                # 두 구독 모두 루프가 끝난 뒤의 key('wrist')를 보게 되는 고전적 함정이다.
                (lambda k: lambda m: self.imgs.__setitem__(k, m))(key),
                # 센서용 QoS. 카메라는 초당 수십 장이 오니 한 장 놓쳐도 다음이 곧 온다 —
                # 재전송을 요구하는 기본 정책으로 구독하면 발행자와 짝이 안 맺어져
                # 콜백이 아예 안 불린다(에러도 경고도 없이).
                qos_profile_sensor_data)
        # ROS 이미지 메시지 ↔ OpenCV 배열 변환기. 실제 변환은 저장 단계에서만 한다.
        self.bridge = CvBridge()

    def sim_now(self):
        """현재 시뮬 시각 [초]. use_sim_time=True라 벽시계가 아니라 시뮬 클록을 읽는다."""
        return self.get_clock().now().nanoseconds / 1e9


def run_episode(node, ep, color, speed):
    """에피소드 1회: 판을 새로 깔고 → 규칙 기반 픽을 돌리며 → 20Hz로 기록 → 채점·저장.

    반환은 (성공했나, 메타정보). 실패한 에피소드는 저장하지 않는다 — 실패 시연을
    섞으면 정책이 실패하는 법도 함께 배운다.

    이 함수의 구조가 곧 데이터 수집의 구조다.
      ① stage()로 큐브 배치 → 시작 좌표를 참값으로 기억
      ② pick 노드를 별도 프로세스로 띄운다 (이게 시연자)
      ③ 그 프로세스가 살아 있는 동안 계속 돌며 20Hz 간격으로 관측을 담는다
      ④ 프로세스가 끝나면 큐브가 실제로 움직였는지 참값으로 채점
      ⑤ 성공한 것만 디스크에 쓴다

    ③의 루프에서 지켜야 할 규칙이 둘 있다.
    - 이미지 디코드를 여기서 하지 않는다. 원시 메시지 객체를 리스트에 담아만 두고
      실제 변환은 ⑤에서 한다. 루프 안에서 디코드하면 그 시간만큼 spin이 밀려
      시뮬 클록을 못 따라가고 에피소드가 중간에 잘린다(2026-08-05 사고).
    - 두 카메라가 모두 도착하기 전에는 담지 않는다. 한쪽만 있는 프레임을 담으면
      front와 wrist의 길이가 어긋나 학습 때 짝이 안 맞는다.
    """
    # 주행 없는 구성: 큐브를 팔 작업 반경(포켓 0.381 근방) 안에 바로 배치한다.
    # 주행 접근을 포함하면 데이터의 61%가 "팔은 정지, 바퀴만 굴러가는" 프레임이
    # 되는데, action에는 주행 명령이 없어 정책이 그 구간을 재현할 수 없다(실측).
    # 물체를 처음부터 팔이 닿는 곳에 두면 action(관절 6)이 전체 행동을 표현한다.
    x = 0.3 + random.uniform(0.355, 0.405)      # 로봇 기준 포켓 거리
    y = random.uniform(-0.035, 0.035)
    yaw = random.uniform(-0.3, 0.3)
    name = stage(color, x, y, yaw)
    start = gz_xyz(name) or [x, y, 0.015]

    env = dict(os.environ)
    proc = subprocess.Popen(
        ['ros2', 'run', 'capstone_pick', 'pick', '--ros-args',
         '-p', f'target_color:={color}', '-p', f'speed_scale:={speed}',
         '-p', 'skip_approach:=true', '-p', 'floor:=true'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)

    rec = {'front': [], 'wrist': [], 'state': []}
    t0 = node.sim_now()
    last_tick = -1
    # 시연자 프로세스가 살아 있는 동안 계속 돈다. poll()이 None이면 아직 실행 중.
    while proc.poll() is None:
        # spin_once가 한 번 돌아야 구독 콜백이 실행되어 self.st·self.imgs가 갱신된다.
        # 이걸 안 부르면 메시지가 도착해도 영원히 옛날 값만 보인다.
        rclpy.spin_once(node, timeout_sec=0.01)
        el = node.sim_now() - t0
        if el > 180:
            # 시연자가 무한 탐색에 빠진 경우의 안전장치 (시뮬 시각 3분)
            proc.terminate()
            break
        # 경과 시간을 20Hz 칸으로 나눈 번호. 같은 칸에서는 이미 담았으므로 건너뛴다.
        # 시간 간격을 sleep으로 맞추지 않고 이렇게 세는 이유는, spin 한 바퀴가
        # 얼마나 걸릴지 모르기 때문이다 — 빠르면 여러 번 건너뛰고 느리면 바로 담는다.
        tick = int(el * FPS)
        if tick <= last_tick or len(node.imgs) < len(CAMS):
            continue
        last_tick = tick
        # 원시 메시지를 그대로 담는다 (디코드는 저장 단계에서 — 위 docstring 참조)
        rec['front'].append(node.imgs['front'])
        rec['wrist'].append(node.imgs['wrist'])
        # 관절 5개 + 그리퍼 1개 = 6차원. 아직 안 받은 관절은 0.0으로 채운다.
        rec['state'].append([node.st.get(j, 0.0) for j in AJ] + [node.st.get('arm_gripper', 0.0)])
    proc.wait(timeout=10)

    # 채점: 큐브가 실제로 옮겨졌는가. 시연자가 "성공했다"고 말하는 것(returncode 0)만
    # 믿으면 안 된다 — 허공을 집고도 자기는 성공으로 판정하는 경우가 있어서,
    # 시뮬 참값으로 물리적 이동을 함께 확인한다. 두 조건을 모두 만족해야 성공이다.
    if len(rec['state']) < 30:
        print(f'[ep{ep}] 프레임 부족({len(rec["state"])}) — sim 시계 정지 의심', flush=True)
    fin = gz_xyz(name) or start
    moved = math.hypot(fin[0] - start[0], fin[1] - start[1])
    success = bool(proc.returncode == 0 and moved > 0.05)
    n = len(rec['state'])
    print(f'[ep{ep}] {n}프레임 | 이동 {moved * 1000:.0f}mm | rc={proc.returncode} | '
          f'{"성공" if success else "실패"}', flush=True)
    if not success or n < 30:
        return False, {'episode': ep, 'frames': n, 'moved_mm': round(moved * 1000, 1),
                       'success': False}

    d = os.path.join(OUT_ROOT, f'ep{ep:03d}')
    for cam in CAMS:
        os.makedirs(os.path.join(d, cam), exist_ok=True)
        for i, msg in enumerate(rec[cam]):
            # 여기서 처음으로 디코드한다 — 루프가 이미 끝나 시뮬 클록을 굶길 일이 없다
            cv2.imwrite(os.path.join(d, cam, f'{i:06d}.jpg'),
                        node.bridge.imgmsg_to_cv2(msg, 'bgr8'))
    state = np.array(rec['state'], dtype=np.float32)
    # 이 한 줄이 학습 데이터의 성격을 정한다 (state-as-action).
    # 정답 행동을 "다음 순간의 관절 각도"로 둔다. 정책은 지금 화면과 지금 자세를
    # 보고 "다음에 어디로 가야 하나"를 맞히도록 학습된다.
    #   state  = [s0, s1, s2, …, sT-1]
    #   action = [s1, s2, s3, …, sT-1]   ← 한 칸씩 당기고 마지막은 복제해 길이를 맞춘다
    # 마지막을 복제하는 이유: 마지막 시점에는 '다음'이 없는데 길이는 같아야 하고,
    # 의미상으로도 "여기서 멈춰 있어라"가 되어 자연스럽다.
    # action[t] = state[t+K] (0.5초 앞 목표). K=1(다음 틱)은 20Hz에서 관절 차이가
    # 0.0038rad로 노이즈 수준이라 "가만히 있기"가 최적해가 되어 학습이 무너진다
    # (실측: loss 0.1인데 평가 0/10). K=10이면 0.037rad로 10배 신호가 되고,
    # ACT의 chunk 100(5초)이 미래 궤적을 담는 구조와도 맞는다.
    K = 10
    action = np.vstack([state[K:], np.repeat(state[-1:], K, axis=0)])
    np.save(os.path.join(d, 'state.npy'), state)
    np.save(os.path.join(d, 'action.npy'), action)
    meta = {'episode': ep, 'frames': n, 'moved_mm': round(moved * 1000, 1), 'success': True,
            'color': color, 'spawn': [round(v, 4) for v in start],
            'final': [round(v, 4) for v in fin]}
    json.dump(meta, open(os.path.join(d, 'meta.json'), 'w'), ensure_ascii=False, indent=1)
    return True, meta


def main():
    """에피소드를 --episodes 횟수만큼 돌린다. 이미 받아 둔 회차는 건너뛴다."""
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--start', type=int, default=0)   # 병렬 워커별 구간 분할용
    ap.add_argument('--color', default='blue')
    ap.add_argument('--speed', type=float, default=3.0)   # 팔 동작 배율 (클수록 빠름)
    args = ap.parse_args()

    os.makedirs(OUT_ROOT, exist_ok=True)
    rclpy.init()          # ROS2 초기화 — 노드를 만들기 전에 반드시 한 번
    node = Rec()
    # /clock 수신 대기 — use_sim_time 노드는 클록이 없으면 now()가 0에 멈춰 틱이
    # 올라가지 않고 첫 프레임만 기록된다(병렬 워커1 실측: 전 에피소드 1프레임).
    t0 = time.time()
    while time.time() - t0 < 30 and node.sim_now() == 0.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.sim_now() == 0.0:
        print('/clock 미수신 — 시뮬 확인 필요', flush=True)
        rclpy.shutdown()
        return
    # 첫 메시지들이 도착할 때까지 잠깐 돌린다. 이 예열이 없으면 1번 에피소드 초반
    # 프레임이 비어 있거나 관절이 전부 0.0으로 기록된다.
    t0 = time.time()
    while time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.05)

    ok = 0
    for ep in range(args.start, args.episodes):
        # 이어받기: meta.json이 있으면 그 회차는 이미 성공 저장된 것이다.
        # 수집이 중간에 끊겨도 처음부터 다시 돌리지 않아도 된다.
        done = os.path.join(OUT_ROOT, f'ep{ep:03d}', 'meta.json')
        if os.path.exists(done):
            ok += 1
            continue
        s, _ = run_episode(node, ep, args.color, args.speed)
        ok += s   # bool이 0/1로 더해진다
    print(f'수집 완료: 성공 {ok}/{args.episodes - args.start} → {OUT_ROOT}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
