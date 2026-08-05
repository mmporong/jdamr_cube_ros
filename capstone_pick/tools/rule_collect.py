"""규칙 기반 픽을 시연자로 쓰는 모방학습 데이터 수집기.

docs/10의 원래 계획("규칙 기반 파이프라인이 시연자 역할을 한다")을 구현한다.
남의 로봇 기록을 우리 몸에 맞추는 대신, 우리가 통제하는 자율 파지가 시연을
양산하므로 성공률이 높고 실패 원인이 좌표로 설명된다.

관측은 로봇 탑재 카메라(전방 RGB-D + 손목) — 규칙 기반 픽이 실제로 쓰는 것과
같고 실물 이식도 가능하다. 액션은 다음 틱의 관절 상태(state-as-action, BC 관례).

수집 루프는 원시 메시지만 잡고 디코드는 저장 단계에서 한다 — 루프 내 디코드가
sim 클록을 굶겨 에피소드가 절단됐던 사고(2026-08-05) 재발 방지.

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
OUT_ROOT = os.path.join(TOOLS, 'logs', 'rule_collect')
POSE_RE = re.compile(r'\[([-\d.eE+ ]+)\]')
AJ = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll']
CAMS = {'front': '/rgbd_camera/image', 'wrist': '/wrist_camera/image_raw'}
FPS = 20.0
COLOR_RGBA = {'blue': '0.1 0.2 0.9 1', 'red': '0.9 0.1 0.1 1', 'green': '0.1 0.8 0.1 1'}


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


def gz_xyz(model):
    for _ in range(3):
        g = POSE_RE.findall(sh(f'gz model -m {model} -p'))
        if len(g) >= 2:
            return [float(v) for v in g[-2].split()]
        time.sleep(0.2)
    return None


def stage(color, x, y, yaw):
    """로봇을 원점으로, 큐브를 앞쪽 랜덤 위치 바닥에 배치."""
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
    def __init__(self):
        super().__init__('rule_collect')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.st = {}
        self.imgs = {}
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.st.update(dict(zip(m.name, m.position))), 10)
        for key, topic in CAMS.items():
            self.create_subscription(
                Image, topic,
                (lambda k: lambda m: self.imgs.__setitem__(k, m))(key),
                qos_profile_sensor_data)
        self.bridge = CvBridge()

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9


def run_episode(node, ep, color, speed):
    x = random.uniform(0.62, 0.78)
    y = random.uniform(-0.06, 0.06)
    yaw = random.uniform(-0.3, 0.3)
    name = stage(color, x, y, yaw)
    start = gz_xyz(name) or [x, y, 0.015]

    env = dict(os.environ)
    proc = subprocess.Popen(
        ['ros2', 'run', 'capstone_pick', 'pick', '--ros-args',
         '-p', f'target_color:={color}', '-p', f'speed_scale:={speed}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)

    rec = {'front': [], 'wrist': [], 'state': []}
    t0 = node.sim_now()
    last_tick = -1
    while proc.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.01)
        el = node.sim_now() - t0
        if el > 180:
            proc.terminate()
            break
        tick = int(el * FPS)
        if tick <= last_tick or len(node.imgs) < len(CAMS):
            continue
        last_tick = tick
        rec['front'].append(node.imgs['front'])
        rec['wrist'].append(node.imgs['wrist'])
        rec['state'].append([node.st.get(j, 0.0) for j in AJ] + [node.st.get('arm_gripper', 0.0)])
    proc.wait(timeout=10)

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
            cv2.imwrite(os.path.join(d, cam, f'{i:06d}.jpg'),
                        node.bridge.imgmsg_to_cv2(msg, 'bgr8'))
    state = np.array(rec['state'], dtype=np.float32)
    action = np.vstack([state[1:], state[-1:]])   # action[t] = state[t+1] (BC 관례)
    np.save(os.path.join(d, 'state.npy'), state)
    np.save(os.path.join(d, 'action.npy'), action)
    meta = {'episode': ep, 'frames': n, 'moved_mm': round(moved * 1000, 1), 'success': True,
            'color': color, 'spawn': [round(v, 4) for v in start],
            'final': [round(v, 4) for v in fin]}
    json.dump(meta, open(os.path.join(d, 'meta.json'), 'w'), ensure_ascii=False, indent=1)
    return True, meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--color', default='blue')
    ap.add_argument('--speed', type=float, default=3.0)
    args = ap.parse_args()

    os.makedirs(OUT_ROOT, exist_ok=True)
    rclpy.init()
    node = Rec()
    t0 = time.time()
    while time.time() - t0 < 5:
        rclpy.spin_once(node, timeout_sec=0.05)

    ok = 0
    for ep in range(args.episodes):
        done = os.path.join(OUT_ROOT, f'ep{ep:03d}', 'meta.json')
        if os.path.exists(done):
            ok += 1
            continue
        s, _ = run_episode(node, ep, args.color, args.speed)
        ok += s
    print(f'수집 완료: 성공 {ok}/{args.episodes} → {OUT_ROOT}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
