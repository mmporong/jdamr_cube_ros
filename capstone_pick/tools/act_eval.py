"""ACT 정책 폐루프 평가기 — autoresearch 평가 계약 구현.

학습된 정책을 시뮬에 붙여 N회 실행: 성공 에피소드들의 스폰 분포에서 무대를
샘플(±5mm 지터)하고, 정책이 관측(상공·측면 카메라 + 관절 6)만으로 행동을 내면
실좌표(gz)로 판정한다. 출력: {"pass", "score", "detail"} JSON.

실행 (환경 순서 중요 — conda 먼저, ROS 나중):
  source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot && \
  source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && \
  python tools/act_eval.py --ckpt logs/act_v1/checkpoints/last/pretrained_model --trials 10
"""
import argparse
import glob
import json
import math
import os
import random
import re
import subprocess
import time

import numpy as np
import torch

TOOLS = os.path.dirname(os.path.abspath(__file__))
POSE_RE = re.compile(r'\[([-\d.eE+ ]+)\]')
AJ = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll']
BASE = (0.3, 0.0)
START = [0.04, -1.74, 1.68, 1.24, -1.31]   # 시연 공통 시작 자세(rest 근방)
FPS = 20.0
TRIAL_SEC = 18.0


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


def spawn_scene(cx, cy, cz, yaw):
    for name in ('demo_cube', 'demo_stand'):
        svc('/world/room/remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')
    svc('/world/room/set_pose', 'gz.msgs.Pose',
        f'name: "jdamr_cube" position {{x: {BASE[0]} y: {BASE[1]} z: 0.03}} orientation {{w: 1}}')
    for _ in range(10):
        left = sh('gz model --list')
        if 'demo_stand' not in left and 'demo_cube' not in left:
            break
        time.sleep(0.5)
    half = 0.0057
    top = cz - half
    stand = ('<sdf version="1.6"><model name="demo_stand"><static>true</static>'
             f'<pose>{cx} {cy} {top / 2} 0 0 0</pose>'
             '<link name="l"><collision name="c"><geometry>'
             f'<box><size>0.6 0.6 {top}</size></box></geometry></collision>'
             '<visual name="v"><geometry>'
             f'<box><size>0.6 0.6 {top}</size></box></geometry>'
             '<material><ambient>0.92 0.92 0.92 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material></visual></link></model></sdf>')
    req = 'sdf: "' + stand.replace('"', '\\"') + '"'
    if 'true' not in svc('/world/room/create', 'gz.msgs.EntityFactory', req):
        time.sleep(1)
        svc('/world/room/create', 'gz.msgs.EntityFactory', req)
    c = ('<sdf version="1.6"><model name="demo_cube">'
         f'<pose>{cx} {cy} {cz} 0 0 {yaw}</pose>'
         '<link name="link"><inertial><mass>0.0025</mass>'
         '<inertia><ixx>3e-7</ixx><ixy>0</ixy><ixz>0</ixz><iyy>3e-7</iyy><iyz>0</iyz><izz>3e-7</izz></inertia></inertial>'
         '<collision name="c"><geometry><box><size>0.0318 0.0158 0.0114</size></box></geometry>'
         '<surface><friction><ode><mu>3.0</mu><mu2>3.0</mu2></ode>'
         '<torsional><coefficient>1.0</coefficient><use_patch_radius>true</use_patch_radius>'
         '<patch_radius>0.008</patch_radius></torsional></friction></surface></collision>'
         '<visual name="v"><geometry><box><size>0.0318 0.0158 0.0114</size></box></geometry>'
         '<material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual></link></model></sdf>')
    svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"')
    time.sleep(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', default=os.path.join(TOOLS, 'logs/act_v1/checkpoints/last/pretrained_model'))
    ap.add_argument('--trials', type=int, default=10)
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.action import ActionClient
    from sensor_msgs.msg import Image, JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from control_msgs.action import GripperCommand
    from lerobot.policies.act.modeling_act import ACTPolicy

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    policy = ACTPolicy.from_pretrained(args.ckpt)
    policy.to(device).eval()
    print(f'정책 로드: {args.ckpt} ({device})')

    rclpy.init()
    node = Node('act_eval')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    obs = {}
    st = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: st.update(dict(zip(m.name, m.position))), 10)
    for cam in ('demo_up', 'demo_side'):
        node.create_subscription(Image, f'/{cam}/image_raw',
                                 (lambda c: lambda m: obs.__setitem__(c, m))(cam),
                                 qos_profile_sensor_data)
    traj = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    grip = ActionClient(node, GripperCommand, '/gripper_controller/gripper_cmd')

    def spin(sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(node, timeout_sec=0.05)

    def move_arm(pos, sec):
        jt = JointTrajectory()
        jt.joint_names = AJ
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in pos]
        pt.time_from_start.sec = int(sec)
        pt.time_from_start.nanosec = int((sec % 1) * 1e9)
        jt.points = [pt]
        traj.publish(jt)

    def gripper_cmd(pos):
        g = GripperCommand.Goal()
        g.command.position = max(float(pos), -0.035)
        g.command.max_effort = 5.0
        grip.send_goal_async(g)
        rclpy.spin_once(node, timeout_sec=0.02)

    spin(2)
    grip.wait_for_server(timeout_sec=15)

    # 스폰 분포: 성공 에피소드들의 실제 스폰에서 샘플 (+지터 5mm)
    spawns = []
    for m in glob.glob(os.path.join(TOOLS, 'logs/collect/ep*/meta.json')):
        d = json.load(open(m))
        if d.get('success'):
            spawns.append(d['spawn'])
    if not spawns:
        raise SystemExit('성공 스폰 분포 없음')
    print(f'스폰 분포: 성공 {len(spawns)}편 기반')

    results = []
    for t in range(args.trials):
        s = random.choice(spawns)
        cx = s[0] + random.uniform(-0.005, 0.005)
        cy = s[1] + random.uniform(-0.005, 0.005)
        cz, yaw = s[2], (s[3] if len(s) > 3 else 0.0)
        spawn_scene(cx, cy, cz, yaw)
        gripper_cmd(0.2)
        # 시작 자세 도달 보장 — 학습 분포의 시작점(rest)에서 출발하지 않으면
        # 첫 관측부터 OOD라 정책이 난동 나선에 빠진다(실측: t0 자세 불일치 → 배회)
        for attempt in range(3):
            move_arm(START, 3.0)
            t0w = time.time()
            ok_pose = False
            while time.time() - t0w < 6:
                rclpy.spin_once(node, timeout_sec=0.05)
                if all(abs(st.get(j, 9.0) - v) < 0.08 for j, v in zip(AJ, START)):
                    ok_pose = True
                    break
            if ok_pose:
                break
            print(f'  시도{attempt + 1} 미도달: 실제={[round(st.get(j, 9.0), 2) for j in AJ]}')
        if not ok_pose:
            print(f'[{t + 1}] 시작 자세 미도달 — 건너뜀')
            results.append({'trial': t, 'error': 'start_pose', 'success': False,
                            'spawn': [round(cx, 3), round(cy, 3), round(cz, 3)],
                            'lift_mm': 0.0, 'moved_mm': 0.0, 'final': []})
            continue
        spin(1)
        policy.reset()

        # 백그라운드 레고 z 추적
        zlog = os.path.join(TOOLS, 'logs', 'eval_z.log')
        open(zlog, 'w').close()
        zp = subprocess.Popen(['bash', '-c',
            f'for i in $(seq 1 60); do gz model -m demo_cube -p 2>/dev/null | '
            f'grep -A2 Pose | sed -n 2p >> {zlog}; sleep 0.3; done'])

        sim0 = node.get_clock().now().nanoseconds / 1e9
        last_tick = -1
        deadline = time.time() + TRIAL_SEC * 2 + 10
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            el = node.get_clock().now().nanoseconds / 1e9 - sim0
            if el > TRIAL_SEC:
                break
            tick = int(el * FPS)
            if tick <= last_tick or 'demo_up' not in obs or 'demo_side' not in obs:
                continue
            last_tick = tick
            imgs = {}
            for cam in ('demo_up', 'demo_side'):
                m = obs[cam]
                # cv_bridge는 conda numpy2와 ABI 충돌(코어 덤프 실측) — 수동 디코드.
                # gz image_bridge는 rgb8을 발행하므로 정책 입력(RGB)과 그대로 정합.
                im = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                if m.encoding == 'bgr8':
                    im = im[:, :, ::-1]
                imgs[cam] = torch.from_numpy(im.copy()).permute(2, 0, 1).float().div(255).unsqueeze(0).to(device)
            state = torch.tensor([[st.get(j, 0.0) for j in AJ] + [st.get('arm_gripper', 0.0)]],
                                 dtype=torch.float32).to(device)
            batch = {'observation.images.up': imgs['demo_up'],
                     'observation.images.side': imgs['demo_side'],
                     'observation.state': state}
            with torch.no_grad():
                action = policy.select_action(batch).squeeze(0).cpu().numpy()
            if tick % 40 == 0:
                cur = [round(st.get(j, 0.0), 2) for j in AJ]
                print(f'  t{tick}: state={cur} → action={[round(float(v), 2) for v in action]}')
            move_arm(action[:5], 0.1)
            gripper_cmd(action[5])
        zp.terminate()

        max_z = cz
        for ln in open(zlog):
            v = ln.strip().strip('[]').split()
            if len(v) >= 3:
                max_z = max(max_z, float(v[2]))
        fin = gz_xyz('demo_cube') or [cx, cy, cz]
        moved = math.hypot(fin[0] - cx, fin[1] - cy)
        lifted = max_z - cz > 0.025
        on_floor = fin[2] < cz - 0.05
        succ = bool(lifted and moved > 0.04 and not on_floor)
        results.append({'trial': t, 'spawn': [round(cx, 3), round(cy, 3), round(cz, 3)],
                        'lift_mm': round((max_z - cz) * 1000, 1), 'moved_mm': round(moved * 1000, 1),
                        'final': [round(v, 3) for v in fin], 'success': succ})
        print(f"[{t + 1}/{args.trials}] lift={results[-1]['lift_mm']}mm moved={results[-1]['moved_mm']}mm "
              f"{'성공' if succ else '실패'}")

    score = sum(r['success'] for r in results) / len(results)
    out = {'pass': score >= 0.5, 'score': score, 'trials': len(results), 'detail': results,
           'ckpt': args.ckpt}
    path = args.out or os.path.join(TOOLS, 'logs', 'act_eval_result.json')
    json.dump(out, open(path, 'w'), ensure_ascii=False, indent=1)
    print(f"성공률 {score:.0%} ({sum(r['success'] for r in results)}/{len(results)}) → {path}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
