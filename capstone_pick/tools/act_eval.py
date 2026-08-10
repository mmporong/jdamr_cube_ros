"""ACT 정책 폐루프 평가기 — autoresearch 평가 계약 구현.

학습된 정책을 시뮬에 붙여 N회 실행: 성공 에피소드들의 스폰 분포에서 무대를
샘플(±5mm 지터)하고, 정책이 관측(상공·측면 카메라 + 관절 6)만으로 행동을 내면
실좌표(gz)로 판정한다. 출력: {"pass", "score", "detail"} JSON.

실행 (환경 순서 중요 — conda 먼저, ROS 나중):
  source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot && \
  source /opt/ros/jazzy/setup.bash && source ~/jdamr_cube_ws/install/setup.bash && \
  python tools/act_eval.py --ckpt logs/act_rule_v2/checkpoints/last/pretrained_model --trials 10
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
START = [0.0, -0.4, 1.0, 0.2, 0.0]         # 규칙 기반 픽의 시작 자세(POSE_FOLDED)
CAMS = {'front': '/rgbd_camera/image', 'wrist': '/wrist_camera/image_raw'}
FPS = 20.0
# 시연 길이(31.6~48.4초, 중앙 37.2초)에 맞춘다. 60초로 두면 태스크를 마친 뒤
# 23초가 남아 정책이 파지를 다시 시도한다 — 학습 데이터에 '완료 후' 상황이
# 없으므로 큐브가 사라진 자리를 집으려 든다. 정책 결함이 아니라 평가 설정 문제다.
TRIAL_SEC = 45.0


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
    """규칙 기반 수집과 동일한 무대: 로봇 원점, 바닥에 3cm 큐브."""
    name = 'pick_blue'
    for n in ('pick_object', 'pick_object_green', 'pick_object_blue', 'pick_object_orange',
              'pick_table', 'pick_blue', 'pick_red', 'pick_green', 'demo_cube', 'demo_stand'):
        svc('/world/room/remove', 'gz.msgs.Entity', f'name: "{n}" type: MODEL')
    svc('/world/room/set_pose', 'gz.msgs.Pose',
        f'name: "jdamr_cube" position {{x: {BASE[0]} y: {BASE[1]} z: 0.03}} orientation {{w: 1}}')
    for _ in range(10):
        if name not in sh('gz model --list'):
            break
        time.sleep(0.5)
    c = (f'<sdf version="1.6"><model name="{name}">'
         f'<pose>{cx} {cy} 0.015 0 0 {yaw}</pose>'
         '<link name="link"><inertial><mass>0.04</mass>'
         '<inertia><ixx>4e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>4e-6</iyy><iyz>0</iyz><izz>4e-6</izz></inertia></inertial>'
         '<collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
         '<surface><friction><ode><mu>3.0</mu><mu2>3.0</mu2></ode>'
         '<torsional><coefficient>1.0</coefficient><use_patch_radius>true</use_patch_radius>'
         '<patch_radius>0.01</patch_radius></torsional></friction></surface></collision>'
         '<visual name="v"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>'
         '<material><ambient>0.1 0.2 0.9 1</ambient><diffuse>0.1 0.2 0.9 1</diffuse></material>'
         '</visual></link></model></sdf>')
    r = svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"')
    if 'true' not in r:
        time.sleep(1)
        svc('/world/room/create', 'gz.msgs.EntityFactory', 'sdf: "' + c.replace('"', '\\"') + '"')
    time.sleep(1.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', default=os.path.join(TOOLS, 'logs/act_rule_v2/checkpoints/last/pretrained_model'))
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
    from lerobot.policies.factory import make_pre_post_processors

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    policy = ACTPolicy.from_pretrained(args.ckpt)
    # 공식 평가 경로는 preprocessor → select_action → postprocessor 다.
    # (lerobot_eval.py:288-300) state/action 정규화는 MEAN_STD이고 그 통계는
    # 체크포인트에 저장돼 있다. 이 두 단계를 건너뛰면 정규화된 숫자가 그대로
    # 관절 명령이 되어, 학습이 아무리 잘 돼도 로봇은 엉뚱하게 움직인다.
    # 실측: 같은 정책·같은 데이터에서 재현 오차 0.5566 rad → 0.0262 rad (21배).
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy.config, pretrained_path=args.ckpt)
    # 기본은 학습 설정 그대로(ACT 표준). 실험할 때만 환경변수로 덮어쓴다.
    if os.environ.get('ACT_N_ACTION_STEPS'):
        policy.config.n_action_steps = int(os.environ['ACT_N_ACTION_STEPS'])
    print(f'실행 길이 n_action_steps = {policy.config.n_action_steps} '
          f'(chunk {policy.config.chunk_size})')
    policy.to(device).eval()
    print(f'정책 로드: {args.ckpt} ({device}) + processor 적용')

    rclpy.init()
    node = Node('act_eval')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    obs = {}
    st = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: st.update(dict(zip(m.name, m.position))), 10)
    for cam, topic in CAMS.items():
        node.create_subscription(Image, topic,
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

    last_grip = {'pos': None, 't': 0.0}

    def gripper_cmd(pos):
        """그리퍼 목표는 실제로 바뀔 때만 보낸다.

        20Hz로 매 틱 goal을 던지면 액션 서버에서 목표가 겹치거나 서로 취소되어,
        관측된 개폐 동작을 정책 탓으로 돌릴 수 없다(2차 검수 R2-U05-F05).
        같은 값이면 0.5초에 한 번만 재전송해 유지 의도만 갱신한다.
        """
        p = max(float(pos), -0.035)
        now = time.time()
        if (last_grip['pos'] is not None
                and abs(p - last_grip['pos']) < 0.01 and now - last_grip['t'] < 0.5):
            return
        last_grip['pos'], last_grip['t'] = p, now
        g = GripperCommand.Goal()
        g.command.position = p
        g.command.max_effort = 5.0
        grip.send_goal_async(g)
        rclpy.spin_once(node, timeout_sec=0.02)

    spin(2)
    grip.wait_for_server(timeout_sec=15)

    # 스폰 분포: 학습에 쓴 그 수집물에서 샘플 (+지터 5mm).
    # 소스가 학습 데이터와 어긋나면 분포 밖에서 시험하면서 정책 탓을 하게 된다
    # (2차 검수 R2-U05-F02: 10회 중 7회가 학습 스폰 박스 밖이었다).
    src = os.environ.get('ACT_SPAWN_SRC', 'rule_std')
    spawns = []
    for m in glob.glob(os.path.join(TOOLS, 'logs', src, 'ep*/meta.json')):
        d = json.load(open(m))
        if d.get('success'):
            spawns.append(d['spawn'])
    if not spawns:
        raise SystemExit(f'성공 스폰 분포 없음 (logs/{src})')
    xs = [s[0] for s in spawns]; ys = [s[1] for s in spawns]
    print(f'스폰 분포: logs/{src} 성공 {len(spawns)}편 '
          f'(x {min(xs):.4f}~{max(xs):.4f}, y {min(ys):.4f}~{max(ys):.4f})')

    results = []
    for t in range(args.trials):
        s = random.choice(spawns)
        cx = s[0] + random.uniform(-0.005, 0.005)
        cy = s[1] + random.uniform(-0.005, 0.005)
        cz, yaw = 0.015, random.uniform(-0.3, 0.3)
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
        # 시행 전체를 덮도록 샘플 수를 늘린다 — 60샘플×0.3s = wall 18초라
        # 60초 시행의 후반 들림을 통째로 놓쳤다(리뷰 U01-F06, 확인됨)
        zp = subprocess.Popen(['bash', '-c',
            f'for i in $(seq 1 400); do gz model -m pick_blue -p 2>/dev/null | '
            f'grep -A2 Pose | sed -n 2p >> {zlog}; sleep 0.3; done'])

        sim0 = node.get_clock().now().nanoseconds / 1e9
        last_tick = -1
        traj_log = []   # [el, state6, action6] — 실패 진단용 (traj는 퍼블리셔라 이름 분리)
        deadline = time.time() + TRIAL_SEC * 2 + 10
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            el = node.get_clock().now().nanoseconds / 1e9 - sim0
            if el > TRIAL_SEC:
                break
            tick = int(el * FPS)
            if tick <= last_tick or len(obs) < len(CAMS):
                continue
            last_tick = tick
            imgs = {}
            for cam in CAMS:
                m = obs[cam]
                # cv_bridge는 conda numpy2와 ABI 충돌(코어 덤프 실측) — 수동 디코드.
                # gz image_bridge는 rgb8을 발행하므로 정책 입력(RGB)과 그대로 정합.
                im = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                if m.encoding == 'bgr8':
                    im = im[:, :, ::-1]
                imgs[cam] = torch.from_numpy(im.copy()).permute(2, 0, 1).float().div(255).unsqueeze(0).to(device)
            # 관절이 하나라도 안 왔으면 이 틱은 버린다. 예전처럼 0.0으로 채우면
            # 정책이 존재하지 않는 자세를 보고 판단한다(2차 검수 R2-U05-F04).
            need = AJ + ['arm_gripper']
            if not all(j in st for j in need):
                continue
            state = torch.tensor([[st[j] for j in need]], dtype=torch.float32).to(device)
            batch = {'observation.images.front': imgs['front'],
                     'observation.images.wrist': imgs['wrist'],
                     'observation.state': state}
            with torch.inference_mode():
                action = postprocessor(policy.select_action(preprocessor(batch)))
            action = action.squeeze(0).cpu().numpy()
            if not np.all(np.isfinite(action)):
                print(f'  t{tick}: 비정상 예측 {action} — 이 틱 건너뜀')
                continue
            if tick % 40 == 0:
                cur = [round(st.get(j, 0.0), 2) for j in AJ]
                print(f'  t{tick}: state={cur} → action={[round(float(v), 2) for v in action]}')
            traj_log.append(np.concatenate([[el], state.squeeze(0).cpu().numpy(), action]))
            move_arm(action[:5], 0.05)   # 명령 주기(20Hz)와 궤적 horizon 정합 (리뷰 U01-F04)
            gripper_cmd(action[5])
        zp.terminate()

        max_z = cz
        for ln in open(zlog):
            v = ln.strip().strip('[]').split()
            if len(v) >= 3:
                max_z = max(max_z, float(v[2]))
        fin = gz_xyz('pick_blue') or [cx, cy, cz]
        moved = math.hypot(fin[0] - cx, fin[1] - cy)
        # 판정은 시연(rule_collect.py:287 `moved > 0.05`)과 같은 기준을 쓰되,
        # 밀어내기를 배제하기 위해 '들림'을 함께 요구한다.
        # 시연 태스크는 집어서 옮긴 뒤 '놓는' 것으로 끝난다(그리퍼 명령이 0.80으로
        # 종료). 따라서 최종 파지(held)를 성공 조건에 넣으면 시연을 정확히
        # 재현한 시행까지 실패로 셌다 — 실측으로 확인하고 되돌린 기준이다.
        lifted = max_z - cz > 0.025      # 실제로 들어올렸나 (밀기 배제)
        held = fin[2] - cz > 0.02        # 끝날 때도 들고 있나 (진단용)
        succ = bool(lifted and moved > 0.05)
        if traj_log:
            np.save(os.path.join(TOOLS, 'logs', f'eval_traj_{t}.npy'), np.array(traj_log))
        results.append({'trial': t, 'spawn': [round(cx, 3), round(cy, 3), round(cz, 3)],
                        'lift_mm': round((max_z - cz) * 1000, 1), 'moved_mm': round(moved * 1000, 1),
                        'final': [round(v, 3) for v in fin], 'lifted': lifted, 'held': held,
                        'success': succ})
        tag = '성공' if succ else ('이동부족' if lifted else
                                 ('밀기만' if moved > 0.01 else '실패'))
        print(f"[{t + 1}/{args.trials}] lift={results[-1]['lift_mm']}mm "
              f"moved={results[-1]['moved_mm']}mm 최종z={fin[2]:.3f} {tag}")

    score = sum(r['success'] for r in results) / len(results)
    out = {'pass': score >= 0.5, 'score': score, 'trials': len(results), 'detail': results,
           'ckpt': args.ckpt}
    path = args.out or os.path.join(TOOLS, 'logs', 'act_eval_result.json')
    json.dump(out, open(path, 'w'), ensure_ascii=False, indent=1)
    print(f"성공률 {score:.0%} ({sum(r['success'] for r in results)}/{len(results)}) → {path}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
