"""LeRobot 시연 궤적 재생 — 고정 베이스, 자가 캘리브레이션 2패스.

패스1(기본): 재생하며 그리퍼 닫힘 순간의 핀치(손끝) 좌표를 실측.
패스2(--cube X Y Z): 그 좌표에 받침대+레고(시연 제원 2x4, 2.5g)를 놓고 재생 → 실좌표 판정.
--auto: 패스1 측정 후 자동으로 패스2 재실행.

핀치 좌표는 실링크(arm_gripper_link)의 gz 자세 + RPY 회전으로 계산한다.
가상 링크(arm_gripper_frame_link)는 물리에서 병합돼 로봇 베이스 좌표라는
허수를 반환했다(실측) — 절대 쓰지 말 것. gz의 `-l` 링크 자세는 모델 상대일
수 있어 모델 자세와 합성한 후보를 함께 출력해 검증한다.

사용 (ROS 환경): python3 lerobot_replay.py [--ep 0] [--cube X Y Z] [--auto]
"""
import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2

AJ = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex', 'arm_wrist_roll']
POSE_RE = re.compile(r'\[([-\d.eE+ ]+)\]')
FPS = 30.0
FINGER_TIP = np.array([-0.0157, -0.0002, -0.0895])   # 고정 죠 손끝 패드 (gripper 링크 좌표계)
BASE = (0.3, 0.0)                                     # 고정 베이스 월드 좌표


def sh(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout


def svc(s_, t_, r_):
    return subprocess.run(['gz', 'service', '-s', s_, '--reqtype', t_,
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '8000', '--req', r_],
                          capture_output=True, text=True).stdout.strip()


def gz_pose(model, link=None):
    """(xyz, rpy) — 마지막 두 대괄호 벡터."""
    cmd = f'gz model -m {model}' + (f' -l {link}' if link else '') + ' -p'
    for _ in range(3):
        g = POSE_RE.findall(sh(cmd))
        if len(g) >= 2:
            return (np.array([float(v) for v in g[-2].split()]),
                    np.array([float(v) for v in g[-1].split()]))
        time.sleep(0.2)
    return None, None


def rot(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def pinch_candidates():
    """핀치 좌표 후보 2개: (링크자세=월드 가정, 모델합성 가정)."""
    lx, lr = gz_pose('jdamr_cube', 'arm_gripper_link')
    mx, mr = gz_pose('jdamr_cube')
    if lx is None or mx is None:
        return None, None, None
    tip_local = rot(lr) @ FINGER_TIP
    cand_world = lx + tip_local
    cand_comp = mx + rot(mr) @ (lx + tip_local)
    return cand_world, cand_comp, (lx, lr)


class Rep(Node):
    def __init__(self):
        super().__init__('lerobot_replay')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.st = {}
        self.imgs = {}
        self.bridge = CvBridge()
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.st.update(dict(zip(m.name, m.position))), 10)
        from rclpy.qos import qos_profile_sensor_data
        for cam in ('demo_up', 'demo_side'):
            # 원시 메시지만 저장 — 매 수신 디코드는 수집 루프를 굶긴다(95/303 실측)
            self.create_subscription(
                Image, f'/{cam}/image_raw',
                (lambda c: lambda m: self.imgs.__setitem__(c, m))(cam),
                qos_profile_sensor_data)
        self.traj = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

    def wall_spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def move_pt(self, pos, sec):
        jt = JointTrajectory()
        jt.joint_names = AJ
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in pos]
        pt.time_from_start.sec = int(sec)
        pt.time_from_start.nanosec = int((sec % 1) * 1e9)
        jt.points = [pt]
        self.traj.publish(jt)

    def send_full(self, R, slow=1.0):
        jt = JointTrajectory()
        jt.joint_names = AJ
        for i in range(len(R)):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in R[i, :5]]
            t = i / FPS * slow + 0.2
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1) * 1e9)
            jt.points.append(pt)
        self.traj.publish(jt)

    def gripper_cmd(self, pos):
        g = GripperCommand.Goal()
        # 닫힘 하한 -0.02: 시연 명령(-0.14)은 레고 폭(15.8mm, 정상 스톨 +0.012)을
        # 한참 지나친 목표라 위치 제어가 손가락을 관통시킨다(실측: 레고가 그리퍼에
        # 꽂힌 채 운반 — 가짜 성공). effort 3: 2.5g 물체에 과한 힘도 관통을 키운다.
        g.command.position = max(float(pos), -0.035)
        g.command.max_effort = 5.0
        self.grip.send_goal_async(g)
        rclpy.spin_once(self, timeout_sec=0.02)


def spawn_scene(cube):
    for name in ('pick_object', 'pick_object_green', 'pick_object_blue', 'pick_object_orange',
                 'pick_table', 'pick_blue', 'pick_red', 'pick_green', 'demo_cube', 'demo_stand'):
        svc('/world/room/remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')
    svc('/world/room/set_pose', 'gz.msgs.Pose',
        f'name: "jdamr_cube" position {{x: {BASE[0]} y: {BASE[1]} z: 0.03}} orientation {{w: 1}}')
    # 제거는 비동기 — 같은 이름 재생성 전에 완료를 폴링으로 확인 (아니면 생성 타임아웃)
    for _ in range(10):
        left = sh('gz model --list')
        if 'demo_stand' not in left and 'demo_cube' not in left:
            break
        time.sleep(0.5)
    if cube is None:
        return
    cx, cy, cz = cube[:3]
    half = 0.0057
    if cz > 0.012:
        top = cz - half
        stand = ('<sdf version="1.6"><model name="demo_stand"><static>true</static>'
                 f'<pose>{cx} {cy} {top / 2} 0 0 0</pose>'
                 '<link name="l"><collision name="c"><geometry>'
                 f'<box><size>0.6 0.6 {top}</size></box></geometry></collision>'
                 '<visual name="v"><geometry>'
                 f'<box><size>0.6 0.6 {top}</size></box></geometry>'
                 '<material><ambient>0.92 0.92 0.92 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material></visual></link></model></sdf>')
        req = 'sdf: "' + stand.replace('"', '\\"') + '"'
        r = svc('/world/room/create', 'gz.msgs.EntityFactory', req)
        if 'true' not in r:
            time.sleep(1)
            r = svc('/world/room/create', 'gz.msgs.EntityFactory', req)
        print('table:', r)
    yaw = cube[3] if len(cube) > 3 else 0.0
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
    print('cube:', svc('/world/room/create', 'gz.msgs.EntityFactory',
                       'sdf: "' + c.replace('"', '\\"') + '"'))
    time.sleep(1)


def main():
    ep = int(sys.argv[sys.argv.index('--ep') + 1]) if '--ep' in sys.argv else 0
    slow = float(sys.argv[sys.argv.index('--slow') + 1]) if '--slow' in sys.argv else 1.0
    cube = None
    if '--cube' in sys.argv:
        i = sys.argv.index('--cube')
        cube = [float(v) for v in sys.argv[i + 1:i + 5] if not v.startswith('--')]
    npy = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs', f'ep{ep}_rad.npy')
    R = np.load(npy)

    rclpy.init()
    n = Rep()
    n.wall_spin(2)
    n.grip.wait_for_server(timeout_sec=15)
    spawn_scene(cube)

    # 닫힘·재개방 프레임 선계산
    fc = fo = None
    for i in range(len(R)):
        if fc is None and R[i, 5] < 0.0 and R[:i + 1, 5].max() > 0.1:
            fc = i
        if fc is not None and fo is None and i > fc + 15 and R[i, 5] > 0.1:
            fo = i
    fc_meas = fc + 10
    print(f'닫힘 프레임 f={fc} (t={fc / FPS:.2f}s), 재개방 f={fo}')

    if cube is None:
        # 패스1 = 정적 FK 프로브: 재생 없이 닫힘 자세로 세워두고 정지 측정.
        # 재생 중 측정은 gz 서브프로세스 지연(샘플당 ~0.8s)에 팔이 움직여
        # 패스 간 17cm까지 표류했다(실측) — 정적 측정이 유일하게 결정적이다.
        n.gripper_cmd(0.2)
        n.move_pt(R[fc_meas, :5], 4.0)
        n.wall_spin(6)
        Bs, dirs = [], []
        for _ in range(3):
            w, comp, raw = pinch_candidates()
            if comp is not None:
                Bs.append(comp)
                cd = rot(raw[1]) @ np.array([1.0, 0.0, 0.0])
                dirs.append(math.atan2(cd[1], cd[0]))
        comp = np.median(np.stack(Bs), axis=0)
        close_dir = float(np.median(dirs))
        lego_yaw = math.atan2(math.sin(close_dir - math.pi / 2), math.cos(close_dir - math.pi / 2))
        print(f'정적 핀치 측정: B=({comp[0]:+.3f},{comp[1]:+.3f},{comp[2]:+.3f}) '
              f'닫힘방향 {math.degrees(close_dir):+.0f}도 → 레고 yaw {math.degrees(lego_yaw):+.0f}도')
        if '--snap' in sys.argv:
            # 시점 비교용: 측정 좌표에 무대를 깔고 두 카메라 프레임 저장
            spawn_scene([comp[0], comp[1], comp[2], lego_yaw])
            n.move_pt(R[fc_meas, :5], 3.0)
            n.wall_spin(5)
            n.imgs.clear()
            n.wall_spin(2)
            out = os.path.dirname(npy)
            for cam in ('demo_up', 'demo_side'):
                if cam in n.imgs:
                    cv2.imwrite(os.path.join(out, f'snap_{cam}.png'),
                                n.bridge.imgmsg_to_cv2(n.imgs[cam], 'bgr8'))
                    print(f'저장: {out}/snap_{cam}.png')
                else:
                    print(f'{cam} 프레임 미수신')
            rclpy.shutdown()
            return
        if '--auto' in sys.argv:
            print('== 자동 패스2 ==')
            extra = [a for a in ('--record',) if a in sys.argv]
            if '--slow' in sys.argv:
                extra += ['--slow', sys.argv[sys.argv.index('--slow') + 1]]
            # execv 대신 서브프로세스 — 이전 sed가 execv를 --slow 분기 안에 가둬
            # slow 없는 배치에서 패스2가 통째로 증발했다(에피소드당 20초 무메타 실측)
            rclpy.shutdown()
            rc = subprocess.run([sys.executable, '-u', __file__, '--ep', str(ep), '--cube',
                                 f'{comp[0]:.3f}', f'{comp[1]:.3f}', f'{comp[2]:.3f}',
                                 f'{lego_yaw:.3f}'] + extra).returncode
            sys.exit(rc)
        rclpy.shutdown()
        return

    # 패스2 = 재생
    n.gripper_cmd(float(R[0, 5]))
    n.move_pt(R[0, :5], 3.0)
    n.wall_spin(4)
    print(f'재생 시작: ep{ep} {len(R)}프레임 ({len(R) / FPS:.1f}s)')
    # 레고 z 추적은 백그라운드 셸로 — 루프 안 서브프로세스는 그리퍼 타임라인을
    # 굶겨 닫힘이 밀린다(실측: 스톨각을 열림값에서 오샘플)
    zlog = os.path.join(os.path.dirname(npy), 'lego_z.log')
    open(zlog, 'w').close()
    zproc = subprocess.Popen(['bash', '-c',
        'for i in $(seq 1 80); do gz model -m demo_cube -p 2>/dev/null | '
        f'grep -A2 Pose | sed -n 2p >> {zlog}; sleep 0.3; done'])
    n.send_full(R, slow)
    sim0 = n.sim_now()
    grip_state = R[0, 5]
    stall = None
    stall_at = None
    rec = None
    if '--record' in sys.argv:
        rec = {'dir': os.path.join(os.path.dirname(npy), 'collect', f'ep{ep}'),
               'demo_up': [], 'demo_side': [], 'state': [], 'action': [], 'fi': [], 'last_fi': -1}
    deadline = time.time() + 60
    while time.time() < deadline:
        rclpy.spin_once(n, timeout_sec=0.01)
        el = (n.sim_now() - sim0 - 0.2) / slow
        if el < 0:
            continue
        if el > len(R) / FPS + 1.0 / slow:
            break
        fi = min(len(R) - 1, int(el * FPS))
        if rec is not None and fi > rec['last_fi'] and 'demo_up' in n.imgs and 'demo_side' in n.imgs:
            rec['last_fi'] = fi
            rec['fi'].append(fi)
            rec['demo_up'].append(n.bridge.imgmsg_to_cv2(n.imgs['demo_up'], 'bgr8'))
            rec['demo_side'].append(n.bridge.imgmsg_to_cv2(n.imgs['demo_side'], 'bgr8'))
            rec['state'].append([n.st.get(j, 0.0) for j in AJ] + [n.st.get('arm_gripper', 0.0)])
            rec['action'].append(R[fi].tolist())
        if abs(R[fi, 5] - grip_state) > 0.015:
            grip_state = R[fi, 5]
            n.gripper_cmd(grip_state)
            if grip_state < 0.0 and stall_at is None:
                stall_at = time.time() + 1.2   # 닫힘 명령 후 스톨 안착을 기다려 읽는다
        if stall_at is not None and time.time() > stall_at:
            a = n.st.get('arm_gripper')
            if a is not None:
                stall = a if stall is None else min(stall, a)
    zproc.terminate()
    if rec is not None:
        os.makedirs(rec['dir'], exist_ok=True)
        for cam in ('demo_up', 'demo_side'):
            os.makedirs(os.path.join(rec['dir'], cam), exist_ok=True)
            for i, im in enumerate(rec[cam]):
                cv2.imwrite(os.path.join(rec['dir'], cam, f'{i:06d}.jpg'), im)
        np.save(os.path.join(rec['dir'], 'fi.npy'), np.array(rec['fi']))
        np.save(os.path.join(rec['dir'], 'state.npy'), np.array(rec['state']))
        np.save(os.path.join(rec['dir'], 'action.npy'), np.array(rec['action']))
        print(f"수집: {len(rec['action'])}프레임 → {rec['dir']} (up/side PNG + state/action npy)")
    max_z = cube[2]
    try:
        for ln in open(zlog):
            v = ln.strip().strip('[]').split()
            if len(v) >= 3:
                max_z = max(max_z, float(v[2]))
    except OSError:
        pass

    if cube is not None:
        time.sleep(1)
        x, r = gz_pose('demo_cube')
        if x is not None:
            d = math.hypot(x[0] - cube[0], x[1] - cube[1])
            lifted = max_z - cube[2] > 0.025
            ok = lifted and d > 0.05 and x[2] > cube[2] - 0.05
            print(f'레고 최종 ({x[0]:.3f},{x[1]:.3f},{x[2]:.3f}) | 스폰 ({cube[0]:.3f},{cube[1]:.3f},{cube[2]:.3f}) '
                  f'| 들림 최대 {(max_z - cube[2]) * 1000:+.0f}mm | 수평 변위 {d * 1000:.0f}mm '
                  f'| {"파지·운반 성공" if ok else ("들었으나 낙하" if lifted else "파지 실패")}')
            if rec is not None:
                import json
                pinch_ok = stall is not None and -0.06 < stall < 0.08
                meta = {'episode': ep, 'frames': len(rec['action']), 'total_frames': len(R),
                        'success': bool(ok and pinch_ok), 'lifted': bool(lifted),
                        'pinch_ok': bool(pinch_ok),
                        'lift_mm': round((max_z - cube[2]) * 1000, 1),
                        'moved_mm': round(d * 1000, 1),
                        'stall_angle': None if stall is None else round(float(stall), 3),
                        'spawn': [round(v, 4) for v in cube], 'final': [round(float(v), 4) for v in x]}
                json.dump(meta, open(os.path.join(rec['dir'], 'meta.json'), 'w'), ensure_ascii=False, indent=1)
    elif close_meas is not None and '--auto' in sys.argv:
        comp = close_meas[3]
        yw = close_meas[4]
        print('== 자동 패스2: 측정 좌표·방향에 레고 스폰 ==')
        rclpy.shutdown()
        rc = subprocess.run([sys.executable, '-u', __file__, '--ep', str(ep), '--cube',
                             f'{comp[0]:.3f}', f'{comp[1]:.3f}', f'{comp[2]:.3f}', f'{yw:.3f}']).returncode
        sys.exit(rc)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
