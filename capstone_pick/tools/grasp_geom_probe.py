"""하강 직후 큐브가 죠의 어디에 있나 — 상대 위치를 숫자와 사진으로 남긴다.

GRASP_BACK을 25~70mm로 45mm나 쓸었는데 물림각이 0.238~0.256으로 거의 안 변했다.
정상이라면 큐브와 죠의 상대 위치가 그만큼 달라져야 한다. 모델과 실제가 어긋난다는
뜻이므로, 여기서는 계산을 믿지 않고 **닫기 전 상태를 그대로 관측**한다.

  · 큐브 실좌표(gz)와 죠 세 프레임(TF)의 상대 위치 — 죠 로컬 좌표로 환산
  · 손목캠 사진 — 눈으로 확인 (사용자가 "낀 거다"라고 지적한 상태)

죠 로컬 좌표로 바꾸는 이유: base 좌표로는 팔 자세가 섞여 "죠 안쪽인지 끝인지"를
읽을 수 없다. 아랫턱 프레임 기준으로 보면 z가 손가락을 따라가는 축이 된다.

사용 (ROS 환경 source 후): python3 grasp_geom_probe.py [back_mm ...]
"""
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from capstone_pick import kinematics as K  # noqa: E402
from capstone_pick.pick_node import (ARM_JOINTS, GRIPPER_CLOSED,  # noqa: E402
                                     PickNode, POSE_FOLDED)

OUT = os.path.expanduser('~/capstone_tools/logs/grasp_geom')
CUBE_X = 0.405


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '4000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def cube_pose():
    for _ in range(3):
        try:
            out = subprocess.run(['gz', 'model', '-m', 'pick_object_green', '-p'],
                                 capture_output=True, text=True, timeout=25).stdout
        except subprocess.TimeoutExpired:
            continue
        L = out.splitlines()
        for i, ln in enumerate(L):
            if '- Pose' in ln and i + 1 < len(L):
                try:
                    v = [float(t) for t in L[i + 1].strip().strip('[]').split()]
                    if len(v) >= 3:
                        return np.array(v[:3])
                except ValueError:
                    continue
    return None


def spin(n, sec):
    t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.05)


def tf_mat(n, frame):
    """base_footprint → frame 의 4x4."""
    for _ in range(20):
        try:
            t = n.tf_buffer.lookup_transform('base_footprint', frame, rclpy.time.Time())
            q = t.transform.rotation
            v = t.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ])
            M = np.eye(4)
            M[:3, :3] = R
            M[:3, 3] = [v.x, v.y, v.z]
            return M
        except Exception:
            spin(n, 0.2)
    return None


def snap(n, path):
    if getattr(n, 'wrist_img', None) is None:
        n.spin_until(lambda: getattr(n, 'wrist_img', None) is not None, 3.0)
    if getattr(n, 'wrist_img', None) is not None:
        try:
            cv2.imwrite(path, n.wrist_img)
            return True
        except Exception:
            pass
    return False


def run(n, back):
    K.GRASP_BACK = back
    path = K.descend_path(CUBE_X, 0.0)
    if path is None:
        print(f'back={back * 1000:.0f}mm: IK 해 없음')
        return
    n.move_gripper(1.2)
    n.move_arm(POSE_FOLDED, 2.5)
    svc('name: "jdamr_cube", position: {x: 0, y: 0, z: 0.05}, orientation: {w: 1}')
    svc(f'name: "pick_object_green", position: {{x: {CUBE_X}, y: 0.0, z: 0.02}}, '
        f'orientation: {{w: 1}}')
    spin(n, 1.2)
    n.move_gripper(1.2)
    n.move_arm(dict(zip(ARM_JOINTS, path[0])), 3.0)
    for q in path[1:]:
        n.move_arm(dict(zip(ARM_JOINTS, q)), 1.2)
    spin(n, 0.8)

    cube = cube_pose()
    Mj = tf_mat(n, 'arm_gripper_link')          # 아랫턱
    if cube is None or Mj is None:
        print(f'back={back * 1000:.0f}mm: 측정 실패')
        return
    # 큐브를 아랫턱 로컬 좌표로 — z가 손가락을 따라가는 축이다
    inv = np.linalg.inv(Mj)
    loc = (inv @ np.append(cube, 1.0))[:3]
    tcp = K.fk_pos(path[-1])
    snap(n, os.path.join(OUT, f'back{back * 1000:.0f}_open.png'))

    n.move_gripper(GRIPPER_CLOSED)
    spin(n, 2.0)
    ang = getattr(n, 'gripper_angle', None)
    snap(n, os.path.join(OUT, f'back{back * 1000:.0f}_closed.png'))
    cube2 = cube_pose()
    moved = None if cube2 is None else float(np.linalg.norm(cube2 - cube))
    fa = '     -' if ang is None else f'{ang:6.3f}'
    fm = '    -' if moved is None else f'{moved * 1000:5.1f}'
    print(f'{back * 1000:6.0f} | {loc[0]:+7.4f} {loc[1]:+7.4f} {loc[2]:+7.4f} | '
          f'{tcp[0]:+6.3f} {tcp[2]:+6.3f} | {fa} {fm}')


def main():
    rclpy.init(args=['--ros-args', '-p', 'detector:=hsv', '-p', 'target_color:=green',
                     '-p', 'place_target:=trash', '-p', 'speed_scale:=1.0'])
    n = PickNode()
    n.spin_until(lambda: getattr(n, 'joint_pos', None), 20.0)
    n.floor_mode = True
    os.makedirs(OUT, exist_ok=True)
    backs = [float(v) / 1000 for v in sys.argv[1:]] or [0.030, 0.045, 0.060, 0.070]
    print(f'큐브 x={CUBE_X} 고정. 하강 직후(닫기 전) 큐브가 아랫턱 로컬로 어디 있나.')
    print('아랫턱 로컬 z는 손가락을 따라가는 축 — 값이 클수록 손가락 끝 쪽이다.\n')
    print(f'{"back":>6} | {"큐브 x":>7} {"큐브 y":>7} {"큐브 z":>7} | '
          f'{"TCPx":>6} {"TCPz":>6} | {"물림각":>6} {"이동mm":>5}')
    print('-' * 72)
    for b in backs:
        run(n, b)
    print(f'\n사진: {OUT}')
    svc('name: "pick_object_green", position: {x: 0.45, y: 0.26, z: 0.02}, orientation: {w: 1}')
    n.move_arm(POSE_FOLDED, 3.0)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
