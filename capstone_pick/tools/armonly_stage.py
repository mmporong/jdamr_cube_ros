"""팔만으로 줍고 놓는 무대를 만든다 — 주행 없음.

전체 파이프라인은 주행 접근과 통 인식에서 막혀 최종 성공이 0/6이었다(A·B 양쪽).
병목 둘 다 **팔 바깥**의 문제라, 큐브와 통을 팔이 닿는 곳에 고정해 두면 이미
검증된 것들(파지 10/11, 들기 4/4, 투입 5/5)만으로 pick-and-place가 완성된다.

배치 근거(kinematics 계산):

  · 큐브  base (0.360, 0.000)   파지 검증 범위(전후 0.29~0.41, 좌우 ±0.055) 한가운데
  · 통    base (0.231, -0.276)  방위 -50도·반경 0.36

통을 옆·가까이 두는 이유가 있다. pan을 돌릴수록 도달 반경이 급히 줄어든다 —
정면 0.545m인데 ±45도에서 0.400, ±75도에서 0.315, ±90도는 아예 닿지 않는다.
그리고 **가까우면 죠를 눕히지 않아도 된다.** 정면 0.5m 투입은 20도 눕혀야 했지만
이 자리는 피치 -95도, 즉 파지 자세 그대로 수직이라 큐브가 흘러내릴 여지가 없다.

큐브와 통은 304mm 떨어져 있어 서로 간섭하지 않고, 통 앞면이 로봇 몸체에서
충분히 나와 팔이 부딪히지 않는다.

사용 (ROS 환경 source 후): python3 armonly_stage.py [색]
"""
import math
import subprocess
import sys
import time

ROBOT = (0.30, 0.0)              # 로봇 base_footprint의 world 위치 (yaw 0)
CUBE_BASE = (0.360, 0.000)       # 파지 지점 (base 기준)
TRASH_BASE = (0.231, -0.276)     # 투입 지점 (base 기준) — 방위 -50도·r 0.36
CUBE_CZ = 0.015
MARKER_DZ, MARKER_BACK = 0.13, 0.09   # 마커는 통 뒤 0.09, 높이 0.13에 선다


def svc(req):
    r = subprocess.run(['gz', 'service', '-s', '/world/room/set_pose',
                        '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000', '--req', req], capture_output=True, text=True)
    return r.returncode == 0 and 'true' in r.stdout.lower()


def model_xy(name, tries=3):
    """모델 좌표 조회. **재시도한다** — gz 서비스가 간헐적으로 응답을 늦춘다.

    한 번만 물어보면 10회 중 1회꼴로 타임아웃해 "배치 실패"가 되는데, 실제로는
    배치가 됐고 확인만 못 한 것이다(실측: 그 실패 3건이 성공률을 9/12로 깎았다).
    """
    for k in range(tries):
        try:
            out = subprocess.run(['gz', 'model', '-m', name, '-p'],
                                 capture_output=True, text=True, timeout=20).stdout
        except subprocess.TimeoutExpired:
            time.sleep(0.5)
            continue
        L = out.splitlines()
        for i, ln in enumerate(L):
            if '- Pose' in ln and i + 1 < len(L):
                try:
                    v = [float(t) for t in L[i + 1].strip().strip('[]').split()]
                    if len(v) >= 2:
                        return v[0], v[1]
                except ValueError:
                    continue
        time.sleep(0.5)
    return None


def cube_name(color):
    """무대에 실제로 있는 픽 대상 이름 — 하드코딩하면 조용히 실패한다."""
    try:
        out = subprocess.run(['gz', 'model', '--list'], capture_output=True,
                             text=True, timeout=10).stdout
        for ln in out.splitlines():
            n = ln.strip().lstrip('-').strip()
            if n.startswith('pick'):
                return n
    except Exception:
        pass
    return f'pick_{color}'


def main():
    color = sys.argv[1] if len(sys.argv) > 1 else 'blue'
    yaw_deg = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    cube = cube_name(color)
    wx, wy = ROBOT

    # 로봇을 정면(yaw 0)으로 세운다. base 기준 좌표를 그대로 world로 옮기려면 필요하다.
    svc(f'name: "jdamr_cube", position: {{x: {wx}, y: {wy}, z: 0.05}}, orientation: {{w: 1}}')
    time.sleep(1.5)

    cx, cy = wx + CUBE_BASE[0], wy + CUBE_BASE[1]
    tx, ty = wx + TRASH_BASE[0], wy + TRASH_BASE[1]
    cy_ = math.radians(yaw_deg)
    svc(f'name: "{cube}", position: {{x: {cx:.4f}, y: {cy:.4f}, z: {CUBE_CZ}}}, '
        f'orientation: {{z: {math.sin(cy_ / 2):.6f}, w: {math.cos(cy_ / 2):.6f}}}')
    svc(f'name: "trash_can", position: {{x: {tx:.4f}, y: {ty:.4f}, z: 0}}, orientation: {{w: 1}}')
    # 마커는 통 뒤에서 로봇 쪽을 본다 — 통과 로봇을 잇는 선의 연장선에 둔다
    ang = math.atan2(ty - wy, tx - wx)
    mx, my = tx + MARKER_BACK * math.cos(ang), ty + MARKER_BACK * math.sin(ang)
    yaw = ang + math.pi           # 무늬가 로봇 쪽(-ang)을 향하도록
    svc(f'name: "trash_marker", position: {{x: {mx:.4f}, y: {my:.4f}, z: {MARKER_DZ}}}, '
        f'orientation: {{z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    time.sleep(1.5)

    print(f'팔 단독 무대 — 주행 없음 · 큐브 yaw {yaw_deg:+.0f}도\n')
    ok = True
    for nm, want in ((cube, (cx, cy)), ('trash_can', (tx, ty))):
        got = model_xy(nm)
        if got is None:
            print(f'  {nm:12s} 좌표 확인 실패'); ok = False; continue
        err = math.dist(want, got)
        print(f'  {nm:12s} world ({got[0]:.3f},{got[1]:+.3f})  오차 {err * 1000:.0f}mm'
              + ('' if err < 0.02 else '  ← 배치 어긋남'))
        ok &= err < 0.02
    print(f'\n  로봇 base 기준: 큐브 {CUBE_BASE} · 통 {TRASH_BASE}')
    print(f'  통 방위 {math.degrees(math.atan2(TRASH_BASE[1], TRASH_BASE[0])):.0f}도 · '
          f'반경 {math.hypot(*TRASH_BASE):.3f}m')
    print(f'\n{"배치 완료" if ok else "배치 실패 — 위 오차를 보라"}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
