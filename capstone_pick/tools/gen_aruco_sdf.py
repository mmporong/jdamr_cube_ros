"""쓰레기통에 붙일 ArUco 마커를 SDF 지오메트리로 만든다.

왜 마커인가: 지금 통 검출은 HSV로 회색 덩어리를 찾는 방식인데, 거리·자세에 따라
+20~237mm로 흔들리고 방향 정보가 아예 없다(스칼라 거리·방위만). 그래서 도착
판정이 수렴해도 팔이 통과 100° 어긋난 방향으로 뻗었다(실측).

피듀셜 마커는 물류 로봇 도킹·팔레트 픽업의 표준이다. solvePnP로 **6-DoF 자세**가
나오고, 마커 한 변 길이가 기지값이라 정확도의 근거가 명확하다. 임계 튜닝이 없다.

텍스처 대신 지오메트리로 만드는 이유: Gazebo 텍스처는 리소스 경로·머티리얼 설정에
의존해 "안 보이는데 원인을 모르는" 상태가 되기 쉽다. 검은 칸을 얇은 박스로 깔면
반드시 렌더된다.

사용: python3 gen_aruco_sdf.py > marker.sdf   (또는 --inject 로 room.world에 삽입)
"""
import argparse
import os
import sys

import cv2
import numpy as np

DICT = cv2.aruco.DICT_4X4_50
MARKER_ID = 0
SIZE = 0.10          # 마커 한 변 [m] — 검출 거리와 정확도를 정하는 기지값
CELLS = 6            # 4x4 + 검은 테두리 1칸
QUIET = 1            # 그 바깥 흰 여백 칸수. ArUco는 검은 테두리 바깥에 흰 여백이
                     # 있어야 검출된다 — 판을 마커와 같은 크기로 만들었더니 테두리가
                     # 판 끝에 붙어 배경(어두운 바닥)과 이어져 0/5 미검출이었다.
PLATE_T = 0.004      # 판 두께
CELL_T = 0.0015      # 검은 칸이 판 앞으로 튀어나오는 두께


def marker_bits():
    """마커의 6x6 흑백 격자. True = 검은 칸."""
    d = cv2.aruco.getPredefinedDictionary(DICT)
    if hasattr(cv2.aruco, 'generateImageMarker'):        # OpenCV 4.7+
        img = cv2.aruco.generateImageMarker(d, MARKER_ID, CELLS)
    else:                                                # 4.6 이하
        img = cv2.aruco.drawMarker(d, MARKER_ID, CELLS)
    return np.asarray(img) < 128


def sdf(x, y, z, yaw_deg):
    """마커판 모델 SDF. (x,y,z)는 판 중심, yaw는 판이 바라보는 방향(+x가 정면)."""
    bits = marker_bits()
    c = SIZE / CELLS
    plate = SIZE + 2 * QUIET * c        # 흰 여백을 포함한 판 크기
    out = [f'''    <model name="trash_marker">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 {np.deg2rad(yaw_deg):.6f}</pose>
      <link name="link">
        <!-- 흰 바탕판(여백 포함). 마커 정면은 +x 방향이다. -->
        <collision name="plate_col"><geometry><box><size>{PLATE_T} {plate:.5f} {plate:.5f}</size></box></geometry></collision>
        <visual name="plate_vis"><geometry><box><size>{PLATE_T} {plate:.5f} {plate:.5f}</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
            <emissive>0.35 0.35 0.35 1</emissive></material></visual>''']
    n = 0
    for r in range(CELLS):
        for col in range(CELLS):
            if not bits[r, col]:
                continue
            # 이미지 좌표(행 아래로, 열 오른쪽으로) → 판 좌표(z 위로, y 오른쪽으로).
            # 판 정면(+x)에서 -x 방향으로 볼 때 시야의 오른쪽은 판의 +y다
            # (forward=(-1,0,0), up=(0,0,1) → right = forward x up = (0,1,0)).
            # 여기서 부호를 뒤집었더니 마커가 좌우 반전돼 사전 조회에 실패했다(0/5).
            py = (col - (CELLS - 1) / 2) * c
            pz = -(r - (CELLS - 1) / 2) * c
            out.append(
                f'        <visual name="c{n}"><pose>{PLATE_T / 2 + CELL_T / 2:.5f} '
                f'{py:.5f} {pz:.5f} 0 0 0</pose>'
                f'<geometry><box><size>{CELL_T} {c:.5f} {c:.5f}</size></box></geometry>'
                f'<material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse>'
                f'<specular>0 0 0 1</specular></material></visual>')
            n += 1
    out.append('      </link>\n    </model>')
    return '\n'.join(out), n


def main():
    ap = argparse.ArgumentParser()
    # 통은 (0.15, 0.62). 로봇은 원점 쪽에서 오므로 마커는 통 뒤에 세워 로봇을 향하게 한다
    # (개구부를 가리지 않으면서 통보다 높아 멀리서도 보인다).
    ap.add_argument('--x', type=float, default=0.15)
    ap.add_argument('--y', type=float, default=0.71)
    ap.add_argument('--z', type=float, default=0.13)
    ap.add_argument('--yaw', type=float, default=-90.0, help='판 정면(+x)이 향할 방향 [deg]')
    ap.add_argument('--inject', metavar='WORLD', help='room.world에 삽입')
    a = ap.parse_args()
    body, n = sdf(a.x, a.y, a.z, a.yaw)
    if not a.inject:
        print(body)
        print(f'<!-- 검은 칸 {n}개, 한 변 {SIZE}m, id {MARKER_ID} -->', file=sys.stderr)
        return
    s = open(a.inject, encoding='utf-8').read()
    if '<model name="trash_marker">' in s:
        head = s.index('    <model name="trash_marker">')
        tail = s.index('</model>', head) + len('</model>')
        s = s[:head] + body + s[tail:]
        print(f'마커 갱신 (검은 칸 {n}개)')
    else:
        anchor = '    <model name="trash_can">'
        s = s.replace(anchor, body + '\n\n' + anchor, 1)
        print(f'마커 삽입 (검은 칸 {n}개)')
    open(a.inject, 'w', encoding='utf-8').write(s)
    print(f'  중심 ({a.x}, {a.y}, {a.z}), 정면 {a.yaw}도, 한 변 {SIZE}m, id {MARKER_ID}')


if __name__ == '__main__':
    main()
