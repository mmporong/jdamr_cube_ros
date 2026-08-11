"""capstone_pick: 비전 기반 물체 판단 + 캘리브레이션 파지 파이프라인.

아키텍처 (모두 계산 기반, 시뮬 참값 비의존):
  1) Perception  : RGBD 컬러 HSV 검출 → 뎁스+카메라 내부행렬 역투영(광학 프레임)
                   → 광학→링크 축 변환 → TF로 base_footprint 3D 좌표
  2) Approach    : 물체가 파지 포켓 좌표(캘리브레이션 상수)에 오도록 cmd_vel P제어 주행,
                   주행 중 주기적 재인식으로 오차 보정
  3) Grasp       : 접힘(주행) 자세 → 전개 → C자세(pan 조준) → 2단 닫기
  4) Verify      : 그리퍼 조인트 스톨 각도로 파지 성공 판정
                   (물체를 물면 완전 닫힘 각도 -0.17에 도달하지 못함)
  5) Lift        : 저속 2단 들어올리기

캘리브레이션 상수의 출처: 2026-07-28 Gazebo 계측 세션
  - 파지 포켓(base): (0.397, 0.004, z≈0.19) — 죠 스윕 회전행렬 실측
  - 핀치 축은 xy 대각 45°, 고정 손끝 (0.388,0.000) ↔ 움직 손끝 닫힘 (0.406,0.018)
  - 손가락 충돌은 URDF에 추가된 primitive box (트라이메시 관통 문제 해결)

======================================================================
아래는 이 파일을 처음 읽을 때의 안내 (기존 주석은 전부 실측 근거다)
======================================================================

■ 읽는 순서

main()이 전체 시퀀스이고, 나머지는 전부 거기서 불린다. 위에서 아래로 읽지 말고
main()을 먼저 보고 필요한 함수로 내려가는 편이 빠르다.

  main()                     3사이클 재시도 정책 + 실패 사유 분류
   ├ approach()              ② 비전 폐루프 주행 — 물체를 '파지 포켓'에 갖다 놓는다
   │   ├ locate_object()        전방 카메라로 물체의 3D 좌표를 구한다
   │   │   └ _pixel_to_base()      픽셀+거리 → 로봇 기준 좌표 (역투영 + TF)
   │   └ drive()                cmd_vel로 실제 주행
   ├ grasp()                 ③④ 하강·정렬·닫기, 그리고 물었는지 판정
   │   ├ wrist_align()          손목 카메라로 최종 미세 정렬 (바닥 모드)
   │   └ gripped()              그리퍼 각도 구간으로 1차 물림 판정
   ├ lift()                  ⑤ 계단식 들어올리기 (첫 순간이 가장 잘 미끄러진다)
   └ carry_to_trash()        ⑥ 쓰레기통까지 운반 (place_target=trash일 때)
       ├ locate_trash()         회색 구조물 검출 (색이 아니라 무채색+높이로)
       ├ holding()              운반 중 계속 "아직 쥐고 있나" 확인
       └ recover_dropped()      떨어뜨렸으면 재접근·재파지

■ 이 파이프라인이 푸는 문제

"카메라로 물체를 보고, 거기까지 가서, 집어서, 든다." 사람에겐 한 동작이지만
로봇에겐 서로 다른 네 가지 문제이고, 각각이 다른 이유로 실패한다.

  본다   → 어느 픽셀이 물체인가 (색 또는 YOLO), 그 픽셀이 3D로 어디인가
  간다   → 지금 위치와 목표의 차이를 어떻게 좁히나 (폐루프 제어)
  집는다 → 손이 정확히 그 자리에 있나, 죠 사이에 물체가 들어왔나
  든다   → 드는 동안 미끄러지지 않나

이 파일이 긴 이유는 네 번째 줄에 있다. 앞의 셋은 원리대로 하면 되는데,
"정말 쥐었나"는 원리로 알 수 없어 신호를 여러 겹 쌓아야 했다(아래 참조).

■ 알아 두면 코드가 읽히는 개념 넷

1) 좌표 프레임(frame)
   같은 점도 "누구 기준인가"에 따라 숫자가 다르다. 이 파일에 나오는 기준은 둘이다.
     rgbd_camera_link : 카메라가 보는 기준
     base_footprint   : 로봇 바닥 중심 기준 (x=전방, y=좌, z=위)
   카메라가 "내 앞 0.5m"라고 해도 팔은 자기 어깨 기준으로 움직이므로 변환이 필요하다.
   그 변환을 관리하는 체계가 TF이고, _pixel_to_base()가 그걸 쓴다.

2) 역투영(back-projection)
   카메라는 3D를 2D로 눌러 찍는다. 그 반대를 하려면 정보가 하나 더 필요한데,
   그게 뎁스(거리)다. 픽셀 좌표 + 거리 + 카메라 내부행렬(초점거리·중심)이 있으면
   3D 점 하나가 복원된다. 수식은 _pixel_to_base() 안 세 줄이 전부다.

3) 폐루프(closed-loop)
   "3m 앞으로 가라"고 명령만 하고 끝내면(개루프) 바퀴가 미끄러진 만큼 어긋난다.
   그래서 approach()는 [보고 → 오차 계산 → 조금 움직이고 → 다시 본다]를 반복한다.
   매 반복이 오차를 줄이므로 개별 명령이 부정확해도 결국 목표에 닿는다.

4) 추측항법(dead reckoning)
   물체가 너무 가까워지면 카메라 시야에서 벗어난다. 그때는 마지막으로 본 위치를
   오도메트리(바퀴 회전으로 추정한 자기 위치) 기준으로 기억해 두고, "내가 이만큼
   움직였으니 물체는 이제 여기쯤"으로 계산해 접근을 이어간다. 시간이 갈수록
   오차가 쌓이므로 임시방편이고, 코드에도 '추정 추적'으로 로그가 남는다.

■ 왜 파지 판정이 이렇게 복잡한가 (holding() 주변)

"쥐고 있나"를 아는 확실한 방법이 없어서, 서로 다른 실패를 각각 막는 신호를
겹쳐 놓았다. 각 층은 앞 층이 뚫린 사고를 겪고 추가된 것이다.

  1층 그리퍼 각도 구간   허공을 쥐면 끝까지 닫히고, 물면 물체 두께에서 멈춘다
                         → 뚫림: 물체가 빠져도 그리퍼는 그 각도를 유지한다
  2층 손목캠 블롭 면적   쥐고 있으면 렌즈 앞을 크게 채운다
                         → 뚫림: 손목을 롤로 돌리면 시야를 벗어난다
  3층 전방캠 블롭 높이   바닥(z≈0.015)과 운반 중(z≈0.19)은 물리 높이가 다르다
                         → 뚫림: 둘 다 안 잡히는 사각이 있다
  4층 능동 판별          10cm 후진해 본다. 쥔 물체는 화면이 안 변하고(강체 결합),
                         떨어진 물체는 뒤로 멀어져 크게 이동한다

이 구조가 이 프로젝트에서 가장 배울 만한 부분이다. 센서 하나로 판정이 안 될 때
"더 좋은 센서"가 아니라 "실패 방식이 다른 신호를 겹치는" 접근을 택했다.
"""
import json
import math
import os
import subprocess
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

from capstone_pick import kinematics as K

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

# ---- 캘리브레이션 상수 (실측 기반) ----
POCKET_BASE = (0.410, 0.000)  # 그리드 스캔으로 파지 확정된 포켓 (2026-07-28, lift0.48)      # 파지 포켓 xy [m, base_footprint]
PAN_AXIS_X = 0.159                # shoulder_pan 회전축 x [m]
PAN_BASE_BEARING = math.radians(-1.1)   # pan=0일 때 포켓 방위각
GRIPPER_OPEN = 0.5                # 3cm 물체 통과에 충분한 최소 열림
GRIPPER_STAGE1 = 0.25
GRIPPER_CLOSED = -0.17
GRIP_HOLD_THRESHOLD = -0.05        # 조임 후 각도가 이보다 크면(덜 닫힘) 접촉은 있었다는 뜻
# 물림 판정 구간. 아래로는 허공 닫힘(-0.17)·모서리 헛집기(-0.07)·미닫힘(0.0)을,
# 위로는 열린 상태(0.5+)를 걸러낸다.
# 상한 0.55: 종전 0.30은 3cm 큐브 시절 값이다. 월드가 4cm로 돌아온 뒤로는 큐브가
# 두꺼운 만큼 죠가 덜 닫힌 채 멈추는데, 그걸 전부 '얕은 걸침'으로 버리고 있었다.
# 포켓 스윕 실측(2026-08-07, tools/pocket_scan.py): 큐브 x=0.365~0.405 다섯 자리
# 전부 실제로 들어올려졌고(gz 좌표 z>0.05) 그때 물림각이 0.408~0.445였다.
# 즉 진짜 파지가 매번 실패로 판정되고, 이어지는 '재물림'이 큐브를 밀어내 망쳤다.
# 하한 0.32: 4cm 큐브를 죠 사이에 제대로 물면 그 두께만큼 벌어진 채 멈춘다. 그보다
# 더 닫혔다면 큐브가 죠 사이에 없다는 뜻이다(모서리 걸침·빈손에 가까움).
# 실측 대조 — 각도 / 들기 후 능동 판별(10cm 후진 시 블롭이 함께 움직이는지):
#     0.341 0.381 0.407 0.436  → 전부 HOLDING 유지, 투입까지 성공
#     0.299 0.222              → 둘 다 DROPPED (블롭 134~146px 뒤로 멀어짐)
# 종전 0.05로는 이 둘을 통과시켜 운반을 다 하고 나서야 실패했다. 여기서 걸러
# 곧바로 재파지하는 편이 훨씬 싸다.
GRIP_HOLD_MIN, GRIP_HOLD_MAX = 0.32, 0.55
# 물림 판정의 1차 신호는 그리퍼 관절 부하다(gripped 참조). 실측 물림 |10.0| 대
# 빈손 |0.001|이라 임계는 어디를 잡아도 되지만, 흔들림 여유로 1.0을 쓴다.
GRIP_LOAD_MIN = 1.0
# 접촉각에서 이만큼만 더 조인다. 크게 잡으면 죠가 큐브를 뚫고, 0으로 두면 위치
# 오차가 없어 물림이 헐거워진다. 4cm 큐브 물림각이 0.38~0.45이므로 0.05는 약 12%.
GRIP_HOLD_MARGIN = 0.05
# 쥐고 있는지를 손목캠 블롭 면적으로 판정하는 임계(px). 실측(운반 자세):
# 쥔 상태 36721 / 큐브가 바닥에 떨어진 상태 6079 / 빈손 0. 6배 차이라 중간값보다
# 낮게 잡아도 안전하다. 쥐고 있으면 큐브가 렌즈 앞 몇 cm에 고정되므로 팔 자세가
# 바뀌어도 이 값은 유지된다.
# FOV 87도에서는 같은 물체가 (tan30/tan43.5)^2 = 0.37배 면적으로 보인다.
# 60도 기준 15000 → 5500 (실측 검증 대상).
# 큐브를 4cm→3cm로 줄이면서 손목캠에 잡히는 면적이 (3/4)²=0.56배가 됐다.
# 5500(4cm 기준)을 그대로 두었더니 실제로 쥐고 있는데도(그리퍼 0.277 유지)
# 면적 2254 < 5500으로 "놓쳤다"고 오판해 성공 직전에 실패 처리됐다(실측).
HOLD_AREA_MIN = 5500.0
# 실측 근거: 3cm 큐브 정상 파지는 항상 +0.07 이상, 모서리 헛집기는 -0.07 부근(가짜 성공 사례)
ARM_JOINTS = ['arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex',
              'arm_wrist_flex', 'arm_wrist_roll']
# 주행 자세: 추종오차 0.000rad 검증(2026-07-28), 그리퍼가 데크 위로 뜨는 단정한 접힘
POSE_FOLDED = dict(zip(ARM_JOINTS, [0.0, -0.4, 1.0, 0.2, 0.0]))
POSE_DEPLOY = dict(zip(ARM_JOINTS, [0.0, 0.55, 0.2, 0.7, 0.0]))
POSE_PRE = dict(zip(ARM_JOINTS, [0.0, 0.15, 0.2, 0.9, 0.0]))    # 상공 대기
POSE_WAY = dict(zip(ARM_JOINTS, [0.0, 0.28, 0.32, 0.9, 0.0]))   # 수직 하강 경유점(실측)
POSE_GRASP = dict(zip(ARM_JOINTS, [0.0, 0.48, 0.2, 0.9, 0.0]))   # 그립(스캔 파지 확정)
POSE_LIFT1 = {'arm_shoulder_lift': 0.15}
POSE_LIFT2 = {'arm_shoulder_lift': 0.05}
# 바닥 모드: 수직 하향 자세족(lift+elbow+wrist≈1.58) TF 스윕 + 파지 실측으로 확정(2026-07-29).
# 닫힘이 큐브를 고정 죠 쪽으로 쓸어담아 물리는 방식 — 포켓은 쓸림 거리까지 반영한 실측값.
POSE_PRE_FLOOR = dict(zip(ARM_JOINTS, [0.0, 0.85, 0.15, 0.58, 0.0]))    # 바닥 상공 대기
# 바닥 파지 lift는 1.15: 1.20에서는 손끝 충돌 박스가 바닥을 눌러 로봇 앞이 들리고
# (pitch −1.15→−1.32°, base z +1mm) 구동륜 접지가 풀려 creep·미세주행이 월드 실변위
# 0mm가 된다(odom만 22mm 허위 전진 → 이후 odom 로직까지 오염). 스윕 실측(2026-08-03):
# lift 1.18 스침 / 1.16·1.14 청정(pitch −1.15 유지, creep 21~23mm 회복, 파지 성립).
# 접촉 경계 ~1.17에서 0.02 여유. wrist는 자세족 불변식(lift+elbow+wrist≈1.58,
# 툴 수직 하향)을 지키도록 lift 감소분만큼 올린 0.28 — lift만 줄이면 죠 평면이
# 2.9° 기울어 들기·운반 내내 유지된다(lift()의 wrist 보상이 합을 보존하므로).
POSE_GRASP_FLOOR = dict(zip(ARM_JOINTS, [0.0, 1.15, 0.15, 0.28, 0.0]))  # 바닥 파지
# 포켓 스윕 실측(2026-08-07, tools/pocket_scan.py — 4cm 큐브, 자세 1.15/0.28):
#     x  0.365 0.375 0.385 0.395 0.405 | 0.415 0.425
#     들림  O     O     O     O     O   |   X     X
#     물림각 .418  .445  .421  .409  .408 | -.170 -.170
# 성립 구간이 40mm나 되고 그 중앙이 0.385다. 종전 0.381은 구자세(1.20/0.23) 스윕
# 값이었다. 구간이 이만큼 넓다는 것이 중요하다 — 접근·정렬 정밀도가 ±20mm면 충분해서
# 하강 후 미세 보정에 매달릴 이유가 없다.
#
# 조준은 중앙(0.385)이 아니라 구간 위쪽 0.400으로 한다. 남는 오차의 부호가 중요하기
# 때문이다: 큐브가 멀면 팔을 뻗어 메울 수 있고(실측 +11/+45mm 둘 다 파지 성공),
# 가까우면 팔을 당겨야 하는데 그건 세 번 다 허공이었다(-22/-28/-45mm). 접근 잔차가
# ±45mm이므로 조준을 위로 밀어 '가까움' 쪽으로 떨어지는 빈도를 줄인다.
# 조준은 WRIST_REF를 교정한 그 자리(0.385)로 둔다. 한때 0.400·0.430으로 옮겨 봤는데,
# 손목캠 기준점이 0.385에서 잰 값이라 조준을 옮긴 만큼 그대로 잔차로 나타난다
# (0.430 조준 → 손목캠 +48mm). 접근 잔차는 계통 편향이 아니라 분산이 크다(±45mm) —
# 조준으로는 못 없애고, 아래 파지 단계가 그 분산을 흡수해야 한다.
# 2026-08-10 재측정 (grasp_ref_scan.py): 성립 구간 0.385~0.425, 중앙 0.405.
# 종전 0.385는 이 구간의 **하단 경계**였다. 기준점을 관절 보간 경로(pocket_scan)에서
# 쟀는데 파이프라인은 직교 하강을 쓰고, 그쪽이 아랫턱을 24mm 더 앞에 둔다
# (아랫턱 x 0.344 대 0.368). 죠가 앞에 있으니 큐브도 그만큼 앞이어야 물린다.
# IK 전환(2026-08-11): 이제 '포켓'은 파지가 성립하는 유일한 점이 아니라
# **IK 작업공간의 중앙**이다. 접근은 큐브를 이 근처로만 데려오면 되고, 나머지는
# 팔이 흡수한다. 실측 성공 구간은 전후 0.29~0.41, 좌우 ±0.05(ik_grasp_probe).
# 0.350(작업공간 중앙)에서 0.395로 올렸다. 그 거리에서는 큐브가 전방캠에 계속
# 보여 파지 직전 재관측이 가능하다 — 근접 사각으로 들어가면 추측에 의존하게 되고,
# 그 추측이 좌우로 38.5mm 어긋났다. IK 범위(0.29~0.41)의 상단이라 여유도 있다.
POCKET_FLOOR = (0.395, 0.000)   # 큐브가 보이는 거리 [m, base_footprint]
# 파지가 **실측으로 검증된** 범위 (ik_grasp_probe.py, 시도 가능 지점 10/11).
# IK 해가 존재하는 범위보다 좁다 — 해가 있어도 실제로 물리는 것은 별개다.
IK_X_MIN, IK_X_MAX, IK_Y_MAX = 0.29, 0.41, 0.055
# ---- 손목캠 비주얼 서보 (2026-08-11 실측, wrist_servo_probe.py) ----
# 전방캠은 파지 거리에서 큐브를 못 본다 — 팔 접힘 최고점(z=0.381)이 카메라(z=0.350)보다
# 높아 카메라를 올려도 팔이 시선을 가린다. 손목캠은 팔에 달려 있어 그 문제가 없다
# (Stretch 같은 상용 모바일 매니퓰레이터가 손목캠을 두는 이유).
# 프리그래스프 자세에서 큐브가 목표대로 있을 때의 blob 위치와, 큐브를 옮겼을 때의 감도.
# 축이 잘 분리된다 — 전후는 blob x, 좌우는 blob y이고 교차항이 4% 이하다.
WRIST_SERVO_REF = (341.4, 42.0)
WRIST_SERVO_FWD = 3091.0        # 전후 1m당 blob x [px]
WRIST_SERVO_LAT = -2554.0       # 좌우 1m당 blob y [px]
WRIST_SERVO_TOL = 0.003         # 이보다 작으면 보정을 멈춘다 [m]
# 완전 닫힘(-0.17)과 '물체를 물어 벌어진 상태'를 가르는 선. 25mm 큐브의 물림각은
# 0.05~0.2라 여유가 크다. 물체를 놓치면 죠가 끝까지 닫히므로 이 하나로 구분된다.
GRIP_EMPTY_MAX = -0.10
# 큐브 중심 높이는 **크기에서 정한다.** 비전 z는 0.005~0.013으로 튀는데(뎁스 노이즈)
# 그 값을 그대로 쓰면 죠가 2~3mm 어긋나게 내려가 물림이 얕아진다 — 실측: cz=0.010으로
# 잡아 물림각 0.043(계산상 0.15)이 나왔고 들다가 놓쳤다. 물체 크기는 아는 값이다.
CUBE_SIZE = 0.030
CUBE_CZ = CUBE_SIZE / 2.0
# 운반 자세도 **IK로** 만든다. 고정 자세(POSE_CARRY)로 전환하면 죠 각도가 급변해
# 물체가 빠진다 — docs/18에 이미 기록된 함정이고, IK 경로에서 그 연속성이 깨졌다
# (실측: 물림각 0.219로 제대로 물어 놓고 전환 순간 -0.170으로 놓쳤다).
# 든 큐브를 로봇 기준 이 위치에 둔다는 뜻이다.
CARRY_X, CARRY_UP = 0.330, 0.180
FLOOR_Z_MAX = 0.08              # 검출 높이가 이보다 낮으면 바닥 모드
# ---- 쓰레기통 투입 (2026-07-29 실측) ----
# 통 = 16cm 정사각, 벽 높이 0.18m, 개구부 13.6cm. 회색이라 색상(H)은 무의미하고
# 무채색(S<45) + 어두움(45<V<115) + 뎁스·높이 게이트로 분리한다(바닥 V=196, 그림자는 z<0).
TRASH_S_MAX, TRASH_V_LO, TRASH_V_HI = 45, 45, 115
TRASH_D_LO, TRASH_D_HI = 0.35, 2.0   # 상한을 넓히면 원경 벽이 통과 한 덩어리로 붙는다(실측)
TRASH_Z_LO, TRASH_Z_HI = 0.02, 0.30
TRASH_MIN_AREA = 400
# ---- 쓰레기통 마커 (ArUco) ----
# HSV 회색 덩어리 검출은 거리·자세에 따라 +20~237mm로 흔들리고 방향 정보가 아예 없어,
# 도착 판정이 수렴해도 팔이 통과 100° 어긋난 방향으로 뻗었다(실측). 피듀셜 마커는
# solvePnP로 6-DoF 자세를 주고 한 변 길이가 기지값이라 정확도 근거가 명확하다 —
# 물류 로봇 도킹의 표준. 실측 정확도: 거리 ±25mm, 방위 ±2°.
TRASH_ARUCO_DICT = cv2.aruco.DICT_4X4_50
TRASH_ARUCO_ID = 0               # 통 마커 id — 월드의 trash_marker와 일치해야 한다
ARUCO_FAIL_DIR = os.path.expanduser('~/capstone_tools/logs/aruco_miss')
TRASH_ARUCO_SIZE = 0.10          # 마커 한 변 [m] — 월드의 trash_marker와 일치해야 한다
# 마커 중심에서 본 통 중심의 위치 [마커 좌표계: x=오른쪽, y=위, z=마커 앞으로].
# 마커는 통 뒤 0.09m·위 0.13m에 세워져 있다(gen_aruco_sdf.py 기본값).
TRASH_MARKER_TO_CENTER = (0.0, -0.13, 0.09)
# 앞면 → 중심 반깊이. 0.08은 과소 보정이었다 — 착지가 일관되게 짧았다
# (실측: 직선 51mm, 회전 114mm 부족). 0.115로 올려 계통 편향을 제거한다.
TRASH_HALF = 0.115
# 운반·투하 자세 = 들어올리기가 끝나는 자세 그대로. 자세를 '전환'하면 팔꿈치가 펴지는
# 관성으로 물체가 빠진다(계단식·저속으로 나눠도 반복 실패). 전환을 없애는 것이 해법이고,
# pan 회전만 하는 것은 옆에 내려놓기에서 이미 검증된 동작이다.
# 실측: 그리퍼 x=0.353 z=0.300 → 큐브 하단 0.205 (통 벽 0.18 위 2.5cm).
# 여유 0.6cm(lift 0.15) 자세로는 주행 흔들림에 큐브가 통 벽에 걸렸다 — 실측 확인.
POSE_CARRY = dict(zip(ARM_JOINTS, [0.0, 0.15, 0.15, 1.28, 0.0]))
# 운반·탐색 중에는 pan을 옆으로 빼 카메라 시야를 연다.
# -1.0(57도)에서 -1.4(80도)로 키웠다. 카메라 수평 반각이 33도인데 팔은 굵어서
# 57도로는 프레임 가장자리에 남고, 그 회색 부분이 통 마스크(무채색·어두움)에
# 걸린다. **팔은 로봇과 함께 도니 base 좌표가 고정**이라, 차체가 22도씩 도는데도
# 통 관측값이 소수점까지 동일한 상태가 71회 이어졌다(실측). 진짜 통이 프레임에
# 들어오자 면적이 228 → 825로 뛰며 방위가 23.7도에서 0.7도로 정상화됐다.
# carry_pan_probe 실측에서도 마커 면적이 -1.0에서 4182, -1.4에서 4317로 더 컸다.
POSE_CARRY_SCAN = dict(POSE_CARRY, arm_shoulder_pan=-1.4)
# 통 중심이 이 거리에 오면 그리퍼가 개구부 바로 위. 0.364→0.344: 짧은 착지
# 편향 보정의 나머지 절반 — 중심 초과 리치 46+20=66mm로 개구부 반경(68mm) 안.
# ArUco 마커 기반으로 바뀌면서 정지 거리를 0.344 → 0.500으로 늘렸다.
# 종전에는 통 앞 0.344m까지 차체로 파고들었는데, 그 거리에서는 통이 카메라 프레임
# 아래로 잘려 검출이 안 되고(실측 미검출) 마지막 0.17m를 오도메트리 추측으로 갔다.
# 그 추측이 10cm 어긋나 큐브가 통에서 0.20~0.55m 떨어진 곳에 떨어졌다(실좌표 확인).
# 지금은 검출이 확실한 거리(ArUco 오차 ±25mm)에서 멈추고 나머지를 팔로 뻗는다 —
# 팔은 야코비안으로 mm 단위 제어가 되고 차체는 안 된다.
TRASH_POCKET_X = 0.500
# 투하 자세: 운반 자세(x=0.364)로는 통 중심에 9.6cm 못 미쳐 통 앞 바닥에 떨어졌다(실측).
# 로봇 전면(0.275)과 통 벽 때문에 더 접근할 수 없으므로 팔을 뻗어 채운다. 이 자세는
# 그리퍼가 기울어 물체를 놓지만, 이미 통 개구부 위이므로 그대로 투입이 된다.
# 리치 스윕 실측(2026-08-07): 죠 높이 0.13~0.32m 조건에서 최대 전방 리치가
# 0.528m(자세 0.35/-0.20/0.30, 죠 z=0.271). 통 벽(0.09)보다 한참 위라 걸리지 않는다.
# 정지 거리 0.500m와 28mm 여유 — 통 개구부 반경 68mm 안이다.
POSE_DROP = dict(zip(ARM_JOINTS, [0.0, 0.35, -0.20, 0.30, 0.0]))   # 리치 x=0.528
# 손목 카메라 최종 정렬(바닥 모드) — 접근 비전은 근접(<0.45m)에서 팔·시야각에 가려지므로
# 마지막 정렬은 손목 RGB로. 실측(2026-07-29, 바닥 호버): 손목캠은 90° 회전 장착이라
# px=전후거리(82px/cm), py=좌우(67px/cm). pan 1rad당 py -1740px(포켓 반경 0.26m×6700px/m와 일치).
# 포켓 0.385에 실제로 큐브를 놓고 상공 자세에서 직접 잰 값(2026-08-07, pocket_scan.py).
# 포켓·상공기준·하강기준을 각각 다른 날 다른 자세에서 재는 바람에 서로 25mm 어긋나
# 있었고, 그 차이를 '하강 후퇴 보정'으로 덧대다 오히려 큐브를 지나쳤다. 이제 셋을
# 한 번의 스윕에서 뽑으므로 중간 보정이 필요 없다 — 포켓을 옮기면 셋 다 다시 잰다.
WRIST_REF = (497.1, 78.2)
WRIST_PX_PER_M = 4054.0         # 전후 1m당 px — 같은 스윕 7점 회귀
# 위 세 상수는 손목캠 FOV 87도(RealSense D405 규격) 기준 실측(2026-08-05, wrist_calib.py).
# FOV 60도 시절 값은 (412.9,360.6)/6073/2000 — FOV를 바꾸면 반드시 재실측할 것.
# pan 게인: 호버 자세에서 pan을 -0.1~+0.1로 돌려 잰 값(423.7→168.2px / 0.2rad).
# 이전 값 1740은 과대라 보정이 매번 부족했고, 남은 약 4mm 오차가 죠 한쪽에 치우친
# 얕은 물림을 만들어 들어올리는 첫 순간 미끄러지는 원인이 됐다.
# pan 1rad당 py 변화. 실측하면 상수가 아니라 방향·크기에 따라 568~2905 px/rad로
# 비선형이다(pan -0.10/-0.05/+0.05/+0.10에서 568/757/2905/1736). 상수 하나로 맞출 수
# 없으므로 큰 쪽에 가깝게 잡아 보정을 보수적으로 만든다 — 작게 잡으면 오버슈트해
# 정렬이 진동한다(종전 1277에서 blob y가 215~448로 흔들렸다).
WRIST_PY_PER_PAN = 641.0
# 하강(파지) 자세 기준 — 손목캠을 36° 기울인 뒤로는 내려간 상태에서도 큐브가 보인다.
# 위 WRIST_REF와 같은 스윕에서, 큐브가 포켓 0.385에 있고 팔이 명목 파지 자세로 내려간
# 상태에서 잰 값이다. 종전 379.2는 상공 기준과 25mm 어긋나 있었다 — 상공에서 잘 맞춰
# 내려와도 여기서는 "아직 멀다"고 읽혀 죠가 큐브를 지나칠 때까지 밀었다.
GRASP_REF = (536.1, 174.4)
GRASP_PX_PER_M = 4772.0         # 직교 하강 경로 5점 회귀 (종전 2365는 관절 보간 기준이라 2배 어긋났다)
# 전후 1m를 팔로 움직이는 관절 조합 (높이 불변). 파지 중에는 차체를 못 쓰므로
# — 팔이 내려간 상태에서 차체를 밀면 죠가 큐브를 쳐낸다 — 팔로 해결해야 한다.
# 2026-08-07 야코비안 실측(파지 자세 기준):
#     lift  1rad당 전후 -56mm / 높이 -123mm
#     elbow 1rad당 전후 -130mm / 높이 -37mm
# 두 방향이 충분히 달라 높이를 유지한 채 전후만 뽑을 수 있다.
# 전후 +10mm = lift +0.0269, elbow -0.0886 → 1m당 아래 값.
REACH_LIFT_PER_M = 2.69
REACH_ELBOW_PER_M = -8.86
# 같은 계산을 상공(호버) 자세에서도 한다. 팔이 더 펴져 있어 감도가 4배 다르므로
# 파지 자세 값을 그대로 쓰면 4배 과보정된다.
# 2026-08-07 야코비안 실측(POSE_PRE_FLOOR 기준):
#     lift  1rad당 전후 -221mm / 높이 -165mm
#     elbow 1rad당 전후 -273mm / 높이 -54mm
HOVER_LIFT_PER_M = 1.631
HOVER_ELBOW_PER_M = -4.983
# 하강은 수직이 아니다. TF 실측(2026-08-07):
#   PRE_FLOOR   죠 x=0.3602 z= 0.0500
#   GRASP_FLOOR 죠 x=0.3352 z=-0.0007   → x -25.0mm / z -50.7mm
# 즉 26° 기울어진 직선이라, 표준 파지(MoveIt Grasps)가 요구하는 "pre-grasp는 grasp에서
# 접근축으로만 떨어진 자세"를 우리 자세족은 만족하지 않는다.
# 그렇다고 이 25mm를 하강 자세에 따로 얹을 필요는 없다 — 상공 기준(WRIST_REF)과
# 포켓(POCKET_FLOOR)을 같은 스윕에서 재면 후퇴량이 이미 그 안에 들어 있기 때문이다.
# 한때 따로 얹었다가 25mm를 두 번 세어 죠가 큐브를 지나쳤다(허공 파지 3/3).
# 파지 성립 구간이 40mm라 이 정도 경사는 그 안에서 흡수된다.
# 팔로 메울 수 있는 전후 한계. 이보다 크면 관절이 자세족을 벗어나 죠 평면이 기울므로
# 접근 단계로 되돌린다.
REACH_LIMIT = 0.045
# 이 안쪽이면 아예 손대지 않는다. 포켓 성립 구간이 40mm(±20mm)라 그 안에서는
# 보정이 이득이 없고, 팔을 움직인 만큼 기준점만 흐려진다.
REACH_DEADZONE = 0.020
# 파지 성립 구간(0.365~0.405)의 반폭. 하강 후 이보다 어긋나 있으면 닫아도 허공이다.
POCKET_HALF = 0.020
# 하강은 큐브 뒤쪽에서 시작한다. 큐브 바로 위로 내리면 아랫턱이 큐브 윗면에 얹히거나
# 큐브를 앞뒤로 쳐낸다 — 아무리 x를 잘 맞춰도 '지나가는 경로'가 큐브를 관통한다.
# 뒤로 물러난 자리에서 바닥까지 내린 다음, 바닥 높이에서 수평으로 밀고 들어간다.
# 죠가 큐브 윗면 위를 지나갈 일이 없어진다. 40mm는 큐브 한 변(40mm)만큼이다.
DESCEND_BACKOFF = 0.040
# 상공 정렬 + 죠 안착 + 재물림을 합친 누적 상한. 계수가 한 점 선형화라 이보다 멀리
# 뻗으면 '높이 불변'이 깨진다(관절 한계 자체는 여유가 있다).
REACH_TOTAL_MAX = 0.080
# 큐브 기울기 정렬: 손목캠 minAreaRect 각도는 큐브 yaw와 1:1 반전(실측: +20도→rect 70).
# 카메라는 roll 관절 앞단이라 롤을 돌려도 측정 불변 — 측정·제어 분리.
WRIST_RECT_REF = 90.0           # 정렬 큐브의 rect 각 (실측)
ROLL_SIGN = 1.0                 # 롤 방향 부호 (파지 실험으로 확정)
# 대상 색 HSV 범위 목록 (OpenCV H 0-179) — 2026-07-28 카메라 실측 기반.
# 실측: 병(오렌지) H15-19/S163/V206, 갈색 장애물 H≈13/V≈106(실조명), 노랑 팔 H30-34, 빨간 데크 H0-4.
# 갈색↔orange는 H15+V160 이중 게이트로, 노랑 팔은 H로 분리. 파란 장애물 실린더(H≈107)는
# blue 범위 안 — 최근접 후보 선택·타깃 락·높이 게이트로 회피. red는 H 랩어라운드라 범위 2개,
# 빨간 데크(자기 몸)는 뎁스 게이트(MIN_OBJECT_DEPTH)가 거른다.
TARGET_COLOR_RANGES = {
    'blue': [((100, 130, 100), (135, 255, 255))],
    'red': [((0, 150, 100), (6, 255, 255)), ((174, 150, 100), (179, 255, 255))],
    'green': [((45, 80, 80), (75, 255, 255))],
    'orange': [((15, 120, 160), (22, 255, 255))],
    'pink': [((140, 80, 80), (170, 255, 255))],
}
MIN_OBJECT_DEPTH = 0.30   # 이보다 가까운 검출은 자기 몸(팔)으로 간주하고 제외
MIN_COMPONENT_AREA = 20
OBJECT_HALF_DEPTH = 0.015
APPROACH_STANDOFF = 0.10  # 팔을 먼저 내린 뒤 이 거리만큼 전진 삽입 (하강 충돌 방지)  # 뎁스는 물체 앞표면을 재므로 중심 보정용 반폭 [m]
CAMERA_FRAME = 'rgbd_camera_link'


class PickNode(Node):
    """파지 파이프라인 전체를 담은 ROS2 노드.

    입출력을 먼저 보면 구조가 잡힌다.

      받는 것(구독)
        rgbd_camera/image        전방 컬러 — 물체 검출
        rgbd_camera/depth_image  전방 거리 — 역투영에 필요
        rgbd_camera/camera_info  카메라 내부행렬 (초점거리·중심)
        wrist_camera/image_raw   손목 컬러 — 근접 정렬, 파지 확인
        odom                     바퀴로 추정한 자기 위치 — 추측항법·스톨 감지
        joint_states             관절 각도 — 도달 확인, 그리퍼 물림 판정

      보내는 것
        cmd_vel                                 주행 속도 (토픽)
        arm_controller/follow_joint_trajectory  팔 자세 (액션)
        gripper_controller/gripper_cmd          그리퍼 개폐 (액션)

      그 외
        TF 버퍼 — 카메라 기준 좌표를 로봇 기준으로 옮기는 데 쓴다

    실행 파라미터는 `--ros-args -p 이름:=값`으로 준다. 자주 쓰는 것은
    target_color(집을 색), place_target(side/trash), detector(yolo/hsv),
    speed_scale(전체 속도 배율)이다.

    상태를 self에 두는 방식이 눈에 띌 텐데(_target_px, _trash_odom, _hold_grace 등),
    콜백과 긴 시퀀스가 값을 주고받아야 해서다. 대부분 getattr(self, 이름, 기본값)으로
    읽어 "아직 없으면 기본값"을 자연스럽게 처리한다.
    """

    def __init__(self):
        super().__init__('capstone_pick')
        # 속도 배율 (조절: --ros-args -p speed_scale:=1.0 ~ 5.0)
        self.scale = float(self.declare_parameter('speed_scale', 3.0).value)
        self.skip_approach = bool(self.declare_parameter('skip_approach', False).value)
        target_color = str(self.declare_parameter('target_color', 'blue').value).strip().lower()
        self.target_color = target_color   # 디버그용 실좌표 대조에서 쓴다
        if target_color not in TARGET_COLOR_RANGES:
            self.get_logger().error(
                f'미지원 색 "{target_color}" — 사용 가능: {sorted(TARGET_COLOR_RANGES)}')
            raise SystemExit(1)
        self.hsv_ranges = TARGET_COLOR_RANGES[target_color]
        # 바닥 모드: 기본은 비전 검출 높이로 자동 결정, skip_approach 시엔 -p floor:=true로 강제
        self.floor_mode = bool(self.declare_parameter('floor', False).value)
        # 하강 후 죠 안쪽으로 큐브를 넣는 전진량(m). 상공-파지 자세의 죠 x 후퇴량이
        # 근거 — 새 파지 자세(1.15/0.28)에서 실측 25.0mm로 이 값과 정확히 일치한다.
        # 값 자체는 구자세 스윕에서 정했다 — 0/25/43/60mm 중 25·43만 큐브가 실제로
        # 옮겨졌고(0은 제자리, 60은 밀어내 실패), 들기 중 각도 변화가 25mm에서 최소(+0.030).
        self.creep = float(self.declare_parameter('creep', 0.025).value)
        # 놓을 곳: side(옆 바닥) | trash(쓰레기통 투입)
        self.place_target = str(self.declare_parameter('place_target', 'side').value).strip().lower()
        # 단계별 추적 기록. 로그를 더 많이 찍는 것이 아니라, 단계마다 '로봇의 믿음'과
        # '실제(시뮬 좌표)'를 한 줄로 나란히 남긴다 — 오늘 가짜 성공(로그는 전 구간
        # HOLDING인데 큐브는 바닥 그대로)을 잡은 것이 실좌표였고, 관절값·각도·면적을
        # 아무리 자세히 찍어도 그건 못 잡는다. 시행끼리 표로 비교하려고 JSONL로 쓴다.
        self.use_ik = bool(self.declare_parameter('use_ik', True).value)
        self.trace_path = str(self.declare_parameter(
            'trace', os.path.expanduser('~/capstone_tools/logs/trace.jsonl')).value)
        self._t0 = time.time()
        self._trace_target = f'pick_object_{target_color}'
        # 검출기: yolo(기본, 시뮬 자동라벨 파인튜닝 모델) / hsv(폴백·orange·pink용)
        self.detector = str(self.declare_parameter('detector', 'yolo').value).strip().lower()
        self.yolo = None
        if self.detector == 'yolo':
            if target_color not in ('blue', 'red', 'green'):
                self.get_logger().error(
                    f'YOLO 모델은 blue/red/green만 학습됨 — "{target_color}"는 -p detector:=hsv로 실행')
                raise SystemExit(1)
            from ultralytics import YOLO as _YOLO   # 지연 임포트 (hsv 모드에선 불필요)
            model_path = os.path.expanduser('~/capstone_tools/yolo_cubes.pt')
            self.yolo = _YOLO(model_path)
            self.yolo_cls = {v: k for k, v in self.yolo.names.items()}[f'{target_color}_box']
        self.get_logger().info(
            f'speed_scale={self.scale} target_color={target_color} detector={self.detector}')
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.cam_info = None
        self.odom = None
        self.gripper_angle = None
        self.wrist_img = None
        self.create_subscription(Image, 'wrist_camera/image_raw', self._wrist_cb, 1)
        self.create_subscription(Image, 'rgbd_camera/image', self._color_cb, 1)
        self.create_subscription(Image, 'rgbd_camera/depth_image', self._depth_cb, 1)
        self.create_subscription(CameraInfo, 'rgbd_camera/camera_info', self._info_cb, 1)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(JointState, 'joint_states', self._joint_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       'arm_controller/follow_joint_trajectory')
        self.grip_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        # 모방학습 수집용: 그리퍼 목표(Goal_Position)를 토픽으로도 남긴다.
        # 액션 goal은 서버 내부에만 있어 외부 수집기가 관측할 수 없다.
        self._grip_cmd_pub = self.create_publisher(Float64, 'capstone/gripper_cmd', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ---- 콜백 ----
    def _wrist_cb(self, m):
        self.wrist_img = self.bridge.imgmsg_to_cv2(m, desired_encoding='bgr8')

    def _color_cb(self, m):
        self.color = self.bridge.imgmsg_to_cv2(m, desired_encoding='bgr8')

    def _depth_cb(self, m):
        self.depth = self.bridge.imgmsg_to_cv2(m)

    def _info_cb(self, m):
        self.cam_info = m

    def _odom_cb(self, m):
        p = m.pose.pose
        yaw = math.atan2(2 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1 - 2 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.odom = (p.position.x, p.position.y, yaw)

    def _joint_cb(self, m):
        if 'arm_gripper' in m.name:
            i = m.name.index('arm_gripper')
            self.gripper_angle = m.position[i]
            # 그리퍼 부하 = 파지 판정의 1차 신호. 실물 SO-101이 Feetech 서보의
            # present load로 판정하는 것과 같은 값이다(URDF의 effort state interface).
            self.gripper_effort = m.effort[i] if len(m.effort) > i else None
        # /joint_states에는 퍼블리셔가 둘이다 — joint_state_broadcaster(팔·그리퍼)와
        # ros_gz_bridge(바퀴 2개만). 종전처럼 통째로 덮어쓰면 바퀴 메시지가 도착할
        # 때마다 팔 관절이 빈 dict가 되고, 그 순간을 읽은 쪽은 "관절 상태 미수신"으로
        # 물러난다(실측: _reach_arm이 3회 연속 보정을 건너뛰어 얕은 걸침으로 직행).
        # 갱신분만 병합해 마지막으로 받은 팔 각도를 유지한다.
        upd = {n: p for n, p in zip(m.name, m.position) if n in ARM_JOINTS}
        if upd:
            self.joint_pos = {**(getattr(self, 'joint_pos', None) or {}), **upd}

    # ---- 단계별 추적 ----
    def _gz_pose(self, model):
        """시뮬의 실좌표 — 판정용 정답지. 제어에는 절대 쓰지 않는다.

        실물에는 없는 정보다. 여기서만(디버그 계층) 읽어 '로봇이 그렇게 믿었나'와
        '실제로 그랬나'를 대조한다.
        """
        try:
            out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                                 capture_output=True, text=True, timeout=8).stdout
        except Exception:
            return None
        L = out.splitlines()
        for i, l in enumerate(L):
            if '- Pose' in l and i + 1 < len(L):
                try:
                    v = [float(x) for x in L[i + 1].strip().strip('[]').split()]
                    return [round(x, 4) for x in v[:3]]
                except ValueError:
                    continue
        return None

    def _trace(self, stage, truth=True, **extra):
        """단계 경계에서 믿음과 실제를 한 줄로 남긴다 (JSONL).

        한 줄에 다 담는 이유는 시행끼리 표로 놓고 비교하기 위해서다. 로그를 길게
        찍으면 한 번의 실패는 읽을 수 있어도 5회를 나란히 놓고 '어느 단계에서
        갈렸나'를 볼 수 없다.
        """
        jp = getattr(self, 'joint_pos', None) or {}
        row = {
            't': round(time.time() - self._t0, 1),
            'stage': stage,
            # 로봇의 믿음
            'joints': {k: round(v, 3) for k, v in sorted(jp.items())},
            'grip_ang': None if self.gripper_angle is None else round(self.gripper_angle, 3),
            'grip_load': None if getattr(self, 'gripper_effort', None) is None
                         else round(self.gripper_effort, 3),
            'odom': None if self.odom is None else [round(v, 3) for v in self.odom],
            'jaw': None,
        }
        try:
            t = self.tf_buffer.lookup_transform(
                'base_footprint', 'arm_gripper_frame_link', rclpy.time.Time())
            p = t.transform.translation
            row['jaw'] = [round(p.x, 4), round(p.y, 4), round(p.z, 4)]
        except Exception:
            pass
        row.update(extra)
        if truth:      # 실제 (시뮬만 가능, 0.6초)
            row['cube_gz'] = self._gz_pose(self._trace_target)
            row['robot_gz'] = self._gz_pose('jdamr_cube')
        try:
            os.makedirs(os.path.dirname(self.trace_path), exist_ok=True)
            with open(self.trace_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(row, ensure_ascii=False) + '\n')
        except OSError as e:
            self.get_logger().warning(f'추적 기록 실패: {e}')

    def spin_until(self, pred, sec):
        """조건 pred()가 참이 될 때까지 최대 sec초 동안 메시지를 처리한다. 참이면 True.

        ROS2는 메시지가 저절로 도착하지 않는다. spin_once()를 불러야 그 순간
        대기 중인 콜백이 실행되어 self.color·self.odom 같은 값이 갱신된다.
        그래서 "카메라 프레임이 올 때까지 기다린다"는 sleep이 아니라 이 함수다.
        sleep으로 기다리면 시간만 가고 값은 영원히 None이다.

        쓰는 꼴이 대개 이렇다.
            self.color = None                                  # 옛 값을 지우고
            self.spin_until(lambda: self.color is not None, 3) # 새 프레임을 받는다
        먼저 None으로 지우는 게 중요하다 — 안 지우면 직전 프레임이 남아 있어
        조건이 즉시 참이 되고, 낡은 화면으로 판단하게 된다.
        """
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pred():
                return True
        return False

    # ---- 1) Perception ----
    def locate_object(self, timeout=15.0):
        """물체의 base_footprint 3D 좌표를 카메라 계산으로 구한다. 실패 시 None."""
        self.color = self.depth = None
        if not self.spin_until(
                lambda: self.color is not None and self.depth is not None
                and self.cam_info is not None, timeout):
            self.get_logger().error('카메라 토픽 수신 실패')
            return None
        if getattr(self, 'yolo', None) is not None:
            # YOLO 검출 경로: 후보 수집 후 최근접 선택 + 타깃 락 (HSV 경로와 동일 정책)
            cands = self._yolo_candidates(self.depth)
            if not cands:
                return None
            best = min(cands, key=lambda c: c[2])
            prev = getattr(self, '_target_px', None)
            if prev is not None:
                near = [c for c in cands
                        if math.hypot(c[0] - prev[0], c[1] - prev[1]) < 120]
                if near:
                    best = min(near, key=lambda c: math.hypot(c[0] - prev[0], c[1] - prev[1]))
            self._target_px = (best[0], best[1])
            return self._pixel_to_base(*best)
        hsv = cv2.cvtColor(self.color, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.hsv_ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        depth_img = np.asarray(self.depth)
        # 연결 성분별로 뎁스 게이팅: 자기 몸(근접)·무효 뎁스 성분을 걸러내고
        # 남은 후보 중 가장 가까운 것을 대상으로 삼는다.
        num, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
        best = None
        for i in range(1, num):
            if stats[i, cv2.CC_STAT_AREA] < MIN_COMPONENT_AREA:
                continue
            comp = labels == i
            ds = depth_img[comp]
            ds = ds[np.isfinite(ds) & (ds > 0.05)]
            if len(ds) < 15:
                continue
            d_med = float(np.median(ds))
            if d_med < MIN_OBJECT_DEPTH:
                continue
            if best is None or d_med < best[2]:
                best = (cents[i][0], cents[i][1], d_med)
        if best is None:
            return None
        # 타깃 고정: 직전 타깃과 가까운 후보 우선 (프레임 간 타깃 전환 방지)
        prev = getattr(self, '_target_px', None)
        if prev is not None:
            cands = []
            for i in range(1, num):
                if stats[i, cv2.CC_STAT_AREA] < MIN_COMPONENT_AREA:
                    continue
                comp = labels == i
                ds = depth_img[comp]
                ds = ds[np.isfinite(ds) & (ds > 0.05)]
                if len(ds) < 15 or float(np.median(ds)) < MIN_OBJECT_DEPTH:
                    continue
                cands.append((cents[i][0], cents[i][1], float(np.median(ds))))
            near = [c for c in cands
                    if math.hypot(c[0] - prev[0], c[1] - prev[1]) < 120]
            if near:
                best = min(near, key=lambda c: math.hypot(c[0] - prev[0], c[1] - prev[1]))
        self._target_px = (best[0], best[1])
        # 검출 근거 로그: 선택 성분의 실제 픽셀 색 — 어떤 색이 검출을 만들었는지 추적 가능하게
        li = int(labels[int(best[1]), int(best[0])])
        if li > 0:
            hm = hsv[labels == li].mean(axis=0)
            self.get_logger().info(
                f'검출 근거: HSV평균=({hm[0]:.0f},{hm[1]:.0f},{hm[2]:.0f}) '
                f'면적={int(stats[li, cv2.CC_STAT_AREA])}px')
        return self._pixel_to_base(*best)

    def _pixel_to_base(self, u, v, d, z_gate=None):
        if z_gate is None:
            # 복구 중에는 큐브가 반드시 바닥에 있다 — 게이트를 바닥 전용으로 조여
            # 통 몸통(z≈0.14, blue_box conf 0.4~0.75로 오검출 실측)·받침대 높이의
            # 유령 표적을 걸러낸다. 평상시엔 받침대(0.13) 파지를 위해 0.20까지 연다.
            z_gate = (-0.05, 0.06) if getattr(self, '_floor_only', False) else (-0.05, 0.20)
        """픽셀+뎁스 → base_footprint 3D (역투영 + 축변환 + TF + 높이 게이트).

        z_gate는 대상별로 다르다 — 바닥 물체는 기본값, 쓰레기통처럼 높은 구조물은 넓혀 준다.
        """
        k = self.cam_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        # 광학 프레임: X=우, Y=하, Z=전방
        ox = (u - cx) * d / fx
        oy = (v - cy) * d / fy
        oz = d
        # 광학 → 링크 프레임(x=전방, y=좌, z=상) 축 변환 — 기존 리포의 좌표 버그 수정 지점
        p = PointStamped()
        p.header.frame_id = CAMERA_FRAME
        p.point.x, p.point.y, p.point.z = oz, -ox, -oy
        if not self.spin_until(
                lambda: self.tf_buffer.can_transform('base_footprint', CAMERA_FRAME,
                                                     rclpy.time.Time()), 5.0):
            self.get_logger().error('TF 대기 실패')
            return None
        tr = self.tf_buffer.lookup_transform('base_footprint', CAMERA_FRAME, rclpy.time.Time())
        out = do_transform_point(p, tr)
        # 물리적 타당성 검증: 대상이 있을 수 있는 높이 범위 안인지
        if not (z_gate[0] < out.point.z < z_gate[1]):
            self.get_logger().warning(
                f'높이 검증 실패 z={out.point.z:.3f} — 오검출로 판단, 무시')
            return None
        return out.point.x, out.point.y, out.point.z, (u, v, d)

    def _yolo_candidates(self, depth_img):
        """YOLO 추론 → 대상 클래스 박스들을 (중심u, 중심v, 뎁스중앙값) 후보로."""
        r = self.yolo.predict(self.color, conf=0.40, verbose=False)[0]
        cands = []
        for b in r.boxes:
            if int(b.cls) != self.yolo_cls:
                continue
            x1, y1, x2, y2 = (max(0, int(t)) for t in b.xyxy[0].tolist())
            region = np.asarray(depth_img)[y1:y2 + 1, x1:x2 + 1]
            ds = region[np.isfinite(region) & (region > 0.05)]
            if len(ds) < 10:
                continue
            d_med = float(np.median(ds))
            if d_med < MIN_OBJECT_DEPTH:
                continue
            self.get_logger().info(
                f'검출 근거(YOLO): {self.yolo.names[int(b.cls)]} conf={float(b.conf):.2f} d={d_med:.2f}')
            cands.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0, d_med))
        return cands

    # ---- 팔/그리퍼 프리미티브 ----
    def move_arm(self, targets, duration):
        """팔 관절을 targets 각도로 duration초에 걸쳐 옮기고, 실제 도달까지 기다린다.

        targets는 일부만 줘도 된다 — 나머지는 직전 자세(_last_arm)를 유지한다.
        `{'arm_shoulder_pan': 0.5}`처럼 한 관절만 돌리는 호출이 많은 이유다.

        팔은 토픽이 아니라 **액션(action)**으로 움직인다. 토픽은 쏘고 잊는 단방향이라
        "다 갔는지"를 알 수 없는데, 팔은 다음 동작이 앞 동작의 완료를 전제하므로
        진행 상황과 완료를 돌려주는 액션이 맞다. 흐름은 이렇다.
          goal 전송 → 수락 여부 확인 → 결과 대기

        여기서 한 가지 함정이 있다. **액션 완료가 도달을 뜻하지 않는다.**
        컨트롤러는 궤적 시간이 끝나면 관절이 못 따라왔어도 종료를 알린다. 그 상태에서
        다음 명령이 겹치면 급가속이 생겨 쥔 물체가 빠진다. 그래서 마지막에
        wait_arm_settled()로 관절이 실제로 목표에 닿을 때까지 한 번 더 기다린다.

        duration은 speed_scale로 나뉜다 — 배율을 올리면 같은 동작이 짧은 시간에
        일어나고, 하한 0.8초 아래로는 내려가지 않는다.
        """
        duration = max(0.8, duration / self.scale)
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        base = {j: 0.0 for j in ARM_JOINTS}
        cur = getattr(self, '_last_arm', base)
        merged = {**cur, **targets}
        self._last_arm = merged
        pt.positions = [merged[j] for j in ARM_JOINTS]
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration % 1) * 1e9)
        goal.trajectory.points = [pt]
        f = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if h is None or not h.accepted:
            return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        # 액션 완료 ≠ 도달. 궤적 시간이 끝나면 컨트롤러는 못 따라온 채로도 종료하는데,
        # 그 상태에서 다음 명령이 겹치면 급가속이 생겨 쥔 물체가 빠진다(실측).
        # 관절이 실제로 목표에 닿을 때까지 기다린다.
        return self.wait_arm_settled(merged)

    def wait_arm_settled(self, target, tol=0.04, timeout=6.0):
        """팔 관절이 목표 각도에 실제 도달할 때까지 대기.

        미달일 때 **어느 관절이 얼마나** 어긋났는지 함께 남긴다. 종전에는 최대
        오차 하나만 찍었는데, 하강이 0.265rad 어긋난 채 끝나도 그것이 어깨가
        바닥에 눌린 것인지 손목이 한계에 걸린 것인지 구분할 수 없었다 — 그
        구분 없이는 파지 실패의 원인을 짚을 수 없다.
        """
        t0 = time.time()
        worst, jp = None, None
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            jp = getattr(self, 'joint_pos', None)
            if not jp:
                continue
            worst = max(abs(jp.get(j, target[j]) - target[j]) for j in target)
            if worst < tol:
                return True
        detail = ''
        if jp:
            bad = sorted(((abs(jp.get(j, target[j]) - target[j]), j) for j in target),
                         reverse=True)
            detail = ' — ' + ', '.join(
                f'{j.replace("arm_", "")} 목표{target[j]:+.2f} 실제{jp.get(j, float("nan")):+.2f}'
                f'({e:+.3f})' for e, j in bad[:3] if e >= tol)
        self.get_logger().warning(
            f'자세 도달 미완 (최대 오차 {float("nan") if worst is None else worst:.3f}rad){detail}')
        return False

    def gripped(self, angle=None):
        """물림 판정 — 그리퍼 관절 부하로 본다. 각도는 보조.

        표준 방식이다. 죠에 '완전 닫힘'을 계속 명령해 두면, 물체가 막고 있는 동안은
        위치 오차가 남아 부하가 실리고 물체가 빠지면 죠가 끝까지 닫혀 부하가 사라진다.
        실물 SO-101이 Feetech 서보의 present load로 파지를 판정하는 것과 같은 신호다.

        실측(2026-08-07, tools/grip_signal_probe.py + effort_drop.py):
            죠 열림          각도 +1.200  effort  +0.003
            큐브 물림        각도 +0.406  effort -10.000
            운반 중 유지     각도 +0.291  effort -10.000
            큐브 이탈        각도 -0.170  effort  -0.001
        1만 배 갈리고, 낙하 시점에 곧바로 죽는다.

        각도를 1차로 쓸 수 없는 이유가 둘 있다.
          · 조인 시간에 따라 변한다 — 같은 큐브·같은 자리인데 1.5초 뒤 0.42,
            10초 뒤 0.26이었다. 시간에 의존하는 값을 임계로 쓸 수 없다.
          · 접촉각을 유지 목표로 명령하면 그때부터 각도는 자기가 명령한 값이다.
            그 방식에서는 큐브를 빼앗아도 각도·부하가 그대로였다(가짜 성공의 원인).
        """
        eff = getattr(self, 'gripper_effort', None)
        if angle is None:
            angle = getattr(self, 'gripper_angle', None)
        # 두 신호를 함께 본다. 부하만 보면 "뭔가 끼었다"까지만 알고 "제대로 물었나"는
        # 모른다 — 큐브가 43도 돌아간 채 대각선(5.7cm)으로 걸리면 부하는 -10.0인데
        # 물림각이 0.78이고 들다가 빠진다(실측). 각도가 물체 두께를 말해 주므로,
        # 4cm 큐브의 정상 물림 구간(0.32~0.55)을 벗어나면 잘못 문 것이다.
        width_ok = angle is None or GRIP_HOLD_MIN < angle < GRIP_HOLD_MAX
        if eff is not None:
            return abs(eff) > GRIP_LOAD_MIN and width_ok
        # effort 미노출(구 URDF)일 때는 각도 구간만으로 판정한다
        return angle is not None and width_ok

    def move_gripper(self, position, wait=True, effort=10.0):
        """그리퍼를 position 각도로 움직인다. 큰 값이 열림, 작은 값이 닫힘.

        기준값은 파일 상단 상수에 있다 — 열림 0.5+, 완전 닫힘 -0.17.
        3cm 큐브를 물면 완전히 닫히지 못하고 0.05~0.30 구간에서 멈추는데,
        **이 '못 닫힌 정도'가 곧 물체 두께**라 파지 판정의 1차 신호가 된다(gripped 참조).

        effort는 쥐는 힘이다. 닫을 때와 유지할 때 값이 다른데 이유가 있다.
        닫는 순간 힘을 세게 주면 강체 큐브를 튕겨내고, 반대로 이미 문 뒤에 약하면
        드는 하중에 죠가 밀려 벌어진다. 그래서 닫기는 10, 유지는 30으로 나눠 쓴다.

        wait=False는 결과를 기다리지 않고 보내기만 한다. 들어올리는 중 단계마다
        쥔 힘을 다시 주장할 때 쓰는데, 매번 액션 왕복을 기다리면 전체 실행이
        시간 예산을 넘겨 중단됐다(실측). 유지 명령은 도착만 하면 된다.
        """
        self._last_grip_cmd = float(position)
        # 모방학습 수집용: 그리퍼 목표를 토픽으로도 남긴다. 액션 goal은 서버
        # 내부에만 있어 외부 수집기가 볼 수 없다 — 실물에서 리더암 명령을
        # 기록하는 것과 같은 역할이고, LeRobot의 action(Goal_Position)에 해당한다.
        self._grip_cmd_pub.publish(Float64(data=float(position)))
        if not self.grip_client.wait_for_server(timeout_sec=10.0):
            return False
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = effort  # 파지력. 파고듦 방지는 목표각 캡이 담당
        f = self.grip_client.send_goal_async(g)
        if not wait:
            # 유지 명령은 결과를 기다릴 필요가 없다. 다만 전송 자체는 spin에서
            # 처리되므로 수락까지만 짧게 돌린다(파라미터가 선언만 되고 무시되던 버그).
            rclpy.spin_until_future_complete(self, f, timeout_sec=0.3)
            return True
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if h is None or not h.accepted:
            return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=8.0)
        return True

    def drive(self, vx, wz, sec):
        """전진속도 vx[m/s]·회전속도 wz[rad/s]로 sec초만큼 주행하고 정지한다.

        바퀴 로봇의 명령은 "어디로 가라"가 아니라 "얼마나 빠르게 움직여라"다.
        vx는 앞뒤, wz는 제자리 회전이고 둘을 동시에 주면 호를 그린다. 거리를 가려면
        속도 × 시간으로 환산해 그만큼 명령을 계속 보내다 멈춘다 — 이 함수가 그 일이다.

        보내는 값은 Twist 메시지이고 cmd_vel 토픽으로 나간다. 중요한 성질이 하나
        있는데, **명령을 멈추면 로봇이 서는 게 아니라 마지막 속도를 유지한다.**
        그래서 마지막 줄에서 빈 Twist(속도 0)를 반드시 보낸다.

        아래 주석들이 길어진 이유는 "명령한 만큼 실제로 움직이지 않기" 때문이다.
        정지 마찰, 가감속 손실, 속도 상하한이 전부 이동량을 갉아먹는다. 그래서 이
        함수는 속도를 고정하지 않고, **이동량(vx×sec)을 불변으로 두고 속도와 시간을
        서로 조정한다.** 아래 주석의 실패 사례 둘이 이 설계의 근거다.
        """
        # 기준(1배) 이동량(vx*sec, wz*sec)을 불변으로 유지하며 speed_scale 적용.
        # 속도 상한(0.35, 1.5)에 걸리면 시간을 늘려 보상 — 고배속에서 회전량이 깎여
        # "탐색 로그만 찍히고 실제로는 안 도는" 문제의 근본 수정.
        # 이동량 보존 + 속도 하한. 두 가지 실패를 모두 피해야 한다:
        #   ① 고배속에서 펄스가 짧으면 가속하다 끝나 이동량이 손실된다(명령 50mm에 실제 7mm)
        #   ② 최소 펄스를 길게 두면 미세 조정이 초속 8mm의 너무 느린 명령이 되어
        #      정지 마찰을 못 이기고 아예 안 움직인다(정렬 실패 → 파지가 허공을 집음)
        # 그래서 시간을 고정하지 않고, 속도를 [하한, 상한] 안에 두고 시간으로 이동량을 맞춘다.
        a0 = self.odom[2] if self.odom else None   # 회전 미달 보정용 시작 각
        s = min(self.scale, 4.0)          # 주행 배율 상한 (팔 배율은 제한하지 않는다)
        t = Twist()
        dur = sec / s
        if vx:
            v = min(0.35, max(0.06, abs(vx) * s))     # 하한 0.06m/s: 정지 마찰 극복
            dur = abs(vx) * sec / v
            t.linear.x = v if vx > 0 else -v
        if wz:
            w = min(1.5, max(0.25, abs(wz) * s))      # 하한 0.25rad/s
            dur = max(dur, abs(wz) * sec / w)
            t.angular.z = w if wz > 0 else -w
        dur *= 1.2                        # 가감속 손실 보상
        tv = vx * sec / dur if vx else 0.0   # 늘어난 시간에 맞춰 속도 재계산 (이동량 불변)
        tw = wz * sec / dur if wz else 0.0
        # 위에서 하한(0.06 / 0.25)을 걸어놨는데 이동량 보존으로 다시 그 아래로
        # 떨어지면 정지 마찰을 못 이겨 통째로 무변위가 된다. 실측: approach가
        # 0.125rad/s로 회전 명령을 내 방위각 -21.8도가 13회 연속 고정이었다
        # (같은 명령을 0.209rad/s로 주면 97% 달성). 하한을 지키고 시간을 줄여
        # 이동량을 맞춘다 — 속도×시간이 불변이므로 결과는 같다.
        shrink = 1.0
        if vx and abs(tv) < 0.06:
            shrink = max(shrink, 0.06 / abs(tv))
        if wz and abs(tw) < 0.25:
            shrink = max(shrink, 0.25 / abs(tw))
        if shrink > 1.0:
            dur /= shrink
            tv *= shrink
            tw *= shrink
        t.linear.x, t.angular.z = tv, tw
        # 운반 중에는 사다리꼴 속도 프로파일로 저크를 없앤다. 즉발 시작·정지의 회전
        # 관성이 물림을 흐트러뜨린다 — 실측: 회전 누적 74도에서 큐브가 죠 안에서
        # 미끄러져(실좌표 z=0.21로 쥔 상태 유지) 손목캠 면적이 51724→8103으로 급감,
        # 낙하 오판으로 운반이 중단됐다. 이동량 보존을 위해 램프 시간만큼 연장한다
        # (사다리꼴 면적 = v×(dur−ramp) → dur+ramp로 원래 이동량 유지). 파지 단계의
        # creep 캘리브레이션은 운반이 아니므로 영향 없다.
        ramp = min(0.3, dur / 3) if getattr(self, '_carrying', False) else 0.0
        # 램프 시작을 0이 아니라 40%에서: 0-시작은 정지 마찰을 못 깨 회전 펄스가
        # 통째로 무변위가 된다(실측: brg 36.4도가 5펄스 연속 고정 — drive 주석의
        # 실패 모드 ②와 동일). 이동량 보존 연장분도 손실 면적(ramp×(1−e0))에 맞춘다.
        E0 = 0.4
        dur += ramp * (1.0 - E0)
        t0 = time.time()
        while time.time() - t0 < dur:
            e = 1.0
            if ramp:
                el = time.time() - t0
                e = max(0.0, min(1.0, el / ramp, (dur - el) / ramp))
                e = E0 + (1.0 - E0) * e
            out = Twist()
            out.linear.x = t.linear.x * e
            out.angular.z = t.angular.z * e
            self.cmd_pub.publish(out)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        # 회전은 명령만으로 재현되지 않는다. 마찰을 어떻게 튜닝해도 같은 명령이
        # 71%/30%로 갈린다(실측, mu2=0.1 2회 반복). 15kg 차체의 정지 마찰이
        # 비선형이기 때문이다. 접근 정렬은 회전 정확도에 직결되므로 — 미달분을
        # 그대로 두면 approach가 같은 방위각을 수십 회 반복하며 제자리가 된다 —
        # 실제로 돈 각을 재서 부족분을 채운다. 직진(drive_dist)과 같은 방식이다.
        if wz and a0 is not None and abs(wz * sec) > 0.03:
            self._turn_fill(wz * sec, a0)

    def _turn_fill(self, goal, a0, tol=0.03, max_iter=4):
        """drive()가 목표만큼 못 돌았으면 odom을 보며 남은 각을 채운다."""
        sign = 1.0 if goal > 0 else -1.0
        target = abs(goal)

        def turned():
            if self.odom is None:
                return target          # 못 재면 더 돌지 않는다(폭주 방지)
            return abs(math.atan2(math.sin(self.odom[2] - a0),
                                  math.cos(self.odom[2] - a0)))

        for _ in range(max_iter):
            rest = target - turned()
            if rest < tol:
                break
            deadline = time.time() + min(1.5, rest / 0.3 + 0.3)
            while time.time() < deadline:
                t = Twist()
                t.angular.z = sign * 0.3
                self.cmd_pub.publish(t)
                rclpy.spin_once(self, timeout_sec=0.02)
                if turned() >= target - tol * 0.5:
                    break
            self.cmd_pub.publish(Twist())
            time.sleep(0.25)
        got = turned()
        if abs(got - target) > tol:
            self.get_logger().info(
                f'회전 보정: 목표 {math.degrees(goal):+.0f}° → 실제 {math.degrees(got * sign):+.0f}°')

    # ---- 2) Approach: 비전 재인식 폐루프 (odom 기반 스톨 감지 포함) ----
    def approach(self, max_iter=40):
        """물체가 '파지 포켓'에 오도록 주행한다. 성공 시 팔의 pan 각도, 실패 시 None.

        파지 포켓은 팔이 손을 뻗었을 때 죠가 닫히는 고정된 한 점이다(POCKET_FLOOR).
        팔의 자세는 이미 정해져 있으므로, 팔을 물체에 맞추는 게 아니라 **로봇을 움직여
        물체를 그 점에 갖다 놓는다.** 역기구학을 풀지 않아도 되는 대신 주행이 정확해야 한다.

        한 반복이 하는 일:
          ① 지금 물체가 어디 있나 다시 본다        locate_object()
          ② 포켓까지 얼마나 남았나 계산            er(전후 거리), brg(좌우 각도)
          ③ 둘 다 충분히 작으면 끝                 반환값 = 팔이 틀어야 할 pan 각도
          ④ 아니면 조금 움직이고 ①로               drive()

        매번 다시 보는 것이 핵심이다(폐루프). 한 번 재고 그만큼 가면 바퀴 미끄러짐과
        추정 오차가 그대로 남지만, 반복하면 매 회차가 남은 오차를 줄인다.

        물체를 놓쳤을 때의 대응이 세 갈래로 갈린다.
          - 기억이 있으면  → 추측항법으로 계속 접근 (오도메트리 기준)
          - 기억도 없으면  → 회전·전진 탐색
          - 팔이 시야를 가리면 → 팔을 물체 반대쪽으로 젖힌다

        반환하는 pan은 "물체가 정면에서 몇 rad 틀어져 있나"이고, grasp()가 그만큼
        어깨를 돌려 죠를 물체 쪽으로 맞춘다. 주행으로 다 못 없앤 좌우 오차를
        팔 관절로 마저 흡수하는 분담이다.
        """
        prev_odom = None
        stall_n = 0
        self._reacq_cnt = 0      # 이번 접근에서 '추측항법 수렴' 후 재관측을 시도한 횟수
        for it in range(max_iter):
            self.spin_until(lambda: self.odom is not None, 5.0)
            # 스톨 = 2회 연속 무이동일 때만 (짧은 회전은 관성으로 1회 무이동이 정상 — 오판 방지)
            if prev_odom is not None and self.odom is not None:
                moved = math.hypot(self.odom[0] - prev_odom[0], self.odom[1] - prev_odom[1]) \
                    + abs(self.odom[2] - prev_odom[2])
                stall_n = stall_n + 1 if moved < 0.005 else 0
                if stall_n >= 2:
                    self.get_logger().warning(f'[{it}] 스톨 감지(odom 변위 {moved * 1000:.1f}mm) — 후진 회복')
                    self.drive(-0.08, 0.3, 1.5)
                    stall_n = 0
            prev_odom = self.odom
            loc = self.locate_object()
            real = loc is not None
            if not real and getattr(self, '_obj_odom', None) is not None and self.odom:
                # 마지막 관측 위치 추적: odom 기준으로 기억한 물체 방향으로 계속 접근
                ox, oy = self._obj_odom
                x, y, yaw = self.odom
                dx, dy = ox - x, oy - y
                xb = math.cos(yaw) * dx + math.sin(yaw) * dy
                yb = -math.sin(yaw) * dx + math.cos(yaw) * dy
                zb, dbg = 0.0, (0, 0, 0)
                if not getattr(self, '_arm_aside', False):
                    # 접힌 팔이 화면 하단 중앙(근접 물체 위치)을 가림 — 물체 반대쪽으로 젖혀 시야 확보.
                    # 방향은 순간 방위(진동 튐)가 아니라 추정 좌표의 좌우 부호로.
                    self._arm_aside = True
                    aside = 0.6 if yb > 0 else -0.6
                    self.get_logger().info(f'근접 시야 확보: 팔을 물체 반대쪽으로 (pan {aside})')
                    self.move_arm({'arm_shoulder_pan': aside}, 1.2)
                    prev_odom = None  # 의도적 정지 — 다음 반복 스톨 오판 방지
                    continue
                self.get_logger().info(f'[{it}] 추정 추적: base=({xb:.3f},{yb:.3f})')
            elif not real:
                # 카메라는 전방 ~0.9m 바닥만 본다(pitch 0.9rad) — 회전과 전진을 섞어 탐색
                self._miss = getattr(self, '_miss', 0) + 1
                if self._miss == 1:
                    self.get_logger().warning(f'[{it}] 물체 미검출 — 한 발 후진(근접 사각 확인)')
                    self.drive(-0.14, 0.0, 2.5)
                    continue
                if self._miss % 14 == 0:
                    # 한 바퀴(13회전×약27°) 스윕을 마친 뒤에만 전진 — 교대·중간전진은 스윕을 상쇄시킨다
                    self.get_logger().warning(f'[{it}] 물체 미검출 — 전진 탐색(0.25m)')
                    self.drive(0.10, 0.0, 2.5)
                else:
                    self.get_logger().warning(f'[{it}] 물체 미검출 — 회전 탐색(한 바퀴 스윕)')
                    self.drive(0.0, 0.4, 1.2)
                continue
            else:
                self._miss = 0
                self._last_seen = it
                xb, yb, zb, dbg = loc
                # IK 파지가 쓸 실측 좌표. 종전처럼 '포켓이라는 한 점'에 갖다 놓는
                # 대신, 큐브가 **어디 있든** 그 좌표를 IK에 넣어 팔이 흡수한다.
                self._cube_base = (xb, yb, max(0.0, zb))
                if zb > 0.001 and not getattr(self, '_mode_locked', False):
                    # 첫 실검출의 높이로 받침대/바닥 모드 결정 (이후 고정)
                    self.floor_mode = zb < FLOOR_Z_MAX
                    self._mode_locked = True
                    if self.floor_mode:
                        self.get_logger().info(f'바닥 모드 진입 (검출 z={zb:.3f})')
                if self.odom:
                    x, y, yaw = self.odom
                    self._obj_odom = (x + math.cos(yaw) * xb - math.sin(yaw) * yb,
                                      y + math.sin(yaw) * xb + math.cos(yaw) * yb)
            pocket = POCKET_FLOOR if self.floor_mode else POCKET_BASE
            r_target = math.hypot(pocket[0] - PAN_AXIS_X, pocket[1])
            r = math.hypot(xb - PAN_AXIS_X, yb)
            brg = math.atan2(yb, xb - PAN_AXIS_X)
            if real:
                self._last_brg = brg  # 팔 젖힘 방향 결정용 (물체가 좌/우 어느 쪽인지)
            # 뎁스=앞표면 → 물체 중심이 포켓에 오도록 앞표면은 반폭만큼 안쪽에
            er = r - (r_target - OBJECT_HALF_DEPTH)  # 물체 중심이 포켓에 오도록 (삽입 없음)
            self.get_logger().info(
                f'[{it}] 비전: base=({xb:.3f},{yb:.3f},{zb:.3f}) px=({dbg[0]:.0f},{dbg[1]:.0f}) '
                f'd={dbg[2]:.2f} | er={er * 1000:.0f}mm brg={math.degrees(brg):.1f}deg')
            # 바닥 모드는 ±30mm면 합격 — 잔여 오차는 손목캠 정렬(pan+미세주행)이 마무리.
            # 좁은 공차로 범퍼 코앞에서 전후 왕복하다 물체를 미는 것 방지.
            tol = 0.045 if self.floor_mode else 0.012   # IK가 흡수하므로 넓혀도 된다
            # 방위 임계는 **IK 좌우 범위에서 역산**한다. 종전 0.30rad(17도)은 큐브를
            # y=±0.11m까지 벌려 놓는데, 파지가 검증된 범위는 ±0.05m다(ik_grasp_probe).
            # 실측: 접근이 y=+0.087에 두고 끝나 IK 해는 나왔지만 허공을 물었다.
            brg_tol = 0.15 if self.floor_mode else 0.30
            if abs(er) < tol and abs(brg) < brg_tol and (real or it - getattr(self, "_last_seen", -99) <= 8):
                if not real:
                    # 접근 종료는 실측으로 내려야 한다. 추측항법 좌표로 끝내면 큐브가
                    # 포켓 밖에 남는다 — 실좌표 대조: 접근 종료 시 큐브까지 0.341 /
                    # 0.354 / 0.363m(전부 파지 실패), 0.394m(성공). 포켓은 0.385이고
                    # 파지 성립 구간은 그 ±20mm라, 34mm 어긋남이 곧 실패였다.
                    # 종전에는 재관측을 한 번만 시도하고, 그 재관측이 실패해도
                    # 플래그 때문에 다음 회차에 그냥 통과했다(로그의 '[6] 추정 추적'
                    # 직후 파지 진입이 그 경로다).
                    n_try = getattr(self, '_reacq_cnt', 0)
                    if n_try < 2:
                        self._reacq_cnt = n_try + 1
                        if n_try == 0:
                            # 주행 중 비전 상실 드리프트 보정 — 멈춰서 다시 본다
                            self.get_logger().info('수렴(추측항법) — 정지 재관측 시도')
                            time.sleep(1.0)
                        else:
                            # 포켓(0.385m)은 전방캠 근접 사각의 경계라 큐브가 화면
                            # 하단으로 잘린다(실측 px y=472/480). 물러나야 다시 보인다.
                            self.get_logger().info('수렴(추측항법) — 후진해 시야 확보 후 재관측')
                            self.drive(-0.10, 0.0, 2.0)
                        prev_odom = None  # 의도적 정지 — 다음 반복 스톨 오판 방지
                        continue
                    self.get_logger().warning(
                        '실측 없이 접근 종료 — 추측 좌표로 파지 진입(정확도 낮음)')
                self._anchor_odom = self.odom
                return -(brg - PAN_BASE_BEARING)
            # **회전과 전진을 섞지 않는다.** 섞으면 마지막 회전이 큐브를 시야 밖으로
            # 밀어내고, 그때부터 오도메트리 추측으로 넘어가 yaw 오차가 좌우로 쌓인다.
            #   실측: 실측 관측은 늘 y≈+0.078인데 추측은 y≈+0.018 — 60mm가 일관되게
            #   벌어졌고, 그 추측으로 파지해 허공을 물었다.
            # 방위를 먼저 끝내고 직진만 하면 전진 중에는 큐브가 화면 중앙 쪽에 남는다.
            # 종전에는 잔여 방위를 파지 단계의 손목캠 정렬이 흡수한다는 전제였는데,
            # IK로 바뀌면서 그 정렬 자체가 없어졌으므로 접근이 방위까지 책임져야 한다.
            # 세 구간으로 나눈다. 핵심은 **근접에서 회전하지 않는 것**이다.
            #   먼 구간   호를 그려 방위와 거리를 함께 좁힌다. 이 거리에서는 큐브가
            #             화면 중앙 쪽이라 돌아도 시야를 벗어나지 않는다.
            #   정렬 구간 아직 큐브가 보이는 거리에서 방위만 맞춘다.
            #   근접 구간 직진만. 여기서 돌면 큐브가 시야 밖으로 나가고, 그때부터
            #             오도메트리 추측이라 좌우가 수십 mm 어긋난다(실측 38mm).
            # 회전·전진을 완전히 분리했더니 직진할수록 방위가 다시 벌어져(같은 좌우
            # 오차가 가까울수록 큰 각이 된다) **마지막에 회전이 남았다.** 그 회전이
            # 근접에서 일어나 큐브를 놓쳤다. 멀리서 미리 좁혀야 그게 없어진다.
            if er > 0.15:
                v = 0.08
                wz = max(-0.30, min(0.30, brg * 0.9)) if real else 0.0
                self.drive(v, wz, min(3.0, abs(er) / v + 0.2))
            elif abs(brg) > brg_tol:
                # 맹회전은 저속으로 (고속 회전 슬립이 yaw 추정을 무너뜨려 지그재그 발진)
                wz = 0.25 if real else 0.10
                # 게인 0.5: 한 번에 목표각을 다 돌면 관성으로 넘어가 반대편으로
                # 같은 만큼 벗어난다 — 실측: brg가 +26.9°↔-26.7°를 정확히 왕복하며
                # 수렴하지 않았다. 절반씩 접근하면 오버슈트 없이 수렴한다.
                self.drive(0.0, wz if brg > 0 else -wz, min(2.0, 0.5 * abs(brg) / wz))
            else:
                # 근접: **직진만** 한다(wz=0). 관측을 유지하는 것이 목적이다.
                v = 0.04 if er > 0 else -0.04
                self.drive(v, 0.0, min(3.0, abs(er) / abs(v) + 0.2))
        return None

    # ---- 손목 카메라 최종 정렬 (바닥 모드): 좌우=pan, 전후=미세 주행 ----
    def drive_dist(self, dist, tol=0.006, max_iter=6):
        """목표 거리만큼 '실제로' 이동할 때까지 odom을 보며 반복한다.

        짧은 저속 펄스는 정지 마찰에 먹힌다. 실측(15kg 차체):

            0.03 m/s × 1.73s (52mm 명령) → 실제 0.2mm  (달성률 0%)
            0.03 m/s × 0.67s (20mm 명령) → 실제 0.0mm  (0%)
            0.15 m/s × 0.35s (52mm 명령) → 실제 5.1mm  (10%)
            0.15 m/s × 4.0s (600mm 명령) → 실제 175mm  (100%, odom 오차 1mm)

        즉 '얼마나 오래 명령했나'가 아니라 '움직이기 시작했나'가 관건이다.
        그래서 명령하고 끝내지 않고, 실제 이동량을 재서 남은 거리를 다시 민다.
        이 보정이 없으면 손목캠이 "52mm 더 가라"고 해도 차체가 제자리라
        팔만 무리하게 뻗다 바닥에 막히고 허공을 문다(실측: 파지 실패 3/3).

        반환값은 실제로 이동한 거리(부호 포함, m).
        """
        if abs(dist) < tol:
            return 0.0
        self.spin_until(lambda: self.odom is not None, 3.0)
        if self.odom is None:
            self.get_logger().warning('drive_dist: odom 없음 — 이동 생략')
            return 0.0
        x0, y0 = self.odom[0], self.odom[1]
        sign = 1.0 if dist > 0 else -1.0
        target = abs(dist)

        def moved():
            return math.hypot(self.odom[0] - x0, self.odom[1] - y0)

        for _ in range(max_iter):
            rest = target - moved()
            if rest < tol:
                break
            # 정지 마찰을 넘길 만큼의 속도로 밀되, 목표에 닿으면 즉시 멈춘다.
            v = sign * 0.12
            deadline = time.time() + min(2.0, rest / 0.12 + 0.5)
            while time.time() < deadline:
                m = Twist()
                m.linear.x = v
                self.cmd_pub.publish(m)
                self.spin_until(lambda: False, 0.05)
                if moved() >= target - tol * 0.5:
                    break
            self.cmd_pub.publish(Twist())
            self.spin_until(lambda: False, 0.3)
        got = moved()
        self.get_logger().info(f'차체 이동: 목표 {dist * 1000:+.0f}mm → 실제 {got * sign * 1000:+.0f}mm')
        return got * sign

    def wrist_align(self, pan, pre):
        """호버 자세에서 손목 RGB로 물체 중간이 파지선(WRIST_REF)에 오도록 정렬한다.

        순서가 중요하다: 기울기(roll)를 먼저 맞추고 그 자세에서 위치를 맞춘다.
        위치를 먼저 맞추고 roll을 돌리면 그리퍼가 회전하면서 파지 중심이 함께 이동해
        맞춰둔 위치가 어긋난다(물체 중점을 벗어나 얕게 물림).
        손목캠은 roll 관절 앞단에 있어 롤을 돌려도 측정 기준이 변하지 않으므로,
        roll을 먼저 적용해도 이후 위치 측정은 그대로 유효하다(실측 확인).

        좌우(pan)뿐 아니라 전후도 여기서 맞춘다. 접근 주행은 근접에서 비전을 잃고
        마지막 2~3스텝을 추측항법으로 끝내므로(로그: px=(0,0) d=0.00) odom 드리프트가
        그대로 남는다 — 접근이 er=+5mm라고 판정한 순간 손목캠은 +30mm를 봤다.
        가까이서 보는 손목캠이 옳으므로 전후 권한도 손목캠에 준다. 차체는 쓰지 않고
        (내려간 팔 앞에서 차체를 밀면 죠가 큐브를 쳐낸다) lift·elbow 조합으로 뻗는다.

        반환값의 reach는 이렇게 뻗은 전후량[m]이다. 호출부는 하강 자세에도 같은 양을
        반영해야 한다 — 상공에서만 뻗고 고정된 파지 자세로 내려가면 도로 제자리다.
        """
        roll = 0.0
        pose = dict(pre)
        reach = 0.0
        ang = self._wrist_cube_angle()
        if ang is not None and abs(ang) > 0.06:
            roll = max(-0.8, min(0.8, ROLL_SIGN * ang))
            self.get_logger().info(f'큐브 기울기 {math.degrees(ang):+.0f}도 → 롤 먼저 정렬 {roll:+.2f}rad')
            # 상공 측정은 절대 yaw다(실측: roll을 바꿔도 산포 0.1도). 그래서 한 번 재고
            # 그만큼 돌리면 끝이고, 재측정으로는 품질을 확인할 수 없다 — 값이 안 변하니까.
            # 대신 **도달**을 확인한다. roll이 명령을 못 따라간 사례가 있었고(목표 -0.68에
            # -0.08로 고착), 그 상태로 내려가면 대각선으로 문다. 하강 후에는 죠가 큐브를
            # 가려 각도를 다시 잴 수 없으므로 여기가 마지막 기회다.
            if not self.move_arm({**pose, 'arm_shoulder_pan': pan,
                                  'arm_wrist_roll': roll}, 1.5):
                got = (self.joint_pos or {}).get('arm_wrist_roll')
                self.get_logger().warning(
                    f'  롤 도달 미달 (목표 {roll:+.2f} 실제 '
                    f'{"?" if got is None else f"{got:+.2f}"}) — 한 번 더 명령')
                self.move_arm({'arm_wrist_roll': roll}, 1.2)
            time.sleep(0.4)

        # pan 게인은 방향·크기에 따라 568~2905 px/rad로 비선형이라(실측) 상수로는 어느
        # 구간에서든 어긋난다. 직전에 적용한 pan과 그때 실제로 움직인 픽셀로 게인을
        # 역산해 갱신하면 그 구간의 실제 감도에 맞춰 붙는다.
        gain = WRIST_PY_PER_PAN
        prev_y, prev_applied = None, 0.0
        for i in range(10):
            best = self._wrist_blob()
            if best is None:
                if i == 0:
                    # 첫 관측부터 없음 = 추측항법이 포켓 포착 범위 밖에 내렸다는 뜻.
                    # 맹목 하강은 허공 파지가 보장된다(실측: 복구 재파지 2회 연속
                    # -0.170) — 하강하지 말고 실패를 알려 재접근을 태운다.
                    self.last_grasp_fail = '손목캠 물체 미검출(하강 전)'
                    self.get_logger().warning('손목캠: 물체 미검출 — 맹목 하강 대신 재접근 요청')
                    return None
                self.get_logger().info('손목캠: 물체 미검출 — 정렬 생략')
                break
            if prev_y is not None and abs(prev_applied) > 0.004:
                measured = abs((best[1] - prev_y) / prev_applied)
                if 300.0 < measured < 6000.0:      # 이상치(측정 튐)는 버린다
                    gain = 0.5 * gain + 0.5 * measured
            dr = (best[0] - WRIST_REF[0]) / WRIST_PX_PER_M     # 전후 오차 [m] (+=멂)
            dpan = 0.9 * (best[1] - WRIST_REF[1]) / gain
            self.get_logger().info(
                f'손목캠 정렬[{i}]: blob=({best[0]:.0f},{best[1]:.0f}) '
                f'전후 {dr * 1000:+.0f}mm(팔 {reach * 1000:+.0f}), '
                f'pan 보정 {dpan:+.3f}rad (게인 {gain:.0f})')
            # 좌우 임계 0.004rad ≈ 포켓 반경 0.222m에서 0.9mm. 죠 중앙에 물리려면
            # 이 수준이어야 하고, 5프레임 중앙값을 쓰므로 이 임계까지 측정이 견딘다.
            # 전후 임계는 보정 데드존과 같은 값이어야 한다. 임계가 데드존보다 작으면
            # 그 사이 구간은 수렴 판정도 못 받고 보정도 안 되어 루프만 10회 소진한다
            # (실측: 전후 -12mm가 10회 내내 그대로였다).
            if abs(dr) <= REACH_DEADZONE and abs(dpan) < 0.004:
                self.get_logger().info(
                    f'정렬 수렴({i + 1}회): 전후 {dr * 1000:+.1f}mm, 좌우 {dpan:+.4f}rad, '
                    f'팔 리치 {reach * 1000:+.0f}mm')
                break
            prev_y = best[1]
            prev_applied = 0.0
            # 보정 데드존을 종료 임계와 맞춘다. 데드존(종전 0.012)이 임계(0.006)보다
            # 크면 그 사이 구간은 수렴 판정도 못 받고 보정도 안 되어, 중앙이 아닌 채로
            # 반복만 소진하고 닫으러 내려갔다.
            if abs(dpan) >= 0.004:
                applied = max(-0.25, min(0.25, dpan))
                new_pan = max(-0.6, min(0.6, pan + applied))
                prev_applied = new_pan - pan      # 관절 한계에 걸린 만큼은 빼고 기록
                pan = new_pan
                # roll을 유지한 채 pan만 조정 (roll을 빼면 정렬 기준이 다시 어긋난다)
                self.move_arm({**pose, 'arm_shoulder_pan': pan, 'arm_wrist_roll': roll}, 1.2)
                time.sleep(0.3)
            # 전후는 팔로 뻗어 맞춘다. 차체는 건드리지 않는다 — 팔이 물체 위에 있는
            # 상태에서 차체를 밀면 죠가 큐브를 쳐서 밀어낸다(실측: 아랫턱이 닿은 뒤에도
            # 계속 밀어 큐브가 굴러감).
            # 큐브가 포켓보다 가까우면(dr<0) 팔을 당기지 않는다 — 당기면 죠가 큐브
            # 앞으로 빠져 허공을 문다(실측: -22/-28/-45mm 세 번 모두 허공).
            # 그렇다고 재접근으로 돌리지도 않는다. 그건 한 번 시도하면 끝날 일을
            # 사이클 전체로 키우는 거래였다 — 접근 잔차 분산(±45mm)이 접근 허용오차
            # (±30mm)보다 커서 재접근해도 같은 자리에 서고, 실측에서 3회 연속 사이클을
            # 통째로 소진했다. 파지 실패는 부하 신호가 즉시·정확히 잡아내므로 싸다.
            # 그냥 내려가서 시도한다.
            if dr > REACH_DEADZONE:
                # 남은 양을 다 뻗지 않고 70%만: 팔이 앞으로 나가면 손목캠도 함께
                # 나가므로 측정과 제어가 같은 방향으로 얽힌다. 덜 뻗어 수렴시킨다.
                want = max(0.0, min(REACH_LIMIT, reach + 0.7 * dr))
                step = want - reach
                if abs(step) <= 0.001 and abs(reach) >= REACH_LIMIT - 1e-9:
                    # 팔을 끝까지 뻗었는데도 남았다. 재접근으로 돌리지 않고 이대로
                    # 내려가 시도한다(위 주석과 같은 이유). 남은 오차를 로그에 남겨
                    # 실패했을 때 원인을 되짚을 수 있게 한다.
                    self.get_logger().warning(
                        f'전후 오차 {dr * 1000:+.0f}mm — 팔 리치 포화'
                        f'({reach * 1000:+.0f}mm), 이대로 하강해 시도')
                    break
                if abs(step) > 0.001:
                    reach = want
                    pose['arm_shoulder_lift'] = pre['arm_shoulder_lift'] + HOVER_LIFT_PER_M * reach
                    pose['arm_elbow_flex'] = pre['arm_elbow_flex'] + HOVER_ELBOW_PER_M * reach
                    # 자세족 불변식(lift+elbow+wrist≈1.58)을 지켜 죠 평면을 수직으로 유지
                    pose['arm_wrist_flex'] = pre['arm_wrist_flex'] \
                        - (HOVER_LIFT_PER_M + HOVER_ELBOW_PER_M) * reach
                    self.move_arm({**pose, 'arm_shoulder_pan': pan, 'arm_wrist_roll': roll}, 1.2)
                    time.sleep(0.3)
        return pan, roll, reach

    def _reach_arm(self, s, sec=1.0):
        """내려간 자세에서 죠를 전후로 s[m]만큼 옮긴다 (높이 유지). 성공하면 True.

        lift 하나만 쓰면 전후 10mm에 높이가 10mm 넘게 딸려 온다 — 파지 높이는 바닥
        접촉 경계(1.17)에서 0.02 여유뿐이라 그걸로는 못 쓴다. elbow를 반대로 섞어
        높이 성분을 상쇄한 것이 아래 조합이다.

        누적 기준은 측정각(joint_pos)이 아니라 직전 명령(_last_arm)이다. 측정 기준으로
        더하면 추종오차(wait_arm_settled 허용 0.04rad = 리치 15mm)와 중력 처짐이 호출
        때마다 목표에 눌러앉아 래칫이 된다.
        """
        cur = dict(getattr(self, '_last_arm', None) or self.joint_pos or {})
        if len(cur) < 5:
            self.get_logger().warning('팔 리치: 팔 자세 미확정 — 보정 생략')
            return False
        # REACH_* 계수는 파지 자세 한 점에서 잰 국소 선형화다. 멀리 뻗을수록
        # '높이 불변' 가정이 조용히 깨지므로 누적에 상한을 둔다.
        total = getattr(self, '_reach_applied', 0.0) + s
        if abs(total) > REACH_TOTAL_MAX:
            self.get_logger().warning(
                f'팔 리치 누적 {total * 1000:+.0f}mm — 선형화 유효범위 초과, 중단')
            return False
        ok = self.move_arm({
            'arm_shoulder_lift': cur['arm_shoulder_lift'] + REACH_LIFT_PER_M * s,
            'arm_elbow_flex': cur['arm_elbow_flex'] + REACH_ELBOW_PER_M * s,
            # 자세족 불변식을 지켜 죠 평면이 기울지 않게 한다
            'arm_wrist_flex': cur['arm_wrist_flex']
                              - (REACH_LIFT_PER_M + REACH_ELBOW_PER_M) * s,
        }, sec)
        self._reach_applied = total
        # 접지 가드: 리치는 높이를 유지하도록 짜였으므로 경계도 같은 양만큼 올라간다.
        lift_now = (self.joint_pos or {}).get('arm_shoulder_lift')
        gate = 1.17 + REACH_LIFT_PER_M * total
        if self.floor_mode and lift_now is not None and lift_now > gate:
            self.get_logger().warning(
                f'리치 후 lift={lift_now:.3f} > {gate:.3f} — 손끝 바닥 간섭 구간')
        time.sleep(0.3)
        return ok

    def _tcp(self):
        """죠(TCP)의 base_footprint 좌표. 못 읽으면 None."""
        try:
            t = self.tf_buffer.lookup_transform(
                'base_footprint', 'arm_gripper_frame_link', rclpy.time.Time())
            p = t.transform.translation
            return p.x, p.y, p.z
        except Exception:
            return None

    def _descend_vertical(self, hover, descended, steps=4, keep_x=None):
        """TCP의 x를 붙들고 z만 내린다 — 직교 직선 하강.

        관절 보간으로 내리면 죠가 26° 기울어진 직선을 그린다(TF 실측: 상공 x=0.3602
        → 파지 x=0.3352, 25mm 후퇴). 그 경로가 큐브를 앞뒤로 쓸고 지나가 아랫턱이
        큐브를 쳐낸다. 포켓 위치로 도착점을 맞춰도 **지나가는 경로**는 그대로라
        소용이 없다 — 표준 파지(MoveIt Grasps)가 접근을 직교 직선으로 하는 이유다.

        각 단계에서 TF로 실제 x를 읽어 되돌리므로 야코비안 계수 오차에 강하다.
        계수는 자세마다 달라지는데(상공 -221mm/rad, 파지 -56mm/rad) 폐루프라
        틀린 만큼 한 번 더 당기면 된다.
        """
        t0 = self._tcp()
        if t0 is None:
            self.get_logger().warning('하강: TCP 조회 실패 — 관절 보간으로 진행')
            self.move_arm(descended, 4.0)
            return
        # 유지할 x를 밖에서 정할 수 있다. 큐브 뒤에서 내릴 때는 '뒤로 물린 x'를
        # 붙들어야 한다 — 상공 x를 붙들면 뒤로 물리는 목표와 서로 당겨 어긋남이
        # 커진다(실측: 의도 40mm인데 최종 60mm).
        if keep_x is None:
            keep_x = t0[0]
        self.get_logger().info(f'수직 하강 시작: x={keep_x:.3f} 유지, z {t0[2]:+.3f} → 목표')
        for i in range(1, steps + 1):
            t = i / steps
            pose = {j: hover.get(j, 0.0) + (descended[j] - hover.get(j, 0.0)) * t
                    for j in ARM_JOINTS}
            self.move_arm(pose, 1.2)
            # 이 단계에서 벌어진 x 어긋남을 팔로 되돌린다 (차체는 쓰지 않는다)
            for _ in range(2):
                cur = self._tcp()
                if cur is None:
                    break
                dx = keep_x - cur[0]
                if abs(dx) < 0.004:
                    break
                self._reach_arm(max(-0.03, min(0.03, dx)), sec=0.8)
            cur = self._tcp()
            if cur:
                self.get_logger().info(
                    f'  하강[{i}/{steps}]: x={cur[0]:+.3f} (어긋남 {(cur[0] - keep_x) * 1000:+.0f}mm) '
                    f'z={cur[2]:+.3f}')

    def _ready_to_close(self, roll):
        """닫기 직전 합격 조건을 **한 번에** 검사한다. (통과여부, 사유목록, 새 roll)

        조건을 하나씩 따로 검사하고 따로 고치다 보니, 전후를 고치면 각도를 놓치고
        각도를 고치면 높이를 놓쳤다. 합격 기준을 한 곳에 모아 전부 통과할 때만 닫는다.

        기준 (docs/18-판정-규칙.md와 같은 값):
          전후   손목캠 blob x vs GRASP_REF   ±POCKET_HALF
          좌우   손목캠 blob y vs GRASP_REF   ±POCKET_HALF (같은 폭 — 미검증, 아래 참조)
          각도   큐브 yaw vs 현재 손목 roll   ±0.15rad (약 9도)
          높이   실측 lift가 명령 lift에 도달  ±0.03rad

        좌우 폭에 대해: 실패 시행들의 좌우가 -8~-18mm였고 성공은 -1mm라, 좌우를
        훨씬 좁게(6mm) 죄어 봤다가 되돌렸다. 이유가 셋이다.
          · 그 값들은 전부 하강이 리치만큼 밀린 자세에서 잰 것이다(위 하강 검사 주석).
            전제가 틀린 데이터로 임계를 정하면 잘못된 상수가 굳는다.
          · 6mm는 GRASP_PX_PER_M 기준 ±14px인데, blob y 산포는 그보다 훨씬 크다
            (`_wrist_blob` 주석의 실측 ±100px/단일프레임).
          · 좌우 오차를 m로 바꿀 때 **전후용** px/m를 쓰고 있다. 손목캠이 36° 기울어
            전후·좌우 스케일이 다르므로(파일 상단 실측 82 vs 67 px/cm) 이 환산부터 틀렸다.
        좌우를 죄려면 하강 자세에서 좌우 성립 폭과 px/m를 먼저 실측해야 한다.
        """
        bad = []
        best = self._wrist_blob(frames=3)
        if best is None:
            return False, ['큐브 미검출'], roll
        dr = (best[0] - GRASP_REF[0]) / GRASP_PX_PER_M
        dy = (best[1] - GRASP_REF[1]) / GRASP_PX_PER_M
        if abs(dr) > POCKET_HALF:
            bad.append(f'전후 {dr * 1000:+.0f}mm')
        if abs(dy) > POCKET_HALF:
            bad.append(f'좌우 {dy * 1000:+.0f}mm')
        # 각도는 **여기서 재지 않는다.** 하강 자세에서는 죠가 큐브를 감싸 마스크가
        # 죠 사이 좁은 틈만 잡고, 그 형상의 주축이 큐브 yaw를 더 이상 반영하지 않는다.
        #   실측(roll_convention_probe.py, 큐브 yaw 30° 고정):
        #     상공  roll 0/0.25/0.50/-0.25 → 23.3/23.2/23.2/23.2  (산포 0.1도, 안정)
        #     하강  같은 조건            →  7.7/ -0.0/ -0.0/ 21.3  (불규칙, 0에서 포화)
        # 이 값으로 판정하면 이미 맞춰 놓은 roll을 "어긋남"으로 읽고 0으로 되돌린다
        # (실측: 롤 +0.50 정렬 → 닫기 전 점검 "각도 -29도" → roll을 0으로 → 대각선
        # 물림 0.58). roll 정렬은 측정이 안정적인 상공에서 wrist_align이 끝낸다.
        d_ang = None
        if d_ang is not None and abs(d_ang) > 0.15:
            bad.append(f'각도 {math.degrees(d_ang):+.0f}도')
        # 높이: 하강이 실제로 끝났나
        want = (self._last_arm or {}).get('arm_shoulder_lift')
        got = (self.joint_pos or {}).get('arm_shoulder_lift')
        if want is not None and got is not None and abs(want - got) > 0.03:
            bad.append(f'하강 미달 {want - got:+.3f}rad')
        self.get_logger().info(
            f'닫기 전 점검: 전후 {dr * 1000:+.0f}mm 좌우 {dy * 1000:+.0f}mm '
            f'각도 {"—" if d_ang is None else f"{math.degrees(d_ang):+.0f}도"} '
            f'→ {"통과" if not bad else " / ".join(bad)}')
        # 각도만 어긋난 경우는 제자리에서 고칠 수 있다 — 손목만 돌리면 된다
        if d_ang is not None and abs(d_ang) > 0.15 and len(bad) == 1:
            new_roll = max(-0.8, min(0.8, roll + d_ang))
            self.get_logger().info(f'  각도 보정: roll {roll:+.2f} → {new_roll:+.2f}')
            # **도달을 확인한다.** roll이 명령을 못 따라간 사례가 실측으로 있다
            # (목표 -0.68에 -0.08로 30초 고착). 도달을 안 보고 통과시키면
            # 큐브를 대각선으로 물어 들다가 놓친다.
            if not self.move_arm({'arm_wrist_roll': new_roll}, 1.0):
                self.get_logger().warning('  각도 보정 미도달 — 닫지 않는다')
                return False, ['각도 보정 미도달'], roll
            time.sleep(0.3)
            return True, [], new_roll
        return (not bad), bad, roll

    def _seat_into_jaws(self, tries=2, tol=0.015):
        """하강한 자세에서 손목캠으로 남은 오차를 확인하고, 크면 조금만 민다.

        핵심은 "밀어서 완벽하게 맞추지 않는다"이다. 큐브는 고정돼 있지 않아
        밀면 도망간다 — 아랫턱이 닿은 뒤에도 계속 밀면 큐브가 앞으로 굴러가
        파지 위치가 오히려 무너진다(실측). tol 이내면 그대로 잡는다. 죠에는
        그만한 여유가 있고, 닫힘이 큐브를 고정 죠 쪽으로 쓸어담아 물기 때문이다.

        tol이 15mm로 큰 이유: 포켓 스윕에서 파지 성립 구간이 40mm였다(0.365~0.405).
        여기서 mm를 다투는 것은 이득이 없고, 시도할수록 큐브를 밀 위험만 늘어난다.
        이 단계는 "크게 어긋났을 때만 되돌리는" 보험이다.

        손목캠을 36° 기울인 뒤로는 하강 상태에서도 큐브가 보이므로(실측 면적
        27k~37k) 오차를 눈으로 확인할 수 있다. 못 보면 종전 고정값으로 물러선다.
        """
        prev = None
        for i in range(tries):
            best = self._wrist_blob(frames=3)
            if best is None:
                if i == 0 and self.creep > 0.001:
                    # 첫 관측부터 못 보면 눈이 없는 것이다. 아무것도 안 하고 닫으면
                    # 하강 후퇴량(25mm)을 그대로 안은 채 죠 끝단으로 물어 미끄러진다.
                    # 종전의 고정 creep으로 물러서되, 차체가 아니라 팔로 뻗는다.
                    self.get_logger().info(
                        f'죠 안착: 큐브 미검출 — 고정 {self.creep * 1000:.0f}mm 폴백(팔)')
                    self._reach_arm(self.creep)
                else:
                    self.get_logger().info('죠 안착: 큐브 미검출 — 현 자세로 잡는다')
                return True      # 눈이 없으면 판단 보류 — 닫아 보고 부하로 가린다
            dr = (best[0] - GRASP_REF[0]) / GRASP_PX_PER_M      # +면 큐브가 멀다
            self.get_logger().info(
                f'죠 안착[{i}]: blob=({best[0]:.0f},{best[1]:.0f}) 전후 {dr * 1000:+.0f}mm')
            if abs(dr) < tol:
                self.get_logger().info(f'죠 안착 완료: 잔여 {dr * 1000:+.1f}mm')
                return True
            # 오차가 줄지 않으면 죠가 이미 큐브에 닿은 것이다. 더 뻗으면 큐브를
            # 밀어내므로 멈춘다 — 닿았으면 그게 안착 완료다.
            if prev is not None and abs(dr) > abs(prev) - 0.002:
                self.get_logger().info(
                    f'죠 접촉 감지: 오차 {prev * 1000:+.0f}→{dr * 1000:+.0f}mm — 그대로 잡는다')
                return True
            prev = dr
            # 팔만 움직인다. 차체는 건드리지 않는다(밀면 큐브가 도망간다).
            if not self._reach_arm(max(-0.02, min(0.02, dr))):   # 한 번에 20mm까지
                return False
        # 시도를 다 쓰고도 남았다 — 마지막 측정값으로 판단한다
        return prev is not None and abs(prev) < POCKET_HALF

    def _wrist_blob(self, frames=5):
        """손목캠에서 가장 큰 색 블롭의 중심을 여러 프레임의 중앙값으로 구한다.

        단일 프레임으로 재면 좌우 좌표가 ±100px 흔들린다(실측: 기준 320인데 215~413).
        그리퍼 손가락이 큐브를 부분적으로 가리고 마스크 경계가 프레임마다 달라지기
        때문이다. 그 노이즈는 pan 약 ±0.08rad에 해당해 정렬 루프가 수렴하지 못하고,
        마지막 반복에서 우연히 작은 값이 나오면 성공하고 크면 얕게 물었다.
        중앙값은 튀는 프레임을 버려 이 운을 없앤다.
        """
        xs, ys, areas = [], [], []
        for _ in range(frames):
            self.wrist_img = None
            if not self.spin_until(lambda: self.wrist_img is not None, 3.0):
                break
            hsv = cv2.cvtColor(self.wrist_img, cv2.COLOR_BGR2HSV)
            mask = np.zeros(hsv.shape[:2], np.uint8)
            for lo, hi in self.hsv_ranges:
                mask |= cv2.inRange(hsv, lo, hi)
            num, _, stats, cents = cv2.connectedComponentsWithStats(mask)
            best = None
            for j in range(1, num):
                a = stats[j, cv2.CC_STAT_AREA]
                if a > 100 and (best is None or a > best[2]):
                    best = (cents[j][0], cents[j][1], a)
            if best is not None:
                xs.append(best[0])
                ys.append(best[1])
                areas.append(best[2])
        # 과반이 안 잡히면 측정으로 쓰지 않는다. 하드코딩 3이면 frames=3 호출에서
        # '3장 전부 성공'을 요구하게 되어(허용 실패 0) 우연한 1프레임 누락이
        # 곧바로 '큐브 미검출 → 재접근'이 된다.
        if len(xs) < max(2, frames // 2 + 1):
            return None
        return (float(np.median(xs)), float(np.median(ys)), float(np.median(areas)))

    def _wrist_cube_angle(self):
        """손목캠 마스크의 최소면적사각형으로 큐브 yaw[rad] 추정 (90도 대칭 랩)."""
        self.wrist_img = None
        if not self.spin_until(lambda: self.wrist_img is not None, 3.0):
            return None
        hsv = cv2.cvtColor(self.wrist_img, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in self.hsv_ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        num, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        best = None
        for i in range(1, num):
            a = stats[i, cv2.CC_STAT_AREA]
            if a > 300 and (best is None or a > best[1]):
                best = (i, a)
        if best is None:
            return None
        pts = np.column_stack(np.where(labels == best[0])[::-1]).astype(np.float32)
        rect_ang = cv2.minAreaRect(pts)[2]
        delta = ((rect_ang - WRIST_RECT_REF + 45.0) % 90.0) - 45.0  # [-45,45)
        return -math.radians(delta)  # 실측: 이미지 각 = 큐브 yaw의 반전

    # ---- 3~5) Grasp / Verify / Lift ----
    def _cube_now(self):
        """큐브의 **현재** base_footprint 좌표.

        `_cube_base`는 관측 시점의 base 좌표라 로봇이 움직이면 무효가 된다 —
        접근하며 전진한 만큼 그대로 어긋난다(실측: 저장값으로 파지하니 큐브가
        y=+0.234에 있는 것으로 읽혔다). `_obj_odom`은 world 좌표라 고정이므로,
        그걸 지금 오도메트리로 되돌려 쓴다. approach()의 추측항법과 같은 변환이다.
        """
        o = getattr(self, '_obj_odom', None)
        base = getattr(self, '_cube_base', None)
        if o is None or not self.odom:
            return base
        ox, oy = o
        x, y, yaw = self.odom
        dx, dy = ox - x, oy - y
        cz = base[2] if base else 0.0125
        return (math.cos(yaw) * dx + math.sin(yaw) * dy,
                -math.sin(yaw) * dx + math.cos(yaw) * dy, cz)

    def _reobserve_cube(self):
        """파지 직전 큐브를 **다시 본다.** 실측이면 좌표를 갱신하고 True.

        오도메트리 추측은 좌우로 크게 어긋난다 — 실측 대조에서 전후는 3.9mm인데
        좌우가 38.5mm 틀렸다(로봇이 21.7도 돌아 있던 상태). 방위가 조금 어긋나면
        거리에 비례해 좌우로 벌어지는 yaw 누적 오차의 특징이다.

        IK로 작업공간이 넓어져(전후 0.29~0.41) 큐브가 **보이는 거리에서** 멈출 수
        있게 됐으므로, 추측 대신 한 번 더 보는 편이 정확하다. 못 보면 종전대로
        추측 좌표를 쓰되 그 사실을 남긴다.
        """
        # **팔을 먼저 확실히 치운다.** 접힌 팔이 카메라 앞을 막는다(저장 화면 확인).
        # 종전 `_arm_aside`는 pan 0.6rad(34도)인데 카메라 수평 반각이 33도라 경계에
        # 걸쳐 여전히 가렸다. 1.2rad(69도)면 확실히 프레임 밖이다.
        # pan 부호 규약은 approach의 팔 젖힘과 같다 — 물체가 왼쪽(y>0)이면 +pan.
        c0 = self._cube_now()
        aside = 1.2 if (c0 is None or c0[1] > 0) else -1.2
        self.move_arm({'arm_shoulder_pan': aside}, 1.5)
        time.sleep(0.4)
        loc = self.locate_object()
        if loc is None:
            # 왜 못 보는지는 화면을 봐야 안다 — 계산상 보여야 하는 거리인데
            # 안 보이면 가림·검출 임계 같은 다른 요인이다.
            path = ''
            if self.color is not None:
                try:
                    d = os.path.expanduser('~/capstone_tools/logs/reobs_miss')
                    os.makedirs(d, exist_ok=True)
                    path = os.path.join(d, f'{time.strftime("%H%M%S")}.png')
                    cv2.imwrite(path, self.color)
                    path = f' → {path}'
                except Exception:
                    path = ''
            self.get_logger().info(f'  파지 전 재관측: 큐브 미검출 — 추측 좌표 사용{path}')
            return False
        xb, yb, zb, _ = loc
        old = self._cube_now()
        self._cube_base = (xb, yb, max(0.0, zb))
        if self.odom:
            x, y, yaw = self.odom
            self._obj_odom = (x + math.cos(yaw) * xb - math.sin(yaw) * yb,
                              y + math.sin(yaw) * xb + math.cos(yaw) * yb)
        if old:
            self.get_logger().info(
                f'  파지 전 재관측: ({xb:.3f},{yb:+.3f}) '
                f'— 추측 대비 전후 {(xb - old[0]) * 1000:+.0f}mm 좌우 {(yb - old[1]) * 1000:+.0f}mm')
        return True

    def _gz_cube_base(self):
        """큐브의 실제 base 좌표 (gz 실좌표 → 현재 로봇 자세 기준). 디버그 전용.

        제어에는 쓰지 않는다 — 실물에서 못 쓰는 코드가 된다. 추측 좌표가 얼마나
        어긋나는지 숫자로 보려고 둔다.
        """
        try:
            def pose(name):
                out = subprocess.run(['gz', 'model', '-m', name, '-p'],
                                     capture_output=True, text=True, timeout=8).stdout
                L = out.splitlines()
                for i, l in enumerate(L):
                    if '- Pose' in l and i + 2 < len(L):
                        p_ = [float(v) for v in L[i + 1].strip().strip('[]').split()]
                        r_ = [float(v) for v in L[i + 2].strip().strip('[]').split()]
                        return p_, r_
                return None, None
            cp, _ = pose(f'pick_object_{self.target_color}')
            rp, rr = pose('jdamr_cube')
            if cp is None or rp is None:
                return None
            dx, dy = cp[0] - rp[0], cp[1] - rp[1]
            yaw = rr[2]
            return (math.cos(yaw) * dx + math.sin(yaw) * dy,
                    -math.sin(yaw) * dx + math.cos(yaw) * dy)
        except Exception:
            return None

    def _servo_correct(self, cx, cy, cz, tries=3):
        """프리그래스프 자세에서 손목캠으로 큐브 좌표를 보정한다. (cx, cy).

        접근이 넘겨준 좌표는 오도메트리 추측이라 좌우로 수십 mm 어긋난다. 그런데
        전방캠은 파지 거리에서 큐브를 볼 수 없으므로(팔이 가림) 여기서 손목캠으로 본다.

        종전 `wrist_align`과 다른 점은 **보정 대상**이다. 저쪽은 픽셀 기준점에 맞춰
        팔 관절을 직접 움직였고, 그 기준점이 파지 자세에 얽혀 하나만 어긋나면 전부
        무효였다. 여기서는 **큐브 좌표를 고치고 IK를 다시 푼다.** 오차를 목표에 더하고
        다시 보는 반복이라 감도 계수가 20% 어긋나도 두세 번이면 수렴한다.
        """
        for i in range(tries):
            q = K.grasp_q(cx, cy, cz, up=K.PREGRASP_UP)
            if q is None:
                return cx, cy
            self.move_arm(dict(zip(ARM_JOINTS, q)), 2.0 if i == 0 else 1.2)
            time.sleep(0.3)
            b = self._wrist_blob(frames=5)
            if b is None:
                self.get_logger().info(f'  서보[{i}]: 손목캠 미검출 — 보정 중단')
                return cx, cy
            dfwd = (b[0] - WRIST_SERVO_REF[0]) / WRIST_SERVO_FWD
            dlat = (b[1] - WRIST_SERVO_REF[1]) / WRIST_SERVO_LAT
            self.get_logger().info(
                f'  서보[{i}]: blob=({b[0]:.0f},{b[1]:.0f}) '
                f'→ 전후 {dfwd * 1000:+.0f}mm 좌우 {dlat * 1000:+.0f}mm')
            if abs(dfwd) < WRIST_SERVO_TOL and abs(dlat) < WRIST_SERVO_TOL:
                break
            # 전후·좌우는 **죠가 향한 방향** 기준이므로 base 좌표로 돌려서 더한다
            brg = math.atan2(cy, cx - K.PAN_X)
            cx += dfwd * math.cos(brg) - dlat * math.sin(brg)
            cy += dfwd * math.sin(brg) + dlat * math.cos(brg)
        return cx, cy

    def _grasp_ik(self, cube_yaw=0.0):
        """실측 큐브 좌표로 역기구학 파지. 물었으면 True.

        종전 `grasp()`와 무엇이 다른가. 저 함수는 팔을 **고정 자세**로만 움직이고
        큐브가 포켓이라는 한 점에 와 있기를 전제한다. 그래서 주행 잔차(±45mm)가
        파지 성립 구간(40mm)을 넘으면 구조적으로 실패했고, 부족분을 손목캠 픽셀
        기준점으로 메우다 상수 스무 개가 서로 무효화했다.

        여기서는 큐브가 **어디 있든** 그 좌표를 IK에 넣는다. 실측 성공 범위는
        전후 0.29~0.41m, 좌우 ±0.05m(`ik_grasp_probe.py`, 시도 가능 지점 10/11).

        경로는 두 구간이다 — 큐브 밖에서 수직으로 내린 뒤 같은 높이에서 수평으로
        밀어 넣는다. 파지 위치가 큐브 반폭보다 안쪽이라 한 번에 내리면 죠가
        큐브 윗면을 찌른다(`kinematics.descend_path` 주석 참조).
        """
        self._reobserve_cube()      # 추측 누적을 끊는다 (위 주석 참조)
        c = self._cube_now()
        if c is None:
            self.get_logger().warning('IK 파지: 큐브 실측 좌표가 없다 — 재접근 필요')
            self.last_grasp_fail = 'IK 파지 좌표 없음'
            return False
        cx, cy, _ = c
        cz = CUBE_CZ        # 비전 z를 믿지 않는다 (위 주석 참조)
        # 손목캠으로 좌표를 보정한다 — 접근이 넘긴 값은 추측이라 좌우가 어긋난다
        cx, cy = self._servo_correct(cx, cy, cz)
        # 파지가 **실측으로 검증된** 범위인지 먼저 본다. IK 해가 있어도 그 밖이면
        # 허공을 문다(실측: y=+0.087에서 해는 나왔으나 실패). 재접근이 더 싸다.
        if not (IK_X_MIN <= cx <= IK_X_MAX) or abs(cy) > IK_Y_MAX:
            self.get_logger().warning(
                f'IK 검증 범위 밖 (큐브 {cx:.3f},{cy:+.3f}) — 닫지 않고 재접근 '
                f'[전후 {IK_X_MIN}~{IK_X_MAX}, 좌우 ±{IK_Y_MAX}]')
            self.last_grasp_fail = 'IK 검증 범위 밖'
            return False
        path = K.descend_path(cx, cy, cz, cube_yaw)
        if path is None:
            self.get_logger().warning(
                f'IK 해 없음 (큐브 {cx:.3f},{cy:+.3f},{cz:.3f}) — 작업공간 밖, 재접근 필요')
            self.last_grasp_fail = 'IK 해 없음'
            return False
        gz = self._gz_cube_base()
        self.get_logger().info(
            f'IK 파지: 큐브 ({cx:.3f},{cy:+.3f},{cz:.3f}) 경로 {len(path)}점'
            + ('' if gz is None else
               f' | 실좌표 ({gz[0]:.3f},{gz[1]:+.3f}) '
               f'오차 전후{(cx - gz[0]) * 1000:+.0f}mm 좌우{(cy - gz[1]) * 1000:+.0f}mm'))
        self._ik_cube = (cx, cy, max(cz, 0.010))   # 들기가 쓴다
        self.last_grasp_angle = None
        self.last_grasp_fail = None
        self.move_gripper(1.2)
        self.move_arm(dict(zip(ARM_JOINTS, path[0])), 3.0)
        for q in path[1:]:
            self.move_arm(dict(zip(ARM_JOINTS, q)), 1.2)
        time.sleep(0.4)
        # **살살 닫는다.** 강체 큐브는 순간 힘이 크면 죠에 튕겨 자리를 벗어난다.
        # 두 단계로 나눠 먼저 약한 힘으로 접촉을 만들고, 닿은 뒤에 쥐는 힘을 올린다.
        # (종전 코드의 '닫기 10 · 유지 30' 분리와 같은 취지를 IK 경로에 옮긴 것)
        self.move_gripper(GRIPPER_CLOSED, effort=4.0)
        time.sleep(0.8)
        self.move_gripper(GRIPPER_CLOSED, wait=False, effort=30.0)
        time.sleep(1.2)
        ang = getattr(self, 'gripper_angle', None)
        eff = getattr(self, 'gripper_effort', None)
        self.last_grasp_angle = ang
        # 판정은 **부하**가 주다. 25mm 큐브의 물림각은 0.0~0.2로 완전 닫힘(-0.17)과
        # 가까워 각도만으로는 허공과 구분이 안 된다(40mm 기준의 0.32~0.55와 다른 대역).
        held = eff is not None and abs(eff) > GRIP_LOAD_MIN and (ang is None or ang > -0.15)
        self.get_logger().info(
            f'  닫힘 신호: 각도={ang if ang is None else round(ang, 3)} '
            f'부하={eff if eff is None else round(eff, 2)} → {"HOLDING" if held else "EMPTY"}')
        if not held:
            self.last_grasp_fail = f'물지 못함(각도 {ang}, 부하 {eff})'
        self._ik_grasped = held
        return held

    def _lift_ik(self):
        """IK로 잡은 것은 IK로 든다. 쥔 채면 True.

        종전 `lift()`는 고정 자세를 계단식으로 밟는데, 그 첫 자세가 IK 파지 자세와
        달라 드는 순간 급변이 생긴다 — 실측: 물림각 0.072·부하 -10.0으로 제대로
        물어 놓고 들다가 놓쳤다(DROPPED). 같은 좌표를 높이만 올려 풀면 죠의 자세가
        유지되므로 그 급변이 없다(팔 단독 프로브에서 6/6).

        한 번에 올리지 않는 이유는 종전과 같다 — 물체가 바닥을 떠나는 첫 순간에
        하중이 걸려 가장 잘 빠진다.
        """
        c = getattr(self, '_ik_cube', None) or self._cube_now()
        if c is None:
            return self.holding()
        cx, cy, cz = c
        for up in (0.02, 0.05, 0.09, 0.14):
            q = K.grasp_q(cx, cy, cz, up=up)
            if q is None:
                break
            self.move_arm(dict(zip(ARM_JOINTS, q)), 1.5)
            # 매 단계 재조이지 않는다. 팔 단독 프로브는 재조임 없이 6/6이었는데
            # 여기서는 재조임을 넣고 들다가 놓쳤다(물림각 0.108 → -0.170).
            # 25mm 큐브는 가벼워 재조임의 순간 힘에 튕겨 나가는 것으로 보인다.
        # 운반 자세에 들어가기 직전에 **유지력**을 건다. 들기 중에는 재조임이
        # 큐브를 튕겨내지만(실측), 운반 중에는 반대로 유지력이 없으면 회전 관성에
        # 미끄러져 나간다 — 실측: 물림각이 0.287에서 0.116으로 서서히 줄다가
        # 35도 회전 직후 -0.170(완전 닫힘)이 됐다. 종전 코드가 '닫기 10, 유지 30'으로
        # 나눠 쓴 이유가 이것이다.
        self.move_gripper(GRIPPER_CLOSED, wait=False, effort=30.0)
        # 운반 자세도 IK로 만든다. 고정 자세로 전환하면 죠 각도가 급변해 빠진다.
        carry = K.grasp_q(CARRY_X, 0.0, CUBE_CZ, up=CARRY_UP)
        if carry is not None:
            self.move_arm(dict(zip(ARM_JOINTS, carry)), 2.5)
            self._ik_carry = carry
        else:
            self.move_arm(POSE_CARRY, 2.5)
        time.sleep(0.5)
        return self.holding()

    def grasp(self, pan):
        """물체 위에서 내려가 죠를 닫고, 실제로 물었는지까지 판정한다. 물었으면 True.

        approach()가 넘겨준 pan은 "물체가 정면에서 얼마나 틀어져 있나"이고,
        여기서 어깨를 그만큼 돌려 죠를 물체 쪽으로 향하게 한다.

        순서에 전부 이유가 있다.
          ① 상공 대기 자세로 이동      물체 위에 뜬 상태 (아직 안 내려감)
          ② 그리퍼 열기                내려가기 전에 열어야 물체를 밀지 않는다
          ③ 스윙 드리프트 보정         팔이 휘두르는 반동으로 베이스가 밀린 만큼 되돌린다
          ④ 손목캠 정렬 (바닥 모드)    전방 카메라는 근접에서 팔에 가려 못 쓴다
          ⑤ 수직 하강                  파지 자세로
          ⑥ creep 전진                 하강하며 죠가 뒤로 물러난 만큼(25mm) 밀어 넣는다
          ⑦ 닫고 판정, 실패면 재시도   최대 3회, 실패 양상에 따라 다르게 대응

        ⑥이 직관에 어긋나는 부분이다. '수직' 하강이라고 부르지만 실제로는 관절이
        호를 그리므로 죠의 x 좌표가 25mm 뒤로 물러난다. 상공에서 중심을 맞춰 놨어도
        내려오면 물체가 죠 끝단에 걸리게 되고, 그 상태로 들면 미끄러진다.

        ⑦의 재시도는 실패 양상을 구분한다. 각도가 상한을 넘으면(얕은 걸침) 죠가
        물체 위로 올라탄 것이라 더 밀어 넣고, 그 외에는 열었다 다시 닫는다.
        """
        # 바닥 모드는 역기구학으로 잡는다(2026-08-11 전환). 종전 고정 자세 경로는
        # `use_ik:=false`로 되돌릴 수 있게 남겨 둔다 — 받침대 모드는 아직 그쪽이다.
        if self.floor_mode and self.use_ik:
            return self._grasp_ik()

        # 실패 사유는 이번 시도의 것만 남긴다. 종전에는 닫기 루프 끝에서만
        # last_grasp_angle을 채웠는데, 손목캠 미검출이나 죠 안착 실패로 **닫기 전에**
        # 빠져나오면 직전 사이클의 각도가 그대로 남아 있었다. 그래서 닫지도 않은
        # 사이클이 '완전 닫힘 = 허공, 각도=-0.170'으로 보고됐다 — 사유를 남기려고
        # 만든 로그가 거짓말을 하는 셈이다.
        self.last_grasp_angle = None
        self.last_grasp_fail = None
        pre = POSE_PRE_FLOOR if self.floor_mode else POSE_PRE
        grasp_pose = POSE_GRASP_FLOOR if self.floor_mode else POSE_GRASP
        self.move_arm({**pre, 'arm_shoulder_pan': pan}, 3.0)
        self.get_logger().info('그리퍼 열기 (물체 근처 도착)')
        self.move_gripper(1.2)
        time.sleep(0.5)
        # 팔 스윙 반동으로 베이스가 밀리므로 odom 변위만큼 되돌린다
        if getattr(self, '_anchor_odom', None) and self.odom:
            x0, y0, yaw0 = self._anchor_odom
            self.spin_until(lambda: self.odom is not None, 3.0)
            x1, y1, _ = self.odom
            fwd = math.cos(yaw0) * (x1 - x0) + math.sin(yaw0) * (y1 - y0)
            self.get_logger().info(f'스윙 드리프트 보정: {fwd * 1000:.0f}mm 후진')
            if abs(fwd) > 0.005:
                v = -0.03 if fwd > 0 else 0.03
                self.drive(v, 0.0, min(2.5, abs(fwd) / 0.03 + 0.1))
        roll, reach = 0.0, 0.0
        pose_hover = dict(pre)        # 직교 하강의 출발 자세 (상공)
        self._reach_applied = 0.0     # 이번 파지의 누적 리치 — lift()가 기준으로 쓴다
        if self.floor_mode:
            # 근접 비전 사각을 손목 RGB로 보완: 물체 중간이 파지선에 오도록 pan 정렬 + 기울기 롤 정렬
            aligned = self.wrist_align(pan, pre)
            if aligned is None:
                return False    # 손목캠이 물체를 전혀 못 봄 — 재접근이 답이다
            pan, roll, reach = aligned
        self.get_logger().info('수직 하강' + (' (바닥 모드)' if self.floor_mode else ''))
        descended = {**grasp_pose, 'arm_shoulder_pan': pan, 'arm_wrist_roll': roll}
        if self.floor_mode and abs(reach) > 0.001:
            # 상공에서 팔로 뻗어 맞춘 전후량을 하강 자세에도 그대로 옮긴다. 하강 후퇴는
            # 따로 더하지 않는다 — 포켓·상공기준을 같은 스윕에서 재서 이미 포함돼 있다.
            # 감도는 상공과 다르므로(펴진 정도가 달라 4배 차이) 파지 자세 계수를 쓴다.
            total = reach
            descended['arm_shoulder_lift'] = \
                grasp_pose['arm_shoulder_lift'] + REACH_LIFT_PER_M * total
            descended['arm_elbow_flex'] = \
                grasp_pose['arm_elbow_flex'] + REACH_ELBOW_PER_M * total
            descended['arm_wrist_flex'] = grasp_pose['arm_wrist_flex'] \
                - (REACH_LIFT_PER_M + REACH_ELBOW_PER_M) * total
            self._reach_applied = total
            self.get_logger().info(f'하강 자세에 상공 리치 {total * 1000:+.0f}mm 반영')
        # 하강은 천천히·끝까지. 실측에서 '자세 도달 미완 0.075~0.142rad'가 반복됐는데,
        # lift 0.142rad는 죠 높이 17mm에 해당해 큐브를 스치는 높이가 된다(사진 확인).
        if self.floor_mode:
            # 큐브 뒤로 물러난 자리에서 직교 직선으로 내린다. 두 가지를 동시에 피한다:
            #   · 관절 보간의 호 → 죠가 앞뒤로 쓸며 큐브를 쳐낸다
            #   · 큐브 바로 위 하강 → 아랫턱이 큐브 윗면에 얹힌다
            # 뒤로 물려 내리는 방식은 쓰지 않는다. 아랫턱이 큐브 윗면을 지나가는 문제는
            # 직교 직선 하강만으로 해결됐고(어긋남 25mm → 0~2mm), 거기에 뒤로물림을
            # 더했더니 GRASP_REF가 무효가 됐다 — 그 기준점은 명목 하강 자세에서 잰
            # 값이라, 40mm 뒤에서 기어들어온 자세에서는 같은 픽셀이 다른 위치를 뜻한다.
            # 실측: 뒤로물림 없이 1사이클 파지 성공(물림각 0.448) / 넣은 뒤 3사이클
            # 모두 실패(물림각 0.575~0.827, 대각선 걸림).
            self._descend_vertical({**pose_hover, 'arm_shoulder_pan': pan,
                                    'arm_wrist_roll': roll}, descended)
        else:
            self.move_arm(descended, 4.0)
        # 하강이 끝나기 전에 닫으면 큐브 상단을 스치며 얕게 물거나 밀어낸다.
        # 목표는 `descended`가 아니라 **실제로 명령한 마지막 자세**(_last_arm)다.
        # `_descend_vertical`은 x를 유지하려고 단계마다 `_reach_arm`으로 lift·elbow·
        # wrist에 오프셋을 얹는데, 그 오프셋이 빠진 `descended`를 목표로 검사하면
        # 리치가 조금이라도 걸린 순간 구조적으로 통과할 수 없다.
        #   실측: '자세 도달 미완 0.265rad (elbow 목표+0.15 실제-0.12)'.
        #   0.265 / REACH_ELBOW_PER_M(8.86) = 29.9mm — `_reach_arm` 1회 상한 30mm와 일치.
        # 게다가 이어지는 재명령이 `descended`로 되돌리면서, 직교 하강으로 어렵게
        # 맞춰 둔 x를 25~30mm 뒤로 밀어낸다 — 직교 하강을 도입한 목적 자체가 무효가 된다.
        want = dict(self._last_arm or descended)
        if not self.wait_arm_settled(want, tol=0.02, timeout=12.0):
            self.get_logger().warning('하강 미완 — 한 번 더 명령')
            self.move_arm(want, 2.5)
            self.wait_arm_settled(want, tol=0.03, timeout=8.0)
        # 접지 가드: 도달 허용오차(0.04)가 접촉 경계 여유(0.02)보다 크므로 실측 lift를
        # 감시한다. 1.17을 넘으면 손끝이 바닥을 눌러 바퀴가 헛도는 상태로 회귀한 것.
        lift_now = getattr(self, 'joint_pos', {}).get('arm_shoulder_lift')
        # 리치 보정은 높이를 유지한 채 lift를 올리므로(elbow가 상쇄) 경계도 같이 올라간다.
        # 안 올리면 정상 자세인데도 매번 간섭 경고가 뜬다.
        lift_gate = 1.17 + REACH_LIFT_PER_M * reach
        if self.floor_mode and lift_now is not None and lift_now > lift_gate:
            self.get_logger().warning(
                f'파지 자세 lift={lift_now:.3f} > {lift_gate:.3f} — 손끝 바닥 간섭 구간, 접지 상실 위험')
        # 하강은 수직이 아니다: 상공 자세와 파지 자세의 죠 x가 25mm 차이 난다
        # (2026-08-03 실측, 새 파지 자세 1.15/0.28 기준. 구자세 1.20/0.23에서는 40mm).
        # 상공에서 큐브 중심에 맞춰 놓아도 내려오면 그만큼 뒤로 물러나 큐브가 죠
        # 끝단에만 걸리고, 들다가 미끄러진다. 열린 죠로 그 차이만큼 전진해 큐브를
        # 죠 안쪽까지 넣는다 — creep 기본값 25mm가 이 후퇴량과 정확히 일치한다.
        # 손목캠을 36° 기울인 뒤로는 내려간 상태에서도 큐브가 보이므로(실측 면적
        # 27k~37k) 고정 25mm가 아니라 실제로 보이는 오차만큼 뻗는다. 안 보일 때만
        # 25mm 폴백으로 물러선다. creep은 그 폴백 값 겸 기능 on/off 스위치다.
        if self.floor_mode and self.creep > 0.001:
            seated = self._seat_into_jaws()
            ok, why, roll = self._ready_to_close(roll)
            if seated and not ok:
                self.get_logger().warning(f'닫기 보류 — {" / ".join(why)}')
                seated = False
            if not seated:
                # 큐브가 죠 사이에 없다는 것을 이미 봤다. 그대로 닫으면 허공을 문다
                # (실측: 죠 안착 -78mm를 측정해 놓고 닫아 부하 -0.0). 확인하고도
                # 무시하느니 열어 두고 재접근하는 편이 싸다 — 부하 판정이 실패를
                # 정확히 잡아 주므로 재시도 비용이 예측 가능하다.
                self.last_grasp_fail = f'죠 안착 실패({" / ".join(why) if why else "미확인"})'
                self.get_logger().warning('죠 안착 실패 — 큐브가 죠 사이에 없다, 닫지 않고 재접근')
                self.move_gripper(1.0)
                return False
        angle = None
        for attempt in range(3):
            self.get_logger().info(f'그리퍼 닫기 (시도 {attempt + 1})')
            # 닫는 힘 10.0 → 7.5 (25% 감소). 세게 물면 큐브를 죠 밖으로 튕겨내거나
            # 모서리를 파고들어 얕게 걸린다. 유지력(30.0)은 그대로 둔다 — 물고 난
            # 뒤에는 놓치지 않는 쪽이 중요하다.
            self.move_gripper(GRIPPER_CLOSED, effort=7.5)
            time.sleep(1.0)
            self.gripper_angle = self.gripper_effort = None
            self.spin_until(lambda: self.gripper_angle is not None, 5.0)
            angle, load = self.gripper_angle, self.gripper_effort
            self.get_logger().info(
                f'  닫힘 신호: 각도={angle if angle is None else round(angle, 3)} '
                f'부하={load if load is None else round(load, 3)}')
            if self.gripped(angle):
                # 유지 목표는 접촉각 바로 안쪽으로 묶는다. 완전 닫힘(-0.17)을 계속
                # 명령하면 시뮬 접촉이 실물 서보처럼 토크 한계에서 멈추지 않아 죠가
                # 큐브를 끝까지 뚫고 지나간다 — 실측(들어올리는 동안):
                #     0.385 → 0.332 → 0.283 → 0.044 → -0.170(완전 닫힘, 큐브 이탈)
                # 부하를 운반 내내 살리려면 도달 불가능한 목표가 필요한데, 이 물리
                # 모델에서 도달 불가능한 목표는 결국 큐브를 부수며 도달한다. 그래서
                # 부하는 '파지 순간'의 신호로만 쓰고, 운반 중 확인은 전방캠으로 잰
                # 큐브 높이가 맡는다(holding 참조). 침투는 아래 여유만큼으로 끝난다.
                self.hold_target = max(GRIPPER_CLOSED, angle - GRIP_HOLD_MARGIN)
                self.move_gripper(self.hold_target, effort=30.0)
                time.sleep(0.3)
                break
            if angle is not None and angle >= GRIP_HOLD_MAX:
                # 얕게 걸친 상태 — 큐브가 죠 사이가 아니라 죠 앞에 있어 죠가 큐브 위로
                # 올라탄 것이다(실측: 0.347/0.349/0.350이 반복되고 전부 제자리).
                # 물러나면 더 멀어지고, 상공으로 올려 다시 내려오면 하강 후퇴(25mm)가
                # 되풀이된다. 파지 자세를 유지한 채 열고 더 뻗는다.
                # 차체가 아니라 팔로 뻗는다 — 죠가 큐브 위에 올라탄 상태에서 차체를
                # 밀면 아랫턱이 큐브를 그대로 밀어내 버린다(실측: 계속 밀어 굴러감).
                # 전진량은 1cm까지만: 2cm로 두 번 밀었더니 큐브가 4cm 밀려나
                # 허공을 물었다(실측 -0.170).
                self.get_logger().info(f'얕은 걸침(각도={angle:.3f}) — 팔로 더 뻗어 재물림')
                self.move_gripper(1.0)
                time.sleep(0.4)
                self._reach_arm(0.01, sec=0.8)
                continue
            self.get_logger().info(f'얕은 물림(각도={angle}) — 재물림')
            self.move_gripper(0.5)
            time.sleep(0.6)
        # 판정 직전에 부하를 새로 받는다 — 유지 명령 직후의 값이라야 의미가 있다
        self.gripper_effort = None
        self.spin_until(lambda: self.gripper_effort is not None, 2.0)
        held = self.gripped(angle)
        load = self.gripper_effort
        # 판정에 쓴 각도를 보존한다 — 이후 재열림(move_gripper 0.5/1.0)으로
        # self.gripper_angle이 열림값으로 덮이므로, main의 실패 사유 로그는 이 값을 봐야 한다
        self.last_grasp_angle = angle
        self.get_logger().info(
            f'파지 검증: 부하={load if load is None else round(load, 3)} '
            f'(임계 {GRIP_LOAD_MIN}) 각도={angle if angle is None else round(angle, 3)} '
            f'→ {"HOLDING" if held else "EMPTY"}')
        if held and not self.floor_mode:
            # 바닥 모드는 자세 급변(원-모션 상승) 없이 곧장 계단 들기로 (실검증 방식)
            self.get_logger().info('수직 상승')
            self.move_arm({**pre, 'arm_shoulder_pan': pan}, 2.5)
            time.sleep(0.3)
        return held

    # ---- 쓰레기통 검출·접근·투입 ----
    def _base_z_of(self, us, vs, ds):
        """픽셀 배열의 base_footprint 높이를 한 번에 계산해 배열로 반환.

        높이만 필요하므로 회전 행렬의 마지막 행만 쓴다. 픽셀마다 TF를 조회하면
        비싸지만, 변환을 한 번 얻어 벡터 연산으로 적용하면 전체 마스크를 훑을 수 있다.
        """
        k = self.cam_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        ox = (us - cx) * ds / fx
        oy = (vs - cy) * ds / fy
        # 광학(X우·Y하·Z전) → 링크(x전·y좌·z상)
        px, py, pz = ds, -ox, -oy
        tr = self.tf_buffer.lookup_transform('base_footprint', CAMERA_FRAME, rclpy.time.Time())
        q, t = tr.transform.rotation, tr.transform.translation
        r20 = 2.0 * (q.x * q.z - q.w * q.y)
        r21 = 2.0 * (q.y * q.z + q.w * q.x)
        r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        return r20 * px + r21 * py + r22 * pz + t.z

    def _locate_trash_aruco(self):
        """마커로 통 중심을 잡는다. base_footprint 좌표, 실패 시 None.

        임계값이 없다 — 마커 네 꼭짓점과 기지 크기(TRASH_ARUCO_SIZE)로 solvePnP를
        풀면 6-DoF 자세가 나온다. 통 중심은 마커에서 고정 변환만큼 떨어진 곳이고,
        그 변환을 마커 자세로 회전시키므로 비스듬히 봐도 성립한다.
        """
        self.color = None
        if not self.spin_until(
                lambda: self.color is not None and self.cam_info is not None, 3.0):
            self._aruco_fail('영상없음')
            return None
        frame = self.color
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        d = cv2.aruco.getPredefinedDictionary(TRASH_ARUCO_DICT)
        if hasattr(cv2.aruco, 'ArucoDetector'):                 # OpenCV 4.7+
            corners, ids, _ = cv2.aruco.ArucoDetector(
                d, cv2.aruco.DetectorParameters()).detectMarkers(gray)
        else:                                                   # 4.6 이하
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, d, parameters=cv2.aruco.DetectorParameters_create())
        if ids is None or len(ids) == 0:
            self._aruco_fail('마커없음', frame)
            return None
        # 통 마커는 id 0이다(gen_aruco_sdf.py). 다른 마커가 잡히면 그걸 통으로
        # 쓰지 않는다 — 종전에는 corners[0]을 무조건 써서 조용히 엉뚱한 곳으로 갔다.
        flat = [int(v) for v in ids.flatten()]
        idx = flat.index(TRASH_ARUCO_ID) if TRASH_ARUCO_ID in flat else None
        if idx is None:
            self._aruco_fail(f'다른 마커만 검출{flat}', frame)
            return None
        k = np.array(self.cam_info.k, dtype=np.float64).reshape(3, 3)
        dist = (np.array(self.cam_info.d, dtype=np.float64)
                if len(self.cam_info.d) else np.zeros(5))
        h = TRASH_ARUCO_SIZE / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        # solvePnP는 입력이 나쁘면 False가 아니라 예외를 던진다. 여기서 안 막으면
        # 운반 루프를 뚫고 나가 실행 전체가 종료된다 — 통을 한 번 못 본 것치고 비싸다.
        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj, corners[idx].reshape(4, 2).astype(np.float64),
                k, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error as e:
            self._aruco_fail(f'solvePnP예외({str(e)[:40]})', frame)
            return None
        if not ok:
            self._aruco_fail('solvePnP실패', frame)
            return None
        # 통 중심 = 마커 원점 + 마커자세 × 고정 오프셋
        rot, _ = cv2.Rodrigues(rvec)
        c = rot @ np.array(TRASH_MARKER_TO_CENTER, dtype=np.float64).reshape(3, 1) + tvec
        # 광학(X=우, Y=하, Z=전) → 링크(x=전, y=좌, z=상). 이 변환을 빼면 높이가
        # 0.13m 대신 1.2m로 나온다(실측) — _pixel_to_base와 같은 규약이다.
        ox, oy, oz = float(c[0]), float(c[1]), float(c[2])
        p = PointStamped()
        p.header.frame_id = CAMERA_FRAME
        p.point.x, p.point.y, p.point.z = oz, -ox, -oy
        if not self.spin_until(
                lambda: self.tf_buffer.can_transform('base_footprint', CAMERA_FRAME,
                                                     rclpy.time.Time()), 2.0):
            self._aruco_fail('TF없음', frame)
            return None
        try:
            tf = self.tf_buffer.lookup_transform('base_footprint', CAMERA_FRAME,
                                                 rclpy.time.Time())
        except Exception as e:
            self._aruco_fail(f'TF조회실패({type(e).__name__})', frame)
            return None
        q = do_transform_point(p, tf).point
        self._aruco_miss = 0
        self.get_logger().info(
            f'쓰레기통(마커 id={TRASH_ARUCO_ID}): base=({q.x:.3f},{q.y:.3f},{q.z:.3f}) '
            f'r={math.hypot(q.x, q.y):.3f}')
        return q.x, q.y

    def _aruco_fail(self, why, frame=None):
        """마커 검출 실패 사유를 남긴다. 실패 프레임은 처음 몇 장만 저장한다.

        "운반 루프에서는 매번 None인데 단독 호출로는 잡힌다"는 상태를 사유 없이
        보고 있었다. 실패는 네 갈래(영상없음/마커없음/solvePnP실패/TF없음)이고
        어느 갈래인지에 따라 손볼 곳이 완전히 다르다 — 마커없음이면 시야·가림 문제,
        TF없음이면 좌표계 문제다. 사유를 안 남기면 그 구분을 못 한다.
        """
        n = getattr(self, '_aruco_miss', 0) + 1
        self._aruco_miss = n
        path = ''
        # 저장 한도는 실행 단위로 센다. `_aruco_miss`는 검출에 성공하면 0으로
        # 돌아가므로 그걸로 한도를 세면 한 실행 안에서도 파일이 계속 덮어써진다.
        saved = getattr(self, '_aruco_saved', 0)
        if frame is not None and saved < 6:
            try:
                os.makedirs(ARUCO_FAIL_DIR, exist_ok=True)
                # 사유를 파일명에 넣는다 — 이미지만 봐서는 네 갈래 중 어느 것인지 모른다.
                # 시각을 붙여 이전 실행의 증거를 덮어쓰지 않는다.
                safe = why.replace('/', '_').replace(' ', '')[:24]
                path = os.path.join(
                    ARUCO_FAIL_DIR, f'{time.strftime("%m%d_%H%M%S")}_{safe}.png')
                cv2.imwrite(path, frame)
                self._aruco_saved = saved + 1
                path = f' → {path}'
            except Exception:
                path = ''
        self.get_logger().info(f'마커 미검출({why}) {n}회차{path}')

    def locate_trash(self):
        """쓰레기통 중심의 base_footprint 좌표. 실패 시 None.

        마커를 먼저 본다. 못 보면 종전 HSV 회색 덩어리 방식으로 물러선다 —
        마커가 없는 환경(실물 초기 세팅 등)에서도 최소한 굴러가게 두기 위해서다.
        """
        m = self._locate_trash_aruco()
        if m is not None:
            self._trash_from_marker = True
            return m
        # 마커로 잡은 값인지 HSV 폴백인지 호출부가 알아야 한다. 둘은 신뢰도가
        # 전혀 다르다 — 마커는 ±25mm/±2°인데 HSV는 방향 정보가 없고 +20~237mm로 튄다.
        self._trash_from_marker = False
        self.color = self.depth = None
        if not self.spin_until(
                lambda: self.color is not None and self.depth is not None
                and self.cam_info is not None, 5.0):
            return None
        hsv = cv2.cvtColor(self.color, cv2.COLOR_BGR2HSV)
        dep = np.nan_to_num(np.asarray(self.depth), nan=0.0, posinf=0.0)
        mask = ((hsv[:, :, 1] < TRASH_S_MAX) & (hsv[:, :, 2] > TRASH_V_LO)
                & (hsv[:, :, 2] < TRASH_V_HI) & (dep > TRASH_D_LO)
                & (dep < TRASH_D_HI)).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        num, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
        best, rejected = None, []
        for i in range(1, num):
            a = int(stats[i, cv2.CC_STAT_AREA])
            if a < TRASH_MIN_AREA:
                continue
            vs, us = np.nonzero(labels == i)
            ds = dep[vs, us]
            ok = (ds > TRASH_D_LO) & (ds < TRASH_D_HI)
            if int(ok.sum()) < 50:
                rejected.append(f'면적{a}:뎁스부족')
                continue
            us, vs, ds = us[ok], vs[ok], ds[ok]
            # 성분 중심이 아니라 성분 안에서 통 높이인 픽셀만 골라 쓴다. 통(0.18m)과
            # 바닥이 같은 회색이라 색 마스크에서 한 덩어리로 붙는데, 덩어리 중심을
            # 쓰면 높이가 바닥으로 끌려가 통이 계속 기각된다(실측: 면적 3만대 덩어리
            # 하나가 높이 -0.02로 12회 연속 기각, 통을 한 번도 못 잡음).
            try:
                zs = self._base_z_of(us.astype(float), vs.astype(float), ds)
            except Exception:
                rejected.append(f'면적{a}:TF실패')
                continue
            sel = (zs > TRASH_Z_LO) & (zs < TRASH_Z_HI)
            n_sel = int(sel.sum())
            if n_sel < TRASH_MIN_AREA // 2:
                rejected.append(f'면적{a}:통높이픽셀{n_sel}')
                continue
            # 통은 높이 0.18m 구조물 — 물체용 기본 게이트(0.20)로는 상단이 걸린다
            # 깊이는 중앙값이 아니라 하위 15퍼센타일을 쓴다. 통은 속이 빈 상자라
            # 선택된 픽셀에 앞벽·바닥·먼 안쪽벽이 모두 섞이는데, 중앙값을 쓰면
            # 통 중심이나 뒤쪽이 '앞면'으로 나온다. 거기에 TRASH_HALF(앞면→중심)를
            # 또 더하니 이중 계산이 되어 거리가 +156~301mm 과대평가됐다(실측).
            # 우리가 원하는 앞면은 가장 가까운 표면이므로 낮은 퍼센타일이 맞다
            # (최솟값은 뎁스 노이즈 한 점에 끌려가므로 쓰지 않는다).
            loc = self._pixel_to_base(float(us[sel].mean()), float(vs[sel].mean()),
                                      float(np.percentile(ds[sel], 15)), z_gate=(-0.10, 0.60))
            if loc is None:
                rejected.append(f'면적{a}:역투영실패')
                continue
            x, y, z = loc[0], loc[1], loc[2]
            if best is None or n_sel > best[0]:
                best = (n_sel, x, y, z)
        if best is None:
            if rejected:
                self.get_logger().info('통 후보 기각: ' + ', '.join(rejected[:4]))
            return None
        a, x, y, z = best
        # 뎁스는 앞면 → 중심 방향으로 반깊이만큼 밀어 통 중심을 추정
        r = math.hypot(x, y)
        k = (r + TRASH_HALF) / r if r > 1e-3 else 1.0
        self.get_logger().info(
            f'쓰레기통: 앞면 base=({x:.3f},{y:.3f},{z:.3f}) 면적={a} → 중심 r={r + TRASH_HALF:.3f}')
        return x * k, y * k

    def carry_to_trash(self, max_iter=70):
        """물체를 든 채 쓰레기통 앞까지 저속 주행. 성공 시 True."""
        # 팔은 들어올린 자세 그대로 손대지 않는다. 자세 전환은 물론 pan 회전만으로도
        # 얕게 물린 물체가 빠진다(실측 반복). 통은 큰 구조물이라 팔 위쪽 시야로 원거리에서
        # 검출되고, 근접해 가려지는 구간은 근접 락(추측 접근)이 담당한다.
        self._carry_drop = False
        self._reseat_cnt = 0
        self._marker_seek = 0            # 도착 직전 마커 확인 탐색 횟수
        self._trash_from_marker = False  # 아직 아무것도 못 봤다
        if not self.holding():
            self._carry_drop = True   # 들기 직후 낙하도 재파지로 복구 가능
            self.get_logger().error('운반 시작 시 물체 없음')
            return False
        # 운반 구간은 배율을 1로 — 회전 관성만으로도 얕게 물린 물체가 빠진다(실측)
        # _carrying: drive()가 사다리꼴 프로파일(저크 제거)을 켜는 플래그
        carry_scale, self.scale = self.scale, 1.0
        self._carrying = True
        # 팔을 옆으로 뺀다. 정면 자세로 두면 팔이 화면 중앙을 가려 검출이 통이 아니라
        # 자기 팔을 잡는다 — 실측(trash_probe.py): 로봇 위치를 0.64m~0.17m로 바꿔도
        # 검출값이 (+0.311,-0.01)로 고정이었고, 팔을 빼자 위치에 따라 변하며 검출률이
        # 1/5 → 3~4/5로 올랐다. POSE_CARRY_SCAN은 이 목적으로 정의돼 있었는데 참조가
        # 한 곳도 없었다. pan 회전만이라 쥔 물체는 흔들리지 않는다(place에서 검증된 동작).
        self.move_arm({'arm_shoulder_pan': POSE_CARRY_SCAN['arm_shoulder_pan']}, 2.0)
        try:
            return self._carry_loop(max_iter)
        finally:
            self.scale = carry_scale
            self._carrying = False

    def _carry_loop(self, max_iter):
        # 탐색 반복은 접근 예산을 쓰지 않는다. 통이 뒤쪽에 있으면 회전 탐색에 37회를
        # 쓰고 접근에 7회만 남아 1.2m에서 0.78m까지 좁히다 예산이 끝났다(실측).
        # 탐색의 종료는 scan_step이 한 바퀴 × 자리이동 횟수로 따로 관장한다.
        miss = 0
        it = 0
        approached = 0
        while approached < max_iter:
            it += 1
            self.spin_until(lambda: self.odom is not None, 3.0)
            # 근접 락: 통이 화면을 채우면 마스크 중심이 한쪽 면으로 치우쳐 방위가 수렴하지 않는다
            # (실측: r=0.50에서 brg -14도 고착). 락 이후에는 기억한 좌표로만 추측 접근한다.
            near_lock = getattr(self, '_trash_lock', False)
            loc = None if near_lock else self.locate_trash()
            if loc is not None and self.odom:
                # 통은 고정물 — 실검출마다 오도메트리 좌표로 기억해 두고 근접 사각에서 재사용.
                # 단, 기억과 0.8m 이상 어긋난 관측은 유령(원거리 벽·그림자 블롭)으로 기각 —
                # 실측: 실통(0.6m)과 유령(2.4m) 검출이 교대로 앵커를 덮어써 접근이 진동하며
                # 예산 절반을 태웠다. 3연속 기각되면 기억이 틀린 것으로 보고 새 관측을 받는다.
                x, y, yaw = self.odom
                wx = x + math.cos(yaw) * loc[0] - math.sin(yaw) * loc[1]
                wy = y + math.sin(yaw) * loc[0] + math.cos(yaw) * loc[1]
                prev = getattr(self, '_trash_odom', None)
                if prev is not None and math.hypot(wx - prev[0], wy - prev[1]) > 0.8 \
                        and getattr(self, '_anchor_rej', 0) < 3:
                    self._anchor_rej = getattr(self, '_anchor_rej', 0) + 1
                    self.get_logger().info(
                        f'[{it}] 통 관측 기각: 기억 좌표와 '
                        f'{math.hypot(wx - prev[0], wy - prev[1]):.2f}m 불일치 (유령 의심)')
                    ox, oy = prev
                    dx, dy = ox - x, oy - y
                    loc = (math.cos(yaw) * dx + math.sin(yaw) * dy,
                           -math.sin(yaw) * dx + math.cos(yaw) * dy)
                else:
                    self._anchor_rej = 0
                    self._trash_odom = (wx, wy)
            elif loc is None and getattr(self, '_trash_odom', None) and self.odom:
                ox, oy = self._trash_odom
                x, y, yaw = self.odom
                dx, dy = ox - x, oy - y
                loc = (math.cos(yaw) * dx + math.sin(yaw) * dy,
                       -math.sin(yaw) * dx + math.cos(yaw) * dy)
                self.get_logger().info(f'[{it}] 통 추정 추적: base=({loc[0]:.3f},{loc[1]:.3f})')
            if loc is None:
                miss += 1
                if not self.scan_step(it):
                    self.get_logger().error('쓰레기통 미검출 — 탐색 실패')
                    return False
                continue
            miss = 0
            approached += 1
            # 운반 중 유지력을 다시 주장한다. 같은 절대 목표를 반복하므로 죠는
            # 움직이지 않고, 하중이나 회전 관성에 밀려 벌어졌을 때만 되돌아온다.
            if self.use_ik and getattr(self, '_ik_grasped', False) and approached % 3 == 1:
                self.move_gripper(GRIPPER_CLOSED, wait=False, effort=30.0)
            tx, ty = loc
            r = math.hypot(tx, ty)
            brg = math.atan2(ty, tx)
            er = r - TRASH_POCKET_X
            # 근접에서는 통이 화면을 채워 마스크가 한쪽 벽으로 치우치고 방위가 편향된다
            # (실측: 락을 0.42로 늦추니 r 0.60·brg 14도에서 고착해 반복 소진). 그래서
            # 0.60에서 락을 걸고 이후는 오도메트리로 추측한다.
            # 락 임계를 0.60 → 0.35로 낮췄다. 정지 거리가 0.50이므로 정상 경로에서는
            # 락이 걸리지 않고, 마지막까지 마커를 보며 간다. 종전에는 0.60에서 락을
            # 걸고 0.17m를 오도메트리로 갔는데 그 추측이 10cm 어긋났다(실좌표 확인).
            # 마커를 아예 못 보는 상황에서만 옛 경로로 물러선다.
            if not near_lock and r < 0.35 and getattr(self, '_trash_odom', None):
                self._lock_trash_pose()
                self._trash_lock = True
                self.get_logger().info(f'근접 락 (r={r:.3f}) — 이후 추측 접근')
            # odom을 함께 찍는다. 관측값이 얼어붙었을 때 차체가 도는지 아닌지를
            # 구분하려면 이 두 줄이 같이 있어야 한다 — 실측에서 통 좌표가 71회
            # 소수점까지 동일했는데, 차체 회전은 별도 측정에서 99% 달성됐다.
            od = self.odom
            self.get_logger().info(
                f'[{it}] 통 접근: r={r:.3f} er={er * 1000:.0f}mm brg={math.degrees(brg):.1f}deg'
                + ('' if not od else f' | odom ({od[0]:+.3f},{od[1]:+.3f}) '
                                     f'yaw={math.degrees(od[2]):+.1f}도'))
            # 개구부 13.6cm 기준 거리 0.4m에서 허용 방위는 약 9.6도 — 임계를 그 안쪽으로 둔다.
            # 좁게 잡으면 회전만 반복하다 전진을 못 한다(실측: 3.4도 임계에서 예산 소진).
            if abs(er) < 0.025 and abs(brg) < 0.10:
                # **최종 도착은 마커로만 인정한다.** HSV 덩어리는 방향 정보가 없고
                # 프레임마다 튄다 — 실측: 같은 통을 연속으로 보면서 y가 0.167 →
                # -0.007로 17cm 튀었고, 그 한 프레임이 도착 조건을 통과시켜 팔을
                # 엉뚱한 곳에 뻗었다(큐브가 통에서 0.183m 떨어진 바닥에 떨어짐).
                # 마커가 안 보이는 건 대개 통을 정면으로 안 보고 있어서다 —
                # 통 앞 0.5m에서 정면이면 pan -1.0에서도 잡힌다(carry_pan_probe 실측).
                # 그래서 제자리에서 조금씩 돌며 마커를 찾은 뒤 다시 판정한다.
                if not getattr(self, '_trash_from_marker', False):
                    n_seek = getattr(self, '_marker_seek', 0)
                    if n_seek < 8:
                        self._marker_seek = n_seek + 1
                        # 좌우로 번갈아 넓혀 가며 훑는다(0 → +12° → -12° → +24° …)
                        step = math.radians(12) * ((n_seek + 2) // 2)
                        wz = step if n_seek % 2 == 0 else -step
                        self.get_logger().info(
                            f'[{it}] 도착 조건 충족했으나 마커 미확인 — '
                            f'{math.degrees(wz):+.0f}도 돌려 마커 탐색 ({n_seek + 1}/8)')
                        self.drive(0.0, 0.4 if wz > 0 else -0.4, min(2.0, abs(wz) / 0.4))
                        continue
                    self.get_logger().warning(
                        '마커를 못 찾은 채 도착 판정 — HSV 좌표로 투입(정확도 낮음)')
                # 투입 자세·리치는 pan=0 기준 실측값이므로 정면으로 되돌린다
                self.move_arm({'arm_shoulder_pan': 0.0}, 2.0)
                self._trace('통 도착', er_mm=round(er * 1000), brg_deg=round(math.degrees(brg), 1),
                            marker=bool(getattr(self, '_trash_from_marker', False)),
                            trash_odom=[round(v, 3) for v in (self._trash_odom or (0, 0))])
                return True
            if not self.holding():
                # 능동 판별이 확정한 실낙하 — 큐브는 근처 바닥에 있다. 복구 루프가
                # 재접근·재파지 후 운반을 재개할 수 있도록 사유를 남긴다.
                self._carry_drop = True
                self.get_logger().error('운반 중 물체 놓침 (재파지 복구 대상)')
                return False
            la = getattr(self, '_last_hold_area', None)
            if la is not None and la < HOLD_AREA_MIN and getattr(self, '_reseat_cnt', 0) < 2:
                # 조기 재안착: 능동 판별로 간신히 살린 상태(면적<임계)는 미끄럼이
                # 진행 중이라는 뜻 — 낙하를 기다리지 말고 지금 내려놓고 다시 깊게
                # 잡는다. 통제된 재파지는 낙하 후 수습보다 성공률이 압도적이다.
                self._reseat_cnt = getattr(self, '_reseat_cnt', 0) + 1
                self.get_logger().warning(
                    f'물림 열화(면적 {la:.0f} < {HOLD_AREA_MIN:.0f}) — 조기 재안착 '
                    f'({self._reseat_cnt}/2)')
                if self._reseat():
                    continue
                self._carry_drop = True   # 재안착 실패 → 외부 복구 루프로
                return False
            # 스톨 감지: 회전·전진 명령에도 관측이 안 변하면(같은 r·brg 3연속) 정지
            # 마찰에 걸린 것 — 실측: wz 0.20은 dur×1.2 재계산(0.208)과 램프 하한을
            # 거치며 제자리에서 반복 소진(45회 전부 r=0.853/brg=23.0 동결). 강한
            # 펄스로 탈출한다.
            key = (round(r, 3), round(brg, 2))
            if key == getattr(self, '_carry_last', None):
                self._carry_stall = getattr(self, '_carry_stall', 0) + 1
            else:
                self._carry_stall = 0
            self._carry_last = key
            if self._carry_stall >= 3:
                self.get_logger().warning('운반 접근 스톨 — 강한 회전 펄스로 탈출')
                self.drive(0.0, 0.5 if brg > 0 else -0.5, 0.8)
                self._carry_stall = 0
                continue
            if abs(brg) > 0.25:
                # 0.20은 정지 마찰을 못 깬다(스톨 실측) — 0.35로 상향, 램프가 저크를 막는다
                self.drive(0.0, 0.35 if brg > 0 else -0.35, min(2.0, abs(brg) / 0.35))
            else:
                # 먼 구간은 빠르게 좁힌다. 0.06m/s 고정은 통까지 1.4m를 가기에
                # 너무 느려 — 램프까지 겹쳐 실효 8% — 70회 반복을 소진하고도
                # 67mm밖에 못 갔다(실측). 0.15m/s는 단독 측정에서 86% 달성.
                # 통 근처(0.5m 이내)에서는 다시 늦춰 물림을 흔들지 않는다.
                v = (0.15 if er > 0.5 else 0.06) if er > 0.15 else (0.03 if er > 0 else -0.03)
                wz = max(-0.12, min(0.12, brg * 0.7))     # 전진하며 방위도 함께 좁힌다
                self.drive(v, wz, min(3.0, abs(er) / abs(v) + 0.2))
        self.get_logger().error('쓰레기통 접근 반복 소진')
        return False

    def _lock_trash_pose(self, samples=5):
        """락을 걸기 직전에 통 좌표를 여러 번 재서 중앙값으로 확정한다.

        락 이후는 오도메트리 추측이라 락 좌표가 그대로 최종 정확도가 된다. 그런데
        단일 프레임으로 락을 걸면 그 순간이 나쁜 프레임일 수 있다 — 실측에서 락 시점
        검출 면적이 764로 앞뒤 프레임(2773~5098)의 1/5이었고, 접근 오차가 20mm까지
        수렴했는데도 큐브가 통 중심에서 206mm 벗어났다. 중앙값이 그 운을 없앤다.
        """
        xs, ys = [], []
        for _ in range(samples):
            s = self.locate_trash()
            if s is not None and self.odom is not None:
                x, y, yaw = self.odom
                xs.append(x + math.cos(yaw) * s[0] - math.sin(yaw) * s[1])
                ys.append(y + math.sin(yaw) * s[0] + math.cos(yaw) * s[1])
            time.sleep(0.15)
        if len(xs) < 3:
            self.get_logger().warning(f'락 좌표 재측정 실패({len(xs)}/{samples}) — 직전 값 유지')
            return
        xs.sort()
        ys.sort()
        mid = len(xs) // 2
        before = getattr(self, '_trash_odom', None)
        self._trash_odom = (xs[mid], ys[mid])
        if before:
            d = math.hypot(self._trash_odom[0] - before[0], self._trash_odom[1] - before[1])
            self.get_logger().info(
                f'락 좌표 확정: {len(xs)}회 중앙값, 직전 단일 프레임과 {d * 1000:.0f}mm 차이')

    def scan_step(self, it, turn=0.45, laps=1, moves=2):
        """대상을 못 찾았을 때의 탐색 한 걸음. 더 시도할 곳이 없으면 False.

        명령을 몇 번 보냈는지로 세면 안 된다. 물체를 든 채로는 주행 배율을 1로 낮춰
        회전 속도가 하한(0.25rad/s)까지 내려가고, 그 상태에서 회전이 실제로 일어나지
        않은 사례가 있다 — 검출 후보의 면적·높이가 세 번 연속 픽셀 단위로 동일했다
        (2026-07-30). 명령은 나갔는데 기체가 안 돈 것이다. 그래서 오도메트리로 실제
        회전량을 누적해 한 바퀴를 보장하고, 돌지 않으면 자리를 옮긴다.
        """
        self.spin_until(lambda: self.odom is not None, 3.0)
        yaw0 = self.odom[2] if self.odom else None
        self.get_logger().warning(
            f'[{it}] 미검출 — 회전 탐색 (누적 {math.degrees(getattr(self, "_scan_yaw", 0.0)):.0f}도'
            f'/{360 * laps}도, 자리이동 {getattr(self, "_scan_moves", 0)}/{moves})')
        self.drive(0.0, turn, 1.2)
        time.sleep(0.4)      # 회전 직후 흔들림이 가라앉은 뒤 관측
        self.spin_until(lambda: self.odom is not None, 3.0)
        # 실제로 돈 각도를 누적 (yaw는 ±π에서 감기므로 최단 차이로 환산)
        turned = 0.0
        if yaw0 is not None and self.odom is not None:
            turned = abs(math.atan2(math.sin(self.odom[2] - yaw0),
                                    math.cos(self.odom[2] - yaw0)))
        self._scan_yaw = getattr(self, '_scan_yaw', 0.0) + turned
        self._scan_stuck = getattr(self, '_scan_stuck', 0) + 1 if turned < 0.05 else 0
        if turned < 0.05:
            self.get_logger().warning(
                f'  회전 명령 {math.degrees(turn * 1.2):.0f}도인데 실제 '
                f'{math.degrees(turned):.1f}도 — 기체가 돌지 않았다')
        if self._scan_stuck >= 2 or self._scan_yaw >= 2 * math.pi * laps:
            # 한 바퀴를 다 봤거나 회전 자체가 막혔다 — 자리를 옮겨 시야를 바꾼다.
            self._scan_moves = getattr(self, '_scan_moves', 0) + 1
            if self._scan_moves > moves:
                return False
            reason = '회전 막힘' if self._scan_stuck >= 2 else '한 바퀴 완료'
            self.get_logger().warning(
                f'  {reason} — 자리 이동 {self._scan_moves}/{moves} (후진 25cm + 측면 회전)')
            self.drive(-0.12, 0.0, 2.0)
            self.drive(0.0, 0.6, 1.5)
            self._scan_yaw = 0.0
            self._scan_stuck = 0
        return True

    def to_pose_holding(self, target, steps=5):
        """물체를 쥔 채 목표 자세로 계단식 전환.

        한 번에 바꾸면 팔꿈치가 펴지는 관성으로 물체가 밀려 빠진다(실측). 들어올리기와
        같은 방식으로 잘게 나누고 단계마다 쥔 힘을 재주장한다.
        """
        cur = dict(getattr(self, '_last_arm', {j: 0.0 for j in ARM_JOINTS}))
        for i in range(1, steps + 1):
            t = i / steps
            pose = {j: cur.get(j, 0.0) + (target[j] - cur.get(j, 0.0)) * t for j in ARM_JOINTS}
            self.move_arm(pose, 1.0 * self.scale)     # 배율 무시: 실제 1초씩
            time.sleep(0.5)
            # 재주장하지 않는다 — 명령을 다시 보내면 죠가 움직여 물체가 덜렁거린다.
        return self.holding()

    def _front_target_heights(self, min_area=120):
        """전방 카메라에서 대상색 블롭들의 base_footprint 높이 목록 — 낙하 판정 제3신호.

        손목캠 면적은 죠 안에서 밀린 상태와 실제 낙하를 못 가른다(둘 다 임계 아래,
        실측 밀림 8103~11048 vs 바닥 낙하 6079). 물리 높이는 가른다: 바닥 z≈0.015
        vs 운반 중 죠 안 z≈0.19. roll과도 무관해 게이트 2의 남은 한계까지 커버한다.
        """
        self.color = self.depth = None
        if not self.spin_until(lambda: self.color is not None and self.depth is not None
                               and self.cam_info is not None, 3.0):
            return []
        hsv = cv2.cvtColor(self.color, cv2.COLOR_BGR2HSV)
        mask = None
        for lo, hi in self.hsv_ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        num, _, stats, cents = cv2.connectedComponentsWithStats(mask)
        zs = []
        for i in range(1, num):
            if stats[i, cv2.CC_STAT_AREA] < min_area:
                continue
            u, v = int(cents[i][0]), int(cents[i][1])
            if not (0 <= v < self.depth.shape[0] and 0 <= u < self.depth.shape[1]):
                continue
            d = float(self.depth[v, u])
            if not np.isfinite(d) or d <= 0.05:
                continue
            r = self._pixel_to_base(u, v, d, z_gate=(-0.05, 0.45))
            if r:
                zs.append(r[2])
        return zs

    def _holding_ik(self):
        """IK 경로의 파지 유지 판정 — **그리퍼 각도**로 본다.

        놓치면 죠가 끝까지 닫혀 -0.17이 되고, 쥐고 있으면 물체 두께만큼 벌어져
        있다. 25mm 큐브의 물림각은 0.05~0.2라 구분이 확실하다.

        전방캠 높이(종전 1차 신호)를 쓰지 않는 이유: 손목캠 서보 이후 팔 자세가
        달라져 큐브가 전방캠 시야를 벗어난다 — 실측에서 `면적 0 → DROPPED`로
        오판했는데 같은 순간 부하는 -10.0으로 제대로 물고 있었다.
        부하도 1차로 못 쓴다. 유지 목표에 도달하면 위치 오차가 0이 되어 부하가
        사라지기 때문이다(파지 '순간'에만 유효 — docs/18 참조).
        """
        self.gripper_angle = None
        self.spin_until(lambda: self.gripper_angle is not None, 3.0)
        ang = self.gripper_angle
        if ang is None:
            self.get_logger().info('파지 확인(각도): 각도 미수신 — 유지로 본다')
            return True
        held = ang > GRIP_EMPTY_MAX
        self.get_logger().info(
            f'파지 확인(각도): {ang:.3f} (빈손 기준 {GRIP_EMPTY_MAX}) '
            f'→ {"HOLDING" if held else "DROPPED"}')
        return held

    def holding(self, allow_active=True):
        """물체를 쥐고 있는지 판정 — 1차 신호는 그리퍼 관절 부하다.

        '완전 닫힘'을 계속 명령해 두므로, 물체가 막고 있는 동안은 위치 오차가 남아
        부하가 실리고 빠지는 순간 죠가 끝까지 닫혀 부하가 사라진다. 죠를 다시 움직일
        필요가 없고(능동 확인은 물체를 밀어낸다), 팔 자세·roll과도 무관하다.
        실측: 운반 중 -10.0 유지 / 큐브 이탈 즉시 -0.001.

        종전 1차 신호였던 손목캠 면적은 **거리를 재는 값이지 파지를 재는 값이 아니다.**
        바닥에 놓인 큐브가 로봇 코앞에 있으면 쥔 것과 같은 면적이 나온다
        (실측: 큐브가 바닥에 그대로인데 면적 42000, 전 구간 HOLDING 오판 → 가짜 성공).
        면적은 부하를 못 읽을 때의 폴백으로만 남긴다.
        """
        # IK로 잡았으면 각도로 판정한다(위 _holding_ik 주석 참조)
        if self.use_ik and getattr(self, '_ik_grasped', False):
            return self._holding_ik()
        self.gripper_angle = self.gripper_effort = None
        self.spin_until(lambda: self.gripper_angle is not None, 3.0)
        ang = self.gripper_angle
        # 1차 신호: 큐브의 실제 높이. 쥐고 있으면 운반 자세 높이(z≈0.19)에, 놓치면
        # 바닥(z≈0.015)에 있다 — 물리량을 직접 재므로 자기충족이 없다.
        # 부하는 여기서 쓰지 않는다: 유지 목표에 도달하면 위치 오차가 0이 되어 부하도
        # 0이 되기 때문이다(파지 '순간'에만 유효한 신호다).
        zs = self._front_target_heights()
        if zs:
            self._last_hold_area = None
            if any(z > 0.10 for z in zs):
                self.get_logger().info(
                    f'파지 확인(높이): {[round(z, 3) for z in zs]} → HOLDING')
                return True
            if all(z < 0.06 for z in zs):
                self.get_logger().info(
                    f'파지 확인(높이): {[round(z, 3) for z in zs]} 바닥 → DROPPED')
                return False
        roll = float(getattr(self, '_last_arm', {}).get('arm_wrist_roll', 0.0) or 0.0)
        if abs(roll) >= 0.1:
            # 기울기 정렬로 손목이 돌아간 상태에서는 큐브가 손목캠 시야를 벗어난다
            # (실측: roll 0.33에서 면적 0인데 실제로는 쥐고 있었다 — 큐브 z=0.19).
            # 이때는 각도 구간으로 폴백한다. 각도는 낙하를 못 잡을 수 있으므로
            # 기울어진 물체에서는 판정 신뢰도가 낮다는 한계가 남는다.
            self._last_hold_area = None   # 시각 신호 없음 — 재안착 게이트 비활성
            held = self.gripped(ang)
            self.get_logger().info(
                f'파지 확인(각도 폴백, roll={roll:+.2f}): '
                f'각도={ang if ang is None else round(ang, 3)} → {"HOLDING" if held else "DROPPED"}')
            return held
        b = self._wrist_blob(frames=3)
        area = b[2] if b else 0.0
        if area < HOLD_AREA_MIN:
            # 회전·가감속 직후에는 쥔 채로도 흔들림·블러로 면적이 일시 급감할 수 있다.
            # 정지 안정화 후 한 번 재확인해 일시 급감과 실제 낙하를 가른다
            # (실제 낙하는 재확인에서도 낮게 유지되므로 감지력 손실 없음).
            time.sleep(0.8)
            b = self._wrist_blob(frames=3)
            area2 = b[2] if b else 0.0
            self.get_logger().info(f'파지 재확인(안정화 후): 면적 {area:.0f} → {area2:.0f}')
            area = max(area, area2)
        self._last_hold_area = area   # 재안착 게이트용 열화 신호
        if area < HOLD_AREA_MIN:
            # 제3신호: 전방 카메라 블롭 높이. 죠 안에서 밀린 큐브(운반 높이 z>0.10)와
            # 바닥에 떨어진 큐브(z<0.06)를 물리 높이로 확정한다. 실측: 회전 운반에서
            # 면적이 임계 아래로 내려간 두 번 모두 실좌표 z=0.19~0.21로 쥔 상태였다.
            zs = self._front_target_heights()
            floor_hit = any(z < 0.06 for z in zs)
            carry_hit = any(z > 0.10 for z in zs)
            zs_txt = '[' + ', '.join(f'{z:.3f}' for z in zs) + ']'
            if floor_hit:
                self.get_logger().info(f'파지 확인(제3신호): 바닥 높이 블롭 {zs_txt} → DROPPED')
                return False
            if carry_hit:
                self.get_logger().warning(
                    f'파지 확인(제3신호): 운반 높이 블롭 {zs_txt} — 죠 안에서 밀림, HOLDING 유지')
                return True
            if allow_active:
                # 능동 판별 직후 유예: 매 반복 후진하면 통에서 멀어지며(실측: r 0.94까지
                # 후퇴) 후진 저크가 물림을 더 흔들어 진짜 낙하를 유발하는 악순환이 된다.
                # HOLDING 확정 후 12초간은 저면적을 "밀린-쥔 상태"로 간주하고 유지한다.
                if time.time() < getattr(self, '_hold_grace', 0.0):
                    self.get_logger().info(
                        f'파지 확인(유예 중): 면적 {area:.0f} — 직전 능동 판별 HOLDING 신뢰')
                    return True
                # 능동 판별(운반 중 한정): 10cm 저속 후진 후 손목캠 블롭 중심 변위 비교.
                # 쥔 큐브는 로봇과 강체 결합이라 화면이 불변이고, 떨어진 큐브는 뒤로
                # 멀어져 중심이 크게 이동한다(실측: 낙하 상태 10cm 후진에 x 314→563,
                # 249px 이동 / 쥔 상태 75px. 밀린-쥔 상태와 낙하 직후는 정적 화면이
                # 완전 동일해 — area 6157/중심까지 일치 실측 — 수동 신호로는 원리적으로
                # 구분 불가). 후진은 통·물체에서 멀어지는 방향이라 안전하고, 저속
                # (0.06m/s)이라 물림을 흔들지 않으며, 운반 폐루프가 위치를 다시 보정한다.
                b0 = self._wrist_blob(frames=3)
                self.drive(-0.05, 0.0, 2.0)
                b1 = self._wrist_blob(frames=3)
                if b0 is not None and b1 is not None:
                    shift = math.hypot(b1[0] - b0[0], b1[1] - b0[1])
                    # 임계 190px: 4cm 큐브 기준은 130px였다(쥔 75 vs 낙하 249).
                    # 3cm로 줄이자 블롭이 작아져 중심 추정이 더 흔들린다 — 쥔 상태
                    # 실측이 136px까지 올라가 130 임계를 넘겨 낙하로 오판했다.
                    held2 = shift < 130.0
                    if held2:
                        self._hold_grace = time.time() + 12.0
                    self.get_logger().info(
                        f'파지 확인(능동 판별): 10cm 후진에 블롭 중심 {shift:.0f}px 이동 '
                        f'→ {"HOLDING(강체 결합)" if held2 else "DROPPED(뒤로 멀어짐)"}')
                    return held2
                self.get_logger().info('파지 확인(능동 판별): 블롭 소실 → DROPPED')
                return False
            self.get_logger().info(f'파지 확인(제3신호 무소득 {zs_txt}): 면적 기준 DROPPED')
            return False
        held = area >= HOLD_AREA_MIN
        self.get_logger().info(
            f'파지 확인: 손목캠 면적={area:.0f} (임계 {HOLD_AREA_MIN:.0f}) '
            f'각도={ang if ang is None else round(ang, 3)} → {"HOLDING" if held else "DROPPED"}')
        return held

    def _reseat(self):
        """조기 재안착: 쥔 큐브를 정면 바닥에 내려놓고 재접근·재파지한다.

        하강(파지 자세, pan 0) → 개방 → recover_dropped()의 검증된 재파지
        파이프라인 재사용. 통 기억 좌표는 유지되므로 운반이 이어진다.
        """
        pose = {**POSE_GRASP_FLOOR, 'arm_shoulder_pan': 0.0}
        self.move_arm(pose, 2.5)
        self.wait_arm_settled(pose, timeout=6.0)
        self.move_gripper(1.0)
        time.sleep(0.5)
        return self.recover_dropped()

    def recover_dropped(self):
        """운반 중 낙하 복구: 떨어진 큐브를 재접근·재파지하고 들기까지 마친다.

        능동 판별이 낙하를 확정한 시점의 큐브는 로봇 바로 앞 바닥에 있다(판별
        후진 10cm 포함 실측 0.3~0.5m). 뎁스 최소거리(0.30m) 밖으로 물러나 시야를
        확보한 뒤 기존 접근·파지 파이프라인을 그대로 재사용한다. 통 기억 좌표
        (_trash_odom)는 월드 고정이므로 유지하고, 근접 락만 풀어 재검출을 허용한다.
        """
        self._carry_drop = False
        self._trash_lock = False
        self._hold_grace = 0.0
        # 떨어진 큐브는 반드시 바닥에 있다 — 높이 게이트를 바닥 전용으로 조여
        # 통 몸통 오검출(z≈0.14 유령 표적)이 접근을 홀리는 것을 차단한다.
        self._floor_only = True
        # main()의 검증된 패턴대로 후진·재접근을 2회 재시도.
        for attempt in range(2):
            self.move_gripper(0.5)
            self.move_arm(POSE_FOLDED, 2.5)
            self.drive(-0.12, 0.0, 2.5)     # 0.3m 후진 — 큐브를 뎁스 최소거리 밖으로
            for attr in ('_reacq_cnt', '_arm_aside', '_miss', '_obj_odom', '_target_px'):
                if hasattr(self, attr):
                    delattr(self, attr)
            pan = self.approach()
            if pan is None:
                self.get_logger().warning(f'복구 재접근 실패 (시도 {attempt + 1}/2)')
                continue
            if self.grasp(pan) and self.lift():
                return True
            self.get_logger().warning(f'복구 재파지 불성립 (시도 {attempt + 1}/2) — 재접근')
        self.get_logger().error('복구 실패')
        return False

    def drop_into_trash(self):
        """통 개구부 위에서 그리퍼를 열어 투입."""
        time.sleep(0.5)
        # 개구부 코앞이라 능동 판별(10cm 후진)은 투하 위치를 망친다 — 수동 신호만
        if not self.holding(allow_active=False):
            # 여기서 중단하지 않는다 — 이미 개구부 앞이라 투하를 진행해도 잃을 게 없다
            # (쥐고 있으면 투입 성공, 아니면 어차피 실패). 신호가 전부 사각인 상태
            # (전방캠 무소득 + 손목캠 시야 이탈)의 오판으로 중단했는데 실좌표는 통 안
            # 44mm — 쥔 채였던 사례가 실측됐다. 아래 팔 뻗기 주석과 같은 철학.
            self.get_logger().warning('투입 직전 파지 확인 실패 — 개구부 앞이므로 투하는 진행')
        # 통 중심까지 팔을 뻗는다. 이 자세는 그리퍼가 기울어 물체가 스스로 빠질 수 있는데,
        # 이미 개구부 위이므로 그것도 투입이다. 그래서 파지 유지를 확인하지 않는다.
        self.get_logger().info('통 중심으로 팔 뻗기')
        self.move_arm(POSE_DROP, 2.0 * self.scale)
        time.sleep(0.8)
        self.get_logger().info('쓰레기통 위 — 그리퍼 열기')
        self.move_gripper(1.0)
        time.sleep(1.2)
        self.move_arm(POSE_FOLDED, 3.0)     # 팔을 접어 통에서 빠져나옴
        time.sleep(0.5)
        return True

    def place(self, pan_target=-0.7):
        """잡은 물체를 옆(pan_target 방향)에 내려놓고 팔을 접는다."""
        grasp_pose = POSE_GRASP_FLOOR if self.floor_mode else POSE_GRASP
        self.move_arm({'arm_shoulder_pan': pan_target}, 2.5)
        time.sleep(0.3)
        self.move_arm({**grasp_pose, 'arm_shoulder_pan': pan_target}, 2.5)
        time.sleep(0.3)
        self.move_gripper(0.8)
        time.sleep(0.6)
        self.move_arm({**POSE_PRE, 'arm_shoulder_pan': pan_target}, 2.0)
        self.move_arm(POSE_FOLDED, 2.5)
        return True

    def lift(self):
        """쥔 물체를 들어올린다. 다 든 뒤에도 쥐고 있으면 True.

        한 번에 올리지 않고 잘게 나누는 데 두 가지 이유가 있다.

        하나는 미끄러짐이다. **물체가 바닥을 떠나는 첫 순간에 하중이 걸려 가장 잘
        빠진다.** 그래서 초반 두 단계만 0.02씩 미세하게 올리고 이후 0.07로 키운다.

        다른 하나는 그리퍼 각도 유지다. 어깨만 올리면 손목이 함께 기울어 물체가
        죠 안에서 굴러 나온다. 그래서 **어깨를 내린 만큼 손목을 올려** 그리퍼의
        절대 기울기를 보존한다(lift 감소분 = wrist 증가분). 파일 상단 자세 상수에서
        말하는 '자세족 불변식(lift+elbow+wrist≈1.58)'이 이것이다.

        단계마다 쥔 힘을 다시 주장하는데, 파지 때 정한 절대 목표를 그대로 반복한다.
        값이 같으므로 죠는 움직이지 않고, 하중에 밀려 벌어졌을 때만 되돌아온다.
        '현재각 기준'으로 다시 조이면 조일 때마다 목표가 깊어지는 래칫이 되어
        물체를 파고든다 — 과거에 실제로 그랬다.
        """
        # 계단식 들기 + wrist 보상(lift 감소분 = wrist 증가분): 그리퍼 절대 피치 유지.
        # 시작 자세는 모드의 파지 자세 — 받침대(0.48/0.9), 바닥(1.15/0.28) 공용.
        if self.floor_mode and self.use_ik and getattr(self, '_ik_grasped', False):
            return self._lift_ik()
        grasp_pose = POSE_GRASP_FLOOR if self.floor_mode else POSE_GRASP
        lift0 = grasp_pose['arm_shoulder_lift']
        elbow0 = grasp_pose['arm_elbow_flex']
        wrist0 = grasp_pose['arm_wrist_flex']
        # 파지 중 팔로 뻗은 누적 리치(r0)를 계단마다 얹었다가 서서히 0으로 뺀다.
        #   · 처음 두 계단은 그대로 유지 — 물체가 바닥을 떠나는 순간에 자세가 급변하면
        #     그때가 가장 잘 빠진다. 명목 자세로 한 번에 되돌리면 죠가 최대 23° 기운다.
        #   · 그 뒤로는 되돌린다 — 뻗은 채로 손목 보상(lift 감소분=wrist 증가분)을
        #     계속하면 wrist_flex가 한계 1.658을 넘는다. 실측(리치 76mm): 목표 1.953이
        #     한계에 잘려 추종오차가 0.109→0.293rad로 커졌고, 죠가 기운 채 운반하다
        #     큐브를 놓쳤다. 끝 자세는 리치 0 = POSE_CARRY와 정확히 같아진다.
        r0 = getattr(self, '_reach_applied', 0.0) or 0.0
        # 물체가 바닥을 떠나는 첫 순간에 하중이 걸려 가장 잘 미끄러진다 — 초반 두 단계를
        # 잘게 올려 대응한다. 그리퍼는 파지 때 정한 목표를 그대로 유지한다(재조임 없음).
        lf, k = lift0, 0
        while lf > 0.151:
            step = 0.02 if k < 2 else 0.07      # 초반만 미세하게
            lf = max(0.15, lf - step)
            wf = wrist0 + (lift0 - lf)
            r = r0 if k < 2 else max(0.0, r0 * (1.0 - (k - 1) / 4.0))
            # elbow를 명시적으로 준다. 빼면 move_arm이 직전 명령(리치가 섞인 값)을
            # 그대로 물려받아 lift·wrist만 명목으로 점프하고 자세족 불변식이 깨진다.
            self.move_arm({
                'arm_shoulder_lift': lf + REACH_LIFT_PER_M * r,
                'arm_elbow_flex': elbow0 + REACH_ELBOW_PER_M * r,
                'arm_wrist_flex': wf - (REACH_LIFT_PER_M + REACH_ELBOW_PER_M) * r,
            }, 1.5)
            time.sleep(0.2)
            # 파지 때 정한 절대 목표를 그대로 다시 주장한다. 값이 같으므로 죠는
            # 움직이지 않고(덜렁거림 없음), 하중에 밀려 벌어졌을 때만 되돌아온다.
            # 과거 파고듦의 원인은 "현재각 기준"으로 다시 조이던 래칫이었고,
            # 절대 목표 반복은 그 문제가 없다.
            # 완료를 기다리지 않는다 — 계단 수만큼 액션 왕복을 기다리면 전체 실행이
            # 400초 예산을 넘겨 중단됐다(실측). 유지 명령은 도착만 하면 된다.
            # 파지 때 정한 절대 목표를 그대로 반복한다. 값이 같으므로 죠는 더 닫히지
            # 않는다 — 완전 닫힘을 반복하면 계단마다 조금씩 파고들어 큐브가 빠진다.
            self.move_gripper(getattr(self, 'hold_target', GRIPPER_CLOSED),
                              wait=False, effort=30.0)
            self.gripper_angle = self.gripper_effort = None
            self.spin_until(lambda: self.gripper_angle is not None, 3.0)
            self.get_logger().info(
                f'  step lift={lf:.2f}: 각도={self.gripper_angle:.3f} '
                f'부하={self.gripper_effort if self.gripper_effort is None else round(self.gripper_effort, 2)}')
            k += 1
        time.sleep(0.5)
        # 손목 roll을 원위치로 돌린다. 기울기 정렬로 돌아간 상태에서는 큐브가 손목캠
        # 시야를 벗어나 낙하 판정이 각도 폴백으로 떨어지고, 그 폴백이 정상 파지를
        # 실패로 오판했다(실측: 각도 0.31이 상한 0.30을 넘어 DROPPED, 실제로는 쥐고
        # 있었다). 쥐고 있으면 큐브가 함께 회전하므로 안전하고, 운반에 roll은 필요 없다.
        roll_now = float(getattr(self, '_last_arm', {}).get('arm_wrist_roll', 0.0) or 0.0)
        if abs(roll_now) > 0.1:
            self.get_logger().info(f'손목 롤 원위치 ({roll_now:+.2f} → 0.00) — 파지 확인용')
            self.move_arm({'arm_wrist_roll': 0.0}, 1.5)
            self.wait_arm_settled({**self._last_arm, 'arm_wrist_roll': 0.0}, timeout=5.0)
            time.sleep(0.3)
        still = self.holding()
        self.get_logger().info(f'들기 후 재검증 → {"HOLDING" if still else "DROPPED"}')
        return still


def main(args=None):
    """전체 시퀀스. 종료 코드 0이면 성공, 1이면 실패 (수집기가 이 값으로 채점한다).

    큰 흐름은 이렇다.

        1. 초기화       팔을 접힘 자세로, 그리퍼 닫기
        2. 접근         approach()      ─┐
        3. 파지         grasp()          │ 실패하면 후진 후 이 셋을 다시 (최대 3사이클)
        4. 들기         lift()          ─┘
        5. 놓기         place() 또는 carry_to_trash() + drop_into_trash()

    재시도가 두 겹이다. 바깥 3사이클은 접근·파지 실패를 다시 태우고, 안쪽
    3회(운반 중 낙하)는 recover_dropped()로 떨어진 큐브를 다시 집는다.
    재시도 전에 `_miss`, `_obj_odom`, `_target_px` 같은 상태를 지우는 것이 중요한데,
    남겨 두면 이전 사이클의 잘못된 기억을 그대로 믿고 같은 실패를 반복한다.

    실패 사유를 그리퍼 각도로 분류하는 부분(마지막)이 디버깅에 쓸모 있다.
        각도 ≥ 0.30   얕은 걸침 — 물체가 죠 사이에 못 들어오고 위에 올라탔다
        각도 ≤ -0.12  완전 닫힘 — 허공을 집었다 (위치가 틀렸다)
        그 사이        모서리 헛집기 또는 미닫힘
    같은 '파지 실패'라도 원인과 고칠 곳이 다르다. 앞의 것은 전진량(creep),
    뒤의 것은 정렬 문제다.

    판정에 last_grasp_angle을 쓰는 이유: grasp()가 실패 후 죠를 다시 열어 두므로
    지금 관절값을 읽으면 열림값이라 아무것도 알 수 없다.
    """
    rclpy.init(args=args)
    n = PickNode()
    ok = False
    try:
        n.get_logger().info('== 1. 초기화: 접힘 자세 ==')
        n.move_gripper(0.0)
        n.move_arm(POSE_FOLDED, 3.0)
        n._trace('시작')
        # 파지 실패 시 자동 재시도: 후진해 비전 재획득 후 재접근 (맹주행 드리프트 회복)
        for cycle in range(3):
            if n.skip_approach and cycle == 0:
                n.get_logger().info('== 2. 접근 생략 (물체 앞 가정, pan=0) ==')
                pan = 0.0
                n.spin_until(lambda: n.odom is not None, 5.0)
                n._anchor_odom = n.odom
            else:
                n.get_logger().info(f'== 2. 비전 접근 주행 (사이클 {cycle + 1}) ==')
                pan = n.approach()
            n._trace('접근 종료', cycle=cycle + 1, pan=None if pan is None else round(pan, 3))
            if pan is None:
                n.get_logger().error('접근 실패')
                if cycle < 2:
                    for attr in ('_reacq_cnt', '_arm_aside', '_miss', '_obj_odom', '_target_px'):
                        if hasattr(n, attr):
                            delattr(n, attr)
                    n.move_arm(POSE_FOLDED, 2.5)
                    continue
                break
            n.get_logger().info(f'== 3. 파지 (pan={math.degrees(pan):.1f}deg) ==')
            held = n.grasp(pan)
            n._trace('파지 종료', 판정=('HOLDING' if held else 'EMPTY'),
                     reach=round(getattr(n, '_reach_applied', 0.0) or 0.0, 3))
            if held:
                n.get_logger().info('== 4. 들어올리기 ==')
                ok = n.lift()
                n._trace('들기 종료', 판정=('HOLDING' if ok else 'DROPPED'))
                if ok and n.place_target == 'trash':
                    n.get_logger().info('== 5. 쓰레기통으로 운반 ==')
                    # 운반 중 실낙하(능동 판별 확정)는 재파지로 복구한다 — 큐브가
                    # 근처 바닥에 있고 접근·파지 파이프라인이 그대로 쓰인다.
                    ok = False
                    for rec in range(3):
                        if n.carry_to_trash() and n.drop_into_trash():
                            ok = True
                            n._trace('투입 종료', 판정='성공주장')
                            break
                        n._trace('투입 실패', rec=rec + 1)
                        if not getattr(n, '_carry_drop', False):
                            break   # 탐색 실패 등 낙하 외 실패는 재파지로 못 고친다
                        n.get_logger().info(f'== 5-복구. 운반 중 낙하 — 재파지 (시도 {rec + 1}/2) ==')
                        if not n.recover_dropped():
                            break
                elif ok:
                    n.get_logger().info('== 5. 옮겨 놓기 (place) ==')
                    n.place()
                break
            # 실패 양상 구분 — 판정 시점의 각도(last_grasp_angle)로. 현재 joint값은
            # grasp()가 실패 후 죠를 다시 열어 두므로(0.5/1.0) 열림값이라 쓸 수 없다.
            a = getattr(n, 'last_grasp_angle', None)
            fail = getattr(n, 'last_grasp_fail', None)
            if fail:
                # 닫기 전에 빠져나온 경우 — 각도로 분류하면 거짓 진단이 된다
                n.get_logger().error(f'파지 실패 ({fail} — 죠를 닫지 않았다)')
            elif a is None:
                n.get_logger().error('파지 실패 (각도 미수신)')
            elif a >= GRIP_HOLD_MAX:
                n.get_logger().error(f'파지 실패 (얕은 걸침 각도={a:.3f} — 물체가 죠 사이에 못 들어옴)')
            elif a <= -0.12:
                n.get_logger().error(f'파지 실패 (완전 닫힘 = 허공, 각도={a:.3f})')
            else:
                n.get_logger().error(f'파지 실패 (모서리 헛집기/미닫힘 구간, 각도={a:.3f})')
            if cycle < 2:
                n.get_logger().info('재시도: 그리퍼 열고 후진 → 재접근')
                n.move_gripper(0.5)
                n.move_arm(POSE_FOLDED, 2.5)
                n.drive(-0.12, 0.0, 3.0)
                for attr in ('_reacq_cnt', '_arm_aside', '_miss', '_obj_odom', '_target_px'):
                    if hasattr(n, attr):
                        delattr(n, attr)
        n.get_logger().info('=== PICK_SUCCESS ===' if ok else '=== PICK_FAIL ===')
        # 마지막 줄에 '코드의 주장'과 '실좌표'를 나란히 남긴다. 이 둘이 어긋나면
        # 판정 신호가 틀린 것이고, 그때는 성공률 숫자 자체를 믿으면 안 된다.
        n._trace('종료', 코드주장=('PICK_SUCCESS' if ok else 'PICK_FAIL'))
    finally:
        n.destroy_node()
        if rclpy.ok():      # 중단 신호로 이미 내려간 뒤 다시 부르면 RCLError가 난다
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
