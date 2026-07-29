# jdamr_cube_ros — 캡스톤 포크

이 저장소는 [perpet99/jdamr_cube_ros](https://github.com/perpet99/jdamr_cube_ros)의 **포크**입니다.
JD-AMR cube(차동구동 + LD14 라이다 + RGB-D)와 SO-101 5축 팔을 ROS 2 Jazzy / Gazebo Harmonic에서 운용합니다.

## 이 포크에서 내가 만든 것

`git diff --stat upstream/main..main` 기준입니다.

| 범위 | 내용 |
|---|---|
| **`capstone_pick/` 패키지 전체** | 비전 기반 자율 픽앤플레이스 파이프라인. `pick_node.py`(주 노드)·`pick_ui.py`(관제 대시보드)·무대 스크립트 5종·YOLOv8n 파인튜닝 가중치 |
| `jdamr_cube_description/urdf/` | **그리퍼 충돌 형상 정밀화** — STL 정점 파싱으로 경계 상자를 산출하고 손가락을 tip/mid/base 3박스로 근사. RGB-D 카메라 각도 조정 |
| `jdamr_cube_gazebo/` | headless GPU 실행 옵션·리소스 경로·월드 배치 |

커밋 22개 전부 `capstone_pick`·`description`·`gazebo` 범위입니다.

## 기반 리포에서 받은 것 (내가 만들지 않음)

혼동을 막기 위해 명시합니다.

| 패키지 | 내용 |
|---|---|
| `jdamr_cube_navigation` | **Nav2 스택** — `nav2_params.yaml`(AMCL·MPPI 컨트롤러·NavfnPlanner·costmap), `navigation.launch.py`, `goto_pose.py` |
| `jdamr_cube_cartographer` | 2D SLAM |
| `jdamr_cube_description` | 기체 URDF 원본 (그리퍼 충돌 형상만 수정) |
| `jdamr_cube_moveit_config` | MoveIt 설정 |
| `jdamr_cube_bringup` · `jdamr_cube_node` · `jdamr_cube_teleop` | 기체 구동 기반 |

## 구현 기록

문제와 해결 과정은 별도 문서 저장소에 있습니다 — [gazebo-so101-capstone](https://github.com/mmporong/gazebo-so101-capstone). 환경 구축 · 파지 물리 · 비전 좌표계 · 손목캠 정렬 · 바닥 파지 · YOLO 전환 · 초기 자세 · 대시보드 · 디버깅 노트 9편.

## 측정된 결과

- 비전 접근 수렴 오차 **3~6mm** (초기 거리 1m)
- YOLO 검출 신뢰도 0.79~0.94 (3색, 0.5~1.1m), **mAP50 0.98**
- 운반 검증 물체 고도 0.182m 실측
- 전체 사이클 약 30초 (4배속, 주행 1m 포함)

## 실행

```bash
~/capstone_tools/start_stack.sh
python3 ~/capstone_tools/tricolor_stage.py
ros2 run capstone_pick pick --ros-args -p target_color:=green -p speed_scale:=4.0
```

주요 파라미터: `target_color`(blue/red/green/orange/pink) · `speed_scale`(0.5~10) · `detector`(yolo/hsv) · `floor`(바닥 모드 강제) · `skip_approach`(파지만 시험).
