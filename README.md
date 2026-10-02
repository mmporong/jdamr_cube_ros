# jdamr_cube_ros

차동구동 AMR의 실기 브링업부터 저장 지도 기반 자율주행, 실시간 장애물 대응, 식당 서빙형
목적지 왕복과 박스 정밀 주차까지를 ROS 2 Jazzy로 구현하고 실차 기록으로 검증하는 저장소다.
[perpet99/jdamr_cube_ros](https://github.com/perpet99/jdamr_cube_ros)의 포크이며, 캡스톤
모바일 매니퓰레이션(SO-101 팔 픽앤플레이스)에서 출발했다.

- 플랫폼: Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · Nav2 · Cartographer
- 로봇: Raspberry Pi 4 + ESP32 제어보드 + YDLIDAR G4 2D LiDAR(2026-08-25 LD14P에서 교체) + Orbbec Astra S RGB-D
- 차체: 2026-09-15부터 양팔 서빙 로봇 하단 프레임(340×450mm, 바퀴 외폭 540mm)으로 교체.
  제원 원본은 [`new_base_geometry.yaml`](jdamr_cube_description/config/new_base_geometry.yaml)

## 현재 상태

| 트랙 | 상태 | 확인된 결과 | 상세 |
|---|---|---|---|
| 실기 브링업·펌웨어 | 완료 | 강사 원본 결함 43건 감사, ESP32 펌웨어 v2와 C++ 베이스 드라이버로 대체. 바퀴·트레드·라이다 위치는 실측값 | [README_JDAMR.md](README_JDAMR.md) |
| 저장 지도 복도 자율주행 | 완료 | 2026-09-04 77m 복도 왕복 20/20 waypoint, recovery 0회. DDS를 `LOCALHOST`로 격리해 LiDAR 최대 공백 10.750s → 0.112s | [README_SLAM_PORTFOLIO.md](README_SLAM_PORTFOLIO.md) |
| 2D SLAM 백엔드 비교 | 완료 | 같은 MCAP 재생에서 Cartographer 시작–종료 불일치 0.996m, SLAM Toolbox 9.139m. 복도 기본 백엔드를 Cartographer로 선택 | [README_SLAM_PORTFOLIO.md](README_SLAM_PORTFOLIO.md) |
| Keepout·실시간 장애물 | 완료 | 2026-09-10 저장 지도·Keepout·장애물 대응을 묶은 왕복 20/20. Collision Monitor StopZone 6회 뒤 같은 goal 재개 | [20260909_REAL_COMBINED_TRIAL.md](jdamr_cube_navigation/evaluation/20260909_REAL_COMBINED_TRIAL.md) |
| 새 차체 재방문 | 부분 완료 | 2026-09-16 기존 지도로 20/20 목표 완료, recovery 0회. 오도메트리 보정값과 새 지도·Keepout은 `candidate` 단계 | [NEW_BASE_REVISIT_CAPTURE.md](jdamr_cube_navigation/evaluation/NEW_BASE_REVISIT_CAPTURE.md) |
| 시뮬레이션·디지털 트윈 | 완료 | 실차 점유격자를 Gazebo 충돌 메시로 바꿔 20 waypoint와 장애물 장면 3건을 재실행 | [2.5D 디지털 트윈](jdamr_cube_navigation/evaluation/20260912_2_5D_DIGITAL_TWIN_PORTFOLIO_HANDOFF.md) |
| RGB-D Visual SLAM | 진행 중 | 입력·visual odometry·RTAB-Map DB 생성은 검증. 저텍스처 P턴에서 연속 3D 지도는 아직 통과하지 못함 | [jdamr_cube_vslam/README.md](jdamr_cube_vslam/README.md) |
| 식당 서빙·박스 정밀 주차 | 진행 중 | 충전소 출발 → 테이블 관측 위치 → 박스 면 정렬 → 5cm 접근 → 2 s 대기 → 박스 이탈 후진 → 충전소 후면 주차를 한 실행기(`box_service --return-home`)로 연결. 2026-09-29 실차에서 관측 위치까지 이동했으나 도착 방향이 강제되지 않아 테이블 표지가 카메라 시야 밖이었고, 안정된 박스 앞면을 확보하지 못함. 2026-09-30 입력 공백 회복·정지 위치추정 판정·계획 끝점 검사·박스 이탈 복귀를 구현해 테스트와 파이 반영까지 마침. 같은 날 실차에서 충전소 후진 도킹(도크 앞 0.7 m 정렬 → 직선 후진)으로 도크 정위치에 도달(현장 확인, 끝 구간 yaw 보정 중 Nav2 105로 서비스 판정은 실패). table_01 박스는 관측·회전 공간이 부족한 자리여서 면 정렬 단계에 들어가지 못함. 같은 날 새 지도(벽 정렬)와 목적지(물 받는 곳·테이블 2곳)를 다시 정하고, 도크 → 물 받는 곳 → 호출 테이블 → 도크를 한 실행으로 잇는 경유 정지를 구현·파이 반영. 실차에서 물 받는 곳·table_02 5cm 정밀 주차와 5 s 대기, 도크 후면 도킹까지 도달했다. table_02 구간은 odom 고정 직선 최종 접근·후진으로 주차(중앙 5.42 cm, 0.87°)·대기·이탈·도킹을 한 실행으로 마쳤다(내부 추정, 외부 실측 없음). 2026-10-01 오전 실차에서 물 받는 곳(간격 5.42 cm, 0.04°)과 table_02(5.53 cm, 1.48°) 정밀 주차·대기·이탈, 도크 대기점 한 방향 회전, 후면 도킹을 모두 지났다(수정 배포로 실행은 세 번에 나뉨, 내부 추정). 현재 파이 반영본은 최종 접근·도크 후진을 RPP로 추종하고, 45°가 넘는 방향 전환은 제자리 회전, 도크 대기점은 위치만 맞춘 뒤 odom 기준 한 방향 회전으로 정렬하고, 옆 어긋남은 도크 앞 0.25 m까지 3차 곡선 후진(회전 반경 0.3 m 이상)으로 없앤 뒤 도크 밖에서 방향을 1.5° 안으로 맞춰 직선 후진으로 도킹하며(끝 방향이 3–5°면 그 자리에서 회전, 5°를 넘으면 빠져나와 한 번 다시 진입), 이동 구간 경유점은 NavigateThroughPoses 한 목표로 잇고, 박스 실행기는 Nav2 세션과 함께 상주한다. 이동 0.12 m/s·정밀 주차와 후진 0.08 m/s(감속은 마지막 0.15 m)다. RPP의 예측 충돌 검사는 끄고 차체 외곽 기준 StopZone 정지를 남겼다. 돌발상황은 현업 조사([20261002_ANOMALY_HANDLING_RESEARCH.md](jdamr_cube_navigation/evaluation/20261002_ANOMALY_HANDLING_RESEARCH.md))에 따라 드라이버 래치 비상정지(리셋 서비스로만 해제), 막힘 20 s 대기 후 재시도 3회, 실패 시 등급별 운영자 호출(PC 알림), 운영자 연결 하트비트 30 s(끊기면 다음 목표를 시작하지 않음), 정차 사이 저전압 도크 복귀로 처리한다. 물리 비상정지·범퍼는 아직 없다. 전역 경로는 이동 구간 NavFn, 도크 대기점 구간 Smac State Lattice로 나눠 쓴다(이 브랜치, 파이 실세션 계획 비교로 결정). 2026-10-01 11:52 실차에서 도크 → 물 받는 곳 → table_02 → 도크 한 사이클을 한 실행으로 186.5 s에 마쳤다(물 받는 곳 5.45 cm·1.86°, table_02 5.54 cm·1.47°, 도킹 4.24 cm·1.19°, 내부 추정). 포트폴리오 데이터는 [20261001_PORTFOLIO_HANDOFF_FOR_CODEX.md](jdamr_cube_navigation/evaluation/20261001_PORTFOLIO_HANDOFF_FOR_CODEX.md). 주행 중 반복 중단 원인(정밀 구간 감속, 탐색 회전 확인, 공분산 정지, 방향 허용, 이탈 확인, 새 프로세스 DDS 수신)은 조치해 파이에 반영 | [20260929_PARKING_FAILURES.md](jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md) |
| 캡스톤 픽앤플레이스 (시뮬) | 완료 | 비전 접근 수렴 오차 3~6mm, YOLO mAP50 0.98, 사이클 약 30초(4배속). 수치의 정본은 구현 기록 저장소 | [capstone_pick/](capstone_pick), [gazebo-so101-capstone](https://github.com/mmporong/gazebo-so101-capstone) |

AMCL은 외부 ground truth가 아니다. 이 저장소의 정렬 RMS·복귀 오차는 ATE나 절대 정확도가
아니며, 시뮬레이션 결과를 실차 성능으로 적지 않는다.

## 시스템 구성

```text
YDLIDAR G4 ──┐
Astra S RGB-D ┼─ Raspberry Pi 4 (ROS 2 domain 12)
ESP32 펌웨어 ─┘    ├─ jdamr_base_driver: cmd_vel ↔ UART, 50Hz odom
                    ├─ AMCL + 저장 지도 + Keepout 필터
                    ├─ Nav2 (planner·controller·BT) → velocity smoother
                    └─ Collision Monitor → /cmd_vel (최종 속도 감독)

노트북: 기록(MCAP)·RViz(SSH 표시 중계)·재생·평가·미디어 생성 (제어 경로에는 참여하지 않음)
```

로봇 제어 그래프는 파이 안에서 닫고, 노트북은 기록과 시각화만 소비한다. 무선 구간이 약해도
원격 구독 상태가 로봇 제어를 멈추지 않게 하기 위한 구조다. 파이의 베이스·센서·박스 관측기·Nav2는
domain 12에서 모두 `LOCALHOST` 탐색 범위(UDPv4)를 쓴다. 한쪽만 다른 범위로 나눴을 때
`/tf`와 Collision Monitor lifecycle 서비스가 간헐적으로 보이지 않았기 때문에 같은 범위로 맞춘다.
노트북 RViz는 DDS에 붙지 않고 `rviz_display_relay`가 SSH로 전달하는 표시용 토픽만 받는다.
주행·실험이 끝나면 띄운 RViz·중계·임시 노드를 종료한다([AGENTS.md](AGENTS.md)).

## 패키지

| 경로 | 역할 | 출처 |
|---|---|---|
| `jdamr_cube_navigation` | Nav2 설정·launch, 복도 경로(`corridor_route`), Keepout 생성·검증, frontier 탐색, 식당 서비스(`restaurant_service`)·박스 주차, 평가 도구(`evaluation/`) | 상류 Nav2 예제에서 크게 확장 |
| `jdamr_cube_cartographer` | Cartographer 2D SLAM 프로파일(강의실·복도·시뮬) | 상류 + 프로파일 추가 |
| `jdamr_cube_vslam` | RGB-D 기록, RTAB-Map 3D 매핑, 정확도 평가 | 이 포크 |
| `jdamr_base_driver` | C++ 베이스 드라이버 | 이 포크 |
| `firmware/jdamr_cube_fw` | ESP32 펌웨어 v2 | 이 포크 |
| `jdamr_cube_node` | 캘리브레이션·폐루프 평가·프로브·웹 조종기 | 상류 + 도구 추가 |
| `jdamr_cube_bringup` | 실기 브링업(드라이버·라이다·TF) | 상류 + 실기 launch |
| `jdamr_cube_description` | URDF, 새 차체 제원, 그리퍼 충돌 형상 | 상류 + 수정 |
| `jdamr_cube_teleop` | 수동 매핑 조종기와 출발 전 점검 | 상류 + 보호 경로 |
| `jdamr_cube_gazebo` | Gazebo Harmonic 월드·브리지 | 상류 |
| `capstone_pick` | 비전 기반 자율 픽앤플레이스, 관제 UI, YOLOv8n 가중치 | 이 포크 |
| `jdamr_cube_so101_arm`, `jdamr_cube_moveit_config` | SO-101 팔 관절 제어·MoveIt 설정 | 상류 |
| `ydlidar_g4_ros2` | 현재 실기 LiDAR 드라이버(YDLIDAR G4) | 외부 |
| `ldlidar_sl_ros2` | 이전 LD14P 드라이버. 활성 launch에서는 쓰지 않음 | 외부 |
| `portfolio_data/`, `reference/` | 분석 결과·지도, 강사 원본 코드와 결함 감사 | 이 포크 |

상류의 초기 빌드 검증, Gazebo Classic → Harmonic 이관, SO-101 팔 사용법은
[README_UPSTREAM.md](README_UPSTREAM.md)에 보존했다.

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd "$HOME/jdamr_cube_ws"
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select jdamr_cube_navigation
```

## 주요 실행

```bash
# 실기 브링업 (파이)
ros2 launch jdamr_cube_bringup real_bringup.launch.py

# 저장 지도 + Keepout 자율주행
ros2 launch jdamr_cube_navigation keepout_navigation.launch.py

# 식당 서비스용 Nav2 서버 시작 (파이, 이동 명령은 보내지 않음)
bash jdamr_cube_navigation/scripts/restaurant_session.sh start
# --precision-parking은 승인된 5cm 박스 주차 시험에서만 추가한다

# 테이블 박스 정밀 주차 → 2 s 대기 → 이탈 후진 → 충전소 복귀 (정밀 세션·위치 초기화 뒤, 출발 요청 시)
python3 -m jdamr_cube_navigation.box_service --registry <registry> --approach-route <route> \
  --camera-mount <mount> --geometry <geometry> --parking-contract <box 계약> \
  --table-id table_01 --region-xy <x> <y> --log <새 jsonl> \
  --candidate-trial --execute --search --return-home --return-timeout-s <s>

# 소프트웨어 비상정지 (PC, ssh 경유: 드라이버가 리셋 전까지 바퀴를 0으로 묶음)
python3 scripts/map_20260930_manual/jdamr_depart.py estop
python3 scripts/map_20260930_manual/jdamr_depart.py estop-reset

# 기록 bag을 격리 도메인에서 재생해 SLAM 백엔드 하나를 실행
bash jdamr_cube_navigation/scripts/offline_slam_replay.sh \
  --bag <bag 경로> --backend cartographer --out <결과 디렉터리>

# Gazebo 시뮬레이션
ros2 launch jdamr_cube_gazebo gazebo.launch.py
```

파이 배포는 `scripts/deploy_navigation_to_pi.sh`를 쓴다. 실차 주행 전 점검은
`jdamr_cube_navigation/scripts/corridor_preflight.sh`에 있다.

## 문서 지도

| 문서 | 내용 |
|---|---|
| [README_JDAMR.md](README_JDAMR.md) | 실기 제원, 펌웨어 v2, 검증 도구, 판정 도구가 틀렸던 경위 |
| [README_SLAM_PORTFOLIO.md](README_SLAM_PORTFOLIO.md) | 복도 자율주행·SLAM 비교 결과와 수치, 결과 미디어 |
| [evaluation/README.md](jdamr_cube_navigation/evaluation/README.md) | 평가 도구·데이터셋·지도 registry·실험 문서 색인 |
| [20260921_NAVIGATION_JD_GAP_ROADMAP.md](jdamr_cube_navigation/evaluation/20260921_NAVIGATION_JD_GAP_ROADMAP.md) | 물류 AMR 채용 요건 대조와 고도화 항목 P1~P6·A1~A2 |
| [20260918_RESTAURANT_SERVICE_IMPLEMENTATION.md](jdamr_cube_navigation/evaluation/20260918_RESTAURANT_SERVICE_IMPLEMENTATION.md) | 서비스 위치 교시·정밀 배치 구현 범위 |
| [20260922_SERVICE_PORTFOLIO_HANDOFF.md](jdamr_cube_navigation/evaluation/20260922_SERVICE_PORTFOLIO_HANDOFF.md) | 서빙 주행 RViz 화면과 기록 재생 인계 |
| [20260929_PARKING_FAILURES.md](jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md) | 테이블 정밀 주차 실패 원인·수정·파이 반영 기록과 남은 실차 확인 항목 |
| [20260930_DOCK_RETURN_PORTFOLIO_HANDOFF.md](jdamr_cube_navigation/evaluation/20260930_DOCK_RETURN_PORTFOLIO_HANDOFF.md) | 충전소 후진 도킹 자료 묶음·그림·주장 범위·포트폴리오 문안 |
| [20260930_MAP_BOX_LAYOUT_PLAN.md](jdamr_cube_navigation/evaluation/20260930_MAP_BOX_LAYOUT_PLAN.md) | 새 지도 작성·keepout 재설계·도크 등록·테이블 박스 배치 계획(승인 대기) |
| [20260929_NAV2_PLATFORM_RESEARCH.md](jdamr_cube_navigation/evaluation/20260929_NAV2_PLATFORM_RESEARCH.md) | 파이 Nav2 부하·통신 구성 조사와 판단 한계 |
| [AGENTS.md](AGENTS.md) | 실차 출발·재개·주행 뒤 정리 운영 규칙 |
| [PORTFOLIO_20260826.md](PORTFOLIO_20260826.md) | 캡스톤 시점의 Physical AI 적용안 |

## 관련 저장소

- [robot-dashboard](https://github.com/mmporong/robot-dashboard): 실행 기록을 되감아 다시 판정하는
  관제 화면. 캡스톤에서 로그상 `PICK_SUCCESS` 중 4건이 실제로는 통 밖이었음을 Gazebo 실좌표로 잡아냈다.
- [gazebo-so101-capstone](https://github.com/mmporong/gazebo-so101-capstone): 캡스톤 구현 기록 9편.
