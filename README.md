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
| 식당 서빙·박스 정밀 주차 | 진행 중 | 충전소 출발 → 물 받는 곳 박스 → 호출 테이블 박스 → 충전소 후면 주차를 한 실행기(`box_service --return-home`, Nav2 세션과 함께 상주)로 잇는다. 박스마다 관측 위치 → 깊이·LiDAR 박스 면 관측 → 면 정렬(10 cm 위치 뒤 방향만) → 직선 최종 접근 → 2 s 대기 → 박스 이탈 후진 순서다. 최종 접근은 박스 주차 계약의 `target_front_gap_m`(1.5 cm, 2026-10-06 전에는 5 cm)까지 닫고, `--final-gap-m`·`--dwell-s`로 시험마다 간격과 대기를 바꿀 수 있다. 라이다는 차체 앞면에서 0.205 m 안쪽을 보지 못해 이 마지막 구간의 간격은 접근 전에 본 박스 면과 odom으로 정한다. `jdamr_depart.py go table_02 table_01`처럼 여러 테이블을 주면 사이에 도크 정차·재정합을 두고 이어 돈다. 이동 구간은 NavFn + RPP, 도크 대기점은 Lattice(찾지 못하면 NavFn) + RPP, 최종 접근은 정밀 주차 컨트롤러(`Parking`)로 따라간다. 2026-10-06 같은 사이클 실차 비교(NavFn + RPP 175.3 s, NavFn + MPPI 284.2 s, Lattice + MPPI는 지나친 경유점으로 돌아가는 고리 경로로 중단)로 정했다. MPPI는 경로 비평이 최종 목표 직선거리 0.5 m 안에서 꺼져, 관측 위치에서 0.42 m 떨어진 사전 지점을 건너뛰고 후진으로 들어왔다. MPPI 구성은 `--transit navfn-mppi`·`lattice-mppi`로 고를 수 있고, 막힌 구간 재시도는 Lattice + MPPI로 한다. 도크는 0.7 m 앞 대기점에서 위치를 맞추고 odom 기준 한 방향 회전으로 정렬한 뒤, 옆 어긋남을 도크 앞 0.25 m까지 3차 곡선 후진(회전 반경 0.3 m 이상)으로 없애고, 도크 밖에서 방향을 1.5° 안으로 맞춰 직선 후진한다. 끝 방향이 1–5°면 저속 제자리 회전으로 1° 안까지 다듬고(최대 3회), 5°를 넘으면 빠져나와 한 번 다시 진입한다. 45°가 넘는 방향 전환은 제자리 회전이고, 이동 0.12 m/s·정밀 주차와 후진 0.08 m/s(감속은 마지막 0.15 m)다. 깊이 카메라는 깊이만 켜고 박스 정차 구간에만 띄운다(`--camera-always-on`이면 상시). 깊이 관측은 테이블 영역 거리로 범위를 좁힌다. 박스 최종 접근은 `--zero-turn-final`로 제자리 각 보정 뒤 회전 없는 직진을 시험할 수 있다. RPP의 예측 충돌 검사는 끄고 차체 외곽 기준 Collision Monitor StopZone 정지를 남겼다. 차체 외곽과 정지 구역은 실측 모양(폭 0.45 m 프레임, 축 부근만 0.54 m 바퀴)에 5 mm를 더한 계단형 다각형(앞 정지 여유 5 cm)이고, 회전 없는 직진에서는 정지 구역이 진행 방향 쪽 절반만 보고, 제자리 회전은 외곽을 회전 방향으로 6° 쓸린 영역에서 멈춘다. 양팔 조립 차체의 기둥처럼 라이다 평면에 걸리는 자기 차체는 `real_bringup.launch.py scan_self_filter_config:=<YAML>`로 걸러 낸다(모델 값 템플릿만 있고 실측 전). 돌발상황은 현업 조사([20261002_ANOMALY_HANDLING_RESEARCH.md](jdamr_cube_navigation/evaluation/20261002_ANOMALY_HANDLING_RESEARCH.md))에 따라 드라이버 래치 비상정지(리셋 서비스로만 해제), 막힘 20 s 대기 후 재시도 3회(앞이 막히면 대기 횟수만큼 0.10·0.20·0.30 m 직진 후진, 축 뒤 장애물이 회전 원 안이면 그만큼 직진 전진, 차체에 닿은 물체는 들어온 odom 궤적을 되짚어 빠져나옴, 앞이 비거나 경로가 생기면 조기 재출발), 실패 시 등급별 운영자 호출(PC 알림, 끌 수 있음), 운영자 연결 하트비트 30 s(끊기면 다음 목표를 시작하지 않음), 정차 사이 저전압 도크 복귀로 처리한다. 물리 비상정지·범퍼는 아직 없다. 실차 기록(내부 추정, 외부 실측 없음): 2026-10-01 한 실행 사이클 table_02 186.5 s·table_01 175.0 s, 박스 앞면 5.4–5.6 cm, 도킹 1.3–4.2 cm, 2번 → 1번 연속 주행 성공. 2026-10-06 NavFn + RPP 사이클 175.3 s, 박스 앞면 5.53–5.54 cm. IMU 장착 방향(z 아래·y 앞)은 URDF `imu_link`로 선언했고, 바퀴 odom + 자이로 EKF와 주행마다 파이 원시 기록은 준비돼 있다(EKF는 실행 구성 미연결). 포트폴리오 데이터는 [20261001_PORTFOLIO_HANDOFF_FOR_CODEX.md](jdamr_cube_navigation/evaluation/20261001_PORTFOLIO_HANDOFF_FOR_CODEX.md)와 [20261006_CONTROLLER_SELECTION_PORTFOLIO_HANDOFF.md](jdamr_cube_navigation/evaluation/20261006_CONTROLLER_SELECTION_PORTFOLIO_HANDOFF.md). | [20260929_PARKING_FAILURES.md](jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md) |
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
