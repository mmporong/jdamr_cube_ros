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
| 식당 서빙·박스 정밀 주차 | 진행 중 | 충전소 출발 → 테이블 이동 → 박스 면 정렬 → 5cm 접근 → 복귀 흐름을 연결. 2026-09-22 실차에서 목표 수락·이동까지 확인했으나 저전압(10.464V < 10.5V)으로 취소. 테이블 도착·정밀 주차·복귀는 미완료 | [20260922_SERVICE_DEPARTURE_FAILURES.md](jdamr_cube_navigation/evaluation/20260922_SERVICE_DEPARTURE_FAILURES.md) |
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

노트북: 기록(MCAP)·RViz·재생·평가·미디어 생성 (제어 경로에는 참여하지 않음)
```

로봇 제어 그래프는 파이 안에서 닫고, 노트북은 기록과 시각화만 소비한다. 무선 구간이 약해도
원격 구독 상태가 로봇 제어를 멈추지 않게 하기 위한 구조다. DDS 탐색 범위는 센서 브링업과
Nav2가 같은 `SUBNET`을 쓴다. LOCALHOST와 SUBNET을 나눴을 때 `/tf`와 Collision Monitor
lifecycle 서비스가 간헐적으로 보이지 않았기 때문이다.

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
| [PORTFOLIO_20260826.md](PORTFOLIO_20260826.md) | 캡스톤 시점의 Physical AI 적용안 |

## 관련 저장소

- [robot-dashboard](https://github.com/mmporong/robot-dashboard): 실행 기록을 되감아 다시 판정하는
  관제 화면. 캡스톤에서 로그상 `PICK_SUCCESS` 중 4건이 실제로는 통 밖이었음을 Gazebo 실좌표로 잡아냈다.
- [gazebo-so101-capstone](https://github.com/mmporong/gazebo-so101-capstone): 캡스톤 구현 기록 9편.
