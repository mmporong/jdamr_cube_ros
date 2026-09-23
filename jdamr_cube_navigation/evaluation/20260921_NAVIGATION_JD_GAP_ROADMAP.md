# 물류 AMR 자율주행 SW 채용 요건 대조와 고도화 로드맵 — 2026-09-21

## 다음 세션 실행 지시

이 문서는 현재 저장소가 물류 AMR 자율주행 SW 개발자 공고의 요구 역량을 어디까지
증거로 갖고 있는지 대조하고, 빈칸을 채우는 고도화 항목을 계약·구현 위치·검증·완료
조건까지 정해 둔 실행 계획이다. 다음 세션은 아래 순서로 진행한다.

1. `git status --short --branch`와 `git log --oneline -5 origin/main`으로 현재 위치를 확인한다.
   로컬 `main`이 원격보다 뒤처져 있으면 `git pull --ff-only origin main`으로 맞춘다.
2. 5절 "고도화 항목"에서 우선순위가 가장 높은 미완료 항목 하나를 고른다. 여러 항목을
   한 세션에 동시에 열지 않는다.
3. 항목마다 `계약 YAML → 실행 스크립트 → summary.json·MCAP 해시 → readiness 문서`
   순서로 산출물을 남긴다. 기존 `evaluation/2026*_READINESS.md` 문서와 같은 구조를 따른다.
4. 완료 조건을 모두 통과한 항목만 7절 진행 현황 표의 상태를 바꾼다. 시뮬레이션 결과를
   실차 성능으로 적지 않는다.
5. 실차 이동이 필요한 검증은 사용자의 명시적 실행 요청이 있을 때만 수행한다.

## 1. 대상 공고와 요구 역량

두 공고를 기준으로 삼는다. 둘 다 2026-09 기준 공개 채용 페이지 원문이다.

| 구분 | 공고 A: 모바일 로봇 자율주행 SW 개발자 (범용형) | 공고 B: AMR Navigation SW 개발자 (분업형) |
|---|---|---|
| 주요 업무 | 지도 제작·SLAM(LiDAR·Camera·IMU 융합), Localization, Global·Local Path Planning, 주행 제어(Differential·Swerve·Ackermann), LiDAR·Camera 기반 정밀 도킹·정렬, 제어 파라미터 튜닝·성능 검증 시뮬레이션, 모듈 통합 테스트·검증 | Global planner·Local planner(장애물 회피), Path Tracking, 정밀 도킹 제어, 군집 주행 planner, AMR Kinematics·Dynamics 모델링 |
| 자격 | C++·Python, ROS·ROS 2 시스템 개발, LiDAR·IMU·Camera 데이터 처리, SLAM·Localization·Navigation 이해. 경력 무관, AMR·AGV 1년 이상 우대 | ROS 경험, Python·C++, 로봇공학·메카트로닉스 계열. 경력 3년 또는 신입은 AMR 관련 프로젝트 참여자 |
| 우대 | 실 로봇 적용 경험 | A*, Hybrid A*, D*, RRT 설계 경험, 영어 회화 |

공고 B를 낸 회사의 공개 제품 자료에서 확인한 현업 조건은 다음과 같다. 이 수치는 그 회사의
주장이며 이 저장소의 목표치가 아니다.

- 구동 방식 3종(Differential·Single-steer·Quad-steer), 리프트형 AMR 80·500·1,200 kg, 지게차형 1.5~3.5 t
- 도킹 정밀도 ±5 mm, 단일 현장 최대 188대 동시 운영, 관제가 초 단위 우선순위 교통 제어
- 3D LiDAR·IMU 융합 SLAM, V2V 통신 기반 로봇 간 충돌 방지, 설비 연동(컨베이어·수직반송기·도킹·충전 스테이션)

## 2. 현재 저장소의 보유 증거

원격 `main` 2026-09-12 커밋 `8204ffd` 기준이다. 경로는 저장소 루트 기준이다.

### 2.1 Localization

| 증거 | 내용 | 위치 |
|---|---|---|
| AMCL 프로필 비교 Axis A | 실차 완주 bag에 P0·P1·P2 × seed 5 = 15회 재생. CPU·지연·`map -> odom` 일관성 비교 | `evaluation/20260907_SLAM_ADVANCEMENT_PLAN.md`, `evaluation/evaluate_amcl_axis_a.py` |
| AMCL 재지역화 Axis B | Gazebo GT 기반 정상 초기화·0.5 m/15° 오프셋·8 m 납치 × 3프로필 × seed 5 = 45회. 납치는 세 프로필 모두 0/5 복구, 거짓 수렴 0건 | `evaluation/evaluate_amcl_axis_b.py`, `evaluation/classify_amcl_axis_b_failures.py` |
| 공분산 게이트 경위 | 코드 기본값 0.5가 정상 주행에서 63회 초과해 50.0으로 완화. 실측 cov_xx 최대 0.731 | `portfolio_data/analysis/summary_stats.json`, `test/test_keepout_config.py` |
| `map -> odom` 갱신 공백 | p50 103.8 ms, p99 713 ms, 최대 23.5 s, 1초 초과 20회 | `portfolio_data/analysis/tf_map_odom_latency.csv` |

### 2.2 SLAM·지도

| 증거 | 내용 | 위치 |
|---|---|---|
| 백엔드 비교 | 같은 bag으로 Cartographer(오도메트리 대비 RMS 0.670 m) vs SLAM Toolbox(0.521 m) | `portfolio_data/analysis/slam_backend_comparison.json`, `evaluation/compare_slam_runs.py` |
| 환경별 프로파일 | 복도·실기·시뮬 lua 분리, 히트확률 0.65 | `jdamr_cube_cartographer/config/*.lua`, `config/slam_toolbox_corridor.yaml` |
| 저장 지도 | 894×212 셀, 0.05 m, Keepout 9,406 셀, 연결성 검사 | `portfolio_data/maps/` |
| 안전 자율 매핑 | frontier 선택 → Nav2 단일 목표, Collision Monitor만 `/cmd_vel` 발행 | `jdamr_cube_navigation/frontier_explorer.py`, `frontier_core.py` |
| 2.5D 디지털 트윈 | 실차 점유격자를 Gazebo 충돌 메시로 변환해 20 waypoint·장애물 장면 3건 재실행 | `evaluation/20260912_2_5D_DIGITAL_TWIN_PORTFOLIO_HANDOFF.md` |

### 2.3 Path Planning·Tracking

| 증거 | 내용 | 위치 |
|---|---|---|
| 플래너·제어기 | NavFn(GridBased) + Regulated Pure Pursuit(FollowPath). 팽창 0.30 m, footprint 0.46×0.40 m | `config/nav2_params.yaml` |
| 행동 트리 4종 | 복도 fail-fast, 동적 장애물 재계획, 주차, 안전 매핑 | `behavior_trees/*.xml` |
| Keepout | 다각형 → 마스크 생성·검증 도구, 경로 연결성 거부 로직 | `keepout_mask` 엔트리, `config/keepout_zones.autonomous_20260826.yaml` |
| 실차 완주 | 20 waypoint 77.278 m 사전 계획, 복구 0회 완주, 시작·종료 위치차 0.113 m | `evaluation/20260904_CORRIDOR_LOCALDDS_SUCCESS.md` |
| 고정·동적 장애물 | 박스 우회 + 보행자 횡단 정지·동일 goal 재개, 접촉 0회, 팔 포함 외곽 최소 여유 0.0396 m | `evaluation/20260908_DYNAMIC_OBSTACLE_READINESS.md` |

### 2.4 정밀 정지·도킹

| 증거 | 내용 | 위치 |
|---|---|---|
| 주차 계약 | `map` 기준 5 cm·3°, Gazebo seed 11 단일 실행 4.541 cm·2.540° 통과 | `config/parking_contract.yaml`, `evaluation/20260908_PARKING_READINESS.md` |
| 주차 제어 | RPP + `parking_goal_checker`, 정지 1.0 s 유지 판정 | `evaluation/prepare_parking_params.py` |
| 도킹 서버 설정 | `opennav_docking` SimpleChargingDock, `use_external_detection_pose: true`, 접근 오프셋 -0.7 m. 설정만 있고 미사용 | `config/nav2_params.yaml` `docking_server` 절 |

### 2.5 제어·안전 계층

| 증거 | 내용 | 위치 |
|---|---|---|
| 보호영역 유도 | URDF collision 외곽 0.3026 m + 반응거리 0.0284 m + 제동거리 0.0324 m → StopZone 0.40 m | `evaluation/robot_stow_envelope.py`, `config/mobile_manipulator_protection.yaml` |
| 팔 없는 프로필 | StopZone 0.35 m·SlowdownZone 0.45 m 근거 | `config/base_obstacle_protection.yaml` |
| 마찰 이상 상태기계 | wheel odom·IMU·localization 불일치 → `TRACTION_FAULT` → 재위치추정 → 1회 저속 재개 | `evaluation/20260911_RESTAURANT_SAFETY_ADVANCEMENT.md`, `test/test_traction_velocity_guard.py` |
| 속도 제한 | velocity smoother 0.18 m/s·0.7 rad/s | `config/nav2_params.yaml` |
| 펌웨어·드라이버 | IMU 단위·정지 보정, 서보 버스, 시리얼 재연결 안전 처리 | `firmware/jdamr_cube_fw/`, `jdamr_base_driver/` |

### 2.6 통합·검증 인프라

| 증거 | 내용 | 위치 |
|---|---|---|
| DDS·무선 진단 | Wi-Fi 경유 DDS에서 `/scan` 공백 10.75 s → LOCALHOST 분리 뒤 0.112 s | `README_JDAMR.md` "Wi-Fi 비의존 실차 구조" |
| 테스트 | 패키지 테스트 1,223 passed | `test/` |
| 증거 계약 | 실험 manifest 스키마, 데이터셋·지도·캘리브레이션 레지스트리, MCAP CRC 검사 | `evaluation/experiment_manifest.schema.json`, `evaluation/inspect_mcap.py` |
| 관제 재생 | MCAP 관측을 영상 시간축에 맞춘 대시보드 | `evaluation/navigation_dashboard.py`, `evaluation/export_dashboard_replay.py` |

## 3. 요구 역량 대조 요약

| 요구 역량 | 공고 | 상태 | 판정 근거 |
|---|---|---|---|
| Localization 개발·검증 | A | 충족 | 60회 재생 비교, 납치 실패 분류, 실측 공분산 |
| SLAM 지도 생성 | A | 충족(2D) | 백엔드 비교, 실차 지도, 자율 매핑 |
| 다중 센서 융합 SLAM(3D LiDAR·IMU·Camera) | A | 미충족 | 2D LiDAR 단일 센서. IMU는 기록만 하고 추정에 융합하지 않음 |
| 센서 융합 오도메트리 | A | 미충족 | `robot_localization` 등 EKF 없음 |
| Global·Local planner | A·B | 부분 | NavFn·RPP 한 조합. 플래너·제어기 비교 없음 |
| Path tracking 지표 | B | 부분 | 완주 증거는 있으나 cross-track·heading 오차 미산출 |
| 정밀 도킹·정렬 | A·B | 부분 | 절대 좌표 주차 5 cm. 상대 센싱 도킹 없음 |
| 군집 주행 | B | 미충족 | 단일 로봇만 |
| Kinematics·Dynamics 모델링 | A·B | 부분 | Differential 1종. Ackermann·Swerve 없음. 하중 변화 미측정 |
| 제어 파라미터 튜닝·성능 검증 시뮬 | A | 충족 | Gazebo 격리 실행기, seed 반복, 계약 대조 |
| 모듈 통합 테스트·검증 | A | 충족 | 테스트·manifest·MCAP 체계 |
| 실 로봇 적용 | A·B | 충족 | 실차 완주, 장애물 실차 통합 절차 |

## 4. 고도화 설계 원칙

- 기존 안전 계층(Collision Monitor, Keepout, footprint, fail-closed 게이트)은 완화하지 않는다. 새 기능은 그 위에 얹는다.
- 새 항목마다 단위가 명시된 계약 YAML을 먼저 만들고, 실행기가 설치된 런타임 값과 계약을 대조한 뒤에만 출발한다.
- 평가는 Gazebo 격리 domain에서 seed 반복으로 먼저 하고, 실차는 별도 승인 범위로 둔다.
- 산출물은 `summary.json`, MCAP, SHA-256 manifest, 재생 명령을 함께 남긴다. 미디어는 성공 실행 하나에서만 만든다.
- 문서에는 "단일 실행", "시뮬레이션", "설정 기반 상한"처럼 주장 범위를 먼저 적는다.

## 5. 고도화 항목

우선순위는 공고 빈칸의 크기와 기존 파이프라인 재사용 가능성으로 정했다.
`P1 → P3 → P2 → P4 → P5 → P6` 순서로 진행하고, 공고 A 전용 항목 `A1`·`A2`는 P3 뒤에 끼운다.

### P1. 조향형 기체와 Hybrid A* 플래너 (Kinematics·Dynamics, 우대 Hybrid A*)

목표: Differential 외에 전륜 조향(Ackermann) 기체 모델을 추가하고, 비홀로노믹 제약을 반영한
플래너·제어기 조합으로 같은 지도에서 계획·완주할 수 있음을 보인다.

계약 `config/kinematics_contract.yaml` (신규):

```yaml
schema_version: 1
vehicles:
  diff_drive:
    wheelbase_m: null
    track_m: 0.34            # URDF에서 유도, 테스트가 대조
    max_linear_mps: 0.18
    max_angular_radps: 0.7
  ackermann_forklift:
    wheelbase_m: 0.40         # 축소 모델 가정값, URDF와 대조
    max_steer_deg: 35.0
    min_turning_radius_m: 0.571   # wheelbase / tan(max_steer)
    max_linear_mps: 0.18
    reverse_allowed: true
comparison:
  route: config/corridor_roundtrip.autonomous_20260826.yaml
  metrics: [plan_length_m, max_curvature_1pm, waypoints_completed, cross_track_p95_m]
physical_validation: NOT_PHYSICALLY_VALIDATED
```

구현 위치:

- `jdamr_cube_description/urdf/jdamr_forklift.urdf`: 전륜 조향·후륜 구동 축소 모델. gz-sim `ackermann-steering-system` 플러그인, 같은 G4 LiDAR 위치, 같은 `base_footprint` 규약.
- `config/nav2_params_ackermann.yaml`: `nav2_smac_planner::SmacPlannerHybrid`(motion model `REEDS_SHEPP`, `minimum_turning_radius`는 계약값), 제어기는 `nav2_mppi_controller`(Ackermann motion model)와 RPP 두 후보.
- `jdamr_cube_gazebo/launch/gazebo.launch.py`: `vehicle:=diff_drive|ackermann_forklift` 인자.
- `evaluation/run_kinematics_comparison.py`: 기체 2종 × 플래너 2종(NavFn·Smac Hybrid) × seed 3을 격리 domain에서 실행. 기존 `run_sim_slam_experiment.py`의 격리·MCAP·정리 로직을 재사용한다.
- `evaluation/compare_kinematics.py`: 계획 길이, 최대 곡률, 완주 waypoint 수, cross-track p95를 표로 만든다.
- `test/test_kinematics_contract.py`: URDF 파라미터와 계약값 대조, 최소 회전반경 계산식 검증.

완료 조건:

- Ackermann 기체가 Smac Hybrid로 20 waypoint 중 20개 완주(seed 3 모두), NavFn 조합은 곡률 위반 또는 실패가 표로 드러남.
- MPPI와 RPP 중 하나를 Ackermann 기본 제어기로 선택한 근거(cross-track p95, 완주율)를 문서화.
- 산출물 manifest SHA-256, 대표 GIF 1개.

주장 범위: 축소 모델 시뮬레이션. 실제 지게차 조향 응답·하중은 포함하지 않는다.

### P2. 상대 센싱 정밀 도킹 (정밀 도킹·정렬)

목표: 절대 좌표 주차를 도크 기준 상대 pose 도킹으로 바꾸고, 반복 정밀도를 통계로 보고한다.

계약 `config/dock_contract.yaml` (신규):

```yaml
schema_version: 1
dock_frame: dock
detector: lidar_v_reflector      # 2D LiDAR로 V자 도크 형상 검출. 카메라 장착 시 apriltag 추가
stage1_nav2:
  xy_tolerance_m: 0.05
  yaw_tolerance_deg: 3.0
stage2_docking:
  xy_tolerance_m: 0.01
  yaw_tolerance_deg: 1.0
  approach_linear_mps: 0.05
  max_retries: 3
repeatability:
  sim_runs: 10
  real_runs_max: 3
  report: [median_xy_m, max_xy_m, median_yaw_deg, success_rate]
physical_validation: NOT_PHYSICALLY_VALIDATED
```

구현 위치:

- `jdamr_cube_navigation/jdamr_cube_navigation/dock_detector.py`: `/scan`에서 V자 반사판 두 변을 RANSAC 직선 적합으로 찾고 꼭짓점·법선으로 `dock` pose를 `detected_dock_pose`로 발행. 검출 실패·불확실 시 발행하지 않는다.
- `jdamr_cube_gazebo/worlds/room.world` 또는 새 `dock.world`: V자 벽 도크 모델.
- `config/nav2_params.yaml` `docking_server` 절: 이미 있는 `use_external_detection_pose: true`에 위 토픽 연결. 접근 속도는 계약값.
- `evaluation/run_docking_repeat.py`: 시작 pose에 ±0.2 m·±10° 무작위 오프셋을 주고 10회 반복. GT 대비 최종 오차와 성공률을 `summary.json`에 기록.
- `evaluation/render_docking_media.py`: 기존 `render_parking_media.py` 확장.
- `test/test_dock_detector.py`: 합성 scan으로 검출 정확도·미검출 처리 검증.

완료 조건:

- 시뮬 10회 성공률과 중앙값·최대 오차 표. 2단계 계약 1 cm·1° 통과 여부를 회차별로 기록.
- 검출 실패 시 도킹이 시작되지 않는 fail-closed 테스트 통과.
- 실차는 기존 주차 문서의 절차(줄자·바닥 기준선, 최대 3회)를 그대로 사용하며 사용자 요청 시에만 수행.

주장 범위: 2D LiDAR 반사판 기준. ±5 mm급 산업 도킹과 동일하다고 쓰지 않는다.

### P3. 추종 오차 지표 (Path Tracking)

목표: 이미 기록된 MCAP에서 계획 경로 대비 실제 궤적의 cross-track·heading 오차를 산출해
완주 증거를 정량 추종 지표로 바꾼다.

구현 위치:

- `evaluation/tracking_error.py`: `/plan`(마지막 유효 계획)과 GT(`Gazebo`) 또는 AMCL·TF(실차)를 시간 정렬하고 최근접점 거리·방향 차이를 계산. p50·p95·max, 구간별(직선·회전) 분리.
- `corridor_run_media.py --metrics-only` 출력에 위 지표를 합친다.
- `test/test_tracking_error.py`: 합성 경로로 계산식 검증.

적용 대상: `corridor_localdds_armed_20260904T152036`(실차), `parking_smoke_20260908_v03`, 동적 장애물 좌·우 실행. P1의 비교 지표로도 사용한다.

완료 조건: 위 4개 실행의 지표 표와, RPP `lookahead_dist`·`desired_linear_vel` 2~3조건 Gazebo 비교 1건.

주장 범위: 실차는 AMCL 추정 대비 오차이므로 절대 정확도가 아니라고 명시한다.

### P4. 2대 군집 교통 관리 (군집 주행 planner)

목표: 같은 지도에서 로봇 2대가 복도 교행·교차로 진입을 충돌·데드락 없이 마치는 최소 교통 관리자를
만든다. 관제 상위 계층의 축소판이며 로봇 단 Nav2·Collision Monitor는 그대로 둔다.

계약 `config/fleet_contract.yaml` (신규):

```yaml
schema_version: 1
robots: [robot1, robot2]
zones: config/fleet_zones.autonomous_20260826.yaml   # 복도를 구간으로 나눈 다각형
policy: zone_reservation           # 구간 단위 배타 점유
priority: first_request            # 동률 시 낮은 id
deadlock:
  detect: wait_for_graph_cycle
  resolve: lower_priority_backs_to_previous_zone
  timeout_s: 20
scenarios: [head_on_corridor, t_junction_merge]
runs_per_scenario: 10
metrics: [collisions, deadlocks_detected, deadlocks_resolved, wait_time_p95_s, makespan_s]
```

구현 위치:

- 새 패키지 `jdamr_cube_fleet/`: `traffic_manager.py`(구간 예약·해제, 대기 그래프 순환 검출), `fleet_goal_dispatcher.py`(로봇별 NavigateToPose 액션 클라이언트, 예약 승인 전 다음 구간 목표를 보내지 않음).
- `jdamr_cube_gazebo/launch/multi_robot.launch.py`: 네임스페이스 2개, TF prefix, 로봇별 Nav2 컨테이너.
- `evaluation/run_fleet_scenarios.py`, `evaluation/compare_fleet_runs.py`.
- `test/test_traffic_manager.py`: 예약 충돌·순환 검출·해소 순서 단위 테스트.

완료 조건: 시나리오 2종 × 10회에서 충돌 0, 감지된 데드락 전부 해소, 대기 시간 p95 표.

주장 범위: 2대 시뮬레이션. 188대급 관제나 V2V 통신 구현이 아니라고 명시한다.

### P5. 하중 변화 동역학과 제동거리 실측 (Dynamics, 제어 파라미터 튜닝)

목표: 보호영역 유도식의 "설정 기반 상한" 제동거리를 시뮬 실측으로 바꾸고, 적재 질량에 따른
가감속 한계를 계약에 넣는다.

구현 위치:

- `evaluation/brake_distance_by_payload.py`: 적재 질량 0·2·5 kg(축소 모델)에서 설정 속도 주행 중 정지 명령 → 정지까지 거리·시간을 GT로 측정. 마찰계수 2조건 병행.
- `config/mobile_manipulator_protection.yaml`, `config/base_obstacle_protection.yaml`: 제동거리 항목의 근거를 "등가속도 상한 계산"에서 "시뮬 실측 최대값 + 여유"로 갱신하고 출처 파일을 적는다.
- velocity smoother `max_accel`·`max_decel`을 질량별 프로필로 분리할지 여부를 결과로 결정.

완료 조건: 질량·마찰 조건별 제동거리 표, StopZone 재계산 결과, 기존 동적 장애물 시나리오 재실행에서 접촉 0 유지.

주장 범위: 축소 모델 시뮬. 실차 제동거리는 별도 승인 뒤 저속 실측.

### P6. 관제 연동 인터페이스 (선택)

목표: 상위 관제가 경로 주문을 내려주는 구조를 최소 메시지로 재현한다. 평가 README의
"VDA 5050 order/action 연동은 현재 구현 범위 밖" 문장을 지울 수 있는 수준이면 충분하다.

구현 위치: `jdamr_cube_fleet/vda5050_adapter.py`. MQTT JSON `order`(노드·엣지 목록)를 받아
`corridor_route` 경로 YAML로 변환하고, `state`(위치·배터리·오류·마지막 노드)를 1 Hz로 발행.
`test/test_vda5050_adapter.py`로 스키마 검증.

완료 조건: 로컬 MQTT 브로커로 order → 주행 → state 왕복 1회, 스키마 필수 필드 테스트 통과.

### A1. 센서 융합 오도메트리 (공고 A: 다중 센서 융합·Localization)

목표: 휠 오도메트리와 IMU를 EKF로 융합한 `odom -> base_footprint`를 만들고, AMCL 입력
오도메트리 품질 변화가 재지역화에 미치는 영향을 기존 Axis A·B 프레임워크로 재평가한다.

구현 위치:

- `config/ekf.yaml`: `robot_localization` `ekf_node`. 휠 odom(x·y 속도·yaw 속도), IMU(yaw 속도·선가속도), 2D 모드. 공분산은 IMU 정지 보정 기록과 `firmware/jdamr_cube_fw/imu_stationary_calibration.h` 값에서 유도.
- `jdamr_cube_bringup/launch/real_bringup.launch.py`: `use_ekf` 인자. 기본은 기존 경로 유지.
- `evaluation/evaluate_amcl_axis_a.py`·`axis_b`: 오도메트리 소스 `wheel|ekf` 축 추가.
- `test/test_ekf_config.py`: 프레임·토픽·2D 모드·공분산 양정치 검증.

완료 조건: 같은 bag에서 wheel vs ekf 오도메트리 드리프트 비교(폐경로 시작·종료 차), Axis B 오프셋 시나리오에서 복구 scan 수 비교. 납치 시나리오 개선을 기대치로 쓰지 않는다.

### A2. 3D 센서 융합 SLAM 재생 (공고 A: 3D LiDAR·IMU 융합, 시뮬 한정)

목표: Gazebo `gpu_lidar` 3D 센서와 IMU로 LiDAR-Inertial SLAM 하나를 재생하고, 2D 결과와
같은 지도 좌표계에서 비교한다. 실차 3D LiDAR가 없으므로 시뮬 한정이며 우선순위는 가장 낮다.

구현 위치: `jdamr_cube_description/urdf/sensors_3d.xacro`(선택 장착), `evaluation/run_sim_slam_experiment.py`의 backend 목록에 3D 후보 1종 추가, 비교는 기존 ATE·RPE 도구 재사용.

완료 조건: 3D backend 1종 실행 성공, 2D 대비 ATE·RPE·CPU 표. 실차 주장 없음.

## 6. 예상 질문과 답이 되는 증거

| 예상 질문 | 지금 답할 수 있는 증거 | 고도화 뒤 추가되는 증거 |
|---|---|---|
| 복도에서 지도가 벌어졌을 때 어디부터 봤나 | `SLAM_DEBUG_HANDOFF_20260824.md`, 백엔드 비교, LD14 시간축·회전 방향 수정 커밋 | — |
| AMCL이 위치를 잃으면 어떻게 감지하나 | 공분산 게이트 경위, Axis B 납치 0/5와 거짓 수렴 0건 | A1 오도메트리 품질 축 |
| Differential용 설정을 Ackermann에 쓰면 무엇이 깨지나 | — | P1 NavFn vs Smac Hybrid 비교표 |
| 도킹 성공률을 올리려면 무엇을 측정하나 | 주차 5 cm 단일 실행, `/amcl_pose`와 TF 시점 차이 분석 | P2 10회 반복 통계, 2단계 계약 |
| 시뮬 게인이 실차에서 안 맞은 경험 | Wi-Fi DDS 지연으로 인한 정지, 공분산 게이트 완화 | P5 제동거리 실측 |
| rosbag을 주면 어떤 순서로 보나 | `inspect_mcap.py`, `extract_amcl_tf.py`, 대시보드 재생 | P3 추종 오차 |
| 로봇 여러 대가 마주치면 | — | P4 예약·데드락 해소 |

## 7. 진행 현황

| 항목 | 상태 | 갱신일 | 근거 문서 |
|---|---|---|---|
| P1 조향형 기체·Hybrid A* | 미착수 | 2026-09-21 | — |
| P2 상대 센싱 도킹 | 미착수 | 2026-09-21 | — |
| P3 추종 오차 지표 | 미착수 | 2026-09-21 | — |
| P4 2대 군집 | 미착수 | 2026-09-21 | — |
| P5 하중·제동거리 | 미착수 | 2026-09-21 | — |
| P6 관제 인터페이스 | 미착수 | 2026-09-21 | — |
| A1 EKF 오도메트리 | 미착수 | 2026-09-21 | — |
| A2 3D SLAM 재생 | 미착수 | 2026-09-21 | — |

상태 값은 `미착수 / 진행 중 / 시뮬 완료 / 실차 완료 / 보류` 다섯 가지만 쓴다.

## 8. 저장소 위생 항목

- 루트에 `README.md`와 `readme.md`가 함께 커밋돼 있다. 대소문자를 구분하지 않는 Windows·macOS
  체크아웃에서는 한 파일이 다른 파일을 덮어써 항상 수정 상태로 보인다. 하나로 합치고 나머지를
  `git rm --cached`로 제거해야 한다. 실제 내용은 `git show origin/main:README.md`로 확인한다.
- 문서 안의 `/home/lim/jdamr_artifacts/...` 절대경로는 기기 종속이다. 새 문서는 `$HOME/jdamr_artifacts/...`로 쓴다.
- `portfolio_data/README.md`가 지적한 대로 경로 총연장은 76.42 m(계획)·77.278 m(사전 계획 로그)가 혼재한다. 공개 자료에는 출처를 붙여 하나만 쓴다.
- `.omc/state`, `.omx/context`는 세션 산출물이다. 커밋 대상에서 제외할지 결정한다.

## 9. 하지 않을 것과 쓰지 않을 표현

- 실차 이동이 필요한 검증을 사용자 요청 없이 실행하지 않는다.
- 기존 안전 계층 파라미터를 성능을 위해 완화하지 않는다.
- 단일 실행을 반복 성공률로, 시뮬 결과를 실차 성능으로, 축소 모델을 산업용 지게차로 쓰지 않는다.
- `±5 mm 도킹 달성`, `수백 대 관제`, `3D SLAM 실차 검증`, `기능 안전 인증`은 해당 실측 없이는 쓰지 않는다.
- 특정 회사의 재무·처우·인물 정보는 이 저장소에 적지 않는다.
