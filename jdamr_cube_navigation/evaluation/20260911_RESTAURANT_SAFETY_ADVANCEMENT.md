# 식당형 AMR 안전·강건성 고도화 — 2026-09-11

## 결론

식당 상황은 유의미하다. 현재 2D LiDAR로 검증 가능한 `통로 장애물 정지·재계획`, wheel
odom·IMU·localization 불일치로 판정하는 `마찰 이상 의심`, 접촉 센서가 있어야 하는 `충돌
사건`을 서로 다른 상태와 주장으로 분리한다. 낮은 이물질을 찾고 피하는 시나리오는 제외한다.
바닥 오염 구간을 이미 밟아 구동 마찰이 떨어진 상황으로 통합하고, 오염물 종류는 판정하지
않는다.

## 먼저 닫을 기준선

실차 SLAM 3회 반복은 수행하지 않는다. 기준선은 2026-09-10 저장 지도 장애물 주행과 기존
Gazebo Cartographer 강건성 결과를 사용한다. 새 시나리오는 먼저 Gazebo에서 같은 설정의
정상 바닥과 저마찰 바닥을 비교하고, 실제 이동이 필요한 검증은 별도 승인 범위로 남긴다.

## 출발 실패 판정

2026-09-10 사용자가 본 네 번의 시도는 하나의 원인이 아니었다. 원격 `pgrep -f`가 후속
명령 문자열을 자기 자신으로 잡은 `already_running` 오탐 1회, 로그가 생기기 전에 읽은
파일 생성 경쟁 1회, 기동 transient까지 포함한 전체 자원 TSV의 CPU p90 302%로 자원
게이트가 막은 1회, local costmap clear 응답이 client timeout을 넘긴 1회였다. 최종 실행은
20/20 waypoint를 완주했다. 따라서 출발 실패를 Nav2 경로 계획 실패나 멀티스레딩 부족으로
묶지 않는다.

자동 실행기는 고정 75초 sleep을 제거한다. 같은 실행에서 Nav2 기동 전 시작한 계측기의
최신 60초만 사용하고, lifecycle·사전점검·전체 경로 계획 중 이미 표본이 찼으면 바로
판정한다. 표본이 부족하거나 process coverage, CPU, 온도, throttle gate가 실패하면
기존처럼 출발하지 않는다. 계측 프로세스가 종료됐거나 마지막 표본이 12초보다 오래돼도
출발하지 않는다. 이 변경은 로컬 회귀검증 단계이며 실차 `--no-execute`와 실제 출발 검증은
아직 수행하지 않았다. 과거 성공 run을 출발 시각까지만 잘라 새 판정에 넣으면 CPU p90
301.5%로 300% 예산을 넘으므로, 고정 대기를 없앴다고 그 run이 즉시 출발 가능한 것은 아니다.

## 식당 시나리오와 상태 전이

| 우선 | 시나리오 | 현재 가능한 검증 | 추가 조건 |
|---|---|---|---|
| P0 | 의자·박스·카트 부분 차단, 사람 대역물 횡단 | scan 관측, StopZone, 0 명령, 새 경로, 동일 goal 재개 | 객체 종류는 영상 주석으로만 구분 |
| P0 | 좁은 통로, 열린 문, 완전 차단, goal occupied | 보호 정지와 재계획 불가의 fault 전환 | 자동 spin·후진 금지 |
| P0 | scan 가림·freeze, timestamp jitter, stale TF | watchdog 감속·정지와 기록 보존 | 실차 fault 주입은 비주행 또는 replay 우선 |
| P1 | 바닥 오염 구간 통과 뒤 마찰 저하·wheel slip | command↔odom, encoder yaw↔IMU, map→odom correction으로 `TRACTION_FAULT` 의심 | Gazebo 물리 마찰 패치 우선, 물체 탐지·회피 주장은 제외 |
| P2 | 실제 접촉 | 현재 센서로 확정 불가 | bumper/contact 또는 모터 전류·토크 입력 필요 |

```text
READY → NAVIGATING
  ├─ 보호영역 침입/입력 무효 → PROTECTIVE_STOP
  │    ├─ 입력 정상 + 유효 새 경로 → 동일 goal 저속 재개
  │    └─ 제한시간 초과/경로 없음 → FAULT_LATCHED
  ├─ 지속적인 구동·관측 불일치 → TRACTION_FAULT → 정지·재위치추정 → 1회 저속 재개 또는 래치
  └─ 인증된 접촉 입력 → COLLISION_LATCHED → 구동 금지·목표 취소·수동 점검/리셋
```

저마찰 구간은 장애물로 회피하지 않는다. 바퀴 구동량과 실제 이동 추정의 불일치가 지속되면
정지하고 위치를 다시 확인한 뒤 한 번만 저속으로 재개한다. 같은 불일치가 다시 발생하거나
재위치추정이 불안정하면 `FAULT_LATCHED`로 전환한다.

접촉 뒤에는 자동 BackUp, Spin, costmap clear 후 재출발을 호출하지 않는다. 접촉은 복구
행동이 아니라 안전사건으로 기록하며, 사람·물체 확인과 운영자 승인 전에는 motion을
재활성화하지 않는다. 실제 사람 충돌이나 물웅덩이 주행은 시험하지 않는다.

## 판단 시간과 멀티스레딩

판단 시간은 평균이 아니라 같은 시계의 `첫 유효 장애물 scan → Collision Monitor STOP →
최종 cmd_vel=0 → odom 속도 문턱 이하` p50/p95/max로 측정한다. 속도별 보호영역은 최악
응답시간과 실제 감속거리로 정하고, 목표 0.25초·상한 0.40초를 검증한다.

현재 Nav2의 `component_container_isolated`는 구성요소마다 전용 single-thread executor와
OS thread를 둔다. 베이스와 LiDAR도 I/O thread가 분리돼 있으며 성공 주행의 container는
최대 40 threads와 한 코어를 넘는 CPU를 사용했다. 즉 구성요소 간 병렬성은 이미 반영돼
있다. 성공 주행의 scan 최대 공백 0.165초와 전체 센서 연속성에는 executor starvation
증거가 없으므로 `component_container_mt`나 스레드 수 증가는 지금 채택하지 않는다.

스레딩 변경은 callback enqueue/start/end trace에서 특정 구성요소 내부 직렬화가 정지 지연의
원인으로 확인된 뒤에만 수행한다. baseline 3회와 후보 3회를 비교해 정지 누락 0, 지연 상한
0.40초, scan gap 비증가, 자원 게이트 PASS, lifecycle·종료 survivor 0을 모두 만족하고 latency
p95가 baseline 분산보다 줄 때만 승격한다.

## 2026-09-11 파이 비주행 검증

`slam_advance_noexecute_20260911T1110`으로 Raspberry Pi 배포본을 `--delay 0
--no-execute --navigation-profile obstacle_base_candidate` 조건에서 실행했다. lifecycle 3/3,
사전점검, 전체 경로 계획과 최신 60초 자원 게이트가 모두 PASS했고, 계획 완료 뒤 자원 판정은
약 2초 만에 끝났다. 고정 75초 sleep은 실행되지 않았다.

자원 표본은 13/13, 62.0초, 최대 간격 5.3초였고 system CPU p90 291%, peak 304%, 최고
온도 69.6°C, throttle clean이었다. MCAP은 97.282초 동안 13,649개 메시지를 기록했고
`/cmd_vel`과 `/cmd_vel_nav`는 모두 0건이었다. metadata가 생성됐고 종료 뒤 autorun,
route, Nav2 container와 recorder 잔존 프로세스는 0개였다. 이 결과는 비주행 기동 게이트를
검증한 것이며 장애물 대응이나 경로 수행 성능 표본은 아니다.

## 2026-09-10 기록의 정지 판단시간 재분석

새 실차 주행 대신 `real_combined_obstacle_retry_20260910T122006` MCAP을 다시 읽어 실제
StopZone 6개를 분석했다. 기록상 직전 scan과 STOP 상태 사이의 시간 간격은 p95·최대
56.543 ms, STOP부터 최종 0 속도 명령은 p95·최대 1.025 ms였다. 100 ms 연속 정지를 요구한
wheel odom 기준에서는 6개 중 4개가 정지로 확인됐고 STOP부터 정지까지 p95·최대
256.444 ms, 직전 scan부터 wheel odom 정지까지의 기록 간격은 최대 305.961 ms였다. 어떤
scan이 STOP을 유발했는지 연결하는 trace ID가 없으므로 이를 end-to-end 판단시간으로
간주하거나 목표 상한 400 ms 통과로 판정하지 않는다. wheel odom 정지도 외부 센서로 측정한
실제 차체 정지가 아니다.

두 사건은 보호영역이 각각 138.538 ms, 300.276 ms 만에 해제돼 100 ms 연속 wheel odom
정지를 확인하지 못했다. 특히 후자는 해제 직전에도 wheel odom 선속도가 약 0.085 m/s여서
`정지 6회`가 아니라 `StopZone 개입 6회, wheel odom 정지 확인 4회`로 표현한다. 장애물 해제
뒤 비영 명령 재개는 6개 모두 확인됐고 최대 69.480 ms였다. STOP 뒤 다른 `/plan` 형상은
6개 모두 0.896초 안에 관측됐지만 주기적 재계획과 장애물 인과를 구분하는 trace ID가 없어
시간적 연관으로만 기록한다.

재현 출력은 `$HOME/jdamr_artifacts/real_combined_obstacle_retry_20260910T122006_latency/`
아래 `stop_latency.json`과 `stop_latency.csv`다.

## 마지막 실차 주행의 실제 지도 3D 재현 범위

직선 복도 시험 월드로 좌표를 축소하는 방식은 최종 재현에서 제외했다. 실제 저장 지도
`autonomous_20260826T161908.pgm`의 점유 셀을 5 cm 해상도와 원점
`(-2.527, -8.975)` 그대로 Gazebo 충돌·시각 메시로 돌출하고, 20개 waypoint도 축척이나
좌표 변환 없이 사용한다. 원본 PGM의 SHA-256은
`ee9b0911f41a7da31a92eca67d261b2a96c286f6f49d65b3d6c884fb5937fc6e`이며, 894×212
격자에서 점유 셀 6,312개를 누락 없이 757개 직육면체 묶음으로 변환했다.

이 결과는 실제 평면 형상을 쓰는 Gazebo 2.5D 기능 재현이다. 저장 지도에는 벽 높이·재질·문·
가구 형상이 없으므로 벽 높이 2.4 m는 명시적인 시각화 가정이다. 일시 장애물의 실제 치수와
사람·박스 의미 분류, 실측 바닥 마찰계수, 실제 센서·네트워크 노이즈도 복원하지 않는다.
따라서 결과 등급은
`actual_occupancy_extrusion_functional_replay_not_3d_digital_twin`으로 고정한다.

경로 위 2.0×1.2 m 구간에 ODE `mu=mu2=0.05`를 적용해 이미 이물질을 밟아 마찰이
떨어진 상황을 물리적으로 재현한다. 이 수치는 실측값이 아닌 합성 스트레스 조건이다. 장애물은
로봇 전방 0.75 m에 배치해 0.28 m LiDAR 최소 거리 밖이면서 0.65 m StopZone 안으로
진입시킨다. 2 Hz LiDAR에서도 비접촉 긴급정지가 관측되도록 장애물 표면은 베이스에서 약
0.50 m 떨어진다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
python3 jdamr_cube_navigation/evaluation/generate_sim_nav_obstacle_assets.py \
  --output jdamr_cube_navigation/evaluation/assets/nav_obstacle
python3 jdamr_cube_navigation/evaluation/prepare_restaurant_traction_sim.py \
  --route jdamr_cube_navigation/config/corridor_roundtrip.autonomous_20260826.yaml \
  --route-log "$HOME/jdamr_artifacts/real_combined_obstacle_retry_20260910T122006.route.log" \
  --stop-events "$HOME/jdamr_artifacts/real_combined_obstacle_retry_20260910T122006_latency/stop_latency.csv" \
  --source-world jdamr_cube_gazebo/worlds/slam_corridor.world \
  --base-bridge jdamr_cube_gazebo/params/bridge.yaml \
  --output-dir "$HOME/jdamr_artifacts/restaurant_traction_sim_20260911"
python3 jdamr_cube_navigation/evaluation/prepare_actual_map_restaurant_sim.py \
  --base-contract "$HOME/jdamr_artifacts/restaurant_traction_sim_20260911/scenario_contract.json" \
  --route jdamr_cube_navigation/config/corridor_roundtrip.autonomous_20260826.yaml \
  --map-yaml "$HOME/maps/autonomous_20260826T161908.yaml" \
  --robot-urdf jdamr_cube_navigation/evaluation/assets/nav_obstacle/jdamr_cube_nav_eval.urdf \
  --output-dir "$HOME/jdamr_artifacts/restaurant_actual_map_3d_v11"
```

### 저마찰 월드 1회 smoke 대조

동일한 Cartographer 설정, seed 42, 14 m 왕복 명령으로 정상 바닥과 저마찰 바닥을 각각 한
번 실행했다. 두 실행 모두 MCAP, pbstream, map YAML/PGM을 생성했고 프로세스 잔존 없이
종료했다. 정상 바닥은 정답 궤적 비율 99.54%, 왕복 완료 PASS, ATE RMS 0.578 m였다. ODE
`mu=mu2=0.05` 접촉면을 넣은 경우 정답 궤적 비율 92.83%, 복귀 진행 10.837/14 m로 완료
게이트를 통과하지 못했고 ATE RMS는 1.010 m였다. 이 단일 seed 결과는 저마찰 패치가 물리
주행 차이를 만들었다는 smoke 증거이며, 복구 제어 성공이나 실차 마찰계수 재현 증거는
아니다.

## 실제 지도 3D 통합 기능 재현 결과

`restaurant_actual_map_integrated_v7_seed42_attempt1`은 실제 점유격자 월드에서 Nav2 20개
목표를 20/20 완료했고, 실차 기록에 대응하는 6개 장애물 개입도 6/6 긴급정지·장애물 제거·
동일 goal 재개까지 통과했다. 마지막 home 목표 완료 시각은 시나리오 기준 511.699초다.

저마찰 상태는 `NAVIGATING → PROTECTIVE_STOP → RELOCALIZE → LOW_SPEED_RESUME →
RECOVERED`로 한 번 복구됐다. 전이는 각각 129.12초, 129.88초, 130.66초, 135.66초에
기록됐다. 제한 재개 속도는 80%, 제한 재개 시간은 5초다. 15초 grace는 고정 대기가 아니라
저속으로 2 m 저마찰 구간을 빠져나가는 동안 재검출을 억제하며 상태를 계속 감시하는 시간이다.
두 번째 지속 이상은 자동 재개하지 않고 `FAULT_LATCHED`로 전환한다.

장애물 개입 중 Collision Monitor가 STOP인 표본과 해제 후 1초는 마찰 판정에서 제외한다.
따라서 장애물 정지와 마찰 이상 정지를 중복 분류하지 않는다. 최종 판정·MCAP·동시 카메라
원본·2배속 합성 영상은
`$HOME/jdamr_artifacts/restaurant_actual_map_integrated_v7/run_seed_42_attempt_1/`에 묶는다.
영상은 실제 지도 전체를 보는 Gazebo 고정 카메라와 로봇 전방 카메라를 동시에 보여주며,
각 카메라의 실제 수신 프레임 수와 steady-clock 기록 시간으로 시간축을 맞춘다. 최종 영상은
267.867초, 1,280×480, 15 fps, H.264/yuv420p이며 SHA-256은
`dca98eab99695b9ea272a9c341b48abe53258260d481e4a8872ddbf8f90d33a8`이다. MCAP은
75,442,779바이트이며 SHA-256은
`00207813eac979108c4c4829cf66dfa1677d9c650d278ac504661d6035ee4785`다. 종료 뒤 잔존
프로세스 그룹과 실행 식별자 일치 프로세스는 모두 0개다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
python3 jdamr_cube_navigation/evaluation/run_restaurant_replay_sim.py \
  --scenario-contract "$HOME/jdamr_artifacts/restaurant_actual_map_3d_v11/scenario_contract.json" \
  --output-dir "$HOME/jdamr_artifacts/restaurant_actual_map_integrated_next/run_seed_42_attempt_1" \
  --run-id restaurant_actual_map_integrated_next_seed42_attempt1 \
  --domain-id 198 \
  --seed 42
```

## 실제 장애물 형상과 사람 횡단을 포함한 3D 재현

최종 실행 `restaurant_actual_map_enhanced_v5`는 실차 영상과 정지 기록을 세 장면으로
묶었다. `IMG_7861.mov`에 대응하는 첫 박스와 `IMG_7862.mov`의 두 번째 박스는 각각
5→6번, 8→9번 waypoint 사이에 고정 골판지 상자로 배치했다. 로봇은 상자 표면에서
최소 0.599 m와 0.615 m를 유지했고 최대 각속도 0.2 rad/s와 0.5 rad/s로 선회한 뒤 같은
goal을 완료했다. 나머지 두 정지 기록은 `IMG_7862.mov`의 사람·박스 동시 등장 장면으로
묶어, 사람 형상 모델이 경로를 횡단할 때 StopZone 긴급정지 후 사람이 빠져나가면 같은
goal을 재개하도록 했다. 20개 waypoint와 세 장면은 모두 PASS했다.

실차 베이스에 없던 SO-101 링크·조인트·제어 플러그인은 생성 URDF에서 제거했고, Gazebo
launch도 베이스 전용 실행에서는 팔 컨트롤러 spawner를 시작하지 않는다. 주황색 바닥은
원본 Keepout PGM의 검은 셀 9,406개를 같은 5 cm 격자 좌표에 올린 마스킹 영역이며,
황색 바닥은 실측 마찰계수가 아닌 저마찰 합성 스트레스 구역이다. 저마찰 판단은
시뮬레이터 주입 활성 신호를 시작 조건으로 사용하고, 정지·재위치화·제한속도 재개·복구는
별도 상태기계가 수행한다. 이는 실차 검출기 성능 증거가 아니라, 마찰 이상이 검출됐다고
가정한 이후의 대응 로직 검증이다. 상태는 `PROTECTIVE_STOP → RELOCALIZE →
LOW_SPEED_RESUME → RECOVERED`로 한 번 전이했다.

최종 영상은 고정 3D 지도 시점과 베이스 전방 카메라를 함께 표시한다. 첫·마지막 프레임만
맞추는 선형 보정 대신 모든 수신 프레임의 Gazebo 타임스탬프로 두 카메라를 15 fps 공통
시간축에 재표본화했다. 이 때문에 카메라별 프레임 누락률이 달라도 박스 선회와 사람
긴급정지 자막이 같은 장면에 놓인다. 결과는 268.200초, 1,280×480, H.264이며 경로는
`$HOME/jdamr_artifacts/restaurant_actual_map_enhanced_v5/gazebo_actual_map_2x_frame_synced.mp4`,
SHA-256은 `8f793abb17003eb38f4993dbc564551d99df3a6303c080dfaf5ebf1f5a1ac802`다.
검증 요약은 같은 디렉터리의 `summary.json`에 있으며 종료 뒤 잔존 프로세스는 0개다.

차량 가림을 개선한 `restaurant_actual_map_visible_vehicle_v3`에서는 충돌 판정용 벽을 원래
2.4 m로 유지하되 화면 표시용 벽만 0.65 m 절개형 메시로 분리했다. 전체 지도 카메라는 더
높은 사선 시점으로 옮겼고, 베이스 뒤 1.8 m·위 1.8 m의 후상방 추적 카메라를 추가해 차체와
주변 장애물을 함께 표시한다. 동일 seed 42 재생에서 waypoint 20/20, 박스 회피 2건과 사람
긴급정지 1건을 포함한 장면 3/3, 저마찰 복구 1회를 통과했다. 포트폴리오 영상은 전체 지도
프레임의 빈 상·하단을 잘라내고 두 카메라를 같은 높이로 정렬했다. 상태 자막은 굵은 고정
폰트와 고대비 불투명 배너를 적용했다. 최종 4배속 영상은 133.867초, 1,760×360, 15 fps,
H.264/yuv420p이며 경로는
`$HOME/jdamr_artifacts/restaurant_actual_map_visible_vehicle_v3/gazebo_actual_map_4x.mp4`,
SHA-256은 `0c9a5aac41a136239365d82f51233954c93ed705eb0036955639ff0c530a7678`다.

이 결과는 실제 점유격자·Keepout·주행 구간·관측 장면을 연결한 데이터 기반 기능 재현형
디지털 트윈이다. 저장 지도에 없는 벽 재질·문·가구·실측 장애물 치수까지 복원한
사진측량형 3D 복제본은 아니다.

## 설계 근거

- [ISO 3691-4:2023](https://www.iso.org/standard/83545.html): AMR 운용 구역과 안전 요구·검증
- [ISO 13855:2024](https://www.iso.org/standard/80590.html): 사람 접근과 응답시간을 반영한 보호영역 분리거리
- [Nav2 Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/): planner 우회의 stop·slow·limit·TTC 계층
- [Nav2 recovery BT](https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/detailed_behavior_tree_walkthrough/detailed_behavior_tree_walkthrough/): Wait·BackUp·재계획의 적용 조건
- [Nav2 tuning guide](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/): 2D ObstacleLayer와 3D VoxelLayer의 센서 범위
- [ROS 2 composition](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html): isolated·multi-threaded executor 선택
