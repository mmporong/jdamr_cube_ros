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

## 마지막 실차 주행의 시뮬레이션 재현 범위

마지막 실차 기록을 디지털 트윈처럼 똑같이 복제할 수는 없다. MCAP에는 실제 건물의 충돌
형상, 바닥 마찰계수, 장애물의 의미 분류와 공통 hardware timecode가 없다. 대신 실차의 20개
waypoint 왕복 순서, StopZone 개입 6회의 활성 waypoint, 동일 goal 재개 조건을 보존한 기능
재현은 가능하다. 실차 좌표는 통제된 Gazebo 복도의 왕복 차선으로 변환하고, 장애물 이벤트는
불안정한 벽시계 시간이 아니라 활성 waypoint에 결합한다.

`prepare_restaurant_traction_sim.py`는 실차 route·route log·정지 CSV의 SHA-256을 묶은
`scenario_contract.json`과 `slam_corridor_traction.world`를 만든다. 저마찰 구간은 경로 위의
실제 접촉면이며 ODE `mu/mu2`를 낮춘 합성 stress다. 이는 실차 바닥에서 측정한 마찰값이
아니며, 이물질 탐지나 물체 회피 결과로 사용하지 않는다. 재현 등급은
`functional_scenario_contract_not_digital_twin`으로 고정한다. 현재 구현 범위는 20개 waypoint
좌표 변환, 실차 StopZone 6회의 활성 waypoint 결합, 물리 저마찰 접촉면 생성까지다. 변환된
20개 waypoint의 Nav2 실행, 장애물 6개 자동 투입, `TRACTION_FAULT` 정지·재위치추정·1회
저속 재개 supervisor는 후속 통합 대상이며 현재 완료로 간주하지 않는다.

지도 파일과 Keepout mask는 원본 occupancy grid를 그대로 로드할 수 있다. 그러나 해당 2D
grid에는 벽 높이·재질·문·가구의 3D 형상이 없다. 점유 셀을 일정 높이로 돌출한 2.5D 충돌
월드는 만들 수 있지만 높이는 가정값이다. 일시적으로 등장한 장애물은 저장 지도에 없으며
LiDAR와 AMCL로 위치·2D 외곽을 근사할 수 있을 뿐, 박스나 사람의 실제 가로·세로·높이를
동일하게 복원할 수 없다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
python3 jdamr_cube_navigation/evaluation/prepare_restaurant_traction_sim.py \
  --route jdamr_cube_navigation/config/corridor_roundtrip.autonomous_20260826.yaml \
  --route-log "$HOME/jdamr_artifacts/real_combined_obstacle_retry_20260910T122006.route.log" \
  --stop-events "$HOME/jdamr_artifacts/real_combined_obstacle_retry_20260910T122006_latency/stop_latency.csv" \
  --source-world jdamr_cube_gazebo/worlds/slam_corridor.world \
  --output-dir "$HOME/jdamr_artifacts/restaurant_traction_sim_20260911"
```

### 저마찰 월드 1회 smoke 대조

동일한 Cartographer 설정, seed 42, 14 m 왕복 명령으로 정상 바닥과 저마찰 바닥을 각각 한
번 실행했다. 두 실행 모두 MCAP, pbstream, map YAML/PGM을 생성했고 프로세스 잔존 없이
종료했다. 정상 바닥은 정답 궤적 비율 99.54%, 왕복 완료 PASS, ATE RMS 0.578 m였다. ODE
`mu=mu2=0.05` 접촉면을 넣은 경우 정답 궤적 비율 92.83%, 복귀 진행 10.837/14 m로 완료
게이트를 통과하지 못했고 ATE RMS는 1.010 m였다. 이 단일 seed 결과는 저마찰 패치가 물리
주행 차이를 만들었다는 smoke 증거이며, 복구 제어 성공이나 실차 마찰계수 재현 증거는
아니다.

## 설계 근거

- [ISO 3691-4:2023](https://www.iso.org/standard/83545.html): AMR 운용 구역과 안전 요구·검증
- [ISO 13855:2024](https://www.iso.org/standard/80590.html): 사람 접근과 응답시간을 반영한 보호영역 분리거리
- [Nav2 Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/): planner 우회의 stop·slow·limit·TTC 계층
- [Nav2 recovery BT](https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/detailed_behavior_tree_walkthrough/detailed_behavior_tree_walkthrough/): Wait·BackUp·재계획의 적용 조건
- [Nav2 tuning guide](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/): 2D ObstacleLayer와 3D VoxelLayer의 센서 범위
- [ROS 2 composition](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html): isolated·multi-threaded executor 선택
