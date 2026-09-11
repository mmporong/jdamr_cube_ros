# 식당형 AMR 안전·강건성 고도화 — 2026-09-11

## 결론

식당 상황은 유의미하다. 다만 현재 2D LiDAR로 검증 가능한 `통로 장애물 정지·재계획`,
wheel odom·IMU·localization 불일치로 판정하는 `마찰 이상 의심`, 접촉 센서가 있어야 하는
`충돌 사건`을 서로 다른 상태와 주장으로 분리한다. 고인 물이나 얇은 이물질을 G4로 직접
식별·회피했다고 주장하지 않는다.

## 먼저 닫을 기준선

Phase 1은 같은 출발 표시·방향·P-loop를 빈 mapping state에서 Cartographer로 3회 수집한다.
각 실행은 설정을 바꾸지 않고 map 저장 또는 SLAM 정지를 마친 뒤 로봇을 들어 올린다.
세 실행 모두 odom 폐루프 0.30 m 이하, SLAM 폐루프 0.10 m 이하, yaw 3도 이하, double wall
0건, scan overlap·역전·drop 0건을 요구한다. 이 세 실행의 scan·TF·CPU·위치 보정 분포를
식당 fault와 executor A/B의 기준선으로 쓴다. 저장 지도 장애물 주행은 이 매핑 재현성
게이트를 대신하지 않는다.

실차 이동은 작업자가 로봇 옆에서 물리 전원을 즉시 차단할 수 있을 때만 수행한다. 실행마다
raw MCAP, pbstream, map YAML/PGM, config hash, 종료 상태를 1:1로 연결한다.

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
| P1 | 마찰 저하·wheel slip | command↔odom, encoder yaw↔IMU, map→odom correction으로 `TRACTION_FAULT` 의심 | Gazebo 마찰 패치 우선, 실차는 건식 매트·최저속 |
| P2 | 낮은 음식물·케이블·액체 | 현재 센서로 검출 보장 불가 | 하향 depth/3D 또는 낮은 safety scanner 필요 |
| P2 | 실제 접촉 | 현재 센서로 확정 불가 | bumper/contact 또는 모터 전류·토크 입력 필요 |

```text
READY → NAVIGATING
  ├─ 보호영역 침입/입력 무효 → PROTECTIVE_STOP
  │    ├─ 입력 정상 + 유효 새 경로 → 동일 goal 저속 재개
  │    └─ 제한시간 초과/경로 없음 → FAULT_LATCHED
  ├─ 지속적인 센서 불일치 → TRACTION_FAULT → 정지·재위치추정 → 1회 저속 재개 또는 래치
  └─ 인증된 접촉 입력 → COLLISION_LATCHED → 구동 금지·목표 취소·수동 점검/리셋
```

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

## 설계 근거

- [ISO 3691-4:2023](https://www.iso.org/standard/83545.html): AMR 운용 구역과 안전 요구·검증
- [ISO 13855:2024](https://www.iso.org/standard/80590.html): 사람 접근과 응답시간을 반영한 보호영역 분리거리
- [Nav2 Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/): planner 우회의 stop·slow·limit·TTC 계층
- [Nav2 recovery BT](https://docs.nav2.org/rolling/getting_started/nav2_behavior_trees/detailed_behavior_tree_walkthrough/detailed_behavior_tree_walkthrough/): Wait·BackUp·재계획의 적용 조건
- [Nav2 tuning guide](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/): 2D ObstacleLayer와 3D VoxelLayer의 센서 범위
- [ROS 2 composition](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Composition.html): isolated·multi-threaded executor 선택
