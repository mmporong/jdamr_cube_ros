# 돌발상황 대처 로직: 현업 조사와 적용 (2026-10-02)

서빙 로봇이 주행 중 이상을 만났을 때 무엇을 먼저 막고 누구에게 알리는지 표준·프로토콜·상용 제품·Nav2 기본값으로 조사했다. 그 결과를 우리 로봇의 기존 처리와 대조해 우선순위를 정하고, 위에서부터 구현했다.

- 조사: 웹 문헌(표준 미리보기, 공식 문서, 소스, 제품 매뉴얼). 유료 표준 본문(ISO 13482, ISO 3691-4, R15.08, UL 3300)은 보지 못했고 공개 미리보기·인증기관 요약만 썼다.
- 기존 처리 현황: 저장소 코드와 펌웨어를 전수 조사했다(11개 영역).
- 구현: 브랜치 `feat/lattice-planner-trial`. 실차 주행 확인은 아직 하지 않았다.
- `(추론)` 표시는 출처가 아니라 판단이다.

## 1. 결론

1. 표준이 정하는 축은 **정지와 재시작**이다. 비상정지는 모든 기능보다 우선하고, 리셋이나 통신 복구가 자동 재시작이 되면 안 된다 [IEC 60204-1 9.2.4, ISO 13850].
2. **"통신이 N초 끊기면 귀환"을 정한 표준·프로토콜·서빙로봇 문서는 찾지 못했다.**
   - VDA 5050: 연결이 끊긴 차량은 이미 허가된 구간(base)의 끝까지 가서 정지한다.
   - Bear Servi: "The robot can run without WiFi".
   - 귀환은 원격 조종 로봇이 통신을 되찾으려는 패턴이다(Spot AutoReturn, iRobot retro-traverse).
3. **위치를 잃으면 모든 출처가 정지 + 사람 개입이다.** 스스로 귀환하지 않는다.
   - VDA 5050 v3: FATAL, 자율 주행 재개 금지.
   - Pudu: "I'm lost. Please push me…".
4. **막힘은 대기 → 재계획 → 운영자 호출 순서다.**
   - Pudu: 기본 30 s 대기 후 재계획.
   - Nav2 기본 BT: 정리·회전·5 s 대기·후진을 최대 6회.
5. **"충돌하면 운영자 호출"은 맞다.** 다만 순서는 정지 → 래치 → 현장 알림 → 호출 → 사람이 확인하고 재개다(Pudu 충돌 센서 동작).
6. **Nav2 Collision Monitor는 안전 인증 장치가 아니다.** 문서에 "does not provide hard real-time safety certifications"라고 적혀 있다.
7. **(추론) 우리 로봇은 물리 비상정지가 없어서 PC 정지 명령이 유일한 원격 정지 수단이다.** 그래서 PC 연결이 끊기면 다음 목표를 시작하지 않고 그 자리에 서 있는 쪽이 맞다(IEC 60204-1의 "정해진 시간 후 정지, 정지 전 미리 정한 상태 허용").

## 2. 표준과 프로토콜 요약

| 출처 | 우리와의 관계 | 가져온 원칙 |
|---|---|---|
| ISO 13482:2014 (mobile servant robot) | 해당 | 보호정지·안전 상태 정의. 5.16 위치·주행 오류. 5.4 재시작 |
| UL 3300 | 해당 (식당 명시) | 존재·이동을 알리는 가청·가시 표시 필수 |
| ISO 3691-4 | 참고 (public zone은 ISO 13482로 넘김) | 정지 원인이 해소된 뒤에만 재시작. 감지 시험편: 누운 70 mm 원통 |
| IEC 60204-1 9.2.4 | 무선 *조종* 장치 기준 | 통신 상실 → 위험성 평가로 정한 시간 후 정지. 복구돼도 자동 재시작 금지. 휴대 무선 비상정지는 "sole means"가 될 수 없음 |
| ISO 13850 | 비상정지 | 카테고리 0/1만 허용. 모든 기능에 우선. 리셋이 재시작이 되면 안 됨 |
| VDA 5050 v2.1/v3 | 플릿 프로토콜 | 끊기면 base 끝까지 수행 후 정지. 오류 4단계(WARNING·URGENT·CRITICAL·FATAL). 위치 상실 FATAL. 막힘은 재시도하지 말고 지시 대기 |
| Open-RMF | 플릿 관리 | 위치 상실 → Error 티켓 + stop. 비상 신호 → 가까운 주차 지점 |

## 3. 상용 서빙로봇 문서 요약

| 상황 | Pudu | Bear | Keenon |
|---|---|---|---|
| 막힘 | 기본 30 s 뒤 재계획(10–600 s), 막힌 경로 180 s 잠금 | "I'm stuck" 표시 | 정지·소리, 계속되면 사람 |
| 위치 상실 | 시작점으로 밀어 달라고 요청 | 알려진 지점으로 밀어 재위치 | 충전대로 옮기면 자동 복구 |
| Wi-Fi 상실 | 4G 전환. 주행 중 동작은 문서 없음 | Servi: Wi-Fi 없이 주행 | 문서 없음 |
| 배터리 | 기본 5%에 자동 복귀, 20% 충전 권장 | 10%에 경고(수동 충전) | 5%에 자동 충전 |
| 충돌 | 충돌 센서 → 정지·일시정지, 사람이 재개 | 언급 없음 | 충돌 스위치 고장 시 주행 실패 |
| E-stop 해제 | 재개 | 미션 취소 | 재개 |
| 알림 | 앱·워치·호출기 | 바닥 LED 색 | 음성·내선·SMS |

## 4. 우선순위와 우리 상태

담당 계층: L0 ESP32, L1 base driver, L2 로봇 내부 안전, L3 Nav2, L4 실행기, L5 운영자 PC.

| 순위 | 상황 | 업계 대응 | 이번 작업 전 | 이번 작업 후 |
|---|---|---|---|---|
| P0 | 물리 비상정지·범퍼 없음, LiDAR 근접 사각(0.28 m), 낮은 장애물 | 하드웨어로 해결 | 없음 | 없음 (하드웨어 과제, 5절) |
| P1 | 임박 충돌 | 보호 영역에서 감속·정지, 비키면 재개 | Collision Monitor 정지·감속·접근 (L2) | 같음 |
| P2 | 센서 데이터 끊김 | 정지, 지속되면 호출 | 실행기 스캔·odom 2.5 s, CM 1.0 s (L2·L4) | 실패 시 **운영자 호출 FATAL sensor** |
| P3 | 구동 명령 단절 | 펌웨어 정지, 운영자 확인 전 재개 금지 | ESP32 400 ms, 드라이버 0.4 s (L0·L1) | 같음 + **래치 비상정지** |
| P4 | 접촉 | 정지·래치·알림·호출·수동 재개 | 감지 수단 없음 | 비상정지 래치·호출 경로는 준비. 감지는 하드웨어 과제 |
| P5 | 들림·밀림·바퀴 걸림 | 정지·알람 | 없음 | 없음 (5절) |
| P6 | 위치 상실 | 정지, 자율 귀환 금지, 사람이 기준점으로 | AMCL 신선도·정지 확인, 공분산 게이트는 운영자 지시로 꺼짐 | 실패 시 **운영자 호출 FATAL localization** |
| P7 | Nav2 붕괴 | 정지(명령 단절), 재연결 후 호출 | 런치 Shutdown → 드라이버·펌웨어 정지 | 취소 미확인 시 **비상정지 래치 + FATAL software** |
| P8 | 하드웨어 이상 | 정지·호출 | 서보 오류 로그만 | 없음 (5절) |
| P9 | 경로 막힘 | 대기 → 재계획 → 호출 | 진행 검사 10 s → 105 → BT 1 s 대기 1회 → 실패 | **20 s 대기 후 같은 구간 재시도 최대 3회 → CRITICAL path_blocked** |
| P10 | 배터리 저하 | 경고 → 새 작업 거부 → 현 작업 마치고 복귀 → 위급 시 정지 | 출발 예비 10.8 V, 주행 하한 10.5 V에서 그 자리 정지 | **정차 사이 10.8 V 미만이면 남은 정차를 건너뛰고 도크로 + URGENT battery** |
| P11 | 운영자 PC 연결 두절 | 허가된 구간까지 마치고 정지, 복구돼도 자동 재개 금지 | 연결과 무관하게 주행 계속. PC는 ssh 실패를 사이클 종료로 오판 | **하트비트 30 s: 다음 목표를 시작하지 않고 정지 + URGENT operator_link.** PC는 끊김을 "알 수 없음"으로 보고 최대 600 s 계속 추적 |
| P12 | 작업 예외 | 시간 초과 시 완료·복귀·알림 | task deadline 240 s | 실패 시 운영자 호출 CRITICAL task_failed |

## 5. 이번 구현 (2026-10-02)

### 5.1 래치 비상정지 (P3, P7, ISO 13850 방식)

- **모터 드라이버** (`jdamr_base_driver`, `estop_latch.hpp`)
  - `/emergency_stop`(std_msgs/Bool)에 true가 오면 걸린다. 걸린 동안은 매 주기(50 Hz) 정지 프레임을 보내고 속도 명령을 무시한다.
  - 해제는 `emergency_stop_reset`(std_srvs/Trigger) 서비스로만 된다. false 발행으로는 풀리지 않는다.
  - 0이 아닌 속도 명령이 아직 들어오는 동안은 해제를 거부한다. 리셋 자체로 움직이지 않게 하기 위해서다.
  - 상태는 `/emergency_stop_state`(transient local)로 변할 때와 1 Hz로 발행한다.
- **실행기**
  - 상태를 구독하고, 걸려 있으면 모든 가드가 `emergency stop engaged`로 실패한다(재시도 대상 아님). 진행 중인 목표는 취소되고 출발도 막힌다.
  - Nav2 목표 취소가 5 s 안에 확인되지 않으면 실행기가 `/emergency_stop`을 걸고 `emergency_stop_requested`를 기록한다.
- **PC 도구**
  - `jdamr_depart.py estop`: 비상정지를 걸고, 진행 중인 시도도 멈춘다.
  - `jdamr_depart.py estop-reset`: 해제한다.
  - `status`에 비상정지 상태를 표시한다.
- **한계:** Wi-Fi와 ssh를 거치는 소프트웨어 정지라 몇 초가 걸린다. 표준상 비상정지가 아니다.

### 5.2 경로 막힘: 대기 → 재시도 → 호출 (P9)

- 중간 구간(최종 주차 제외)의 NavigateToPose·NavigateThroughPoses가 다음 Nav2 코드로 끝나면 막힘으로 본다.
  - FollowPath: 104, 105, 106
  - 계획: 205, 206, 208, 305, 306, 308
- 막히면 제자리에서 `PATH_BLOCKED_WAIT_S` 20 s를 기다린 뒤 같은 구간을 다시 시도한다(완료한 경유점은 유지).
- 최대 `PATH_BLOCKED_RETRIES` 3회 뒤 `path_blocked_give_up`을 남기고 사이클을 끝낸다. 운영자 호출 등급은 CRITICAL이다.
- 대기 중 비상정지나 배터리 하한처럼 회복할 수 없는 가드가 걸리면 기다림을 멈춘다(`path_blocked_wait_aborted`).
- (10:33 추가, 실패 기록 §20.20) 컨트롤러 정지(104·105·106)이고 차체 앞 끝에서 0.30 m 안에 물체가 보이면, 기다리기 전에 0.10 m 직선 후진한다(Nav2 기본 트리의 BackUp에 해당). 옆에 있는 물체로는 후진하지 않는다.
- 경로가 없는 정지(205·206·208·305·306·308)는 2 s마다 다시 계획해 보고, 경로가 생기면 바로 재시도한다(`path_blocked_cleared`). 비킨 사람을 20 s 동안 기다리지 않는다.
- (10:57 추가, 실패 기록 §20.21) 컨트롤러 정지도 2 s마다 앞 0.30 m 띠를 확인해 두 번 연속 비면 바로 재시도한다. 대기점 구간은 Lattice가 경로를 못 찾으면 NavFn으로 계획한다.
- 실차에서 처음 막힌 원인은 정지 구역 모양이었다. 바퀴 폭 직사각형이 프레임 옆 6.5 cm의 책상 다리를 모든 방향 정지로 잡았다. 그래서 차체 외곽과 정지 구역을 실측 계단형으로 바꿨다.
- (추론) 사람 때문에 막혔을 때 한 번의 시도는 진행 검사 10 s, BT 재시도, 대기 20 s를 합쳐 약 40 s다. 호출까지는 약 2분이다. 시작값이며 실측으로 조정한다.

### 5.3 운영자 호출 (P2, P6, P9–P12)

- 사이클이 실패로 끝나면 실행기가 `operator_call`을 한 번 남긴다. 필드는 `level`, `category`, `reason`, `failures`다.
- 그 실행에서 기록된 실패 이벤트 중 가장 심각한 원인을 고른다.

| 등급 (VDA 5050 v3 차용) | 범주 |
|---|---|
| FATAL | emergency_stop, software, localization, sensor |
| CRITICAL | path_blocked, task_failed |
| URGENT | operator_link, battery |

- 운영자가 보낸 정지(`stop`, `session-stop`)로 끝난 경우에는 호출하지 않는다.
- PC 도구는 `operator_call`을 받으면 아래를 수행한다.
  - `OPERATOR CALL` 줄 출력
  - 터미널 벨
  - `notify-send` 데스크톱 알림
  - `JDAMR_OPERATOR_NOTIFY_CMD` 환경변수가 있을 때만 그 명령 실행(예: 텔레그램 헬퍼). 없으면 PC 밖으로 나가는 알림은 없다.

### 5.4 운영자 연결 하트비트 (P11)

- `go`는 시작할 때 파이에 `~/jdamr_data/box_executor_spool/<run>.heartbeat`를 만들고, 10 s 폴링마다 같은 ssh 명령으로 갱신한다.
- 실행기는 매 목표(주행, 후진, 회전)를 보내기 전에 하트비트 나이를 본다.
  - 30 s를 넘으면 `departure_blocked`(`operator_link_lost`)를 남긴다.
  - 그 자리에서 사이클을 끝내고 URGENT 운영자 호출을 남긴다.
- 진행 중인 목표는 끝까지 간다(VDA 5050 base). 도크로 귀환시키지 않는다. 귀환도 감독 없는 주행이기 때문이다. 연결이 돌아와도 스스로 재개하지 않는다.
- PC에서 `go`를 Ctrl-C로 끊으면 하트비트가 멈춘다. 그래서 로봇은 지금 목표를 마친 뒤 멈춘다(이전에는 계속 주행했다).
- PC 쪽도 ssh 실패(코드 255, 시간 초과)를 "사이클 종료"가 아니라 "알 수 없음"으로 본다. 기록을 멈추지 않고 최대 600 s 계속 추적한다. 실패한 읽기가 이미 본 이벤트 수를 0으로 되돌려 이벤트를 다시 출력하던 문제도 고쳤다.

### 5.5 저전압 복귀 (P10)

- 물 받는 곳 정차와 이탈을 마친 뒤 전압이 출발 예비 전압(10.8 V) 미만이면 `battery_return`을 남긴다. 테이블은 건너뛰고 도크로 돌아간 뒤 URGENT 운영자 호출을 남긴다.
- 주행 하한(10.5 V)은 그대로 그 자리 정지다(위급).
- 충전은 수동이고 감지하지 않으므로, 연속 주행의 다음 출발은 기존 출발 예비 검사가 막는다.

## 6. 남은 공백

**하드웨어 (로직으로 메울 수 없음)**

1. 물리 비상정지: 모터 전원을 끊는 버섯형 스위치, 상태는 ESP32 입력.
2. 범퍼·접촉 스트립: 접촉 감지와 래치의 입력.
3. LiDAR 근접 사각: 최소 거리 0.28 m, 레이저가 x −0.010 m에 있다. 정면 StopZone 띠(x 0.085–0.135 m)는 스캔에 잡히지 않는다.
   - Collision Monitor 정지는 "점이 사라지면" 풀린다. 그래서 장애물이 사각 안으로 들어오면 정지가 풀릴 수 있다(추론).
4. 스캔 평면(0.15 m) 밖 장애물: 의자 좌판, 테이블 상판, 누운 물체. RGB-D를 Collision Monitor 소스로 넣는 방안은 Pi 4 부하를 실측해야 한다.
5. 현장 알림 장치(소리·LED): UL 3300 요구. Wi-Fi가 끊기면 지금은 현장에서 알 길이 없다.

**소프트웨어 (다음 후보)**

- 접촉 간접 감지(IMU 감속도, 명령 대비 속도 불일치): 급제동 오인 위험이 있어 실측이 먼저다.
- 들림·밀림 감지, 서보 오류 플래그를 정지로 연결.
- 주행 중 위치 건전성 재검사. 공분산 게이트는 2026-09-30 운영자 지시로 꺼져 있다.
- Nav2 bond는 2026-09-30 heartbeat 정지 사고 뒤 꺼 두었다.

## 7. 확인

- **테스트**
  - 실행기 4개 파일 431건 통과. 신규: 비상정지 가드, 취소 미확인 시 래치, 막힘 대기·재시도·포기·중단, 막힘 코드 구분, 호출 분류 6종, 하트비트, 저전압 복귀, 저전압 흐름, 운영자 정지 무호출.
  - PC 도구 38건 통과(ssh 끊김 판정, 호출 알림).
  - 드라이버 gtest 20건 통과(래치 3건).
  - `ament_flake8` 통과(실행기 파일).
- **파이 반영 (2026-10-02 00:38–00:45, 이동 없음)**
  - 백업: `~/jdamr_data/deploy_backup_20261002_003758/`.
  - 드라이버와 실행기를 빌드했다. 실행기 설치본 SHA-256이 로컬과 같다.
  - 드라이버는 8월 25일 빌드였다. 그래서 그 뒤 들어간 차륜 제원 범위 검사(`geometry.hpp`)도 함께 반영됐다. 파이 값(반지름 0.0329 m, 간격 0.510 m, 비 1.0)은 범위 안이다.
  - `jdamr_depart.py recover`로 드라이버·세션·실행기·화면을 재시작하고 초기화했다(AMCL과 스캔 정합 0.006 m, DDS OK).
- **비상정지 실기 확인 (정지 상태)**
  - `status` released → `estop` engaged(드라이버 로그 "비상정지 걸림") → `estop-reset` released(드라이버 로그 "비상정지 해제").
- **실차 미확인**
  - 주행 중 비상정지
  - 막힘 대기·재시도
  - 하트비트 끊김 정지
  - 저전압 복귀
  - PC 알림 표시

## 출처

- ISO 13482:2014 미리보기 — https://cdn.standards.iteh.ai/samples/53820/5ddca453a6d141e5a558f4f791ea3229/ISO-13482-2014.pdf
- ISO 3691-4:2020 미리보기 — https://cdn.standards.iteh.ai/samples/70660/7c26a1bc79ae4d948f9b4752eb44433c/ISO-3691-4-2020.pdf
- TÜV Rheinland AGV 백서 — https://www.tuv.com/content-media-files/master-content/services/industrial-services/pdf/tuv-rheinland-automatic-guided-vehicles-whitepaper-en_neu.pdf
- IEC 60204-1:2016 해설(9.2.4) — https://www.jmf.or.jp/jmf/wp-content/uploads/2024/04/IEC-60204-1_2016.pdf
- ISO 13850 요약 — https://machinerysafety101.com/2026/05/18/iso-13850-emergency-stop-requirements/
- UL 3300 해설(ULSE) — https://ulse.org/insight/ul-standards-engagement-standards-matter-how-standards-are-making-robots-safe-public-and-commercial/
- LG CLOi ServeBot UL 3300 — https://www.prnewswire.com/news-releases/lg-announces-us-launch-of-cloi-servebot-worlds-first-service-robot-to-achieve-ul-certification-301458997.html
- VDA 5050 v2.1 — https://github.com/VDA5050/VDA5050/blob/2.1.0/VDA5050_EN.md
- VDA 5050 v3.0 — https://github.com/VDA5050/VDA5050/blob/3.0.0/VDA5050_EN.md
- MassRobotics AMR Interoperability Standard — https://github.com/MassRobotics-AMR/AMR_Interop_Standard
- Open-RMF EmergencyPullover — https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/src/rmf_fleet_adapter/events/EmergencyPullover.cpp
- Open-RMF RobotContext(set_lost) — https://github.com/open-rmf/rmf_ros2/blob/main/rmf_fleet_adapter/src/rmf_fleet_adapter/agv/RobotContext.cpp
- Nav2 Collision Monitor (Jazzy) — https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/
- nav2_collision_monitor README — https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_collision_monitor/README.md
- Nav2 기본 BT — https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
- Nav2 Pause Near Goal-Obstacle — https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/trees/nav_to_pose_and_pause_near_goal_obstacle/nav_to_pose_and_pause_near_goal_obstacle/
- Nav2 Lifecycle Manager — https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/
- diff_drive_controller (Jazzy) — https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
- Bad networks dragging down localhost communication — https://discourse.openrobotics.org/t/bad-networks-dragging-down-localhost-communication/20611
- Spot AutoReturn — https://dev.bostondynamics.com/docs/concepts/autonomy/auto_return
- Spot Autowalk — https://dev.bostondynamics.com/docs/concepts/autonomy/autowalk_service.html
- MiR Network and Wi-Fi Guide — https://jk.de/media/a2/cc/90/1739201876/MiR_Network_Wi-Fi_Guide.pdf
- Pudu BellaBot Pro 매뉴얼 — https://cdn.robotshop.com/media/P/Pud/RB-Pud-20/pdf/bellabot-pro-guide-v-1-240624.pdf
- Pudu KettyBot 매뉴얼 — https://6083570.fs1.hubspotusercontent-na1.net/hubfs/6083570/2022%20Robotics/Kettybot%20User%20Manual.pdf
- Bear Servi 매뉴얼 — https://www.bearrobotics.ai/s/Servi-User-Manual.pdf
- Bear Servi Plus 매뉴얼 — https://www.bearrobotics.ai/s/Servi-Plus_User-Manual_EN.pdf
- Keenon T8 매뉴얼 — https://store.todsystem.com/wp-content/uploads/2024/07/Guide-dutilisation-Dinnerbot-T8.pdf
- Keenon W3 매뉴얼 — https://6083570.fs1.hubspotusercontent-na1.net/hubfs/6083570/Knowledge%20base%20uploads/Butlerbot%20W3%20user%20manual.pdf
- Aethon TUG (RIA) — https://aethon.com/aethon-tug-autonomous-mobile-robot-discussed-ria-magazine/
- 산업용 로봇 검사기준 별표12 — https://www.law.go.kr/flDownload.do?flSeq=132875103
