# Nav2·논문 리서치와 현재 설정 대조 (2026-10-06, 실차 없음)

대상은 JD-AMR(차동구동, 0.12 m/s, Pi 4 4 GB, ROS 2 Jazzy, Nav2 1.3.12)의 Lattice + MPPI 기본 경로, Collision Monitor(CM), 라이다 자기 필터, 박스 정밀 주차다. 근거 등급은 세 가지로 적는다.

- **[확인]** 공식 문서·소스·설치된 헤더·논문 원문을 읽었거나, 이 PC에서 실행해 재현했다.
- **[2차]** 검색 요약이나 미러 문서만 읽었다.
- **[추론]** 위 근거에서 끌어낸 해석이다. 실차로 확인하지 않았다.

docs.nav2.org 설정 페이지는 이번 세션의 WebFetch에서 404가 났다. 그래서 파라미터 의미는 jazzy 브랜치 소스·README와 `/opt/ros/jazzy/include`의 헤더로 확인했다.

## 0. 요약

| # | 주제 | 결론 | 등급 | 조치 |
|---|---|---|---|---|
| 1 | Lattice 중간 경유점 방향 | Jazzy에는 `goal_heading_mode`가 없다. 중간 포즈 방향은 하드 제약이다. 지금처럼 "다음 경유점을 향하는 방향"을 넣는 방식이 맞다. | [확인]+실측 | 유지 |
| 2 | Lattice `rotation_penalty` | 리서치는 기본값 5.0 쪽을 권했다. 실제 지도 오프라인 프로브에서는 5.0이 경로를 2.05–3.85배로 늘렸다. 우리 값 0.5는 1.00–1.02배다. | 실측 | 0.5 유지 |
| 3 | Lattice 1 m 회전 반경 원시경로 | 0.5 m 세트보다 길다(1.05–1.47배). | 실측 | 0.5 m 유지 |
| 4 | MPPI `PathAlignCritic` | `offset_from_furthest` 20은 지평 0.36 m(약 7점)에서 거의 켜지지 않는다. 가중 14의 critic이 사실상 꺼져 있다. | [확인] | 실차 A/B 1순위 (§2.2) |
| 5 | MPPI `ax_max` | 0.5로, 스무더 `max_accel` 0.35보다 크다. MPPI가 실제보다 빠른 가속을 가정한다. | [확인] | 실차 A/B (§2.3) |
| 6 | MPPI 풋프린트 검사 부하 | 감쇠 10에서는 외접 반경 지점의 비용이 약 13이다. 장애물 근처의 거의 모든 궤적 점이 풋프린트 검사를 받는다. 감쇠 3에서는 약 105였다. | [확인]+계산 | 다음 주행 부하 기록으로 확인 (§2.4) |
| 7 | MPPI 제어 주기 | 10 Hz와 `model_dt` 0.1이 맞물려 제어열을 한 칸씩 민다. Pi 부하로 5.8 Hz까지 떨어지면 이 가정이 어긋난다. | [확인]+[추론] | 부하 기록 후 판단 (§2.5) |
| 8 | CM 외곽 안 장애물 교착 | APPROACH는 t=0에서 정적 검사를 먼저 하고, 안에 점이 있으면 속도를 0으로 만든다. 전용 해제 옵션은 없다. Nav2의 답은 방향별 다각형(PR #3708)이다. | [확인] | 현 구조(방향별 다각형 + 토글 되짚기)가 그 방향이다 |
| 9 | CM 토글 | 끄면 명령을 그대로 통과시킨다. 다만 소스 시간 초과 검사는 꺼진 상태에서도 정지시킨다. | [확인] 소스 | 되짚기 중 스캔이 끊기면 멈춘다(안전 쪽) |
| 10 | 자기 필터 출력값 | +inf와 NaN 모두 CM·AMCL·장애물 층(`inf_is_valid` 기본 false)에서 버려진다. REP 117의 의미로는 NaN(무효 측정)이 더 맞다. | [확인]+[추론] | +inf 유지. `inf_is_valid`를 켜게 되면 NaN으로 바꾼다 (§3.3) |
| 11 | opennav_docking | Jazzy에 `NonChargingDock`이 있다. 하지만 종료 판정 기본값이 5 cm이고 최저 속도가 0.1 m/s라 1 cm 목표에는 맞지 않는다. | [확인] 헤더·README | 박스 정밀 주차는 현 파이프라인 유지 |
| 12 | Astra S | 최소 거리가 0.4 m여서 박스 앞 5 cm 간격을 깊이로 잴 수 없다. 마지막 구간은 오도메트리나 라이다로 마무리해야 한다. | [확인] 제조사 규격 | 1–2 cm 계획의 전제 |
| 13 | 정밀도 주장 | 외부 측정 20회 이상으로 정확도(평균)와 반복성(산포)을 나눠 보고해야 한다. | [확인] 표준 개요·논문 | 실차 재개 시 첫 작업 |

## 1. Smac State Lattice

### 1.1 확인된 사실
- diff 원시경로(`/opt/ros/jazzy/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json`)는 헤딩 16개와 궤적 112개로 이루어져 있다. 반경 0의 제자리 회전 원시경로도 들어 있다. [확인] 로컬 파일
- 이 원시경로의 점 간격은 0.049–0.056 m(평균 0.051 m)다. §2.2의 근거가 된다. [확인] 로컬 파일 계산
- `rotation_penalty`는 제자리 회전 원시경로에만 붙는다. README는 "필요할 때가 아니면 불리하도록 높게"라고 쓰고, 기본값은 5.0이다. [확인] https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_smac_planner/README.md
- Jazzy `smac_planner_lattice.cpp`는 목표 방향을 가장 가까운 헤딩 칸으로 고정한다. `goal_heading_mode`(DEFAULT / BIDIRECTIONAL / ALL_DIRECTION)는 이후 배포판 기능이다. 설치된 1.3.12의 헤더와 `.so`에도 해당 문자열이 없다. [확인] 소스와 로컬 바이너리 검색. 도입 배포판은 미확인이다. https://discourse.openrobotics.org/t/nav2-enable-goal-orientation-flexibility-in-smac-planners/43475 , https://github.com/ros-navigation/navigation2/issues/5803
- `ComputePathThroughPoses`는 각 구간의 목표 포즈를 그대로 계획기에 넘긴다. 중간 포즈 방향을 고쳐 주지 않는다. [확인] https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_planner/src/planner_server.cpp
- 논문 근거: Pivtoraiko & Kelly 2005(IROS, 최소 생성 제어 집합), Pivtoraiko, Knepper & Kelly 2009(J. Field Robotics 26(3), 상태 격자의 모든 경로가 차량 모델을 만족). Macenski 외 Smac 논문(arXiv:2401.13078)은 5 cm 격자와 16 헤딩으로 실험했다. [확인] https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.20285 , https://arxiv.org/abs/2401.13078

### 1.2 오프라인 프로브 (이 PC, 도메인 77, 로봇 없음)
- 스크립트·결과·로그: `$HOME/jdamr_data/analysis_20261006/lattice_rotation/`(`run.sh`, `probe.py`, `results_*.json`, `logs_*`)
- 조건: 실제 지도와 keepout을 쓰고, 글로벌 costmap 인플레이션 0.45 / 감쇠 3, `max_planning_time` 1.0으로 계획했다. 세 구간 × 네 가지 방향 처리 × 세 가지 설정을 돌렸다.
- 아래 표는 현재 방식(중간점 방향 = 다음 경유점 방향)의 경로 길이 / 직선 연결 길이 비율이다.

| 구간 | NavFn | Lattice rp 0.5, r 0.5 m (현재) | rp 5.0, r 0.5 m | rp 0.5, r 1.0 m |
|---|---|---|---|---|
| A 도크→물 받는 곳 | 1.02 | **1.01** | 2.05 | 1.11 |
| B 물 받는 곳→table_02 | 1.04 | **1.02** | 3.85 | 1.47 |
| D table_02 탈출→대기점 | 1.02 | **1.00** | 2.39 | 1.05 |

- 계획 시간은 모든 경우 0.04 s 이하였다. 오류 코드는 모두 0이었다.
- **판단:** 이 지도에서는 제자리 회전을 싸게 두는 것(0.5)이 통로 안에서 방향을 바꾸는 유일한 수단이다. 페널티를 올리면 넓은 곳까지 돌아가서 방향을 바꾸는 경로가 나온다. 리서치 권고(5.0 쪽 A/B)는 이 실측으로 기각한다.
- **한계:** 정적 지도만 썼다. 실주행에서 local costmap의 장애물이 더해지면 결과가 달라질 수 있다.

## 2. MPPI

### 2.1 확인된 동작
- `CostCritic`(consider_footprint true)의 처리 순서 [확인] https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_mppi_controller/src/critics/cost_critic.cpp 와 `/opt/ros/jazzy/include/nav2_mppi_controller/critics/cost_critic.hpp`
  - 중심점 비용이 `possible_collision_cost` 이상일 때만 풋프린트 비용을 계산한다.
  - 풋프린트 모드에서는 LETHAL만 충돌로 본다. INSCRIBED는 충돌이 아니다.
  - 점수는 중심점 비용으로 매긴다.
  - `possible_collision_cost`는 외접 반경 지점의 인플레이션 비용이다. 인플레이션 반경이 외접 반경보다 작으면 0이 되어 모든 점을 검사하고, 성능 경고를 낸다.
- `PathAlignCritic`은 `furthest_reached_path_point < offset_from_furthest`이면 점수를 매기지 않는다. `furthest_reached_path_point`는 모든 샘플 궤적의 마지막 점에 가장 가까운 경로점 번호 중 최댓값이다. [확인] jazzy `src/critics/path_align_critic.cpp`, 설치 헤더 `/opt/ros/jazzy/include/nav2_mppi_controller/tools/utils.hpp`의 `findPathFurthestReachedPoint`
- 설치된 1.3.12 MPPI는 xtensor 구현이다(`utils.hpp`가 `xt::view` 사용). Eigen 이식으로 얻은 45–50 % 속도 개선은 이 버전에 없다. [확인] 로컬 헤더, https://discourse.openrobotics.org/t/nav2-speedups-in-mppi-smac-planner/41667
- `controller_frequency`의 주기가 `model_dt`와 같으면 제어열을 한 칸 밀어 다음 계산의 초기값으로 쓴다. 주기가 `model_dt`보다 크면 설정 단계에서 예외가 난다. [확인] jazzy `src/optimizer.cpp`
- 메인테이너는 MPPI 최소 20 Hz, 권장 30 Hz 이상이라고 답했다. 저속 로봇에 대한 별도 권고는 찾지 못했다. [확인] https://github.com/ros-navigation/navigation2/issues/4049
- 원 논문: Williams 외 "Information Theoretic MPC" (arXiv:1707.02342). 샘플이 상태공간을 충분히 탐색해야 하고, 충돌 임펄스 비용에서는 대부분의 샘플이 버려져 샘플 결핍이 생긴다고 서술한다. [확인] https://arxiv.org/abs/1707.02342
- 비원형 풋프린트에 정확한 부호 거리를 쓰는 EXACT-MPPI(arXiv:2605.29663)가 있다. GPU(JAX) 기반이라 Pi 4와는 맞지 않는다. 초록만 읽었다. [2차] https://arxiv.org/abs/2605.29663

### 2.2 `PathAlignCritic`이 꺼져 있다
- 지평: `time_steps` 30 × `model_dt` 0.1 = 3 s. `vx_max`가 0.12이므로 0.36 m다.
- 경로점 간격은 Lattice 약 0.051 m, NavFn 0.05 m(해상도)다. 따라서 `furthest_reached_path_point`는 최대 약 7이다.
- `offset_from_furthest` 20은 경로점 20개(약 1 m) 이상을 앞서야 켜진다. Nav2 기본값은 0.5 m/s 로봇의 지평 약 1.4 m에 맞춘 값이다.
- **결과:** 가중 14인 경로 정렬 critic이 평소 주행에서 점수를 내지 않는다. 경로 추종은 `PathFollowCritic`(앞쪽 5점 끌림)과 `PathAngleCritic`에만 기대고 있다. [확인] 설정 + 소스 + 원시경로 간격 계산
- **예외:** 제자리 회전 원시경로는 같은 위치에 점이 여러 개 생긴다. 그래서 회전이 많은 구간에서는 번호가 부풀어 드물게 켜질 수 있다. [추론]
- **영향 추정 [추론]:** 좁은 통로에서 Lattice가 차체를 고려해 만든 경로보다, 0.25 m 앞 점을 향해 모서리를 깎는 궤적이 더 유리해진다. 그러면 CostCritic이 감속하고 머뭇거리게 된다. 10-02 "회전만 하고 못 지나감"과 같은 방향의 증상이지만, 원인이라고 단정할 근거는 없다.
- **A/B 제안:** `offset_from_furthest`를 5로 낮춘다. 기본값의 설계 비율(지평 대비 약 70 %)을 우리 지평 7점에 맞춘 값이다.
  - MPPI critic 파라미터는 동적 갱신을 지원하므로 재배포 없이 바꿀 수 있다. `ros2 param set /controller_server MPPI.PathAlignCritic.offset_from_furthest 5`
  - 위험: 탈출·되짚기 직후 경로에서 벗어난 상태에서 강하게 경로로 끌려가며 뒤쪽이 쓸릴 수 있다. 확인은 A/B 지표(구간 시간, CM 정지, 105 횟수, 최소 거리)로 한다.
  - 이번 밤에는 기본 설정을 바꾸지 않았다. 이미 실차 확인 전인 변경(감쇠 10, 가중 1.5, 패딩 5 mm, Lattice 기본)이 쌓여 있어서, 한 번에 하나씩 비교해야 원인을 가릴 수 있기 때문이다.

### 2.3 `ax_max`와 스무더
- MPPI `ax_max` 0.5, `ax_min` -0.5. 스무더 `max_accel` 0.35, `max_decel` -0.5. [확인] `config/new_base_nav2_params.yaml`
- 스무더는 OPEN_LOOP에서 직전 출력을 현재 속도로 쓴다. 그래서 실제 가속은 0.35로 잘린다. [확인] https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_velocity_smoother/README.md
- 0.12 m/s 도달 시간은 MPPI 예측 0.24 s, 실제 0.34 s다. 출발할 때마다 예측보다 1–2 cm 뒤처진다. [추론]
- **A/B 제안:** `ax_max` 0.35. 위험이 작아 `offset_from_furthest`와 묶지 말고 따로 바꾼다.

### 2.4 풋프린트 검사 부하 (감쇠 10의 비용)
- 현재 풋프린트의 외접 반경은 꼭짓점 (-0.28, ±0.23)까지의 거리 0.362 m다. 내접 반경은 앞변까지 0.07 m다.
  - 리서치 에이전트는 0.393 m로 계산했지만 이는 틀렸다. (-0.28, ±0.275)라는 꼭짓점은 없다. local 인플레이션 0.40 m는 외접 반경보다 3.8 cm 크므로 "전 점 검사" 경고 조건은 아니다.
  - global costmap 주석의 "circumscribed 0.414 m"는 5 mm 풋프린트 이전 값이다.
- `possible_collision_cost`는 252·exp(-k·(0.362-0.07))로 계산한다. k=10이면 약 13이고, k=3이면 약 105다. [계산]
- 결과적으로 감쇠를 10으로 올리면서 장애물 0.36 m 안의 거의 모든 평가점이 풋프린트 검사를 받는다.
  - 평가점은 `trajectory_point_step` 4 기준으로 주기당 500 × 8 = 4,000개다.
  - 풋프린트 둘레는 약 36칸이다. 주기당 수만에서 십수만 칸을 조회한다. [추론]
- 10-02 Pi 벤치(500×30, 감쇠 3 시절)는 단일 코어 20.7 %, 10.27 Hz였고 주기 누락 경고는 0이었다(`$HOME/jdamr_data/analysis_20261002/mppi_bench/bench_run3_500x30.out`). 감쇠 10의 부하는 측정하지 않았다.
- **확인 방법:** 다음 주행에서 `jdamr_depart.py go`가 함께 띄우는 부하 기록기의 controller_server CPU와, journal의 "Control loop missed its desired rate"를 본다.

### 2.5 제어 주기와 Pi 4
- 10-02 실주행에서 시스템 부하는 75–85 %였고, 제어 주기는 5.8–9.9 Hz였다(`20261005_DRIVE_ROOT_CAUSE_AND_PRECISION_PLAN.md` §1.1).
- 주기가 늦어지면 실제로는 0.17 s가 지났는데 제어열은 0.1 s만큼 밀린다. 초기값이 어긋나는 것이지 발산은 아니다. [추론]
- **대안 (부하 기록 뒤 판단):**
  - (A) `batch_size` 300–400
  - (B) `controller_frequency` 5, `model_dt` 0.2, `time_steps` 15. 지평은 3 s로 같고 계산은 절반이다.
  - (B)는 `controller_frequency`가 controller_server 전체 값이라 ParkingReverse 같은 다른 컨트롤러에도 걸린다. 그래서 정밀 주차까지 다시 확인해야 한다.
- `use_realtime_priority`(현재 false)는 컨트롤러 스레드를 우선순위 90으로 올린다. `/etc/security/limits.conf`의 rtprio 설정이 필요하다. Pi 4 4코어에서 다른 노드를 굶길 수 있어, 부하 기록으로 여유를 본 뒤에 판단한다. [확인] https://discourse.openrobotics.org/t/nav2-soft-realtime-controller-server-option-available/34384
- 온보드 Nav2는 이미 `component_container_isolated` 한 프로세스로 합성되어 있고, CM만 별도 프로세스다(`launch/onboard_nav2_core.launch.py`). composition 권고는 이미 충족한다. [확인]
- Pi 4에서 Nav2·MPPI를 측정한 공개 벤치마크는 찾지 못했다.

### 2.6 그 밖의 MPPI 관찰 (변경 제안 없음)
- `wz_std` 0.3 = `wz_max` 0.3. 기본값 비율(0.4/1.9)보다 크다. 회전 샘플 상당수가 한계에서 잘린다. [추론] 증상이 확인되면 0.2를 시험한다.
- `vx_min` 0이고 스무더 `min_velocity` 0이라 후진 샘플이 없다. `PreferForwardCritic`은 사실상 점수를 내지 않는다. [확인]
- 크립(아주 느린 전진)이 계속되면 `VelocityDeadbandCritic`을 쓸 수 있다. [확인] jazzy `src/critics/velocity_deadband_critic.cpp`

## 3. Collision Monitor와 자기 필터

### 3.1 CM 동작 [확인] jazzy `nav2_collision_monitor/src/*.cpp`, README
- VelocityPolygon은 목록 순서로 첫 번째로 맞는 하위 다각형을 쓴다. 비홀로노믹은 `x`와 `tw` 범위만 보고, 경계는 닫힌 구간이다. 우리 로컬 검증과 같다.
- APPROACH는 t=0에서 다각형 안 점이 `min_points` 이상이면 충돌 시간 0을 돌려준다. 그래서 속도가 0이 된다.
- 외곽 안 장애물을 "빠져나가게 해 주는" 옵션(히스테리시스, 탈출 허용)은 없다. 관련 논의는 이슈 #3313과 VelocityPolygon PR #3708이다. https://github.com/ros-navigation/navigation2/issues/3313 , https://github.com/ros-navigation/navigation2/pull/3708
- `~/toggle`로 끄면 `cmd_vel_in`을 그대로 내보낸다. 다만 소스 유효성(`source_timeout`) 검사는 `enabled_`를 보지 않는다. 꺼진 상태에서도 스캔이 1.0 s 넘게 끊기면 정지한다.
- `stop_pub_timeout`은 정지 명령 재발행을 끊는 시간이다.

### 3.2 산업 관행 [확인] 표준 개요
- ISO 3691-4는 muting(자동 일시 정지), override(수동 정지), 조건 해소 뒤 자동 재개, 속도·방향에 따른 보호 영역 형상 변경을 정의한다. https://www.iso.org/standard/83545.html
- [추론] 우리 토글 되짚기는 muting과 override 사이에 있다. 안전 인증을 받기 전까지는 "비안전 기능"으로 다룬다. 거리·속도 상한, 끝나면 반드시 다시 켜기, 다시 켜지 못하면 비상정지 래치를 둔다. 현재 구현의 상한과 복구 경로 점검은 코드 리뷰 결과(`20261006_CODE_REVIEW.md`)에 있다.

### 3.3 라이다 무효값 처리 [확인]
- `laser_filters`의 Box/Footprint 필터는 NaN을 쓴다. AngularBounds는 기본이 `range_max + 1`이고 선택하면 NaN이다. Jazzy 패키지 `ros-jazzy-laser-filters` 2.0.10이 apt에 있다(미설치). https://github.com/ros-perception/laser_filters/tree/ros2/include/laser_filters
- REP 117: -Inf는 너무 가까움, NaN은 오류 측정, +Inf는 범위 안에 반사 없음이다. https://www.ros.org/reps/rep-0117.html
- 소비자별 처리:
  - CM은 `range_min ≤ r ≤ range_max`만 쓴다(`scan.cpp`).
  - AMCL likelihood field는 `range_max` 이상과 NaN을 건너뛴다.
  - 장애물 층은 `inf_is_valid`가 true일 때만 +inf를 `range_max`로 바꿔 지우기 광선으로 쓴다. 우리 설정에는 `inf_is_valid`가 없어 기본값 false다.
- **판단:** 지금 설정에서는 +inf와 NaN의 동작이 같다. 다만 누군가 `inf_is_valid`를 켜면 +inf인 기둥 방향이 "비어 있음"으로 지우기에 쓰여, 기둥 뒤 장애물 흔적이 지워진다. 의미상 NaN이 더 안전하다. [추론] 이번에는 +inf를 유지했고, `inf_is_valid`를 켜는 변경과 함께 NaN으로 바꾼다.
- 기둥 뒤 가려진 섹터는 어떤 필터로도 보이지 않는다. 그 섹터의 안전은 센서 배치(마스트 깊이 카메라를 CM 소스로 추가 등)로 메워야 한다. Nav2에 이 경우를 다루는 공식 사례는 찾지 못했다. [추론]

## 4. 박스·테이블 정밀 주차

### 4.1 opennav_docking (Jazzy)
- `opennav_docking_core::NonChargingDock`과 `SimpleNonChargingDock`이 설치되어 있다(`/opt/ros/jazzy/include/opennav_docking_core/non_charging_dock.hpp`). 필수 메서드는 `getStagingPose`, `getRefinedPose`, `isDocked`다. [확인]
- 공식 README는 컨베이어·팔레트 같은 비충전 대상을 지원한다고 적는다. 검색 요약 하나는 "지원 없음"이라고 했는데, 로컬 헤더가 이를 반박한다. [확인] https://api.nav2.org/nav2-rolling/html/md_nav2_docking_README.html
- 기본값: `docking_threshold` 0.05 m, `v_linear_min` 0.1 m/s, `staging_x_offset` -0.7 m, `filter_coef` 0.1, `max_retries` 3. [2차] 설정 문서 미러 https://ros.ncnynl.com/en/nav2/configuration/packages/configuring-docking-server.html
- 공식 문서에서 cm·mm 단위 도킹 정확도 수치는 찾지 못했다. 발표 글은 98 % 이상 성공률만 적는다. https://discourse.openrobotics.org/t/new-nav2-docking-server/38036
- **판단 [추론]:**
  - 지금의 "대기점 → 검출 → 정렬 → 오도메트리 직진" 구조는 도킹 서버의 "staging → 검출 → 제어 루프"와 같다.
  - 서버를 들이면 재시도·필터·시간 초과를 얻지만 정밀도는 얻지 못한다. 1 cm 목표에는 별도 종료 조건이 필요하다.
  - 충전기 후진 도킹은 `dock_backwards`로 검토할 만하다. `rotate_to_dock`이 1.3.12에 있는지는 로컬 바이너리 문자열에서 찾지 못했다.

### 4.2 Astra S 깊이
- 제조사 규격: 0.4–2 m, ±3 mm @1 m, 58.4°×45.5°, 640×480@30fps. [확인] https://www.orbbec.com/products/structured-light-camera/astra-series/
- 기선 75 mm(Giancola 외 측정 연구). [확인] https://link.springer.com/chapter/10.1007/978-3-319-91761-0_5
- 구조광 깊이 잡음은 거리의 제곱에 비례한다(σz ≈ z²·σd/(f·b)). 모서리 근처에서는 투영기–카메라 가림으로 무효 픽셀 띠가 생긴다. [추론] 삼각측량 일반 원리
- **영향:**
  - 박스 앞 5 cm 간격은 0.4 m 최소 거리 안이라 깊이로 측정할 수 없다. 마지막 0.4 m는 오도메트리·라이다 구간이다. [확인]
  - 양팔 마스트(0.8–0.97 m, 35–40°)에서는 테이블 가장자리까지 시선 거리가 대략 1.3–1.6 m다. 범위 안이지만 잡음은 1 m 때의 약 2배다. [추론] 설치 후 기하로 다시 계산한다.

### 4.3 cm 단위 상대 위치 추정 문헌
- 마커(AprilTag 가장자리 개선): 위치 3 mm 미만, 요 0.15° 미만(정밀 설정)을 보고한다. 같은 논문의 도킹 전체 반복성은 σz 0.96 cm, σy 2.57 cm, σψ 1.11°이고, 경로 계획 부정확으로 평균 약 4.5 cm 편차가 났다. 카메라–패턴 각 25° 미만은 피하라고 한다. [확인] https://robotik.informatik.uni-wuerzburg.de/telematics/download/ta2022_1.pdf
- RGB-D 컨테이너 포즈 추정(KittingBot). [확인] https://arxiv.org/abs/1809.05380
- AMCL을 모션캡처와 비교했을 때 최대 약 0.12 m 오차가 났다. 수술실 로봇 사례에서는 약 10 cm 오차가 자주 관찰되었다. [2차] https://link.springer.com/chapter/10.1007/978-3-031-91463-8_33 , https://arxiv.org/abs/2509.15600
- **판단 [추론]:** 지도 좌표(AMCL) 정확도는 10 cm 급이다. 1–2 cm 주차는 반드시 국소 센서 기준으로 마무리해야 한다. 우리 도크에서 관찰한 AMCL–정합 차이 9–11 cm와도 맞는다.

### 4.4 외부 측정 프로토콜
- ISO 18646-2(서비스 로봇 항법 성능, 2024판)는 ISO 9283을 참조한다. 기준점당 20–50회, 측정 위치 3초 이상 정지라는 요약이 있다. [2차] https://www.iso.org/standard/69057.html
- 상용 이동 베이스 5종을 레이저 트래커로 비교한 연구에서 정적 위치 정확도 중앙값은 8.2–63.5 mm였다. [확인] https://arxiv.org/abs/2609.03794
- **우리 적용:**
  1. 틈새 게이지나 캘리퍼로 박스 앞 간격을 20회 이상 잰다.
  2. 시작 포즈를 바꿔 가며(횡 오프셋·요) 측정한다.
  3. 평균 편차(정확도)와 표준편차·최대–최소(반복성), ±1 cm 통과 비율을 따로 보고한다.
  4. 내부 기록 5.51 ± 0.10 cm(38회)는 같은 오도메트리 안의 반복성일 뿐이다. 외부 측정 없이는 정확도라고 부르지 않는다.
  5. 라이다–깊이 2.4–2.7 cm 불일치를 먼저 분리한다(센서 외부 보정인지, 측정 면 기준 차이인지).

## 5. 이번 리서치가 바로잡은 것
- 리서치 권고 "rotation_penalty를 5.0 쪽으로": 실제 지도 프로브에서 경로가 2–4배로 늘어 기각했다(§1.2).
- 리서치 계산 "외접 반경 0.393 m, 인플레이션 여유 7 mm": 꼭짓점을 잘못 골랐다. 실제는 0.362 m, 여유 3.8 cm다(§2.4).
- 검색 요약 "opennav_docking은 비충전 미지원": 로컬 헤더와 README로 반박했다(§4.1).

## 6. 실차 재개 때 순서 (제안)
1. 배포 후 기본 설정(Lattice + MPPI, 감쇠 10)으로 A/B를 한 번 돈다. 부하 기록기로 controller_server CPU와 실제 제어 주기를 받는다.
2. 그 결과로 §2.5(주기)를 판단한다.
3. `PathAlignCritic.offset_from_furthest` 5를 단독 A/B한다(§2.2).
4. `ax_max` 0.35를 단독 A/B한다(§2.3).
5. 박스 정밀 주차는 외부 측정 20회부터 한다(§4.4).

## 출처
- Nav2 jazzy 소스·README: `nav2_smac_planner`, `nav2_planner/src/planner_server.cpp`, `nav2_mppi_controller`(README, `src/critics/*.cpp`, `src/optimizer.cpp`), `nav2_collision_monitor`(README, `src/*.cpp`), `nav2_velocity_smoother` README — https://github.com/ros-navigation/navigation2/tree/jazzy
- 설치 헤더·데이터: `/opt/ros/jazzy/include/nav2_mppi_controller/`, `/opt/ros/jazzy/include/nav2_smac_planner/`, `/opt/ros/jazzy/include/opennav_docking*/`, `/opt/ros/jazzy/share/nav2_smac_planner/sample_primitives/`
- Nav2 이슈·공지: #3313, PR #3708, #4049, #5531, #5803; discourse 41667, 34384, 38036, 43475
- 논문: Williams 외 arXiv:1707.02342; Pivtoraiko & Kelly IROS 2005; Pivtoraiko, Knepper & Kelly JFR 2009; Macenski 외 arXiv:2401.13078; EXACT-MPPI arXiv:2605.29663(초록); Würzburg ta2022_1(AprilTag 도킹); KittingBot arXiv:1809.05380; 이동 베이스 정확도 arXiv:2609.03794; Giancola 외 Springer 2018(Astra)
- 표준: REP 117; ISO 3691-4; ISO 18646-2(개요만)
- 이 PC 실측: `$HOME/jdamr_data/analysis_20261006/lattice_rotation/`, `$HOME/jdamr_data/analysis_20261005/lattice_probe/`, `$HOME/jdamr_data/analysis_20261002/mppi_bench/`
