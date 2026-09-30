# 3·4번 항목: AMCL·EKF(위치 추정)와 Lattice(전역 경로) — 오프라인 검증 기록

- 브랜치: `feat/nav-items34-localization-lattice` (기준 `feat/restaurant-service-destinations` fc73b12). 파이에 배포하지 않았다.
- 모든 수치는 PC 오프라인 재생·계획 결과다. 실차 주행은 없다. 로봇 도메인(12)과 다른 격리 도메인(87·88·89, localhost)만 썼다.
- 1·2번(Graceful 최종 접근, 도크 대기점 재계획 정책·한 방향 회전)은 기준 브랜치에 있고, 다음 주행에서 `analyze_run.py`로 확인한다.

## 1. 결론

| 항목 | 이번 판정 | 근거 요약 | 다음 확인 |
|---|---|---|---|
| 4 Lattice 전역 경로 | **도입하지 않음** (NavFn 유지) | 새 지도 6개 구간에서 Lattice 0.5 m는 경로가 직선의 2.1–3.3배, 1 m는 구간에 따라 6.7–7.7배. NavFn은 1.00–1.05배, 30/30 성공 | 없음 |
| 4 inflation 0.45 | **효과 없음** (바꿀 이유 없음) | NavFn 경로가 0.30과 bit 단위로 같다. 경로 셀 비용이 두 값 모두 0 | local costmap·RPP 쪽 영향은 미측정 |
| 3a AMCL `likelihood_field_prob` + beamskip | **브랜치에 적용, 판정 보류** | 재생 도구는 동작 확인. 9/29 bag은 정지 기준 구간 2개·옛 지도라 우열을 가릴 수 없다 | 다음 주행 PC bag으로 A/B 재생(§3.3) |
| 3b EKF(odom + 자이로) | **브랜치에 구현, 미연결** | 자이로 z 부호 반대 확인. 제자리 회전에서 odom은 스캔 기준보다 2.8 % 크게 잰다. 바이어스를 뺀 EKF는 오차 중앙값 0.10°(odom 0.50°), 표본 4건 | 온보드 bag으로 표본 확대(§4.4) |

## 2. 4번: 전역 플래너 비교 (새 지도)

전체 보고서는 `$HOME/jdamr_data/planner_bench_20260930_newmap/REPORT.md`, 경로 그림은 같은 폴더의 `paths_overlay_infl030.png`·`paths_overlay_infl045.png`다.

- 입력: `map_20260930_manual/aligned/{map,keepout}.yaml`, 저장소 `global_costmap`·`GridBased` 블록(obstacle_layer만 제외), 경로 yaml과 같은 자세.
- 구간: A home_exit → 물 받는 곳 관측, B 물 이탈 → table_01 관측, C 물 이탈 → table_02 관측, D table_01 이탈 → 도크 대기점, E table_02 이탈 → 도크 대기점, F 물 이탈 → 도크 대기점.
- 플래너: NavFn(현재), Smac2D, Lattice(설치된 5 cm `diff` 프리미티브, 회전 반경 0.5 m·1 m; 제자리 회전 포함). Hybrid는 Ackermann용이라 뺐다. `omni` 프리미티브는 옆걸음이 있어 차동 구동에 맞지 않는다.

| 플래너 | 길이/직선 비율 | 최소 footprint 여유 (점유, keepout) | PC 계획 시간 중앙값 |
|---|---|---|---|
| NavFn (0.30·0.45 동일) | 1.00–1.05 | +0.239 m, +0.189 m | 0.12–0.26 ms |
| Smac2D | 1.00–1.05 | +0.124 m, +0.075 m | 0.16–0.61 ms |
| Lattice 0.5 m | 2.10–3.33 | 최소 −0.062 m, −0.144 m(겹침) | 0.6–47.5 ms |
| Lattice 1 m | 1.00–1.18, C(0.45) 6.74, route RC 7.4–7.7 | +0.034 m, +0.076 m | 1.7–33.3 ms |

- NavFn은 방향을 계획하지 않는다. 목표에서 31–138°, 출발에서 90–180° 제자리 회전이 남고, 추종기와 도크 대기점 Spin이 맡는다. 회전 위치는 점유 셀에서 0.69 m 이상 떨어져 있어 0.414 m 회전 원이 닿는 경로 자세는 0개다.
- keepout은 지도의 unknown 셀과 같고 필터가 팽창 뒤에 LETHAL로 쓰므로, inflation_radius를 키워도 keepout 여유는 늘지 않는다.
- 이전 판단 재확인: "비원형 차동 로봇에는 Lattice가 공식 권장"은 Nav2 문서 설명으로는 맞지만, 이 지도에서 샘플 프리미티브·기본 penalty로는 9/29 옛 지도(C7 4.2 m 대 1.7 m)와 같은 루프가 재현된다. 파라미터 스윕은 하지 않았다.

## 3. 3a: AMCL 관측 모델 (브랜치 적용)

### 3.1 변경

`config/new_base_nav2_params.yaml`의 amcl 두 줄만 바꿨다.

| 파라미터 | 기준 브랜치 | 이 브랜치 |
|---|---|---|
| `laser_model_type` | `likelihood_field` | `likelihood_field_prob` |
| `do_beamskip` | false | true (beam_skip_distance 0.5, threshold 0.3, error_threshold 0.9는 기존 값) |

beamskip은 `likelihood_field_prob`에서만 동작한다. 지도에 없는 물체(박스·사람)에 맞은 빔을 빼는 기능이라, 도크·박스 근처 방향 오차(9/30 약 15°)와 관련이 있을 수 있다. 원인이 이것인지는 확인되지 않았다.

### 3.2 재생 도구와 9/29 bag 확인

`scripts/map_20260930_manual/amcl_replay.py`:
1. `prepare`: bag에서 map→odom만 뺀 재생용 bag을 만들고, 정지 구간(odom 5 mm·0.5° 안에서 2 s 이상)마다 `scan_match.py`로 지도 정합 기준 자세를 구한다. 기준은 AMCL과 독립된 스캔-지도 정합이다. 빔의 45 % 이상이 벽 5 cm 안에 들어온 구간만 기준으로 쓴다. `jdamr_depart.py` init 정합의 채택 기준(`MATCH_MIN_INLIER`)과 같다. 도크 근처 지도에 없는 박스 때문에 맞는 정합도 0.48–0.49가 나온다.
2. `run`: map_server + amcl을 sim time으로 띄우고 bag의 첫 AMCL 자세로 초기화한 뒤, AMCL이 내는 map→odom을 기록한다. systemd scope(MemoryMax 3G, CPUQuota 300 %, 1시간 제한) 안에서만 돈다.
3. `compare`: 정지 구간마다 기록된 파이 AMCL과 각 재생의 자세 오차를 표로 낸다.

9/29 bag(`parking_attempt_20260929_preserved/table_departure_20260929_latest_bag`, 옛 지도 `local_updated.yaml`)에서 네 단계가 끝까지 돌았다. 결과는 `$HOME/jdamr_data/amcl_replay_20260930/bag_20260929_table01/`에 있다.

- 정지 기준 구간은 2개뿐이다(도크 16.7 s, 테이블 관측 위치 64 s).
- 수치(A = 기준 브랜치, B = 이 브랜치)는 §3.4에 두었다. 구간이 2개이고 지도가 옛 것이라 우열 판정에 쓰지 않는다.

### 3.3 다음 주행에서 판정하는 방법

PC 중계 bag(`jdamr_depart.py display-start`가 켜는 `jdamr-p2-record`)에는 `/scan`(5 Hz)·`/tf`(10 Hz)·`/tf_static`이 있어 그대로 쓸 수 있다.

```bash
cd ~/jdamr_rgbd_ws/src/jdamr_cube_ros/scripts/map_20260930_manual
source /opt/ros/jazzy/setup.bash
R=~/jdamr_data/amcl_replay_20260930
python3 amcl_replay.py prepare <PC bag 폴더> ~/jdamr_data/map_20260930_manual/aligned/map.yaml $R/<run 이름>
python3 amcl_replay.py run $R/<run 이름> A $R/params/A_likelihood_field.yaml
python3 amcl_replay.py run $R/<run 이름> B $R/params/B_prob_beamskip.yaml
python3 amcl_replay.py compare $R/<run 이름> A B
```

- 같은 입력을 같은 설정으로 다시 돌리면 결과가 같다(§3.4, 두 번씩 돌려 모든 값 일치). 반복 대신 주행 수와 정지 구간 수를 늘려 판단한다.
- 보는 값: 정지 구간 기준 대비 방향 오차(중앙값·최대·3° 초과 건수)와 위치 오차(5 cm 초과 건수). 도크 도착·박스 주차 정지 구간이 핵심이다.
- 한 주행에는 적어도 도크 출발 전·물 받는 곳 5 s 대기·테이블 5 s 대기·도킹 후의 정지 구간이 있다. 두 번 이상의 주행에서 B가 방향 오차 최대값과 3° 초과 건수를 함께 줄일 때 채택 근거로 쓴다.

### 3.4 9/29 bag 재생 수치 (참고용)

A = 기준 브랜치 설정, B = 이 브랜치 설정, 뒤의 2는 같은 설정의 두 번째 재생이다. 값은 기준 자세 대비 위치 오차와 방향 오차다. "recorded"는 9/29 당시 파이 AMCL(그때 설정)이다.

| 정지 구간 시작 | recorded | A | A2 | B | B2 |
|---|---|---|---|---|---|
| 0.0 s (도크) | 3.6 cm +1.0° | 6.8 cm −4.54° | 6.8 cm −4.54° | 4.1 cm −2.06° | 4.1 cm −2.06° |
| 190.5 s (테이블 관측 위치) | 9.2 cm −0.75° | 10.8 cm −0.95° | 10.8 cm −0.95° | 15.9 cm −0.25° | 15.9 cm −0.25° |

- 기준 정합 잔차는 0.053 m·0.085 m, 빔 비율 0.61·0.56이다(옛 지도와 그날 가구 배치 차이).
- 두 번째 재생이 첫 번째와 모든 값에서 같다. 입력이 같으면 재생 결과가 같다.
- B는 방향 오차가 두 구간 모두 작고(2.06°·0.25° 대 4.54°·0.95°), 테이블 위치 오차는 크다(15.9 cm 대 10.8 cm). 구간 2개로는 판단하지 않는다.

## 4. 3b: 자이로를 쓰는 EKF (브랜치 구현, 미연결)

### 4.1 측정 사실 (9/15–9/16 새 차대 bag, `/data/lim/jdamr_artifacts/new_base_*`)

| 항목 | 값 | 표본 |
|---|---|---|
| 자이로 z와 odom 회전율 상관 | −0.954, −0.999, −0.975, −0.999 | 회전 중 샘플 132·2768·156·1510개, bag 4개 |
| 정지 중 가속도 z | −9.34 ~ −9.45 m/s² | bag 8개 |
| 정지 중 자이로 z 바이어스 | −0.00182 ~ +0.00113 rad/s (세션·시간대별로 다름), 9/30 −0.00016 | 정지 20–115 s |
| 제자리 회전 자이로/odom 비 | 0.975–0.983 | 10구간(±13–15°) |
| 주행 중 곡선 구간 자이로/odom 비 | 0.965–1.032 | 12구간 |
| 18분 bag 누적 방향 차(자이로 − odom) | +107°, 20 s 구간마다 거의 일정하게 쌓임(평균 +2.0°) | `wide_start` |

- 보드의 z축은 아래를 향한다(자이로 z 부호 반대, 정지 가속도 z 음수). x·y축 방향은 가속도 신호가 약해(상관 −0.25·+0.17) 정하지 못했다. EKF에는 z 회전율만 쓴다.
- 18분 누적 차 107°는 odom 튐이 아니다. odom 자세 변화와 twist 적분이 −312.26° 대 −312.32°로 맞고, 차이가 일정 속도로 쌓인다. 그 세션의 정지 구간 평균 바이어스 −0.00138 rad/s를 18.2분에 곱하면 86°로, 107°의 대부분이다.
- 바이어스를 빼지 않으면 자이로가 분당 최대 약 6° 흐른다. robot_localization에는 자이로 바이어스 상태가 없고, 파이에 `imu_complementary_filter`·`imu_filter_madgwick`이 없다.

### 4.2 어느 쪽이 회전을 정확히 재는가 (스캔 기준)

`scripts/map_20260930_manual/rotation_truth.py --turnarounds`: 제자리 왕복 회전의 방향 전환 순간(회전율 0) 스캔끼리 맞춰 회전량을 구하고, odom·자이로·EKF 회전량과 비교했다. 지도가 필요 없다.

| 대상 (`new_base_revisit_ready_20260915`, ±14° 회전 4건) | 스캔 대비 비율 중앙값 | 오차 중앙값 | 오차 최대 |
|---|---|---|---|
| 바퀴 odom | 1.028 | 0.50° | 1.26° |
| 자이로 원값(바이어스 미보정) | 1.006 | 0.40° | 0.77° |
| EKF(`imu_bias_relay`가 정지 10 s에서 +0.00078 rad/s를 빼고 융합) | 1.006 | 0.10° | 0.19° |

- 스캔 정합 잔차는 0.85–0.90 cm다. 나머지 bag은 로봇이 멈추지 않고 계속 움직여 쓸 수 있는 방향 전환이 없었다(bag 10개 중 1개에서만 4건).
- 표본 4건이라 비율의 불확실성이 크다. 방향은 "제자리 회전에서 odom이 2–3 % 크게 잰다"로 10구간 자이로/odom 비(0.975–0.983)와 일치한다.
- 의미: 180° 회전(물 받는 곳 → table_01)에서 odom 오차는 약 5°, EKF는 1° 안팎이다. odom 고정 구간(최종 접근·이탈·도크 레그)은 회전이 거의 없어 odom 스케일 오차의 영향이 작다.

### 4.3 구현 (어떤 bringup에도 넣지 않음)

| 파일 | 내용 |
|---|---|
| `jdamr_cube_navigation/imu_bias_relay.py` | `/odom` twist가 정확히 0인 상태가 1 s 넘게 이어질 때만 자이로 바이어스를 배운다(처음 5 s 평균, 이후 시정수 5 s). `/imu/data_raw`에서 빼서 `imu_link` 프레임의 `/imu/data`로 낸다. z 분산 0.005² |
| `config/ekf_odom_imu.yaml` | robot_localization 평면 EKF. odom vx·vy·vyaw + `/imu/data` vyaw. 드라이버 odom vyaw 분산이 1e-2라 융합 회전율은 자이로를 따른다 |
| `launch/ekf_odom_imu.launch.py` | base_link→imu_link 정적 TF(x축 180°) + relay + ekf_node |
| `test/test_imu_bias_relay.py` | 회전 중에는 배우지 않고, 정지 10 s에 오프셋으로 수렴하는지 |

연결할 때 할 일:
1. 베이스 드라이버를 `publish_tf:=false`로 띄운다. odom→base_footprint는 EKF만 낸다.
2. `ros2 launch jdamr_cube_navigation ekf_odom_imu.launch.py`를 드라이버와 같은 도메인에서 띄운다.
3. Nav2 `odom_topic`을 `/odometry/filtered`로 바꿀지는 따로 정한다(RPP·Graceful·진행 검사가 `/odom` 속도를 읽는다).

### 4.4 판정에 더 필요한 데이터

PC 중계 bag에는 `/odom`·`/imu/data_raw`가 없다(중계는 표시용 `/tf`·`/scan`·`/plan`·`/amcl_pose`만). 표본을 늘리려면 파이 쪽에서 `/odom`·`/imu/data_raw`·`/scan`·`/tf_static`을 기록하거나(파이 디스크 39 %), 로봇이 도크 대기점 회전처럼 멈췄다 도는 구간이 있는 기록이 필요하다. 그 bag이 있으면:

```bash
bash ~/jdamr_rgbd_ws/src/jdamr_cube_ros/scripts/map_20260930_manual/ekf_replay.sh <bag> ~/jdamr_data/imu_check_20260930/ekf_replay/<이름>
cd ~/jdamr_rgbd_ws/src/jdamr_cube_ros/scripts/map_20260930_manual
python3 rotation_truth.py --ekf=$HOME/jdamr_data/imu_check_20260930/ekf_replay/<이름>/filtered <bag>
```

정지 구간으로 감싼 회전이 있으면 `--turnarounds` 없이 정지 구간 모드가 쓰인다.

## 5. 데이터 위치

| 내용 | 경로 |
|---|---|
| 플래너 비교(스크립트·원시·표·그림·보고서) | `$HOME/jdamr_data/planner_bench_20260930_newmap/` |
| AMCL 재생(파라미터 A/B, 9/29 bag 준비·재생·비교) | `$HOME/jdamr_data/amcl_replay_20260930/` |
| IMU 축·바이어스·회전 비교 스크립트와 로그, EKF 재생 | `$HOME/jdamr_data/imu_check_20260930/` |
