# 새 지도 운영 도구 사본 (2026-09-30 14:10 이후)

실행 원본은 `$HOME/jdamr_data/map_20260930_manual/tools/`와 이 세션의 임시 폴더에 있었다. 이 폴더는 그 시점 사본이다. 원본을 고치면 따라가지 않는다. 패키지 밖이라 flake8 대상이 아니다.

| 파일 | 역할 | 로봇 이동 |
|---|---|---|
| `jdamr_depart.py` | 새 지도(`aligned/`)용 출발 도구. `destinations.json`의 `water_station`·`table_01`·`table_02`로 세 경로를 만들고, `go table_0N`은 `box_service --via-id water_station ...`로 물 받는 곳 → 테이블 → 도크를 한 실행에 보낸다. 나머지 하위 명령은 Phase 2 도구(`scripts/phase2_20260930/`)와 같다. `go`는 완료된 init(`state.json`의 `localized`)이 있고 주행 중인 사이클이 없을 때만 출발한다. 실행기 첫 확인이 지도를 받지 못해 이동 없이 끝나면 `recover`(사이클·linger 확인 → 표시·세션 종료 → 센서 서비스 3개 재시작 → 표시·세션 기동 → 직전 init 자세로 재초기화, 전역 정합 필수)를 거쳐 한 번만 다시 출발한다. 재개 옵션이나 `--local-only` init 뒤에는 자동 복구 대신 안내하고 멈춘다. `session-start`는 Nav2 세션과 함께 상주 실행기 `jdamr-box-executor`(`box_service --serve`)를 띄워, `go`가 매번 새 프로세스로 그래프를 다시 탐색하지 않고 요청 파일만 넣는다(`/home/lim/jdamr_data/box_executor_spool`). 상주 실행기가 없으면 예전처럼 출발마다 유닛을 만든다. 실행기 코드를 바꾸면 세션을 다시 띄워야 반영된다. `init`은 등록된 도크 자세를 시작 추정값으로 스캔 정합·AMCL 일치를 확인하고 도크 등록은 그대로 둔다. 도크가 아닌 곳에 놓였을 때만 RViz 클릭(`init --click`)을 쓴다. `health`는 점검만, `recover --seed X Y YAW_DEG [--local-only]`는 로봇이 옮겨졌을 때 쓴다. `--via-route`·`--skip-via`·`--resume-at-observation`·`--resume-parked-log`는 중단된 구간 재개용, `--dock-only`는 도크 복귀만(재도킹). `go table_01 table_02`처럼 여러 테이블을 주면 테이블마다 도크 → 물 받는 곳 → 테이블 → 도크 사이클을 차례로 돌린다. 사이클 사이에는 도크에서 `--dock-wait-s`(기본 10 s, 충전 정차) 기다린 뒤 도크 init(스캔 정합, 이동 없음)을 다시 하고 출발하며, 사이클이 도크에서 확정되지 않으면 그 자리에서 멈춘다(재개·경로·영역 옵션은 한 테이블에만). `--resume-at-observation`은 로봇이 테이블 관측 지점에 있으면 끝난 물 받는 곳을 건너뛴다. `display-start`는 PC에서 중계 토픽 bag 기록도 켠다(`record-start`·`record-stop`, 파이 디스크를 쓰지 않음). `go`는 출발 요청 직전 파이에서 `/odom`·`/imu/data_raw`·`/scan`·`/tf`·`/amcl_pose`·속도 명령을 실행 단위로 기록하고(유닛 `jdamr-onboard-bag-<run>`, 최대 1시간·400 MB), 사이클이 끝나면 닫아 run 폴더째 PC로 복사한다(`runs/<run>/onboard_bag`) | `go`만 |
| `mapping_display_relay.py` | 수동 매핑 중 PC RViz에 실시간 `/map`을 보이게 한 임시 중계. 설치본 `rviz_display_relay`에 `/map`만 더해 ssh 표준입력으로 파이에서 실행한다(파이 쪽은 구독만) | 없음 |
| `dds_probe.py` | 파이에서 새 참가자로 `/map`·`/keepout_filter_mask`·`/tf_static`의 `laser_link`·`/scan`을 실행기와 같은 30 s 안에 받는지 한 줄로 보고한다(구독만, 박스 관측 상태는 수만 기록) | 없음 |
| `box_escape_once.py` | 박스 앞에서 멈춘 로봇에 실행기 `_leave_parked_pose` 후진 이탈만 한 번 실행한다. 앞면은 로봇 방향+측정 거리 또는 로그의 관측 면(단위 법선만 허용). 선택적으로 정지 확인 대기, `--stop-id`로 로그 표지 지정. SIGINT·SIGTERM·SIGHUP이면 후진을 취소한다 | 후진 이탈 |
| `test_jdamr_depart_flow.py` | `jdamr_depart.py`의 출발·복구 흐름을 ssh 없이 모의로 확인한다(36건). 지도 데이터가 있는 PC에서만 돈다 | 없음 |
| `analyze_run.py` | 한 실행의 `cycle_events.jsonl`(+ PC 중계 bag)을 단계별 시간·재계획 수·회전 방향 전환·map→odom 보정으로 요약한다 | 없음 |
| `udp_drop_monitor.py` | 파이에서 5 s마다 UDP 수신 넘침을 프로세스별로 기록한다. `go`가 실행 동안 transient 유닛으로 띄우고 끝나면 멈춘다(`runs/<run>/udp_drops.jsonl`) | 없음 |
| `imu_axes.py` | `/odom`·`/imu/data_raw`가 든 bag에서 IMU 보드 축을 정한다. z는 자이로와 바퀴 회전율의 기울기, x·y는 바퀴 전진 가속·원심 가속에 대한 회귀. 2026-10-01 결과(보드 z 아래·y 앞·x 왼쪽)가 URDF `imu_joint`다 | 없음 |
| `rotation_truth.py` | 정지 구간으로 감싼 회전(또는 `--turnarounds` 왕복 회전)을 스캔끼리 맞춘 회전량 기준으로 odom·자이로·EKF 회전 오차를 잰다(`--ekf=<bag>`) | 없음 |
| `ekf_replay.sh` | bag의 `/odom`·`/imu/data_raw`를 `imu_bias_relay` + robot_localization에 PC 도메인 89로 흘려 `/odometry/filtered`를 기록한다 | 없음 |
| `imu_check.sh` | 한 run의 `onboard_bag`에 `imu_axes.py` → `ekf_replay.sh` → `rotation_truth.py`를 차례로 돌려 `runs/<run>/imu_check/`에 남긴다. URDF를 파이에 반영한 뒤에는 `ros2 run tf2_ros tf2_echo base_footprint imu_link`로 RPY (180°, 0°, 90°)를 확인한다 | 없음 |
| `test_imu_axes.py` | 보드를 z 아래·y 앞·x 왼쪽으로 단 합성 신호에서 `imu_axes.analyse`가 축을 되찾는지 확인한다 | 없음 |
| `compare_runs.py` | 여러 run 폴더를 같은 기준(첫 목표 수락부터)으로 비교한다: 사이클, 대기점 도착 방향·Spin, 대기점 도착 → 도크 후진, 이탈 → 도크 후진 시간과 odom 회전량, 도크 끝 방향·확인, 면 정렬 시간, 주차. run 폴더의 `run_config.json`(계획기 등 설정)을 함께 적는다 | 없음 |
| `plan_compare.py` | 파이의 살아 있는 세션에서 이동 구간 5개(도크→물 받는 곳, 물 받는 곳→각 테이블, 각 테이블→도크 대기점)를 `GridBased`와 `Lattice`로 계획만 해 길이 비·계획 시간·끝 오차를 비교한다(이동 없음) | 없음 |
| `table_click_capture.py` | PC 전용 도메인 78에서 RViz 2D Pose Estimate 클릭을 목적지 후보로 기록하고 지도 축 0/90/180/270°로 고정해 표시한다 | 없음 |

사용 순서와 결과는 `jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md` §18.
