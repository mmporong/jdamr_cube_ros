# 새 지도 운영 도구 사본 (2026-09-30 14:10 이후)

실행 원본은 `$HOME/jdamr_data/map_20260930_manual/tools/`와 이 세션의 임시 폴더에 있었다. 이 폴더는 그 시점 사본이다. 원본을 고치면 따라가지 않는다. 패키지 밖이라 flake8 대상이 아니다.

| 파일 | 역할 | 로봇 이동 |
|---|---|---|
| `jdamr_depart.py` | 새 지도(`aligned/`)용 출발 도구. `destinations.json`의 `water_station`·`table_01`·`table_02`로 세 경로를 만들고, `go table_0N`은 `box_service --via-id water_station ...`로 물 받는 곳 → 테이블 → 도크를 한 실행에 보낸다. 나머지 하위 명령은 Phase 2 도구(`scripts/phase2_20260930/`)와 같다. `go`는 완료된 init(`state.json`의 `localized`)이 있고 주행 중인 사이클이 없을 때만 출발한다. 실행기 첫 확인이 지도를 받지 못해 이동 없이 끝나면 `recover`(사이클·linger 확인 → 표시·세션 종료 → 센서 서비스 3개 재시작 → 표시·세션 기동 → 직전 init 자세로 재초기화, 전역 정합 필수)를 거쳐 한 번만 다시 출발한다. 재개 옵션이나 `--local-only` init 뒤에는 자동 복구 대신 안내하고 멈춘다. `health`는 점검만, `recover --seed X Y YAW_DEG [--local-only]`는 로봇이 옮겨졌을 때 쓴다. `--via-route`·`--skip-via`·`--resume-at-observation`·`--resume-parked-log`는 중단된 구간 재개용, `--dock-only`는 도크 복귀만(재도킹). `display-start`는 PC에서 중계 토픽 bag 기록도 켠다(`record-start`·`record-stop`, 파이 디스크를 쓰지 않음) | `go`만 |
| `mapping_display_relay.py` | 수동 매핑 중 PC RViz에 실시간 `/map`을 보이게 한 임시 중계. 설치본 `rviz_display_relay`에 `/map`만 더해 ssh 표준입력으로 파이에서 실행한다(파이 쪽은 구독만) | 없음 |
| `dds_probe.py` | 파이에서 새 참가자로 `/map`·`/keepout_filter_mask`·`/tf_static`의 `laser_link`·`/scan`을 실행기와 같은 30 s 안에 받는지 한 줄로 보고한다(구독만, 박스 관측 상태는 수만 기록) | 없음 |
| `box_escape_once.py` | 박스 앞에서 멈춘 로봇에 실행기 `_leave_parked_pose` 후진 이탈만 한 번 실행한다. 앞면은 로봇 방향+측정 거리 또는 로그의 관측 면(단위 법선만 허용). 선택적으로 정지 확인 대기, `--stop-id`로 로그 표지 지정. SIGINT·SIGTERM·SIGHUP이면 후진을 취소한다 | 후진 이탈 |
| `test_jdamr_depart_flow.py` | `jdamr_depart.py`의 출발·복구 흐름을 ssh 없이 모의로 확인한다(22건). 지도 데이터가 있는 PC에서만 돈다 | 없음 |
| `table_click_capture.py` | PC 전용 도메인 78에서 RViz 2D Pose Estimate 클릭을 목적지 후보로 기록하고 지도 축 0/90/180/270°로 고정해 표시한다 | 없음 |
| `analyze_run.py` | 한 실행의 `cycle_events.jsonl`(+ PC 중계 bag)을 단계·최종 접근·간격·도크 대기점 회전·이탈·도킹과 단계별 재계획 수·회전 방향 뒤집힘·map→odom 점프로 요약한다 | 없음 |
| `scan_match.py` | 정지한 로봇의 스캔 묶음을 지도에 맞춰 자세를 구한다(도크 init에 쓴 도구의 사본). `amcl_replay.py`가 기준 자세를 만들 때 쓴다 | 없음 |
| `amcl_replay.py`, `amcl_replay.launch.py` | `/scan`·`/tf`·`/tf_static`이 든 bag(PC 중계 bag 포함)을 PC 도메인 88에서 AMCL 파라미터별로 다시 돌리고, 정지 구간마다 `scan_match` 기준 자세와 비교한다(`prepare` → `run <tag> <params>` → `compare`) | 없음 |
| `rotation_truth.py` | 제자리 회전 전후 스캔끼리 맞춘 회전량을 기준으로 odom·자이로·EKF 회전량 오차를 잰다(`--turnarounds`, `--ekf=<bag>`) | 없음 |
| `ekf_replay.sh` | bag의 `/odom`·`/imu/data_raw`를 `imu_bias_relay` + robot_localization에 PC 도메인 89로 흘려 `/odometry/filtered`를 기록한다 | 없음 |

사용 순서와 결과는 `jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md` §18.
