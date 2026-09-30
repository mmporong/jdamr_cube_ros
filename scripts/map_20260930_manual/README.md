# 새 지도 운영 도구 사본 (2026-09-30 14:10 이후)

실행 원본은 `$HOME/jdamr_data/map_20260930_manual/tools/`와 이 세션의 임시 폴더에 있었다. 이 폴더는 그 시점 사본이다. 원본을 고치면 따라가지 않는다. 패키지 밖이라 flake8 대상이 아니다.

| 파일 | 역할 | 로봇 이동 |
|---|---|---|
| `jdamr_depart.py` | 새 지도(`aligned/`)용 출발 도구. `destinations.json`의 `water_station`·`table_01`·`table_02`로 세 경로를 만들고, `go table_0N`은 `box_service --via-id water_station ...`로 물 받는 곳 → 테이블 → 도크를 한 실행에 보낸다. 나머지 하위 명령은 Phase 2 도구(`scripts/phase2_20260930/`)와 같다. 16:20 판부터 `go`는 출발 직전 `dds_probe.py`로 새 프로세스가 지도·라이다 TF·스캔을 받는지 보고, 못 받으면 `recover`(표시·세션 종료 → `jdamr-base` 재시작 → 표시·세션 기동 → 직전 init 자세로 재초기화, 전역 정합 필수)를 거친다. `health`는 점검만, `recover --seed X Y YAW_DEG`는 로봇이 옮겨졌을 때 쓴다. `--via-route`·`--skip-via`·`--resume-at-observation`·`--resume-parked-log`는 중단된 구간 재개용 | `go`만 |
| `mapping_display_relay.py` | 수동 매핑 중 PC RViz에 실시간 `/map`을 보이게 한 임시 중계. 설치본 `rviz_display_relay`에 `/map`만 더해 ssh 표준입력으로 파이에서 실행한다(파이 쪽은 구독만) | 없음 |
| `dds_probe.py` | 파이에서 새 참가자로 `/map`·`/tf_static`의 `laser_link`·`/scan`을 20 s 안에 받는지 한 줄로 보고한다(구독만) | 없음 |
| `box_escape_once.py` | 박스 앞에서 멈춘 로봇에 실행기 `_leave_parked_pose` 후진 이탈만 한 번 실행한다. 앞면은 로봇 방향+측정 거리 또는 로그의 관측 면. 선택적으로 정지 확인 대기 | 후진 이탈 |
| `table_click_capture.py` | PC 전용 도메인 78에서 RViz 2D Pose Estimate 클릭을 목적지 후보로 기록하고 지도 축 0/90/180/270°로 고정해 표시한다 | 없음 |

사용 순서와 결과는 `jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md` §18.
