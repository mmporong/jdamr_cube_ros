# 새 지도 운영 도구 사본 (2026-09-30 14:10 이후)

실행 원본은 `$HOME/jdamr_data/map_20260930_manual/tools/`와 이 세션의 임시 폴더에 있었다. 이 폴더는 그 시점 사본이다. 원본을 고치면 따라가지 않는다. 패키지 밖이라 flake8 대상이 아니다.

| 파일 | 역할 | 로봇 이동 |
|---|---|---|
| `jdamr_depart.py` | 새 지도(`aligned/`)용 출발 도구. `destinations.json`의 `water_station`·`table_01`·`table_02`로 세 경로를 만들고, `go table_0N`은 `box_service --via-id water_station ...`로 물 받는 곳 → 테이블 → 도크를 한 실행에 보낸다. 나머지 하위 명령은 Phase 2 도구(`scripts/phase2_20260930/`)와 같다 | `go`만 |
| `mapping_display_relay.py` | 수동 매핑 중 PC RViz에 실시간 `/map`을 보이게 한 임시 중계. 설치본 `rviz_display_relay`에 `/map`만 더해 ssh 표준입력으로 파이에서 실행한다(파이 쪽은 구독만) | 없음 |
| `table_click_capture.py` | PC 전용 도메인 78에서 RViz 2D Pose Estimate 클릭을 목적지 후보로 기록하고 지도 축 0/90/180/270°로 고정해 표시한다 | 없음 |

사용 순서와 결과는 `jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md` §18.
