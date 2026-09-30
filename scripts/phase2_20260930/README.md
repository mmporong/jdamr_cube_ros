# Phase 2 운영 도구 사본 (2026-09-30)

2026-09-30 table_01 시도와 충전소 후진 도킹에 쓴 PC 쪽 운영 도구의 저장소 사본이다. 실행 원본은 데이터 폴더에 있다.

```bash
cd "$HOME/jdamr_data/map_update_20260929_4XUrkc/phase2_20260930"
python3 tools/jdamr_depart.py status
```

이 사본은 12:14 시점 원본과 같은 파일이다. 데이터 폴더 도구를 고치면 이 사본은 자동으로 따라가지 않는다. 패키지 밖에 두었으므로 `jdamr_cube_navigation`의 flake8 검사 대상이 아니다.

## 파일

| 파일 | 역할 | 로봇 이동 |
|---|---|---|
| `jdamr_depart.py` | 출발 절차 한 명령. `display-start/stop`(PC 지도·중계·표지·RViz·클릭 수신), `session-start/stop`(파이 Nav2 정밀 세션, prepare-only, LOCALHOST, 비합성), `init`(RViz 클릭 → 스캔 정합 → `activate_navigation` → teach-home), `go <table>`(파이에서 `box_service --return-home` 실행), `stop`, `status`, `sync`, `routes` | `go`만 |
| `scan_match.py` | 정지한 로봇의 LiDAR 끝점과 지도 점유 칸 사이 거리(절단 평균)를 최소화해 RViz 클릭 자세를 다듬는다. 각도 binning으로 scan마다 다른 빔 수를 흡수한다 | 없음 |
| `dock_survey.py` | 전역 자세 탐색(정합 결과 교차 확인)과 지도에 없는 LiDAR 군집(박스 면 후보) 추출 | 없음 |
| `capture_scan2.py` | 파이에서 `/scan`·`/odom`·`/tf_static`·`/battery_state`를 읽어 JSON 한 줄로 출력 | 없음 |
| `initialpose_capture.py` | PC에서 RViz `2D Pose Estimate`를 받아 `rviz_initialpose.json`으로 저장(주행 노드에 전달하지 않음) | 없음 |
| `geometry_check.py` | 후보 자세의 지도 장애물·keepout 거리, footprint 겹침을 오프라인으로 계산 | 없음 |
| `pose_scan_probe.py` | 파이에서 map→base_link TF, `/amcl_pose`, `/scan` 한 장을 읽는다(도크 최종 자세 측정에 사용) | 없음 |
| `layout_prototypes/` | 지도·박스 설치 계획(§5 참고안)을 계산한 읽기 전용 원형 3개. 경로가 이 PC 기준으로 고정돼 있다. 계획은 이를 `geometry_check.py layout`으로 정리할 것을 제안한다 | 없음 |

## 환경 가정

- `HOST = 'lim@192.168.0.159'`, 파이 작업공간 `$HOME/jdamr_ws`, ROS 도메인 12, LOCALHOST discovery.
- 자산 경로는 `$HOME/jdamr_data/map_update_20260929_4XUrkc/phase2_20260930/`가 PC·파이에서 같다. `sync`가 파이로 복사한다.
- `MAP_YAML`은 12:00 이후 `map_dockfix.yaml`이다(도크 통로 흔적 3칸 수정, `dockfix_provenance.json`).
- 파이 lim linger가 켜져 있어야 한다(저장소 `AGENTS.md`).

## 충전소 복귀 명령 (2026-09-30 11:58)

`jdamr_depart.py`에 복귀 하위 명령은 없다. 도킹은 파이에서 기존 CLI를 systemd 유닛으로 띄웠다. 실제 명령은 `$HOME/jdamr_data/dock_return_success_20260930/run/command.txt`에 있다.

```bash
python3 -m jdamr_cube_navigation.restaurant_service home \
  --registry "$HOME/jdamr_data/map_update_20260929_4XUrkc/phase2_20260930/service_destinations.yaml" \
  --log <run>/home_events.jsonl --execute \
  --parking-contract "$HOME/jdamr_ws/install/jdamr_cube_navigation/share/jdamr_cube_navigation/config/box_parking_contract.yaml"
```

정밀 세션에서는 박스 계약을 넘겨야 `home`이 이동 전 검사를 통과한다. 도크 자체는 `parking_contract.yaml`(5 cm·3°)로 판정한다. 박스 앞 약 0.6 m 안에서는 이 단독 복귀를 쓰지 않는다(`restaurant_service home --help`).

## 기록

- 실행·변경 대장과 실패 원인: `jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md` §16–§17
- 도킹 자료 묶음과 포트폴리오 사용 범위: `jdamr_cube_navigation/evaluation/20260930_DOCK_RETURN_PORTFOLIO_HANDOFF.md`
