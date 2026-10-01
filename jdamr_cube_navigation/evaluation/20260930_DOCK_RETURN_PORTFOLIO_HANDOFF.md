# 충전소 후진 도킹 — 포트폴리오 인계 (2026-09-30)

## 이번 자료의 용도

JD-AMR이 방 가운데에서 출발해 도크 앞 0.7 m 정렬 지점(staging)에 선 뒤 직선 후진으로 충전소 정위치에 들어간 실행이다. 이동 전 경로 검사가 찾아낸 지도 흔적을 LiDAR 근거로 정리한 과정과, 스캔 정합으로 보강한 출발 초기화를 함께 보여 주는 자료다. 도크 도달은 사용자가 현장에서 확인했다. 테이블 박스 5 cm 정밀 주차 성공 자료가 아니다.

실패·원인 해석의 원본은 [실패 기록 §16–§17](20260929_PARKING_FAILURES.md)이다. 이 문서는 그중 포트폴리오에 쓸 수 있는 사실과 쓰지 않을 주장을 나눈다. 포트폴리오 사이트 수정·배포는 이번 작업에 포함하지 않는다.

## 그림

생성 스크립트: `jdamr_cube_navigation/evaluation/dock_return_20260930_figures.py` (입력은 아래 데이터 묶음만 읽는다).

```bash
cd "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros"
python3 jdamr_cube_navigation/evaluation/dock_return_20260930_figures.py --out portfolio_data/figures
```

| 파일 (`portfolio_data/figures/`) | 내용 | 혼동하면 안 되는 내용 |
|---|---|---|
| `dock_return_20260930_overview.png` | 수정한 지도·keepout 위에 출발 자세(초기화 AMCL), staging·도크 차체 윤곽, 후진 직선, 도킹 뒤 LiDAR 한 장(스캔 정합 자세로 투영), 수정 칸·해제 칸 | 출발에서 staging까지 선은 그리지 않았다. 계획·주행 궤적을 기록하지 않았다 |
| `dock_return_20260930_dock_detail.png` | 도크 주변 수정 전후. 흔적 3칸, staging 회전 원 0.414 m, 도크 초기화(10:58) LiDAR | 오른쪽 LiDAR는 10:58 도크 초기화 scan이다. 도킹 뒤 scan이 아니다 |
| `dock_return_20260930_timeline.png` | 세션 재시작 → 스캔 정합 초기화 → staging 이동 → 정지 확인 → 후진 도킹 시각 | 시각은 Nav2 journal·이벤트·도구 출력에서 옮겼다. 보간하지 않았다 |

## 원본

데이터 묶음: `$HOME/jdamr_data/dock_return_success_20260930/` (1.9 MB, `MANIFEST.sha256` 39개 파일). 폴더 구성은 그 안의 `README.md`에 있다. 큰 원본은 Git에 넣지 않는다.

| 항목 | 경로 (묶음 기준) |
|---|---|
| 실행 명령·이벤트 | `run/command.txt`, `run/home_events.jsonl` |
| Nav2 로그 | `logs/nav2_session_115630_120230.log` |
| 초기화 | `init/refine.json`(스캔 정합), `init/activation.jsonl`(AMCL) |
| 최종 자세 | `final_pose/final_pose_vs_dock.json` |
| 지도·keepout·근거 | `assets/map_dockfix.yaml`, `assets/keepout_dockfix.yaml`, `assets/dockfix_provenance.json`, 원본 `assets/local_updated.yaml`·`assets/keepout.yaml` |
| 계산 수치 | `VERIFIED_METRICS.json` |

지도 식별: `map_dockfix.yaml` sha256 `376a5ecd…`, 이미지 `1558ce07…`; `keepout_dockfix.yaml` `8a9bc37b…`, 이미지 `39efdbe6…` (전체 값은 `SESSION.yaml`).

## 한 일과 근거

| 단계 | 한 일 | 근거 |
|---|---|---|
| 이동 전 경로 검사 | 복귀 명령이 움직이기 전에 staging→도크 후진 통로 전체의 차체 윤곽(0.38×0.58 m)을 저장 지도·keepout에 대조했다. 미확인 칸도 막힌 것으로 본다(`static_corridor_clear`) | `precheck_blocked/home_events.jsonl` |
| 흔적 판정 | 막은 칸 3개 중 2개는 확인된 도크 자세에서 차체 안쪽(base_link 뒤 0.19–0.24 m)이고, 도크 LiDAR는 뒤쪽 0.77 m 안에 반사가 없었다. 나머지 미확인 1칸은 도크 scan 빔 287개가 통과하고 반사가 0이었다 | `assets/dockfix_provenance.json`, `init/dock_init_*_105836.json` |
| 최소 수정 | 지도는 그 3칸만, keepout은 staging 회전 원(0.414+0.10 m)과 후진 통로(반폭 0.29+0.10 m)에 걸린 46칸만 열었다. 전부 비운 마스크는 세션 기동 검사가 거부한다(`keepout mask contains no blocked cells`) | 같은 파일, 실패 기록 §16.3 |
| 출발 초기화 | 세션을 다시 띄운 뒤 초기 자세를 LiDAR 스캔 정합으로 다듬고(inlier 0.50, 전역 탐색 일치) AMCL 결과와 대조했다(0.4 cm·0.1°) | `init/refine.json`, `init/activation.jsonl` |
| staging 이동 | Nav2가 (0.94, 0.07)에서 (−0.64, −0.14)로 106 s 동안 이동했다(직선거리 1.59 m, 주행 길이 미기록). 정지 확인에서 staging 목표 대비 2.6 cm·1.7°를 1.0 s 유지했다 | Nav2 journal, `run/home_events.jsonl` |
| 후진 도킹 | 정지 자세에서 다시 만든 0.7 m 직선 후진 경로를 검사한 뒤 FollowPath로 52 s 동안 후진했다. 도크 정위치 도달은 사용자가 현장에서 확인했다 | Nav2 journal, 사용자 확인 |
| 최종 자세 측정 | 12:13–12:14에 무이동으로 쟀다. 도크 목표 대비 TF 0.25 cm·+3.8°, 독립 스캔 정합 3.9 cm·−1.2° | `final_pose/final_pose_vs_dock.json` |

## 쓸 수 있는 주장과 쓰지 않을 주장

| 쓸 수 있음 | 쓰지 않음 |
|---|---|
| 이동 전 차체 윤곽 통로 검사가 지도 흔적을 찾아냈고, LiDAR 근거로 필요한 칸만 고쳤다 | 지도를 새로 만들었다(이번은 3칸 수정본이다) |
| 스캔 정합으로 초기 자세를 보정하고 AMCL과 교차 확인한 뒤 출발했다 | 스캔 정합이 외부 ground truth다 |
| 도크 앞 정렬 뒤 0.7 m 직선 후진으로 충전소 정위치에 도달했다(현장 확인) | 도킹 정밀도 0.25 cm(AMCL 내부 추정일 뿐이다), 2.6 cm(staging 값이다) |
| 실행 시간: staging 이동 106 s, 후진 52 s | 주행 거리·궤적(기록 없음) |
| — | 서비스가 도킹 성공으로 판정했다(끝 구간 yaw 보정 중 Nav2 진행 검사 중단 105로 실패 판정이었다) |
| — | 테이블 박스 5 cm 정밀 주차·왕복 성공 |

면접에서 끝 구간을 물으면 실패 기록 §16.4의 해석으로 답한다. 컨트롤러 기준 yaw 오차 3.8°가 도크 허용 3°를 넘어 방향 보정이 이어졌고, AMCL yaw 1σ(4–5°)가 허용보다 컸다. 이 해석은 cmd_vel 기록이 없어 추론이다.

## 포트폴리오 문안 초안

아래는 외부 공개용 초안이다(문체 규칙 `humanizer-ko` 적용). 수치는 위 표 범위 안에서만 쓴다.

한 줄 요약

> 이동 전 통로 검사가 찾은 지도 흔적을 LiDAR 근거로 정리하고, 도크 앞 정렬 뒤 직선 후진으로 충전소 정위치에 복귀시켰습니다.

본문

> 충전소 복귀는 도크 앞 0.7 m 지점에서 방향을 맞춘 뒤 곧게 후진해 들어가는 방식입니다. 복귀 명령은 로봇이 움직이기 전에 차체 윤곽이 지나갈 통로 전체를 저장 지도와 금지구역 마스크에 대조하는데, 이 검사가 도크 뒤와 통로 가운데에서 지도에만 있는 칸 세 개를 찾아냈습니다.
>
> 지우기 전에 실제 장애물인지 확인했습니다. 두 칸은 도크에 선 차체 안쪽이라 물체가 있을 수 없는 자리였고, 나머지 한 칸은 도크에서 쏜 LiDAR 빔이 그대로 지나갔습니다. 지도를 만들 때 잠깐 있던 물체로 보고 그 세 칸만 고쳤고, 금지구역도 도크 앞 회전 반경과 후진 통로에 걸린 부분만 열었습니다.
>
> 다시 띄운 Nav2 세션에서는 RViz로 찍은 초기 위치를 LiDAR 스캔 정합으로 다듬고 AMCL 결과와 1 cm 안에서 맞는지 확인한 뒤 출발했습니다. 로봇은 방 가운데에서 도크 앞 정렬 지점까지 이동해 정지를 확인했고, 0.7 m 직선 후진으로 충전소 정위치에 들어갔습니다.

개조식 항목 (이력서·카드용)

- 이동 전 차체 윤곽 통로 검사로 도크 경로의 지도 흔적 3칸 검출, LiDAR 빔 통과 근거로 해당 칸만 수정
- RViz 초기 위치를 LiDAR 스캔 정합으로 보정하고 AMCL과 교차 확인하는 출발 초기화 도구 구성
- 도크 앞 정렬 뒤 0.7 m 직선 후진으로 충전소 정위치 복귀

그림 캡션

1. overview: 저장 지도 위에 출발 자세, 도크 앞 정렬 지점과 도크의 차체 윤곽, 후진 경로, 도킹 뒤 LiDAR 스캔을 겹쳤습니다. 주황 칸은 LiDAR 근거로 고친 지도 흔적, 연녹색은 도크 통로를 위해 연 금지구역입니다.
2. dock_detail: 도크 주변 수정 전후입니다. 흔적 칸이 차체 윤곽과 회전 반경 안에 있어 이동 전 검사가 통로를 막았고, 고친 뒤에는 회전과 후진 통로가 같은 검사를 통과했습니다.
3. timeline: Nav2 세션 재시작부터 스캔 정합 초기화, 정렬 지점 이동, 정지 확인, 후진 도킹까지 약 5분 20초의 실행 순서입니다.

## 다음 실차에서 확보할 것

1. rosbag으로 cmd_vel·odom·`/plan`·AMCL을 남겨 도크 끝 구간 왕복·회전을 궤적으로 보여 준다.
2. 도크 최종 자세를 줄자나 바닥 표시로 한 번 실측해 내부 추정과 대조한다.
3. 외부 촬영으로 staging 정렬과 후진 진입을 한 화면에 담는다.
4. 새 지도·박스 배치([설치 계획](20260930_MAP_BOX_LAYOUT_PLAN.md)) 뒤 테이블 왕복을 같은 형식의 새 run 폴더로 분리한다. 이번 자료를 덮어쓰지 않는다.
