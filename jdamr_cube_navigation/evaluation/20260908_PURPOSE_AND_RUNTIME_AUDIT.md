# SLAM·자율주행 목적 및 실행 점검 — 2026-09-08

## 작업 목적과 현재 증거

목표는 기존 복도 SLAM·저장지도 주행을 기반으로, 지도에 없던 장애물 우회와 돌발 장애물
정지·재개를 실제 로봇의 기록으로 설명할 수 있는 포트폴리오다. 탐사 목표 선택 정책의
성능 개선은 이 기반 위에 얹는 연구 항목이다.

| 항목 | 현재 확보한 증거 | 남은 범위 |
|---|---|---|
| 저장지도 실차 주행 | 복도 왕복 20개 목표, AMCL 누적 경로 77.090 m | 장애물 시나리오 실차 기록 |
| SLAM·센서 강건성 G001/G008 | 시뮬레이션 강건성, 실차 bag subdivision 비교 | 다른 실차 환경에 대한 일반화 |
| 위치 추정 G002/G009 | 정상·초기 오프셋 복구, 납치 비복구 분류, P0 유지 | 납치 원인의 입자 수준 계측은 미완 |
| 고정 장애물 G003 | 별도 평가 구성에서 시뮬레이션 25회 통과, 온보드 후보 연결 구현 | 후보 실행·실차 우회 검증 |
| 돌발 장애물 G004 | 시뮬레이션 정지·동일 goal 재개 15회 통과 | 실차 감시 설정에서 정지·재개 증거 |
| 탐사 정책 G005 | 실제 Nav2 첫 목표 도착, 짧은 실행 경로 | 정책별 본평가·우열은 미확정 |
| 통합 후보 G006/G010 | G006 smoke 구현, G010 계획 | G006 READY 경로와 G010 구현 미완 |

실차 위치는 AMCL 추정값이다. 77.090 m를 외부 정답 위치로 측정한 절대 정확도라고
설명하지 않는다. 기존 실차 성공의 상세 근거는
[인계 문서](20260904_HANDOFF.md)의 1.11절에 있다.

## 재현한 G005 오정지와 수정

commit `cdc4f32`에서 `current`, layout seed 11을 60초 실행했다. 첫 Nav2 목표 도착
후 다음 목표를 수행하는 중 `runtime_readiness_timeout`이 발생했다.

원인은 `_ready()`에 서로 다른 두 조건이 들어 있었기 때문이다. 초기 준비에는 노드·센서·
지도 연결이 필요하고, 다음 탐사 목표 계산에는 새 지도가 안정된 상태가 필요하다. 기존
코드는 출발 후에도 시작 시점 기준 45초 제한을 계속 적용해서, 주행 중 정상적인 지도
갱신으로 `_ready()`가 일시적으로 false가 되면 전체 실행을 무효 처리했다.

최초 준비 완료를 `_startup_ready`로 보존하도록 수정했다. 이후 지도 안정화는 다음 목표
계산에만 적용하고, 기존 lifecycle·센서·TF·명령 발행 권한 검증은 계속 수행한다.
초기 준비가 한 번도 끝나지 않은 경우의 timeout도 유지한다.

수정 전 재현 자료는
`$HOME/jdamr_artifacts/g005_audit_20260908.ULa0nU/probe/current__seed_11.runtime.log`와
같은 디렉터리의 request JSON에 보존했다. wall 실행 시간은 정리를 포함해 61.25초다.
이 진단은 full15 결과가 아니다.

수정 후 단일 60초 진단은
`$HOME/jdamr_artifacts/g005_diagnostic_20260908_v01/diagnostic.json`에 있다. 첫 목표
도착 뒤 두 번째 목표를 시작했고, `runtime_readiness_timeout`을 포함한 runtime invalid
사유는 0개였다. 종료 정리를 포함한 wall time은 70.63초, 잔존 프로세스는 0이며 production
입력 hash는 같았다. 보존한 runtime 로그는 88,082 bytes다.

두 번째 목표에서는 RPP(경로를 추종하며 예상 충돌을 검사하는 컨트롤러)의
`detected collision ahead`와 `Controller patience exceeded`가 반복됐다. 계획 경로와
local costmap/footprint의 관계는 추가 확인 대상이다. 충돌 검사를 완화하지 않았으며,
이번 진단을 G005 완주나 full15 실행 성공으로 해석하지 않는다. 종료 중 Nav2 callback의
지연 때문에 launch 프로세스는 SIGTERM까지 사용했고, 그 종료 코드도 결과에 남겼다.

## 짧은 실행과 실패 증거 보존

`run_frontier_policy_full.py --diagnostic`은 지정한 정책·seed 한 개만 제한시간 동안
실행한다. 기본은 60초이며 `diagnostic.json`에 `full_matrix_complete=false`와
`production_change_authorized=false`를 기록한다. `TIME_BUDGET_REACHED`는 정해 둔
관찰 시간이 끝났다는 뜻이다. 목표 완주나 정책 평가 PASS를 뜻하지 않는다.

실행 오류가 발생해도 요청, 측정값이 존재하면 그 원문, 프로세스 종료 코드, CPU·RSS,
잔존 프로세스 수와 로그를 보존한다. 본평가 실패는 고유한 `*.failed` 디렉터리로 남고
`failure.json`을 가지며, 유효한 `manifest.json` 이름으로 제공하지 않는다. 긴 로그는
시작과 끝을 합쳐 약 128 KiB만 보존한다. 정상 결과의 최종 검증 실패도 동일하게 보존한다.

본평가의 15회 계약을 바꾼 것은 아니다. 짧은 진단 결과를 본평가 validator에 넣어 정책을
승격할 수 없다. 명령은 [G005 실행 안내](20260907_G005_VERTICAL_READINESS.md)에 있다.

action 취소 응답은 요청 goal UUID와 응답 코드를 확인한다. planner timeout 뒤에 다음
계획을 겹쳐 보내지 않으며, goal/cancel 응답 대기가 제한시간을 넘으면 현재 명령값을
포함한 INVALID 측정 원문을 기록하고 runner 종료로 넘긴다. 일찍 끝난 INVALID 기록에
미래 900초의 관측값을 만들어 넣지 않는다.

## 전체 계획에서 바로잡은 의존성

G005는 GT(시뮬레이터 정답 위치)와 관측 영역 공개 지도를 사용하는 탐사 정책 평가다.
저장지도·AMCL 기반의 실차 장애물 주행과 입력·역할이 다르므로, G005/G010 완료를 모든
실차 장애물 준비의 필수 선행조건으로 두지 않는다.

점검에서 확인한 실차 장애물 준비의 차이와 처리 상태는 다음과 같다.

1. G003은 주행 중 경로를 다시 계산하는 평가용 BT(행동 순서를 정의한 트리)를 사용하지만,
   기본 온보드 복도 BT는 경로 추종 실패 후 복구·재시도 중심이다. 아래 후보 프로필에
   평가 BT와 Wait 전용 behavior server를 함께 연결했다. 기존 기본 프로필은 유지했다.
2. G004는 전용 StopZone과 scan 전달 경로를 사용했다. 실차의 기존 감시 polygon·scan
   설정에 그 정지거리와 지연 수치를 그대로 적용할 수 없다.
3. G006은 `READY_PATH_ENABLED=False`이며 builder가 G002 완료 결과를 아직 소비하지
   않는다. G009/G010 소비도 미구현이다. 이를 준비 완료 플래그로만 바꾸지 않는다.
4. 기존 bag의 충돌 감시 상태·경로 형상을 분석해 다음 주행에서 필요한 증거를 수집하고,
   동일 goal 재개 여부는 action UUID가 있는 기록으로 별도 확인해야 한다.

## 주요 구현 우선으로 추가한 내용

### 온보드 장애물 후보 프로필

`navigation_profile`은 `corridor`가 기본값이며 `obstacle_candidate`를 명시하면 다음이
함께 바뀐다. 실제 로봇에 배포하거나 이 후보로 주행한 상태는 아니다.

| 연결 위치 | 구현 |
|---|---|
| `scripts/corridor_autorun.sh` | `--navigation-profile` 하나를 core launch·계획 확인·실제 route 실행에 동일하게 전달 |
| `launch/onboard_keepout_navigation.launch.py` | 프로필을 core에 전달하고 기존 온보드 MCAP 기록 유지 |
| `launch/onboard_nav2_core.launch.py` | 후보는 `navigate_to_pose_dynamic_obstacle_eval.xml`, 기본은 기존 fail-fast BT 선택 |
| 후보의 추가 구성 | 기존 합성 컨테이너에 Wait 전용 behavior server를 넣고 lifecycle·노드 감시에 함께 등록 |
| `corridor_route.py` | 각 NavigateToPose goal의 명시 BT에도 동일한 프로필 적용 |

후보 BT는 경로 추종 중 경로를 다시 계산하고, 복구 시 costmap 초기화와 대기를 사용한다.
새로운 회전·후진 복구는 켜지 않는다. 지도·Keepout(진입 금지 영역)·AMCL P0·Collision
Monitor(충돌 감시기)·속도 및 timeout 값은 기존 것을 사용한다. 따라서 정지 감시 영역이나
정지 지연까지 G004 평가 구성과 동일해진 것은 아니다. 장시간 통로가 막혔을 때 같은 목표로
무기한 대기하는 기능도 아니다.

실차 배포 후 자동 실행기의 `--navigation-profile obstacle_candidate` 옵션으로 선택한다.
launch와 route를 따로 실행한다면 각각 `navigation_profile:=obstacle_candidate`,
`--navigation-profile obstacle_candidate`로 맞춰야 한다. 옵션을 생략하면 기존 복도
프로필을 사용한다. 기본 자동 실행기의 대기·종료 시간도 이번 작업에서는 변경하지 않았다.

### 목표 ID와 주행 이벤트 데이터

route 로그에 `route_event` JSON을 추가했다. 목표 수락, 취소 요청, 수신한 실행 결과를
goal UUID(개별 이동 요청의 고유 ID), 원본 경로의 waypoint 번호·ID, 프로필과 함께 남긴다.
기존 사람이 읽는 로그는 유지한다. 취소 요청만으로 취소 완료를 만들어 기록하지 않는다.

후처리는 다음 자료를 기존 bag을 읽는 한 번의 루프에서 추출한다.

- 충돌 감시기의 action·polygon 관측과 상태 전환
- 최종 속도 명령의 0 → 비영 값 전환
- `/plan`의 좌표계·경로 형상 해시·길이·변경 횟수
- route 로그의 goal UUID 이벤트

속도 명령의 0은 실제 바퀴 정지를 뜻하지 않고, 경로 변경 횟수만으로 우회 성공을
판정하지 않는다. bag은 recorder 시간, goal 이벤트는 ROS logger 시간이므로 시간 기준도
각각 기록한다. 동일 goal 재개의 자동 판정은 아직 `NOT_MEASURED`다. 기존 로그에 없던
UUID나 충돌 감시 상태를 추정해서 채우지 않는다.

`--metrics-only`는 지표 JSON/YAML·궤적 CSV·출처 해시만 생성한다. 지도·GIF·MP4를
다시 만들지 않으며, 기존 결과를 섞거나 덮어쓰지 않도록 새 출력 디렉터리를 사용한다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
export PYTHONPATH="$PWD/jdamr_cube_navigation/evaluation:$PWD/jdamr_cube_navigation:$HOME/.local/share/jdamr-slam-eval/python:$PYTHONPATH"
python3 jdamr_cube_navigation/evaluation/corridor_run_media.py \
  --run-dir "$HOME/jdamr_artifacts/corridor_localdds_armed_20260904T152036" \
  --output-dir "$HOME/jdamr_artifacts/corridor_obstacle_evidence_20260908_v02" \
  --metrics-only
```

기존 실차 완주 bag에 한 번 적용한 결과는
`$HOME/jdamr_artifacts/corridor_obstacle_evidence_20260908_v01/metrics.json`에 있다.
출력 5개 파일의 디스크 사용량은 약 76 KiB이며 원본 bag과 영상은 복제하지 않았다.
주행 구간에서 경로 메시지 20개, 경로 형상 전환 19개, 속도 명령 재개 8개를 관측했다.
이 구간의 충돌 감시 메시지와 UUID 로그는 없으므로 장애물 대응 성공으로 해석하지 않는다.
원본 실차 완주 20개 목표·AMCL 경로 77.090 m라는 기존 결과는 유지된다.

### 확인 범위

사용자가 반복 평가를 후순위로 정하기 전에 수행한 G005 관련 집중 검증은 96개 통과했다.
그 이후 추가한 온보드 후보·로그·metrics-only는 Python 구문 검사, 셸 구문 검사,
launch 인자 로딩과 기존 bag 지표 생성까지만 확인했다. 후보로 Nav2를 실제 기동한 결과나
전체 회귀 테스트 통과를 주장하지 않는다. 이번 턴에서 full15 추가 실행은 하지 않았다.
별도 verifier가 프로필 전달·Wait 의존성·기록 경로·렌더 생략 분기를 읽기 전용으로
확인해 구현 수준 PASS, 구체적 기능 blocker 없음으로 판정했다. 실차 실행 승인은 아니다.

## 다음 순서

운영용 BT의 후보 연결과 기존 온보드 기록 후처리는 구현했다. 다음은 새 목표 이벤트와
감시 상태를 함께 사용한 동일 목표 정지·재개 판정, 후보의 실행 확인이다. 그 뒤 고정
장애물 우회 1회와 돌발 장애물 정지·제거·재개 1회를 감독하에 기록한다. 이 횟수로
성공률의 통계적 일반화를 주장하지 않는다. 문제를 발견한 항목에만 추가 검증을 한다.

탐사 연구는 짧은 G005 진단 이후 정책 비교가 필요할 때 full15를 사용한다. gain-nav
후보가 실제로 나타날 때만 G010의 요소 제거 대조·미사용 환경 검증을 진행한다. G006은
탐사 후보를 통합할 때 의존성을 구현한다. 이 연구 분기를 완료했다고 실물 이동 권한이나
실차 장애물 검증이 자동으로 생기는 것은 아니다.

`.omx/ultragoal/goals.json`에는 G002/G009의 오래된 진행 상태가 남아 있다. 기존 완료
artifact와 이 문서의 범위를 먼저 확인하며, 원장 상태만 보고 완료 실험을 재실행하지
않는다. 기존 원장의 미커밋 변경은 이번 수정에 포함하지 않았다.
