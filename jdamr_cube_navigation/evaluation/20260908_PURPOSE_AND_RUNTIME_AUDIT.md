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
| 고정 장애물 G003 | 별도 평가 구성 25회 통과, 실제 온보드 후보 detour 1회 통과 | 실차 우회 검증 |
| 돌발 장애물 G004 | 별도 구성 15회 통과, 기존 대표 bag에서 동일 목표 재개 판정 재현 | 온보드 감시 설정에서 정지·재개 증거 |
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
함께 바뀐다. 파이에는 아래 런타임 파일을 배포·빌드했지만 후보로 실차 주행하지 않았다.

| 연결 위치 | 구현 |
|---|---|
| `scripts/corridor_autorun.sh` | `--navigation-profile` 하나를 core launch·계획 확인·실제 route 실행에 동일하게 전달 |
| `launch/onboard_keepout_navigation.launch.py` | 프로필을 core에 전달하고 기존 온보드 MCAP 기록 유지 |
| `launch/onboard_nav2_core.launch.py` | 후보는 `navigate_to_pose_dynamic_obstacle_eval.xml`, 기본은 기존 fail-fast BT 선택 |
| 후보의 추가 구성 | 기존 합성 컨테이너에 Wait 전용 behavior server를 넣고 lifecycle·노드 감시에 함께 등록 |
| `corridor_route.py` | 각 NavigateToPose goal의 명시 BT에도 동일한 프로필 적용 |

후보 BT는 경로 추종 중 경로를 다시 계산하고, 복구 시 costmap 초기화와 대기를 사용한다.
새로운 회전·후진 복구는 켜지 않는다. 지도·Keepout(진입 금지 영역)·AMCL P0·Collision
Monitor(충돌 감시기)·속도 및 timeout 값은 기존 것을 사용한다. 단, 아래 합성 컨테이너
수정으로 자식 costmap에 이 설정이 실제로 전달되므로 기존 실행과 유효 설정까지 같다고
말할 수는 없다. 따라서 정지 감시 영역이나
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
각각 기록한다. 동일 goal 재개의 자동 판정은 이제 bag 안의 action status와 충돌 감시·
속도 명령을 모두 recorder 시간으로 정렬해 수행한다. 기존 로그에 없던 UUID나 충돌
감시 상태를 추정해서 채우지 않는다.

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
새 reader와 판정을 연결한 뒤 `corridor_obstacle_evidence_20260908_v02`에도 지표만
재생성했다. 완주 결과는 같고 action status 누락으로 동일 목표 재개는 `NOT_MEASURED`다.
이 결과도 약 76 KiB이며 영상·원본 bag은 복제하지 않았다.

### 동일 목표 정지·재개 판정

`same_goal_resume_evidence.py`는 STOP 관측 → 그 뒤의 0 속도 명령 → 감시 해제 →
비영 속도 명령을 하나의 구간으로 묶고, 그 사이 action UUID가 유지됐는지 확인한다.
취소·실패·다른 목표·여러 활성 목표·비정상 메시지·사건 순서가 모호한 동일 timestamp는
확인 결과에서 제외한다. 출발 전에 관측한 오래된 0 명령은 STOP 증거로 쓰지 않는다.
관측한 명령 재개와 최종 목표 성공은 별도 필드다. 바퀴 정지·제동거리·안전 인증을
판정하지 않으며 결과 범위는 `COMMAND_SPACE_ONLY`다.

기록기에 숨겨진 `/navigate_to_pose/_action/status`를 명시하고
`--include-hidden-topics`를 추가했다. 이 상태 토픽은 reliable·transient_local·depth 1로
받고 고주기 센서·제어 토픽의 best-effort 정책은 유지한다. ROS action의 상태 QoS와
숨겨진 이름 규약은 [ROS 2 action 설계](https://design.ros2.org/articles/actions.html)에
따른다. 상태 메시지가 없는 과거 bag은 계속 `NOT_MEASURED`로 남는다.

일부 정상 rosbag MCAP은 메시지 타입 이름만 있고 schema 본문이 비어 있었다. 허용한
topic/type 조합의 빈 schema만 설치된 ROS 메시지 타입으로 CDR 역직렬화한다. schema가
포함된 기존 bag은 자체 정의로 해독하며, 타입 불일치나 해독 실패를 무시하지 않는다.
이 reader는 복도 후처리와 단독 판정 도구가 함께 사용한다. ROS 그래프 재생은 하지 않는다.

기존 G004 `sudden_obstacle_stop_resume__seed_11` 대표 bag에 단독 도구를 적용했다.
action status 2개, 충돌 감시 상태 2개, 속도 명령 1,734개에서 동일 UUID의 명령 재개
1구간과 최종 성공을 확인했다. 원본 hash와 시각은
`$HOME/jdamr_artifacts/same_goal_resume_20260908_v01/g004_existing_sudden.json`에 있다.
새 주행이 아니라 기존 평가용 설정의 기록 재판정이며, 온보드 후보 검증을 대신하지 않는다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/jdamr_cube_navigation/evaluation:$PWD/jdamr_cube_navigation:$HOME/.local/share/jdamr-slam-eval/python:$PYTHONPATH"
python3 jdamr_cube_navigation/evaluation/evaluate_same_goal_bag.py \
  --mcap "$HOME/jdamr_artifacts/sim_collision_monitor_eval_20260906_v08_representatives_seed11_cyclone/sudden_obstacle_stop_resume__seed_11/bag/bag_0.mcap" \
  --output "$HOME/jdamr_artifacts/same_goal_resume_20260908_v02/g004_existing_sudden.json"
```

### 합성 컨테이너의 costmap 설정 전달 수정

실제 `onboard_nav2_core.launch.py`를 격리 Gazebo에서 기동하니 지도·mask 서버와
lifecycle은 정상이었지만, 자식 costmap의 `keepout_filter.enabled`는 설정되지 않았다.
planner/controller component에만 ParameterFile을 전달하고, 이들이 내부에서 생성하는
costmap에 필요한 컨테이너 프로세스의 인자에는 전달하지 않은 것이 원인이었다.

Nav2 설치본 `nav2_bringup/launch/bringup_launch.py`와 동일하게
`ComposableNodeContainer(parameters=[configured_params])`를 추가했다. 수정 후 양쪽
costmap에서 Keepout 활성값과 filter info·mask 실제 수신을 확인했다. mask 서버의
ACTIVE만 보고 필터 적용 완료로 판단하면 안 된다. 기존 완주 기록은 유지하지만 당시의
동일 커스텀 launch에 대해 Keepout 실제 적용까지 입증했다고 주장하지 않는다.

격리 실행은 `run_onboard_candidate_smoke.py`에서 domain 186/187·LOCALHOST·고유
Gazebo partition으로 제한한다. 운영 설정에서 시뮬레이터 초기 위치와 전체 costmap
관측값 전송만 바꾸고 원본 설정은 보존한다. 별도 평가의 `inf_is_valid` 값을 운영 설정에
추가하지 않는다. 비어 있지 않은 경로 밖 금지구역도 생성해 필터 활성 상태를 확인한다.
원본 로그와 action UUID/status를 남기고 action 성공만으로 우회 성공을 판정하지 않는다.
GT(시뮬레이터 정답 위치)의 목표 도달, 장애물 marking, 후속 경로 관측과 무접촉이 필요하다.

첫 후보 실행에서는 action 성공인데 GT가 시작점에 남는 오판도 잡았다. 초기 위치는
`map→base=-8 m`로 정상이었고, 직접 원인은 시간 혼용이었다. core 로그의 경로 시각은
wall time인데 TF는 sim time이었으며 충돌 감시기에도 같은 시간차 오류가 있었다.
원본 YAML에 `use_sim_time` 키가 없어 leaf 이름만 지정한 RewrittenYaml이 이를 추가하지
못했다. component와 컨테이너의 자식 노드에 시간 설정을 명시 전달하도록 수정했다.
정지 중 AMCL 오수렴이라는 초기 추측은 이 로그로 대체한다. 출발 전 계측을 줄이는 것은
지연 최적화일 뿐 원인 수정이 아니다. 실차 기본값 `use_sim_time=false`는 유지한다.

### 수정 후 실제 온보드 후보의 격리 실행 결과

`$HOME/jdamr_cube_ws/onboard_candidate_smoke_20260908_detour_v3`의 seed 11 detour
한 건이 `SIM_INTEGRATION` 범위에서 통과했다. 별도 G003 평가 launch가 아니라 수정한
`onboard_nav2_core.launch.py`를 사용했다. 운영 AMCL·RPP·충돌 감시 설정은 그대로이고,
시뮬레이션 초기 위치·전체 costmap 관측 전송·시간 선택만 실행 환경에 맞췄다.

| 확인 항목 | 기록 |
|---|---|
| GT 시작 → 최종 위치 | `(-8, 0) → (5.96797, 0.07743) m` |
| 장애물 관측 | marking scan, global/local blocking 모두 기록 |
| marking 이후 경로 | 56개 관측; 이 개수만으로 우회를 판정한 것은 아님 |
| 후보 동작 | 주행 중 경로 재계산, bounded 복구에서 Wait 실행·완료 |
| 목표 식별 | `ef13dc4b1155412c90b16441b1c05046`, `EXECUTING → SUCCEEDED` |
| 종료 | 접촉 0, 최종 0 명령 유지 2.005초, 잔존 프로세스 0 |
| 적용 상태 | 양 Keepout 활성·mask 수신, 출발 전 controller/CM sim time 확인 |
| 보존 용량 | 해당 case 약 6.46 MB, 별도 영상·원본 bag 복제 없음 |

이는 고정 장애물 우회 한 건이다. 돌발 STOP 후 재개나 `event_driven_removal`는 이번
온보드 후보 실행에서 수행하지 않았다. 장애물 제거·재계획을 돌발 정지·재개로 바꿔
부르지 않는다. 기존 G004의 같은 목표 재개 결과는 별도 감시 설정의 증거다.

재현 기준 SHA-256:

- core launch: `e4b19a17072a3756e8f1cb9a146d7adc8144dc88a670800cc5d6166d4e0ab5c6`
- 운영 YAML: `99db2ca2ca6790c6f27ba2a56a55c12ac1146dec29a4913a4d73d9948a7da50c`
- `detour/scenario.json`: `91b5c82e79daf60febb141fff2c15aa7807b4f53dafea036a9237f0287bf174f`
- `detour/summary.json`: `141a9e021644ea5433aea7512ce1c764fda3b7217486d85b97a52b6f36fd761a`

### 파이 배포 상태

SSH 대상은 `lim@jdamr.local`, 실제 workspace는 `$HOME/jdamr_ws`다. PC의
`$HOME/jdamr_cube_ws`와 다르다. 파이 source는 Git 작업 트리가 아니므로 선택 파일만
동기화했다. PC와 파이는 Nav2 upstream 버전 `1.3.12`이며 아키텍처별 패키지 빌드 시점은
다르다. 운영 nav2 YAML·liveness guard·복도 route YAML의 기존 hash가 일치함을 확인했다.

배포 파일은 core/wrapper launch, 자동 실행 shell, corridor route, 온보드 기록 설정,
기록 QoS YAML, 후보 BT의 7개다. 덮어쓴 파일은 파이의
`$HOME/jdamr_artifacts/onboard_candidate_backup_20260908.flpNF9/navigation_before.tar.gz`
에 보관했다. 후보 BT는 새 파일이다. 파이 패키지 빌드는 통과했고 core 최종 수정 파일도
다시 동기화해 PC와 hash를 맞췄다. launch `--show-args`, route `--help`, recorder의
숨겨진 topic 옵션 지원을 확인했다. 기존 base 서비스는 그대로 두고 Nav2·주행·새 기록은
시작하지 않았다. 물리 위치 초기화나 장애물 주행 검증 완료를 뜻하지 않는다.

### 이번 추가 작업의 검증

기록기·Keepout launch·동일 목표 판정·MCAP reader·후보 runner의 집중 테스트
126개가 1.14초에 통과했다. 기존 MCAP 의존성의 deprecation warning 1개는 남아 있다.
새 코드와 core launch의 `ament_flake8`, Python 구문 검사, 자동 실행 shell 구문 검사,
PC 패키지 빌드도 통과했다. 전체 저장소 테스트나 다른 패키지의 lint 통과 주장은 아니다.

별도 verifier가 구현과 원본 증거를 검토했다. STOP 이전의 오래된 0 명령이나 중간의
알 수 없는 감시 상태를 재개 증거로 인정하지 않도록 수정했다. 후보 runner도 양쪽
costmap blocking과 접촉 관측 publisher 존재를 필수로 확인한다. 접촉 로그가 없어서
0건인 상태를 무접촉으로 승격하지 않는다. v3 원본은 이 강화된 조건도 충족했으며,
원본 scenario·summary 파일을 덮어쓰거나 추가 시뮬레이션을 돌리지 않고 재판정했다.

### 확인 범위 — 이번 추가 작업 이전 기록

사용자가 반복 평가를 후순위로 정하기 전에 수행한 G005 관련 집중 검증은 96개 통과했다.
그 이후 추가한 온보드 후보·로그·metrics-only는 Python 구문 검사, 셸 구문 검사,
launch 인자 로딩과 기존 bag 지표 생성까지만 확인했다. 후보로 Nav2를 실제 기동한 결과나
전체 회귀 테스트 통과를 주장하지 않는다. 이번 턴에서 full15 추가 실행은 하지 않았다.
별도 verifier가 프로필 전달·Wait 의존성·기록 경로·렌더 생략 분기를 읽기 전용으로
확인해 구현 수준 PASS, 구체적 기능 blocker 없음으로 판정했다. 실차 실행 승인은 아니다.

## 고도화 우선순위

새 알고리즘을 추가하기 전에 온보드에 실제 연결된 설정·목표 ID·감시 상태·GT를 하나의
근거로 묶는 것이 우선이다. 이번에 발견한 자식 costmap 누락처럼 구성 차이가 알고리즘
비교보다 먼저 결과를 바꿀 수 있기 때문이다.

- 현재 후보: 주행 중 재계획과 짧은 정지 후 같은 목표 재개. LiDAR만 사용한다.
- 필요할 때 다음: 원인별 local/global costmap 복구. clear 서비스 성공은 장애물 제거
  확인이 아니므로 센서 재관측과 경로 변화를 함께 확인한다.
  [Nav2 기본 BT 설명](https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/detailed_behavior_tree_walkthrough/detailed_behavior_tree_walkthrough/)
- 목표 가까이에서 불필요한 우회가 실제로 관측될 때: 목표 근처 경로가 길어지면 잠시
  기다리는 BT를 별도 후보로 비교한다. 현재 후보에 추가한 기능은 아니다.
  [Nav2 목표 근처 장애물 대기 예제](https://docs.nav2.org/jazzy/getting_started/nav2_behavior_trees/trees/nav_to_pose_and_pause_near_goal_obstacle/nav_to_pose_and_pause_near_goal_obstacle/)
- MPPI 전환·사람 추적·깊이 카메라는 이번 실차 준비의 선행조건이 아니다. RPP의 실제
  한계와 파이의 여유 자원이 확인될 때만 별도 범위로 검토한다.

컨트롤러의 `failure_tolerance`는 명령 계산 실패 허용 시간이지, Collision Monitor의
정지 상태를 무조건 기다려 주는 시간은 아니다. 장시간 차단을 해결하려고 이 수치를
임의로 늘리지 않는다.
[Nav2 controller 설정](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/)

## 다음 순서

운영용 BT·목표별 기록·동일 목표 명령 재개 판정·파이 배포와 후보 detour를 확인했다.
다음 실차 순서는 실제 위치 초기화 후 고정 장애물 우회 1회, 이어서 돌발 장애물
정지·제거·재개 1회를 감독하에 기록하는 것이다. 운영 감시 설정의 돌발 STOP·재개는
아직 검증되지 않았으므로 해당 결과를 별도로 확인해야 한다. 이 횟수로
성공률의 통계적 일반화를 주장하지 않는다. 문제를 발견한 항목에만 추가 검증을 한다.

탐사 연구는 짧은 G005 진단 이후 정책 비교가 필요할 때 full15를 사용한다. gain-nav
후보가 실제로 나타날 때만 G010의 요소 제거 대조·미사용 환경 검증을 진행한다. G006은
탐사 후보를 통합할 때 의존성을 구현한다. 이 연구 분기를 완료했다고 실물 이동 권한이나
실차 장애물 검증이 자동으로 생기는 것은 아니다.

`.omx/ultragoal/goals.json`에는 G002/G009의 오래된 진행 상태가 남아 있다. 기존 완료
artifact와 이 문서의 범위를 먼저 확인하며, 원장 상태만 보고 완료 실험을 재실행하지
않는다. 기존 원장의 미커밋 변경은 이번 수정에 포함하지 않았다.
