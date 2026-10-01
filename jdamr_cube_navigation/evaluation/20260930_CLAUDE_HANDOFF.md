# Claude 인계: 테이블 정밀주차 실패와 실행 경로 재검증

작성일: 2026-09-30 KST. 아래 장비 상태와 검증 결과는 직전 세션 관측이며, 새 세션의 현재 상태를 보장하지 않는다.

## 1. 결론부터

**현 상태는 실차 태스크 완료가 아니다. 출발 준비 완료 또는 모든 문제 해결로 보고하면 안 된다.**

- 목표는 저장된 지도에서 테이블 근처로 이동하고, 박스 면에 차체를 평행하게 맞춰 차체 앞면 기준 약 5cm에 정지하는 것이다.
- 관측 위치까지 이동한 적은 있지만 9월 29일 시도의 박스 정밀주차는 완료되지 않았다. 과거 다른 조건에서 사용자가 확인한 5cm 접근 성공과 이번 수정본의 성공을 혼동하지 않는다.
- 여러 코드 결함을 수정·파이에 반영했으나, 추가 검증에서 아직 종료로 이어지는 경로 두 곳을 재현했다. 두 곳 모두 미수정이다.
- 센서·제어 지연 및 Nav2 연쇄 비활성화의 근본 원인도 확정되지 않았다. 박스 앞면 후보 전환 문제도 남아 있다.
- 테스트 수를 늘리는 것보다 아래 재현 조건과 실제 실행 연결을 확인하는 것이 우선이다.

## 2. 사용자의 목표와 작업 방식

사용자는 저속 학습·취업용 시제품으로 SLAM·자율주행·정밀주차를 구현 중이다. 출발을 요청할 때마다 새 검사·코딩·리뷰를 시작해 배터리를 소모하고, 정상 태스크는 완료하지 못한 일이 반복됐다. 같은 대기를 다시 요구하지 말 것.

당장 우선순위:

1. 준비·수정 단계에서 반복 출발 실패와 단계 전환 결함을 해결한다.
2. 사용자가 출발을 요청하면 준비된 실행 경로로 테이블 접근 → 면 정렬 → 5cm 정지까지 한 사이클을 수행한다.
3. 이후 동일 지도상의 충전소 위치·방향으로 돌아가 후면 주차하는 왕복 태스크를 완성한다.

전체 시나리오: 충전소 근처에서 시작 → 충전소 정렬/주차 → 5초 대기 → 선택한 테이블 → 정밀주차 → 5초 대기 → 충전소 복귀·주차. 물 따르는 중간 지점은 사용자가 생략했다. 일부 기존 CLI의 dwell 기본값은 20초이므로 최종 시나리오와 별도로 확인할 것.

- 1번 테이블은 지도상 가로 방향 주차, 2번은 정면 주차로 요청됐다. 그림에서 임의 좌표·방향을 다시 추정하지 말고 저장 목적지와 실측 박스 면을 따른다.
- 새 출발 위치는 충전소지만, 진행 중 재개 위치를 충전소로 덮어쓰면 안 된다.
- 출발 전에 배치·케이블 분리를 확인받았으면 같은 질문을 반복하지 않는다.
- 실행기가 끝났으면 계속 진행 중이라고 말하지 않는다. 관측 위치 도착을 정밀주차 완료로 처리하지 않는다.
- 사용자가 차체 손상을 감수한다는 의사를 밝혔다. 이를 사람·주변 물건과의 충돌 허용, 센서 소실 후 이동, 사용자 정지 무시로 확대하지 않는다. 동시에 관제 상태·문서·추가 리뷰를 출발 조건으로 만들지 않는다.
- 이 인계는 실물 이동 명령이 아니다. 이번 인계 작성에서는 주행 코드나 파이 상태를 바꾸지 않았다.

## 3. 저장소·장비·보존해야 할 변경

| 항목 | 인계 시 확인 상태 |
|---|---|
| PC 저장소 | `$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros` |
| 작업 브랜치 | `feat/restaurant-service-destinations` |
| 인계 작성 전 HEAD | `caf5ddb` |
| 원격 대비 | 로컬이 14커밋 앞섬. 이번 인계 커밋 전 기준이며 원격 push는 하지 않음 |
| 파이 접속 | `ssh lim@jdamr.local`로 직전 확인 성공 |
| 파이 workspace | `$HOME/jdamr_ws` |
| 파이 배치 소스 | `$HOME/jdamr_ws/src/jdamr_cube_ros` — Git 저장소가 아닌 배치본 |
| ROS | Jazzy, 온보드 domain 12, LOCALHOST 제어 그래프 |
| 표시 | PC RViz에는 별도 표시 중계 사용. 제어 그래프와 혼동하지 말 것 |
| 파이 마지막 확인 | `jdamr-base.service` active, MainPID 17643. system/user의 `jdamr-restaurant-navigation.service`는 inactive |
| 기존 사용자 변경 | `jdamr_cube_cartographer/rviz/jdamr_cube_cartographer.rviz` 수정 상태. 되돌리거나 임의 스테이징하지 말 것 |

차체 제원은 이미 확정됐다는 사용자 지시가 있다. 바퀴·제어보드·파이는 기존 계열을 사용하며 차체를 이식했다. 제원 원본은 `jdamr_cube_description/config/new_base_geometry.yaml`이다. 현재 코드의 앞면-바퀴축 거리는 0.065m, 바퀴 포함 폭은 0.540m를 사용한다. 사용자 측정 카메라 렌즈 중심 높이는 바닥에서 0.215m다. 다른 위치·회전·축간거리 등은 추정해서 덮어쓰지 말고 provenance와 설정을 확인한다.

지도/목적지의 세션 스크립트 기본 경로는 파이 `$HOME/jdamr_data/maps/20260922_manual_final_run2/service_destinations.yaml`이다. 실행 명령에 다른 registry가 전달됐는지는 별도로 확인한다. 지도·keepout 원본을 임의 재생성하지 않는다.

## 4. 가장 먼저 다룰 미수정 문제 — 코드 경로로 재현됨

### A. 정지 자세 캡처가 AMCL 메시지 나이만으로 실패

- 위치: `jdamr_cube_navigation/jdamr_cube_navigation/restaurant_service.py`, `capture_stationary_pose()`. caf5ddb 기준 약 730행.
- 해당 함수는 `_guard_failure(require_fresh_amcl=True)`를 호출한다.
- 기반 guard의 기본 `amcl_freshness_s`는 15초다.
- 재현: AMCL age 20초, 이동 누적 0, 정상 covariance, 새 scan/odom/battery, 새 TF 응답을 준비했다. 실제 `ServiceRoute._guard_failure`와 `capture_stationary_pose`를 사용했다.
- 결과: 일반 실행 guard(False)는 정상인데 캡처 guard(True)는 `AMCL pose stale`이었다. 2초 캡처 제한 안에서 `stationary teaching unavailable`로 끝났고 TF 조회는 0회였다.
- 이는 합성 입력으로 확인한 코드 동작이다. 과거 모든 출발 실패의 원인이 이것이라고 단정하지 않는다.
- 기존 `test_stationary_teaching_uses_fresh_unique_samples`는 guard를 항상 정상으로 대체하므로 이 조합을 검출하지 못했다.

수정 판단: 정지 중 메시지 갱신 빈도와 실제 위치추정 상실을 구분해야 한다. 무조건 나이 조건을 삭제하거나 가짜 AMCL/TF를 발행하지 않는다. 초기 위치 미설정, 실제 이동 후 위치추정 소실, 정지 중 정상 TF 유지의 세 경우를 나눠 기존 코드 안에서 처리한다. 최초 `wait_until_ready()`와 단계 전환 캡처가 서로 다른 전제를 갖는지도 함께 확인한다.

### B. 박스 관측 중 입력 공백은 재개 없이 즉시 종료

- 위치: `jdamr_cube_navigation/jdamr_cube_navigation/box_service.py`, `observe_target()`. caf5ddb 기준 약 205–208행.
- 관측 반복문에서 `_navigation_ready(False)`가 한 번 실패하면 즉시 `RuntimeError`를 발생시킨다.
- 재현: readiness를 `[False, True]`로 준비하고 첫 실패 이유를 `scan stale: age=2.6s limit=2.5s`로 넣었다. 실제 `observe_target()`를 실행했다.
- 결과: readiness 확인 1회, `_wait_for_input_recovery` 호출 0회, 예외 종료. 다음 정상 입력을 확인하지 않았다.
- `_observe_with_search()`는 `BoxObservationUnavailable`만 다루므로 이 RuntimeError는 탐색/회복 분기로 들어가지 않는다.
- 앞선 입력 회복 수정은 이동 action과 waypoint 경계에 적용됐지만 이 관측 단계에는 적용되지 않았다.

수정 판단: 차체가 정지한 관측 단계에서는 기존의 제한된 입력 회복을 활용하고, 회복 뒤 새 정지 pose와 새 관측을 취해야 한다. 예전 박스/pose를 재사용하거나 센서 지연을 박스 미검출로 바꿔 회전시키지 않는다. 반복 대기는 기존 관측/태스크 시간 예산 안에 둔다. `plan_pose`, 최종 정지 확인 등 다른 단계 경계에도 동일한 누락이 있는지 확인하되, 전체 조건을 일괄 완화하지 않는다.

## 5. 아직 해결 근거가 없는 실차 문제

### 박스 앞면 후보가 바뀜

실패 기록에서는 거리 0.638–0.656m와 1.04–1.125m, 각도 약 +32–35도와 -15–-21도가 번갈아 나왔다. 안정된 앞면 관측을 확보하지 못했다. 원시 RGB-D가 없는 후속 구간이 있어 배경 면인지 다른 박스 면인지 확정하지 못했다. 임계값만 바꾼 시도도 실패했다.

현재 코드는 각 단계에서 새 면을 관측하고 지도상 테이블 영역과 LiDAR 지지를 확인한다. 이것을 동일 물체·동일 면 추적이 완성됐다는 뜻으로 해석하지 않는다. 관측 위치 이동, 면 선택, 정렬 뒤 재관측이 같은 대상에 이어지는지 확인해야 한다.

### 센서·제어 지연 및 Nav2 연쇄 비활성화

18:29 보존 journal에는 scan age 2.637초, 10Hz controller 주기 미달, TF 미래 외삽, keepout heartbeat 10초 미수신과 연쇄 비활성화가 있다. 높은 부하가 있었지만 CPU·DDS·전원 중 어느 것이 최초 원인인지 확정되지 않았다.

Nav2를 끈 정지 기준선은 scan 약 9.638Hz, 최대 stamp age 0.118초였다. **이 결과는 Nav2·RGB-D·관제가 함께 동작하는 주행 부하의 정상 증거가 아니다.**

상세 근거는 `20260929_NAV2_PLATFORM_RESEARCH.md`와 `20260929_PARKING_FAILURES.md`를 읽는다. Pi라는 이유만으로 불가능하다고 단정하거나, 새 런타임·DDS·스택을 한꺼번에 바꾸지 않는다.

## 6. 이미 수정한 것 — 중복 작업 금지, 효과 범위는 구분

| 수정 | 현재 의미/한계 |
|---|---|
| 관측 transit에 `execute(final_parking=False)` | 관측 지점에 최종 1cm/1도 주차 판정을 적용하던 오류 수정 |
| 실제 Spin action과 `--search` | 동일 XY의 NavigateToPose를 탐색 회전 대용으로 쓰지 않음. 최대 30도씩 12회; 전체 태스크 시간도 적용됨 |
| `--resume-at-observation` | 도달한 관측 위치에서는 transit 생략. 최종 주차 생략 아님 |
| parameter client 재사용 (`b146696`) | 실제 rclpy 반복 조회에서 client 72개 누적 → 1개 유지. 지연의 유일 원인이었다는 증거는 없음 |
| RViz 중계 경량화 (`d704d76` 등) | 표시 부하 완화. 제어/센서 지연 해결로 승격하지 않음 |
| 현재 waypoint 1회 재개 (`cb21ce1`, `caf5ddb`) | 이전 action terminal 및 새 입력·정지 확인 후 남은 구간 재개. 관측 단계 누락은 §4-B |
| Spin/후진의 남은 구간 재계산 | 중간 취소 후 처음부터 같은 각도/거리를 반복하지 않음 |
| 중간 정렬 기준 분리 | 중간 45cm 정렬은 5cm/3도, 최종 주차는 기존 엄격 기준. 저속 Parking 제어기 유지 |
| 최종 접근 재계획 | 1Hz 재계획, 복구 가능한 오류에 1초 대기 후 1회 재시도 |
| 출발/진행 중 배터리 기준 분리 | 첫 action 수락 전 10.8V, 같은 실행기의 후속 단계 10.5V. SOC 측정값이나 부하 교정 결과 아님 |
| 지도 검사 캐시 (`caf5ddb`) | 문자열 캐시 키가 dict로 덮이던 오류 수정. 첫 호출→두 번째 재사용으로 검증 |
| 탐색 회전 차단/시간 초과 후 관측 위치 후보 | 정상 입력과 action 종료 확인 후 앞·뒤 20cm 후보를 Nav2로 계획, 가능한 위치 1회 시도. 임의 속도 명령 우회 없음 |
| 통신 기본값 통일 (`caf5ddb`) | 세션 셸·서비스 launch 기본 LOCALHOST, 명시적 SUBNET 옵션은 보존 |
| graph 감시와 실제 lifecycle 구분 | graph 감시기 종료만으로 전체 Nav2 shutdown하지 않음. 실제 핵심 프로세스/lifecycle 실패 보호는 유지 |
| 기동 후 후속 조회 실패 | 이미 정상 활성화된 서버를 단순 조회 실패로 RESET하지 않음 |

후진 goal 인덱스, 수락 handle 보관 순서, action UUID별 취소/terminal 확인, 정밀주차 smoother 속도 불일치도 앞선 수정에 포함됐다. 전체 이력은 실패 기록을 참고한다.

## 7. 실제 실행 연결과 검증의 빈틈

```text
restaurant_session.sh → restaurant_service.launch.py → onboard_nav2_core.launch.py
                                                       └ Nav2·AMCL·costmap·collision monitor
box_service --execute --candidate-trial [--search]
  → 지도/설정/위치 준비
  → 관측 위치 transit
  → 정지 pose → RGB-D 앞면 + LiDAR 지지 → 필요하면 Spin 탐색/관측 위치 변경
  → 45cm 중간 면 정렬
  → 새 정지 pose → 앞면 재관측
  → 5cm 최종 접근 → 정지·중앙/모서리 간격·방향 확인
```

- `box_service` 단독은 한 테이블 접근 태스크다. 일반 `restaurant_service.roundtrip()`은 저장 목적지 방문·관측 대기·복귀 경로이며, 위의 새 면 관측 기반 정밀접근 전체와 동일한 함수가 아니다. 한 명령으로 모두 통합 완료됐다고 가정하지 않는다.
- `test_box_service.py::test_transit_alignment_final_sequence`는 `execute`, `plan_pose`, `observe_target`, 정지 pose를 Mock으로 대체한다. 순서 검증이지 실제 인식→Nav2→제어→센서의 통합 검증이 아니다.
- `test_parking_integration.py`도 파일명만 보고 실제 Nav2/실물 통합이라고 부르면 안 된다.
- 수정별 단위 검사와 함께 실제 guard·캡처·관측·재개 함수를 연결한 실패 조건 주입을 해야 한다. 통과시키려고 실패 분기나 실제 비교 함수를 정상 Mock으로 덮지 않는다.

검증 이력:

- cb21ce1 시점 관련 629개 통과 → 이후 캐시/경계 누락이 발견됨.
- caf5ddb 시점 관련 651개 통과, Python/launch/test 6개 flake8 및 PC·파이 빌드 통과.
- 최종 재감사에서 인식·실행·기동·표시 관련 385개 통과, PC↔파이 핵심 소스/설정 15개 SHA-256 일치. 같은 재감사에서 §4의 미수정 두 경로를 재현함.
- 일부 로컬 테스트는 실제 rclpy client/server 동작을 확인했으나 실물 제어가 아니다.
- 최근 독립 code-reviewer는 thread limit으로 생성 실패했다. 독립 승인 완료라고 기록하면 안 된다.
- 실차 5cm 주차/재출발 소요시간/주행 부하 지연 감소/충전소 복귀는 이번 수정본으로 검증하지 않았다.

## 8. `ros-graph-triage` 판단

- 원본: `$HOME/PKM/05_AI_Config/skills/ros-graph-triage/SKILL.md`.
- 설치본: `$HOME/.codex/skills/ros-graph-triage/`, `$HOME/.claude/skills/ros-graph-triage/`.
- 모터 제동 코드가 아니라 ROS 통신·TF·Nav2 실패 계층을 좁히는 읽기 전용 지침이다.
- PKM 원본과 Codex 설치본의 `agents/openai.yaml`은 `allow_implicit_invocation: false`다. 저장소 AGENTS.md에도 출발 자동 호출 금지가 있다. 이는 Codex 설정이며 모든 도구의 자동 호출을 기술적으로 막는 동일한 스위치라고 일반화하지 않는다.
- 최근 두 차례 검증에서는 이 스킬을 진단 워크플로로 실행하지 않고 내용만 검토했다.
- 진단 시간 단축·재발률 개선의 비교 실험은 없다. 현재 원본에는 SKILL.md·참고문서·호출 정책만 있고 효과 평가 세트는 없다.
- **따라서 “진단 지침이니 제 역할을 한다”는 설명만으로 유지 가치를 입증할 수 없다.** 실제 문제 해결 성과에 기여했는지는 미검증이다. 이를 사용했다는 사실을 완료 증거로 쓰지 않는다.
- 주차 코드 검증 누락은 Codex의 검증 책임이다. 스킬 탓으로 대체하지 않는다.
- 사용자는 불필요하면 폐기를 요구했다. 출발 필수 절차로 사용하지 말고, 유지·축소·폐기를 판단하더라도 로봇 실행 코드와 별개로 다룬다. 스킬 수정 시 PKM 원본/설치 동기화 규칙을 따른다.

## 9. 근거 자료와 복구 경로

저장소 상대경로:

- `jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md`: 실제 실패·수정·미해결 기록. §12와 §13을 최종 성공 보고로 읽지 않는다.
- `jdamr_cube_navigation/evaluation/20260929_NAV2_PLATFORM_RESEARCH.md`: Pi 구성/통신/부하 조사와 판단 한계.
- `jdamr_cube_navigation/jdamr_cube_navigation/{restaurant_service,box_service,corridor_route}.py`: 핵심 실행 경로.
- `jdamr_cube_navigation/config/{new_base_nav2_params,box_parking_contract,restaurant_service_contract}.yaml`: 제어 및 판정 설정.
- `jdamr_cube_navigation/behavior_trees/navigate_to_pose_{alignment,parking}.xml`: 정렬·최종 접근 트리.

PC 데이터:

- `$HOME/jdamr_data/parking_attempt_20260929_preserved/`: 최초 주행 bag와 실행 로그.
- `$HOME/jdamr_data/parking_charging_fix_20260929_hwT9Pe/`: 후속 다섯 run JSONL, `nav2_latency_journal.log`, `stationary_sensor_baseline.json`.
- 후속 원시 RGB-D가 모두 보존된 것은 아니다. 없는 입력으로 인식 개선을 실증했다고 쓰지 않는다.

파이 백업:

- `$HOME/jdamr_data/development_flow_20260929_T8eNdh/before.tar.gz`: cb21ce1 배치 전.
- `$HOME/jdamr_data/development_verify_20260929_ENEC2Z/before.tar.gz`: caf5ddb 배치 전 여섯 파일. 같은 디렉터리에 `changed_paths.txt`, `deployed_sha256.txt`, `build.log`.
- `$HOME/jdamr_data/restaurant_service.before_parameter_clients_20260929.py`: parameter client 수정 전.

백업은 복구 선택지이지 지금 복원을 지시하는 것이 아니다. 광범위 reset/rollback이나 데이터 삭제를 하지 않는다.

## 10. Claude에 요청할 다음 작업과 완료 기준

1. 현재 cwd·branch·dirty 변경을 확인하고 §4 두 재현을 먼저 정식 회귀 검사로 고정한다. 정상 Mock에 가려지지 않도록 실제 함수 조합으로 확인한다.
2. 기존 실행기에서 관측·계획·캡처·정지 확인의 일시적 입력 공백을 어떻게 처리할지 일관되게 보완한다. 관측 결과가 낡으면 새 pose/새 관측을 취한다. 새 승인 계층이나 별도 실행 프레임워크를 만들지 않는다.
3. 박스 면 후보 안정화와 실제 부하 장애를 코드 수정과 분리해 추적한다. 확보된 데이터로 답할 수 없는 원인은 미확정으로 남기고 필요한 최소 계측을 기존 기록 경로에 연결한다.
4. 변경한 함수뿐 아니라 실제 사용할 launch·BT·파이 설치 파일 연결을 확인한다. 준비 단계에서 끝내고 출발 때 다시 빌드·리뷰를 시작하지 않는다.
5. 파이 반영 시 실행 중인 주행기를 확인하고 변경 파일만 백업·배치한다. ROS setup 전에 `set -u`를 켜서 다시 실패하지 않도록 기존 환경 로딩 방법을 따른다. 광범위 배포 스크립트로 베이스 서비스를 불필요하게 재시작하지 않는다.
6. 사용자가 주행을 요청하면 한 실행의 위치·명령·센서·오류 로그를 보존하면서 정밀주차까지 이어간다. 중간 완료/전체 완료를 구분한다. 실패를 새 시작점 초기화로 덮지 않는다.

완료 보고는 세 가지로 분리한다:

- 소프트웨어: 어떤 실패를 재현했고 어떻게 수정했는가.
- 배치/런타임: 실제 파이에 어떤 파일이 설치·로드됐고 어떤 구성에서 확인했는가.
- 실차: 관측 위치, 면 정렬, 5cm 정지, 대기, 충전소 복귀 중 어디까지 실제 완료했는가. 외부 거리 실측과 내부 추정도 구분한다.

**기존 테스트 통과, 문서 작성, 스킬 사용, 프로세스 active만으로 “이번에는 문제없다” 또는 “출발 준비 완료”라고 보고하지 않는다.**
