# G005 프런티어 정책 평가 준비 상태 — 2026-09-07

## 결론

G005는 실제 Gazebo·Nav2를 사용하는 15회 평가를 실행할 수 있는 코드 경로와 실패 차단
조건까지 구현됐다. 실제 Nav2 목표 수락과 바퀴 명령 발생을 확인한 짧은 수직 스모크도
통과했다. 다만 `current`·`nearest`·`gain-nav` 세 정책을 5개 고정 layout에서 비교하는
full15는 실행하지 않았다. 따라서 현재 상태는 **평가 실행 준비 완료**이며, 정책 우승이나
production 변경 완료가 아니다.

관련 구현은 commit `5c84289` (`feat(frontier): G005 실제 평가 실행 경로를 완성함`)에
있다.

## 검증된 범위

| 항목 | 확인 결과 |
|---|---|
| 실행 계획 | 정책 3개 × layout seed 5개 = 정확히 15회로 고정 |
| 실행 대상 | mock adapter가 아닌 실제 Gazebo와 Nav2 action 사용 |
| 수직 스모크 | Nav2가 목표를 수락하고 `/cmd_vel` 비영(0이 아닌) 명령 `[0.0, -0.075]` 발생 |
| 센서 관측 | 10 Hz LiDAR, 관측 메시지 32/32 수락, 거부 0 |
| 런타임 판정 | 스모크 관측 구간에 `runtime invalid` 없음 |
| 종료 격리 | 시험이 만든 프로세스 그룹의 잔존 프로세스 0 |
| 집중 회귀 | G005 관련 테스트 `138 passed` |
| 패키지 회귀 | `1094 passed, 1 skipped` |
| 빌드·정적 검사 | navigation 패키지 build, 변경 Python flake8, `git diff --check` 통과 |
| 분리 리뷰 | code-reviewer와 architect가 아래의 제한된 준비 완료 범위를 승인 |

수직 스모크는 첫 실제 이동을 확인한 직후 의도적으로 종료했다. 그러므로 종료 과정의
action cancel·deactivate 로그는 강제 종료에 따른 진단 로그이며, 900초 run의 최종 성과
증거로 쓰지 않는다.

## 실행 구조

```text
고정 asset·15회 plan
        │
        ▼
Gazebo + 평가 전용 Nav2 launch
        │
        ├─ observer: 지도·LiDAR·접촉·명령·lifecycle 관측
        ├─ coordinator: 후보 계산·Nav2 action·TF 권한 조정
        └─ runner: 프로세스 그룹·자원·종료·산출물 봉인
        │
        ▼
evaluator: 15/15 유효성 확인 후에만 정책 비교·승격 판정
```

GT(시뮬레이터가 제공하는 정답 위치)의 `map -> odom`은 전용 토픽으로 받은 뒤
coordinator가 허용된 transform과 발행자를 검증해 `/tf`로 다시 발행한다. 이 구조는
Gazebo 정답 변환과 다른 노드의 변환이 섞인 상태를 정상 실행으로 오인하지 않게 한다.

## 구현된 실패 차단 조건

1. asset, production 입력, 실행 코드와 15회 순서를 SHA-256으로 고정한다.
2. `ComputePathToPose`와 `NavigateToPose`의 실제 Nav2 action만 허용한다. 외부 runtime
   adapter는 받지 않는다.
3. 지도 sequence와 지도 payload hash를 함께 묶고, 실행 중 지도가 바뀌면 무효 처리한다.
4. planner의 실제 path frame을 보존하며 `map`이 아닌 경로는 실패로 닫는다.
5. 점이 아닌 로봇 footprint polygon 전체를 occupancy cell에 투영해 clearance를 계산한다.
6. recovery 횟수는 목표 전체에서 합산하며, 늦게 도착한 action callback이 종료 판정을
   바꾸지 못하게 한다.
7. launch와 coordinator가 만든 전체 프로세스 그룹의 CPU·RSS를 측정하고 sampler 코드도
   provenance hash에 포함한다.
8. `/cmd_vel` 권한, Nav2 lifecycle ACTIVE 이탈, lifecycle service 연속 누락, TF 발행자·
   payload 불일치를 실행 전 구간에서 감시한다. 한 번 발생한 위반은 이후 정상으로 돌아와도
   유효 실행으로 복구하지 않는다.
9. 같은 map·시작 위치·후보 집합에서 세 번 연속 모든 후보가 도달 불가일 때만 정상적인
   탐사 종료로 판정한다.
10. 평가 전후 production 입력 hash가 같아야 하며, 평가가 운영 설정을 수정하지 않는다.

## 용어

- **프런티어(frontier)**: 이미 확인한 빈 공간과 아직 관측하지 않은 공간의 경계다. 탐사
  정책은 이 경계 중 다음 목표를 고른다.
- **vertical smoke**: asset 생성부터 실제 Nav2 목표 수락과 이동 명령까지 전체 연결이
  한 번 관통하는지 보는 짧은 통합 확인이다. 성능 비교 시험은 아니다.
- **full15**: `current`·`nearest`·`gain-nav` 정책 3개를 고정 seed 5개에서 실행하는 총
  15회의 본평가다.
- **fail-closed**: 권한·지도·센서·lifecycle처럼 결과 신뢰 조건이 깨지면 해당 run을
  성공으로 추정하지 않고 `INVALID` 또는 실패로 판정하는 방식이다.
- **promotion gate**: 모든 run이 유효할 때만 성능을 비교하고, 사전 등록한 조건을 만족한
  정책만 다음 실기체 후보로 넘기는 절차다.

## 아직 완료하지 않은 범위

- full15 본평가와 최종 artifact는 아직 없다.
- 정책별 coverage, first-85 path, CPU·RSS, clearance 우열을 아직 비교하지 않았다.
- `gain-nav`를 production 또는 실기체 후보로 승격하지 않았다.
- 이번 결과는 simulation 실행 준비 증거다. 실차 탐사, SLAM 지도 생성 결과, 사람 안전,
  안전 인증을 증명하지 않는다.

## 다음 실행 순서

full15의 run별 최대 simulation horizon 합계는 `15 × 900초 = 225분`이다. terminal
조건에 일찍 도달하면 짧아질 수 있고 각 run의 기동·정리 시간은 별도로 추가된다. 반복
횟수를 임의로 늘리지 않고 아래 한 번의 고정 matrix만 실행한다.

```bash
cd $HOME/jdamr_cube_ws/src/jdamr_cube_ros
source /opt/ros/jazzy/setup.bash
source $HOME/jdamr_cube_ws/install/setup.bash

G005_ASSETS="$HOME/jdamr_artifacts/g005_frontier_assets_20260907_full"
G005_PLAN="$HOME/jdamr_artifacts/g005_frontier_plan_20260907.json"
G005_RUNS="$HOME/jdamr_artifacts/g005_frontier_full15_20260907"

python3 jdamr_cube_navigation/evaluation/generate_frontier_policy_assets.py \
  --output-root "$G005_ASSETS" --mode full

python3 jdamr_cube_navigation/evaluation/run_frontier_policy_full.py \
  --asset-root "$G005_ASSETS" --output-root "$G005_PLAN" \
  --base-domain-id 100 --timeout-s 1200 --dry-run

python3 jdamr_cube_navigation/evaluation/run_frontier_policy_full.py \
  --asset-root "$G005_ASSETS" --output-root "$G005_RUNS" \
  --base-domain-id 100 --timeout-s 1200

python3 jdamr_cube_navigation/evaluation/evaluate_frontier_policy.py \
  --root "$G005_RUNS" --mode full
```

세 출력 경로는 실행 전에 존재하지 않아야 한다. runner는 일부 정책·seed만 골라 실행하는
명령을 제공하지 않는다. full15 완료 뒤 `evaluate_frontier_policy.py`의 유효성·승격
판정을 통과한 경우에만 G010 held-out 평가 또는 실기체 후보 단계로 넘어간다.
