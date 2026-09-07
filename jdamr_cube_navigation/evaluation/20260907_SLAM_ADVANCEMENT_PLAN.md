# SLAM 고도화 실행 계획

작성일: 2026-09-07

이 문서는 `/home/lim/Downloads/SLAM_고도화_검토_2026-09-07.md`의 제안을
현재 코드, OMX 목표, 완료 산출물과 대조한 결과다. 이미 구현된 평가 기반은 반복하지 않고,
새로 확인된 정확성 보완과 알고리즘 개발 단계만 반영한다.

## 현재 판단

현재 방향은 유효하다. G001·G003·G004·G008로 센서 변동, 지도 생성, 고정·돌발
장애물 대응과 실차 bag 기반 설정 비교를 이미 확보했다. 다음 깊이는 시험 수를 더 늘리는
것이 아니라 G002에서 재지역화 실패를 분류하고, 관측된 원인 하나를 직접 개선하는 데서
만든다.

## 완료된 G002 Axis A

- 입력: 동일 실차 완주 bag
- 비교: P0·P1·P2 × AMCL seed 5개, 총 15회
- 결과: 15/15 실행 유효성 통과
- 산출물: `/home/lim/jdamr_artifacts/amcl_axis_a_20260907_v06_full15`
- manifest SHA-256:
  `84e40fba68a716c9191cd7c5d1c5847ee2e96ce474d554bc7b4f80acef305e73`
- tree SHA-256:
  `846bddfed4ba5f2199ffb1101fbaccada3d80414a99046ef645cc64ec49d3221`
- P2/P0 CPU 비율: 중앙값 1.0860, 개별 최대 1.1543
- P2/P0 p95 지연 비율: 중앙값 1.0065, 개별 최대 1.0716
- 15/15 `map -> odom` 권한: `STRICT_ENDPOINT_PROCESS_BINDING`

P2는 seed 23·89에서 CPU 개별 상한 1.10을 넘었다. Axis A 기준 후보는 P0 유지다.
이 결과는 위치 정확도 우열이 아니라 동일 관측을 처리한 추정기의 자원·지연·일관성
비교다. 실차 GT가 없으므로 ATE/RPE와 절대 정확도는 주장하지 않는다.

## Axis B가 증명할 범위

Axis B는 동일한 Gazebo 입력 bag을 P0·P1·P2와 AMCL seed 5개에 반복 재생하는
고정 입력 추정기 비교다.

- `evaluation_design`: `FIXED_INPUT_ESTIMATOR_REPLAY`
- `seed_semantics`: `ESTIMATOR_STOCHASTICITY_ONLY`
- `input_diversity_across_estimator_seeds`: `false`
- `gt_usage`: `EVALUATION_ONLY_NOT_ESTIMATOR_INPUT`
- `closed_loop_recovery_behavior_evaluated`: `false`
- `observation_schedule_source`: `PRERECORDED_FORCED_ROTATION`

따라서 Axis B는 회전 관측을 제공했을 때 AMCL이 얼마나 잘 복구하는지 비교한다.
로봇이 스스로 회전 행동을 선택하는 능력이나 폐루프 복구 정책은 증명하지 않는다.

### kidnapped 시나리오

1. Gazebo base와 GT를 `(-8, 0, 0)`에서 `(0, 0, pi)`로 순간 이동한다.
2. wheel odometry는 8m 이동을 정상 주행처럼 누적하지 않아야 한다.
3. 순간 이동 뒤 정답 `/initialpose`를 다시 주입하지 않는다.
4. 평가기만 GT를 읽는다.
5. `2pi`는 yaw 숫자에 값을 더하는 연산이 아니라 `/cmd_vel`로 수행한 실제 회전이다.
6. 회전 구간의 scan 수, timestamp 단조성, 최대 gap을 10Hz LiDAR 계약으로 검증한다.
7. teleport 전·후 GT sample을 연결해 보간하지 않는다. 불연속 경계를 가르는 bracket은
   `INVALID`다.

### 고정 입력과 폐루프의 분리

Axis B 45회는 후보별 입력을 동일하게 유지해 AMCL profile의 효과를 분리한다. 이 결과로
후보를 정한 뒤, 선택된 후보만 별도의 폐루프 복구 시험과 실기체 통합 검증에 사용한다.

## 완료된 G002 Axis B

- 입력: Gazebo에서 한 번씩 수집한 정상 초기화·0.5m/15도 초기 오프셋·8m 납치 bag
- 입력 산출물: `/home/lim/jdamr_artifacts/amcl_axis_b_inputs_20260907_v13`
- 비교: 시나리오 3개 × P0·P1·P2 × AMCL seed 5개, 총 45회
- 결과 산출물: `/home/lim/jdamr_artifacts/amcl_axis_b_20260907_v02_full45`
- manifest SHA-256:
  `1626b78a68a186ced212fa8fb2d517f9838c0752d66807b666c4d378fcd84121`
- tree SHA-256:
  `6257d68a6fa3a6d70c137e8125e3d7c12563057f25df845439d46accb903cc36`
- 실행 유효성: 45/45 PASS
- 정상 초기화: 세 프로필 모두 5/5 복구, 복구 판정 30 scan
- 초기 오프셋: 세 프로필 모두 5/5 복구
- kidnapped: 세 프로필 모두 0/5 복구
- false convergence: 45회 전체 0건
- 접촉 센서 발행 경로 확인·접촉 메시지 0건·최종 속도 0·생존 프로세스 0: 45/45
- `map -> odom` 권한: 입력 bag의 해당 변환 0건과 `/amcl`·`/rosbag2_player`
  `/tf` endpoint를 결합한 `STRICT_ENDPOINT_PROCESS_BINDING`, 45/45
- 최종 판정: `selected_profile=P0`, `promote_p2=false`

`map -> odom` 권한은 지도 좌표와 odometry 좌표를 연결하는 변환을 누가 만드는지에 대한
계약이다. 입력 bag에서 이 변환을 제거하고, 격리된 ROS domain의 `/tf` 발행 endpoint를
AMCL과 bag player 두 프로세스에 묶었다. PRELUDE와 FINAL 스냅샷에서 endpoint 수·FQN·
GID가 정확히 일치하지 않으면 실행을 무효로 처리한다. 이는 실행 경계의 단일 권한
증거이며, Jazzy `rclpy`가 제공하지 않는 메시지별 publisher GID나 두 스냅샷 사이의 연속
graph 감사를 주장하지 않는다.

`contact0`은 충돌이 없었다는 추정이 아니다. Gazebo Contact sensor를 별도
`ros_gz_bridge`로 연결하고 발행자 1개를 확인한 뒤, 기록된 contact 메시지가 0개인지
검증한 결과다. 평가 runner·evaluator·observer와 Axis B 입력 generator·driver 소스는
각 artifact의 `harness_sources/`에 복사했다. observer·generator·driver 실행 기록은 이
스냅샷과 직접 결합해 현재 worktree가 바뀌어도 당시 바이트를 재검증한다.

## G002 이후 알고리즘 개발

Axis B 결과를 먼저 동결한 뒤 실패와 outlier를 다음 taxonomy로 분류한다.

| 유형 | 확인할 증거 | 가능한 단일 변경 |
|---|---|---|
| 정답 주변 입자 부재 | 납치 뒤 정답 지지 입자 비율 | recovery particle 주입 또는 전역 재초기화 |
| 정답 후보 소멸 | 관측 갱신·재표본화 전후 후보 질량 | 후보 유지 또는 재표본화 조건 |
| 오위치 확신 수렴 | 작은 공분산과 큰 GT 오차의 동시 지속 | false-confidence 검출과 정지·복구 |
| 판정 기준 과보수 | GT 복구 뒤에도 실패 판정 지속 | 복구 완료 판정 |
| 처리 지연 | particle 수·callback·CPU와 복구 시간 | 계산량 또는 update 정책 |

GT는 분류와 평가에만 사용하고 검출·복구 코드에는 넣지 않는다. 관측된 원인 중 하나만
선택한다. 알고리즘 변경은 G002에서 정의한 false convergence가 최소 1개 run에서
3 sample 연속 관측됐을 때만 시작한다. false convergence는 translation GT error가
0.15m를 넘거나 yaw error가 0.25rad를 넘는데도 3-sigma covariance가 두 임계 안인
상태다. 관측되지 않으면 `NO_FALSE_CONFIDENCE_OBSERVED_NO_CHANGE_JUSTIFIED`로
종료하고 P0를 유지한다.

관측됐다면 구현 전에 `failure_selection.json`에 source·input·metric hash와 함께 다음을
봉인한다.

- 검출 임계와 최소 지속 3 sample
- particle cluster 연결 기준과 map-LiDAR scan agreement 산식
- GT를 입력으로 쓰지 않는 단일 recovery action
- 개발 입력: 기존 kidnapped bag, seed 11·23·42·67·89
- 비교군: P0·covariance-only guard·one-change guard+recovery, 총 15 replay

P0는 별도 gate가 없으므로 false-normal 비교에서 항상 `VALID`인 reference로 기록한다.
covariance-only guard는 3-sigma translation이 0.15m를 넘거나 3-sigma yaw가 0.25rad를
넘는 상태가 3 sample 연속이면 `INVALID`, 아니면 `VALID`이다. one-change guard는 봉인한
GT-free 결합식이 3 sample 연속 참이면 `INVALID`, 아니면 `VALID`이다. 두 guard arm에는
정확히 같은 recovery action과 parameter를 적용해 detector만 다르게 유지한다.

held-out은 입력 생성 전에 반대 방향 teleport를 봉인한다. true start `(8, 0, pi)`에서
center `(0, 0, 0)`로 이동하는 correct-init·kidnapped 두 입력과 seed
101·103·107·109·113을 사용해 3 비교군 × 2 case × 5 seed, 총 30 replay를 수행한다.
승격 조건은 kidnapped false-normal 0이면서 P0·covariance-only 각각보다 엄격히 적음,
correct-init false alarm 0이면서 covariance-only보다 많지 않음, recovery scan이 두 비교군
각각에 대해 5쌍 중 4쌍 이상 non-worse 및 중앙값 strict improvement, 재개 뒤 정상
10 sample 연속, P0 대비 CPU·RSS 비율 각각 1.10 이하, `INVALID` artifact·비완주 0이다.
통과하면
`FALSE_CONFIDENCE_ONE_CHANGE_EVAL_CANDIDATE`, 아니면 `RETAIN_P0`다.

후보가 통과한 경우에만 mirrored kidnapped 입력과 seed 101·103·107에서 P0·후보를 비교하는
simulation-only 폐루프 6회를 수행한다. 위치 추정 이상 판정 뒤 정지, 봉인된 recovery action,
3 sample 연속 수렴과 같은 navigation goal 재개를 검증한다. 6/6에는 input·runtime·artifact
유효, contact 0, final zero, runner cancellation 0, survivor 0을 적용한다. 후보 3/3에는
`INVALID` 뒤 0.42초 안에 final zero, recovery action이 30초 또는 newer scan 300개 중 먼저
도달한 제한 안에 성공, 이후 3 sample 연속 수렴, goal 재개 뒤 180초 안에 동일 goal
`SUCCEEDED`를 추가로 요구한다. P0의 미검출·미복구는 비교 결과로 남기되 artifact
`INVALID`로 만들지 않는다. 후보 3/3 통과 전에는 후보를 통합하지 않는다. G006은 이
결과와 artifact hash를 기록하되 production에는 stock P0만 사용한다.

이 작업은 OMX `G009-amcl`로 추가했다.

### G009 완료 결과 — 2026-09-07

`/home/lim/jdamr_artifacts/amcl_axis_b_20260907_v02_full45`를 코드·임계 변경 없이 다시
검증하고 `/home/lim/jdamr_artifacts/amcl_failure_taxonomy_20260907_v01`에 결과를
동결했다. 45개 run 전체, 비복구 15개, false convergence 0개, 기존 G002의 1.10 자원
상한을 넘은 P2/P0 CPU 비율 2개를 빠짐없이 기록했다.

비복구 15개는 모두 kidnapped(초기 위치 정보 없이 로봇 위치를 순간 이동시킨 상황)이며
profile별로 P0·P1·P2 각 5개다. 15개 모두 false-confidence(실제 위치는 틀렸는데 추정
공분산만 낮은 상태) 연속 길이가 0이고, post-t0에서 기존 복구 기준 안에 들어온 연속
길이도 0이다. translation 오차/0.15m와 yaw 오차/0.25rad 중 큰 값인 최소 정규화 오차도
47.8274~53.5150으로 임계 경계와 멀었다. 따라서 오위치 확신 수렴은 배제했고 판정 기준
과보수는 이번 frozen 데이터에서 원인으로 뒷받침되지 않았다. 반면 기존 cloud evidence에는
입자별 좌표·가중치와
재표본화 전후 후보 질량이 없어 정답 주변 입자 부재와 정답 후보 소멸을 구분할 수 없다.
CPU 비율 1.1129(seed 23), 1.1543(seed 89)은 자원 비용 outlier지만 처리 지연이
비복구의 원인이라는 인과 증거는 아니다.

알고리즘 변경 gate는 열리지 않았다. `failure_selection.json`, development 15회,
held-out 30회, 폐루프 6회는 생성하지 않았고 결정은
`NO_FALSE_CONFIDENCE_OBSERVED_NO_CHANGE_JUSTIFIED`다. production Nav2/AMCL은 stock
P0 그대로다. G009 manifest SHA-256은
`3b373c56b66b537d938e05d4fe9e1f0bdf6647d1053c4d2d17309e2b7df096ce`, tree SHA-256은
`db4e9970c3b4148f0962de486b4d2f2c262f8af973ca99b2eed06e5369571b07`다.

## Frontier 정책의 추가 검증

G005의 current·nearest·gain-nav 15회 비교를 먼저 완료한다. G005가
`RETAIN_CURRENT`이면 추가 motion run 없이 `NOT_RUN_G005_RETAINED`와 G005 artifact
hash만 기록한다. `GAIN_NAV_EVAL_CANDIDATE`일 때만 같은 5개 입력·seed로 gain-only를
정확히 5회 추가하고, 기존 gain-nav·nearest(nav-only) 결과와 비교한다.

held-out 계약은 실행 전에 다음처럼 고정한다.

- topology family: `t_junction`, `asymmetric_rooms`
- seed: 101·103·107
- 정책: gain-nav·gain-only·nav-only
- 실행 수: 2 topology × 3 seed × 3 정책 = 18회
- 종료: G005와 같은 terminal rule 또는 simulation time 900초
- 공통 gate: coverage 85% 이상, contact 0, final zero, authority·lifecycle 통과,
  survivor 0

18회 모두 G005의 candidate universe·canonical sampler·frozen planner·normalization·tie-break·
decision token을 그대로 사용한다. input/hash/initial parity, frozen costmap batch 전후 동일,
required finite metrics, runner cancellation 0, `INVALID` 0도 hard gate다. 18/18 유효성
gate를 통과한 뒤에만 성능을 비교한다.

gain-nav는 두 대조군 각각에 대해 first-85 path가 6쌍 중 5쌍 이상 non-worse이고 중앙값
비율 0.95 이하, t50·t70·t85·CPU·RSS 중앙값 비율 각각 1.05 이하, AUC 비율 0.95 이상,
clearance 비율 0.95 이상, unreachable·failure regression 0이어야 한다. 모두 통과하면
`GAIN_NAV_HELD_OUT_CANDIDATE`, 아니면 `RETAIN_CURRENT`다. G006은 raw G005가 아니라
이 decision과 artifact를 소비한다. G006의 통합 held-out은 정책 요소의 기여 분리를
대신하지 않는다.

이 작업은 OMX `G010-frontier-held-out`으로 추가했다.

## 실행 순서

현재 1~5단계는 완료됐다. G005는 실제 Gazebo·Nav2 실행 경로와 수직 스모크까지
완료했지만, 6단계 full15 본평가는 아직 실행하지 않았다. 구현 검증과 주장 범위는
`20260907_G005_VERTICAL_READINESS.md`에 기록했다.

1. G002 Axis B의 GT 불연속·관측 schedule·claim 경계를 보완한다.
2. 정상 초기화·초기 오프셋·kidnapped 입력을 Gazebo에서 한 번씩 생성한다.
3. P0·P1·P2 × 5 AMCL seed × 3시나리오, 총 45회를 실행한다.
4. G002 결과와 P0 유지 또는 후보 판정을 동결한다.
5. `G009-amcl`에서 실패 taxonomy를 동결한다. false convergence가 실제 관측된 경우에만
   두 guard에 같은 recovery action을 쓰는 개발 15회·held-out 30회를 수행하고, 후보가
   통과한 경우에만 숫자 제한이 있는 폐루프 6회를 수행한다.
6. G005 full15를 실행한다.
7. `G010-frontier-held-out`에서 G005 후보가 있을 때만 gain-only 5회와 held-out 18회를
   수행한다.
8. G006은 G009 결과를 연구 증거로 기록하되 stock P0를 유지하고, G010 최종 decision으로
   frontier 정책을 선택해 candidate bundle을 동결한다.
9. 마지막으로 기존 복도·지도·Keepout mask에서 감독 실기체 시험을 수행한다.

실기체 시험은 평상시 주행, 고정 장애물 우회, 돌발 장애물 정지·동일 goal 재개,
Wi-Fi 단절 중 온보드 기록 보존을 포함한다. Depth camera는 이 LiDAR 기반 단계의 필수
조건이 아니다.

## 제외하는 주장

- 실차 절대 위치 정확도 또는 안전 인증
- 자동 복구 행동 선택을 Axis B가 증명했다는 주장
- AMCL seed 5개를 서로 다른 실차 환경 5개로 해석하는 주장
- G008에서 기존 궤적과 다르다는 이유만으로 정확도가 나빠졌다는 주장
- 평가용 patched AMCL을 production 후보로 직접 배포하는 주장
