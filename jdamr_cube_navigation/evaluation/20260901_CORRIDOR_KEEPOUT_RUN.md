# 2026-09-01 저장지도 Keepout 복도 왕복 실차 진단

## 결론

금지구역 지도와 80m 왕복 경로는 재사용 가능하고, 저장지도 기반 자율주행도 반환점과
남은 23.573m 부분 복귀까지 확인했다. 그러나 시작부터 복귀까지 끊기지 않은 프로토콜
적격 데이터 수집에는 성공하지 못했다. 최초 Wi-Fi 병목을 제거한 뒤에도 합성 Nav2 내부
노드 소실, 재시작 시 AMCL 원점 초기화, 복도 종방향 공분산의 과민 차단, 독립 프로세스
전환 후 20ms BT action 응답 제한이 차례로 드러났다.

현재 수정본은 필수 Nav2 노드를 독립 프로세스로 격리하고, 구간당 한 번만 계획하며,
action 응답 제한을 1,000ms로 늘렸다. AMCL은 x/y 공분산을 분리하고 중간 재개 위치가
첫 잔여 waypoint에서 6m보다 멀면 출발을 차단한다. 코드와 회귀 테스트는 통과했으며,
실제 recorder 부하를 포함한 비주행 정적 소크 뒤 80m 전체 왕복을 처음부터 다시 수집한다.

재사용 가능한 결과는 다음과 같다.

- 저장 지도와 두 계단 가지의 Keepout 마스크: 재사용 가능
- Nav2 전체 왕복 사전 계획: 재사용 가능
- 출발부터 복귀 중단 지점까지 MCAP: 원인 분석·오프라인 실험용으로 사용 가능
- MCAP을 프로토콜 적격 최종 데이터나 완전 왕복 성공 증거로 사용하는 것: 불가

## 지금 수행한 작업과 사용 가능 범위

이번 주행은 새 지도를 실시간으로 만드는 온라인 SLAM이 아니다. 이미 만든 점유 격자와
AMCL로 위치를 잡고, Keepout을 적용한 경로를 따라가는 **저장지도 기반 자율주행**이다.
동시에 원시 센서를 MCAP으로 남겼으므로, 별도 격리 domain에서 Cartographer나 SLAM
Toolbox를 새로 실행하는 **오프라인 2D LiDAR SLAM 입력**으로는 다시 쓸 수 있다.

| 산출물 | 사용 가능 여부 | 가능한 사용 | 제한 |
|---|---|---|---|
| 저장 지도 | 가능 | AMCL 위치추정·Nav2 경로 계획 | 현재 복도 상태가 바뀌면 재검증 필요 |
| Keepout 마스크 | 가능 | 두 계단 입구 경로 차단 | 실차 접근 경계 재확인 필요 |
| 20-point 80.047m 경로 | 가능 | 다음 왕복의 동일 조건 재실행 | 완주는 아직 증명되지 않음 |
| 현재 MCAP | 조건부 가능 | 센서 노이즈·주기·TF 지연 분석, 오프라인 2D SLAM | 136건 손실과 미완주로 최종 성능 비교에는 부적격 |
| Visual SLAM | 현재 데이터로 불가 | 해당 없음 | 이미지·camera_info가 기록되지 않음 |
| 새 온보드 실행 구조 | 독립 프로세스 수정·전체 경로 계획·회귀 테스트 완료 | Wi-Fi와 무관한 제어·로컬 기록 | 수정본 80m급 왕복 완주가 남음 |

저장 지도와 Keepout은 오프라인 SLAM에 주입하지 않는다. 재생 시 저장 `/map`과 이동
명령을 제외하고, 기록된 AMCL의 `map -> odom`도 제거해 새 mapping backend 하나만 TF
권한자가 되게 한다. 따라서 기존 지도를 따라 수집했다고 새 지도 결과가 기존 지도의
복사본이 되지는 않는다.

### 파이에 남기는 범위

| 실행 위치 | 주행 중 실행 | 이유 |
|---|---|---|
| 파이 | 하드웨어 bringup | 센서·모터의 로컬 연결 |
| 파이 | 독립 프로세스 Nav2·AMCL·Collision Monitor·Keepout | Wi-Fi가 끊겨도 제어와 정지가 동작해야 함 |
| 파이 | `corridor_route` | 목표 전송과 센서·배터리·위치추정 게이트 |
| 파이 | 저우선순위 필수 MCAP recorder | 고속 원본을 Wi-Fi로 보내지 않고 보존 |
| 노트북 | 저대역폭 view-only RViz | 지도·마스크·위치·계획 확인만 수행 |
| 주행 후 노트북 | 2D SLAM 재생·그래프·통계, 카메라를 별도 수집한 경우 Visual SLAM | 주행 중 파이 CPU·디스크와 경쟁하지 않음 |

첫 온보드 정적 시험에서 공식 Nav2 전체 구성은 load1이 9.33~11.27까지 올라 4.0
게이트를 넘었다. 기능은 map server, AMCL, controller, planner, velocity smoother,
Collision Monitor, BT navigator로 줄였다. 이 최소 구성을 한 합성 컨테이너로 실행하자
실차 약 4~9분 사이에 컨테이너 프로세스는 살아 있지만 내부 노드가 DDS graph에서 함께
사라졌다. 현재는 같은 최소 기능을 독립 프로세스로 실행하고 각 필수 프로세스의 종료를
launch가 감지한다.

## 문제와 해결 상태

| 문제 | 확인된 원인 | 반영한 해결 | 현재 상태 |
|---|---|---|---|
| 계단 가지 진입 위험 | 원본 점유 지도만으로는 가지 입구를 막지 않음 | 두 다각형+0.55m 팽창 Keepout, 해시·연결성 고정 | 마스크 생성 검증 완료, 다음 실차 확인 필요 |
| 출발 전 `Empty Tree` | 저장지도 launch의 기본 NavigateToPose BT 경로가 비어 있었음 | launch에서 검증된 BT 절대 경로 주입 | 코드·회귀 테스트 완료 |
| 복귀 중 TF·센서 지연 | 약한 Wi-Fi에서 노트북 Nav2·RViz·rosbag의 고속 구독 fan-out | 파이의 Nav2+저우선순위 필수 recorder, 노트북 view-only RViz | 온보드 전환 완료 |
| 복구 회전으로 상태가 불명확 | 기존 BT가 회전·대기 복구를 최대 3회 수행 | 왕복 실행기는 회전·후진·재시도 없는 fail-fast BT 사용 | 코드·회귀 테스트 완료 |
| 지도 재생 시 TF 충돌 위험 | AMCL과 mapping backend가 모두 `map -> odom`을 발행할 수 있음 | 격리 domain에서 `/map`, 이동 명령, 기록 AMCL TF를 제거 | 오프라인 재생 가드 구현됨 |
| 파이 load gate 초과 | Jazzy 기본 bringup이 복도에서 쓰지 않는 서버까지 10개 실행 | 복도에 필요한 서버만 유지 | 최소 합성 구성 1.17~1.93, 독립 프로세스 구성 4.13~9.85로 재개선 필요 |
| launch 생성 실패 뒤 recorder 잔류 | 포함 launch보다 recorder가 먼저 시작됨 | navigation 생성 뒤 recorder를 시작하고 process-group 정리 | 재발 방지 테스트·실기 확인 |
| planning-only 준비 실패 | AMCL은 마지막 pose를 `TRANSIENT_LOCAL`로 보관하지만 실행기는 기본 `VOLATILE`로 늦게 구독 | AMCL pose 구독을 reliable+`TRANSIENT_LOCAL`로 일치 | 3,098 poses·78.600m 계획 통과 |
| 반복 중 합성 노드 소실 | 컨테이너는 살아 있으나 AMCL·planner·navigator가 DDS graph에서 함께 사라짐 | 필수 Nav2 노드를 독립 프로세스로 격리하고 exit handler 등록 | 독립 프로세스에서 노드 생존 확인, 장시간 소크 재검증 |
| 재시작 뒤 잘못된 잔여 경로 | AMCL이 원점으로 초기화됐는데 `--start-index`만 유지 | AMCL과 첫 잔여 waypoint 거리 6m 초과 시 출발 차단 | 회귀 테스트 완료 |
| 복귀 중 과민 정지 | 복도 진행축 x 공분산 0.519와 15초 AMCL freshness가 고정 0.5/15초 게이트를 넘음 | x 2.0·y 1.0 분리, AMCL freshness 60초, 정확한 차단 원인 로그 | 회귀 테스트 완료 |
| 독립 프로세스 첫 목표 중단 | 1Hz 연속 재계획과 20ms action 응답 제한이 Pi DDS 지연을 허용하지 못함 | 구간당 한 번 계획, action timeout 1,000ms, service wait 5,000ms | 회귀 테스트 완료, 정적 소크 후 실차 재검증 |

## 실행 목표와 방식

- 저장 지도: `/home/lim/maps/autonomous_20260826T161908.yaml`
- Keepout 마스크: `/home/lim/maps/autonomous_20260826T161908_keepout_multi.yaml`
- 위치추정: AMCL, 시작 위치 `(0, 0, 0)` 초기화
- 계획기: NavFn `GridBased`, unknown-space 진입 금지
- 제어기: Regulated Pure Pursuit, 선속도 상한 0.18m/s, 후진 금지
- 안전 경계: KeepoutFilter + Collision Monitor + 작업자 물리 전원 차단
- 경로: 위쪽 라인으로 출발, `(37.5, -4.25)`에서 전환, 아래쪽 라인으로 복귀
- 경로 사전 계획: 20 waypoint, 3,147 pose, 80.047m, 오류 코드 0

## 실제 진행 결과

1. 두 Keepout 다각형과 0.55m 팽창 마스크를 표시해 사용자가 계단 벽선 정렬을 확인했다.
2. AMCL 초기 위치를 `(0, 0, 0)`으로 설정하고 배터리 12.0V대, `/cmd_vel` 단일 발행자,
   주요 lifecycle 노드 ACTIVE를 확인했다.
3. 첫 `FollowWaypoints`는 저장지도 launch가 빈
   `default_nav_to_pose_bt_xml`을 전달해 `BehaviorTreeEngine: Empty Tree`로 출발 전에
   중단됐다. 로봇은 움직이지 않았다.
4. 각 `NavigateToPose` 목표에 검증된 forward-only BT를 명시해 재시작했다.
5. 출발 방향 waypoint 2~10과 반환점 `(37.5, -4.25)`에 도착했고, 복귀 waypoint
   11~12도 성공했다.
6. 복귀 waypoint 13 진행 중 Wi-Fi 지연으로 `map/odom` TF가 1~2초 이상 늦어져
   controller가 오류 코드 102로 목표를 중단했다. 당시 마지막 신뢰 가능한 AMCL 위치는
   약 `(26.76, -4.33)`이었다.
7. RViz를 끄고 이어서 원격 rosbag을 종료하자 `/odom`과 `/battery_state`가 즉시 다시
   수신됐다. 이는 원격 시각화·기록 fan-out이 무선 병목을 악화했다는 직접 증거다.
8. 녹화 없이 제한적으로 복귀를 재개했지만 무선 지연과 AMCL 흔들림이 반복됐고,
   남은 거리 추정이 4.1m에서 6.6m로 증가하며 복구 동작이 4회 발생해 목표와 Nav2
   전체를 종료했다.
9. 종료 후 `/cmd_vel` 발행자 0개와 odom 선속도·각속도 0을 확인했다.

## 저장 데이터

- run id: `corridor_keepout_roundtrip_20260901T150446`
- 경로: `/home/lim/jdamr_artifacts/corridor_keepout_roundtrip_20260901T150446`
- 형식: MCAP
- 크기: 106.9MiB
- 기록 시간: 905.090초
- 총 메시지: 128,791
- MCAP SHA-256:
  `0390852648db3c4e1c1c68c3b9499ab7c37676d6651aef43a58c5da20633af6f`
- metadata SHA-256:
  `d4952657564bab95b0dfaf2f3c76136c37df8a2b3160136caf9b3da770383772`
- recorder transport loss: 136 messages

| 토픽 | 메시지 수 | 사용 판단 |
|---|---:|---|
| `/scan` | 6,859 | 오프라인 SLAM·drop 분석용 |
| `/odom` | 37,654 | 궤적·주기·지연 분석용 |
| `/imu/data_raw` | 37,658 | IMU 주기·노이즈 분석용 |
| `/tf` | 28,277 | TF 지연·권한 분석용 |
| `/tf_static` | 1 | 센서 고정 변환 재생용 |
| `/joint_states` | 8,867 | 상태 재생용 |
| `/cmd_vel` | 7,995 | 정지·감속·제어 명령 분석용 |
| `/amcl_pose` | 292 | 위치추정 흔들림 분석용 |
| `/battery_state` | 754 | 저전압 원인 배제용 |
| `/plan` | 434 | 계획 경로와 실패 시점 분석용 |

이번 전압은 약 11.8~12.0V였으므로 과거 9.68V 저전압에 의한 파이 종료와는 다른
장애다. Ping은 복도 끝에서 20% 손실, 평균 약 710ms, 최대 약 1.1초까지 악화됐다.

## 후속 온보드 실차 기록

Wi-Fi 의존성을 제거한 뒤 같은 날 네 번의 온보드 기록을 추가로 남겼다. 모두
`ros2 bag info`로 끝까지 읽히며 센서·TF·제어 실패 분석과 오프라인 SLAM 입력으로는
사용할 수 있다. 다만 첫 위치부터 80m 전체 왕복을 한 프로세스에서 끝낸 기록은 없다.

| run id | 크기·시간·메시지 | 실제 결과 | 판정 |
|---|---|---|---|
| `20260901T165606` | 78.0MiB · 555.333s · 87,577 | waypoint 7/20 완료 뒤 합성 컨테이너 내부 Nav2 노드 소실, recorder transport loss 20 | `DIAGNOSTIC_ONLY` |
| `20260901T170550` | 130.2MiB · 922.032s · 141,757 | 재시작 시 AMCL 원점 초기화, 위치 복원 뒤 반환점 도달, x 공분산 0.519와 15.008초 freshness에 차단 | `DIAGNOSTIC_ONLY` |
| `20260901T172141` | 43.2MiB · 302.940s · 50,968 | 현재 위치에서 남은 23.573m, 7/7 waypoint, recovery 0으로 복귀 성공 | `PARTIAL_SUCCESS` |
| `20260901T172738` | 24.8MiB · 178.080s · 27,217 | 독립 프로세스는 생존했지만 20ms planner action acknowledgement 제한으로 첫 목표 중단 | `DIAGNOSTIC_ONLY` |

`172141`은 깨끗한 부분 복귀와 controller 동작 증거로 보존한다. 하지만 시작점부터의
전체 왕복이 아니며, 경로 완료 뒤 합성 lifecycle manager가 map server와 controller
bond 실패를 보고하고 종료 중 SIGSEGV를 남겼다. 따라서 네 기록 어느 것도 최종 80m
성능 표본으로 승격하지 않는다.

## 반영한 수정

1. 저장지도 Nav2 launch가 forward-only BT 절대 경로를 항상 주입한다.
2. 검증된 20-point 왕복 경로를 해시 고정 Keepout 마스크와 함께 설정 파일로 남긴다.
3. `corridor_route`는 기본 planning-only이며 `--execute`가 있을 때만 움직이고,
   전체 경로 사전 계획·배터리·scan/odom freshness·AMCL 공분산을 통과해야 한다.
   실행 중에도 같은 게이트를 계속 확인한다. 복도 전용 BT는 waypoint마다 한 번만
   계획하고 회전·후진·재시도를 하지 않으며 action 응답 제한은 1,000ms다.
4. Nav2와 MCAP recorder를 파이에서 함께 실행하는
   `onboard_keepout_navigation.launch.py`를 추가했다. recorder가 끝나면 navigation도
   종료해 증거 없이 계속 달리지 않는다. 파이 전용 core는 복도 왕복에 필요한 Nav2
   기능만 남기되 필수 노드를 독립 프로세스로 실행하고, 하나라도 종료되면 launch 전체를
   종료한다.
5. 노트북용 RViz는 별도 view-only launch로 분리했다. Global Costmap과 LaserScan은
   기본 OFF, frame rate는 10Hz로 낮춰 제어 루프와 무선 대역폭을 경쟁하지 않는다.
6. 파이에는 최소 Nav2, 경로 실행기, 필수 토픽 recorder만 둔다. recorder는 CPU
   nice 10과 I/O best-effort 우선순위 7로 낮추고, RViz용 `/joint_states`는 기본
   기록에서 제외했다. 온라인 SLAM과 bag 분석은 주행 중 실행하지 않는다.
7. AMCL gate는 복도 진행축 x 2.0, 횡축 y 1.0, freshness 60초로 분리했다. 모든 차단은
   누락 토픽·나이·전압·축별 공분산을 수치로 기록한다. `--start-index` 재개는 현재
   AMCL 위치와 첫 잔여 waypoint가 6m 이내일 때만 허용한다.

## 온보드 정적 소크 결과

- run id: `onboard_minimal_soak_20260901T161338`
- 기록 시간: 107.862초
- MCAP: 14.7MiB, 16,150 messages
- MCAP SHA-256: `2a1f2f9fafec4713c705914d248d2e37ce2a17180c8eb57ea0c04f8b4196e231`
- metadata SHA-256: `f982e28f058237ef3f49aebf255e940b357865b6bacff7af9f059e6dcbcd59a6`
- 무결성: 15/15 chunk CRC, data-section CRC, summary CRC PASS
- load1: 1.17~1.93, 온도 66.2~70.6°C, thermal throttle `0x0`
- 주기: scan 약 9.48Hz, odom·IMU 약 50Hz
- `/cmd_vel` 5개와 `/cmd_vel_nav` 1개는 모두 0, 비영점 명령 0건
- odom 시작-종료 변위: 0.000025236m
- 종료 뒤 Nav2·recorder 잔류 0, `/cmd_vel` publisher 0

활성 구간에는 `bt_navigator`와 `collision_monitor`가 모두 `active [3]`이었고 root
`/cmd_vel` 발행자는 Collision Monitor 하나였다. 종료 중 합성 컨테이너의 기존 lifecycle
race가 SIGSEGV를 기록했지만 recorder는 cache를 flush하고 clean exit했으며 MCAP CRC와
종료 후 명령 소유권은 정상이다. 이 문제를 숨기지 않고 정상 종료 품질 항목으로 계속
추적한다. 사용자의 결정으로 별도 2~6m 단거리 시험은 생략하고, 다음 실주행은 전체
왕복으로 진행한다. 실제 출발 시 작업자가 로봇 옆에서 물리 전원을 즉시 차단할 수 있어야
한다.

## 반복 중단 수정본 비주행 소크

- run id: `static_repeat_stop_fix_20260901T174200`
- 실행 구조: map server, AMCL, controller, planner, velocity smoother, Collision Monitor,
  BT navigator와 두 lifecycle manager를 독립 프로세스로 실행
- 기록 시간: 330.746634840초
- MCAP: 45.4MiB, 49,556 messages
- MCAP SHA-256:
  `e1ad0afa71d86746b5530d66a34be11eed836fc0878f3846a31b22f81c864578`
- metadata SHA-256:
  `5f26bf46bfb86f3f3a9a3c8c2968ff7b4452520c3dfc0fed257ce4f7bb0dd5de`
- 무결성: 45/45 chunk CRC, data-section CRC, summary CRC PASS
- 필수 Nav2 노드: 시작 뒤 2분 시점 9/9, 5분 36초까지 process exit·bond failure 0
- 온도: 72.5~74.5°C, thermal throttle `0x0`
- load1: 4.13→8.11→8.59→9.85, 4.0 게이트 FAIL
- recorder transport loss: 3 messages, 최종 데이터 게이트 FAIL
- `/cmd_vel` 0건, `/cmd_vel_nav` 1건은 값 0, 비영점 명령 0건
- odom 시작-종료 변위: 0.000025301m
- SIGINT 뒤 8초 이내 process group 0, Nav2·recorder 잔류 0, `/cmd_vel` publisher 0

이 결과는 반복 중단의 직접 원인이던 합성 컨테이너 내부 노드 소실을 독립 프로세스가
막았다는 증거다. 반면 DDS participant 수가 늘면서 파이 부하와 recorder 손실이 커졌다.
부하를 줄이기 위해 localization과 navigation을 두 `component_container_mt`로 나눈
후속 시험은 32초의 load1 2.19와 2분 14초의 7.56을 기록했고, 사용자 전원 종료 요청으로
5분 전에 중단됐다. 종료 시 navigation container는 SIGTERM 뒤에도 남아 SIGKILL이
필요했으므로 이 구조는 채택하지 않았다.

## 다음 실차 PASS 조건

- Nav2·AMCL·Collision Monitor·MCAP을 파이에서 실행한다.
- 노트북은 `keepout_operator_view.launch.py`만 실행한다.
- 작업자가 로봇 옆에서 물리 전원을 즉시 차단할 수 있어야 한다.
- 최소 온보드 구성의 전체 왕복 planning-only는 3,098 poses·78.600m로 통과했다.
- 파이 로컬 bag에서 transport loss 0과 필수 토픽 수를 확인한다.
- 지속 CPU가 코어 예산의 75% 이내, thermal throttle 0, 최고 온도 75도 이하를 확인한다.
  2026-09-03에 `load1 < 4`를 이 기준으로 교체했다. 근거는 아래 소크 절에 있다.
- 전 구간 왕복 성공, 시작점 복귀, 최종 정지, 배터리 10.5V 이상을 확인한다.
- MCAP·metadata 해시와 토픽 수를 등록한 뒤에만 최종 데이터로 승격한다.

## 2026-09-03 주행 전 정비와 증거 등록

실주행 없이 처리했다. 상세는 `README_SLAM_PORTFOLIO.md`의 "2026-09-03 주행 전 정비"에
있고, 여기에는 이 실주행 기록들에 직접 관계된 사실만 남긴다.

### 기록 손실 게이트에 대한 조치

recorder는 11개 토픽을 구독하는데 QoS override는 6개뿐이었다. 나머지는 기본 depth
10으로 떨어졌고 50Hz `/imu/data_raw` 기준 0.2초 버퍼다. load1 9.85 구간의 정지 시간을
견딜 수 없는 값이다. 기록 계약을 `onboard_recording.py`로 모으고 depth를 `rate x 2초`로
정했다.

이 과정에서 발행자가 실제로 제시하는 QoS를 bag 3종으로 대조했다. `/imu/data_raw`는
**best_effort**, `/amcl_pose`는 **transient_local**이다. 두 값을 확인하지 않고 전 토픽을
reliable로 적었다면 IMU 구독이 매칭되지 않아 이후 모든 bag에서 IMU가 통째로 비었을
것이다. 회귀 테스트가 이 불일치를 막는다.

### 증거 원장 등록에서 새로 확인한 사실

`evaluation/ledger.py`로 15건을 등록하면서 문서에 없던 두 가지가 드러났다.

1. `corridor_keepout_nav_20260901T144128`과 `corridor_keepout_roundtrip_20260901T145437`은
   `/imu/data_raw`를 **아예 기록하지 않았다**. 두 기록은 IMU를 쓰는 오프라인 실험의
   입력이 될 수 없다.
2. `static_split_repeat_stop_fix_20260901T174845`는 `metadata.yaml`이 0바이트이고 MCAP도
   footer 없이 잘려 있다. 2컨테이너 분할 구조가 종료 시 SIGKILL을 필요로 했다는 서술의
   직접적인 파일 증거다. 이 기록은 읽을 수 없으므로 어떤 분석 입력으로도 쓰지 않는다.

### 파이 단독 보관 해소

`corridor_keepout_onboard_*` 4건을 포함한 9건이 파이 SD카드에만 있었다. 노트북
`~/jdamr_artifacts/`로 미러링하고 MCAP SHA-256 일치를 확인했다. 부분 복귀 성공 기록
`20260901T172141`도 이제 두 곳에 있다.

### 다음 소크에서 추가로 남길 것

`soak_metrics`로 프로세스별 CPU를 함께 기록한다. 이전 소크는 load1 총량만 남겨
9.85의 출처를 지목할 수 없었다.

## 2026-09-03 소크 재실행에서 확인한 두 가지 운영 조건

게이트 재측정을 시도하면서 문서에 없던 실행 조건 두 가지가 드러났다. 둘 다 실주행
전에 반드시 맞춰야 하는 항목이다.

### 1. 파이 bringup은 `ROS_DOMAIN_ID=12`에서 돈다

비대화형 SSH 세션은 `~/.bashrc`를 읽지 않아 도메인이 0으로 떨어진다. 이 상태로 Nav2를
띄우면 로봇 센서와 완전히 격리된 채 기동한다. 첫 시도에서 실제로 그렇게 됐고, MCAP에는
`/tf_static` 1건만 남았다. `/scan`·`/odom`·`/imu/data_raw`는 구독조차 되지 않았다.

원격 실행 스크립트는 `export ROS_DOMAIN_ID=12`를 명시해야 한다. 기록이 비정상적으로
작으면(5분 30초 소크 기준 45MB 대신 21KB) 도메인 불일치를 먼저 의심한다.

### 2. 오래 떠 있던 노트북 RViz가 lifecycle 기동을 막았다

도메인을 맞춘 두 번째 시도는 `map_server`가 지도를 읽고도
`failed to send response to /map_server/change_state (timeout)`으로 응답 전송에
실패했다. `lifecycle_manager_localization`은 AMCL을 configure하지 못한 채 멈췄고,
`map -> odom`이 없어 global_costmap 활성화가 타임아웃되며 navigation bringup이 abort
됐다.

당시 도메인 12에는 41시간 동안 떠 있던 노트북 `keepout_operator_view` RViz가 붙어
있었다. RViz를 종료하고 같은 명령을 다시 실행하자 세 lifecycle 매니저가 모두
`Managed nodes are active`에 도달했고 TF 프레임 오류는 0건이었다.

기존 문서는 원격 시각화 fan-out이 **주행 중 지연**을 악화시킨다고만 적었다. 실제로는
**기동 자체를 막을 수 있다.** 운영 순서는 다음과 같다.

1. 파이에서 Nav2와 recorder를 먼저 기동하고 lifecycle 활성화를 확인한다.
2. 그 뒤에 노트북에서 `keepout_operator_view.launch.py`를 새로 띄운다.
3. 실행이 끝나면 RViz도 함께 종료한다. 다음 실행까지 켜 둔 채 두지 않는다.

## 2026-09-03 게이트 정비본 정적 소크

- run id: `static_gatefix_20260903T104916`
- 기록 시간: 318.6초, MCAP 45.9MiB, 47,894 messages
- 무결성: 43/43 chunk CRC, data-section CRC, summary CRC, 전수 순회 PASS
- 필수 Nav2 노드: 세 lifecycle 매니저 모두 `Managed nodes are active`, TF 프레임 오류 0건
- `/cmd_vel` 0건, `/cmd_vel_nav` 0건 — 비영점 속도 명령 없음
- recorder: 손실 이벤트 보고 없음, 종료 시 캐시 flush 후 정상 종료

### 기록 손실 게이트 PASS

발행률 대비 기록률은 `/odom` 100.00%(15,930/15,930), `/imu/data_raw` 99.95%,
`/scan` 99.90%, `/tf` 99.19%다. 부족분은 시작·종료 경계에서 생기는 수준이다.

특히 `/imu/data_raw`가 **best_effort로 15,922건** 기록됐다. QoS override를 실측 없이
reliable로 적었다면 구독이 매칭되지 않아 0건이었을 자리다.

### 부하 게이트는 여전히 FAIL, 그러나 측정 대상이 틀렸다

load1은 3.97~10.66으로 4.0 게이트를 넘었다. 그런데 같은 구간에서 Nav2 프로세스
전체의 CPU 합계는 **4코어 400% 중 약 203%로 일정**했고, 스레드 수도 197개로 고정,
온도 65~69도에 thermal throttle 0x0이었다. CPU가 51%에서 평평한데 load1만 4에서 9로
출렁인 것이다.

즉 이 load1은 CPU 포화가 아니라 대기 상태 스레드를 함께 세고 있다. 파이 4코어 기준
약 두 코어가 놀고 있는 상태에서 게이트가 실행을 막고 있다.

프로세스별 평균 CPU는 다음과 같다. 특정 프로세스가 폭주하는 구조가 아니라 12~20%씩
고르게 퍼져 있다.

| 프로세스 | 평균 CPU | 최대 스레드 |
|---|---:|---:|
| controller_server | 20.1% | 17 |
| lifecycle_manager (3개 합) | 18.4% | 14 |
| bt_navigator | 17.2% | 13 |
| amcl | 16.4% | 14 |
| planner_server | 16.2% | 18 |
| map_server | 12.6% | 12 |
| collision_monitor | 12.2% | 15 |
| velocity_smoother | 12.2% | 12 |
| bringup | 2.5% | 14 |

### 게이트 교체 결과

`load1 < 4`를 지속 CPU·throttle·온도 세 항목으로 교체했고, 문서상 수동 기준이 아니라
`soak_metrics`가 판정한다. 같은 소크 데이터를 파이 4코어 기준으로 재판정하면
지속 CPU(p90) 204% / 예산 300%, throttle `0x0`, 최고 온도 69.6도로 **PASS**다.

기동 순간 296% 한 샘플은 12개 프로세스 동시 configure 구간이며 로봇이 움직이기 전에
끝난다. 그래서 순간 최대가 아니라 p90으로 판정한다. 회귀 테스트는 완화된 게이트가
여전히 실패할 수 있는지도 함께 확인한다. 지속 380%는 FAIL, throttle이 걸리면 CPU와
무관하게 FAIL, 온도 78도는 FAIL, 온도 미측정은 UNKNOWN이다.

충돌 감시, 배터리 전압, AMCL 공분산 같은 실제 안전 인터록은 건드리지 않았다. 이번
완화는 실행을 막던 잘못된 자원 지표 하나에만 적용된다.

## 2026-09-04 실주행과 확인된 두 가지

### RViz 를 띄우면 주행이 죽는다

`corridor_roundtrip_20260904T103147` 은 waypoint 1 을 통과하고 2 를 1.06m
남긴 지점에서 `odom stale: age=2.503s limit=2.500s` 로 취소됐다. 배터리는
11.72V 로 정상이었고 전날 넣은 재시도가 실제로 한 번 작동해 복구 1회를
기록했다.

취소 시점은 노트북에서 RViz 를 띄운 직후다. RViz 를 종료하자 끊겼던 파이
링크가 즉시 복구됐다. RViz 는 `/tf` 를 reliable QoS 로 구독하는데, 복도처럼
여유가 없는 무선에서 이 재전송이 파이의 송신 큐를 막아 로봇 내부 통신까지
밀린다. 2026-09-03 실내 시험에서 문제가 없었던 것은 링크에 여유가 있었기
때문이고, 복도에서는 성립하지 않는다.

**주행 중에는 RViz 를 띄우지 않는다.** 복도 구간은 어차피 무선이 끊겨 화면도
멈춘다. 확인이 필요하면 주행 전 실내에서만 연다.

### sensor_freshness_s 2.5 -> 10.0

`/odom` 은 50Hz 다. 2.5초는 125개를 놓친 것이지만, 무선이 잠깐 막히면 그
정도는 난다. 즉시 정지는 Collision Monitor 가 자체 `source_timeout` 2.0초로
이미 담당하므로, 경로 실행기의 이 게이트는 센서가 진짜로 죽었을 때 주행을
끝내는 2차 방어다. 10초로 완화했다.

### 라이다 CP2102 보드가 파이 종료 후 걸린다

2026-09-03 저녁에 파이를 `shutdown -h` 로 내린 뒤 라이다가 살아나지 않았다.
증상은 모터가 매번 정확히 5바퀴 돌다 멈추고, 커널 USB 이벤트는 부팅 시
허브 하나뿐이며 `/dev/ttyUSB*` 가 생기지 않는 것이었다. 파이를 여러 번
재부팅해도 그대로였다.

원인은 라이다 전원이 HAT 계통에서 오고 데이터만 파이 USB 로 가는 구조다.
파이를 꺼도 CP2102 변환 보드는 전원을 유지한 채 남고, 걸린 상태가 파이
재부팅으로는 풀리지 않는다. 라이다 케이블을 뽑아 보드 전원을 완전히 끊었다
다시 꽂자 `cp210x converter now attached to ttyUSB0` 가 뜨고 `/scan` 이
9.696Hz 로 복구됐다.

**파이를 종료할 때는 라이다 전원도 함께 내린다.** 라이다가 안 잡히면 파이
재부팅이 아니라 라이다 케이블을 먼저 뽑았다 꽂는다.
