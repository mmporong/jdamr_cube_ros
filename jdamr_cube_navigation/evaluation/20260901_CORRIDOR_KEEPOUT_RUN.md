# 2026-09-01 저장지도 Keepout 복도 왕복 실차 진단

## 결론

이번 세션은 금지구역 지도와 80m 왕복 경로의 계획·전반부 실주행을 검증했지만,
완전한 왕복이나 프로토콜 적격 데이터 수집에는 성공하지 못했다. 로봇이 복도 끝에서
멀어질수록 노트북과 파이 사이 Wi-Fi 지연이 커졌고, 노트북에서 추가한 rosbag 구독이
고속 토픽 fan-out을 늘리면서 TF와 센서 전달이 2초 이상 밀렸다. Collision Monitor와
Nav2가 정지·목표 중단으로 fail-closed 동작했고, 최종적으로 Nav2 전체를 정상 종료했다.

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
| 새 온보드 실행 구조 | 비주행 정적 소크·전체 경로 계획 완료 | Wi-Fi와 무관한 제어·로컬 기록 | 80m급 왕복 완주가 남음 |

저장 지도와 Keepout은 오프라인 SLAM에 주입하지 않는다. 재생 시 저장 `/map`과 이동
명령을 제외하고, 기록된 AMCL의 `map -> odom`도 제거해 새 mapping backend 하나만 TF
권한자가 되게 한다. 따라서 기존 지도를 따라 수집했다고 새 지도 결과가 기존 지도의
복사본이 되지는 않는다.

### 파이에 남기는 범위

| 실행 위치 | 주행 중 실행 | 이유 |
|---|---|---|
| 파이 | 하드웨어 bringup | 센서·모터의 로컬 연결 |
| 파이 | 합성 Nav2·AMCL·Collision Monitor·Keepout | Wi-Fi가 끊겨도 제어와 정지가 동작해야 함 |
| 파이 | `corridor_route` | 목표 전송과 센서·배터리·위치추정 게이트 |
| 파이 | 저우선순위 필수 MCAP recorder | 고속 원본을 Wi-Fi로 보내지 않고 보존 |
| 노트북 | 저대역폭 view-only RViz | 지도·마스크·위치·계획 확인만 수행 |
| 주행 후 노트북 | 2D SLAM 재생·그래프·통계, 카메라를 별도 수집한 경우 Visual SLAM | 주행 중 파이 CPU·디스크와 경쟁하지 않음 |

첫 온보드 정적 시험에서 공식 Nav2 전체 구성을 composition으로 묶어도 load1이
9.33~11.27까지 올라 4.0 게이트를 넘었다. 그래서 복도 실행에 필요한 map server,
AMCL, controller, planner, velocity smoother, Collision Monitor, BT navigator만 한
컨테이너에 남겼다. 사용하지 않는 smoother server, route server, behavior server,
waypoint follower, docking server는 파이 전용 launch에서 제외했다. Keepout mask·info와
그 lifecycle manager는 fail-closed 감시를 위해 별도 프로세스로 유지한다.

## 문제와 해결 상태

| 문제 | 확인된 원인 | 반영한 해결 | 현재 상태 |
|---|---|---|---|
| 계단 가지 진입 위험 | 원본 점유 지도만으로는 가지 입구를 막지 않음 | 두 다각형+0.55m 팽창 Keepout, 해시·연결성 고정 | 마스크 생성 검증 완료, 다음 실차 확인 필요 |
| 출발 전 `Empty Tree` | 저장지도 launch의 기본 NavigateToPose BT 경로가 비어 있었음 | launch에서 검증된 BT 절대 경로 주입 | 코드·회귀 테스트 완료 |
| 복귀 중 TF·센서 지연 | 약한 Wi-Fi에서 노트북 Nav2·RViz·rosbag의 고속 구독 fan-out | 파이의 합성 Nav2+저우선순위 필수 recorder, 노트북 view-only RViz | 구조 변경 완료, 실차 재검증 전 |
| 복구 회전으로 상태가 불명확 | 기존 BT가 회전·대기 복구를 최대 3회 수행 | 왕복 실행기는 회전·후진·재시도 없는 fail-fast BT 사용 | 코드·회귀 테스트 완료 |
| 지도 재생 시 TF 충돌 위험 | AMCL과 mapping backend가 모두 `map -> odom`을 발행할 수 있음 | 격리 domain에서 `/map`, 이동 명령, 기록 AMCL TF를 제거 | 오프라인 재생 가드 구현됨 |
| 파이 load gate 초과 | Jazzy 기본 bringup이 복도에서 쓰지 않는 서버까지 10개 실행 | 파이 전용 최소 합성 launch로 5개 주행 서버만 유지 | 정적 load1 1.17~1.93 통과 |
| launch 생성 실패 뒤 recorder 잔류 | 포함 launch보다 recorder가 먼저 시작됨 | navigation 생성 뒤 recorder를 시작하고 process-group 정리 | 재발 방지 테스트·실기 확인 |
| planning-only 준비 실패 | AMCL은 마지막 pose를 `TRANSIENT_LOCAL`로 보관하지만 실행기는 기본 `VOLATILE`로 늦게 구독 | AMCL pose 구독을 reliable+`TRANSIENT_LOCAL`로 일치 | 3,098 poses·78.600m 계획 통과 |
| 합성 종료 로그의 SIGSEGV | lifecycle manager와 합성 노드가 동시에 preshutdown하는 Jazzy 종료 race | process-group 전체 종료와 종료 뒤 `/cmd_vel`·잔류 프로세스 확인 | 운용상 정지는 확인, 정상 종료 개선은 추적 |
| 이번 왕복 미완료 | Wi-Fi 지연 뒤 위치추정 흔들림과 controller 102 | 같은 경로를 온보드 구조로 재수집 | 미해결, 다음 실차 PASS 필요 |

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

## 반영한 수정

1. 저장지도 Nav2 launch가 forward-only BT 절대 경로를 항상 주입한다.
2. 검증된 20-point 왕복 경로를 해시 고정 Keepout 마스크와 함께 설정 파일로 남긴다.
3. `corridor_route`는 기본 planning-only이며 `--execute`가 있을 때만 움직이고,
   전체 경로 사전 계획·배터리·scan/odom freshness·AMCL 공분산을 통과해야 한다.
   실행 중에도 같은 게이트를 계속 확인하고, 복도 전용 BT는 회전·후진·재시도를 하지
   않는다.
4. Nav2와 MCAP recorder를 파이에서 함께 실행하는
   `onboard_keepout_navigation.launch.py`를 추가했다. recorder가 끝나면 navigation도
   종료해 증거 없이 계속 달리지 않는다. 파이 전용 core는 복도 왕복에 필요한 Nav2
   구성만 composition으로 실행하며, 종료는 컨테이너 단독 kill이 아니라 launch process
   group 전체 종료로 수행한다.
5. 노트북용 RViz는 별도 view-only launch로 분리했다. Global Costmap과 LaserScan은
   기본 OFF, frame rate는 10Hz로 낮춰 제어 루프와 무선 대역폭을 경쟁하지 않는다.
6. 파이에는 합성 Nav2, 경로 실행기, 필수 토픽 recorder만 둔다. recorder는 CPU
   nice 10과 I/O best-effort 우선순위 7로 낮추고, RViz용 `/joint_states`는 기본
   기록에서 제외했다. 온라인 SLAM과 bag 분석은 주행 중 실행하지 않는다.

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

## 다음 실차 PASS 조건

- Nav2·AMCL·Collision Monitor·MCAP을 파이에서 실행한다.
- 노트북은 `keepout_operator_view.launch.py`만 실행한다.
- 작업자가 로봇 옆에서 물리 전원을 즉시 차단할 수 있어야 한다.
- 최소 온보드 구성의 전체 왕복 planning-only는 3,098 poses·78.600m로 통과했다.
- 파이 로컬 bag에서 transport loss 0과 필수 토픽 수를 확인한다.
- 출발 전 1분 정적 소크의 load<4와 thermal throttle 없음은 통과했다. 주행 중에도 같은
  항목을 계속 기록한다.
- 전 구간 왕복 성공, 시작점 복귀, 최종 정지, 배터리 10.5V 이상을 확인한다.
- MCAP·metadata 해시와 토픽 수를 등록한 뒤에만 최종 데이터로 승격한다.
