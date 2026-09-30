# 2026-09-29 테이블 주차 실패와 재발 방지

상태: 충전 중 수정. 테이블 관측 위치까지 이동했으나 박스 앞 5cm 주차는 미완료다. 이후 수정본의 실차 성공으로 승격하지 않는다.

## 1. 관측 지점에 정밀주차를 적용해 발생한 Nav2 105

`home_exit`는 성공했다. 다음 `table_01_observation`의 목표는 (1.45, 0.25, yaw 0)이었는데, `ServiceRoute.execute()`가 마지막 waypoint를 무조건 주차 트리로 선택했다. 이 waypoint는 실제 박스 면을 찾기 전 관측 지점이다.

주행 MCAP에서 확인한 연쇄:

| 항목 | 근거 |
|---|---|
| 17:01:47.194 위치 | (1.445658, 0.242186), 목표와 8.94mm |
| 같은 시점 방향 오차 | 1.0798rad, 약 61.87도 |
| 회전 요청 | `/cmd_vel_nav` 약 -0.15rad/s |
| 충돌 방지 이후 | `/cmd_vel` 39개 모두 (0, 0) |
| 4.196초 동안 실제 변화 | odom 0.479mm, 방향 -0.000099rad |
| 17:01:51.390 종료 | `FAILED_TO_MAKE_PROGRESS`, 코드 105 |

오류 직전 10초 이동은 34.1mm, 회전은 0.0230rad였다. 진행 기준 50mm 또는 0.10rad를 충족하지 못했다. 충돌 방지가 회전을 차단한 것은 실제 기록으로 확인됐다. 전체 105 오류가 같은 원인이라고 일반화하지 않는다.

조치: `execute(final_parking=False)`를 관측 transit에만 적용한다. 관측 지점에서는 일반 재계획/위치 도착 판정을 사용하고, 이후 `face_alignment`와 `final_approach`에는 기존 주차 컨트롤러와 엄격한 방향·거리 확인을 유지한다. 회전 StopZone·진행 timeout·주차 허용치는 완화하지 않는다.

검증: 변경 전 회귀 테스트 6개 실패, 변경 후 관련 135개 통과. 실제 재주행은 충전 이후에 확인해야 한다.

## 2. 이동할 필요 없는 앵커를 정밀주차로 판정

현재 측정 좌표·방향으로 앵커 두 개를 만들었던 재개는 Nav2 목표 둘 모두 성공했지만 `parking odometry or final command missing`으로 종료됐다. 실제 이동이 없어 command가 발행되지 않은 상황에 최종 주차 확인을 요구한 운영 실수였다.

조치: 관측 위치 재개는 현재 정지 자세와 기존 관측 waypoint 거리 범위를 확인한 뒤 transit만 생략한다. 박스 관측·45cm 정렬·5cm 접근·최종 gap/corner/yaw 판정은 그대로 수행한다. 속도 0 메시지를 만들어 채우지 않는다. 수정 커밋 `9cbcefe`.

## 3. 박스 앞면 후보가 바뀌어 관측 실패

앞면 관측 거리 0.638~0.656m와 1.04~1.125m가 섞였고, 각도 약 +32~35도와 -15~-21도가 번갈아 나타났다. 8프레임 안정성이 확보되지 않아 `stable detected front surface is required`로 끝났다. 이 단계에서는 정렬/최종 접근 goal을 보내지 않았다.

최대 거리 0.85m와 평면 잔차 허용 0.015m으로 후보 영역을 좁힌 한 차례 시도도 실패했다. 임계값 변경만으로 해결된 문제가 아니며 원래 관측 설정으로 복원했다. 당시 원시 RGB-D 이미지가 없어 후보 전환의 기하학적 원인을 확정하지 않는다. 배경 평면/박스의 다른 면이 섞였다는 것은 후보 추정이며, 특정 물체 분류 결과가 아니다.

남은 수정/검증: 목표 영역에 결합된 일관된 면 선택을 구현하고 원시 RGB-D와 LiDAR 정합으로 검증해야 한다. 안정성·LiDAR 지지점 조건을 해제하거나 임의 박스를 목표로 삼지 않는다.

## 4. 후속 기록 유닛은 생성됐지만 rosbag 시작 실패

후속 두 유닛은 실행 사용자를 지정하지 않아 `rcutils_expand_user failed`와 `Failed to get logging directory`로 종료됐다. 첫 주행 MCAP은 존재하며 정상 마감됐지만, 후속 두 MCAP은 없다. 후속 JSONL은 보존했다.

재발 방지: 관리되는 기록 명령에 `User=<운영 사용자>`와 운영 사용자의 `ROS_LOG_DIR`를 명시하고 실제 recorder의 기록 시작과 metadata를 확인한다. `systemd-run`의 Started 응답을 기록 성공이라고 보고하지 않는다. 주행 요청 이후 이 준비 작업을 새로 작성하지 않는다.

## 5. PC RViz에 현재 로봇 표시 없음

PC에는 저장 지도와 목적지 마커가 있었으나 Current Robot Error가 남았다. 파이는 LOCALHOST 제어 그래프를 사용하고 PC RViz에는 실시간 TF/robot_description이 전달되지 않았다. 지도와 과거 표시가 보인다는 이유로 현재 로봇 위치를 확인했다고 판단한 운영상 누락이다.

조치 중: ROS 제어 경로를 바꾸지 않는 SSH 표시 전용 중계. 원래 필드와 `TransformStamped` timestamp를 보존하며 /tf, /tf_static, /robot_description, /scan, /plan, /amcl_pose만 전달한다. /tf는 자식 프레임별 최신 변환을 합쳐 전달하므로 메시지 경계 자체는 유지하지 않는다. /cmd_vel, action, initialpose, 서비스는 전달하지 않는다. 중계의 실제 화면·TF 연결 검증은 별도로 기록한다.

## 6. 백그라운드 실행 구분

파이의 베이스·RGB-D·관측기·Nav2 세션은 각각 하나였다. `lifecycle_manager_keepout`, `lifecycle_manager_localization`, `lifecycle_manager_navigation`은 서로 다른 역할로, 같은 주행의 3중 실행이 아니다. 관측기의 `ros2 run` 부모와 Python 자식도 서로 다른 검출기가 아니다.

종료 확인 당시 이동 스크립트와 rosbag 기록기는 남아 있지 않았다. 종료 뒤 3초 동안 odom 83개에서 선속도/각속도 최대 모두 0.0이었다. GUI의 백그라운드 숫자가 어느 목록을 뜻하는지는 이 파이 프로세스 목록만으로 확정하지 않는다.

## 7. 출발 요청을 수정/리뷰 단계로 바꾼 운영 실수

출발·재출발 도중 새 코드 작성, 앵커 재주행, 반복 관측과 검증을 이어가 실제 주행을 늦췄다. 해결되지 않은 센서 목표 문제를 남겨 둔 상태에서 접근을 이어가겠다고 예고한 것도 부정확했다.

재발 방지: 저장소 `AGENTS.md`에 출발/재출발을 기존 실행 경로만 사용하는 단계로 명시한다. 실패는 즉시 로그에 남기고 기존 기능으로 재개 가능한 구간만 이어간다. 코드·리뷰·빌드는 주행 종료 후 또는 요청된 충전 중 수정 단계에서 수행한다. 배치 확인을 다시 묻지 않고, 무의미한 같은 실패 반복을 하지 않는다.

## 8. 회전 탐색이 앞뒤 이동으로 바뀐 문제

관측 위치에서 동일 XY와 변경된 yaw를 NavigateToPose 주차 목표로 보냈다. RPP는 XY 허용 범위 밖에서는 목표 방향보다 경로 추종을 선택하므로, 이를 제자리 탐색 명령으로 쓰면 앞뒤 이동이 섞일 수 있다. 당시 behavior_server는 Wait만 로드해 Spin action을 사용할 수 없었다. 이것은 탐색 명령 선택과 실행 설정의 결함이며, 모든 왕복 현상이 이 원인 하나였다고 단정하지 않는다.

수정: 정밀 서비스에서만 Spin을 로드하고 `box_service --search`로 최대 30도씩 12회 탐색한다. Spin 전후 새 정지 자세로 회전량과 XY 이동량을 확인하며, 박스 면이 확보되면 45cm 정렬→5cm 접근으로 같은 실행기 안에서 이어진다. 관측 지점에 이미 도착했을 때는 `--resume-at-observation`으로 transit만 건너뛴다. 가까운 최종 접근 단계에는 탐색 회전을 넣지 않는다. 명령은 기존 감속기와 충돌 방지 경로를 통과한다.

센서 지연·비정상 scan metadata·geometry 오류를 박스 미검출과 구분한다. 회전으로 회복 가능한 면 관측 실패만 명시적 목록으로 재시도한다. 리뷰에서 발견한 모든 witness ValueError를 회전 재시도로 처리하던 결함도 수정했다.

제약: 회전 공간이 막혔을 때 전후진해 공간을 확보하는 자동 복구는 이번 구현에 포함하지 않았다. 당일 약 0.106m 전진과 약 0.115m 후진 기록은 개별 회피 동작이지 통합 복구 또는 주차 성공 근거가 아니다.

## 9. 후진 goal 수락 후 예외와 속도 설정 불일치

후진 실행기는 waypoint가 하나여도 이벤트 기록에 인덱스 1을 사용했다. goal 수락 후 IndexError가 날 수 있었고, handle을 기록 함수 이후 등록해 예외 시 취소 추적도 불완전했다.

수정: 마지막 waypoint 인덱스를 사용하고 빈 경로는 goal 전송 전에 거부한다. 수락한 handle은 기록보다 먼저 보관한다. NavigateToPose·FollowPath·Spin을 각각의 UUID와 action 취소/결과 서비스로 종료 확인한다.

별도로 박스 주차 계약의 0.04m/s와 서비스 launch의 smoother 상한 0.08m/s가 달랐다. 런타임 임시 변경은 재기동 시 유지되지 않았다. 후면 주차 서비스의 전진/후진 smoother 제한을 계약에 맞춰 생성하도록 수정했다. 회전 탐색 속도 역시 주차 계약에서 가져온다.

## 10. 18:29 센서·제어 지연 및 연쇄 비활성화

보존 journal에서 scan 입력 2.637초 지연, 10Hz controller 주기 미달, TF 미래 외삽 오류, keepout heartbeat 10초 미수신에 따른 lifecycle 비활성화가 확인됐다. 후속 재접근도 collision ahead와 controller patience 초과로 실패했다. 이를 모두 105로 묶지 않는다. 정밀주차는 완료되지 않았다.

높은 시스템 부하가 관측됐지만 CPU 부하만을 확정 원인으로 삼을 근거는 부족하다. 표시 중계는 구독 큐를 최신 1개로 줄이고, TF를 callback마다 재조합하지 않고 송신 시 조합하도록 바꿨다. 새 exporter는 낮은 CPU 우선순위로 실행된다. 이는 표시 부하 완화이며 온보드 scan 지연의 근본 해결 실측은 아니다. 충돌 방지 입력 timeout이나 lifecycle timeout을 늘려 오류를 숨기지 않았다.

충전 중 `jdamr-restaurant-navigation`을 종료했고 inactive를 확인했다. 센서 서비스는 유지했다. 수정 검증 중 실차 목표나 속도 명령을 전송하지 않았다.

19:07 정지 상태에서 기존 읽기 전용 probe로 15초를 측정했다. scan 142개, 9.638Hz, stamp age 최대 0.118초, 0.2초 초과 수신 간격 0회였다. odom 740개, 50.007Hz이며 선속도·각속도·변위 최대 모두 0이었다. Nav2가 꺼진 상태의 기준선이므로 주행 부하에서의 지연 해결 증거는 아니다. 파이 소스에 probe 파일이 없어 첫 호출은 실행되지 않았고, 로컬의 기존 probe를 SSH 표준입력으로 전달해 측정했다. 새 이동 코드는 만들지 않았다.

## 11. 반복 설정 조회에서 ROS service client 누적

관측용 지도 확인·탐색 회전 설정·후진 속도·costmap footprint 조회마다 `AsyncParameterClient`를 새로 만들었다. 설치된 Jazzy 구현은 인스턴스마다 6개 service client를 만들며, 지역 변수만 사라져도 Node가 보유한 client는 남는다. 회전 설정 조회 12회를 실제 rclpy Node에서 재현했을 때 72개가 남았다. 응답은 대체했지만 client 생성과 개수는 실제 rclpy 객체로 측정했다.

수정: `ServiceRoute._read_parameters()`가 대상 노드별 `GetParameters` client 하나를 재사용한다. 반환된 설정값은 캐시하지 않고 매번 서버에서 새로 읽는다. 사용하지 않는 다른 5개 parameter service의 발견을 기다리지 않는다. 기존 서비스 발견 2초·응답 2초 상한을 유지하고, 중단·timeout 때 미완료 요청을 client에서 제거한다. 지도 식별·footprint·속도·회전 설정 비교와 충돌 방지 조건은 유지했다.

검증 결과:

- 수정 전 회귀 실패: 12회 조회 뒤 client 72개. 수정 후 같은 반복에서 1개 유지.
- 새 설정값 수신, 서비스 부재 시 요청 금지, 중단 시 pending request 제거를 검증했다. GetParameters만 제공하는 실제 ROS 서버에서도 두 번의 서로 다른 응답을 받았다.
- restaurant service·box service·activate navigation 관련 229개 테스트 통과. 변경 Python 2개 파일 ament_flake8, py_compile, diff 검사 통과.
- 파이의 실행 모듈 SHA-256은 로컬과 같은 `4087e2cd05c87e18c85cfc8980143cf22758ea0ded0cc9a016071592beae8998`이다. 파이 관측 노드의 `processing_hz`를 12번 읽어 모두 2.0을 받았고 client는 1개였다. 해당 조회 구간은 0.283초이며, 서비스 시작·Nav2 활성화·실차 태스크 시간을 뜻하지 않는다.
- 파이 원본은 `$HOME/jdamr_data/restaurant_service.before_parameter_clients_20260929.py`에 보존했다. Python 모듈은 빌드 경로에서 소스를 참조하므로 다음 실행부터 적용된다. Nav2·모터 서비스를 재기동하거나 이동 goal/cmd_vel을 보내지 않았다.
- 독립 리뷰 에이전트는 런타임 thread limit으로 시작하지 못했다. 작성과 분리한 검토 패스에서 호출부·값의 신선도·실패 전파·pending request 정리·수명 종료를 확인했다. 독립 승인으로 표시하지 않는다.

연결 누적 결함은 재현·수정했다. 하지만 이것이 18:29 heartbeat 상실의 원인이었다는 인과는 아직 확인하지 못했다. 당시 생성 client 수와 장애 시각의 대응 기록이 없으므로 모든 지연의 해결이라고 보고하지 않는다. 기존 `BoxService.precision_parameters`와 `CorridorRoute.parking_parameters`는 인스턴스를 재사용하므로 이번 누적 결함과 구분해 그대로 두었다.

파이 조회 출력은 `$HOME/jdamr_data/nav2_architecture_20260929_tF5td9/parameter_reader_pi_check.json`에 보존했다. 구성 판단과 남은 계측 조건은 [Nav2 실행 위치 조사](20260929_NAV2_PLATFORM_RESEARCH.md)를 따른다.

## 12. 개발 우선 운영과 불필요한 태스크 종료 축소

사용자 요청: 교실·복도의 저속 학습용 시제품에서 실제 태스크 구현·실행을 최우선으로 한다. 반복 출발 검수·코딩·리뷰로 배터리를 소모하거나, 일시적인 문제마다 처음부터 재주행하는 과정을 줄인다. 저장소 `AGENTS.md`에 이후 세션의 우선순위로 기록했다. 차체 손상 감수 의사를 사람·물건과의 충돌 허용 또는 센서 없이 이동하는 권한으로 확대하지 않는다.

변경 범위:

| 기존 처리 | 반영한 처리 |
|---|---|
| 입력 지연 시 전체 경로 종료 | 일반 CorridorRoute와 ServiceRoute는 이전 action의 terminal 확인 후 새 입력·새 정지 odom을 기다려 현재 waypoint부터 1회 재개. 대기는 최대 8초이며 정상 출발에 고정 대기를 추가하지 않음 |
| 탐색·후진의 입력 지연 후 처음부터 재시작 | Spin은 남은 각도만, 후진은 현재 실제 pose부터 경로를 다시 만들고 충돌 확인 후 재개 |
| 매 이동·회전에 출발 예비전압 적용 | 첫 action 수락 전 10.8V, 같은 실행기의 후속 단계는 기존 진행 중 10.5V. 저전압·비정상 값 차단은 유지 |
| 45cm 중간 정렬에도 1cm/1도 최종 판정 | 저속 Parking 제어기는 유지하고 중간 정렬용 5cm/3도 goal checker 분리. 최종 5cm 간격·양쪽 모서리·1도 판정은 유지 |
| 최종 접근의 단발 경로 계산 | 1Hz 재계획 및 복구 가능한 계획/제어 오류에 한해 1초 대기 후 1회 재시도. 가까운 물체 앞 임의 Spin/BackUp은 넣지 않음 |
| 탐색 Spin 충돌 차단 즉시 태스크 종료 | COLLISION_AHEAD가 확인된 경우에만 앞·뒤 20cm 관측 후보를 계획하고 가능한 첫 위치로 1회 이동 시도. 속도 명령을 우회 발행하지 않음 |
| 같은 지도 파일·서버 설정 조회 반복 | 동일 실행기에서 확인한 지도 자산을 재사용. 실제 grid 변경과 명령 경로 검사는 유지 |
| 준비 시간까지 240초 태스크 예산에 포함 | 준비가 끝나고 이동을 시작하는 시점부터 계산 |
| ROS graph 누락으로 전체 Nav2 종료 | graph 감시기는 진단 알림으로 한정. 일반 주행·자율 매핑 launch에서 감시기 종료를 전체 shutdown 조건에서 제외. 핵심 프로세스 종료·lifecycle bond 보호는 유지 |
| 기동 완료 후 후속 조회 실패에도 RESET | 기동·ACTIVE 확인이 끝난 서버는 유지하고 준비 미완료를 보고. STARTUP 자체 실패·부분 활성화는 기존 rollback 유지 |
| 세션 스크립트 기본 SUBNET | 현재 온보드 제어 기본 LOCALHOST와 일치. 명시한 discovery-range 옵션은 유지 |

스킬 확인:

- 설치된 개인 스킬에서 주행 진단에 해당하는 것은 `ros-graph-triage`다. 모터 제어 코드는 없으며 진단 지침이다. 출발/재출발 자동 적용을 제외하고 Codex `allow_implicit_invocation: false`로 바꿨다. PKM 원본을 수정하고 설치본을 동기화한다.
- `arm-motion-gate`·`arm-motion-guard` 설치 스킬과 해당 Codex 훅은 이번 조회에서 발견되지 않았다. 없는 스킬을 제거했다고 기록하지 않는다.
- `servo-register-triage`는 팔 서보 진단용이며 이번 베이스 주행 차단 원인으로 확인되지 않아 변경하지 않았다. OMX·리뷰 스킬 전체를 삭제하지 않고 출발 단계의 새 필수 절차로 사용하지 않도록 프로젝트 규칙에 명시했다.

유지한 조건: 실제 footprint·keepout, CollisionMonitor, 유효 입력, 명령 watchdog, 저전압 cutoff, 사용자 정지. 정밀 접근 앞 여유 5cm·costmap 차체 패딩 2cm 및 회전 swept 영역은 변경하지 않았다. 정지거리·입력 지연의 실측 근거 없이 수치를 낮추지 않는다. 수동 조종에는 사용자가 버튼을 놓은 뒤 자동으로 다시 움직이는 기능을 넣지 않았다.

검증: 관련 629개 테스트 통과, 변경 핵심 Python 9개 파일 ament_flake8 통과, 셸 구문·diff 확인 및 로컬 navigation 패키지 빌드 통과. 입력 공백 시 취소 terminal 뒤 현재 waypoint만 재전송하는 시나리오, 남은 Spin 각도와 후진 경로 재계산, 지도 자산 재사용 후 grid 변경 거부를 검사했다. 독립 리뷰 에이전트는 thread limit으로 시작되지 않아 작성 후 별도 로컬 검토 패스를 수행했으며 독립 승인으로 표시하지 않는다.

이번 변경은 18:29 센서·제어 지연의 근본 원인이 사라졌다는 증거가 아니다. 박스 면 후보가 바뀌는 인식 문제도 별도이며, 수정본의 실제 주차 성공·소요시간 단축·충돌 회피 성능은 아직 측정하지 않았다. 새 거리 완화나 실물 주행은 실행하지 않았다.

파이 반영: 변경 파일을 기존 소스에 반영하고 navigation 패키지 빌드, `box_service --help`, 실행 모듈 구문 확인을 완료했다. 원본 백업은 파이 `$HOME/jdamr_data/development_flow_20260929_T8eNdh/before.tar.gz`, 반영 목록은 같은 디렉터리의 `changed_paths.txt`다. 주행 세션은 inactive를 유지했고 베이스 MainPID 17643은 변경되지 않았다. 새 정렬 goal checker·BT 설정은 다음 Nav2 세션 시작 때 로드된다. 실행 중 센서·모터 서비스 재시작이나 이동 명령은 없었다.

## 13. 개발 우선 변경의 실행 경로 추가 검증

사용자 요청에 따라 cb21ce1의 구현을 다시 확인했다. 앞선 629개 테스트 통과만으로 반영이 충분했다고 볼 수 없었다. 다음 네 가지 누락을 확인해 수정했다.

| 확인된 문제와 근거 | 수정 및 확인 |
|---|---|
| 지도 검사 캐시 키 문자열을 서버 확인 반복문의 지도 dict가 덮어썼다. 첫 호출부터 두 번 실행하니 parameter 조회가 기대 2회가 아니라 4회였다. 기존 테스트는 캐시를 미리 넣어 이 결함을 놓쳤다. | 캐시 키 변수를 분리. 첫 호출에서 생성한 캐시를 두 번째 호출이 재사용하는 검사를 추가했다. 실제 grid 변경과 명령 경로 확인은 유지한다. |
| action 수행 중 입력 공백은 재개했지만 다음 waypoint 발행 직전 공백은 재개 이유를 기록하지 않고 종료했다. 첫 waypoint 성공 후 다음 입력만 늦추면 전체 실행이 실패했다. | 일반 경로·테이블 경로·Spin·후진의 action 시작 경계에도 기존 입력 회복 분기를 연결. 완료 waypoint를 건너뛰는 전환과 사용자 정지·저전압 미재개를 확인했다. |
| 탐색 Spin의 COLLISION_AHEAD만 관측 위치 변경 대상이었다. 로컬 20초 제한으로 취소하면 오류 코드가 None이어서 이 분기에 진입하지 못했다. | 정상 입력·사용자 정지 아님을 확인한 로컬 만료를 TIMEOUT으로 기록한다. 취소 terminal 확인 후 COLLISION_AHEAD 또는 TIMEOUT에 한해 기존 앞·뒤 20cm 후보 계획을 1회 시도한다. TIMEOUT을 충돌 원인 확정으로 해석하지 않는다. |
| 세션 셸은 LOCALHOST였으나 별도 restaurant_service.launch.py 기본값은 SUBNET이었다. | 현재 온보드 센서와 같은 LOCALHOST로 통일. 명시적인 SUBNET 옵션이 보존되는 것도 검사했다. |

검증 근거:

- 수정 전 새 재현 검사에서 지도 캐시·waypoint 경계 두 실패를 확인하고 수정 후 통과했다.
- 관련 14개 테스트 파일에서 651개 통과. 변경 Python/launch/test 6개 파일 ament_flake8 통과, 로컬 navigation 빌드 통과.
- 로컬 변경과 기존 파이 실행 소스가 cb21ce1 기준으로 일치하는지 대조한 후 여섯 파일만 백업·반영했다. 파이 navigation 빌드와 설치된 Python 모듈 import가 통과했다. 설치된 Spin TIMEOUT 값은 701이다.
- 파이 백업: `$HOME/jdamr_data/development_verify_20260929_ENEC2Z/before.tar.gz`. 같은 디렉터리에 반영 목록·SHA-256·빌드 로그를 보존했다.
- 반영 전후 주행 서비스는 system/user 모두 inactive, 베이스 MainPID는 17643으로 같았다. 센서·모터 서비스 재시작, action 실행, 속도 명령 발행은 하지 않았다.
- 독립 리뷰 에이전트는 thread limit으로 생성되지 않았다. 작성 후 별도 로컬 diff·회귀 검증 패스를 수행했으며 독립 승인으로 표기하지 않는다.

`ros-graph-triage`는 이번에 진단 절차로 실행하지 않고 지침 내용을 검토했다. 모터 제어·제동 실행부가 아닌 TF·통신·Nav2 최초 장애를 좁히는 읽기 전용 지침이며, 안전 차단만을 목적으로 하는 스킬은 아니므로 삭제하지 않았다. PKM 원본과 Codex 설치본의 `allow_implicit_invocation: false`, 저장소의 출발 자동 적용 금지를 확인했다. 이번 변경에서 새 스킬·검수 단계·출발 조건을 추가하지 않았다.

남은 검증 범위: 실차 주차 완료·주행 지연 감소는 측정하지 않았다. 반복 센서 지연의 근본 원인, Nav2 실행 중 비활성화 재발 여부, 박스 면 후보 변경 문제까지 해결됐다는 증거는 아니다. 위 네 코드 결함의 수정 및 반영과 실차 태스크 성공을 구분한다.

## 14. 정지 캡처·관측 입력 공백·계획 끝점·박스 이탈 복귀 (2026-09-30, Claude 인계 후)

인계 문서 `20260930_CLAUDE_HANDOFF.md` §4의 미수정 두 경로와, 이어받으며 보존 기록에서 새로 확인한 실행 경로 결함을 수정했다. 계획은 OMC ralplan 합의(Planner·Architect·Critic 3회차, 정오표 반영 v4.1)를 거쳤다. 이 절은 소프트웨어 수정과 파이 반영까지의 기록이며 **실차 주행은 하지 않았다.** 5 cm 주차·복귀 성공은 주장하지 않는다.

### 14.1 확정 사실과 근거

| 사실 | 근거 |
|---|---|
| AMCL은 odom 축별 변위가 0.05 m/0.05 rad를 넘을 때만 갱신·발행한다. 정지 중에는 `/amcl_pose` 없이 map→odom TF만 scan마다 재발행한다. 그래서 같은 프로세스에서 15 s 넘게 정지한 뒤 `capture_stationary_pose()`와 `wait_until_ready()`의 guard(True)가 TF가 정상이어도 `AMCL pose stale`로 막혔다 | nav2_amcl 1.3.12 소스, `new_base_nav2_params.yaml` update_min_d/a, 합성 재현(인계 §4-A) |
| 새 프로세스는 TRANSIENT_LOCAL `/amcl_pose`를 받는 순간 나이가 0이다. 막히는 것은 같은 프로세스 안의 긴 정지 뒤다 | `_amcl_callback` 수신 시각 기준 |
| `observe_target()`은 입력 공백 1회에 회복 없이 종료했다 | 인계 §4-B 재현 |
| `observe_target()`은 같은 관측 status를 매 반복 다시 평가했다. 수신 뒤 나이가 0.5 s를 넘으면 신선도 실패로 판정이 덮여, 보존 bag 기준 창 끝의 약 27 %가 재시도 불가로 끝났다 | 보존 MCAP status 563건 재계산 |
| 관측 transit BT의 기본 goal checker는 방향을 보지 않는 `position_goal_checker`다. 9/29 네 관측 실행은 모두 (1.446, 0.243)에서 실제 yaw 61.7°였고, 영역 표지 (1.896, 0.303)는 카메라 반시야각 약 29° 밖이었다. 번갈아 나온 앞면 후보는 다른 면이었을 가능성이 크다(지정 박스 식별은 미확정) | 보존 MCAP 재투영, BT XML |
| 플래너 tolerance 0.5에서 `plan_pose()`는 목표에서 떨어진 곳에서 끝나는 경로도 성공으로 받았다(플래너 비교 범위 0.15–0.26 m, 현재 NavFn은 0.15 m) | 실제 지도 오프라인 planner 비교 |
| KeepoutFilter 출력은 팽창되지 않고 unknown 셀을 FREE로 덮어쓴다. 점 로봇 플래너가 keepout 경계에 중심을 붙여 계획한다. 깊이로 계산한 박스 면 후보는 keepout 띠(y ≥ 0.619) 안이라 5 cm 목표가 keepout 안에 떨어진다 | 같은 오프라인 비교 |
| 박스 앞 5 cm 정지 자세에서는 LiDAR(range_min 0.28 m)와 깊이(최소 0.35 m) 모두 면을 보지 못한다. 그 자리의 제자리 회전은 collision monitor가 막지 못한다 | provenance, `new_base_geometry.yaml` |
| 정밀 세션은 box 계약으로 Parking·ParkingReverse를 띄우는데 `restaurant_service`는 일반 계약 파일을 고정으로 읽어, 같은 세션의 `home --execute`가 이동 전에 결정적으로 실패했다 | launch·CLI 코드 |
| 9/29 실행 중 한 번은 관측 위치로 가던 중 AMCL x 공분산이 0.0102 m²(한계 0.01)로 잠깐 넘어 재개 경로 없이 끝났다. 주행 중 공분산은 수렴 뒤 0.002–0.009 m²였다 | 파이 events.jsonl, 보존 MCAP `/amcl_pose` |

보존 로그에는 `AMCL pose stale`·`stationary teaching unavailable` 종료 기록이 없다. 위 AMCL 나이 결함을 과거 출발 실패의 원인이라고 단정하지 않는다.

### 14.2 조치 (커밋 `2fbed38`)

| 결정 | 내용 |
|---|---|
| D1 | `ServiceRoute._guard_failure(True)`는 마지막 AMCL pose 이후 odom 축별 변위가 AMCL 갱신 임계 이하이고 map→base_link TF가 0.5 s 이내일 때만 메시지 나이 조건을 면제한다. 공분산·지도·배터리 등 나머지 검사는 그대로다. 초기화 전·이동 후·TF 정지·미래 시각은 계속 거부한다 |
| D2 | 관측 중 입력 공백은 함수당 1회 기존 회복으로 기다린 뒤 새 정지 pose와 새 cutoff로 다시 관측한다. 실패는 plain `RuntimeError`라 탐색 회전으로 이어지지 않는다. status는 stamp별로 한 번만 평가한다. 판정을 만든 지연은 회전 대신 무이동 재관측 1회로 처리한다 |
| D3 | `_verify_parking_stop()`은 입력 공백에 명령 없이 1회 재확인한다. box 계획 시점 공백은 공백 전 목표를 버리고 다시 관측한다 |
| D4 | (이 커밋 시점) 이동 중 공분산 초과는 코드를 바꾸지 않았다. 사용자 답변(불필요하면 제거)에 따라 §15.2에서 중간 단계 한도로 바꿨다 |
| D5 | `box_service --return-home --return-timeout-s <s>`: 최종 접근 성공 뒤 검증된 5 s 대기 → 박스 이탈 → 충전소 복귀를 한 실행기에서 이어간다. 복귀 도킹은 충전소 계약(5 cm/3°)으로 판정한다. `--task-timeout-s`로 테이블 구간 예산을 받는다(기본 240 s) |
| D6 | 관측 전 카메라 자세·영역 방위·시야 포함 여부·거리를 기록하는 무이동 진단 이벤트를 남긴다 |
| D7 | `plan_pose()`는 경로 끝이 goal checker 허용오차 밖이면 `goal_not_reachable_within_tolerance`로 실패한다. box transit preflight도 같은 검사를 한다 |
| D8 | 이탈은 박스 면에서 0.565 m(StopZone 회전 반경 0.43 + 여유)까지 직선 후진한다. 검증은 현재 차체 앞쪽 띠를 뺀 꼬리로 한다(직선 후진은 그 띠를 다시 쓰지 않는다). 재시도는 이미 후진한 거리만큼 띠를 줄인다. 이탈 전 방향·거리 타당성을 확인하고, 실패 사유는 `box_escape_*`로 남긴다 |
| D9 | 관측 실패 시 최근 status 16개·정지 pose·scan·mount를 묶은 증거 이벤트를 남긴다. 새 이벤트는 `allow_nan=False`로 직렬화할 수 있고, 증거 기록 실패가 제어 흐름 예외를 바꾸지 않는다 |

`restaurant_service home`만 `--parking-contract`를 받는다. 박스 앞 약 0.6 m 안에서는 이 단독 복귀를 쓰지 말라는 안내를 도움말에 넣었다(센서가 그 거리의 면을 보지 못한다).

### 14.3 검증

- 테스트를 먼저 작성했다. 수정 전 HEAD `5ab15ee`에서 회귀 테스트 49건이 계획한 사유(`HEADFAIL[Tn]`)로 실패했고, 하네스 오류·수집 오류는 0건이었다. 기존 테스트는 모두 통과했다.
- 수정 후: 핵심 5개 파일 434건, 관련 24개 스위트 934건이 통과했다.
- 변이 M1–M12가 모두 지정 테스트에서 검출됐다. M2에서 살아남은 세 사례(지도 불일치·사용자 정지)는 앞선 `stop_requested` 경로 때문에 도달할 수 없는 등가 사례다.
- 독립 코드 리뷰에서 차단 결함은 없었다. 지적된 이탈 재시도 검증 구멍, 이탈 전 타당성, 복귀 checker 확인 순서 등은 테스트를 먼저 실패시킨 뒤 고쳤다.
- 변경 파일의 flake8·pep257 위반은 0건이다. 패키지 전체의 기존 flake8 위반 29건은 HEAD와 같다.
- PC와 파이에서 `colcon build --packages-select jdamr_cube_navigation`이 통과했다.

### 14.4 파이 반영

- 반영 전 파이 런타임 3파일은 `5ab15ee`와 같았다. 백업·반영 기록: 파이 `$HOME/jdamr_data/development_verify_20260930_ad77a4/`(`before.tar.gz`, `changed_paths.txt`, `build.log`, `deployed_sha256.txt`, `no_motion_cli_check.txt`).
- PC 커밋, 파이 소스, 파이 설치 site-packages의 런타임 3파일 SHA-256이 모두 같다.
- 베이스 MainPID는 반영 전후 44342로 같고, 주행 서비스는 inactive를 유지했다. Nav2 기동·action·속도 명령은 없었다.
- 무이동 점검: 새 CLI 옵션, `--return-timeout-s` 없는 `--return-home` 거부, departure_mask_v2 registry·경로의 지도 식별 일치, 계약·카메라 장착 로드를 확인했다.
- 운영 관찰: 2026-09-30 06:52–06:56에 무인 보안 업데이트(openssl·curl·expat·sudo 등, ROS 패키지 아님)가 라이브러리 교체 뒤 `jdamr-base`·`jdamr-box-observer`·`jdamr-box-rgbd`·ssh·네트워크 서비스를 재시작했다. 새 커널 6.8.0-1065가 설치되어 재부팅을 기다린다(실행 중 1064). 주행 중 같은 재시작이 일어나면 태스크가 끊긴다. 업데이트 정책은 사용자 결정 사항으로 남긴다.

### 14.5 남은 항목 (출발 전 Phase 2, 사용자 답변 필요)

- 지정 박스의 실제 위치와 붙을 면: 현재 데이터로는 면 후보가 keepout 안이라 최종 접근 계획이 끝점 검사에서 실패한다. 답변에 따라 region·관측 waypoint·keepout 사본을 만들고, 실측 면 선분·LiDAR 잡음 0–0.02 m·줄자 ±0.01 m 조합을 오프라인으로 검증한 뒤 출발한다.
- 관측 지점 옆 지도에 없는 물체(폭 약 0.32 m)의 유지 여부, 이동 중 공분산 정책, 실제 충전소와 registry home_dock(route 시작점과 0.2 m 차이)의 일치 여부.
- 첫 주행은 진단 성격이 크다. 5 cm 간격과 0.05 m costmap 격자가 겹쳐 최종 접근이 목표 직전에 멈출 수 있다. 면 셀과 2 cm 잡음을 둔 격자 모델(추론, 실측 아님)에서 최종 자세 footprint 외곽선이 면 셀과 겹쳐 목표 직전에 멈출 비율은 면이 지도 축과 평행해도 약 80 %였다(잡음 없이 면 셀만이면 0° 40 %, 30°·45° 80–96 %). 최종 목표 셀 자체가 inscribed가 되어 끝점 검사가 이동 전에 `goal_not_reachable_within_tolerance`로 거부할 비율은 0° 약 20 %, 15° 36 %, 30° 80 %, 45° 60 %였다.
- 이탈 시작 자세에서 RPP 현재 자세 충돌 검사가 접근 중 남은 costmap 면 셀과 겹치면 `box_escape_failed`로 멈춘다(fail-closed). ObstacleLayer footprint clearing이 이 셀을 지우는지는 실차에서 확인하지 않았다.
- 활성화 단계의 공분산 통과는 `/request_nomotion_update` 반복 뒤의 값이라 위치추정 품질의 독립 증거가 아니다.
- status를 stamp별로 한 번만 평가하면서, HEAD가 같은 status를 0.5 s 안의 새 scan으로 LiDAR witness에 다시 넣던 암묵적 재시도도 사라졌다. 첫 주행에서 `box_lidar_witness_rejected` 빈도를 본다.
- 리뷰에서 보류한 알려진 한계(회복 뒤 cutoff의 시간축 혼합, D1 참조 시점 서술, 잘못된 관측기 JSON의 침묵 판정, 정밀 세션 전진 home 미지원 등)는 보존 폴더의 `review/code_review_phase1.md`에 있다.
- 권고(설정 변경 없음): inflation_radius 0.42 m 이상은 오프라인에서 NavFn 여유를 늘렸고 불가능해진 목표가 없었다. keepout 마스크를 차체 여유만큼 팽창하는 것도 권한다. 플래너는 NavFn을 유지한다(Smac Hybrid는 Ackermann용, 비원형 차동의 공식 권장인 Lattice는 이 지도에서 큰 루프·keepout 침범을 보였다). 정밀 세션 footprint 앞변(물리 0.065 m) 또는 costmap 해상도 0.025 m는 사용자 선택 항목이다. 파이는 대기 중에도 74–78 °C라 냉각을 권한다.
- 근거 자료 보존(PC): `$HOME/jdamr_data/claude_phase1_20260930/`(합의 계획·검토, 박스 면·지연 분석, 오프라인 planner 비교, HEAD 실패 증명, 변이·리뷰 기록).

## 15. 중간 단계 공분산 한도, 파이 DDS 공유메모리 삭제, 9/29 초기 자세 오차 (2026-09-30 오후)

### 15.1 확정 사실과 근거

| 사실 | 근거 |
|---|---|
| 09:0x 파이에서 새로 뜬 ROS 참가자는 3 s 안에 `/scan`·`/odom`·`/tf`와 발행자를 발견하면서도 20 s 동안 메시지를 0개 받았다. 새 참가자끼리는 정상 수신(12 s 38개) | 읽기 전용 probe 3종 |
| 원인: 센서 서비스 3개는 시스템 유닛이지만 `User=lim`이고, logind `RemoveIPC`는 기본 yes, `Linger=no`였다. 08:52:38 PC의 `jdamr-rviz-display-relay`·`robot-dashboard-link` 정지로 파이 lim의 마지막 ssh 세션이 끝나 08:52:49에 `user-1000.slice`가 제거되면서 `/dev/shm/fastrtps_*`가 지워졌다. 기존 노드의 매핑은 `(deleted)`로 남아 기존 노드끼리만 통신했다 | `/proc/<pid>/maps`, `/dev/shm` 0개, 파이·PC journal 시각 대조 |
| `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`를 준 LOCALHOST 모드 프로세스도 공유메모리 포트를 매핑하고 있었다 | 같은 maps |
| 조치(사용자 실행): `sudo loginctl enable-linger lim` 뒤 `jdamr-base`·`jdamr-box-rgbd`·`jdamr-box-observer` 재시작. 이후 새 참가자 수신 19 s에 `/scan` 156·`/odom` 774·`/tf` 469. 베이스 드라이버는 시작 시 속도 명령 없이 정지 명령만 보낸다(`base_driver_node.cpp:168-172`) | probe 재실행, `loginctl show-user` |
| 9/29 출발 대기 구간(0–54 s)의 AMCL 자세 (−0.622, −0.878, 87.7°)는 같은 스캔 40개를 지도에 정합한 자세 (−0.533, −0.896, 93.3°)와 9 cm·5.5° 달랐다. AMCL 자세로 투영하면 남쪽 벽이 약 5° 기울고 5 cm 안 점은 17 %, 정합 자세는 64 %였다. 관측 지점 정지 구간도 8.5 cm 차이였다 | 보존 MCAP `/scan`·`/tf`, 오프라인 likelihood-field 정합(전역 탐색에서 유일해) |
| 9/29 출발 위치(정합 자세)는 registry `home_dock`에서 0.30 m 떨어져 있었다 | 같은 정합 |
| 정합 자세로 다시 투영하면 앞서 박스 후보로 보던 L자 형태는 지도의 큰 장애물 서쪽 벽(x≈2.33)과 겹친다. 9/29 출발 위치에서 LiDAR 높이로 table_1·table_2 표지 주변의 지도에 없는 박스 크기 물체는 확인되지 않았다 | 같은 투영 |
| 9/29 관측 transit 도착 시점 공분산 5건은 x 0.0033–0.0071, y 0.0017–0.0062였다. 정지 뒤 재개 실행 3건은 같은 값을 그대로 기록했다(정지 중 AMCL 미발행) | 파이 events.jsonl(독립 리뷰가 집계) |

9/29 AMCL 오차가 생긴 과정(시드 근처 nomotion 반복으로 입자가 실제 자세에 닿기 전에 모였을 가능성)은 추론이며 재현하지 않았다.

### 15.2 조치

| 결정 | 내용 |
|---|---|
| D10 | AMCL 공분산 한도를 단계별로 나눴다. 중간 단계(관측 경로 transit, 관측·탐색 회전·면 정렬, 관측 지점 재개 준비, 정밀 세션 복귀 staging의 계획·주행)는 위치추정 상실 한도만 본다. x·y 0.04 m²(0.2 m)는 9/29 수렴 상태 최대 0.010213과 기록된 미수렴 최소 0.17 사이 값이고, yaw는 미수렴 기록이 없어 초기 pose 시드 분산 (15°)²를 쓴다(근거는 계약 파일 `intermediate_bound_provenance`). 새 출발 준비, 최종 접근, staging 정지 확인, 최종 캡처, 박스 이탈, 도킹 후진, 활성화는 0.01 m²·(10°)²를 유지한다. 비유한·음수 검사는 모든 단계에 남는다 |
| D10 구현 | 계약 키 `intermediate_max_*`와 로더 검증(유한·수치·엄격 한도 이상). `ServiceRoute._localization_bound()`는 단계 값을 정확히 설정하고 finally에서 복원한다. `execute()`는 transit이면 중간, alignment는 호출 단계 상속, final은 항상 엄격이며 입력 공백 회복 대기도 같은 한도를 쓴다. 공분산 실패 메시지에 x·y 값(소수 6자리)과 `bound=intermediate/strict`를, yaw 메시지에 값·한계를 넣고, 정지 캡처 실패에는 마지막 가드 사유를 붙였다. 표준 세션 경로는 바뀌지 않는다 |
| D11 | 파이 lim linger 활성화(사용자 실행). 저장소 AGENTS.md "주행·실험 뒤 정리"에 linger 전제를 추가했다 |

### 15.3 검증

- 새 테스트 25건(T31a–g, T32a–c, T33, T34a–e, T35)과 계약 로더 테스트 7건을 먼저 작성했다. 수정 전 코드에서 20건이 계획한 사유로 실패했다(`HEADFAIL[Tn]`, `AMCL x covariance high: value=0.010 limit=0.010`, `localization or sensor data unavailable`, 단계 한도 기능 없음). 수정 전에도 통과한 5건(T31c 2건, T34a, T35 2건)은 수정 전후 모두 0.01·미수렴에서 멈춰야 하는 음성 대조군이다. 로더 테스트 7건은 수정 전 모두 실패했다.
- 변이 6종(final의 중간 한도 상속, 새 출발 준비의 중간 한도, final 단계의 중간 한도, finally 복원 제거, staging 정지 확인의 중간 한도, 엄격 한도 상향)이 모두 지정 테스트에서 검출됐다.
- 수정 후 핵심 7개 파일 568건 통과. 패키지 전체를 같은 조건의 HEAD 사본과 비교해 새 실패 0건. 변경 파일 ament_flake8·pep257 0건.
- 독립 코드 리뷰 1차(transit만 완화한 초안): APPROVE, 권고 반영(T31f·T31g, 기록 갱신, 도착 뒤 잠긴 공분산 → 단계 확장). 2차(단계 확장본): APPROVE, 권고 반영(엄격 유지 테스트 T34, 미수렴을 통과시키던 0.25 → 0.04, staging 정지 확인 엄격화, 메시지 정밀도, HEADFAIL 규약).

### 15.4 남은 항목

- 중간 한도는 발산 감지일 뿐 정확도 보증이 아니다. keepout은 map 좌표로 적용되므로 LiDAR로 보이지 않는 금지 영역은 위치추정 정확도에 기댄다.
- 9/29처럼 초기 자세가 틀린 채 수렴하면 공분산 한도는 이를 잡지 못한다. 출발 초기화에서 스캔 정합 자세와 AMCL 자세를 대조한다(운영 절차, 코드 아님).
- 도착 뒤 AMCL 값이 0.04를 넘은 채 잠기면 관측 단계에서, 정렬 뒤 0.01을 넘은 채 잠기면 최종 접근에서, staging 뒤 0.01을 넘으면 도킹 전에 멈춘다. 실패 메시지에 값과 적용 한도가 남는다. 운영자 조치는 충전소 이동 뒤 재초기화다.
- 데이터 폴더의 `resume_table_trial.py`는 `covariance high` transit 중단을 복구 가능으로 보고 nomotion을 반복한다. D10 뒤에는 그 사유가 위치추정 상실을 뜻하므로 쓰지 않는다.

## 16. table_01 박스 탐색 실패와 도크 복귀 (2026-09-30 11:36–12:02)

데이터 폴더: `$HOME/jdamr_data/map_update_20260929_4XUrkc/phase2_20260930/` (파이 같은 경로).

### 16.1 관찰

| 시각 | 실행 | 결과 |
|---|---|---|
| 11:36–11:41 | `runs/table_01_20260930_113650` (box_service `--search`, 박스 경로 `table_01_route_box.yaml`) | 관측점 (1.0, 0.05) 도착 뒤 박스 면 안정 판정(8프레임·영역 안)을 한 번도 통과하지 못하고 +33° 탐색 회전을 이어 갔다. 박스 쪽 방위에서는 면이 카메라에서 0.42 m라 화면을 넘쳐 대부분 `no_box_surface_candidate`, 다른 방위는 비스듬한 면의 흔들림이나 다른 평면을 골랐다. 7번째 탐색 회전에서 `nav2_spin_failed`로 끝났다(누적 210°) |
| 11:46 | `runs/home_20260930_114600` (`restaurant_service home --execute`, 박스 계약) | 움직이기 전 도크 후진 경로 정적 검사 `static_obstacle_unknown_keepout_or_map_boundary`로 종료. 로봇 이동 없음 |
| 11:58–12:01 | `runs/home_20260930_115853` (같은 명령, 아래 16.2 수정 뒤) | staging 도착(Nav2 4), staging 정지 확인 통과, 후진 경로 검사 통과, 도크 후진 중 52 s 뒤 FollowPath 중단 `Failed to make progress`(Nav2 6, 오류 105). 중단 직후 정지 확인은 도크 목표 대비 2.6 cm·1.7°(내부 추정, 외부 실측 없음)로 confirmed였지만 서비스 판정은 `failed`, 종료 코드 1 |

### 16.2 도크 경로 차단 원인과 조치

- 정적 검사(`static_corridor_clear`, 런타임 footprint 0.38×0.58 m)가 세 곳에서 막혔다.
  1. staging 자세 차체 왼쪽이 도크 서쪽 keepout 블록 동쪽 열(x −0.93~−0.88)과 수 mm 겹침.
  2. 도크 자세 차체 뒤쪽(y −1.138)이 지도 점유 2칸(x −0.63~−0.58, y −1.08~−0.98)과 겹침. 이 칸은 스캔 정합·AMCL이 1 cm 안에서 일치한 도크 자세에서 차체 안(base_link 뒤 0.19–0.24 m)에 들어가고, 같은 자리 초기화 스캔에서 뒤쪽 0.77 m 안에 반사가 없었다 → 지도 작성 때의 일시 물체로 판단.
  3. 통로 가운데 미확인 1칸(−0.70, −0.51): 도크 초기화 스캔 빔 287개가 통과, 반사 0.
- 사용자 지시("킵아웃 해제하고 도킹 진행해", "킵아웃만들지말고 그냥해")로 keepout을 전부 비운 마스크를 먼저 썼으나, 세션 런치가 `keepout mask contains no blocked cells`로 거부했다(`keepout_mask.py`). 그래서 원래 keepout에서 staging 회전 원(0.414+0.10 m)과 도크 통로(반폭 0.29+0.10 m)에 걸리는 46칸만 열었다.
- 새 자산 `map_dockfix.yaml`(3칸 수정), `keepout_dockfix.yaml`(46칸 해제), 근거 `dockfix_provenance.json`. 원본 `local_updated.yaml`·`keepout.yaml`과 이전 registry `service_destinations.before_dockfix.yaml`은 보존. registry 해시 교체 → Nav2 세션 재시작 → 현재 자리에서 스캔 정합 초기화(0.933, 0.074, 185.2°; AMCL과 0.4 cm·0.1°, inlier 0.50). 도구 `jdamr_depart.py`의 `MAP_YAML`도 새 지도로 바꿨다.
- 새 지도에서는 PC 표시 유닛 `jdamr-p2-markers`가 `table annotation belongs to a different map`으로 시작하지 않는다(표시 전용, 주행 무관).

### 16.3 남은 항목

- 도크 후진 끝의 진행 없음 중단 원인은 미확정이다. 진행 검사는 10 s 안 5 cm 이동을 요구한다(`PoseProgressChecker`). 저속 끝 구간의 바퀴 불감대, 또는 컨트롤러 목표 판정과 정지 확인이 다른 시점의 map→odom을 쓴 차이가 후보이며, 다음 도킹에서 cmd_vel·odom 속도와 goal checker 판정을 함께 기록해 가린다.
- 중단 뒤 정지 자세가 도크 계약 안이어도 서비스는 실패로 판정한다. 재시도는 원인 확인 전에는 하지 않았다.
- 사용자는 지도 재작성, 박스 재배치, 목적지 재설정을 따로 진행할 예정이다. 그때 이번 흔적 칸과 keepout 경계도 새 지도 기준으로 다시 정한다.

## 앞선 충전 중 수정본 검증

- 로컬: box service, restaurant service, relay, new-base 설정, keepout, reverse parking, parking contract/integration, depth target, LiDAR witness, session, stop profile 관련 570개 테스트 통과.
- 변경 Python 9개 파일 ament_flake8 통과, `git diff --check` 통과.
- 별도 code-reviewer: 차단 결함 수정 후 코드 finding 0, 관련 358개 테스트 통과. LSP 도구 부재로 형식상 판정은 COMMENT이며 실차 승인으로 해석하지 않는다.
- PC와 파이에서 navigation 패키지 빌드 통과. 파이 `box_service --help`에 탐색·관측 위치 재개 옵션이 표시되고 소스 3개 파일 SHA-256이 로컬과 일치한다. 주행 서비스는 inactive 상태를 유지했다.
- 파이 반영 전 백업: `$HOME/jdamr_data/charging_code_backup_20260929_3fDZK3/navigation_before_fix.tar.gz`. 파이 배치 경로는 Git 저장소가 아니므로 파일별 비교·백업 후 여섯 실행 파일만 반영했다. 기존 실행 중 표시 중계는 재시작하지 않았으므로 부하 완화 변경은 다음 중계 시작부터 적용된다.
- 실차 5cm 거리·박스 면 평행 정렬·복귀, 장시간 센서 지연 재발 여부는 미검증이다.

## 보존 데이터

- 데이터 루트: `$HOME/jdamr_data/parking_attempt_20260929_preserved/`
- 최초 주행: `table_departure_20260929_latest_bag/`
- MCAP SHA-256: `d40f57c88818b68f326616aaf8b5d0c49f4bdf8dc431dd532cc6cf11a5e21d3e` (PC와 파이 일치)
- 실행 이벤트: `table_departure_20260929_zkmrDR`, `table_face_resume_20260929_lsuar_qj`, `table_park_resume_20260929__9ovfvaf`, `table_near_face_20260929_qrq4gvlf`
- 충전 중 추가 보존: `$HOME/jdamr_data/parking_charging_fix_20260929_hwT9Pe/`의 탐색·회피·재접근 JSONL 다섯 run과 `nav2_latency_journal.log`. 이 후속 구간의 원시 RGB-D rosbag은 확보하지 못했다.
- 정지 기준선: 같은 폴더의 `stationary_sensor_baseline.json` (파이 원본은 위 백업 폴더). 지도 위치추정은 종료 상태라 map→odom 자료는 없으며 센서/베이스 기준선만 제공한다.

기록과 코드 검증은 주행 성공 증거가 아니다. 테이블 앞 5cm 정밀주차는 아직 성공하지 않았다. 충전소 복귀는 2026-09-30 12:01에 도크 계약 안(내부 추정 2.6 cm·1.7°)에 정지했지만 Nav2 진행 없음 중단으로 서비스 판정은 실패다(§16).
