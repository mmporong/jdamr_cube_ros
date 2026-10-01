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

## 16. table_01 박스 탐색 실패와 충전소 복귀 (2026-09-30 11:36–12:14)

데이터 폴더: `$HOME/jdamr_data/map_update_20260929_4XUrkc/phase2_20260930/` (파이 같은 경로). 도킹 성공 자료는 `$HOME/jdamr_data/dock_return_success_20260930/`에 따로 묶었다([포트폴리오 인계](20260930_DOCK_RETURN_PORTFOLIO_HANDOFF.md)).

### 16.1 관찰

| 시각 | 실행 | 결과 |
|---|---|---|
| 11:36–11:41 | `runs/table_01_20260930_113650` (box_service `--search`, 박스 경로 `table_01_route_box.yaml`) | 관측점 (1.0, 0.05) 도착 뒤 박스 면 안정 판정(8프레임·영역 안)을 한 번도 통과하지 못하고 +33° 탐색 회전을 이어 갔다. 박스 쪽 방위에서는 면이 카메라에서 0.42 m라 화면을 넘쳐 대부분 `no_box_surface_candidate`, 다른 방위는 비스듬한 면의 흔들림이나 다른 평면을 골랐다. 탐색 회전 6회는 Nav2 성공(4), 7번째가 703으로 끝났고, 11:41:07 운영자 정지(`jdamr_depart.py stop`)로 종료 |
| 11:46 | `runs/home_20260930_114600` (`restaurant_service home --execute`, 박스 계약) | 움직이기 전 도크 후진 경로 정적 검사 `static_obstacle_unknown_keepout_or_map_boundary`로 종료. 로봇 이동 없음 |
| 11:58–12:01 | `runs/home_20260930_115853` (같은 명령, 16.3 수정 뒤) | 11:59:11 (0.94, 0.07)에서 staging (−0.64, −0.14)로 출발, 12:00:57 도착(106 s, Nav2 4). staging 정지 확인 통과(staging 목표 대비 2.6 cm·1.7°, 1.0 s 유지). 후진 경로 검사 통과, 12:01:00 후진 시작, 12:01:52 FollowPath 중단 `Failed to make progress`(Nav2 6, 오류 105). 서비스 판정 `failed`, 종료 코드 1 |
| 12:01 이후 | 사용자 현장 확인 | 도크 정위치에 도달했다. 끝 구간에서 앞뒤 왕복과 좌우 반복 회전이 있었다 |
| 12:13–12:14 | 무이동 측정 (`final_pose/`) | 도크 목표 대비: map→base_link TF 0.25 cm·+3.83°, 래치된 `/amcl_pose` 4.0 cm·+3.76°, 독립 스캔 정합 3.9 cm(도크 축 2.2 cm·측방 3.2 cm)·−1.17°(inlier 0.61, 전역 탐색 일치). 줄자 실측 없음 |

### 16.2 정정 (커밋 `5462b62` 기록)

`5462b62`의 "중단 직후 정지 확인은 도크 목표 대비 2.6 cm·1.7°"는 틀렸다. `failed` 이벤트의 `confirmation`은 staging 정지 확인 결과가 그대로 남은 값이다(hold 1.0139 s·오차가 staging 이벤트와 같다). 후진 중단 뒤에는 도크 정지 확인이 실행되지 않았다. 도크 최종 자세 수치는 12:14 무이동 측정값뿐이다.

### 16.3 도크 경로 차단 원인과 조치

- 정적 검사(`static_corridor_clear`, 런타임 footprint 0.38×0.58 m, 미확인 = 막힘)가 세 곳에서 막혔다.
  1. staging 자세 차체 왼쪽이 도크 서쪽 keepout 블록 동쪽 열(x −0.93~−0.88)과 수 mm 겹침.
  2. 도크 자세 차체 뒤쪽(y −1.138)이 지도 점유 2칸(x −0.63~−0.58, y −1.08~−0.98)과 겹침. 이 칸은 스캔 정합·AMCL이 1 cm 안에서 일치한 도크 자세에서 차체 안(base_link 뒤 0.19–0.24 m)에 들어가고, 같은 자리 초기화 스캔에서 뒤쪽 0.77 m 안에 반사가 없었다 → 지도 작성 때의 일시 물체로 판단.
  3. 통로 가운데 미확인 1칸(−0.70, −0.51): 도크 초기화 스캔 빔 287개가 통과, 반사 0.
- 사용자 지시("킵아웃 해제하고 도킹 진행해", "킵아웃만들지말고 그냥해")로 keepout을 전부 비운 마스크를 먼저 썼으나, 세션 런치가 `keepout mask contains no blocked cells`로 거부했다(`keepout_mask.py`, 11:53:18). 그래서 원래 keepout에서 staging 회전 원(0.414+0.10 m)과 도크 통로(반폭 0.29+0.10 m)에 걸리는 46칸만 열었다.
- 새 자산 `map_dockfix.yaml`(3칸 수정), `keepout_dockfix.yaml`(46칸 해제), 근거 `dockfix_provenance.json`. 원본 `local_updated.yaml`·`keepout.yaml`과 이전 registry `service_destinations.before_dockfix.yaml`은 보존. registry 해시 교체 → Nav2 세션 재시작 → 현재 자리에서 스캔 정합 초기화(0.933, 0.074, 185.2°; AMCL과 0.4 cm·0.1°, inlier 0.50). 도구 `jdamr_depart.py`의 `MAP_YAML`도 새 지도로 바꿨다. 수정 뒤 staging 회전(185°→92.3°)과 후진 통로가 같은 정적 검사를 통과했다.
- 새 지도에서는 PC 표시 유닛 `jdamr-p2-markers`가 `table annotation belongs to a different map`으로 시작하지 않는다(표시 전용, 주행 무관).

### 16.4 도크 후진 끝의 105 해석

| 구분 | 내용 |
|---|---|
| 확인된 근거 | 정지 뒤 컨트롤러가 보는 자세(map→base_link TF)는 위치 0.25 cm, yaw +3.83°로 도크 계약 yaw 3°를 넘는다. 진행 검사는 `PoseProgressChecker` 5 cm·0.10 rad / 10 s다. 후진 수락 시 AMCL yaw 분산 0.00489 rad²(1σ 4.0°), 종료 시 0.00788 rad²(1σ 5.1°)로 계약 yaw 3°보다 크다. 사용자가 끝 구간 왕복·좌우 회전을 봤다 |
| 추론(미검증) | 위치는 허용 안인데 yaw가 허용 밖이라 goal checker가 도착을 인정하지 않았고, 컨트롤러가 방향을 고치려 왕복·회전하는 동안 10 s에 5 cm·0.1 rad 넘는 진전을 만들지 못해 중단했다. cmd_vel·컨트롤러 내부 오차는 기록하지 않아 확정하지 못한다 |
| 가리지 못한 것 | TF yaw(+3.8°)와 스캔 정합 yaw(−1.2°)가 5° 다르다. 도크 목표 yaw 92.3°도 10:24 teach-home 때의 TF 값이다. 외부 실측이 없어 어느 쪽이 실물에 가까운지 이 자료로 정할 수 없다 |

### 16.5 남은 항목

- 후진 중단 뒤 서비스는 도크 정지 확인 없이 실패로 끝나고, 실패 이벤트에 이전 단계(staging) 확인값을 싣는다. 충전 중 수정 후보: 중단 뒤 기존 `_verify_parking_stop` 경로로 도크 계약 정지 확인 1회, 이벤트에 확인 단계 표시.
- 도크 yaw 허용 3°가 AMCL yaw 1σ(4–5°)보다 작다. 허용을 넓힐지, 도크 끝 구간에서 yaw를 스캔 정합으로 보정할지는 다음 도킹에서 cmd_vel·odom·goal checker 판정을 함께 기록한 뒤 정한다.
- 사용자가 지도 재작성, 박스 재배치, 목적지 재설정을 진행할 예정이다. 이번 흔적 칸과 keepout 경계는 새 지도 기준으로 다시 정한다([지도·박스 설치 계획](20260930_MAP_BOX_LAYOUT_PLAN.md)).

## 17. 2026-09-30 전체 실행·변경 대장

오전 인계부터 12:14까지. 로봇을 움직인 실행은 사용자 요청("출발", "주차좀 진행해", "충전소 도킹 진행해봐") 범위다.

### 17.1 실행 대장

| 시각 | run (`phase2_20260930/runs/`) | 경로·영역 | 결과 | 기록된 사유·근거 |
|---|---|---|---|---|
| 10:22 | `init_20260930_102257` | — | 중단 | 정합 단계 도구 오류(당시 수정: 빔 수가 다른 scan의 각도 binning, map origin 부호 파싱). `scan.json`만 남음 |
| 10:23–10:53 | `init_…102327`, `…103424`, `…104502`, `…104729`, `…105342` | — | 활성화 | 반복 초기화(세션 재시작·교착 조사 중). 각 `activation.jsonl` |
| 10:30 | `table_01_20260930_103006` | `table_01_route.yaml`, (1.896, 0.303) | 이동 없음 | `live map data unavailable: keepout, map`. Nav2 합성 컨테이너 교착(`container_hang_evidence/`: futex 대기, `dds.*` 스레드) |
| 10:35 | `table_01_20260930_103516` | 같음 | 이동 없음 | `stationary teaching unavailable`. 같은 교착 |
| 10:45–11:03 | `churn_test_*.log` | — | 진단 | 참가자 반복 접속: 합성+LOCALHOST 2회째 교착, 중계 없이 4회째 교착, 루프백 UDP 2회째 교착, 비합성 10/10 통과 → 세션을 `--use-composition false`로 운영 |
| 10:57 | `init_20260930_105723` | — | 중단 | 정합 통과(inlier 0.61) 뒤 도구 JSON 직렬화 오류(numpy bool). 수정 뒤 재실행 |
| 10:58 | `init_20260930_105836` | — | 활성화 | 스캔 정합 (−0.613, −0.834, 92.2°)와 AMCL 1 cm 이내. home_dock 유지 |
| 11:03 | `table_01_20260930_110340` | `table_01_route.yaml`, (1.896, 0.303) | 관측점 도착, 관측 실패 | `box observation unavailable: stable detected front surface is required`(표지 영역에 박스 없음) |
| 11:08 | `table_01_20260930_110834` | `table_01_route_forward.yaml` | 관측점 도착 | 탐색 회전 1회 뒤 `nav2_spin_failed` |
| 11:18, 11:20 | `…111834`, `…112042` | `table_01_route_box.yaml`, (1.13, 0.50) | transit 실패 | `table_01_box_pre` Nav2 104(PATIENCE_EXCEEDED). keepout이 회전 원 안 |
| 11:22 | `…112240` | 같음 | 관측점·search_view 도착 | 탐색 회전 703(COLLISION_AHEAD) ×2 → `nav2_spin_failed` |
| 11:25 | `…112549` | 같음 | transit 실패 | `table_01_box_back_off` 104 |
| 11:28 | `…112806` | 같음 | 관측점·search_view 도착 | 안정 앞면 없음(`box_observation_unavailable_evidence`) |
| 11:31 | `…113143` | 같음 | transit 실패 | `table_01_box_front` 104 |
| 11:35 | `…113541` | 같음 | 이동 없음 | `parking controller parameters unavailable`(부하 중 일시, 다음 실행 통과) |
| 11:36 | `…113650` | 같음 | 관측점 도착, 운영자 정지 | §16.1 |
| 11:46 | `home_20260930_114600` | 도크 | 이동 없음 | §16.3 정적 검사 |
| 11:53 | 세션 재시작 1차 | — | 기동 거부 | 전부 비운 keepout(`keepout mask contains no blocked cells`) |
| 11:57 | `init_20260930_115717` | — | 활성화 | 재시작 뒤 현재 자리 스캔 정합(inlier 0.50, 전역 일치), AMCL 0.4 cm·0.1° |
| 11:58 | `home_20260930_115853` | 도크 | 도크 도달(사용자 확인) | 서비스 판정 105 실패, §16.4 |

박스 정밀 주차(정렬 → 5 cm → 5 s 대기 → 이탈)는 이날 한 번도 정렬 단계에 들어가지 못했다. 막힌 이유는 세 가지였다. 박스 자리가 keepout과 지도 장애물에 가까워 회전이 막혔고(104·703), 관측 거리와 방향 때문에 앞면이 안정되지 않았으며, 가까운 transit waypoint는 BT에서 곧바로 도착 처리됐다.

### 17.2 변경 대장

| 구분 | 변경 | 위치·근거 | 반영 |
|---|---|---|---|
| 코드 | 중간 단계 AMCL 공분산 한도(x·y 0.04 m², yaw (15°)²) | 커밋 `e3b00ca`, §15 | 파이 런타임 3파일 해시 일치 |
| 설정 | 박스 앞면 법선 하한 `minimum_front_normal_z` 0.80 → 0.90(앞면·윗면 혼합 평면 제외) | 커밋 `6f4764f` | 파이 설치본 해시 일치, 관측기 재시작 |
| 규칙 | 파이 lim linger 전제 | 커밋 `523bec1` | 사용자 `loginctl enable-linger lim` |
| 운영 | Nav2 세션 비합성(`--use-composition false`), LOCALHOST | `jdamr_depart.py` `SESSION_COMPOSITION` | 10:59 이후 모든 세션 |
| 운영 도구 | `jdamr_depart.py`(RViz 클릭 → 스캔 정합 → 활성화 → teach-home, `go`·`stop`·`status`·세션·표시 제어), `scan_match.py`, `dock_survey.py`, `capture_scan2.py`, `initialpose_capture.py`, `geometry_check.py` | 데이터 폴더 `tools/`, 저장소 사본 `scripts/phase2_20260930/` | PC에서 실행, 파이는 ssh |
| 지도 자산 | Phase 2 keepout(표 1 앞 띠 개방, `keepout_provenance.json`), home_dock 재교시(10:24, (−0.608, −0.843, 92.3°)), table_01 경로 3종 | 데이터 폴더 | registry 해시 |
| 지도 자산 | `map_dockfix.yaml`(흔적 3칸), `keepout_dockfix.yaml`(46칸 해제), registry 교체 | §16.3, `dockfix_provenance.json` | 11:57 세션부터 |
| 기록 | 이 §16·§17, 포트폴리오 인계, 그림 3장, 지도·박스 설치 계획 | `evaluation/`, `portfolio_data/figures/` | — |

### 17.3 기록 부재

- 이날 어떤 실행도 rosbag을 남기지 않았다. 도킹 구간의 cmd_vel·odom 궤적, Nav2 계획 경로, 카메라 영상이 없다.
- 도크 최종 자세는 내부 추정(TF·AMCL·스캔 정합)과 사용자 육안 확인뿐이다. 줄자·외부 촬영 실측이 없다.
- 박스 관측 실패 run은 최근 status 16개 증거 이벤트만 있고 RGB-D 원본 프레임은 없다.

### 17.4 정리

12:23에 PC 표시 유닛 5개(`jdamr-p2-mapserver`·`relay`·`markers`·`rviz`·`click`)와 파이 Nav2 세션(`jdamr-restaurant-navigation`)을 `jdamr_depart.py display-stop`·`session-stop`으로 끄고 inactive를 확인했다. 파이에는 상시 센서 서비스 `jdamr-base`·`jdamr-box-rgbd`·`jdamr-box-observer`만 남았다(이 작업이 띄운 것이 아니다). 같은 지도로 다시 주행한다면 `display-start` → `session-start` → 도크에서 RViz 초기화 → `init --keep-home` 순서다(표시는 Nav2 세션 기동 전에 띄운다). 새 지도로 바꾸면 설치 계획의 등록 절차를 따른다.

## 18. 새 지도, 목적지 재지정, 물 받는 곳 경유 흐름 (2026-09-30 14:10–14:50)

데이터: `$HOME/jdamr_data/map_20260930_manual/` (파이 같은 경로). 로봇 이동은 사용자 수동 매핑 주행뿐이다.

### 18.1 지도와 도크

| 항목 | 내용 | 근거 |
|---|---|---|
| 매핑 | 파이 `jdamr-operator-mapping.service`(Cartographer 2D + map_saver + 웹 조종기 8080)로 사용자가 14:10–14:23 주행. 14:23:47 `/finish_trajectory` 뒤 저장해 이후 스캔을 넣지 않았다 | `map_provenance.json`, `map.pbstream` |
| 매핑 중 표시 | 기존 중계는 `/map`을 옮기지 않아, 설치본 중계에 `/map`만 더한 임시 스크립트를 ssh 표준입력으로 실행하고 PC RViz를 띄웠다(파이 설치·빌드 없음) | `scripts/map_20260930_manual/mapping_display_relay.py` |
| 도크 | 사용자 지정("마지막 차량 자리 부근을 충전소로"). 정지 스캔 정합 inlier 0.80, 전역 탐색 일치, Cartographer 추정과 0.7 cm·0.75°. 도크 차체 영역의 점유·미확인 0칸, 0.7 m 대기점 회전 여유 0.67 m | `dock_pose.json` |
| 벽 정렬 | 사용자 요청으로 벽 방향(원 지도 축 −5.7°, 두 방법 일치)에 맞춰 원점 기준 +5.7° 회전한 격자 `aligned/map.yaml`(최근접 재표본, 벽 잔여 0.0°). 도크 재정합 (−0.185, 0.191) inlier 0.72. 도크 목표 방향은 사용자 요청으로 0°(실제 정지 6.7°) | `aligned/alignment.json`, `aligned/dock_pose.json` |
| keepout | 지도 미확인 칸만 막았다(22,485칸). 세션의 비어 있지 않은 마스크 조건을 만족하고 KeepoutFilter가 미확인을 빈 칸으로 덮는 문제(§14.1)를 막는다 | `aligned/keepout.yaml`, `keepout_mask validate` 통과 |

### 18.2 목적지와 배치 검사

RViz 2D Pose Estimate 클릭(화살표 = 박스 앞면 바깥 법선, 지도 축 0/90/180/270°로 고정)으로 지정했다. 박스 0.40×0.30 m 가정, 회전 여유 기준 0.514 m.

| 이름 | 앞면 가운데(정렬 지도) | 법선 | 결과 |
|---|---|---|---|
| `water_station` (P2) | (−0.145, −1.531) | 0°(동쪽에서 접근) | 통과 |
| `table_01` (P1) | (1.496, −1.487) | 180°(서쪽에서 접근) | 통과. 두 박스가 1.64 m 간격으로 마주 본다 |
| `table_02` (P3) | 클릭 (0.518, −3.578) | 90°(북쪽에서 접근) | 불통과: 박스 자리에 지도 점유 17칸, 접근 자세 회전 여유 0.37–0.46 m. 제안 자리 (0.62, −2.83) 북향(북쪽 75 cm·동쪽 10 cm)은 통과. 사용자 확인 전 |

경로는 앞면 법선 위 한 직선에 둔다(사전 1.10 m, 관측 0.60 m, 간격 0.50 m. 이후 `jdamr_depart.py`에서 사전 1.00 m·관측 0.58 m·간격 0.42 m로 바꿨고, 간격은 BT 목표 허용 0.15 m보다 크다). 박스 세 개를 넣은 지도에서 회전 지점 13곳 모두 0.514 m 이상, 세 접근 통로와 도크 후진 통로는 `static_corridor_clear` 통과. `water_station` 경로는 도크에서, 테이블 경로는 `water_station` 이탈 자세(앞면 + 0.565 m)에서 시작한다. `table_01`의 사전 지점은 이탈 자세와 5 cm라 곧바로 끝나고 관측 지점(0.48 m 앞)에서 방향이 맞춰진다.

### 18.3 새 흐름 (커밋 `a34b230`)

- 사용자 결정: 웹 호출은 만들지 않고 사용자의 "출발 table_0N"을 호출로 본다. 흐름은 도크 → `water_station` 정밀 주차·5 s·후진 이탈 → 호출 테이블 정밀 주차·5 s·후진 이탈 → 도크 후면 도킹.
- `box_service --via-id water_station --via-route --via-region-xy [--via-region-radius-m]`: 경유지를 기존 `visit_observed_box`로 방문하고 `dwell_and_leave`(검증된 정지 유지 뒤 `_leave_parked_pose` 이탈)를 거쳐 테이블 방문과 기존 `dwell_and_return_home`으로 이어진다. 경유 인자는 함께만 받고 `--return-home`이 필요하며 관측 지점 재개와 함께 쓰지 않는다. 경유 실패·이탈 실패면 테이블로 가지 않는다.
- 테스트 T36 7건은 수정 전 모두 실패, 수정 뒤 통과. 핵심 7개 파일 605건 통과, 변경 파일 flake8·pep257 0건.
- 파이 반영: `scripts/deploy_navigation_to_pi.sh`는 파이 사전 검증에서 멈췄다(검증 사본에 `jdamr_cube_description`이 없어 `new_base_geometry.yaml`을 읽는 기존 테스트 2건 실패, 주 워크스페이스 미변경). 백업(`$HOME/jdamr_data/deploy_backup_20260930_via/`) 뒤 `box_service.py` 한 파일을 복사하고 `colcon build --packages-select jdamr_cube_navigation` 통과. PC·파이 소스·파이 설치본 SHA-256 `7f954a6c…` 일치, `--help`에 경유 옵션 표시.
- 출발 도구: `$HOME/jdamr_data/map_20260930_manual/tools/jdamr_depart.py`(저장소 사본 `scripts/map_20260930_manual/`). 새 지도 폴더, `destinations.json`, 세 경로 생성, init 때 세 목적지 영역 등록, `go table_0N`에 경유 인자 추가. 14:49 새 registry로 정밀 준비 세션 기동(localization·keepout active) 확인 뒤 종료.

### 18.4 최종 배치 (14:55, 사용자 조정)

사용자 요청: "워터스테이션은 조금 위로, 테이블1은 내릴 수 있을 만큼 내리고, 테이블2는 제안한 거로", "주행 중 멈추지 않을 정도로". 세 목적지를 함께 넣은 검사기(`scripts/map_20260930_manual/layout_eval.py`: 박스 자리, 박스 간격 0.30 m, 회전 지점 13곳 0.514 m, 직선 접근 통로, 도크 후진 통로, 0.30 m 측면 여유 연결성)로 정했다.

| 이름 | 앞면 가운데 | 법선 | 조정과 한계 |
|---|---|---|---|
| `water_station` | (−0.145, −1.331) | 0° | 북쪽 0.20 m. +0.30 m까지 통과, +0.40 m 접근 통로 차단 |
| `table_01` | (1.596, −1.687) | 180° | 남쪽 0.20 m(−0.35 m부터 사전 지점 회전 여유 부족), 동쪽 0.10 m. 두 박스 앞면 간격 1.64 → 1.74 m로 넓혀 서로의 사전 지점 여유를 0.03 m에서 0.13 m로 늘렸다 |
| `table_02` | (0.67, −2.83) | 90° | 제안 자리에서 동쪽 0.05 m(서쪽 설치 오차 대비) |

설치 오차 ±0.10 m(9방향 × 3곳) 검사에서 남은 실패는 `water_station`을 서쪽으로 0.10 m 놓을 때(뒤쪽 지도 물체와 0.10 m 이내)와 `table_02`를 서쪽으로 0.10 m 놓을 때(0.510 m < 0.514 m)뿐이다. 카메라 시야 1.5 m 안 점유 칸: `water_station` 19칸(최근접 1.20 m), `table_01` 15칸(1.28 m), `table_02` 0칸. 경로·주석·표시를 다시 만들고 파이에 동기화했다.

### 18.5 남은 항목

- 박스 세 개 설치, 로봇을 도크에 벽과 나란히 두고 `display-start` → `session-start` → RViz 초기화 → `init`(teach-home). 그 뒤 "출발 table_0N"에 `go table_0N`.
- 실차 미검증: 두 박스가 마주 본 1.64 m 공간에서의 회전 탐색, 테이블 경로 첫 사전 지점 즉시 완료, 도크 끝 yaw(§16.4)는 첫 주행에서 본다.
- 표시: `service_visualization`은 테이블 두 곳만 그리고 `water_station`은 그리지 않는다.

## 19. 새 지도 첫 "2번 출발"과 반복 중단 정리 (2026-09-30 15:02–16:24)

데이터: `$HOME/jdamr_data/map_20260930_manual/aligned/runs/` (파이 원본을 PC로 복사). 모든 실행은 사용자의 "2번 출발"(물 받는 곳 경유 → table_02 → 도크)과 이어가기 요청 범위다. 정확도 값은 모두 내부 추정(AMCL·TF)이고 외부 실측은 없다.

### 19.1 실행 대장

| 시작 | 실행 | 끝난 단계 | 중단 사유(로그 근거) | 조치 |
|---|---|---|---|---|
| 15:02 | `table_02_150238` 경유 포함 | 물 받는 곳 최종 접근 | 첫 관측에서 앞면 미검출 → 탐색 회전 1회 뒤 검출, 면 정렬 완료. 최종 접근이 목표 8 cm 앞에서 `nav2_error_code` 105(진행 검사). Parking 최저 0.01 m/s에 SlowdownZone 0.6 감속이 겹쳐 0.004 m/s로 진행 요구(0.05 m/10 s) 미달 | `5449400` 정밀 세션 SlowdownZone 끔, 최종 접근 최저 속도 0.02 m/s |
| 15:22 | `table_02_152226` 경유 포함 | 물 받는 곳 탐색 회전 | 요청 30° 회전에 관측 38.2°(변위 1.5 cm), 확인 허용 ±5° 초과로 `nav2_spin_failed` | `33e960f` 확인 허용 ±15° |
| 15:27 | `home_152750` 도크 복귀 | 대기점 이동 중 사용자 중단 | Claude가 물 받는 곳 정밀 주차·대기와 테이블 방문을 마치지 않은 채 복귀를 시작했다. 사용자 지적("아직 물 받는 데 앞인데") 뒤 즉시 취소(`operator_or_timeout`) | 남은 정차를 건너뛰지 않는다 |
| 15:29 | `table_02_152914` 경유 포함 | 물 받는 곳 사전 지점 | 박스 앞에 남은 자리에서 제자리 회전이 105로 멈춤(회전 영역 안 박스). BackUp 서버가 없어 사용자가 로봇을 뒤로 옮김 | `box_escape_once.py`(실행기 후진 이탈만 실행) 작성 |
| 15:38 | `table_02_153849` 경유 포함 | 물 받는 곳 최종 접근 | `AMCL x covariance high: value=0.010 limit=0.010 (bound=strict)`로 중단 | 사용자 지시("공분산이나 한도 쓰지 말라")로 `87ff341` 운영 계약 공분산 정지 6항목을 1e6으로(사실상 끔) |
| 15:44 | `escape_154451` 이탈 단독 | 후진 완료, 확인 실패 | 앞면 0.512 m(기준 0.515), 위치 5.3 cm, 방향 2.9° | 16:23 `e32cc3a`(19.2) |
| 15:46 | `table_02_154603` 경유 포함 | 물 받는 곳 최종 접근 도착 | 위치 오차 0.65 cm, 방향 1.83° > 당시 허용 1°로 `parking_not_confirmed` | `86581fb` 방향 허용 3°(계약 상한). 실시간 파라미터를 계약과 대조하므로 세션 재시작 |
| 15:51 | `water_dwell_escape_155149` | 대기 시작 직후 | 새 프로세스에는 최종 명령이 없어 `wait_parked`가 `parking odometry or final command missing` | 정지 자세 두 번 캡처(5 s 간격, 2 cm 이내)로 대기 확인. `b3687d9`의 `--resume-parked-from-log`에 반영 |
| 15:52 | `water_dwell_escape_155235` | 대기 완료(이동 0.0 m), 후진 완료, 확인 실패 | 앞면 0.526 m로 충분히 물러났는데 방향 6.0° > 3°로 `box_escape_not_confirmed` | 16:23 `e32cc3a`(19.2) |
| 15:53, 15:54 | `table_02_155334`, `155424` 테이블만 | 시작 직후 | `live map data unavailable: map`(실행기 대기 10 s) | 19.3 |
| 16:00 | `table_02_160052` 테이블만 | table_02 사전 지점 이동 13 s | 104. 파이 journal 16:01:23에 RPP `detected collision ahead!` 반복과 collision_monitor `Robot to stop due to StopZone polygon`이 같은 순간에 나옴. 앞 물체는 기록이 없어 특정하지 못했다 | 미해결(19.5) |
| 16:03 | `table_02_160312` 테이블만 | 출발 전 | `battery_departure_reserve` 10.784 V < 출발 하한 10.8 V | 사용자가 충전소로 옮겨 충전 |

초기화(`init`)는 14:57–15:59에 13회 시도해 5회 활성화했다(14:57, 15:21:28, 15:37, 15:50, 15:59:49). 실패 중 사유가 남은 것은 15:32 정합 inlier 0.19(로봇을 옮긴 직후), 15:33 활성화 중 `map_server lifecycle service unavailable`, 15:56 스캔 0개, 15:59:21 전역 탐색 불일치다. 15:14·15:15(스캔 40개, 정합 결과 없음)와 15:20·15:20:56(정합 0.49·0.48 통과, 활성화 기록 없음)은 도구 출력이 남지 않아 사유를 모른다. 15:37·15:59:49는 `--local-only`(inlier 0.43·0.38, 사용자 배치 기준)로 활성화했다.

도달한 것: 물 받는 곳 5 cm 정밀 주차 한 번(15:46, 내부 추정 0.65 cm·1.83°, 외부 실측 없음)과 그 자리 5 s 대기(이동 0.0 m)·후진 이탈 동작. 테이블 정밀 주차와 도크 복귀를 포함한 "2번 출발" 한 사이클은 아직 끝까지 가지 못했다.

### 19.2 후진 이탈 확인 (커밋 `e32cc3a`)

- 두 번의 이탈 모두 후진 자체는 Nav2 성공(상태 4)인데 확인 단계가 막았다. 확인은 위치 5 cm·방향 3°·앞면 거리 0.515 m 이상을 모두 요구했다.
- 이탈의 목적은 다음 회전 여유다(`ESCAPE_CLEARANCE_M` 0.565 m ≥ 회전 정지 영역 최대 반지름 + 0.1 m, `test_t29`). 이제 앞면 거리만 판정하고 방향·측면 오차는 `box_escape_finished`에 기록만 한다. 다음 구간은 멈춘 자리에서 Nav2가 다시 계획한다.
- T44(방향 6°·측면 4 cm 표류에도 staging으로 이어짐)는 수정 전 실패, 수정 뒤 통과. 이탈 전 정렬 조건(앞면을 3° 안으로 바라봄)과 앞면 거리 미달 시 중단(`test_m3_escape_not_confirmed_never_stages`)은 그대로다.

### 19.3 새 프로세스의 DDS 수신 실패

관찰:
- 실행 중인 베이스·세션과 무관하게, 새로 띄운 프로세스가 기존 노드의 데이터를 받지 못한 경우가 이어졌다. robot_state_publisher TF 없음, map_server 수명주기 서비스 없음, `/map` 없음, 스캔 0개.
- 매번 `jdamr-base` 재시작이나 세션 재시작으로 풀렸다. 당시 파이 부하 평균은 9–15였다.

확인한 것:
- 참가자 수 가설(LOCALHOST 모드의 `maxInitialPeersRange=32`)은 기각했다. 세션과 표시 중계를 모두 띄운 16:10 측정에서 참가자 21개(id 0–20)였다. 번호를 고정해도 풀리지 않는다.
- 한가한 상태에서 새 프로세스 18회(SIGKILL 뒤 포함)는 모두 지도·TF·스캔을 받았다. 재현하지 못했으므로 원인은 확정하지 않았다.

조치(원인 대신 증상 대응):
- `e82d45c` 실행기의 지도 수신 대기를 10 s → 30 s. 한가할 때도 1–6 s가 걸렸고 주행 중에는 10 s를 넘겼다.
- `aa454e8`은 `go`가 출발 직전 `dds_probe.py`로 수신을 확인하고, 실패하면 `recover`를 거친 뒤 출발했다. 코드 리뷰(19.6)에서 주행 중 재입력 시 세션을 끄는 순서 결함과 저장소 AGENTS.md "새로운 출발 차단 계층을 덧붙이지 않는다"와의 충돌이 나와 바꿨다.
- 현재 방식: 정상 출발에는 추가 확인이 없다. 실행기 첫 확인(`verify_live_maps` 30 s, 이동 명령 전)이 `live map data unavailable`로 끝나고 다른 이벤트가 없을 때만 `recover`를 거쳐 한 번 더 출발한다(1회 한정).
  - `recover` 순서: 사이클 없음·linger 확인 → 위치추정 무효화 → 표시·세션 종료 → `jdamr-base`·`jdamr-box-rgbd`·`jdamr-box-observer` 재시작(시작 시각이 바뀐 것까지 확인) → 표시·세션 기동 → 직전 init 자세로 재초기화(전역 정합 필수) → 수신 점검.
  - 재개 옵션으로 출발했거나 직전 init이 `--local-only`면 자동 복구 대신 `recover --seed X Y YAW_DEG [--local-only]`를 안내하고 멈춘다.
- `health`(수동 점검)는 대기를 실행기와 같은 30 s로 맞췄고 `/keepout_filter_mask`도 본다. 박스 관측 상태는 수만 기록한다.
- 검증(로봇 정지·충전 중): 16:16–16:20 `health` 4회 통과(0.8–4.6 s). 다른 도메인(99)에서 같은 점검은 `MISSING`(당시 대기 20 s). 바뀐 흐름은 ssh를 가짜로 바꾼 `scripts/map_20260930_manual/test_jdamr_depart_flow.py` 22건으로 확인했다. `recover`의 실제 재시작 경로는 아직 실행하지 않았다.

### 19.4 그 밖의 변경

- `c5b066f` 탐색 회전을 영역 방위 쪽으로 먼저 돌림(8° 이상 벗어났을 때, 한 번에 30° 이하, 전체 예산 유지).
- `b3687d9` 중단 지점에서 이어가기: 관측 지점부터 재개가 경유와 함께 첫 정차에만 적용, `--resume-parked-from-log`(로그의 마지막 앞면으로 정지 확인 대기 → 이탈 → 경유면 테이블, 아니면 도크).
- `8c5b92d` RViz 서비스 표지에 `water_station` 영역(청록)을 그린다(§18.5의 표시 항목 해결).
- 배포: 모든 파이 반영은 백업(`$HOME/jdamr_data/deploy_backup_20260930_*`) → 파일 복사 → `colcon build --packages-select jdamr_cube_navigation` → PC·파이 소스·설치본 SHA-256 대조 순서다. `scripts/deploy_navigation_to_pi.sh`는 §18.3의 사전 검증 문제로 쓰지 않았다.

### 19.5 남은 항목

- "2번 출발" 한 사이클 끝까지: 물 받는 곳 → table_02 정밀 주차·대기·이탈 → 도크 후면 도킹.
- table_02 사전 지점 이동 중 104: 센서가 정지 영역 안 물체를 보았다. 사람인지, 박스나 지도 물체인지 가리려면 다음 발생 때 로컬 코스트맵과 스캔을 함께 남겨야 한다.
- 도크 끝 방향 허용 3°와 AMCL 방향 잡음(§16.4).
- DDS 수신 실패의 원인. 부하 상태 재현과 `recover` 실제 경로(지도 미수신 뒤 자동 복구·재출발)는 다음 주행에서 본다.
- 새로 추가한 조건: 이번 판부터 `go`는 완료된 init이 있어야 출발한다(`state.json`의 `localized`). 충전소로 옮긴 지금은 `display-start` → `session-start` → RViz 클릭 → `init`이 먼저다.
- 이탈 전 정렬 3°는 그대로다. 새 프로세스 재개(`--resume-parked-log`)에서 AMCL 방향 잡음(§16.4)이 `heading_not_facing_box_face`를 반복시키는지 본다.

### 19.6 코드 리뷰 반영 (충전 중, 로봇 이동 없음)

`e82d45c`·`e32cc3a`·`aa454e8`을 별도 리뷰 에이전트가 검토했다(REQUEST CHANGES, HIGH 1·MEDIUM 6·LOW 13·열린 질문 2). 앞의 두 커밋은 승인이었다.

| 항목 | 지적 | 처리 |
|---|---|---|
| HIGH | 사이클 주행 중 `go` 재입력 시 탐지 실패 → `recover`가 세션을 끄고 베이스를 재시작. 단독 `recover`·`session-stop`에는 사이클 확인이 없음 | `go`·`recover`·`session-stop` 맨 앞에서 `jdamr-table-cycle-*` 확인, 실행 중이면 아무것도 끄지 않고 멈춤 |
| M1 | init·recover가 실패해도 이전 `regions`로 다음 `go`가 출발 | `localized` 표지: init 시작·세션 종료·새 세션 기동 때 false, init 완료 때만 true. `go`가 요구 |
| M2 | 재개 경로에서 자동 복구가 전역 정합으로 늘 실패, 안내 문구 없음 | 재개 옵션·`--local-only` init이면 자동 복구 대신 안내 후 멈춤. init 실패 시 안내 출력 |
| M3 | 복구가 `jdamr-base`만 재시작, linger 미확인, 탐지에 keepout 없음 | 센서 서비스 3개 재시작, linger가 yes가 아니면 멈춤, 탐지에 keepout 추가 |
| M4 | `recover --seed`가 전역 정합 생략을 강제 | `--local-only`를 따로 줄 때만 생략 |
| M5 | 출발 직전 탐지·자동 복구가 저장소 규칙("새로운 출발 차단 계층")과 충돌 | 출발 전 탐지 제거. 실행기 첫 확인 실패 때만 복구·재출발(19.3) |
| M6 | `box_escape_once.py` face 모드가 법선을 검증·정규화하지 않음(\|n\| 1.118이면 정렬 허용 약 26.7°) | \|n\|이 1±0.01이 아니면 거부, 정규화해 사용 |
| L1·L2 | 탐지 실행 불가와 MISSING 미구분, `' ok '` 문자열 파싱 | ok·missing·error 구분(ssh 시간 초과는 error), 세 번째 토큰으로 판정 |
| L3 | 재시작 실패가 active로 보임 | `ActiveEnterTimestampMonotonic`이 바뀌어야 통과 |
| L4 | 표시 유닛 일부만 떠 있으면 나머지를 띄우지 않음 | 없는 유닛만 기동 |
| L5·L6 | 탐지에 keepout 없음, 탐지 20 s와 실행기 30 s 불일치 | keepout 추가, 30 s로 맞춤 |
| L7 | `restaurant_service`의 `visit`·`go_home`이 기한을 먼저 잡아 최대 30 s 지도 대기가 예산을 씀 | 대기 동안 기한 해제, 대기 뒤 설정. 테스트 2건(수정 전 실패·수정 뒤 통과). 파이 반영(백업 `deploy_backup_20260930_legbudget`, PC·파이 소스·설치본 SHA-256 `ee624af8…` 일치) |
| L8 | 새 프로세스 재개에서 이탈 전 3°가 AMCL 방향 잡음에 걸릴 수 있음 | 설계대로 둠, 남은 항목(19.5) |
| L9 | 큰 표류·경로 시작점 검사 연동 테스트 없음 | 추가하지 않음. 경로 시작점 0.3 m 검사는 `test_box_service`의 `starting place` 테스트가 다루고, 이탈 뒤 표류는 기록만 하는 설계다 |
| L10 | 주석·문서의 경로 간격이 옛 값 | 주석과 §18.2 갱신 |
| L11 | `--resume-parked-log` 경로 미인용, `--skip-via`에도 물 받는 곳 경유 문구 | `shlex.quote`, 문구 분기 |
| L12 | `box_escape_once.py`에 SIGTERM·SIGHUP 처리 없음, 정류지 id 고정 | 두 신호와 SIGINT에서 정지 요청, `--stop-id` |
| L13 | PC 로컬 설치본이 옛 이탈 규칙 | PC 재빌드, 소스·설치본 해시 일치 |
| 열린 질문 1 | 이탈 뒤 큰 AMCL 도약을 잡을 느슨한 상한 | 추가하지 않음. 사용자 지시("공분산이나 한도 쓰지 말라")와 충돌 |
| 열린 질문 2 | 출발 직전 탐지 참가자의 입퇴장이 교착 방아쇠일 가능성 | 출발 경로에서 탐지를 없앴다. `health`·`recover`에서만 돈다 |

검증: 패키지 테스트 2347 통과. 실패 28·오류 9는 frontier·AMCL 평가·G006·저장소 전체 린트 파일로, 같은 파일을 HEAD 임시 worktree에서 돌려도 같은 수가 나와 이번 변경과 무관하다. 수정 파일 `ament_flake8`·`ament_pep257` 문제 없음. 스크립트는 flake8 F·E9 통과.

### 19.7 저녁 "2번 출발"과 후진 문제 (2026-09-30 16:56–18:20)

데이터: `$HOME/jdamr_data/map_20260930_manual/aligned/runs/` (`table_02_20260930_1656*`–`1736*`, `home_20260930_173307`, `go_table_02_*_monitor.log`). 정확도 값은 모두 내부 추정이고 외부 실측은 없다.

| 시작 | 실행 | 결과 | 원인(근거) | 조치 |
|---|---|---|---|---|
| 16:56 | 도크 출발 | 이동 전 `live map data unavailable: map` | 실행기 시작 12 s 뒤 16:57:00 lifecycle_manager `controller_server IS DOWN after not receiving a heartbeat for 10000 ms` → 스택 종료. 15:54에도 같은 기록. 파이 UDP `RcvbufErrors` 누적 3,664,856(수신 데이터그램의 약 10 %), `rmem_max` 212992 | 파이 `/etc/sysctl.d/60-ros2-dds-udp.conf`(rmem_default 4 MB, rmem_max 16 MB), `f4f375d` bond_timeout 0. 이후 주행 중 폐기 0 |
| 16:57 | 자동 복구 | 재초기화 활성화 실패(`navigation rollback unconfirmed`) | 기동 요청 45 s 무응답(부하 평균 8–10) | 17:07 버퍼 적용 뒤 `recover` 성공 |
| 17:11 | 도크 출발 | 물 받는 곳 5 cm 주차(0.31 cm, 1.39°), 5 s 대기. 이탈 0.526 m에서 105 | 아래 후진 문제 | 그 자리에서 `--skip-via`로 이어감 |
| 17:15 | table_02 | 이동 성공(16:00의 104 구간 통과). 관측 13회 실패로 탐색 소진 | 라이다 앞면 직선(지지점 45–51, 잔차 1.3 mm, 방향 차 약 1°)이 깊이 앞면보다 일정하게 2.4–2.7 cm 뒤. 같은 날 통과 관측 8건은 −0.43~+0.19 cm | `5bd2708` 2–3.5 cm 차이는 가까운 면 기준 |
| 17:28 | table_02 관측 지점부터 | 5 cm 주차(중앙 5.7 cm, 2.74°) 뒤 `box_gap_not_confirmed` | 모서리 4.4·7.0 cm가 ±1 cm 밖. 방향 허용을 3°로 넓힐 때(`86581fb`) 모서리 기준을 그대로 둔 결함 | `665f891` 중앙 ±1 cm와 계약 방향으로 판정, 모서리는 기록 |
| 17:30 | 주차 자리 재개 | 5 s 대기 뒤 이탈 0.554 m에서 105, AMCL 방향 33° 틀어짐 | 아래 후진 문제 | `665f891` 회전 여유(0.515 m)면 이탈 완료·생략 |
| 17:33 | `restaurant_service home` | 이동 전 거부 | 도크 계약 파라미터를 요구하는데 세션은 박스 계약 | `9017be0` `--dock-only`(box_service `--home-only`) |
| 17:36 | 도크 복귀 | 대기점 1.5 cm·1.4°, 후면 도킹 뒤 105. AMCL 방향 18.8° | 사용자 확인: 로봇은 충전기에 평행, 충전 중(12.1–12.2 V). 18:13 스캔 정합 3.5°·AMCL 3.1°. AMCL 방향 오차 약 15° | 아래 |

후진 문제(세 번 모두 105): 목표 몇 cm 앞에서 좌우로 흔들리다 진행 검사에 걸렸다(사용자 관찰: 후진·회전 중 좌우 왕복). 확정 사실은 AMCL 방향이 도크 근처에서 약 15° 틀렸다는 것이다. RPP 끝단에서 추종점 거리가 줄어 곡률이 폭증하는 현상이 이를 키웠다는 것은 가설이다. `c1770a2` 주차·후진 RPP에 `use_fixed_curvature_lookahead`·`curvature_lookahead_dist` 0.6·`interpolate_curvature_after_goal`을 켰다(Nav2 1.3.12 지원 확인, 파이 반영·세션 재시작 적용). 실차 확인은 다음 주행에서 한다.

남은 항목:
- 짧은 직선 후진(이탈 0.45 m, 도킹 마지막 0.7 m)을 odom 좌표 경로로 보내 AMCL 방향 튐의 영향을 없애는 방안. 도킹 확인이 AMCL에 기대는 부분(물리적으로 도킹됐는데 105)도 같이 다뤄야 한다.
- 도크·박스 근처 AMCL 방향 오차(15°)의 원인.
- 출발 순간 발견 트래픽 소켓(104xx 짝수 포트)의 폐기는 버퍼 확대 뒤에도 기동 중에 남았다(주행 중에는 0).

### 19.8 odom 고정 후진·직선 최종 접근과 table_02 무중단 구간 (2026-09-30 18:25–18:52)

사용자 확인: 17:39 도킹 뒤 로봇은 충전기에 평행했고(충전 12.1–12.2 V) RViz만 삐뚤게 보였다. 18:13 스캔 정합 3.5°·AMCL 3.1°로, 도킹 직후 AMCL 18.8°는 약 15° 방향 오차였다. 18:32·18:41 최종 접근은 11°·23° 기울어진 채 도착해 RPP 충돌 예상으로 104가 났다(사용자 확인: 두 번 모두 기울어진 채 정밀 주차).

| 커밋 | 내용 |
|---|---|
| `64bb05c` | 박스 이탈·도킹 마지막 후진: 지도에서 만들고 검증한 경로를 보내기 직전의 map-odom 관계로 odom에 고정해 보낸다. 정밀 도킹은 대기점을 지도에서, 마지막 구간을 같은 도크 계약으로 odom에서 확인 |
| `f9af87f` | `c1770a2` 곡률 설정 되돌림. 켠 뒤 첫 최종 접근이 앞면 2.5 cm까지 들어갔다. (정정 2026-09-30 20:10: 커밋 메시지의 "속도 조절용 곡률에만 쓰인다"는 틀렸다. Nav2 1.3.12 소스에서 켜면 조향 곡률에도 쓰인다. 되돌린 근거는 켠 직후 접근이 더 들어간 시점 대응뿐이고, 원인은 확인하지 않았다) |
| `093f559` | odom 고정 이탈의 회전 여유를 odom에서 판정(18:37 AMCL 0.5146 m로 0.4 mm 미달 판정) |
| `a681f62` | 최종 접근: 면 정렬 방향을 유지한 채 앞면 간격 5 cm까지 Parking 제어기로 odom 고정 직선 경로. 간격·방향 판정도 odom, 대기는 도착 자세 유지. 이탈 전 정면 조건 15° |

18:47 table_02 관측 지점부터(`table_02_20260930_184711`) 한 실행으로 끝까지 갔다(`parked=True dwell=True escape=True home=True`). 모두 내부 추정, 외부 실측 없음.

| 단계 | 결과 |
|---|---|
| 직선 최종 접근 | 이동 0.397 m, 앞면 대비 방향 1.1°. 정지 확인 0.42 cm·0.17°(odom) |
| 간격 | 중앙 5.42 cm, 모서리 5.83·5.02 cm, 앞면 방향 0.87° |
| 5 s 대기 | 이동 0 |
| 후진 이탈 | Nav2 성공, 앞면 거리 0.520 m(odom) |
| 도크 대기점 | 3.57 cm·0.78°(AMCL), 이동 2분 7초 |
| 후면 도킹 | Nav2 성공, 4.44 cm·1.2°(odom, 도크 계약 안) |

- 같은 날 물 받는 곳은 18:37 odom 직선 이탈이 Nav2 성공(끝 방향 0.28°)이었다. 그 전 최종 접근은 곡률 설정 상태의 104였다.
- 충전기는 사용자가 연결하는 방식이다("충전소"는 위치 이름). 도킹 뒤 전압(18:52 11.4 V대)은 도킹 판정에 쓰지 않는다.

남은 항목:
- 물 받는 곳부터 도크까지 새 방식으로 끊김 없는 한 사이클.
- 도크 대기점 이동 2분 7초(평소보다 길다), 사용자가 본 앞뒤 왕복과 같은 구간인지.

### 19.9 퇴근 전 정리 (2026-09-30 18:52–)

- `b5edc5a` 사용자 요청으로 이동 속도 +50 %(0.08→0.12 m/s)와 이동 감속 완화. 바꾼 값은 최저 접근 0.075 m/s, 곡선 감속 반경 0.6 m, 목표 앞 감속 0.4 m, 비용 감속 0.2 m, SlowdownZone 비율 0.8이다. 주차·후진 제어기는 0.04 m/s와 기존 감속값(0.6·0.9·0.3)을 명시해 유지했다. 회전 0.3 rad/s는 탐색 회전 넘침(15:22 30°에 38.2°) 때문에 그대로 두었다. 파이 반영 완료, 다음 세션 기동부터 적용, 실차 확인 전.
- 도크 대기점 앞 큰 회전(사용자 관찰: 한쪽으로 조금만 돌면 되는데 반대로 크게 돈다): 18:48:54–18:51:01(2분 7초) 동안 controller_server가 `Passing new path to controller`를 113회 남겼다. 내비게이터가 약 1초마다 경로를 새로 계획했다는 기록이다. AMCL 자세가 튈 때마다 경로의 돌아 들어가는 쪽이 바뀌어 회전 방향이 뒤집혔다는 것은 가설이고, 경로를 기록하지 않아 확인하지 못했다. 개선안은 두 가지다. (1) 대기점 구간도 한 번 계획한 경로를 odom에 고정해 FollowPath로 보내기, (2) 마지막 방향 맞춤은 정지 자세에서 계산한 최단 각도로 Spin 한 번. 다음 주행에서 `/plan`·`/cmd_vel`·`/odom`·`/amcl_pose`를 기록해 확인한 뒤 적용한다.
- 보존: 파이 run 로그 전체를 PC `aligned/runs/`로 회수했다. Nav2 세션 journal 16:50–18:55와 실행 유닛 journal 14개는 `aligned/runs/journal_20260930_evening/`에 있다.
- 포트폴리오 데이터 점검: `20260930_EVENING_E2E_PORTFOLIO_NOTES.md`.
- 주행 기록기: 파이 디스크가 98 %(여유 844 MB, `~/jdamr_data/vslam` 14 GB·`~/jdamr_artifacts` 2.5 GB)라 파이에서 기록하지 않는다. 표시 중계가 이미 PC로 넘기는 `/tf`·`/tf_static`·`/scan`·`/plan`·`/amcl_pose`를 PC에서 mcap으로 기록한다(`display-start`가 켜고 `display-stop`이 끈다, `record-start`·`record-stop`도 있음). `/tf`에 odom 궤적과 AMCL 보정이 들어 있다. 중계는 설계상 명령 토픽을 싣지 않으므로 `/cmd_vel`은 없다. 19:06 중계만 켠 시험에서 10 s에 `/scan` 139·`/tf` 235·`/tf_static` 2를 기록했다(시간당 약 120 MB). 도크 대기점 회전 수정은 이 기록으로 원인을 확인한 뒤 한다.
- 파이 디스크 정리(`vslam` 14 GB 보관·삭제)는 사용자 결정 사항이다.
- 코드 리뷰 LOW 반영(출발 도구): 사이클 종료 직후 로그 재읽기, ssh 실패와 사이클 실행 구분, session-start 시간 초과 시 위치추정 무효화, 도크에 닿지 못한 `go`는 종료 코드 1, `recover --local-only`는 `--seed` 필요, 상태 파일 덮어쓰기 방지. `box_escape_once.py`는 대기 중 2 cm 넘게 움직이면 중단, 잘못된 모드 입력은 인자 오류. 모의 흐름 테스트 26건 통과.
- `scripts/deploy_navigation_to_pi.sh`: 검증 사본에 `jdamr_cube_description/config/new_base_geometry.yaml`을 넣었다(§18.3의 테스트 2건 실패 원인). 스크립트는 `jdamr-base`를 멈추고 다시 켜므로 퇴근 전에는 실행하지 않았다. 다음 배포 때 확인한다.

## 20. 2026-10-01 오전 "2번 출발" — 첫 무중단 구간과 막힌 원인 다섯 가지

수치는 모두 로봇 내부 추정(AMCL·odom)이고 외부 실측은 없다. 런 로그는 PC `aligned/runs/table_02_20261001_*`, 궤적은 PC bag `aligned/runs/bag_20261001_092908`·`bag_20261001_095541`(`/tf`·`/scan`·`/plan`·`/amcl_pose`).

### 20.1 결과 (10:12–10:28 실행)

| KST | 구간 | 결과 |
|---|---|---|
| 10:11:54 | 도크 init(등록 도크 자세 기준) | 스캔 정합과 AMCL 0.7 cm·0.3° |
| 10:13:54 | 물 받는 곳 정밀 주차·5 s 대기 | 정지 0.42 cm·0.18°, 간격 5.42 cm(모서리 5.40·5.44), 면 0.04° |
| 10:14:19 | 물 받는 곳 후진 이탈 | Nav2 성공 |
| 10:14:32 | 이탈 → table_02 접근 지점 | Nav2 성공 13 s. 회전을 먼저 하고 출발, 꼬리~박스 면 최소 0.129 m(패딩 포함) |
| 10:15:09 | table_02 최종 접근 | 시작 전 실패(§20.2 4) → 수정 후 관측 위치에서 재개 |
| 10:20:10 | table_02 정밀 주차·5 s 대기 | 정지 0.53 cm·0.23°, 간격 5.53 cm(모서리 6.23·4.83), 면 1.48° |
| 10:20:47 | table_02 후진 이탈 | Nav2 성공 |
| 10:21:37 | 도크 대기점 이동 | Nav2 성공 50 s(9/30 같은 구간 118 s) → 회전에서 실패(§20.2 5) |
| 10:27:09 | 대기점 한 방향 회전 | 필요 −122.8°, 실제 −125.8°(odom), 이동 0.1 cm |
| 10:28:10 | 후면 도킹 | Nav2 성공, 도착 확인 4.6 cm·1.42° |

물 받는 곳 → table_02 → 도크의 모든 단계를 같은 오전에 실차로 지났다. 수정 배포 때문에 세 번의 실행(10:12, 10:19 재개, 10:26 도크만)으로 나뉘었고, 한 실행으로 끊김 없이 끝난 사이클은 아직이다.

### 20.2 막힌 원인과 조치

1. **PC 절전 뒤 Wi-Fi 변경(08:56).** 깨어나며 `aicampus_291_5G`에 붙어 파이(`aicampus_286`)에 닿지 않았다. 파이 문제로 오진해 불필요한 전원 재시작을 요청했다. 그 재부팅으로 임시 유닛이던 `jdamr-box-observer`가 사라져 이전 부팅 journal의 `systemd-run` 명령으로 다시 만들었고, `jdamr-box-rgbd`는 disabled라 수동 시작했다. 조치: PC를 `aicampus_286`으로 되돌림.
2. **Graceful 최종 접근 105(09:43).** Nav2 1.3.12 Graceful은 목표까지 궤적을 costmap으로 충돌 검사하고 끌 수 없다. 박스 5 cm 앞 목표에서 "Collision detected in trajectory" 뒤 105. 로봇은 목표 2.8 cm 안(앞면 약 4.8 cm, 0.14°)까지 갔다. 조치 `311a23d`: RPP `Parking`·`ParkingReverse`를 기본으로, Graceful은 `--graceful-final`.
3. **이탈 뒤 전진 원호로 꼬리가 박스 쪽으로(09:45, 9/30 16:01과 같은 104).** `rotate_to_heading_min_angle` 1.57이라 90° 미만 방향 차이는 제자리 회전 없이 전진하며 돌았다. bag 궤적에서 차체 중심이 박스 쪽으로 7 cm 들어가며 꼬리 여유가 0.405→0.082 m(패딩 포함, 실제 약 10 cm)로 줄었고, RPP 1.5 s 예측과 StopZone 정지 영역(옆 5 cm 띠)이 그 자리에서 진행을 막았다. 실제 접촉은 없었다(사용자 확인). 1.57은 회전을 진행으로 세지 않던 SimpleProgressChecker 시절(9/15 `2d7a7d8`) 값이다. 조치 `311a23d`: 0.785. `1641650`(사용자 지시, 모의 주행이라 박스 접촉 허용): RPP 예측 충돌 검사 끔(FollowPath·Parking·ParkingReverse), StopZone 정지·후진 영역의 옆·뒤를 패딩 footprint로, FootprintApproach 2.0→0.5 s. 남긴 것은 패딩 footprint StopZone, 회전 반경 원(0.43 m 16각형), 정밀 주차 앞면 5 cm, 센서 timeout 1 s.
4. **table_02 최종 접근 시작 전 실패(10:15).** 정렬은 관측 지점에서 본 면 기준 3° 안이었지만 가까이서 다시 본 면과 3.52° 달라 직선 접근 조건(3°)에 걸렸다. 조치 `abb3eef`: 3°를 넘으면 직선 경로를 다시 본 면 법선 방향으로 만들고 Parking이 그 방향으로 돌며 접근.
5. **도크 대기점 회전 AttributeError(10:21).** 한 방향 회전이 박스 탐색 Spin 함수를 공유하는데 그 함수의 `_search_target_yaw`가 탐색에서만 만들어졌다. 탐색을 거치지 않은 재개 실행에서 종료. 단위 테스트는 회전 함수를 모의로 바꿔 잡지 못했고, 테스트 하네스는 생성자를 거치지 않는다. 조치 `50bdc1e`: 생성자에서 초기화, 탐색 목표는 `search_rotation`일 때만 기록.
6. **출발 전 `/cmd_vel` 발행자 2.5 s 탐색 실패(10:18, 10:24, 이동 없음).** 파이 UDP RcvbufErrors가 부팅 직후 20,175, 10:18까지 84,538로 늘었고 새 프로세스의 탐색이 늦었다. 같은 원인으로 09:26 첫 세션 기동 때 keepout lifecycle과 09:36 내비게이션 lifecycle 관리자·planner_server 서비스가 새 프로세스에서 보이지 않아 세션을 다시 띄웠다. 조치 `35a54eb`: 발행자가 아직 안 보일 때만 30 s까지 기다림(다른 발행자는 즉시 거부 유지). UDP 넘침의 근본 원인(어느 소켓·어느 트래픽)은 확인하지 않았다.

### 20.3 9/30 이후 104·105 전체 (Nav2 journal 기준)

| 시각 | 구간 | 코드 | 사유 | 상태 |
|---|---|---|---|---|
| 9/30 15:04 | 최종 접근 | 105 | 진행 없음 | 직선 접근으로 해결 |
| 9/30 15:29 | 물 받는 곳 접근 | 105 | 진행 없음 + StopZone | 원인 미확정 |
| 9/30 16:01 | 물 이탈 → table_02 | 104 | collision ahead + StopZone | §20.2 3 |
| 9/30 17:14·17:31 | 박스 이탈 | 105 | 진행 없음 | odom 고정 이탈로 해결 |
| 9/30 17:39 | 도킹 | 105 | 진행 없음 | odom 도크 레그로 해결 |
| 9/30 18:31·18:41 | 최종 접근 | 104 | collision ahead | 직선 접근으로 해결 |
| 10/01 09:43 | 최종 접근 | 105 | Graceful 궤적 충돌 | §20.2 2 |
| 10/01 09:45 | 물 이탈 → table_02 | 104 | collision ahead ×45 + StopZone | §20.2 3 |

### 20.4 남은 확인

- 한 실행으로 끊김 없는 사이클(이번 수정이 모두 들어간 상태로).
- 파이 UDP 수신 넘침의 원인 트래픽.
- 충돌 검사 완화 뒤 박스 근접 거동(접촉 여부는 영상·사용자 관찰로).
- 3·4번 브랜치 AMCL 비교: 이번 bag 두 개를 `amcl_replay.py`에 넣는다.

### 20.5 파이 UDP 수신 넘침 측정 (2026-10-01 11:20–11:35, 이동 없음)

| 상황 | 측정 | 넘침(RcvbufErrors 증가) |
|---|---|---|
| 정지 상태 60 s | 수신 15,639건 | 0 |
| 새 참가자 접속 3회 | `/cmd_vel` 발행자 발견 0.97–1.91 s, 자기 소켓 대기열 최대 818 KB(버퍼 4 MB) | 0 |
| Nav2 세션 재기동 | 기동 중 부하 평균 6.26 | 0 |
| 부팅 직후(09:15–09:28) | — | 20,175 |
| 09:28–10:18(세션 재기동 3회, 주행 2회, 진단 프로브) | — | 64,363 |
| 10:18 이후 주행 2회 | — | 0 |

- 지금 살아 있는 소켓 중 넘침이 있는 것은 카메라 프로세스(1,355건)뿐이다. 84,538건의 대부분은 이미 종료된 프로세스의 소켓에서 났고, `/proc/net/udp`의 소켓별 계수는 소켓과 함께 사라져 사후에 주인을 가릴 수 없다.
- 오늘 2.5 s 발행자 탐색 실패(10:18, 10:24)는 조용한 상태의 발견 시간 1–2 s와 겹치는 짧은 한도 문제였다. 30 s 대기(`35a54eb`)와 상주 실행기(`4c5adcf`)로 출발마다의 탐색 자체를 없앴다.
- 원인 귀속을 위해 `go`가 실행 동안 파이에서 5 s마다 프로세스별 넘침을 `runs/<run>/udp_drops.jsonl`에 남긴다(`udp_drop_monitor.py`). 다음 주행에서 넘침이 다시 나면 그 파일로 프로세스를 가린다.

### 20.6 사이클 정지 시간과 시간 상수 (오늘 bag·이벤트, 11:40)

정지는 odom 기준 0.5 cm/s·0.02 rad/s 미만이 0.5 s 이상인 구간이다. 10:12 실행은 167 s 중 정지 7회 40 s, 10:19 재개는 116 s 중 5회 21 s, 10:26 도크만은 80 s 중 4회 9 s였다.

| 정지 | 시간 | 정하는 값 | 판단·조치 |
|---|---|---|---|
| 박스 관측(정렬 전·접근 전, 사이클당 4회) | 5.4–7.8 s | 관측기 `processing_hz` 2.0 × 안정 프레임 8 = 최소 4 s | 2 Hz는 9/29 임시 유닛 명령값이고 근거 기록이 없다. 설정값 5 Hz로 되돌림(최소 1.6 s). CPU 관측기 23 → 35 %, 전체 27 → 30 %(4코어). 관측기를 정식 유닛 파일로 설치 |
| 주차 확인 + 5 s 대기 | 8.6 s | 정지 확인 `hold_s` 1.0 s, 대기 5 s(과제) | 유지 |
| 이탈 뒤 다음 출발 | 2.2–3.2 s | 정지 확인 1.0 s + 계획 | 유지(정지 확인이 다음 경로의 출발 자세) |
| 대기점 방향 측정 | 2.0 s | 정지 확인 1.0 s | 유지 |
| 도킹 전·후 확인 | 2–3 s | 정지 확인 1.0 s | 유지(도크 판정 근거) |

상한으로만 쓰이는 시간(실패할 때만 기다림): 발행자 탐색 30 s, 지도 수신 30 s, 정지 포착 10 s, 관측 대기 5 s, 과제 600 s. Nav2: 경로 만료 10 s(대기점·이동 BT), 진행 판정 10 s 안에 5 cm 또는 0.1 rad, 제어 실패 허용 2 s. `session-start`는 재부팅 뒤 꺼져 있는 RGB-D·관측기 유닛을 먼저 켠다.

### 20.7 11:52 한 번의 실행으로 끝난 사이클 (run `table_02_20261001_115239`, bag `bag_20261001_115154`)

§20.2–§20.6 수정(`311a23d`·`1641650`·`abb3eef`·`50bdc1e`·`35a54eb`·`e6b1e3e`·`869b8d0`·`4c5adcf`·`ff85980`, 관측기 5 Hz)이 모두 들어간 첫 주행이다. 상주 실행기로 출발했고 run 이름 시각 11:52:39, 첫 Nav2 목표 수락 11:52:49였다. 시간은 첫 목표 수락부터 센 값이다.

| s | 구간 | 결과 |
|---|---|---|
| 0.1–23.0 | 도크 → 물 받는 곳 관측 지점 | 경유점 3개를 NavigateThroughPoses 한 목표로 23.0 s(오전 목표 3개 44.0 s) |
| 23.0–33.4 | 관측 2.7 s → 면 정렬 4.5 s → 재관측 2.1 s | 정렬 잔차 2.1°라 직선 접근은 로봇 방향 기준(`heading_basis: robot`) |
| 33.4–41.4 | 최종 접근 0.447 m | 8.0 s(오전 22.4–22.7 s) |
| 42.6 | 물 받는 곳 주차 확인, 5 s 대기 | 정지 0.44 cm·0.25°, 간격 5.45 cm(모서리 6.32·4.57), 면 1.86° |
| 50.0–55.9 | 후진 이탈 | 5.9 s(오전 22.3–22.9 s) |
| 59.3–69.5 | table_02 관측 지점 | 경유점 2개 한 목표로 10.3 s(오전 16.5 s) |
| 69.5–83.0 | 관측 2.0 s → 면 정렬 7.5 s → 재관측 2.9 s | 정렬 잔차 1.9°, 로봇 방향 기준 |
| 83.0–91.5 | 최종 접근 0.493 m | 8.5 s |
| 92.7 | table_02 주차 확인, 5 s 대기 | 정지 0.54 cm·0.38°, 간격 5.54 cm(모서리 6.23·4.84), 면 1.47° |
| 100.1–105.8 | 후진 이탈 | 5.7 s |
| 107.6–151.2 | 도크 대기점 이동 | Nav2 성공 43.6 s. 막바지 대기점 5–7 cm 앞에서 좌우 회전 약 14 s(아래) |
| 151.6–175.4 | 대기점 방향 맞춤 | Spin 요청 −132.8° → odom −136.0°, 보정 +3.9° → +7.1°, 정지 확인 |
| 175.4–185.2 | 곡선 후진 도킹 | 정렬 이동 없이 9.8 s(오전 28.4 s), 도착 확인 4.24 cm·1.19° |

- 사이클 186.5 s, 이동 139.7 s. Nav2 목표 10개 모두 error_code 0. 0.5 s 이상 정지 11회 45.0 s(관측 4회 1.7–3.8 s — 이 중 Nav2 결과부터 박스 확정까지 2.0–2.9 s, 주차 확인 + 5 s 대기 8.4·8.7 s, 이탈 뒤 2.2·3.2 s, 대기점 방향 맞춤 전후 2.9·3.2·4.7 s). 오전 세 실행 합은 363.3 s(정지 55.5 s).
- 파이 UDP: 실행 동안 5 s 간격 39회 기록에서 RcvbufErrors 84,538, 카메라 소켓 1,355가 그대로였다(증가 0, `runs/table_02_20261001_115239/udp_drops.jsonl`).
- 면 방향 1.5–1.9°는 정렬 잔차(2.1°·1.9°)가 직선 접근 동안 그대로 남은 값이다. 정렬 허용 안이면 로봇 방향으로 곧게 들어가도록 한 설계(`abb3eef`)의 결과이며, 오전 물 받는 곳 0.04°는 정렬 잔차가 작았던 경우다.

**대기점 앞 좌우 회전(사용자 관찰 "도킹 지점에서 좌우로 회전").** bag에서 대기점 5–7 cm 앞, 우 65° → 좌 105° → 우 35°를 약 14 s 돌았다. 대기점 도착 허용이 5 cm라 남은 몇 cm 경로를 따라야 했고, 그 짧은 경로의 방향이 차체 옆을 가리켜 45°(`rotate_to_heading_min_angle` 0.785) 넘는 차이로 제자리 회전이 반복됐다. 곡선 후진(`869b8d0`, 최소 회전 반경 0.3 m)이 대기점 위치 오차를 흡수하므로 5 cm 도착 정밀도는 필요 없다. 조치 `4621399`: 대기점 도착 허용 15 cm(PositionGoalChecker), 큰 회전 뒤 10° 이하 잔차는 보정 Spin 없이 곡선 후진에 맡김. 파이 반영·세션 재기동·init까지 마쳤고 실차 확인 전이다.

**Spin 초과 회전.** 두 번 모두 요청보다 3.2° 더 돌았다(−132.8° → −136.0°, +3.9° → +7.1°). 원인은 확인하지 않았다. 다음 주행에서는 10° 이하 보정 Spin이 빠져 잔차가 곡선 후진으로 넘어간다.

남은 확인:

- `4621399`: 대기점 앞 좌우 회전이 사라지는지, 15 cm 안의 대기점 오차에서 곡선 후진이 성립하는지(`dock_curve_unavailable` 이벤트가 없어야 함).
- Spin 3.2° 초과 회전의 원인(Spin 정지 판정과 odom 갱신 시점 비교).
- 남은 정지 중 줄일 수 있는 것: 주차 확인 1.0 s, 이탈 뒤 다음 목표까지 2.2–3.2 s, 관측 2.0–2.9 s. 5 s 대기는 과제 조건이다.
- `jdamr_depart.py record-stop`이 이번 bag(`bag_20261001_115154`) 대신 이전 이름 `bag_20261001_095541`을 출력했다. `record_start`가 mcap 크기 4 KB 초과를 30 s 안에 보지 못하면 상태 파일에 새 bag 경로를 남기지 않았기 때문이다(rosbag2는 메시지를 캐시에 모았다가 쓴다). 조치: 기록기 시작 직후 경로를 저장하고 쓰기 확인은 로그로만 남김. 또 115154 bag에 `metadata.yaml`이 없었다(mcap 끝 표식은 정상, 원인 미확인). `ros2 bag reindex`로 만들었다(275.1 s, 메시지 3,109개).
- UDP 넘침이 다시 늘면 `udp_drops.jsonl`로 프로세스를 가린다. 이번 실행에서는 늘지 않았다.
- 포트폴리오 데이터 묶음: `$HOME/jdamr_data/portfolio_20261001/`(지표 `metrics.json`, 궤적·타임라인 그림, run 이벤트 사본, `make_figures.py`, `MANIFEST.sha256`).

### 20.8 IMU 장착 방향 확정과 주행마다 파이 원시 기록 (2026-10-01 12:10–12:45, 이동 없음)

지금까지 주행은 바퀴 odom과 AMCL만 썼고 IMU(`/imu/data_raw`, 50 Hz)는 어디에도 연결되지 않았다. 3·4번 브랜치 분석에서 "자이로 z 부호 반대"만 확인하고 x·y 방향은 정하지 못했다.

| 확인 | 결과 |
|---|---|
| 자이로 z / 바퀴 회전율 기울기 | −0.966 ~ −0.996 (9/15–16 bag 5개) → 보드 z 아래 |
| 보드 y / 바퀴 전진 가속 | +1.15 ~ +1.25, 상관 0.87–0.93 (bag 4개) → 보드 y가 로봇 앞 |
| 보드 x / 바퀴 전진 가속 | −0.06 ~ −0.26, 원심 가속 v·ω에는 4개 모두 양수 → 보드 x가 로봇 왼쪽 |
| 정지 가속도 z | −9.35 ~ −9.45 m/s² |

- 보드는 뒤집힌 채 90° 돌아가 있다(base_link 기준 roll 180°, yaw 90°). 그런데 드라이버는 `frame_id: base_link`로 내고 있었다. z축만 쓰는 회전율 융합에는 영향이 없지만, 가속도나 x·y 회전율을 쓰면 축이 틀린다. 앞선 분석의 정적 TF(roll 180°만)는 x·y가 틀렸다.
- 조치: URDF `imu_joint`(`rpy 3.1416 0 1.5708`, 위치는 재지 않아 0)를 추가했고, 근거는 `new_base_geometry.yaml` `imu_rotation_rpy`에 남겼다. 드라이버 `imu_frame`은 `imu_link`로 바꿨다. 보정 대장도 `esp32_imu_base_link_to_imu_link_v2`로 바꿨다(UNVERIFIED, 융합 불허 유지). 파이에 반영(백업 `$HOME/jdamr_data/deploy_backup_20261001_1236_imu`)한 뒤 `jdamr-base`를 재시작했다. 확인 결과 `/imu/data_raw` frame `imu_link`, TF base_link→imu_link RPY (180°, 0°, 90°), `/odom`·`/imu/data_raw` 50 Hz, `/scan` 9.7 Hz.
- odom+IMU EKF(`imu_bias_relay`, `ekf_odom_imu.yaml`, 오프라인 `ekf_replay.sh`, `rotation_truth.py`)를 메인 브랜치로 가져왔다. 실행 구성에는 아직 넣지 않았다. 9/15 bag 재생 결과(제자리 회전 4건, 스캔 기준 오차 중앙값)는 이전과 같다. odom 0.50°, 자이로 원값 0.40°, EKF 0.10°(최대 0.18°)다.
- 표본을 늘리려고 `go`가 출발 요청 직전 파이에서 `/odom`·`/imu/data_raw`·`/scan`·`/tf`·`/tf_static`·`/amcl_pose`·속도 명령(`/cmd_vel_nav`·`/cmd_vel_smoothed`·`/cmd_vel`)을 기록한다. 유닛은 `jdamr-onboard-bag-<run>`이고 캐시 1 MB, 최대 1시간·400 MB로 묶었다. 사이클이 끝나면 닫고(메타데이터가 없으면 reindex) run 폴더째 PC로 복사한다. 이동 없는 26.5 s 시험에서 IMU·odom 1,326건, 스캔 257건이 기록됐고 static TF에 imu_link가 들어 있었다. 약 8 MB/분이다. 기록기가 첫 메시지를 쓰기까지 약 8 s 걸리므로 출발 전 정지 구간은 짧다. 바이어스는 주행 중 정지 구간(주차 확인·대기)에서 배운다.
- 주행 뒤 분석: `scripts/map_20260930_manual/imu_check.sh <run_dir>` → `runs/<run>/imu_check/`(보드 축, EKF 재생, 정지 구간으로 감싼 회전의 odom·자이로·EKF 오차). 도크 대기점 Spin(약 133°)과 박스 앞 회전이 표본이 된다. 같은 기록의 속도 명령으로 Spin 3.2° 초과 회전도 본다.

남은 확인:

- 다음 주행 bag으로 보드 축이 지금 장착에서도 같은지(`imu_check.sh`의 `axes.jsonl`)와 큰 회전에서 EKF가 odom보다 나은지 확인. 나으면 드라이버 `publish_tf:=false` + EKF가 odom TF를 내도록 연결하고, Nav2가 읽는 `/odom` 속도를 바꿀지는 따로 정한다.
- 보드 위치와 가속도 스케일(정지 0.95 g)은 재지 않았다. 가속도는 융합하지 않는다.

### 20.9 14:42 주행: 대기점 좌우 회전 해소, 도크 끝 방향 4.4°에서 105 (run `table_02_20261001_144234`)

`4621399`(대기점 허용 15 cm·작은 보정 회전 생략)와 §20.8 파이 원시 기록이 들어간 첫 주행이다. 사용자 관찰: 도착까지 깔끔했고, 도착 뒤 앞뒤로 조금 왔다 갔다 했다.

| s (첫 목표 수락 기준) | 구간 | 결과 |
|---|---|---|
| 0.4–23.8 | 도크 → 물 받는 곳 관측 지점 | 23.4 s |
| 26.5–43.6 | 면 정렬 5.1 s, 최종 접근 7.9 s | 정지 0.56 cm·0.34°, 간격 5.56 cm(모서리 5.56·5.56), 면 0.00° |
| 52.2–58.4 | 후진 이탈 | 6.2 s |
| 61.2–92.2 | table_02 관측 10.5 s, 면 정렬 6.2 s, 최종 접근 7.8 s | 정지 0.48 cm·2.38°, 간격 5.49 cm(모서리 4.36·6.61), 면 2.38° |
| 100.8–106.8 | 후진 이탈 | 6.0 s |
| 109.0–137.5 | 도크 대기점 이동 | 28.6 s, 좌우 회전 없음(11:52는 14 s 좌우 회전 포함 43.6 s) |
| 138.9–153.6 | 대기점 Spin | 요청 −121.9°, odom −125.7°. 남은 2.3°는 보정 회전 없이 곡선 후진으로 |
| 156.1–179.4 | 곡선 후진 도킹 | 12.8 s에 도크 위치 0.5 cm 안, 방향 4.4° 남음 → 10 s 앞뒤 왕복 → 105 |

- 도크 위치 도달까지 약 169 s. 주행 중 파이 UDP 넘침 증가 0.
- **105 원인(파이 bag `/odom`·`/cmd_vel_nav`·`/cmd_vel`).** 후진은 0.08 m/s로 경로를 따라 12.8 s에 목표 0.5 cm 안까지 왔다. 방향은 곡선 추종 지연으로 4.4° 남았다(도크 허용 3°). 그 뒤 컨트롤러 명령이 −0.020 ↔ +0.020 m/s로 바뀌며 0.1–0.5 cm 안에서 앞뒤로 움직였다. 명령은 Collision Monitor를 그대로 통과했다(충돌 정지 아님). 10 s 진행 없음으로 105. Nav2 RPP는 `use_rotate_to_heading`과 `allow_reversing`을 함께 켤 수 없어(설치 라이브러리 문구 "Both use_rotate_to_heading and allow_reversing parameter cannot be set to true") 후진 컨트롤러는 목표에서 돌지 못한다. 목표 판정기는 위치와 방향을 같이 요구해 끝나지 않았다. 11:52에는 끝 방향이 1.19°라 이 상황이 생기지 않았다.
- **조치 `b7bc384`.** 도크 후진은 `dock_position_checker`(PositionGoalChecker, 2 cm)로 위치에서 끝낸다. 남은 방향이 도크 허용(3°)을 넘으면 전진 `Parking` 컨트롤러(제자리 회전 가능)에 같은 위치·도크 방향의 두 점 경로를 보내 돌린다. 박스 면 정렬과 같은 방식이고, 종료는 도크 판정기가 허용 안에서 끊는다. 이후 odom 기준 정지 확인. 도크에서 가장 가까운 장애물은 왼쪽 벽 0.448 m라 회전 정지 영역(0.43 m) 밖이다(14:50 스캔). 단위 테스트(4.4°면 회전 1회, 2°면 없음)와 관련 612건 통과. 파이 반영(백업 `$HOME/jdamr_data/deploy_backup_20261001_1453_dockturn`), 세션 재기동·init 완료, 실차 확인 전.
- **Spin 초과 회전 원인(같은 bag).** Spin은 끝까지 0.2 rad/s로 돌았다(감속 기준 `rotational_acc_lim` 1.5 rad/s²라 목표 0.8° 전부터만 감속). 정지 명령이 나간 순간 odom은 이미 목표를 2.6° 지나 있었고(동작 서버 10 Hz·TF 20 Hz·속도 평활기 지연, 약 0.2 s), 정지 명령 뒤 0.23 s 동안 1.3° 더 돌았다. 스캔 정합 기준 실제 회전은 −123.85°라 실제 초과는 약 2.0°다. odom이 3.8°로 보인 것은 바퀴 odom이 회전을 1.5 % 크게 재기 때문이다. 감속 설정은 바꾸지 않았다(대기점 잔차는 곡선 후진이, 도크 방향은 위 제자리 회전이 받는다).
- **IMU(`imu_check.sh`).** 지금 장착에서도 보드 축이 같다. 자이로 z/바퀴 회전율 −0.994, 보드 y/전진 가속 +0.96(상관 0.87), 보드 x는 왼쪽이다. 대기점 Spin 1건을 스캔 정합(잔차 3.0 cm)과 비교하면 오차는 odom 1.88°, 바이어스 뺀 자이로 0.25°, EKF 0.19°다. 이번 자이로 바이어스는 −0.0114 rad/s(9/15–16은 ±0.002)라 빼지 않으면 분당 약 39° 흐른다.
- **정지 판정 수정.** 정지 중 odom 속도에 한쪽 바퀴 엔코더 한 칸(0.00126 m/s, 0.00495 rad/s)이 섞여 "정확히 0"이 1 s를 못 넘었다. 그래서 회전 비교가 대기점 Spin 앞 정지를 못 잡았고, `imu_bias_relay`도 바이어스를 거의 배우지 못했을 것이다. 두 곳 모두 0.002 m/s·0.006 rad/s 안을 정지로 본다.

남은 확인:

- 다음 주행에서 도크 끝 제자리 회전(`dock_heading_measured` → `turn_in_place`)이 도크 허용 안에서 끝나는지.
- EKF 연결: 큰 회전 표본은 1건(EKF 0.19° vs odom 1.88°)이다. 표본을 더 모은 뒤 드라이버 `publish_tf:=false` + EKF odom TF로 바꾼다.
- 14:55 init에서 table_02 영역이 표지(0.689, −2.591) 대신 지도에 없는 라이다 덩어리(0.823, −2.721)로 잡혔다. 다음 출발 전 박스 위치를 확인한다.

### 20.10 table_01 첫 실차 (15:35–15:56, run `table_01_20261001_153549`, `_154936`, `_155418`)

| 실행 | 결과 |
|---|---|
| 15:35 1번 출발 | 물 받는 곳 주차 5.65 cm(모서리 5.64·5.67), 면 0.04°, 정지 0.65 cm·0.32°. table_01 관측 지점 도착 뒤 박스 관측 실패 → 탐색 회전 12번 소진 |
| 15:49 재개(`--resume-at-observation`) | 경유지가 남아 물 받는 곳 관측 지점 확인에서 즉시 종료(이동 없음). 재개에는 `--skip-via`를 함께 줘야 한다 |
| 15:49 재개(`--skip-via --resume-at-observation`) | 박스 관측 성공, 면 정렬 후 최종 접근 사전 계획이 목표 0.05 m 앞에서 끝나(허용 0.01) 실패 |
| 15:54 재개 | table_01 주차 5.63 cm(모서리 5.63·5.64), 면 0.01°, 정지 0.63 cm·0.37°, 5 s 대기, 이탈 6.0 s, 대기점 21.6 s, Spin −129.3° → −131.7°(잔차 −0.29°는 보정 없이), 곡선 후진 12.4 s, 도크 방향 0.87°(3° 안이라 제자리 회전 없음), 도크 확인 1.48 cm·0.88° |

**관측 실패 원인(실시간 깊이 영상으로 확인).** 라이다는 박스 앞면을 x ≈ 1.50 m, y −1.84 ~ −2.01 m에 보고 있었다(사용자 RViz 확인과 같음, 등록 앞면 (1.438, −1.817)에서 동쪽 6 cm·남쪽 10 cm). 깊이 영상에도 박스가 0.65 m에 10,135픽셀로 찍혔다. 그러나 검출기는 다음 두 가지 때문에 박스를 고르지 못했다.

1. 화면 전체에서 가장 큰 평면 하나를 고르는데, 박스 뒤 1.33 m에 폭 1 m 면이 있었다(9/30 배치 검사의 "카메라 시야 1.5 m 안 15칸, 1.28 m" 경고와 같은 곳).
2. 범위를 좁혀도 박스 면(평면 잔차 1.9 mm, nz 0.98)의 폭이 0.158 m(5–95 %)라 최소 폭 0.18 m에 걸렸다.

같은 실시간 20프레임 비교:

| 설정 | 박스 검출 | 뒤쪽 면 검출 |
|---|---|---|
| 범위 2.0 m, 최소 폭 0.18 m | 1/20 | 대부분 |
| 범위 2.0 m, 최소 폭 0.12 m | 6/20 | 14/20 |
| 범위 1.117 m, 최소 폭 0.12 m | 20/20 | 0/20 |

조치 `ad09a74`: 실행기가 관측마다 관측기 `maximum_depth_m`을 "영역까지 거리 + 영역 반경"(설정 2.0 m 이하)으로 정한다(`box_depth_window` 이벤트). 관측기는 실행 중 이 값만 받는다. 최소 폭은 0.12 m이고, 라이다 교차 확인은 그대로 둔다.

**최종 접근 사전 계획(조치 `552b818`).** 정적 지도·keepout은 목표 주변이 비어 있었고, 계획은 만들어졌지만(error_code 0) 정확히 한 칸(0.05 m) 앞에서 끝났다. 라이다로 본 박스가 전역 코스트맵 장애물로 올라가 칸 경계에 따라 목표 칸이 팽창 안쪽에 걸린 것으로 판단한다(코스트맵 값은 발행이 없어 직접 읽지 못함, 추정). 이 계획은 길이 비었는지만 보는 확인이라 허용을 한 칸 + 1 cm(0.06 m)로 했다. 실제 접근은 직선 경로와 1 cm 정지 확인이 결정한다.

**UDP 넘침(`udp_drops.jsonl`).** 15:35 실행 329 s 동안 +3,182(box_service 2,876, planner_server 306), 15:54 실행 101 s 동안 +655(파이 원시 기록기 `ros2 bag`).

남은 확인:

- 도크 끝 제자리 회전은 이번에 방향이 0.87°라 쓰이지 않았다. 3°를 넘는 경우는 아직 실차로 보지 않았다.
- table_01 등록 앞면을 실제 위치(약 (1.50, −1.92))로 갱신할지. 관측이 실제 면을 쓰므로 주행에는 지장이 없었다.
- 원시 기록기와 실행기의 UDP 수신 넘침이 주행에 영향을 주는지.

### 20.11 16:06 1번 출발: 한 번의 실행으로 끝난 table_01 사이클 (run `table_01_20261001_160621`)

`ad09a74`·`552b818`·`ffc8970`이 들어간 첫 1번 출발이다. 탐색 회전·재시도·실패 없이 첫 목표 수락부터 `home_arrived`까지 173.8 s.

| s | 구간 | 결과 |
|---|---|---|
| 0.0–31.7 | 도크 → 물 받는 곳 관측 지점 | 31.7 s |
| 35.5–53.6 | 면 정렬 6.0 s, 최종 접근 7.8 s | 간격 5.52 cm(모서리 4.72·6.33), 면 1.71°, 정지 0.52 cm·0.21° |
| 62.2–68.2 | 후진 이탈 | 6.0 s |
| 70.8–102.4 | table_01 관측 12.7 s, 면 정렬 3.4 s, 최종 접근 7.8 s | 간격 5.50 cm(모서리 6.67·4.33), 면 2.49°, 정지 0.50 cm·2.49° |
| 111.0–117.0 | 후진 이탈 | 6.0 s |
| 118.7–140.9 | 도크 대기점 | 22.2 s, 좌우 회전 없음 |
| 142.6–160.5 | 대기점 방향 맞춤 | Spin −121.9° → −125.1°(11.0 s), 잔차 0.98°는 보정 없이. 회전 외 정지 확인·경로 확인 합 약 8 s |
| 160.5–172.7 | 곡선 후진 도킹 | 12.2 s, 끝 방향 2.40°(3° 안), 도크 확인 1.63 cm·2.38° |

- table_01 관측은 `box_depth_window`(영역 거리 + 반경)로 매번 첫 시도에 확정됐다.
- 파이 UDP 넘침 증가 0. 파이 원시 기록 확인(25 s)은 4.0 MB로 정상 표시됐다.
- 줄일 수 있는 시간: 대기점 방향 맞춤 중 회전 외 대기 약 8 s(정지 포착 2.1 s, 재측정 1.0 s, 정지 확인 1.1 s, 경로 확인 1.5 s 등). 도크 → 물 받는 곳 31.7 s는 오전 23 s보다 길다(원인 미확인).

### 20.12 연속 주행(1번 → 충전 → 2번 → 충전) 가능 여부 (16:14, 이동 없음)

| 확인 | 결과 |
|---|---|
| 사이클 뒤 출발 조건 | `go`는 `localized` 상태만 본다. 사이클이 끝나도 유지돼 init 없이도 다음 `go`가 가능하다 |
| 첫 구간 출발 위치 | 1번·2번 모두 도크 → 물 받는 곳 경로로 시작하고, 출발 조건은 도크 0.3 m 안이다. 16:09 도킹 뒤 상태에서 상주 실행기에 계획만 요청(`--execute` 없음, run `dryrun_chain_*`): 결과 코드 0, `transit_planned_only` |
| 배터리 | 12.24 V(출발 기준 10.8 V, 주행 하한 10.5 V) |
| 충전 | 충전 케이블은 손으로 꽂고 시스템은 감지하지 않는다(전압으로 판정하지 않음). 케이블을 꽂은 채 다음 사이클이 출발하면 안 된다 |

조치 `jdamr_depart.py go table_01 table_02`(테이블 순서대로, 반대도 가능): 사이클마다 도크에서 확정돼야 다음으로 간다. 사이에 `--dock-wait-s`(기본 10 s) 정차 뒤 도크 init(스캔 정합·AMCL 확인, 이동 없음)을 다시 한다. `stop`은 대기 중에도 듣는다. 실제로 충전을 하려면 사이클마다 `go`를 따로 실행하고 사이에 케이블을 꽂았다 뺀다. 모의 테스트 3건(순서, 첫 실패에서 멈춤, 단일 사이클 옵션 거부)을 추가했다. 실차 연속 주행은 아직 하지 않았다.

### 20.13 16:22 연속 주행 2번 → 도크 → 1번 → 도크 (run `table_02_20261001_162204`, `table_01_20261001_162642`)

`jdamr_depart.py go table_02 table_01` 한 명령으로 두 사이클을 이어 마쳤다. 사이클 사이에는 도크 10 s 정차와 init(이미 활성인 스택에서 STARTUP 없이, `252cb6e`)을 했다.

| 사이클 | 시간 | 물 받는 곳 | 테이블 | 도크 끝 방향 → 제자리 회전 | 도크 확인 |
|---|---|---|---|---|---|
| 1 (table_02) | 198.2 s | 5.61 cm, 면 0.15° | 5.46 cm(모서리 4.36·6.57), 면 2.35° | 4.27° → 0.51 s | 1.46 cm·0.92° |
| 2 (table_01) | 167.9 s | 5.44 cm, 면 0.06° | 5.64 cm(모서리 5.24·6.03), 면 0.84° | 5.93° → 0.71 s | 1.33 cm·1.56° |

- **도크 끝 제자리 회전(`b7bc384`) 첫 실차 사용.** 두 번 모두 0.5–0.7 s 만에 허용 3° 안으로 들어왔다.
- **사이클 사이 간격:** 도크 도착 16:25:35 → 사이클 종료 16:25:51 → init 완료 16:26:40 → 다음 출발 16:26:50, 약 75 s.
- **두 번째 사이클 끝 대기:** 도크 도착부터 종료까지 110 s 걸렸다. 기록기는 16:29:56에 바로 멈췄고, 나머지는 bag 정리와 PC 복사로 보인다(원인 미확인).
- **UDP 넘침:** 파이 원시 기록기에서만 +587, +791.
- **table_02 면 정렬 불필요한 회전(사용자 관찰).** 정렬 목표가 관측 위치에서 앞 0.27 m, 옆 0.119 m였다(다른 실행은 옆 0.006–0.067 m).
  - 0–9 s: 곡선으로 목표 5.6 cm까지 왔다.
  - 9–21 s: 5 cm 판정을 채우려고 남은 몇 cm 경로를 따랐는데, 경로 방향이 옆을 가리켜 제자리 회전이 좌우로 반복됐다(1 Hz 재계획·AMCL 잡음).
  - 21–31 s: 3 cm 전진 뒤 103°를 되돌아 돌았다.
  - 합계 30.5 s, 왼쪽 124.8°·오른쪽 129.4°, 방향 전환 5번. 도크 대기점 좌우 회전(`4621399`)과 같은 현상이다.
- **조치 `46e50d2`.** 박스 면 정렬은 `face_alignment_checker`(위치 0.10 m, 방향 3°)와 전용 BT `navigate_to_pose_face_alignment.xml`을 쓴다. 이 기록이었다면 6.5 s에 위치를 채우고 약 35°만 돌았을 것이다. 도크 대기점 정렬 대체 경로는 직선 후진이 이어지므로 5 cm `alignment_goal_checker`를 유지한다. 관련 635건 통과, 파이 반영·세션 재기동·init 완료(16:38), 실차 확인 전.

### 20.14 Lattice 시험 (브랜치 `feat/lattice-planner-trial`, 18:30–18:40, 이동 없음)

파이의 살아 있는 세션에서 `scripts/map_20260930_manual/plan_compare.py`로 같은 시작·목표를 `GridBased`(NavFn)와 `Lattice`(SmacPlannerLattice, 차동 0.5 m 프리미티브, 전역 팽창 0.45)로 계획만 했다.

| 구간 | NavFn 길이 비 | Lattice 회전 패널티 1.5 | 0.5 | 0.0 |
|---|---|---|---|---|
| A 도크 → 물 받는 곳(경유 3) | 1.11 | 실패(도크 출발, NO_VALID_PATH) | 2.00 | 1.99 |
| B 물 받는 곳 → table_02(경유 2) | 1.04 | 1.00 | 3.91 | 1.02 |
| C 물 받는 곳 → table_01(경유 2) | 1.03 | 5.18 | 5.30 | 3.83 |
| D table_01 → 도크 대기점 | 1.04 | 1.34 | 1.03 | 1.02 |
| E table_02 → 도크 대기점 | 1.05 | 1.05 | 1.04 | 1.01 |

- 계획 시간은 파이에서 0.02–0.8 s로, 9/30 PC 비교의 수십 초 문제는 재현되지 않았다.
- 경유점 구간은 Lattice가 중간 경유점의 방향까지 맞추려고 2–5배 돌아간다. 도크 안 출발은 차체 외곽 충돌 검사에 걸릴 때가 있다.
- 도크 대기점 구간은 1.01–1.04에 도크 방향으로 도착한다. 대기점에서의 약 −122° Spin을 줄일 수 있는 후보다.
- 브랜치 마지막 설정: 대기점 BT만 Lattice(회전 패널티 0.5), 경유점 이동은 NavFn. 실차 주행은 하지 않았다. 메인은 Lattice 이전 상태로 두었다.

### 20.15 Lattice 대기점 구간 첫 실차 (18:51, run `table_02_20261001_185143`)

브랜치 `feat/lattice-planner-trial` 설정(대기점 BT만 Lattice, 회전 패널티 0.5, 전역 팽창 0.45)으로 2번 사이클을 한 번에 마쳤다. 같은 table_02 사이클의 이전 실행과 비교한다.

| 항목 | 18:51 Lattice | 16:22 NavFn | 14:42 NavFn | 11:52 NavFn |
|---|---|---|---|---|
| 사이클 | **171.0 s** | 198.2 s | 도킹 실패 | 186.4 s |
| 이탈 뒤 대기점 도착 | 34.0 s | 29.2 s | 28.6 s | 43.6 s |
| 대기점 도착 방향 오차 | **0.4°** | −114.6° | −121.9° | −132.8° |
| 대기점 Spin | **없음** | −117.9° | −125.7° | −136.0°, +7.1° |
| 대기점 도착 → 도크 후진 시작 | **15.8 s** | 30.6 s | 18.6 s | 24.2 s |
| 이탈 → 도크 후진 시작 합 | **49.8 s** | 59.8 s | – | – |
| 도크 끝 방향 → 제자리 회전 | 6.94° → 회전 | 4.27° → 회전 | – | – |
| 도크 확인 | 1.54 cm·0.38° | 1.46 cm·0.92° | – | 4.24 cm·1.19° |
| table_02 면 정렬 | 10.7 s | 30.6 s | 6.2 s | 7.5 s |
| 주차(물 받는 곳 / table_02) | 5.62 cm·0.83° / 5.69 cm·0.54° | 5.61·0.15 / 5.46·2.35 | 5.56·0.00 / 5.49·2.38 | 5.45·1.86 / 5.54·1.47 |

- 이탈부터 도크 후진 시작까지 odom 회전 합은 비슷하다(Lattice 348°, 16:22 NavFn 361°, 이동 3.04 m vs 3.15 m). 다만 Lattice는 회전을 주행 중에 나눠 하고 도크 방향으로 도착해 대기점 Spin(약 12 s)과 그 앞뒤 정지 확인이 없어졌다.
- table_02 면 정렬 10.7 s는 `46e50d2`(면 정렬 10 cm 판정기, main 포함)의 첫 실차 확인이다(16:22 30.6 s).
- 표본 1건. 다음은 1번(table_01 → 대기점 구간)과 연속 주행으로 확인한다.

### 20.16 NavFn과 Lattice 대기점 구간 비교 (18:51–19:10, 설정 고정)

같은 날 설정이 다른 실행이 섞여 있어, 실행마다 `run_config.json`에 대기점 계획기와 그때 반영된 수정을 남겼다(`scripts/map_20260930_manual/compare_runs.py`로 집계). 18:51 이후 네 실행은 모두 같은 설정이다(Lattice 대기점, 면 정렬 10 cm, 도크 위치 + 제자리 회전, 깊이 범위).

| run | 대기점 계획기 | 사이클 s | 대기점 도착 방향 오차 | Spin | 대기점 도착 → 도크 후진 s | 이탈 → 도크 후진 s | 도크 확인 |
|---|---|---|---|---|---|---|---|
| table_02 11:52 | NavFn | 186.4 | −132.8° | −136.0°, +7.1° | 24.2 | 69.6 | 4.24 cm·1.19° |
| table_02 16:22 | NavFn | 198.2 | −114.6° | −117.9° | 30.6 | 61.4 | 1.46 cm·0.92° |
| table_02 18:51 | Lattice | 171.0 | 0.4° | 없음 | 15.8 | 51.6 | 1.54 cm·0.38° |
| table_02 19:02 | Lattice | 171.0 | −90.1° | −92.8° | 16.6 | 50.2 | 1.49 cm·2.00° |
| table_01 16:06 | NavFn | 173.8 | −121.9° | −125.1° | 19.6 | 43.5 | 1.63 cm·2.38° |
| table_01 16:26 | NavFn | 167.9 | −123.3° | −126.4° | 31.1 | 54.6 | 1.33 cm·1.56° |
| table_01 18:58 | Lattice | 160.4 | −88.9° | −92.2° | 16.1 | 41.4 | 1.49 cm·2.99° |
| table_01 19:07 | Lattice | 172.4 | −108.4° | −112.0° | 29.2 | 53.5 | 1.30 cm·0.80° |

- table_02: 이탈 → 도크 후진이 Lattice 50.2–51.6 s, NavFn 61.4–69.6 s. table_01: 41.4–53.5 s 대 43.5–54.6 s로 차이가 뚜렷하지 않다.
- 대기점에 도크 방향으로 도착한 것은 4번 중 1번(18:51)이다. 나머지는 위치 판정(15 cm)이 Lattice 경로의 마지막 꺾임보다 먼저 끝났다. 18:58 table_01은 북쪽(90°)을 보고 0.12 m/s로 직진하다 대기점 12 cm 앞에서 끝났다(odom·TF 확인). Spin은 92–112°로 NavFn의 115–136°보다 작다.
- 사이클 시간은 면 정렬 수정(16:36 반영) 전후가 섞여 있다. NavFn 실행은 모두 수정 전이라 사이클 차이를 Lattice 효과로만 볼 수 없다. 16:22 table_02는 면 정렬 30.6 s가 포함됐다.
- 표본은 테이블별로 NavFn 2건, Lattice 2건이다.

비교가 끝난 뒤 처리할 항목(설정을 바꾸면 비교가 깨지므로 지금은 기록만 함):

1. 도크 끝 제자리 회전 대신 방향을 맞춘 뒤 후진할 것(사용자 지적). 지금은 도크 위치에 도달한 뒤 남은 방향을 제자리에서 돈다.
2. Lattice 대기점 구간은 위치 판정(15 cm)이 마지막 꺾임 전에 끝나지 않도록 할 것(방향도 보는 판정, 또는 꺾임이 판정 밖에서 끝나는 경로).
3. 연속 주행에서 사이클 사이 도크 대기를 없앨 것(지금 `--dock-wait-s` 기본 10 s).
4. 각 정차 지점(물 받는 곳·테이블)의 5 s 대기를 2 s로 줄일 것.

### 20.17 비교 뒤 수정과 계획 탐침 (2026-10-01 19:45 – 10-02 00:12, 이동 없음)

§20.16의 처리 항목 네 건을 반영했다. 계획기 선택은 파이 실세션에서 계획만 해서 정했다. 로봇은 움직이지 않았다.

**계획기 선택: 구간별 선택 유지 (이동 구간 NavFn, 대기점 구간 Lattice)**

| 구간 (경유점 없이 시작 → 목표) | NavFn 길이 비 | Lattice 길이 비 | Lattice 계획 시간 |
|---|---|---|---|
| A 도크 → 물 받는 곳 관측 | 1.08 | 2.78 | 0.109 s |
| B 물 받는 곳 → table_02 관측 | 1.05 | 3.14 | 0.004 s |
| C 물 받는 곳 → table_01 관측 | 1.05 | 2.64 | 0.008 s |
| D table_01 → 도크 대기점 | 1.00 | 1.03 | 0.200 s |
| E table_02 → 도크 대기점 | 1.02 | 1.04 | 0.175 s |

- 경유점을 포함한 같은 날 19:45 계획에서는 A 2.05, C 2.23이었다(NavFn 1.12, 1.03). 경유점을 빼면 Lattice가 더 길어지므로, Lattice 통일 판정 기준(길이 비 1.1 이하)은 계획 단계에서 통과하지 못했다. 통일 판정용 주행 3회는 하지 않는다.
- 계획 시간은 NavFn 0.001–0.039 s, Lattice 0.004–0.543 s. 두 계획기를 올린 `planner_server` 메모리는 77 MB(파이 사용 1.1 / 3.8 GB).
- Smac Hybrid-A*는 후보에서 뺀다. 설치본 `constants.hpp`의 모션 모델은 `TWOD`, `DUBIN`, `REEDS_SHEPP`, `STATE_LATTICE`이고 Hybrid-A*는 DUBIN·REEDS_SHEPP(최소 회전 반경 곡선)만 쓰므로 제자리 회전을 쓰지 못한다.
- Jazzy 1.3.12 Smac 헤더에 `goal_heading_mode`가 없다. 경유점마다 정해진 방향으로 도착해야 하는 것이 이동 구간 우회의 원인으로 보인다(추론).
- NavFn 헤더 주석은 "ROS values of 253 are obstacles"다. 우리 내접 반경이 0.085 m여서 NavFn은 약 0.25 m 이상 틈을 지나갈 수 있다고 본다(차체 폭 0.58 m, 회전 지름 0.83 m). 이동 구간에 NavFn을 남긴 대가이고, 지금은 Collision Monitor 정지 구역이 막는다. 좁아진 통로 실측은 하지 않았다.
- 원본: `$HOME/jdamr_data/map_20260930_manual/aligned/plan_compare_20261001_1945.jsonl`, `plan_probe_direct_staging_20261001_2356.jsonl`, `plan_probe_rotation_penalty_20261001_2357.jsonl`, 탐침 `plan_probe_20261001.py` (보관본 `/data/lim/jdamr_artifacts/jdamr_runs_20261001/plan_probes/`).

**처리 항목 2 (Lattice 대기점의 방향 판정): 효과 없음으로 보류**

- 대기점 구간 Lattice 경로는 직진한 뒤 목표점에서 제자리 회전으로 끝난다. 마지막 0.3 m 안의 제자리 회전이 D 63.4°, E 90.0°다. 회전 패널티를 1.5·3.0·5.0으로 올려도 D·E 모두 제자리 회전 63.4°로 같았다(길이 비 1.03–1.14).
- 판정에 방향을 넣으면 그 회전을 Spin 대신 RPP가 하게 된다. RPP의 끝 회전은 AMCL 방향을 따르며, 2026-09-30 도크 근처에서 AMCL이 약 15° 튀어 먼 쪽으로 돌았다가 돌아왔다(§`_reach_staging` 설명). 그래서 odom Spin을 유지한다.
- 18:51의 방향 정렬 도착(0.4°)은 그 실행의 시작 자세에서 나온 경로 모양으로 본다(추론). Lattice의 대기점 이점은 Spin이 92–112°로 NavFn의 115–136°보다 작은 것이다.
- 탐침 중 사고: 회전 패널티 원래 값 읽기가 "Node not found"로 비어 복원 명령이 실패했고, 실행 중 `planner_server`의 `Lattice.rotation_penalty`가 23:57부터 복원 명령까지 약 1–2분 5.0으로 남았다. 설정 파일 값 0.5로 다시 설정하고 `Double value is: 0.5`를 확인했다. 그 사이 주행은 없었다.

**처리 항목 1 (도크 안 제자리 회전 없애기): 방향을 도크 밖에서 맞춘 뒤 직선 후진**

도크 끝 방향 오차는 대기점의 옆 어긋남에서 나왔다. 대기점 Spin 뒤 방향은 0.3–2.3°로 정확했지만, 곡선 후진이 옆 어긋남을 도크까지 끌고 가 바로잡으면서 방향이 그만큼 남았다(odom, onboard bag).

| run | 대기점 옆 어긋남 | 도크 끝 방향 오차 |
|---|---|---|
| table_01 15:54 | −0.4 cm | −0.87° |
| table_01 16:06 | −4.5 cm | −2.39° |
| table_02 19:02 | −4.1 cm | −2.00° |
| table_01 18:58 | −4.4 cm | −2.99° |
| table_01 19:07 | +4.6 cm | +3.32° |
| table_02 16:22 | −7.5 cm | −4.27° |
| table_01 16:26 | −8.3 cm | −5.93° |
| table_02 18:51 | −9.6 cm | −6.93° |

옆 어긋남 1 cm마다 약 0.7°이고, 도크 끝 위치는 모두 0.2 cm 안·1.7–1.9 cm 앞에서 멈췄다. 수정(`restaurant_service.py` `_dock_straight`):

1. 대기점 옆 어긋남이 1 cm를 넘으면 도크 앞 0.25 m의 도크 축 위 지점(pre-dock)까지만 곡선 후진한다. 0.25 m는 10 cm 어긋남에서도 곡선 반경이 0.3 m보다 크게 남는 거리다. 곡선이 안 맞으면 기존 정렬 구간(turn-move-turn)으로 간다.
2. 직선 시작점(pre-dock 또는 대기점)에서 방향이 3°를 넘으면 그 자리에서 Parking 컨트롤러로 제자리 회전한다. 도크 밖이라 회전 원(반경 0.414 m)이 도크 위치의 차체 뒤 끝보다 0.13 m 앞에 머문다.
3. 도크까지는 odom 직선 후진(`reverse_waypoints`, 5 cm·3°)이고 위치 판정 2 cm로 끝낸다.
4. 도크 끝 방향이 3°를 넘으면 도크 안에서 돌지 않는다. 직선 시작점까지 직선 전진으로 빠져나와 방향을 맞추고 한 번 더 들어간다. 그래도 넘으면 `dock_heading_out_of_tolerance`로 실패한다.
5. 모든 구간은 대기점에서 고정한 odom 기준이다. 이벤트: `dock_leg_frozen_in_odom`(대기점 옆 어긋남·pre-dock 자세), `dock_entry_heading_measured`, `dock_heading_measured`(시도 번호), `dock_heading_out_of_tolerance`.

**처리 항목 3·4**

- 3: `jdamr_depart.py` `DOCK_WAIT_S` 10 → 0 s. 연속 주행 사이의 init(스캔 정합 재위치추정, 이동 없음)은 남겼다. 19:02 연속 주행의 사이클 사이 61 s는 대기 10 s, init 41 s(클릭 5 s, 스캔 정합 18 s, 활성화 확인 14 s, 지역 확인 3 s), 출발 준비 10 s였다. 이 init에서 스캔 정합이 클릭 기준 0.10 m·1.2° 차이를 잡았다.
- 4: `box_service.py` `STOP_DWELL_S` 2.0 s. 물 받는 곳, 테이블, 정차 재개(`resume_parked`) 모두 같은 값이다. restaurant_service의 `serve` 흐름(지금 쓰지 않음)의 5 s는 그대로다.

**검증과 배치**

- 테스트: `test_input_recovery.py`·`test_restaurant_service.py`·`test_box_service.py` 390건, `test_jdamr_depart_flow.py` 36건 통과. 도크 테스트 t27(2.0°는 한 번에, 4.4°는 빠져나와 다시 진입, 회전은 직선 시작점에서만), t27b(두 번째에도 넘으면 실패, 도크 안 회전 없음), t27c(6 cm 어긋남은 pre-dock 곡선 후 직선). 변경 Python 4개 `ament_flake8` 통과.
- 파이: 기존 두 파일을 `~/jdamr_data/deploy_backup_20261002_000927/`에 백업하고 `box_service.py`·`restaurant_service.py`만 반영, `colcon build --packages-select jdamr_cube_navigation` 통과. 설치본 SHA-256이 로컬과 같다. Nav2 세션은 설정 변화가 없어 그대로 두고 `jdamr-box-executor`만 00:10:43에 재시작했다(설치 00:09:30 이후).
- 파이 소스의 `service_visualization.py`는 로컬과 다르지만 이번 변경과 무관해 반영하지 않았다.
- PC 기록 `bag_20261001_160232`를 00:11에 닫고(906 MB) 보관본에 맞췄다(해시 255건 확인). 다음 주행용으로 PC 표시 유닛 다섯 개와 기록 `bag_20261002_001136`을 다시 띄워 두었다. 유휴 중에도 bag이 커지므로(16:02–00:11에 906 MB) 주행하지 않으면 `python3 $HOME/jdamr_data/map_20260930_manual/tools/jdamr_depart.py display-stop`으로 끈다.
- 실차 미확인: pre-dock 곡선, 직선 진입 뒤 도크 방향 오차, 빠져나와 다시 진입하는 경로, 정차 2 s, 연속 주행 대기 제거.

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

기록과 코드 검증은 주행 성공 증거가 아니다. 테이블 앞 5cm 정밀주차는 아직 성공하지 않았다. 물 받는 곳 앞 정밀 주차는 2026-09-30 15:46에 내부 추정 0.65 cm·1.83°로 도달했다(외부 실측 없음, 당시 방향 허용 1°라 실행기 판정은 미확인, §19). 충전소 후진 도킹은 2026-09-30 12:01에 도크 정위치에 도달했다(사용자 현장 확인, 내부 추정 TF 0.25 cm·3.8°, 스캔 정합 3.9 cm·1.2°, 외부 실측 없음). 끝 구간 yaw 보정 중 Nav2 진행 검사 중단(105)으로 서비스 판정은 실패였다(§16).
