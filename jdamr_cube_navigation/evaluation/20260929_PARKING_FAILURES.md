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

## 충전 중 수정본 검증

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

기록과 코드 검증은 주행 성공 증거가 아니다. 테이블 앞 5cm 정밀주차와 충전소 복귀는 아직 성공하지 않았다.
