# 2026-09-29 반복 주행 실패의 원인 분리와 실행 구성 복구

## 범위와 상태

충전 중 이동 goal이나 속도 명령을 보내지 않고 실행 구성·센서 지연을 점검했다.
정지 성능 시험의 위치 prior는 재시작 전 AMCL 계산 조건을 보존하기 위한 것이며,
충전 위치의 실제 지도 좌표나 다음 주행 준비 완료를 증명하지 않는다.
실차 테이블 도착·박스 면 정렬·5 cm 주차·충전소 복귀는 아직 완료되지 않았다.

## 관찰된 실패와 근거

| 구간 | 확인된 결과 | 판단 범위 |
|---|---|---|
| 14:40 접근 | 출발 waypoint 성공 뒤 `FollowPath` 오류 102 | 목표 수락 실패와 구분되는 이동 중 TF 오류 |
| 14:40:47–48 | collision monitor가 scan age 1.42–1.49초를 기록 | 그 소비자의 수신/처리 시점에 입력이 오래됨; 드라이버 지연으로 단정할 수 없음 |
| 14:49 접근 | 뎁스 관측기를 중지한 시도도 89초 뒤 오류 102 | 관측기 하나를 끄는 것으로 해결되지 않음 |
| 박스 관측 | `LiDAR face support is below five points` | 뎁스 면과 라이다 지지점 불일치; 당시 원시 데이터 부재로 세부 원인 미확정 |

원본 실행 기록은 데이터 루트의 `table_departure_20260929_RT7S1w`,
`staged_table_transit_odozw1k3`, `table_box_handover_20260929_qjxf5szo`에 보존했다.

오류 102는 설치된 Nav2의 `TF_ERROR`다. controller 주기 경고 자체는 실행 취소가
아니며, 위치 변환 예외 처리에서는 속도 0을 발행하고 action을 종료한다.
AMCL은 정상 수락한 scan의 stamp에 `transform_tolerance`를 더해 `map→odom`을
발행한다. 현재 값 1초는 이 미래 유효 시각이고, 실제 수신 지연을 없애는 값이 아니다.
근거: [Jazzy controller 구현](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/src/controller_server.cpp),
[Jazzy AMCL 구현](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp).

## 복구한 구성

- 식당 서비스의 기본값을 `use_composition=true`로 복구했다. 완주 기록에서 사용한
  `component_container_isolated`이며 컴포넌트마다 분리된 executor를 사용한다.
- `--prepare-only`는 keepout/map/AMCL 활성화, navigation 미활성을 유지한다.
  초기 위치추정 전에 planner를 활성화하던 시작 순서 문제의 이전 수정은 보존했다.
- collision monitor와 liveness guard는 container 밖에 유지했다.
- `--use-composition false`는 비교 진단용 명시적 선택이며 자동 폴백하지 않는다.
  세션 identity에 실행 방식을 넣어 다른 구성의 실행 세션을 재사용하지 않는다.
- 기존 지도·마스크·차체 외곽·센서 timeout·TF tolerance·배터리/충돌 판정은 바꾸지 않았다.
  라이다 stamp를 현재 시각으로 덮어써 지연을 숨기지 않는다.

9월 22일의 standalone 전환은 프로세스 장애 분리를 위한 선택이었다. 당시에도
RMW/lifecycle 응답 실패가 있어 그 전환을 DDS 정체의 근본 수정으로 볼 수 없다.
9월 4일 기록에서 standalone load 13.31, 합성 실행 load 3.53과 완주 사례를 확인했지만,
이 과거 비교만으로 오늘의 센서·주행 조건에서 효과를 확정하지 않는다.
근거: [9월 4일 인계](20260904_HANDOFF.md).

## 정지 상태 비교 방법

`probe_live_sensor_latency.py`는 발행·서비스 호출·파라미터 변경이 없는 관측 도구다.
scan/odom과 `/tf`의 BEST_EFFORT·RELIABLE을 함께 수신한다. 수신 시각과 header age,
간격 분포·원시 시계열·CPU 주파수/사용률, odometry 정지 여부를 보존한다.
두 TF 구독의 공백과 `TF stamp - 1초`를 scan stamp와 대조해 관측 손실과 AMCL 경로
정체를 구분한다. 이 외부 관측만으로 executor와 DDS 내부 원인을 완전히 분리하지는 못한다.

데이터는 `$HOME/jdamr_data/root_cause_20260929/`에 보존한다.
기존 30초 BEST_EFFORT 측정에서는 scan 약 9.6 Hz·최대 age 약 0.11초,
odom 약 50 Hz·최대 age 약 0.007초가 확인됐다. 한 구간의 `map→odom` 수신 공백은
1.73초였으나 이어진 30초 기록에서는 재현되지 않았다. 따라서 라이다 드라이버 backlog나
AMCL callback 정체를 이 예비 측정만으로 확정하지 않는다.

## 박스 실패의 재현성 보강

같은 관측 stamp의 실패는 한 번만 기록한다. 원시 scan과 frame/stamp, 선별 전 유효 점 수,
면 거리·접선 범위별 점 수, 최종 지지점 수, 선형 적합 오차, 목표·차체 위치·센서 설치값을
`box_lidar_witness_rejected` 이벤트에 남긴다. 비유한 숫자는 JSON null로 보존하고
scan 배열은 최대 9,000개로 제한하며 절단 여부를 기록한다. YAML 날짜도 직렬화한다.

`laser_link` 외 frame에 측정 mount를 적용하지 않는다. 기존 5점·±4 cm·±20 cm와
면 적합 임계값은 유지했다. 이 변경은 증거 기록 보강이지 박스 검출 성공의 증명이 아니다.
충전 위치에서 촬영한 RGB에는 목표 박스가 보이지 않아 그 데이터로 박스 면 정합을
검증하지 않았다. 카메라 렌즈 높이 215 mm와 라이다 설치 yaw는 기존 provenance를
보존하며 실제 교차 센서 정합 결과 없이 보정 완료로 승격하지 않는다.

## 60초 비교 결과와 초기화 전달 보강

동일한 카메라·2 Hz 관측기, domain 12/SUBNET/UDPv4, RViz 미실행 조건에서 비교했다.
읽기 전용 probe도 두 구간에서 동일하게 실행했고, 비영 속도 명령 0개·odom 정지
판정 true·원시 표본 절단 0개를 확인했다. CPU는 4코어 전체 사용률의 1초 표본 평균이다.

| 항목 | standalone | composition |
|---|---:|---:|
| CPU 평균, 전체 60초 | 86.44% | 71.89% |
| CPU 평균, 첫 10초 제외 | 86.20% | 71.87% |
| scan Hz | 9.669 | 9.640 |
| scan 최대 age | 0.120초 | 0.115초 |
| odom Hz | 50.007 | 49.999 |
| map→odom RELIABLE 최대 수신 간격, 첫 10초 제외 | 0.244초 | 0.137초 |

composition 구간의 RELIABLE TF 최대 age 1.715초·수신 간격 1.535초는 측정 시작 후
0.58–2.16초에만 나타났다. BEST_EFFORT 첫 수신도 2.12초로 늦었고, 이후 정상화됐다.
따라서 이 값은 steady AMCL 처리 지연이 아니라 신규 구독 시작 구간의 늦은 데이터
전달을 포함한다. 고정 warm-up 3초만으로 DDS 초기 정합 완료를 가정하지 않는다.
단일 정지 비교에서 관찰된 차이이며, 이동 controller가 10 Hz를 유지한다는 검증은 아니다.
CPU 주파수는 구간 중 변화했으므로 고정 주파수 벤치마크로 주장하지 않는다.

재시작 비교 도중 `/initialpose` 단발 발행은 AMCL 도달이 확인되지 않아 활성화가
실패했다. 이를 반복하지 않도록 `activate_navigation.py --initial-pose PATH`를 추가했다.
문서에는 `frame_id: map`, `x_m`, `y_m`, `yaw_rad`, `covariance_x_m2`,
`covariance_y_m2`, `covariance_yaw_rad2`를 모두 명시해야 한다. 좌표·불확실성을 자동
생성하거나 충전 위치를 저장된 출발 위치로 가정하지 않는다.

초기화는 `/set_initial_pose` 응답을 받은 뒤 ACK보다 새로운 AMCL pose를 확인한다.
정지 중에는 제한된 `/request_nomotion_update`로 갱신을 요청하며 이동 명령은 없다.
실패하면 navigation STARTUP을 하지 않고, 성공해도 기존 지도·정지·센서·배터리·
위치추정 검증을 거쳐야 한다. 옵션 미사용 시 기존 동작은 유지한다.

또 성능 시험의 첫 lifecycle 요청 응답은 확인되지 않았지만, 나중에 새 client의 동일
서비스 요청은 성공했다. 이것을 해결 완료로 처리하지 않는다. 설치된
`rmw_fastrtps_cpp 8.4.4`에는 서비스 응답 endpoint 매칭을 기다리는 workaround가 있다.
관찰과 양립하는 가능성이지, 이번 누락 원인으로 확정한 것은 아니다.
근거: [설치 버전 서비스 응답 구현](https://raw.githubusercontent.com/ros2/rmw_fastrtps/8.4.4/rmw_fastrtps_shared_cpp/src/rmw_response.cpp).

## Transport 후보 시험과 채택 제외

진단 중 UDPv4와 DEFAULT를 선택하는 임시 실행 옵션을 사용했다.
설치된 Fast DDS 2.14.6에서 DEFAULT는 UDPv4+SHM으로 같은 호스트의 지원
endpoint 간 데이터를 SHM으로 보낼 수 있지만 discovery는 UDP로 남는다. 서비스 초기
매칭 문제의 해결을 보장하지 않으며 zero-copy라고 설명하지 않는다.
근거: [2.14.6 환경변수](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/env_vars/env_vars.html#fastdds-builtin-transports),
[2.14.6 SHM](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/transport/shared_memory/shared_memory.html).

첫 DEFAULT 요청 세션은 하위 `onboard_nav2_core.launch.py`의 UDPv4 강제 설정 때문에
실제 component 프로세스가 UDPv4로 실행됐다. `/proc/<component PID>/environ`에서 확인했다.
그 구간의 `latency_composed_shm_dualqos_20260929.json`은 SHM 효과 비교에서 제외하고,
로컬에 `latency_transport_override_discarded_20260929.json`이라는 이름으로 보존했다.
옵션 문자열·세션 요청만으로 실제 middleware 설정을 확인했다고 주장하지 않는다.
두 번째 시험은 선택값을 session→restaurant launch→core launch로 전달해
실제 component PID 11095의 환경값 DEFAULT와 `/dev/shm`의 Fast DDS 리소스를 확인했다.
이것도 endpoint별 전송 경로나 정상 작동의 증명으로 삼지 않았다.

15:41:03에 navigation 활성화 응답을 받았으나 15:41:37–38에 세 lifecycle manager가
container 내부 노드의 heartbeat를 10초간 받지 못했다고 기록했다. 뒤이어 15:41:49와
15:41:54에 liveness guard가 해당 노드들의 그래프 소실을 확인했고 launch가 종료됐다.
15:42:06까지 container와 manager가 정상 종료되지 않아 launch가 SIGKILL로 종료했다.
먼저 heartbeat 실패가 있었으므로 guard의 단순 오판이라고 설명하지 않는다.
통신 정체·container 내부 정체 중 어느 원인이 선행했는지는 아직 분리하지 못했다.

`latency_composed_shm_confirmed_20260929.json`의 실제 측정 구간은
15:41:38–15:42:38이며 `map→odom` 표본 0개·AMCL pose null이다. 낮게 나온 CPU 평균
34.69%는 Nav2가 멈춘 구간을 포함하므로 성능 개선 수치로 사용하지 않는다.
이 후보를 채택하지 않고 임시 transport 옵션과 전달 코드를 제거했다.
최종 실행은 기존 UDPv4를 유지하며 RMW·DDS 버전·XML·QoS를 바꾸지 않는다.

## UDP 복구 후 확인과 변경 검증

실패 후보 세션 종료 뒤 UDPv4/합성 실행의 prepare-only 세션을 새로 기동했다.
배포된 `initialize_localization()`을 실제 파이에서 호출해 지도 identity·localization
lifecycle 상태·초기화 ACK·ACK 이후 AMCL pose 확인이 1.134초에 끝났다.
`production_initializer_stationary_20260929.json`에 성공·이동 goal 없음·navigation
STARTUP 요청 없음·물리 위치 미확정을 명시했다. 시험 prior를 실차 위치로 확정하지 않는다.

이후 localization만 활성화된 20초 정지 관측에서 scan 9.671 Hz·최대 age 0.112초,
양 QoS `map→odom` 각 193개·최대 수신 간격 0.112초, AMCL pose 수신을 확인했다.
비영 속도 명령 0개·odom 정지 true·표본 절단 0개다.
기록은 `latency_restored_udp_localization_20260929.json`이다. 앞의 navigation 활성 상태
60초 CPU 비교와는 다른 조건이므로 별도 복구 확인으로만 사용한다.

최종 변경의 관련 테스트 332개 통과, ROS `ament_flake8` 13개 파일 통과
(마지막 GetState 변경 파일 2개 재검증 통과),
`bash -n`과 `git diff --check` 통과를 확인했다. 일반 flake8 기본 설정과 ROS의
ament 설정은 다르며 검증 근거는 위 ament 실행 결과다. 기존 사용자의 RViz 변경은
이 수정의 스테이징·커밋 대상에서 제외한다.

최종 확인에서는 Nav2를 새 prepare-only 세션으로 재시작해 시험 prior를 지웠다.
실행 invocation은 `0b524f4cfda04d60852254cd2e448922`다.
map/AMCL/keepout 4개 노드는 ACTIVE, navigation 6개 노드는 UNCONFIGURED였다.
첫 동시 조회에서 collision monitor 응답이 누락됐고, 후속 개별 조회에서 UNCONFIGURED를
확인했다. 실제 component 환경은 UDPv4이고 AMCL 로그는 초기 pose 미설정을 알렸다.
5초 읽기 전용 관측에서 비영 `/cmd_vel`과 활성 action 상태는 각각 0개였다.

15:48:59의 collision monitor 로그에는 `/get_state` 응답을 보내려다
`client will not receive response` timeout이 기록됐다. 서버 프로세스는 살아 있었고
후속 조회는 성공했으므로 이 조회 실패를 충돌이나 노드 사망으로 설명하지 않는다.
앞서 확인한 RMW 서비스 응답 매칭 실패가 실제 로그로 관찰된 사례다.
다만 모든 이전 서비스 실패의 원인이 같았다고 일반화하지 않는다.

`require_active()`의 읽기 전용 `GetState`만 같은 client로 최대 두 번 조회하도록 보강했다.
첫 응답 대기는 최대 2초, 두 번째는 기존 응답 예산 5초의 남은 시간만 사용한다.
첫 요청이 미완료 timeout이고 중단 요청이 없을 때만 재조회하며 미완료 pending을 정리한다.
재조회 직전 warning에 노드 이름·1/2회차·첫 오류·기존 5초 응답 예산을 남겨
응답 손실이 성공 결과 뒤에 숨지 않도록 했다.
완료된 future의 예외·실제 다른 lifecycle 상태·중단은 재시도하지 않는다.
STARTUP·초기 pose 설정·이동 action에는 자동 재시도를 추가하지 않았다.
서비스 discovery 대기 5초도 기존 값이며 총 대기 상한을 늘리는 변경은 아니다.
배포 후 production helper로 4개 localization ACTIVE와 6개 navigation UNCONFIGURED를
모두 확인했으며 1.134초에 끝났다. 이 호출은 초기 pose·STARTUP·이동 goal을 보내지 않았다.
`final_prepare_read_confirmed_20260929.json`에 별도 보존했다.

## 별도 미해결: container 종료

15:30:15에 명시적 서비스 중지와 cleanup 중 `Magick: abort due to signal 11`,
container exit -6이 발생했다. 주행 중 crash가 아니라 종료 중 관찰이며, 현재 기록으로
GraphicsMagick·AMCL·plugin unload 중 어느 경로인지 확정하지 못했다. 이후 새 세션
기동은 성공했으나 종료 문제까지 해결됐다고 주장하지 않는다. 유사 upstream 사례는
[Nav2 #5358](https://github.com/ros-navigation/navigation2/issues/5358)이지만 Humble의
AMCL 실행 중 사례로 이번 Jazzy 종료 crash의 수정 근거로 사용할 수 없다.

## 다음 실차에서 확인할 항목

1. 충전 케이블 분리·실제 출발 배치 뒤 새 위치추정을 수행한다. 성능 시험 prior는 사용하지 않는다.
2. 같은 실행 구성에서 이동 중 scan age, 양 TF 수신 간격과 오류 102의 재발 여부를 기록한다.
3. 박스 관측 실패가 재현되면 새 이벤트로 면 선택·라이다 설치각·가림·지지점 부족을 분리한다.
4. 테이블 도착, 회전된 박스 면과 방향 일치, 외부 측정 5 cm, 원래 충전소 위치·각도 복귀를
실제 실행 결과로 판정한다. 정지 계측이나 단위 테스트만으로 완료 처리하지 않는다.
