# 2026-09-22 서빙 주행 출발 실패 기록

## 상태와 범위

- 시각 기준: Asia/Seoul. 기록 범위는 이번 오후 출발 준비와 실패한 실행 시도다.
- 목표: 충전소 후면 주차 → 5초 대기 → 1번 테이블 주변 이동 → 박스 면 기준 정밀 접근 → 5초 대기 → 충전소 복귀·후면 주차.
- 주행 완료 아님. 16:24까지는 목표 수락 전 실패했고, 16:34 후속 시도에서는 목표 수락·이동 후 저전압으로 취소됐다. 아래 후속 실행 기록을 함께 읽는다.
- 16:24 확인: Pi `jdamr-restaurant-navigation.service` inactive. 실행 중인 corridor_route/home 명령 프로세스도 발견되지 않았다.
- 지도 원본, Keepout, 충전소 pose, 센서 bag은 보존했다. 이번 실패를 박스 충돌 또는 장애물 감속 사례로 분류하지 않는다.

## 실패 목록

| ID | 관찰·오류 | 확인된 사실 / 원인 확정 범위 | 상태·후속 조치 |
|---|---|---|---|
| D01 | 준비 완료 안내 뒤에도 테이블 목적지가 없음 | 실행 registry에 `tables: []`. 파란 영역은 대략 위치 `(1.896, 0.303)m`이며 교시된 최종 pose가 아님 | 미해결. 사용자가 박스를 보고 정렬하도록 명확히 했으므로 영역 이동과 센서 기반 최종 목표 생성을 연결해야 함 |
| D02 | 박스 접근 실행기가 있다고 가정함 | 현재 `depth_box_parking`은 관측 전용이며 `control_ready=false`, 속도 발행 없음. 오전 정렬·접근 기록은 있으나 현재 소스의 재실행 가능한 통합 제어기는 확인하지 못함 | 미해결. 과거 폐기된 실행기를 무검토 복원하지 않고 현재 관측과 Nav2 제어 경로를 연결해야 함 |
| D03 | 기동 후 `map` TF 부재 | `set_initial_pose=false`인 새 AMCL에 초기 자세가 없었음. 위치 추정 전 `/set_initial_pose` 요청이 필요했음 | 초기화 요청 ACK 확인. 이후 실행부 종료와는 별개 |
| D04 | `ssh: ... Connection timed out` | hostname SSH 한 차례 timeout. 이후 `192.168.0.159` ping 2/2 응답, IP SSH 성공 | 해당 접속 회복. 무선 또는 부하를 원인으로 단정하지 않음 |
| D05 | `/keepout_filter_mask publisher must be keepout_filter_mask_server only` | `home --execute`가 여기서 실패. 로컬·Pi 양쪽 topic info에서 발행자 1개지만 이름은 `_NODE_NAME_UNKNOWN_`, reliable/transient-local. 코드가 요구하는 이름과 불일치 | 미해결. 이름이 사라진 그래프 원인과 실제 발행자 식별을 확인. 이름 검사나 충돌 방지를 무조건 제거하지 않음 |
| D06 | `invalid choice: new_base_candidate` | launch의 profile 이름을 corridor_route CLI에도 사용할 수 있다고 오인. CLI 허용값은 corridor / obstacle_candidate / obstacle_base_candidate / new_base_revisit_candidate | 명령 수정. 실행 route에는 해당 BT의 `obstacle_base_candidate` 사용. 기체 형상과 Nav2 params는 변경하지 않음 |
| D07 | `navigation readiness timeout: AMCL pose missing` 3회 | 노트북 2회, Pi 1회 동일 오류. 첫 실패는 heartbeat 장애 전, 뒤 시도는 실행부 종료와 겹치거나 종료 후 발생 | 미해결. 노트북 네트워크 문제만으로 단정했던 판단을 철회. 서비스 생존·발행 상태 확인 전 같은 명령을 반복하지 않음 |
| D08 | `/request_nomotion_update` 서비스 대기 timeout 2회 | 노트북과 Pi 모두 응답을 얻지 못함. 그 시간대 Nav2가 종료 중 또는 종료된 상태였음 | 해당 시도 실패. 다음 시도 전 AMCL active 확인 필요 |
| D09 | 여러 lifecycle 서버 heartbeat 동시 timeout | 16:22:30 keepout_filter_mask_server, map_server, planner_server에서 각각 10000ms heartbeat 부재 보고 | 핵심 미해결. heartbeat 미수신은 확정, 서버 프로세스 사망·executor 정체·DDS 문제 중 최초 원인은 미확정 |
| D10 | 전체 주행 실행부 종료 | 16:22:44/49 liveness guard가 필수 노드 9개 소실을 연속 확인. 16:22:52 guard exit 1 → required process 종료로 launch 전체 shutdown | D09 뒤 연쇄 종료 확인. guard만 끄는 것으로 해결됐다고 보지 않음 |
| D11 | 정상 종료 지연·강제 종료 | 16:22:59 lifecycle managers와 container SIGTERM, 16:23:04 일부 SIGKILL/exit -9 | 종료 수렴 실패. 최초 장애 원인과 구분하여 shutdown 처리도 확인 필요 |

## 핵심 타임라인

1. 독립 map_server/AMCL만 있던 상태에서 RGB-D와 서빙 Nav2를 시작했다. 중복 방지를 위해 기존 localization unit은 먼저 종료했다.
2. 16:19:17 새 AMCL이 초기 자세를 요구했다. 마지막 정지 TF `x=-0.255m, y=-0.816m, yaw≈1.610rad`로 `/set_initial_pose` 요청을 보냈고 ACK를 받았다. 이 값은 실측 주차 오차가 아니다.
3. 16:19:57 `Managed nodes are active`를 확인했다. 이는 이후 상태 유지나 미션 성공을 보장하지 않는다.
4. 충전소 실행이 D05로 실패했다. 이후 1번 관측 영역 진입 명령은 D06 수정 뒤 D07로 실패했다.
5. 16:22:30 지도·마스킹·경로 계획 서버의 lifecycle bond heartbeat가 함께 끊겼다.
6. 16:22:44/49 필수 노드 소실 → 16:22:52 전체 launch 종료 → 16:23:04 강제 종료가 이어졌다.
7. 이 과정에서 실행부 상태를 다시 읽기 전에 명령을 재시도했다. 이미 종료 중인 AMCL 서비스를 기다리면서 시간을 추가로 소모했다.

## 실행 경로와 근거

- 구현 커밋: 후면 주차 `30eb34e`, RViz 표시 `6da1c84`.
- 저장소: `$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros`, 브랜치 `feat/restaurant-service-destinations`.
- Pi 배포 경로: `$HOME/jdamr_ws/src/jdamr_cube_ros`.
- 실행 자료: `$HOME/jdamr_data/service_run_20260922_922PvE/`.
  - `initial_home_failure.jsonl`: 첫 충전소 실행의 실제 실패 이벤트 원본.
  - `navigation_shutdown_excerpt.log`: Pi journal에서 발췌한 heartbeat·guard·종료 로그.
  - `table_01_observation_route.yaml`: 영역 관측을 위한 경유점. 최종 주차 pose가 아님.
  - `SESSION.yaml`, `assets/`: 실행 전 지도·마스크·설정과 상태.
  - `bag/`: 16:14:46~16:36:17, 1,291.400881654초, 119,663개 메시지로 정상 종료. odom 63,310개, scan 11,568개, plan 8개를 원본 순회로 확인했다. 이전 주행 기록을 소급 복원한 것이 아님.
- 문서의 CLI 오류와 timeout은 이번 도구 실행 출력에서 확인했다. 모든 CLI stdout을 별도 파일로 저장하지는 못했으므로 재현 로그 전체가 있다고 주장하지 않는다.

## 재발 방지 및 남은 작업

- [ ] D09의 최초 장애를 확인: 같은 시점의 container/lifecycle graph와 자원·DDS 증거를 대조. 부하나 네트워크를 추측만으로 원인 처리하지 않음.
- [ ] D05를 실제 발행자와 연결해 해결. 고유 발행자, 현재 마스크 데이터, 서버 설정 검증은 유지.
- [ ] 실행부 종료 시 모든 다음 미션 요청을 취소하고 종료 이유 하나로 보고. 꺼진 AMCL을 기다리는 재시도 금지.
- [ ] D01/D02: 지도상의 관측 영역 이동 → RGB-D 박스 면 위치·법선 → 접근 자세 생성 → 주행 제어 → 최종 거리·각도 확인 연결.
- [ ] 주차 성공은 지도 pose 오차와 박스-차체 간격을 구분. 5cm는 목표이며 외부 거리 측정 없이 달성 수치로 기재하지 않음.
- [ ] 다음 실행의 stdout/stderr, 미션 이벤트, bag, 사용 설정을 같은 run 폴더에 기록.
- [ ] 주행 종료 후 기록 정상 종료·수신량 확인, 원본 보존 뒤 중복·임시 파일만 선별 정리.

## 주행과 별개인 준비 오류

- RViz 사용자 service 최초 실행: `Failed to connect to bus: No medium found`. 현재 셸에 XDG_RUNTIME_DIR/DBUS_SESSION_BUS_ADDRESS가 없었고 실제 사용자 bus 경로를 지정한 뒤 실행 성공.
- RViz 화면에서 초기 텍스트 공백이 과도하고 지도 가장자리가 잘림. 짧은 라벨과 view scale로 수정. 최초 표시 화면은 `rviz_before_label_adjustment.png`로 구분했다.
- RViz shader warning과 LaserScan TF 지연 메시지도 관측됐으나 D09의 원인으로 확정하지 않았다.
- 노드 구성·빌드 완료를 실제 출발 가능 또는 박스 접근 완료처럼 안내한 것은 잘못이었다. 이후 보고는 코드 반영 / 실행부 active / 목표 수락 / 실제 이동 / 주차 완료를 구분한다.

## 후속 실행: 16:34 목표 수락 후 배터리 저전압 취소

앞의 `목표 수락 전 실패`는 16:24까지의 상태다. 후속 시도에서는 상태가 달라졌다.

- `accce91`: 선택형 `use_composition=false`를 추가해 공통 container 대신 Nav2 서버를 개별 프로세스로 실행했다. 지도·마스킹 설정·충돌 모니터·lifecycle·liveness guard와 required-exit 종료 처리는 유지했다. 관련 테스트 186개와 독립 리뷰를 통과했다. 최초 DDS/heartbeat 장애 원인을 해결했다고 단정하지 않는다.
- 16:34:17: 경로 116 poses, 길이 2.918m 계획 성공. `home_exit` 목표 UUID `44704344af014d51be8551331c422c04` 수락.
- 16:34:22: 첫 경유점까지 남은 거리 0.79m, 배터리 10.55V 피드백. 초기 목표 직선 거리는 약 1.016m였으나 두 값을 빼 실측 이동 거리로 기록하지 않는다.
- **D12 / 16:34:23**: 배터리 10.464V < 설정 하한 10.500V → 목표 취소 요청. Nav2 `Goal canceled`, controller `Cancellation was successful. Stopping the robot.` 확인. 하한은 변경하지 않았다.
- 종료 후 일회성 odom/TF 관측 명령은 출력 없이 timeout. 이후 저장 bag 원본에서 16:36:17.616 마지막 odom의 선속도·각속도 0을 확인했다. controller 취소 로그 외에 속도 관측 근거도 확보했다.
- 16:34:17~16:34:25의 odom 397개에서 첫·마지막 위치 차이는 0.297818m다. 바퀴 odom 추정 변위이며 외부 실측 이동 거리나 주차 정확도가 아니다.
- **D13 / 16:34:24~25**: global/local costmap의 `KeepoutFilter: Filter mask was not received` 경고가 남았다. 마스킹 파일 검증이나 RViz 표시를 실제 필터 수신 증거로 간주하면 안 된다. 다음 출발 전 실시간 필터 수신을 복구해야 한다.
- 이후 배터리 소모를 줄이기 위해 이번에 시작한 navigation/RGB-D 서비스에 중지 요청을 보냈다. 기본 베이스 서비스는 건드리지 않았다.
- 근거: 실행 폴더 `table01_low_battery.log`. 목표 수락은 확인됐지만 테이블 도착·박스 정밀주차·충전소 복귀는 완료하지 못했다.

## 재발 방지 변경과 검증 범위

| 대상 | 반영한 변경 | 검증 범위 / 남은 조건 |
|---|---|---|
| ROS setup과 `set -u` 충돌 | `restaurant_session.sh`에서 nounset을 끈 상태로 ROS·workspace setup을 읽고 실패를 확인 | shell mock으로 미정의 변수 setup 및 실패 종료 검증. 서비스 시작이나 실물 이동을 수행한 테스트가 아님 |
| 매번 다른 임시 시작 명령, 중복 Nav2 | 단일 session 스크립트에서 registry·base·중복 노드·센서 토픽 확인 후 고정 unit으로 Nav2만 시작 | `active`를 위치 추정·출발 준비 완료로 안내하지 않음. 초기 pose와 주행 action은 발행하지 않음 |
| D09 공통 container 장애 영향 | 식당 서비스 기본값을 개별 프로세스 실행으로 변경 | 프로세스 경계 분리. 최초 heartbeat/DDS 장애 원인 해결 또는 실차 재발 없음으로 주장하지 않음 |
| D13 mask 준비 실패 후 다른 서버만 활성화 | 식당 서비스는 lifecycle manager 하나로 mask → filter info → map → AMCL → 주행 서버를 순서대로 관리 | mask configure 실패 시 뒤 단계 활성화가 진행되지 않는 구성. 실제 costmap 수신 여부는 다음 실차에서 확인해야 함 |
| D12 출발 직후 전압 하락 | 식당 미션의 새 전진/후진 action 전 출발 기준 10.8V, 주행 중 기존 하한 10.5V 유지 | 경계값·비유한 값·목표 미전송 회귀 테스트. 10.8V는 부하 시험으로 보정되지 않은 후보 정책이며 잔량 추정이나 완주 보장이 아님 |
| 자료 재생 중 실물 토픽 혼입 | RViz 기록 재생은 LOCALHOST/domain 78, 표시 토픽만 허용 | 속도·action·서비스 요청 재생 제외. domain 12 실차와 분리 |

### D13 추가 원인 근거

- Pi journal 16:33:17: mask server가 올바른 YAML/PGM(177×149)을 읽었다.
- 16:33:18: `/keepout_filter_mask_server/change_state` 응답 전송이 RMW timeout으로 실패했다. keepout lifecycle의 활성화 완료는 확인되지 않았다.
- 별도 manager의 local/global costmap은 기동했고, 16:34:24~25에 mask 미수신 경고를 냈다. 세 manager가 독립적으로 시작해 마스킹 준비 실패와 주행 서버 활성화가 함께 존재할 수 있었다.
- 토픽 설정은 양쪽 모두 `/keepout_costmap_filter_info` → `/keepout_filter_mask`로 일치한다. 단순 토픽 오타로 분류하지 않는다.
- [Nav2 Jazzy KeepoutFilter 원본](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.12/nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp)은 mask가 없으면 경고 후 필터 처리를 반환한다. RViz에 저장 마스크가 보이는 것만으로 실제 필터 수신을 증명할 수 없다.
- 최초 RMW 응답 timeout 자체의 원인은 미확정이다. 순차 기동은 불완전한 준비 상태의 전파를 막는 수정이지 통신 장애 전체를 제거했다는 주장이 아니다.

### 재개 시 유지할 조건

- 충전 중에는 Nav2·목표 action을 자동 재시작하지 않는다.
- 기체를 옮겼다면 과거 출발 pose를 자동 재사용하지 않는다. 실제 배치와 현재 위치 추정을 맞춘다.
- D01/D02 박스 면 기반 최종 제어 연결과 실차 후면 주차 검증은 미완료다. 코드·표시 준비를 미션 성공으로 보고하지 않는다.
- `restaurant_session.sh start`는 서버 기동 요청만 수행한다. 충전 완료와 초기 위치 확인 뒤 별도 미션 실행을 요청한다.
