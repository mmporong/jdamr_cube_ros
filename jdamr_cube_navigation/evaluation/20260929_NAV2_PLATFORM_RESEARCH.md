# Nav2 실행 위치와 반복 장애 조사 — 2026-09-29

## 판단

파이에서 Nav2를 실행하는 것 자체는 오류가 아니다. 그러나 **Pi 4에서 RGB-D·박스 인식·표시 중계까지 함께 실행한 현재 구성을 안정적인 주행 구성으로 승인할 근거는 아직 부족하다.** 과거의 짧은 정지 기동 성공과 코드 테스트 통과를 실차 태스크 성공으로 확장한 판단을 철회한다.

현재 유지할 후보는 LOCALHOST/standalone이다. 합성 실행은 공식적으로 자원 절감에 유리하지만, 이 기체의 앞선 전체 활성화 시험에서 heartbeat 상실이 재발했다. 문서 권장만으로 다시 전환하지 않는다. 장애가 재현되지 않은 정지 구간도 이동 중 안정성의 보증은 아니다.

이번 작업은 충전 중 조사다. 이동 goal·속도 명령은 보내지 않았다. 성능 비교용 AMCL prior는 충전 위치의 검증 좌표가 아니며, 조사 종료 때 Nav2를 중지하여 다음 실차 초기화에 재사용하지 않는다.

## 공식 자료에서 확인한 범위

| 근거 | 확인한 내용 | 이 기체에 적용할 때의 경계 |
|---|---|---|
| [Nav2 실행 구성·튜닝](https://docs.nav2.org/jazzy/configuration_and_development/tuning_guide/) | composition은 CPU·메모리 절감 목적의 정식 구성 | 절감 효과가 이 구성의 heartbeat 장애 해결을 보증하지 않음 |
| [ROBOTIS TurtleBot3 Navigation](https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/) | Remote PC에서 Navigation을 실행하는 공식 예제가 있음 | 원격 Nav2도 타당한 구조지만 모든 현업 로봇의 표준이라는 뜻은 아님 |
| [Nav2 lifecycle manager](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/) | 서버 heartbeat 상실 시 관리 대상 서버를 내림 | 비활성화는 후속 동작일 수 있으므로 그 전에 생긴 지연·종료를 조사 |
| [Nav2 1.3.12 FollowPath 오류 정의](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_msgs/action/FollowPath.action) | 105는 FAILED_TO_MAKE_PROGRESS | 숫자만으로 DDS 장애·파이 성능 부족을 판정하지 않음 |
| [Fast DDS 2.14.6 대용량 전송](https://fast-dds.docs.eprosima.com/en/v2.14.6/fastdds/use_cases/large_data/large_data.html) | 큰 메시지·높은 전송률은 CPU와 전송 부하를 만들며 버퍼 포화·재전송이 지연을 키울 수 있음 | 누적 drop만으로 이번 실패를 설명하지 않고 구간 증분과 소유 소켓을 확인 |
| [rmw_fastrtps 8.4.4 구현](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/participant.cpp) | LOCALHOST 분기는 built-in transport를 끄고 SHM·UDP user transport를 추가 | 환경변수 UDPv4만으로 UDP 전용 전송이라고 기록하면 잘못된 비교가 됨 |
| [ROS 2 executor](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Executors.html) | callback group과 executor의 실행 방식이 callback 병렬성과 대기를 결정 | 스레드 수 증가만으로 수신·계산 지연이 줄어든다고 판단하지 않음 |
| [Nav2 성능 분석](https://docs.nav2.org/jazzy/tutorials/general_tutorials/get_profile/get_profile/) / [ros2_tracing](https://github.com/ros2/ros2_tracing) | 프로세스/함수·callback 실행을 계측하는 공식 수단 | 계측 부하를 분리하고 이동 중 ptrace 정지는 사용하지 않음 |

웹의 최신 개발판 설명보다 설치 버전의 소스를 우선했다. 현재 확인된 버전은 Ubuntu 24.04/Jazzy, Nav2 1.3.12, rmw_fastrtps_cpp 8.4.4, Fast DDS 2.14.6이다.

## 서로 다른 세 장애

1. **경로 진행 실패 105**: 17:01 기록은 waypoint 도착 후 불필요한 최종 yaw 회전과 충돌 방지 차단이 겹쳤다. 마지막 10초 이동 34.1 mm는 진행 기준 50 mm보다 작았다. 관측용 waypoint와 최종 주차 판정을 분리한 `8042a31`의 수정 대상이다. 다른 모든 105의 원인까지 확정한 것은 아니다.
2. **Nav2 자동 비활성화**: 18:29 기록에는 scan 지연 2.637초, controller 주기 미달, TF 미래 외삽 뒤 keepout heartbeat 상실이 있다. manager의 종료는 관찰됐지만 지연의 최종 원인은 아직 특정되지 않았다. 임계값 완화나 bond 해제는 해결책으로 적용하지 않았다.
3. **RViz 로봇 미표시**: PC와 파이의 LOCALHOST 그래프가 분리돼 실제 TF·모델이 전달되지 않은 표시 문제다. 로봇 제어와 분리한 표시 중계로 다룬다. 과거 지도·마커가 보인다는 사실로 현재 로봇 상태를 확인했다고 판단하지 않는다.

상세 사건은 [기존 실패 기록](20260929_PARKING_FAILURES.md), 이전 비교 실험은 [runtime 조사](20260929_RUNTIME_ROOT_CAUSE.md)를 따른다.

## 하드웨어와 실행 근거

- Raspberry Pi 4 Model B Rev 1.4, RAM 4 GB. 센서만 켠 구간에서 메모리 약 715 MB, swap 사용 0이었다. 전체 workload의 메모리 수치로 일반화하지 않는다.
- 루트 디스크는 29 GB 중 27 GB 사용, 가용 약 811 MB였다. 기록·빌드 공간 위험은 있지만 현재 Nav2 멈춤의 원인으로 확정하지 않았다. 원시 주행 데이터는 삭제하지 않았다.
- 과거 LOCALHOST/standalone 전체 활성화 60초 원시 기록의 전체 CPU 평균은 약 81.20%였다. 카메라만의 부하가 아니다.
- 이번 실행 invocation: `05b3cc7e5c834ef8bc06fafa1a5f2be6`. 19:44:43 navigation 전체 활성화. prepare-only로 시작한 뒤 승인된 충전 중 성능 비교용 prior와 STARTUP만 사용했다.
- 19:49:31–34 전체 CPU는 평균 약 72.10% non-idle였다. I/O wait 약 0.17%를 포함하며, 4코어 전체를 100%로 표시한 값이다. 3초 표본을 장시간 성능 판정으로 쓰지 않는다.
- 같은 시점 map_server 약 10.30%, navigation manager 약 21.26%는 **단일 코어 100% 기준**이다. 둘 다 DDS 수신뿐 아니라 주 스레드에서 CPU를 썼다. CPUQuota는 infinity, cgroup throttled 0으로 quota 제한 근거는 없었다.
- map_server에 대한 3초 strace는 futex 호출이 가장 많았다. ptrace 오버헤드가 있으므로 순수 성능 비교에는 포함하지 않았고, futex가 많다는 것만으로 deadlock을 확정하지 않았다.

### UDP 오류의 해석

19:38:11–16 센서 구간에서 RcvbufErrors는 1,452,016으로 일정했다. Nav2 시작 이후 1,534,841까지 증가했으나 19:48:58–19:49:03 정상 상태에서는 다시 증분 0이었다. 시작 전후 총 증분 82,825건과 정상 구간의 증분 0건을 구분한다. 초기 큰 증분을 특정 5초 구간에서 발생한 값처럼 쓰지 않는다.

`ss -uanpm`에서는 카메라·manager·map_server의 DDS discovery 포트에 drop이 관측됐고, 해당 데이터 수신 포트의 drop은 0이었다. 따라서 이 자료는 **기동 시 발견 트래픽 손실**의 근거이지 RGB-D 이미지 손실 또는 Nav2 bond 단절의 확정 근거는 아니다. 과거 사라진 소켓의 누적 오류는 현존 PID에 귀속시킬 수 없다.

현재 kernel receive buffer 기본/최대는 212,992바이트다. 버퍼 확대를 무조건 적용하지 않았다. 지속적인 처리 병목이면 더 큰 버퍼는 오래된 표본 적체를 늘릴 수 있으며, 기본/최대값 변경만으로 기존 소켓 설정이 바뀌는 것도 아니다.

### 표시 중계 갱신 전후

기존 중계 PID 28264는 수정 파일이 배포된 뒤에도 예전 코드를 실행 중이었다. PC의 정확한 `jdamr-rviz-display-relay.service`만 재시작하여 TF 묶음 생성 주기 제한·구독 depth 1·nice 10 변경을 실제 실행에 반영했다. 새 파이 export PID는 33904다. Nav2·카메라·박스 관측기는 이 비교 중 재시작하지 않았다.

| 30초 정지 계측 | 갱신 전 | 갱신 후 |
|---|---:|---:|
| 전체 CPU 평균 | 79.85% | 79.64% |
| scan 수신 | 288개 | 290개 |
| scan stamp age p95 | 114.77 ms | 112.25 ms |
| scan stamp age 최대 | 412.41 ms | 129.36 ms |
| map→odom 최대 수신 공백, RELIABLE | 404.75 ms | 194.00 ms |
| odom 최대 stamp age | 308.87 ms | 59.14 ms |
| 비영 cmd_vel 수신 | 0 | 0 |

전후 각각 한 번이며 warmup도 3초/5초로 다르다. 최대 지연 감소를 중계 변경의 인과적 성능 개선으로 확정하지 않는다. **CPU가 사실상 같으므로 관제 중계만 고쳐 전체 부하 문제가 해결됐다는 결론도 불가하다.** 후속 전체 구간 journal에는 costmap TF cache 경고 3건이 남았다. 평균 Hz만 보고 경고를 지우거나 모든 지연이 해결됐다고 쓰지 않는다.

19:55 전체 실행 상태에서 RAM 사용 1,026 MiB, available 2,757 MiB, swap 사용 0이었다. 이 구간에서는 메모리 용량 부족보다 CPU 실행 대기와 통신·callback 지연 조사가 우선이다. `top` 한 장이나 보드 이름으로 RAM 업그레이드가 해결책이라고 결론 내리지 않는다.

### 기존 transport 비교의 정정

PID 환경에는 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`가 있었지만 map_server와 manager에 `dds.shm.*` 수신 스레드가 동작했다. 설치 RMW 소스가 그 이유를 설명한다. **LOCALHOST 구간을 UDP-only라고 부른 이전 기록은 환경변수 설정 기록으로만 유효하며 실제 전송 경로 비교 근거로 사용할 수 없다.** 이 정정은 과거 SUBNET 실행의 전송 경로까지 확인했다는 뜻이 아니다.

## 권장 역할 배치

| 실행 위치 | 현재 유지할 역할 | 이유 |
|---|---|---|
| 로봇 로컬 | 모터·odom·LiDAR, 마지막 명령 timeout, 충돌 방지 | PC 연결 유무와 무관하게 오래된 명령을 차단할 수 있어야 함 |
| Pi — 현재 후보 | AMCL·Nav2, 필요한 박스 관측 | 현재 연결 구조 보존. 제어와 인식이 함께 켜진 상태의 피크 지연을 기준으로 계속 판단 |
| PC | RViz·대시보드 렌더링·데이터 분석·무거운 3D 재구성 | 표시·후처리가 주행 제어의 CPU·메모리를 잠식하지 않게 분리 |

**원격 Nav2**는 공식적으로 가능한 대안이다. 다만 현재 LOCALHOST를 단순 SUBNET으로 바꾸는 것으로 완료되지 않는다. 시간 동기화, 센서/TF QoS, 연결 손실 시 로봇 로컬 정지, stale 명령 폐기, 복구 후 소유권을 함께 구현해야 한다. 네트워크 영향을 분리하지 않은 채 PC로 옮기면 다른 출발 장애가 생길 수 있다.

**더 강한 온보드 컴퓨터**도 대안이다. RGB-D SLAM과 학습 기반 양팔 인식까지 상시 운용하려면 Pi 하나에 모두 추가하는 방향은 권장하지 않는다. 다만 지금 측정만으로 특정 기기 구매가 필수라고 결론 내리지는 않는다.

주행과 박스 정렬을 단계별로 나누는 기존 목표에도 맞춰야 한다. 알려진 테이블 부근까지는 지도 기반 이동, 도착 후에는 박스 면 관측·정렬을 수행한다. 이때 무거운 인식·3D 재구성의 상시 실행 필요성을 따로 판단한다. 인식 offload나 주기 제한을 적용하려면 충돌 방지가 해당 입력에 의존하는지와 다시 켰을 때의 새 관측 수신을 확인해야 하므로 이번 충전 조사에서 센서를 임의로 껐다 켜지는 않았다.

## 완료 판정과 다음 수정의 조건

- 매 출발마다 환경변수·launch·RMW를 갈아끼우지 않는다. 승인한 실행 구성 하나를 유지하며 변경은 충전 중에 한다.
- 전체 Nav2가 ACTIVE인 동일 구간에서 sensor stamp age, map→odom 공백, controller 10 Hz deadline, lifecycle/service 응답, UDP 증분을 맞춰야 한다. 낮은 CPU만으로 성공 판정하지 않는다.
- 새 client가 붙을 때만 실패한다면 discovery/endpoint 매칭을 조사하고, 기존 연결에서도 지연되면 executor·연산 경합을 조사한다. CPU 백분율이나 누적 drop 하나만으로 결론을 내리지 않는다.
- 정지 계측 통과와 실제 테이블 이동→박스 면 정렬→5 cm 정지는 별도 결과다. 현재 문서는 실차 성공 승인이 아니다.
- 진단 스킬은 잘못된 `/clock`, QoS, RViz, transport 단정을 수정하고 symptom별 조사로 바꿨다. 형식 검증과 기술 근거 검토는 했으나 실제 작업 속도 향상을 실측했다고 주장하지 않는다.

이번 원시 계측은 `$HOME/jdamr_data/nav2_architecture_20260929_tF5td9/`에 보존한다. `thread_cpu_socket_profile.txt`, `strace_map_server.txt`, `before_relay_refresh.json`은 각각 다른 계측 구간이다. ROS probe 파일의 첫 호출은 파이 source 경로 부재로 실패했고 계측값이 없었다. 검토한 기존 파일을 data 경로로 복사한 뒤 수행한 결과만 사용했다.

스킬 원본 PKM 커밋은 `d1e7309`이며 origin/master push와 원격 일치를 확인했다. Codex·Claude 설치본은 원본과 동일하다. 독립 리뷰 에이전트는 런타임 thread limit으로 시작하지 못해, 별도 패스에서 공식 소스·diff·형식·설치본을 대조했다. 이를 독립 승인이라고 표시하지 않는다.

## 마지막 상태와 검증 한계

- 19:57:09 이후 새 읽기 client에서 10개 lifecycle 서버 모두 ACTIVE 응답을 2.783초 안에 받았다. `/cmd_vel` 발행자는 collision_monitor 하나였다. 최초 전체 활성화 이후 약 12분의 journal에 heartbeat 상실은 없었지만, 이동 action의 계산 부하는 포함하지 않았다.
- 이 조회의 앞선 잘못된 시도는 keepout 이름 두 개가 틀렸고 discovery 전에 요청했다. 그때의 NO_RESPONSE는 서버 실패 판정에서 제외했으며 `invalid_prematch_state_probe.txt`로 보존했다. 올바른 이름과 service readiness를 사용한 조회는 `final_lifecycle_states.txt`다. 새 조회를 계속 반복하지 않았다.
- 19:58:20 Nav2 service는 inactive/dead, MainPID 0이었다. 충전 중 시험 prior와 지도 위치추정 프로세스는 종료했다. 표시 중계도 다시 시작하여 시험 TF 캐시를 비웠다. base·RGB-D·박스 관측 서비스는 유지했다. 다음 출발의 실제 위치 초기화는 아직 하지 않았다.
- 갱신한 중계와 사용한 계측기의 관련 테스트 34개가 통과했다. 스킬 형식 검증·설치본 대조·diff 검사도 통과했다. 이는 반복 주행 실패의 완전 해결이나 실차 주차 성공을 뜻하지 않는다.
- **남은 핵심**: 과거 이동 중 수초 지연의 최종 원인은 미확정이다. 지금 자료는 파이의 전체 부하가 높고 기동 시 discovery 손실이 있다는 근거이며, 둘 중 하나가 모든 실패를 일으켰다는 증거는 아니다. 다음 원인 분리는 동일 운영 구성의 실패 시각 전후 callback/스케줄링 계측 또는 허용된 실제 이동 기록이 필요하다. 보호 timeout·차체 치수·목표 거리를 바꾸어 성공처럼 처리하지 않았다.
