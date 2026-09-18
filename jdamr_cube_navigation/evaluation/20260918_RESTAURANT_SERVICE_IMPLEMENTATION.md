# 책상 앞 서비스 위치 교시·정밀 배치

## 구현 범위

`table_01` 또는 `table_02`를 선택하면 해당 책상에 교시한 `map → base_link`
위치·방향으로 이동한다. 접근점은 교시 위치에서 차체 전방의 반대 방향으로 0.5m 떨어진
곳으로 계산하며, Nav2가 접근점과 정차점까지의 경로를 계획한 뒤 이동한다. 접근점 거리는
교시할 때 `--approach-offset-m`으로 0.2–2.0m 안에서 지정할 수 있다.

마지막 구간은 기존 Parking 제어기를 사용한다. 목표 위치 오차 0.05m, 방향 오차 3°,
최종 선속도 0.01m/s 이하·각속도 0.02rad/s 이하, 최종 명령 0을 1초 연속 확인한다.
판정에는 최신 합성 TF와 odometry를 사용한다. `/amcl_pose`만으로 현재 자세를 대신하지 않는다.

이 수치는 지도 좌표 기준 목표 허용 오차다. 책상과 차체 사이 간격 5cm를 측정하거나
보장하는 기능은 아니다. 실제 테이블 간격은 교시 위치를 배치할 때 정하고, 최종 외부
측정값은 별도로 남긴다. Depth 상판 인지와 팔 작업 시작은 이번 베이스 도착 판정에
포함하지 않았다. 카메라가 근거리에서 상판을 계속 볼 수 있다는 가정을 사용하지 않는다.

## 보존하는 동작

- 기존 footprint, Keepout, Collision Monitor와 센서 상태 확인을 사용한다.
- 목적지 프로그램은 `/cmd_vel`을 발행하지 않는다. Nav2가 장애물을 반영해 이동한다.
- 지도·금지구역은 YAML과 이미지 SHA-256을 모두 저장한다. origin·resolution 변경도
  교시값을 무효화한다. 실행 시 활성 map 서버와 mask 서버의 파일 및 발행 중인
  `/map`·`/keepout_filter_mask`의 격자 데이터·해상도·원점도 대조한다. 현재 프로젝트의
  trinary P5 PGM 지도 형식을 지원한다. 실행 도중 격자가 달라지면 해당 goal을 취소한다.
- 위치추정 공분산의 후보 상한은 x/y 각각 0.01m², yaw 0.03046174198rad²다.
  표준편차로 각각 0.1m·10°이며, 5cm/3° 목표 오차와 다른 값이다. 이 기준은 실차
  반복 측정으로 얻은 보장치가 아니다. 값이 크거나 유효하지 않으면 교시·도착을 확인하지 않는다.
- 주 목적지의 `GOAL_OCCUPIED` 또는 `NO_VALID_PATH`에서만 같은 테이블의 대체 위치를
  한 번 계획한다. TF 오류·시작점 충돌·센서 문제·실제 이동 실패에는 다른 곳으로 재시도하지 않는다.
- Nav2 성공 후에도 정지와 목표 오차가 확인되지 않으면 `arrived`를 기록하지 않는다.
- Ctrl+C·타임아웃·실패 시 해당 navigation goal의 취소와 terminal 상태를 확인한다.
  응답이 끊기면 `cancel_unconfirmed`를 기록하며 정지 확인 성공으로 표시하지 않는다.

## 사용 순서

아래 명령은 ROS가 실행되는 기기에서 사용한다. 노트북에만 설치하면 파이에는 아직 없다.
실차용 지도·마스크·새 차체 파라미터는 기존 검증된 파일을 사용한다.
자리 이동과 실차 실행은 사용자가 주행을 요청한 시점에 수행한다.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_rgbd_ws/install/setup.bash"
mkdir -p "$HOME/jdamr_data/service"
```

1. 사용할 지도와 마스크를 지정해 빈 등록부를 만든다. 예시 변수에는 사용 중인 파일의
   전체 경로를 넣는다. 테이블 좌표를 임의의 숫자로 채우지 않는다.

```bash
ros2 run jdamr_cube_navigation restaurant_service init \
  --map "$SERVICE_MAP_YAML" --keepout "$SERVICE_KEEPOUT_YAML" \
  --registry "$HOME/jdamr_data/service/destinations.yaml"
```

2. 베이스·라이다·TF가 실행된 상태에서 서비스용 Nav2를 켠다. 기존 Nav2와 중복 실행하지 않는다.
   원래 파라미터를 보존하고 임시 파일에 Parking 플러그인만 추가한다.

```bash
ros2 launch jdamr_cube_navigation restaurant_service.launch.py \
  registry:="$HOME/jdamr_data/service/destinations.yaml" \
  params_file:="$SERVICE_NAV2_PARAMS" \
  navigation_profile:=new_base_candidate
```

`new_base_candidate`는 기존 launch의 새 차체 지도·마스크 검사(`new_base_` 파일명)를
유지한다. 이전 복도 지도를 사용하는 세션은 기존처럼 `new_base_revisit_candidate`의
지도·마스크 검증이 적용된다. `corridor`는 기존 launch가 허용하는 시뮬레이션용으로만 쓴다.
기존 Nav2를 유지하려면 `evaluation/prepare_parking_params.py`로 Parking 파라미터를
생성해 해당 세션에 적용해도 된다. 실행 중인 서버의 Parking 설정을 프로그램이 대조한다.

시뮬레이션에서는 launch에 `use_sim_time:=true`를 주고, 아래 `teach/go` 명령에도
`--ros-args -p use_sim_time:=true`를 붙인다. Nav2와 서비스 노드의 시간을 함께 맞춰야 한다.

3. 로봇을 책상 앞 원하는 서비스 위치·방향으로 세우고 교시한다. 이 명령은 이동하지 않는다.

```bash
ros2 run jdamr_cube_navigation restaurant_service teach \
  --registry "$HOME/jdamr_data/service/destinations.yaml" \
  --table-id table_01 --pose-id table_01_main \
  --log "$HOME/jdamr_data/service/teach_table01.jsonl"
```

다른 책상에서는 `table_02`, `table_02_main`과 새 로그 파일명을 사용한다. 같은 책상의
대체 서비스 위치는 다른 위치에서 `--pose-id table_01_alternate --priority 2`로 교시한다.
기존 pose를 다시 교시할 때만 `--replace`를 쓴다. 다른 테이블과 기존 로그는 보존한다.
상태가 안정된 1초 구간의 실제 TF·odom, AMCL 공분산과 command 관측 여부를 교시 근거에 남긴다.

4. 저장값 확인과 경로 계획은 이동 없이 실행할 수 있다.

```bash
ros2 run jdamr_cube_navigation restaurant_service list \
  --registry "$HOME/jdamr_data/service/destinations.yaml"
ros2 run jdamr_cube_navigation restaurant_service go \
  --registry "$HOME/jdamr_data/service/destinations.yaml" \
  --table-id table_01 --log "$HOME/jdamr_data/service/plan_table01.jsonl"
```

5. 주행 요청 후 실제 배치에는 `--execute`를 붙인다.

```bash
ros2 run jdamr_cube_navigation restaurant_service go \
  --registry "$HOME/jdamr_data/service/destinations.yaml" \
  --table-id table_01 --execute \
  --log "$HOME/jdamr_data/service/visit_table01.jsonl"
```

전체 작업의 최대 시간은 180초다. 이미 존재하는 로그에는 덮어쓰지 않으므로 실행마다 새
이름을 지정한다. `arrived`는 베이스의 내부 추정 도착을 뜻한다. 팔에는 상판·컵 상태를
추가로 확인한 뒤 작업을 전달해야 한다.

## 데이터와 판정

등록부에는 테이블 ID, 활성 여부, 주·대체 pose, 접근 거리, 교시 시각·관측 근거,
지도·금지구역 파일 신원을 저장한다. 로그는 JSONL이며 선택 결과와 각 Nav2 goal의
종료 상태, 최종 `position_error_m`, `yaw_error_rad`, `hold_s`, `physical_accuracy`를 남긴다.

이번 구현 검증은 파일 기반 단위 테스트와 ROS 메시지·action 응답을 사용한 통합 테스트,
기존 주차·차체 설정 회귀 검증까지 190개 PASS다. `colcon build`, 변경 Python의
`ament_flake8`·`ament_pep257`, 설치된 CLI와 launch 인자 확인도 통과했다.
localhost 전용 domain 187에서 실제 ROS 노드 생성·종료를 확인했으며 발행 토픽은
`/parameter_events`, `/rosout`이었다. 이 확인에서는 실물 연결·이동 명령을 사용하지 않았다.
`use_sim_time:=true` ROS 인자를 받은 서비스 노드의 실제 parameter 값도 확인했다.
같은 격리 domain의 Nav2 Jazzy map server에서 보관 지도와 Keepout을 각각 발행해,
파일에서 계산한 fingerprint와 894×212 격자 메시지의 fingerprint가 모두 일치함을 확인했다.

실제 테이블 좌표 등록, 실차 도착 오차 및 5cm 간격은 미측정이다.
다음 실차 작업은 올바른 지도에서 1·2번 책상 위치를 교시하고 한 목적지 도착을 확인하는 것이다.
이전에 다른 기체로 진행했던 시뮬레이션 수치를 이번 실차 성능으로 사용하지 않는다.

격자 변환 참고: [Nav2 Jazzy map_io.cpp](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_map_server/src/map_io.cpp).
