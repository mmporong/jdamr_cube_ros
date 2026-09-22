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

완성 전 양팔 서빙 로봇의 선행 검증에서는 현재 JD-AMR 베이스를 같은 임무의 축소 실행기로
사용한다. 새 2D 지도를 만든 뒤 시작 자세를 `home_dock`, 박스가 놓인 지점을 `table_01`,
`table_02` 같은 이름으로 등록한다. 왕복 명령은 목적지 도착, 전면 박스의 안정 관측과 대기,
시작 위치·방향 복귀를 순서대로 실행한다. 현재 `home_dock`은 지도 pose 이름이며 실제 충전
접점, 충전 전류 또는 도킹 센서 성공을 판정하지 않는다.

```text
Cartographer 지도 생성·저장
  → 저장 지도 + AMCL + Nav2 기동
  → home_dock 및 박스 station 교시
  → station 접근·5cm/3° 내부 자세 확인
  → Depth 전면 박스 stable 관측을 20초 연속 유지
  → home_dock 접근·5cm/3° 내부 자세 확인
```

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

파이에 새 navigation 커밋을 반영할 때는 저장소 루트에서 배포 스크립트를 사용한다.
이 스크립트는 커밋의 패키지만 묶어 체크섬을 확인하고, 파이의 별도 작업공간에서
빌드·테스트한 뒤 베이스를 정지해 주 작업공간에 반영한다. 실패하면 이전 소스·빌드·설치를
복원하고 베이스를 다시 기동한다. ROS Jazzy 환경 스크립트와 충돌하는 `set -u`는 쓰지 않는다.

```bash
cd "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros"
bash scripts/deploy_navigation_to_pi.sh lim@jdamr.local <배포할-커밋>
```

작업 중인 미커밋 파일은 배포하지 않는다. 두 번째 인자는 실행 시작 시 전체 커밋 해시로
고정되므로 배포 도중 브랜치가 바뀌어도 대상 소스는 달라지지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_rgbd_ws/install/setup.bash"
mkdir -p "$HOME/jdamr_data/service"
```

### 0. 먼저 새 지도를 만든다

현재 실차 데이터에서 비교 우세를 확인한 Cartographer를 지도 작성 백엔드로 유지한다.
하드웨어·Cartographer가 실행된 파이에서 자율 매핑을 띄운 뒤 별도 명령으로 시작한다.
`frontier_explorer` 단독 실행은 사용하지 않는다.

```bash
ros2 launch jdamr_cube_navigation autonomous_mapping.launch.py use_sim_time:=false
ros2 service call /autonomy/start std_srvs/srv/Trigger '{}'
```

탐색을 마치면 `stop`이 현재 목표 취소, 정지 확인, 지도 저장을 순서대로 처리한다. 상태의
`save=succeeded`와 생성된 YAML·PGM을 확인한 뒤에만 저장 지도 주행으로 전환한다.

```bash
ros2 service call /autonomy/stop std_srvs/srv/Trigger '{}'
ros2 topic echo --once /frontier_explorer/status
ls -lt "$HOME"/maps/autonomous_*.yaml "$HOME"/maps/autonomous_*.pgm | head
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
  navigation_profile:=new_base_candidate \
  use_box_observer:=true
```

`use_box_observer:=true`는 속도 명령을 발행하지 않는 Depth 박스 관측기를 같은 launch에
포함한다. 카메라 드라이버는 별도로 실행돼 있어야 한다. 단순 목적지 계획·교시만 할 때는
기본값 `false`로 생략할 수 있지만, 실제 `roundtrip --execute`에는 관측기가 필요하다.

`new_base_candidate`는 기존 launch의 새 차체 지도·마스크 검사(`new_base_` 파일명)를
유지한다. 이전 복도 지도를 사용하는 세션은 기존처럼 `new_base_revisit_candidate`의
지도·마스크 검증이 적용된다. `corridor`는 기존 launch가 허용하는 시뮬레이션용으로만 쓴다.
기존 Nav2를 유지하려면 `evaluation/prepare_parking_params.py`로 Parking 파라미터를
생성해 해당 세션에 적용해도 된다. 실행 중인 서버의 Parking 설정을 프로그램이 대조한다.

시뮬레이션에서는 launch에 `use_sim_time:=true`를 주고, 아래 `teach/go` 명령에도
`--ros-args -p use_sim_time:=true`를 붙인다. Nav2와 서비스 노드의 시간을 함께 맞춰야 한다.

3. 로봇을 책상 앞 원하는 서비스 위치·방향으로 세우고 교시한다. 이 명령은 이동하지 않는다.

먼저 지도 생성 때 정한 충전소 시작 자세에 로봇을 세우고 위치와 방향을 함께 교시한다.

```bash
ros2 run jdamr_cube_navigation restaurant_service teach-home \
  --registry "$HOME/jdamr_data/service/destinations.yaml" \
  --pose-id home_dock \
  --log "$HOME/jdamr_data/service/teach_home.jsonl"
```

실제 충전 장치가 없으므로 이 pose는 귀환 기준점이다. 기체를 같은 방향으로 세운 상태에서
교시하며, 다시 교시할 때만 `--replace`를 쓴다.

그 다음 로봇을 박스가 있는 목적지 앞에 세우고 목적지를 교시한다.

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

교시 시 차체 맨 앞과 테이블의 가장 가까운 면 사이 간격을 쟀다면 다음 옵션을 함께 쓴다.
`--target-front-gap-m 0.05`는 요청 간격이고, `--measured-front-gap-m`에는 그 자리에서
잰 값을 넣는다. 측정하지 않았다면 측정 옵션을 생략한다. 아래 예시는 실제 계측 결과가 아니다.

```bash
ros2 run jdamr_cube_navigation restaurant_service teach \
  --registry "$HOME/jdamr_data/service/destinations.yaml" \
  --table-id table_01 --pose-id table_01_main \
  --target-front-gap-m 0.05 \
  --measured-front-gap-m "$MEASURED_FRONT_GAP_M" \
  --gap-measurement-note "줄자: 차체 맨 앞부터 테이블의 가장 가까운 면" \
  --log "$HOME/jdamr_data/service/teach_table01_measured.jsonl"
```

요청 간격과 교시 간격은 서로 다른 값이며 목표 좌표를 자동으로 당겨 수정하지 않는다.
실측값은 해당 교시 시각·pose에만 속한다. 테이블 또는 차체 외형을 바꾸면 다시 교시한다.

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

목적지 도착부터 박스 확인·대기·충전소 복귀까지 한 번에 실행하려면 `roundtrip`을 사용한다.
기본값은 이동 없는 전체 경로 계획이며 `--execute`를 붙인 경우에만 움직인다. 목적지에서는
`/box_parking/perception_status`의 전면 박스가 최신·안정 상태로 20초 연속 유지돼야 복귀한다.
관측이 끊기면 대기 시간이 처음부터 다시 계산되고, 45초 안에 만족하지 못하면 그 자리에서
실패 종료해 근거 없는 성공·복귀로 넘어가지 않는다.

```bash
# 이동 없는 목적지·복귀 경로 확인
ros2 run jdamr_cube_navigation restaurant_service roundtrip \
  --registry "$HOME/jdamr_data/service/destinations.yaml" \
  --table-id table_01 \
  --log "$HOME/jdamr_data/service/plan_roundtrip_table01.jsonl"

# 실제 왕복
ros2 run jdamr_cube_navigation restaurant_service roundtrip \
  --registry "$HOME/jdamr_data/service/destinations.yaml" \
  --table-id table_01 --dwell-s 20 --box-timeout-s 45 --execute \
  --log "$HOME/jdamr_data/service/run_roundtrip_table01_$(date +%Y%m%dT%H%M%S).jsonl"
```

복귀 성공은 `home_arrived`의 `position_error_m ≤ 0.05`, `|yaw_error_rad| ≤ 0.05236`,
정지 hold 1초가 모두 만족된 경우다. 박스 분류나 목적지 ID를 영상으로 알아내는 기능은
아니며, 지도에서 선택한 목적지에 박스 형태의 전면이 안정적으로 존재하는지 확인한다.

전체 작업의 최대 시간은 180초다. 이미 존재하는 로그에는 덮어쓰지 않으므로 실행마다 새
이름을 지정한다. `arrived`는 베이스의 내부 추정 도착을 뜻한다. 팔에는 상판·컵 상태를
추가로 확인한 뒤 작업을 전달해야 한다.

## 데이터와 판정

등록부에는 테이블 ID, 활성 여부, 주·대체 pose, 접근 거리, 교시 시각·관측 근거,
지도·금지구역 파일 신원을 저장한다. 로그는 JSONL이며 선택 결과와 각 Nav2 goal의
종료 상태, 최종 `position_error_m`, `yaw_error_rad`, `hold_s`, `physical_accuracy`를 남긴다.

`selected.waypoints`에는 접근점과 최종점을 모두 남긴다. `front_gap`에는 요청 간격,
교시 실측 간격·측정 방법, 이번 도착의 간격 확인 상태를 분리한다. 교시에서 5cm를
기록했더라도 도착 시 센서·외부 측정이 없으면 `arrival_measured_m: null`,
`arrival_verification: NOT_MEASURED`다. `arrived`만으로 5cm 주차 성공이나 팔 작업
허가를 판정하지 않는다. 기존 등록부에 간격이 없으면 추측값을 채우지 않는다.

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

## 2026-09-21 오프라인 보완·실행 결과

- Nav2 core launch에서 `discovery_range` 선언보다 환경변수 설정이 먼저 실행되는 오류를
  재현했다. 인자 생략 시 시작하지 못했던 순서를 수정하고 기본 LOCALHOST와 명시적
  SUBNET 실행 인자를 모두 테스트했다. 과거 실차 중단 전체의 원인으로 단정하지 않는다.
- 교시 간격 근거, 선택된 접근점·최종점 로그, 취소 종료 미확인 예외 처리를 추가했다.
  기존 Nav2 제어기·충돌 설정·교시 좌표는 바꾸지 않았다.
- 관련 회귀 테스트 297개, 빌드, flake8·pep257 통과. 독립 코드 리뷰 APPROVE.
- 기존 Gazebo 주차 시나리오를 localhost 전용 ROS domain 186에서 실행했다.
  접근점·최종점 두 목표 모두 SUCCEEDED, 정지 확인 PASS, 실행 종료 코드 0.
  시뮬레이터 ground truth 기준 최종 위치 오차 0.0410278683m, 방향 오차 2.672°.
  MCAP 795,785 bytes를 남겼고 종료 후 잔존 프로세스는 없었다.
- 결과 파일: `$HOME/jdamr_artifacts/parking_offline_20260921_after_launch_fix/summary.json`.
  SHA-256: `2889bd8f8ca5e14565a4af4de69d51006e60c29489dbccf95f0811c3b607fec9`.

이 실행은 기존 Gazebo 자산과 `corridor_route --park-final`의 주차 제어 흐름을 확인한 것이다.
새 차체의 실측 재현이나 실제 테이블 간격 5cm 검증, `restaurant_service` 실물 end-to-end
검증은 아니다. 이름별 테이블 선택은 별도의 코드 통합 테스트로 확인했다.
실제 테이블 좌표 교시와 근접 구간 거리 센싱·실차 검증이 남아 있으며,
이번 작업에서는 파이 접속·배포·실물 주행을 수행하지 않았다.
