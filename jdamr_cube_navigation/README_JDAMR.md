# JD-AMR 안전 자율 매핑

이 패키지는 Cartographer가 갱신하는 `/map`에서 frontier를 고르고, Nav2에 한 번에 목표 하나만 전달한다. `frontier_explorer`는 시작 시 항상 `IDLE`이며 직접 `/cmd_vel`을 발행하지 않는다. 최종 속도 명령은 Collision Monitor만 `/cmd_vel`에 발행해야 한다.

실기 launch는 파이에서 재현된 Fast DDS 공유메모리 user-data 장애를 피하도록 자식 노드 시작 전에 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`를 설정한다. `real_bringup.launch.py`, `cartographer_real.launch.py`, `autonomous_mapping.launch.py`를 사용하면 transport 별도 설정이 필요 없다. launch 밖에서 실기 ROS 노드를 직접 실행할 때도 같은 환경변수를 적용한다. 로봇의 ROS domain은 12이며 비대화형 셸에서는 `.bashrc`가 적용되지 않을 수 있으므로 아래 실행 예시처럼 domain과 discovery 범위를 명시한다.

## 시작 전 안전 조건

- 로봇은 바닥의 평평하고 열린 곳에 정지시킨다. 사람, 반려동물, 케이블, 계단과 낙하 지점을 작업 구역에서 치운다.
- 작업자는 로봇 옆에서 전원을 즉시 끌 수 있어야 한다. 무인 운전이나 다른 방에서의 운전을 금지한다.
- 3S 배터리 전압과 구동 상태를 먼저 확인한다. explorer는 `/battery_state`가 10초보다 오래됐거나 전압이 10.5V 미만이면 `battery_ok=False`로 판단해 시작·재개와 계속 주행을 fail-closed로 막는다. 센서 자동 차단과 별개로 작업자도 전압을 확인한다.
- `jdamr-webteleop`을 중지하고 `/cmd_vel` publisher가 Collision Monitor 하나뿐인지 확인한다. webteleop은 `/cmd_vel`을 직접 발행하므로 함께 실행하면 explorer가 시작을 거부한다.
- `/scan`, `/map`, `/odom`, `map -> base_footprint` TF가 최신인지, Nav2 planner/navigator가 준비됐는지, Collision Monitor lifecycle `ACTIVE` 응답이 1초 이내인지 확인한다.
- G4는 높이 약 0.1 m의 단일 평면만 본다. 투명 물체, 검은/반사 재질, 라이다 평면보다 낮거나 높은 장애물, 빠르게 움직이는 사람을 확실히 검출하지 못한다. Collision Monitor는 안전 인증 장치가 아니며 작업자의 감시를 대체하지 않는다.

## 실행과 수동 제어

빌드 후 환경을 source하고 Cartographer 및 하드웨어가 이미 실행 중인 상태에서 자율 매핑 launch를 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/jdamr_cube_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
ros2 launch jdamr_cube_navigation autonomous_mapping.launch.py use_sim_time:=false
```

## 저장 지도 자율주행의 금지구역

복도 반복 수집처럼 이미 저장된 지도로 이동할 때는 `keepout_navigation.launch.py`만 사용한다. 이 launch는 AMCL과 저장 지도로 위치를 잡고, 동일한 금지구역 마스크를 global/local costmap에 함께 적용하며 전용 RViz도 기본으로 연다. RViz에는 원본 지도, `/keepout_filter_mask`, AMCL 파티클, 라이다, 전역 계획 경로와 초기 위치·Nav2 목표 도구가 미리 설정돼 있다. 마스크 또는 filter info 서버가 종료되면 전체 자율주행도 종료한다. 일반 `navigation.launch.py`의 `use_keepout:=false` 상태로 복도 자율주행을 시작하지 않는다.

2026-09-01 RViz `Publish Point`로 확정한 두 영역은 `config/keepout_zones.autonomous_20260826.yaml`에 map-frame 다각형으로 반영했다. 왼쪽 계단 입구는 0.55m 팽창 뒤 경계가 실제 벽선과 일치하도록 보정했고, 오른쪽 가지도 입구 전체를 막았다. 마스크 생성기는 본선의 지정 시작점 `(-0.002, 0.000)`에서 끝점 `(37.498, -4.300)`까지 0.25m 장애물 여유를 적용한 격자 연결성을 다시 계산하며, 경로가 끊기면 파일 생성을 거부한다. 현재 저장 지도에 사용할 마스크는 아래 명령으로 재생성한다.

```bash
cd "$HOME/jdamr_cube_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run jdamr_cube_navigation keepout_mask build \
  --zones "$HOME/jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_navigation/config/keepout_zones.autonomous_20260826.yaml" \
  --output-prefix "$HOME/maps/autonomous_20260826T161908_keepout_multi" \
  --force
ros2 run jdamr_cube_navigation keepout_mask validate \
  --map "$HOME/maps/autonomous_20260826T161908.yaml" \
  --mask "$HOME/maps/autonomous_20260826T161908_keepout_multi.yaml"
```

금지구역 좌표를 다시 정하려면 전용 capture launch를 실행하고 RViz의 `Publish Point`로 경계 꼭짓점을 클릭한다. 아래 예시는 네 번 클릭할 때마다 사각형 한 곳을 저장하며, 클릭 순서와 무관하게 중심점 둘레로 꼭짓점을 정렬해 교차 다각형을 방지한다. 저장 뒤 화면은 열린 상태를 유지하므로 다음 사각형을 계속 네 번 클릭할 수 있다. 모든 사각형을 다 찍은 뒤 launch 터미널에서 `Ctrl-C`로 종료한다. 완성된 사각형은 하나의 YAML에 `keepout_area_1`, `keepout_area_2` 순으로 누적되고, 미완성 클릭은 저장하지 않는다. 이 launch는 저장 지도 server, RViz, 클릭 수집기만 실행하며 planner, controller, navigator를 시작하지 않으므로 목표나 속도 명령을 보내지 않는다.

```bash
cd "$HOME/jdamr_cube_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch jdamr_cube_navigation keepout_capture.launch.py \
  map:="$HOME/maps/autonomous_20260826T161908.yaml" \
  output:="$HOME/maps/autonomous_20260826T161908_keepout_zones.yaml" \
  zone_id:=stairs \
  point_count:=4 \
  margin:=0.55
```

`margin 0.55`는 금지 다각형 바깥으로 추가 차단하는 거리다. 도구는 0.35m 미만을 거부하며, 실제 위치추정 오차가 크면 더 늘린다. 클릭 결과를 마스크로 만들고 원본 지도와 크기·해상도·원점이 같은지 검증한다.

```bash
ros2 run jdamr_cube_navigation keepout_mask build \
  --zones "$HOME/maps/autonomous_20260826T161908_keepout_zones.yaml" \
  --output-prefix "$HOME/maps/autonomous_20260826T161908_keepout"
ros2 run jdamr_cube_navigation keepout_mask validate \
  --map "$HOME/maps/autonomous_20260826T161908.yaml" \
  --mask "$HOME/maps/autonomous_20260826T161908_keepout.yaml"
```

활성화 전에는 RViz에 `/keepout_filter_mask`를 Map display로 추가해 원본 지도와 겹쳐 보고, 두 서버가 `active`인지 확인한다. 금지구역 안 waypoint나 경로 실패는 건너뛰지 않고 전체 waypoint 실행을 중단한다.

```bash
ros2 launch jdamr_cube_navigation keepout_navigation.launch.py \
  map:="$HOME/maps/autonomous_20260826T161908.yaml" \
  keepout_mask:="$HOME/maps/autonomous_20260826T161908_keepout.yaml"
ros2 lifecycle get /keepout_filter_mask_server
ros2 lifecycle get /keepout_costmap_filter_info_server
ros2 topic echo --once /keepout_costmap_filter_info
```

단일 PC 정적 진단에서는 `keepout_navigation.launch.py`의 RViz 기본값을 사용할 수 있다.
파이 온보드 실차 주행에서는 아래 전용 wrapper가 RViz를 포함하지 않고, 노트북의
view-only RViz에서 `Keepout Zones`와 계획 경로가 금지구역을 침범하지 않는지 목표 전송
전에 확인한다.

이 모드는 기존 지도를 경로 통제에만 사용한다. 실차 수집 중 Cartographer나 SLAM Toolbox mapping을 동시에 실행하지 않는다. 원시 `/scan`, `/odom`, TF를 bag으로 기록한 뒤, `offline_replay_guard.launch.py`로 `/map`과 이동 명령을 재생 목록에서 제외하고 기록된 AMCL의 `map -> odom`을 TF에서 제거한다. 새 mapping backend는 격리 domain의 빈 상태에서 실행한다. 따라서 Keepout이나 저장 지도가 새 지도 결과를 덮어쓰거나 정답으로 주입되지 않는다. 실행 절차는 `evaluation/README.md`의 "저장 지도 주행 bag의 오프라인 SLAM 재생"을 따른다.

## Wi-Fi 비의존 실차 구조

2026-09-01 복도 왕복에서는 노트북에서 Nav2·RViz·rosbag을 함께 실행하자 복도 끝에서
ping 20% 손실, 평균 약 710ms, 최대 약 1.1초 지연이 발생했다. `/scan`, `/odom`, TF가
2초 이상 늦어져 Collision Monitor가 정지했고, recorder 종료 직후 센서가 다시
수신됐다. 이 결과로 실차의 제어 루프와 원본 데이터 수집을 Wi-Fi 건너편에서 실행하는
구성을 폐기한다.

파이에는 하드웨어 bringup이 먼저 실행돼 있어야 한다. 그 위에는 저장 지도·AMCL,
controller, planner, velocity smoother, Collision Monitor, BT navigator, Keepout,
경로 실행기, 필수 토픽 MCAP recorder만 둔다. 파이 전용 launch는 쓰지 않는 smoother
server, route server, behavior server, waypoint follower, docking server를 실행하지
않는다. RViz, Cartographer·SLAM Toolbox, 그래프·통계 계산, bag 재생은 주행 중 파이에
띄우지 않는다.
recorder는 CPU nice 10과 I/O best-effort 최저 우선순위 7로 실행해 제어 루프가 먼저
스케줄되게 한다. 기본 기록도 `/scan`, `/odom`, TF, IMU, 제어 전·후 속도, AMCL,
배터리, 계획, Collision Monitor 상태로 제한하며 RViz용 `/joint_states`는 제외한다.
recorder가 예상치 않게 끝나면 navigation도 종료한다.

```bash
cd "$HOME/jdamr_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch jdamr_cube_navigation onboard_keepout_navigation.launch.py \
  map:="$HOME/maps/autonomous_20260826T161908.yaml" \
  keepout_mask:="$HOME/maps/autonomous_20260826T161908_keepout_multi.yaml"
```

노트북은 지도·마스크·AMCL·계획 경로만 보는 저대역폭 RViz를 실행한다. 원시 LaserScan과
전체 Global Costmap 표시는 기본 OFF이고 frame rate는 10Hz다. 필요할 때만 잠깐 켠다.
노트북에서는 rosbag을 동시에 기록하지 않는다.

```bash
cd "$HOME/jdamr_cube_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=12
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch jdamr_cube_navigation keepout_operator_view.launch.py
```

검증된 20-point 왕복 경로는
`config/corridor_roundtrip.autonomous_20260826.yaml`에 저장돼 있다. 다음 두 명령도
파이에서 실행한다. 첫 명령은 80m 전체 경로를 계획만 하고, 두 번째 명령만 실제로
움직인다. `corridor_route`는 waypoint마다 회전·후진·대기·재시도가 없는 복도 전용
fail-fast BT를 명시한다. 배터리·scan·odom freshness 또는 AMCL 공분산 게이트가
깨지면 현재 목표를 취소하며 다음 waypoint를 보내지 않는다.

```bash
ros2 run jdamr_cube_navigation corridor_route \
  --route "$HOME/jdamr_ws/src/jdamr_cube_ros/jdamr_cube_navigation/config/corridor_roundtrip.autonomous_20260826.yaml"
ros2 run jdamr_cube_navigation corridor_route \
  --route "$HOME/jdamr_ws/src/jdamr_cube_ros/jdamr_cube_navigation/config/corridor_roundtrip.autonomous_20260826.yaml" \
  --execute
```

2026-09-01 노트북 기록은 106.9MiB·128,791개 메시지로 정상 종료됐지만 transport
loss 136건과 미완주가 있어 진단용이다. 상세 수치·해시·실패 시점은
`evaluation/20260901_CORRIDOR_KEEPOUT_RUN.md`에 고정했다. 파이 로컬 기록에서 손실 0과
완전 왕복을 확인하기 전에는 최종 포트폴리오 데이터로 승격하지 않는다. 다음 실차에서는
출발 전 1분 정적 소크에서 4코어 load average가 4.0 미만이고 thermal throttle이 없음을
확인한 뒤 주행한다. 최소 온보드 구성은 107.862초 정적 소크에서 load1 1.17~1.93,
66.2~70.6°C, throttle 0을 기록했고 MCAP 16,150개 메시지의 CRC가 통과했다. 속도 명령은
전부 0이었고 정지 변위는 0.025mm였다. 사용자의 결정으로 별도 2~6m 단거리 단계는
생략하고 전체 왕복으로 진행한다. 실제 출발 시 작업자가 로봇 옆에서 물리 전원을 즉시
차단할 수 있어야 한다.

자율 매핑은 항상 `autonomous_mapping.launch.py`로 실행한다. `ros2 run jdamr_cube_navigation frontier_explorer` 단독 실행은 explorer 오류 시 전체 Nav2 종료를 보장하지 않으므로 금지한다.

자율 매핑과 노트북 실행 기본값은 Nav2 lifecycle 노드를 별도 프로세스로 띄운다(`use_composition=False`). 실기 scan/TF 부하에서 한 구성 컨테이너가 멈추면 planner와 navigator action server가 함께 사라지는 결합을 피하려는 설정이다. 파이의 저장지도 복도 주행만 `onboard_nav2_core.launch.py`의 최소 구성 composition을 사용한다. 공식 전체 bringup 정적 시험의 load1 9.33~11.27을 1.17~1.93으로 낮췄다. 베이스는 `/odom`과 IMU를 약 50Hz로 유지하면서 `odom -> base_footprint` TF만 20Hz로 제한하고, Cartographer의 `map -> odom`도 20Hz로 발행한다. explorer 프로세스가 종료되면 전체 자율 매핑 launch가 종료되도록 event handler가 등록돼 있다.

운용 종료는 launch를 실행한 터미널의 `Ctrl-C` 또는 launch process group 종료로 수행한다. Nav2 자식 프로세스 하나만 직접 `kill`하면 lifecycle 정리와 최종 정지 상태를 확인하기 어렵다. Jazzy 합성 컨테이너는 종료 중 lifecycle race로 SIGSEGV를 남길 수 있으므로 종료 뒤 Nav2·recorder 잔류 0과 `/cmd_vel` publisher 0을 반드시 확인한다. 이 현상은 활성 주행 실패와 분리해서 기록하며 정상 종료 품질 개선 항목으로 남긴다.

2026-08-26 팔 장착 전 실기 자율탐사에서는 최종 상태 `FINISHED`, `save=succeeded`, `readiness_missing=none`을 확인했다. 저장 지도는 894×212셀(0.05m/셀), 오도메트리 누적 경로는 45.018m였다. 이 결과는 당시 복도와 작업자 감시 조건에서의 자율 매핑 기록이며 안전 인증이나 무인 운전 승인을 뜻하지 않는다.

`autostart:=true` 기본값은 Nav2와 map saver lifecycle 노드를 활성화할 뿐이다. explorer는 자동 출발하지 않고 `IDLE`을 유지한다. 안정 운용 API는 다음과 같다.

```bash
ros2 service call /autonomy/start std_srvs/srv/Trigger '{}'
ros2 service call /autonomy/pause std_srvs/srv/Trigger '{}'
ros2 service call /autonomy/resume std_srvs/srv/Trigger '{}'
ros2 service call /autonomy/stop std_srvs/srv/Trigger '{}'
```

기존 `/frontier_explorer/start`, `/frontier_explorer/pause`, `/frontier_explorer/resume`, `/frontier_explorer/stop`도 호환 alias로 유지된다. 새 운용 스크립트에는 `/autonomy/*`를 사용한다.

`start`와 `resume`은 최신 odom으로 확인한 로봇 정지, 최신 map/scan/TF와 배터리, Collision Monitor `ACTIVE`, planner/navigator 준비, `/cmd_vel` 단일 소유가 모두 확인되지 않으면 실패한다. 목표가 진행 중일 때 map/scan/TF/odom의 짧은 freshness 공백은 explorer가 중복으로 `PAUSED`에 고정하지 않는다. Nav2 controller와 Collision Monitor가 데이터를 기다리며 자동 회복한다. Collision Monitor 비활성, planner/navigator 이탈, 명령 소유권 충돌, 저전압과 `/probe_abort`는 그대로 목표를 취소하고 `PAUSED`에 머문다.

SELECT 상태에서 현재 robot pose가 지도 밖이거나 known-free가 아니면 다음 지도 갱신까지 기다린다. 움직임이 없는 단계라 수동 복구를 요구하지 않는다. 최종 경로 가능 여부는 `allow_unknown=false`인 Nav2 global costmap과 planner 결과가 판정한다.

Nav2 목표 취소가 3초 안에 응답하지 않거나 취소가 거부되면 explorer는 오류로 종료한다. launch는 explorer 프로세스 종료를 감지하면 전체 자율 매핑과 Nav2를 함께 종료하는 fail-closed 경계로 동작한다.

`pause`는 저장 없이 즉시 안전정지를 요청한다. `stop`은 cancellation terminal과 최신 map/odom 기반 정지를 확인한 뒤 지도를 자동 저장하며, 그동안 `save=waiting_for_stop`이다. `motion_transition=clear`와 `save=succeeded`를 확인하기 전에는 별도 저장을 요청하거나 로봇을 들지 않는다.

상태는 `/frontier_explorer/status`에서 `state`, `fault`, `goal`, `battery_voltage`, `battery_ok`, `stationary`, `readiness_missing`, `collision_lifecycle_error`, `save`, `motion_transition`, `frontiers`, `map_sequence` 필드로 확인한다. `readiness_missing`은 시작 또는 운행을 막은 조건이며 `none`이면 누락 조건이 없다. `collision_lifecycle_error`는 최근 Collision Monitor lifecycle 서비스 조회 예외를 `<ExceptionType>: <message>`로 표시하고, 정상 응답을 받으면 `none`으로 복구한다. 이 값이 `none`이 아니면 fail-closed로 운행을 차단한다. 정상 탐색 이동 중 `stationary=false`는 차단 조건으로 중복 표시하지 않고, `SETTLING`에서는 저장 대기 조건으로 표시한다. 새 지도나 새 탐색 세대의 frontier 계산 전에는 `frontiers=unknown`이다.

설정은 후진을 사용하지 않고 낮은 선속도·각속도와 Regulated Pure Pursuit, known-space-only 계획을 사용한다. 그래도 장애물에 접근하면 즉시 `pause` 또는 `stop`하고 필요하면 물리 전원을 끈다.

## 지도 저장과 종료

탐색 완료 또는 `/autonomy/stop` 시 explorer가 취소 완료와 정지 상태를 확인한 뒤 `/map_saver/save_map`을 호출해 기본적으로 `/home/lim/maps/autonomous_YYYYmmddTHHMMSS.{yaml,pgm}`을 저장한다. 저장 요청 직전에 prefix의 부모 디렉터리를 자동 생성하며, 생성할 수 없으면 map saver를 호출하지 않고 `save=failed`로 중단한다. 자동 저장 없이 중간 지도를 수동 저장할 때는 `/autonomy/pause` 후 `motion_transition=clear`와 로봇 정지를 확인하고 안정 API를 호출한다.

```bash
ros2 service call /autonomy/save_map std_srvs/srv/Trigger '{}'
ros2 topic echo --once /frontier_explorer/status
```

수동 저장은 상태가 `IDLE`, `PAUSED`, `FINISHED` 중 하나이고 map이 최신이며 로봇이 정지했고 map saver가 준비됐을 때만 접수된다. Trigger 응답의 `success=true`, `map save request accepted`는 **비동기 요청이 접수됐다는 뜻이지 파일 저장 완료를 뜻하지 않는다.** `/frontier_explorer/status`의 `save`가 `in_progress`에서 `succeeded`로 바뀐 뒤 파일을 확인한다. `rejected`는 요청 미접수, `failed`는 디렉터리 준비 실패 또는 접수 후 저장 실패다.

```bash
ls -lt /home/lim/maps/autonomous_*.yaml /home/lim/maps/autonomous_*.pgm | head
```

YAML의 `image:` 경로가 실제 PGM을 가리키는지 확인한다. **로봇을 들거나 라이다 위치를 바꾸기 전에 반드시 탐색을 중단하고 `save=succeeded`와 파일 생성을 확인한다.** 로봇을 든 채 Cartographer를 계속 실행하면 odom과 scan의 일관성이 깨져 잘 만들어진 지도도 손상된다.
