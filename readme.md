# jdamr_cube_ros

JD-AMR "cube" 로봇용 ROS 2 워크스페이스. 실물 하드웨어 브링업, URDF/디스크립션, 라이다 드라이버, teleop, cartographer 기반 SLAM, Gazebo 시뮬레이션 패키지로 구성되어 있습니다.

## 패키지 구성

| 패키지 | 설명 |
|---|---|
| `jdamr_cube_description` | URDF, RViz 디스플레이 launch |
| `jdamr_cube_bringup` | 실물 로봇 구동(모터 노드 + 라이다 + robot_state_publisher) |
| `jdamr_cube_node` | 모터/오도메트리 하드웨어 인터페이스 노드 |
| `jdamr_cube_teleop` | 키보드 teleop (`geometry_msgs/Twist` → `cmd_vel`) |
| `jdamr_cube_cartographer` | Cartographer 기반 2D SLAM |
| `jdamr_cube_gazebo` | Gazebo 시뮬레이션 launch/월드/브릿지 설정 |
| `jdamr_cube_controller` | Nav2 기반 좌표 이동(`goto_pose`). 저장된 맵을 로드해 목표 좌표로 자율주행 |
| `ldlidar_sl_ros2` | LDRobot LD14 라이다 드라이버 (C++) |

## 빌드 환경 검증 (ROS 2 Jazzy / WSL Ubuntu 24.04)

`ROS_DISTRO=jazzy`, Ubuntu 24.04(Noble) 기준으로 전체 워크스페이스 빌드를 검증했습니다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/jdamr_cube_ws
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
colcon build --symlink-install
```

결과: 7개 패키지(`jdamr_cube_bringup`, `jdamr_cube_cartographer`, `jdamr_cube_description`,
`jdamr_cube_gazebo`, `jdamr_cube_node`, `jdamr_cube_teleop`, `ldlidar_sl_ros2`) 모두 정상 빌드,
`rosdep install` 정상 통과 확인. `jdamr_cube_gazebo/launch/gazebo.launch.py`를 실제로 실행해
`/scan`, `/odom`, `/tf`, `/imu/data`, `/joint_states`, `/clock` 토픽에 데이터가 정상적으로
흐르는 것까지 확인했습니다.

### 발견된 문제와 수정 내역

1. **`ldlidar_sl_ros2` 컴파일 실패 (실제 빌드 에러)**
   - 증상: `log_module.cpp`에서 `pthread_mutex_init` 등이 "not declared in this scope" 에러로 컴파일 실패.
   - 원인: `<pthread.h>`를 직접 include하지 않고 다른 헤더의 전이(transitive) include에 의존하고 있었는데,
     Jazzy(Ubuntu 24.04, GCC 13) 툴체인에서는 그 전이 include가 더 이상 발생하지 않음.
   - 수정: [`ldlidar_driver/src/log_module.cpp`](ldlidar_sl_ros2/ldlidar_driver/src/log_module.cpp)에
     `#include <pthread.h>` 명시적 추가.

2. **`jdamr_cube_gazebo`가 Gazebo Classic(`gazebo_ros`)에 의존 → Jazzy에서 사용 불가**
   - 증상: `rosdep install` 시 `Cannot locate rosdep definition for [gazebo_ros]` 에러.
   - 원인: ROS 2 Jazzy는 Gazebo Classic을 공식 지원하지 않고, 새 Gazebo(Harmonic, `gz-sim` /
     `ros_gz_*` 패키지군)를 기본 시뮬레이터로 사용함. 기존 URDF의 `<gazebo>` 플러그인들
     (`libgazebo_ros_diff_drive.so`, `libgazebo_ros_ray_sensor.so` 등)과 `gazebo.launch.py`의
     `gzserver.launch.py`/`spawn_entity.py` 사용도 모두 Gazebo Classic 전용이라 Jazzy에서 동작 불가.
   - 수정 (Gazebo Harmonic으로 전면 이관):
     - [`jdamr_cube_description/urdf/jdamr_cube.urdf`](jdamr_cube_description/urdf/jdamr_cube.urdf):
       diff drive / joint state publisher / IMU / 라이다 플러그인을 gz-sim 시스템 플러그인
       (`gz-sim-diff-drive-system`, `gz-sim-joint-state-publisher-system`, `gpu_lidar`/`imu` 센서)으로 재작성.
     - [`jdamr_cube_gazebo/worlds/empty.world`](jdamr_cube_gazebo/worlds/empty.world): gz-sim SDF 포맷
       (Physics/Sensors/Imu/SceneBroadcaster 등 시스템 플러그인 포함)으로 재작성.
     - [`jdamr_cube_gazebo/launch/gazebo.launch.py`](jdamr_cube_gazebo/launch/gazebo.launch.py):
       `ros_gz_sim`의 `gz_sim.launch.py`로 시뮬레이터 기동, `ros_gz_sim create`로 로봇 스폰,
       `ros_gz_bridge`로 ROS ↔ Gazebo Transport 토픽 브리지.
     - [`jdamr_cube_gazebo/params/bridge.yaml`](jdamr_cube_gazebo/params/bridge.yaml) 신규 추가:
       `cmd_vel`, `odom`, `tf`, `scan`, `imu/data`, `joint_states`, `clock` 브리지 정의.
     - `package.xml`: `gazebo_ros` 의존성 제거, `ros_gz_sim`/`ros_gz_bridge` 추가.

3. **일부 패키지의 `package.xml`에 실행 시 필요한 의존성 누락**
   - `jdamr_cube_bringup`이 `jdamr_cube_description`, `jdamr_cube_node`, `ldlidar_sl_ros2`,
     `robot_state_publisher`를 launch에서 사용하지만 선언이 없었음 → 추가.
   - `jdamr_cube_cartographer`가 `cartographer_ros`, `rviz2`를 사용하지만 선언이 없었음 → 추가.
   - `jdamr_cube_description`이 `joint_state_publisher_gui`, `rviz2`, `robot_state_publisher`를
     사용하지만 선언이 없었음 → 추가.
   - 빌드 자체를 막는 문제는 아니었지만(ament_python은 컴파일 단계가 없어 미선언 의존성이 있어도
     `colcon build`는 통과함), 새 환경에서 `rosdep install`만으로 필요한 시스템 패키지를 모두
     설치할 수 있도록 정리함.

## Gazebo 시뮬레이션 실행 (Jazzy / Gazebo Harmonic)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch jdamr_cube_gazebo gazebo.launch.py
```

- 기본 월드: `jdamr_cube_gazebo/worlds/empty.world` (바닥만 있는 빈 월드, 기본 동작 확인용)
- 벽/장애물이 있는 방 월드도 포함되어 있습니다: `jdamr_cube_gazebo/worlds/room.world`
  (6m x 6m 방 + 박스/원기둥 장애물). SLAM 맵을 생성하려면 라이다가 뭔가를 감지할 수 있어야 하므로
  이 월드를 사용해야 합니다. 아래처럼 `world` 인자로 지정합니다.

  ```bash
  ros2 launch jdamr_cube_gazebo gazebo.launch.py \
    world:=$(ros2 pkg prefix jdamr_cube_gazebo)/share/jdamr_cube_gazebo/worlds/room.world
  ```

- 로봇 스폰 위치는 `x_pose`/`y_pose`/`z_pose` launch 인자로 조정 가능
- teleop 연결: `ros2 run jdamr_cube_teleop jdamr_cube_teleop` (Twist를 `/cmd_vel`로 발행 → 브릿지를
  통해 Gazebo diff drive 플러그인으로 전달)

## Gazebo + Cartographer로 맵 생성하기

### 1. 시뮬레이터 + SLAM 실행

**Windows에서 배치 파일로 한 번에 실행**: 저장소 루트의 [`run_gazebo_slam.bat`](run_gazebo_slam.bat)을
더블클릭(또는 `cmd`에서 실행)하면 WSL 안에서 자동으로 워크스페이스를 빌드(`scripts/wsl_build.sh`,
소스를 `~/jdamr_cube_ws`로 rsync 후 `colcon build`)한 뒤, Gazebo / Cartographer+RViz / Teleop을
각각 별도 콘솔 창으로 띄워줍니다. WSL(Ubuntu 24.04, ROS 2 Jazzy)이 설치되어 있어야 합니다.
내부적으로 사용하는 스크립트는 [`scripts/`](scripts) 폴더에 있으며, 아래 수동 실행 절차와 동일한
동작을 합니다.

수동으로 터미널 3개를 띄워 실행할 수도 있습니다.

```bash
# 1) Gazebo (장애물이 있는 room.world 사용)
source install/setup.bash
ros2 launch jdamr_cube_gazebo gazebo.launch.py \
  world:=$(ros2 pkg prefix jdamr_cube_gazebo)/share/jdamr_cube_gazebo/worlds/room.world

# 2) Cartographer + RViz (map, scan, robot model이 미리 세팅된 rviz 설정 로드)
source install/setup.bash
ros2 launch jdamr_cube_cartographer cartographer.launch.py

# 3) 로봇 조종 (방 구석구석을 라이다가 볼 수 있도록 천천히 이동/회전)
source install/setup.bash
ros2 run jdamr_cube_teleop jdamr_cube_teleop
```

RViz의 `Map` 디스플레이(고정 프레임 `map`)에 점유격자 지도가 실시간으로 채워지는 것을 확인할 수
있습니다. 로봇을 방 전체에 대해 한 바퀴 돌리면 벽과 장애물 윤곽이 모두 채워집니다.

### 2. 맵 저장

**Windows에서 배치 파일로**: `run_gazebo_slam.bat`으로 시뮬레이터 + cartographer를 띄운 상태에서
저장소 루트의 [`save_map.bat`](save_map.bat)을 실행하면 됩니다.

```bat
save_map.bat                 :: 기본 이름(jdamr_cube_room)으로 저장
save_map.bat my_room         :: 이름을 직접 지정해서 저장
```

내부적으로 [`scripts/wsl_save_map.sh`](scripts/wsl_save_map.sh)가 `nav2_map_server`의
`map_saver_cli`로 현재 `/map` 토픽을 `~/maps/<이름>.pgm`(WSL)에 저장한 뒤, Windows에서 바로 볼 수
있도록 저장소의 `maps/<이름>.pgm` / `.yaml`로도 복사합니다(`maps/`는 `.gitignore`에 등록되어
있어 커밋되지 않습니다).

**수동으로 실행**: `nav2_map_server`의 `map_saver_cli`로 직접 저장할 수도 있습니다.

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/jdamr_cube_room
```

`~/maps/jdamr_cube_room.pgm`(이미지)과 `~/maps/jdamr_cube_room.yaml`(메타데이터: 해상도, 원점,
occupied/free threshold)이 생성됩니다. 실제로 방(벽 4면 + 박스/원기둥 장애물)을 반영한 맵이
생성되는 것까지 확인했습니다.

### 3. 저장한 맵으로 좌표까지 자율주행하기 (`jdamr_cube_controller`)

저장된 맵(.yaml)을 로드해서 Nav2(AMCL 로컬라이제이션 + 경로계획/추종)를 띄우고, 좌표를 주면
그 지점까지 이동시키는 [`jdamr_cube_controller`](jdamr_cube_controller) 패키지가 있습니다.

```bash
# 1) Gazebo (SLAM 없이, room.world)
source install/setup.bash
ros2 launch jdamr_cube_gazebo gazebo.launch.py \
  world:=$(ros2 pkg prefix jdamr_cube_gazebo)/share/jdamr_cube_gazebo/worlds/room.world

# 2) Nav2 (기본값으로 ~/maps/jdamr_cube_room.yaml을 로드함)
source install/setup.bash
ros2 launch jdamr_cube_controller navigation.launch.py
# 다른 맵을 쓰려면: ros2 launch jdamr_cube_controller navigation.launch.py map:=/path/to/map.yaml

# 3) 목표 좌표로 이동 (map 프레임 기준 x, y[, --yaw])
source install/setup.bash
ros2 run jdamr_cube_controller goto_pose 1.0 -1.8
ros2 run jdamr_cube_controller goto_pose 0.0 0.0 --yaw 1.57
```

`goto_pose`는 `navigate_to_pose` 액션으로 목표를 보내고 도착/실패까지 대기하는 1회성 CLI 노드로,
성공하면 종료 코드 0, 실패(장애물에 너무 붙은 목표, 경로 없음 등)하면 1을 반환합니다.

로봇은 room.world에 스폰될 때 항상 `(0, 0, 0)`이고 맵 원점도 그 지점이므로, AMCL이
`jdamr_cube_controller/config/nav2_params.yaml`의 `set_initial_pose`로 시작 시 자동으로
초기 위치를 잡습니다(RViz에서 수동으로 "2D Pose Estimate"를 클릭할 필요 없음). 시각적으로
확인하고 싶다면 `rviz2`를 별도로 띄우고 Fixed Frame을 `map`으로, Map/LaserScan/RobotModel
디스플레이를 추가해서 보면 됩니다.

`jdamr_cube_controller/config/nav2_params.yaml`은 Jazzy 기본 `nav2_bringup` 파라미터를 베이스로
jdamr_cube의 실제 몸체 크기(0.5 x 0.3m + 라이다/휠 돌출부 반영 footprint)와 저속 실내 로봇에 맞는
속도 제한만 조정한 것입니다. 장애물/벽에 너무 붙은(수십 cm 이내) 좌표를 목표로 주면 costmap
inflation 때문에 `xy_goal_tolerance`(0.15m) 안으로 들어가지 못하고 근처에서 멈출 수 있습니다 —
실제 Nav2 동작이며 버그는 아닙니다. 목표는 장애물/벽에서 최소 0.3~0.5m 이상 떨어뜨려 주세요.

### 발견된 문제와 수정 내역 (cartographer 관련)

Gazebo(room.world)를 띄운 상태에서 `cartographer.launch.py`를 실제로 실행하며 아래 문제들을
순서대로 발견하고 수정했습니다. 모두 [`jdamr_cube_cartographer/config/jdamr_cube_2d.lua`](jdamr_cube_cartographer/config/jdamr_cube_2d.lua)
설정 문제였고, 로그에는 `cartographer_node`가 즉시 `FATAL`로 죽는 형태로 나타났습니다.

1. **`publish_frame_projected_to_2d` 키 누락 → `cartographer_node` 즉시 크래시**
   - `Check failed: HasKey(key) Key 'publish_frame_projected_to_2d' not in dictionary`.
   - Jazzy에 설치된 cartographer_ros 버전이 이 옵션을 필수로 요구함. `false`로 추가.

2. **더 이상 쓰이지 않는 `provide_untracked_odom_frame` 키 → 크래시**
   - `Key 'provide_untracked_odom_frame' was used the wrong number of times` (cartographer의
     lua 옵션 dictionary는 정의는 됐지만 코드에서 한 번도 읽지 않는 키가 있으면 fatal 처리함).
   - `publish_frame_with_odometry`와 함께 제거(둘 다 이 버전에서 읽히지 않는 옵션).

3. **`published_frame`과 `odom_frame`이 둘 다 `"odom"`인데 `provide_odom_frame = true` →
   `TF_SELF_TRANSFORM` 에러 반복 발행**
   - cartographer가 `odom → odom` 자기 자신에게 transform을 발행하려 시도.
   - `provide_odom_frame = false`로 변경(map → odom만 cartographer가 발행하고, odom → base_footprint는
     이미 Gazebo diff drive 플러그인이 발행하므로 그대로 사용). turtlebot3_cartographer의
     검증된 설정과 동일한 패턴으로 맞춤.

4. **`use_odometry = true` → 트래젝토리 시작 직후 `map_by_time.h` 타임스탬프 assertion으로 크래시**
   - `Check failed: data.time > ... (X vs. X)`: gz-sim DiffDrive 플러그인이 스폰 직후 첫 `/odom`
     메시지를 동일한 시뮬레이션 타임스탬프로 중복 발행하는 경우가 있어 cartographer의 시간순 큐가 깨짐.
   - `use_odometry = false`로 변경. `use_online_correlative_scan_matching = true`가 이미 켜져
     있어 라이다 스캔 매칭만으로도 정상 동작함을 확인.

5. **`jdamr_cube_gazebo/worlds/empty.world`에는 벽/장애물이 전혀 없어 라이다 값이 항상 `.inf`
   → cartographer가 `Dropped empty horizontal range data` 경고만 반복하고 맵을 만들지 못함**
   - 코드 버그는 아니지만 맵 생성 데모 자체가 불가능한 환경 문제. 벽 4면 + 장애물 2개로 구성된
     [`jdamr_cube_gazebo/worlds/room.world`](jdamr_cube_gazebo/worlds/room.world)를 신규 추가.

6. **`jdamr_cube_cartographer.launch.py`가 존재하지 않는 `rviz/jdamr_cube_cartographer.rviz`를
   참조 → rviz2가 지정한 화면 구성 없이 기본 빈 화면으로 뜸**
   - `rviz2`는 `-d`에 존재하지 않는 경로를 줘도 에러 없이 조용히 기본 설정으로 대체되기 때문에
     `colcon build`/실행 단계에서는 드러나지 않던 문제.
   - [`jdamr_cube_cartographer/rviz/jdamr_cube_cartographer.rviz`](jdamr_cube_cartographer/rviz/jdamr_cube_cartographer.rviz)
     신규 작성(Map, LaserScan, RobotModel, TF, Cartographer 스캔매칭 포인트 디스플레이 포함,
     Fixed Frame = `map`) 및 `setup.py`에 설치 규칙 추가.

위 수정 후 room.world에서 실제로 로봇을 주행시켜 `/map` 토픽에 방 형태(벽 + 장애물)가 반영된
점유격자 지도가 쌓이는 것, `map_saver_cli`로 `.pgm`/`.yaml`이 정상 저장되는 것까지 확인했습니다.

## 참고

- 실물 하드웨어용 `jdamr_cube_bringup`/`jdamr_cube_node`(시리얼 포트, IMU 등)는 이번 검증 범위에
  포함되지 않았습니다(WSL에는 해당 하드웨어가 없음). 빌드 성공 여부만 확인했습니다.
- 검증은 WSL(Ubuntu 24.04) 상에서 소스 트리를 별도 임시 워크스페이스로 복사해 진행했으며,
  저장소에 커밋되어 있는 `build/`, `install/`, `log/` 디렉터리는 건드리지 않았습니다. 필요 시
  워크스페이스 루트에서 직접 `colcon build`를 실행해 새로 갱신하세요.
