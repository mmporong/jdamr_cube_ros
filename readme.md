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

- 기본 월드: `jdamr_cube_gazebo/worlds/empty.world`
- 로봇 스폰 위치는 `x_pose`/`y_pose`/`z_pose` launch 인자로 조정 가능
- teleop 연결: `ros2 run jdamr_cube_teleop jdamr_cube_teleop` (Twist를 `/cmd_vel`로 발행 → 브릿지를
  통해 Gazebo diff drive 플러그인으로 전달)

## 참고

- 실물 하드웨어용 `jdamr_cube_bringup`/`jdamr_cube_node`(시리얼 포트, IMU 등)는 이번 검증 범위에
  포함되지 않았습니다(WSL에는 해당 하드웨어가 없음). 빌드 성공 여부만 확인했습니다.
- 검증은 WSL(Ubuntu 24.04) 상에서 소스 트리를 별도 임시 워크스페이스로 복사해 진행했으며,
  저장소에 커밋되어 있는 `build/`, `install/`, `log/` 디렉터리는 건드리지 않았습니다. 필요 시
  워크스페이스 루트에서 직접 `colcon build`를 실행해 새로 갱신하세요.
