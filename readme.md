# jdamr_cube_ros

JD-AMR "cube" 로봇용 ROS 2 워크스페이스. 실물 하드웨어 브링업, URDF/디스크립션, 라이다 드라이버, teleop, cartographer 기반 SLAM, Gazebo 시뮬레이션 패키지로 구성되어 있습니다.

## 패키지 구성

| 패키지 | 설명 |
|---|---|
| `jdamr_cube_description` | URDF(SO-101 팔 포함), RViz 디스플레이 launch, 팔 컨트롤러 설정 |
| `jdamr_cube_bringup` | 실물 로봇 구동(모터 노드 + 라이다 + robot_state_publisher) |
| `jdamr_cube_node` | 모터/오도메트리 하드웨어 인터페이스 노드 |
| `jdamr_cube_teleop` | 키보드 teleop (`geometry_msgs/Twist` → `cmd_vel`) |
| `jdamr_cube_cartographer` | Cartographer 기반 2D SLAM |
| `jdamr_cube_gazebo` | Gazebo 시뮬레이션 launch/월드/브릿지 설정 |
| `jdamr_cube_navigation` | Nav2 기반 좌표 이동(`goto_pose`). 저장된 맵을 로드해 목표 좌표로 자율주행 |
| `jdamr_cube_so101_arm` | SO-101 팔 관절 각도 제어 + 뎁스카메라 픽셀 기반 MoveIt2 pick CLI(`joint_control`) |
| `jdamr_cube_moveit_config` | SO-101 팔 MoveIt2 설정(SRDF/kinematics/controllers/OMPL, `move_group` launch) |
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

## SO-101 로봇 팔

jdamr_cube 몸체 위(base_link 상단면, `arm_mount_joint`)에
[SO-101](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101) 팔이 붙어
있습니다. 링크/조인트/메쉬는 원본 저장소의 `so101_new_calib.urdf`를 그대로 가져오되, jdamr_cube의
기존 이름(`base_link` 등)과 겹치지 않도록 전부 `arm_` 접두사를 붙여
[`jdamr_cube_description/urdf/jdamr_cube.urdf`](jdamr_cube_description/urdf/jdamr_cube.urdf)에
합쳐 넣었습니다. STL 메쉬는 [`jdamr_cube_description/meshes/so101/`](jdamr_cube_description/meshes/so101)에
포함되어 있습니다(Apache-2.0, 출처는 같은 폴더의 `NOTICE.md` 참고).

**관절 구성** (전부 `revolute`, 값 단위 rad):

| 조인트 | 설명 | 범위 |
|---|---|---|
| `arm_shoulder_pan` | 베이스 회전 | -1.92 ~ 1.92 |
| `arm_shoulder_lift` | 어깨 상하 | -1.75 ~ 1.75 |
| `arm_elbow_flex` | 팔꿈치 | -1.69 ~ 1.69 |
| `arm_wrist_flex` | 손목 굽힘 | -1.66 ~ 1.66 |
| `arm_wrist_roll` | 손목 회전 | -2.74 ~ 2.84 |
| `arm_gripper` | 그리퍼 개폐 | -0.17(닫힘) ~ 1.75(열림) |

### 실제로 움직이기 (ros2_control + gz_ros2_control)

팔은 `<ros2_control>` + gz-sim `gz_ros2_control-system` 플러그인으로 시뮬레이션에 연결되어
있어서, 스폰 이후 [`jdamr_cube_gazebo/launch/gazebo.launch.py`](jdamr_cube_gazebo/launch/gazebo.launch.py)가
자동으로 아래 컨트롤러를 로드/activate합니다(컨트롤러 설정:
[`jdamr_cube_description/config/so101_controllers.yaml`](jdamr_cube_description/config/so101_controllers.yaml)).

- `joint_state_broadcaster` — 팔 관절 상태를 `/joint_states`로 발행(휠 관절은 기존처럼 별도
  gz-sim 조인트 상태 플러그인 → 브릿지로 같은 토픽에 함께 발행됩니다)
- `arm_controller` (`joint_trajectory_controller`) — 5개 팔 관절을
  `/arm_controller/follow_joint_trajectory` (`FollowJointTrajectory`) 액션으로 제어
- `gripper_controller` (`GripperActionController`) — 그리퍼를
  `/gripper_controller/gripper_cmd` (`GripperCommand`) 액션으로 제어

```bash
source install/setup.bash
ros2 launch jdamr_cube_gazebo gazebo.launch.py

# 다른 터미널에서 팔 이동 (raw 액션 호출 — 아래 jdamr_cube_so101_arm 패키지가 더 쓰기 편합니다)
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [arm_shoulder_pan, arm_shoulder_lift, arm_elbow_flex, arm_wrist_flex, arm_wrist_roll],
    points: [{ positions: [0.5, -0.3, 0.4, 0.2, 0.0], time_from_start: { sec: 3 } }]
  }
}"

# 그리퍼 닫기/열기 (0.0=닫힘 근처 ~ 1.0=열림 근처)
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 1.0, max_effort: 5.0}}"
```

실제로 `ros2 launch jdamr_cube_gazebo gazebo.launch.py`로 room.world를 띄운 뒤 위 두 액션을
호출해서 관절이 목표 각도까지 정확히 이동하고(`/joint_states`로 확인), `base_link`→`arm_gripper_link`
TF가 그에 맞게 갱신되는 것까지 확인했습니다.

**빌드 중 발견해서 고친 문제**: `<gazebo><plugin filename="gz_ros2_control-system" ...>` 안의
`<parameters>` 태그에 `package://jdamr_cube_description/config/so101_controllers.yaml` 같은
package URI를 그대로 넣으면 gz_ros2_control이 이를 해석하지 못하고 그 문자열을 그대로
`--params-file` 인자로 넘겨서 YAML 파싱에 실패, **gz-sim 프로세스 자체가 죽는** 문제가 있었습니다.
URDF는 xacro가 아닌 순수 XML이라 launch 시점 치환이 불가능하므로,
`jdamr_cube_gazebo/launch/gazebo.launch.py`에서 URDF 문자열을 읽은 뒤 그 부분만
`get_package_share_directory()`로 구한 실제 절대경로로 치환해서 로봇을 스폰하도록 고쳤습니다.

### 관절 값으로 제어하기 (`jdamr_cube_so101_arm`)

위 raw 액션 호출을 감싼 CLI 노드 `joint_control`이 있습니다. 원하는 관절만 골라서 값을 주면
그 값으로 이동하고, **지정하지 않은 관절은 현재 각도를 그대로 유지**합니다(다른 관절이 0으로
튀는 것을 방지하기 위해 이동 전에 `/joint_states`에서 현재 각도를 읽어와 병합합니다).

**Windows에서 배치 파일로**: 시뮬레이터(`run_gazebo_slam.bat` 또는 `run_navigation.bat`)가
떠 있는 상태에서,

```bat
arm_control.bat --shoulder-pan 0.5 --elbow-flex 0.4   :: 지정한 관절만 이동, 나머지는 유지
arm_control.bat --gripper 1.0                          :: 그리퍼 열기
arm_control.bat --gripper 0.0                           :: 그리퍼 닫기
```

수동으로 실행할 수도 있습니다.

```bash
source install/setup.bash
ros2 run jdamr_cube_so101_arm joint_control --shoulder-pan 0.5 --elbow-flex 0.4
ros2 run jdamr_cube_so101_arm joint_control --gripper 1.0
ros2 run jdamr_cube_so101_arm joint_control --shoulder-pan 0 --shoulder-lift 0 --elbow-flex 0 --wrist-flex 0 --wrist-roll 0  # 홈 자세
```

옵션: `--shoulder-pan`, `--shoulder-lift`, `--elbow-flex`, `--wrist-flex`, `--wrist-roll`, `--gripper`
(전부 rad 단위, 관절 범위는 위 표 참고), `--duration`(팔 이동 시간, 초 단위, 기본 3.0). 최소 하나는
지정해야 하며, 아무것도 지정하지 않으면 사용법 에러(종료 코드 1)를 출력합니다.

**카메라 화면**: `run_gazebo_slam.bat`/`run_navigation.bat`을 실행하면 손목 카메라(`wrist`),
상단 RGBD 컬러 카메라(`rgbd`), RGBD 뎁스 카메라(`depth`) 창 3개가 시작할 때 자동으로 함께 뜹니다.
뎁스 화면은 근/원 범위(기본 0.1~3.0m) 안에 있는 픽셀은 흰색, 그 범위를 벗어나거나(너무
가깝거나/멀거나) 반사가 없는 곳(NaN/무한대)은 검정으로 표시합니다. 수동으로 띄우거나 닫힌 창을
다시 열고 싶으면:

```bat
arm_control.bat view --camera wrist
arm_control.bat view --camera rgbd
arm_control.bat view --camera depth
arm_control.bat view --camera depth --near 0.2 --far 1.0   :: 표시 범위를 직접 지정해서 시작
```

뎁스 창의 near/far 범위는 창이 떠 있는 동안에도 실시간으로 조절할 수 있습니다(0.05m 단위):
- **버튼**: 뎁스 창을 열면 `Near -`/`Near +`/`Far -`/`Far +` 버튼이 있는 Qt 컨트롤 패널이
  함께 뜹니다(OpenCV가 Qt로 빌드된 경우; 이 저장소가 검증한 환경에서는 정상 동작).
- **키보드**: 이미지 창에 포커스를 준 상태에서 `[`/`]` = near 감소/증가, `,`/`.` = far 감소/증가.

조절할 때마다 터미널에 현재 값이 로그로 출력됩니다.

창에서 `q`를 누르거나 터미널에서 Ctrl+C로 닫습니다. 여러 창을 동시에 띄워도 ROS 2 노드 이름이
겹치지 않도록 카메라별로 고유한 이름(`jdamr_cube_camera_view_<토픽>`)을 사용합니다.

**테스트 방법** (실제로 이 순서로 검증):

1. `run_gazebo_slam.bat` 또는 `ros2 launch jdamr_cube_gazebo gazebo.launch.py`로 시뮬레이터 실행
2. `ros2 control list_controllers`로 `arm_controller`/`gripper_controller`/`joint_state_broadcaster`가
   모두 `active`인지 확인
3. `arm_control.bat --shoulder-pan 0.8 --wrist-roll -1.0` 실행 → 팔이 이동하고 "팔 이동 완료" 로그 출력
4. `arm_control.bat --elbow-flex 0.6` 실행(다른 관절 값은 주지 않음) → `/joint_states`로 확인했을 때
   `arm_shoulder_pan`이 여전히 0.8, `arm_wrist_roll`이 여전히 -1.0로 유지된 채 `arm_elbow_flex`만
   0.6으로 바뀌는지 확인 (터미널 로그의 "팔 목표 전송:" 줄에 그대로 표시됩니다)
5. `arm_control.bat --gripper 1.5` 실행 → "그리퍼 이동 완료" 로그와 함께 그리퍼가 열림

### 뎁스카메라 픽셀 위치로 집기 (MoveIt2, `pick --at-pixel`)

상단 RGBD 뎁스카메라 화면에서 **집고 싶은 물체의 픽셀 좌표 (u, v)** 를 주면, 그 픽셀의 뎁스 값으로
3D 위치를 복원한 뒤 [MoveIt2](https://moveit.ai/)(OMPL 모션플래닝)로 팔을 그 지점까지 움직여 집습니다.
앞의 `pick`(손목 카메라 색상 검출 + 튜닝된 고정 자세)과 달리, 이 방식은 **임의의 위치**에 대해
매번 IK/모션플래닝을 다시 풀기 때문에 물체가 팔 작업공간(reach) 안에 있기만 하면 됩니다.

동작 순서: (1) 픽셀 → 뎁스 → `rgbd_camera_link` 기준 3D → TF로 `base_footprint` 기준 3D 변환,
(2) 그리퍼 열기, (3) 목표 지점 8cm 위(pre-grasp)로 이동, (4) 목표 지점까지 하강, (5) 그리퍼 닫기,
(6) 다시 8cm 들어올리기. 각 이동은 `move_group`을 따로 띄울 필요 없이 CLI 안에서 `MoveItPy`가
직접 플래닝/실행합니다.

먼저 뎁스/컬러 창으로 집을 픽셀 좌표를 확인한 뒤 그 값을 넘깁니다.

```bat
:: 1) rgbd(또는 depth) 창을 띄워 집을 물체의 픽셀 좌표(u, v)를 눈으로 확인
arm_control.bat view --camera rgbd

:: 2) 그 픽셀 위치를 집기 (예: u=402, v=449)
arm_control.bat pick --at-pixel 402 449
```

수동 실행:

```bash
source install/setup.bash
ros2 run jdamr_cube_so101_arm joint_control pick --at-pixel 402 449
```

- 픽셀 → 3D 복원식은 gz-sim 카메라 센서가 (ROS 광학 프레임이 아니라) 링크 로컬 +X축을 바라보는
  규약을 반영해, room.world의 물체를 카메라 광선 위에 정확히 놓고 실측으로 검증한 값입니다
  (화면 중앙 물체가 계산상 예상 깊이와 mm 단위로 일치). `arm_gripper_frame_link`(그리퍼 TCP)를
  목표 위치에 맞추며, SO-101은 5-DOF라 자세(orientation)까지는 맞추지 못하므로 위치만 맞추는
  IK(`position_only_ik`)를 씁니다.
- 물체가 팔 리치를 벗어나 IK 해가 없으면 "MoveIt 플래닝 실패" 로그를 남기고 종료 코드 1로 끝납니다.
- `moveit_py`는 이 명령에서만 지연 임포트하므로, MoveIt2가 설치되지 않은 환경에서도 나머지 명령
  (`move`/`view`/`pick`/`place`)은 정상 동작합니다. MoveIt2가 필요하면:
  `sudo apt-get install ros-jazzy-moveit-py ros-jazzy-moveit-configs-utils ros-jazzy-moveit-simple-controller-manager`

**테스트 방법** (실제로 이 순서로 Gazebo에서 검증):

1. `run_gazebo_slam.bat` 등으로 시뮬레이터 실행(room.world). `pick_object`(핑크 큐브)가 팔 작업공간
   안(예: 로봇 상판 위)에 놓여 있어야 합니다.
2. `arm_control.bat view --camera rgbd`로 큐브의 화면 픽셀 좌표를 확인
3. `arm_control.bat pick --at-pixel <u> <v>` 실행 → 로그에 "픽셀 (u, v) -> base_footprint 기준
   x=…, y=…, z=…"로 복원된 3D 좌표가 찍히고, pre-grasp → 하강 → 그리퍼 닫기 → 들어올리기가
   차례로 실행된 뒤 "pick --at-pixel 동작을 완료했습니다."가 출력됩니다.
4. Gazebo에서 큐브가 그리퍼에 물려 함께 들려 올라가는지 확인(월드 상태에서 큐브의 z가 올라감).

**구현하면서 발견/해결한 문제**:

- **`move_group`이 "Planning plugin name is empty"로 크래시**: 이 MoveIt 버전(2.12.4)은 예전
  예제의 평평한 `move_group.planning_plugin` 스키마가 아니라 `MoveItConfigsBuilder`가 만드는
  버전에 맞는 파라미터를 요구함. launch를 `MoveItConfigsBuilder` 기반으로 재작성해 해결.
- **`MoveItPy`가 "Failed to load planning pipelines"**: `move_group`은 최상위
  `planning_pipelines`(문자열 리스트)를, `MoveItCpp`(moveit_py)는 `planning_pipelines.pipeline_names`를
  읽는 등 파라미터 이름이 달라서, config_dict에 `pipeline_names`와 `plan_request_params.*`를 직접 보강.
- **`use_sim_time`을 주면 `/clock` 구독의 `qos_overrides` 파라미터 선언 중 크래시**: qos_overrides
  기본값 4개를 미리 넣어 회피.
- **PoseStamped 목표로는 플래닝이 항상 실패**: 5-DOF + `position_only_ik`에서는 목표에 방향까지
  포함되면 IK 해가 전부 기각됨. 위치만 담은 `construct_link_constraint`로 목표를 준다.
- **집은 뒤 들어올릴 때 자기충돌로 플래닝 거부**: 그리퍼를 닫으면 `arm_moving_jaw_link`가
  `arm_wrist_camera_link`와 접촉 판정됨. SRDF `disable_collisions`에 해당 쌍들을 추가.

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

### 3. 저장한 맵으로 좌표까지 자율주행하기 (`jdamr_cube_navigation`)

저장된 맵(.yaml)을 로드해서 Nav2(AMCL 로컬라이제이션 + 경로계획/추종)를 띄우고, 좌표를 주면
그 지점까지 이동시키는 [`jdamr_cube_navigation`](jdamr_cube_navigation) 패키지가 있습니다.

**Windows에서 배치 파일로**: 먼저 `run_gazebo_slam.bat` + `save_map.bat`으로
`~/maps/jdamr_cube_room.yaml`을 만들어둔 상태에서,

```bat
run_navigation.bat            :: Gazebo(room.world) + Nav2 창 2개를 띄움
goto_pose.bat 1.0 -1.8         :: (1.0, -1.8)로 이동
goto_pose.bat 0.0 0.0 1.57     :: 원점으로, yaw=1.57rad 방향으로 이동
```

`run_navigation.bat`은 `run_gazebo_slam.bat`과 동일하게 시작 전에 남아있는 프로세스를 정리하고
워크스페이스를 빌드한 뒤 Gazebo/Nav2를 각각 새 창으로 띄웁니다. `goto_pose.bat`은 그 상태에서
목표 좌표만 보내는 1회성 명령이라 필요할 때마다 반복 실행하면 됩니다. 내부적으로
[`scripts/wsl_run_navigation.sh`](scripts/wsl_run_navigation.sh),
[`scripts/wsl_goto_pose.sh`](scripts/wsl_goto_pose.sh)를 사용합니다.

수동으로 터미널을 띄워 실행할 수도 있습니다.

```bash
# 1) Gazebo (SLAM 없이, room.world)
source install/setup.bash
ros2 launch jdamr_cube_gazebo gazebo.launch.py \
  world:=$(ros2 pkg prefix jdamr_cube_gazebo)/share/jdamr_cube_gazebo/worlds/room.world

# 2) Nav2 (기본값으로 ~/maps/jdamr_cube_room.yaml을 로드함)
source install/setup.bash
ros2 launch jdamr_cube_navigation navigation.launch.py
# 다른 맵을 쓰려면: ros2 launch jdamr_cube_navigation navigation.launch.py map:=/path/to/map.yaml

# 3) 목표 좌표로 이동 (map 프레임 기준 x, y[, --yaw])
source install/setup.bash
ros2 run jdamr_cube_navigation goto_pose 1.0 -1.8
ros2 run jdamr_cube_navigation goto_pose 0.0 0.0 --yaw 1.57
```

`goto_pose`는 `navigate_to_pose` 액션으로 목표를 보내고 도착/실패까지 대기하는 1회성 CLI 노드로,
성공하면 종료 코드 0, 실패(장애물에 너무 붙은 목표, 경로 없음 등)하면 1을 반환합니다.

로봇은 room.world에 스폰될 때 항상 `(0, 0, 0)`이고 맵 원점도 그 지점이므로, AMCL이
`jdamr_cube_navigation/config/nav2_params.yaml`의 `set_initial_pose`로 시작 시 자동으로
초기 위치를 잡습니다(RViz에서 수동으로 "2D Pose Estimate"를 클릭할 필요 없음). 시각적으로
확인하고 싶다면 `rviz2`를 별도로 띄우고 Fixed Frame을 `map`으로, Map/LaserScan/RobotModel
디스플레이를 추가해서 보면 됩니다.

`jdamr_cube_navigation/config/nav2_params.yaml`은 Jazzy 기본 `nav2_bringup` 파라미터를 베이스로
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
