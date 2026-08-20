# JDAMR Cube 실기 브링업 문서 — 조사 브리프 (Planner/Architect/Critic 공용 입력)

## 0. 사용자 요청 (원문 취지)
JDAMR Cube 로봇(라즈베리파이 + Waveshare ESP32 제어보드 + LD14 라이다)을 만들었고,
SLAM 및 자율주행을 하려 한다. 순차적으로 진행할 수 있는 **HTML 문서 한 편**을 원한다.
진행 순서: 라즈베리파이 초기화 → 펌웨어에서 모터·OLED·센서 확인 → ROS 펌웨어 →
파이썬 구동 테스트 → ROS 텔레옵 → SLAM → 자율주행(맵 생성 + 주행).
**강사 리포 코드에 오류가 많아 보이니 개선 가능한 것은 개선하면서 고도화**할 것.

## 1. 사용자 확정 답변 (2026-08-12)
| 항목 | 확정값 |
|---|---|
| SBC / OS / ROS | **Raspberry Pi 4 + Ubuntu 24.04 + ROS 2 Jazzy** (사용자가 "2번으로 하고 싶은데 잘 안 되나?"라고 물었고, 조사 결과 가능하다고 확인해 줌) |
| Pi ↔ ESP32 연결 | **USB 케이블** (`/dev/ttyUSB0` 또는 `/dev/ttyACM0`) |
| 라이다 | **LDROBOT LD14 / LD14P** |
| 문서 성격 | **본인 브링업 작업 문서** (수강생 교재 아님 — 체크박스·판정 기준·트러블슈팅 중심, 개선 코드 전문 수록) |
| 현재 상태 | 라즈베리파이 **아직 초기화 안 됨** → 이미지 굽기부터 문서에 포함 |

## 2. 확인된 사실 (추측 아님, 소스/인덱스 확인)

### 2-1. 라이다 프레임 = 47바이트 (사용자가 들은 "7바이트"는 오정보)
`ldlidar_sl_ros2/ldlidar_driver/include/lipkg.h`의 `LiDARFrameTypeDef` (packed) 기준:
```
header   uint8   = 0x54      (1)
ver_len  uint8   = 0x2C      (1)
speed    uint16            (2)
start_angle uint16         (2)
point[12] {uint16 distance; uint8 intensity;}  (3 × 12 = 36)
end_angle   uint16         (2)
timestamp   uint16         (2)
crc8        uint8          (1)
------------------------------------------------ 합계 47 바이트
```
`POINT_PER_PACK = 12`, `PKG_HEADER = 0x54`, `PKG_VER_LEN = 0x2C`. 보레이트 115200.
→ "7"에 근접한 유일한 숫자는 **측정점 1개 = 3바이트**. 문서에 이 정정을 명시할 것.

### 2-2. Jazzy 가용성
- `cartographer_ros`: **Jazzy 바이너리 릴리스 존재** (2.0.9003, 2025-05-24). `sudo apt install ros-jazzy-cartographer-ros` 가능.
- `ldlidar_sl_ros2`: 바이너리 없음. **소스 빌드** 필요(Foxy 이상 지원, 순수 C++/colcon이라 Jazzy 빌드 가능). 강사 리포 사본보다 상류(ldrobotSensorTeam) 최신 clone 권장.
- `slam_toolbox`, `nav2`: Jazzy arm64 바이너리 존재.
- Ubuntu 24.04 arm64는 Jazzy Tier 1. Pi 4 공식 이미지 있음.

### 2-2b. ⚠️ Pi 4 + Ubuntu 24.04 + Jazzy의 실제 함정 (문서 필수 항목)
`ros2/ros2` **이슈 #1789 — libzstd 의존성 파손 (2026-01-19 등록, 확인 시점 기준 open)**
- 증상: Ubuntu Server **24.04.3** LTS arm64에서 `sudo apt install ros-jazzy-*` 실패
- 에러: `libzstd-dev : Depends: libzstd1 (= 1.5.5+dfsg2-2build1) but 1.5.5+dfsg2-2build1.1 is to be installed`
- 연쇄: `ros-jazzy-rcl`이 `ros-jazzy-tracetools` 의존을 못 풀어 설치 중단
- 원인: noble-updates가 libzstd1을 `...2build1.1`로 올렸는데 ROS 2 바이너리가 `...2build1`에 고정 핀됨
- 우회: `libzstd1`을 구버전으로 다운그레이드 후 `apt-mark hold` (보안 업데이트와 상충하므로 문서에 트레이드오프 명시)
- **문서 처리 방침**: "설치가 안 되면 여기부터 의심하라"는 형태로 Pi 초기화 절에 **사전 경고 + 우회 절차**를 넣을 것. 사용자가 애초에 "2번으로 하고 싶은데 잘 안 되나?"라고 물은 지점이 정확히 이것일 가능성이 높다.
- 대안 경로도 병기: ros-realtime/ros-realtime-rpi4-image (Jazzy + 24.04 사전 설치 이미지, Pi 3/4/5 지원)

### 2-2c. 하드웨어 제원 (확인 완료)
**LD14 라이다**: 측정 0.15~8 m, 스캔 6 Hz, 측距 2300 Hz(LD14P는 4000 Hz), UART 115200
→ 강사 lua의 `min_range=0.12`는 **센서 최소거리 미만**, `max_range=10.0`은 **최대거리 초과**. 둘 다 실제 결함 확정. 권장 `min_range=0.16`, `max_range=8.0`.

**STS3215 서보**: 12비트 자기 엔코더 **4096 step/회전** (0.088°/step), wheel mode 속도 단위 = **step/s**, 최대 약 3400 step/s(≈0.83 rev/s ≈ 50 rpm)
→ 오도메트리 환산식: `거리 = (Δstep / 4096) × π × 바퀴지름`. Δstep은 0~4095 랩어라운드 언랩 필수(결함 A8).

**Waveshare General Driver for Robots (ESP32)**: ESP32-WROOM-32, 9축 IMU = **QMI8658C + AK09918**, **INA219** 전압/전류 감시, ST3215 시리얼 버스 서보 최대 253개, 입력 **7~13 V (2S/3S 리튬)**, 라이다 시리얼→USB 회로 내장, IIC/SD/PWM 서보 인터페이스
→ 결함 A10(INA219 주소 0x42 하드코딩) 검증 시 이 보드의 실제 주소를 `i2cdetect`로 확인하는 절차를 문서에 넣을 것.
→ **보드에 라이다 시리얼→USB 회로가 있다**는 점 주목: LD14를 보드 경유로 붙일지 Pi USB에 직결할지 문서에서 갈라야 함.

### 2-3. 리포 구조
- `JD-edu/jdamr_cube` — ESP32 펌웨어 전용. `basic_firmware/`(Serial_servo 101~105, Imu_9_axis, Oled_display, Volt_monitor, Protocol, speed_control) + `ros_firmware/ros_firmware_no_rtos/`.
- `JD-edu/jdamr200_ros` — ROS 2 측. description / node / teleop / bringup / cartographer / gazebo / ldlidar_sl_ros2. **navigation(Nav2) 패키지 없음.**
- 로컬 `~/jdamr_cube_ws/src/jdamr_cube_ros` (사용자 소유, origin=mmporong/jdamr_cube_ros) — **시뮬(Gazebo) 캡스톤용**. bringup/cartographer/description/gazebo/moveit_config/navigation/node/so101_arm/teleop + capstone_pick. 실기 브링업과 별개 트랙이나 URDF·Nav2 파라미터 재사용 후보.

## 3. 확인된 결함 목록 (문서에서 개선 대상)

### A. ESP32 펌웨어 — `ros_firmware_no_rtos.ino`
| # | 결함 | 영향 |
|---|---|---|
| A1 | 자이로 Y/Z, 마그네틱 X/Y/Z(패킷 10~19바이트)를 **아예 채우지 않음**. 주석에 "(나머지 축 및 마그네틱은 유사 방식으로 채움)"만 있음 | IMU 대부분이 0 고정 → EKF/Cartographer IMU 사용 불가 |
| A2 | 가속도는 `×1000`, 자이로는 `×100`으로 **스케일 불일치**. 수신 측(`107_protocol.py`)은 둘 다 `/1000` 가정 | 자이로 10배 오차 |
| A3 | 체크섬 바이트(28)를 **항상 0x00**으로 채움 | 무결성 검증 불가, 깨진 프레임 탐지 불가 |
| A4 | 명령 수신이 `if(Serial.available()>=5) readBytes(cmd_buf,5)` — **헤더 재동기화 없음** | 1바이트만 유실되면 이후 모든 5바이트 프레임이 영구히 밀림 |
| A5 | 방향 분기에 **BACKWARD / TURN_LEFT / TURN_RIGHT가 없음** (FORWARD/STOP/RESET만 구현) | 후진·회전 불가 → 텔레옵/SLAM 자체가 불가능 |
| A6 | 100 ms마다 `st.getCurrentPosition()`을 **4회** 호출(send_sensors 2 + debug_line_3 2) | 서보 버스 왕복 지연 누적 |
| A7 | 100 ms마다 `display.display()` (SSD1306 128×32 전체 프레임버퍼 I2C 전송) | 루프 수십 ms 블로킹 → 시리얼 RX 버퍼 오버런 |
| A8 | STS3215 위치는 **0~4095 랩어라운드**인데 언랩 처리 없음. `int16_t`로 캐스팅 | 오도메트리 누적 불가, 부호 폭주 |
| A9 | **cmd 워치독 없음** — Pi가 죽거나 USB가 빠져도 모터가 마지막 속도로 계속 회전 | 안전 결함 (최우선) |
| A10 | `INA219_ADDRESS 0x42` 하드코딩 (일반 기본값은 0x40/0x41) | 보드 리비전 따라 전압 측정 실패 |
| A11 | 좌우 부호 규약이 FORWARD 한 곳(`LEFT=+, RIGHT=-`)에만 존재하고 문서화 안 됨 | 미러 장착 부호 실수 재발 |

### B. ESP32 예제 간 비일관 — `speed_control.ino` vs `ros_firmware`
| # | 결함 | 영향 |
|---|---|---|
| B1 | `speed_control.ino`: `st.init(&Serial1, 1000000)` (인자 2개) / `ros_firmware`: `st.init(255, &ServoSerial, 1000000)` (인자 3개) | **서로 다른 STSServoDriver 버전을 가정 → 한쪽은 컴파일 실패** |
| B2 | `speed_control.ino`가 `Serial1` 사용, `ros_firmware`가 `HardwareSerial(2)` 사용 (핀은 둘 다 18/19) | 예제 간 포트 불일치 |
| B3 | `speed_control.ino` 주석 "UART2 사용 예시: RX=16, TX=17" ↔ 실제 코드 18,19 | 주석/코드 불일치 |
| B4 | `105_oled_display.ino`의 `loop(){ Screenupdate(); delay(1); }` — 1 ms마다 전체 프레임 I2C 전송 | I2C 버스 포화 (데모지만 잘못된 본보기) |

### C. ROS 2 노드 — `jdamr200_node.py` / 로컬 `jdamr_cube_node.py`
| # | 결함 | 영향 |
|---|---|---|
| C1 | **`cmd_vel`을 이산 5방향(전/후/좌/우/정지)으로 축약**. `linear.x`와 `angular.z`를 동시에 못 보냄 | **자율주행 불가.** Nav2는 v와 ω를 동시에 낸다. 아크 주행이 원천 차단 |
| C2 | 회전 속도를 상수 150으로 고정 | 각속도 지령 무시 |
| C3 | `jdamr200_node.py`의 `/odom`: orientation 미설정(쿼터니언 전부 0 → **무효**), `child_frame_id` 없음, `header.stamp` 없음, twist 없음, **odom→base TF 미발행** | Cartographer/Nav2가 odom을 못 씀 |
| C4 | `from jdamr200_lib import Jdamr200` — **해당 모듈이 리포에 없음** | import 실패 |
| C5 | Voltage를 `Int8`에 100 하드코딩 (더미) | 배터리 감시 무의미. `sensor_msgs/BatteryState`가 표준 |
| C6 | 로컬 `jdamr_cube_node.py`: IMU covariance 미설정(0 = "정확히 앎"으로 해석). `orientation_covariance[0]=-1` 규약 미적용 | 융합 필터가 잘못된 신뢰도 사용 |
| C7 | `cmd_vel` 콜백마다 즉시 시리얼 write. rate limit 없음 | Nav2 20 Hz 지령 시 시리얼 폭주 |
| C8 | 포트 `/dev/ttyAMA0` 하드코딩, 파라미터화 안 됨 (USB 연결이면 틀림) | 사용자 환경(USB)과 불일치 |
| C9 | `main_loop()`에서 `while rclpy.ok(): rclpy.spin_once(self)` — 불필요한 재구현 | busy-loop, CPU 낭비 |

### D. 텔레옵 — `jdamr_teleop.py`
| # | 결함 | 영향 |
|---|---|---|
| D1 | **속도를 `twist.linear.z`에 실어 보냄** — Twist 의미 파괴 | `teleop_twist_keyboard`, Nav2, RViz 등 표준 도구와 전부 비호환 |
| D2 | `'a'`(좌회전)가 `linear.y = 0.5`를 설정 | 차동구동은 y 이동 불가. 무의미 |
| D3 | `__init__` 안에서 `self.run()` 호출 후 `main()`에서 또 `rclpy.spin()` | run()이 끝나야 spin 진입. 구조 오류 |
| D4 | 키 없어도(`key == ''`) 매 루프 publish | 불필요한 트래픽 |

### E. Cartographer — `cartographer.launch.py` / `jdamr200_lidar.lua`
| # | 결함 | 영향 |
|---|---|---|
| E1 | **static `map→odom` + static `odom→body_link` 를 동시에 발행**하는데 Cartographer도 (`published_frame=body_link`, `provide_odom_frame=false`) `map→body_link`를 발행 | **TF 트리 파손** — `body_link`의 부모가 둘. 근본 결함 |
| E2 | static `odom→body_link`가 0 고정 | 로봇이 TF상 영원히 원점. odom이 존재 의미 없음 |
| E3 | `use_odometry=false` **와** `use_online_correlative_scan_matching=false` 동시 설정 | 초기 추정값이 전무 → 회전 시 스캔매칭 발산. 맵이 겹쳐 찍힘 |
| E4 | `motion_filter.max_angle_radians = math.rad(0.1)` (기본 1.0°의 1/10) | 노드 폭증 → Pi 4 CPU 과부하 |
| E5 | `rviz2`를 **config 없이** 실행. 패키지에 `cartographer.rviz`가 있는데도 미사용 | 매번 수동 설정 |
| E6 | `DeclareLaunchArgument('cartographer_config_dir', ...)` 선언해 놓고 실제로는 미선언 변수 `jdamr200_config_dir`을 사용 | 런치 인자가 먹지 않음 (죽은 인자) |
| E7 | `ld14.launch.py`가 이미 `robot_state_publisher` + static `body_link→base_laser`를 띄우는데 `cartographer.launch.py`가 그것을 include | 노드/TF 중복 발행 |
| E8 | 주석에 "ydlidar launch file" (실제는 LD14) | 복붙 흔적 |
| E9 | `TRAJECTORY_BUILDER_2D.min_range = 0.12` — LD14 실제 최소거리와 대조 필요 | 근거리 노이즈 유입 |
| E10 | `use_imu_data = false`인데 `tracking_frame = "body_link"` | IMU를 살릴 경우 tracking_frame을 imu_link로 바꿔야 함 (문서에 분기 필요) |

### G. URDF — **가장 치명적. 사용자가 직접 의심을 제기해 확인된 항목**
확인 대상: 강사 `jdamr200_description/urdf/jdamr200.urdf`, 로컬 `~/jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_description/urdf/jdamr_cube.urdf`
**로컬 cube URDF는 강사 jdamr200 URDF의 복사본이다** (치수 동일, 바퀴 x오프셋과 라이다 위치만 미세 수정, SO-101 팔 추가).

| # | 결함 | 영향 |
|---|---|---|
| **G1** | **프레임 이름 공간이 서로 다른 세 세계로 갈라져 있음.** URDF = `base_footprint`/`base_link`/`laser_link` / `carto.lua` = `body_link` / `ld14.launch.py` = `body_link`→`base_laser`, 스캔 `frame_id=base_laser`. **URDF에 `body_link`도 `base_laser`도 없다** | `/scan`(base_laser)과 로봇 트리(laser_link)가 이어지지 않아 **TF 조회 실패 → SLAM 원천 불가**. E1보다 상위 결함 |
| **G2** | **기하 자체 모순 — 바퀴가 땅에 파묻힘.** base_joint z=+0.075, wheel z=-0.008 → 바퀴 중심 지면고 0.067 m, 바퀴 반지름 0.075 m → 바퀴 바닥이 **-0.008 m (지면 아래 8 mm)** | `base_footprint`가 지면 평면에 없음 → Nav2 코스트맵/풋프린트 전제 붕괴, 오도메트리 원점 어긋남 |
| **G3** | **캐스터가 공중에 뜸.** 캐스터 중심 0.070 m, 반지름 0.03 → 바닥이 **+0.040 m** | 접지점이 셋 중 둘뿐. Gazebo에선 기울고 실기에선 URDF가 실물과 불일치 |
| **G4** | 치수가 JDAMR Cube 실물과 무관. 몸통 **0.5×0.3×0.15 m**, 바퀴 반지름 **0.075 m**, 트레드 **0.35 m** | Cube는 소형 큐브형. 오도메트리 환산 계수(`wheel_separation`, `wheel_radius`)가 전부 틀림 → 주행거리·회전각 오차 |
| **G5** | 라이다 `xyz="0.3 0 0.15"` — 몸통 반길이 0.25 m → **몸통 밖 5 cm 공중**. 로컬 cube는 `0.25 0 0.1`로 몸통 앞 모서리 | 스캔 원점이 실제 라이다 위치와 불일치 → 맵이 일정하게 오프셋된 채 생성 |
| **G6** | 로컬 cube URDF는 바퀴를 `x=+0.15`로 이동 → 바퀴축이 중심 앞 15 cm인데 앞 캐스터는 `x=+0.2` 그대로 | 바퀴와 앞 캐스터가 5 cm 간격으로 겹침. 회전 중심이 바퀴축이 되어 Nav2 회전 반경과 어긋남 |
| **G7** | Gazebo Classic 플러그인 잔존: `libgazebo_ros_diff_drive.so`, **`libgazebo_ros_imu.so`** (Foxy 이후 제거됨. 올바른 이름은 `libgazebo_ros_imu_sensor.so`) | Jazzy/Harmonic에서 미동작. 실기 브링업엔 무해하나 **원본이 검증된 적 없다는 방증** |
| **G8** | `<gazebo reference="caster_link_*">` 태그가 `<link>` **안에** 위치 | URDF 스펙 위반 |
| **G9** | 관성값 전부 더미 등방성 (`ixx=iyy=izz=0.010`) | 실기 무해, 시뮬 전환 시 문제 |

**→ 문서 처리 방침: URDF를 강사 원본에서 고쳐 쓰는 게 아니라 실측 기반으로 새로 쓴다.**
필수 실측 항목: 몸통 W×D×H / 바퀴 지름 / 트레드(좌우 바퀴 접지점 간격) / 캐스터 반지름·위치 / 라이다 장착 x·y·z 오프셋 및 정면 기준 회전(yaw).
프레임 이름은 **ROS 관례로 통일**: `base_footprint` → `base_link` → {`left_wheel_link`, `right_wheel_link`, `caster_link`, `imu_link`, `laser_link`}.
`body_link`·`base_laser`를 쓰는 lua/launch는 전부 이 이름에 맞춰 고친다. 라이다 노드의 `frame_id`도 `laser_link`로.

### F. 구조적 공백
| # | 공백 |
|---|---|
| F1 | **Nav2 설정 자체가 강사 리포에 없음** (jdamr200_ros에 navigation 패키지 부재) |
| F2 | 로봇 물리 제원(바퀴 지름, 트레드/휠베이스, STS3215 counts/rev)이 어디에도 없음 → 오도메트리 계산 불가. **실측 기입란 필요** |
| F3 | (G1로 승격·정정) URDF는 `base_link`를 쓰고 lua/launch가 `body_link`를 쓴다. 어긋난 쪽은 lua/launch. G 표 참조 |
| F5 | 실측 없이는 URDF·오도메트리·Nav2 풋프린트가 전부 성립 불가 → **문서 맨 앞에 실측 절을 두고, 실측값이 뒤 단계로 전파되는 경로를 명시**해야 함 (바퀴지름·트레드 → 오도메트리 환산 + URDF + Nav2 footprint/inflation) |
| F4 | 강의실 LAN 환경에서 `ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY` 미지정 시 DDS 오염 위험 (사용자 기존 메모리에 기록된 실제 사고 이력 있음) |

## 4. 핵심 설계 판단 (계획에서 반드시 다뤄야 할 것)

**가장 큰 결정: 시리얼 프로토콜을 이산 방향(0x51)에서 연속 차동구동(0x52)으로 확장할 것인가.**
- C1이 자율주행의 원천 차단 요인이다. 펌웨어 주석에도 `// ToDo: 여기에 0x52 (Diff Drive) ... 추가` 라고 남아 있다.
- 0x51을 유지하면 텔레옵까지는 되지만 SLAM 품질이 낮고 Nav2는 사실상 불가.
- 따라서 문서는 **0x51(기존, 검증용) → 0x52(신규, v/ω 또는 좌우 휠 속도) 이행**을 단계로 넣어야 한다.
- 0x52 페이로드·체크섬·워치독 설계가 문서의 기술적 심장부.

**두 번째 결정: SLAM 스택.** Cartographer(강사 기준, Jazzy 바이너리 있음) vs slam_toolbox(Nav2 표준 조합, 튜닝 쉬움, Pi에서 가벼움).
사용자 메모리 규칙: "비표준이면 먼저 권할 것 — 현업 정합성·포트폴리오·구현 명확성 세 축으로".

**세 번째: 판정 기준.** 사용자 메모리 규칙 "지표는 귀무모형부터", "판정은 실좌표로(로그의 SUCCESS를 믿지 말 것)".
각 단계의 통과 판정을 로그 문구가 아니라 **관측 가능한 물리량/토픽 값**으로 정의해야 한다.

## 5. 남은 미확정 (문서에 실측 기입란으로 처리)
- Pi 4 RAM (4GB / 8GB) — RViz를 Pi에서 띄울지 노트북에서 띄울지가 갈림
- 바퀴 지름, 좌우 바퀴 간격(트레드), STS3215 wheel mode의 속도 단위 ↔ 실제 m/s 환산 계수
- 배터리 사양(셀 수, 정격 전압) — 저전압 컷오프 임계값
- LD14 장착 높이·정면 오프셋·회전 방향(`laser_scan_dir`)

## 6. 산출물 제약
- **HTML 파일 1개**, 밝은 테마 고정(사용자 메모리 규칙: 다크 대응하지 말 것, OS/토글이 어두워도 밝은 화면 유지)
- 한글 서술
- 경로는 `~` 기준 전체 경로로 붙여넣기 가능한 형태
- 개선 코드 전문 수록(펌웨어 .ino, ROS 노드 .py, launch, lua, Nav2 yaml)
- 단계마다: 목적 / 실행 명령 / **통과 판정 기준** / 실패 시 트러블슈팅 / 강사 원본 대비 변경점
