# 라이다·등록 뎁스 장애물 통합 — 2026-09-18

## 구현 범위

기존 라이다 주행 YAML은 변경하지 않았다. `depth_obstacle_navigation.launch.py`에서 선택할 때만 뎁스 관측과 Nav2 overlay를 사용한다. 기본 `observe` 모드는 관측 노드만 실행하며 드라이버·Nav2·이동 목표를 시작하지 않는다.

1. Astra S의 `/camera/depth/image_raw`와 `/camera/depth/camera_info`를 읽는다.
2. 영상 촬영 시각의 TF로 `camera_color_optical_frame` → `base_footprint` 변환을 적용한다.
3. 바닥·차체·범위 밖 점을 제외한 장애물 점군과, 바닥을 포함한 광선 끝점 점군을 나눠 발행한다.
4. 기존 LiDAR obstacle layer는 유지하고 별도의 depth VoxelLayer를 inflation 앞에 추가한다. 높이를 구분하는 3D clearing 후 2D costmap에 최대 비용으로 결합한다. 라이다 높이의 빈 공간 때문에 뎁스가 본 상판이 지워지는 문제를 줄이기 위한 구조다.
5. 선택한 통합 모드에서는 Collision Monitor가 scan과 depth pointcloud를 함께 소비한다. LiDAR 정지 영역·속도·source timeout은 기존 값을 유지한다.

출력은 `/depth_navigation/obstacles`, `/depth_navigation/rays`, `/depth_navigation/status`다. 상태에는 입력 유효 비율, 점 개수, 처리 시간, 영상 나이와 중단 이유를 포함한다. 이 노드는 속도 명령을 발행하지 않는다.

## 기록에서 확인한 입력 조건

- 9월 17일 bag의 등록 뎁스는 320×240 `16UC1`이며 RGB optical frame을 사용한다. depth optical frame으로 가정하면 잘못된 변환을 적용할 수 있다.
- 해당 CameraInfo의 `K`에는 NaN이 있고 `P`는 유효하다. `rectified_projection` 모드에서 `P`를 명시적으로 사용한다. 왜곡 계수 0, rectification 행렬 단위행렬, 프레임·영상 크기 일치를 확인하며 임의 리사이즈나 NaN 보간은 하지 않는다.
- 차체 자기점 제거 범위는 `jdamr_cube_description/config/new_base_geometry.yaml`의 실측값에서 읽는다. 카메라 장착 정보는 `jdamr_cube_vslam/config/camera_mount.yaml`에 있다.

## 오정상 판정 방지

- 입력이 끊기거나 오래됨, 중복 시각, TF 누락, 잘못된 보정값, 전체 무효 뎁스, 관측 ROI가 비어 있음, 점 개수 초과를 정상 빈 공간으로 발행하지 않는다.
- 바닥만 관측한 정상 프레임은 빈 장애물 점군과 유효한 광선 점군을 발행한다. 센서 드롭아웃과 구분한다.
- 점군 시각은 원본 영상 시각을 유지한다. 처리 시각으로 갱신해 오래된 관측을 새 관측처럼 만들지 않는다.
- 입력 queue depth 1, 기본 처리 상한 5 Hz, 4픽셀 간격 샘플링, 4 cm voxel 대표점으로 부하를 제한한다. TF 수신은 2-thread executor의 별도 callback group에서 처리한다.

## 확인된 검증

- 관련 회귀 테스트 466개 통과: 기존 주행·주차·마스킹·차체 제원 및 새 depth core/filter/launch/replay.
- 설치된 Nav2 Collision Monitor의 격리 시험 14개 통과: 기존 scan 동작, depth-only 고도 장애물 정지, depth 입력 끊김, 복구 후 재개. 물리 주행 시험이 아니며, 근거리 합성 점 입력은 카메라가 그 거리까지 측정한다는 뜻이 아니다.
- 설치된 Nav2 1.3.12 VoxelLayer 기능 시험 4개 통과: 높은 장애물 등록 비용 254, 낮은 라이다 광선 후 254 유지, 바닥 뎁스 광선 후 254 유지, 같은 높이를 통과하는 뎁스 광선 후 0으로 제거. 합성 TF 시험이며 실물 광학 좌표 정렬 검증은 아니다.
- 저장 데이터 약 105초 구간에서 선정한 100프레임 모두 변환 성공. 장애물 점 평균 641.53개, 최대 853개.
- 재검증 처리 시간 평균 2.000 ms, p95 2.259 ms, 최대 2.615 ms. 현재 PC의 오프라인 투영·필터 시간이며 Pi 성능, DDS 지연, 전체 제동 지연은 포함하지 않는다.
- 관측 launch 실제 기동 및 SIGINT 정상 종료, navigation 패키지 빌드, 변경 Python 파일 ament_flake8 통과.

로컬 원본 산출물:

```text
$HOME/jdamr_artifacts/depth_obstacles_20260918/replay_summary_reviewed.json
$HOME/jdamr_artifacts/depth_obstacles_20260918/monitor_smoke/summary.json
$HOME/jdamr_artifacts/depth_obstacles_20260918/voxel_smoke_reviewed4/summary.json
```

최초 VoxelLayer 시험의 standalone costmap 프로세스는 네 기능을 확인하고 lifecycle deactivate·cleanup을 마친 뒤 SIGINT 종료에서 `-11`을 반환했다. 당시 원인 미규명 상태와 실행 로그는 보존한다. 아래 9월 21일 후속 검증에서 원인과 시험 전용 대응을 확인했다. 전체 Nav2 통합 실행의 정상 종료까지 확인됐다고 주장하지 않는다. 앞선 harness 설정·서비스 이름·셀 경계 문제로 실패한 실행 로그도 별도 디렉터리에 보존했다.

## 9월 21일: 파이 없는 종료 오류 수정

### 원인과 대응 범위

설치된 Nav2 `1.3.12-1noble.20260615.154707`의 `Costmap2DROS`는 `plugin_loader_`를 `callback_group_`보다 먼저 파괴한다. 이때 `liblayers.so`가 먼저 내려가지만 callback group에는 구독 weak pointer의 control block이 남는다. 이후 그 control block의 소멸 코드를 호출하면서 SIGSEGV가 발생했다.

GDB에서 `rclcpp::CallbackGroup::~CallbackGroup()` 충돌, vtable 주소의 unmapped 상태, shared-library 목록에서 `liblayers.so`가 사라진 상태를 함께 확인했다. 센서 입력 지연이나 장애물 판정 실패가 아니라, 이 standalone 시험의 종료 시 객체·라이브러리 수명 문제다.

`smoke_depth_voxel_layer.py`의 자식 프로세스 환경에만 동일한 설치 라이브러리를 `LD_PRELOAD`로 유지한다. 다른 라이브러리로 교체하는 것이 아니라 마지막 callback 정리까지 unload를 늦추는 대응이다. 경로는 ament package prefix로 찾고 기존 preload 값은 보존한다. 부모 환경·실차 launch·시스템 패키지는 바꾸지 않는다. 이 대응은 현재 두 layer가 함께 들어 있는 `liblayers.so`에 한정되며, 다른 플러그인을 추가하면 수명 조건을 다시 확인해야 한다. upstream 소스의 파괴 순서를 고친 것은 아니다.

시험 성공 조건도 강화했다. 네 기능이 통과해도 자식 종료 코드가 0이 아니거나, lifecycle deactivate·cleanup이 끝나지 않았거나, cleanup 예외가 있으면 JSON과 CLI 모두 실패로 기록한다. `-11`, 강제 종료, 종료 코드 누락을 성공으로 인정하지 않는다. 재실행 시 이전 오류가 남지 않으며 자식 시작 실패 시 임시 파라미터 파일도 정리한다.

### 검증 결과

- 수정 전: 동일 네 기능 통과, 자식 종료 `-11` 재현.
- 동일 라이브러리 수명 유지 A/B: 네 기능 유지, 자식 종료 `0`.
- 반영 코드로 독립 실행 2회: 매회 비용 `254 → 254 → 254 → 0`, lifecycle 정리 완료, 자식 종료 `0`, CLI 종료 `0`.
- 관련 회귀 테스트 269개 통과. 이 중 새 종료·환경 격리·실패 판정 회귀는 14개다.
- 변경 Python 2개 파일의 `ament_flake8`, `ament_pep257`와 navigation 패키지 빌드 통과. 기존 `mcap_ros2` reader의 deprecation warning 1건은 별개로 남는다.
- 최초에는 과거 IP `192.168.0.205`, `192.168.0.160`과 잘못된 호스트명 `deepthinkcar.local`만 조회해 Pi 연결 불가로 보고했다. 이는 기존 로봇의 접속 확인으로 충분하지 않았다. 올바른 접속 대상은 `lim@jdamr.local`이며 아래 정정 기록을 따른다.

증거 위치:

```text
$HOME/jdamr_artifacts/depth_obstacles_20260921/shutdown_baseline/summary.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/shutdown_gdb_vtable/nav2_costmap.log
$HOME/jdamr_artifacts/depth_obstacles_20260921/shutdown_library_lifetime_ab/summary.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/shutdown_fixed/summary.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/shutdown_fixed_repeat/summary.json
```

과거 baseline의 `status: pass`는 수정 전 판정 결함의 재현 기록이다. `cleanup.anomaly: true`와 자식 `-11` 때문에 전체 성공 증거가 아니다. GDB 실행 2건은 debugger 프로세스 종료 0과 실제 target의 SIGSEGV를 구분하도록 `diagnostic_only_target_sigsegv`로 표시했다.

근거 소스: [Nav2 1.3.12 멤버 선언 순서](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_costmap_2d/include/nav2_costmap_2d/costmap_2d_ros.hpp), [Nav2 lifecycle cleanup](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_costmap_2d/src/costmap_2d_ros.cpp), [rclcpp 28.1.21 callback group](https://github.com/ros2/rclcpp/blob/28.1.21/rclcpp/include/rclcpp/callback_group.hpp), [pluginlib loader 소멸](https://github.com/ros/pluginlib/blob/5.4.5/pluginlib/include/pluginlib/class_loader_imp.hpp).

### Pi 접속 기록 정정

- 9월 21일 09:46 KST: `jdamr.local`이 `192.168.0.159`로 해석됐고 기존 SSH 키 검증을 유지한 접속에 성공했다. 원격 `hostname`은 `jdamr`, uptime은 약 45분이었다. `real_bringup`, `robot_state_publisher`, `ydlidar_g4_node` 실행을 확인했다. bringup 인자는 바퀴 반경 `0.0329 m`, 간격 `0.510 m`였다. 프로세스 존재 확인이며 토픽 데이터 품질 검증은 아니다.
- 성공 당시 작업 PC는 `aicampus_286`, IP `192.168.0.217`이었다. 10:02 KST에는 PC가 `robot`, IP `192.168.0.42`로 바뀌었고 기본 게이트웨이 MAC도 달랐다. 이 네트워크에서 `jdamr.local` 조회는 timeout, 직전 확인 주소의 SSH는 `No route to host`였다. PC의 네트워크 변경은 확인됐지만 Pi의 현재 연결망·전원 상태는 원격으로 확정하지 않았다.
- 고정 IP를 현재 주소로 단정하지 않는다. 재접속은 기존 호스트명과 SSH 키로 식별하고 작업 PC의 Wi-Fi를 함께 확인한다. 주소가 해석되지 않을 때 네트워크를 임의 변경하거나 파이 재부팅을 먼저 요구하지 않는다.

### Pi 관측 전용 배포 — 9월 21일 후속

`aicampus_286` 재연결 후 `lim@jdamr.local` SSH에 성공했다. 커밋 `8936476`의 navigation·description·vslam 3개 패키지를 아래 별도 작업 공간에 복사하고 Pi에서 빌드했다. 기존 `$HOME/jdamr_ws` 소스·설치·서비스는 덮어쓰거나 재시작하지 않았다.

```text
Pi: $HOME/jdamr_deploy/depth_observer_8936476_UDRrKw
PC 증거: $HOME/jdamr_artifacts/depth_obstacles_20260921/pi_observer_8936476
```

- Pi에서 3개 패키지 빌드 성공. depth core/filter/config 테스트 70개 통과. 핵심 코드 2개와 뎁스·차체·카메라 설정 3개의 SHA-256이 PC 원본과 일치했다.
- 베이스 프로세스의 실효 설정은 `ROS_DOMAIN_ID=12`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`였다. 동일 설정에서 현재 시각의 `/scan` 메시지를 수신했고 frame은 `laser_link`였다. 첫 짧은 자동 타입 탐색은 실패했으나 명시적 LaserScan 타입 수신으로 데이터 존재를 확인했다.
- `mode:=observe publish_camera_mount:=false`로 30초 한정 실행했다. 상태 메시지는 `waiting_for_depth`, `healthy=false`였고 노드의 발행 목록에 속도 명령은 없었다. Nav2·정적 camera mount TF·카메라 드라이버·주행 목표는 실행하지 않았다.
- Pi USB 목록에는 Orbbec 장치가 없었고 `jdamr-astra-camera.service`는 inactive였다. 따라서 실제 뎁스 처리, 두 센서 좌표 정렬, Pi 실시간 처리 성능과 실차 회피는 검증하지 못했다. 카메라가 인식되면 등록 RGB-D 설정으로 입력을 시작해야 하며, 기존 camera service의 색상 비활성 기본 설정을 그대로 사용해 검증하지 않는다.
- 한정 실행의 timeout wrapper는 예정된 SIGINT 후 `124`를 반환했다. ROS launch는 관측 자식의 clean exit를 보고했지만 `cannot use Destroyable because destruction was requested` 경고가 2건 남았다. 종료 로그를 보존했고 무경고 종료로 주장하지 않는다. 관측 노드는 계속 켜 두지 않았다.
- 종료 후 `jdamr-base.service`는 `active`, MainPID `997`로 유지됐다. 센서 통합 허용 상태는 `blocked_until_cross_sensor_validation` 그대로다.

카메라 연결 후 사용할 관측 실행 환경(Pi에서 실행):

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_ws/install/setup.bash"
source "$HOME/jdamr_deploy/depth_observer_8936476_UDRrKw/install/local_setup.bash"
export ROS_DOMAIN_ID=12 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch jdamr_cube_navigation depth_obstacle_navigation.launch.py mode:=observe
```

이 명령만으로 카메라 드라이버나 장착 TF가 생기지는 않는다. 기존 TF 발행 여부와 실제 장착 상태를 확인한 뒤 카메라 드라이버 및 필요한 장착 TF를 별도로 기동한다.

### 카메라 재연결 후 실측과 수정

Orbbec Astra S (`2bc5:0402`) 재인식 후 등록 RGB-D를 기동했다. RGB 640×480, 뎁스 320×240이며 둘 다 `camera_color_optical_frame`으로 수신했다. 저장된 `base_link → camera_link` 장착값을 관측용으로 발행했지만, 실물 장착 재측정이나 센서 통합 승인을 뜻하지 않는다.

1. **TF 시간 문제:** Pi의 Astra 드라이버는 고정 카메라 내부 변환을 기본 `tf_publish_rate=10.0`에서 `/tf`로 발행했다. 관측 노드의 영상 시각 TF 조회가 50 ms 안에 성공하지 못하는 경우가 있었다. 12초 상태 표본은 `missing_transform_at_measurement_time` 17개, `healthy` 1개, `depth_stream_timeout` 2개였다. Pi에 설치된 `ob_camera_node.cpp`의 `publishStaticTransforms()` 분기에서 rate≤0은 동일한 고정 변환을 `/tf_static`으로 발행함을 확인했다. `tf_publish_rate:=0.0`으로 전환했고 기록 스크립트와 센서 설정 문서에도 반영했다. 영상 시각 검증·처리 기한은 완화하지 않았다.
2. **정적 TF 전환 직후 관측:** 첫 12초 실측의 상태 메시지는 22/22 healthy, TF 누락 0이었다. 이는 상태 메시지 표본 비율이며 모든 영상의 처리 성공률이 아니다. 배분 루프 수정 전 8초 캡처 중에는 14/15 healthy, `expired_during_processing` 1개가 있었다. 해당 프레임은 기존 0.5초 기한으로 거부됐다. 상태 표본의 처리 시간 p50 46.07 ms, p95 323.24 ms였으며 RGB·뎁스 동시 수집 부하를 포함한다. 정적 TF 변경만으로 Pi의 부하 문제가 해결된 것은 아니었다.
3. **시각 대응 캡처:** 새 `evaluation/capture_depth_alignment.py`는 읽기 전용으로 depth/scan/CameraInfo/RGB/status를 수집한다. 최신 depth 하나만 사용하던 초기 실행은 scan과 130.5 ms 차이로 실패했다. 최근 0.5초 내 양수·비미래 stamp 중 최신 depth부터 최근접 scan이 100 ms 이내인 쌍을 선택하도록 수정했다. 오래된 쌍이나 허용치 완화로 성공시키지 않는다. 원본 stamp의 TF, 영상 stride·endianness, RGB 시각 차이, 실패 원인 및 TF 재시도를 JSON/NPZ에 보존한다. `success`는 스냅샷 확보일 뿐 정렬 승인이나 주행 성공이 아니다.
4. **재시도 진단 추가 후 캡처:** 8초 동안 depth 219개, scan 73개, RGB 160개 수신. 선택한 depth/scan 시각 차이는 77.31 ms, 캡처 시 영상 나이는 107.39 ms, scan 나이는 184.70 ms였다. 원 stamp TF 조회는 첫 시도에 성공했고 재시도 오류는 없었다. 기존 12초 성공 캡처도 보존했다.
5. **장면 확인:** RGB·뎁스·XY 겹침 그림을 생성하고 확인했다. 12초 성공 캡처에서 0.4–2.5 m 범위의 뎁스는 전체 픽셀의 8.31%였다. 명목 높이 0.15±0.025 m의 뎁스 489점과 가장 가까운 라이다 XY 점의 거리 중앙값은 약 0.413 m였다. 같은 표면 대응을 보장하지 않으므로 이를 센서 위치 오차나 보정값으로 쓰지 않는다. 현재 장면만으로 두 센서 정렬을 입증하지 못했다. 다음 실측은 로봇 앞 약 1 m, 바닥에서 시작하는 넓은 세로 박스·판을 두 센서가 함께 보게 하고 비교한다. `lidar_rgbd_fusion`은 계속 차단 상태다.
6. **배분 루프 부하 수정:** RGB 저장을 하지 않는 12초 표본에서도 healthy 17개, stream timeout 3개, 처리 기한 초과 1개가 발생했다. 처리 시간 p50/p95는 15.42/261.55 ms였다. `ps`에서 관측 프로세스 CPU 94.9%, 주 스레드 79.1%, 작업 스레드 각각 6.8/6.9%를 관찰했다. 이 값은 프로세스별 누적 평균이며 통제된 CPU 벤치마크는 아니다. 두 작업 스레드는 유지하고 `spin_once(timeout_sec=0.05)` 뒤 1 ms 실행권 양보를 추가했다. TF timeout·영상 시각·0.5초 freshness 정책은 바꾸지 않았다. 같은 12초 후속 표본은 22/22 healthy, p50/p95 9.30/12.33 ms였다. [rclpy의 유사 고부하 보고](https://github.com/ros2/rclpy/issues/1223)는 참고 자료이며 동일 upstream 결함으로 확정하지 않는다.
7. **수정 후 동시 수집:** 8초 RGB·뎁스 동시 캡처에서 depth 140개, scan 73개, RGB 111개를 수신했다. 상태는 11/11 healthy, 처리 시간 p50/p95 9.78/14.06 ms였다. depth/scan 차이는 98.40 ms, 데이터 나이는 각각 55.01/153.41 ms, exact-stamp TF 첫 시도 성공이었다. 프레임 수는 BEST_EFFORT depth 1 구독의 수신 개수이므로 센서 FPS나 전체 프레임 처리율을 뜻하지 않는다. 짧은 정지 실측 개선이며 장시간·Nav2 동시 주행 안정성은 미검증이다.

카메라와 관측 프로세스는 SSH 종료와 무관한 임시 systemd 서비스 `jdamr-rgbd-live-20260921`, `jdamr-depth-live-20260921`로 실행한다. 초기 user 서비스가 SSH 로그아웃과 함께 종료되는 현상을 확인해 system manager의 `User=lim` 서비스로 옮겼다. 부팅 자동 시작 서비스는 바꾸지 않았다. 기존 베이스 MainPID `997`과 바퀴 설정을 유지했고, 이동 명령과 Nav2는 실행하지 않았다. 관측 프로세스만 중단하려면 Pi에서 아래 명령을 사용한다.

```bash
sudo systemctl stop jdamr-depth-live-20260921 jdamr-rgbd-live-20260921
```

PC 근거:

```text
$HOME/jdamr_artifacts/depth_obstacles_20260921/live_static_tf_paired/capture.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/live_static_tf_paired/capture.npz
$HOME/jdamr_artifacts/depth_obstacles_20260921/live_static_tf_paired/alignment_preview.png
$HOME/jdamr_artifacts/depth_obstacles_20260921/live_static_tf_final/capture.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/live_static_tf_final/capture.npz
$HOME/jdamr_artifacts/depth_obstacles_20260921/live_static_tf_final/observer_only_status.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/live_static_tf_final/observer_worker_yield_status.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/live_worker_yield/capture.json
```

코드 검증은 새 캡처 회귀 26개와 배분 루프 회귀 2개를 포함한 관련 98개 테스트, 변경 Python의 ament_flake8·ament_pep257, 기록 셸의 `bash -n`, navigation·vslam 로컬 빌드를 통과했다. LSP와 shellcheck는 실행하지 못했다. Pi에는 별도 관측 공간의 필터·캡처 도구·기록 스크립트·센서 설정만 갱신했다.

### 박스 배치 후 정지 관측과 대시보드

사용자가 전방에 박스를 놓은 뒤 8초 캡처를 두 번 수행했다. RGB에서 Hantek 박스가 확인되고, 등록 뎁스에는 전면의 연속된 거리 영역, 라이다에는 같은 방향의 평면 구간이 나타났다. 로봇 이동·Nav2 실행·장착 TF 수정은 하지 않았다.

| 관측 | 대시보드 미실행 | 기존 Depth 화면 표시 중 |
| --- | ---: | ---: |
| 정상 상태 메시지 | 12/12 | 15/15 |
| 처리 시간 p95 | 16.47 ms | 22.64 ms |
| 선택한 depth/scan 시각 차이 | 98.60 ms | 83.74 ms |
| 박스 뎁스 전방 좌표 중앙값 | 0.949 m | 0.949 m |
| 박스 라이다 전방 좌표 중앙값 | 0.975 m | 0.976 m |

전방 좌표의 원점은 `base_footprint`이며, 실측 차체 앞면 x=0.065 m를 빼면 약 0.884 m와 0.910–0.911 m다. 화면을 보고 선택한 뎁스 내부 영역(u=145–235, v=153–205, 0.6–1.2 m)과 같은 좌우 범위의 라이다 점을 비교했다. 자동 박스 분류 결과가 아니다. 두 센서가 보는 표면 높이가 다르므로 26–27 mm 차이를 캘리브레이션 오차나 주차 정확도로 해석하지 않는다.

**남은 불일치:** 저장된 명목 장착 TF로 박스의 라이다 점을 뎁스 영상에 투영하면 약 140행인데, 실제 박스 영역은 중앙 열에서 148–210행이다. 선택한 박스 뎁스의 명목 높이도 -0.048–0.111 m로 계산돼 일부 점이 바닥 아래로 내려간다. 따라서 전방 물체 관측은 확인했지만 높이·자세 정합은 승인할 수 없다. `camera_mount.yaml`의 roll/pitch는 명목 0이며, 현재 자료만으로 카메라 기울기·박스 기울기·등록/내부 파라미터 오차를 분리할 수 없다. 임의 TF 보정이나 `lidar_rgbd_fusion` 허용 변경은 하지 않았다. 다음 단계는 바닥과 수직 기준면을 이용한 장착 자세 및 투영 검증이다.

대시보드는 보조 저장소 `$HOME/bimanual-robot-ammr/tools/ammr_dashboard`의 읽기 전용 AMMR 센서 화면을 사용한다. Pi의 동일 소스 `$HOME/ammr_dashboard`를 원래 카메라 서비스를 재기동하지 않는 별도 임시 서비스로 실행했다. 최종 서비스는 `jdamr-dashboard-box-20260921`이며 Pi의 127.0.0.1:8090을 SSH로 PC의 127.0.0.1:8090에 전달한다. 구동 명령 API는 없다. 기존 화면은 raw `laser_link` +X를 차체 전방으로 표시하는 문제가 있어, 실제 장착 yaw=180°·x=-0.010 m를 명시한 표시 변환으로 수정했다. 웹의 전방 값이 약 2.7 m에서 약 0.98 m로 바뀌었으며 차체 외곽 여유거리와 구분해 표시한다. 이는 대시보드 좌표 표시 수정이며 베이스·Nav2 설정 변경이 아니다. RGB 640×480·Depth 320×240 동시 영상, 장애물 관측 상태, 처리 시간을 브라우저에서 확인했다. 중앙 Depth 값 약 3.64 m는 화면 중앙의 배경 거리이며 아래쪽 박스의 거리가 아니다.

대시보드 회귀 테스트 9개와 별도 코드 검토를 통과했다. 상태 메시지 원본 영상 나이(`source_age_s`)와 대시보드 수신 나이(`age_s`)를 분리했다. 브라우저에만 통신 오류를 주입했을 때 센서 배지는 STALE, 카운트는 `—/2 LIVE`, 영상 주소는 제거됐으며 복구 후 LIVE/HEALTHY와 두 영상이 다시 표시됐다. 로봇·센서 서비스를 끄는 장애 주입은 하지 않았다.

근거와 재현 분석:

```text
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_01/capture.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_01/capture.npz
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_01/box_comparison.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_01/box_comparison.png
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_02_dashboard/capture.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_02_dashboard/capture.npz
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_02_dashboard/box_comparison.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_02_dashboard/box_comparison.png
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_02_dashboard/dashboard_state.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_02_dashboard/dashboard_live.png
$HOME/jdamr_artifacts/depth_obstacles_20260921/box_alignment_02_dashboard/runtime_evidence.txt
$HOME/jdamr_artifacts/depth_obstacles_20260921/analyze_box_alignment.py
```

## 실행 및 실차 적용 조건

카메라 드라이버와 차체 TF는 기존 시스템에서 제공해야 한다. `publish_camera_mount:=true`를 선택하면 실측 mount 파일에서 `base_link` → `camera_link` 정적 TF를 발행한다. 같은 TF를 발행하는 이전 wrapper는 함께 사용하지 않는다. 장착 TF 발행은 센서 간 정렬의 실측 검증을 대신하지 않는다.

기본 관측 모드는 기존 장착 TF를 사용한다. TF가 없으면 누락 상태만 보고하며, 아래 명령에 `publish_camera_mount:=true`를 추가해 저장된 장착값을 적용할 수 있다. 물리 navigation/mapping은 승인된 mount 파일과 이 launch의 장착 TF 발행을 모두 요구한다.

관측 전용 실행:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_rgbd_ws/install/setup.bash"
ros2 launch jdamr_cube_navigation depth_obstacle_navigation.launch.py mode:=observe
```

관측한 벽·박스가 두 센서에서 같은 위치에 놓이는지 확인한 뒤 장착 provenance와 fusion 허용 상태를 갱신해야 한다. 현재 `usage_gate.lidar_rgbd_fusion`은 `blocked_until_cross_sensor_validation`으로 유지했다. 물리 `navigation`/`mapping` 모드는 이 상태에서 시작되지 않는다. 시뮬레이션 모드는 명시적 DDS domain 100–232, LOCALHOST 탐색, 비어 있는 static peers를 요구한다. 실차 domain 12에서는 `use_sim_time`으로 검증을 건너뛸 수 없다.

Pi 별도 공간에서 실제 RGB-D 입력·장애물 관측·정지 상태 시각 대응 캡처까지 확인했다. 실제 센서 정렬 승인·회피 주행은 아직 수행하지 않았다. 새 layer가 생성하는 장애물은 RGB-D 기반 2.5D 주행 비용지도이며, SLAM으로 정합된 3D 지도나 물체 분류 결과가 아니다. 기본 depth 사용 범위는 0.4–2.5 m이므로 이 구현만으로 5 cm 테이블 밀착을 보장하지 않는다. 센서 최소 거리·가림·보이지 않는 방향은 별도 접근 제어에 반영해야 한다.

참고 구현: [Nav2 1.3.12 VoxelLayer](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_costmap_2d/plugins/voxel_layer.cpp), [ObstacleLayer](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_costmap_2d/plugins/obstacle_layer.cpp), [Collision Monitor PointCloud](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_collision_monitor/src/pointcloud.cpp).
