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

VoxelLayer 시험의 standalone costmap 프로세스는 네 기능을 확인하고 lifecycle deactivate·cleanup을 마친 뒤 SIGINT 종료에서 `-11`을 반환했다. 잔존 자식 프로세스는 없으나 종료 오류의 원인은 규명하지 못했다. 기능 시험 통과와 종료 안정성은 구분하며, 전체 Nav2 통합 실행의 정상 종료까지 확인됐다고 주장하지 않는다. 앞선 harness 설정·서비스 이름·셀 경계 문제로 실패한 실행 로그도 별도 디렉터리에 보존했다.

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

현재 Pi의 알려진 IP 두 곳에 SSH가 닿지 않아 배포·실시간 센서 정렬·회피 주행은 수행하지 않았다. 새 layer가 생성하는 장애물은 RGB-D 기반 2.5D 주행 비용지도이며, SLAM으로 정합된 3D 지도나 물체 분류 결과가 아니다. 기본 depth 사용 범위는 0.4–2.5 m이므로 이 구현만으로 5 cm 테이블 밀착을 보장하지 않는다. 센서 최소 거리·가림·보이지 않는 방향은 별도 접근 제어에 반영해야 한다.

참고 구현: [Nav2 1.3.12 VoxelLayer](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_costmap_2d/plugins/voxel_layer.cpp), [ObstacleLayer](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_costmap_2d/plugins/obstacle_layer.cpp), [Collision Monitor PointCloud](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_collision_monitor/src/pointcloud.cpp).
