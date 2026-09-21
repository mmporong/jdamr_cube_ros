# JD-AMR RGB-D Visual SLAM

이 패키지는 기존 2D LiDAR SLAM·Nav2 운용을 변경하지 않고 Orbbec Astra S 입력을
별도 기록해 RTAB-Map으로 미터 스케일의 3D 지도를 생성한다.

현재 상태는 입력·visual odometry·DB 생성까지 검증됐지만 연속적인 3D SLAM 지도는 아직
통과하지 못했다. 저텍스처 P턴에서 발생한 재초기화와 분리 점군을 wheel odometry 보조와
LiDAR graph 제약으로 개선하는 것이 현재 목표다. 다음 실행 순서와 성공 기준은
[RGB-D Visual SLAM 다음 실행 계획](evaluation/20260917_RGBD_VSLAM_NEXT_RUN.md)을 따른다.
같은 P턴 bag의 wheel-assisted A/B 결과와 TF frame 수정 근거는
[휠 오도메트리 보조 A/B](evaluation/20260917_RGBD_WHEEL_ASSISTED_AB.md)에 기록했다.

## 실행 구조

1. Raspberry Pi에서 `capture_rgbd_bag.sh`가 기존 Depth 전용 서비스를 잠시 멈춘다.
2. RGB, 컬러 정렬 Depth, camera_info, camera TF, wheel odometry를 MCAP으로 기록한다.
3. 기록이 끝나면 기존 `jdamr-astra-camera.service`를 자동 복구한다.
4. 노트북은 공식 `introlab3it/rtabmap_ros:jazzy` 컨테이너에서 bag을 재생해
   `jdamr_rgbd.db`, 3D cloud, visual/reference trajectory CSV를 만든다.
5. `vslam_accuracy`가 스케일을 보정하지 않은 SE(2) 정렬로 거리·각도 오차를 계산한다.

## Raspberry Pi 기록

```bash
ros2 run jdamr_cube_vslam capture_rgbd_bag.sh --duration 120
```

패키지를 파이에 설치하지 않았으면 저장소의 스크립트를 직접 실행한다.

```bash
bash "$HOME/jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_vslam/scripts/capture_rgbd_bag.sh" --duration 120
```

## 노트북 오프라인 매핑

```bash
bash "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_vslam/scripts/run_rtabmap_docker.sh" \
  "$HOME/jdamr_data/vslam/rgbd_YYYYMMDDTHHMMSS/bag" \
  "$HOME/jdamr_data/vslam/rgbd_YYYYMMDDTHHMMSS/rtabmap"
```

이 명령은 실제 assembled 3D cloud가 생성되지 않으면 실패한다. 센서 연결만 확인하는
정적 smoke bag에는 `--allow-static`을 추가한다.

기본 카메라 전용 경로는 입력 bag을 수정하지 않고 `visual_input` 처리본을 만든다.
카메라 내부 TF만 남기고 바퀴 TF와 주행 명령 토픽을 제외하며, 시각 추정은
`/rtabmap/odom`으로 발행한다. 원본 `/odom`은 비교 기록에만 사용한다.
오프라인 재생은 0.5배속이며, 점군 export는 depth decimation 2를 사용한다.

회색 벽처럼 특징이 적은 구간을 포함한 기록은 동기 RGB-D bag과 보수적인
저텍스처 프로파일을 사용한다.

```bash
bash "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_vslam/scripts/run_rtabmap_docker.sh" \
  "$HOME/jdamr_data/vslam/rgbd_YYYYMMDDTHHMMSS/bag_paired_10fps" \
  "$HOME/jdamr_data/vslam/rgbd_YYYYMMDDTHHMMSS/rtabmap_low_texture" \
  --profile low-texture
```

`low-texture`는 최소 inlier를 10으로 유지하고 5회 연속 추적 실패 시 새 map으로
재초기화한다. 최소 inlier를 더 낮추면 로그상 추적 손실은 줄어도 잘못된 대응으로
점군이 찢어질 수 있으므로 기본 설정으로 사용하지 않는다.

2026-09-21부터 `--odom-guess-frame` 혼합 경로는 실행 전에 오류를 반환한다.
기존 구성은 같은 카메라 자식 frame에 바퀴·시각 TF가 함께 연결되거나,
시각 TF를 끄면 mapping의 `vslam_odom → camera_link`가 끊기는 문제가 있다.
장착값의 측정 여부와 별개인 TF 구조 문제이며, 별도 참조/시각 트리 검증 전까지
카메라 전용 모드 또는 아래 외부 odometry 모드를 사용한다.

Visual Odometry 프런트엔드와 분리해 RGB-D 매핑 데이터 자체를 확인할 때는 wheel
`/odom`을 외부 odometry로 사용한다.

```bash
bash "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_vslam/scripts/run_rtabmap_docker.sh" \
  "$HOME/jdamr_data/vslam/rgbd_YYYYMMDDTHHMMSS/bag_paired_10fps" \
  "$HOME/jdamr_data/vslam/rgbd_YYYYMMDDTHHMMSS/rtabmap_external_odom" \
  --profile low-texture \
  --external-odom
```

이 결과는 Visual SLAM이 아니라 odometry-seeded RGB-D mapping으로 분류한다.

## 거리·각도 검증

```bash
ros2 run jdamr_cube_vslam vslam_accuracy \
  --visual visual_trajectory.csv \
  --reference reference_trajectory.csv \
  --output-json accuracy.json \
  --output-md accuracy.md
```

휠 odometry나 AMCL은 비교 기준이지 외부 ground truth가 아니다. 포트폴리오에서 절대
정확도를 주장하려면 줄자로 잰 직선 구간과 각도 지그로 고정한 회전 구간을
`config/accuracy_protocol.yaml`에 기록하고 같은 명령에 `--measurements`로 넘긴다.

## Wheel odometry 기반 point cloud 융합

카메라 단독 visual odometry가 저텍스처 구간에서 실패하더라도 RGB-D 역투영과
좌표변환 파이프라인은 별도로 검증할 수 있다. 다음 명령은 wheel odometry의 yaw를
초기 pose로 사용해 5° 간격 프레임을 고르고 2 cm voxel 컬러 점군을 만든다.

```bash
ros2 run jdamr_cube_vslam fuse_rgbd_odom \
  --input "$HOME/jdamr_data/vslam/rgbd_YYYYMMDDTHHMMSS/bag_paired_10fps" \
  --output-dir "$HOME/jdamr_data/vslam/rgbd_YYYYMMDDTHHMMSS/odom_fusion_5deg" \
  --yaw-step-deg 5 \
  --voxel-m 0.02 \
  --expected-yaw-deg 90
```

출력은 `fused_cloud.ply`, `selected_frames.csv`, `fusion_summary.json`이다. 이 방식에는
loop closure나 scan matching이 없으므로 SLAM 지도로 부르지 않는다. 카메라 장착
변환을 실측하지 않은 상태에서는 기본값 0을 사용했다는 제한도 JSON에 기록된다.

회전 실험은 `--duration 120`처럼 준비 시간을 포함해 충분한 녹화 시간을 주거나,
시간 제한 없이 기록한 뒤 `Ctrl+C`로 끝낸다. `--expected-yaw-deg`를 지정하면 목표의
90% 미만만 저장된 기록을 `partial_capture`로 표시한다.

## 3D 결과물 고도화

빠른 검토용 PLY는 2 cm voxel과 노이즈 필터를 적용한다.

```bash
bash "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_vslam/scripts/export_3d_assets.sh" \
  /path/to/jdamr_rgbd.db /path/to/export preview
```

포트폴리오용 출력은 1 cm voxel 컬러 cloud와 4096 px texture mesh를 함께 만든다.
내보내기는 데이터베이스의 마지막 저장 pose만 재사용하지 않고 graph를 다시
최적화한다.

```bash
bash "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_vslam/scripts/export_3d_assets.sh" \
  /path/to/jdamr_rgbd.db /path/to/export portfolio
```

## 고도화 순서

- 선행 조건: `base_link → camera_link` 외부 파라미터 실측과 정적 TF 공급
- 1차: 카메라 단독 RGB-D baseline과 loop closure 성공 여부
- 2차: wheel odometry guess를 사용한 저텍스처 복도 강건성 비교
- 3차: 2D LiDAR scan을 추가한 RGB-D+LiDAR graph 최적화 비교
- 4차: voxel 크기·feature detector·inlier 문턱을 한 변수씩 조정
- 5차: 동일 bag에 대한 A/B 결과를 거리·yaw·ATE·loop error로 비교

실행 데이터, database, PLY/OBJ는 대용량 산출물이므로 저장소에 커밋하지 않는다.
정적 연결 검증 수치는 `evaluation/20260916_RGBD_VSLAM_SMOKE.md`에 분리했다.
P턴 실주행 A/B와 3D 산출물 판정은
`evaluation/20260916_RGBD_VSLAM_PTURN.md`에 기록했다.
wheel odometry를 사용한 90° 회전 point cloud 실험은
`evaluation/20260916_RGBD_ODOM_FUSION_90TURN.md`에 기록했다.
0.7m 자동 폐루프의 Visual SLAM과 external wheel odom RGB-D mapping 비교는
`evaluation/20260917_RGBD_CLOSED_LOOP_PROBE.md`에 기록했다.
