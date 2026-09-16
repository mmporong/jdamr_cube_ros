# JD-AMR RGB-D Visual SLAM

이 패키지는 기존 2D LiDAR SLAM·Nav2 운용을 변경하지 않고 Orbbec Astra S 입력을
별도 기록해 RTAB-Map으로 미터 스케일의 3D 지도를 생성한다.

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

- 1차: 카메라 단독 RGB-D baseline과 loop closure 성공 여부
- 2차: wheel odometry guess를 사용한 저텍스처 복도 강건성 비교
- 3차: 2D LiDAR scan을 추가한 RGB-D+LiDAR graph 최적화 비교
- 4차: voxel 크기·feature detector·inlier 문턱을 한 변수씩 조정
- 5차: 동일 bag에 대한 A/B 결과를 거리·yaw·ATE·loop error로 비교

실행 데이터, database, PLY/OBJ는 대용량 산출물이므로 저장소에 커밋하지 않는다.
정적 연결 검증 수치는 `evaluation/20260916_RGBD_VSLAM_SMOKE.md`에 분리했다.
P턴 실주행 A/B와 3D 산출물 판정은
`evaluation/20260916_RGBD_VSLAM_PTURN.md`에 기록했다.
