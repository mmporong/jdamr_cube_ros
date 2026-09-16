# Astra S 90° 회전 point cloud 융합 평가

## 판정

- 실기체는 시계 방향으로 91.25° 회전했고 병진 잔차는 1.1 mm였다.
- rosbag에는 회전 중 48.64°만 RGB-D와 함께 저장됐다. 목표 90° 대비 54.04%이므로
  이번 산출물의 판정은 `partial_capture`다.
- 저장된 구간에서는 5° 간격 11개 RGB-D 프레임을 미터 단위로 역투영하고 wheel
  odometry pose로 변환해 2 cm voxel 컬러 점군을 만들었다.
- 이 결과는 RGB-D back-projection, optical-to-base 좌표변환, pose 적용, voxel
  fusion을 실제 센서 데이터로 검증한 증거다. loop closure와 scan matching이 없으므로
  SLAM 지도나 디지털 트윈으로 사용하지 않는다.

## 입력과 동기화

| 항목 | 값 |
|---|---:|
| 기록 경로 | `$HOME/jdamr_data/vslam/rgbd_20260916T195627` |
| rosbag 구간 | 2026-09-16 19:56:53.422~19:57:44.800 KST |
| rosbag 길이 | 51.378 s |
| 동기 RGB-D pair | 415 |
| RGB-Depth 평균 차이 | 10.696 ms |
| RGB-Depth P95 차이 | 22.688 ms |
| RGB-Depth 최대 차이 | 26.490 ms |
| wheel odometry 전체 yaw | -49.339° |
| 융합에 사용한 yaw | -48.636° |

모션 프로브는 파일 시각과 track monotonic 시각을 대조하면 약 19:57:35.6부터
19:57:53.2까지 실행됐다. 50초 rosbag이 약 8.4초 먼저 종료되어 회전 후반부가
저장되지 않았다. 따라서 센서나 융합 코드의 90° 회전 실패로 해석하지 않는다.

## 출력

출력 디렉터리는
`$HOME/jdamr_data/vslam/rgbd_20260916T195627/odom_fusion_5deg`이다.

| 파일 | 내용 |
|---|---|
| `fused_cloud.ply` | 21,878점의 2 cm voxel 컬러 점군 |
| `selected_frames.csv` | 목표·실제 yaw, timestamp, pair delta, frame별 점 수 |
| `fusion_summary.json` | 입력·동기화·회전 coverage·점군 범위·가정 |
| `fusion_preview.png` | 등각 시점과 상단 시점 검토 이미지 |

융합 전 유효점은 30,702점이며 voxel 평균 뒤 21,878점이다. 점군 범위는 X
0.163~4.456 m, Y -3.883~2.006 m, Z -1.325~1.336 m다. Z는 카메라 광학 중심을
0으로 둔 값이며 바닥 기준 높이가 아니다.

## 재현 명령

```bash
ros2 run jdamr_cube_vslam fuse_rgbd_odom \
  --input "$HOME/jdamr_data/vslam/rgbd_20260916T195627/bag_paired_10fps" \
  --output-dir "$HOME/jdamr_data/vslam/rgbd_20260916T195627/odom_fusion_5deg" \
  --yaw-step-deg 5 \
  --pixel-step 2 \
  --voxel-m 0.02 \
  --expected-yaw-deg 90
```

## 다음 데이터의 통과 조건

1. rosbag 구독 완료 뒤 회전을 시작하고, 동작 종료 뒤 5초 이상 기록을 유지한다.
2. `capture_complete=true`, yaw coverage 90% 이상을 확인한다.
3. 카메라의 base 기준 x/y/z/roll/pitch/yaw를 실측해 CLI에 넣는다.
4. 90° 구간의 점군 정합을 확인한 뒤에만 내일 주행 기록으로 범위를 확장한다.
5. 주행 데이터에서는 wheel odometry seed와 실제 SLAM 결과를 별도 산출물로 비교한다.
