# Astra S RGB-D P턴 주행 3D 매핑 평가

## 결론

- 182.45초 실주행 기록에서 RGB와 Depth를 센서 타임스탬프로 다시 짝지어
  1,482쌍의 입력을 만들었다.
- 카메라 단독 RTAB-Map baseline은 회색 벽 선회 구간에서 추적이 끊겼다.
- `low-texture` 프로파일은 특징 추적 실패 비율을 32.49%에서 1.40%로 줄이고,
  내보낸 점군을 10,894점에서 44,498점으로 늘렸다.
- 저텍스처 선회 중 visual odometry 자동 재초기화가 2회 발생했다. 따라서 이번
  결과는 RGB-D 3D 매핑 가능성과 실패 복구 실험의 증거이며, 완전한 연속 궤적이나
  절대 위치 정확도 증거로 사용하지 않는다.
- 최소 inlier를 5까지 낮춘 실험은 추적 손실 수치가 더 작았지만, 점군 범위가
  8.68 m까지 늘어나며 공간이 찢어졌다. 이 설정은 최종 프로파일에서 제외했다.

## 입력 데이터

| 항목 | 값 |
|---|---:|
| 원본 경로 | `$HOME/jdamr_data/vslam/rgbd_20260916T172638/bag` |
| 기록 길이 | 182.4529 s |
| 원본 메시지 | 39,365 |
| 처리 입력 | `$HOME/jdamr_data/vslam/rgbd_20260916T172638/bag_paired_10fps` |
| 동기 RGB-D pair | 1,482 |
| RGB-Depth 평균 차이 | 10.826 ms |
| RGB-Depth P95 차이 | 22.610 ms |
| RGB-Depth 최대 차이 | 27.740 ms |
| 동기 허용 상한 | 30 ms |

RGB와 Depth를 각각 10 Hz로 줄인 최초 입력은 같은 프레임 쌍을 보장하지 않았다.
최종 입력은 센서 헤더 시각을 기준으로 1:1 pair를 먼저 만든 뒤 10 Hz 제한을
적용했다. 원본 RGB 5,394장과 Depth 5,297장은 보존한다.

## 같은 기록 A/B

| 처리 | odom pose | quality=0 | 비율 | registration failure | graph pose/link | 2 cm 점군 |
|---|---:|---:|---:|---:|---:|---:|
| 개별 10 Hz 축소 | 890 | 318 | 35.73% | 643 | 19 / 18 | 15,031 |
| 정확 동기 baseline | 1,416 | 460 | 32.49% | 925 | 18 / 17 | 10,894 |
| 정확 동기 low-texture | 1,427 | 20 | 1.40% | 32 | 33 / 36 | 44,498 |
| inlier 5 실험, 제외 | 1,380 | 6 | 0.43% | 10 | 53 / 52 | 78,464 |

`quality=0`과 registration failure는 RTAB-Map 로그에서 계수했다. graph와 점군은
`rtabmap-export`의 Full Global Optimization 결과다. inlier 5 실험은 수치만 보면
좋지만, 렌더링에서 벽이 떨어져 나가고 전체 X 범위가 8.68 m까지 확대되어
오대응 위험이 확인됐다. 최종 선택은 inlier 10의 `low-texture`다.

## 채택한 설정

```bash
bash "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_vslam/scripts/run_rtabmap_docker.sh" \
  "$HOME/jdamr_data/vslam/rgbd_20260916T172638/bag_paired_10fps" \
  "$HOME/jdamr_data/vslam/rgbd_20260916T172638/rtabmap_paired_low_texture" \
  --profile low-texture
```

주요 차이는 최대 특징점 2,000개, 3×4 특징 분할, GFTT quality `0.0001`, 최소
inlier 10, 5회 연속 실패 후 odometry 재초기화다. odometry 전용 인자는
`odom_args`로 분리해 SLAM 노드에 전달되지 않게 했다.

## 3D 산출물

최종 출력 경로는
`$HOME/jdamr_data/vslam/rgbd_20260916T172638/portfolio_low_texture_global`이다.

| 파일 | 용도 | 크기 |
|---|---|---:|
| `jdamr_rgbd_portfolio_cloud.ply` | 1 cm voxel 컬러 점군, 582,544점 | 18,059,730 B |
| `jdamr_rgbd_portfolio_mesh.obj` | 텍스처 메시 | 19,260,011 B |
| `jdamr_rgbd_portfolio_mesh.jpg` | 4,096 px 텍스처 | 2,687,155 B |
| `jdamr_rgbd_portfolio_cloud_overview.png` | 등각·상단 검토 이미지 | 1,111,851 B |

산출물은 `--opt 0`으로 graph를 다시 최적화해 내보냈다. 데이터베이스에 마지막으로
저장된 단일 map의 pose만 사용하는 `--opt 2`는 재초기화 이후 구간 19 pose만
선택하므로 이번 비교와 포트폴리오 출력에는 사용하지 않는다.

## 해석 제한과 다음 게이트

1. 이번 데이터에는 `base_link -> camera_link` 실측 변환이 없다. wheel odometry나
   2D LiDAR를 visual odometry guess로 결합하지 않았다.
2. wheel odometry는 외부 ground truth가 아니며 카메라 장착 변환도 없으므로,
   ATE·RPE·거리 스케일·yaw 정확도를 주장하지 않는다.
3. 시작 후 약 90초가 정지 상태이고 실제 이동 구간에는 저텍스처 회색 벽 선회가
   포함됐다. 지도 밀도와 loop closure 평가에 불리한 입력이다.
4. 다음 실차 단계는 카메라의 x/y/z/roll/pitch/yaw를 실측해 TF를 추가한 뒤, 같은
   bag에서 camera-only와 wheel-guess를 A/B하는 것이다. 그 전에는 3D 지도 생성
   파이프라인과 실패 복구까지를 완료 범위로 본다.
