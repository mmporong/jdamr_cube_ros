# 제한 복귀 회전의 동기 녹화와 카메라 기반 국소 3D 재구성

## 결과

2026-09-21 사용자의 계속 진행 요청에 따라, 이전 관측에서 돌아간 차체를 시계방향으로 한 번 복귀시켰다. 사용자에게 종이나 기체를 다시 옮기도록 요청하지 않았다. 모션 결과는 -16.414°였고, 저장된 bag의 시작/끝 odom 차이는 -16.420°였다. 최종 odom 방향은 기존 초기 방향 기준 +0.028°이며 정지 후 선속도·각속도와 마지막 명령은 모두 0이었다. 오도메트리 수치이므로 외부 기준의 복귀 정확도로 해석하지 않는다.

먼저 바퀴 기반 자세를 적용해 7개 RGB-D 시점의 컬러 점군을 생성했다. 이 기준선은 visual SLAM이 아니다. 이어 같은 저장 자료를 RTAB-Map의 카메라 기반 위치 추정으로 처리해 연결된 keyframe 3개와 18,711점의 국소 3D 재구성을 얻었다. 아래에서 두 결과를 구분한다. 높이 및 카메라 내부 교정 검증은 미완료라 depth navigation 승인 상태는 바꾸지 않았다.

## 앞선 녹화 실패의 수정

앞선 실행은 녹화 시간 제한이 회전보다 먼저 끝나 `/cmd_vel`이 bag에 없었다. 이번에는 하나의 감독 스크립트에서 순서를 묶었다.

1. recorder를 시작하고 RGB·depth·CameraInfo·scan·odom·TF·입출력 명령·footprint의 구독 로그를 확인한다.
2. 모든 필수 구독 확인 시점부터 3초의 정지 구간을 기록한다. 준비 실패나 정지 구간 부족이면 회전하지 않는다.
3. `cmd_vel_smoothed → Collision Monitor → cmd_vel` 경로로 최대 3초, -0.12 rad/s, 목표 15° 회전을 실행한다.
4. 모든 동작 종료 경로에서 2초간 영점 명령을 발행한 다음 recorder를 SIGINT로 닫고 로그와 결과를 남긴다.
5. 저장된 명령의 비영점 구간에 영상과 센서 메시지가 실제로 들어 있는지 파일을 다시 읽어 확인한다.

기존 LiDAR 회전 StopZone·SlowdownZone·FootprintApproach는 유지했다. 영점 출력 확인을 위해 임시 monitor의 `stop_pub_timeout`만 2초에서 30초로 늘렸다. 이는 정지 명령의 발행 지속 시간이며 센서 지연 허용이나 충돌 경계를 완화한 것이 아니다. odom·scan의 stamp 및 수신 지연 0.3초, monitor 출력 지연 0.4초, 사방 관측 및 보수적 최소거리 0.46 m 제한을 유지했다. 실행 전 `/cmd_vel` publisher는 Collision Monitor 하나였다. 종료 후 이 임시 monitor는 중단했고 기존 base·카메라·관측 노드·대시보드는 유지했다.

## 실제 기록 검증

| 항목 | 값 |
| --- | ---: |
| bag의 비영점 명령 | 46개 / 첫·마지막 간격 2.701초 |
| 비영점 구간 RGB / depth | 각각 23개 — 10fps 처리본 기준 |
| 같은 구간 scan / odom | 26 / 135개 |
| 센서별 녹화 선행 구간 | 3.14~4.73초 |
| 센서별 녹화 후행 구간 | 1.98~2.18초 |
| 원본 RGB / depth / scan / odom | 255 / 249 / 93 / 402개 |
| 원본 `/cmd_vel` / monitor 상태 | 135 / 2개 |
| 처리본 RGB-D pair | 74쌍 |
| RGB-depth 시간차 평균 / p95 / 최대 | 12.30 / 20.73 / 25.34 ms |

`recorder.log`는 transport loss 11개를 보고했다. 어느 토픽의 손실인지는 이 로그만으로 확정하지 않았으며 무손실 데이터라고 표기하지 않는다. 위 검증은 실제 수신된 자료가 움직임 전·중·후를 포함한다는 의미다. 센서 메시지의 header stamp와 header가 없는 명령의 bag 수신 시각을 비교했다.

색상 CameraInfo의 K/P는 유한했고, registered depth의 K는 NaN이지만 P는 유한했다. 기존 융합 도구는 색상 K를 depth 해상도로 축소한 내부 파라미터를 사용한다. 해당 focal 값은 depth P와 일치하지만 이것이 공장 교정이나 RGB-depth 등록의 정확도를 보증하지는 않는다.

## 점군 생성과 반복 관측

| 항목 | 값 |
| --- | ---: |
| 선택 시점 / yaw 간격 | 7개 / 약 3° |
| 융합에 들어간 회전 범위 | -16.414° |
| 유효 거리 범위 | 0.4~4.0 m |
| 투영점 / 1cm voxel 이후 점 | 297,054 / 184,270점 |
| 사용한 명목 카메라 위치 | x=0.065, y=0, z=0.215 m |
| 사용한 명목 roll/pitch/yaw | 모두 0° |

`capture_complete=true`는 요청한 15° 구간의 관측 범위를 충족했다는 뜻이다. 방 전체 지도를 완성했다는 뜻은 아니다. 점군 Z 범위에는 -0.403 m도 포함되므로 명목 높이를 실제 바닥 기준 높이로 승인하지 않는다. 보기 좋게 만들기 위해 바닥을 생성하거나 음수 높이를 잘라내지 않았다.

초기 차체 좌표에서 눈으로 선정한 박스 내부 범위(x=0.75~1.05, y=-0.53~-0.24, z=-0.03~0.10 m)를 각 시점에 동일하게 적용했다. 첫 프레임에서 맞춘 평면에 대한 잔차 p95는 기준 프레임 자체가 2.19 mm, 나머지 각도 프레임이 3.02~4.19 mm였다. 이는 해당 작은 면에서의 반복 관측 일관성이며, **절대 위치 정확도·주차 오차·센서 교정 완료를 의미하지 않는다.** 자동 박스 인식도 아니다.

## 산출물 및 재현

Pi 원본은 `$HOME/jdamr_data/vslam/recorded_return_20260921/bag`에 남겼다. Linux PC에는 비영상 토픽을 모두 유지한 RGB-D 10fps 처리본, 실행 결과, recorder 로그와 점군을 복사했다. 원본 전체의 PC 백업 완료로 표기하지 않는다.

```text
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return.py
$HOME/jdamr_artifacts/depth_obstacles_20260921/guarded_turn.py
$HOME/jdamr_artifacts/depth_obstacles_20260921/verify_recorded_return.py
$HOME/jdamr_artifacts/depth_obstacles_20260921/render_recorded_return.py
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/motion.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/recorder.log
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/paired_10fps/
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/recording_verification.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/box_repeatability.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/recorded_return_preview.png
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/fusion_nominal/
```

`recorded_return.py`는 이번 실행용 아티팩트다. Pi에서는 `guarded_turn_20260921.py`라는 이름의 검증된 모듈을 import했다. Linux PC의 보존 이름은 `guarded_turn.py`이며 두 파일의 SHA256은 `beeba6b72cd7e791b81d4338fa3567c8e194e69b0548624bd864b30460de8e7c`로 같았다. 제품용 실행 명령이나 재실행 승인을 뜻하지 않는다.

오프라인 처리에는 기존 `downsample_rgbd_bag`, `fuse_rgbd_odom`을 재사용했다. 해당 도구의 관련 테스트 11개가 통과했다. 아래 명령은 이미 복사된 처리본만 읽고 실물 명령을 발행하지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_rgbd_ws/install/setup.bash"
python3 "$HOME/jdamr_artifacts/depth_obstacles_20260921/verify_recorded_return.py"
ros2 run jdamr_cube_vslam fuse_rgbd_odom \
  --input "$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/paired_10fps" \
  --output-dir "$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/fusion_nominal" \
  --yaw-step-deg 3 --pixel-step 1 --minimum-depth-m 0.4 --maximum-depth-m 4.0 \
  --voxel-m 0.01 --expected-yaw-deg 15 --camera-x-m 0.065 --camera-z-m 0.215
python3 "$HOME/jdamr_artifacts/depth_obstacles_20260921/render_recorded_return.py"
```

## 카메라 기반 RTAB-Map 재처리

실물 회전을 추가하지 않고 동일 처리본을 PC의 Docker에서 재생했다. 로봇 domain 12와 분리된 domain 72를 사용했으며, 카메라 전용 입력에서는 주행 명령과 LiDAR 토픽을 제외했다. 바퀴 `/odom` 메시지는 비교용 CSV에만 기록하고 시각 추정이나 매핑 입력으로 사용하지 않았다.

### 실패 원인과 수정 근거

1. 기본 프로파일이 빈 `odom_args:=`를 전달해 ROS launch 단계에서 실패했다. 값이 있는 경우에만 인자를 전달하도록 수정했다.
2. 기존 launch는 visual odometry와 mapper 모두 `/odom`을 사용하면서 CSV는 `/rtabmap/odom`을 구독했다. 바퀴 입력과 시각 출력이 같은 토픽으로 연결될 수 있어, 시각 출력과 mapper 입력을 `/rtabmap/odom`으로 분리했다.
3. 기록된 `odom → base → camera_link`와 시각 `vslam_odom → camera_link`가 같은 자식에 연결됐다. 시각 TF를 끈 중간 시도에서도 mapper에 필요한 연결이 사라져 6개 DB 노드에 연결 0개가 남았다. 새 `prepare_visual_replay.py`는 원본을 보존하고 카메라 내부 TF만 남긴 별도 bag을 만든다. 시각 추정이 `camera_link`의 유일한 TF 부모를 제공하도록 했다.
4. 기본 export의 depth decimation 4와 반경 5cm/이웃 5개 필터 조합에서 점군이 매우 성겼다. 같은 baseline DB를 decimation 2로 내보내자 1,368점에서 18,497점으로 늘었다. 형상을 새로 생성하지 않고 원래 depth 표본을 더 사용한 결과다. export 기본값을 2로 변경했다.

RTAB-Map의 `CoreWrapper::odomUpdate`는 영상 시각의 odometry frame과 처리 frame 사이 변환을 조회한다. 따라서 토픽 이름만 바꾸고 TF 연결을 끊는 수정으로는 충분하지 않았다. 근거: [RTAB-Map ROS 2 CoreWrapper 소스](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_slam/src/CoreWrapper.cpp).

`--odom-guess-frame` 혼합 모드는 독립적인 바퀴 참조/시각 TF 트리 구성을 아직 검증하지 못했으므로 실행 전에 명시적으로 차단했다. 실패하는 구성을 지원한다고 표기하지 않는다. 외부 odometry 모드의 인자 전달 회귀 테스트는 통과했지만 이번 자료로 해당 모드를 다시 실측한 것은 아니다.

### 동일 기록의 오프라인 비교

| 실행 | 유효 시각 자세 | 무효 시각 메시지 | 최대 자세 시각 간격 | 바퀴 대비 yaw 차이 RMSE | 내보낸 연결 keyframe |
| --- | ---: | ---: | ---: | ---: | ---: |
| baseline 1배속 | 44 | 2 | 0.612초 | 0.953° | 3 |
| low-texture 1배속 | 42 | 0 | 0.576초 | 1.019° | 2 |
| 최종 baseline 0.5배속 | 54 | 0 | 0.267초 | 1.036° | 3 |

각 실행의 시각 자세와 바퀴 yaw를 공통 timestamp 구간에서 보간하고 첫 yaw를 맞춰 비교했다. 바퀴는 외부 ground truth가 아니므로 이 수치를 절대 각도 정확도로 해석하지 않는다. 공통 구간도 각각 6.693/6.236/6.477초로 다르다. 한 번씩의 재생 비교이며 반복 실험의 통계적 성능 차이는 아니다.

최종 실행은 바퀴 기준 -16.420° 회전에 대해 카메라 -14.903°, 끝점 차이 +1.517°였다. 재생 속도를 낮춘 실행에서 무효 자세와 최대 간격은 줄었지만 yaw 일치도가 더 좋아진 것은 아니다. 저텍스처 프로파일도 모든 지표를 개선하지 않아 기본값으로 채택하지 않았다.

최종 DB에는 11개 노드와 4개 방향성 Link 행이 있고, export가 선택한 연결 그래프는 **3개 pose / 2개 link**다. 최종 2cm voxel 점군은 **18,711점**이다. 전체 DB 노드가 모두 연결됐다고 표기하지 않는다. 로그의 `not part of the same tree` 경고는 0회지만, 시작 시 camera-local 정적 TF가 도착하기 전 `camera_link` lookup 경고 4회가 남았다. 무효 메시지 0개는 모든 영상 프레임이 처리되거나 센서 손실이 없었다는 뜻이 아니다.

기존 설정인 `Reg/Force3DoF=true`를 유지했다. 따라서 x/y/yaw 평면 운동 모델을 적용한 RGB-D 국소 재구성이며 자유로운 6DoF 자세 추정 검증은 아니다. 영상에서도 벽·의자 표면은 확인되지만 바닥과 가려진 면은 비어 있다. 좌표 원점은 초기 카메라이며 바닥 기준 높이로 교정하지 않았다. 방 전체 주행, 재방문 loop closure, 재위치추정, 정밀 접근은 이번 결과의 완료 범위가 아니다.

### 최종 산출물과 검증

```text
$HOME/jdamr_artifacts/depth_obstacles_20260921/compare_visual_odom.py
$HOME/jdamr_artifacts/depth_obstacles_20260921/render_visual_map.py
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/visual_odom_comparison.json
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/visual_odom_comparison.png
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/visual_map_preview.png
$HOME/jdamr_artifacts/depth_obstacles_20260921/recorded_return_20260921/rtabmap_visual_final_v2/
  jdamr_rgbd.db
  jdamr_rgbd_cloud.ply
  jdamr_rgbd_poses.txt
  visual_trajectory.csv
  reference_trajectory.csv
  visual_input/visual_replay_summary.json
  rtabmap.log
  rtabmap_export.log
```

최종 Docker 실행은 exit 0으로 종료했다. orchestration·TF 선별·MCAP roundtrip·기존 융합/동기화 관련 테스트 32개가 통과했고, 이후 패키지 디렉터리에서 실행한 전체 테스트도 flake8·pep257를 포함해 **40개 통과**했다. 셸 문법 검사와 `git diff --check`도 통과했다. MCAP 테스트는 원본 RGB 바이트 및 시각 보존, 장착 TF 제거, 주행 명령 토픽 제외를 확인한다. 저장소 루트에서 실행한 lint는 다른 패키지까지 검사해 실패했으므로 패키지 경계에서 다시 검증했다. 이 결과는 저장소 전체 lint 통과를 뜻하지 않는다.

사용자는 작업 마무리 중 충전기를 연결하고 기체를 다른 위치로 옮겼다고 알렸다. 위 지도와 박스 ROI는 이동 전 기록의 좌표이며 새 위치의 정밀 접근에 재사용하지 않는다. 충전 상태에서 추가 주행하지 않았다.

다음 과제는 정적 TF를 영상보다 먼저 공급하는 재생 초기화, 미검증 높이·등록 오차 분리, 더 넓은 구간의 연결성과 재방문 검증이다. 같은 종이를 반복 재배치하거나 회전을 반복하는 방식으로 교정 완료를 주장하지 않는다.
