# RGB-D 폐루프 자동 주행과 3D 매핑 검증

## 판정

2026-09-17 실차에서 0.7m 전진·복귀와 제자리 360° 회전을 자동 수행하고 RGB, Depth,
wheel odometry, 2D LiDAR, TF를 함께 기록했다. 외부 wheel odometry를 직접 사용한 RGB-D
매핑은 한 개의 map ID를 유지하며 25,086점의 점군을 생성했다. 따라서 센서 수집부터 3D
점군 export까지의 파이프라인은 동작한다.

다만 주행 범위가 편도 0.7m에 불과해 결과는 박스·주변 물체·전방 벽을 담은 국소 3D
재구성이다. 방 전체의 완성된 3D 지도나 camera-only Visual SLAM 성공으로 표현하지 않는다.

## 실차 동작

| 항목 | 결과 |
| --- | ---: |
| 전진·복귀 누적 거리 | 1.44m |
| 복귀 odometry 위치 잔차 | 0.0cm |
| 제자리 회전 | 365.9° |
| 회전 후 방향 잔차 | +5.9° |
| 카메라 렌즈 중심 높이 | 0.215m |
| 전방 박스 초기 거리 | 약 1.5m |

자동 동작 로그는 실행 데이터 디렉터리의 `probe_vslam_back_0917_1353.json`과
`probe_vslam_spin_0917_1354.json`에 있다.

## 기록 무결성

전체 원본 bag은 Raspberry Pi의
`$HOME/jdamr_data/vslam/rgbd_20260917T135149/bag`에 보존했다. PC에는 처리용으로 완성된
`$HOME/jdamr_data/vslam/rgbd_20260917T135149/bag_paired_10fps`를 보존했다.

| 토픽 | 메시지 수 |
| --- | ---: |
| RGB image | 2,771 |
| Depth image | 2,712 |
| wheel odometry | 4,716 |
| LiDAR scan | 910 |
| dynamic TF | 2,722 |
| static TF | 1 |

10Hz 처리 bag은 778개의 RGB-D 페어를 포함한다. 센서 header 기준 시간차는 평균
11.989ms, p95 22.261ms, 최대 29.899ms다.

첫 180초 정적 기록은 RGB-D만 저장되고 `/odom`과 `/scan`이 0건이었다. 같은 환경에서
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 기록은 wheel·scan을 받지 못했지만 `SUBNET`
기록은 5.34초 동안 odometry 169건과 scan 33건을 받았다. 원인을 DDS 구현 내부 동작으로
단정하지 않고 관찰된 데이터 전달 차이로 기록한다. capture 스크립트는 `SUBNET`을 사용하며,
RGB·Depth뿐 아니라 `/odom`과 `/scan` 실제 샘플이 없으면 본 기록을 거부하도록 변경했다.

## 동일 bag 세 설정 비교

| 설정 | DB map ID | DB node | export pose | 점군 | 결과 |
| --- | ---: | ---: | ---: | ---: | --- |
| camera-only Visual SLAM | 2 | 142 | 14 | 3,713점 | 재초기화로 분리, 희소 점군 |
| wheel-guess Visual SLAM | 2 | 111 | 13 | 23,042점 | 스케일 개선, 재초기화 잔존 |
| external wheel odom RGB-D mapping | 1 | 108 | 14 | 25,086점 | 단일 지도 유지, 국소 재구성 성공 |

camera-only와 wheel-guess는 wheel odometry를 reference로만 사용해 SE(2) 정렬 평가했다.

| 지표 | camera-only | wheel-guess |
| --- | ---: | ---: |
| 시간 정렬 pose | 585 | 678 |
| 이동 ATE RMSE | 0.0229m | 0.0392m |
| yaw RMSE | 2.750° | 5.341° |
| 거리 스케일 오차 | 19.460% | 0.010% |
| 폐루프 거리 차이 | 0.0494m | 0.0546m |

wheel-guess는 거리 스케일을 wheel odometry와 일치시켰지만 visual odometry reset이 남았다.
external wheel odom 결과는 wheel odometry를 직접 입력으로 사용하므로 같은 정확도 표로
자기 비교하지 않는다. 이 모드는 Visual SLAM이 아니라 odometry-seeded RGB-D mapping이다.

## 처리 안정화

- wheel 연동 offline replay는 0.5배속으로 실행하고 TF 대기를 1.5초로 늘렸다.
- filtered point-cloud export가 NaN point의 PCL radius filter에서 중단되면 동일 DB를
  radius-noise filter 없이 다시 export한다.
- 외부 wheel odometry 모드는 `base_link`를 robot frame으로 사용하고 실측
  `base_link → camera_link` 정적 TF를 필수로 한다.

## 산출물

- 3설정 비교 이미지:
  `$HOME/jdamr_data/vslam/rgbd_20260917T135149/map_comparison_3way.png`
- camera-only:
  `$HOME/jdamr_data/vslam/rgbd_20260917T135149/rtabmap_camera_only`
- wheel-guess:
  `$HOME/jdamr_data/vslam/rgbd_20260917T135149/rtabmap_wheel_assisted_slow`
- external wheel odom:
  `$HOME/jdamr_data/vslam/rgbd_20260917T135149/rtabmap_external_odom_v2`

## 다음 성공 게이트

방 전체 3D 지도를 주장하려면 0.7m 왕복 대신 방 둘레를 도는 translational closed loop를
기록한다. 카메라 높이와 각도는 유지하고 다음 조건을 확인한다.

- external wheel odom 기준 단일 map ID 유지
- 출발 위치 재방문을 포함한 한 개의 연속 점군
- 벽의 중복 평면과 부채꼴 찢어짐 없음
- camera-only와 wheel-guess의 visual odometry reset 0회
- point-cloud filter가 NaN 없이 정상 종료
