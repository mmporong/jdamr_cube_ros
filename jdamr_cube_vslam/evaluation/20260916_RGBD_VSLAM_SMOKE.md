# Astra S RGB-D Visual SLAM 정적 연결 검증

## 판정

- Astra S의 RGB·Depth 동시 기록: **PASS**
- Depth를 컬러 광학 프레임에 정렬: **PASS**
- RTAB-Map RGB-D 동기화·특징 추적·visual odometry 출력: **PASS**
- 미터 단위 컬러 point cloud 단일 프레임 생성: **PASS**
- 여러 위치를 합친 3D SLAM 지도: **미검증**
- 거리·yaw 절대 정확도: **미검증**

정지 상태 자료로 지도와 정확도를 주장하지 않는다. 이번 시행은 센서부터 RTAB-Map까지
연결되고 실제 Depth가 3D 좌표로 투영되는지만 검증했다.

## 실측 증거

실행 데이터는 저장소 밖 `$HOME/jdamr_data/vslam_smoke/rgbd_20260916T162426`에 둔다.

| 항목 | 결과 |
|---|---:|
| bag 유효 길이 | 5.255149314 s |
| RGB frame | 76 |
| Depth frame | 76 |
| RGB camera_info | 76 |
| Depth camera_info | 77 |
| wheel odometry | 263 |
| 2D LiDAR scan | 51 |
| 정적 PLY point | 6,772 |
| 선택 frame RGB↔Depth timestamp 차이 | 0.0186214447 s |
| 선택 frame 유효 Depth 범위 | 0.684~4.961 m |
| 선택 frame Depth 중앙값 | 3.6410002708 m |
| RTAB-Map visual odometry pose | 54 |
| RTAB-Map DB node | 5 |
| RTAB-Map DB odometry length | 0.024212 m |
| graph link | 0 |

정지 중 계산된 0.024212 m는 이동 거리가 아니라 visual odometry의 정적 drift다. 실행
결과이므로 코드 상수로 사용하지 않는다.

## 확인된 제한

1. 정지 데이터에는 graph link가 없어 RTAB-Map의 assembled cloud export가 거부됐다.
   이는 입력 실패가 아니라 이동 baseline이 없는 상태를 fail-closed로 처리한 결과다.
2. RGB와 Depth header 차이는 선택 frame에서 약 18.6 ms였다. offline mapping은 30 ms
   이내 approximate sync만 허용한다.
3. `base_link -> camera_link` 실측 extrinsic이 아직 없으므로 camera-only RGB-D SLAM만
   허용한다. wheel odometry·LiDAR 융합은 `config/camera_mount.yaml`이 채워지기 전까지
   차단한다.
4. wheel odometry는 비교 기준일 뿐 ground truth가 아니다. 거리와 yaw 정확도는 줄자와
   각도 기준으로 만든 통제 구간에서만 확정한다.

## 파이 배포 명령 재검증

최종 설치본의 `ros2 run jdamr_cube_vslam capture_rgbd_bag.sh --duration 2`를 다시
실행했다. rosbag 초기 구독 구간을 포함해 4.499705721초가 저장됐고 RGB 125장, Depth
125장, wheel odometry 140건, LiDAR scan 28건, `/tf_static` 1건이 들어갔다. 종료 뒤
카메라와 base systemd 서비스는 다시 `active`였다. 이 짧은 부하 시험에서 transport
loss 12건이 보고됐으므로 장거리 주행에서는 손실률을 별도 계산한다. 재검증 bag은
동일 장면 중복 데이터라 수치만 남기고 삭제했다.

## 다음 실차 데이터 계약

1. 출발점과 종료점이 같은 짧은 폐루프를 저속으로 한 번 주행한다.
2. 별도 직선 구간은 바닥 기준점 사이 거리를 줄자로 기록한다.
3. 회전 구간은 직각 벽 또는 각도 지그에 맞춘 시작·종료 heading을 기록한다.
4. 같은 bag으로 camera-only baseline을 먼저 계산한다.
5. camera mount extrinsic 측정 후 wheel odometry guess와 2D LiDAR graph를 각각 추가해
   A/B 비교한다.
6. 거리 scale error, translation ATE/RPE, yaw RMSE/P95, 폐루프 복귀 오차를 보고한다.

임시 AprilTag를 평가 기준으로 쓸 수 있지만 runtime 필수 구성에는 넣지 않는다. 태그를
사용하면 위치·자세 ground truth 확보용이라는 사실과 태그 실측 오차를 별도로 기록한다.
