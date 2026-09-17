# RGB-D Visual SLAM 다음 실행 계획

## 결론

이 작업은 의미가 있다. 단, RGB-D 프레임을 휠 오도메트리 자세에 단순히 붙여 만든 PLY가
아니라 **저텍스처 환경에서 Visual SLAM이 실패하는 조건을 측정하고, 휠 오도메트리 보조와
LiDAR 제약으로 그래프의 연속성과 재방문 정합을 개선하는 프로젝트**로 진행해야 한다.

현재 결과는 Visual SLAM 완성이 아니다. 카메라 입력·깊이 역투영·RTAB-Map visual
odometry 실행은 확인했지만, 실제 P턴에서는 추적 재초기화와 분리된 점군이 발생했다.
90도 회전 점군은 graph optimization과 loop closure가 없는 odometry-seeded fusion이므로
SLAM 지도 증거로 사용하지 않는다.

## 현재 증거

| 항목 | 결과 | 판정 |
| --- | --- | --- |
| RGB와 registered Depth 기록 | 정상 | PASS |
| 정적 bag RTAB-Map 실행 | visual pose와 DB 생성 | 연결 확인만 PASS |
| 저텍스처 P턴 camera-only | quality=0 비율 감소 | 재초기화 2회, 지도 연속성 FAIL |
| inlier 5 완화 | 추적 손실 수치 감소 | 오대응과 점군 찢어짐으로 REJECT |
| wheel-odom 점군 융합 | 3D 역투영·pose 적용 확인 | SLAM 아님 |
| `base_link → camera_link` | bag과 설정에 없음 | 센서 융합 BLOCKED |

기존 P턴 bag은 다음 토픽을 포함한다.

- RGB 1,482장, Depth 1,482장
- `/odom` 9,008개: `odom → base_footprint`
- `/scan` 1,743개
- `/tf_static`: `base_footprint → base_link`, `base_link → laser_link`
- 카메라 내부 TF: `camera_link → camera_color_optical_frame` 등

기록에는 `base_link → camera_link`가 없다. 따라서 `odom_guess_frame_id=base_footprint`를
활성화하기 전에 현재 임시 장착의 카메라 외부 파라미터를 실측하고 정적 TF로 공급해야 한다.

## 다음 실행 순서

### 1. 카메라 외부 파라미터 실측

현재 장착 상태에서 다음 값을 `config/camera_mount.yaml`에 기록한다.

- `base_link` 원점에서 카메라 `camera_link` 원점까지의 `x`, `y`, `z` [m]
- 차체 전방 기준 카메라의 `roll`, `pitch`, `yaw` [rad]
- 측정일, 측정 방법, 이 값이 유효한 브래킷 상태

카메라 높이나 방향을 바꾸면 이 값은 폐기하고 다시 측정한다. 사진으로 추정한 값은 센서
융합 결과의 근거로 사용하지 않는다.

### 2. 동일 bag A/B

동일 입력을 사용해 다음을 비교한다.

1. `camera-only`: 현재 `low-texture` 프로파일
2. `wheel-guess`: 휠 odometry의 `base_footprint` 이동량을 visual odometry의 초기 추정으로 사용
3. `wheel-odom`: visual odometry를 끄고 `/odom`을 외부 odometry로 사용한 graph baseline
4. `RGB-D + 2D LiDAR`: 위 두 단계가 안정된 뒤 `/scan` 제약 추가

변수는 한 번에 하나만 바꾼다. camera-only와 wheel-guess의 차이는 초기 추정 사용 여부,
wheel-guess와 LiDAR 융합의 차이는 scan 제약 사용 여부로 제한한다.

### 3. 새 폐루프 bag 1개

기존 P턴은 출발점 재방문이 없어 loop closure 검증에 불충분하다. 새 기록은 다음 조건을
지킨다.

- 정지 상태 5초로 시작
- 특징이 있는 물체와 저텍스처 벽을 모두 통과
- 출발 위치와 방향으로 복귀
- 복귀 후 5초 이상 정지한 뒤 기록 종료
- RGB, Depth, camera_info, `/odom`, `/scan`, `/tf`, `/tf_static` 동시 기록
- 녹화 종료가 회전이나 복귀보다 먼저 끝나지 않음

실차 움직임은 별도 명시적 실행 요청이 있을 때만 수행한다.

## 통과 기준

다음 항목을 모두 기록하며 pose 개수만으로 성공을 판정하지 않는다.

- assembled cloud export 성공
- visual odometry quality=0 비율과 registration failure 횟수
- odometry reset 또는 map 재초기화 횟수
- graph pose와 neighbor/proximity/loop link 수
- 시작·종료 pose의 위치 및 yaw 차이
- 휠 odometry 기준 SE(2) 정렬 오차와, 가능하면 외부 실측 기준 오차
- 점군이 여러 조각으로 분리되거나 벽이 중복되는지 시각 판정
- 처리 시간, peak memory, 입력 프레임 수

성공 조건은 폐루프 전체가 하나의 연속된 graph와 점군으로 유지되고, 시작점 재방문에서
올바른 loop 또는 proximity 제약이 생기며, camera-only보다 wheel-guess 또는 LiDAR 융합의
드리프트와 재초기화가 줄어드는 것이다.

## 포트폴리오에서의 위치

주요 성과는 2D LiDAR SLAM·Nav2 기반의 실차 자율주행으로 유지한다. RGB-D Visual SLAM은
다음 역량을 보여주는 고도화 항목으로 둔다.

- 저텍스처 환경의 feature tracking 실패 분석
- 카메라·베이스·라이다 외부 파라미터 관리
- RGB-D, wheel odometry, 2D LiDAR의 시간·좌표계 정합
- 동일 bag 기반 A/B 실험과 graph 품질 평가
- Raspberry Pi는 동기 데이터를 수집하고 노트북은 무거운 3D 최적화를 수행하는 자원 분리

휠 보조 후에도 graph와 점군 연속성이 확보되지 않으면, 실패 분석까지 남기고 Visual SLAM을
주요 결과로 과장하지 않는다. 이 경우 Depth 카메라는 매니퓰레이션용 국소 인지로 전환한다.

