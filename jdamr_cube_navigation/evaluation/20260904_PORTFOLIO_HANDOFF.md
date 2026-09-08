# SLAM 포트폴리오 고도화 인계

## 공개 서사의 중심

JD-AMR은 저장 지도와 Keepout을 사용해 복도 왕복 경로의 20개 목표를 Nav2 recovery 없이
완주했다. 제어 경로를 온보드 DDS로 격리한 뒤 LiDAR 최대 공백은 10.750초에서
0.112초로 줄었다. 성공 주행 MCAP을 두 SLAM 백엔드에 같은 조건으로 다시 재생해 복도
환경에 맞는 백엔드를 골랐다. 이어 실제 평면도의 긴 반복 복도를 단순화한 Gazebo
통제 환경에서 독립 ground truth ATE/RPE와 합성 LiDAR 노이즈 민감도를 측정해 이 선택을
다시 확인했다.

공개 페이지는 다음 순서로 쓴다.

1. 약한 무선 구간에서 센서와 TF가 함께 정체된 문제
2. 온보드 제어 그래프와 원격 시각화 그래프의 분리
3. 20/20 waypoint, recovery 0회, AMCL 경로 77.090m의 완주 결과
4. 가변 LiDAR 격자와 반복 복도 문제를 오프라인 원인분리
5. 동일한 정규화 MCAP으로 Cartographer와 튜닝한 SLAM Toolbox 비교
6. 독립 Gazebo ground truth에서 두 백엔드의 ATE/RPE 비교
7. LiDAR 노이즈 stress에서 단기 상대 오차가 변하는 정도 확인
8. 저장 지도를 유지한 채 횡단 보행자에 정지하고 같은 목표로 주행 재개

## 공개에 쓸 수치

| 항목 | 확인된 값 | 표현 범위 |
|---|---:|---|
| Nav2 waypoint | 20/20 | 단일 완주 기록 |
| Nav2 recovery | 0회 | 해당 주행 구간 |
| 사전 계획 경로 | 77.278m | Nav2 planning 결과 |
| 기록된 AMCL 경로 | 77.090m | 저장 지도 localization 기준 |
| 시작점 복귀 | 0.113m | AMCL 시작·종료 위치 차이 |
| LiDAR 최대 공백 | 10.750초에서 0.112초 | DDS 격리 전후 동일 계산식 |
| 원본 SLAM Toolbox 시작–종료 | 9.139m | 입력 계약 수정 전 기준선 |
| 튜닝한 SLAM Toolbox 시작–종료 | 1.032m | 정규화 MCAP 전체 재생 |
| Cartographer 시작–종료 | 1.026m | 같은 정규화 MCAP 전체 재생 |
| 튜닝한 SLAM Toolbox AMCL 기준 정렬 RMS | 0.817m | ground truth가 아닌 일관성 지표 |
| Cartographer AMCL 기준 정렬 RMS | 0.553m | ground truth가 아닌 일관성 지표 |
| LiDAR 주기 jitter p99 | 0.002164초 | 성공 주행 구간 header timestamp |
| 정적 IMU z축 robust sigma | 0.0018113rad/s | 주행 직전 60초 구간 |
| 시뮬레이션 기준 Cartographer ATE | 0.645m | seed 42, 27.905m GT 경로 |
| 시뮬레이션 기준 SLAM Toolbox ATE | 4.355m | 같은 world·경로·LiDAR |
| 시뮬레이션 노이즈 Cartographer ATE | 0.752m | LiDAR σ 0.01→0.05m 합성 stress |
| 시뮬레이션 노이즈 SLAM Toolbox ATE | 3.990m | 같은 stress 조건 |
| 돌발 장애물 목표 전송 / 취소 | 방향별 1회 / 0회 | Gazebo 온보드 통합 2회 |
| 돌발 장애물 접촉 | 양방향 모두 0회 | Gazebo Contact sensor |
| 팔 포함 보호 외곽 최소 여유 | 0.03960~0.04503m | 고정 수납 자세 시뮬레이션 |
| 보행자 횡단 / 물리 접촉 | 좌·우 0.93~0.94초 / 0회 | 독립된 3D Gazebo 실행 2회 |

입력 정규화와 복도 설정 후 SLAM Toolbox의 시작–종료 불일치는 9.139m에서 1.032m로
줄었다. 같은 정규화 입력에서 Cartographer의 시작–종료 1.026m는 수치상 유사했고,
AMCL 기준 정렬 RMS는 0.553m로 SLAM Toolbox의 0.817m보다 낮았다. 현재 복도용 기본
mapping backend는 Cartographer로 유지한다.

독립 ground truth 실험에서도 같은 결론이 나왔다. 기준 조건에서 SLAM Toolbox의 이동
ATE는 Cartographer의 6.75배, 1초 이동 RPE는 8.41배였다. 5배 LiDAR 노이즈에서는
각각 5.31배와 4.75배였다. Cartographer의 노이즈 전후 이동 ATE는 0.645m에서
0.752m(+16.6%), 1초 RPE는 0.0366m에서 0.0601m(+64.5%)로 변했다. 이 결과는 단일
시드의 센서 민감도 사례이며 반복 성공률로 표현하지 않는다.

## 공개 미디어

- `media/corridor_localdds_armed_20260904T152036/success_card.png`
- `media/corridor_localdds_armed_20260904T152036/route_evidence.png`
- `media/corridor_localdds_armed_20260904T152036/continuity_comparison.png`
- `media/corridor_localdds_armed_20260904T152036/corridor_roundtrip.mp4`
- `media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.png`
- `media/corridor_localdds_armed_20260904T152036/slam_toolbox_ablation.png`
- `media/corridor_localdds_armed_20260904T152036/sensor_profile.png`
- `media/corridor_localdds_armed_20260904T152036/timing_profile.png`
- `media/sim_slam_corridor_gt_20260904/sim_slam_robustness.png`
- `media/onboard_dynamic_obstacle_20260908/gazebo_dynamic_obstacle_highlight.mp4`
- `media/onboard_dynamic_obstacle_20260908/gazebo_dynamic_obstacle_right.mp4`
- `media/onboard_dynamic_obstacle_20260908/gazebo_pedestrian_bidirectional_reel.mp4`
- `media/onboard_dynamic_obstacle_20260908/gazebo_dynamic_obstacle_highlight.gif`
- `media/onboard_dynamic_obstacle_20260908/gazebo_obstacle_stop.png`
- `media/onboard_dynamic_obstacle_20260908/gazebo_goal_arrival.png`

## 공개 문구에서 제외할 내용

내부 디버깅에 필요했던 프로세스 ID, 중간 실행 횟수, 종료 신호 수정 과정, 과거 계측기
오분류는 공개 본문에서 뺀다. 가변 scan 길이 경고 횟수를 나열하는 대신, 센서 출력
계약을 정규화하고 거리 범위·키프레임·루프 클로징을 한 변수씩 분리했다는 문제 해결
과정과 개선 결과를 보여준다.

한계는 짧고 정확하게 남긴다. AMCL은 외부 ground truth가 아니므로 ATE/RPE나 절대
정확도를 주장하지 않는다. 한 번의 완주로 반복 성공률을 만들지 않으며, 카메라 데이터가
없는 현재 결과를 Visual SLAM으로 부르지 않는다. 시뮬레이션 ATE/RPE는 실제 평면도의
축척 복원이 아니라 긴 반복 복도의 정성적 구조를 단순화한 한 seed 결과라고 밝힌다.
돌발 장애물 영상도 실제 Gazebo 카메라 센서 프레임이지만 실차 제동거리나 사람 안전의
증거는 아니다.

## 근거 파일

- `20260904_CORRIDOR_LOCALDDS_SUCCESS.md`
- `media/corridor_localdds_armed_20260904T152036/metrics.yaml`
- `media/corridor_localdds_armed_20260904T152036/sensor_profile.md`
- `media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.md`
- `20260904_SLAM_TOOLBOX_ROOT_CAUSE.md`
- `media/corridor_localdds_armed_20260904T152036/slam_toolbox_ablation.md`
- `media/corridor_localdds_armed_20260904T152036/media_manifest.yaml`
- `media/sim_slam_corridor_gt_20260904/sim_slam_robustness.md`
- `media/sim_slam_corridor_gt_20260904/media_manifest.json`
- `20260908_DYNAMIC_OBSTACLE_READINESS.md`
- `media/onboard_dynamic_obstacle_20260908/portfolio_media_manifest.json`

포트폴리오 세션은 이 문서의 공개 수치와 미디어만 먼저 사용한다. 자세한 디버깅 기록은
면접에서 원인 분석 과정을 질문받았을 때 근거로 연다.
