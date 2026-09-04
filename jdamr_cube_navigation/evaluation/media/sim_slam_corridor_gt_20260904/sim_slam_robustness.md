# 시뮬레이션 SLAM 정답 궤적 비교

> 판정: 두 센서 조건의 이동·회전 ATE/RPE가 모두 낮은 **Cartographer를 복도용 기본 백엔드로 유지한다.**

## 측정 결과

| 센서 조건 | 백엔드 | GT 경로 (m) | SLAM 경로 (m) | ATE RMS (m) | 정규화 ATE | 1초 RPE RMS (m) | yaw ATE RMS (rad) |
|---|---|---:|---:|---:|---:|---:|---:|
| 10 Hz / σ 0.01 m | Cartographer | 27.905 | 26.066 | 0.645 | 2.31% | 0.0366 | 0.0090 |
| 10 Hz / σ 0.01 m | SLAM Toolbox | 27.908 | 33.569 | 4.355 | 15.61% | 0.3075 | 0.0238 |
| 10 Hz / σ 0.05 m | Cartographer | 27.878 | 24.690 | 0.752 | 2.70% | 0.0601 | 0.0272 |
| 10 Hz / σ 0.05 m | SLAM Toolbox | 27.885 | 32.027 | 3.990 | 14.31% | 0.2857 | 0.0588 |

기준 조건에서 SLAM Toolbox의 이동 ATE는 Cartographer의 6.75배, 1초 이동 RPE는 8.41배였다. 노이즈 조건에서는 각각 5.31배와 4.75배였다.

## 센서 노이즈 민감도

LiDAR 가우시안 표준편차를 0.01m에서 0.05m로 높였을 때 Cartographer의 이동 ATE는 +16.6%, 1초 이동 RPE는 +64.5% 변했다. 센서 열화가 전역 오차보다 단기 상대 오차에 더 크게 나타났다.

SLAM Toolbox의 이동 ATE와 RPE는 각각 -8.4%와 -7.1%였지만, yaw ATE와 RPE는 각각 +147.0%와 +77.3% 악화됐다. 단일 시드에서 이동 오차가 줄어든 값을 노이즈 개선 효과로 해석하지 않는다.

## 실험 계약과 해석 범위

- Gazebo seed 42, 동일 world hash와 동일 왕복 명령을 사용했다.
- `/ground_truth_pose`는 Gazebo 모델 pose이며 wheel odom과 SLAM TF에서 독립적으로 기록했다.
- ATE는 축척을 바꾸지 않는 SE(2) 강체 정렬 후 계산했고, RPE는 1초 간격의 상대 운동 오차다.
- 실제 평면도에서는 긴 복도·반복 벽·양 끝 회차라는 정성적 구조만 가져왔다. 축척이나 실제 방 위치를 복원한 지도가 아니다.
- 이 결과는 한 시드·한 경로의 백엔드 선택 근거다. 반복 성공률이나 실차 절대 정확도로 확대하지 않는다.

## 논문과 구현 연결

- Kümmerle et al., [On Measuring the Accuracy of SLAM Algorithms](https://doi.org/10.1007/s10514-009-9155-6): 전역 원점보다 상대 pose 관계를 이용한 SLAM 비교의 필요성을 제시한다.
- Sturm et al., [A Benchmark for the Evaluation of RGB-D SLAM Systems](https://cvg.cit.tum.de/_media/spezial/bib/sturm12iros.pdf): timestamp 연계, 정답 궤적 정렬, ATE/RPE 계산 절차를 참고해 2D SE(2)로 적용했다.
- Hess et al., [Real-Time Loop Closure in 2D LIDAR SLAM](https://research.google/pubs/real-time-loop-closure-in-2d-lidar-slam/): Cartographer의 scan-to-submap 제약과 실시간 loop closure 설계 근거다.
- Macenski and Jambrecic, [SLAM Toolbox: SLAM for the dynamic world](https://joss.theoj.org/papers/10.21105/joss.02783): 비교 대상 백엔드의 ROS 2 pose-graph 및 lifelong mapping 설계를 설명한다.

원시 MCAP은 저장소 밖 `$HOME/jdamr_artifacts/`에 보존하고, 이 디렉터리의 JSON과 manifest가 입력 해시를 연결한다.
