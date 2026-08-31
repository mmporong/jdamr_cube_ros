# PRD — JD-AMR Robust SLAM·자율탐색 포트폴리오

- 상태: 실행 전 계획, 기존 SO-101/JD-AMR 계획 대조 완료
- 기준일: 2026-08-31
- 예상 투자: 4~6주, 60~90시간
- 정본 저장소: `/home/lim/jdamr_cube_ws/src/jdamr_cube_ros`
- SO-101 연계 저장소: `/home/lim/so101-mobile-manipulation`
- 범위: 계획·검증 설계만. 코드 수정·실차 이동·push는 하지 않고 이 계획 문서만 커밋한다.

## 1. 목표 결과

2D LiDAR 기반 실내 자율주행에서 센서 시간축 결함을 찾아 고친 기존 경험을 출발점으로 삼아, 동일 입력에서 SLAM 백엔드·센서 노이즈 처리·저장 지도 위치추정·frontier 탐색을 반복 가능한 실험으로 비교하고 선택 근거를 남긴다.

최종 포트폴리오는 단순히 “지도를 만들었다”가 아니라 다음 질문에 재현 가능한 데이터로 답해야 한다.

1. 센서 입력이 정상이라는 것을 어떻게 검증했는가.
2. Cartographer와 SLAM Toolbox 중 무엇을 어떤 지표로 선택했는가.
3. 실차에 참값이 없을 때 무엇을 재고, 시뮬레이션 참값이 있을 때 무엇을 재는가.
4. 센서 노이즈·오도메트리·강건 파라미터가 결과에 어떤 영향을 주는가.
5. 저장 지도 위치추정과 미지 환경 탐색을 어떻게 분리해 검증했는가.
6. 결과가 SO-101의 `주행 → 정지 → 팔 동작` 전환을 어떻게 안전하게 뒷받침하는가.
7. 시뮬레이션에서 고른 설정이 실차로 넘어갈 때 어떤 domain gap이 생기며, 이를 어떤 승격 게이트로 통제했는가.

## 2. 기존 계획 확인 결과

### 2.1 SO-101에 이미 있던 계획

기존 SO-101 PRD는 폐기 대상이 아니다. 다만 SLAM 구현 정본이 아니라 SLAM/Nav2 결과를 소비하는 모바일 매니퓰레이션 통합 계획이다.

- 문서상 상태는 “구현 시작 전”이다 (`/home/lim/so101-mobile-manipulation/.omx/plans/prd-so101-autonomy-pick-place.md:1-10`). 현재 작업 트리에는 별도 `mobile_mission.py`와 미션 상태 열거가 존재하므로 (`/home/lim/so101-mobile-manipulation/mobile_mission.py:36-41`), 나중에 통합 실행을 시작하기 전 문서 상태와 실제 구현을 다시 대조해야 한다.
- Phase 0A는 고정 작업대에서 팔의 stationary pick/place를 검증하는 별도 트랙이다 (`/home/lim/so101-mobile-manipulation/.omx/plans/prd-so101-autonomy-pick-place.md:311-331`). SLAM 포트폴리오의 선행조건은 아니다.
- Phase 0B는 동일 P-loop 3회, 단일 `map→odom` authority, Nav2 도착 오차와 arm capture basin 결합을 요구한다 (`/home/lim/so101-mobile-manipulation/.omx/plans/prd-so101-autonomy-pick-place.md:333-348`). 이번 SLAM 포트폴리오가 직접 채워야 할 연결 지점이다.
- 이후 Phase 1~6은 ROS Action/arm authority, base motion gate, mission ledger, co-sim, dashboard, 감독 실차 승격 순서다 (`/home/lim/so101-mobile-manipulation/.omx/plans/prd-so101-autonomy-pick-place.md:350-450`).
- SO-101 테스트 명세도 실차 SLAM 3회와 localization loss/TF jump 시 safe stop을 요구한다 (`/home/lim/so101-mobile-manipulation/.omx/plans/test-spec-so101-autonomy-pick-place.md:63-73`).

결론: SLAM 알고리즘·센서 노이즈·replay 평가·frontier는 JD-AMR에 구현한다. SO-101에는 localization/도착 품질을 소비하는 계약과 arm handoff 차단 조건만 반영한다.

### 2.2 JD-AMR에 이미 있던 계획과 구현

- G4 입력은 9.652Hz, scan overlap 0, `Dropped earlier points=0`, `Ignored subdivision=0`까지 검증됐다 (`SLAM_DEBUG_HANDOFF_20260824.md:38-47`). 다만 `map→odom`의 0.28~0.42m 보정 구간이 같은 경로에서 반복되는지는 남아 있다 (`SLAM_DEBUG_HANDOFF_20260824.md:40-47`).
- 최종 실차 재현 게이트는 같은 P-loop 3회, odom 폐루프 ≤0.30m, SLAM 폐루프 ≤0.10m, yaw ≤3°, 겹벽 0, ignored/dropped scan 0이다 (`SLAM_DEBUG_HANDOFF_20260824.md:79-94`).
- 기존 자율매핑 PRD는 ROS 독립 frontier core, fail-closed explorer, mapping 전용 Nav2 launch, 안전 파라미터, 운영 문서, staged verification 순서를 이미 정의한다 (`.omx/plans/prd-jdamr-autonomous-mapping.md:110-184`).
- frontier 추출·clearance·도달성·결정론적 scoring 구현은 이미 존재한다 (`jdamr_cube_navigation/jdamr_cube_navigation/frontier_core.py:138-219`, `jdamr_cube_navigation/jdamr_cube_navigation/frontier_core.py:349-519`).
- explorer는 Cartographer가 `/map`을 만들고 Nav2가 이동을 소유하는 경계를 유지한다 (`jdamr_cube_navigation/README_JDAMR.md:1-14`).
- 현재 Nav2 구성은 NavFn global planner, RPP controller, Collision Monitor다 (`jdamr_cube_navigation/config/nav2_params.yaml:91-115`, `jdamr_cube_navigation/config/nav2_params.yaml:217-220`, `jdamr_cube_navigation/config/nav2_params.yaml:300-337`).
- SLAM Toolbox 예제 launch/config는 학습 저장소에만 있고 JD-AMR 정본에는 아직 통합되지 않았다 (`/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/nav2_goals_py/launch/slam_toolbox_bringup.launch.py:1`, `/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/config/slam_toolbox_mapping.yaml:1`).

### 2.3 이전에 하기로 했지만 아직 남은 연구 항목

- 시뮬레이션 참값 기반 ATE/RPE, AMCL 공분산 기반 NEES, `use_odometry` 비교, `huber_scale` 실험이 기존 로드맵에 남아 있다 (`/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/ROADMAP_MAP.md:91-158`, `/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/ROADMAP_MAP.md:193-244`).
- 기존 문서의 권장 순서도 참값 기록 → ATE/RPE → NEES → odometry on/off → Huber 실험이다 (`/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/ROADMAP_MAP.md:282-298`).
- 실차 실패 시에는 timestamp 확인부터 시작해 속도, range, odometry, 검증된 IMU, SLAM Toolbox 순으로 한 변수씩 바꾸기로 했다 (`/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/JDAMR_NEXT_SESSION.md:153-170`).

### 2.4 원 논문 대조와 적용 판정

논문은 이름을 늘리기 위한 장식이 아니라 현재 스택에서 검증 가능한 가설을 고르는 근거로 사용한다.

| 논문 | 이 프로젝트에 적용할 것 | 판정 |
|---|---|---|
| Hess et al., [Real-Time Loop Closure in 2D LIDAR SLAM](https://research.google.com/pubs/archive/45466.pdf), [DOI](https://doi.org/10.1109/ICRA.2016.7487258) | Cartographer의 nonlocal scan/node↔submap constraint를 TP/FP/missed opportunity로 audit | 직접 적용 |
| Sturm et al., [A Benchmark for the Evaluation of RGB-D SLAM Systems](https://cvai.cit.tum.de/_media/spezial/bib/sturm12iros.pdf), [DOI](https://doi.org/10.1109/IROS.2012.6385773) | GT time association, ATE/RPE convention과 horizon 고정. 센서 종류와 무관한 SE(2) trajectory 평가 원칙만 사용 | 직접 적용 |
| Barczyk et al., [Observability, Covariance and Uncertainty of ICP Scan Matching](https://arxiv.org/abs/1410.7632) | 복도 진행축·횡축·yaw 오차 분해. Cartographer가 ICP라는 주장이나 논문 Hessian 복제는 금지 | 평가 원칙만 적용 |
| Thrun et al., [Robust Monte Carlo Localization for Mobile Robots](https://www.cs.cmu.edu/~thrun/papers/thrun.robust-mcl.pdf), [DOI](https://doi.org/10.1016/S0004-3702(01)00069-8) 및 Fox, [KLD-Sampling](https://proceedings.neurips.cc/paper/2001/hash/c5b2cebf15b205503560c4e8e6d1ea78-Abstract.html) | Nav2 AMCL recovery-off/on kidnapped fault와 adaptive particle 설정을 A/B 평가. Mixture-MCL 재구현이라고 부르지 않음 | 직접 적용 |
| Yamauchi, [A Frontier-Based Approach for Autonomous Exploration](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated2/yamauchi_frontier_explor.pdf), [DOI](https://doi.org/10.1109/CIRA.1997.613851) | distance-only frontier를 기준 정책으로 사용 | 직접 적용 |
| Stachniss et al., [Information Gain-based Exploration Using RBPF](https://roboticsproceedings.org/rss01/p09.pdf), [DOI](https://doi.org/10.15607/RSS.2005.I.009) | 현재 unknown-cell count를 `frontier-cell-gain proxy`로 한정하고 score decomposition을 평가 | 평가 원칙만 적용 |
| Biber and Duckett, [Dynamic Maps for Long-Term Operation](https://roboticsproceedings.org/rss01/p03.pdf), [DOI](https://doi.org/10.15607/RSS.2005.I.003) | transient obstacle과 structural change를 분리해 map lifecycle을 검증. multi-timescale map estimator는 구현하지 않음 | 운영 평가 원칙만 적용 |
| Peng et al., [Sim-to-Real Transfer with Dynamics Randomization](https://xbpeng.github.io/projects/SimToReal/SimToReal_2018.pdf), [DOI](https://doi.org/10.1109/ICRA.2018.8460528) | 실차 quantile에 근거한 parameter vector randomization과 승격 사다리. 학습 policy transfer라고 부르지 않음 | 실험 설계에 적용 |
| Furgale et al., [Unified Temporal and Spatial Calibration](https://doi.org/10.1109/IROS.2013.6696514) 및 Lv et al., [Observability-Aware LiDAR-IMU Calibration](https://april.zju.edu.cn/wp-content/papercite-data/pdf/lv2022oai.pdf) | timestamp/extrinsic을 함께 기록하고 운동 excitation별 `VERIFIED/UNOBSERVABLE` 상태를 부여 | 원칙만 적용 |
| Sünderhauf and Protzel, [Switchable Constraints](https://nikosuenderhauf.github.io/assets/papers/IROS12-switchableConstraints.pdf), [DOI](https://doi.org/10.1109/IROS.2012.6385590) 및 Agarwal et al., [Dynamic Covariance Scaling](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/agarwal13icra.pdf), [DOI](https://doi.org/10.1109/ICRA.2013.6630557) | false-loop fault와 outlier audit의 이론 근거로만 사용 | optimizer 포크가 필요하므로 구현 제외 |

Furgale의 Kalibr 계열은 camera-IMU, Lv 연구는 3D LiDAR·6DoF가 대상이다. JD-AMR에는 해당 도구를 그대로 이식하지 않고 평면 주행에서 calibration observability를 검증하는 원칙만 가져온다.

## 3. 개념과 저장소 경계

| 계층 | 현재 선택 | 이번 포트폴리오에서 비교할 것 | 소유 저장소 |
|---|---|---|---|
| 센서 계약 | G4 LaserScan timestamp/beam order | 시간축 회귀, invalid beam, range/dropout 노이즈 | JD-AMR |
| SLAM mapping backend | Cartographer | Cartographer vs SLAM Toolbox mapping | JD-AMR |
| 저장 지도 localization | AMCL | AMCL vs SLAM Toolbox localization | JD-AMR |
| exploration policy | repo-local frontier scoring | score component ablation | JD-AMR |
| global planner | NavFn | 문제가 관측될 때만 NavFn A* 또는 Smac 2D | JD-AMR |
| local controller | RPP | tracking 문제가 관측될 때만 별도 비교 | JD-AMR |
| task mission | 미션 FSM/arm authority | localization freshness, arrival envelope, handoff 차단 | SO-101 |

`frontier scoring`, `global planner`, `controller`는 서로 다른 알고리즘 계층이다. 이 셋을 하나의 “탐색 알고리즘 비교”로 섞어 결론 내리지 않는다.

### 3.1 현업 문제 프레임

최신 공식 채용·상류 자료와 대조하면 현업의 핵심은 새 SLAM 이름을 많이 아는 것이 아니라 현장 로그에서 결함을 재현하고, 계산·안전 제약 안에서 승격 여부를 결정하는 것이다.

- RIVR의 SLAM Software Engineer는 real-world data, online/offline localization·SLAM, calibration, ground-truth/map workflow, onboard compute 제약을 함께 요구한다 ([공식 채용](https://jobs.lever.co/rivr/54dfe3aa-3732-428a-8476-28de54795007)).
- Field AI의 Calibration, Localization, and Mapping 역할은 intrinsic/extrinsic calibration, time synchronization, noisy/outlier-resilient algorithm, benchmark, real-time compute를 요구한다 ([공식 채용](https://jobs.lever.co/field-ai/ca85d200-ee59-4d7f-93ac-655e7077b398)).
- Autoware sensor configuration은 PTP/time sync, calibration, NIC packet drop, CPU/I/O monitor를 같은 sensor contract로 다룬다 ([공식 문서](https://autowarefoundation.github.io/LSA-reference-design-docs/main/software-configuration/sensor-configuration/)).
- ROS 2 MCAP과 rosbag2는 CRC/index/compression 및 record/playback QoS가 데이터 무결성과 재생에 영향을 준다고 명시한다 ([MCAP upstream](https://github.com/ros2/rosbag2/blob/rolling/rosbag2_storage_mcap/README.md), [현재 QoS override 문서](https://docs.ros.org/en/ros2_documentation/lyrical/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html)).
- Nav2 Collision Monitor는 planner/costmap과 별도인 마지막 software safety layer지만 hard-real-time safety certification을 제공하지 않는다 ([공식 문서](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)).

따라서 최종 report의 장은 알고리즘 이름이 아니라 아래 incident backlog로 구성한다.

| Incident | 현업 문제 | 로컬 실제 근거/재현 방식 | 담당 Phase |
|---|---|---|---|
| I-01 | 센서 시간축·beam order·drop | G4/LD14에서 실제 발생한 dropped point·이중벽 원인 (`SLAM_DEBUG_HANDOFF_20260824.md:17-30`, `SLAM_DEBUG_HANDOFF_20260824.md:53-67`) | 0~1 |
| I-02 | calibration/TF extrinsic drift | sensor pose checksum + synthetic extrinsic perturbation | 0, 4B |
| I-03 | 반복·대칭 복도 퇴화와 false/missed loop closure | P-loop 실제 correction audit + GT simulation fault | 1~3 |
| I-04 | wheel slip·노이즈·outlier·IMU 신뢰 | real-calibrated noise/odometry/Huber/IMU ablation | 4 |
| I-05 | localization loss·kidnapped robot·false convergence | blackout, low-rate, wrong initial pose, pose jump, bag seek/reset | 5 |
| I-06 | 오래된 지도·map/config mismatch | map lifecycle ledger와 stale-map localization fault | 5, 9 |
| I-07 | blocked path·동적 장애물·recovery exhaustion | planner/controller/recovery/Collision Monitor fault injection | 6~7 |
| I-08 | CPU·latency·queue backlog·thermal | clock 검증된 구간별 sensor/SLAM/control latency와 resource/thermal trace | 1~6 |
| I-09 | sim-to-real domain gap | real-quantile randomization과 S0~S5 승격 사다리 | 4B, 8 |
| I-10 | 현장 데이터 재현 실패 | MCAP/QoS/checksum/index, immutable failure run, deterministic replay | 0, 2, 9 |

실제로 관측한 I-01과 실제 로그 기반 I-03/I-08은 “현장 장애”로, 아직 실차에서 관측하지 않은 fault injection은 “검증 시나리오”로 표기한다. 시뮬레이션으로 만든 장애를 실제 현장 사고처럼 서술하지 않는다.

## 4. 공통 실험 규약

모든 실험은 다음 규약을 먼저 고정한다.

1. 같은 복도 P-loop에서는 출발 십자, 앞 방향, LiDAR 높이·yaw, 속도 상한, 장애물 배치, 시작 시 빈 지도를 동일하게 유지한다. 폐루프의 `odom→base_footprint`와 `map→base_footprint`를 분리해 기록한다 (`/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/JDAMR_NEXT_SESSION.md:119-148`).
2. 매 실험은 `run_id`, 환경, bag SHA-256, Git SHA, ROS/패키지 버전, backend, config hash, seed, 시작 pose, 속도, 성공/실패 이유를 manifest에 남긴다.
   - sensor serial/firmware, calibration date, TF/extrinsic checksum
   - publisher/recorder/playback QoS, MCAP writer profile, CRC/index 상태
   - NIC/driver/kernel drop counter, host/CPU/RAM/thermal profile
   - 각 host의 clock source, run 시작/종료 clock offset·uncertainty와 측정 방법
3. mapping backend, localization node 중 `map→odom` publisher는 항상 정확히 하나만 실행한다. 원본 bag의 `map→odom`은 backend replay 입력에서 제외한다.
4. 실차에는 전체 궤적 ground truth가 없으므로 ATE/RPE라고 부르지 않는다. 실차는 폐루프 끝점 오차, 지도 아티팩트, scan drop/latency, `map→odom` correction, CPU/RSS를 기록한다. 폐루프 오차는 ATE의 실기 대용일 뿐 ATE가 아니다 (`/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/JDAMR_NEXT_SESSION.md:141-148`).
5. ATE/RPE/NEES는 Gazebo ground truth와 추정 pose를 같은 clock으로 기록한 시뮬레이션에서 계산한다.
6. 센서/설정 실험은 한 번에 변수 하나만 바꾸고, baseline과 treatment를 같은 bag·seed로 짝지어 비교한다 (`/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/JDAMR_NEXT_SESSION.md:153-170`).
7. 성공 표본만 남기지 않는다. 실패 run도 bag, config, reason을 같은 형식으로 보존한다.

## 5. 실행 순서

### Phase 0 — 기준선 봉인과 기존 계획 상태 대조 (4시간)

작업:

1. 현재 navigation package를 clean build/test하고 실패를 0으로 만든다. 2026-08-31 fresh 결과는 98 tests 중 96 passed, 1 skipped, 1 failed이며, 실패는 `jdamr_cube_navigation/jdamr_cube_navigation/frontier_explorer.py:67`의 PEP257 D213이다.
2. 기존 G4 reference bag `/home/lim/jdamr_artifacts/g4_userloop_reset_20260824T175151`을 dataset index에 진단용으로 등록한다. 이 bag은 195.889초, 63,955 messages, `/scan` 1,883, `/odom` 9,757, `/tf` 48,533개다. 마지막 리프트 뒤 scan/SLAM 급변 구간이 있으므로 raw bag은 immutable하게 보존하고, last nonzero command와 scan/pose discontinuity로 정한 pre-lift cutoff만 회귀 분석에 쓴다 (`SLAM_DEBUG_HANDOFF_20260824.md:40-47`). 이 bag과 저장 지도는 새 3회 합격 표본으로 세지 않는다.
3. 아래 평가 골격을 JD-AMR에 둔다.
   - 새 `jdamr_cube_navigation/evaluation/experiment_manifest.schema.json`
   - 새 `jdamr_cube_navigation/evaluation/datasets.yaml`
   - 새 `jdamr_cube_navigation/evaluation/README.md`
4. SO-101 PRD의 “구현 시작 전” 상태와 현재 untracked/working-tree 미션 코드를 대조하되, 이번 단계에서는 SO-101 코드를 변경하지 않는다.
5. 새 `jdamr_cube_navigation/evaluation/map_registry.yaml`에 map 상태를 `candidate → published → deprecated`로 기록한다. quality gate 실패, map/config checksum mismatch, sensor extrinsic 변경, 구조적 환경 변경이 있으면 기존 map을 자동 선택하지 않고 candidate 재평가로 되돌린다.
6. 새 `jdamr_cube_navigation/evaluation/calibration_registry.yaml`에 sensor pair별 static TF checksum, timestamp source, 검증일, uncertainty, `VERIFIED/UNOBSERVABLE` 상태를 기록한다. 정지·직진·CW·CCW 회전 bag을 분리해 어느 방향이 관측 불가능했는지 남긴다.

통과 기준:

- `colcon test --packages-select jdamr_cube_navigation` 결과 failure 0.
- 모든 등록 bag의 경로, SHA-256, topic/count, duration, sensor/backend 포함 여부가 manifest schema를 통과한다.
- MCAP CRC/index와 record/playback QoS가 검증되고, topic count/drop counter 불일치는 명시적 실패다.
- map registry의 모든 published map은 source bags, backend/config hash, quality report로 역추적된다.
- `UNOBSERVABLE` 또는 미검증 LiDAR-IMU pair는 IMU fusion run에 사용할 수 없다.
- 기존 사용자 변경을 수정·스테이징하지 않는다.

### Phase 1 — 동일 복도 실차 재현성 게이트 (8시간)

작업:

1. 기존 G4 run은 진단 reference로만 두고, 같은 출발 표시·방향·P-loop를 새 규약의 raw bag으로 3회 수집한다.
2. 각 run은 빈 mapping state에서 시작하고, 종료 시 먼저 map 저장 또는 SLAM 정지를 완료한 뒤 로봇을 들어 올린다 (`SLAM_DEBUG_HANDOFF_20260824.md:43-47`).
3. odom/SLAM 폐루프 x/y/yaw, 주행거리 대비 상대오차, double-wall/ghost-cell audit, scan gap/overlap/drop, `map→odom` correction p95/max, CPU/RSS를 산출한다.
4. 반복·대칭 구간을 scenario tag로 표시하고 accepted loop-closure event, correction 크기, 지도 아티팩트를 함께 audit한다. 실차에서는 true/false를 추측하지 않고 바닥 기준점·수동 map audit과 연결한다.
5. cross-host latency는 먼저 NTP/PTP 또는 NTP-style ping-pong probe로 Pi↔laptop clock offset/uncertainty를 구한다. 그 뒤 sensor stamp→laptop receipt, laptop-local backend processing, TF age, controller callback period/jitter를 별도 stage로 기록한다. backend가 같은 scan stamp를 노출하지 않으면 receipt→backend를 인과 latency로 만들지 않는다.
6. obstacle fault에서는 Collision Monitor의 scan receipt→최종 zero command를 별도 causal trace로 측정한다. 일반 scan과 `/cmd_vel`을 억지로 1:1 대응시킨 단일 `scan→command` latency는 보고하지 않는다.
7. gate가 실패하면 timestamp 반영 확인부터 기존 실패 사다리를 한 단계씩 적용한다. 한 run에서 둘 이상의 설정을 바꾸지 않는다.

통과 기준:

- protocol-qualified 3회 모두 odom 폐루프 ≤0.30m, SLAM 폐루프 ≤0.10m, yaw ≤3°.
- double wall 0, `Ignored subdivision=0`, `Dropped earlier points=0`, scan overlap/역전 0.
- 0.28~0.42m `map→odom` correction 구간의 재현 여부와 원인 가설을 3회 표로 남긴다.
- G4 scan period를 `T_scan`으로 두고 clock uncertainty≤5ms일 때만 cross-host sensor stamp→receipt를 산출한다. causal ID가 확인된 backend processing p95≤`T_scan`, TF age≤`2×T_scan`, backlog 유발 drop 0을 통과한다. clock/causal 조건이 없으면 해당 metric은 `INVALID`, 구간별 local metric만 보고한다.
- raw bag과 map/config/manifest가 run별로 1:1 연결된다.

### Phase 2 — 시뮬레이션 ground-truth 평가 harness (10시간)

작업:

1. 기존 `record_path.py`와 `compare_maps.py`를 참고하되 정본 구현은 JD-AMR evaluation 패키지에 둔다 (`/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/nav2_goals_py/nav2_goals_py/record_path.py:1`, `/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/compare_maps.py:1`).
2. 새 `record_trajectory.py`가 동일 timestamp로 ground truth, odom, SLAM, AMCL pose/covariance를 기록한다.
3. 새 `evaluate_trajectory.py`가 SE(2) timestamp association 뒤 ATE RMSE, RPE translation/yaw, endpoint error, NEES를 계산한다.
4. 새 `run_replay_benchmark.py`가 bag/backend/config/seed matrix를 실행하고 CSV/JSON 요약을 만든다.
5. hand-written ATE/RPE는 `evo` 결과와 한 run 이상 교차 검산한다.
6. backend별 pose graph를 공통 constraint schema로 변환한다. Cartographer는 `INTER_SUBMAP`에 해당하는 accepted nonlocal node↔submap constraint만, SLAM Toolbox는 accepted nonlocal node↔node loop constraint만 평가하고 local/intra-submap constraint는 제외한다.
7. GT revisit opportunity는 keyframe/node 쌍이 시간상 ≥30초, 누적 경로상 ≥3m 떨어져 있으면서 최종 GT 상대 pose가 ≤0.30m, ≤10°인 경우로 사전 정의한다. accepted constraint의 **최종 최적화 후** 추정 상대변환과 같은 쌍의 GT 상대변환 residual이 ≤0.20m, ≤10°면 TP, 이를 넘으면 FP다. missed opportunity는 trajectory 종료와 final optimization grace 10초 뒤에도 대응 accepted nonlocal constraint가 없을 때만 1회 집계한다.
8. metric manifest에 association max gap 기본 50ms, known world→map SE(2) transform, post-hoc alignment 사용 여부, RPE horizon `Δt={1s,10s}`를 고정한다. 복도 local frame에서 `e_parallel`, `e_perp`, `e_yaw`를 함께 출력한다.

통과 기준:

- zero-error, constant offset, linear drift, yaw drift synthetic fixtures에서 기대값과 오차 `1e-6` 이내.
- timestamp mismatch, frame mismatch, missing ground truth는 숫자를 만들지 않고 명시적 실패로 종료.
- 같은 bag/config/seed를 2회 replay했을 때 metric JSON이 동일하다.
- `evo`와 ATE translation RMSE 차이 ≤1mm, RPE yaw 차이 ≤0.1° 또는 차이의 frame/alignment 원인을 문서화하고 동일 convention으로 재검산한다.
- symmetric-corridor fixture에서 nonlocal constraint TP/FP/missed-opportunity label과 backend adapter가 synthetic expected count와 정확히 일치한다.
- 두 backend는 동일 timestamp association과 pose-pair set을 사용하며 symmetric-corridor fixture의 accepted nonlocal FP는 0건이다.

### Phase 3 — Cartographer vs SLAM Toolbox mapping A/B (10시간)

작업:

1. 새 `jdamr_cube_navigation/launch/slam_toolbox_mapping.launch.py`와 `config/slam_toolbox_mapping.yaml`을 JD-AMR 정본으로 만든다. 학습 저장소 파일은 참고만 하고 복제 후 분기시키지 않는다.
2. backend는 동시에 실행하지 않는다. 서로 다른 5개 simulation bags/noise realizations와 Phase 1의 protocol-qualified real raw bags 3개를 각각 Cartographer/SLAM Toolbox로 paired replay한다. 같은 고정 stream을 seed 이름만 바꿔 독립 표본으로 세지 않는다.
3. 시뮬레이션은 ATE/RPE, `e_parallel/e_perp/e_yaw`, loop TP/FP/missed, correction, runtime/CPU/RSS를 비교한다. 실차는 축별 폐루프·map artifact·scan error/runtime/CPU/RSS로 비교한다.
4. 필수 안전/정합 gate를 먼저 적용한 뒤, 통과 후보끼리 정확도·강건성·자원·운영 복잡도를 비교한다. primary metric은 simulation ATE와 real 폐루프 오차, secondary metric은 RPE·artifact 실패 수, tertiary metric은 CPU/RSS·운영 복잡도다. 대안 backend의 paired 개선이 baseline 반복성 envelope(IQR 또는 동일 조건 run 간 변동폭)를 넘고 다른 primary metric을 그 envelope보다 악화시키지 않을 때만 전환한다. 동률이면 기존 Cartographer를 유지한다.

통과 기준:

- backend별 5 sim seeds + 3 real replays 결과가 같은 schema로 생성된다.
- 모든 replay에서 단일 `map→odom` authority, timestamp/drop error 0.
- 선택 문서에 winner, 패배 조건, trade-off, 유지/전환 이유가 raw metric 링크와 함께 기록된다.
- simulation accepted nonlocal FP가 1건이라도 있는 후보는 ATE 개선만으로 기본 backend가 될 수 없다.
- 비교 전까지 SO-101 기본 backend를 바꾸지 않는다. 기존 PRD도 근거 없는 즉시 교체를 비목표로 둔다 (`/home/lim/so101-mobile-manipulation/.omx/plans/prd-so101-autonomy-pick-place.md:40-50`).

### Phase 4 — 센서 노이즈·오도메트리·강건성 ablation (12시간)

작업:

1. 정지 bag에서 beam angle별 median/MAD, invalid beam 비율, range dropout, scan period jitter를 측정해 관측 noise profile을 만든다.
2. 고정된 5개 noise realization으로 observed 1×와 stress 2× 노이즈를 replay에 주입한다. 무근거 임의 필터부터 넣지 않는다.
3. 다음 조건을 한 번에 하나씩 비교한다.
   - raw baseline
   - invalid/non-finite beam sanitization
   - `min_range`/`max_range` clipping
   - observed dropout/range-noise injection
   - `use_odometry=false/true`
   - Cartographer pose-graph optimization에만 적용되는 `huber_scale` baseline/한 단계 변경
   - 각 dataset에서 IMU 축 방향·scale·bias·timestamp 검증 통과 뒤 IMU off/on
4. driver timestamp/beam-order 계약과 backend 파라미터 효과를 같은 “노이즈 처리”로 뭉치지 않고 계층별로 보고한다.
5. `huber_scale`을 Switchable Constraints나 DCS와 동치라고 표현하지 않는다. 모든 ablation은 전체 ATE/RPE와 함께 `e_parallel/e_perp/e_yaw`를 출력한다.

통과 기준:

- 조건별 5개 paired noise realizations, baseline 포함 모든 metric과 config hash가 보존된다.
- paired delta와 median/IQR을 설명 통계로 보고하고 n=5만으로 통계적 우월성을 주장하지 않는다. 단일 최고 run으로 결론 내리지 않는다.
- 최신 실측 IMU는 약 45.9Hz, 정지 가속도 norm 9.358m/s², 720° gyro 오차 1.3%로 사용 가능하다. 다만 각 dataset의 축·scale·bias·timestamp 검증 실패 시 해당 fusion run을 금지한다 (`SLAM_DEBUG_HANDOFF_20260824.md:115-126`).
- 채택한 처리마다 “어떤 실패를 줄였고 어떤 비용을 늘렸는지”가 수치로 남는다.

### Phase 4B — Sim-to-Real 모델과 사전승격 S0~S2 (6시간)

작업:

1. 새 `jdamr_cube_navigation/evaluation/sim_to_real_matrix.yaml`에 simulation과 real의 차이를 센서 주기/jitter, range noise/dropout, wheel slip/scale, TF extrinsic 오차, command/transport latency, 마찰, 장애물·복도 형상으로 분해한다.
2. Phase 1·4 실차 bag의 median/MAD/quantile와 calibration registry uncertainty로 domain-randomization 범위를 정한다. 실차에서 관측되지 않은 임의 범위를 “현실적 노이즈”라고 부르지 않는다.
3. 각 S1 run은 `θ={scan period/jitter, range noise/dropout, wheel scale/slip, time offset, TF extrinsic, transport latency, friction}`의 sampled value, seed, 분포 형식, 실차 source run과 quantile 근거를 manifest에 기록한다. S0/S1은 같은 initial pose와 command schedule을 쓴다.
4. mapping/noise 후보 config를 다음 사전승격 순서로 검증한다. 이는 최종 통합 stack의 real default 승격이 아니다.
   - S0 deterministic simulation
   - S1 real-calibrated noisy/randomized simulation 5 seeds
   - S2 real MCAP offline replay 3 bags
5. 새 `evaluate_transfer.py`가 양쪽에서 공통으로 잴 수 있는 endpoint error, scan health, map artifact, Nav success, CPU/RSS의 transfer gap을 만든다. ATE/RPE/NEES와 GT reachable-cell coverage는 simulation-only다. 실차 coverage가 필요하면 사전 audit한 reference-map polygon 대비 observed-area ratio로 별도 이름을 쓰고, reference가 없으면 생략한다.
6. S0/S1에서 이긴 mapping/noise 후보도 S2 전에는 후보 승격조차 하지 않는다. nominal calibration만 통과하고 uncertainty 범위 perturbation에서 실패한 후보도 S2로 올리지 않는다. 최종 real default는 Phase 5~7 선택 뒤 Phase 8의 통합 S0~S5를 모두 통과해야 한다.

통과 기준:

- S1은 5 runs 모두 manifest/config hash뿐 아니라 전체 `θ` vector와 각 범위의 real-data 근거를 재현한다.
- S2의 protocol-qualified 3 bags 모두 timestamp/TF authority gate를 통과한다.
- 이 Phase의 산출물은 `candidate`이며 real default 승격 기록을 만들지 않는다.
- 사전 report에 각 공통 metric의 `sim nominal → sim randomized → real replay` 변화와 남은 domain-gap 가설이 연결된다.

### Phase 5 — 저장 지도 localization benchmark (10시간)

작업:

1. Phase 3 winner와 Phase 4 채택 설정을 확정한 뒤 candidate map artifact checksum을 고정하고 mapping 노드를 종료한다.
2. localizer component 비교용으로 동일 SLAM Toolbox mapping session에서 `serialized posegraph + yaml/pgm` bundle을 만든다. AMCL은 이 session의 occupancy map을, SLAM Toolbox localization은 같은 session의 posegraph를 소비한다.
3. deployment 선택은 다음 호환 bundle 전체로 한다.
   - Cartographer `yaml/pgm` + AMCL: 가능
   - SLAM Toolbox `posegraph` + SLAM Toolbox localization: 가능
   - 같은 SLAM Toolbox session의 `yaml/pgm` + AMCL: 가능
   - Cartographer map + SLAM Toolbox localization: **불가**, 후보에서 제외
4. AMCL과 SLAM Toolbox localization을 별도 세션에서 실행한다. 두 localizer를 동시에 실행하지 않는다.
5. 시뮬레이션에서 시작/중간/코너 위치와 x/y/yaw 부호를 바꾼 고정 15-case initial-pose manifest를 만든다. 크기는 0.2m/5°, 0.5m/15°, 1.0m/30°의 3단계이며 같은 15 cases를 두 localizer에 paired 적용한다.
6. convergence는 ground-truth SE(2) 오차 ≤0.15m, yaw ≤5°를 3초 연속 유지한 상태로 정의하고 timeout은 60초로 둔다. false convergence는 readiness가 3초 true인데 ground-truth 오차가 >0.30m 또는 >10°인 경우다.
7. convergence rate/time, post-convergence ATE/RPE, recovery time, false convergence, CPU/RSS를 비교한다. NEES는 `[x,y,wrap(yaw)]`와 3×3 covariance를 사용하고, non-finite/non-symmetric/singular covariance는 invalid로 분리한다. 시간 sample을 독립 표본으로 세지 않고 run별 95% chi-square coverage와 15-run aggregate를 보고한다.
8. 실차에서는 바닥 기준점의 반복 도착 오차를 측정한다. 이를 ATE라고 부르지 않는다.
9. relocalization fault suite에 scan blackout 0.5/2/5초, scan 저주기, kidnapped pose jump, 잘못된 initial pose, TF stale, bag seek/reset, published map checksum mismatch를 포함한다.
10. system localization state를 `VALID`, `RECOVERING`, `INVALID`로 정의한다. `RECOVERING`과 `INVALID`에서는 base를 정지하고 arm dispatch를 차단한다. 새 pose가 convergence gate를 다시 통과해야만 `VALID`로 돌아간다.
11. published map에서 false convergence 또는 구조적 환경 변경이 재현되면 기존 map registry 항목을 `published → deprecated`로 전환한다. 재매핑 산출물은 별도 `candidate`로 등록해 전체 quality gate를 다시 통과시키며, 새 candidate가 없거나 실패하면 published map 없는 차단 상태를 유지한다.
12. 현재 AMCL의 `recovery_alpha_fast=0.0`, `recovery_alpha_slow=0.0`을 recovery-off 기준선으로 고정하고, Nav2가 제시하는 후보값 `recovery_alpha_fast=0.1`, `recovery_alpha_slow=0.001`을 recovery-on으로 둔다 (`jdamr_cube_navigation/config/nav2_params.yaml:30-31`, [Nav2 Jazzy AMCL 문서](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/others/configuring_amcl/)). particle/KLD/laser/odom 설정은 고정한 채 같은 15-case manifest의 중간 시점에 결정론적 kidnapped teleport를 주입해 조건별 15회, 총 30회 추가 실행한다. 이 실험을 논문의 Mixture-MCL 재구현이라고 부르지 않는다.
13. 변화 환경은 장애물을 넣었다 제거하는 transient fixture와 문·벽 상태가 지속해서 바뀌는 structural fixture로 나눈다. 이번 범위에서는 multi-timescale dynamic-map estimator를 구현하지 않고, published map 보존·candidate 생성·deprecated 전환이라는 lifecycle 동작만 검증한다.

통과 기준:

- 15-case × 2 localizer의 30 simulation runs, 즉 15개 paired comparison 모두 manifest와 terminal reason을 갖는다.
- 추가 AMCL recovery A/B 30회는 config hash, injected pose/time, `VALID/RECOVERING/INVALID` 전이, recovery time, false convergence, CPU/RSS를 보존한다. recovery-on에서 false convergence 또는 `VALID` 복귀 전 arm dispatch가 1건이라도 있으면 후보를 제외하고, recovery time·성공률 개선이 없으면 recovery-off를 유지한다.
- loss/TF stale/dual authority 주입 시 base stop과 failure가 기록되고 자동 arm handoff는 0건.
- 호환 가능한 mapping+localization bundle 중 SO-101이 소비할 기본 bundle과 초기 pose/recovery 운용 규칙을 선택한다.
- SO-101 Phase 0B의 Nav final pose envelope에 사용할 dx/dy/dyaw 분포가 생성된다.
- fault suite의 각 case에서 localization state, Nav pause/stop, arm block, terminal reason이 expected state table과 일치한다.
- transient fixture는 장애물 제거 뒤 기존 map을 `published`로 유지한 채 recovery한다. structural fixture는 사전 정의한 지속시간 뒤 기존 map을 `deprecated`로 바꾸고, 재매핑 산출물이 있을 때만 이를 별도 `candidate`로 등록한다. 두 fixture 모두 run/trace ID로 원인과 상태 전이를 역추적한다.

### Phase 6 — frontier 탐색 정책 평가 (8시간)

작업:

1. 기존 frontier core/state machine을 새로 쓰지 않고 회귀 기준선으로 사용한다 (`jdamr_cube_navigation/jdamr_cube_navigation/frontier_core.py:153-219`, `jdamr_cube_navigation/jdamr_cube_navigation/frontier_explorer.py:226-248`).
2. simulation에서 distance-only, frontier-cell-gain-proxy+distance, 현재 full score(+heading/blacklist) 3개 정책을 서로 다른 5개 layout/noise seeds로 paired 비교한다. 현재 `information_gain`은 posterior entropy나 mutual information이 아니라 unknown frontier-cell count 기반 프록시로 명명한다 (`jdamr_cube_navigation/jdamr_cube_navigation/frontier_core.py:474-506`).
3. coverage, time-to-coverage, path length, goal reject/fail, recovery count, minimum clearance, collision/contact, CPU/RSS를 기록한다.
4. 실차는 최고 정책 하나만 감독 환경에서 실행하며 기존 fail-closed start/pause/stop과 map save 규약을 유지한다 (`jdamr_cube_navigation/README_JDAMR.md:29-69`).
5. blocked path, obstacle emergence, planner `NO_PATH`, controller oscillation, recovery exhaustion, Collision Monitor stop/slowdown을 simulation에서 주입하고 explorer/Nav2/command-owner terminal trace를 기록한다.
6. 각 goal 선택마다 frontier-cell gain, path distance, heading, blacklist/reject 항과 최종 score를 함께 기록해 어떤 항이 선택을 바꿨는지 재생할 수 있게 한다.

통과 기준:

- 15개 simulation runs가 같은 map/seed matrix로 완료되며, n=5 결과는 effect size와 설명 통계로만 해석한다.
- 15개 run 모두 goal별 score decomposition을 갖고, 결과에는 entropy-aware 또는 RBPF exploration을 구현했다는 주장을 하지 않는다.
- reachable free-cell coverage ≥85%, simulator contact 0, invalid/unknown goal 전송 0이라는 기존 게이트를 유지한다 (`.omx/plans/test-spec-jdamr-autonomous-mapping.md:52-60`).
- 최고 정책 선택이 coverage 하나가 아니라 시간·경로·실패·안전·자원 trade-off를 포함한다.
- 실차 run은 `/scan /map /odom /tf /cmd_vel /collision_monitor_state`와 explorer status를 기록한다.
- fault별 terminal reason이 expected table과 일치하고 unsafe contact/nonzero command-owner conflict는 0건이다. Collision Monitor trigger 뒤 final nonzero command가 0이 되는 시간은 목표 0.25초, 필수 상한 0.4초 watchdog 이전이다.

### Phase 7 — global planner/controller 비교는 증거가 있을 때만 (선택, 6시간)

진입 조건:

- 유효 frontier goal 20개 중 NavFn path 생성 실패가 2회 이상이거나,
- planner 성공 뒤 controller recovery/oscillation이 2회 이상 재현되거나,
- 현재 경로가 동일 costmap의 grid 최단경로 대비 median 1.25배를 넘는다.

작업:

1. planner 문제면 NavFn Dijkstra/`use_astar` 또는 Smac 2D를 같은 costmap/goal set으로 비교한다.
2. controller 문제면 RPP 파라미터와 대안 controller를 별도 실험으로 비교한다. planner와 controller를 동시에 바꾸지 않는다.
3. 진입 조건이 없으면 “현재 NavFn+RPP 유지”를 근거 있는 stop decision으로 기록하고 이 Phase를 생략한다.
4. Collision Monitor를 safety-rated 장치로 표현하지 않는다. software layer를 우회하는 물리 비상정지와 감독 운용 경계는 별도다.

통과 기준:

- 비교를 했다면 동일 20-goal set에서 success, planning time, path length/clearance, tracking error, recovery를 분리 보고한다.
- 변경이 mandatory gate를 개선하지 않으면 현행 조합으로 rollback한다.

### Phase 8 — 최종 통합 Sim-to-Real S0~S5와 held-out 일반화 (8시간)

작업:

1. Phase 3~7에서 선택한 `{sensor profile, mapping backend/config, map artifact, localizer, frontier policy, planner, controller}`를 하나의 immutable deployment bundle과 checksum으로 고정한다.
2. 이 **통합 bundle**을 순서대로 검증한다.
   - S0 deterministic same-corridor simulation
   - S1 real-calibrated randomized simulation 5 seeds
   - S2 protocol-qualified real MCAP replay 3 bags
   - S3 read-only hardware preflight
   - S4 감독 same-corridor real run 3회
   - S5 held-out simulation 5 seeds + held-out real run 3회
3. S3에서 graph, TF, sensor rate, lifecycle, command owner가 read-only로 확인되기 전 real motion은 0건이어야 한다.
4. 같은 복도는 통제 실험용으로 유지한다. 최종 설정이 그 복도에만 맞춰지는 것을 막기 위해 두 held-out을 분리한다.
   - simulation held-out: 방+복도, 코너, 문, 동적 장애물, ground truth coverage/ATE/RPE 사용.
   - real held-out: 사람·동물·계단이 없는 정적 감독 환경, 바닥 기준점·폐루프·scan health·map artifact만 사용.
5. S0 시작 뒤 config를 바꾸지 않는다. gate 실패 시 bundle을 candidate로 되돌리고 원인 가설 하나를 검증한 뒤 새 checksum으로 S0부터 다시 시작한다.
6. 동일 복도 대비 simulation은 ATE/RPE/GT coverage, real은 폐루프/기준점 오차와 scan/map artifact를 비교한다. 실차 observed-area ratio는 audit된 reference polygon이 있을 때만 별도 이름으로 보고한다.
7. S1은 Phase 4B와 같은 `θ` schema와 calibration uncertainty 범위를 사용하되 통합 bundle hash에 묶는다. 후보별 유리한 seed나 범위로 다시 튜닝하지 않는다.

통과 기준:

- S0~S5가 동일 deployment bundle hash와 순서 증거를 갖는다.
- S1의 모든 run이 전체 `θ` vector, sampling distribution, seed, 실차 quantile source와 calibration perturbation 결과를 보존한다. seed만 남거나 nominal calibration만 실행한 bundle은 S2로 진행하지 않는다.
- S4 3회는 same-corridor SLAM/localization/navigation gate와 0.4초 이내 software stop 상한을 통과한다.
- simulation held-out 5 seeds와 real held-out 3회 모두 사전 고정 config hash로 실행된다.
- 실패를 숨기지 않고 환경 퇴화 요인과 다음 실험을 분리 기록한다.
- 안전 gate(contact 0, invalid goal 0, 단일 command/TF authority)는 환경과 무관하게 유지된다.

### Phase 9 — 포트폴리오 패키징과 SO-101 Phase 0B 연결 (8시간)

작업:

1. JD-AMR의 `jdamr_cube_navigation/evaluation/reports/slam_portfolio.md`에 문제→원인→실험 설계→결과→선택→한계 순서의 benchmark report를 만든다. 대용량 원본은 `$HOME/jdamr_artifacts/slam_portfolio/`에 두고 repo의 dataset index에서 checksum으로 연결한다.
2. 대표 MCAP manifest, config snapshot, metric CSV/JSON, map 전후 이미지, correction timeline, 실패 사례, 2~3분 영상을 연결한다. 대용량 bag 자체는 Git에 넣지 않는다.
3. 재현 명령 하나로 offline replay→metric report가 생성되게 한다.
4. Phase 8의 통합 sim-to-real 표에 nominal sim, randomized sim, real replay, preflight, real motion, held-out을 같은 run ID 체계로 연결한다.
5. MCAP writer/QoS/CRC/index, map registry의 candidate/published/deprecated history, calibration/TF checksum, latency budget을 incident별 evidence index에 연결한다.
6. SO-101에는 다음 소비 계약만 반영한다.
   - localization/TF freshness와 covariance/quality 상태
   - Nav2 final dx/dy/dyaw와 capture-basin 판정
   - localization loss/TF jump 시 arm dispatch 금지
   - `map_id`, config hash, run/mission ID telemetry
7. 위 계약을 기존 SO-101 Phase 0B와 Navigation·SLAM 테스트 명세에 연결하고, SLAM backend 코드는 복제하지 않는다.
8. Phase 0A의 measured capture-basin dataset이 없으면 Nav dx/dy/dyaw 분포까지만 전달하고 Phase 0B 완료를 주장하지 않는다. basin dataset이 있을 때만 `R-NAV-07` 결합 gate를 실행한다.

통과 기준:

- 새 환경에서 clone 후 문서화된 offline 명령으로 대표 report를 재생성한다.
- sim-to-real 표의 모든 real default 승격에는 통합 deployment bundle의 S0~S4 evidence가 연결되고, 중간 gate가 빠진 설정은 승격되지 않는다. 포트폴리오 완료에는 S5도 필요하다.
- 모든 표의 숫자가 manifest/run artifact로 역추적된다.
- backend 선택과 noise/localization/exploration 결정마다 반례·실패 사례가 하나 이상 포함된다.
- SO-101 arm handoff는 localization/arrival gate가 false 또는 unknown일 때 0건이다.
- Phase 0A capture-basin evidence가 없을 때 Phase 0B status는 `BLOCKED_ON_CAPTURE_BASIN`으로 남는다.

## 6. 전체 완료 기준

1. Phase 0~6(Phase 4B 포함), 8, 9가 완료되고 Phase 7은 진입 조건 충족 여부가 기록된다.
2. 동일 복도 3회 실차 gate와 held-out 환경 3회가 raw evidence를 남긴다.
3. Cartographer+AMCL, SLAM Toolbox+SLAM Toolbox localization, SLAM Toolbox+AMCL의 호환 bundle을 서로 다른 세션에서 공정하게 비교하고, 불가능한 Cartographer+SLAM Toolbox localization 조합은 실행하지 않는다.
4. 실차 폐루프 지표와 시뮬레이션 ATE/RPE/NEES를 혼용하지 않는다.
5. noise/odometry/Huber/IMU 실험은 한 변수 원칙과 fixed seed를 지킨다.
6. frontier policy, global planner, controller를 별도 결과로 보고한다.
7. offline replay와 metric report가 재현 가능하고 모든 결론이 bag/config/code SHA로 역추적된다.
8. SO-101에는 결과 소비 계약만 들어가며 JD-AMR의 SLAM 코드를 복제하지 않는다.
9. 최종 real default는 S0 deterministic sim → S1 randomized sim → S2 real replay → S3 read-only preflight → S4 supervised real gate를 순서대로 통과한다.
10. 논문에서 가져온 항목은 실제 구현·평가 범위와 판정이 일치하고, 원칙만 차용하거나 구현에서 제외한 방법은 포트폴리오 성과로 주장하지 않는다.

## 7. 일정과 병렬화

| 주차 | 주 작업 | 완료 증거 |
|---|---|---|
| 1주 | Phase 0~1 | clean test, same-corridor 3-run dataset, 기존 SLAM gate 판정 |
| 2주 | Phase 2~3 | GT metric harness, backend A/B report |
| 3주 | Phase 4~5 | noise ablation, sim-to-real matrix/transfer ladder, localization benchmark |
| 4주 | Phase 6 | frontier ablation, supervised real exploration evidence |
| 5주 | Phase 7 조건부 + Phase 8 | planner/controller stop-or-compare, 통합 S0~S5, held-out 결과 |
| 6주 | Phase 9와 재검증 | reproducible report, sim-to-real evidence chain, SO-101 Phase 0B contract |

실차 capture는 순차로 수행한다. offline bag replay, metric 구현, 문서/시각화는 파일 소유권을 나눌 수 있을 때만 병렬화한다.

## 8. 위험과 완화

| 위험 | 조기 신호 | 완화/중단 조건 |
|---|---|---|
| 같은 복도 과적합 | held-out에서 오차·coverage 급락 | Phase 8 config freeze, 결과 후 재튜닝 금지 |
| 지표 이름 과장 | 실차 결과에 ATE/RPE 표기 | 실차/시뮬 schema 분리, review gate에서 차단 |
| backend 동시 TF 발행 | `map→odom` publisher 2개 | preflight fail-closed, 해당 run 폐기 |
| 여러 변수를 동시에 변경 | manifest diff가 2개 이상 | run 무효 처리 후 단일 변수로 재실행 |
| 선택적 알고리즘 비교가 범위 팽창 | planner/controller 문제 증거 없음 | Phase 7 생략, 현행 유지 결정 기록 |
| IMU 오염 | scale/bias 검증 실패 | 실차 fusion 금지, offline 분석만 유지 |
| 실차 표본 손실 | 성공 run만 보존 | failure artifact도 동일 index에 등록 |
| simulation winner의 성급한 실차 승격 | S0/S1 결과만 있고 S2~S4 evidence 없음 | transfer ladder 중간 gate 생략 금지, 이전 real default 유지 |
| SO-101과 코드 정본 분기 | SLAM launch/config 복제 | JD-AMR 의존/결과 계약만 소비, 복제 PR 거부 |
| 논문 이름만 빌린 과장 | 구현하지 않은 entropy·robust optimizer·dynamic map을 성과로 표기 | 논문 적용 판정표와 report claim을 대조하고 해당 표현을 review gate에서 차단 |

## 9. 비목표와 stop rule

- 이번 포트폴리오에서 3D LiDAR SLAM, LIO-SAM/FAST-LIO2, VIO, 새 센서 구매, 멀티로봇/FMS를 구현하지 않는다. 현재 센서는 2D LiDAR이며 3D 계열을 그대로 적용할 수 없다는 기존 분석을 유지한다 (`/home/lim/physical-ai-lab/learning/M02_linux_ros2/slam_nav2/ROADMAP_MAP.md:12-49`).
- SO-101 팔 제어·pick/place 자체는 별도 Phase 0A 트랙이다. SLAM 포트폴리오를 팔 프로젝트 전체로 확장하지 않는다.
- SLAM Toolbox 비교 결과가 현행보다 낫지 않아도 억지로 교체하지 않는다.
- Phase 7 진입 조건이 없으면 알고리즘 수를 늘리지 않는다.
- Switchable Constraints/DCS optimizer 포크, RBPF posterior-entropy 탐색, multi-timescale dynamic-map estimator는 구현하지 않는다. Huber·frontier-cell gain·map lifecycle 실험을 이들 알고리즘 구현으로 포장하지 않는다.
- camera texture·조명 randomization은 2D LiDAR 중심 실험에 직접 적용하지 않는다. Sim-to-Real randomization은 실차에서 관측한 센서·운동·시간·마찰 파라미터로 제한한다.
- 필수 결과가 재현되고 held-out 한계까지 설명되면 종료한다. 추가 튜닝 횟수 자체를 성과로 삼지 않는다.

## 10. 계획 자체의 검증 절차

1. 이 문서에 인용한 파일과 라인이 실제 존재하는지 확인한다.
2. 기존 SO-101 Phase 0B, JD-AMR autonomous mapping PRD, SLAM handoff의 gate가 누락되지 않았는지 대조한다.
3. 현재 bag을 `ros2 bag info`로 다시 읽어 duration/topic/count를 확인한다.
4. 현재 navigation package를 fresh `colcon test`해 Phase 0 baseline을 고정한다.
5. 별도 verifier가 저장소 경계, 지표의 유효성, 단계 의존성, 수치화된 acceptance criteria를 검토한다.
