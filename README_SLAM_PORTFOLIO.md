# JD-AMR SLAM 포트폴리오 재개 가이드

이 문서는 SLAM 포트폴리오 작업을 다음 세션에서 바로 이어가기 위한 진입점이다. 전체 요구사항과 수치 기준의 정본은 [SLAM 포트폴리오 PRD](.omx/plans/prd-jdamr-slam-portfolio.md)다.

## 현재 상태

- 계획 수립과 논문 대조는 완료됐고 `57ffd92` 커밋에 기록돼 있다.
- SLAM 포트폴리오용 평가 코드와 새 실차 3회 데이터 수집은 아직 시작하지 않았다.
- 2026-08-31 기준 `jdamr_cube_navigation` 테스트는 98개 중 96개 통과, 1개 건너뜀, 1개 실패였다. 실패는 `jdamr_cube_navigation/jdamr_cube_navigation/frontier_explorer.py`의 PEP257 D213 한 건이다.
- 기존 G4 bag은 진단 reference일 뿐 새 합격 표본이 아니다. 로봇을 들어 올린 뒤의 scan/pose 급변 구간이 포함돼 있다.
- 계획 문서가 인용하는 일부 기존 `.omx` 계획·테스트 명세는 아직 Git 미추적 상태다. 해당 파일을 임의로 함께 스테이징하지 않는다.

## 저장소 경계

| 저장소 | 담당 범위 |
|---|---|
| 이 JD-AMR 저장소 | 센서 계약, Cartographer·SLAM Toolbox, 저장 지도 localization, ATE/RPE/NEES 평가, frontier, Nav2 fault injection, Sim-to-Real 검증 |
| `so101-mobile-manipulation` | localization/TF 상태와 Nav 도착 오차 소비, 상태가 불량할 때 팔 동작 차단 |

SLAM backend, launch, config, 평가 코드를 SO-101 저장소에 복제하지 않는다. SO-101 연계는 최종 Phase 9에서 결과 계약만 반영한다.

## 논문에서 실제로 반영할 것

- Hess: Cartographer의 nonlocal loop constraint를 TP·FP·누락으로 감사한다.
- Sturm: 시뮬레이션 ground truth와 추정 궤적의 시간 정합, ATE/RPE 규약을 고정한다.
- Barczyk: 반복·대칭 복도 오차를 진행축, 횡축, yaw로 나눠 본다.
- Thrun·Fox: 현재 recovery가 꺼진 AMCL과 recovery 후보를 kidnapped-robot fault로 비교한다.
- Yamauchi·Stachniss: distance-only frontier를 기준선으로 두고, 현재 unknown-cell count는 posterior entropy가 아닌 `frontier-cell-gain proxy`로 평가한다.
- Biber·Duckett: 일시 장애물과 구조 변경을 구분해 map registry 상태 전이를 검증한다.
- Peng: 실차 bag의 quantile로 센서·wheel slip·시간 지연·TF·마찰 randomization 범위를 정한다.
- Furgale·Lv: timestamp와 extrinsic을 함께 관리하고, 관측할 수 없는 calibration은 `UNOBSERVABLE`로 차단한다.

Switchable Constraints/DCS optimizer, RBPF posterior-entropy 탐색, multi-timescale dynamic-map estimator는 이번 범위에서 구현하지 않는다.

## 다음 세션에서 가장 먼저 할 일: Phase 0

Phase 0의 목적은 알고리즘을 바꾸는 것이 아니라 이후 모든 비교가 재현되도록 기준선을 봉인하는 것이다.

1. 작업 트리와 현재 브랜치를 확인하고 기존 사용자 변경을 분리한다.
2. `jdamr_cube_navigation`을 빌드하고 테스트해 기존 PEP257 실패를 재현한다.
3. 해당 실패만 수정한 뒤 테스트 failure를 0으로 만든다.
4. 다음 평가 골격을 만든다.
   - `jdamr_cube_navigation/evaluation/experiment_manifest.schema.json`
   - `jdamr_cube_navigation/evaluation/datasets.yaml`
   - `jdamr_cube_navigation/evaluation/map_registry.yaml`
   - `jdamr_cube_navigation/evaluation/calibration_registry.yaml`
   - `jdamr_cube_navigation/evaluation/README.md`
5. 기존 G4 reference bag을 checksum, duration, topic count와 함께 진단 데이터로만 등록한다.
6. MCAP writer profile, CRC/index 상태, record/playback QoS와 topic count를 기록한다. bag에서 확인할 수 없는 값은 추측하지 않고 `UNKNOWN`으로 기록해 Phase 0 통과를 막는다. NIC·driver·kernel drop counter도 수집 가능 여부와 값을 함께 남긴다.
7. `$HOME/so101-mobile-manipulation/.omx/plans/prd-so101-autonomy-pick-place.md`의 “구현 시작 전” 상태와 현재 `$HOME/so101-mobile-manipulation/mobile_mission.py`·미션 상태 열거를 읽기 전용으로 대조한다. 이 단계에서는 SO-101 파일을 수정하거나 스테이징하지 않는다.
8. 같은 dataset metadata로 manifest를 다시 생성·검증했을 때 checksum과 validation 결과가 같은지 확인한다.

여기서 fresh build/test는 현재 세션에서 선택 패키지를 다시 빌드·테스트하고 그 패키지의 result base만으로 판정한다는 뜻이다. 기존 사용자 빌드가 섞인 workspace 전체의 `build/`, `install/`, `log/`를 삭제하지 않는다.

```bash
cd "$HOME/jdamr_cube_ws"
source /opt/ros/jazzy/setup.bash
colcon build --packages-select jdamr_cube_navigation
source install/setup.bash
colcon test --packages-select jdamr_cube_navigation
colcon test-result --test-result-base build/jdamr_cube_navigation --verbose
ros2 bag info "$HOME/jdamr_artifacts/g4_userloop_reset_20260824T175151"
sha256sum "$HOME/jdamr_artifacts/g4_userloop_reset_20260824T175151/g4_userloop_reset_20260824T175151_0.mcap"
```

Phase 0 통과 조건은 다음과 같다.

- navigation package 테스트 failure 0
- 모든 dataset에 경로, SHA-256, topic/count, duration, sensor/backend 정보 존재
- MCAP CRC/index, writer profile, record/playback QoS가 검증됐으며 topic count와 수집 가능한 drop counter가 일치함. 확인 불가능한 필수 값은 `UNKNOWN` 실패로 처리
- published map이 source bag, backend/config hash, quality report로 역추적 가능
- `UNOBSERVABLE` 또는 미검증 LiDAR-IMU 조합의 fusion 실행 차단
- SO-101 문서 상태와 실제 미션 코드의 차이가 읽기 전용 대조표로 기록됨
- 기존 사용자 변경이 이번 커밋에 포함되지 않음

이 조건을 만족하기 전에는 Cartographer 파라미터 튜닝이나 실차 3회 수집으로 넘어가지 않는다.

## Phase 0 이후 순서

1. 같은 복도 P-loop를 새 규약으로 실차 3회 수집한다.
2. 시뮬레이션 ground-truth 기반 ATE/RPE/NEES와 loop audit 도구를 만든다.
3. Cartographer와 SLAM Toolbox를 동일 bag으로 비교한다.
4. LiDAR noise/dropout, odometry, Huber, 검증된 IMU를 한 변수씩 비교한다.
5. 실차 분포 기반 Sim-to-Real randomization을 적용한다.
6. AMCL·SLAM Toolbox localization과 AMCL recovery를 비교한다.
7. frontier 정책 3종을 비교하고, planner/controller는 문제가 재현될 때만 비교한다.
8. 동일 복도와 held-out 환경에서 통합 bundle을 S0~S5 순서로 검증한다.
9. 결과 보고서와 SO-101 소비 계약을 연결한다.

## 실차 작업 전 중단선

- 실차 이동은 작업자가 로봇 옆에서 물리 전원을 즉시 차단할 수 있을 때만 수행한다.
- Phase 0은 offline 작업이므로 실차를 움직이지 않는다.
- Phase 1을 시작할 때도 지도 저장 또는 SLAM 정지를 먼저 완료하고 나서 로봇을 들어 올린다.
- 실차에는 전체 ground truth가 없으므로 폐루프 오차를 ATE/RPE라고 부르지 않는다.

## 다음 세션 재개 요청문

다음과 같이 요청하면 된다.

> `$HOME/jdamr_cube_ws/src/jdamr_cube_ros/README_SLAM_PORTFOLIO.md`와 `.omx/plans/prd-jdamr-slam-portfolio.md`를 읽고, 기존 사용자 변경을 보존하면서 Phase 0만 구현·검증·커밋해줘. 실차는 움직이지 마.
