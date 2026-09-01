# JD-AMR SLAM 포트폴리오 재개 가이드

이 문서는 SLAM 포트폴리오 작업을 다음 세션에서 바로 이어가기 위한 진입점이다. 전체 요구사항과 수치 기준의 정본은 [SLAM 포트폴리오 PRD](.omx/plans/prd-jdamr-slam-portfolio.md)다.

## 현재 상태

- 계획 수립과 논문 대조는 완료됐고 `57ffd92` 커밋에 기록돼 있다.
- Phase 0 오프라인 평가 기준선과 Phase 1 실제 복도 Keepout·TF replay guard 사전구성은 2026-09-01에 구현·검증됐다. `jdamr_cube_navigation` 결과는 120 tests, 0 errors, 0 failures, 1 skipped다.
- 평가 계약, dataset/map/calibration registry, MCAP writer 설정, QoS override, 무결성 검사 도구는 `jdamr_cube_navigation/evaluation/`에 있다.
- 기존 G4 bag은 63,955개 메시지를 끝까지 읽고 해시와 토픽 수를 고정했지만 진단 reference일 뿐 새 합격 표본이 아니다. chunk CRC, 기록·재생 QoS, drop counter, 리프트 전 cutoff가 부족하다.
- SO-101 PRD는 Architect 승인 상태지만 구현 시작 전이라고 선언한다. 동시에 미추적 `mobile_mission.py`가 있어 이 차이를 읽기 전용 감사 결과로 남겼고 SO-101 파일은 수정하지 않았다.
- 새 실차 3회 데이터 수집은 시작하지 않았고 `real_motion_authorized: false`다.
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

## 완료한 Phase 0

Phase 0의 목적은 알고리즘을 바꾸는 것이 아니라 이후 모든 비교가 재현되도록 기준선을 봉인하는 것이다.

완료한 항목은 다음과 같다.

1. 기존 사용자 변경을 건드리지 않고 `frontier_explorer.py`의 PEP257 D213 한 건만 수정했다.
2. 다음 평가 골격을 만들고 설치 패키지에 포함했다.
   - `jdamr_cube_navigation/evaluation/experiment_manifest.schema.json`
   - `jdamr_cube_navigation/evaluation/datasets.yaml`
   - `jdamr_cube_navigation/evaluation/map_registry.yaml`
   - `jdamr_cube_navigation/evaluation/calibration_registry.yaml`
   - `jdamr_cube_navigation/evaluation/mcap_writer_options.yaml`
   - `jdamr_cube_navigation/evaluation/qos_overrides.yaml`
   - `jdamr_cube_navigation/evaluation/phase0_status.yaml`
   - `jdamr_cube_navigation/evaluation/inspect_mcap.py`
   - `jdamr_cube_navigation/evaluation/README.md`
3. 기존 G4 reference bag을 checksum, duration, topic count와 함께 `DIAGNOSTIC_ONLY`로 등록했다.
4. MCAP 전체 메시지를 CRC 검증 모드로 읽고 summary CRC와 index는 유효하지만 chunk CRC와 data-section CRC가 없음을 분리해 기록했다.
5. 다음 수집부터 chunk/data/summary CRC와 index를 남기고, 기록·재생 QoS 6개를 같은 파일로 고정하도록 했다.
6. LiDAR와 IMU의 static TF checksum, timestamp source, 검증 상태를 등록하고 `UNVERIFIED` 상태의 fusion을 차단했다.
7. SO-101 PRD와 미추적 미션 코드의 상태 차이를 읽기 전용으로 감사했다.
8. Python `mcap==1.4.0`과 압축 모듈을 평가 전용 디렉터리에 고정 설치하고 JSON Schema, ROS 2 Jazzy QoS 파서, 패키지 설치 레이아웃, 전체 패키지 테스트를 통과했다.

재검증할 때는 선택 패키지만 다시 빌드·테스트한다. 기존 사용자 빌드가 섞인 workspace 전체의 `build/`, `install/`, `log/`를 삭제하지 않는다.

```bash
cd "$HOME/jdamr_cube_ws"
source /opt/ros/jazzy/setup.bash
python3 -m pip install --disable-pip-version-check --no-input --upgrade \
  --requirement src/jdamr_cube_ros/jdamr_cube_navigation/evaluation/requirements.txt \
  --target "$HOME/.local/share/jdamr-slam-eval/python"
colcon build --packages-select jdamr_cube_navigation
source install/setup.bash
colcon test --packages-select jdamr_cube_navigation
colcon test-result --test-result-base build/jdamr_cube_navigation --verbose
PYTHONNOUSERSITE=1 PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
  python3 src/jdamr_cube_ros/jdamr_cube_navigation/evaluation/inspect_mcap.py \
  "$HOME/jdamr_artifacts/g4_userloop_reset_20260824T175151/g4_userloop_reset_20260824T175151_0.mcap"
```

현재 Phase 0 판정은 `OFFLINE_READY`다.

- navigation package: 120 tests, 0 errors, 0 failures, 1 skipped
- G4 diagnostic bag: 63,955 messages, SHA-256 `9525afb5d693e63c9ff07541e761aca6f196b69374634d714d49142028cea6d6`
- QoS override: ROS 2 Jazzy 파서에서 6개 profile 통과
- 설치 레이아웃: evaluation 파일 10개 확인
- 평가 의존성: `mcap==1.4.0`, `lz4==4.4.5`, `zstandard==0.25.0`을 별도 target에 고정하고 user site를 끈 상태에서 bag 검사 확인
- protocol-qualified 데이터의 필수 provenance가 `UNKNOWN`이면 승격 차단
- diagnostic-only 데이터는 미상 값을 명시한 채 원인 분석에만 사용
- published map은 source bag, backend/config hash, quality report, 수동 검토를 모두 요구
- `UNOBSERVABLE` 또는 미검증 LiDAR-IMU 조합의 fusion 실행 차단

이 판정은 실차 안전이나 센서 보정 완료를 뜻하지 않는다. 실제 이동 권한은 계속 false이며 Phase 1은 운영자가 로봇 옆에서 비상 정지를 확보한 뒤 시작한다.

## Phase 0 이후 순서

1. 같은 복도 P-loop를 새 규약으로 실차 3회 수집한다. 저장 지도+AMCL+Keepout은 안전한 반복 경로에만 사용하고 live mapping은 끈다. raw bag은 `offline_replay_guard.launch.py`로 격리 domain의 빈 SLAM 상태에서 재생하며 `/map`, 이동 명령, AMCL `map→odom`은 입력에서 제외한다.
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

> `$HOME/jdamr_cube_ws/src/jdamr_cube_ros/README_SLAM_PORTFOLIO.md`, `jdamr_cube_navigation/evaluation/README.md`, `.omx/plans/prd-jdamr-slam-portfolio.md`를 읽고 Phase 1 동일 복도 3회 수집의 비주행 preflight부터 검증해줘. 실제 이동은 내가 로봇 옆에서 비상 정지를 확보했다고 명시한 뒤에만 진행해.
