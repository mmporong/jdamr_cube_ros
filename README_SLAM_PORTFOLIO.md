# JD-AMR SLAM 포트폴리오 재개 가이드

이 문서는 SLAM 포트폴리오 작업을 다음 세션에서 바로 이어가기 위한 진입점이다. 전체 요구사항과 수치 기준의 정본은 [SLAM 포트폴리오 PRD](.omx/plans/prd-jdamr-slam-portfolio.md)다.

## 현재 상태

- 오프라인 평가 기준선, Keepout, TF replay guard, 데이터·지도·보정 registry와 MCAP
  무결성 검사는 구현돼 있다.
- 프로토콜 적격 80m 왕복 표본은 아직 0회다. 기존 실차 bag은 원인 분석과 오프라인
  SLAM 입력에는 쓸 수 있지만, 미완주 또는 기록 손실 때문에 최종 성능 표본은 아니다.
- 최신 `corridor_roundtrip_20260904T115600`은 MCAP 263,763개 메시지와 모든 CRC를
  통과했다. 세 번째 목표 도중 실제 `x=7.079m`에서 Nav2가 최신 sensor/TF를 사용하지
  못하고 lifecycle이 무너졌으며, raw scan·odom·IMU는 그 뒤에도 계속 기록됐다.
- 이 기록은 주행 종료 뒤 recorder가 31분 더 살아 있어 전체 duration과 평균 rate를
  성능 지표로 쓰지 않는다. 고아 recorder는 해당 PID만 종료해 metadata를 마감했고,
  이후 실행기는 자신이 시작한 프로세스 그룹 전체를 추적하도록 고쳤다.
- 과거 자원 계측기는 rosbag의 토픽 인자를 실행 파일로 오인하고 합성 Nav2 container를
  누락했다. 당시 프로세스별 CPU 결론은 폐기했으며, 수정본은 원문 command와 process
  coverage가 없으면 PASS를 거부한다.
- 현재 온보드 Nav2는 합성 컨테이너와 별도 lifecycle·graph liveness 감시를 함께 쓴다.
  composition은 확인됐지만 intra-process 통신은 켜지 않았으므로 zero-copy라고 설명하지
  않는다.
- AMCL freshness는 출발 시에만 요구한다. 복도 BT만 `PositionGoalChecker`를 선택해
  home 도달을 위치로 판정하고, 일반 자율주행의 방향 판정은 유지한다.
- 고주기 recorder 구독은 best-effort로 낮췄지만 역압이 원인이었다는 가설은 아직
  recording A/B로 검증하지 않았다. DDS 단계에서 빠진 메시지를 단순 메시지 수만으로
  검출할 수도 없으므로, 적격 주행 전에는 토픽별 gap/rate 게이트도 추가해야 한다.
  근본 원인은 계속 미규명 상태다.
- 현재 로컬 검증은 `198 passed, 1 skipped`다. 수정본의 파이 비주행 기동과 실주행은
  아직 수행하지 않았다.
- SO-101 PRD는 Architect 승인 상태지만 구현 시작 전이라고 선언한다. 동시에 미추적 `mobile_mission.py`가 있어 이 차이를 읽기 전용 감사 결과로 남겼고 SO-101 파일은 수정하지 않았다.
- 현재 세션은 작업자가 로봇 옆에 있다고 확인되지 않았으므로 `real_motion_authorized: false`다.
- 계획 문서가 인용하는 일부 기존 `.omx` 계획·테스트 명세는 아직 Git 미추적 상태다. 해당 파일을 임의로 함께 스테이징하지 않는다.

## 저장소 경계

| 저장소 | 담당 범위 |
|---|---|
| 이 JD-AMR 저장소 | 센서 계약, Cartographer·SLAM Toolbox, 저장 지도 localization, ATE/RPE/NEES 평가, frontier, Nav2 fault injection, Sim-to-Real 검증 |
| `so101-mobile-manipulation` | localization/TF 상태와 Nav 도착 오차 소비, 상태가 불량할 때 팔 동작 차단 |

SLAM backend, launch, config, 평가 코드를 SO-101 저장소에 복제하지 않는다. SO-101 연계는 최종 포트폴리오 승격 단계에서 결과 계약만 반영한다.

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

현재 Phase 0 판정은 `OFFLINE_READY`다. 이번 수정본의 온보드 비주행 판정은 아직
재수집 전이므로 기존 `ONBOARD_STATIC_READY`를 승계하지 않는다.

- navigation package: 198 passed, 1 skipped
- G4 diagnostic bag: 63,955 messages, SHA-256 `9525afb5d693e63c9ff07541e761aca6f196b69374634d714d49142028cea6d6`
- QoS override: ROS 2 Jazzy 파서에서 6개 profile 통과
- 설치 레이아웃: evaluation 파일 10개 확인
- 평가 의존성: `mcap==1.4.0`, `lz4==4.4.5`, `zstandard==0.25.0`을 별도 target에 고정하고 user site를 끈 상태에서 bag 검사 확인
- protocol-qualified 데이터의 필수 provenance가 `UNKNOWN`이면 승격 차단
- diagnostic-only 데이터는 미상 값을 명시한 채 원인 분석에만 사용
- published map은 source bag, backend/config hash, quality report, 수동 검토를 모두 요구
- `UNOBSERVABLE` 또는 미검증 LiDAR-IMU 조합의 fusion 실행 차단

이 판정은 실차 안전이나 센서 보정 완료를 뜻하지 않는다. 실제 이동 권한은 계속 false이며 다음 단계는 운영자가 로봇 옆에서 물리 전원을 즉시 차단할 수 있을 때만 진행한다.

## 전체 실행 순서와 통과 조건

1. **로컬 정적 검증(완료):** YAML·BT·셸 구문, 단위 테스트, 패키지 빌드와 전체
   테스트를 통과하고 과거 결함 TSV가 `process_coverage=FAIL`로 재판정되는지 확인한다.
2. **파이 비주행 기동:** 지도·Keepout 해시, lifecycle, 단일 `/cmd_vel` 소유자와 함께
   `nav2_container`·recorder 원문 command가 TSV에 남는지 확인한다. 종료 뒤 잔류
   프로세스 0과 `metadata.yaml` 생성을 확인한다.
3. **짧은 위치 왕복:** 최종 yaw 정렬 없이 위치 도달로 끝나는지, AMCL 토픽 정지만으로
   정상 주행을 취소하지 않는지 확인한다.
4. **recording A/B:** 같은 경로와 실행 구조에서 기록 조건만 바꿔 TF 처리 지연,
   container CPU, lifecycle heartbeat를 비교한다.
5. **80m급 자율 왕복 3회:** 저장 지도+AMCL+Keepout으로 한 번 완주한 뒤 같은 프로토콜을
   두 번 반복한다. 수동으로 세 바퀴를 도는 방식이 아니다.
6. **오프라인 2D SLAM 비교:** 적격 raw bag을 Cartographer와 SLAM Toolbox에 동일하게
   재생하고 폐루프 오차·loop audit·처리시간을 비교한다.
7. **센서·TF 강건성 및 Sim-to-Real:** 실차 분포로 노이즈·dropout·wheel slip·시간
   지연·extrinsic 오차를 한 번에 하나씩 주입하고, 시뮬레이션 ground truth로
   ATE/RPE/NEES를 계산한다.
8. **위치추정·탐색 선택:** AMCL과 SLAM Toolbox localization, kidnapped-robot recovery,
   frontier 정책을 비교한다. planner/controller 교체는 재현된 실패 근거가 있을 때만 한다.
9. **Visual SLAM 선택 트랙:** 카메라 timestamp·intrinsic·extrinsic과 이미지 기록이
   확보된 뒤 2D LiDAR SLAM과 별도 실험으로 수행한다.
10. **포트폴리오 승격:** 동일 복도와 held-out 결과, 실패 사례, 정량 지표, 재현 명령을
    묶는다. SO-101에는 SLAM 코드를 복제하지 않고 localization/TF/도착 오차 계약만
    연결한다.

현재 위치는 **1 완료, 2 미수행**이다. 기존 부분 복귀와 실패 bag은 진단·오프라인 SLAM
입력으로 보존하되, 새 코드의 합격 증거로 소급 사용하지 않는다.

## 기존 정비 기록과 2026-09-04 정정

실주행 없이 처리한 항목이다. 근거는 모두 저장소 안 파일과 회귀 테스트다.

### recorder transport loss 게이트

- 기록 토픽은 11개인데 `evaluation/qos_overrides.yaml`에는 6개만 있었다. 나머지는
  기본 depth 10으로 떨어졌고, 50Hz `/imu/data_raw` 기준 0.2초 버퍼다. 2026-09-01
  소크의 load1 9.85 구간 정지 시간을 견디지 못한다.
- 기록 계약을 `jdamr_cube_navigation/onboard_recording.py` 한 곳으로 모으고 launch가
  이를 import한다. reliable 토픽은 `rate x 2초`, best-effort 고주기 토픽은
  `rate x 0.5초`로 계산한다.
- bag 3종(`corridor_keepout_roundtrip_20260901T150446`,
  `onboard_minimal_soak_20260901T161338`, `static_repeat_stop_fix_20260901T174200`)의
  metadata를 대조해 발행자가 실제로 제시하는 QoS를 확인했다. `/imu/data_raw`는
  **best_effort**, `/amcl_pose`는 **transient_local**이다. reliable 오버라이드를
  걸었다면 구독이 매칭되지 않아 IMU가 통째로 비었을 것이다. 계약과 회귀 테스트가
  이 불일치를 막는다.
- launch에 `--disable-keyboard-controls`를 추가했다. launch 아래 recorder는 터미널을
  갖지 않으므로 키 입력 폴링 스레드는 부하만 더한다.

### 파이 load 게이트

- 전역 코스트맵은 894x212 = 189,528셀이다. `always_send_full_costmap: True`와
  `publish_frequency: 1.0`은 이 전체 격자를 초당 한 번 reliable DDS로 직렬화한다.
  노트북 RViz는 view-only이고 Global Costmap 표시는 기본 OFF다.
- 두 코스트맵 모두 `always_send_full_costmap: False`로 바꿔 변경분만 보내고,
  전역 발행 주기를 0.5Hz, 지역 발행 주기를 1.0Hz로 낮췄다. 계획에 쓰는
  `update_frequency`는 건드리지 않았다.
- AMCL 입자 수와 게이트 임계는 바꾸지 않았다. 위치추정 품질을 CPU 때문에 흔들지
  않는다.

### 부하 귀속 측정

- 이전 소크는 `load1` 총량만 남겨 9.85가 어느 프로세스에서 왔는지 근거가 없었다.
- `jdamr_cube_navigation/soak_metrics.py`(실행 이름 `soak_metrics`)가 `/proc`을 직접
  읽어 전체 시스템 CPU와 프로세스별 CPU·스레드·RSS·원문 command를 TSV로 남긴다.
  전체 CPU는 자원 게이트에, 프로세스 수치는 부하 귀속에 쓴다.
- 이전 분류기는 토픽 인자까지 정규식으로 검색해 rosbag recorder를
  `collision_monitor`로 잘못 붙였고, 합성 `nav2_container`는 누락했다. 수정본은 실행
  파일 토큰으로 분류한다. recorder와 합성 컨테이너 또는 독립 Nav2 필수 집합이
  워밍업 뒤 60초 이상 같은 샘플에 계속 있어야 하고 인접 샘플 간격도 10초를 넘으면
  안 된다. 프로세스가 모두 사라진 시점도 sentinel 행으로 남겨
  `process_coverage=FAIL`을 낸다. 따라서 이전 11:56 TSV의
  `CPU 61~146%`는 폐기한다.

```bash
ros2 run jdamr_cube_navigation soak_metrics \
  --output "$HOME/jdamr_artifacts/<run_id>.per_process.tsv" \
  --duration 330 --interval 5
```

### 부하 게이트 재정의 (2026-09-03)

기존 재측정에서는 load1과 프로세스 CPU 합계가 서로 다른 추세를 보였고, 이를 근거로
`load1 < 4`를 단독 게이트에서 제외했다. 이 설계 판단은 유지한다. 다만 당시 분류기는
원문 command를 남기지 않았고 recorder 오분류가 확인됐으므로, 과거의 **203%**와
프로세스별 귀속값은 참고치로만 남긴다. 수정된 process coverage 게이트로 비주행 소크를
다시 통과하기 전에는 CPU 여유가 검증됐다고 주장하지 않는다.

게이트를 실측 가능한 네 항목으로 바꿨고, 문서상의 수동 기준이 아니라 코드가 판정한다.

| 항목 | 기준 | 2026-09-03 실측 |
|---|---|---|
| 전체 시스템 지속 CPU (p90) | 코어 예산의 75% 이내 (4코어 = 300%) | 과거 수치 무효, 재측정 필요 |
| thermal throttle | `0x0` | `0x0` PASS |
| 최고 온도 | 75도 이하 (소프트 스로틀 80도 대비 여유) | 69.6도 PASS |
| 프로세스 coverage | recorder+Nav2 필수 집합이 워밍업 뒤 60초 연속, 최대 샘플 간격 10초 | 과거 command 부재, 재측정 필요 |

순간 최대와 프로세스별 합계는 진단용으로 함께 출력하되, 판정은 전체 시스템 CPU의
p90 지속 부하를 사용한다. 워밍업 제외, 단위 변환, 프로세스 전멸 샘플 기록은 계산
주석 대신 단위 테스트로 고정했다. load1은 계속 기록하되 판정에서는 뺐다.

```bash
# 소크와 동시에 측정하며 판정
ros2 run jdamr_cube_navigation soak_metrics \
  --output "$HOME/jdamr_artifacts/<run_id>.per_process.tsv" \
  --duration 330 --interval 5

# 기존 TSV 재판정 (다른 기기에서 볼 때는 --cores 로 원본 기기 코어 수를 준다)
ros2 run jdamr_cube_navigation soak_metrics \
  --evaluate "$HOME/jdamr_artifacts/<run_id>.per_process.tsv" --cores 4
```

게이트가 통과해도 실차 이동 권한은 별개다. 작업자가 로봇 옆에서 물리 전원을 즉시
차단할 수 있을 때만 주행한다. 충돌 감시, 배터리 전압, AMCL 공분산 같은 실제 안전
인터록은 그대로 둔다.

### 남은 것

기존 기록 게이트는 통과했다. 수정된 프로세스 coverage를 포함한 부하 게이트는 비주행
소크 재검증이 필요하며, 그 다음이 짧은 위치 왕복과 80m 왕복이다.

## 포트폴리오로 보여줄 수 있는 것

면접에서 화면을 열어 설명할 수 있는 단위로 정리한다. 전부 이 저장소 안에 있고
회귀 테스트가 붙어 있다.

### 그림

| 파일 | 내용 |
|---|---|
| `$HOME/jdamr_artifacts/portfolio/corridor_route_map.png` | 저장 지도 + 계단 금지구역 2곳 + 20 waypoint 경로 |
| `$HOME/jdamr_artifacts/portfolio/corridor_route_with_drive.png` | 위 그림에 2026-09-01 실제 주행 궤적(AMCL) 중첩 |

RViz 화면 캡처 대신 `evaluation/render_route_map.py`로 파일에서 다시 그린다. 세션이
끝나도 커밋 해시만으로 같은 그림을 재생성할 수 있고, 가려진 창이 잘못 캡처되는 문제도
없다.

```bash
python3 evaluation/render_route_map.py \
  --output "$HOME/jdamr_artifacts/portfolio/corridor_route_map.png"
python3 evaluation/render_route_map.py \
  --output "$HOME/jdamr_artifacts/portfolio/corridor_route_with_drive.png" \
  --trajectory-bag "$HOME/jdamr_artifacts/corridor_keepout_onboard_20260901T172141/corridor_keepout_onboard_20260901T172141_0.mcap"
```

### 노드와 도구

| 대상 | 해결한 문제 | 이야기할 거리 |
|---|---|---|
| `frontier_explorer` (1,507줄) + `frontier_core` (560줄) | 자율 탐사 정책 | ROS 의존을 분리해 탐사 로직만 단위 테스트한다. distance-only 기준선과 gain proxy를 비교 대상으로 둔다 |
| `corridor_route` | fail-closed 경로 실행기 | 사전 전체 계획, 배터리·센서 freshness·AMCL 축별 공분산 게이트, 전체 시작은 home 1m 이내, 재개는 첫 잔여 waypoint 6m 이내로 제한한다. AMCL freshness는 출발 시에만 확인하고 최종 도달은 위치로 판정한다 |
| `keepout_zone_capture` + `keepout_mask` | 운영자가 그린 금지구역을 마스크로 | 꼭짓점 순서 자동 보정, 0.55m 팽창, 해시 고정, 연결성 검사 |
| `tf_replay_filter` | 오프라인 재생 시 TF 권한 충돌 | 기록된 AMCL `map→odom`을 제거해 새 mapping backend 하나만 권한자가 되게 한다. 이게 없으면 재생 결과가 기존 지도의 복사본이 된다 |
| `soak_metrics` | 자원 게이트 판정 | 전체 시스템 CPU로 여유를 판정하고 프로세스별 CPU로 원인을 추적한다. 워밍업 뒤 60초 연속·최대 간격 10초 측정이 아니면 PASS를 거부한다 |
| `evaluation/ledger.py` | 증거 원장 | 해시·무결성·게이트 판정을 같은 규칙으로 등록. 게이트 FAIL이면 승격을 거부한다 |
| `evaluation/inspect_mcap.py` | 기록 무결성 | chunk/data/summary CRC와 인덱스를 분리해 확인 |
| `scripts/corridor_preflight.sh` | 출발 전 자동 점검 | 도메인·RViz 순서·lifecycle·금지구역·`/cmd_vel` 소유권·코스트맵·배터리 |
| `test/test_route_clearance.py` | 경로가 복도에 물리적으로 맞는가 | 로봇 없이 지도만으로 내접 반경 침범과 금지구역 진입을 막는다 |

### 이야기로 만들 수 있는 실패 사례

수치와 파일이 함께 남아 있는 것들이다.

- QoS reliability 불일치: `/imu/data_raw`는 best_effort 발행이라 reliable 오버라이드를
  걸면 이후 모든 bag에서 IMU가 0건이 된다. 실측 대조로 막았다.
- BT XML의 `server_timeout`이 `bt_navigator`의 `default_server_timeout`을 덮어써
  파라미터만 올려서는 효과가 없었다.
- 오래 떠 있던 원격 RViz가 `map_server`의 `change_state` 응답을 막아 기동 자체가 실패.
- 경로가 벽에서 0.06m까지 붙어 로봇 발자국이 내접 반경을 침범, 컨트롤러가 출발 직후
  `collision ahead`로 중단.
- `load1 < 4` 게이트가 CPU 포화가 아닌 대기 스레드를 세고 있어 여유 있는 로봇을 막았다.

## 증거 원장

주행·지도 산출물은 `evaluation/ledger.py`로 등록한다. 해시, 토픽별 메시지 수, MCAP
무결성, 자동 게이트 판정이 같은 규칙으로 기록되므로 포트폴리오 주장을 파일까지
되짚을 수 있다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_navigation/evaluation"
export PYTHONNOUSERSITE=1 PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python"

# 주행 기록 등록 (--apply 없으면 dry-run)
python3 ledger.py bag "$HOME/jdamr_artifacts/<run_id>" \
  --role corridor_roundtrip --qualification DIAGNOSTIC_ONLY \
  --outcome '로봇이 실제로 한 일 한 문장' --transport-loss 0 --apply

# 지도 등록
python3 ledger.py map "$HOME/maps/<map>.yaml" \
  --state published --source-run <run_id> --note '설명' --apply
```

`PROTOCOL_QUALIFIED`는 자동 게이트가 전부 통과하고 운영자가 완주를 명시할 때만
부여된다. 게이트가 하나라도 FAIL이면 도구가 승격을 거부한다. 파일이 로봇의 복귀를
증명할 수는 없기 때문이다.

파이 SD카드에만 있던 9건은 2026-09-03에 노트북으로 미러링했고 MCAP 해시 일치를
확인했다.

## 실차 작업 전 중단선

- 실차 이동은 작업자가 로봇 옆에서 물리 전원을 즉시 차단할 수 있을 때만 수행한다.
- Phase 0은 offline 작업이므로 실차를 움직이지 않는다.
- Phase 1을 시작할 때도 지도 저장 또는 SLAM 정지를 먼저 완료하고 나서 로봇을 들어 올린다.
- 실차에는 전체 ground truth가 없으므로 폐루프 오차를 ATE/RPE라고 부르지 않는다.

## 다음 세션 재개 요청문

다음과 같이 요청하면 된다.

> `$HOME/jdamr_cube_ws/src/jdamr_cube_ros/README_SLAM_PORTFOLIO.md`와 `$HOME/jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_navigation/evaluation/20260904_HANDOFF.md`를 읽고, 수정된 프로세스 계측의 비주행 결과부터 확인한 뒤 짧은 위치 왕복과 recording A/B를 순서대로 진행해. 실제 이동은 내가 로봇 옆에서 비상 정지를 확보했다고 명시한 뒤에만 진행해.
