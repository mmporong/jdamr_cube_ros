# JD-AMR SLAM 포트폴리오 재개 가이드

이 문서는 SLAM 포트폴리오 작업을 다음 세션에서 바로 이어가기 위한 진입점이다. 전체 요구사항과 수치 기준의 정본은 [SLAM 포트폴리오 PRD](.omx/plans/prd-jdamr-slam-portfolio.md)다.

## 현재 상태

`corridor_localdds_armed_20260904T152036`으로 복도 왕복을 마쳤다. 저장 지도와 Keepout을
사용해 20개 목표를 모두 통과했고, Nav2 복구 동작 없이 출발점으로 돌아왔다. 사전 계획은
77.278m, 기록된 AMCL 경로는 77.090m였다.

이전 주행은 네 번째 목표 앞에서 sensor와 TF가 함께 멈췄다. 무선 association은 살아
있었지만 노트북의 ROS 노드가 파이의 raw sensor를 구독하면서 제어 경로까지 무선 DDS에
묶여 있었다. 로봇의 DDS discovery를 `LOCALHOST`로 제한하자 `/scan` 최대 공백이
10.750초에서 0.112초로 줄었고, 같은 복도를 끝까지 주행했다. `/odom`, IMU,
`map -> odom` TF도 긴 정체가 사라졌다.

성공 MCAP은 130,798개 메시지를 담고 있으며 chunk, data-section, summary CRC와 인덱스
검사를 통과했다. recorder transport-loss 카운터가 없고 자원 TSV의 마지막 경계 표본이
한 개 빠져 원장 등급은 `PARTIAL_SUCCESS`다. 자율주행 완주 판정과 데이터 provenance
등급을 한 문장으로 뭉치지 않는다.

경로와 Keepout, 실제 AMCL 궤적, 전후 센서 공백 비교, 주행 GIF와 H.264 영상은 저장소에
생성했다. 같은 스크립트로 원본 MCAP에서 다시 만들 수 있다. 다음 작업은 새 주행이 아니라
이 77m bag의 Cartographer/SLAM Toolbox 격리 재생과 센서 노이즈 분석이다.

![복도 왕복 자율주행 완주](jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036/success_card.png)

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
5. 다음 수집부터 chunk/data/summary CRC와 index를 남기고, 처음 6개였던 기록·재생 QoS
   정본을 현재 발행 계약 12개까지 같은 파일에 고정했다.
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

현재 Phase 0 판정은 `ONBOARD_STATIC_READY`다. 수정본의 온보드 비주행 실행에서
위치추정·Keepout·전체 경로 planning-only·자원 계측·기록 마감을 다시 확인했다.

- navigation package: 214 passed, 1 skipped
- Pi target regression: 78 passed
- onboard static run: `corridor_static_20260904T140326`, resource gate PASS,
  25,375 messages, nonzero `/cmd_vel` absent
- G4 diagnostic bag: 63,955 messages, SHA-256 `9525afb5d693e63c9ff07541e761aca6f196b69374634d714d49142028cea6d6`
- QoS override: ROS 2 Jazzy 파서에서 12개 profile 통과
- 설치 레이아웃: evaluation 파일 10개 확인
- 평가 의존성: MCAP reader와 ROS 2 decoder, 압축·수치·미디어 라이브러리 8개를 별도
  target에 고정하고 user site를 끈 상태에서 bag 검사와 미디어 생성을 확인
- protocol-qualified 데이터의 필수 provenance가 `UNKNOWN`이면 승격 차단
- diagnostic-only 데이터는 미상 값을 명시한 채 원인 분석에만 사용
- published map은 source bag, backend/config hash, quality report, 수동 검토를 모두 요구
- `UNOBSERVABLE` 또는 미검증 LiDAR-IMU 조합의 fusion 실행 차단

이 판정은 실차 안전이나 센서 보정 완료를 뜻하지 않는다. 실제 이동 권한은 계속 false이며 다음 단계는 운영자가 로봇 옆에서 물리 전원을 즉시 차단할 수 있을 때만 진행한다.

## 전체 실행 순서와 통과 조건

1. **로컬 정적 검증, 완료:** 설정과 회귀 테스트, 비주행 기동, lifecycle, Keepout,
   planning-only와 기록 마감을 확인했다.
2. **복도 왕복, 완료:** 저장 지도와 AMCL, Keepout으로 20개 목표를 자율주행했다. 같은 날
   실패 기록과 성공 기록의 sensor/TF 공백을 같은 계산식으로 비교했다.
3. **증거와 미디어, 완료:** 해시와 무결성을 원장에 등록하고 CSV, JSON, PNG, GIF, MP4를
   원본 MCAP에서 생성했다.
4. **오프라인 2D SLAM 비교:** 성공 bag을 Cartographer와 SLAM Toolbox에 같은 조건으로
   재생한다. 폐루프 오차와 지도 형태, 처리시간을 비교한다.
5. **센서와 TF 강건성:** 실차 데이터에서 scan, odom, IMU, timestamp jitter 분포를
   계산한다. 그 범위로 시뮬레이션에 noise, dropout, wheel slip, 지연을 하나씩 넣는다.
6. **위치추정과 탐색 선택:** AMCL과 SLAM Toolbox localization, kidnapped-robot recovery,
   frontier 정책을 비교한다. planner나 controller는 재현된 실패 근거가 있을 때만 바꾼다.
7. **Visual SLAM 선택 트랙:** 카메라 timestamp, intrinsic, extrinsic과 이미지 기록이
   확보된 뒤 2D LiDAR SLAM과 별도 실험으로 진행한다.
8. **포트폴리오 반영:** 성공 결과와 실패 원인, 전후 수치, 재현 명령을 묶는다. SO-101에는
   SLAM 코드를 복제하지 않고 localization, TF, 도착 오차 계약만 연결한다.

현재 위치는 **1~3 완료, 다음은 4 오프라인 2D SLAM 비교**다. 반복 실주행은 성공률을
수치로 말해야 할 때만 추가한다.

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
프로세스별 귀속값은 참고치로만 남긴다. 수정된 process coverage 게이트는
`corridor_static_20260904T140326` 비주행 소크에서 다시 통과했다.

게이트를 실측 가능한 네 항목으로 바꿨고, 문서상의 수동 기준이 아니라 코드가 판정한다.

| 항목 | 기준 | 2026-09-04 수정본 비주행 실측 |
|---|---|---|
| 전체 시스템 지속 CPU (p90) | 코어 예산의 75% 이내 (4코어 = 300%) | 296% PASS |
| thermal throttle | `0x0` | PASS |
| 최고 온도 | 75도 이하 (소프트 스로틀 80도 대비 여유) | 72.5도 PASS |
| 프로세스 coverage | recorder+Nav2 필수 집합이 워밍업 뒤 60초 연속, 최대 샘플 간격 10초 | 33/33, 165초, 최대 5.4초 PASS |

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

성공 bag을 두 SLAM backend에 격리 재생하고, 같은 주행에서 센서 노이즈 분포를
계산한다. 새 실차 주행은 필요하지 않다.

## 포트폴리오로 보여줄 수 있는 것

면접에서 화면을 열어 설명할 수 있는 단위로 정리한다. 전부 이 저장소 안에 있고
회귀 테스트가 붙어 있다.

### 그림

| 파일 | 내용 |
|---|---|
| `evaluation/media/.../success_card.png` | README와 포트폴리오 대표 이미지 |
| `evaluation/media/.../route_evidence.png` | 저장 지도, 금지구역, 계획과 실제 AMCL 경로 |
| `evaluation/media/.../continuity_comparison.png` | DDS 격리 전후 sensor/TF 최대 공백 |
| `evaluation/media/.../telemetry.png` | 목표 진행, 배터리, AMCL 공분산, stream gap |
| `evaluation/media/.../corridor_roundtrip.gif` | 실제 AMCL timestamp로 만든 왕복 애니메이션 |
| `evaluation/media/.../corridor_roundtrip.mp4` | 발표 자료용 H.264 영상 |

RViz 화면 캡처 대신 `evaluation/corridor_run_media.py`가 지도와 로그, MCAP에서 다시
그린다. 애니메이션도 실제 AMCL timestamp를 사용한다.

```bash
PYTHONNOUSERSITE=1 \
PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
python3 jdamr_cube_navigation/evaluation/corridor_run_media.py \
  --run-dir "$HOME/jdamr_artifacts/corridor_localdds_armed_20260904T152036" \
  --compare-run-dir "$HOME/jdamr_artifacts/corridor_roundtrip_20260904T142436" \
  --output-dir \
    jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036
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
- 원격 ROS 참여자가 raw sensor를 구독해 파이의 제어 경로까지 무선 DDS에 묶였다. RF
  association은 유지됐지만 scan과 TF가 함께 멈췄다. discovery를 `LOCALHOST`로 제한한
  뒤 같은 복도를 완주했고 stream gap을 전후 기록으로 남겼다.

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

> `$HOME/jdamr_cube_ws/src/jdamr_cube_ros/README_SLAM_PORTFOLIO.md`와 `$HOME/jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_navigation/evaluation/20260904_CORRIDOR_LOCALDDS_SUCCESS.md`를 읽고, `corridor_localdds_armed_20260904T152036`을 Cartographer와 SLAM Toolbox에 격리 재생해. 같은 bag의 센서 노이즈와 timestamp jitter도 계산하고 결과를 기존 포트폴리오 미디어와 연결해.
