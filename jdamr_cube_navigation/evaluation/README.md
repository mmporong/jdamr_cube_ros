# SLAM 평가 Phase 0

이 디렉터리는 실제 로봇을 움직이기 전에 데이터와 실험 조건을 고정하는 오프라인 게이트다. 여기서 통과했다고 해서 센서 보정이나 실제 주행 안전이 검증된 것은 아니다.

## 구성

- `experiment_manifest.schema.json`: 실험마다 남겨야 할 환경·버전·센서·TF·결과·산출물 계약
- `datasets.yaml`: rosbag의 해시, 토픽 수, 무결성, 사용 가능 범위
- `map_registry.yaml`: 지도 후보·게시·폐기 상태와 승격 조건
- `calibration_registry.yaml`: 센서 외부 파라미터와 시간 기준의 검증 상태
- `mcap_writer_options.yaml`: 다음 MCAP 기록에서 CRC와 인덱스를 보존하는 저장 설정
- `qos_overrides.yaml`: 기록·재생에 공통으로 사용할 명시적 QoS
- `phase0_status.yaml`: 현재 오프라인 게이트와 SO101 경계 감사 결과
- `inspect_mcap.py`: ROS 노드와 재생 없이 MCAP 전체 메시지와 CRC를 읽는 검사 도구
- `corridor_run_media.py`: 주행 구간 지표와 CSV·PNG·GIF·MP4를 MCAP에서 재생성하는 도구
- `../launch/offline_replay_guard.launch.py`: 저장 지도와 이동 명령을 재생하지 않고 AMCL `map -> odom`을 제거하는 launch

`inspect_mcap.py`와 미디어 생성기는 `requirements.txt`에 고정한 MCAP reader, ROS 2
decoder, 수치·그림 라이브러리를 사용한다. 로봇 런타임의 전역 Python 환경이나 OS
패키지를 바꾸지 않도록 평가 전용 디렉터리에 설치한다. MP4 변환에는 시스템 `ffmpeg`를
사용하며, 없으면 GIF와 나머지 산출물만 만든다.

```bash
python3 -m pip install --disable-pip-version-check --no-input --upgrade \
  --requirement "$HOME/jdamr_cube_ws/src/jdamr_cube_ros/jdamr_cube_navigation/evaluation/requirements.txt" \
  --target "$HOME/.local/share/jdamr-slam-eval/python"
PYTHONNOUSERSITE=1 PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
  python3 -c "import mcap; print(mcap.__version__)"
```

## 기존 G4 bag 판정

`g4_userloop_reset_20260824T175151_0.mcap`은 63,955개 메시지를 끝까지 읽었고 summary CRC와 인덱스가 있다. 그러나 39개 chunk의 CRC가 모두 0이며 data-section CRC도 0이다. 당시 기록·재생 QoS와 NIC·드라이버·커널 드롭 카운터가 없고, 로봇을 든 뒤의 꼬리 구간도 분리되지 않았다. 따라서 원인 탐색에는 사용할 수 있지만 지도나 성능 결과를 승격하는 입력으로는 사용할 수 없다.

`validate_crcs: PASS`는 파일에 존재하는 CRC가 일치했다는 뜻이다. `chunks_with_crc: 0`은 핵심 데이터 chunk에 검증할 CRC 자체가 없다는 뜻이므로 서로 모순되지 않는다.

검사 명령은 메시지를 발행하지 않는다.

```bash
cd $HOME/jdamr_cube_ws/src/jdamr_cube_ros
PYTHONNOUSERSITE=1 PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
  python3 jdamr_cube_navigation/evaluation/inspect_mcap.py \
  $HOME/jdamr_artifacts/g4_userloop_reset_20260824T175151/g4_userloop_reset_20260824T175151_0.mcap
```

## 다음 기록의 고정 조건

아래 명령은 운영자가 안전 구역과 비상 정지를 확인한 뒤 별도 주행 세션에서만 사용한다. `<run_id>`는 실행 전에 고유한 실제 값으로 바꿔야 한다. 이 문서는 이동 명령을 포함하지 않는다. 실차 Nav2가 파이에서 실행될 때 이 recorder도 반드시 파이에서 실행한다. 노트북에서 같은 고속 토픽을 추가 구독하면 약한 Wi-Fi 구간에서 TF·scan 지연이 제어 루프까지 전파될 수 있다. 저장지도 Keepout 주행은 recorder와 navigation을 함께 묶은 `onboard_keepout_navigation.launch.py`를 우선 사용한다. 파이에는 RViz, 온라인 mapping backend, bag 재생·분석을 추가로 띄우지 않는다.

```bash
cd $HOME/jdamr_cube_ws/src/jdamr_cube_ros
source /opt/ros/jazzy/setup.bash
ionice --class best-effort --classdata 7 \
nice --adjustment 10 \
ros2 bag record \
  --storage mcap \
  --storage-config-file jdamr_cube_navigation/evaluation/mcap_writer_options.yaml \
  --qos-profile-overrides-path jdamr_cube_navigation/evaluation/qos_overrides.yaml \
  --output $HOME/jdamr_artifacts/<run_id> \
  /scan /odom /tf /tf_static /imu/data_raw \
  /cmd_vel /cmd_vel_nav /amcl_pose /battery_state /plan \
  /collision_monitor_state
```

`/joint_states`는 RViz 바퀴 표현용이라 기본 온보드 기록에서 제외한다. MCAP 압축은 CPU를
아끼기 위해 사용하지 않고, 1MiB chunk의 CRC와 인덱스만 보존한다. 출발 전 1분 정적
소크에서 4코어 load average 4.0 미만과 thermal throttle 없음도 함께 확인한다.

프로토콜 적격 데이터는 다음을 모두 만족해야 한다.

1. bag SHA-256과 metadata SHA-256을 등록한다.
2. chunk CRC, summary CRC, chunk/message index를 보존한다.
3. 기록·재생 QoS 파일과 소프트웨어 버전을 manifest에 남긴다.
4. NIC·드라이버·커널 드롭 카운터와 rosbag 토픽 수를 남긴다.
5. 로봇을 들거나 센서를 가린 구간은 시간 경계로 제외한다.
6. `map -> odom` TF 권한자는 한 개만 허용한다.
7. `UNVERIFIED` 또는 `UNOBSERVABLE` 보정값은 센서 융합 성능 주장에 사용하지 않는다.

진단 전용 데이터에서는 미상 항목을 명시한 채 보존할 수 있다. 하지만 `protocol_qualified: true` 입력에 QoS, CRC, 드롭 또는 구간 경계가 미상이라면 승격을 차단한다.

## 완주 기록의 지표와 미디어 생성

`corridor_run_media.py`는 경로 실행 로그의 첫 목표 전송과 성공 시각을 분석 구간으로
사용한다. MCAP recorder timestamp로 토픽 rate, p99 gap과 최대 gap을 계산하고, 마지막
메시지 뒤 무응답 시간도 최대 gap에 포함한다. AMCL 누적 경로는 5cm보다 작은 연속 위치
변화를 제거한 뒤 계산한다. 이 숫자는 저장 지도 localization의 일관성 지표이며 외부
ground truth ATE가 아니다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
PYTHONNOUSERSITE=1 \
PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
python3 jdamr_cube_navigation/evaluation/corridor_run_media.py \
  --run-dir "$HOME/jdamr_artifacts/corridor_localdds_armed_20260904T152036" \
  --compare-run-dir "$HOME/jdamr_artifacts/corridor_roundtrip_20260904T142436" \
  --output-dir \
    jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036
```

출력에는 집계 YAML/JSON, AMCL 궤적과 목표 진행 CSV, 지도 중첩 그림, stream gap 비교,
텔레메트리, GIF와 H.264 MP4가 포함된다. GIF는 실제 AMCL timestamp를 균등 시간축으로
압축한 시각화이며 카메라 촬영 영상이 아니다.

## 센서와 시간 프로파일

성공 주행의 route log가 있으면 실제 목표 전송부터 성공 시각까지만 분석한다. IMU 정적
노이즈는 주행 직전 60초를 별도 구간으로 잡아 이동 구간과 섞지 않는다. LiDAR의 `inf`는
토픽 손실이 아니라 최대 거리 안에서 반사가 없었던 ray로 기록한다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
PYTHONNOUSERSITE=1 \
PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
python3 jdamr_cube_navigation/evaluation/profile_sensor_streams.py \
  --bag "$HOME/jdamr_artifacts/corridor_localdds_armed_20260904T152036" \
  --output-dir \
    jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036
```

출력은 sensor profile YAML/JSON/Markdown과 센서·시간 분포 PNG다. 실차에 외부 ground
truth가 없으므로 LiDAR additive range noise와 odometry 정확도는 이 데이터만으로 만들지
않는다. 시뮬레이션 후보값은 관측 범위의 시작점이며 튜닝 완료값이 아니다.

## 저장 지도 주행 bag의 오프라인 SLAM 재생

저장 지도와 AMCL을 사용해 안전 경로로 수집한 bag도 새 지도 생성 입력으로 쓸 수 있다. 단, 새 mapping backend는 `use_sim_time=true`와 빈 상태로 먼저 실행하고, localization node와 저장 map server는 실행하지 않는다. 재생은 실차 domain 12가 아닌 격리 domain 199에서만 허용한다.

```bash
cd "$HOME/jdamr_cube_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=199
ros2 launch jdamr_cube_navigation offline_replay_guard.launch.py \
  bag:="$HOME/jdamr_artifacts/<run_id>"
```

이 launch는 `/scan`, `/odom`, `/tf`, `/tf_static`, `/imu/data_raw`, `/joint_states`만 재생한다. `/map`과 `/cmd_vel`은 재생 목록에 없으며, TF는 전용 토픽으로 우회한 뒤 기록된 `map -> odom`만 제거해서 원래 `/tf`로 전달한다. 따라서 `/tf`의 `odom -> base_footprint`와 센서 고정 TF는 보존되고, 새 mapping backend만 `map -> odom` 권한자가 된다. TF filter나 bag player가 종료되면 전체 재생도 종료한다. TF queue 유실을 막기 위해 `rate`는 실시간 1.0 이하만 허용한다.

두 backend 실행이 끝나면 같은 원본 기준의 비교 자료를 만든다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
PYTHONNOUSERSITE=1 \
PYTHONPATH="$HOME/.local/share/jdamr-slam-eval/python" \
python3 jdamr_cube_navigation/evaluation/compare_slam_runs.py \
  --results "$HOME/jdamr_artifacts/offline_slam_20260904" \
  --source-root "$HOME/jdamr_artifacts" \
  --output jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.json \
  --plot jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.png \
  --report jdamr_cube_navigation/evaluation/media/corridor_localdds_armed_20260904T152036/slam_backend_comparison.md
```

설정 필드와 QoS 형식은 [rosbag2 MCAP 저장소 공식 문서](https://github.com/ros2/rosbag2/blob/rolling/rosbag2_storage_mcap/README.md)와 [rosbag2 QoS override 공식 문서](https://github.com/ros2/rosbag2/blob/rolling/README.md)를 기준으로 한다.

### 가변 LaserScan 격자 원인분리

스캔마다 빔 수나 시작 각도가 달라지는 bag은 원본을 수정하지 않고 고정 각도 격자의
파생 bag으로 만든다. `--beam-count`는 원본 분포를 확인한 뒤 실험 manifest에 기록한다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
python3 jdamr_cube_navigation/evaluation/normalize_scan_bag.py \
  --bag "$HOME/jdamr_artifacts/<run_id>" \
  --output "$HOME/jdamr_artifacts/<run_id>_scan_fixed" \
  --beam-count <count> \
  --storage-config-file \
    jdamr_cube_navigation/evaluation/mcap_writer_options.yaml
```

출력 bag에는 원본·출력 SHA-256, 입력 빔 수 분포, 정규화 방식, 전수 검증 결과를 담은
manifest가 함께 생성된다. 검증은 메시지 순서와 timestamp, 비-scan payload, 정규화된
모든 scan을 대조한다.

파라미터 실험은 검토된 기본 YAML에서 허용된 값만 바꾼 파생 파일을 만들고 실행 라벨과
설정 경로를 로그에 남긴다.

```bash
python3 jdamr_cube_navigation/evaluation/make_slam_toolbox_ablation.py \
  --base jdamr_cube_navigation/config/slam_toolbox_corridor.yaml \
  --output "$HOME/jdamr_artifacts/<experiment>/params.yaml" \
  --label <label> \
  --max-laser-range-m <metres>

bash jdamr_cube_navigation/scripts/offline_slam_replay.sh \
  --bag "$HOME/jdamr_artifacts/<run_id>_scan_fixed" \
  --backend slam_toolbox \
  --label <unique_run_label> \
  --slam-params "$HOME/jdamr_artifacts/<experiment>/params.yaml" \
  --out "$HOME/jdamr_artifacts/<results>" \
  --rate 1.0
```

현재 복도 실험의 판정과 한계는 `20260904_SLAM_TOOLBOX_ROOT_CAUSE.md`에 기록했다.
