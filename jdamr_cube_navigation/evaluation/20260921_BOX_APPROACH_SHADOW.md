# 박스 상대 정밀 접근 — 비구동 구현과 검증

## 실물 시험 및 후속 수정 — 2026-09-21 16:43

아래 준비 이력 이후 사용자 요청으로 실물 정밀 주차 시험을 1회 실행했다.
결과는 **ABORTED / 주차 검증 미완료**다. 성공 또는 충돌 발생으로 기록하지 않는다.

- 사용자 확인 시작 간격 0.82m, 시작 Depth 관측 0.812m, 목표 간격 0.45m.
- `/box_parking/start_validation_approach`의 성공 응답과 보호 체인을 통과한
  비영 명령을 확인했다. 시험은 속도 0.03m/s, 각속도 0.10rad/s, 누적 이동
  0.45m, 실행 30초로 제한하며 자동 재출발하지 않는다.
- 기록상 시작·종료 odometry 위치 차이는 0.1630m, 방향 변화는 시계방향
  16.35°였다. 종료 Depth 간격은 0.633m로 목표에 도달하지 못했다.
- Pi 로그 16:43:07.141의 실제 개입은 `SlowdownZone`의 `SLOWDOWN`이었다.
  16:43:07.187에는 normal로 복귀했다. 실행기가 감속도 실패로 취급해
  `collision_monitor_intervention`으로 중단한 것이 확인됐다.
  이 기록은 20cm 이내 장애물이나 접촉이 있었다는 근거가 아니다.
- SlowdownZone은 base 기준 x/y ±0.5m 영역이며 차체 외곽부터의 20cm와
  같은 기준이 아니다. 어느 점·물체가 들어왔는지는 이 기록만으로 확정하지 않는다.
- 종료 후 보호 명령과 odometry 속도 0을 관측했다. 자동 재시도는 하지 않았다.

후속 코드 변경은 `SLOWDOWN`에서 CM의 감속 출력을 유지하고 접근을 계속하는
것이다. `STOP`, `APPROACH`, `LIMIT`, 미정의 동작은 기존처럼 중단한다.
선속도와 각속도를 각각 잘라 회전 비율을 바꾸던 시험 제한도 동일 배율 축소로
수정했다. 회전 비율 변경은 코드에서 확인됐지만, 관측된 16.35° 회전 전체의
원인이나 수정 후 실제 주차 성능을 입증한 것은 아니다.

원시 기록: `$HOME/jdamr_artifacts/parking_trial_20260921_sm4Nya/telemetry_run.jsonl`
(49.69초, 63개 API 표본), 같은 폴더의 `start_rgb.jpg`, `end_rgb.jpg`.
API 표본은 전송 지연이 있을 수 있으므로 중단 원인은 Pi journal과 대조했다.

### 다음 시작은 한 번의 명시적 요청

부팅 시 기존 systemd 서비스가 센서와 접근 체인을 비무장 상태로 준비한다.
운영 설정을 매번 재빌드하거나 이미 완료한 검사를 반복하지 않는다.
ROS 환경과 배포 overlay가 로드된 Pi에서 다음 명령 하나가 임시 확인값 설정과
시작 요청을 순서대로 처리한다.

```bash
ros2 run jdamr_cube_navigation box_parking_start --reference-m <현재_측정값_m> --charger-unplugged
```

이 명령은 현재 박스 간격 0.65~1.0m와 충전선 분리 확인을 명시해야 하며,
실행기에서 새 관측과 ±0.04m 일치를 검사한다. 이전 0.82m는 이동 후 재사용하지
않는다. 이번 종료 위치의 0.633m는 시작 범위 밖이므로 이 자리에서 자동 재출발하지
않는다. 거부 응답을 재시도하지 않고, 시작 응답이 유실되면 취소를 요청한다.

일반 `/box_parking/start_approach`의 실물 검증 파일 요구는 유지한다.
시험 서비스는 사전 성공 증거 대신 명시적인 제한 시험 계약을 적용하며,
성공하더라도 `trial_completed`로 기록할 뿐 승인 파일을 자동 생성하지 않는다.

후속 수정의 관련 테스트는 119개 통과했고 독립 코드 리뷰를 통과했다.
Pi 배포본은 `$HOME/jdamr_deploy/box_parking_fixed_20260921_ou2vUW`이며,
navigation 빌드·manifest 검사와 시작 명령의 `--help` 실행을 확인했다.
모터·RGB-D 서비스는 유지하고 접근 서비스만 비무장 상태로 재시작했다.
수정 후 실물 주차 재시험은 아직 하지 않았다.

## 후속: 출발 준비와 실행 연결

`box_approach_execution.launch.py`는 박스 관측기, 상대 접근 실행기, 기존
velocity smoother, Collision Monitor와 lifecycle manager를 함께 준비한다.
부팅이나 launch 실행은 출발 명령이 아니다. 시작은
`/box_parking/start_approach`, 취소는 `/box_parking/cancel_approach` 서비스로 분리했다.
준비 상태와 차단 사유는 `/box_parking/execution_status`에서 확인한다.

- Pi underlay의 `camera_mount.yaml`은 unmeasured/null 상태였고 PC의 측정 설정과
  달랐다. 명시적 snapshot 경로와 SHA256 manifest를 사용해 다른 overlay의 설정이
  섞이지 않게 했다. 빈 설정은 프로세스 실행 전에 거부한다.
- 명령은 `/cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → collision_monitor
  → /cmd_vel → base_driver`로만 전달한다. 중복 명령 발행자와 필수 노드 누락을
  확인하며, local costmap을 실행하지 않는 대신 같은 설정의 padded footprint를 발행한다.
- 준비 단계의 0속도 확인과 주행 중 검사를 구분한다. 정지한 Collision Monitor는
  `stop_pub_timeout` 이후 0속도 발행을 중단하므로 이를 주행 중 통신 단절과 동일하게
  취급하지 않는다. 근거는 [Nav2 Jazzy 구현](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_collision_monitor/src/collision_monitor_node.cpp)의
  `publishVelocity`다. 활성 상태 조회, 센서 시각, 명령 소유권 검사는 유지한다.
- 실물 검증 승인은 카메라·차체·Nav2·주차 계약의 SHA256과 근거가 포함된 별도 파일을 요구한다.
  관측기의 `control_ready=false`를 고치거나 승인 파일을 자동 생성하지 않는다.
  `charger_unplugged_confirmed`는 프로세스가 시작될 때 false이며 영구 저장하지 않는다.

현재 문서의 승인 파일 부재는 실제 차단 조건이다. 소프트웨어 테스트 통과나
`systemctl is-active`만으로 출발 준비 완료를 판정하지 않는다. 실물 정밀 접근과
정지 정확도는 아직 완료 증거가 없다.

운영 파일은 `jdamr_cube_bringup/systemd/jdamr-box-rgbd.service`,
`jdamr-box-approach.service`, `scripts/box_approach_prepare.sh`다. 서비스는
`/etc/jdamr-box-approach.env`의 `JDAMR_APPROACH_RELEASE`가 지정하는 배포본을 사용한다.
배포본의 `config/physical_validation.yaml`은 실물 검증 후에만 별도로 작성한다.
서비스를 실행해도 시작 서비스 호출과 모든 조건 충족 전에는 0속도만 허용한다.

실물 로봇과 분리된 준비 체인 재현:

```bash
cd "$HOME/jdamr_rgbd_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/jdamr_cube_ros/jdamr_cube_navigation/evaluation/smoke_box_approach_startup.py
```

이 검사는 LOCALHOST/domain 198의 가상 센서와 설치된 Nav2를 사용한다.
실물 domain 12 또는 모터 드라이버를 실행하지 않는다.

`--armed` 옵션은 같은 격리 domain의 합성 승인 파일과 가상 베이스 수신기를 사용해
실제 Nav2를 통과한 비영 속도를 검사한다. 실물 승인 파일을 만들거나 재사용하지 않는다.
출발→취소, scan 단절, 실제 CM STOP, CM 출력 단절의 4개 경로가 통과했다.
관련 테스트 160개, 변경 파일 flake8, navigation colcon build도 통과했다.
독립 verifier는 비구동 배포·커밋 범위 PASS로 판단했으며 실차 정밀 주차 승인은 아니다.

### Pi 반영 및 남은 조건 — 2026-09-21

- 배포 위치: `$HOME/jdamr_deploy/box_approach_startup_20260921_79rBbd`.
  운영 중에는 파일을 수정하지 않고, 갱신할 때 서비스를 정지한 뒤 manifest를 다시 검증한다.
  manifest는 파일 일치성을 검사하며 관리자에 의한 manifest 재생성까지 막지는 않는다.
- `jdamr-base`, `jdamr-box-rgbd`, `jdamr-box-approach` 서비스가 active/enabled다.
  예전 depth-only/LOCALHOST 설정의 `jdamr-astra-camera`는 disabled로 전환했다.
  실제 재부팅 검증은 하지 않았다. 부팅 후 접근기는 비무장 상태이며 자동 출발하지 않는다.
- 기존 Nav2·수동 운전과 접근기의 명령 소유권은 공유하지 않는다. 다른 운전 모드로
  전환할 때 `sudo systemctl stop jdamr-box-approach.service`로 이 체인을 먼저 해제한다.
- CM/smoother ACTIVE, CM enable 응답, 보호 경로의 0속도 수신을 확인했다.
  그래프는 0.5초마다 한 번 조회하며 tick에서 전체 그래프를 반복 조회하지 않는다.
  정지 이후에는 시작 조건을 다시 평가하여 오래된 주행 이력이 재출발을 막지 않는다.
- 센서 구독은 KEEP_LAST 1로 제한했다. 평면 후보의 결과를 바꾸지 않는 계산 생략과
  percentile 통합을 적용했다. `age_s`는 검출 완료 시각 기준이고 처리 시간도 따로 발행한다.

같은 날 부하가 높은 Pi에서 얻은 관측 결과:

| 항목 | 수정 전 관측 | 최종 수정 후 관측 |
| --- | --- | --- |
| 관측 구간 / 검출 결과 수 | 8초 / 8개 | 15초 / 50개 |
| 수신 시점 영상 age | 0.883~1.567초 | 중앙값 0.270초, p95 0.561초, 최대 0.660초 |
| `perception_stale` 상태 수 | 75개 중 35개 | 140개 중 8개 |

부하와 온도가 일정한 통제 벤치마크는 아니다. 최종 측정에서 파이는 85.2°C,
`get_throttled=0xe0008`이었고 p95가 0.5초 기준을 넘었으므로 실차 운전 성능 PASS로
취급하지 않는다. 140개 `/cmd_vel_nav` 명령은 모두 0이었다.

현재 남은 차단은 실제 측면 근접점에 따른 CM STOP, 박스 후보의 연속 안정성 부족,
실물 외부 파라미터 검증 파일 부재, 프로세스 재시작 후 충전 분리 확인 초기화다.
물체 의미나 카메라 기울기를 현재 관측만으로 확정하지 않았다. 사용자에게 충전 분리를
다시 요구한 것은 아니며, 이번 배포에서는 실물 출발을 호출하지 않아 false를 유지했다.
추후 출발을 요청받으면 기존 확인의 유효성과 현재 상태를 판단해 프로세스에 반영한다.
확인은 영구 저장하지 않으며 재충전 이후 재사용하지 않는다.

남은 단계는 냉각/부하가 안정된 상태의 입력 지연 확인, 현재 목표의 안정된 관측과 카메라
외부 파라미터 검증, 실제 장애물과의 간격 확보 후 제한 접근이다. 소프트웨어 기동 수정과
실차 주차 완료를 구분한다. SIGINT 시 0속도 drain은 best-effort이며 전체 launch 동시
종료에서는 베이스 watchdog이 최종 정지를 담당한다.

## 앞선 shadow 구현의 완료 범위

사용자가 충전기를 연결하고 기체를 옮긴 상태에서 정밀 주차 구현을 요청했다.
실물 주행·Pi 배포·Collision Monitor 설정 변경 없이, 관측 처리와 접근 제어 계산을
구현하고 PC에서 검증했다. 비주얼 SLAM의 기존 자료는 보존했다.

목표는 **하부차체 앞면과 박스 전면 사이 명목 이격 0.45m**, 계산상 위치 오차 0.02m와
방향 오차 3° 이내다. 5cm 물리 밀착이 아니다. `new_base_geometry.yaml`의
`claim_scope=installed_lower_base_without_arms_or_payload`를 확인하며, 팔·선반·적재물의
돌출을 포함한 전체 로봇 이격으로 해석하지 않는다. 미래에 팔을 장착하면 결합 외곽을
검증하기 전 이 수치를 실차 접근 기준으로 재사용할 수 없다.

## 구현

1. `depth_box_parking.py`: Astra의 NaN K 대신 명시적인 rectified P를 사용한다.
   기존 내부 파라미터·이미지 decoder를 재사용해 frame, 해상도, stride, endian을 검증한다.
   잘못된 입력, 0.5초를 넘긴 영상, 미래 영상은 안정성 기록을 초기화한다.
   `control_ready=false`와 속도 publisher 없음은 유지한다.
2. `box_approach.py`: 박스 중심과 전면 법선으로 목표 pose를 계산한다. 카메라 optical의
   오른쪽 양수 좌표를 차체 왼쪽 양수로 변환하고 장착 위치를 반영한다. 좌우 오차가 있는
   경우 곡선 접근을 하고, 근처에서 최종 방향을 정렬한 뒤 기존 `ParkingHold`로 정지 유지를
   확인한다. 단순히 한 번 회전하고 직진하는 방식은 아니다.
3. 최초 획득에는 `stable=true`를 요구한다. 이동 중에는 안정성 창이 변할 수 있으므로
   stable 플래그만으로 반복 정지하지 않고, 신뢰도·시각·고정 odom 기준 목표 일관성을
   확인한다. 다른 평면 선택 또는 기체/odom 점프를 발견하면 중단한다.
4. `box_approach_shadow.py`: 영상 시각의 odom을 보간해 상대 관측을 변환한다.
   시간 범위를 벗어나면 외삽하지 않는다. 관측 당시와 현재 차체 외곽의 평면 간격을
   확인하며, 안전 하한을 벗어나는 전진·후진 마감은 하지 않는다.
5. `WAITING → ALIGN/APPROACH → ALIGN_FINAL → HOLD → SUCCEEDED`로 계산한다.
   시작 후 관측 실패, timeout, 목표 변경, 보호 조건 실패는 `ABORTED`에 고정한다.
   조건이 다시 좋아져도 reset 전에는 해당 guarded policy를 재개하지 않는다.

후보 최대 속도 0.06m/s, 최대 각속도 0.20rad/s, 하부차체 이격 하한 0.38m와 freshness 상한은
테스트한 계산 범위다. 파라미터를 함께 바꿔 5cm 목표나 빠른 속도로 우회할 수 없게
상하한을 검증한다. 이 값들은 실측 제동 성능이 아니다.

## Preview와 실행 승인 분리

새 노드는 `/box_parking/approach_status`에 JSON만 발행한다. `/cmd_vel`,
`/cmd_vel_smoothed` publisher나 NavigateToPose action client가 없다.

- `proposal_only`: 신선한 박스/odom 관측으로 계산한 가상 속도 제안. 충전 중에도
  기하·알고리즘을 확인할 수 있지만 실제 속도 명령이 아니다.
- `guarded_proposal`, `guarded_state`: 별도 상태기계에 모든 보호 조건을 적용한 결과.
  미승인 perception, 충전/불명확한 배터리 상태, scan·CM·명령의 누락/지연,
  Collision Monitor 정지 상태, 다른 입력 명령 소유자가 있으면 0이다.
- `motion_output_enabled=false`: 모든 상태에서 고정. guarded 결과가 0이 아니더라도
  실제 이동 권한을 부여하거나 주행 토픽으로 전달하지 않는다.
- `envelope_scope`, `clearance_reference`: 하부차체 한정임을 계속 표시한다.

현재 battery publisher가 충전 여부를 UNKNOWN으로 보내면 차단 이유를 유지한다.
CM 상태는 이벤트 발행 특성상 정지 관측 중 stale로 표시될 수 있다. 이 shadow의 보호
표시는 실차 기동 가능 판정기가 아니며, 실물 명령 연결 단계에서 lifecycle·상태 발행
특성과 배터리 의미를 검증해야 한다. 이번 작업에서는 운전 보호를 완화하지 않았다.

## 검증 결과

- 좌우 위치·방향·박스 각도를 조합한 이상적 차동구동 15개 조건: 15개 수렴.
- 합성 모델 최종 위치 오차 최대 **0.019892m**, 방향 오차 최대 **2.966°**.
- 제어·ROS shadow·관측 입력·기존 주차 계약·Depth obstacle 관련 검사 **145개 통과**.
- 실제 ROS 노드 생성 후 publisher 목록에 주행 토픽이 없음을 검사.
- ROS callback 경로에서 미승인 관측으로 preview를 계산하되 guarded 제안은 0인 것을 검사.
  독립 guarded 상태의 획득 후 충전 차단 및 차단 해제 뒤에도 ABORTED/0 유지 확인.
- 변경 Python/launch/evaluation 및 setup.py 9개 파일의 ament flake8 통과.
- `colcon build --packages-select jdamr_cube_navigation --symlink-install` 통과.

추가로 이전 실차 회전의 RGB-D 10fps 처리본을 읽어 수정한 decoder와 전면 검출기를
오프라인 실행했다. CameraInfo 도착 전 첫 depth 1개를 제외한 **73개 영상 모두 디코딩 및
전면 후보 검출**에 성공했다. 중앙값은 전방 거리 0.847m, optical 우측 편차 0.271m,
신뢰도 0.9405였다. 영상 header 시각으로 오프라인 처리한 것이므로 실시간 지연 검증은
아니며, 후보의 물체 정답 라벨 또는 73개 모두 동일 박스라는 검증도 아니다.
기체를 옮기기 전 자료이므로 현재 위치의 주차 목표로 사용하지 않는다.

15개 조건은 **이상적 평면 기구학 시뮬레이션**이다. Gazebo 물리 실험, 영상 검출 오차,
카메라 교정, 마찰/미끄럼, 모터 지연, 사람·의자 회피 또는 실차 정차 정확도를 검증한
결과가 아니다. 전면 평면의 형상 후보이며 박스 의미 분류나 여러 목표의 식별도 아니다.

CSV 15개와 결과 JSON:

```text
$HOME/jdamr_artifacts/box_approach_20260921_offline/case_00.csv ... case_14.csv
$HOME/jdamr_artifacts/box_approach_20260921_offline/summary.json
$HOME/jdamr_artifacts/box_approach_20260921_offline/recorded_observer_check.json
```

## 실행 및 다음 단계

오프라인 재현은 존재하지 않는 새 output 경로를 사용한다.

```bash
cd "$HOME/jdamr_rgbd_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 src/jdamr_cube_ros/jdamr_cube_navigation/evaluation/evaluate_box_approach.py \
  --output "$HOME/jdamr_artifacts/box_approach_new_evaluation"
```

비구동 관측 launch는 카메라·odom의 기존 ROS 네트워크 설정을 유지한 셸에서 실행한다.
동일한 박스 관측 노드가 이미 실행 중이면 중복 실행하지 않는다.

```bash
ros2 launch jdamr_cube_navigation box_approach_shadow.launch.py
ros2 topic echo /box_parking/approach_status
```

현재 기체가 이동됐으므로 이전 목표를 이어 쓰지 않는다. 새 관측을 획득하고,
필요한 경우 아래 서비스로 preview와 guarded 목표를 함께 지운다. 이 서비스는 운전 시작이 아니다.

```bash
ros2 service call /box_parking/reset_shadow std_srvs/srv/Trigger '{}'
```

다음 실차 단계는 충전선 분리 후 새 위치의 박스 관측/외부 파라미터 일치 확인,
기존 Collision Monitor 경로에 대한 명령 인계 구현 및 비구동 검증, 그 뒤 승인된 제한 접근이다.
이번 커밋을 실차 정밀 주차 완료나 출발 준비 완료로 표기하지 않는다.
