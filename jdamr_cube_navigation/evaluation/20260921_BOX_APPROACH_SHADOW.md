# 박스 상대 정밀 접근 — 비구동 구현과 검증

## 완료 범위

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
