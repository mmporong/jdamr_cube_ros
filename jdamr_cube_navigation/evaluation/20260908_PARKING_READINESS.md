# 정밀 주차 후보 — Gazebo 검증 완료 / 실차 검증 대기

## 범위와 판정

사용자가 수용한 목표는 위치 오차 3–5 cm, 방향 오차 2–3도다. 현재 계약은
`map` 좌표계의 `base_link` 기준 평면 거리 0.05 m 이내, 방향 오차 3도 이내다.
Gazebo seed 11 단일 실행은 이 계약을 통과했다. 이 결과는 시뮬레이션 후보의
동작 증거이며 실차 정확도, 반복성 또는 벽과 차체 사이의 실제 거리를 입증하지
않는다.

### 성공 실행 결과

| 항목 | 결과 | 판정에서의 역할 |
| --- | ---: | --- |
| Gazebo ground truth 위치 오차 | 4.541 cm | 시뮬레이션 최종 정확도 판정 |
| Gazebo ground truth 방향 오차 | 2.540° | 시뮬레이션 최종 정확도 판정 |
| 확인 시점 `map→base_link` TF 위치 오차 | 3.916 cm | 로봇 내부 완료 확인 |
| 확인 시점 `map→base_link` TF 방향 오차 | 1.707° | 로봇 내부 완료 확인 |
| 연속 정지 유지 | 1.007 s | 완료 후 정지 판정 |
| Nav2 action 결과 | 2개 모두 `STATUS_SUCCEEDED` | 접근·최종 목표 완료 판정 |
| MCAP | 454,286 bytes, 630 cmd / 663 ground-truth samples | 재생·재평가 원본 |
| 종료 정리 | 잔존 프로세스 0 | 격리 실행 종료 판정 |

마지막 `/amcl_pose` 메시지는 위치 3.916 cm, 방향 8.107°였다. 이 토픽은
입자 필터가 발행한 시점의 pose이고, odometry가 계속 반영된 현재
`map→base_link` 합성 TF와 같은 시점의 값이 아니다. 따라서 완료 판정에는 최신
TF와 odometry를 사용하고, 미디어에는 `/amcl_pose`도 숨기지 않고 별도 궤적으로
표시한다. 실차에서는 외부 줄자·바닥 기준선 측정을 최종 판정으로 사용한다.

## 구현

- `config/parking_contract.yaml`: 단위가 명시된 정확도·속도·정지 계약.
- `evaluation/prepare_parking_params.py`: 운영 Nav2 YAML을 덮어쓰지 않고 Parking
  제어기와 `parking_goal_checker`가 포함된 새 후보 파일을 생성한다.
- `behavior_trees/navigate_to_pose_parking.xml`: 주차용 controller/checker ID를
  고정한다.
- `corridor_route --park-final`: 마지막 waypoint에만 주차 BT를 적용한다. 명시적
  yaw, 지도·마스크 hash, 주차 런타임 파라미터를 시작 전에 확인한다.
- `evaluation/run_parking_smoke.py`: localhost 전용 ROS domain에서 Gazebo, 운영과
  같은 onboard Nav2 core, keepout, compact MCAP 기록기를 띄우고 1 m 접근·주차를
  실행한다. 실차 명령 경로는 사용하지 않는다.
- `evaluation/render_parking_media.py`: 성공한 MCAP에서 지도 위 ground truth,
  AMCL, Nav2 plan, 5 cm 목표 원, 최종 방향과 시간축 오차를 MP4/GIF/PNG로 만든다.

SimpleGoalChecker의 stateful latch를 끄고 위치·각도를 함께 검사한다. RPP의 최종
방향 회전이 종료되기 전에 정지 속도까지 요구하지 않고, Nav2 성공 후 최신 TF,
odometry, 최종 `cmd_vel`을 연속 관찰한다. route가 직접 속도를 발행하지 않으며,
금지구역·Collision Monitor·footprint·기존 FollowPath 설정도 완화하지 않는다.

## 실패에서 확인한 원인과 수정

1. 첫 장거리 후보는 최종 ground truth 위치 오차가 6.93 cm여서 계약을 넘었다.
2. 1 m로 축소한 두 번째 후보는 route의 observation age가 약 1.788e9초로
   계산됐다. Gazebo 센서 header는 simulation time인데 route만 wall time을 써서
   정상 관측을 stale로 거부한 것이 원인이었다.
3. route에 `use_sim_time:=true`를 명시해 시간 기준을 맞췄다. 세 번째 실행은
   두 goal 성공, 내부 TF 확인, ground truth 오차, 정지 유지, 종료 정리를 모두
   통과했다.

두 실패 실행은 진단 근거로 보존하지만 대표 미디어와 성능 수치는 세 번째 성공
실행만 사용한다. 단일 성공을 반복성 검증으로 확대 해석하지 않는다.

## 증거와 미디어

원본 성공 실행:

- `/home/lim/jdamr_artifacts/parking_smoke_20260908_v03/summary.json`
- `/home/lim/jdamr_artifacts/parking_smoke_20260908_v03/route.log`
- `/home/lim/jdamr_artifacts/parking_smoke_20260908_v03/bag/bag_0.mcap`
- MCAP SHA-256:
  `7efa6de68ac26c29e3cf4b22b702c6f6faafbafecc41973e92a13809c062e56c`

최종 미디어:

- `/home/lim/jdamr_artifacts/parking_smoke_20260908_v03_media_v03/parking_seed11_story.mp4`
- `/home/lim/jdamr_artifacts/parking_smoke_20260908_v03_media_v03/parking_seed11_story.gif`
- `/home/lim/jdamr_artifacts/parking_smoke_20260908_v03_media_v03/parking_seed11_final.png`
- `/home/lim/jdamr_artifacts/parking_smoke_20260908_v03_media_v03/metrics.json`
- `/home/lim/jdamr_artifacts/parking_smoke_20260908_v03_media_v03/manifest.json`

MP4는 H.264 1280×720, 8 fps, 10초다. GIF는 768×432, 60 frame이다. 미디어
전체는 약 3.65 MB라 원본 MCAP과 함께 보존해도 부담이 작다. RViz 화면 녹화는
별도로 띄우지 않았다. 같은 MCAP에서 공간 경로와 시간축 오차를 한 화면에
재구성한 현재 미디어가 목표 반경과 최종 방향을 더 직접적으로 보여주며, Pi나
실차 프로세스를 추가하지 않는다.

재생 미디어를 새 디렉터리에 다시 생성하는 명령:

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
export PYTHONPATH="$PWD/jdamr_cube_navigation:$PWD/jdamr_cube_navigation/evaluation:${PYTHONPATH:-}"
python3 jdamr_cube_navigation/evaluation/render_parking_media.py \
  --run-root "$HOME/jdamr_artifacts/parking_smoke_20260908_v03" \
  --output "$HOME/jdamr_artifacts/parking_smoke_20260908_v03_media_replay"
```

출력 디렉터리가 이미 있으면 실패해 기존 증거를 덮어쓰지 않는다.

## 검증 결과

- `colcon build --packages-select jdamr_cube_navigation --symlink-install`: PASS.
- 패키지 전체 테스트: 1,223 passed, 1 skipped, 0 failed.
- 주차 smoke·미디어 단위 테스트: 5 passed.
- 변경 Python 4파일 `ament_flake8`: PASS.
- MP4 codec·해상도·길이, GIF frame 수, manifest의 모든 SHA-256 대조: PASS.

전체 테스트의 첫 시도는 평가 모듈 디렉터리를 `PYTHONPATH`에 넣지 않아 수집
전에 종료됐다. 경로를 명시한 재실행에서 기능 테스트는 모두 통과했고, 기존
미디어 파일 두 곳에서 발견된 import 순서·빈 줄 3건도 수정한 뒤 전체 PASS를
확인했다.

## 다음 실차 검증

1. 실차를 주차 시작 위치에 놓고 `map` 초기 자세만 맞춘다.
2. 바닥 목표점과 기준 방향을 표시한 뒤 첫 회를 저속 실행한다.
3. 줄자로 중심 위치 오차를, 바닥 기준선으로 방향 오차를 외부 측정한다.
4. 첫 회가 안전하게 통과하면 총 3회까지만 반복해 중앙값과 최대 오차를 남긴다.

실차에서 벽면까지의 직접 거리가 필요하면 지도 pose만으로 주장하지 않고,
별도의 벽·모서리 상대 정렬 또는 외부 측정 단계를 추가한다. 현재 자동 실행
스크립트에는 `--park-final`을 연결하지 않았으므로 기존 주행이 자동으로 주차
모드로 바뀌지 않는다.

## 기존 돌발 장애물 준비 작업

`evaluation/run_onboard_candidate_smoke.py`는 여러 `topic echo` 대신 compact MCAP
기록기 하나를 사용한다. hidden action status와 wall log-time/sim header-time을
구분해 기록한다. `evaluation/onboard_stop_contract.py`와
`sim_collision_monitor_scenario.py`의 `--direct-scan`은 운영 Collision Monitor
프로필의 돌발 장애물 평가 경로다. 이후 팔 collision 외곽에서 유도한 StopZone 0.40 m,
SlowdownZone 0.50 m와 `/joint_states` 수납 자세 게이트를 실제
`obstacle_candidate` launch에 연결했고, 대표 통합 실행이 접촉 0·동일 목표 재개·최종
도착으로 통과했다. 상세 근거는
[이동형 로봇팔 장애물 대응](20260908_DYNAMIC_OBSTACLE_READINESS.md)에 있다.

주차 후보와 장애물 후보는 같은 Nav2 기반이지만 완료 조건이 다르다. 주차 결과가 장애물
대응을 대신 증명하지 않고, 장애물 통합 PASS도 실차 주차 정확도를 대신하지 않는다.

근거: [Nav2 Jazzy SimpleGoalChecker](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/plugins/simple_goal_checker.cpp),
[RPP 최종 방향 제어](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp).
