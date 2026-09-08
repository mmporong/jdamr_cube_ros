# 정밀 주차 후보 — 중간 인계

## 범위와 상태

사용자가 수용한 검증 목표는 위치 오차 3–5 cm, 방향 오차 2–3도다.
초기 후보는 map 좌표계의 base_link 기준 평면 거리 0.05 m 이내,
방향 오차 3도 이내로 설정한다. 차체 앞면과 벽 사이 거리를 직접 측정하는
기능은 아직 없으며, 지도 기반 추정 오차를 실측 주차 정확도로 주장하지 않는다.

구현 작성 중간본이다. ROS 비의존 계산·파라미터 생성 및 route 연결 테스트를
진행 중이며, 독립 리뷰와 실제 Nav2 시뮬레이션/실차 주차는 미완료다.
원래 진행하던 운영 후보 돌발 장애물 통합 검증도 별도 미완료 작업이다.

## 구현

- `config/parking_contract.yaml`: 단위가 명시된 검증 목표와 후보 속도/정지 조건.
- `evaluation/prepare_parking_params.py`: 기존 Nav2 YAML을 읽고 신규 파일에만
  Parking 제어기와 parking_goal_checker를 추가한다. 원본·기존 파일 덮어쓰기 금지.
- `behavior_trees/navigate_to_pose_parking.xml`: controller/checker ID를 고정한다.
- `corridor_route --park-final`: 마지막 waypoint에만 주차 BT를 선택하며,
  명시적인 yaw가 없거나 런타임 주차 파라미터가 다르면 시작 전에 거부한다.
  기존 주행 프로필과 중간 waypoint 판정은 유지한다.
- Nav2 성공 뒤 최신 TF 위치·방향, odometry 속도, 최종 cmd_vel의 정지를
  연속 관찰한다. 성공 사건은 `parking_estimate_confirmed`이며
  `physical_accuracy=NOT_MEASURED`를 함께 남긴다. 직접 속도를 발행하지 않는다.

SimpleGoalChecker의 stateful latch를 끄고 위치·각도를 함께 검사한다.
RPP의 최종 방향 회전이 종료되기 전에 정지 속도까지 요구하는 방식 대신,
Nav2 성공 후 별도 관찰로 정지 유지를 확인한다. 금지구역, Collision Monitor,
footprint와 기존 FollowPath 설정을 완화하지 않는다. 주차 실패를 숨기는
자동 목표 재전송·강제 전진도 넣지 않는다.

## 재개 순서

1. parking 관련 focused tests 및 기존 keepout 회귀, lint/build.
2. 독립 코드/구조 리뷰와 지적사항 수정.
3. 분리된 시뮬레이션에서 최종 각도 정렬·정지 관찰 한 건 검증.
4. 실차에서는 주차 기준점을 지정하고 줄자·바닥 기준선으로 실제 위치와
   각도 오차를 측정한다. 필요 시 벽/모서리 기반 상대 정렬을 추가한다.

파이에서 새 프로세스를 시작하거나 로봇을 움직이지 않았다. 현재 자동 실행
스크립트에는 주차 옵션을 연결하지 않았으므로 기존 주행이 자동으로 바뀌지 않는다.

## ROS를 실행하지 않는 후보 생성

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
export PYTHONPATH="$PWD/jdamr_cube_navigation:${PYTHONPATH:-}"
python3 jdamr_cube_navigation/evaluation/prepare_parking_params.py \
  --output /tmp/jdamr_parking_candidate.yaml
```

파일이 이미 있으면 실패한다. 출력 경로를 새 이름으로 정해 원본을 보존한다.
이후 후보 Nav2 launch에 `params_file`로 전달하고 동일 계약을 사용하는 route에
`--park-final`을 지정해야 적용된다. `--execute` 없는 route는 기존과 같이
계획 검사만 수행한다. 위 명령 자체는 파일 생성만 하며 주행하지 않는다.

근거: [Nav2 Jazzy SimpleGoalChecker](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_controller/plugins/simple_goal_checker.cpp),
[RPP 최종 방향 제어](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp).
