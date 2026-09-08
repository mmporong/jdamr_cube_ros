# 정밀 주차 후보 — 중간 인계

## 범위와 상태

사용자가 수용한 검증 목표는 위치 오차 3–5 cm, 방향 오차 2–3도다.
초기 후보는 map 좌표계의 base_link 기준 평면 거리 0.05 m 이내,
방향 오차 3도 이내로 설정한다. 차체 앞면과 벽 사이 거리를 직접 측정하는
기능은 아직 없으며, 지도 기반 추정 오차를 실측 주차 정확도로 주장하지 않는다.

구현 후보 중간본이다. ROS 비의존 계산·파라미터 생성, route 연결과 실제
정지 관찰 루프의 fake-clock 회귀, 빌드, 설치본 CLI/파일 생성까지 검증했다.
실제 Nav2 시뮬레이션/실차 주차는 미완료다.
원래 진행하던 운영 후보 돌발 장애물 통합 검증도 별도 미완료 작업이다.

## 구현

- `config/parking_contract.yaml`: 단위가 명시된 검증 목표와 후보 속도/정지 조건.
- `evaluation/prepare_parking_params.py`: 기존 Nav2 YAML을 읽고 신규 파일에만
  Parking 제어기와 parking_goal_checker를 추가한다. 원본·기존 파일 덮어쓰기 금지.
- `behavior_trees/navigate_to_pose_parking.xml`: controller/checker ID를 고정한다.
- `corridor_route --park-final`: 마지막 waypoint에만 주차 BT를 선택하며,
  명시적인 yaw가 없거나 계약 대상 런타임 주차 파라미터가 다르면 시작 전에 거부한다.
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

1. 분리된 시뮬레이션에서 최종 각도 정렬·정지 관찰 한 건 검증.
2. 실차에서는 주차 기준점을 지정하고 줄자·바닥 기준선으로 실제 위치와
   각도 오차를 측정한다. 필요 시 벽/모서리 기반 상대 정렬을 추가한다.

파이에서 새 프로세스를 시작하거나 로봇을 움직이지 않았다. 현재 자동 실행
스크립트에는 주차 옵션을 연결하지 않았으므로 기존 주행이 자동으로 바뀌지 않는다.

## 중간 저장과 리뷰

- 최초 후보: `49bffb6`, `origin/main` 푸시 완료. 해당 시점 focused 88 passed.
- 후속 주차 focused/기존 keepout 회귀 106 passed, 대상 Python 6파일
  ament_flake8 PASS. colcon 패키지 빌드와 설치본 `--help`/후보 YAML 생성 PASS.
- 후속 수정: custom 계약의 오차·속도 상한 및 hold 하한 고정, 수신된 짧은
  비영 명령/odom 움직임의 누적 기록, odom stamp 역행 시 hold 초기화.
- 독립 코드 리뷰의 완화된 계약 허용 지적은 실행 경계와 회귀 테스트로 수정.
  실제 verifier를 fake spin/clock으로 실행해 정상·움직임·시간 역행·stale·동일
  stamp 반복을 검증한다. 구조 리뷰의 최소 주행 속도 파라미터 검사 누락도 수정.
- 구조 리뷰가 발견한 zero 명령 재발행 주기 결합은 제거했다. 최신성이 필요한
  입력은 TF와 odom이며, 마지막 수신 명령은 zero여야 한다. 이후 관찰한 비영
  명령과 odom 움직임은 revision을 올려 hold를 초기화한다. BEST_EFFORT 명령의
  무손실 수신을 보장하지 않으므로 실제 움직임을 odom으로 함께 검사하며,
  시뮬레이션/실차 검증 전 전체 승인 주장은 보류한다.
- 코드 리뷰 최종 `COMMENT`: 남은 CRITICAL/HIGH/MEDIUM/LOW 코드 지적 0건.
  리뷰어의 focused 53 passed, pyflakes/AST/YAML/XML 검증 PASS. 해당 리뷰
  환경에 LSP 진단 도구가 없어 형식적 APPROVE는 하지 않았다. 구조 WATCH와
  합친 전체 판정도 COMMENT이며, 완전한 런타임 검증으로 해석하지 않는다.

### 기존 돌발 장애물 준비 작업의 중간 상태

주차 논의 전 작성된 `evaluation/run_onboard_candidate_smoke.py`는 여러 topic
echo를 compact MCAP 기록기 하나로 교체했다. hidden action status를 포함하고
wall log-time과 sim header-time을 구분하며 원본 로그를 자르거나 삭제하지 않는다.
기록기 관련 회귀/기존 온보드·동일 목표 평가 묶음은 57 passed다.

`evaluation/onboard_stop_contract.py`와 `sim_collision_monitor_scenario.py`의
`--direct-scan` 경로는 운영 Collision Monitor 설정을 보존하는 돌발 장애물
평가 준비 중간본이다. 기존 Collision Monitor 회귀는 211 passed지만 신규 경로를
runner에 연결해 한 번 실제 실행하는 작업은 남았다. 이를 운영 설정 돌발 회피
성공으로 해석하지 않는다. 기존 성공한 detour 증거와 구분해 이어서 작업한다.

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
