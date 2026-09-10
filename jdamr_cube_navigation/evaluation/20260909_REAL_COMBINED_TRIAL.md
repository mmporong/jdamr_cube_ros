# 실차 통합 장애물 주행 절차 — 2026-09-09

## 목적

실차 사용 횟수를 줄이기 위해 아래 항목을 77 m 복도 왕복 한 번에 수집한다.

1. 저장 지도와 Keepout을 사용한 금지구역 통제
2. 지도에 없던 고정 장애물의 LiDAR 반영과 경로 재계획
3. 통로를 횡단하는 이동 장애물 관측 뒤 감속·정지
4. 통로가 비워진 뒤 새 목표를 보내지 않은 동일 goal UUID의 주행 재개
5. 남은 왕복 경로 수행과 원점 복귀

이 실행은 사람을 분류하는 인지 실험이 아니다. LiDAR가 관측한 형상을 장애물로 다루는
Nav2·Collision Monitor 통합 검증이다. 실제 사람 대신 크기와 이동 경로가 보이는 이동형
대체물을 우선 사용한다. 사람 횡단 촬영을 추가하더라도 별도의 안전 감시자가 물리 전원
차단을 즉시 수행할 수 있는 조건에서만 같은 절차를 적용한다.

## 한 번의 주행에 묶는 순서

| 구간 | 현장 구성 | 확인할 증거 |
|---|---|---|
| 출발 전 | 저장 지도·Keepout, 수납 자세, 고정 장애물 사전 배치 | 마스크 hash, 양쪽 costmap filter, travel-pose gate |
| 첫 외향 구간 | 고정 장애물이 예정 경로 중심을 막되 한쪽 통과 폭은 남김 | 새 scan marking, 장애물 이후 `/plan` 형상, 연속 주행 |
| 고정 장애물 통과 후 | 이동 장애물이 문 쪽에서 반대편까지 통로를 완전히 횡단 | Slowdown/StopZone, 0 속도 명령, odom 감속·정지 |
| 통로가 다시 비워진 뒤 | 추가 목표 입력 없이 대기 | 같은 goal UUID, 비영 속도 명령 재개 |
| 복귀 구간 | 장애물을 통로 밖으로 치운 상태로 원점까지 진행 | 20개 waypoint 결과, 최종 home 오차, 종료 뒤 0 속도 |

고정 장애물의 지도 좌표를 문서에서 임의로 정하지 않는다. 실차 배치 시 예정 외향 경로를
막는지와 로봇 보호 외곽이 지나갈 실제 폭을 현장에서 확인하고, 배치 사진을 원본 실행
디렉터리에 둔다. 이동 장애물은 고정 장애물을 통과한 뒤 시작한다. 두 사건을 동시에 만들면
각 원인에 따른 경로 변경과 정지를 분리할 수 없으므로 포트폴리오 증거로 쓰지 않는다.

## 현장 시나리오 카드

### 준비물과 배치

- 실차에는 팔이 없으므로 `obstacle_base_candidate` 프로필을 사용한다.
- 고정 장애물은 실제 보유한 **갈색 우체국 택배 박스**를 사용한다. 색은 판정 조건이
  아니다. 넓은 면을 로봇 쪽으로 세우고, 약 0.175 m 높이의 LiDAR 스캔 평면이 박스
  몸체를 통과하게 둔다. 이 높이는 URDF의 `base_joint` 0.075 m와 `laser_joint`
  0.100 m 장착 변환을 합한 값이며 박스 실측 높이를 뜻하지 않는다.
- 박스는 예정 중심선을 막되 로봇이 한쪽으로 통과할 실제 폭은 남긴다. 복도 폭 전체를
  막으면 우회 시험이 아니라 경로 차단 시험이 되므로 배치 실패로 다시 놓는다.
- 동적 장애물 횡단 지점은 박스 지점보다 진행 방향 뒤쪽의 문 하나로 정한다. 박스 우회와
  횡단 반응이 한 시점에 겹치지 않도록 두 지점 사이에 충분한 분리 거리를 둔다.
- 카메라는 박스 우회와 횡단 장면에 로봇, 우회 공간, 횡단 시작 문과 반대편 도착 지점이
  함께 보이도록 고정한다. 한 구도로 담기지 않으면 두 대로 나눈다.

### 진행 순서

| 단계 | 현장 행동 | 로봇에 기대하는 행동 | 기록으로 판정할 항목 |
|---|---|---|---|
| S0 준비 | 로봇을 home에 놓고 갈색 박스를 고정 장애물 지점에 세운다. 횡단자는 문 안쪽에서 대기한다. | 움직이지 않고 자동 사전 점검을 마친다. | map·mask hash, Keepout 양쪽 filter, 센서 freshness, planning-only PASS |
| S1 출발·Keepout | 추가 조작 없이 외향 경로를 시작한다. | 금지구역을 침범하지 않고 저장 경로를 따라간다. | AMCL 궤적과 Keepout mask가 겹치지 않음 |
| S2 박스 우회 | 박스를 건드리거나 치우지 않는다. | 박스를 `/scan`과 local costmap에 반영하고, 남은 통로로 계획을 바꿔 접촉 없이 통과한다. | 박스 전후 `/plan` 변화, scan 근접점, 연속 odom, 접촉 없음 |
| S3 사건 분리 | 로봇 전체가 박스를 지난 뒤에만 횡단 준비를 시작한다. | 기존 목표를 계속 수행한다. | 박스 우회 완료 뒤 횡단 시작 시각이 기록됨 |
| S4 사람 횡단 | 횡단자는 문에서 나와 반대편까지 **평소 걷는 속도**로 이동한다. 뛰어들거나 로봇 앞에 멈춰 서지 않는다. | LiDAR 형상을 동적 장애물로 관측하고 필요하면 감속·정지한다. | `/scan`, Collision Monitor 상태, `/cmd_vel`, `/odom` |
| S5 통로 해제·재개 | 횡단자가 진행선 밖으로 완전히 빠진 뒤 통로를 비운다. 새 목표는 입력하지 않는다. | 같은 goal UUID를 유지한 채 비영 속도 명령과 실제 이동을 재개한다. | 정지 전후 동일 UUID, 취소 없음, `/cmd_vel`·odom 재개 |
| S6 복귀·정차 | 갈색 박스를 통로 밖으로 치운 뒤 남은 왕복 경로를 방해하지 않는다. | 남은 waypoint를 수행하고 home에서 정지한다. | 20개 waypoint 성공, 최종 위치 오차, 종료 뒤 0 속도 |

### 사람 횡단의 판정 범위

2D LiDAR는 횡단자를 사람으로 분류하지 않는다. 이번 실차 시나리오에서 주장할 수 있는
범위는 `움직이는 미등록 형상 관측 → 감속·정지 → 통로 해제 → 동일 목표 재개`다.
`사람 감지` 또는 `보행자 인식`으로 표현하려면 별도의 카메라·분류기와 그 출력 토픽이
필요하다.

사람 횡단은 영상 표현용 사건이지 몸으로 제동 한계를 시험하는 절차가 아니다. 횡단자가
혼자 로봇을 운용해야 하거나 물리 전원 차단 담당자가 따로 없으면 사람 대신 끈으로
옆에서 이동할 수 있는 박스·카트 같은 대체물을 사용한다. 로봇 진행선에 사람이 머무르게
해서 정지를 유도하지 않는다.

### 즉시 중단 조건

- 박스가 흔들리거나 우회 통로 쪽으로 넘어짐
- 로봇이 박스에 접촉하거나 금지구역 경계를 침범함
- 횡단자가 로봇 진행선에서 넘어지거나 멈춤
- 정지해야 할 상황에서 계속 접근하거나 통로 해제 뒤 비정상적으로 회전함
- 센서·기록기·Nav2 종료 또는 물리적으로 예상하지 못한 움직임 발생

중단 뒤 같은 실행에 이어 붙이지 않는다. 해당 실행은 실패 증거로 보존하고, 배치를 고친
새 run id로 다시 시작한다.

## 실행 명령

실제 이동은 결정적 이동 차단 훅과 현장 감시 조건이 있는 터미널에서만 시작한다. 파이에서
아래 한 명령이 Nav2, Keepout, 후보 BT, Collision Monitor, 경로 실행기와 MCAP 기록을
함께 관리한다. `--delay 0`은 로봇과 장애물 배치를 끝낸 뒤 실행할 때만 사용한다.

```bash
cd "$HOME/jdamr_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rm -f "$HOME/jdamr_abort"

run_id="real_combined_obstacle_$(date +%Y%m%dT%H%M%S)"
setsid nohup bash \
  "$(ros2 pkg prefix jdamr_cube_navigation)/share/jdamr_cube_navigation/scripts/corridor_autorun.sh" \
  --run-id "$run_id" --delay 0 --navigation-profile obstacle_base_candidate \
  > "$HOME/jdamr_artifacts/${run_id}.start.log" 2>&1 &
```

정상 중단 요청은 파이에서 다음 한 줄만 사용한다. 실행기는 현재 goal을 취소하고 자신이
시작한 Nav2·기록기·계측기 프로세스 그룹을 정리한다.

```bash
touch "$HOME/jdamr_abort"
```

센서 bringup은 `LOCALHOST`, 온보드 Nav2·기록기는 `SUBNET` discovery를 사용한다. 이
비대칭은 같은 파이 안에서 센서 user data를 전달하면서 원시 센서 publisher가 노트북의
구독자를 발견하지 않도록 한다. SSH는 제어와 기록 경로에 포함되지 않으므로 연결이
끊겨도 파이의 실행과 MCAP 기록은 계속된다. 로봇과 사람이 가까워지거나 예상하지 못한
움직임이 보이면 네트워크 명령을 기다리지 않고 물리 전원을 차단한다.

## 실행 전 자동 확인과 예상 대기

자동 실행기는 이동 전에 lifecycle, Keepout 발행, `/cmd_vel` 단일 소유권, costmap 초기화,
배터리, 전체 경로 planning-only와 온보드 자원 게이트를 확인한다. 이 과정에는 기존 계약상
75초 자원 계측이 포함된다. 이 대기를 장애물 시나리오마다 반복하지 않고 통합 주행 앞에서
한 번만 수행한다.

이 절차는 팔이 없는 실차용 `obstacle_base_candidate`를 선택한다. 팔 장착용
`obstacle_candidate`의 수납 자세 게이트를 비활성화하거나 가짜 `/joint_states`로
통과시키지 않는다.

## 성공 판정

다음 항목을 모두 만족해야 `REAL_INTEGRATION PASS`로 분류한다.

- Keepout mask와 원본 지도의 해시가 route 계약과 일치한다.
- global/local costmap 모두 Keepout filter와 mask 수신이 활성이다.
- 고정 장애물 관측 뒤 계획 경로가 갱신되고 접촉 없이 해당 구간을 통과한다.
- 이동 장애물 구간에서 Collision Monitor 정지와 odom 정지가 함께 관측된다.
- 정지 전후 action status의 goal UUID가 같고 취소·교체가 없다.
- 장애물 제거 뒤 비영 `/cmd_vel`과 odom 이동이 재개된다.
- 20개 waypoint가 성공하고 최종 home에 도착한다.
- MCAP CRC·index 검증이 통과하고 필수 토픽 공백이 계약 범위 안이다.
- 종료 뒤 Nav2·route·recorder 잔류 프로세스가 없다.

경로 형상 변화만으로 고정 장애물 우회를 주장하지 않고, 0 속도 명령만으로 물리 정지를
주장하지 않는다. 실제 사람 사용 여부는 영상으로 구분하며 LiDAR 데이터만으로 사람 인식
성공을 주장하지 않는다.

## 주행 뒤 처리

파이의 실행 디렉터리를 노트북 `$HOME/jdamr_artifacts/` 아래로 한 번만 복사한 뒤 기존
후처리기를 사용한다. 원본 MCAP과 현장 영상은 복제하지 않고, 지표·궤적·대표 미디어만
별도 출력 디렉터리에 생성한다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"
export PYTHONPATH="$PWD/jdamr_cube_navigation/evaluation:$PWD/jdamr_cube_navigation:$HOME/.local/share/jdamr-slam-eval/python:$PYTHONPATH"

python3 jdamr_cube_navigation/evaluation/corridor_run_media.py \
  --run-dir "$HOME/jdamr_artifacts/<real_combined_run_id>" \
  --output-dir "$HOME/jdamr_artifacts/<real_combined_run_id>_evidence" \
  --metrics-only
```

실차 실행 전까지의 현재 범위는 `READY_FOR_REAL_TRIAL`이다. 시뮬레이션 통합 PASS와 파이
배포 이력은 실차 결과를 대신하지 않으며, 위 주행의 원본이 생긴 뒤에만 포트폴리오의
실차 장애물 대응 문구와 미디어를 갱신한다.

## 2026-09-09 출발 직전 점검

파이의 센서 bringup 프로세스는 살아 있었지만 `LOCALHOST`로 띄운 새 구독자는 endpoint
이름만 찾고 `/scan`, `/odom`, `/battery_state` user data를 받지 못했다. 같은 구독자를
`SUBNET`으로 바꾸자 세 토픽 표본이 즉시 수신됐고, 반대 방향인 `SUBNET` publisher에서
`LOCALHOST` subscriber로 보내는 무해한 probe 토픽도 통과했다. 이에 원시 센서 publisher는
`LOCALHOST`로 유지하고, 실차 wrapper가 온보드 Nav2·기록기·경로 실행기와 core launch에
`SUBNET`을 전달하도록 수정·배포했다. core launch의 기본값은 시뮬레이션 격리를 위해
`LOCALHOST`로 남겼다. ROS가 같은 호스트의 두 범위 조합을 지원하는 공식 행렬은
[Improved Dynamic Discovery](https://docs.ros.org/en/rolling/Tutorials/Advanced/Improved-Dynamic-Discovery.html)에 있다.

수정 뒤 읽기 전용 실측에서는 `/scan`, `/odom`, `/battery_state`가 수신됐고 배터리는
11.976 V였다. `/joint_states`는 실제 서보 telemetry가 아니라 `joint_state_publisher`가
만든 전 관절 0 rad 표본이었고, 사용자는 실차에 팔이 없음을 확인했다. 따라서 팔 장착용
후보의 travel-pose gate를 우회하지 않고, 차체 외곽용 `obstacle_base_candidate`와
`base_obstacle_protection.yaml`을 별도로 연결했다.
