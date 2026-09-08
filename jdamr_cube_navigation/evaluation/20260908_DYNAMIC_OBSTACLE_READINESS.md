# 이동형 로봇팔 장애물 대응 — 시뮬레이션 통합 검증 완료

## 결론

저장 지도 위에서 주행 중 나타난 장애물을 LiDAR로 감지해 접촉 없이 정지하고, 장애물이
사라진 뒤 새 목표를 보내지 않은 채 원래 목표로 주행을 재개하는 경로를 실제 온보드
Nav2 구성에 연결했다. 2026-09-08 대표 Gazebo 실행은 다음 조건을 모두 통과했다.

- 실차와 같은 `onboard_nav2_core.launch.py`의 `obstacle_candidate` 프로필 사용
- 고정 수납 자세의 SO-101 충돌 형상을 포함한 StopZone·SlowdownZone 적용
- `/joint_states`가 수납 자세에서 벗어나거나 오래되면 목표 전송 또는 주행 지속 차단
- 목표 전송 1회, 취소 0회, 동일 goal UUID로 `EXECUTING → SUCCEEDED`
- Collision Monitor의 StopZone 정지와 장애물 제거 뒤 명령 재개 확인
- 로봇 접촉 0회, 이동형 로봇팔 보호 외곽과 장애물 사이 최소 여유 0.01864 m
- 최종 위치 `(5.9762, 0.0428) m`, 종료 뒤 잔존 프로세스 0

이는 `SIM_INTEGRATION` 범위의 결과다. 실차 제동거리, 사람 인식, 동적 팔 자세 전체,
기능 안전 인증을 뜻하지 않는다.

## 지도·장애물·정지 계층의 역할

저장 지도는 바꾸지 않는다. Keepout은 계단처럼 항상 진입하면 안 되는 영역을 전역·지역
costmap에 넣고, 새 장애물은 LiDAR 관측으로 costmap과 Collision Monitor에 반영한다.
따라서 이번 실행은 기존 지도를 다시 생성하거나 오염시키지 않는다.

```text
저장 지도 + Keepout ──> 경로 계획·재계획
실시간 LiDAR ─────────> costmap의 임시 장애물
실시간 LiDAR ─────────> Collision Monitor의 감속·즉시 정지
고정 팔 자세 ─────────> 보호 외곽 유효성 게이트
```

장애물이 사라지면 costmap 관측이 갱신되고, 후보 BT는 같은 NavigateToPose 요청 안에서
경로를 다시 계산한다. Collision Monitor의 정지는 지도 셀을 영구 변경하는 동작이 아니다.

## 팔을 포함한 보호영역 도출

차체 footprint만으로 접촉 여유를 계산하면 전방으로 나온 그리퍼를 놓칠 수 있다.
`robot_stow_envelope.py`는 production URDF의 초기 관절 자세에서 `arm_` collision mesh
119만여 점을 `base_footprint` 좌표로 변환한다. 가장 앞선 점은
`arm_moving_jaw_link`의 0.302556 m였다.

| 항목 | 값 | 의미 |
|---|---:|---|
| 기존 차체 전방 | 0.230000 m | base footprint |
| 수납 팔 전방 | 0.302556 m | URDF collision 외곽 |
| 기존 StopZone 전방 | 0.280000 m | 팔보다 0.022556 m 짧음 |
| 반응 거리 | 0.028354 m | 최대 scan gap + smoother 한 주기 |
| 제동 거리 | 0.032400 m | 설정 속도·감속도의 등가속도 상한 계산 |
| 후보 StopZone 전방 | 0.400000 m | 보호 외곽 + 정지거리 + 반 셀 대각선 |
| 후보 SlowdownZone 전방 | 0.500000 m | StopZone보다 두 costmap 셀 앞에서 감속 시작 |
| 계산상 잔여 여유 | 0.036689 m | 설정 기반 상한이며 실차 측정값이 아님 |

계산 입력과 유도 결과는 실행별
`assets/onboard_stop_contract.json`에 저장한다. 실제 런타임 값은
`config/mobile_manipulator_protection.yaml`에 있고, 평가 runner는 두 값이 다르면
주행을 시작하지 않는다.

## 런타임 연결

- `onboard_nav2_core.launch.py`: `obstacle_candidate`일 때만 Collision Monitor에
  StopZone 0.40 m와 SlowdownZone 0.50 m를 덮어쓴다. 기본 `corridor` 프로필은 유지한다.
- `corridor_route.py`: 후보 프로필에서 `/joint_states`를 계속 확인한다. 관절 오차가
  0.03 rad를 넘거나 토픽이 stale이면 fail-closed로 목표를 막거나 활성 목표를 취소한다.
- `run_onboard_candidate_smoke.py`: 별도 평가용 Nav2 YAML을 만들지 않고 설치된 후보
  프로필을 그대로 띄운다. 출발 전에 Collision Monitor의 실제 파라미터를 다시 읽어
  계약과 비교한다.
- 온보드 MCAP에는 고주기 `/joint_states` 전체를 넣지 않는다. 출발 전 자세 표본과 판정만
  summary JSON에 보존해 제어 경로의 기록 부하를 늘리지 않는다.

## 대표 실행 증거

원본은 `$HOME/jdamr_artifacts/onboard_candidate_sudden_20260908_v08`에 있다.

| 확인 항목 | 결과 |
|---|---|
| 전체 판정 | PASS |
| 런타임 StopZone / SlowdownZone | 0.40 m / 0.50 m, 설치 노드에서 재조회 |
| 팔 자세 최대 오차 | `4.15e-10 rad` / 허용 `0.03 rad` |
| 목표 상태 | goal UUID 1개, 상태 `2 → 4`, 취소 0 |
| 정지·재개 | StopZone 정지, 동일 목표 명령 재개 `CONFIRMED` |
| 접촉 | filtered/raw 모두 0 |
| 차체 기준 최소 여유 | 0.09120 m |
| 팔 포함 보호 외곽 최소 여유 | 0.01864 m |
| 최종 위치 | `(5.9762, 0.0428) m` |
| MCAP | 1,348,408 B, SHA-256 `797365ab…f409` |
| 전체 실행 산출물 | 1,594,044 B |
| 종료 정리 | 잔존 프로세스 0 |

observer가 scan을 받은 시점부터 0 속도 명령을 받은 시점까지의 0.03725초도 기록했지만,
센서 취득·전송과 물리 제동을 포함하지 않으므로 실차 반응시간으로 사용하지 않는다.

## 미디어와 재현

미디어는
`$HOME/jdamr_artifacts/onboard_candidate_sudden_20260908_v08_media_v02`에 있다.
단순 수치 비교 차트가 아니라 실제 복도 경로, Nav2 계획, 장애물 출현·정지·재개 상태와
차체·팔·StopZone·SlowdownZone의 상대 위치를 한 화면에 재생한다.

- `sudden_stop_same_goal_story.mp4`: H.264/yuv420p, 1280×720, 9초, 178,681 B
- `sudden_stop_same_goal_story.gif`: 768×432, 9초, 97,903 B
- `sudden_stop_protected_clearance.png`: 정지 시점 공간 관계 포스터
- `metrics.json`, `manifest.json`: 핵심 수치와 모든 입력·출력 SHA-256

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"

python3 jdamr_cube_navigation/evaluation/run_onboard_candidate_smoke.py \
  --output-root "$HOME/jdamr_artifacts/<new_run_id>" \
  --domain-id 186 \
  --case sudden_stop_resume

python3 jdamr_cube_navigation/evaluation/render_onboard_sudden_stop_media.py \
  --run-root "$HOME/jdamr_artifacts/<new_run_id>" \
  --output "$HOME/jdamr_artifacts/<new_media_id>"
```

시뮬레이션 runner는 ROS domain 186·187, localhost discovery, 고유 Gazebo partition만
허용한다. 출력 루트가 비어 있지 않으면 기존 증거를 보호하기 위해 종료한다.

## 실차에서 남은 확인

후보 런타임 파일은 파이의 `$HOME/jdamr_ws`에 동기화했다. 패키지 빌드, launch 인자,
설치된 보호 계약 로딩, `corridor_route --help`를 확인했고 Nav2나 이동 명령은 시작하지
않았다. 갱신 전 launch·route 백업은
`$HOME/jdamr_artifacts/onboard_protection_backup_20260908.I7yFqG`에 있다.

첫 실차 검증은 수납 자세·저속·감독 조건의 고정 장애물 우회 1회로 시작한다. 이어서
정면 통로에 물체를 배치해 정지 여유를 외부 줄자로 측정하고, 물체 제거 뒤 같은 목표가
재개되는지 온보드 MCAP과 route goal UUID로 확인한다. 아래 조건 중 하나면 해당 실행은
성공 증거로 쓰지 않는다.

- `/joint_states` 수납 자세 게이트 불통과 또는 stale
- 보호영역 파라미터 불일치
- contact 발생, 목표 취소·교체, 최종 목표 미도착
- MCAP 미완결, 기록 용량 초과, 종료 뒤 owned process 잔존

Depth camera는 이 LiDAR 기반 단계에 필요하지 않다. 사람의 종류·방향·속도를 구분하거나
가까운 저반사·낮은 물체의 3차원 인지가 필요할 때 별도 고도화 범위로 추가한다.
