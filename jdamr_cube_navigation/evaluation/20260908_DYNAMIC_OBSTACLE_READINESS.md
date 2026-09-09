# 이동형 로봇팔 장애물 대응: 시뮬레이션 통합 검증 완료

## 결론

저장 지도 위에서 주행 중 나타난 장애물을 LiDAR로 감지해 접촉 없이 정지하고, 장애물이
사라진 뒤 새 목표를 보내지 않은 채 원래 목표로 주행을 재개하는 경로를 실제 온보드
Nav2 구성에 연결했다. 2026-09-08 좌·우 진입 Gazebo 실행은 다음 조건을 모두 통과했다.

- 실차와 같은 `onboard_nav2_core.launch.py`의 `obstacle_candidate` 프로필 사용
- 고정 수납 자세의 SO-101 충돌 형상을 포함한 StopZone·SlowdownZone 적용
- `/joint_states`가 수납 자세에서 벗어나거나 오래되면 목표 전송 또는 주행 지속 차단
- 목표 전송 1회, 취소 0회, 동일 goal UUID로 `EXECUTING → SUCCEEDED`
- Collision Monitor의 StopZone 정지와 장애물 제거 뒤 명령 재개 확인
- 양방향 실행 모두 로봇 접촉 0회, 보호 외곽 최소 여유 0.03960~0.04503 m
- 양방향 실행 모두 최종 목표 도착, 종료 뒤 잔존 프로세스 0
- 중앙 통로를 막은 고정 박스 우회와 보행자 돌발 횡단을 한 목표 안에서 연속 통과
- 최신 통합 실행에서 보행자가 통로 `y=+1.0 m → y=-1.0 m`를 완전히 횡단

이는 `SIM_INTEGRATION` 범위의 결과다. 실차 제동거리, 사람 인식, 동적 팔 자세 전체,
기능 안전 인증을 뜻하지 않는다.

## 용어와 증거 범위

- **Nav2 plan**: Nav2가 현재 지도와 costmap을 보고 만든 목표까지의 계획 경로다.
- **Collision Monitor**: Nav2 계획과 별도로 LiDAR 관측을 확인해, 보호영역 안의 장애물에
  대해 최종 속도 명령을 감속하거나 0으로 만드는 실행 계층이다.
- **MCAP**: ROS 2 토픽을 시간 순서와 메타데이터와 함께 보존하는 기록 파일이다.
- **ground truth**: Gazebo 월드가 직접 제공한 로봇의 기준 위치다. SLAM이나 AMCL 추정값과
  구분해 실제 우회 거리와 장애물 간격을 계산할 때 썼다.
- **동일 목표 재개**: 장애물 정지 전후 goal UUID가 같고 새 목표 전송이나 기존 목표 취소가
  없었다는 뜻이다. 이것만으로 물리 안전을 주장하지 않고 접촉·정지 증거를 따로 확인한다.

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
  계약과 비교한다. `/joint_states` 토픽이 먼저 나타나도 팔 관절 전체가 담긴 표본을
  받을 때까지 제한 시간 안에서 기다린 뒤 자세를 판정한다.
- 사람형 장애물은 ROS와 Gazebo의 `SetEntityPose` 서비스 연결 하나를 재사용한다.
  통로 경계 `y=±1.0 m` 중 한쪽에서 경로 안으로 들어오고, 정지 확인 뒤 같은 속도로
  반대쪽 경계까지 계속 이동한다. 프레임마다 별도 CLI 프로세스를 만들지 않는다.
- 온보드 MCAP에는 고주기 `/joint_states` 전체를 넣지 않는다. 출발 전 자세 표본과 판정만
  summary JSON에 보존해 제어 경로의 기록 부하를 늘리지 않는다.

## 대표 실행 증거

좌측 진입 원본은
`$HOME/jdamr_artifacts/onboard_candidate_sudden_20260908_v13_followcam`, 우측 진입
원본은 `$HOME/jdamr_artifacts/onboard_candidate_sudden_20260908_v15_followcam_right`에
있다.

| 확인 항목 | 좌측에서 진입 | 우측에서 진입 |
|---|---:|---:|
| 전체 판정 | PASS | PASS |
| 횡단 시간 | 0.93083초 | 0.94267초 |
| 목표 전송 / 취소 | 1회 / 0회 | 1회 / 0회 |
| 동일 목표 재개·도착 | `CONFIRMED` | `CONFIRMED` |
| filtered/raw 접촉 | 0회 / 0회 | 0회 / 0회 |
| 차체 기준 최소 여유 | 0.11216 m | 0.11759 m |
| 팔 포함 보호 외곽 최소 여유 | 0.03960 m | 0.04503 m |
| 최종 위치 | `(6.1311, 0.0468) m` | `(6.0839, 0.0181) m` |
| MCAP | 1,394,538 B | 1,367,337 B |
| Gazebo 센서 원본 | 2,069프레임 | 2,090프레임 |
| 종료 뒤 잔존 프로세스 | 0 | 0 |

두 실행 모두 StopZone 0.40 m, SlowdownZone 0.50 m를 설치 노드에서 다시 읽었고,
팔 자세 오차는 허용치 0.03 rad 안이었다. 각 원본은 1280×720 Gazebo 카메라 센서
프레임이며, MP4 스트림은 30fps로 기록했다.

observer가 scan을 받은 시점부터 0 속도 명령을 받은 시점까지의 0.04537초와
0.06863초도 기록했지만, 센서 취득·전송과 물리 제동을 포함하지 않으므로 실차
반응시간으로 사용하지 않는다.

## 고정 장애물 우회와 돌발 보행자 연속 시나리오

`detour_sudden_stop_resume`는 서로 다른 기능을 따로 보여주는 대신 한 번의 목표에서
연결했다. 출발 전에 0.50×0.40 m 박스를 `(x=-1.0, y=0.0) m`에 고정해 복도 중앙의
직선 경로를 막고, 로봇이 이를 우회한 뒤 `x=1.0 m`를 통과하면 보행자가 옆에서
경로 안으로 들어오게 했다. 보행자는 정지 유도 뒤 사라지는 대신 통로의 반대쪽
경계까지 계속 횡단한다. 최종 실행은
`$HOME/jdamr_artifacts/onboard_candidate_combined_20260908_v05`에 있다.

| 확인 항목 | 결과 |
|---|---:|
| 전체 판정 | PASS |
| 직선 중심선 차단 | 확인 |
| 실제 최대 횡방향 이동 | 0.57716 m |
| 고정 박스 최소 이격 | 0.13620 m |
| 보행자 횡단 범위 | `y=+1.0 → -1.0 m` |
| 보행자 기준 차체 최소 이격 | 0.14018 m |
| 보행자 기준 팔 포함 보호 외곽 최소 이격 | 0.06766 m |
| scan 관측→0 속도 명령 관측 | 0.06306초 |
| 목표 전송 / 취소 | 1회 / 0회 |
| 동일 목표 재개·최종 도착 | `CONFIRMED` / `succeeded` |
| 접촉 | 0회 |
| 최종 위치 | `(6.07430, 0.09130) m` |
| MCAP | 1,627,772 B |

고정 박스 이격은 321개의 ground-truth 표본과 박스의 방향을 포함한 직사각형 경계로
계산했다. 보행자 대응은 Collision Monitor 상태, 0 속도 명령, 물리 정지, 접촉 센서,
goal UUID를 함께 대조했다. 따라서 단순 경로 그림만 보고 우회·정지를 판정하지 않는다.

## MuJoCo 3D 증거 재생

Gazebo 카메라보다 장애물·경로·상태를 한 화면에서 읽기 쉽도록, 위 PASS 실행의
ground truth·Nav2 plan·시나리오 이벤트를 MuJoCo 3D 장면으로 재생한다. 상체의 SO-101은
시각 모델에서 제외하고 모바일 베이스, 실제 크기의 고정 박스, 횡단 보행자만 남겼다.
복도 측면에서 18° 비스듬히 보는 카메라가 주행 거리의 75%만 따라가므로 로봇이
화면 왼쪽에서 오른쪽으로 이동한다. 반투명 벽 너머로 계획 경로, 실제 이동 궤적,
LiDAR 광선, 미니맵과 상태 대시보드를 함께 표시한다.

이 영상은 MuJoCo에서 Nav2를 다시 실행한 독립 물리 실험이 아니다. 주행·접촉·목표 상태의
원본은 Gazebo/ROS 2 MCAP이며, 화면의 LiDAR 광선만 MuJoCo 장면에 raycast해 가시화한다.
manifest는 이 구분과 원본·출력 SHA-256을 보존한다. 렌더러는 프레임을 한 장씩 ffmpeg로
전달하고, 사용 가능 메모리 2 GiB 또는 디스크 1 GiB 아래에서는 중단한다.

- 원본 재생: `$HOME/jdamr_artifacts/mujoco_nav2_combined_20260908_v11`
- 저장소 미디어: `evaluation/media/mujoco_nav2_combined_20260908`
- 대표 영상: `mujoco_nav2_obstacle_challenge.mp4` (1280×720, 24fps, 24초)
- 웹 미리보기: `mujoco_nav2_obstacle_challenge.gif`
- 정지 이미지: `mujoco_nav2_obstacle_challenge.jpg`
- 추적 정보: `mujoco_portfolio_manifest.json`

### Isaac Sim을 현재 로컬 경로에 넣지 않은 이유

Isaac Sim 6.0은 ROS 2 Jazzy·Nav2 연동을 지원하지만, 공식 최소 요구사항이
GeForce RTX 4080, VRAM 16 GB, RAM 32 GB, 저장공간 50 GB다. 현재 호스트의
RTX 5050 Laptop VRAM 8 GB·여유 저장공간 약 3.4 GB에서는 설치·실행을
강행하지 않는다. Isaac Sim은 16 GB 이상 VRAM과 50 GB 이상 여유 공간을
갖춘 별도 워크스테이션이나 클라우드 GPU에서 사람 센서·인지 확장을 시작할
때 재검토한다.

- [Isaac Sim 요구사항](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim 설치 안내](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/quick-install.html)
- [Isaac Sim ROS 2 Nav2 튜토리얼](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_navigation.html)

## 미디어와 재현

미디어는 저장소의
`evaluation/media/gazebo_corridor_crossing_20260908`,
`evaluation/media/onboard_dynamic_obstacle_20260908`과 두 원본 실행의
`portfolio_media_directional`에 있다. 이전 도식 재생본을 우선 증거로 쓰지 않고,
모바일 베이스를 따라가는 Gazebo 3D 카메라 센서의 실제 프레임을 사용한다. 시작점은
파랑, 목표점은 초록, 횡단 보행자는 빨강으로 표시했다. 하단 수치는 각 실행의 MCAP과
Contact sensor에서 가져왔다.

- `gazebo_pedestrian_bidirectional_reel.mp4`: 좌·우 두 실행을 이은 34초 대표 영상
- `gazebo_dynamic_obstacle_highlight.mp4`: 좌측 진입 17초 영상
- `gazebo_dynamic_obstacle_right.mp4`: 우측 진입 17초 영상
- `gazebo_dynamic_obstacle_highlight.gif`: 800×450 웹 미리보기
- `gazebo_obstacle_stop.png`, `gazebo_obstacle_stop_right.png`: 방향별 정지 포스터
- `portfolio_media_manifest.json`: 두 원본·출력 SHA-256과 사용 수치

통로 완전 횡단의 최신 Gazebo 하이라이트·GIF·포스터는
`evaluation/media/gazebo_corridor_crossing_20260908`에 있다. 기존 양방향 릴은
좌·우에서 경로로 진입하는 방향 다양성 증거로 보존하되, 완전 횡단 주장은 최신
통합 실행의 manifest만 근거로 삼는다.

저장소 용량을 줄이기 위해 편집 전 원본과 전체 길이 영상은 artifact 루트에만 보존한다.
양방향 릴은 독립된 단일 보행자 실행 2개를 연결한 것으로, 동시 다중 보행자 검증이 아니다.

```bash
cd "$HOME/jdamr_cube_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_cube_ws/install/setup.bash"

python3 jdamr_cube_navigation/evaluation/run_onboard_candidate_smoke.py \
  --output-root "$HOME/jdamr_artifacts/<new_run_id>" \
  --domain-id 186 \
  --case sudden_stop_resume \
  --record-simulator-video \
  --pedestrian-entry left

python3 jdamr_cube_navigation/evaluation/render_simulator_portfolio_media.py \
  --run-root "$HOME/jdamr_artifacts/<new_run_id>" \
  --output-dir "$HOME/jdamr_artifacts/<new_media_id>"

python3 jdamr_cube_navigation/evaluation/render_bidirectional_reel.py \
  --left-manifest "$HOME/jdamr_artifacts/<left_media_id>/portfolio_media_manifest.json" \
  --right-manifest "$HOME/jdamr_artifacts/<right_media_id>/portfolio_media_manifest.json" \
  --output-dir "$HOME/jdamr_artifacts/<reel_id>"

python3 jdamr_cube_navigation/evaluation/navigation_dashboard.py \
  --port 8765 \
  --media-manifest jdamr_cube_navigation/evaluation/media/onboard_dynamic_obstacle_20260908/portfolio_media_manifest.json \
  --video jdamr_cube_navigation/evaluation/media/onboard_dynamic_obstacle_20260908/gazebo_pedestrian_bidirectional_reel.mp4

jdamr_cube_navigation/evaluation/setup_mujoco_renderer.sh
jdamr_cube_navigation/evaluation/run_mujoco_portfolio_render.sh \
  --run-root "$HOME/jdamr_artifacts/onboard_candidate_combined_20260908_v05" \
  --output-dir "$HOME/jdamr_artifacts/<new_mujoco_media_id>"

python3 jdamr_cube_navigation/evaluation/navigation_dashboard.py \
  --port 8765 --no-ros \
  --summary "$HOME/jdamr_artifacts/onboard_candidate_combined_20260908_v05/summary.json" \
  --video \
  jdamr_cube_navigation/evaluation/media/mujoco_nav2_combined_20260908/mujoco_nav2_obstacle_challenge.mp4
```

우측 진입 자료는 `--pedestrian-entry right`와 서로 다른 run·media 출력 디렉터리로
같은 절차를 한 번 실행한다. 양쪽 manifest가 모두 PASS일 때만 마지막 릴 생성기가
결과를 연결한다.

시뮬레이션 runner는 ROS domain 186·187, localhost discovery, 고유 Gazebo partition만
허용한다. 출력 루트가 비어 있지 않으면 기존 증거를 보호하기 위해 종료한다.

## 실차에서 남은 확인

후보 런타임 파일은 파이의 `$HOME/jdamr_ws`에 동기화했다. 패키지 빌드, launch 인자,
설치된 보호 계약 로딩, `corridor_route --help`를 확인했고 Nav2나 이동 명령은 시작하지
않았다. 갱신 전 launch·route 백업은
`$HOME/jdamr_artifacts/onboard_protection_backup_20260908.I7yFqG`에 있다.

실차 사용 횟수를 줄이기 위해 고정 장애물 우회와 이동 장애물 정지·재개는 복도 왕복
한 번에 순차 실행한다. 먼저 예정 경로를 막은 고정 장애물을 우회하고, 해당 장애물을
통과한 뒤 별도 이동 장애물이 통로를 완전히 횡단하도록 구성한다. 두 사건을 겹치지 않아
경로 재계획과 Collision Monitor 정지의 원인을 분리한다. 저장 지도·Keepout 준수와 최종
원점 복귀도 같은 MCAP에 기록한다. 실행 순서와 판정 기준은
[실차 통합 장애물 주행 절차](20260909_REAL_COMBINED_TRIAL.md)를 따른다. 아래 조건 중
하나면 해당 실행은 성공 증거로 쓰지 않는다.

- `/joint_states` 수납 자세 게이트 불통과 또는 stale
- 보호영역 파라미터 불일치
- contact 발생, 목표 취소·교체, 최종 목표 미도착
- MCAP 미완결, 기록 용량 초과, 종료 뒤 owned process 잔존

Depth camera는 이 LiDAR 기반 단계에 필요하지 않다. 사람의 종류·방향·속도를 구분하거나
가까운 저반사·낮은 물체의 3차원 인지가 필요할 때 별도 고도화 범위로 추가한다.
