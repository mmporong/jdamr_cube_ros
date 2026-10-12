# 2026-10-06–10-12 JD-AMR 상태와 이어서 할 일 (Codex용)

Claude Code가 2026-10-06 오후부터 10-08까지 처리한 내용과 남은 일을 Codex가 이어받을 수 있게 정리한 인계다. 최초 인계의 소프트웨어 기준은 `0c79182`이며, 인계 문서와 README 문서 지도는 `b1daa6a`로 `origin/main`에 반영됐다. 2026-10-12 Codex 작업 시작 시 로컬 HEAD와 `origin/main` 추적 참조가 `b1daa6a`로 같았다. 사실·수치·경로는 아래와 원 기록에 있다. 새 숫자를 만들지 말고, 추론은 추론으로 표시한다. 최초 인계 작성 시점에 Pi는 꺼져 있었다.

## 0. 지금 상태 (한눈에)

| 항목 | 상태 | 근거 |
|---|---|---|
| 이동 방식 기본값 | NavFn + RPP. MPPI는 `--transit navfn-mppi`·`lattice-mppi` 비교 옵션만 | `restaurant_service.py` `DEFAULT_TRANSIT`, README 서빙 행 |
| 박스 주차 | 목표 간격 1 cm(`config/box_parking_contract.yaml`), 면 방향은 깊이 평면 법선, 판정은 가장 앞선 프레임 모서리 | 커밋 `ea3fbf0`·`db20d5f`·`0ceeab8` |
| 도크 방향 | 1–5°면 1° 안까지 제자리 보정(최대 3회). 10-06 9회 1.45–4.85° → 0.22° 이하 | 커밋 `4d9dd3d`, run별 `dock_heading_*` 이벤트 |
| 실행기 CPU | 경로 노드마다 EventsExecutor. 실행기 Pi 한 코어의 78 % → 25–34 % | 커밋 `3603ea1`, run별 `load_profile.jsonl` |
| 테스트 기준선 | 내비게이션 패키지 2572 통과, 5 실패(AMCL Axis B, §1-B). 10-12 전체 재실행 결과가 아님 | 커밋 `0c79182` |
| Pi 이번 반영 | `ace59b8`, 하향 카메라 기하, USB 열거 판정, 수동 웹 명령 대기, 탐색 Spin의 odom 검증 반영·빌드·설치본 해시 대조 완료 | §2-14·16·17. 관측 지점 이동과 Spin action 성공은 확인했지만 새 수정본의 실차 주차 사이클은 미확인 |
| Pi 배포 범위 | 필요한 파일만 복사·빌드했다. Pi 작업공간은 Git 저장소가 아니며 PC `main` 전체와 같다고 주장하지 않는다 | §2-17의 파일별 설치본 해시·백업·빌드 기록 |
| 로봇 위치·복귀점 | 17:24 재등록 HOME: 지도 (0.126851, 0.057200) m, yaw 0.111160 rad(약 6.37°), `parking_direction: forward` | §2-17. 14:42 HOME을 대체한다. 마지막에는 운영자가 도크에 놓겠다고 했으며 실제 배치 완료는 아직 관측하지 않음 |
| 차체 결합 | 10-12 결합 완료. 결합 뒤 돌출부·팔·적재물의 치수와 가동 상태는 미확인 | 10-12 운영자 보고. `new_base_geometry.yaml`의 주장 범위는 하단 프레임 |
| LiDAR | USB 연결 복구 뒤 수신 확인. 장착 위치·기존 TF 유지. 차체 후방 기둥 반사 두 군집만 기존 자기 반사 필터로 제거 | §2-15·17. 80 scan 재생에서 779점 제거·0.45 m보다 먼 반사 변경 0. 기둥의 물리 치수 실측은 아님 |
| 접촉센서 | 운영자가 5V·GND 두 선만 연결했다고 확인. 눌림 입력·정지 연동은 사용할 수 없음 | 신호선 미연결, 모델 미확인. 펌웨어에도 접촉 입력 경로 없음 |
| 뎁스 카메라 | 상판 뒤끝 하향 장착. x=-0.275 m, z=0.9071 m·pitch=26.7465°·roll=0.6838° 계산. 17:55 물리 USB 재연결 뒤 실제 영상 복구 확인 | §2-16·17. `derived`, y=0·yaw=0·상하판 정렬은 가정. 1 cm 주차 외부 실측은 미완료. 현재 서비스는 종료 |
| 바퀴 모터선 | UART 읽기 오류 0건. 재연결 뒤 관측 지점 주행과 Spin action 성공 확인. 오른쪽 바퀴 들뜸·나사 조임은 운영자 보고 | §2-13·17. Spin odom과 IMU 회전량은 부합하지만 실물 병진은 외부 계측하지 않음 |
| Pi 접속·서비스 | Wi-Fi 접속 정상. 정지 요청 뒤 Nav2·이동 실행기·카메라·수동 제어·대시보드·PC RViz/중계/기록기/터널 종료 | §2-17. 사용자 기존 `jdamr-base` PID 1284만 유지. 다음 시작은 새 HOME에서 새 `init` 필요 |

## 1. 할 일

### A0. 차체 결합 뒤 이번 요청 범위 (2026-10-12)

운영자는 다른 반영 가능 항목부터 진행하고, 재연결한 바퀴 모터선을 확인해 달라고 요청했다. §1-B의 AMCL 선택지나 §1-C의 미지시 고도화를 승인한 것으로 확대하지 않는다.

1. **Pi 접속과 모터 확인.** Wi-Fi 변경 뒤 기존 주소 접속이 복구됐다. UART 읽기에서 좌우 오류 0건·CRC 오류 0건·워치독 정지를 확인했다(§2-13). 펌웨어 기대값은 좌측 ID 2·우측 ID 1이다. 이후 사용자의 출발 지시에 따라 관측 지점 이동과 제한된 Spin action을 성공했다(§2-17). 이를 좌우 실물 배선 도면이나 외부 이동 계측으로 확대하지 않는다. 운영자는 오른쪽 바퀴 들뜸이 약간 개선됐다고 했고 정지 중 나사를 조였다. 충전 중에는 구동하지 않는다. 등록 HOME은 17:24의 재등록 값이며, 다음 새 세션에서 `init`으로 배치와 위치를 확인한다.
2. **차체·LiDAR 상태 반영.** 하단 프레임 치수·라이다 TF는 유지한다. 운영자의 네 기둥 설명과 이동 전후 스캔으로 후방 자기 반사 두 군집을 확인해 기존 필터에 반영했다(§2-17). 영구 설정은 `$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_bringup/config/self_filter_columns_20261012.yaml`이다. `self_filter_hold_flow_model.yaml`은 계속 `model_not_measured`이며 실측 설정으로 취급하지 않는다. 반사점 외접 범위는 기둥 치수나 결합 차체 전체 제원을 뜻하지 않는다.
3. **접촉센서 배선과 입력 확인.** 운영자는 5V·GND 두 선만 연결했다고 확인했다. 신호선이 없고 펌웨어 입력 경로도 없어 눌림 검증은 미완료다. 모델·단자 표기를 요청한 상태다. 핀·출력 형식이 확인되기 전에는 GPIO 번호를 추측해 펌웨어에 넣지 않는다. 입력 전환을 읽고 기존 정지 경로와 연동하는 확인은 그 뒤에 수행한다. 현재 소프트웨어 래치 비상정지와 접촉센서 연동은 별개다.
4. **뎁스 설치·계산 보정.** 운영자는 높이 약 1 m, 상판 뒤끝, 약간 하향으로 장착했고 URDF로 위치를 계산하라고 지시했다. 계산값·가정·원본 깊이는 §2-16과 `config/camera_mount_derived_20261012.yaml`에 기록했다. 기존 9/17의 x=0.065 m·z=0.215 m·수평 설정은 새 설치에 사용하지 않는다. 상판 모서리 주차는 여전히 `20261006_TABLE_EDGE_DOCKING_PLAN.md`의 별도 설계 후보다.
5. **실행기 수정 반영.** `ace59b8` 배포·Pi 검증 뒤 실제 `go`가 이동 전 파라미터 읽기를 통과하고 관측 지점까지 도달했다. 마지막 실패는 Spin action 성공 뒤 지도 위치 보정을 병진으로 오판한 별도 문제였다. 종료 후 탐색 회전 검증을 odom 기준으로 고쳤고 실패 사유도 분리했다(§2-17). 새 수정본의 실차 주차·복귀 사이클 확인은 다음 출발에서 수행한다.

### A. Pi 반영 상태와 다음 주행 준비 (사용자 요청 범위에서)

1. **`ace59b8` 반영 완료, 이동 전 읽기 단계 통과 확인.** 상주 실행기(`jdamr-box-executor`)가 `restaurant_service.py`를 불러오므로 기동할 때 설치된 수정본을 읽는다.
   - 10-12 수행: Pi 백업 → 소스·관련 테스트·누락된 YAML 테스트 입력을 `rsync -cR`로 복사 → `colcon build --packages-select jdamr_cube_navigation`(복사 설치) → Pi 테스트 6건 통과 → 소스/설치본 SHA-256 대조 → 기존 `jdamr_depart.start_executor()`로 비구동 기동 확인 → 종료. 상세·백업·로그는 §2-14.
   - 실행기는 초기 inactive였으며 확인 때 PID 2479로 `active/running`에 도달했다. 요청 파일이나 이동 명령을 추가하지 않았고 확인 뒤 inactive로 복원했다. 다음 사용은 기존 `session-start` 경로로 기동한다. 이를 Nav2 활성화나 실제 사이클 성공으로 기록하지 않는다.
   - 10-12 실제 `go`에서 이동 전 읽기 단계 통과와 관측 지점 도착을 확인했다. 재요청 발생 시 실행기 저널에 `parameter read timeout: node=... attempt=1/2` 경고가 남는 계약은 유지한다. 박스 최종 주차·복귀 성공은 별도 확인 항목이다.
2. **시작점과 `init`.** 10-12부터 등록 HOME은 새 시작점이다. 같은 장소에서 다음 세션을 시작할 때 기존 `jdamr_depart.py init` 경로를 쓴다. 주행 도중 재개에서는 현재 실제 위치를 보존한다.
   - 10-06 17:52에는 도크 주변의 지도에 없는 물체 때문에 스캔 일치율(inlier)이 0.34–0.43으로 기준 0.45 아래였다. 지역 매칭과 전역 탐색이 같은 자리를 가리키면 `init --local-only --min-inlier 0.30`(저장소의 운영자 재정의, 하한 0.25)으로 통과시켰고, AMCL 일치 검사(5 cm, 2°)는 그대로 둔다.
   - 복구 주행 전에는 AMCL 공분산을 먼저 본다(평소 방향 분산 약 0.009 rad²). 10-06 17:44에 이를 보지 않고 도크 복귀를 보내 방향 표준편차가 45°까지 무너졌다(§2-8).

### B. 사용자 결정 대기: AMCL Axis B 테스트 5건

`test_amcl_axis_b.py` 4건, `test_amcl_fault_benchmark.py::test_axis_b_final_identity_and_root_escape` 1건.

- 원인(확인)
  1. 2026-09-10 홈 오프로드로 `~/jdamr_artifacts`가 `/data/lim/jdamr_artifacts` 심링크가 됐다 → `G004 root must be canonical`.
  2. 해시 봉인된 `contract.json`에 `/home/lim/jdamr_artifacts/...` 절대 경로가 있다 → 루트를 정규화하면 `escaped final root`.
  3. 계약이 가리키는 원본 URDF(`jdamr_cube_nav_eval.urdf`)는 gitignore 생성물이라 9/21에 다시 만들어져 해시가 다르다. 같은 바이트는 `/data/lim/jdamr_artifacts/sim_nav_obstacle_eval_20260906_v64_full25_cyclone/assets/`에 있다.
- 선택지
  - A. 계약·자산을 저장소 스냅샷으로 옮기고 기록된 절대 경로 접두사를 번역해 검증. 어느 PC에서나 통과하지만 경로 결합 의미가 바뀐다.
  - B. 루트 위쪽 심링크만 허용(루트 안 심링크는 계속 거부), 원본 출처 검사만 봉인 스냅샷으로. 변경이 가장 작고 이 PC의 외부 산출물 의존은 남는다. **Claude 권장.**
  - C. 고치지 않고 환경 의존 실패로 기록.
- 사용자가 고르기 전에는 손대지 않는다. 사용자는 관문을 조용히 초록으로 만드는 것을 원하지 않는다.

### C. 고도화 후보 (사용자 지시 없음, 목록만)

10-08 "현업·포트폴리오 기준 고도화" 질문에 대한 답과 사용자 응답이다. 사용자가 고르기 전에는 진행하지 않는다.

| 후보 | 사용자 응답·상태 |
|---|---|
| 1 cm 주차 외부 실측 반복 | "이미 된다, 두세 번 주행으로 확인"(구두). 진행 안 함 |
| 의자 충돌 대응 | "운영자가 의자를 갑자기 내서 그런 것, 신경 안 써도 된다". 진행 안 함 |
| 구간별 시간 상한(예상 시간 × 배수 넘으면 정지·복구) | 지시 없음. 근거: 10-06 18:12 도크 대기점 구간이 평소 26–42 s인데 0.005–0.015 m/s로 기어 148 s까지 진행 검사(10 s에 5 cm)를 통과했다. 남은 상한은 `RETURN_TIMEOUT_S` 600 s뿐 |
| 위치 추정 관문을 실행기 안에 | 지시 없음. 지금은 비교 시리즈 스크립트에만 있다(`ab_series_wheelfix/amcl_gate.py`, 방향 8°·위치 12 cm) |
| MPPI 비교 구성의 도크 대기점 구간도 RPP로 끝내기 | 지시 없음. 경유 이동은 `37ec84c`로 MPPI가 사전 지점까지만 맡지만, `home_dock_approach`는 MPPI 구성에서 MPPI가 끝까지 맡아 18:12에 목표를 32 cm 지나쳤다 |
| MPPI 비교 표본 추가 | 지시 없음. 유효 표본은 RPP 3회·MPPI 2회(§2-1) |
| 바퀴 odom + 자이로 EKF 연결 | 질문에 답만 함(§2-11). 조건이던 "큰 회전 표본 추가"는 10-01 이후 주행마다 저장한 `onboard_bag`으로 Pi 없이 확인할 수 있다(`scripts/map_20260930_manual/imu_check.sh <run_dir>`) |
| 근거리 센서(ToF)·범퍼 | 10-12 접촉센서 5V·GND 연결 보고가 추가됨(§1-A0). 신호 입력·정지 연동은 미확인. ToF는 지시 없음. LiDAR는 차체 앞 0.205 m 안을 못 본다 |

### D. 문서·발행 (각자 세션)

- 포트폴리오: `20261006_CONTROLLER_SELECTION_PORTFOLIO_HANDOFF.md`(10-08 갱신, 반복 비교 3.4절·도크 보정·1 cm·CPU 반영).
- 가든: `20261006_CONTROLLER_SELECTION_GARDEN_HANDOFF.md`(10-08 갱신, 반복 비교 3.4절·"틀렸던 판단" 행 추가).
- **공개 금지:** 면접 QA 문서(`20261002_JDAMR_면접QA_50.md`, 최신 커밋 `076c3b7`)는 로컬 브랜치 `local/interview-qa`에만 둔다. push하지 않는다. main·원격·포트폴리오·가든 어디에도 넣거나 링크하지 않는다.

## 2. 10-06 오후–10-12 사실 기록

### 2-1. 추종기 반복 비교 (10-06 17:10–17:36, `go table_02`)

MPPI 구성은 사전 지점까지만 맡고(전진만, 목표 방향 비용은 마지막 0.1 m) 관측 지점까지는 RPP가 이었다. 사이클은 첫 목표 수락부터 도크 도착까지다.

| 회차 | 구성 | 사이클 | 누적 회전 / 필요한 순회전 | 주차(가장 앞선 모서리, 면 각) |
|---|---|---|---|---|
| 1 | RPP | 163.3 s | 554° / 445° | 0.85 cm 0.62°, 0.94 cm 0.89° |
| 2 | MPPI | 359.9 s | 1398° / 502° | 1.08 cm 0.28°, 1.25 cm 0.54° |
| 3 | RPP | 175.4 s | 580° / 453° | 0.76 cm 0.58°, 1.01 cm 0.29° |
| 4 | MPPI | 262.9 s | 813° / 460° | 0.98 cm 0.28°, 0.94 cm 0.61° |
| 5 | RPP | 173.2 s | 586° / 447° | 1.19 cm 0.0°, 0.92 cm 0.39° |

- 2회차는 물 받는 곳 사전 지점에서 78 s 뒤 진행 없음(105)으로 끝나 4 s 대기 후 재시도했다.
- 6회차(MPPI)는 table_02 관측에서 실패했다. 직후 운영자가 바퀴 풀림을 발견하고 다시 체결했다(구두, 로그 기록 없음). 운영자 판단으로 6회차만 제외하고 1–5회차는 유효 데이터로 쓴다.
- 기록: `$HOME/jdamr_data/analysis_20261006/ab_series_1710/series.log`, run 폴더 `table_02_20261006_{171039,171405,172049,172419,172919,173249}`.

### 2-2. 이동 트리의 지난 경유점 제거 (커밋 `aac1828`)

- 현상: 오전 Lattice + MPPI의 고리 경로, 16:48 RPP의 80 s 동안 제자리 회전 11번(약 800°).
- 원인(확인): `RemovePassedGoals`가 재계획 가지 안에 있어 10 s 재계획 때만 돌았다. 지난 경유점이 남은 채 재계획돼 경로가 되돌아갔다. 재현에서 남기면 Lattice 5.43 m·NavFn 2.56 m, 지우면 1.61·1.64 m(`$HOME/jdamr_data/analysis_20261006/loop_replay/results.json`).
- 조치: 세 이동 트리 모두 1 Hz 틱마다 먼저 지우고, 목표가 바뀌면 바로 재계획.

### 2-3. MPPI가 느린 이유 (1·2회차 bag)

- RPP 이동 구간: 직진 0.104–0.118 m/s, 제자리 회전 0.27–0.29 rad/s. 상한(0.12 m/s, 0.3 rad/s) 근처다.
- MPPI 사전 지점 3구간: 직진 0.015·0.061·0.031 m/s, 제자리 회전 0.12–0.18 rad/s, 제자리 회전 비율 16·90·62 %.
- `/cmd_vel_nav`(컨트롤러 출력)와 `/cmd_vel`(최종 명령)이 같다. 느린 원인은 평활기·Collision Monitor가 아니라 MPPI 출력이다.
- [추론] 표본 500개를 상한에서 잘라 평균하는 구조, 좁은 통로에서 느린 궤적이 비용이 낮은 점, Pi 4의 5.8–9.9 Hz 제어 주기(권장 20–30 Hz)가 겹친 것으로 본다. 분리 실험은 하지 않았다.
- 낮은 각속도(0.05–0.15 rad/s) 제자리 회전에서 odom 각속도 변동계수가 약 0.3, 0.3 rad/s에서는 0.03–0.04였다. 사용자가 본 "떨리며 회전"과 맞는다. 바퀴 하나가 초당 약 1.4–4 cm로 도는 저속 영역 때문으로 추론하며, 바퀴 풀림 영향과는 분리하지 못했다.

### 2-4. 실행기 CPU (커밋 `3603ea1`)

- 원인: rclpy 대기 집합 실행기가 초당 약 120개 메시지마다 엔티티 약 80개를 다시 모았다(PC 격리 재생 17.3 % → EventsExecutor 3.5 %).
- 실주행: 14:11 run 실행기 평균 78 % → 16:27 run 25 %, Pi 평균 57 → 43 %, 최고 온도 74.0 → 70.6 °C. 같은 날 이후 run은 실행기 28–34 %, Pi 평균 45–53 %, 최고 73.5 °C.

### 2-5. `init` 활성화 실패 (커밋 `b1deb7a`, Pi 배포됨)

- 현상: 10-06 17:53–18:00 `init` 활성화가 `request interrupted, failed or timed out`으로 4번 연속 실패.
- 원인(확인): `activate_navigation.py`가 AMCL 초기화를 기다리며 `rclpy.spin_once(node)`를 불렀다. 이 호출이 노드를 전용 EventsExecutor에서 떼어 내 이후 `map_server` `get_state` 응답이 처리되지 않았다. Pi에서 같은 절차가 waitset 실행기로는 통과, EventsExecutor로는 실패(3/3)했고, rclpy `Executor.add_node`·`Node.executor` setter 소스로 기전을 확인했다.
- 조치: 두 곳을 `spin_node()`로. 배포 직후 `init` 성공(AMCL과 스캔 매칭 차 0.004 m, 0.4°).

### 2-6. 실행기 이동 전 실패 (커밋 `ace59b8`, 당시 Pi 미배포·10-12 반영은 §2-14)

- 현상: 10-06 10:26·17:09 사이클이 이동 전 `request interrupted, failed or timed out`으로 끝남(run `table_02_20261006_102633`, `_170919`). 10:26은 EventsExecutor 도입(15:23) 전이다.
- 위치(코드 경로 분석으로 확인): 이동 전 단계에서 이 예외를 위로 올리는 경로는 `verify_live_maps()`의 `map_server`·`keepout_filter_mask_server` `yaml_filename` 읽기뿐이다. 실행기는 시도마다 새 노드라 매번 새 클라이언트의 첫 요청이고, 한 번 2 s 기다렸다.
- 기전(같은 조건, 이번 두 건의 서버 로그는 미확인): `20260929_RUNTIME_ROOT_CAUSE.md`에 서버 로그로 남은 첫 응답 유실(`client will not receive response`).
- 조치: `_read_parameters`도 `require_active`의 GetState처럼 첫 요청이 미완료로 끝나고 중단 요청이 없을 때만 같은 5 s 예산 안에서 한 번 더 보낸다. 실패 문구에 노드 이름, `_wait` 문구에 중단·무응답·예외 구분을 붙였다. 관련 테스트 578건 통과, 새 테스트 3건은 수정 전 코드에서 실패한다.

### 2-7. 6회차 실패 (10-06 17:35)

- table_02 사전 지점 MPPI 구간에서 AMCL 방향 분산이 0.009 → 0.027 rad²로 커졌다. 관측 지점에서 박스 방향이 15° 비껴 보였고 깊이 카메라가 박스 앞면을 찾지 못했다. 탐색 회전 중 5 cm가 밀려 `nav2_spin_failed`로 끝났다.
- 직후 바퀴 풀림 발견(§2-1).

### 2-8. 복구 주행 판단 오류와 위치 추정 붕괴 (10-06 17:37–17:45)

- 도크 복귀 1·2차: 도크 앞 후진 경로가 `is_path_valid`에서 거부(이동 없음). 도크 오른쪽 0.25–0.35 m 옆 global costmap에 정적 지도에는 없는 치명 표시(253/254)가 있었다. 로봇 위치에서는 table_02 박스가 그쪽을 가려 LiDAR가 지우지 못했다.
- 3차: `clear_entirely_global_costmap` 뒤 복귀를 보냈다. 26 s 주행 뒤 방향 표준편차 45°, 위치 0.8 m로 무너졌고, 도크 앞 회전은 Spin `COLLISION_AHEAD`(703)로 거부됐다. 사용자 "정지" 요청으로 멈췄다.
- 교훈: 실패 시점 방향 분산이 평소의 3배였는데 확인하지 않고 주행을 보냈다. "LiDAR에 안 보인다"는 판단 자체가 AMCL 자세 투영이었다. [추론] costmap 잔상도 위치 추정 오류의 증상일 수 있다.

### 2-9. 의자 충돌 시리즈 폐기 (10-06 18:05–18:14)

- 바퀴 체결 뒤 새 시리즈(`ab_series_wheelfix`) 2회차(MPPI)의 도크 대기점 구간에서 운영자가 꺼낸 의자와 충돌. 운영자 지시로 이 시리즈 전체를 폐기했다(`runs/table_02_20261006_180518/DISCARDED.txt`, `_180848/DISCARDED.txt`).
- 로봇은 (0.81, 0.21)에서 정지 명령으로 멈췄다. 속도 명령 0 확인.

### 2-10. 테스트 기준선 (커밋 `0c79182`)

- 28 실패·9 오류 → 5 실패·0 오류, 2572 통과(PYTHONPATH 유무 둘 다 같음).
- G005 프론티어 평가: 9/15 `laser_joint` origin 변경(0 0 0.1 → −0.010 0 0.075)으로 drift 검사에 걸렸다. 봉인 커밋 `5c84289`의 입력 5개를 `test/fixtures/sealed_g005_5c84289/`에 바이트 그대로 두고 테스트가 읽게 했다. 다시 만든 자산의 `asset_tree_sha256`이 봉인 매니페스트와 같다.
- G006 후보 번들: 9/17 매핑 런치 구성 변경. 봉인 커밋 `84b9128`의 원본 21개 스냅샷(`test/fixtures/sealed_g006_84b9128/`)으로 빌드해 `bundle_id`가 봉인 번들과 같다.
- `test/conftest.py`: PYTHONPATH 없이도 evaluation 모듈을 찾는다.
- 린트: import 순서·들여쓰기·따옴표·docstring만. 생산 코드 4개 파일은 AST가 같다.
- 남은 것: AMCL Axis B 5건(§1-B). 평가 에이전트 추정으로 이 워크스페이스는 복사 설치라 G005 실제 `ros2 launch`는 실패할 것(실행 확인 안 함, 주행 경로와 무관).

### 2-11. 바퀴 odom + 자이로 EKF

- 10-01 `ekf_odom_imu.yaml`·`imu_bias_relay`를 main에 넣었고(`dcd6b11`) 실행 구성에는 연결하지 않았다.
- 보류 이유(`20260929_PARKING_FAILURES.md` §20.8–20.9): 큰 회전 표본 1건(스캔 정합 기준 odom 1.88°, 바이어스 뺀 자이로 0.25°, EKF 0.19°), 날마다 다른 자이로 바이어스(9/15–16 ±0.002, 10-01 −0.0114 rad/s), IMU 보정 대장 `UNVERIFIED`(융합 불허).
- AMCL과의 관계: EKF는 바퀴 odom + 자이로로 `odom → base`를 내고, AMCL은 스캔을 지도에 맞춰 `map → odom`을 낸다. 대체가 아니라 보완이며, AMCL 입자 예측이 그 odom 이동량을 쓴다.

### 2-12. 문서 정정 (커밋 `4518da3`)

README 서빙 행과 두 인계 문서를 위 사실로 고쳤고, 별도 검증 에이전트가 원본 수치와 대조했다. `jdamr_depart.py --final-gap-m` 도움말은 0.01(계약값)로 고쳐 PC 실행본(`~/jdamr_data/map_20260930_manual/tools/`)에도 반영했다.

### 2-13. 차체 결합 뒤 Pi 접속·모터 읽기 (2026-10-12)

- 초기 SSH는 `No route to host`, mDNS 조회도 실패했다. 운영자가 Wi-Fi 변경을 알린 뒤 12:46에 `jdamr.local` → `192.168.0.159`, SSH·hostname `jdamr`·wlan0 같은 주소를 확인했다. 초기 실패의 원인을 단정하지 않는다.
- 접속 때 베이스·내비게이션·실행기는 inactive였다. `Linger=yes`이며 `/dev/ttyS0`을 점유한 프로세스가 없었다. 베이스 드라이버를 기동하거나 속도 지령을 보내지 않고 기존 프로토콜 파서로 UART를 읽었다. 읽기가 끝난 뒤 시리얼 설정을 복원하고 포트를 닫았다.
- 12:48:34부터 약 5 s 동안 상태 패킷 250개, CRC 오류 0건, 좌우 읽기 오류 각각 0건, 전 패킷 `flags=0x04`(워치독 정지)였다. 배터리 읽기는 12.488–12.528 V였으며, 충전 케이블 연결 여부를 이 전압으로 판정하지 않는다. 엔코더 누적값 범위는 좌 1 count·우 0 count였다.
- 판정: 두 바퀴 서보의 통신·위치 읽기 응답과 워치독 상태는 확인했다. 실제 좌우 배치·전후진 방향·하중이 걸린 구동·구동 뒤 물리 정지는 검증하지 않았다.
- 같은 시점 USB에는 Astra S가 열거됐지만 LiDAR 어댑터와 `/dev/ydlidar_g4`·`ttyUSB*`는 없었다. LiDAR 장착 위치가 같다는 운영자 보고와 USB 연결 상태는 구분한다. 구동 시험은 보류하고 케이블 연결 상태를 요청했다. 자기 반사 필터도 활성화하지 않았다.
- 원본: PC `$HOME/jdamr_data/analysis_20261012_reassembly_124830/uart_stdout.json`(모든 상태 샘플), `capture_uart_readonly.py`(읽기 코드), `uart_stderr.txt`.

### 2-14. `ace59b8` Pi 배포·검증 (2026-10-12)

- 배포 전 Pi 소스와 설치본의 SHA-256 `14ff1d30…`이 `b1deb7a`의 `restaurant_service.py`와 같았다. 최신 PC 소스는 `ace59b8` 이후 변경되지 않았으며 SHA-256은 `c0dce4f2…`이다.
- PC의 기존 `test_restaurant_service.py -k parameter_reader` 6건 통과 후 Pi에 소스와 테스트를 복사했다. 첫 Pi 빌드는 통과했으나 테스트 입력 `test/fixtures/restaurant_service_contract_gated.yaml`이 Pi에 없어 5건 실패·1건 통과였다. 수정 코드의 실패로 단정하지 않았으며, 소스·설치본을 백업으로 복원했다. §1-B의 AMCL Axis B 5건과 다른 실패다.
- 누락된 YAML을 포함해 다시 백업·복사·빌드했다. Pi 빌드 통과와 같은 테스트 6건 통과(282건 deselected), 실제 import되는 설치 모듈·Pi 소스·PC 소스의 SHA-256 일치를 확인했다. 최종 SHA-256: `c0dce4f26e189a63b461f1448439f2fd0ee8e55b98e28acc56705d6da98265d0`.
- 반영 파일은 Pi의 `$HOME/jdamr_ws/src/jdamr_cube_ros/` 기준으로 `jdamr_cube_navigation/jdamr_cube_navigation/restaurant_service.py`, `jdamr_cube_navigation/test/test_restaurant_service.py`, `jdamr_cube_navigation/test/fixtures/restaurant_service_contract_gated.yaml` 세 개다. 이전 Pi 수정과 무관한 `main` 전체를 덮어쓰지 않았다.
- 실행기를 기존 기동 함수로 띄워 PID 2479·`active/running`을 확인하고 종료했다. 요청 파일 0개·이동 명령 0개이며 마지막 상태는 inactive다. 베이스·Nav2·카메라·표시 중계는 띄우지 않았다. 실제 `go`에서의 재요청·사이클 완료는 미검증이다.
- Pi 최종 배포 전 백업: `$HOME/jdamr_data/deploy_backup_20261012_125249_ace59b8_retry/`. 첫 시도 백업도 `$HOME/jdamr_data/deploy_backup_20261012_125108_ace59b8/`에 보존했다.
- PC 원본 로그: `$HOME/jdamr_data/analysis_20261012_reassembly_124830/`의 `deploy_build_verify.log`·`deploy_rollback.log`(첫 시도), `deploy_retry_build_verify.log`·`deployment_retry.json`(최종), `executor_smoke.json`·`executor_smoke_journal.log`(기동·종료). 전체 테스트 기준선과 AMCL 결정 대기 항목은 변경하지 않았다.

### 2-15. LiDAR USB 연결 복구·뎁스 수신 재확인 (2026-10-12)

- 13:24 재부팅 뒤에도 CP2102가 USB에 없었고 라이다 드라이버는 `/dev/ydlidar_g4` 열기 실패로 종료됐다. 당시 배터리 읽기는 12.544–12.552 V였다. 9/3의 비슷한 증상은 라이다 전원이 남아 CP2102가 걸린 사례였지만, 이번 원인으로 확정하지 않았다.
- 운영자는 이후 "USB 연결 문제였다, 라이다가 돈다"고 보고했고 뎁스 연결도 재확인을 요청했다. 13:40 조회에서 CP2102 `10c4:ea60`·Astra S `2bc5:0402`, `/dev/ydlidar_g4 → ttyUSB0`, G4 드라이버 기동을 확인했다. 이번 사건은 운영자가 확인한 USB 연결 문제로 기록한다. 배터리 부족이나 과거 CP2102 전원 유지 문제로 단정하지 않는다.
- 뎁스 서비스는 초기 inactive였다. 다른 카메라 서비스·실행기·내비게이션이 inactive이고 대기 중인 사이클 요청도 없음을 확인한 뒤 기존 `jdamr-box-rgbd.service`를 점검용으로 켰다. 파라미터·TF·장착 보정 파일은 변경하지 않았다.
- 13:43 읽기 전용 관측 약 5 s: 스캔 26개·odom 141개·깊이 영상 48개·CameraInfo 48개·배터리 2개를 받았다. 수신을 시작한 뒤 메시지 timestamp 간격으로 구한 속도는 스캔 9.6267 Hz·깊이 29.5298 Hz다. 짧은 단일 관측 구간이며, 관측 5 s 전체에 걸쳐 같은 수신 속도를 유지했다는 뜻은 아니다.
- 깊이는 320×240 `16UC1`, 153600 bytes, 프레임 `camera_color_optical_frame`이었다. 첫 영상은 CameraInfo가 아직 없어 검증에서 제외했고, 이후 47개는 기존 `decode_depth_observation`의 CameraInfo 값·프레임·버퍼·신선도 검증을 통과했다. 마지막 영상 나이는 약 0.0022 s, 스캔 나이는 약 0.1069 s였고, 0·65535를 제외한 깊이 픽셀 비율은 72.81–72.88 %였다. 이는 연결·영상 수신 확인이며 장착 높이·피치·차체 기준 외부 보정·주차 정확도 검증은 아니다.
- 이동 명령은 0개다. 점검 뒤 카메라를 stop하여 inactive를 확인했다. 운영자가 기동해 둔 베이스는 active로 유지했으며 박스 관측기·내비게이션·실행기는 기동하지 않았다. 모터 회전 방향 시험과 `go`·`init`도 하지 않았다.
- PC 근거: `$HOME/jdamr_data/sensor_recheck_20261012_134306/`의 `probe_stdout.json`, `probe_stderr.log`, `verification.json`, `sensor_readonly_probe.py`, `camera_start.log`, `camera_journal.log`. probe JSON의 `started_at`은 수집 종료 뒤 생성된 출력 시각이다(필드 이름 오류). 원본은 보존하고 `probe_timestamp_note.json`에 관측 창 해석을 따로 기록했다. 메시지 timestamp 기반 수신 속도와 개수는 이 필드에 의존하지 않는다.

### 2-16. 새 시작점·하향 카메라·출발 대기 (2026-10-12)

이 절은 14:42–14:58의 준비 시점 기록이다. HOME·서비스·PID의 현재값은 §2-17이 대체한다.

- 사용자 요청 범위는 출발 직전까지다. 이동 목표·비영 속도·팔 구동 명령은 보내지 않았다.
- 실제 위치는 새 라이다 40개로 매칭했다. 초기화에 사용한 inlier는 약 0.39, 전역 탐색도 동의했다. 기존 문서의 `--local-only --min-inlier 0.30`를 사용하되 이번 실행에서는 전역 동의도 확인했다. AMCL과 라이다 위치 차이는 약 0.006 m·0.2°였다.
- 기존 `teach-home --parking-direction forward --replace`로 현재 정지 위치를 저장했다. 새 HOME은 (0.175466, -0.099308) m·yaw 0.138536 rad이며 모든 운영 경로의 HOME을 갱신했다. 이전 레지스트리·상태·경로 백업은 아래 근거 폴더 `home_change_backup/`, Pi 레지스트리 옆 `.before_start_home_20261012`다.
- 카메라 x=-0.275 m는 실제 하단 뒤끝(0.065-0.340)에 상·하판 외곽 정렬과 렌즈 뒤끝 중앙 설치 가정을 결합한 추론이다. 현재 실차 URDF에는 상판 링크가 없으며, CAD 계획 모델의 뒤끝 -0.170 m를 실제 차축 좌표로 혼용하지 않았다. y=0·yaw=0은 설치 가정이다.
- 사용자 높이 약 1 m를 후보 선택의 선행 추정치로 사용했다. 정지 깊이 20프레임 중 첫·중간·끝 3프레임에서 넓은 바닥 후보를 맞춰 z=0.907054 m, roll=0.0119341 rad, pitch=0.466815 rad를 산출했다. 인라이어율 82.23–82.58%, 평면 RMS 최대 7.38 mm. 관측 평면의 반복성은 1 cm 주차의 물리 정확도를 증명하지 않는다.
- 기하 계산에 광학 수직 위치와 3D 회전을 반영하고, 수직 위치·평면 법선의 관측 안정성을 검사한다. 박스 정면 판정도 카메라 roll/pitch를 반영하며 기존 법선 임계값 0.90을 유지한다. 설정은 `status: derived`로 출처와 가정을 보존한다. 코드 리뷰 승인 및 PC 관련 테스트 62건 통과.
- Pi 빌드와 설치본 SHA-256 대조 완료. Pi 관련 테스트는 처음 61통과·1실패였으며, 실패는 별도 VSLAM 소스의 오래된 nominal 설정을 읽는 입력 차이였다. 해당 운영 설정은 바꾸지 않고 격리한 테스트 트리에 PC와 같은 입력을 넣어 설치된 코드의 62건 통과를 확인했다. 백업·빌드·두 테스트 로그: Pi `$HOME/jdamr_data/deploy_backup_camera_20261012_1455/`.
- 14:58 최종 관측에서 깊이 180개·CameraInfo 182개·박스 관측 32개를 수신했다. 관측기의 실제 roll/pitch 파라미터가 새 계산값과 같고, 10개 lifecycle 노드가 모두 active였다. 비상정지는 해제 상태, 이동 목표·속도 명령은 발행하지 않았다. 현재 장면은 front 검출 후 안정화 수집 중이며, 목표 박스의 주차 성공을 뜻하지 않는다.
- RViz 표시 중계와 HOME 마커를 갱신했고 map→base_link 및 base_link→camera_link 실제 TF를 확인했다. 대시보드 `$HOME/.local/share/robot-dashboard/current`의 기존 서비스를 재사용해 `http://127.0.0.1:8090/`에서 라이다·깊이·위치를 제공한다. 브라우저 자동 열기는 도구의 request-header policy 로딩 오류로 실패했으나 HTTP와 실제 데이터는 확인했다.
- 사용자의 즉시 출발 대기·화면 표시 요청 때문에 다음 서비스를 유지한다. Pi: `jdamr-restaurant-navigation`, `jdamr-box-executor`(PID 8133), `jdamr-box-rgbd`(5426), `jdamr-box-observer`(7979), `jdamr-camera-mount`(8259), `jdamr-ammr-dashboard`(5220). 원래 켜져 있던 `jdamr-base`(1177)도 유지한다. PC: `jdamr-p2-{mapserver,relay,markers,rviz,click,record}`, `robot-dashboard`, `robot-dashboard-link`. 관측기·RGB-D·대시보드와 중계는 준비용으로 켜져 있으며 상주 운용을 기본값으로 바꾼 것은 아니다.
- 대기 종료 시 PC에서 `python3 "$HOME/jdamr_data/map_20260930_manual/tools/jdamr_depart.py" session-stop`, 같은 도구의 `display-stop`, `systemctl --user stop robot-dashboard robot-dashboard-link`를 실행한다. Pi에는 `ssh lim@192.168.0.159 'sudo -n systemctl stop jdamr-box-observer jdamr-box-rgbd jdamr-camera-mount jdamr-ammr-dashboard'`를 실행한다. PC 도구에서 user bus가 없으면 `XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus`를 붙인다. 베이스는 이번 준비가 새로 켠 서비스가 아니므로 이 종료 목록에 넣지 않는다.
- 촉각센서는 5V·GND 두 선만 연결됐다. 신호 입력과 펌웨어 경로가 없어 눌림 검증·접촉 정지는 미완료다. 임의 GPIO를 사용하거나 제어보드 펌웨어를 변경하지 않았다.
- 근거 폴더: PC `$HOME/jdamr_data/predeparture_20261012_141234/`. 주요 파일: `start_home_after.json`, `camera_floor_samples.npz`, `floor_plane_fit.json`, `floor_candidate.png`, `camera_mount_derived_20261012.yaml`. 초기화 기록: `$HOME/jdamr_data/map_20260930_manual/aligned/runs/init_20261012_144049/`.

### 2-17. 실차 중단 원인·새 HOME·종료 후 수정 (2026-10-12 15:17 이후)

#### HOME과 실제 완료 범위

- 운영자는 기존 실물 도크 안으로 진입할 수 없어 시작점을 복귀점으로 사용하도록 지시했다. 17:24 `teach-home --parking-direction forward --replace`로 등록한 HOME은 (0.12685095998538692, 0.057200166122187035) m, yaw 0.11115982800318104 rad다. 레지스트리·상태·운영 경로를 갱신했으며 14:42의 값은 폐기한 이전 등록값이다. 원본은 `$HOME/jdamr_data/map_20260930_manual/aligned/service_destinations.yaml`, 이전 값 백업은 `runs/dock_reteach_20261012_171717/previous_home/`이다.
- 17:21 초기화에서 스캔과 AMCL 차이는 약 0.003 m·0.1°였다. 카메라 재연결 뒤 17:56 새 초기화는 전역·지역 탐색이 동의했고 inlier 약 0.34, AMCL 차이 약 0.005 m·0.5°였다. 기존 `init --local-only --min-inlier 0.30`를 사용했고 AMCL 5 cm·2° 검사는 유지했다. 이 초기화는 HOME 재등록이 아니며 등록값은 바꾸지 않았다.
- `table_01_20261012_173445`의 복귀 실행은 HOME 대기점 도착(status 4) 뒤 최종 HOME 목표를 수락했으나 사용자 정지 요청으로 끝났다. HOME 도착 성공으로 기록하지 않는다. 이후 마지막 `table_01_20261012_175643`은 물 받는 곳 관측 지점에 도착했고 카메라 준비도 통과했지만 탐색 Spin 후 검증에서 실패했다. 박스 주차·대기·이탈·HOME 복귀 완료는 모두 미확인이다.
- 사용자 최종 요청은 정지, 퇴근 후 개선, 도크에 놓겠다는 내용이다. 17:59 이동 중지와 18:00 세션 종료를 마쳤다. 실제 도크 재배치 완료는 새 관측으로 확인하지 않았다. `state.json.localized=false`이며 다음 새 세션은 등록 HOME에서 `init`을 새로 수행한다. 주행 중 재개에 HOME pose를 덮어쓰지 않는다.

#### 차체 기둥 자기 반사

- `table_01_20261012_151708`은 약 0.44 m 이동 뒤 자기 반사로 충돌 방지 정지가 걸렸다. 사용자는 주변에 장애물이 없고 높인 네 기둥이 LiDAR 평면에 걸린다고 설명했다. 이동 전후의 차체 기준 좌표가 유지된 후방 두 군집을 기존 `scan_self_filter`에 반영했다. 앞 기둥 위치에는 유효 반사점이 없어 필터 박스를 추가하지 않았다.
- 영구 저장소 설정: `jdamr_cube_bringup/config/self_filter_columns_20261012.yaml`, `status: derived_from_lidar_returns`, margin 0.005 m. 왼쪽 중심 (-0.251, 0.230) m·범위 (0.018, 0.026) m, 오른쪽 중심 (-0.283, -0.200) m·범위 (0.024, 0.024) m다. 실제 기둥 치수가 아니라 반사점 외접 범위이며 각도 전체를 차단하지 않는다.
- 원본 스캔 40+40개 재생에서 자기 반사 779점 제거, 0.4 m 안 유효 반사 잔여 0, 0.45 m보다 먼 반사 변경 0을 확인했다. 적용 당시 raw/filtered 동시 48개 비교에서도 근거리 461점 → 0, 먼 반사 변경 0이었다. 기둥 자체 가림은 남으며 사각은 왼쪽 132.5–140.3°·오른쪽 -147.7–-139.7°다. 기록 목록에 `/scan_raw`가 없어 주행 전체의 연속 원본은 없고 준비 시점의 별도 원본 스캔을 보존했다.
- Pi 설정 `$HOME/jdamr_deploy/self_filter_columns_20261012.yaml`과 `/etc/systemd/system/jdamr-base.service.d/95-column-self-filter.conf`로 베이스 기동에 통합했다. 처음 적용할 때는 베이스 PID를 유지하고 임시 raw/filter 유닛을 사용했지만 17:01 재부팅 뒤에는 베이스 안에서 통합 기동한다. 임시 유닛을 다시 중복 기동하지 않는다.
- 근거: `runs/table_01_20261012_151708/filter_application.json`, `column_points_current.json`, 준비 폴더의 `scan.json`·`scan_current.json`; 재생 재검증은 `$HOME/jdamr_data/predeparture_improvement_20261012/self_filter_replay.json`이다.

#### 사이트 끊김과 수동 명령

- 사이트와 LiDAR가 함께 꺼진 시점에는 Pi가 17:01에 재부팅한 사실을 확인했다. 배터리·전원 원인은 미확정이며 `vcgencmd get_throttled`는 0이었다. 웹 대시보드가 LiDAR를 중지시킨 근거는 없다. 정지된 서비스 조회의 `systemctl is-active` exit 3, 존재하지 않는 임시 유닛의 stop exit 1은 명령 상태 코드이며 그 자체로 하드웨어 고장을 뜻하지 않는다.
- 수동 페이지는 100 ms마다 새 명령을 보내면서 진행 중 HTTP 요청을 취소해 느린 응답을 계속 버리는 문제가 있었다. `/cmd` 한 요청만 진행시키고 대기 중에는 최신 명령 하나만 보존하도록 수정했다. 키 변경·해제·STOP·창 포커스 이탈은 즉시 새 명령을 보낸다. 300 ms 응답 시간 상한과 `/state` 단일 요청·800 ms 상한을 추가했고 서버 350 ms deadman과 펌웨어 400 ms watchdog은 유지했다.
- 200 ms 응답 지연, 키 해제·STOP·포커스 이탈, 무응답 복구, 상태 요청 중첩, deadman 회귀를 확인했다. 이는 발견한 클라이언트 결함 수정이며 사용자가 본 모든 가다 서다 현상의 유일 원인으로 단정하지 않는다. `web_teleop.py`를 Pi에 반영·빌드하고 기본 정지 시간 0.35 s와 설치본 해시를 확인했다. 실제 수동 구동 재검증은 정지 요청 뒤 수행하지 않았다.
- 수동 운용은 Pi `jdamr-webteleop`와 `jdamr-manual-motion`, PC `jdamr-manual-control-link`가 한 구성이다. Pi 출력은 `cmd_vel_nav`를 거쳐 기존 평활기·충돌 방지를 통과한다. 현재 모두 종료했다. 다음 수동 요청 때 주소는 PC `http://127.0.0.1:8080/`, 같은 Wi-Fi 기기는 `http://192.168.0.159:8080/`다. 자율 실행과 동시에 켜지 않는다. 수동 구성 기록: `runs/manual_control_20261012/manual_control_state.json`.

#### Astra USB와 실제 영상

- `table_01_20261012_172507`은 관측 지점에 도착(status 4, Nav2 error 0)했지만 30 s 카메라 준비가 끝나 실패했다. USB VID/PID는 보였으나 `bNumInterfaces=0`이었고 드라이버는 `Entity not found`를 보고했다. 기존 복구 헬퍼가 `lsusb` 존재만으로 성공 처리한 것은 불충분했다.
- 기존 Astra 전용 포트 reset으로 인터페이스는 3개로 복구됐지만 실제 영상은 0개였다. 이후 SDK segfault(-11)와 장치 busy 오류도 관찰했다. 사용자의 물리 USB 재연결 뒤 17:55 깊이 281개·박스 관측 39개, 마지막 깊이 나이 약 0.0025 s, camera-ready true를 확인했다. 소프트웨어 포트 reset만으로 영상이 복구됐다고 기록하지 않는다.
- 헬퍼는 VID/PID 외에 authorized·configuration·interface 수, descriptor, vendor class ff의 Bulk IN endpoint 2개 이상을 검사하도록 수정했다. 출력도 USB 열거 성공이라고만 표현하며 영상 수신 성공과 구분한다. 기존 포트 원상 복구 trap과 재시도 상한은 유지한다. Pi `/usr/local/sbin/jdamr-astra-usb-recover`에 반영하고 정상 열거 확인을 통과했다.
- 근거: `runs/table_01_20261012_172507/camera_usb_recovery.json`, `$HOME/jdamr_data/predeparture_improvement_20261012/camera_usb_and_crash.txt`. 다음 기동에서는 이름·USB 존재만 확인하지 말고 기존 실행기의 실제 영상 신선도 검사를 사용한다. 현재 카메라는 정지 요청에 따라 꺼져 있다.

#### 성공한 Spin을 실패로 기록한 원인과 수정

- 마지막 run `table_01_20261012_175643`의 Spin 요청은 +0.455108 rad였다. Nav2 결과는 status 4·error 0으로 성공했지만 map 기준 관측 이동량 0.135171 m가 5 cm 검사를 넘어서 `nav2_spin_failed`로 잘못 분류됐다.
- 같은 MCAP 구간을 분리하면 odom 회전 0.516950 rad, odom 계산 병진 0.000401 m, IMU 회전 적분 절댓값 0.498068 rad다. map→odom 보정은 병진 0.096534 m·yaw -0.113655 rad 변했다. 따라서 map 위치 변화에는 위치 추정 보정이 섞여 있었다. 바퀴 odom과 IMU는 회전에 부합하지만 실물 병진은 외부 계측하지 않았다.
- 종료 후 일반 탐색 회전도 기존 staging 회전처럼 odom 기준으로 바꿨다. 최초 target·before, 종료 after, 입력 공백 복구의 남은 각도를 같은 프레임에서 계산하며 정지 확인은 유지한다. **5 cm 이동·회전 오차 15° 제한은 완화하지 않았다.** MCAP의 map 13.5 cm 변화/odom 0.4 mm 회귀는 통과하고 odom 5 cm 초과는 계속 거부된다.
- 실패 사유는 `nav2_spin_failed`, `nav2_spin_rejected`, `rotation_observation_not_confirmed`, `operator_stop` 등으로 구분한다. 중단 요청과 센서/AMCL 오류가 겹치면 `operator_stop`을 우선한다. 복구 뒤 남은 각도가 5° 이내라 추가 Spin 없이 성공할 때 이전 실패 사유를 지운다. 관측 이벤트에 `observation_frame`도 기록한다.
- 분석 스크립트·입력 해시·JSON: `$HOME/jdamr_data/predeparture_improvement_20261012/spin_frame_analysis/`. 수정본의 실물 Spin이나 다음 박스 주차는 정지 요청 뒤 수행하지 않았다.

#### 종료 후 검증·반영

- 작성과 독립 code-reviewer 검토를 분리했고 USB·웹·Spin·필터 설정 모두 승인됐다. 최종 관련 테스트는 **437 통과**(내비게이션 두 파일 397, 수동 웹 17, USB 11, 기존 자기 반사 필터 12), 문법 검사·`bash -n`·diff 검사 통과다. 전체 내비게이션 기준선 5개 실패(§1-B)를 해결했다고 주장하지 않는다.
- 처음 묶음 검사에서 PC 작업공간 overlay를 source하지 않아 package-not-found 19건이 발생했다. 올바른 `/opt/ros/jazzy/setup.bash`와 `$HOME/jdamr_rgbd_ws/install/setup.bash`를 읽고 재실행한 결과가 위 437 통과다. 실패 출력도 `targeted_tests_without_overlay.txt`로 보존했다. 최종 로그는 `final_targeted_tests.txt`·`final_targeted_tests.xml`이다.
- Pi에서 `jdamr_cube_node` 빌드와 이후 `jdamr_cube_navigation`·`jdamr_cube_bringup` 2패키지 빌드를 완료했다. 카메라·이동 실행기·Nav2가 꺼진 상태에서 파일 반영과 import만 확인했으며 이동 명령은 보내지 않았다. 최종 빌드 기록: Pi `$HOME/jdamr_data/predeparture_improvement_20261012/colcon_log_final/`.
- PC 파일, Pi 소스와 설치본 해시를 대조했다. `restaurant_service.py`는 `64cf86e512df0c2bc86495760f94198ea12d89b9b49a3a35ca6508760b477cef`, `box_service.py`는 `25648612336a835703da2601c793d19b2259fbb2c392a3580d114ca141203dc5`, `web_teleop.py`는 `5e9eae3ec6b4560607fc206281de40032953dc74e5c8ff65c810077ae1442d61`, USB 헬퍼는 `d6ff74990b611e2ca7af5094fad1e7a268929951d1f91d303581265d08f5694e`, 필터 YAML은 `1ec5ffc28f3cf1189594445fe88fbb5225f5ac3758b0dbee88a6106a53b7e46f`다. 필터 YAML은 Pi deploy·설치 share와도 같다.
- 해시·서비스 상태·import 근거: PC `$HOME/jdamr_data/predeparture_improvement_20261012/deployment_final.json`, PC 표시·터널 종료 상태는 `cleanup_final.json`. Pi 백업: 같은 이름의 폴더 `backup/`에 USB·웹 이전 파일, `backup/spin_odom/`에 네비게이션 이전 파일과 이전 필터 YAML을 보존했다. Python의 구문·잘못된 식·미정의 이름 범위를 검사한 flake8(`E9,F63,F7,F82`)도 통과했다. PC 수정은 로컬 커밋으로만 보존하며 이번 작업의 원격 push는 수행하지 않는다.

#### 유지 서비스와 다음 시작

- 종료 확인: Pi Nav2·상주 실행기·RGB-D·박스 관측·카메라 mount·대시보드·웹 수동·수동 평활기/충돌 방지 inactive. PC RViz·mapserver·relay·markers·click·record·대시보드/짝 터널·수동 터널 inactive. 이미 제거된 transient unit은 `LoadState=not-found`이며 새 고장이 아니다.
- 사용자 기존 `jdamr-base.service`만 active, MainPID 1284로 유지한다. 이동 실행기가 아니라 기존 베이스 센서·UART 상태 공급을 유지하는 서비스이며 이번 종료 후 새 속도 명령은 보내지 않았다. 사용자가 전원을 끄거나 베이스 종료를 요청하면 `ssh lim@192.168.0.159 'sudo -n systemctl stop jdamr-base'`로 끈다. 이전 준비 절의 PID·유지 목록으로 서비스를 다시 켜지 않는다.
- 다음 출발 요청에는 기존 `$HOME/jdamr_data/map_20260930_manual/tools/jdamr_depart.py`의 `session-start` → 등록 HOME의 `init` → 요청한 `go` 순서를 쓴다. 일치율이 낮을 때는 지역·전역 위치 동의 근거가 있는 경우에만 기존 `--local-only --min-inlier 0.30`를 사용하며 AMCL 일치 기준은 유지한다. 센서 입력은 기존 실행기의 신선도 확인을 사용하고 준비된 코드 검사·문서·리뷰를 출발 때 반복하지 않는다.
- 다음 실차 확인은 수정본의 탐색 Spin 이후 박스 면 정렬·목표 간격 접근·정지·복귀 완료다. 1 cm 외부 정확도나 접촉센서 연동 완료는 주장하지 않는다. 촉각센서는 여전히 5V·GND뿐이며 신호선 없이는 눌림 입력을 읽을 수 없다. §1-B의 결정 대기와 §1-C의 미지시 고도화는 그대로 둔다.

## 3. 최초 인계의 커밋 (10-06 15:00 이후, 아래 표는 `origin/main`)

| 커밋 | 내용 |
|---|---|
| `c99ecb6` | 이동·대기점 기본 구성을 NavFn + RPP로 고정 |
| `d6ba9b5` | 박스 목표 간격 1.5 cm와 시험용 `--final-gap-m`·`--dwell-s` |
| `3603ea1` | 경로 노드 EventsExecutor(CPU) |
| `95b820f`·`db20d5f`·`0ceeab8`·`ec1b236` | 박스 직진 전 면 법선 회전, 깊이 법선, 가장 앞선 모서리 판정, 대기 odom 감시 |
| `2c596fe` | 조립 뒤 테이블 상판 모서리 정밀 주차 설계 메모(`20261006_TABLE_EDGE_DOCKING_PLAN.md`) |
| `add61c4`·`ea3fbf0` | 도크 대기점 NavFn만, 박스 목표 간격 1 cm |
| `aac1828` | 이동 트리 지난 경유점 매 틱 제거 |
| `e1ae88a` | 막힌 구간 재시도의 Lattice + MPPI 전환 제거 |
| `37ec84c` | MPPI는 사전 지점까지·전진만·목표 방향 비용 마지막 0.1 m |
| `b1deb7a` | `init` 활성화 대기에서 노드 전용 실행기 유지 |
| `ace59b8` | 실행기 첫 파라미터 읽기 재요청 (10-12 Pi 배포·관련 테스트·비구동 기동 확인 완료, §2-14) |
| `4518da3` | README·인계 문서 정정 |
| `0c79182` | 봉인 평가 입력 스냅샷·린트 기준선 |

10-12 수정은 로컬 `main`에만 있으며 원격 기준은 `b1daa6a`다. 아래 로컬 커밋과 이 인계 갱신을 원격 배포 완료로 취급하지 않는다.

| 로컬 커밋 | 내용 |
|---|---|
| `95fff23` | 차체 결합 상태와 Pi 반영 검증 기록 |
| `df70044` | USB 복구 뒤 LiDAR·뎁스 수신 기록 |
| `58763db` | 후방 하향 카메라 보정과 출발 대기 설정 |
| `187d03d` | Astra USB 열거 검사와 기둥 반사 설정 보존 |
| `1301d9e` | 느린 HTTP 응답에서 수동 명령 순서 처리 |
| `f4313a1` | 탐색 Spin의 odom 검증과 중단 사유 구분 |

## 4. 원본 경로

| 내용 | 경로 |
|---|---|
| 주행 run | `$HOME/jdamr_data/map_20260930_manual/aligned/runs/` (`cycle_events.jsonl`, `onboard_bag`, `load_profile.jsonl`, `udp_drops.jsonl`) |
| 반복 비교 | `$HOME/jdamr_data/analysis_20261006/ab_series_1710/` (`ab_series.sh`, `series.log`, `series_metrics.py`) |
| 폐기한 시리즈와 AMCL 관문 | `$HOME/jdamr_data/analysis_20261006/ab_series_wheelfix/` (`ab_series.sh`, `amcl_gate.py`, `stop_after_5.sh`) |
| 분석 스크립트 | `$HOME/jdamr_data/analysis_20261006/claude_scripts/` — `speed_compare.py`(구간별 속도), `jitter.py`(누적 회전·부호 반전), `ripple.py`(회전 속도 변동), `homeleg.py`·`clearance.py`(복귀 구간 궤적·외곽 간격), `costcheck*.py`·`scan*.py`·`amclnow.py`(Pi에서 costmap·스캔·AMCL 읽기), `svclat.py`·`mapsvc.py`·`reqact.py`·`actdiag.py`·`evexec.py`(서비스 응답·활성화 진단). run 폴더 이름을 인자로 받는다(`runs/`에서 실행) |
| 고리 경로 재현 | `$HOME/jdamr_data/analysis_20261006/loop_replay/results.json` |
| 실패·수정 원 기록 | `jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md` |
| 9/29 응답 유실 근거 | `jdamr_cube_navigation/evaluation/20260929_RUNTIME_ROOT_CAUSE.md` |
| 10-12 결합 뒤 읽기·배포 근거 | PC `$HOME/jdamr_data/analysis_20261012_reassembly_124830/` (UART 전 샘플, 배포·복원·재검증·실행기 종료 로그, §2-13–14) |
| 10-12 USB 복구 뒤 센서 재확인 | PC `$HOME/jdamr_data/sensor_recheck_20261012_134306/` (센서 수신·깊이 디코드·카메라 종료 근거, §2-15) |
| 10-12 실차 중단 뒤 수정·검증 | PC `$HOME/jdamr_data/predeparture_improvement_20261012/` (Spin 프레임 분리, 필터 재생, 최종 테스트·설치본 해시·서비스 상태, §2-17) |

## 5. 운영 규칙 (이 작업에 걸리는 것)

- 로봇 이동은 사용자의 실행 요청 범위에서만 한다. 시험 중 실패는 스스로 진단·복구하되, 복구 주행 전 AMCL 공분산을 확인한다.
- Pi는 ROS 도메인 12, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. 비대화형 ssh 명령은 `jdamr_depart.PI_SOURCE`를 앞에 붙인다.
- 프로세스는 PID·유닛 이름으로 끈다. `pkill -f`를 쓰지 않는다.
- 충전기는 운영자가 연결한다. 전압으로 도킹을 판정하지 않는다.
- 수정 뒤 테스트는 관련 테스트 파일만 돌린다. 전체 실행은 `source /opt/ros/jazzy/setup.bash && source ~/jdamr_rgbd_ws/install/setup.bash` 뒤 `cd jdamr_cube_navigation && python3 -m pytest -q test`(`test/conftest.py`가 evaluation 경로를 넣는다).
- 사용자 변경인 `jdamr_cube_cartographer/rviz/jdamr_cube_cartographer.rviz`는 스테이징하지 않는다. 스테이징은 경로별로 하고 커밋 전 `git diff --cached --stat`.
- 커밋 메시지는 한글 Conventional Commits.
