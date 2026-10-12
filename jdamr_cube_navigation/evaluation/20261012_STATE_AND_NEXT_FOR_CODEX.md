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
| Pi 이번 반영 | `ace59b8`(실행기 첫 파라미터 읽기 재요청) 배포·빌드·관련 테스트 6건·소스/설치본 해시 대조 완료. 실행기 비구동 기동 확인 후 종료 | 10-12 확인(§2-14). 실제 `go` 사이클 확인은 남음 |
| Pi 이전 배포 기준 | `b1deb7a`까지(10-06 18:03 배포·빌드, `init` 성공 확인). 이번에는 실행기 수정과 관련 테스트 입력만 추가 반영 | §2-5·§2-14. `main` 전체 배포라고 주장하지 않음 |
| 로봇 위치 | 마지막 기록은 10-06 18:14 도크 앞 (0.81, 0.21)에서 정지. 그 뒤 위치는 기록 없음 | §2-9 |
| 차체 결합 | 10-12 결합 완료. 결합 뒤 돌출부·팔·적재물의 치수와 가동 상태는 미확인 | 10-12 운영자 보고. `new_base_geometry.yaml`의 주장 범위는 하단 프레임 |
| LiDAR | USB 연결 문제를 해결했다는 운영자 보고 뒤 CP2102·`/dev/ydlidar_g4` 인식과 스캔 약 9.63 Hz 수신 확인. 장착 위치와 기존 TF는 유지 | 10-12 13:40–13:43 재확인(§2-15). 결합 차체의 자기 반사는 실측 전 |
| 접촉센서 | 제어보드 5V·GND 연결. 센서 모델·신호선/핀·접촉 시 정지 연동 미확인 | 10-12 운영자 보고. 현재 펌웨어·베이스 드라이버에는 접촉센서 입력 경로 없음 |
| 뎁스 카메라 | USB·깊이 영상·CameraInfo 수신 확인. 영상은 320×240 `16UC1`·약 29.53 Hz. 미장착·약 100 cm 하향 설치 예정 보고는 유지, 피치·수평 위치 미확정 | 10-12 13:43 재확인(§2-15). 새 설치 TF·외부 보정 완료로 취급하지 않음 |
| 바퀴 모터선 | 분리 후 재연결(운영자 보고). UART 읽기 전용 확인에서 좌우 읽기 오류 0건·워치독 정지 유지. 회전 방향·구동 뒤 정지는 미검증 | 10-12 12:48 UART 5 s·250개 상태 패킷(§2-13) |
| Pi 접속·서비스 | Wi-Fi 변경 뒤 기존 주소 SSH 복구. 운영자 재시작 뒤 베이스 active, 내비게이션·실행기 inactive. 점검용으로 켠 뎁스 서비스는 종료 | 10-12 13:40–13:43 조회. 12:46의 모두 inactive 상태는 당시 기록 |

## 1. 할 일

### A0. 차체 결합 뒤 이번 요청 범위 (2026-10-12)

운영자는 다른 반영 가능 항목부터 진행하고, 재연결한 바퀴 모터선을 확인해 달라고 요청했다. §1-B의 AMCL 선택지나 §1-C의 미지시 고도화를 승인한 것으로 확대하지 않는다.

1. **Pi 접속과 모터 확인.** Wi-Fi 변경 뒤 기존 주소 접속이 복구됐다. 베이스 드라이버가 꺼진 상태에서 UART를 읽어 좌우 읽기 오류 0건·CRC 오류 0건·워치독 정지 유지를 확인했다(§2-13). 펌웨어의 기대값은 좌측 ID 2·우측 ID 1(`LEFT_ID`·`RIGHT_ID`)이다. 이는 버스 응답 확인이며 실제 좌우 배치·회전 방향의 검증은 아니다. 초기의 LiDAR USB 미인식은 운영자의 연결 수정 뒤 복구됐다(§2-15). 구동 시험은 아직 하지 않았다. 기존 실행 경로에서 실제 배선·회전 방향·최종 정지 확인이 남아 있으며, 충전 중에는 구동하지 않는다. 로봇의 실제 위치와 충전 케이블 연결 여부는 확인 전이다.
2. **차체·LiDAR 상태 반영.** 결합 완료와 LiDAR 위치 유지를 기록했다. 기존 하단 프레임 치수·라이다 TF는 유지한다. 결합 차체의 돌출부와 LiDAR 평면에 걸리는 기둥은 실측 후 필요한 제원과 자기 반사 필터에 반영한다. `$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_bringup/config/self_filter_hold_flow_model.yaml`은 `model_not_measured`이며 실측 설정으로 취급하지 않는다. 라이다 위치가 같아도 결합 뒤 반사·가림까지 같다고 가정하지 않는다.
3. **접촉센서 배선과 입력 확인.** 5V·GND 연결은 운영자 보고로 기록했다. 모델과 신호선/핀 정보를 요청한 상태다. 핀·출력 형식이 확인되기 전에는 GPIO 번호를 추측해 펌웨어에 넣지 않는다. 입력 전환을 읽고 기존 정지 경로와 연동하는 확인은 그 뒤에 수행한다. 현재 소프트웨어 래치 비상정지와 접촉센서 연동은 별개다.
4. **뎁스 설치 준비.** 약 1 m·하향 시야는 예정 배치이며, 미장착 보고는 유지한다. 13:43 재확인에서는 USB 인식에 더해 깊이 영상·CameraInfo와 기존 디코더 검증까지 확인했다(§2-15). 이는 새 마스트 설치의 위치·각도 보정 완료를 뜻하지 않는다. 기존 0.215 m·수평 배치의 관측/보정 기록을 새 설치의 실측으로 쓰지 않는다. 설치 뒤 높이·피치·수평 위치를 측정하고 카메라 TF·보정 및 테이블 관측을 확인한다. Pi의 `$HOME/jdamr_deploy/camera_mount_confirmed_20260917.yaml`은 기존 카메라 배치용이며, 새 마스트 설치에 그대로 적용하지 않는다(10-12 Pi에서 파일 존재·내용 확인). 상판 모서리 주차는 `20261006_TABLE_EDGE_DOCKING_PLAN.md`의 설계 후보이고 구현·실차 검증 완료 상태가 아니다.
5. **실행기 수정 반영.** §1-A1의 `ace59b8` 배포·Pi 검증과 실행기 비구동 기동 확인을 마쳤다(§2-14). 처음 꺼져 있던 실행기는 확인 뒤 다시 종료했으며 베이스·내비게이션도 기동하지 않았다. 다음 주행에서의 첫 파라미터 읽기 응답 확인은 남아 있다. 도크 배치가 확인되기 전에는 이전 위치 기록만으로 `init`을 수행하지 않는다.

### A. Pi 반영 상태와 다음 주행 준비 (사용자 요청 범위에서)

1. **`ace59b8` 반영 완료, 실제 사이클 확인 대기.** 상주 실행기(`jdamr-box-executor`)가 `restaurant_service.py`를 불러오므로 기동할 때 설치된 수정본을 읽는다.
   - 10-12 수행: Pi 백업 → 소스·관련 테스트·누락된 YAML 테스트 입력을 `rsync -cR`로 복사 → `colcon build --packages-select jdamr_cube_navigation`(복사 설치) → Pi 테스트 6건 통과 → 소스/설치본 SHA-256 대조 → 기존 `jdamr_depart.start_executor()`로 비구동 기동 확인 → 종료. 상세·백업·로그는 §2-14.
   - 실행기는 초기 inactive였으며 확인 때 PID 2479로 `active/running`에 도달했다. 요청 파일이나 이동 명령을 추가하지 않았고 확인 뒤 inactive로 복원했다. 다음 사용은 기존 `session-start` 경로로 기동한다. 이를 Nav2 활성화나 실제 사이클 성공으로 기록하지 않는다.
   - 남은 확인: 다음 `go`의 사이클이 이동 전 `request interrupted ...`로 끝나지 않는지. 재요청이 일어나면 실행기 저널에 `parameter read timeout: node=... attempt=1/2` 경고가 남는다(`journalctl -u jdamr-box-executor`).
2. **로봇 도크 배치와 `init`.** 운영자가 로봇을 도크에 놓은 뒤 `python3 jdamr_depart.py init`.
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

## 2. 10-06 오후–10-08 사실 기록

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

### 2-6. 실행기 이동 전 실패 (커밋 `ace59b8`, Pi 미배포)

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

## 3. 커밋 (10-06 15:00 이후, 모두 `origin/main`)

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

## 5. 운영 규칙 (이 작업에 걸리는 것)

- 로봇 이동은 사용자의 실행 요청 범위에서만 한다. 시험 중 실패는 스스로 진단·복구하되, 복구 주행 전 AMCL 공분산을 확인한다.
- Pi는 ROS 도메인 12, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. 비대화형 ssh 명령은 `jdamr_depart.PI_SOURCE`를 앞에 붙인다.
- 프로세스는 PID·유닛 이름으로 끈다. `pkill -f`를 쓰지 않는다.
- 충전기는 운영자가 연결한다. 전압으로 도킹을 판정하지 않는다.
- 수정 뒤 테스트는 관련 테스트 파일만 돌린다. 전체 실행은 `source /opt/ros/jazzy/setup.bash && source ~/jdamr_rgbd_ws/install/setup.bash` 뒤 `cd jdamr_cube_navigation && python3 -m pytest -q test`(`test/conftest.py`가 evaluation 경로를 넣는다).
- 사용자 변경인 `jdamr_cube_cartographer/rviz/jdamr_cube_cartographer.rviz`는 스테이징하지 않는다. 스테이징은 경로별로 하고 커밋 전 `git diff --cached --stat`.
- 커밋 메시지는 한글 Conventional Commits.
