# JD-AMR Cube — 실기 브링업과 SLAM

라즈베리파이 4 + Waveshare ESP32 제어보드 + LD14P 라이다로 조립한 차동구동 AMR을
ROS 2 Jazzy 자율주행 스택에 올린 기록. 강사 제공 코드에 결함이 많아 펌웨어부터
다시 만들었고, 그 과정과 근거를 전부 남겼다.

## 무엇이 어디에 있나

| 경로 | 내용 |
|---|---|
| `reference/JDAMR_Cube_조사기록.md` | **강사원본 결함 43건 감사** (A~G 분류, 메커니즘 수준) |
| `reference/instructor_original/` | 강사 제공 원본 코드 — 대조군 |
| `firmware/jdamr_cube_fw/` | **ESP32 펌웨어 v2** (원본 대체). 라이브러리 동봉 |
| `jdamr_base_driver/` | **C++ 베이스 드라이버** — cmd_vel↔펌웨어, 50Hz 오도메트리 |
| `jdamr_cube_node/` | Python 도구 — 캘리브레이션·폐루프 평가·자동 프로브·웹 조종기 |
| `jdamr_cube_bringup/launch/real_bringup.launch.py` | 실기 브링업 (드라이버+라이다+TF) |
| `jdamr_cube_cartographer/config/` | SLAM 프로파일 2종 (강의실 `_real` / 복도 `_corridor`) |
| `jdamr_cube_description/urdf/` | URDF — 제원은 전부 실측값 |

학습 노트·진행 로그·이론 정리는 별도 리포에 있다:
`physical-ai-lab/learning/M02_linux_ros2/slam_nav2/JDAMR_NEXT_SESSION.md`

논문 근거의 SLAM·localization·frontier·Sim-to-Real 포트폴리오 작업은 [SLAM 포트폴리오 재개 가이드](README_SLAM_PORTFOLIO.md)에서 이어간다. 현재는 계획 수립 단계까지 완료됐으며 다음 작업은 실차 이동이 없는 Phase 0 평가 기준선 구축이다.

## 실측으로 확정한 제원 (추측 금지 — 전부 한 번씩 틀렸던 값들)

| 항목 | 값 | 확정 방법 |
|---|---|---|
| 파이↔ESP32 통신 | **40핀 헤더 UART** `/dev/ttyS0` | USB 아님. 프레임 수신 실측 |
| 서보 ID | **ID1 = 오른쪽**(+가 전진), ID2 = 왼쪽(−가 전진) | 단일 바퀴 구동 시험 |
| 바퀴 반지름 | **32.9 mm** | 주행 캘리브레이션(검산 0.986), 자 실측 65mm 지름과 일치 |
| 유효 트레드 | **183.6 mm** | 주행 캘리브레이션(검산 0.999). 기하 200mm 와 다름 — 접지폭 효과 |
| 라이다 위치 | **전방 40 mm · 좌 55 mm** | 줄자. 사진 판독값(후방 75mm)은 오류였다 |
| 라이다 회전방향 | `laser_scan_dir: True` | 실물 코너 대조 |

## 펌웨어 v2 가 고친 것 (감사 A1~A11)

워치독 400ms · 부호 있는 바퀴별 연속 속도(원본은 5방향 이산이라 자율주행 불가) ·
12비트 랩어라운드 언랩 · CRC-8 실계산과 바이트 단위 재동기 · 9축 IMU 전량 전송 ·
OLED 블로킹 제거 · INA219 주소 스캔 · 서보 생존감시와 모드 자동복구.

## 검증 도구

```bash
ros2 run jdamr_cube_node wheel_calibration   # 바퀴 제원 (UMBmark 축소판)
ros2 run jdamr_cube_node motion_probe spin   # 자동 제자리회전 (폰 STOP 으로 중단)
ros2 run jdamr_cube_node loop_eval           # 폐루프 오차 4분면 판정
ros2 run jdamr_cube_node web_teleop          # 브라우저/폰 조종기 :8080
```

## 실패에서 배운 것

지도가 나흘간 무너진 원인은 환경도 튜닝도 아니고 **정상 스캔에 거울을 씌운 것**이었다.
자작 판정 프로브가 "로봇이 CCW 로 +θ 돌면 로봇 좌표계에서 세상은 −θ 돈다"는 부호를
반대로 읽고 `laser_scan_dir` 을 뒤집었고, 그 위에서 max_range·상관매칭·가중치를
아무리 바꿔도 안 고쳐졌다. 실물 코너 하나로 30초 만에 잡혔다.

판정 도구 넷이 같은 방식으로 틀렸다 — **두 계층의 일치를 품질로 오인**했다. 정합의
심판은 지도이지 센서 간 합의가 아니고, 기하의 심판은 줄자와 실물이다.
