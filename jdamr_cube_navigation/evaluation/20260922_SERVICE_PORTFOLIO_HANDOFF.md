# 식당 서빙 주행 — RViz·데이터 인계

## 이번 자료의 용도

저장 지도에서 충전소와 테이블 영역을 지정하고, Keepout을 적용하는 서빙 주행 시스템의 구성·실행 기록·장애 대응을 보여주는 자료다. 이번 오후 실행은 저전압으로 중단됐으며, 테이블 정밀 주차와 충전소 복귀 성공 영상으로 사용하지 않는다.

## 유지할 RViz 화면

설정 원본: `$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_navigation/rviz/restaurant_service.rviz`

| 표시 | 의미 | 혼동하면 안 되는 내용 |
|---|---|---|
| Saved Map | Cartographer 수동 주행으로 저장한 2D 점유 지도 | RGB-D 3D SLAM 결과가 아님 |
| Saved Keepout Reference / 붉은 영역 | 사용자가 표시한 진입 금지 영역 | 저장 마스크 표시만으로 Nav2 필터 수신을 증명하지 않음 |
| TABLE_1 / 파랑 | 1번 테이블 관측 영역, 선택 목적지 | 최종 주차 pose는 미등록. 박스 면 추정 결과와 구분 |
| TABLE_2 / 초록 | 2번 테이블 관측 영역 | 같은 기준 적용 |
| HOME_REVERSE / 주황 화살표 | 등록한 충전소 위치와 최종 차체 방향 | 화살표는 후진 속도 방향이 아님. 후면 주차 성공 증거도 아님 |
| Nav2 Planned Path / 보라 | 당시 planner가 발행한 계획 경로 | 실제 이동 궤적과 구분 |
| LiDAR / 빨강 | 해당 시각의 LaserScan 관측 | 박스·사람 자동 분류 결과가 아님 |
| Recorded odometry / 재생 전용 | bag의 odom 위치·방향 샘플 | 외부 ground truth 또는 주차 오차가 아님 |

흰 배경, map 좌표계, 전체 지도가 잘리지 않는 축척을 유지한다. 테이블 미교시 상태에는 `AREA_ONLY`를 표시하고, 등록된 pose가 있을 때만 방향 화살표를 추가한다. 저장 마스크·실시간 센서·목표·계획 경로를 같은 종류로 표시하지 않는다.

## 원본과 재생

실행 자료 루트: `$HOME/jdamr_data/service_run_20260922_922PvE/`

- `assets/`: 당시 지도 PGM/YAML/pbstream, 마스킹, 목적지 registry, Nav2 설정, RViz 설정 스냅샷. 실행 후의 설정으로 덮어쓰지 않는다.
- `bag/`: 정상 종료한 MCAP 원본. 1,291.4009초, 119,663개 메시지.
- `VERIFIED_METRICS.json`: 원본을 끝까지 순회한 토픽 수신량·계획 경로·odom 분석 결과.
- `SESSION.yaml`: 시행 식별, 상태, 입력 자산 해시.
- `initial_home_failure.jsonl`, `navigation_shutdown_excerpt.log`, `table01_low_battery.log`: 실패와 목표 취소 근거.
- `rviz_before_label_adjustment.png`: 라벨 수정 전 참고 화면. 최종 대표 이미지로 사용하지 않는다.
- `desktop_start.png`, `desktop_window_capture.png`: 터미널이 함께 찍힌 원본. 공개용 메인 미디어에서 제외한다.

실물과 분리한 재생 명령:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_rgbd_ws/install/setup.bash"
python3 "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros/jdamr_cube_navigation/evaluation/replay_service_evidence.py" \
  --bag "$HOME/jdamr_data/service_run_20260922_922PvE/bag"
```

재생 스크립트는 LOCALHOST·domain 78을 사용하고 표시용 토픽만 발행한다. 속도 명령, 목표 action, 서비스 요청은 재생하지 않는다. bag 중간부터 재생하면 최초 지도·static TF·마커가 누락될 수 있으므로 처음부터 재생한다. 원속도는 `--rate 1`; 빠르게 확인할 때만 배속을 명시한다. 이 화면은 기록 재생이며 실시간 로봇 상태가 아니다.

## 주요 수치와 주장 범위

| 항목 | 확인값 | 근거 |
|---|---|---|
| 지도 | 177×149 cells, 0.05m/cell | 저장 YAML/PGM |
| odom 수신 | 63,310개 | MCAP 원본 순회 |
| LiDAR 수신 | 11,568개 | MCAP 원본 순회 |
| 계획 경로 | 8회, 첫 계획 116 poses / 2.9182m | `/plan` 역직렬화 |
| 출발 후 변위 | odom 약 0.298m | 16:34:17~25 첫·마지막 odom 좌표 차이 |
| 중단 | 10.464V < 주행 하한 10.5V | 실행 로그와 goal 취소 확인 |
| 마지막 기록 속도 | 선속도 0m/s, 각속도 0rad/s | 16:36:17.616 odom |
| RGB·Depth 원본 | 이번 bag에 없음 | 토픽 목록 |

포트폴리오의 기술 설명은 `실차 지도 생성 → 지도 기준 목적지/Keepout 구성 → Nav2 계획·주차 인터페이스 → 실행 기록 분석 → 기동 의존성 수정` 순서로 구성할 수 있다. 계획 경로 길이를 실제 주행 거리로 쓰거나, odom 변위를 5cm 주차 정밀도로 쓰지 않는다.

## 다음 실차에서 확보할 대표 장면

1. 같은 화면에 지도·Keepout·충전소·선택 테이블과 현재 위치를 보여준다.
2. 계획 경로와 실제 이동을 함께 기록한다. 리셋·초기화 구간을 정상 주행 궤적에 합치지 않는다.
3. 박스 면 인식·목표 방향·잔여 거리·최종 간격은 RGB-D 관측과 별도 외부 촬영으로 남긴다.
4. 테이블 정지·5초 대기·충전소 복귀·최종 방향과 후면 간격을 확인한 뒤에만 왕복 성공으로 표시한다.

다음 시행은 새 run 폴더로 분리한다. 현재 실패 원본을 삭제하거나 성공 기록으로 바꾸지 않는다. 큰 bag·영상은 Git에 넣지 않고 경로·해시·요약·생성 명령만 저장소에 둔다.

## 수정 상태

재발 방지 코드와 오프라인 검증 근거는 [출발 실패 기록](20260922_SERVICE_DEPARTURE_FAILURES.md)에 연결한다. 실물 재검증 전에는 `구현·회귀 테스트 통과`와 `실차 확인 완료`를 구분한다. 포트폴리오 사이트 수정이나 배포는 이번 작업에 포함하지 않는다.
