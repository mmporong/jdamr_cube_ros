# 포트폴리오용 데이터 묶음

JD-AMR 복도 왕복 주행의 실측 산출물. 사이트 그림을 실물로 교체하고
면접 질문에 수치로 답하기 위한 원본이다.

## maps/ — 저장 지도와 금지구역 마스크

| 파일 | 내용 |
|---|---|
| `autonomous_20260826T161908.yaml` / `.pgm` | 저장 지도. 894x212 셀, 해상도 0.05 m, origin [-2.527, -8.975, 0] |
| `autonomous_20260826T161908_keepout_multi.yaml` / `.pgm` | 금지구역 마스크. 원본과 같은 격자 |
| `autonomous_20260826T161908_keepout_zones_multi.yaml` | 금지구역 2개의 다각형 꼭짓점 실좌표, safety_margin 0.55 m |
| `autonomous_20260826T161908_keepout_multi.json` | 마스크 생성 결과. keepout 9,406셀, 연결성 검사 통과(47,746/48,435셀), 원본·마스크 sha256 |

금지구역을 덮은 뒤에도 복도 좌우 끝(`[-0.002, 0.0]` -> `[37.498, -4.3]`)이
0.25 m 여유로 연결된다는 것이 `connectivity_checks`로 검증돼 있다.

## figures/ — 완성 그림

- `corridor_route_map.png` — 점유 격자 + 금지구역 다각형 + 계획 경로
- `corridor_route_with_drive.png` — 위에 AMCL 실주행 궤적을 얹은 것

경로 총연장은 **76.42 m**, waypoint 20개다. 사이트에 80 m로 적혀 있다면 고쳐야 한다.

## analysis/ — bag에서 뽑은 수치

원본 bag: `corridor_keepout_roundtrip_20260901T150446` (905.09초, 128,791 메시지)
추출 스크립트: `extract_amcl_tf.py`

### amcl_pose_covariance.csv (292행)

`/amcl_pose` 전량. x, y, yaw와 cov_xx / cov_yy / cov_yawyaw, 표준편차 환산까지.

**주의 — "공분산 임계 2.0"이라는 값은 이 저장소에 없다.**
실제 게이트는 `jdamr_cube_navigation/config/corridor_roundtrip.autonomous_20260826.yaml`의
`max_amcl_x_covariance: 50.0` / `max_amcl_y_covariance: 50.0`이고,
`corridor_route.py`의 코드 기본값이 0.5다.

실측값은 cov_xx 최대 0.731, cov_yy 최대 0.164다. 292건 중 63건이 0.5를 넘었다.
즉 **기본값 0.5 게이트가 정상 주행을 중단시켜서 50.0으로 완화한 것**이 실제 경위이며,
근거는 `jdamr_cube_navigation/test/test_keepout_config.py`에
"drive at x covariance 0.519"로 남아 있다. 게이트 자체는 살려 두어
진짜로 위치를 잃은 경우는 여전히 걸린다.

### tf_map_odom_latency.csv (6,710행)

bag의 `/tf` 28,277건 중 map->odom 변환만 추린 것.

**latency_ms는 음수가 정상이다.** AMCL은 map->odom을 `transform_tolerance`
(nav2_params.yaml에서 1.0초)만큼 미래 시각으로 찍어 발행한다.
따라서 `recv_time - header.stamp`는 -900 ms 근처에 몰린다(p50 -901.2 ms).

온보드 이전 판단의 근거로 쓸 지표는 **발행 간격(gap_since_prev_ms)** 이다.

| 지표 | 값 |
|---|---|
| p50 | 103.8 ms (약 10 Hz) |
| p90 | 174.1 ms |
| p99 | 713.3 ms |
| 최대 | 23,540.5 ms |
| 1초 초과 | 20회 |
| 5초 초과 | 2회 |

정상 구간은 10 Hz를 지키는데 꼬리가 길다. 최대 23.5초 동안 map->odom이
갱신되지 않았고, 이 구간에서 로봇은 오도메트리만으로 달린 셈이다.

### slam_backend_comparison.json

Cartographer와 SLAM Toolbox를 **같은 bag**으로 오프라인 재생한 대조 결과.
소스는 `corridor_keepout_onboard_20260901T172141` (302.94초)다.

| | Cartographer | SLAM Toolbox |
|---|---|---|
| map->odom 갱신 | 6,053회 | 30,978회 |
| 오도메트리 대비 RMS | 0.670 m | 0.521 m |
| 오도메트리 대비 최대 편차 | 1.542 m | 1.162 m |
| 지도 점유 길이 | 83.5 m | 97.9 m |
| 지도 범위 | 28.4 x 9.35 m | 32.2 x 11.3 m |

## 아직 없는 것

- **UMBmark 축소판 실행 로그** — `wheel_calibration.py`는 있으나 출력 로그를
  저장해 둔 것이 없다. 남은 것은 README_JDAMR.md의 결과 수치뿐이라
  수렴 그래프는 실기 재측정 없이는 만들 수 없다.
- **loop closure 제약 TP/FP 표** — 오프라인 재생 가드가 `/map`과 `/tf`만
  녹화한다. Cartographer의 `/constraint_list`가 어느 재생분에도 남아 있지 않아
  녹화 토픽을 늘려 재생을 다시 돌려야 한다.
- **922초 bag(`corridor_keepout_onboard_20260901T170550`) 재생 결과** —
  재생이 6.3초 만에 `tf_replay_filter` 조기 종료로 실패했다.
  백엔드 비교 목적은 위 172141 대조로 충족된다.
