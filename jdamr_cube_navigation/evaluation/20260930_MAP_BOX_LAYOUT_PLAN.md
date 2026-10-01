# 새 지도 작성 · 테이블 박스 배치 · 도크 등록 계획 v1

- 상태: **승인 대기 (pending approval)**. 실행은 사용자가 나중에 진행한다.
- 모드: RALPLAN-DR SHORT (사용자 인터뷰 없음). Planner 초안이다. Architect·Critic 합의 검토는 사용자 지시("검수 없이 진행")로 생략했다. 초안 뒤 반영은 R8·후속 3의 실패 기록 절 번호(§16.4)와 원형 스크립트 경로뿐이다
- 작성: 2026-09-30, Planner
- 범위: 계획만. 이 계획 작성 중 로봇 이동·파이 접속·저장소 수정·커밋 없음. 현 지도에 대한 읽기 전용 오프라인 분석만 수행했다.
- 대상 시나리오(기존 실행기 그대로): 도크 배치·RViz 초기화 → 관측 지점 → RGB-D 박스 앞면 검출 → 면 정렬 → 5 cm 접근 → 5 s 대기 → 후진 이탈 → 도크 후진 복귀 (`jdamr_depart.py go <table>` = `box_service --execute --search --return-home`).

---

## 0. 결론

1. 현 지도(`map_dockfix` + `keepout_dockfix`)에서는 두 테이블을 동시에 만족하는 배치가 없다. 현 keepout이 운영 공간을 폭 약 1.2 m 띠로 좁혀, 회전 원(지름 1.03 m)을 만족하는 박스 후보가 y≈0 한 줄에만 있고 그 박스가 동쪽 통로를 막는다. 오늘 쓴 table_01 배치 (1.80, 0.05)도 관측 지점 회전 여유 0.50 m(요구 0.514 m)와 카메라 시야 1.18–1.23 m 안의 큰 장애물 서쪽 벽 때문에 기준 미달이다(§5).
2. 그래서 **새 지도를 먼저 만들고, 그 지도에서 배치를 한 번 계산한 뒤 박스를 놓는다.** 현 지도 결과는 방법 검증과 물리 참고안으로만 쓴다.
3. 지도는 기존 수동 매핑 경로(`jdamr-operator-mapping.service` = Cartographer + map saver + 웹 조종기)로 만든다. 도크에서 시작해 도크로 돌아오고, 방에서 사람·가방·박스를 치운다. 박스는 지도에 넣지 않는다.
4. keepout은 "LiDAR가 몸체를 못 보는 금지 물체 + 방 밖으로 새는 틈"만 담는 최소 설계로 바꾸고, 기존 `keepout_mask build`로 만든다. 모든 회전 자세에서 0.514 m, 모든 이동 통로에서 런타임 footprint 기준으로 떨어뜨린다.
5. 배치는 정면 직선 접근 규칙(사전 지점 → 관측 지점 → 정렬 → 5 cm가 한 직선)으로 계산하고, 오프라인 검사 1건(기존 `tools/geometry_check.py`의 `layout` 확장)으로 수치 판정한다. 출발 절차에는 새 검사를 넣지 않는다. 설치 뒤 `jdamr_depart.py init` 한 번과 조건부 재확인만 거친 뒤 `jdamr_depart.py go table_01`로 출발한다.

---

## 1. 근거와 고정 수치

| 항목 | 값 | 출처 |
|---|---|---|
| 런타임 footprint (base_footprint) | x ∈ [−0.295, 0.085], y ∈ [−0.29, 0.29] | `config/new_base_nav2_params.yaml` |
| 제자리 회전 반경 (외접원) | 0.414 m (= √(0.295² + 0.29²)) | footprint에서 계산 |
| 회전 원 요구 여유 `R_req` | 0.414 + 0.10 = **0.514 m** | 도크 수정 때 쓴 기준(`dockfix_provenance.json`) |
| 이동 통로 반폭 | 0.29 + 0.10 = **0.39 m** | 같은 기준 |
| 차체 앞면–바퀴축 `front_extent` | 0.065 m | `new_base_geometry.yaml` `front_to_wheel_axis` (box_service가 이 값 사용) |
| 카메라 렌즈 위치 | 차체 앞면과 같은 평면 → base_link 앞 0.065 m, 높이 0.215 m | `depth_box_parking_provenance.yaml`, 인계 문서 |
| LiDAR | base x −0.01, 높이 0.150 m, yaw π, range_min 0.28 m | `new_base_geometry.yaml`, `/scan` |
| 박스 면 기준 자세 (면 중심 F, 바깥 법선 n) | 최종 F + 0.115n, 정렬 F + 0.515n(면 정렬 gap 0.45), 이탈 F + 0.565n | `box_service.visit_observed_box`(gap 0.45/0.05, `front_extent + gap`), `ESCAPE_CLEARANCE_M = 0.565` |
| 앞면 검출기 | depth 0.35–2.0 m, 폭 0.18–1.20 m, 높이 0.08–0.80 m, 법선 z ≥ 0.90(시선과 약 ±25.8°), 안정 8프레임·3 cm/3 cm/3°, RANSAC 최대 평면 1개/프레임 | `config/depth_box_parking.yaml`, `box_top_detection.py` |
| 카메라 | Orbbec Astra S, 수평 반시야각 약 29° | 설계 문서, 오늘 관측 |
| LiDAR 면 증인 | 면 중심 접선 ±0.20 m, 평면 0.04 m 안 점 ≥ 5, 퍼짐 ≥ 0.08 m, RMS ≤ 0.015 m, 법선 차 ≤ 5° | `box_lidar_witness.py` |
| 영역 등록 | init이 도크 스캔에서 마커 0.5 m 안, 길이 0.25–0.7 m의 지도 밖 LiDAR 군집을 찾고 반경 0.45 m 영역으로 등록. box_service는 관측 면 중심이 영역 안이어야 함 | `jdamr_depart.py init`, `box_service.observe_target` |
| transit 도착 판정 | 위치만 보는 goal checker, xy 0.15 m. 로봇 0.25–0.45 m 안의 waypoint는 즉시 완료 | §14.1, 오늘 관측 |
| RPP 제자리 회전 | `rotate_to_heading_min_angle` 1.57 rad, 회전 원 안 keepout·장애물이면 104/703로 실패 | nav2 params, 오늘 관측 |
| keepout | build 시 다각형 확장 `safety_margin_m ≥ 0.35`, 세션은 막힌 칸 1개 이상 요구, KeepoutFilter는 팽창 없이 unknown 칸을 FREE로 덮어씀 | `keepout_mask.py`, §14.1 |
| 지도 | Cartographer 2D(`jdamr_cube_2d_real.lua`), 해상도 0.05 m, trinary 0.65/0.196 | `cartographer_real.launch.py`, 현 지도 yaml |

**앞선 배치 수치 정정.** 오늘 계산한 "좋은 배치"(면 (1.80, 0.05))는 최종 base를 면에서 0.135 m, 이탈을 1.15 m로 적었지만 실행 코드는 `front_extent` 0.065 m를 써서 최종 0.115 m, 이탈은 면에서 0.565 m(= x 1.235)다. 이 계획은 코드 값을 쓴다.

---

## 2. RALPLAN-DR 요약

### 2.1 원칙 (Principles)

- **P1 기존 경로 재사용과 출발 경량성.** `jdamr_depart.py init/go`, `restaurant_service init/teach-home`, `box_service --search`, `keepout_mask build/validate`, `static_corridor_clear`, `scan_match`/`dock_survey`를 그대로 쓴다. 출발 게이트나 승인 계층을 추가하지 않는다. 검증은 준비·설치 단계에서 한 번만 하고, 바뀐 것이 없으면 반복하지 않는다.
- **P2 지도에는 빈 방의 정적 구조만 담는다.** 사람·가방·박스는 지도에서 뺀다. 박스는 매 실행에서 LiDAR와 Depth로 새로 관측한다.
- **P3 수치로 먼저 판정한다.** 회전 자세 0.514 m, 이동 통로는 런타임 footprint의 `static_corridor_clear`, 관측은 정면 0.6–0.75 m 기준으로 판정한다.
- **P4 판정 기준은 실물이다.** 도크·박스 위치는 테이프 표시로 재현하고 스캔 정합과 LiDAR 군집으로 등록한다. 내부 추정과 외부 실측을 구분한다.
- **P5 최소 변경.** 저장소 코드는 바꾸지 않고 데이터(지도·마스크·registry·route·TABLES)로 해결한다. 코드 변경이 필요한 항목은 후속으로 분리한다.

### 2.2 결정 동인 (Top 3)

- **D1 오늘 실패 원인 제거:** 회전 원 안 keepout·장애물(104/703, `nav2_spin_failed`), 비스듬한 관측·다른 평면 선택(탐색 회전 반복), 지도 흔적 칸(도크 복귀 정적 검사 거부).
- **D2 출발 절차를 늘리지 않음(AGENTS.md):** 출발은 `go` 한 줄로 유지하고 검사는 준비 단계에 둔다.
- **D3 새 지도 좌표계:** 현 좌표, 마커, TABLES, 주석, registry 해시를 재사용할 수 없다. 도크 기준 물리 좌표와 새 지도 재계산이 필요하다.

### 2.3 선택지

**결정 1 — 배치를 언제 계산하나**

| 선택지 | 장점 | 단점 |
|---|---|---|
| 1A 현 지도에서 지금 확정하고 새 지도로 이식 | 박스 위치를 일찍 정해 설치를 미리 준비할 수 있음 | 현 지도와 현 keepout에서 두 테이블 동시 만족안이 없음(§5). 새 지도는 좌표계·흔적·keepout이 달라 어차피 재검증 필요 |
| **1B 새 지도 뒤 계산, 현 지도 결과는 참고안 (채택)** | 실제 좌표와 실제 keepout으로 한 번 계산. 오늘 원인을 새 지도에서 수치로 확인 | 박스 설치가 지도 작성·계산 뒤로 밀림. 오프라인 도구 확장 1건 필요 |

**결정 2 — 박스 위치의 기준**

| 선택지 | 장점 | 단점 |
|---|---|---|
| **2A 계산 배치 → 벽/도크 기준 테이프로 설치 → init LiDAR 군집으로 등록하고 계획값과 대조 (채택)** | 다음 날에도 같은 자리에 재현 가능. 회전 원·시야·상호 차단 검사가 설치 전에 끝남. 등록 기준은 실측 | 2–3 m 줄자 오차(약 ±5 cm, ±3°) → 영역 반경 0.45 m와 매 실행 면 관측이 흡수. 오차 ±0.10 m/±5°로 섭동 검사 |
| 2B 자유 배치 → LiDAR 등록만 | 측정이 없어 빠름 | 회전 원·시야·통로 차단을 설치 뒤에야 알 수 있어 재배치를 반복함. 오늘 table_01 협소 배치(keepout 띠 옆 회전 차단)와 같은 실패 유형 |

**결정 3 — keepout 설계**

| 선택지 | 장점 | 단점 |
|---|---|---|
| **3A 최소·내용 기반 (LiDAR로 몸체를 못 보는 가구, 낮은/투명 물체, 사용자 지정 금지 구역) + 벽 바깥 폐쇄 다각형 + 누출 검사 (채택)** | 운영 공간이 가장 넓음. 회전 원 충돌과 현장 셀 해제(오늘 46칸)가 사라짐 | KeepoutFilter의 unknown→FREE 때문에 벽 틈으로 새는지 검사해야 함. 어디가 진짜 금지 구역인지 사용자 확인 필요(U2) |
| 3B 영역 기반 (현행처럼 운영 구역 밖을 큰 사각형으로 덮음) | 누출 걱정이 적은 기존 방식 | 현 지도에서 통로 폭 약 1.2 m. 후보가 한 줄에 몰리고(33개 모두 y∈[−0.05, 0.05]), 도크 staging을 막아 46칸을 현장에서 풀었고, 박스 면 후보가 keepout 띠 안에 떨어짐(§14.1) |

**배제한 대안과 이유**

- keepout 전부 해제: `keepout_mask.validate_mask`가 막힌 칸 0개 마스크를 거부한다. unknown→FREE 누출도 생긴다.
- 박스를 지도에 포함: 정밀 접근은 면이 지도 밖이어야 한다. 면 셀이 정적 장애물이면 최종 목표가 inscribed가 되어 끝점 검사에서 거부될 수 있다(§14.5 격자 모델). init 영역 검출도 지도 밖 군집을 전제로 한다.
- 새 SLAM 스택: 기존 수동 매핑 경로로 현 운영 지도(`manual_final`)를 만든 기록이 있다.
- 관측 지점 yaw 강제로 방향 맞추기: transit checker가 위치만 본다. 코드 변경 없이 직선 사전 지점으로 도착 방향을 만든다.

---

## 3. 단계별 계획 (A–F)

진행 순서: **C-1(도크 자리·테이프) → A(지도) → C-2(도크 자세 확인) → B(keepout) → D(배치 계산) → E(박스 설치) → F(등록·확인) → `go table_01`**

새 자산은 새 데이터 폴더(예: `$HOME/jdamr_data/map_<날짜>_<태그>/`, 이름은 실행 때 정함)에 만든다. `$HOME/jdamr_data/map_update_20260929_4XUrkc/`와 `$HOME/jdamr_data/maps/20260922_manual_final_run2/`는 되돌림용으로 그대로 둔다.

### A. 새 지도 작성과 품질 게이트

**절차 (기존 경로)**

1. 사전 조건
   - 파이 `loginctl show-user lim -p Linger` = yes인지 확인한다.
   - PC 표시와 Nav2 세션을 정지한다(`jdamr_depart.py display-stop`, `session-stop`). 두 유닛 모두 inactive인지 확인한다.
   - 방 정리(E-1~E-3)를 마치고 로봇을 도크 테이프에 놓은 뒤 충전 케이블을 뺀다.
2. 파이에서 `sudo systemctl start jdamr-operator-mapping.service`를 실행하고 웹 조종기(`http://jdamr.local:8080`, mDNS가 실패하면 `http://192.168.0.159:8080`)로 수동 주행한다.
3. 주행 방법
   - 저속으로 움직이고 급회전하지 않는다.
   - 벽을 따라 방을 한 바퀴 돈다.
   - 도크 주변과 두 테이블 후보 구역은 서로 다른 방향에서 두 번 이상 스캔한다.
   - (권장) 두 테이블 후보 구역에서 각각 3 s 정지하고, PC에서 정지 스캔을 1회 캡처한다(`tools/capture_scan2.py`를 ssh로 실행하는 기존 방식).
   - 마지막에 도크 테이프로 돌아와 정지한다.
4. 파이에서 저장한다.
   - `ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: <폴더>/map, image_format: pgm, map_mode: trinary, free_thresh: 0.196, occupied_thresh: 0.65}"`
   - 이어서 `/finish_trajectory`(trajectory 0)와 `/write_state`(`<폴더>/map.pbstream`)를 호출해 원본을 보존한다.
5. 로봇을 도크에 둔 채 정지 스캔 1회를 캡처해 `scan_dock_end.json`으로 저장한다.
6. `sudo systemctl stop jdamr-operator-mapping.service`를 실행하고 inactive인지 확인한다.

**수용 기준 (PC 오프라인 판정)**

| ID | 기준 | 수치 | 확인 방법 |
|---|---|---|---|
| A-G1 | 형식 | 해상도 0.05, origin yaw 0, trinary, 0.65/0.196, 픽셀 값 ⊂ {0, 205, 254} | yaml 읽기 + `keepout_mask._map_metadata`로 로드. `static_corridor_clear`와 keepout build가 요구하는 조건 |
| A-G2 | 운영 구역 관측 | 도크 반경 1.5 m, 두 테이블 후보 구역 반경 2.0 m 안 unknown 칸 0개(고립 2칸 이하 구멍은 A-G4로 처리) | layout 도구의 영역별 unknown 집계 |
| A-G3 | 벽 일관성 | 같은 벽의 평행 이중선(간격 ≥ 0.10 m) 0개 | 지도 overlay PNG 육안 확인. 실패하면 **재매핑**(셀 수정 금지) |
| A-G4 | 매핑 흔적 | 운영 구역 안에서 벽과 닿지 않는 점유 성분(≤ 8칸)과 unknown 구멍을 모두 목록화하고 하나씩 해소한다. 실물이 있으면 유지, 없으면 정지 스캔 증거(해당 칸을 통과한 빔 ≥ 100, 반사 0. 오늘 도크 칸은 287/0)로만 해제하고 provenance JSON을 남긴다. 해제 칸이 3개를 넘으면 재매핑 | `dockfix_provenance.json`과 같은 형식 |
| A-G5 | 정합 품질 | 도크 스캔: `dock_survey.py` `pose_ok`(inlier_5cm ≥ 0.50, 2위와 평균거리 차 ≥ 0.02 m), **목표 inlier_5cm ≥ 0.60**(9/29 올바른 자세 64 %). 테이블 구역 스캔(권장): 같은 기준 | `tools/dock_survey.py scan_dock_end.json map.yaml <keepout> <out>`. keepout이 아직 없으면 전부 FREE인 임시 사본으로 돌린다 |
| A-G6 | 도크 footprint | 정합된 도크 자세에서 footprint 안 점유·unknown 칸 0개 | `geometry_check.footprint_cells_clear`. 오늘 12:00 흔적 2칸 재발 방지 |

**산출물:** `map.{pgm,yaml,pbstream}`, `scan_dock_end.json`, `map_quality.json`(G1–G6 결과), overlay PNG.

### B. keepout 재설계 규칙

**대상 (이것만 넣는다)**

- K1-a: LiDAR 평면(바닥 위 0.15 m)에서 몸체 일부만 보이는 가구. 책상·테이블 상판, 의자 좌판, 선반 돌출이 해당한다. 현 지도의 "central obstacle" 구역처럼 다리만 점유로 보이는 경우다.
- K1-b: 0.15 m보다 낮은 물체, 케이블, 문턱.
- K1-c: 유리, 거울, 검은 광택면.
- K1-d: 방 밖으로 이어지는 문과 틈. KeepoutFilter가 unknown을 FREE로 덮어쓰므로 폐쇄 다각형이 필요하다.
- K1-e: 사용자가 지정한 출입 금지 구역(U2).

**넣지 않는 것:** 박스(지도 밖이어야 함), LiDAR가 온전히 보는 벽(지도 점유로 충분), 도크·staging·경로 주변의 임의 띠.

**만드는 법:** map 좌표 다각형으로 `keepout_zones.yaml`을 쓰고 `python3 -m jdamr_cube_navigation.keepout_mask build --zones <폴더>/keepout_zones.yaml --output-prefix <폴더>/keepout`을 실행한다(`safety_margin_m: 0.35`, 도구 최소값).

- 다각형은 실물 외곽선에 그린다. 미리 팽창시키지 않는다.
- 벽 바깥이나 문을 막는 폐쇄 다각형은 안쪽 변을 벽·문 선에서 0.35 m **바깥**에 둔다. 그래야 팽창한 마스크가 벽 선에서 멈추고 방 안을 먹지 않는다.
- 파일 이름은 `keepout.{pgm,yaml,json}`으로 한다. `jdamr_depart.py sync`가 이 이름으로 해시를 대조한다.

**수용 기준**

| ID | 기준 | 수치 | 확인 방법 |
|---|---|---|---|
| B-1 | 세션 조건 | 막힌 칸 ≥ 1, 지도와 격자 일치 | `python3 -m jdamr_cube_navigation.keepout_mask validate --mask keepout.yaml --map map.yaml` |
| B-2 | 회전 여유 | 모든 회전 자세(staging=home_exit, 방향 전환 waypoint, pre, 관측, 정렬, 이탈)에서 점유·unknown·keepout 거리 ≥ 0.514 m | layout 도구 L1 |
| B-3 | 통로 | 모든 이동 통로에서 `static_corridor_clear`(런타임 footprint, keepout 포함) = True | layout 도구 L2(실행기가 쓰는 같은 함수) |
| B-4 | 박스·최종 자세 | 박스 몸체 둘레 0.30 m와 최종·정렬 footprint 둘레 0.10 m 안 keepout 칸 0개 | layout 도구 L3 |
| B-5 | 누출 | 도크에서 (지도 FREE ∪ unknown) − keepout을 4-이웃으로 채웠을 때 지도 가장자리에 닿지 않음(= bounded) | layout 도구 L6 |

**충돌 시 우선순위:** keepout(K1-a~d)은 안전 사실이므로 배치를 옮긴다. K1-e(사용자 임의 구역)만 사용자 확인 뒤 줄일 수 있다.

**변경 규칙:** keepout을 바꾸면 registry 해시가 바뀐다. F 이전에 확정하고, F 이후에 바꿨다면 세션 재시작과 init을 다시 한다.

### C. 도크 등록

**C-1 (지도 작성 전, 물리)**

- 차체 뒷면 뒤 여유 ≥ 10 cm.
- 도크 뒤 모서리 선에서 앞으로 **150 cm** 직선 빈 바닥이 필요하다(0.295 + 0.70 + 0.514). 앞 70 cm 지점을 중심으로 지름 **105 cm** 원 안도 비어 있어야 한다.
- 테이프: 두 바퀴 바깥 접지점 옆 L자(각 10 cm), 차체 뒤 모서리 선, 차체 중심선 앞쪽 30 cm 화살표. 배치 허용은 ±2 cm, ±2°로 본다.
- 현 도크 자리를 유지할지는 U4에서 정한다. 현 지도에서 도크 자세의 회전 여유는 0.47 m다. 도크에서는 회전하지 않으므로 허용하지만, 새 자리에서도 staging 기준을 만족해야 한다.

**C-2 (지도 저장 뒤, 오프라인)**

- A-G5의 도크 정합 자세를 계획용 도크 자세로 쓴다. 매핑을 도크에서 시작하면 도크 ≈ (0, 0, 0°)가 기대되지만 판정에 쓰지 않고 정합값을 쓴다.
- `home_exit` = staging = 도크 + 0.70 m(도크 방향)로 기본 설정한다. 출발 첫 구간은 직진 0.70 m이고(0.45 m보다 길다), 방향 전환은 staging 원 한 곳에서만 한다.

| ID | 기준 | 수치 | 확인 방법 |
|---|---|---|---|
| C-a | 도크 footprint | 점유·unknown·keepout 칸 0 | A-G6 |
| C-b | staging 회전 | 거리 ≥ 0.514 m | layout 도구 L1 |
| C-c | 도크 후진 통로 | `reverse_waypoints(staging→dock)` + `static_corridor_clear`(런타임 footprint) = True | `restaurant_service._reverse_path_valid`와 같은 함수 조합 |
| C-d | 배치 오차 내성 | 도크 ±0.03 m, ±3° 섭동에서도 C-a~c 통과 | layout 도구 섭동 |

**C-3 (F 단계):** `jdamr_depart.py init`의 teach-home(`--approach-offset-m 0.7 --parking-direction reverse --replace`)으로 등록한다. 수용 기준은 F-5에 있다.

### D. table_01 / table_02 배치 계산

**입력:** 새 `map.yaml`, `keepout.yaml`, C-2 도크 자세, 박스 치수 W×D×H(U1), 접근 방향 선호(U3).

**자세 계산 (면 중심 F, 바깥 법선 n, yaw = atan2(−n))**

| 자세 | base_link 위치 | 비고 |
|---|---|---|
| 최종 | F + 0.115n | 앞면 5 cm (코드 값) |
| 정렬 | F + 0.515n | 재관측 시 카메라–면 0.45 m |
| 이탈 | F + 0.565n | `ESCAPE_CLEARANCE_M`, 복귀 전 회전 지점 |
| 관측 | F + 0.70n (기본) | 카메라–면 0.635 m. 위치 전용 checker(xy 0.15)가 짧게 멈춰도 0.60–0.79 m. 11:36 실행은 waypoint에서 0.14 m 떨어져 도착 |
| 사전(pre) | F + 1.25n | 관측과 0.55 m, 같은 직선 → 도착 방향이 −n |
| route | 도크 → home_exit(staging) → (필요한 방향 전환 지점) → pre → 관측 | 연속 waypoint 간격 ≥ 0.50 m |

**수용 기준 (테이블별, 모두 layout 도구 한 번 실행으로 판정)**

| ID | 기준 | 수치 |
|---|---|---|
| D-1 회전 원 | pre, 관측, 정렬, 이탈, 방향 전환 지점의 점유·unknown·keepout 거리 | ≥ 0.514 m |
| D-2 직선 접근 | pre→관측→정렬→최종이 n 축 위(횡오차 0). 통로 `static_corridor_clear` | True. 간격 ≥ 0.50 m |
| D-3 시야 | 관측 자세에서 면 중심 방위, 계획 카메라–면 거리 | 0°, 0.60–0.70 m |
| D-4 다른 평면 | 관측 카메라 시야(±29°) 안, 박스 실루엣에 가려지지 않은 점유 칸 | depth ≤ 1.5 m에서 **0개(필수)**, ≤ 2.0 m도 0개 권장. 못 지키면 R2 완화 |
| D-5 박스 몸체 | 박스 외곽 + 0.30 m 안 점유·unknown·keepout 칸 | 0개. 벽과 붙으면 지도 밖 군집(`UNMAPPED_M` 0.10)과 앞면 평면이 분리되지 않음 |
| D-6 상호 차단 | 다른 박스 외곽 + 0.39 m가 이 테이블 통로(pre→최종)나 이탈→staging 직선과 겹침 / 외곽 + 0.514 m 안에 회전 자세가 들어옴 | 0건 |
| D-7 도크 가시성 | 도크 LiDAR에서 박스 외곽까지 지도 점유에 가리지 않은 시선. 도크–박스 거리 | 가시, ≤ 4.0 m (추론: init 군집 지속성) |
| D-8 설치 오차 내성 | 박스 ±0.10 m, ±5° 섭동에서 D-1, D-4, D-5 재판정 | 모두 통과 |

**산출물 (layout 도구 출력)**

- `layout_report.json`: 기준별 PASS/FAIL, 최소 여유, overlay PNG.
- `jdamr_depart.py`에 넣을 `TABLES` 항목(`marker` = 계획 박스 중심, waypoints)과 `HOME_EXIT`.
- 표시용 `annotation.json`: `source_map_sha256`, `markers.table_1/2.approximate_map_xy_m`. 지도 이미지 해시가 바뀌면 다시 만든다.
- 설치용 표: 박스 앞면 두 바닥 모서리 좌표. 가까운 두 벽까지 거리(cm)를 우선 쓰고, 도크 기준점에서 앞/우(cm)를 보조로 쓴다. 앞면 방향도 적는다.

### E. 사용자 설치 체크리스트 (현장용)

**[지도 만들기 전]**

- [ ] E-1 운영 구역에서 사람·가방·박스를 치운다. 의자는 치우거나, 시연 때와 똑같은 자리에 둔다.
- [ ] E-2 문은 시연 때와 같은 상태(열림/닫힘)로 둔다.
- [ ] E-3 박스 두 개는 방 밖에 둔다. 지도에 박스가 들어가면 안 된다.
- [ ] E-4 도크 테이프를 붙인다. 두 바퀴 바깥 접지점 옆 L자(각 10 cm), 차체 뒤 모서리 선, 중심선 앞쪽 30 cm 화살표를 표시한다. 뒤쪽은 벽까지 10 cm 이상, 앞쪽은 뒤 모서리 선에서 150 cm까지 비워 둔다. 앞 70 cm 지점을 중심으로 지름 105 cm 원 안에 물건이 없어야 한다.
- [ ] E-5 로봇을 도크 테이프에 맞추고(±2 cm) 충전 케이블을 뺀다.

**[배치 계산표를 받은 뒤]**

- [ ] E-6 박스 두 개의 가로(W, 목표 면 폭)·깊이(D)·높이(H)를 cm로 적는다. 권장: W 30–45, H 30–45, D 20 이상. 골판지처럼 평평하고 무광인 면이 좋고, 검은색·광택면은 피한다. 책 등으로 무게를 더해 밀리지 않게 한다.
- [ ] E-7 계산표의 두 점(목표 면 두 바닥 모서리)을 벽 기준 거리(cm)로 재서 바닥에 테이프를 붙인다. 벽 기준이 어려우면 도크 기준점(두 바퀴 접지점의 가운데)에서 앞 a cm, 오른쪽 b cm로 잰다.
- [ ] E-8 목표 면(넓은 면)을 계산표의 방향(로봇이 다가오는 쪽)으로 두고, 면 아래 모서리를 테이프 선에 맞춘다. 비틀림은 45 cm 폭 기준 양 끝 차이 2 cm 이내(약 ±3°)로 한다.
- [ ] E-9 목표 면 앞으로 180 cm, 폭 105 cm를 비운다. 박스 둘레 30 cm 안에는 벽·가구가 없어야 한다.
- [ ] E-10 박스 뒤로 90 cm, 좌우 각 85 cm 안에 의자·가방·판자·벽면처럼 로봇 쪽을 향한 평평한 면이 없게 한다(박스 자신 제외).
- [ ] E-11 (5 cm 확인용, 권장) 목표 면 앞 5 cm에 면과 평행한 테이프 선을 붙이고, 주차 순간을 위에서 휴대폰으로 촬영한다. 대기가 5 s라 줄자로 재기엔 짧다.
- [ ] E-12 설치 뒤에는 박스, 도크 테이프, 의자를 움직이지 않는다. 움직였다면 말해 주면 F-5부터 다시 한다.

### F. 설치 후 등록·확인 순서 (끝나면 `go table_01` 준비 완료)

| 순서 | 할 일 | 수용 기준 |
|---|---|---|
| F-0 (PC, 1회) | 새 폴더 자산을 구성한다. `restaurant_service init --map <폴더>/map.yaml --keepout <폴더>/keepout.yaml --registry <폴더>/service_destinations.yaml`, `annotation.json`, `layout_report.json`을 만든다. 기존 `tools/`, `restaurant_phase2.rviz`, `fastdds_*.xml`을 복사하고 `jdamr_depart.py` 상수(`BASE`/`P2`, `MAP_YAML`, `KEEPOUT_YAML`, `REGISTRY`, `ANNOTATION`, `TABLES`, `HOME_EXIT`)를 새 값으로 바꾼다 | keepout validate OK, registry 로드 OK(세션 스크립트의 `validate_registry`와 같은 `load_registry`), layout 전 항목 PASS |
| F-1 | `jdamr_depart.py sync` | "hashes match" |
| F-2 | 사용자가 E-5~E-12를 마쳤다고 알림 | 이후 배치·케이블 질문을 반복하지 않음 |
| F-3 | `jdamr_depart.py display-start` (Nav2 세션보다 먼저) | 표시 유닛 5개 active |
| F-4 | `jdamr_depart.py session-start` | `STARTED` 또는 `REUSED` (precision, prepare-only) |
| F-5 | RViz 2D Pose Estimate를 클릭하고 `jdamr_depart.py init` 실행 | 스캔 정합 inlier_5cm ≥ 0.50(기록, 목표 ≥ 0.60), 전역 일치. AMCL–정합 ≤ 0.05 m, 2°. teach-home 완료. `regions.json`에서 두 테이블 basis = "unmapped LiDAR cluster near marker", 군집 중심이 계획 면 중심에서 ≤ 0.30 m |
| F-6 (조건부, 오프라인) | 교시 home이 C-2 계획 도크에서 0.05 m 또는 3°를 넘게 벗어났거나 F-5 군집이 0.30 m를 넘었을 때만 layout 도구를 실제 home/영역으로 다시 실행 | 전 항목 PASS. 실패하면 박스나 로봇을 물리적으로 옮긴 뒤 F-5부터 반복. 변화 없이 재시도하지 않음 |
| F-7 | 준비 완료. 사용자의 출발 요청에 `jdamr_depart.py go table_01` 실행. 도크 복귀 뒤 요청하면 `go table_02` (로봇이 안 움직였으면 재init 불필요) | 출발 시 추가 검사 없음. box_service 내부 preflight(계획 끝점 허용오차)가 기존대로 이동 전에 돈다 |
| F-8 | 주행이 끝나면 `display-stop`, `session-stop`을 실행하고 모두 inactive인지 확인한다. 결과는 `20260929_PARKING_FAILURES.md`에 새 절로 기록한다(실차 5 cm는 E-11 외부 근거와 함께) | AGENTS.md 정리 규칙 |

첫 주행에서 볼 기존 기록(판정 게이트 아님):

- 관측 진단 이벤트의 카메라 방위 오차와 영역 거리(D6 진단)
- `box_lidar_witness_rejected` 빈도
- 정렬 지점 재관측 결과(R1)

---

## 4. 오프라인 검사 도구 1건 (정당화)

- **무엇:** 기존 `tools/geometry_check.py`에 `layout` 서브명령을 추가한다(데이터 폴더 도구, 저장소 밖).
- **왜 새로 필요한가:** 박스 후보 몸체, 두 테이블 route, 회전 원, 카메라 시야, keepout 누출을 함께 판정하는 기존 도구가 없다. 오늘 수작업 계산으로 협소 배치(회전 차단)와 시야 속 다른 평면을 놓쳤다. 오프라인 전용이고 출발 경로에 끼지 않는다.
- **재사용:** `reverse_parking.static_corridor_clear`·`reverse_waypoints`(실행기와 같은 함수), `geometry_check.fields/clearance/footprint_cells_clear`, `keepout_mask._map_metadata`.
- **수정:** `ROTATION_RADIUS` 0.43(근사)을 footprint에서 계산한 0.414 + 여유 0.10으로 바꾼다.
- **검사 항목:** L1 회전 여유, L2 통로(static_corridor_clear), L3 박스 몸체·최종 둘레, L4 상호 차단, L5 카메라 시야 평면, L6 keepout 누출(flood fill), L7 waypoint 간격·직선성·route 시작=home, L8 도크 footprint, 섭동(도크 ±0.03 m/±3°, 박스 ±0.10 m/±5°).
- **출력:** `layout_report.json`, overlay PNG, `TABLES`/`HOME_EXIT` 조각, `annotation.json`, 설치 cm 표.
- **원형:** 이번 분석에 쓴 읽기 전용 스크립트가 원형이다. 저장소 `scripts/phase2_20260930/layout_prototypes/`의 `layout_probe.py`, `layout_search.py`, `layout_joint.py`.
- **완료 기준:** 현 지도에서 §5 수치를 재현한다(T1 (1.80, 0.05)의 관측 회전 여유 0.50 m 판정 FAIL, 시야 1.5 m 안 점유 39칸).

---

## 5. 현 지도 참고안 (읽기 전용 분석, 새 지도에서 재계산 필수)

박스 W 0.40, D 0.30 가정. 지도는 `map_dockfix.yaml`, 도크는 (−0.608, −0.843, 92.3°). "서향"은 목표 면이 서쪽을 보고 로봇이 동향으로 다가간다는 뜻이다. 판정 기준은 관측 0.70/pre 1.25(격자 탐색 행만 0.75/1.30)다.

| 후보 | keepout | 결과 | 핵심 수치 |
|---|---|---|---|
| T1 면 (1.80, 0.05), 서향(오늘 안) | 현행 | **FAIL** (D-1, D-4) | 관측 (1.10, 0.05) 회전 여유 0.50 m < 0.514. 큰 장애물 서쪽 벽(x 2.35–2.40, y 0.44–0.69)이 시야 1.18–1.23 m, 방위 17–29°에 있음(1.5 m 안 점유 39칸) |
| T1 면 (1.00, −0.05), 서향 | 현행 | 단독 PASS, 두 테이블 조합 **FAIL** (D-6) | 최소 회전 여유 0.55 m, 시야 1.98 m에 1칸. 박스(y −0.25~0.15)가 폭 1.18 m 통로(y −0.58~0.60)를 막아 남는 틈이 0.45/0.33 m로 차폭 0.58 m보다 좁음. 도크 기준 앞 0.73 m, 오른쪽 1.64 m |
| T2 면 (2.30, −0.75), 남향(로봇이 남쪽에서 북향으로 접근) | 현행 | **FAIL** | 관측 (2.30, −1.45)과 pre (2.30, −2.00)가 현 "bottom" keepout 띠 안 |
| 같은 T2 | keepout 무시(최소 keepout 가정) | PASS | 최소 회전 여유 0.65 m, 시야 2.0 m 안 점유 0칸. 도크 기준 앞 −0.02 m, 오른쪽 2.91 m |
| 격자 탐색 (0.1 m, 4방향, 관측 0.75/pre 1.30) | 현행 | 단독 후보 33개, 모두 y∈[−0.05, 0.05] 한 줄 | D-4 통과 후보는 x ≤ 1.5뿐이고 모두 통로를 막음 |
| 같은 탐색 | 무시 | 단독 66개, 두 테이블 조합 199쌍 | 예: T1 (1.00, −0.05) 서향 + T2 (2.30, −0.75) 남향. 단, T2 접근로가 현 "bottom" 구역을 지나고 중앙 가구 구역(다리만 보임)은 K1-a로 계속 막아야 해 U2 확인 전에는 채택 불가 |

**해석:** 현 방에서 두 테이블을 놓으려면 현 keepout의 "top"·"bottom" 띠 가운데 실제로 들어가도 되는 곳을 사용자가 정해야 한다(U2). 이 결정이 새 지도 배치의 가장 큰 변수다.

---

## 6. 위험과 완화

| ID | 위험 | 완화 |
|---|---|---|
| R1 | 정렬 지점 재관측이 카메라–면 0.45 m에서 이뤄진다. 0.42 m에서 `no_box_surface_candidate`가 대부분이었고(오늘), Astra S 공칭 최소 거리는 약 0.4 m(제조사 사양, 이 차체에서 미검증)다. 최종 접근 관측이 실패할 수 있다 | 박스 W ≤ 0.45 m, H ≤ 0.45 m로 면이 화면 안에 들어오게 한다(E-6). 첫 주행 정렬 지점 결과를 본다. 반복 실패하면 준비 단계에서 면 정렬 gap을 0.45에서 0.55 m로 바꾸는 코드 변경을 후속으로 검토한다. 배치는 관측 0.70/pre 1.25라 정렬 지점이 F + 0.615n으로 바뀌어도 수용된다 |
| R2 | 벽·가구·의자 평면이 박스보다 큰 RANSAC 평면이 되어 박스를 가림(오늘 탐색 회전 반복의 한 원인) | 정면 관측(D-3), D-4 시야 검사, E-10 현장 비우기. 물리적으로 불가하면 `depth_box_parking.yaml` `maximum_depth_m` 2.0 → 1.2 설정 변경을 후속으로 사용자 결정(U6) |
| R3 | 위치 전용 checker라 관측 지점 도착 방향이 틀어짐(9/29 yaw 61.7°) | pre를 같은 직선 0.55 m 앞에 둔다. 첫 주행 D6 진단 이벤트로 방위 오차를 기록한다. 실패하면 기존 `--search` 회전이 이어받는다 |
| R4 | 지도 좌표계가 바뀌어 registry·주석·route·TABLES·도구 상수가 모두 무효가 됨 | 새 폴더에 새 자산을 두고 F-0에서 교체 목록을 적용한다. `sync` 해시 대조. 기존 폴더는 보존. `dock_survey.py` 단독 실행 상수(MARKERS, REGISTRY_HOME)는 현 지도 값이라 새 지도에서 단독 보고서로 쓰지 않는다 |
| R5 | 최소 keepout에서 unknown→FREE 때문에 planner가 벽 틈이나 미관측 구역으로 경로를 냄 | K1-d 폐쇄 다각형, B-5 누출 검사, A-G2 운영 구역 unknown 0 |
| R6 | 매핑 중 일시 물체가 흔적으로 남거나 unknown 구멍이 생김 | E-1~E-3, 도크 시작·종료, A-G4 증거 기반 해소와 3칸 초과 시 재매핑, A-G6 |
| R7 | 박스가 init 뒤 밀리거나 설치 오차가 큼 | 무게 보강, 테이프, D-8 섭동, F-5 군집 대조(≤ 0.30 m), 옮겼으면 F-5부터 다시 |
| R8 | 도크 후진 끝의 진동과 `FAILED_TO_MAKE_PROGRESS`(105). 해석(§16.4, 미검증): 정지 뒤 TF yaw 오차 3.83°가 도크 허용 3°를 넘었고 AMCL yaw 1σ(4–5°)도 허용보다 크다 | 이 계획으로 해결되지 않는다. 도크 후진 통로를 깨끗하게 하는 것(C-c)만 간접 기여한다. 다음 도킹에서 cmd_vel·odom 속도와 goal checker 판정을 함께 기록하는 §16.5 후속을 따른다 |
| R9 | 현 keepout의 금지 구역을 풀면 실제 위험물이 드러날 수 있음 | U2에서 구역별로 사용자가 확인한다. K1-a~d는 안전 사실로 유지한다 |

---

## 7. 사용자 결정 필요 (가정하지 않음)

| ID | 결정 | 기본 제안 | 왜 필요한가 |
|---|---|---|---|
| U1 | 박스 두 개의 치수(W×D×H, cm)와 재질 | W 30–45, H 30–45, D ≥ 20, 무광 골판지, 무게 보강 | 모든 자세·시야·군집 판정의 입력 |
| U2 | 현 keepout 사각형("top", "bottom", "left_strip", "central_obstacle") 중 실제 금지(가구·출구)와 임의 제한 구분 | 가구 구역은 유지하고, 들어가도 되는 바닥은 연다 | 두 테이블 동시 배치 가능 여부를 가르는 가장 큰 변수(§5) |
| U3 | 접근 방향 요구 유지 여부(이전 요청: table_01 지도상 가로, table_02 정면) | 유지하되, 레이아웃 검사를 통과하는 방향이 없으면 검사 결과를 우선 | 방향에 따라 필요 공간이 크게 달라짐 |
| U4 | 도크 위치를 현 자리로 유지할지, C-1 기준에 맞는 새 자리로 옮길지 | C-1을 만족하면 유지 | 매핑 시작점이자 모든 경로의 시작 |
| U5 | 의자 처리 | 시연·매핑 때 운영 구역 밖으로 치움 | 의자는 동적 물체이자 평면 경쟁자 |
| U6 | D-4를 물리적으로 못 지킬 때의 대안 | 가구 재배치를 우선하고, 불가하면 `maximum_depth_m` 1.2 설정 변경(준비 단계에서 반영·검증) | 코드·설정 변경 범위 결정 |
| U7 | 새 데이터 폴더 이름 | `map_<YYYYMMDD>_<태그>` | 되돌림과 기록 추적 |

`.omc/plans/open-questions.md`는 만들지 않았다. 호출자가 산출 파일을 이 문서 하나로 지정했고 저장소 수정을 금지했기 때문이다. 위 표가 열린 질문 목록이다.

---

## 8. ADR

- **결정:**
  1. 새 지도는 기존 수동 매핑 경로(`jdamr-operator-mapping.service`)로 만든다. 도크에서 시작·종료하고, 사람·가방·박스를 뺀 방을 매핑한다.
  2. keepout은 내용 기반 최소 설계와 벽 바깥 폐쇄로 바꾸고 기존 `keepout_mask build`로 만든다.
  3. 테이블 배치는 새 지도에서 정면 직선 접근 규칙과 수치 기준으로 오프라인 계산한다(`geometry_check.py layout` 1건).
  4. 박스는 벽·도크 기준 테이프로 설치하고 init의 지도 밖 LiDAR 군집으로 등록한다.
  5. 출발은 기존 `go` 그대로 두고 새 게이트를 넣지 않는다.
- **동인:** D1 오늘 실패 원인(회전 원, 다른 평면, 지도 흔적) 제거, D2 출발 절차 경량 유지, D3 새 좌표계.
- **검토한 대안:** 1A 현 지도 배치 이식, 2B 자유 배치 뒤 LiDAR 등록만, 3B 영역 기반 keepout 유지, keepout 전부 해제, 박스 지도 포함, 새 SLAM 스택, 관측 yaw 강제 코드 변경(§2.3).
- **선택 이유:**
  - 현 지도와 현 keepout에서는 두 테이블 조합안이 없음을 분석으로 확인했다(§5).
  - 영역 기반 keepout이 오늘 세 번의 차단(테이블 회전, 도크 staging, 면 후보가 keepout 안)을 만든 공통 원인이다.
  - 계산 후 설치하는 방식은 실패를 출발 전에 드러내고, 등록은 실측(LiDAR 군집, 매 실행 면 관측)에 맡겨 줄자 오차를 흡수한다.
  - 저장소 코드 변경 없이 데이터와 데이터 폴더 도구만으로 끝난다.
- **결과:**
  - (+) 운영 공간 확대, 수치 판정된 배치, 재현 가능한 설치, 출발은 한 줄.
  - (−) 설치가 지도 작성 → 계산 → 설치 → init의 두 번 방문으로 나뉜다.
  - (−) 사용자가 금지 구역과 박스 치수를 정해야 한다(U1, U2).
  - (−) 도구 상수를 교체해야 한다(F-0).
  - (−) 최소 keepout은 누출 검사가 필요하다.
  - (−) 기존 자산은 보존용으로만 남는다.
- **후속:**
  1. 첫 주행 정렬 지점 재관측 결과로 면 정렬 gap 변경 여부를 판단한다(R1).
  2. D-4를 못 지키면 `maximum_depth_m` 변경 여부를 정한다(U6).
  3. 도크 후진 끝 105 원인을 §16.5 계측으로 가린다(R8).
  4. 실행 결과를 `20260929_PARKING_FAILURES.md` 새 절로 기록한다.
  5. layout 도구가 두 번 이상 쓰이면 저장소 `evaluation/`으로 옮길지 검토한다.

---

## 9. 범위 밖

- 저장소 코드·설정 변경: 면 정렬 gap, `maximum_depth_m`, 도크 진행 검사. 모두 후속이며 사용자 결정 뒤 준비 단계에서만 한다.
- 파이 무인 업데이트 정책과 냉각(§14.4)은 이 계획과 무관하다.
- 실차 5 cm 성공 판정은 E-11 외부 근거와 실행 로그로 별도 보고한다. 이 계획의 오프라인 판정 통과는 주행 성공 증거가 아니다.
