# Claude 인계: 최신 서빙 주행을 기준으로 한 기능 고도화·포트폴리오 적용안

갱신일: 2026-09-30

- 기준 저장소: `mmporong/jdamr_cube_ros`
- 기준 브랜치: `feat/restaurant-service-destinations`
- 원격 확인 커밋: [`45d1de9`](https://github.com/mmporong/jdamr_cube_ros/commit/45d1de9ef4faaca9095ef6bfa8586b7f1fd82897) (`45d1de9ef4faaca9095ef6bfa8586b7f1fd82897`)
- 원격 브랜치, 로컬 원격 추적 브랜치, HEAD의 일치를 확인한 시점의 코드·설정을 기준으로 재작성했다.
- 2026-09-30 밤 검토 반영: `45d1de9` 이후 `02de1e7`·`2bb6bc7`·`fc73b12`의 변경(§3 끝 목록)과 새 지도 플래너 비교·위치추정 오프라인 검증(§4.1)을 덧붙이고, §2 테이블 방향을 경로 파일 값으로 바로잡았다. 위치추정 변경안은 `feat/nav-items34-localization-lattice` 브랜치에 있고 파이에 배포하지 않았다.

## 1. 이번 인계의 목적과 범위

이미 구현한 서빙 이동·박스 주차·복귀를 보존하면서 기능을 확장하고, 선정 근거와 실물 수행 결과를 포트폴리오에 남긴다. 과거 오류의 재분석·수정 목록은 이 문서의 범위에서 제외한다. 처음부터 주행을 다시 만드는 인계가 아니다.

Claude의 첫 작업은 현재 기능과 공식 자료를 비교해 적용안을 작성하는 것이다. 이 문서를 전달받았다는 이유만으로 플래너·컨트롤러·지도·주행 계약을 교체하거나 로봇을 움직이지 않는다. 후속 구현 요청에서는 선정한 부분만 바꾸고 기존 실행기를 재사용한다. 새 검수·승인 계층을 출발 절차에 추가하지 않는다.

- 최종 시스템: [bimanual-robot](https://github.com/mmporong/bimanual-robot)의 지도 기반 이동과 매니퓰레이션 연결.
- 현재 담당 범위: 목적지 이동, 표적 면 정렬, 근접 정지, 태스크 대기, 충전소 복귀.
- 현재 차체에서는 팔 동작 대신 정지 후 5초 대기를 태스크 완료 신호로 사용한다.
- 최신 실행기의 기본 흐름은 물 받는 곳을 경유한다. 짧은 테이블 왕복은 `--skip-via`로 분리한다. 예전 인계의 경유 생략 기본값을 최신 코드 설명에 적용하지 않는다.
- 관제·RViz는 주행 상태와 근거를 보여주는 수단이다. 화면 개선을 주행 기능 구현의 선행 조건으로 두지 않는다.
- 공개 성과 문구와 수치는 실측 후 작성한다. 알고리즘 도입 자체를 성과로 쓰지 않는다.

## 2. 완성할 주행 시나리오

등록한 충전소에서 출발 → 물 받는 곳 관측·면 정렬·근접 정지 → 5초 대기 → 후진 이탈 → 선택한 테이블의 관측 지점으로 이동 → 박스 앞면 관측·면 정렬 → 차체 앞면 기준 약 5cm 간격으로 정지 → 5초 대기 → 후진 이탈 → 충전소 대기점으로 이동 → 등록한 위치와 방향으로 후면 주차.

한 사이클에서는 물 받는 곳과 선택한 테이블 한 곳을 방문한다. 테이블 1과 테이블 2는 각각 별도 사이클로 평가한다. `--skip-via`는 테이블만 방문하는 비교용 단축 흐름이고, 두 테이블 연속 방문은 후속 확장이다. 충전소 근처 배치에서 초기 후면 주차부터 시작하는 흐름은 별도 초기 태스크로 둔다. 이미 도크에 배치한 경우 초기 주차를 중복 실행하지 않는다.

| 지점 | 수행할 기능 |
|---|---|
| 충전소 | 고정된 map 좌표와 방향으로 등록. 복귀 시 같은 자세로 후면 주차 |
| 물 받는 곳 | 근접 정지 후 5초 대기로 팔 태스크를 대체. 후진 이탈 뒤 호출 테이블로 이동 |
| 테이블 1 | 관측 자세 yaw 0°(동쪽). 물 받는 곳 이탈 자세(yaw 180°)에서 약 180° 방향 전환 뒤 접근. 최종 방향은 박스의 실제 앞면 법선으로 보정 |
| 테이블 2 | 관측 자세 yaw −90°(남쪽). 물 받는 곳 이탈 자세에서 약 90° 방향 전환 뒤 접근. 최종 거리와 평행 정렬은 박스 앞면 기준으로 보정 |

방향 값은 `map_20260930_manual/aligned/destinations.json`의 면 법선(물 받는 곳 0°, 테이블 1 180°, 테이블 2 90°)과 경로 파일의 관측 자세에서 계산했다.

충전소 도달과 실제 충전 접점 연결은 다른 태스크다. 이번 범위는 위치·방향을 맞춘 후면 주차이며, 충전 전류 감지나 접점 체결은 포함하지 않는다. 테이블에서 벗어난 뒤 임의의 후진·회전으로 종료하지 않고 지도에 등록한 충전소로 돌아온다.

테이블은 ID, 관측 waypoint, 접근 방향, 표적 검색 영역으로 관리한다. 최초 등록 후 같은 영역을 재사용하고 근접 관측으로 실제 면 위치를 보정한다. 테이블 추가·이동·제거는 해당 등록 항목을 갱신하는 작업으로 분리한다. 가구나 통행 구조가 바뀐 경우에만 지도·마스크 갱신 여부를 판단한다.

## 3. 현재 비교 기준선

아래는 확인된 설정이며, 실물 정확도나 태스크 완료율을 뜻하지 않는다. Claude는 작업 시작 시 현 브랜치와 변경 사항을 확인하고 다른 작업자의 편집을 보존한다.

- 차체: 직사각형 차동구동, 바퀴 포함 폭 0.540m, 길이 0.340m, 바퀴 중심 간격 0.510m.
- 주행 footprint: x = -0.295~+0.085m, y = ±0.290m. 물리 외곽에 0.020m가 포함된 비대칭 사각형이다.
- 카메라 렌즈 높이: 바닥에서 0.215m. 표적 좌표는 센서 장착 변환을 거쳐 차체 기준으로 계산한다.
- 지도 생성: Cartographer 2D. 저장 지도 주행의 위치추정은 AMCL.
- 전역 경로: NavFn, `use_astar: false`인 Dijkstra 모드, planner ID `GridBased`.
- 주행 제어: RPP `FollowPath`. 설정 파일의 일반 이동은 0.120m/s이고, 후면 도킹을 포함하는 서비스 세션에서는 `SERVICE_TRANSIT_MAX_MPS` 0.060m/s로 제한된다(`fc73b12` 기준, 이전 `45d1de9`에서는 주차 계약 0.040m/s로 묶여 있었다). 박스 주차는 0.040m/s·최저 접근 0.020m/s다. 도크는 별도 `parking_contract.yaml`을 사용한다.
- 기본 `FollowPath`는 후진 비활성이다. `ParkingReverse`가 있다는 이유로 일반 경로의 모든 후진·방향 전환을 처리한다고 가정하지 않는다.
- 표적 면 정렬은 map 기준으로 접근하고, 마지막 약 0.45m→0.05m 접근은 정렬 방향을 유지하는 odom 고정 직선 경로로 실행한다. 박스 이탈과 도크의 마지막 후진도 odom 고정 경로를 사용한다. `02de1e7`부터 최종 접근은 `GracefulParking`, 도크 후진은 `GracefulReverse`(Graceful controller)가 추종하고, `jdamr_depart.py go --rpp-final`로 RPP `Parking`·`ParkingReverse`에 되돌릴 수 있다. 실차 확인 전이다.
- 박스 완료 판정은 중앙 간격 0.05±0.01m와 계약 방향 허용 3°다. 박스 목표 위치 계약 0.01m와 도크 위치 계약 0.05m를 구분한다.
- costmap 해상도: 0.050m. Depth 박스 관측은 주차 목표 생성에 사용하며, 기본 장애물 레이어는 LiDAR 입력이다.
- 확인한 PC 설치 버전: ROS 2 Jazzy / Nav2 1.3.12. 최신 문서의 파라미터를 설치 버전에 그대로 복사하지 않는다.

`45d1de9` 이후 기준 브랜치에 들어간 변경:

- `02de1e7`: 최종 접근·도크 후진 Graceful 추종, 도크 대기점 전용 BT(`navigate_to_pose_staging.xml`, 경로 무효·10 s 경과·새 목표일 때만 재계획, 위치만 보는 goal checker), 대기점 도착 뒤 odom 기준 한 방향 Spin, 서비스 이동 0.06m/s.
- `2bb6bc7`: 출발 도구 `--rpp-final` 전환.
- `fc73b12`: 실행 이벤트에 벽시계 시각(`wall_time_s`), 주행 분석 스크립트 `scripts/map_20260930_manual/analyze_run.py`.

참조 파일은 저장소 루트 기준이다.

- `jdamr_cube_description/config/new_base_geometry.yaml`
- `jdamr_cube_navigation/config/new_base_nav2_params.yaml`
- `jdamr_cube_navigation/config/depth_box_parking.yaml`
- `jdamr_cube_navigation/config/parking_contract.yaml`
- `jdamr_cube_navigation/config/box_parking_contract.yaml`
- `jdamr_cube_navigation/jdamr_cube_navigation/{box_service,restaurant_service,parking,reverse_parking}.py`
- `scripts/map_20260930_manual/jdamr_depart.py` (최신 운영 스크립트. `scripts/phase2_20260930/`의 이전 사본과 구분)
- `jdamr_cube_navigation/evaluation/20260929_PARKING_FAILURES.md` §19.8 (현재 기능의 실행 근거만 참조)

`박스 간격 5cm`, `목표 좌표 오차 5cm`, `지도 격자 5cm`는 서로 다른 값이다. 등록 위치는 map 좌표로 표현하고, 마지막 정렬·간격은 표적 면에 대한 차체의 상대 자세로 평가한다.

2026-09-30 18:47의 `table_02_20260930_184711`은 테이블 관측 지점부터 주차·5초 대기·이탈·충전소 후면 도킹을 한 실행으로 마쳤다. 중앙 간격 5.42cm·면 방향 0.87°가 기록됐다. 이는 내부 pose와 관측면으로 계산한 값이고 외부 실측이 아니다. 물 받는 곳부터 도크까지 전체 흐름을 끊김 없이 수행한 결과와 반복 주차 정밀도는 아직 별도 측정 대상으로 둔다.

odom 고정은 단기 경로와 확인 좌표를 일관되게 유지하는 방법이다. 표적 면을 처음 관측한 뒤 이동하는 동안 표적 위치를 계속 갱신하는 시각 폐루프와 같지 않다. 표적이 움직이거나 바퀴가 미끄러질 때 생기는 상대 오차까지 자동으로 제거한다고 설명하지 않는다. [Nav2 좌표계·위치추정 원리](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/state_estimation/)

## 4. 알고리즘 적용안에서 비교할 내용

| 부분 | 후보 | 적용 효과와 판단 기준 |
|---|---|---|
| 전역 경로 | NavFn 유지 / A* 모드 / Smac2D | 경로 길이·형태·계획 시간을 비교하는 기준선. 사각 차체의 방향별 통과 검사를 추가하는 교체는 아님 |
| 자세 포함 경로 | Smac Hybrid | 방향과 곡률, Reeds–Shepp의 후진을 경로에 반영. 제자리회전은 표현하지 못한다. 9/29 지도 비교에서 Dubin(r=0.2)은 inflation 0.3일 때 home_exit·관측 지점 출발 구간을 모두 풀지 못했고, Reeds–Shepp는 풀었지만 후진 1.8–2.6 m를 써서 `FollowPath`의 후진 비활성과 맞지 않았다. Nav2 선택표상 Ackermann용이라 새 지도 비교에서 뺐다 |
| 차동구동 경로 | Smac Lattice | 사각 차체와 차동구동의 전진·후진·제자리회전 후보를 반영. 운동 후보·방향 해상도·회전 비용에 따라 우회가 커질 수 있음 |
| 주행 제어 | RPP 유지 / Graceful / MPPI / DWB | RPP는 경로 추종 기준선. 최종 접근·도크 후진은 `02de1e7`부터 Graceful. `new_base_contract.py`는 RPP와 Graceful만 허용하므로 MPPI·DWB는 계약 변경이 먼저다. 회피 품질과 파이 처리 주기를 함께 비교 |
| 경로 평활화 | 기존 구성 / Constrained Smoother | 설정에 `smoother_server`는 있지만 어떤 BT도 `SmoothPath`를 호출하지 않는다. 도입하려면 BT 변경이 먼저다. 평활화 뒤 곡률·후진 전환·차체 통과 가능성을 보존하는지 비교 |
| 장애물 표현 | LiDAR ObstacleLayer / Depth VoxelLayer 병용 | 라이다 높이에서 보이지 않는 돌출물 반영. 바닥 제거, 높이 범위, 점 수와 갱신 주기를 설계 |
| 위치추정·주차 | AMCL 유지와 조정 / 표적 상대 자세 보정 / odom+자이로 EKF | 지도에서 목적지 근처까지 이동하고 표적 면 기준으로 마지막 거리·방향을 보정. AMCL 관측 모델 변경과 EKF는 §4.1 |
| 주차 인터페이스 | 현재 실행기 유지 / Nav2 Docking Server의 표적 pose 연동 | staging 이동·표적 자세 보정·접근·완료를 표준 action으로 연결하는 후보. 태그 사용을 전제로 하지 않되 검출 입력·센서 시야·설치 버전 호환성을 확인 |

Smac Lattice는 공식 선택표에서 비원형 차동구동에 적합한 후보다. 9/29 지도 비교(`$HOME/jdamr_data/claude_phase1_20260930/planner_bench/REPORT.md`)와 현재 지도 비교(`$HOME/jdamr_data/planner_bench_20260930_newmap/REPORT.md`) 모두에서 동봉 `diff` 운동 후보(제자리 회전 포함)와 기본 rotation_penalty 5.0이 짧은 접근을 큰 우회로로 풀었다. 이는 해당 조건의 결과이며 파라미터 스윕은 하지 않았다. 둘 다 PC 정적 경로 생성 결과이며 파이 계산 시간이나 실차 수행 결과로 표시하지 않는다.

플래너 교체는 파라미터만으로 끝나지 않는다. 서비스 BT 세 개(`navigate_to_pose_parking.xml`·`_alignment.xml`·`_staging.xml`)가 planner ID `GridBased`를 고정하고, BT 다섯 개가 1 Hz RateController로 경로 계획을 돈다(staging은 그 안에서 경로 무효·10 s 경과·새 목표일 때만 다시 계획한다). Lattice는 inflation_radius가 외접 반지름(0.414 m)보다 작으면 전체 footprint 검사로 넘어가며 `computeCircumscribedCost` 오류를 낸다(현재 지도 0.30에서 79줄, 0.45에서 0줄).

### 4.1 2026-09-30 밤 검증 결과 (PC 오프라인, 실차 전)

기록: `jdamr_cube_navigation/evaluation/20260930_ITEMS34_LOCALIZATION_LATTICE.md`(`feat/nav-items34-localization-lattice` 브랜치).

| 부분 | 기존 | 변경·비교 | 수치 | 상태 |
|---|---|---|---|---|
| 전역 경로 | NavFn | Smac2D, Lattice 0.5 m·1 m | 새 지도 6구간, 길이/직선: NavFn 1.00–1.05, Lattice 0.5 m 2.10–3.33, 1 m 1.00–1.18(한 구간 6.74) | NavFn 유지 |
| inflation | 0.30 | 0.45 | NavFn 경로 bit 단위 동일, 경로 셀 비용 0. keepout은 팽창되지 않음 | 바꾸지 않음 |
| 최종 접근 | 플래너 경로 + RPP | odom 고정 직선 | 면 대비 방향 11–23°(104 실패) → 0.87°(성공), 9/30 실차 내부 추정 | 적용됨 |
| 최종 접근 추종기 | RPP `Parking` | Graceful | — | 적용, 실차 전 |
| 도크 대기점 이동 | 1 Hz 재계획 | 조건부 재계획 + 한 방향 Spin | 기존 구간 118.4 s, 그 사이 Nav2 journal의 새 경로 전달(`Passing new path`) 113줄(18:48:54–18:51:01) | 적용, 실차 전 |
| AMCL 관측 모델 | `likelihood_field` | `likelihood_field_prob` + beamskip | 재생 도구 동작 확인, 9/29 bag 정지 구간 2개라 판정 불가 | 브랜치, 다음 주행 bag으로 판정 |
| odom 방향 | 바퀴 odom | 바퀴 + 자이로 EKF(바이어스 제거) | 제자리 회전 스캔 기준 오차 중앙값 odom 0.50° → EKF 0.10°(4건). 자이로 z 부호 반대, 바이어스 −0.0018~+0.0011 rad/s | 브랜치, 미연결 |

마스킹은 차체 중심뿐 아니라 외곽까지 고려해 계획에 반영되는지 확인한다. 일반 장애물 inflation이 KeepoutFilter에도 자동 적용된다고 가정하지 않는다. 전역 경로가 통과 가능해도 선택한 컨트롤러가 중간 회전·후진 구간을 수행할 수 있는지는 별도 항목이다.

최신 코드에서 마지막 접근·이탈·도크 후진은 전역 플래너와 분리된 직선 경로다. 따라서 Smac 교체의 주요 비교 구간은 목적지까지의 이동과 면 정렬 접근이며, 현재 odom 직선 주차의 5cm 정확도 개선을 플래너 교체 효과로 기대하지 않는다.

적용안의 우선순위는 다음과 같다.

1. 현재 구성으로 물 받는 곳·선택 테이블·충전소의 한 사이클을 완료하고 외부 실측과 동기화 영상을 확보한다. 최근 완료한 table_02 구간을 기준선으로 활용한다.
2. 회전한 박스와 서로 다른 테이블 방향에서 면 정렬·간격을 평가한다. 관측 가능한 구간에서는 재관측과 상대 자세 갱신을 후보로 삼고, 근거리 관측 한계에서는 odom 이행과 외부 실측을 구분한다. 카메라가 볼 수 없는 마지막 구간을 시각 폐루프로 표시하지 않는다.
3. 테이블 등록·호출과 팔 작업 완료 신호를 기존 실행기의 상태 전환에 연결하는 적용안을 만든다. 5초 대기는 시험용 기본값으로 두고, 최종 양팔 시스템은 팔 태스크 완료 이벤트를 사용한다.
4. 전역 경로는 현재 지도 비교에서 NavFn을 유지하기로 했다(§4.1). Lattice는 운동 후보·회전 비용을 이 차체에 맞춰 다시 만들 때만 다시 비교한다. MPPI는 동적 회피·복합 전진/후진의 필요성이 확인됐을 때 후보로 두고, 계약(`new_base_contract.py`) 변경을 함께 계획한다.
5. 위치추정은 다음 주행의 PC bag으로 AMCL 관측 모델 A/B를 재생 비교하고, EKF는 `/odom`·`/imu/data_raw`가 든 기록을 확보한 뒤 표본을 늘린다.

Nav2 Docking Server는 고정 위치로의 staging 이동과 표적 자세 보정·접근을 나누고 기본 local frame으로 odom을 사용하는 구조여서 현재 설계와 비교할 가치가 있다. 다만 설정에 `docking_server` 항목이 있다는 것과 현재 `DockRobot` action으로 실행한다는 것은 다르다. 현재는 자체 실행기와 RPP FollowPath가 수행한다. RGB-D 면 추정을 도킹 검출 pose로 연결하는 방식과 일반 서비스 위치의 완료 조건을 먼저 검토한다. 후면 주차에서 전방 카메라가 도크 표적을 계속 볼 수 있다고 가정하지 않는다. 충전 전류나 접점 판정은 위치 주차와 분리한다. [Jazzy Docking Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_docking_server/)

Jazzy의 외부 검출 인터페이스는 `detected_dock_pose`의 `PoseStamped`다. 박스 면 검출을 이 형식으로 변환할 수 있다면 태그나 새 dock plugin이 무조건 필요한 것은 아니다. 다만 외부 검출을 사용하는 접근은 최신 검출 pose를 계속 요구한다. 근거리·후면에서 관측이 끊기는 경우 현재의 odom 직선 단계와 live detection 접근을 구분해 설계한다. Jazzy 1.3.12의 `dock_backwards`와 Rolling의 `dock_direction`·`rotate_to_dock`을 혼용하지 않는다. [Jazzy 도킹 적용 절차](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_docking/)

RGB-D 3D 지도는 RTAB-Map 등으로 분리해 다루며 테이블 주차의 선행 조건으로 만들지 않는다. 현재 파이 제어·PC 기록 구조와 완료한 주차 구간을 유지한 채, 비교할 모듈 하나만 선정한다.

## 5. 주행 기능별 평가

| 기능 | 실행 장면 | 남길 측정값 |
|---|---|---|
| 목적지 이동 | 충전소에서 물 받는 곳과 선택 테이블로 이동 | 구간별 경로 길이, 이동 시간, 정지·방향 전환 횟수 |
| 면 정렬·근접 정지 | 정면 박스와 회전한 박스에 접근 | 실측 앞면 간격, 면과의 각도 오차, 관측 가능 거리 |
| 후진 복귀 | 테이블 이탈 후 충전소 후면 주차 | 충전소 위치·방향 오차, 전체 사이클 완료 여부 |
| 주변 물체 회피 | 의자 다리·박스가 있는 통로 | 회피 경로, 차체 전체의 최소 여유, 태스크 완료 여부 |
| 자원 사용 | 같은 주행에 동일 센서·기록 구성 사용 | 계획 시간, 제어 주기, 파이 CPU·메모리, 기록으로 인한 입력 지연 |
| 태스크 연결 | 물 받는 곳·테이블·도크 한 실행 | 단계별 완료 신호, 전체 사이클 시간, 중복 없이 이어진 구간 |

먼저 저장 지도로 경로만 비교하고, 선정한 구성의 실물 동작을 확인한다. 같은 지도·마스크·시작 자세·목표·속도에서 한 번에 비교 변수 하나를 바꾼다. 실차 반복 횟수는 가용 시간에 맞추고 모든 시도를 분모에 남긴다. 반복 주행을 아직 하지 않았다면 반복 정밀도나 성공률을 채워 넣지 않는다. 준비 단계에서 확보한 검증을 매 출발 때 반복하지 않는다.

목표 좌표와 내부 추정 오차는 로그로, 최종 간격과 평행 정렬은 줄자·기준선·영상 같은 외부 근거로 구분해 기록한다. 5초 대기는 서비스 상태 전환이며, 목표 도착이나 박스 검출만으로 완료를 선언하지 않는다.

## 6. 포트폴리오 산출물과 데이터

프로젝트 제목 후보: `지도 기반 서빙 주행과 센서 표적 면 정렬·정밀주차·후진 복귀`

사례는 차체의 비대칭 footprint와 관측 범위를 설명한 뒤, 플래너·컨트롤러 선정 근거와 주행 결과를 이어서 보여준다. 최종 양팔 시스템에서는 주차 완료 신호를 팔 태스크로 전달하고, 팔 태스크 완료 신호를 받은 뒤 이탈·복귀하는 연결을 제시한다. 현재의 5초 대기를 실제 양팔 작업 수행으로 표현하지 않는다.

- 대표 영상: 출발, 물 받는 곳 정지, 테이블 이동, 면 정렬, 근접 정지, 대기, 충전소 후면 주차가 이어지는 한 사이클. table_02 구간 영상은 부분 수행 근거로 구분한다.
- 동기화 화면: 실물 영상과 RViz. 지도·keepout·차체 외곽·계획 경로·목표 자세·표적 면 법선을 표시한다.
- 결과 표: 같은 조건의 알고리즘 비교와 외부 실측 주차 오차. 시뮬레이션·오프라인 계획·실차 측정을 구분한다.
- 재현 자료: 설정 파일, 지도·마스크 식별값, 실행 명령, 코드 commit, 결과 로그, 영상 시간축.
- 공개 설명: 선택한 운동 모델, 차체 좌표 변환, 표적 면 추정, 단계별 제어 전환과 측정 결과를 쓴다.

최신 코드에는 PC 기록기가 이미 있다. `scripts/map_20260930_manual/jdamr_depart.py`의 `display-start/stop`이 기록기와 연결되고 `record-start/stop`도 제공한다. 현재 MCAP 기록 토픽은 `/tf`, `/tf_static`, `/scan`, `/plan`, `/amcl_pose`다. `/tf`로 전달되는 odom→base와 map→odom을 구분해 궤적을 복원할 수 있지만 원본 `/odom` 메시지나 `/cmd_vel`이 기록된 것으로 표시하지 않는다. 표시 중계는 명령 토픽을 전달하지 않는다. 주행 결과·박스면·단계 상태는 기존 실행 이벤트 로그를 함께 사용한다.

실행별 manifest와 실측 표·미디어 색인을 추가하는 것을 다음 데이터 관리안으로 둔다. 기존 `$HOME/jdamr_data/map_20260930_manual/aligned/runs/` 로그와 PC MCAP을 이동·복제하지 않고 실행 ID로 연결한다. 다른 실행의 속도 변경 전후 자료를 섞지 않는다. 필요하다면 `$HOME/jdamr_data/service_<실행ID>/manifest.json`에서 원본 경로를 참조한다. manifest에는 코드 버전, 지도·마스크 식별값, 사용 설정, 시작·목표 자세, 테이블 ID, 알고리즘, 실행 결과와 기록 시간 범위를 넣는다.

제어 품질까지 비교하려면 원본 `/odom`, 단계별 속도 명령, 표적 관측 입력, 시간 정보를 더 확보해야 한다. 이는 아직 연결하지 않은 기록 확장 후보이며 현재 bag에 있다는 뜻이 아니다. 파이에 대용량 녹화를 추가하는 대신 PC 기록 구성의 추가 비용과 시간 동기화를 먼저 검토한다. 카메라 원본은 표적 관측 구간·키프레임부터 확보하고 전체 RGB-D 녹화는 처리량에 맞춰 선택한다.

내부 위치추정 오차를 물리 실측으로 표시하지 않는다. 시뮬레이션 재현은 실차 주행과 분리하고, 실측 센서·환경 검증 없이 디지털 트윈으로 명명하지 않는다. 이번 인계에서 사이트 배포나 데이터 삭제는 수행하지 않는다.

## 7. Claude가 먼저 반환할 내용

1. `45d1de9` 이후 갱신 여부와 현재 map 이동·odom 주차 기준선. 이미 연결한 기능을 새 구현 목록에 넣지 않는다.
2. 현재 한 사이클 외부 실측, 표적 상대 자세 갱신, 양팔 상태 연결, 플래너 비교 중 먼저 할 항목과 선정 근거.
3. Docking Server·Lattice·MPPI의 예상 효과와 부작용, 설치 버전·차체·센서 조건, 유지할 기존 기능과 변경할 모듈.
4. 현재 PC 기록으로 산출 가능한 지표와 추가 입력이 필요한 지표, 실물 영상·실측·알고리즘 비교를 포트폴리오로 연결하는 구성.

과거 오류 목록 대신 위 기능·비교·산출물을 중심으로 답한다. 실제 적용 전에는 예상 효과로, 실물 확인 후에는 측정 결과로 표현한다.

## 공식 참고 자료

- [Nav2 플래너 선택 기준](https://docs.nav2.org/jazzy/configuration_and_development/tuning_guide/)
- [Smac Hybrid](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)
- [Smac Lattice](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)
- [Jazzy RPP](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
- [Jazzy MPPI](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)
- [map·odom·base 좌표계와 위치추정](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/state_estimation/)
- [Jazzy Docking Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_docking_server/)
- [Jazzy 도킹 적용 절차·검출 pose 인터페이스](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_docking/)
- [KeepoutFilter와 inflation의 구분](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_filters/keepout_filter/)
- [Depth VoxelLayer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/)
- [RTAB-Map](https://github.com/introlab/rtabmap/blob/master/doxygen/mainpage.md)

Rolling 참고 자료는 동작 원리 설명용이다. 실제 적용 파라미터·기능은 설치된 Jazzy 버전에서 확인한다.
