# Claude 인계: 서빙 주행·정밀주차 고도화와 포트폴리오 실험안

작성일: 2026-09-30

## 1. 이번 인계의 목적과 범위

양팔 서빙 로봇의 이동 기능을 현재 차체로 구현하고, 알고리즘 선정 근거와 실물 수행 결과를 포트폴리오에 남긴다. 과거 오류의 재분석·수정 목록은 이 문서의 범위에서 제외한다.

Claude의 첫 작업은 현재 기능과 공식 자료를 비교해 적용안을 작성하는 것이다. 이 문서를 전달받았다는 이유만으로 플래너·컨트롤러·지도·주행 계약을 교체하거나 로봇을 움직이지 않는다. 후속 구현 요청에서는 선정한 부분만 바꾸고 기존 실행기를 재사용한다. 새 검수·승인 계층을 출발 절차에 추가하지 않는다.

- 최종 시스템: [bimanual-robot](https://github.com/mmporong/bimanual-robot)의 지도 기반 이동과 매니퓰레이션 연결.
- 현재 담당 범위: 목적지 이동, 표적 면 정렬, 근접 정지, 태스크 대기, 충전소 복귀.
- 현재 차체에서는 팔 동작 대신 정지 후 5초 대기를 태스크 완료 신호로 사용한다.
- 물을 따르는 곳의 경유는 기본 시나리오에서 생략한다. 관련 기능이 있어도 선택 경유지로 분리한다.
- 관제·RViz는 주행 상태와 근거를 보여주는 수단이다. 화면 개선을 주행 기능 구현의 선행 조건으로 두지 않는다.
- 공개 성과 문구와 수치는 실측 후 작성한다. 알고리즘 도입 자체를 성과로 쓰지 않는다.

## 2. 완성할 주행 시나리오

충전소 근처 배치 → 등록한 충전소 자세로 주차 → 5초 대기 → 선택한 테이블의 관측 지점으로 이동 → 박스 앞면 관측 → 면과 평행하게 정렬 → 차체 앞면 기준 약 5cm 간격으로 정지 → 5초 대기 → 후진 이탈 → 충전소로 복귀 → 등록한 위치와 방향으로 후면 주차.

한 사이클에서는 선택한 테이블 한 곳을 방문한다. 테이블 1과 테이블 2는 각각 별도 사이클로 평가하고, 두 곳 연속 방문은 후속 확장으로 둔다.

| 지점 | 수행할 기능 |
|---|---|
| 충전소 | 고정된 map 좌표와 방향으로 등록. 복귀 시 같은 자세로 후면 주차 |
| 테이블 1 | 지도에 지정한 접근 방향으로 약 90° 방향 전환. 최종 방향은 박스의 실제 앞면 법선으로 보정 |
| 테이블 2 | 정면 접근. 최종 거리와 평행 정렬은 박스 앞면 기준으로 보정 |

충전소 도달과 실제 충전 접점 연결은 다른 태스크다. 이번 범위는 위치·방향을 맞춘 후면 주차이며, 충전 전류 감지나 접점 체결은 포함하지 않는다. 테이블에서 벗어난 뒤 임의의 후진·회전으로 종료하지 않고 지도에 등록한 충전소로 돌아온다.

테이블은 ID, 관측 waypoint, 접근 방향, 표적 검색 영역으로 관리한다. 최초 등록 후 같은 영역을 재사용하고 근접 관측으로 실제 면 위치를 보정한다. 테이블 추가·이동·제거는 해당 등록 항목을 갱신하는 작업으로 분리한다. 가구나 통행 구조가 바뀐 경우에만 지도·마스크 갱신 여부를 판단한다.

## 3. 현재 비교 기준선

아래는 확인된 설정이며, 실물 정확도나 태스크 완료율을 뜻하지 않는다. Claude는 작업 시작 시 현 브랜치와 변경 사항을 확인하고 다른 작업자의 편집을 보존한다.

- 차체: 직사각형 차동구동, 바퀴 포함 폭 0.540m, 길이 0.340m, 바퀴 중심 간격 0.510m.
- 주행 footprint: x = -0.295~+0.085m, y = ±0.290m. 물리 외곽에 0.020m가 포함된 비대칭 사각형이다.
- 카메라 렌즈 높이: 바닥에서 0.215m. 표적 좌표는 센서 장착 변환을 거쳐 차체 기준으로 계산한다.
- 지도 생성: Cartographer 2D. 저장 지도 주행의 위치추정은 AMCL.
- 전역 경로: NavFn, `use_astar: false`인 Dijkstra 모드, planner ID `GridBased`.
- 주행 제어: RPP `FollowPath`. 주차는 `Parking`, 후진 주차는 별도 `ParkingReverse`.
- 기본 `FollowPath`는 후진 비활성이다. `ParkingReverse`가 있다는 이유로 일반 경로의 모든 후진·방향 전환을 처리한다고 가정하지 않는다.
- costmap 해상도: 0.050m. Depth 박스 관측은 주차 목표 생성에 사용하며, 기본 장애물 레이어는 LiDAR 입력이다.
- 확인한 PC 설치 버전: ROS 2 Jazzy / Nav2 1.3.12. 최신 문서의 파라미터를 설치 버전에 그대로 복사하지 않는다.

참조 파일은 저장소 루트 기준이다.

- `jdamr_cube_description/config/new_base_geometry.yaml`
- `jdamr_cube_navigation/config/new_base_nav2_params.yaml`
- `jdamr_cube_navigation/config/depth_box_parking.yaml`
- `jdamr_cube_navigation/config/parking_contract.yaml`
- `jdamr_cube_navigation/config/box_parking_contract.yaml`
- `jdamr_cube_navigation/jdamr_cube_navigation/{box_service,restaurant_service,parking,reverse_parking}.py`

`박스 간격 5cm`, `목표 좌표 오차 5cm`, `지도 격자 5cm`는 서로 다른 값이다. 등록 위치는 map 좌표로 표현하고, 마지막 정렬·간격은 표적 면에 대한 차체의 상대 자세로 평가한다.

## 4. 알고리즘 적용안에서 비교할 내용

| 부분 | 후보 | 적용 효과와 판단 기준 |
|---|---|---|
| 전역 경로 | NavFn 유지 / A* 모드 / Smac2D | 경로 길이·형태·계획 시간을 비교하는 기준선. 사각 차체의 방향별 통과 검사를 추가하는 교체는 아님 |
| 자세 포함 경로 | Smac Hybrid | 방향과 곡률, Reeds–Shepp의 후진을 경로에 반영. 제자리회전은 표현하지 못하며 기본 RPP 후진 설정과 연결을 맞춰야 함 |
| 차동구동 경로 | Smac Lattice | 사각 차체와 차동구동의 전진·후진·제자리회전 후보를 반영. 운동 후보·방향 해상도·회전 비용에 따라 우회가 커질 수 있음 |
| 주행 제어 | RPP 유지 / MPPI / DWB | RPP는 경로 추종 기준선. MPPI·DWB는 주변 비용을 고려한 이동 궤적 선택 후보. 회피 품질과 파이 처리 주기를 함께 비교 |
| 경로 평활화 | 기존 구성 / Constrained Smoother | 실제 BT 호출 여부를 확인. 평활화 뒤 곡률·후진 전환·차체 통과 가능성을 보존하는지 비교 |
| 장애물 표현 | LiDAR ObstacleLayer / Depth VoxelLayer 병용 | 라이다 높이에서 보이지 않는 돌출물 반영. 바닥 제거, 높이 범위, 점 수와 갱신 주기를 설계 |
| 위치추정·주차 | AMCL 유지와 조정 / 표적 상대 자세 보정 | 지도에서 목적지 근처까지 이동하고 표적 면 기준으로 마지막 거리·방향을 보정. 유효한 IMU가 있으면 odometry 융합도 별도 후보 |

Smac Lattice는 공식 선택표에서 비원형 차동구동에 적합한 후보다. 다만 보존된 PC 정적 지도 비교에서는 동봉 16방향·0.5m 회전반경 운동 후보와 당시 설정이 짧은 접근을 큰 우회로로 풀었다. 이는 해당 조건의 결과이며 Lattice 전체의 한계로 일반화하지 않는다. Hybrid 전진 전용과 Reeds–Shepp도 따로 비교한다. 어느 후보든 즉시 전면 교체할 근거로 사용하지 않는다.

이 비교의 보존 자료는 `$HOME/jdamr_data/claude_phase1_20260930/planner_bench/REPORT.md`, `compact_tables.md`, `results.json`이다. PC 정적 경로 생성 결과이며 파이 계산 시간이나 실차 수행 결과로 표시하지 않는다.

마스킹은 차체 중심뿐 아니라 외곽까지 고려해 계획에 반영되는지 확인한다. 일반 장애물 inflation이 KeepoutFilter에도 자동 적용된다고 가정하지 않는다. 전역 경로가 통과 가능해도 선택한 컨트롤러가 중간 회전·후진 구간을 수행할 수 있는지는 별도 항목이다.

우선 비교안은 `NavFn + RPP`를 기준선으로 두고 `차체에 맞춘 Lattice + 실행 가능한 제어 구성`을 후보로 삼는다. MPPI는 동적 회피나 복합 전진·후진의 개선 필요성이 확인됐을 때 비교한다. RGB-D 3D 지도는 RTAB-Map 등으로 분리해 다루며, 테이블 주차의 선행 조건으로 만들지 않는다.

## 5. 주행 기능별 평가

| 기능 | 실행 장면 | 남길 측정값 |
|---|---|---|
| 목적지 이동 | 충전소에서 테이블 1·2 접근 | 경로 길이, 이동 시간, 정지·방향 전환 횟수 |
| 면 정렬·근접 정지 | 정면 박스와 회전한 박스에 접근 | 실측 앞면 간격, 면과의 각도 오차, 관측 가능 거리 |
| 후진 복귀 | 테이블 이탈 후 충전소 후면 주차 | 충전소 위치·방향 오차, 전체 사이클 완료 여부 |
| 주변 물체 회피 | 의자 다리·박스가 있는 통로 | 회피 경로, 차체 전체의 최소 여유, 태스크 완료 여부 |
| 자원 사용 | 같은 주행에 동일 센서·기록 구성 사용 | 계획 시간, 제어 주기, 파이 CPU·메모리, 기록으로 인한 입력 지연 |

먼저 저장 지도로 경로만 비교하고, 선정한 구성의 실물 동작을 확인한다. 같은 지도·마스크·시작 자세·목표·속도에서 한 번에 비교 변수 하나를 바꾼다. 실차 반복 횟수는 가용 시간에 맞추고 모든 시도를 분모에 남긴다. 반복 주행을 아직 하지 않았다면 반복 정밀도나 성공률을 채워 넣지 않는다. 준비 단계에서 확보한 검증을 매 출발 때 반복하지 않는다.

목표 좌표와 내부 추정 오차는 로그로, 최종 간격과 평행 정렬은 줄자·기준선·영상 같은 외부 근거로 구분해 기록한다. 5초 대기는 서비스 상태 전환이며, 목표 도착이나 박스 검출만으로 완료를 선언하지 않는다.

## 6. 포트폴리오 산출물과 데이터

프로젝트 제목 후보: `지도 기반 서빙 주행과 RGB-D 표적 면 정렬·후진 복귀`

사례는 차체의 비대칭 footprint와 관측 범위를 설명한 뒤, 플래너·컨트롤러 선정 근거와 주행 결과를 이어서 보여준다. 최종 양팔 시스템에서는 주차 완료 신호를 팔 태스크로 전달하고, 팔 태스크 완료 신호를 받은 뒤 이탈·복귀하는 연결을 제시한다. 현재의 5초 대기를 실제 양팔 작업 수행으로 표현하지 않는다.

- 대표 영상: 출발, 테이블 이동, 면 정렬, 근접 정지, 대기, 충전소 후면 주차가 이어지는 한 사이클.
- 동기화 화면: 실물 영상과 RViz. 지도·keepout·차체 외곽·계획 경로·목표 자세·표적 면 법선을 표시한다.
- 결과 표: 같은 조건의 알고리즘 비교와 외부 실측 주차 오차. 시뮬레이션·오프라인 계획·실차 측정을 구분한다.
- 재현 자료: 설정 파일, 지도·마스크 식별값, 실행 명령, 코드 commit, 결과 로그, 영상 시간축.
- 공개 설명: 선택한 운동 모델, 차체 좌표 변환, 표적 면 추정, 단계별 제어 전환과 측정 결과를 쓴다.

실행별로 `$HOME/jdamr_data/service_<실행ID>/`에 `manifest.json`, 지도·마스크, 사용 설정, 상태 이벤트, 주행 기록, 실측 표, 미디어를 연결한다. manifest에는 코드 버전, 시작·목표 자세, 테이블 ID, 적용 알고리즘, 실행 결과와 파일 경로를 넣는다. 필수 주행 기록은 `/scan`, `/odom`, `/tf`, `/tf_static`, `/amcl_pose`, 단계별 속도 명령과 실제 계획 경로, 박스 관측 결과다. 이름이 다른 토픽은 현 실행 구성의 이름을 manifest에 적는다. 카메라 원본은 표적 관측 구간·키프레임부터 확보하고 전체 RGB-D 녹화는 처리량에 맞춰 선택한다.

내부 위치추정 오차를 물리 실측으로 표시하지 않는다. 시뮬레이션 재현은 실차 주행과 분리하고, 실측 센서·환경 검증 없이 디지털 트윈으로 명명하지 않는다. 이번 인계에서 사이트 배포나 데이터 삭제는 수행하지 않는다.

## 7. Claude가 먼저 반환할 내용

1. 기준선과 후보 조합, 설치 버전에서의 호환성.
2. 기능별 예상 개선과 부작용, 차체·센서 조건에 따른 선정 근거.
3. 한 번에 바꿀 모듈과 기존 기능을 유지할 경계. 변경안은 실제 적용과 구분한다.
4. 비교 시나리오와 기록 항목, 실물 수행 결과를 포트폴리오로 연결하는 방식.

과거 오류 목록 대신 위 기능·비교·산출물을 중심으로 답한다. 실제 적용 전에는 예상 효과로, 실물 확인 후에는 측정 결과로 표현한다.

## 공식 참고 자료

- [Nav2 플래너 선택 기준](https://docs.nav2.org/jazzy/configuration_and_development/tuning_guide/)
- [Smac Hybrid](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)
- [Smac Lattice](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)
- [Jazzy RPP](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)
- [Jazzy MPPI](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/)
- [KeepoutFilter와 inflation의 구분](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_filters/keepout_filter/)
- [Depth VoxelLayer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/)
- [RTAB-Map](https://github.com/introlab/rtabmap/blob/master/doxygen/mainpage.md)

Rolling 참고 자료는 동작 원리 설명용이다. 실제 적용 파라미터·기능은 설치된 Jazzy 버전에서 확인한다.
