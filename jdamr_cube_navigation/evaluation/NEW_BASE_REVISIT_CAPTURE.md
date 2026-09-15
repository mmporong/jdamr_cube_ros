# 새 차체 기존 코스 재방문: 지도·주행 증거 계약

이 실행은 미지 공간 탐색이 아니다. 새 차체가 기존 지도와 Keepout을 참조해 기존
waypoint를 Nav2로 다시 계획·주행하고, 기록된 센서 데이터를 격리 도메인의
Cartographer로 재생해 새 2D 지도를 만드는 실험이다. 실차에서 Cartographer와
AMCL은 동시에 `map→odom`을 발행하지 않는다.

## 출발 전

- 기존 지도·마스크 YAML/PGM 네 파일은 `onboard_nav2_core.launch.py`의 고정
  SHA-256과 같아야 한다. 이 지도·마스크는 **재방문 안내용**이며 새 지도에 바로
  이식하지 않는다.
- 새 차체 URDF, Nav2 footprint, StopZone, SlowdownZone, `/scan` 충돌 소스가
  새 차체 측정값과 맞아야 한다. 라이다 높이·방향과 실제 TF를 확인한다.
- 로봇을 기존 경로의 `home` 출발 위치에 두고 차체 앞을 복도 진행 방향으로
  향하게 한다. 실차 베이스가 정지했고, Cartographer·웹 조종기·다른 Nav2가
  꺼져 있어야 한다. 정지·배터리·노드·토픽 확인에 실패하면 출발하지 않는다.
- 실행기는 Nav2 lifecycle, AMCL, Keepout, Collision Monitor,
  `ComputePathThroughPoses` 계획과 경로 길이 상한을 검증한 후에만 실제 goal을
  전송한다. `/cmd_vel`은 Collision Monitor 한 노드만 발행해야 한다.

## 파이에서 남는 원본

실행 ID `RUN_ID`마다 `$HOME/jdamr_artifacts/RUN_ID/`에 CRC·인덱스가 포함된
MCAP bag, 같은 상위 디렉터리에 `RUN_ID.route.log`, `RUN_ID.launch.log`,
`RUN_ID.autorun.log`, `RUN_ID.inputs.sha256`, `RUN_ID.provenance.txt`,
`RUN_ID.per_process.tsv`가 남는다. 로그에는 시작·정지·목표 상태, 오류,
출발·종료 UTC 시각이 기록된다. 원본은 잘라 쓰거나 이름을 바꾸기 전에 그대로
보존한다.

주요 관측 토픽은 `/scan`(2D 라이다), `/odom`(바퀴 위치·속도), `/tf`·`/tf_static`
(지도→오도메트리→차체→센서), `/amcl_pose`(기존 지도 위치추정), `/plan`
(Nav2 계획), `/cmd_vel_nav`·`/cmd_vel`(계획 명령·최종 감독 출력),
`/collision_monitor_state`(정지·감속), `/battery_state`,
`/navigate_to_pose/_action/status`(목표 결과), `/imu/data_raw`다.
bag에 `/map`이 없더라도 지도 생성이 빠진 것은 아니다. 기존 지도는 안내용이고,
새 지도는 **오프라인 Cartographer 결과 bag의 `/map`**에서 관찰한다.

## 지도 생성과 품질 확인

원본 bag을 노트북으로 회수한 뒤 `$HOME/jdamr_cube_ws/src/jdamr_cube_ros`에서
다음 순서로 처리한다. 오프라인 재생기는 물리 ROS 도메인 12를 거부하고,
기록된 저장 지도·이동 명령·AMCL `map→odom`을 재생에서 제외한다.

1. `jdamr_cube_navigation/scripts/offline_slam_replay.sh --bag <원본-bag-디렉터리> --backend cartographer --out <새-결과-디렉터리>`로 전체 bag을 재생한다.
2. 최종 최적화된 `.pbstream`과 여기서 변환한 지도 YAML/PGM이 생성됐는지
   확인한다. 최종 지도는 재생 중 마지막 `/map` 스냅샷과 구분한다.
3. 실제 주행 거리, 왕복 종료 위치, TF·scan 시간 단절, 지도 폭·길이·해상도,
   알려진 셀과 미지 셀, 벽의 연결·이중화·왜곡 여부를 확인한다. 이상한 지도는
   이름만 `final`로 바꿔 승격하지 않고 원본 bag에서 원인을 분석한다.
4. 새 지도와 기존 지도의 벽·계단 특징을 정합해 **같은 물리적 금지구역**을 새
   `map` 좌표에 다시 그린다. 새 Keepout 마스크의 크기·origin·해상도·연결성,
   새 차체 footprint 여유를 검증한다.

## 영상·포트폴리오 근거

- `jdamr_cube_navigation/evaluation/corridor_run_media.py --run-dir <원본-bag-디렉터리> --output-dir <새-미디어-디렉터리>`는
  실차 bag·경로 이벤트를 바탕으로 궤적·정지·경로 이탈 지표, PNG·MP4와
  입력 해시가 담긴 미디어 manifest를 만든다.
- `jdamr_cube_navigation/evaluation/render_new_base_mapping.py --map-mcap <Cartographer-결과-MCAP> --source-bag <원본-bag-디렉터리> --run-log <오프라인-재생-로그> --output-dir <새-지도-미디어-디렉터리>`는
  결과 bag의 실제 `/map` 변화로 지도 성장 MP4·마지막 기록 지도 PNG와
  SHA-256·프레임 시간 provenance JSON을 만든다. **실물 카메라 영상이나 최종
  최적화 지도라고 표시하지 않는다.**
- 실물 주행 모습도 필요하면 복도 측면의 고정 카메라/휴대폰으로 별도 촬영한다.
  시작·정지 장면과 벽·금지구역 주변을 프레임에 넣고, 원본 영상 파일과
  `RUN_ID.provenance.txt`의 UTC 시각을 함께 보존한다. 외부 영상이 없다면
  ROS 재생 영상만 실물 촬영으로 표기하지 않는다.
- 포트폴리오에는 최초 지도, 기존 Keepout, 새 footprint·라이다 TF,
  재계획·정지 이벤트, 최종 최적화 지도와 새 마스크를 각각 실제 근거의
  SHA-256으로 연결한다. 사람·박스 자동 분류 결과가 없으면 그 분류를 주장하지
  않는다.
