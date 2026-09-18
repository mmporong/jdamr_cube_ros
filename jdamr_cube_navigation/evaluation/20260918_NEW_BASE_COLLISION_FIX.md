# 새 차체 충돌 감시 적용과 정지 판정 재검증

## 적용 범위

사용자가 확정한 하부 차체 제원을 유지한다. 기준은
`jdamr_cube_description/config/new_base_geometry.yaml`이며, 바퀴 포함 폭 0.540 m,
길이 0.340 m, 차축 앞쪽 길이 0.065 m이다. Nav2 footprint는 기존처럼 각 방향
0.020 m를 더한 전방 0.085 m, 후방 −0.295 m, 좌우 ±0.290 m를 사용한다.
기체 크기·속도·StopZone 여유를 이번 작업에서 임의로 축소하지 않았다.

## 수정

1. **저장지도 주행과 테이블 접근에서도 충돌 감시 프로세스 분리**
   - `onboard_nav2_core.launch.py`의 CollisionMonitor를 Nav2 container 밖의
     별도 실행 파일로 옮겼다. 자율 지도 생성에만 적용됐던 구성을 일반 주행에도 적용했다.
   - TF remap, 기존 보호 프로필 override, sim time, lifecycle 활성화와 필수
     프로세스 종료 시 전체 주행 종료를 보존했다.
   - 프로세스 분리는 계획·제어 노드와의 프로세스 내부 간섭을 줄이기 위한 수정이다.
     파이의 전체 CPU 부하나 DDS 지연까지 없어졌다는 실측 결과는 아니다.
2. **확정 제원 검증을 지도 생성·저장지도 주행에서 공유**
   - 기존 차체 검증을 `new_base_contract.py`로 옮겼다. 물리 지도 생성에서
     사용자 지정 params 파일로 작은 옛 footprint를 불러오는 경로도 거부한다.
   - 회전·전진·후진·정지 polygon의 순서와 속도 범위를 검사한다. 정지 fallback이
     앞에 와서 회전 영역을 가리거나, NaN 범위로 영역 선택이 깨지는 설정을 막는다.
   - RPP 충돌 예측과 footprint 기반 접근 감시를 끈 설정을 거부한다.
   - 추가 SSH/토픽 점검이나 대기를 넣은 것이 아니라, 시작 전 로컬 YAML 검증이다.
3. **테이블 접근의 센서 탐색 설정 누락 보완**
   - `restaurant_service.launch.py`에서 `discovery_range`를 하위 Nav2 launch로
     전달한다. 기본은 기존 실차 wrapper와 같은 `SUBNET`이며, 격리 재생에서는
     `LOCALHOST`를 명시할 수 있다.
4. **저장 데이터 비교와 실제 Nav2 실행 검증 도구 추가**
   - `replay_new_base_collision.py`: ROS 발행 없이 bag의 scan, 정적 TF,
     `/cmd_vel_smoothed` 입력과 실제 모니터 상태를 읽어 CSV/JSON으로 비교한다.
   - `smoke_new_base_collision_monitor.py`: 파이 대신 로컬 domain 199,
     `/collision_probe/*` 토픽에서 설치된 CollisionMonitor 실행 파일을 검증한다.
     베이스 드라이버·Nav2 주행 목표·실물 `/cmd_vel` 발행은 사용하지 않는다.

## 설치된 Nav2 프로그램 검증

환경: ROS 2 Jazzy, `nav2_collision_monitor 1.3.12`.
산출물: `~/jdamr_artifacts/new_base_collision_20260918/monitor_smoke_reviewed/`.
10개 조건에서 마지막 안정 구간의 모든 출력 표본을 확인했다.

| 입력 상황 | 관찰 출력 |
|---|---|
| 빈 공간, 전진 0.080 m/s | 0.080 m/s 유지 |
| 앞오른쪽 의자다리 대역, 전진 | 0.048 m/s 감속, StopZone 정지 아님 |
| 같은 의자다리, 좌회전 / 우회전 | 회전용 16각 영역 선택, 각각 0속도 |
| 같은 의자다리, 정지 입력 | 정지용 사각형 선택, StopZone 상태 해제 |
| 앞왼쪽 / 앞오른쪽 StopZone 침범 | 각각 0속도 |
| 입력 단절 전 빈 공간 | 정상 전진 |
| scan 발행 중단, 속도 입력 유지 | `invalid source`, 0속도 |
| 새 scan 공급 재개 | 정상 전진 복구 |

이는 합성 라이다·정적 모의 odom으로 수행한 모니터 통합 시험이다. 실차 제동거리,
의자 회피 경로 완주, 테이블 접근 정확도를 검증한 결과로 사용하면 안 된다.
`invalid source` 보호를 끄거나 source timeout을 늘려 통과시키지 않았다.

## 어제 기록의 기하 비교

입력: `~/jdamr_data/vslam/rgbd_20260917T172715/bag`, scan 1,017개.
산출물: `~/jdamr_artifacts/new_base_collision_20260918/replay/`.

- 현 설정에서는 정지 입력이 회전 원에 들어가지 않는다. 예전처럼 회전 범위에
  각속도 0을 포함시키는 가정과 비교해 정지 영역 선택 차이를 계산한다.
- 선택 기준은 모니터 **입력** `/cmd_vel_smoothed`다. 정지 후 출력 `/cmd_vel`을
  다시 입력으로 간주하면 원인과 결과가 뒤바뀌므로 비교 관측으로만 저장한다.
- scan별 비교는 각 프레임의 기하 판정이다. 실제 프로그램은 속도 입력 callback에서
  동작하므로 scan 수를 실제 정지 횟수로 세지 않는다. 명령 age와 유효 구간도 구분한다.
- 입력 수신 후 0.5초 이내인 221프레임에서 StopZone 조건에 들어간 프레임은
  옛 회전 영역 가정 23개, 현재 설정 6개다. 최초 입력 전 188프레임과 오래된 명령을
  유지하는 608프레임은 이 비교에서 제외했다. 실제 기록된 상태 전환은 StopZone 1개,
  `invalid source` 1개이며, 위 프레임 수와 별개다.
- 기록된 laser→base 변환은 (−0.010, 0, 0.150) m, yaw π이며 현재 제원과 일치한다.
- 의자다리 참고점 (0.29, −0.30) m는 padded footprint의 앞 경계보다 0.205 m 앞이다.
  옆 경계에서 0.010 m 차이 난다는 이유만으로 차체와 접촉했다고 판단하지 않는다.
- 이 계산은 callback 시각의 odom 보정, 전체 FootprintApproach 예측,
  실행 당시 스케줄링까지 재현하는 폐루프 시험이 아니다.

## 로컬 회귀 검증

- 차체 제원·주행 설정·지도 생성·서비스 경로·주차·탐색 정책·재생기 관련 12개
  테스트 파일: **390 passed**. MCAP 의존 패키지의 기존 deprecated 경고 1개가 남았다.
- 핵심 변경 Python 10개 파일의 `ament_flake8`, 새 실행 모듈 3개의
  `ament_pep257`, `git diff --check`: 통과.
- `colcon build --packages-select jdamr_cube_navigation --symlink-install`: 통과.
- 독립 리뷰에서 확인한 회전 도형·속도 범위·제어기 변경·관측 소스 누락·감속 및
  예측 비활성화 설정을 거부하는 회귀 사례를 추가했다. 정상 실행 params는 변경하지 않았다.
- 저장소 전체 lint의 기존 위반까지 해소한 결과는 아니다.
- 독립 검증자가 같은 390개 검사와 산출물 hash를 재확인하고 이번 코드 변경 범위를
  승인했다. 실차 충돌 방지 성능 승인과는 구분한다.
- 별도 패키지 전체 검사는 1,516 passed, 46 failed, 9 errors였다. 확인한 일부
  실패는 `evaluation/assets/nav_obstacle/asset_manifest.json` 누락으로 이번 변경
  코드에 도달하기 전에 발생했다. 전체 실패의 변경 전후 대조까지 수행한 것은 아니므로
  전체 테스트 통과나 모든 실패가 기존 문제라는 주장은 하지 않는다.

## 남은 실차 확인 범위

- 기록된 LiDAR 최소 유효 거리는 약 0.28 m다. 그보다 가까운 무효 측정은 빈 공간이
  아니다. 최종 5 cm 밀착·접촉 검출을 현재 라이다만으로 보장할 수 없다.
- 현재 CollisionMonitor와 costmap의 관측 입력은 `/scan`이다. RGB-D 카메라가
  연결됐다는 이유만으로 depth 장애물이 Nav2에 반영됐다고 주장하지 않는다.
- 3점 미만의 얇은 물체, 라이다 평면 밖 장애물, 실제 접촉과 바퀴 헛돎은 이번
  모니터 시험으로 검출을 보장하지 않는다. 충돌을 이미 감지한 범퍼가 있는 것처럼
  복구 동작을 추가하지 않았다.
- 저장 위치로의 주차와 물체 표면으로부터의 5 cm 간격은 다른 목표다. 기존 서비스
  경로의 0.05 m·3° 판정은 지도 추정 자세 기준이며 실제 오차 측정 결과가 아니다.
- 이번 작업은 로컬 구현·빌드·재생 검증이다. 파이 설치·실차 이동은 수행하지 않았다.

## 재현

```bash
cd "$HOME/jdamr_rgbd_ws/src/jdamr_cube_ros"
source /opt/ros/jazzy/setup.bash
source "$HOME/jdamr_rgbd_ws/install/setup.bash"
python3 jdamr_cube_navigation/evaluation/smoke_new_base_collision_monitor.py \
  --output "$HOME/jdamr_artifacts/new_base_collision_local_check"
```

출력 폴더가 이미 있으면 실행을 거부해 기존 시험 근거를 보존한다.
영역 선택 순서는 [Nav2 VelocityPolygon 구현](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_collision_monitor/src/velocity_polygon.cpp)을 기준으로 확인했다.
