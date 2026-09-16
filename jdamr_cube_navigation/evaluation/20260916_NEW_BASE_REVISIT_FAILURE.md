# 2026-09-16 새 차체 재방문: 두 번째 목표 중단

## 확인된 기록

- 이전 차체의 `real_combined_obstacle_retry_20260910T122006.route.log`는
  `obstacle_base_candidate`에서 `navigate_to_pose_dynamic_obstacle_eval.xml`을
  선택했고, 20개 목표를 모두 성공했다. 이 트리는 주행 중 1Hz 재계획을 수행한다.
- 새 차체의 `new_base_revisit_20260916_b07419e_retry.route.log`도 같은 트리를
  선택했다. 첫 목표 `outbound_01m`은 성공했지만 두 번째 `outbound_06m`은
  해당 목표 수락 이후 `/odom` 누적이동 약 0.19m에서 `STATUS_ABORTED`,
  Nav2 오류 104로 끝났다.
- 실패 실행의 `launch.log`에는 RPP `collision ahead`, Collision Monitor
  `StopZone` 정지, 컨트롤러 진행 시간 초과가 반복된다. `/plan`은 갱신됐으므로
  재계획 기능이 꺼진 것은 아니다. 계획을 실제 속도 명령으로 따라가는 단계에서
  전진·선회가 막혔다.
- 이전 차체 footprint의 측면은 `±0.20m`, 새 차체는 `±0.29m`이다. 이전의
  완주 기록만으로 새 차체가 같은 장애물을 통과할 수 있다고 판단할 수 없다.
- 실패 MCAP의 두 번째 목표 구간 scan에 변경 전 StopZone의 3점 조건을 만족한
  프레임은 48개, 변경 후 5cm 외곽의 조건을 만족한 프레임도 15개였다. 따라서
  StopZone 축소만으로 중단이 해결됐다고 볼 수 없다.

## 이번 수정과 검증

- 1Hz 재계획 트리, NavFn 계획기, RPP 충돌 예측, FootprintApproach, 지도와
  Keepout을 유지했다. 주행 없는 전체 경로 계획에서 반복 실패한 실험용 Smac
  설정은 기본 프로필에 남기지 않았다.
- 첫 목표를 `(1.00,-0.04)`에서 `(1.00,-0.30)`으로 이동해 저장지도에서
  북쪽 벽 근접 경로를 줄였다. 이후 목표와 복귀 경로는 그대로다.
- 새 차체 StopZone의 footprint 밖 기하 여유를 앞·뒤·양측 5cm로 조정했다.
  이것은 제동거리나 사람 주변의 안전 여유를 보증하는 값이 아니다.
- 로컬 설정·경로 회귀 테스트가 통과했다. 파이의
  `new_base_navfn_preflight_20260916.autorun.log`에는 Nav2 lifecycle 3/3,
  주행 없는 전체 20개 목표 경로 계획 PASS, 3054 poses / 77.429m가 기록됐다.
  사전 계획 후 Nav2·기록기는 종료됐고 중단 파일을 다시 설치했다.

## 남은 판별점

현재 근접 scan의 원인이 실제 박스·벽, 차체 반사, TF/지도 오차 중 무엇인지
분류되지 않았다. 주행 없는 계획 PASS는 두 번째 목표에서 새 차체가 실제로
선회·우회할 수 있다는 증거가 아니다. 다음 실차 기록에서는 최초 중단 전후의
`/scan`, `/plan`, `/cmd_vel_nav`, `/cmd_vel`, `/collision_monitor_state`,
`/tf`, `/amcl_pose`를 동일한 시각축과 차체 좌표계로 맞춰 원인을 가린다.
인접한 사람이 있거나 실제 회전 공간이 부족한데 충돌 보호를 끄지는 않는다.
