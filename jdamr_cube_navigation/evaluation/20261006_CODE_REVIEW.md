# 코드 리뷰와 수정 (2026-10-06, 실차 없음)

- **범위:** `git diff 87652f2..ba0ef54`의 코드 변경 전체(문서·rviz 제외)
  - 탈출·되짚기: `restaurant_service.py`, `corridor_route.py`, `reverse_parking.py`
  - 박스 최종 접근: `box_service.py`
  - 계약과 설정: `new_base_contract.py`, `docking_stop_profile.py`, `config/new_base_nav2_params.yaml`, `behavior_trees/*.xml`
  - 라이다 자기 필터: `jdamr_cube_bringup`의 `scan_self_filter.py`, `real_bringup.launch.py`
  - PC 도구: `scripts/map_20260930_manual`의 `jdamr_depart.py`, `pi_load_profile.py`, `analyze_load.py`
- **방법:** 작성자와 다른 리뷰어 3개가 영역을 나눠 읽기 전용으로 검토했다(oh-my-claudecode code-reviewer). 지적마다 코드를 읽거나 재현해서 확인한 것만 고쳤다. 수정한 뒤에는 다시 다른 검증자 2개가 해결 여부를 판정했다(§4).
- **실차:** 파이가 꺼져 있어 실차 확인은 없다. 모든 판정은 단위 테스트, PC에서 띄운 Nav2 노드, 기록된 bag 재생으로 했다.

심각도는 리뷰어가 매긴 값을 그대로 적는다. "확인"은 이 문서 작성자가 다시 재현하거나 코드로 확인했다는 뜻이다.

## 1. 탈출·되짚기 (안전)

| # | 심각도 | 지적 | 확인 | 조치 |
|---|---|---|---|---|
| 1-1 | HIGH | `_retrace`가 odom 궤적에 기록된 후진 구간까지 거꾸로 따라간다. 직진 후진 탈출이나 앞선 되짚기 뒤에는, CM을 끈 채 닿았던 지점 쪽으로 최대 0.30 m 전진한다. | 확인. 궤적은 방향 구분 없이 2 cm마다 쌓이고, 되짚기는 시간 역순으로 따라간다. | 궤적을 "앞으로 들어온 길"로만 유지한다. 후진 중에는 쌓지 않고, 로봇보다 앞에 놓인 점을 지운다(`CorridorRoute._extend_odom_trail`). 되짚기 경로도 한 걸음마다 뒤쪽 60° 안이어야 하고, 간격 6 cm 이하, 회전 20° 이하여야 한다(`trail_retrace_points`). |
| 1-2 | HIGH | 차체 안에 아무것도 없는데, 앞이 막히고 뒤가 막혔거나 읽지 못하면 CM을 끄고 되짚는다. | 확인 | CM은 차체 안에 반사점이 있을 때만 끈다(`pause_monitor=intrusion is not None`). 그 밖의 되짚기는 CM을 켠 채 한다. |
| 1-3 | HIGH | 직진 탈출(전진·후진)에서 costmap 경로 검증이 빠졌다. 라이다는 0.28 m 안을 보지 못한다. 장착 위치(-0.01 m) 기준으로 차체 앞 0.08–0.20 m가 사각이고, CM도 같은 `/scan`을 쓴다. | 확인. 드라이버 `range_min = 0.28`, URDF `laser_joint` (-0.01, 0, π). | 움직일 때마다(5 cm 또는 10°) 스캔을 한 장씩 기억한다. 최대 20장, 120 s까지다. 그중 **지금 사각 안에 들어온 점만** 모든 탈출 판정(앞·뒤 띠, 차체 침범, 회전원)과 되짚기 경로 검증에 더한다. 사각 밖은 실시간 스캔이 판정한다. 되짚기 경로는 실시간 점과 기억 점으로 차체가 쓸고 지나갈 영역을 검사하고, 이미 차체 안에 있는 점은 뺀다(`path_sweep_hit`). |
| 1-4 | MEDIUM | CM이 꺼진 채 남을 수 있다. 끄기가 시간 초과로 늦게 적용된 경우 다시 켜기 결과를 확인하지 않는다. 복구 실패 뒤에도 다음 재시도로 넘어간다. 시작할 때 다시 켜지 않는다. 시간 초과 요청을 정리하지 않는다. | 확인 | `_resume_collision_monitor`: 3번 시도하고, 실패하면 비상정지 래치와 `request_stop`을 건다. 실행기마다 첫 이동 전에 `verify_live_maps`에서 CM 켜짐을 확인하고, 실패하면 출발하지 않는다. 시간 초과 요청은 `remove_pending_request`로 정리한다. |
| 1-5 | MEDIUM | 궤적에 연속성 검사가 없다. odom이 리셋되면 6.5 m짜리 되짚기 경로가 나온다. | 확인(재현) | 궤적 끝에서 0.25 m 넘게 튀면 궤적을 새로 시작하고, 되짚기는 6 cm보다 큰 간격에서 멈춘다. |
| 1-6 | MEDIUM | 되짚기 경로를 map으로 보내므로 AMCL 보정이 들어오면 궤적이 밀린다. | 확인 | 기존 `_odom_path`로 odom 경로를 보낸다. |
| 1-7 | MEDIUM | 판정마다 스캔과 footprint를 따로 읽는다. 스캔 나이를 확인하지 않고, footprint 파싱 오류(ValueError)를 잡지 않는다. | 확인 | 판정 한 번에 스냅샷 하나(스캔, 레이저 자세, 외곽, 기억 점)를 쓴다. 스캔이 0.5 s보다 오래됐거나 파싱이 실패하면 움직이지 않는다. |
| 1-8 | LOW | 컨트롤러를 거치지 않는 속도 명령 루프에 입력 가드가 없다. 회전량이 π 이상이면 끝나지 않는다. 정지 지연을 보정하지 않는다. | 확인 | `_direct_motion`으로 합쳤다. 0.1 s마다 `_guard_failure`(스캔·odom 신선도, 비상정지)를 확인한다. 감속 v²/2a에 지연 0.05 s를 더해 일찍 멈추고, 정지한 뒤의 값을 돌려준다. 회전은 누적해서 잰다. |
| 1-9 | LOW | `use_rpp_transit` 설명과 실제 트리가 다르고, 복구 이벤트 이름이 계획기 전환을 빠뜨린다. | 확인 | docstring을 고치고 이벤트에 `planner='Lattice'`를 추가했다. |
| 1-10 | LOW | `wait = 4` 테스트 경우는 실제로 도달하지 않는다. | 확인 | 그대로 둔다(경계 테스트로 무해). |

### 1.1 수정에 대한 독립 검증에서 나온 추가 결함 (2차)

| # | 심각도 | 지적 | 확인 | 조치 |
|---|---|---|---|---|
| V-1 | HIGH | 되짚기 검증이 "이미 차체 안에 있는 점"을 방향과 무관하게 뺀다. 그래서 뒤쪽 절반에 닿은 물체 쪽으로 CM을 끈 채 후진한다. | 확인. (-0.25, 0)에서 경로 검사는 통과, 직진 검사는 충돌로 판정했다. | 안에 있던 점은 이동 중 **침투 깊이**(가장 가까운 변까지 거리)가 5 mm 넘게 커지면 충돌로 본다. 떨어지거나 옆으로 미끄러지는 것은 허용하고, 밀고 들어가는 것은 거부한다. 11:09처럼 뒤 프레임 옆에 붙은 다리도, 0.30 m 후진하면 더 넓은 바퀴가 그 자리를 지나므로 거부된다. 되짚기가 거부되고 뒤만 닿아 있으며 앞이 비어 있으면, CM을 끈 채 앞으로 직진해 뒤를 떼어 낸다(92606a1의 설계). `_retrace`는 이동 전 거부면 None을 돌려준다. |
| V-2 | MEDIUM | 기억 점이 차체 침범 판정에 들어가, 사라진 물체의 점 3개만으로 CM이 꺼질 수 있다. odom 리셋 때 기억을 비우지 않는다. | 확인 | 침범(CM 정지 여부)은 실시간 점으로만 판정한다. CM도 사각을 보지 못하므로 사각 점이 CM을 붙잡을 수 없다. 기억 점은 막힘 판정과 경로 검사에만 쓴다. 키프레임 사이가 0.5 m 넘게 튀면 기억을 비운다. |
| V-3 | LOW-MEDIUM | 조기 정지량(0.351°)이 최소 보정각(0.3°)보다 크다. 그래서 0.30–0.35° 잔차는 돌지 않고 `final_trim_missed`로 실패한다. | 확인 | 조기 정지량을 명령의 절반 이하로 제한했다(직진도 같음). |
| V-4 | LOW | 속도 명령 루프 가드의 공분산 한계가 회전 중 트림을 멈출 수 있다. | 확인 | 그대로 둔다. `_execute_reverse_once`가 모든 컨트롤러 동작에 쓰는 가드와 같다. |
| V-5 | 잔여 | CM을 끈 직진 탈출이 출발 때 스냅샷 한 장으로만 앞을 본다. | 확인 | CM을 끈 직진 동안에는 0.1 s마다 최신 스캔으로 진행 방향 0.10 m 띠를 확인하고, 막히거나 스캔이 0.5 s 넘게 끊기면 멈춘다(`_band_watch`). 되짚기(컨트롤러 경로)는 출발 전 경로 검사만 한다. 이 구간은 최대 0.60 m를 주차 계약 속도로 움직인다. |

재현 테스트(`test/test_restaurant_service.py`)는 아래와 같다.
- 후진 탈출 뒤 되짚기가 뒤로만 간다: `test_retrace_after_a_back_off_continues_backwards_not_over_the_back_off`
- 방향 반전·odom 점프·제자리 회전·옆 방향에서 걷기가 멈춘다: `test_trail_walk_stops_where_the_way_back_is_not_the_way_in`
- CM 정지 조건: `test_a_touching_object_backs_out_along_the_trail_further_each_hold`의 `pause_monitor` 단정
- 사각 기억: `test_returns_the_base_drove_into_the_blind_range_are_remembered`, `test_something_in_the_front_blind_range_backs_off_instead_of_driving_forward`
- 되짚기 거부와 접촉 물체 제외: `test_retrace_refuses_a_way_back_that_is_not_clear`, `test_retrace_ignores_what_already_touches_the_body`
- CM 복구 실패와 끄기 실패: `test_retrace_that_cannot_restore_the_monitor_stops_the_run`, `test_retrace_whose_pause_failed_switches_the_monitor_back_on`
- 출발 전 CM 확인: `test_collision_monitor_is_confirmed_on_once_per_executor`
- 속도 명령 루프: `test_direct_motion_stops_on_a_failed_input`, `test_in_place_trim_counts_a_turn_across_pi`, `test_a_trim_just_over_the_minimum_still_turns`
- 2차: `test_retrace_refuses_a_way_back_that_is_not_clear`(뒤 접촉, 11:09 다리), `test_retrace_slides_off_what_touches_the_body_but_never_into_it`, `test_a_touching_rear_drives_off_it_forward_when_the_way_back_pushes_it`, `test_remembered_returns_never_pause_the_monitor`, `test_paused_straight_drive_switches_the_monitor_back_on`, `test_paused_straight_drive_stops_when_someone_steps_in_front`

## 2. 계약·설정·최종 접근

| # | 심각도 | 지적 | 확인 | 조치 |
|---|---|---|---|---|
| 2-1 | MEDIUM | 계약이 앞 5 cm 띠를 \|y\|≤0.05로 좁힌 정지 구역을 통과시킨다. | 확인 | `stopped`·`translation_forward`·`translation_forward_straight`가 footprint 앞변 전체 폭의 0.05 m 띠를 담아야 한다. |
| 2-2 | LOW | 둘레 5 mm 샘플링은 포함 증명이 아니다. 1 mm 홈이 통과한다. | 확인. 이전 계약은 홈 설정을 통과시키고 새 계약은 거부한다. | `polygon_contains`로 정확히 판정한다. 꼭짓점·변 중점 포함, 바깥 꼭짓점이 안쪽에 없음, 변 내부 교차 없음을 본다. 회전 쓸림은 0.5°마다 회전한 footprint 전체를 판정한다. |
| 2-3 | LOW | 기본 트리가 쓰는 MPPI가 `controller_plugins`에서 빠져도 통과한다. `vx_min` 음수도 허용한다. | 확인 | MPPI 등록과 `vx_min == 0`을 요구한다. |
| 2-4 | LOW | BT XML 주석의 `--`는 XML 규격 위반이다. | 확인(ElementTree 파싱 실패 3개) | 주석을 고치고, 모든 트리를 ElementTree로 파싱하는 테스트를 넣었다. |
| 2-5 | MEDIUM | 회전 없는 직진에 정지 거리 보정이 없어, 박스 쪽으로 4–6 mm 치우친다(설정값 계산). | 확인 | 1-8과 같다. |
| 2-6 | LOW | 최종 접근 실패 이벤트가 두 번 나가고 두 번째 사유가 틀린다. 5 mm 짧음을 실패로 보는데, 간격 판정은 ±1 cm다. 각 보정 뒤 남은 각을 기록하지 않는다. | 확인 | 실패는 사유와 함께 한 번만 보고한다(`_fail_final`). 짧음 허용치를 1 cm로 하고, `residual_after_deg`를 기록한다. 시계방향, 30° 초과, 이동 불가, odom 없음 경우를 테스트했다. |
| 2-7 | MEDIUM | 바퀴 앞 차선(0.23<\|y\|≤0.275)은 어떤 전진 정지 구역에도 없다. | 확인. 바퀴 바로 앞 (0.05, 0.25)는 레이저에서 0.257 m라 라이다 사각이기도 하다. | 다각형은 바꾸지 않았다. 다각형을 넓혀도 그 점은 스캔에 나오지 않는다. FootprintApproach와 §1-3의 기억 점이 이 영역을 맡는다. |
| 2-8 | MEDIUM | MPPI가 작은 v(0.005–0.03)로 돌면 `translation_forward`가 선택되어 회전 쓸림 다각형이 빠진다. | bag으로 측정했다. 10-02 18:18 MPPI 주행에서 움직임 명령의 1.5 %(78/5258, smoothed)였다. 회전 대부분(3765)은 v≤0.005라 회전 다각형이 선택되었다. | 바꾸지 않았다. FootprintApproach가 (v, w) 궤적으로 차체를 시뮬레이션해 이 경우를 덮는다. A/B 때 같은 지표를 다시 센다(`$HOME/jdamr_data/analysis_20261006/small_v_turns.py`, 아래 §5). |
| 2-9 | 미결 | 6° 쓸림과 실제 정지각의 정합이 확인되지 않았다. 주석의 "≈5.2°"는 계산하면 3.4°다. | 확인(산술) | 주석을 고쳤다. 0.3 rad/s 정지각 실측이 남아 있다. |
| 2-10 | 미결 | Lattice가 실패하면 빈 경로로 이전 경로를 계속 따를 수 있다. `RemovePassedGoals` 0.25 m와 MPPI가 경로에서 비켜 가는 것이 겹칠 수 있다. | 미확인(Nav2 C++ 소스 없음) | 실차 A/B에서 이벤트로 확인한다. |

계약 검사는 배포 설정과 정밀 주차 프로파일 둘 다 통과한다(`test_docking_stop_profile.py`).

## 3. 라이다 자기 필터·PC 도구

| # | 심각도 | 지적 | 조치 |
|---|---|---|---|
| 3-1 | MEDIUM | 레이저 자세를 저장한 뒤에도 /tf(100 Hz)를 계속 받는다. PC 기준 한 코어의 약 4.9 %다. | 저장 직후 `TransformListener.unregister()`를 호출한다. |
| 3-2 | MEDIUM | 예외나 중단으로 끝나면 UDP 모니터와 부하 기록기가 최대 1시간 남는다. | 유닛 이름을 state에 두고 모든 종료 경로와 `stop`·`estop`에서 정지한다. |
| 3-3 | MEDIUM | 프로세스 이름이 겹친다(python 스크립트끼리, `ros2 bag`과 다른 ros2 호출). | 인터프리터는 argv[1], `-m`이면 모듈, ros2는 하위 명령까지 이름에 붙인다. |
| 3-4 | LOW | 출력 QoS가 드라이버(RELIABLE)와 다르다. | RELIABLE·depth 10으로 맞췄다. best-effort 구독자와도 호환된다. |
| 3-5 | LOW | `--min-inlier nan`이 inlier 관문을 끈다. | ssh 전에 유한값이고 0.25–1.0인지 검사하고, 사용한 값을 `refine.json`에 남긴다. |
| 3-6 | LOW | `du -sh /usr/lib/` 같은 cmdline에서 기록기가 IndexError로 멈춘다. pid 재사용과 단위 혼재도 있다. | 이름 함수를 고치고, 프로세스별로 예외를 처리한다. 캐시 키를 (pid, starttime)으로, 단위를 MB로 바꾸고 읽지 못하면 null을 쓴다. |
| 3-7 | LOW | 요약 도구가 빈 파일과 잘린 줄에서 실패한다. | 잘린 줄은 세어서 건너뛰고, 빈 파일은 안내만 출력한다. |
| 3-8 | LOW | 설정 오류의 영향 범위가 경우마다 다르다. | launch에서 `load_boxes`로 미리 검증한다. 파일 이름만 주면 share/config에서 찾고, `model_not_measured`면 경고한다. |
| 3-9 | LOW | TF가 없으면 로그 없이 모든 스캔을 버린다. | 계속 버리되(필터 안 거친 스캔은 내보내지 않음) 5 s마다 경고한다. |
| 3-10 | LOW | 알림 off 상태가 눈에 띄지 않는다. | `status`와 `DEPARTED` 줄에 표시한다. |
| 3-11 | LOW | 진단 모니터 시작이 실패하면 주행 추적이 끊긴다. | 시작을 try/except로 감싸 로그만 남긴다. |
| 3-12 | MEDIUM (2차) | `stop`이 로봇 정지보다 bag 정리(reindex 최대 120 s)와 모니터 정리를 먼저 한다. Pi가 느리면 정지가 늦어지거나 시간 초과로 아예 나가지 않는다. bag 쪽은 87652f2 이전부터 있던 순서다. | 로봇 정지를 먼저 보내고 정리는 그 뒤에 한다. 순서 테스트를 추가했다. |
| 3-13 | LOW (2차) | 요약 도구가 dict가 아닌 JSON 줄과 없는 파일에서 실패한다. | 표본이 아닌 줄은 세어서 건너뛰고, 파일이 없으면 안내만 출력한다. |

필터 출력값은 +inf로 유지했다. CM·AMCL·장애물 층(`inf_is_valid` 기본 false)이 +inf와 NaN을 똑같이 버린다(`20261006_RESEARCH_NAV2_PAPERS.md` §3.3). `inf_is_valid`를 켜게 되면 NaN으로 바꿔야 한다.

## 4. 검증

- **내비게이션 전체 스위트:** 2274 통과, 실패 23·오류 9. 실패 목록이 수정 전 기준선(2234 통과)과 같다. 모두 이번 범위 밖의 기존 실패다: frontier·g005·g006 평가 자산 누락, AMCL 평가 모듈 누락, 패키지 전체 flake8·pep257(09-30 커밋).
- **변경 파일 린트:** flake8 문제 없음. pep257은 09-30 커밋의 `test_input_recovery.py:1854` 1건뿐이다.
- **bringup:** 37 통과, 1 건너뜀(수정 전 31). **PC 도구:** 68 통과(수정 전 44). 운영 사본 4개(`$HOME/jdamr_data/map_20260930_manual/tools/`)는 저장소와 같다.
- **계약 변이 대조:** 1 mm 홈 설정을 이전 계약은 통과시키고 새 계약은 거부한다. 배포 설정은 둘 다 통과한다.
- **독립 검증(oh-my-claudecode verifier 2개, 작성자와 별도):**
  - 자기 필터·PC 도구: 7개 항목 모두 해결로 판정했다. 실제 노드 실행(100 Hz /tf 중 구독 해제, QoS 혼합 구독자 수신 198/200), LaunchContext 평가, 13개 cmdline 입력으로 확인했다. 새 결함 3-12·3-13은 고쳤다. 남은 낮음 항목은 두 가지다: state 파일 읽기-쓰기 경쟁, `name_from_cmdline` 중복(Pi에 파일별로 복사하는 구조라 의도함).
  - 탈출·계약: 1차는 8개 중 5개 해결, 3개 부분 해결이었다. 검증자의 재현이다:
    - 1 m 호 60° 전진 뒤 20° 후진에서 궤적 끝 오차 7 mm, 되짚기 0.300 m
    - 기억 변환 3 mm 이내
    - 무작위 곡선 300개 기준 구현 대조 불일치 0
    - 다각형 포함 10개 경우 기대대로
  - 2차 추가 결함(§1.1)을 고친 뒤 재검증 판정은 N1–N3 해결, 조건부 승인이었다(무작위 곡선 400개, 접촉점 포함 대조 불일치 0). 남은 위험으로 지적된 V-5는 그 뒤 이동 중 띠 감시로 막았다.

## 5. 실차에서 확인할 것
1. 배포 뒤 첫 주행에서 `verify_live_maps`의 CM 켜짐 확인이 통과하는지 본다(`/collision_monitor/toggle`).
2. 사각 기억: 막힘 정지가 났을 때 이벤트의 판정 결과와 bag의 실제 물체 위치를 대조한다.
3. 되짚기: 후진 탈출 다음에 되짚기가 나면, 경로가 뒤로만 가는지 bag의 odom으로 확인한다.
4. 회전 없는 직진의 일찍 멈춤(5 mm)과 최종 접근 간격을 외부 측정과 비교한다.
5. 작은 v 회전 비율(§2-8)과 0.3 rad/s 정지각(§2-9)을 잰다.
