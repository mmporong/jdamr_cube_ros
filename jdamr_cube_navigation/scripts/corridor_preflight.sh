#!/usr/bin/env bash
# 복도 왕복 실주행 전 자동 점검. 로봇을 움직이지 않는다.
#
# 2026-09-03에 실주행이 네 번 막혔고 원인은 모두 출발 전에 확인할 수 있는
# 것이었다. 같은 것을 사람이 매번 기억하지 않도록 한 곳에 모은다.
#   1. 비대화형 ssh 가 ROS_DOMAIN_ID 를 물려받지 못해 센서와 격리
#   2. 오래 떠 있던 RViz 가 map_server change_state 응답을 막음
#   3. keepout_costmap_filter_info_server 가 inactive 로 남아 금지구역 미적용
#   4. 누적된 전역 코스트맵이 경로를 막음
set -uo pipefail

: "${ROS_DOMAIN_ID:?ROS_DOMAIN_ID 를 지정해야 한다 (파이 bringup 은 12)}"
FAIL=0
ok()   { printf '  [PASS] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; FAIL=1; }
warn() { printf '  [주의] %s\n' "$1"; }

# 파이의 DDS 서비스 응답은 부하에 따라 느려진다. 2026-09-03 점검에서 15초
# 조회가 planner_server 와 bt_navigator 를 무응답으로 읽어 거짓 FAIL 을 냈다.
# 두 노드 모두 실제로는 active 였다. 판정 전에 반드시 재시도한다.
lifecycle_state() {
  local node="$1" state
  for _ in 1 2 3; do
    state=$(timeout 25 ros2 lifecycle get "$node" 2>/dev/null | head -1)
    [ -n "$state" ] && { printf '%s' "$state"; return 0; }
  done
  return 1
}

echo "== 1. 센서 도메인 =="
topics=$(timeout 15 ros2 topic list 2>/dev/null)
for t in /scan /odom /tf /battery_state; do
  if grep -qx "$t" <<<"$topics"; then ok "$t 보임"
  else bad "$t 없음 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID 확인)"; fi
done

echo "== 2. 원격 시각화 =="
# 규칙은 "RViz 금지"가 아니라 순서다. Nav2 활성화 전에 붙어 있으면
# map_server 의 change_state 응답을 막아 기동 자체가 실패한다.
# 이 시점에는 이미 Nav2 가 떠 있으므로 상태만 알린다. 조회가 실패하면
# 안전한 쪽(모름)으로 보고한다.
nodes=$(timeout 25 ros2 node list --no-daemon --spin-time 2 2>/dev/null)
if [ -z "$nodes" ]; then
  bad "노드 목록 조회 실패 - DDS 연결을 먼저 확인할 것"
elif grep -q rviz <<<"$nodes"; then
  warn "RViz 연결됨. Nav2 활성화 이후에 연 것이 맞는지 확인할 것"
  warn "주행 중 TF 지연이 보이면(controller 오류 102) RViz 를 닫고 재시도"
else
  ok "RViz 미연결"
fi
if grep -q -E 'cartographer|web_teleop' <<<"$nodes"; then
  bad "Cartographer 또는 웹 조종기 실행 중 — 실차 Nav2 와 동시 사용 금지"
else
  ok "Cartographer·웹 조종기 미실행"
fi

echo "== 3. Nav2 lifecycle =="
for n in /map_server /amcl /controller_server /planner_server /bt_navigator \
         /collision_monitor /keepout_filter_mask_server \
         /keepout_costmap_filter_info_server; do
  if state=$(lifecycle_state "$n"); then
    case "$state" in
      active*) ok "$n $state" ;;
      *)
        # 2026-09-03 에 keepout_costmap_filter_info_server 가 inactive 로
        # 남아 금지구역이 코스트맵에 적용되지 않은 채 출발할 뻔했다.
        # 한 번 활성화를 시도하고 결과를 다시 읽는다. 숨기지 않고 알린다.
        warn "$n $state - 활성화를 시도한다"
        timeout 30 ros2 lifecycle set "$n" activate >/dev/null 2>&1
        if state=$(lifecycle_state "$n") && [ "${state:0:6}" = "active" ]; then
          ok "$n $state (활성화 복구)"
        else
          bad "$n ${state:-무응답} - 활성화 실패"
        fi
        ;;
    esac
  else
    bad "$n 3회 조회 모두 무응답"
  fi
done

echo "== 4. 금지구역 필터 =="
for t in /keepout_filter_mask /keepout_costmap_filter_info; do
  c=$(timeout 12 ros2 topic info "$t" 2>/dev/null | awk '/Publisher count/{print $3}')
  if [ "${c:-0}" -ge 1 ]; then ok "$t 발행자 $c"; else bad "$t 발행자 없음"; fi
done

echo "== 5. 속도 명령 소유권 =="
command_info=$(timeout 12 ros2 topic info /cmd_vel --verbose \
  --no-daemon --spin-time 2 2>/dev/null)
c=$(awk '/Publisher count/{print $3}' <<<"$command_info")
publisher=$(awk '/Node name:/{node=$3} /Endpoint type: PUBLISHER/{print node}' \
  <<<"$command_info")
if [ "${c:-0}" -eq 1 ] && [ "$publisher" = collision_monitor ]; then
  ok "/cmd_vel 유일한 발행자: Collision Monitor"
else
  bad "/cmd_vel 발행자 ${c:-0}개, 노드=${publisher:-없음} (Collision Monitor 하나여야 한다)"
fi

echo "== 6. 출발 전 정지 상태 =="
linear=$(timeout 12 ros2 topic echo /odom --once \
  --field twist.twist.linear.x 2>/dev/null | head -1)
angular=$(timeout 12 ros2 topic echo /odom --once \
  --field twist.twist.angular.z 2>/dev/null | head -1)
if [ -n "$linear" ] && [ -n "$angular" ] && \
   awk "BEGIN{exit !(($linear >= -0.01 && $linear <= 0.01) &&
                       ($angular >= -0.02 && $angular <= 0.02))}"; then
  ok "로봇 정지 확인 (v=${linear}m/s, w=${angular}rad/s)"
else
  bad "출발 전 정지 확인 실패 (v=${linear:-없음}, w=${angular:-없음})"
fi

echo "== 7. 코스트맵 초기화 =="
clear_costmap() {
  local service="$1" label="$2" attempt
  for attempt in 1 2 3; do
    if timeout 20 ros2 service call "$service" \
         nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1; then
      return 0
    fi
    [ "$attempt" -eq 3 ] || {
      warn "$label 응답 지연 - 재시도 $((attempt + 1))/3"
      sleep 2
    }
  done
  return 1
}

if clear_costmap /global_costmap/clear_entirely_global_costmap "전역 코스트맵" \
   && clear_costmap /local_costmap/clear_entirely_local_costmap "지역 코스트맵"; then
  # 비운 직후에는 전역 코스트맵이 아직 미지 상태다. planner 는
  # allow_unknown:false 라 그 상태에서 계획하면 크게 우회한다. 2026-09-03에
  # 같은 경로가 비운 직후 214.863m, 3초 기다린 뒤 78.032m 로 나왔다.
  sleep 5
  ok "전역·지역 코스트맵 초기화 (재구축 대기 5초 포함)"
else
  bad "코스트맵 초기화 실패"
fi

echo "== 8. 배터리 =="
v=$(timeout 12 ros2 topic echo /battery_state --once 2>/dev/null \
    | awk '/^voltage/{print $2; exit}')
if [ -n "${v:-}" ] && awk "BEGIN{exit !($v >= 10.5)}"; then
  ok "배터리 ${v}V"
else
  bad "배터리 ${v:-측정 실패}V (10.5V 이상 필요)"
fi

echo
if [ "$FAIL" -eq 0 ]; then
  echo "사전점검 PASS — planning-only 확인 후 --execute 로 출발할 수 있다."
else
  echo "사전점검 FAIL — 위 항목을 해결하기 전에는 출발하지 않는다."
fi
exit "$FAIL"
