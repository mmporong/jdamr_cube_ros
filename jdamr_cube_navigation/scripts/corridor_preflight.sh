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
ok()  { printf '  [PASS] %s\n' "$1"; }
bad() { printf '  [FAIL] %s\n' "$1"; FAIL=1; }

echo "== 1. 센서 도메인 =="
topics=$(timeout 15 ros2 topic list 2>/dev/null)
for t in /scan /odom /tf /battery_state; do
  if grep -qx "$t" <<<"$topics"; then ok "$t 보임"
  else bad "$t 없음 (ROS_DOMAIN_ID=$ROS_DOMAIN_ID 확인)"; fi
done

echo "== 2. 원격 시각화 =="
if timeout 15 ros2 node list 2>/dev/null | grep -q rviz; then
  bad "RViz 가 이미 떠 있다. Nav2 기동 전에는 끄고, 활성화 확인 후 열 것"
else
  ok "RViz 미실행"
fi

echo "== 3. Nav2 lifecycle =="
for n in /map_server /amcl /controller_server /planner_server /bt_navigator \
         /collision_monitor /keepout_filter_mask_server \
         /keepout_costmap_filter_info_server; do
  state=$(timeout 15 ros2 lifecycle get "$n" 2>/dev/null | head -1)
  case "$state" in
    active*) ok "$n $state" ;;
    *)       bad "$n ${state:-응답 없음}" ;;
  esac
done

echo "== 4. 금지구역 필터 =="
for t in /keepout_filter_mask /keepout_costmap_filter_info; do
  c=$(timeout 12 ros2 topic info "$t" 2>/dev/null | awk '/Publisher count/{print $3}')
  if [ "${c:-0}" -ge 1 ]; then ok "$t 발행자 $c"; else bad "$t 발행자 없음"; fi
done

echo "== 5. 속도 명령 소유권 =="
c=$(timeout 12 ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/{print $3}')
if [ "${c:-0}" -eq 1 ]; then ok "/cmd_vel 발행자 1 (Collision Monitor)"
else bad "/cmd_vel 발행자 ${c:-0} (1이어야 한다)"; fi

echo "== 6. 코스트맵 초기화 =="
if timeout 20 ros2 service call /global_costmap/clear_entirely_global_costmap \
     nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1 \
   && timeout 20 ros2 service call /local_costmap/clear_entirely_local_costmap \
     nav2_msgs/srv/ClearEntireCostmap >/dev/null 2>&1; then
  ok "전역·지역 코스트맵 초기화"
else
  bad "코스트맵 초기화 실패"
fi

echo "== 7. 배터리 =="
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
