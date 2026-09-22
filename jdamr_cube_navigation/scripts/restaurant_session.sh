#!/usr/bin/env bash
# 식당 서비스용 Nav2 세션만 관리한다. 목적지 action과 초기 pose는 보내지 않는다.
set -o pipefail

UNIT="jdamr-restaurant-navigation.service"
WORKSPACE="${JDAMR_WORKSPACE:-$HOME/jdamr_ws}"
REGISTRY="${JDAMR_RESTAURANT_REGISTRY:-}"
PARAMS_FILE="${JDAMR_RESTAURANT_PARAMS:-}"
PRECISION_PARKING=false

usage() {
  cat <<'EOF'
사용법:
  restaurant_session.sh status
  restaurant_session.sh start [--workspace PATH] [--registry PATH] [--params-file PATH]
                              [--precision-parking]
  restaurant_session.sh stop

start는 센서가 이미 실행 중인 Pi에서 식당 서비스용 Nav2 서버만 시작한다.
초기 pose, NavigateToPose, FollowPath 등 이동 명령은 보내지 않는다.
--precision-parking은 명시적으로 승인된 5 cm 박스 주차 시험에서만 사용한다.

기본값:
  workspace   $HOME/jdamr_ws
  registry    $HOME/jdamr_data/maps/20260922_manual_final_run2/service_destinations.yaml
  params-file <workspace>/install/jdamr_cube_navigation/share/
              jdamr_cube_navigation/config/new_base_nav2_params.yaml

status의 active는 프로세스 실행 상태일 뿐 위치추정·주행 준비 완료를 뜻하지 않는다.
EOF
}

die() {
  printf '실패: %s\n' "$*" >&2
  exit 4
}

absolute_file() {
  local path="$1" label="$2"
  [ -n "$path" ] || die "$label 경로가 비어 있다"
  [ "${path#/}" != "$path" ] || die "$label 경로는 절대경로여야 한다: $path"
  [ -f "$path" ] && [ -r "$path" ] || die "$label 파일을 읽을 수 없다: $path"
}

absolute_directory() {
  local path="$1" label="$2"
  [ "${path#/}" != "$path" ] || die "$label 경로는 절대경로여야 한다: $path"
  [ -d "$path" ] && [ -x "$path" ] || die "$label 디렉터리를 사용할 수 없다: $path"
}

load_ros_environment() {
  local ros_setup="${JDAMR_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"
  local overlay="$WORKSPACE/install/setup.bash"
  [ -r "$ros_setup" ] || die "ROS setup을 읽을 수 없다: $ros_setup"
  [ -r "$overlay" ] || die "workspace overlay를 읽을 수 없다: $overlay"

  # ROS setup 파일은 정의되지 않은 변수를 조회할 수 있어 nounset보다 먼저 읽는다.
  set +u
  # shellcheck disable=SC1090
  source "$ros_setup" || die "ROS setup 적용 실패: $ros_setup"
  # shellcheck disable=SC1090
  source "$overlay" || die "workspace overlay 적용 실패: $overlay"
  set -u

  export ROS_DOMAIN_ID=12
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
}

system_service_state() {
  local service="$1" rc
  systemctl is-active --quiet "$service"
  rc=$?
  case "$rc" in
    0) return 0 ;;
    3|4) return 1 ;;
    *) die "systemd에서 $service 상태를 확인할 수 없다 (exit=$rc)" ;;
  esac
}

reject_conflicting_services() {
  local service
  for service in \
      jdamr-cartographer-session.service \
      jdamr-operator-mapping.service \
      jdamr-localization.service \
      jdamr-slam-toolbox.service \
      jdamr-navigation.service; do
    if system_service_state "$service"; then
      die "$service 실행 중: 중지시키지 않았으며 localization/Nav2 중복 기동을 거부한다"
    fi
  done
}

check_ros_graph_and_sensors() {
  local nodes topics conflict missing="" topic
  if ! nodes=$(timeout 15 ros2 node list --no-daemon --spin-time 2 2>/dev/null); then
    die "ROS 노드 목록 조회 실패: domain 12/SUBNET 센서 연결을 확인할 것"
  fi
  conflict=$(grep -E \
    '(^|/)(amcl|map_server|controller_server|planner_server|bt_navigator|behavior_server|waypoint_follower|velocity_smoother|collision_monitor|slam_toolbox|lifecycle_manager_localization|lifecycle_manager_navigation)$|cartographer' \
    <<<"$nodes" || true)
  if [ -n "$conflict" ]; then
    die "기존 localization/Cartographer/Nav2 노드가 있어 중복 기동하지 않는다: $(tr '\n' ' ' <<<"$conflict")"
  fi

  if ! topics=$(timeout 15 ros2 topic list --no-daemon --spin-time 2 2>/dev/null); then
    die "ROS 토픽 목록 조회 실패: 센서 세션을 별도로 진단할 것"
  fi
  for topic in /scan /odom /tf; do
    grep -qx "$topic" <<<"$topics" || missing="$missing $topic"
  done
  [ -z "$missing" ] || die "센서 진단 실패: 다음 토픽이 없다:$missing (jdamr-base.service와 센서 상태 확인)"
  printf '센서 진단: /scan /odom /tf 발견 (데이터 품질이나 주행 준비 완료 판정은 아님)\n'
}

validate_registry() {
  absolute_file "$REGISTRY" "registry"
  absolute_file "$PARAMS_FILE" "Nav2 params"
  if ! python3 - "$REGISTRY" <<'PY'
import sys
from jdamr_cube_navigation.service_destinations import load_registry

load_registry(sys.argv[1])
PY
  then
    die "registry 스키마·지도·keepout 경로 또는 해시 검증 실패: $REGISTRY"
  fi
}

run_navigation() {
  local parking_contract
  load_ros_environment
  validate_registry
  parking_contract="$WORKSPACE/install/jdamr_cube_navigation/share/jdamr_cube_navigation/config/parking_contract.yaml"
  if [ "$PRECISION_PARKING" = true ]; then
    parking_contract="$WORKSPACE/install/jdamr_cube_navigation/share/jdamr_cube_navigation/config/box_parking_contract.yaml"
  fi
  cd "$WORKSPACE" || die "workspace로 이동할 수 없다: $WORKSPACE"
  exec ros2 launch jdamr_cube_navigation restaurant_service.launch.py \
    "registry:=$REGISTRY" \
    "params_file:=$PARAMS_FILE" \
    navigation_profile:=new_base_candidate \
    use_composition:=false \
    coordinated_startup:=true \
    "precision_parking:=$PRECISION_PARKING" \
    "parking_contract:=$parking_contract" \
    use_box_observer:=false \
    discovery_range:=SUBNET
}

start_navigation() {
  absolute_directory "$WORKSPACE" "workspace"
  REGISTRY="${REGISTRY:-$HOME/jdamr_data/maps/20260922_manual_final_run2/service_destinations.yaml}"
  PARAMS_FILE="${PARAMS_FILE:-$WORKSPACE/install/jdamr_cube_navigation/share/jdamr_cube_navigation/config/new_base_nav2_params.yaml}"
  load_ros_environment
  validate_registry

  system_service_state jdamr-base.service || die "jdamr-base.service가 active가 아니다; 이 스크립트는 베이스를 시작하지 않는다"
  if system_service_state "$UNIT"; then
    die "$UNIT가 이미 active다; singleton 세션을 중복 시작하지 않는다"
  fi
  reject_conflicting_services
  check_ros_graph_and_sensors

  local script_path run_user precision_argument=()
  script_path=$(readlink -f "${BASH_SOURCE[0]}") || die "스크립트 경로 확인 실패"
  run_user=$(id -un) || die "실행 사용자 확인 실패"
  if [ "$PRECISION_PARKING" = true ]; then
    precision_argument=(--precision-parking)
  fi
  sudo -n systemd-run \
    --unit="$UNIT" --collect \
    --property="User=$run_user" \
    --property=KillMode=mixed \
    --property=KillSignal=SIGINT \
    --property=TimeoutStopSec=20 \
    --setenv="HOME=$HOME" \
    --setenv="ROS_DISTRO=${ROS_DISTRO:-jazzy}" \
    --setenv=JDAMR_RESTAURANT_INTERNAL=1 \
    --working-directory="$WORKSPACE" \
    /bin/bash "$script_path" __run \
    --workspace "$WORKSPACE" --registry "$REGISTRY" --params-file "$PARAMS_FILE" \
    "${precision_argument[@]}" \
    || die "$UNIT 기동 요청 실패"

  printf '%s\n' "$UNIT 기동 요청을 수락했다."
  printf '%s\n' "이는 Nav2 프로세스 시작 요청이며 위치추정·주행 준비 완료가 아니다."
  printf '%s\n' "초기 pose를 추측해 넣거나 주행 action을 보내지 않았다. status와 journal로 별도 확인할 것."
}

status_navigation() {
  systemctl status --no-pager "$UNIT"
  local rc=$?
  printf '%s\n' "주의: active는 프로세스 상태이며 위치추정·주행 준비 완료 판정이 아니다."
  return "$rc"
}

stop_navigation() {
  sudo -n systemctl stop "$UNIT" || die "$UNIT 중지 요청 실패"
  printf '%s\n' "$UNIT 중지 요청을 보냈다. 베이스·센서 서비스는 변경하지 않았다."
}

COMMAND="${1:-}"
[ $# -gt 0 ] && shift
case "$COMMAND" in
  -h|--help|help)
    [ $# -eq 0 ] || die "help에는 추가 인자를 사용할 수 없다"
    usage
    ;;
  status|stop)
    [ $# -eq 0 ] || die "$COMMAND에는 추가 인자를 사용할 수 없다"
    "${COMMAND}_navigation"
    ;;
  start|__run)
    while [ $# -gt 0 ]; do
      case "$1" in
        --workspace|--registry|--params-file)
          [ $# -ge 2 ] || die "$1 값이 필요하다"
          case "$1" in
            --workspace) WORKSPACE="$2" ;;
            --registry) REGISTRY="$2" ;;
            --params-file) PARAMS_FILE="$2" ;;
          esac
          shift 2
          ;;
        --precision-parking)
          PRECISION_PARKING=true
          shift
          ;;
        *) die "알 수 없는 인자: $1 (도움말: --help)" ;;
      esac
    done
    if [ "$COMMAND" = start ]; then
      start_navigation
    else
      [ "${JDAMR_RESTAURANT_INTERNAL:-}" = 1 ] || \
        die "내부 실행 경로는 start가 만든 systemd 유닛에서만 사용할 수 있다"
      run_navigation
    fi
    ;;
  '')
    usage >&2
    exit 2
    ;;
  *)
    die "알 수 없는 명령: $COMMAND (status/start/stop 또는 --help)"
    ;;
esac
