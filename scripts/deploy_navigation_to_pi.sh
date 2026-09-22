#!/bin/bash
# 커밋된 navigation 패키지를 파이에서 선검증한 뒤 주 워크스페이스에 반영한다.
# ROS setup 스크립트는 unset 변수를 참조하므로 nounset(set -u)을 사용하지 않는다.
set -eo pipefail

usage() {
  echo "usage: $0 [ssh-target] [git-commit]" >&2
}

ssh_target="${1:-lim@jdamr.local}"
git_commit="${2:-HEAD}"
repo_root="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "$repo_root" ] || ! git -C "$repo_root" cat-file -e "$git_commit^{commit}"; then
  usage
  exit 2
fi
git_commit="$(git -C "$repo_root" rev-parse --verify "$git_commit^{commit}")"

package_path=jdamr_cube_navigation
if ! git -C "$repo_root" cat-file -e "$git_commit:$package_path/package.xml"; then
  echo "commit does not contain $package_path" >&2
  exit 2
fi

temporary_dir="$(mktemp -d /tmp/jdamr-navigation-deploy-XXXXXX)"
archive="$temporary_dir/jdamr_cube_navigation.tar.gz"
cleanup_local() {
  rm -f "$archive"
  rmdir "$temporary_dir" 2>/dev/null || true
}
trap cleanup_local EXIT

git -C "$repo_root" archive --format=tar.gz --output="$archive" \
  "$git_commit" "$package_path"
archive_sha256="$(sha256sum "$archive" | cut -d' ' -f1)"
remote_archive_name="jdamr_navigation_${archive_sha256}.tar.gz"
ssh_options=(-o BatchMode=yes -o ConnectTimeout=5 -o HostKeyAlias=jdamr.local)

scp "${ssh_options[@]}" "$archive" "$ssh_target:$remote_archive_name"
ssh "${ssh_options[@]}" "$ssh_target" bash -s -- \
  "$remote_archive_name" "$archive_sha256" "$git_commit" <<'REMOTE'
set -Eeo pipefail

archive_name="$1"
expected_sha256="$2"
git_commit="$3"
workspace="$HOME/jdamr_ws"
source_root="$workspace/src/jdamr_cube_ros"
package=jdamr_cube_navigation
if [[ ! "$expected_sha256" =~ ^[0-9a-f]{64}$ ]] \
    || [ "$archive_name" != "jdamr_navigation_${expected_sha256}.tar.gz" ]; then
  echo "invalid deployment archive identity" >&2
  exit 2
fi
archive="$HOME/$archive_name"
stage=
backup=
source_slot_owned=0
build_slot_owned=0
install_slot_owned=0
cleanup_transient() {
  case "$archive" in
    "$HOME"/jdamr_navigation_*.tar.gz) rm -f "$archive" ;;
    *) echo "refusing to remove unexpected archive path: $archive" >&2 ;;
  esac
  if [ -n "$stage" ]; then
    case "$stage" in
      "$HOME"/jdamr_deploy/stage_navigation_*) find "$stage" -depth -delete ;;
      *) echo "refusing to remove unexpected stage path: $stage" >&2 ;;
    esac
  fi
  if [ -n "$backup" ] \
      && [ "$source_slot_owned" = 0 ] \
      && [ "$build_slot_owned" = 0 ] \
      && [ "$install_slot_owned" = 0 ]; then
    case "$backup" in
      "$HOME"/jdamr_deploy/pre_navigation_*) find "$backup" -depth -delete ;;
      *) echo "refusing to remove unexpected backup path: $backup" >&2 ;;
    esac
  fi
  return 0
}
trap cleanup_transient EXIT

# ROS 환경을 읽은 뒤에도 nounset은 켜지 않는다. Jazzy setup과의 호환 계약이다.
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"

actual_sha256="$(sha256sum "$archive" | cut -d' ' -f1)"
if [ "$actual_sha256" != "$expected_sha256" ]; then
  echo "archive checksum mismatch" >&2
  exit 1
fi

stage="$(mktemp -d "$HOME/jdamr_deploy/stage_navigation_XXXXXX")"
backup="$(mktemp -d "$HOME/jdamr_deploy/pre_navigation_XXXXXX")"
mkdir -p "$stage/src" "$backup/source" "$backup/build" "$backup/install"
tar -xzf "$archive" -C "$stage/src"

# 실행 중인 주 워크스페이스와 분리해 파이 호환성을 먼저 확인한다.
(
  cd "$stage"
  colcon build --packages-select "$package" --symlink-install
  source "$stage/install/setup.bash"
  ros2 run "$package" restaurant_service teach --help >/dev/null
  python3 -m pytest -q \
    "src/$package/test/test_service_destinations.py" \
    "src/$package/test/test_restaurant_service.py" \
    "src/$package/test/test_parking_contract.py" \
    "src/$package/test/test_parking_integration.py"
)

rollback() {
  reason="$1"
  original_status="$2"
  if [ "$rollback_running" = 1 ]; then
    exit "$original_status"
  fi
  rollback_running=1
  trap - ERR
  trap '' HUP INT TERM
  set +e
  rollback_failed=0
  mkdir -p "$backup/failed" || rollback_failed=1
  if [ "$source_slot_owned" = 1 ] \
      || [ "$build_slot_owned" = 1 ] \
      || [ "$install_slot_owned" = 1 ]; then
    sudo systemctl stop jdamr-base.service || rollback_failed=1
  fi
  if [ "$source_slot_owned" = 1 ]; then
    [ ! -e "$source_root/$package" ] \
      || mv "$source_root/$package" "$backup/failed/source" \
      || rollback_failed=1
    if [ -e "$backup/source/$package" ]; then
      mv "$backup/source/$package" "$source_root/" || rollback_failed=1
    else
      rollback_failed=1
    fi
  fi
  if [ "$build_slot_owned" = 1 ]; then
    [ ! -e "$workspace/build/$package" ] \
      || mv "$workspace/build/$package" "$backup/failed/build" \
      || rollback_failed=1
    [ ! -e "$backup/build/$package" ] \
      || mv "$backup/build/$package" "$workspace/build/" \
      || rollback_failed=1
  fi
  if [ "$install_slot_owned" = 1 ]; then
    [ ! -e "$workspace/install/$package" ] \
      || mv "$workspace/install/$package" "$backup/failed/install" \
      || rollback_failed=1
    [ ! -e "$backup/install/$package" ] \
      || mv "$backup/install/$package" "$workspace/install/" \
      || rollback_failed=1
  fi
  sudo systemctl start jdamr-base.service || rollback_failed=1
  systemctl is-active --quiet jdamr-base.service || rollback_failed=1
  if [ "$rollback_failed" = 0 ]; then
    echo "deployment interrupted ($reason); previous package restored from $backup" >&2
  else
    echo "deployment interrupted ($reason); rollback incomplete, inspect $backup" >&2
  fi
  exit "$original_status"
}
rollback_running=0
trap 'status=$?; rollback ERR "$status"' ERR
trap 'rollback HUP 129' HUP
trap 'rollback INT 130' INT
trap 'rollback TERM 143' TERM

sudo systemctl stop jdamr-base.service
if systemctl is-active --quiet jdamr-base.service; then
  echo "base service did not stop" >&2
  false
fi

mv "$source_root/$package" "$backup/source/"
source_slot_owned=1
if [ -e "$workspace/build/$package" ]; then
  mv "$workspace/build/$package" "$backup/build/"
fi
build_slot_owned=1
if [ -e "$workspace/install/$package" ]; then
  mv "$workspace/install/$package" "$backup/install/"
fi
install_slot_owned=1
mv "$stage/src/$package" "$source_root/"

(
  cd "$workspace"
  colcon build --packages-select "$package" --symlink-install
  source "$workspace/install/setup.bash"
  ros2 run "$package" restaurant_service teach --help >/dev/null
  python3 -m pytest -q \
    "$source_root/$package/test/test_service_destinations.py" \
    "$source_root/$package/test/test_restaurant_service.py" \
    "$source_root/$package/test/test_parking_contract.py" \
    "$source_root/$package/test/test_parking_integration.py"
)

sudo systemctl start jdamr-base.service
systemctl is-active --quiet jdamr-base.service

source_sha256="$(sha256sum \
  "$source_root/$package/jdamr_cube_navigation/restaurant_service.py" | cut -d' ' -f1)"
printf 'commit=%s\narchive_sha256=%s\nsource_sha256=%s\nbackup=%s\n' \
  "$git_commit" "$actual_sha256" "$source_sha256" "$backup" \
  >"$backup/deployment_receipt.txt"
test -s "$backup/deployment_receipt.txt"

echo "navigation deployment complete"
cat "$backup/deployment_receipt.txt"
trap - ERR HUP INT TERM
REMOTE
