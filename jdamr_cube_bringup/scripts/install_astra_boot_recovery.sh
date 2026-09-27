#!/bin/bash
# Install Astra boot recovery assets without changing current service/USB state.
set -euo pipefail

package_name='jdamr_cube_bringup'
sudo_bin="${SUDO_BIN:-/usr/bin/sudo}"
install_bin="${INSTALL_BIN:-/usr/bin/install}"
systemctl_bin="${SYSTEMCTL_BIN:-/usr/bin/systemctl}"
install_root="${ASTRA_INSTALL_ROOT:-}"

require_absolute_executable() {
  local command_path="$1"
  case "$command_path" in
    /*) ;;
    *)
      echo "Command path must be an absolute executable: $command_path" >&2
      return 1
      ;;
  esac
  if [ ! -x "$command_path" ] || [ -d "$command_path" ]; then
    echo "Command path must be an absolute executable: $command_path" >&2
    return 1
  fi
}

require_absolute_executable "$sudo_bin"
require_absolute_executable "$install_bin"
require_absolute_executable "$systemctl_bin"

if [ -n "$install_root" ]; then
  case "$install_root" in
    /*) ;;
    *)
      echo "ASTRA_INSTALL_ROOT must be absolute: $install_root" >&2
      exit 2
      ;;
  esac
fi

script_path="$(/usr/bin/readlink -f -- "${BASH_SOURCE[0]}")"
script_dir="$(/usr/bin/dirname -- "$script_path")"
source_root="$(/usr/bin/readlink -f -- "$script_dir/..")"

if [ -f "$source_root/systemd/jdamr-astra-usb-recover.service" ]; then
  asset_root="$source_root"
else
  install_prefix="$(/usr/bin/readlink -f -- "$script_dir/../..")"
  asset_root="$install_prefix/share/$package_name"
fi

helper_source="$(/usr/bin/readlink -f -- \
  "$script_dir/jdamr-astra-usb-recover")"
unit_source="$(/usr/bin/readlink -f -- \
  "$asset_root/systemd/jdamr-astra-usb-recover.service")"
drop_in_source="$(/usr/bin/readlink -f -- \
  "$asset_root/systemd/jdamr-base.service.d/15-astra-usb-recover.conf")"

for source_file in "$helper_source" "$unit_source" "$drop_in_source"; do
  if [ ! -f "$source_file" ] || [ ! -r "$source_file" ]; then
    echo "Required Astra recovery asset is unavailable: $source_file" >&2
    exit 1
  fi
done

helper_target="${install_root}/usr/local/sbin/jdamr-astra-usb-recover"
unit_target="${install_root}/etc/systemd/system/jdamr-astra-usb-recover.service"
drop_in_target="${install_root}/etc/systemd/system/jdamr-base.service.d/15-astra-usb-recover.conf"

"$sudo_bin" -n "$install_bin" -D -m 0755 -- \
  "$helper_source" "$helper_target"
"$sudo_bin" -n "$install_bin" -D -m 0644 -- \
  "$unit_source" "$unit_target"
"$sudo_bin" -n "$install_bin" -D -m 0644 -- \
  "$drop_in_source" "$drop_in_target"
"$sudo_bin" -n "$systemctl_bin" daemon-reload
"$sudo_bin" -n "$systemctl_bin" enable jdamr-astra-usb-recover.service

echo 'Astra boot recovery installed and enabled; no service was started or restarted.'
