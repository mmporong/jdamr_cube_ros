#!/usr/bin/env bash
set -euo pipefail

evaluation_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${evaluation_dir}/../.." && pwd)"
workspace_root="$(cd "${repo_root}/../.." && pwd)"
venv_path="${JDAMR_MUJOCO_VENV:-${workspace_root}/.venv-mujoco}"

if [[ ! -x "${venv_path}/bin/python" ]]; then
  echo "MuJoCo 환경이 없습니다: ${venv_path}" >&2
  echo "먼저 ${evaluation_dir}/setup_mujoco_renderer.sh 를 실행하세요." >&2
  exit 2
fi

available_memory_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
available_disk_kib="$(df -Pk "${workspace_root}" | awk 'NR == 2 {print $4}')"
if (( available_memory_kib < 2097152 )); then
  echo "MuJoCo 렌더링 중단: 사용 가능 메모리가 2 GiB보다 적습니다." >&2
  exit 2
fi
if (( available_disk_kib < 1048576 )); then
  echo "MuJoCo 렌더링 중단: 여유 디스크가 1 GiB보다 적습니다." >&2
  exit 2
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
fi
if [[ -f "${workspace_root}/install/setup.bash" ]]; then
  set +u
  source "${workspace_root}/install/setup.bash"
  set -u
fi

export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYTHONPATH="${evaluation_dir}${PYTHONPATH:+:${PYTHONPATH}}"
exec "${venv_path}/bin/python" \
  "${evaluation_dir}/render_mujoco_nav2_evidence.py" "$@"
