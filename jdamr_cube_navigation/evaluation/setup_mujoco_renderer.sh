#!/usr/bin/env bash
set -euo pipefail

evaluation_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${evaluation_dir}/../.." && pwd)"
workspace_root="$(cd "${repo_root}/../.." && pwd)"
venv_path="${JDAMR_MUJOCO_VENV:-${workspace_root}/.venv-mujoco}"

available_kib="$(df -Pk "${workspace_root}" | awk 'NR == 2 {print $4}')"
if (( available_kib < 1048576 )); then
  echo "MuJoCo 설치 중단: 여유 디스크가 1 GiB보다 적습니다." >&2
  exit 2
fi
if [[ -e "${venv_path}" && ! -f "${venv_path}/pyvenv.cfg" ]]; then
  echo "MuJoCo 설치 중단: 대상이 Python 가상환경이 아닙니다: ${venv_path}" >&2
  exit 2
fi

uv venv --allow-existing --system-site-packages --python 3.12 "${venv_path}"
UV_LINK_MODE=copy uv pip install \
  --python "${venv_path}/bin/python" \
  --no-cache \
  -r "${evaluation_dir}/requirements-mujoco.txt"

"${venv_path}/bin/python" -c \
  'import cv2, mujoco; print(f"MuJoCo {mujoco.__version__}, OpenCV {cv2.__version__}")'
