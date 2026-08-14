#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ── ROS 2 environment ──
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Source the Robonix ROS 2 IDL overlay (canonical message definitions).
if [[ -f "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash" ]]; then
  set +u; source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"; set -u
fi

#export ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-127.0.0.1}"
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
  export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
else
  export PYTHONPATH="$PKG:${PYTHONPATH:-}"
fi

exec python3 -m nero_arm.main
