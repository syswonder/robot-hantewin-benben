#!/usr/bin/env bash
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

if [[ -f "$PKG_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash" ]]; then
  set +u; source "$PKG_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash"; set -u
fi

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT/rbnx-build/codegen/robonix_mcp_types:$PKG_ROOT:${PYTHONPATH:-}"

exec python3 -m nero_wave.main
