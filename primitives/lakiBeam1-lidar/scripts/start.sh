#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Spawn the lakibeam1_lidar capability process.
#
# Layout invariant (populated by scripts/build.sh):
#   rbnx-build/codegen/proto_gen/      atlas_pb2.py, sensor_msgs_mcp, std_msgs_mcp, …
#   rbnx-build/codegen/ros2_idl/       colcon-built ROS 2 message overlay
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

# ── PYTHONPATH ──
# robonix_api on the host; lidar_driver/ must be on the path so that
# `python3 -m lidar_driver.driver` resolves.
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
  export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
else
  export PYTHONPATH="$PKG:${PYTHONPATH:-}"
fi

exec python3 -m lidar_driver.driver
