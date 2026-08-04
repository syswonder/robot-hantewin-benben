#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Spawn the benben_chassis capability process.
#
# Layout invariant (populated by scripts/build.sh):
#   rbnx-build/codegen/proto_gen/    atlas_pb2.py, chassis_pb2.py, std_msgs_pb2.py, …
#   rbnx-build/codegen/ros2_idl/     colcon-built ROS 2 message overlay
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
# robonix_api on the host; chassis_driver/ must be on the path so that
# `from tbox_sdk import ...` resolves (tbox_sdk lives in chassis_driver/).
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
  export PYTHONPATH="$ROBONIX_API:$PKG:$PKG/chassis_driver:${PYTHONPATH:-}"
else
  export PYTHONPATH="$PKG:$PKG/chassis_driver:${PYTHONPATH:-}"
fi

# The TBox shared library is in chassis_driver/tbox_sdk/lib/.
export LD_LIBRARY_PATH="$PKG/chassis_driver/tbox_sdk/lib:${LD_LIBRARY_PATH:-}"

exec /home/hty/pyenv/env/bin/python3 -m chassis_driver.driver
# exec python3 -m chassis_driver.driver
