#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build phase: rbnx codegen so the driver can import atlas_pb2,
# chassis_pb2, std_msgs_pb2, and robonix_contracts_pb2_grpc.
#
# This primitive does NOT have its own ROS workspace — it depends on the
# system-level ROS 2 installation for geometry_msgs and nav_msgs.
# Codegen output goes into rbnx-build/codegen/.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp --ros2)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[benben_chassis/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# Build the ROS 2 IDL overlay so rclpy type-support is available.
if [[ -d "$PKG/rbnx-build/codegen/ros2_idl" ]]; then
  ROS_DISTRO="${ROS_DISTRO:-jazzy}"
  set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
  echo "[benben_chassis/build] colcon build (Robonix ROS 2 interfaces)"
  (cd "$PKG/rbnx-build/codegen/ros2_idl" && colcon build)
fi

touch "$PKG/rbnx-build/.rbnx-built"
echo "[benben_chassis/build] done."
