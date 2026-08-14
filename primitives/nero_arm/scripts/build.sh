#!/usr/bin/env bash
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

echo "[nero_arm] build done"

