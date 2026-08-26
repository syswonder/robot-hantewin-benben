#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp --ros2)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[nero_return_home/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

if [[ -d "$PKG/rbnx-build/codegen/ros2_idl" ]]; then
  ROS_DISTRO="${ROS_DISTRO:-jazzy}"
  set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u
  echo "[nero_return_home/build] colcon build (Robonix ROS 2 interfaces)"
  (cd "$PKG/rbnx-build/codegen/ros2_idl" && colcon build)
fi

touch "$PKG/rbnx-build/.rbnx-built"
echo "[nero_return_home] build done"
