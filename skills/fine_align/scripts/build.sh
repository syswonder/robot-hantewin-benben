#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build phase: rbnx codegen for the align MCP tool dataclasses.
#
# fine_align consumes upstream deps over MCP (nero_grasp.detect, chassis.move)
# and reads odom over ROS; its own contract uses only primitive types, so
# codegen needs only --mcp (no ROS interfaces, no colcon).
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[fine_align/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[fine_align/build] done."
