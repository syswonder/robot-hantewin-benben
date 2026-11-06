#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build phase: rbnx codegen for the run/status/cancel MCP tool dataclasses.
#
# guided_tour consumes upstream deps over MCP (navigation/*, speech/speak);
# its own contracts use only primitive types, so codegen needs only --mcp
# (no ROS interfaces, no colcon) — same minimal flag set as fine_align/p2pmov.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[guided_tour/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[guided_tour/build] done."
