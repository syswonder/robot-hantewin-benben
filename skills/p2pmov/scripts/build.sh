#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build phase: rbnx codegen so the skill can import p2pmov_pb2 (gRPC) and
# p2pmov_mcp (MCP dataclasses) generated from capabilities/lib/p2pmov/srv/*.
#
# This skill has no ROS 2 contracts and consumes no atlas inputs (it talks
# to the chassis via the TBox SDK directly), so codegen needs only --mcp —
# the same minimal flag set the explore skill uses.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[p2pmov/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[p2pmov/build] done."
