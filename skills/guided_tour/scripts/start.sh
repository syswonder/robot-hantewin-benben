#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start the guided_tour skill capability process (host-native).
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

export PYTHONPATH="$(rbnx path robonix-api):$PKG/rbnx-build/codegen/robonix_mcp_types:$PKG:${PYTHONPATH:-}"

exec python3 -u -m guided_tour_skill.atlas_bridge
