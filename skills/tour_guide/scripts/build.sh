#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# --mcp because @cap.mcp tools need typed Request/Response classes
# under robonix_mcp_types/. rbnx codegen also picks up this package's
# own capabilities/ tree (tour_guide.v1.toml + lib/tour_guide/srv/) so
# robonix_contracts.proto carries the skill's servicer and
# tour_guide_mcp.py carries IntroduceArea_Request/_Response.
FLAGS=(--mcp)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[build] done."
