#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Spawn the p2pmov skill capability process (host-native, no Docker).
#
# Layout invariant (populated by scripts/build.sh):
#   rbnx-build/codegen/proto_gen/    atlas_pb2.py, p2pmov_pb2.py, ...
#   rbnx-build/codegen/robonix_mcp_types/   p2pmov_mcp.py, ...
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ── ROS 2 environment ──
# The skill itself is ROS-agnostic (TBox SDK only), but keep parity with the
# benben_chassis native pattern: the host pyenv python is a ROS-aware build
# and sourcing ROS setup is harmless.
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# ── PYTHONPATH ──
# robonix_api on the host; package root so `p2pmov_skill` + the vendored
# `tbox_sdk` (under p2pmov_skill/) resolve.
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
  export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
else
  export PYTHONPATH="$PKG:${PYTHONPATH:-}"
fi

# The TBox shared library is in p2pmov_skill/tbox_sdk/lib/.
export LD_LIBRARY_PATH="$PKG/p2pmov_skill/tbox_sdk/lib:${LD_LIBRARY_PATH:-}"

PY="${P2PMOV_PYTHON:-/home/hty/pyenv/env/bin/python3}"
exec "$PY" -u -m p2pmov_skill.atlas_bridge
