#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start the fine_align skill capability process (host-native).
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

export PYTHONPATH="$(rbnx path robonix-api):$PKG/rbnx-build/codegen/robonix_mcp_types:$PKG:${PYTHONPATH:-}"

exec python3 -u -m fine_align_skill.atlas_bridge
