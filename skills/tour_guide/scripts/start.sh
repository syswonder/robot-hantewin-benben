#!/usr/bin/env bash
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT/rbnx-build/codegen/robonix_mcp_types:$PKG_ROOT:${PYTHONPATH:-}"

# The Robonix runtime venv carries the MCP stack (mcp / fastmcp) plus
# modern grpc deps that the host python3 lacks. Prefer it, and fall back
# to whatever `python3` is on PATH when the venv isn't provisioned.
PY="${RBNX_CODEGEN_PYTHON:-${HOME}/.robonix/venv/bin/python}"
[[ -x "$PY" ]] || PY=python3

exec "$PY" -m tour_guide_skill.main
