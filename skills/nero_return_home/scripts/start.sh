#!/usr/bin/env bash
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT/rbnx-build/codegen/robonix_mcp_types:$PKG_ROOT:${PYTHONPATH:-}"

exec python3 -m nero_return_home.main
