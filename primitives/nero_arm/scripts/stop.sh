#!/usr/bin/env bash
set -euo pipefail
# Ensure standalone stop tears down any leftover nero_arm python process for this package.
pkill -f "python3 -m nero_arm.main" 2>/dev/null || true
echo "[nero_arm] stop hook complete"
