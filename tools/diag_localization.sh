#!/usr/bin/env bash
# One-command localization diagnosis for the BenBen navigation problem.
#
# Captures /tf, /odom and /cmd_vel while you move the robot, then runs
# analyze_localization.py to decide whether RTAB-Map localization is actually
# correcting wheel odometry.
#
# Usage:
#   bash tools/diag_localization.sh [seconds]   # default 40 s
#
# What to do during the capture:
#   1. 脚本会等你 3 秒让你准备。
#   2. 让机器人【前进 0.5 米】，然后【原地转 90°】，再【后退 0.5 米】，停下。
#      （关键是要有【平移】——纯原地旋转测不出定位漂移。小空间也能做。）
#   3. 脚本到时会自动结束并打印判定。
# NOTE: do NOT use `set -u` here — it breaks `source /opt/ros/jazzy/setup.bash`
# (AMENT_TRACE_SETUP_FILES is unset in a fresh shell) and silently aborts the
# script before the first echo, which looks like "no output".

SECONDS_="${1:-40}"
OUT="/tmp/robonix-diag"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash 2>/dev/null

rm -rf "$OUT"; mkdir -p "$OUT"
echo "采集目录: $OUT"

# record a baseline of the mapping log so we can count rtabmap localization
# failures that happen during this exact window.
MAPPING_LOG="${MAPPING_LOG:-/home/hty/robot-hantewin-benben/rbnx-boot/logs/mapping.log}"
before=$(wc -l < "$MAPPING_LOG" 2>/dev/null || echo 0)

echo
echo "=============================================================="
echo "  3 秒后开始采集 ${SECONDS_} 秒。请在采集期间移动机器人："
echo "    前进 0.5 米 → 原地转 90° → 后退 0.5 米 → 停下"
echo "  （必须有平移，纯旋转测不出定位漂移；小空间来回动即可）"
echo "=============================================================="
sleep 3

echo "采集开始: $(date '+%H:%M:%S')"
timeout "$SECONDS_" ros2 topic echo /tf                    > "$OUT/tf.txt"    2>/dev/null &
timeout "$SECONDS_" ros2 topic echo /odom                  > "$OUT/odom.txt"  2>/dev/null &
timeout "$SECONDS_" ros2 topic echo /cmd_vel               > "$OUT/cmd_vel.txt" 2>/dev/null &
timeout "$SECONDS_" ros2 topic echo /collision_monitor_state > "$OUT/collision.txt" 2>/dev/null &

wait
echo "采集结束: $(date '+%H:%M:%S')"

# count rtabmap localization failures inside this window
after=$(wc -l < "$MAPPING_LOG" 2>/dev/null || echo "$before")
if [ -n "${MAPPING_LOG:-}" ] && [ -f "$MAPPING_LOG" ]; then
    tail -n $((after - before)) "$MAPPING_LOG" 2>/dev/null \
        | grep -c "Missing visual features" > "$OUT/missing_visual_count.txt" || true
    echo
    echo "本次窗口内 rtabmap 'Missing visual features' 次数: $(cat "$OUT/missing_visual_count.txt" 2>/dev/null || echo 0)"
fi

echo
python3 "$HERE/analyze_localization.py" "$OUT"
