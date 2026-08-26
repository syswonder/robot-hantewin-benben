#!/usr/bin/env python3
"""Localization drift analyzer for the BenBen navigation problem.

Reads raw ROS topic captures produced by diag_localization.sh and prints a
verdict on whether RTAB-Map localization is actually correcting wheel odometry.

The decisive test is translation, not rotation:
  * odom -> base_link  : dead-reckoned wheel odometry (drifts on skid-steer).
  * map  -> odom       : RTAB-Map's localization correction.
If the robot translates several meters but map->odom stays frozen, localization
is not correcting odometry and the robot is navigating on dead reckoning.

Pure stdlib - no rclpy needed, re-runnable on saved captures.
"""
from __future__ import annotations

import math
import re
import sys
from pathlib import Path


def yaw_of(z: float, w: float) -> float:
    # quaternion x=y=0 (2D TF / odom in planar robots)
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * (z * z))


# ── /tf parser ────────────────────────────────────────────────────────────────
def parse_tf(text: str):
    """Yield (frame_id, child_frame_id, x, y, yaw) for every transform."""
    # Each ROS message is separated by '---'.  A TFMessage may contain several
    # transforms; split on the '- header:' stanza instead.
    for block in re.split(r"\n---\n", text):
        for tfm in re.finditer(
            r"- header:.*?frame_id:\s*(\w+)\s*\n"
            r"\s*child_frame_id:\s*(\w+)\s*\n"
            r".*?translation:\s*\n\s*x:\s*(-?[\d.eE+-]+)\s*\n"
            r"\s*y:\s*(-?[\d.eE+-]+)\s*\n"
            r"\s*z:\s*(-?[\d.eE+-]+)\s*\n"
            r".*?rotation:\s*\n\s*x:\s*(-?[\d.eE+-]+)\s*\n"
            r"\s*y:\s*(-?[\d.eE+-]+)\s*\n"
            r"\s*z:\s*(-?[\d.eE+-]+)\s*\n"
            r"\s*w:\s*(-?[\d.eE+-]+)",
            block,
            re.S,
        ):
            fid, cid = tfm.group(1), tfm.group(2)
            x, y = float(tfm.group(3)), float(tfm.group(4))
            z, w = float(tfm.group(8)), float(tfm.group(9))
            yield fid, cid, x, y, yaw_of(z, w)


# ── /odom parser ──────────────────────────────────────────────────────────────
def parse_odom(text: str):
    """Yield (x, y, yaw, linear_x, angular_z) per Odometry message."""
    for block in re.split(r"\n---\n", text):
        m = re.search(
            r"position:\s*\n\s*x:\s*(-?[\d.eE+-]+)\s*\n"
            r"\s*y:\s*(-?[\d.eE+-]+)\s*\n",
            block,
        )
        o = re.search(
            r"orientation:\s*\n\s*x:\s*-?[\d.eE+-]+\s*\n"
            r"\s*y:\s*-?[\d.eE+-]+\s*\n"
            r"\s*z:\s*(-?[\d.eE+-]+)\s*\n"
            r"\s*w:\s*(-?[\d.eE+-]+)",
            block,
        )
        lx = re.search(r"linear:\s*\n\s*x:\s*(-?[\d.eE+-]+)", block)
        az = re.search(
            r"angular:\s*\n\s*x:\s*-?[\d.eE+-]+\s*\n"
            r"\s*y:\s*-?[\d.eE+-]+\s*\n"
            r"\s*z:\s*(-?[\d.eE+-]+)",
            block,
        )
        if not (m and o):
            continue
        x, y = float(m.group(1)), float(m.group(2))
        yaw = yaw_of(float(o.group(1)), float(o.group(2)))
        lxv = float(lx.group(1)) if lx else 0.0
        azv = float(az.group(1)) if az else 0.0
        yield x, y, yaw, lxv, azv


def path_length(pts):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])
    )


def fmt(v, d=3):
    return f"{v: .{d}f}"


def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/robonix-diag")
    tf_file = out / "tf.txt"
    odom_file = out / "odom.txt"
    cmd_file = out / "cmd_vel.txt"

    print("=" * 72)
    print("Localization drift analysis")
    print("=" * 72)

    # ── TF ─────────────────────────────────────────────────────────────────────
    m2o, o2b = [], []
    if tf_file.exists():
        for fid, cid, x, y, yaw in parse_tf(tf_file.read_text(errors="replace")):
            if fid == "map" and cid == "odom":
                m2o.append((x, y, yaw))
            elif fid == "odom" and cid == "base_link":
                o2b.append((x, y, yaw))
    print(f"\n[tf] map->odom samples      : {len(m2o)}")
    print(f"[tf] odom->base_link samples: {len(o2b)}")

    # ── odom ──────────────────────────────────────────────────────────────────
    odom = []
    if odom_file.exists():
        odom = list(parse_odom(odom_file.read_text(errors="replace")))
    print(f"[odom] samples               : {len(odom)}")

    if not m2o or not o2b:
        print("\n[!] 没有采集到足够的 TF 数据。确认 ROS2 环境已 source、/tf 正在发布。")
        return 2

    # distance the robot actually travelled (dead-reckoned odom translation)
    o2b_pos = [(x, y) for x, y, _ in o2b]
    odom_travel = path_length(o2b_pos)
    odom_net = math.hypot(
        o2b_pos[-1][0] - o2b_pos[0][0], o2b_pos[-1][1] - o2b_pos[0][1]
    )

    # how much map->odom changed over the whole session
    m2o_dx = m2o[-1][0] - m2o[0][0]
    m2o_dy = m2o[-1][1] - m2o[0][1]
    m2o_dist = math.hypot(m2o_dx, m2o_dy)
    m2o_dyaw = abs(
        math.atan2(math.sin(m2o[-1][2] - m2o[0][2]), math.cos(m2o[-1][2] - m2o[0][2]))
    )

    print("\n" + "-" * 72)
    print("Movement summary")
    print("-" * 72)
    print(f"odom translation travelled  : {fmt(odom_travel)} m (path length)")
    print(f"odom net displacement       : {fmt(odom_net)} m")
    print(f"odom start -> end           : {fmt(o2b_pos[0][0])},{fmt(o2b_pos[0][1])} -> {fmt(o2b_pos[-1][0])},{fmt(o2b_pos[-1][1])}")
    print(f"map->odom translation change: {fmt(m2o_dist)} m")
    print(f"map->odom yaw change        : {fmt(math.degrees(m2o_dyaw))} deg")
    print(f"map->odom start             : {fmt(m2o[0][0])},{fmt(m2o[0][1])} yaw={fmt(math.degrees(m2o[0][2]))}")
    print(f"map->odom end               : {fmt(m2o[-1][0])},{fmt(m2o[-1][1])} yaw={fmt(math.degrees(m2o[-1][2]))}")

    # commanded velocity range
    if cmd_file.exists():
        lx, az = [], []
        for block in re.split(r"\n---\n", cmd_file.read_text(errors="replace")):
            m = re.search(r"linear:\s*\n\s*x:\s*(-?[\d.eE+-]+)", block)
            a = re.search(r"angular:\s*\n\s*x:\s*-?[\d.eE+-]+\s*\n\s*y:\s*-?[\d.eE+-]+\s*\n\s*z:\s*(-?[\d.eE+-]+)", block, re.S)
            if m:
                lx.append(float(m.group(1)))
            if a:
                az.append(float(a.group(1)))
        if lx or az:
            print(f"cmd_vel linear.x range      : {fmt(min(lx))} .. {fmt(max(lx))}")
            print(f"cmd_vel angular.z range     : {fmt(min(az))} .. {fmt(max(az))}")

    # ── verdict ────────────────────────────────────────────────────────────────
    MIN_TRAVEL = 0.5   # metres the robot must move for a meaningful test
    FROZEN_DIST = 0.03  # map->odom translation change below this => frozen
    FROZEN_YAW = math.radians(1.0)

    print("\n" + "=" * 72)
    print("VERDICT")
    print("=" * 72)

    if odom_travel < MIN_TRAVEL:
        print(f"[!] 机器人平移距离 {fmt(odom_travel)} m < {MIN_TRAVEL} m，测试无效。")
        print("    请让机器人实际前进/后退累计 0.5 米以上再测（如前进 0.5m + 后退 0.5m）。")
        print("    纯原地旋转测不出定位漂移。")
        return 3

    frozen = m2o_dist < FROZEN_DIST and m2o_dyaw < FROZEN_YAW
    if frozen:
        print("❌ 定位没有在纠正里程计（map->odom 冻结）。")
        print("   机器人平移了 %.2f m，但 map->odom 变换几乎不变（位移 %.3f m，%.2f°）。" % (
            odom_travel, m2o_dist, math.degrees(m2o_dyaw)))
        print("   机器人的地图位姿 = 纯轮式里程计 + 一个固定偏移 → 打滑即漂移。")
        print("   → 这就是『轨迹正常但乱走/绕路/撞墙』的根因。")
        print("   下一步：检查 mapping 日志是否每秒报『Missing visual features』。")
    else:
        print("✅ 定位在纠正里程计（map->odom 随运动更新）。")
        print("   若仍乱走，问题不在定位，请转向底盘/控制器执行链路排查。")

    return 0 if frozen else 0


if __name__ == "__main__":
    raise SystemExit(main())
