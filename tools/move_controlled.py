#!/usr/bin/env python3
"""Controlled chassis motion for the localization diagnostic, via the raw UDP
channel (same mechanism as ~/Desktop/htysdk/examples/udp_vel.py).

This bypasses Nav2 and the chassis primitive entirely, so it works even when
localization is broken. It drives the exact maneuver the diag script needs:

    forward 0.5 m  ->  rotate 90 deg (left)  ->  back 0.5 m

Protocol (mirrors udp_vel_server.py):
    9 x float64 + 1 x int32, little-endian, sent to 192.168.10.1:11451.
    type=0  single command: forward_m / rotate_deg are self-contained
            (the chassis-side server drives for the right duration then stops).

Usage:
    python3 tools/move_controlled.py            # full maneuver
    python3 tools/move_controlled.py --test     # tiny 0.2 m sanity check first
    python3 tools/move_controlled.py --forward 0.3 --angle 90 --back 0.3

Safety: run --test first and watch which way the robot goes. If forward_m>0
moves it BACKWARD on your chassis, pass --flip to invert the linear sign.
"""
from __future__ import annotations

import argparse
import socket
import struct
import time

HOST = "192.168.10.1"
PORT = 11451


def send_cmd(cmd: list[float]) -> None:
    if len(cmd) != 10:
        raise ValueError("command needs 10 numbers")
    # 9 float64 (72 bytes) + 1 int32 (4 bytes) = 76 bytes
    data = struct.pack("<9di", *cmd)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(data, (HOST, PORT))
    sock.close()


def single_forward(distance_m: float, flip: bool) -> None:
    sign = -1.0 if flip else 1.0
    # type=0, forward_m = signed distance. Server uses 0.5 m/s default speed.
    send_cmd([0.0, 0, 0, 0, 0, 0, 0.0, sign * distance_m, 0.0, 0])


def single_rotate(angle_deg: float, flip: bool) -> None:
    sign = -1.0 if flip else 1.0
    # type=0, rotate_deg = signed angle in DEGREES (server uses 0.5 rad/s).
    send_cmd([0.0, 0, 0, 0, 0, 0, 0.0, 0.0, sign * angle_deg, 0])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--forward", type=float, default=0.5, help="forward distance (m)")
    ap.add_argument("--angle", type=float, default=90.0, help="rotate angle (deg, + = left)")
    ap.add_argument("--back", type=float, default=0.5, help="backward distance (m)")
    ap.add_argument("--test", action="store_true", help="tiny 0.2 m forward sanity check")
    ap.add_argument("--flip", action="store_true", help="invert linear direction")
    ap.add_argument("--no-back", action="store_true", help="skip the backward leg")
    args = ap.parse_args()

    if args.test:
        print(f"== 测试：前进 0.2 m (flip={args.flip}) ==")
        single_forward(0.2, args.flip)
        print("   已发送。观察方向：前进=方向对；后退=加 --flip 再测。")
        return 0

    print("== 受控动作序列 ==")
    print(f"   前进 {args.forward} m")
    single_forward(args.forward, args.flip)
    # forward at 0.5 m/s -> duration = dist / 0.5, plus 0.3 s settle
    time.sleep(args.forward / 0.5 + 0.4)

    print(f"   原地转 {args.angle} 度")
    single_rotate(args.angle, args.flip)
    # rotate at 0.5 rad/s -> duration = rad / 0.5, plus settle
    import math
    time.sleep(math.radians(abs(args.angle)) / 0.5 + 0.4)

    if not args.no_back:
        print(f"   后退 {args.back} m")
        single_forward(-args.back, args.flip)
        time.sleep(args.back / 0.5 + 0.4)

    print("== 完成，已停车 ==")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
