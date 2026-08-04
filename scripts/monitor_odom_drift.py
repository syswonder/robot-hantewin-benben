#!/usr/bin/env python3
"""Monitor odometry drift by comparing raw odom vs rtabmap-corrected pose.

Shows three values:
  odom_pos  : raw odometry position (odom -> base_link)
  map_pos   : rtabmap-corrected position (map -> base_link)
  drift     : difference = map_pos - odom_pos (translation + rotation)

The drift is exactly the map -> odom TF transform that rtabmap publishes.
Large or growing drift means the wheel odometry is accumulating error.

Usage:
  python3 monitor_odom_drift.py [update_hz]

  default update_hz = 2.0
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomDriftMonitor(Node):
    def __init__(self, hz: float):
        super().__init__("odom_drift_monitor")
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._odom_x = None
        self._odom_y = None
        self._odom_yaw = None
        self._hz = hz
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_timer(1.0 / hz, self._tick)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _tick(self) -> None:
        # map -> odom (drift correction applied by rtabmap)
        drift = self._lookup("map", "odom")
        # odom -> base_link (raw odometry)
        odom_tf = self._lookup("odom", "base_link")

        if drift is None and odom_tf is None:
            print("\r  waiting for TF data...", end="", flush=True)
            return

        parts = []
        if odom_tf is not None:
            tx, ty = odom_tf.translation.x, odom_tf.translation.y
            q = odom_tf.rotation
            yaw = math.degrees(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            ))
            parts.append(f"odom_pos=({tx:+.2f}, {ty:+.2f}) {yaw:+.1f}°")
        else:
            parts.append("odom_pos=<unavailable>")

        if drift is not None:
            dx, dy = drift.translation.x, drift.translation.y
            dist = math.sqrt(dx * dx + dy * dy)
            q = drift.rotation
            dyaw = math.degrees(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            ))
            parts.append(f"drift=({dx:+.2f}, {dy:+.2f}) {dyaw:+.1f}° |{dist:.2f}m|")
        else:
            parts.append("drift=<map->odom not available, rtabmap may not be running>")

        print("\r  " + "  ".join(parts) + "    ", end="", flush=True)

    def _lookup(self, target: str, source: str):
        try:
            return self._tf_buf.lookup_transform(
                target, source, rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            ).transform
        except Exception:
            return None


def main():
    hz = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    rclpy.init()
    node = OdomDriftMonitor(hz)
    print(f"Monitoring odom drift @ {hz:.1f} Hz  (Ctrl+C to stop)\n")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()