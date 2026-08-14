"""Minimal geometry helpers for Nero arm <-> ROS Pose conversion."""
from __future__ import annotations

import math


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def rpy_pose_to_ros_pose(pose_rpy: list[float]):
    from geometry_msgs.msg import Pose  # type: ignore

    x, y, z, roll, pitch, yaw = pose_rpy
    qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)
    msg = Pose()
    msg.position.x = float(x)
    msg.position.y = float(y)
    msg.position.z = float(z)
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    return msg


def ros_pose_to_rpy(pose) -> list[float]:
    roll, pitch, yaw = quaternion_to_rpy(
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )
    return [
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        roll,
        pitch,
        yaw,
    ]
