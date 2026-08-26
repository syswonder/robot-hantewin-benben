"""Map detected camera targets to arm TCP poses (translation only)."""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from .config import GraspConfig

log = logging.getLogger("nero_grasp.target")


@dataclass(frozen=True)
class GraspTarget:
    """Grasp point in robot-base frame, optionally with source camera coords."""

    robot_xyz: list[float]
    camera_xyz: list[float] | None = None
    label: str = "object"
    confidence: float = 1.0


class ArmPoseReader(Protocol):
    provider_id: str

    def get_tcp_pose_rpy(self) -> list[float]: ...


def apply_robot_offset(xyz: list[float], offset: list[float] | None) -> list[float]:
    if not offset:
        return [float(v) for v in xyz]
    return [float(xyz[i]) + float(offset[i]) for i in range(3)]


def resolve_grasp_tcp_pose(
    arm: ArmPoseReader,
    cfg: GraspConfig,
    target: GraspTarget | None,
) -> list[float]:
    """Build 6D TCP command: detected/base XYZ + current roll/pitch/yaw.

    Rotation is taken from the arm's present pose (after initial_joint_pose),
    so only translation changes during the grasp approach.
    """
    current = arm.get_tcp_pose_rpy()
    roll, pitch, yaw = current[3], current[4], current[5]

    if target is not None:
        xyz = apply_robot_offset(target.robot_xyz, cfg.grasp_offset_m)
        log.info(
            "[%s] grasp target robot xyz -> %s (offset=%s)",
            arm.provider_id,
            [round(v, 4) for v in xyz],
            cfg.grasp_offset_m,
        )
        if target.camera_xyz is not None:
            log.info(
                "[%s] source camera xyz -> %s",
                arm.provider_id,
                [round(v, 4) for v in target.camera_xyz],
            )
        return [xyz[0], xyz[1], xyz[2], roll, pitch, yaw]

    grasp_pose = list(current)
    # TCP Y axis = backward; decreasing Y moves toward the object in front.
    grasp_pose[1] -= cfg.grasp_forward_offset_m
    log.info(
        "[%s] fallback grasp pose y=%.3f -> %.3f (no detection)",
        arm.provider_id,
        current[1],
        grasp_pose[1],
    )
    return grasp_pose


def resolve_fixed_grasp_tcp_pose(
    arm: ArmPoseReader,
    cfg: GraspConfig,
) -> list[float]:
    """Build 6D TCP command from manifest fixed_tcp_pose (+ optional offset on xyz)."""
    if cfg.fixed_tcp_pose is None:
        raise RuntimeError("fixed_tcp_pose is not configured")
    if len(cfg.fixed_tcp_pose) not in {3, 6}:
        raise RuntimeError(f"fixed_tcp_pose must have 3 or 6 values, got {len(cfg.fixed_tcp_pose)}")

    pose = [float(v) for v in cfg.fixed_tcp_pose]
    xyz = apply_robot_offset(pose[:3], cfg.grasp_offset_m)
    if len(pose) == 6:
        roll, pitch, yaw = pose[3], pose[4], pose[5]
    else:
        current = arm.get_tcp_pose_rpy()
        roll, pitch, yaw = current[3], current[4], current[5]

    log.info(
        "[%s] fixed grasp tcp -> %s (offset=%s)",
        arm.provider_id,
        [round(v, 4) for v in xyz + [roll, pitch, yaw]],
        cfg.grasp_offset_m,
    )
    return [xyz[0], xyz[1], xyz[2], roll, pitch, yaw]


def resolve_lift_tcp_pose(grasp_pose: list[float], lift_offset_m: float) -> list[float]:
    lift_pose = list(grasp_pose)
    # TCP X axis = up.
    lift_pose[0] += lift_offset_m
    return lift_pose
