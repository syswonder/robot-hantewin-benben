"""Return-home motion sequence (offline grasp_test relay->home flow)."""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Protocol

from .config import ReturnHomeConfig

if TYPE_CHECKING:
    from .arm_client import ArmClient

log = logging.getLogger("nero_return_home.sequence")


class ArmDriver(Protocol):
    provider_id: str

    def wait_for_feedback(self, timeout_s: float) -> None: ...
    def move_joints(self, joints: list[float], *, timeout_s: float | None = None) -> None: ...
    def set_gripper(self, width_m: float, force_n: float) -> None: ...


def _pause_after_step(cfg: ReturnHomeConfig, arm_id: str, step: str) -> None:
    if cfg.step_pause_s <= 0:
        return
    log.info("[%s] pause %.1fs after %s", arm_id, cfg.step_pause_s, step)
    time.sleep(cfg.step_pause_s)


def _run_joint_pose_sequence(
    arm: ArmDriver,
    cfg: ReturnHomeConfig,
    poses: list[list[float]],
    *,
    label: str,
) -> None:
    if not poses:
        return
    total = len(poses)
    for index, joints in enumerate(poses, start=1):
        log.info(
            "[%s] %s[%d/%d] -> %s",
            arm.provider_id,
            label,
            index,
            total,
            [round(v, 3) for v in joints],
        )
        arm.move_joints(joints, timeout_s=cfg.motion_timeout_s)
        _pause_after_step(cfg, arm.provider_id, f"{label} {index}/{total}")


def _run_relay_poses(arm: ArmDriver, cfg: ReturnHomeConfig) -> None:
    _run_joint_pose_sequence(arm, cfg, cfg.relay_joint_poses, label="relay")


def _move_to_standby_pose(arm: ArmDriver, cfg: ReturnHomeConfig) -> None:
    if cfg.standby_joint_pose is None:
        return
    log.info(
        "[%s] standby_joint_pose -> %s",
        arm.provider_id,
        [round(v, 3) for v in cfg.standby_joint_pose],
    )
    arm.move_joints(cfg.standby_joint_pose, timeout_s=cfg.motion_timeout_s)
    _pause_after_step(cfg, arm.provider_id, "standby")


def _run_standby_exit_poses(arm: ArmDriver, cfg: ReturnHomeConfig) -> None:
    if not cfg.standby_exit_joint_poses:
        return
    _run_joint_pose_sequence(arm, cfg, cfg.standby_exit_joint_poses, label="standby_exit")


def _move_to_home(arm: ArmDriver, cfg: ReturnHomeConfig) -> None:
    log.info(
        "[%s] home_joint_pose -> %s",
        arm.provider_id,
        [round(v, 3) for v in cfg.home_joint_pose],
    )
    arm.move_joints(cfg.home_joint_pose, timeout_s=cfg.motion_timeout_s)


def _move_relay_to_home(arm: ArmDriver, cfg: ReturnHomeConfig) -> None:
    """relay sequence -> home without step pause before home segment."""
    _run_relay_poses(arm, cfg)
    log.info(
        "[%s] home_joint_pose -> %s (continuous from relay)",
        arm.provider_id,
        [round(v, 3) for v in cfg.home_joint_pose],
    )
    arm.move_joints(cfg.home_joint_pose, timeout_s=cfg.motion_timeout_s)


def _run_return_motion(arm: ArmDriver, cfg: ReturnHomeConfig) -> None:
    if cfg.start_from == "current":
        log.info("[%s] start_from=current — skip relay, move to home", arm.provider_id)
        _move_to_home(arm, cfg)
        return

    if cfg.start_from == "standby":
        log.info("[%s] start_from=standby — sync standby, optional exit, then relay", arm.provider_id)
        _move_to_standby_pose(arm, cfg)
        _run_standby_exit_poses(arm, cfg)
        _move_relay_to_home(arm, cfg)
        return

    log.info("[%s] start_from=relay — relay then home", arm.provider_id)
    _move_relay_to_home(arm, cfg)


def _run_single_arm_return_home(arm: ArmDriver, cfg: ReturnHomeConfig) -> None:
    # 1) Release gripper, wait.
    log.info("[%s] open gripper (release)", arm.provider_id)
    arm.set_gripper(cfg.gripper_open_width_m, cfg.gripper_force_n)
    if cfg.open_gripper_wait_s > 0:
        log.info("[%s] wait %.1fs after open gripper", arm.provider_id, cfg.open_gripper_wait_s)
        time.sleep(cfg.open_gripper_wait_s)

    # 2) Optionally close gripper before moving.
    if cfg.close_gripper:
        log.info("[%s] close gripper before return", arm.provider_id)
        arm.set_gripper(cfg.gripper_close_width_m, cfg.gripper_force_n)

    # 3) Return motion per start_from.
    _run_return_motion(arm, cfg)
    _pause_after_step(cfg, arm.provider_id, "home")


def run_return_home(arms: list[ArmClient], cfg: ReturnHomeConfig) -> None:
    if not arms:
        raise RuntimeError("no arm clients configured")

    for arm in arms:
        arm.wait_for_feedback(timeout_s=cfg.motion_timeout_s)

    if cfg.execution_mode == "dual":
        for arm in arms:
            _run_single_arm_return_home(arm, cfg)
        return

    if len(arms) != 1:
        raise RuntimeError(
            f"execution_mode=single expects one arm, got {len(arms)}: "
            f"{[a.provider_id for a in arms]}"
        )
    _run_single_arm_return_home(arms[0], cfg)
