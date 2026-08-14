"""Hard-coded Nero grasp motion sequence for Robonix skill."""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Protocol

from .camera_client import CameraClient
from .config import GraspConfig
from .target import resolve_fixed_grasp_tcp_pose, resolve_grasp_tcp_pose, resolve_lift_tcp_pose

if TYPE_CHECKING:
    from .arm_client import ArmClient

log = logging.getLogger("nero_grasp.sequence")

_ENABLE_GRIPPER = True


class ArmDriver(Protocol):
    provider_id: str

    def wait_for_feedback(self, timeout_s: float) -> None: ...
    def move_joints(self, joints: list[float], *, timeout_s: float | None = None) -> None: ...
    def get_tcp_pose_rpy(self) -> list[float]: ...
    def move_tcp_linear_rpy(self, pose_rpy: list[float], *, timeout_s: float | None = None) -> None: ...
    def set_gripper(self, width_m: float, force_n: float) -> None: ...


def _pause_after_step(cfg: GraspConfig, arm_id: str, step: str) -> None:
    if cfg.step_pause_s <= 0:
        return
    log.info("[%s] pause %.1fs after %s", arm_id, cfg.step_pause_s, step)
    time.sleep(cfg.step_pause_s)


def _run_relay_poses(arm: ArmDriver, cfg: GraspConfig) -> None:
    if not cfg.relay_joint_poses:
        return
    total = len(cfg.relay_joint_poses)
    for index, joints in enumerate(cfg.relay_joint_poses, start=1):
        log.info(
            "[%s] relay[%d/%d] -> %s",
            arm.provider_id,
            index,
            total,
            [round(v, 3) for v in joints],
        )
        arm.move_joints(joints, timeout_s=cfg.motion_timeout_s)
        _pause_after_step(cfg, arm.provider_id, f"relay {index}/{total}")


def _move_to_initial_pose(arm: ArmDriver, cfg: GraspConfig , relay: bool = True) -> None:
    if cfg.initial_joint_pose is None:
        log.info("[%s] keep current joints as initial pose", arm.provider_id)
        return
    if relay:
        _run_relay_poses(arm, cfg)
    log.info(
        "[%s] initial_joint_pose -> %s (continuous from relay)",
        arm.provider_id,
        [round(v, 3) for v in cfg.initial_joint_pose],
    )
    arm.move_joints(cfg.initial_joint_pose, timeout_s=cfg.motion_timeout_s)
    _pause_after_step(cfg, arm.provider_id, "initial pose")


def _open_gripper(arm: ArmDriver, cfg: GraspConfig) -> None:
    if not _ENABLE_GRIPPER:
        log.info("[%s] skip open gripper (effector disabled)", arm.provider_id)
        return
    log.info("[%s] open gripper", arm.provider_id)
    arm.set_gripper(cfg.gripper_open_width_m, cfg.gripper_force_n)
    _pause_after_step(cfg, arm.provider_id, "open gripper")


def _close_gripper(arm: ArmDriver, cfg: GraspConfig) -> None:
    if not _ENABLE_GRIPPER:
        log.info("[%s] skip close gripper (effector disabled)", arm.provider_id)
        return
    log.info("[%s] close gripper", arm.provider_id)
    arm.set_gripper(cfg.gripper_close_width_m, cfg.gripper_force_n)
    _pause_after_step(cfg, arm.provider_id, "close gripper")


def run_return_initial(arm: ArmDriver, cfg: GraspConfig , relay: bool = True) -> None:
    if not cfg.return_initial:
        return
    _move_to_initial_pose(arm, cfg, relay)


def _move_to_standby_pose(arm: ArmDriver, cfg: GraspConfig) -> None:
    if cfg.standby_joint_pose is None:
        return
    log.info(
        "[%s] standby_joint_pose -> %s",
        arm.provider_id,
        [round(v, 3) for v in cfg.standby_joint_pose],
    )
    arm.move_joints(cfg.standby_joint_pose, timeout_s=cfg.motion_timeout_s)
    _pause_after_step(cfg, arm.provider_id, "standby pose")


def recover_after_failure(arm: ArmDriver, cfg: GraspConfig) -> None:
    """Best-effort retreat to initial_joint_pose only; never relay->home, never disable."""
    if not cfg.recover_on_failure:
        log.warning("[%s] grasp failed with recovery disabled — arm left at last pose", arm.provider_id)
        return

    log.warning("[%s] grasp failed — attempting recovery to initial_joint_pose", arm.provider_id)

    try:
        _open_gripper(arm, cfg)
    except Exception as exc:  # noqa: BLE001
        log.warning("[%s] open gripper during recovery failed: %s", arm.provider_id, exc)

    if cfg.initial_joint_pose is None:
        log.warning("[%s] no initial_joint_pose configured; staying at current pose", arm.provider_id)
        return

    try:
        _move_to_initial_pose(arm, cfg, relay=False)
        log.info("[%s] failure recovery reached initial_joint_pose", arm.provider_id)
    except Exception as exc:  # noqa: BLE001
        log.warning(
            "[%s] recovery to initial_joint_pose failed (%s) — staying at current pose",
            arm.provider_id,
            exc,
        )


def _resolve_grasp_pose(
    arm: ArmDriver,
    cfg: GraspConfig,
    camera: CameraClient | None,
    *,
    object_name: str | None = None,
):
    if cfg.target_source == "fixed_tcp":
        return resolve_fixed_grasp_tcp_pose(arm, cfg)

    if camera is None:
        raise RuntimeError("camera client is required for target_source=vlm")
    target = camera.detect_object(object_name=object_name)
    if cfg.require_detection and target is None:
        raise RuntimeError("object detection required but no target was returned")
    return resolve_grasp_tcp_pose(arm, cfg, target)


def _run_single_arm_grasp(
    arm: ArmDriver,
    cfg: GraspConfig,
    camera: CameraClient | None,
    *,
    object_name: str | None = None,
) -> None:
    timeout = cfg.motion_timeout_s

    try:
        # 1) Observation pose + open gripper (standby).
        _move_to_initial_pose(arm, cfg)
        _open_gripper(arm, cfg)

        # 2) Resolve grasp TCP: VLM detection or fixed pose from config.
        grasp_pose = _resolve_grasp_pose(arm, cfg, camera, object_name=object_name)
        log.info(
            "[%s] linear approach grasp pose -> %s (target_source=%s)",
            arm.provider_id,
            [round(v, 3) for v in grasp_pose],
            cfg.target_source,
        )
        arm.move_tcp_linear_rpy(grasp_pose, timeout_s=timeout)
        _pause_after_step(cfg, arm.provider_id, "grasp approach")

        # 3) Close gripper and return to initial pose.
        _close_gripper(arm, cfg)

        if cfg.lift_offset_m > 0:
            current = arm.get_tcp_pose_rpy()
            lift_pose = [current[0] + cfg.lift_offset_m, 0.3426, 0.2918, current[3], current[4], current[5]]
            arm.move_tcp_linear_rpy(lift_pose, timeout_s=timeout)
            _pause_after_step(cfg, arm.provider_id, "lift")

        run_return_initial(arm, cfg, relay=False)
        _move_to_standby_pose(arm, cfg)
    except Exception as grasp_exc:
        recover_after_failure(arm, cfg)
        raise grasp_exc


def run_grasp(
    arms: list[ArmClient],
    cfg: GraspConfig,
    camera: CameraClient | None,
    *,
    object_name: str | None = None,
) -> None:
    if not arms:
        raise RuntimeError("no arm clients configured")

    for arm in arms:
        arm.wait_for_feedback(timeout_s=cfg.motion_timeout_s)

    if cfg.execution_mode == "dual":
        for arm in arms:
            _run_single_arm_grasp(arm, cfg, camera, object_name=object_name)
        return

    if len(arms) != 1:
        raise RuntimeError(
            f"execution_mode=single expects one arm, got {len(arms)}: "
            f"{[a.provider_id for a in arms]}"
        )
    _run_single_arm_grasp(arms[0], cfg, camera, object_name=object_name)
