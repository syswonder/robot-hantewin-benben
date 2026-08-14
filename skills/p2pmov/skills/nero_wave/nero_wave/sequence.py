"""Automated wave motion sequence for Robonix skill."""
from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Protocol

from .config import MotionStep, WaveConfig

if TYPE_CHECKING:
    from .arm_client import ArmClient

log = logging.getLogger("nero_wave.sequence")


class ArmDriver(Protocol):
    provider_id: str

    def wait_for_feedback(self, timeout_s: float) -> None: ...
    def move_joints(self, joints: list[float], *, timeout_s: float | None = None) -> None: ...
    def get_tcp_pose_rpy(self) -> list[float]: ...
    def move_tcp_rpy(self, pose_rpy: list[float], *, timeout_s: float | None = None) -> None: ...
    def move_tcp_linear_rpy(self, pose_rpy: list[float], *, timeout_s: float | None = None) -> None: ...


def _pause_after_step(cfg: WaveConfig, arm_id: str, step: str) -> None:
    if cfg.step_pause_s <= 0:
        return
    log.info("[%s] pause %.1fs after %s", arm_id, cfg.step_pause_s, step)
    time.sleep(cfg.step_pause_s)


def _resolve_tcp_target(arm: ArmDriver, step: MotionStep) -> list[float]:
    if len(step.values) == 6:
        return list(step.values)
    if len(step.values) != 3:
        raise ValueError(f"tcp step {step.name!r} expects 3 or 6 values, got {len(step.values)}")
    current = arm.get_tcp_pose_rpy()
    return [float(step.values[i]) for i in range(3)] + list(current[3:])


def _use_linear_move(step: MotionStep, cfg: WaveConfig) -> bool:
    return step.linear or cfg.default_tcp_linear


def _execute_step(arm: ArmDriver, cfg: WaveConfig, step: MotionStep) -> None:
    timeout = cfg.motion_timeout_s
    if step.kind == "joint":
        log.info("[%s] step %r: move_j -> %s", arm.provider_id, step.name, [round(v, 3) for v in step.values])
        arm.move_joints(step.values, timeout_s=timeout)
    else:
        target = _resolve_tcp_target(arm, step)
        if _use_linear_move(step, cfg):
            log.info("[%s] step %r: move_l -> %s", arm.provider_id, step.name, [round(v, 3) for v in target])
            arm.move_tcp_linear_rpy(target, timeout_s=timeout)
        else:
            log.info("[%s] step %r: move_p -> %s", arm.provider_id, step.name, [round(v, 3) for v in target])
            arm.move_tcp_rpy(target, timeout_s=timeout)
    _pause_after_step(cfg, arm.provider_id, step.name)


def _run_single_arm_wave(arm: ArmDriver, cfg: WaveConfig) -> None:
    if cfg.start_at_home:
        log.info(
            "[%s] move home -> %s",
            arm.provider_id,
            [round(v, 3) for v in cfg.home_joint_pose],
        )
        arm.move_joints(cfg.home_joint_pose, timeout_s=cfg.motion_timeout_s)
        _pause_after_step(cfg, arm.provider_id, "home")

    for step in cfg.steps:
        _execute_step(arm, cfg, step)
    
    arm.move_joints(cfg.home_joint_pose, timeout_s=cfg.motion_timeout_s)


def run_wave(arms: list[ArmClient], cfg: WaveConfig) -> None:
    if not arms:
        raise RuntimeError("no arm clients configured")

    for arm in arms:
        arm.wait_for_feedback(timeout_s=cfg.motion_timeout_s)

    if cfg.execution_mode == "dual":
        for arm in arms:
            _run_single_arm_wave(arm, cfg)
        return

    if len(arms) != 1:
        raise RuntimeError(
            f"execution_mode=single expects one arm, got {len(arms)}: "
            f"{[a.provider_id for a in arms]}"
        )
    _run_single_arm_wave(arms[0], cfg)
