"""Closed-loop chassis fine-alignment for fine_align.

The controller is transport-agnostic: it receives three injectable callables
(detect / move / read_odom) so the geometry + loop can be unit-tested without
hardware, while atlas_bridge wires the real MCP / ROS clients.

Control strategy (differential drive, one action per step):
  1. If the object is off to the side beyond ``lateral_tol_m``, rotate in
     place to bring it toward centre (the chassis cannot strafe).
  2. Else if the object is beyond ``forward_tol_m``, drive straight.
Each ``move`` is odom-closed-loop: the actual displacement is measured from
odometry and a bounded residual correction is issued when the burst falls
short of the commanded amount.

Coordinate conventions (all TODO-verify on the physical robot):
  * ``robot_xyz`` from nero_grasp.detect is in the arm's ``robot_base`` frame
    (Nero convention: index 0 = up, 1 = backward, 2 = right).
  * chassis ``move`` operates in ``base_link`` (ROS convention: x = forward).
  * ``base_to_robot_rotation`` maps base_link -> robot_base; the correction
    planner rotates the error back into base_link with its transpose.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Any, Callable

log = logging.getLogger("fine_align.controller")


@dataclass(frozen=True)
class AlignConfig:
    """Deployment-tunable geometry + control parameters."""

    graspable_center: tuple[float, float, float] = (0.0, 0.0, 0.0)          # robot_base frame; TODO 待测
    graspable_tolerance: tuple[float, float, float] = (0.05, 0.05, 0.03)    # per-axis tolerance (m)
    base_to_robot_rotation: tuple[tuple[float, float, float], ...] = (      # base_link -> robot_base; TODO 待测
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    base_to_robot_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)  # TODO 待测（仅记录，误差用不到平移）
    max_iterations: int = 8
    timeout_s: float = 60.0
    forward_gain: float = 1.0          # 修正量 = 前向误差 * gain
    rotate_gain: float = 1.0           # 修正角 = 侧向误差转角 * gain
    forward_step_max_m: float = 0.30   # 单步前向上限
    rotate_step_max_deg: float = 30.0  # 单步旋转上限
    forward_tol_m: float = 0.02        # 前向误差低于此视为到位
    lateral_tol_m: float = 0.02        # 侧向误差低于此视为居中
    move_settle_s: float = 0.5         # move 后等待底盘稳定
    odom_tolerance_m: float = 0.02     # 闭环核对：位移欠量低于此不补走
    odom_yaw_tolerance_deg: float = 3.0


def _rot_transpose(r: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(r[j][i] for j in range(3)) for i in range(3))


def _apply_rot(r: tuple[tuple[float, float, float], ...], v: list[float]) -> list[float]:
    return [r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2] for i in range(3)]


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def in_graspable_zone(robot_xyz: list[float], cfg: AlignConfig) -> bool:
    """True when the object (robot_base frame) sits inside the configured box."""
    for i in range(3):
        if abs(robot_xyz[i] - cfg.graspable_center[i]) > cfg.graspable_tolerance[i]:
            return False
    return True


def plan_correction(robot_xyz: list[float], cfg: AlignConfig) -> tuple[float, float]:
    """Map a robot_base-frame error to a single (forward_m, rotate_deg) action.

    At most one of the two is non-zero (chassis ``move`` applies forward_m
    before rotate_deg). Lateral error is corrected first by rotation, then
    forward error by translation. Pure geometry; sign conventions TODO.
    """
    err_robot = [robot_xyz[i] - cfg.graspable_center[i] for i in range(3)]
    # Error vector is a delta, so the mount translation drops out.
    err_base = _apply_rot(_rot_transpose(cfg.base_to_robot_rotation), err_robot)

    # base_link: x = forward, y = left (ROS). TODO 确认符号。
    forward_err = err_base[0]
    lateral_err = err_base[1]

    if abs(lateral_err) > cfg.lateral_tol_m:
        # 差分底盘不能横移：原地旋转使物体居中。TODO 确认旋转方向符号。
        angle = math.degrees(math.atan2(lateral_err, max(abs(forward_err), 0.1)))
        return 0.0, _clip(angle * cfg.rotate_gain, cfg.rotate_step_max_deg)

    if abs(forward_err) > cfg.forward_tol_m:
        # 物体偏前/偏后：直线前进/后退补偿。TODO 确认符号。
        return _clip(-forward_err * cfg.forward_gain, cfg.forward_step_max_m), 0.0

    return 0.0, 0.0


class FineAlignController:
    """Run the closed alignment loop over injectable I/O callables.

    ``detect_fn(object_name) -> dict`` with keys ``found`` / ``robot_xyz`` /
    ``message``; ``move_fn(forward_m, rotate_deg) -> None``; ``odom_fn() -> dict``
    with keys ``x`` / ``y`` / ``yaw`` (yaw in radians).
    """

    def __init__(
        self,
        cfg: AlignConfig,
        detect_fn: Callable[[str | None], dict],
        move_fn: Callable[[float, float], None],
        odom_fn: Callable[[], dict],
    ) -> None:
        self.cfg = cfg
        self._detect = detect_fn
        self._move = move_fn
        self._odom = odom_fn

    def _move_closed_loop(self, forward_m: float, rotate_deg: float) -> None:
        """Issue one bounded odom-closed-loop move (forward OR rotate)."""
        if forward_m == 0.0 and rotate_deg == 0.0:
            return

        before = self._odom()
        self._move(forward_m, rotate_deg)
        time.sleep(self.cfg.move_settle_s)
        after = self._odom()

        if forward_m != 0.0:
            actual = self._forward_distance(before, after)  # signed, TODO 符号
            residual = forward_m - actual
            if abs(residual) > self.cfg.odom_tolerance_m:
                log.info("closed-loop residual fwd=%.3fm -> compensate", residual)
                self._move(_clip(residual, self.cfg.forward_step_max_m), 0.0)
                time.sleep(self.cfg.move_settle_s)
        elif rotate_deg != 0.0:
            actual = self._yaw_delta(before, after)  # signed radians
            residual_deg = rotate_deg - math.degrees(actual)
            if abs(residual_deg) > self.cfg.odom_yaw_tolerance_deg:
                log.info("closed-loop residual rot=%.1fdeg -> compensate", residual_deg)
                self._move(0.0, _clip(residual_deg, self.cfg.rotate_step_max_deg))
                time.sleep(self.cfg.move_settle_s)

    @staticmethod
    def _forward_distance(before: dict, after: dict) -> float:
        dx = after.get("x", 0.0) - before.get("x", 0.0)
        dy = after.get("y", 0.0) - before.get("y", 0.0)
        return math.hypot(dx, dy)  # magnitude; sign dropped, TODO infer from direction

    @staticmethod
    def _yaw_delta(before: dict, after: dict) -> float:
        return math.remainder(after.get("yaw", 0.0) - before.get("yaw", 0.0), 2.0 * math.pi)

    def align(self, object_name: str | None) -> dict:
        deadline = time.monotonic() + self.cfg.timeout_s
        last_robot_xyz: list[float] = []

        for iterations in range(1, self.cfg.max_iterations + 1):
            if time.monotonic() >= deadline:
                return {"success": False, "iterations": iterations,
                        "robot_xyz": last_robot_xyz, "message": "timeout"}

            det = self._detect(object_name)
            if not det.get("found") or not det.get("robot_xyz"):
                msg = det.get("message") or "detection failed"
                log.warning("align iter %d: %s", iterations, msg)
                return {"success": False, "iterations": iterations,
                        "robot_xyz": last_robot_xyz, "message": f"detect: {msg}"}

            robot_xyz = [float(v) for v in det["robot_xyz"]]
            last_robot_xyz = robot_xyz

            if in_graspable_zone(robot_xyz, self.cfg):
                log.info("align done in %d iters at robot_xyz=%s", iterations,
                         [round(v, 3) for v in robot_xyz])
                return {"success": True, "iterations": iterations,
                        "robot_xyz": robot_xyz, "message": "in graspable zone"}

            forward_m, rotate_deg = plan_correction(robot_xyz, self.cfg)
            log.info("align iter %d: robot_xyz=%s -> move fwd=%.3f rot=%.1f",
                     iterations, [round(v, 3) for v in robot_xyz], forward_m, rotate_deg)
            self._move_closed_loop(forward_m, rotate_deg)

        return {"success": False, "iterations": self.cfg.max_iterations,
                "robot_xyz": last_robot_xyz,
                "message": f"max iterations ({self.cfg.max_iterations}) reached"}
