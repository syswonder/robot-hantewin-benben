"""Thin wrapper around pyAgxArm for a single Nero 7-DOF arm."""
from __future__ import annotations

import logging
import math
import time
from typing import Any

log = logging.getLogger("nero_arm.driver")

FW_ALIASES = {
    "default": "DEFAULT",
    "v111": "V111",
    "v112": "V112",
    "v120": "V120",
    "1.10": "DEFAULT",
    "1.11": "V111",
    "1.12": "V112",
    "1.20": "V120",
}

_ARM_STATUS_NAMES = {
    0x00: "normal",
    0x01: "e-stop",
    0x02: "no_ik",
    0x03: "singularity",
    0x04: "joint_limit",
    0x06: "brake_not_open",
    0x07: "collision",
}

_TRANSIENT_ARM_STATUSES = {0x06}
JOINT_TOLERANCE_RAD = 0.05
TCP_POS_TOLERANCE_M = 0.005


class NeroArmDriver:
    def __init__(
        self,
        *,
        can_channel: str,
        can_interface: str,
        firmware_version: str,
        speed_percent: int,
        tcp_offset: list[float] | None,
        motion_timeout_s: float = 20.0,
    ) -> None:
        self.can_channel = can_channel
        self.can_interface = can_interface
        self.firmware_version = firmware_version
        self.speed_percent = speed_percent
        self.tcp_offset = tcp_offset or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.motion_timeout_s = motion_timeout_s
        self._robot: Any | None = None
        self._effector: Any | None = None
        self._enabled = False

    @property
    def _fw_key(self) -> str:
        return self.firmware_version.strip().lower()

    def _uses_follower_mode(self) -> bool:
        fw_key = FW_ALIASES.get(self._fw_key, "DEFAULT")
        return fw_key in {"V112", "V120"}

    def connect(self, *, effector: str | None = None) -> None:
        from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

        fw_key = FW_ALIASES.get(self._fw_key, "DEFAULT")
        fw = getattr(NeroFW, fw_key, NeroFW.DEFAULT)
        cfg = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=fw,
            channel=self.can_channel,
            interface=self.can_interface,
        )
        self._robot = AgxArmFactory.create_arm(cfg)
        if effector:
            self._init_effector(effector)
        self._robot.connect()
        if any(v != 0.0 for v in self.tcp_offset):
            self._robot.set_tcp_offset(self.tcp_offset)
        self._robot.set_speed_percent(int(self.speed_percent))
        self._prepare_mode()

    @property
    def robot(self) -> Any:
        if self._robot is None:
            raise RuntimeError("Nero arm is not connected")
        return self._robot

    def _prepare_mode(self) -> None:
        if self._uses_follower_mode():
            self.robot.set_follower_mode()
        else:
            self.robot.set_normal_mode()

    def _joints_enabled(self) -> list[bool] | None:
        return self.robot.get_joints_enable_status_list()

    def _all_joints_enabled(self) -> bool:
        enabled = self._joints_enabled()
        return bool(enabled) and all(enabled)

    def _arm_status_code(self) -> int | None:
        status = self.robot.get_arm_status()
        return None if status is None else int(status.msg.arm_status)

    def _ensure_disabled(self, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.robot.disable()
            enabled = self._joints_enabled()
            if not enabled or not any(enabled):
                self._enabled = False
                return
            time.sleep(0.01)
        log.warning("joints still enabled after disable: %s", enabled)

    def _wait_arm_normal(self, timeout_s: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_s
        last_status: int | None = None
        while time.monotonic() < deadline:
            arm_status = self._arm_status_code()
            if arm_status is None:
                time.sleep(0.05)
                continue
            if arm_status == 0x00:
                return
            if arm_status != last_status:
                name = _ARM_STATUS_NAMES.get(arm_status, f"0x{arm_status:02X}")
                if arm_status in _TRANSIENT_ARM_STATUSES:
                    log.info("arm_status=%s, waiting...", name)
                else:
                    log.warning("arm_status=%s", name)
                last_status = arm_status
            if arm_status in _TRANSIENT_ARM_STATUSES:
                time.sleep(0.1)
                continue
            name = _ARM_STATUS_NAMES.get(arm_status, f"0x{arm_status:02X}")
            raise RuntimeError(f"arm fault: arm_status={name}")
        last_name = _ARM_STATUS_NAMES.get(last_status or -1, "unknown")
        raise TimeoutError(f"arm not ready within {timeout_s:.1f}s (last arm_status={last_name})")

    def _enable_once(self, timeout_s: float = 30.0) -> None:
        """Enable all joints; do not wait for arm_status normal (recover_faults handles that)."""
        self._ensure_disabled(min(5.0, timeout_s))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._prepare_mode()
            if self.robot.enable() and self._all_joints_enabled():
                self._enabled = True
                self.robot.clear_joint_error(255)
                return
            time.sleep(0.01)
        self._enabled = False
        raise TimeoutError(f"failed to enable arm within {timeout_s:.1f}s")

    def recover_faults(self, *, max_attempts: int = 3) -> None:
        """Recover from e-stop / transient faults until arm_status is normal."""
        self.robot.clear_joint_error(255)
        for attempt in range(max_attempts):
            arm_status = self._arm_status_code()
            if arm_status == 0x01:
                log.warning("arm in e-stop, calling reset() (attempt %d)", attempt + 1)
                self.robot.reset()
                time.sleep(0.5)
                self._ensure_disabled()
                self._enable_once(timeout_s=10.0)
                continue
            try:
                self._wait_arm_normal(timeout_s=15.0)
                log.info("arm recovered to normal status")
                return
            except (TimeoutError, RuntimeError):
                log.warning("arm not normal after enable (attempt %d)", attempt + 1)
                self._ensure_disabled()
                self._enable_once(timeout_s=10.0)
        raise RuntimeError("could not recover arm to normal status")

    def enable(self, timeout_s: float = 30.0) -> bool:
        try:
            self._enable_once(timeout_s=timeout_s)
            self.recover_faults()
            return True
        except (TimeoutError, RuntimeError) as exc:
            log.error("enable/recover failed: %s", exc)
            self._enabled = False
            return False

    def disable(self, timeout_s: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.robot.disable():
                self._enabled = False
                return True
            time.sleep(0.01)
        return False

    @property
    def enabled(self) -> bool:
        return self._enabled and self._all_joints_enabled()

    def prepare_for_motion(self) -> None:
        """Clear faults and ensure joints are enabled before move_j/move_p."""
        self.robot.clear_joint_error(255)
        if self._arm_status_code() == 0x01:
            log.warning("e-stop before motion, recovering")
            self.robot.reset()
            time.sleep(0.5)
            self._ensure_disabled()
            self._enable_once(timeout_s=10.0)
        try:
            self._wait_arm_normal(timeout_s=10.0)
        except (TimeoutError, RuntimeError):
            log.warning("arm not normal before motion, re-enabling")
            self.recover_faults()
        if not self._all_joints_enabled():
            log.warning("joints not enabled before motion, re-enabling")
            self._enable_once(timeout_s=10.0)
            self._wait_arm_normal(timeout_s=10.0)

    @staticmethod
    def _joints_close(a: list[float], b: list[float], *, tol: float = JOINT_TOLERANCE_RAD) -> bool:
        return max(abs(x - y) for x, y in zip(a, b)) <= tol

    @staticmethod
    def _tcp_pos_close(a: list[float], b: list[float], *, tol: float = TCP_POS_TOLERANCE_M) -> bool:
        return math.dist(a[:3], b[:3]) <= tol

    def _wait_motion_done(
        self,
        label: str,
        *,
        timeout_s: float | None = None,
        joint_target: list[float] | None = None,
        tcp_target: list[float] | None = None,
    ) -> None:
        timeout_s = timeout_s or self.motion_timeout_s
        time.sleep(0.2)
        deadline = time.monotonic() + timeout_s
        saw_moving = False
        while time.monotonic() < deadline:
            status = self.robot.get_arm_status()
            if status is None:
                time.sleep(0.05)
                continue
            motion_status = int(status.msg.motion_status)
            if motion_status == 0x01:
                saw_moving = True
            if motion_status == 0x00 and saw_moving:
                self._wait_arm_normal(timeout_s=5.0)
                return
            if joint_target is not None:
                current = self.get_joint_angles()
                if current is not None and self._joints_close(current, joint_target):
                    log.info("%s reached joint target", label)
                    self._wait_arm_normal(timeout_s=5.0)
                    return
            if tcp_target is not None:
                current = self.get_tcp_pose_rpy()
                if current is not None and self._tcp_pos_close(current, tcp_target):
                    log.info("%s reached tcp target", label)
                    self._wait_arm_normal(timeout_s=5.0)
                    return
            arm_status = int(status.msg.arm_status)
            if arm_status != 0x00:
                if arm_status in _TRANSIENT_ARM_STATUSES:
                    time.sleep(0.1)
                    continue
                name = _ARM_STATUS_NAMES.get(arm_status, f"unknown(0x{arm_status:02X})")
                raise RuntimeError(f"{label} failed: arm_status={name}")
            time.sleep(0.05)
        if joint_target is not None:
            current = self.get_joint_angles()
            if current is not None and self._joints_close(current, joint_target):
                log.info("%s reached joint target (late feedback)", label)
                return
        if tcp_target is not None:
            current = self.get_tcp_pose_rpy()
            if current is not None and self._tcp_pos_close(current, tcp_target):
                log.info("%s reached tcp target (late feedback)", label)
                return
        if saw_moving:
            raise TimeoutError(f"{label} timed out after {timeout_s:.1f}s")
        raise TimeoutError(
            f"{label} did not reach target within {timeout_s:.1f}s "
            f"(motion_status never reported moving)"
        )

    def get_joint_angles(self) -> list[float] | None:
        msg = self.robot.get_joint_angles()
        return None if msg is None else list(msg.msg)

    def get_tcp_pose_rpy(self) -> list[float] | None:
        msg = self.robot.get_tcp_pose()
        return None if msg is None else list(msg.msg)

    def move_j(self, joints: list[float]) -> None:
        if len(joints) != 7:
            raise ValueError(f"expected 7 joint targets, got {len(joints)}")
        before = self.get_joint_angles()
        target = [float(v) for v in joints]
        self.prepare_for_motion()
        self.robot.move_j(target)
        self._wait_motion_done("move_j", joint_target=target)
        after = self.get_joint_angles()
        log.info(
            "move_j target=%s before=%s after=%s",
            [round(v, 3) for v in target],
            None if before is None else [round(v, 3) for v in before],
            None if after is None else [round(v, 3) for v in after],
        )

    def move_p_tcp(self, tcp_pose_rpy: list[float]) -> None:
        if len(tcp_pose_rpy) != 6:
            raise ValueError(f"expected 6D TCP pose, got {len(tcp_pose_rpy)}")
        before = self.get_tcp_pose_rpy()
        target = [float(v) for v in tcp_pose_rpy]
        self.prepare_for_motion()
        flange_pose = self.robot.get_tcp2flange_pose(target)
        self.robot.move_p(flange_pose)
        self._wait_motion_done("move_p", tcp_target=target)
        after = self.get_tcp_pose_rpy()
        log.info(
            "move_p target=%s before=%s after=%s",
            [round(v, 3) for v in target[:3]],
            None if before is None else [round(v, 3) for v in before[:3]],
            None if after is None else [round(v, 3) for v in after[:3]],
        )

    def move_l_tcp(self, tcp_pose_rpy: list[float]) -> None:
        if len(tcp_pose_rpy) != 6:
            raise ValueError(f"expected 6D TCP pose, got {len(tcp_pose_rpy)}")
        before = self.get_tcp_pose_rpy()
        target = [float(v) for v in tcp_pose_rpy]
        self.prepare_for_motion()
        flange_pose = self.robot.get_tcp2flange_pose(target)
        self.robot.move_l(flange_pose)
        self._wait_motion_done("move_l", tcp_target=target)
        after = self.get_tcp_pose_rpy()
        log.info(
            "move_l target=%s before=%s after=%s",
            [round(v, 3) for v in target[:3]],
            None if before is None else [round(v, 3) for v in before[:3]],
            None if after is None else [round(v, 3) for v in after[:3]],
        )

    def _init_effector(self, effector: str) -> None:
        if self._robot is None:
            raise RuntimeError("Nero arm is not created")
        if self._effector is not None:
            raise RuntimeError("effector already initialized")
        effector_key = effector.strip().lower()
        option_map = {
            "agx_gripper": "AGX_GRIPPER",
            "revo2": "REVO2",
            "revo2_touch": "REVO2_TOUCH",
        }
        option_name = option_map.get(effector_key)
        if option_name is None:
            raise ValueError(f"unsupported effector: {effector}")
        effector_opt = getattr(self._robot.OPTIONS.EFFECTOR, option_name)
        self._effector = self._robot.init_effector(effector_opt)

    def init_effector(self, effector: str) -> None:
        """Initialize effector before connect(); prefer connect(effector=...)."""
        self._init_effector(effector)

    @property
    def has_effector(self) -> bool:
        return self._effector is not None

    def move_gripper(self, width_m: float, force_n: float = 1.0, *, settle_s: float = 1.0) -> None:
        if self._effector is None:
            raise RuntimeError("gripper effector is not initialized")
        force_n = max(0.0, min(3.0, float(force_n)))
        self._effector.move_gripper_m(value=float(width_m), force=force_n)
        time.sleep(settle_s)

    def emergency_stop(self) -> None:
        self.robot.electronic_emergency_stop()

    def disconnect(self) -> None:
        if self._robot is None:
            return
        try:
            if self._enabled:
                self.disable(timeout_s=3.0)
        except Exception as exc:  # noqa: BLE001
            log.warning("disable during disconnect failed: %s", exc)
        try:
            self._robot.disconnect()
        except Exception as exc:  # noqa: BLE001
            log.warning("disconnect during disconnect failed: %s", exc)
        finally:
            self._robot = None
            self._enabled = False
