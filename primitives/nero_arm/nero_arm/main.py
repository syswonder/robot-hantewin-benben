#!/usr/bin/env python3
"""AgileX Nero 7-DOF arm primitive for Robonix."""
from __future__ import annotations

import logging
import os
import threading
import time
from typing import Any

from robonix_api import Deferred, Err, Ok, Primitive

from .arm_driver import NeroArmDriver
from .geometry_utils import ros_pose_to_rpy, rpy_pose_to_ros_pose

logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
log = logging.getLogger("nero_arm")

INSTANCE_NAME = os.environ.get("RBNX_INSTANCE_NAME", "nero_arm")
provider = Primitive(id=INSTANCE_NAME, namespace="robonix/primitive/arm")

_driver: NeroArmDriver | None = None
_joint_names: list[str] = []
_publish_rate_hz = 50.0
_reference_frame = "base_link"
_tf_joint_states_topic = "/joint_states"
_stop_event = threading.Event()
_state_thread: threading.Thread | None = None
_command_lock = threading.Lock()


def _joint_names_for_prefix(prefix: str) -> list[str]:
    prefix = prefix or ""
    return [f"{prefix}joint{i}" for i in range(1, 8)]


def _cfg_str(cfg: dict, key: str, default: str = "") -> str:
    value = cfg.get(key)
    if value is None:
        value = os.environ.get(key.upper(), default)
    return str(value).strip()


def _cfg_bool(cfg: dict, key: str, default: bool) -> bool:
    value = cfg.get(key)
    if value is None:
        env = os.environ.get(key.upper())
        if env is None:
            return default
        value = env
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _cfg_float(cfg: dict, key: str, default: float) -> float:
    value = cfg.get(key)
    if value is None:
        env = os.environ.get(key.upper())
        if env is None:
            return default
        value = env
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _cfg_int(cfg: dict, key: str, default: int) -> int:
    value = cfg.get(key)
    if value is None:
        env = os.environ.get(key.upper())
        if env is None:
            return default
        value = env
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _cfg_float_list(cfg: dict, key: str) -> list[float] | None:
    value = cfg.get(key)
    if value is None:
        return None
    if not isinstance(value, list):
        return None
    try:
        return [float(v) for v in value]
    except (TypeError, ValueError):
        return None


def _ensure_enabled() -> None:
    if _driver is None:
        raise RuntimeError("arm driver is not initialized")
    if not _driver.enabled:
        if not _driver.enable():
            raise RuntimeError("failed to enable arm joints")

def _build_joint_state(positions: list[float]):
    from sensor_msgs.msg import JointState  # type: ignore
    from std_msgs.msg import Header  # type: ignore

    msg = JointState()
    now = time.time()
    msg.header = Header()
    msg.header.stamp.sec = int(now)
    msg.header.stamp.nanosec = int((now % 1.0) * 1e9)
    msg.header.frame_id = _reference_frame
    msg.name = list(_joint_names)
    msg.position = [float(v) for v in positions]
    msg.velocity = []
    msg.effort = []
    return msg


def _publish_state_loop() -> None:
    period = 1.0 / max(_publish_rate_hz, 1.0)
    while not _stop_event.is_set():
        try:
            if _driver is None:
                time.sleep(period)
                continue
            joints = _driver.get_joint_angles()
            if joints is None:
                time.sleep(period)
                continue
            js = _build_joint_state(joints)
            provider.emit("robonix/primitive/arm/joint_states", js)
            if _tf_joint_states_topic:
                provider.emit("_tf_joint_states", js)
            tcp = _driver.get_tcp_pose_rpy()
            if tcp is not None:
                provider.emit("robonix/primitive/arm/end_pose", rpy_pose_to_ros_pose(tcp))
        except Exception as exc:  # noqa: BLE001
            log.warning("state publish failed: %s", exc)
        time.sleep(period)


def _on_joint_command(msg) -> None:
    with _command_lock:
        try:
            _ensure_enabled()
            if not msg.name and not msg.position:
                return
            log.info(
                "joint_command received names=%d positions=%s",
                len(msg.name),
                [round(float(v), 3) for v in msg.position[:7]],
            )
            if msg.name and msg.position and len(msg.name) != len(msg.position):
                raise ValueError("joint_command name/position length mismatch")
            if msg.name:
                target = _driver.get_joint_angles() if _driver else None
                if target is None:
                    target = [0.0] * 7
                name_to_idx = {name: idx for idx, name in enumerate(_joint_names)}
                for name, pos in zip(msg.name, msg.position):
                    if name not in name_to_idx:
                        log.warning("ignoring unknown joint %s", name)
                        continue
                    target[name_to_idx[name]] = float(pos)
            else:
                if len(msg.position) != 7:
                    raise ValueError(f"expected 7 joint positions, got {len(msg.position)}")
                target = [float(v) for v in msg.position]
            _driver.move_j(target)
        except Exception as exc:  # noqa: BLE001
            log.error("joint_command failed: %s", exc)


def _on_pos_command(msg) -> None:
    with _command_lock:
        try:
            _ensure_enabled()
            log.info(
                "pos_command received xyz=%.3f,%.3f,%.3f",
                float(msg.position.x),
                float(msg.position.y),
                float(msg.position.z),
            )
            tcp_pose = ros_pose_to_rpy(msg)
            _driver.move_p_tcp(tcp_pose)
        except Exception as exc:  # noqa: BLE001
            log.error("pos_command failed: %s", exc)


def _on_linear_pos_command(msg) -> None:
    with _command_lock:
        try:
            _ensure_enabled()
            log.info(
                "linear_pos_command received xyz=%.3f,%.3f,%.3f",
                float(msg.position.x),
                float(msg.position.y),
                float(msg.position.z),
            )
            tcp_pose = ros_pose_to_rpy(msg)
            _driver.move_l_tcp(tcp_pose)
        except Exception as exc:  # noqa: BLE001
            log.error("linear_pos_command failed: %s", exc)


def _on_gripper_command(msg) -> None:
    with _command_lock:
        try:
            _ensure_enabled()
            if _driver is None or not _driver.has_effector:
                raise RuntimeError("gripper effector is not initialized")
            if not msg.data:
                raise ValueError("gripper_command requires at least width in data[0]")
            width_m = float(msg.data[0])
            force_n = float(msg.data[1]) if len(msg.data) > 1 else 1.0
            _driver.move_gripper(width_m, force_n)
        except Exception as exc:  # noqa: BLE001
            log.error("gripper_command failed: %s", exc)


@provider.on_init
def init(cfg: dict):
    global _driver, _joint_names, _publish_rate_hz, _reference_frame
    global _tf_joint_states_topic, _state_thread

    cfg = cfg or {}
    prefix = _cfg_str(cfg, "joint_name_prefix", "")
    _joint_names = _joint_names_for_prefix(prefix)
    _publish_rate_hz = _cfg_float(cfg, "publish_rate_hz", 50.0)
    _reference_frame = _cfg_str(cfg, "reference_frame", prefix + "base_link")
    _tf_joint_states_topic = _cfg_str(cfg, "tf_joint_states_topic", "/joint_states")

    can_channel = _cfg_str(cfg, "can_channel", "can0")
    can_interface = _cfg_str(cfg, "can_interface", "socketcan")
    firmware_version = _cfg_str(cfg, "firmware_version", "default")
    speed_percent = _cfg_int(cfg, "speed_percent", 50)
    enable_on_init = _cfg_bool(cfg, "enable_on_init", False)
    connect_timeout_s = _cfg_float(cfg, "connect_timeout_s", 30.0)

    joint_states_topic = _cfg_str(cfg, "joint_states_topic", f"/{INSTANCE_NAME}/joint_states")
    joint_command_topic = _cfg_str(cfg, "joint_command_topic", f"/{INSTANCE_NAME}/joint_command")
    pos_command_topic = _cfg_str(cfg, "pos_command_topic", f"/{INSTANCE_NAME}/pos_command")
    linear_pos_command_topic = _cfg_str(
        cfg, "linear_pos_command_topic", f"/{INSTANCE_NAME}/linear_pos_command"
    )
    end_pose_topic = _cfg_str(cfg, "end_pose_topic", f"/{INSTANCE_NAME}/end_pose")
    gripper_command_topic = _cfg_str(cfg, "gripper_command_topic", f"/{INSTANCE_NAME}/gripper_command")
    effector = _cfg_str(cfg, "effector", "")

    log.info(
        "init instance=%s can=%s joints=%s reference=%s",
        INSTANCE_NAME,
        can_channel,
        _joint_names,
        _reference_frame,
    )

    try:
        _driver = NeroArmDriver(
            can_channel=can_channel,
            can_interface=can_interface,
            firmware_version=firmware_version,
            speed_percent=speed_percent,
            tcp_offset=_cfg_float_list(cfg, "tcp_offset"),
        )
        _driver.connect(effector=effector or None)
        if effector:
            log.info("effector initialized: %s", effector)
    except Exception as exc:  # noqa: BLE001
        log.exception("failed to connect Nero arm")
        return Deferred(f"Nero arm connect failed: {exc}")

    if enable_on_init:
        if not _driver.enable(timeout_s=connect_timeout_s):
            return Err(f"failed to enable/recover arm within {connect_timeout_s:.1f}s")
        status = _driver._arm_status_code()
        log.info("arm ready arm_status=0x%02X", status if status is not None else -1)

    startup_joint_pose = _cfg_float_list(cfg, "startup_joint_pose")
    if startup_joint_pose is not None:
        if len(startup_joint_pose) != 7:
            return Err(f"startup_joint_pose must have 7 values, got {len(startup_joint_pose)}")
        startup_motion_timeout_s = _cfg_float(cfg, "startup_motion_timeout_s", connect_timeout_s)
        if not _driver.enabled:
            if not _driver.enable(timeout_s=connect_timeout_s):
                return Err(
                    f"failed to enable arm for startup_joint_pose within {connect_timeout_s:.1f}s"
                )
        log.info(
            "moving to startup joint pose: %s",
            [round(v, 3) for v in startup_joint_pose],
        )
        old_motion_timeout_s = _driver.motion_timeout_s
        try:
            if startup_motion_timeout_s > 0:
                _driver.motion_timeout_s = startup_motion_timeout_s
            _driver.move_j(startup_joint_pose)
        except Exception as exc:  # noqa: BLE001
            log.exception("startup joint move failed")
            return Err(f"startup_joint_pose move failed: {exc}")
        finally:
            _driver.motion_timeout_s = old_motion_timeout_s
        log.info("startup joint pose reached")

    from geometry_msgs.msg import Pose  # type: ignore
    from sensor_msgs.msg import JointState  # type: ignore
    from std_msgs.msg import Float64MultiArray  # type: ignore

    provider.declare_ros2_topic("robonix/primitive/arm/joint_states", joint_states_topic, qos="reliable")
    provider.declare_ros2_topic("robonix/primitive/arm/joint_command", joint_command_topic, qos="reliable")
    provider.declare_ros2_topic("robonix/primitive/arm/pos_command", pos_command_topic, qos="reliable")
    provider.declare_ros2_topic(
        "robonix/primitive/arm/linear_pos_command",
        linear_pos_command_topic,
        qos="reliable",
    )
    provider.declare_ros2_topic("robonix/primitive/arm/end_pose", end_pose_topic, qos="reliable")
    if effector:
        provider.declare_ros2_topic(
            "robonix/primitive/arm/gripper_command",
            gripper_command_topic,
            qos="reliable",
        )

    provider.create_publisher(
        "robonix/primitive/arm/joint_states",
        topic=joint_states_topic,
        msg_type=JointState,
        qos="reliable",
    )
    provider.create_publisher(
        "robonix/primitive/arm/end_pose",
        topic=end_pose_topic,
        msg_type=Pose,
        qos="reliable",
    )
    if _tf_joint_states_topic and _tf_joint_states_topic != joint_states_topic:
        provider.create_publisher(
            "_tf_joint_states",
            topic=_tf_joint_states_topic,
            msg_type=JointState,
            qos="reliable",
            declare=False,
        )

    provider.create_subscription(
        "robonix/primitive/arm/joint_command",
        topic=joint_command_topic,
        msg_type=JointState,
        callback=_on_joint_command,
        qos="reliable",
    )
    provider.create_subscription(
        "robonix/primitive/arm/pos_command",
        topic=pos_command_topic,
        msg_type=Pose,
        callback=_on_pos_command,
        qos="reliable",
    )
    provider.create_subscription(
        "robonix/primitive/arm/linear_pos_command",
        topic=linear_pos_command_topic,
        msg_type=Pose,
        callback=_on_linear_pos_command,
        qos="reliable",
    )
    if effector:
        provider.create_subscription(
            "robonix/primitive/arm/gripper_command",
            topic=gripper_command_topic,
            msg_type=Float64MultiArray,
            callback=_on_gripper_command,
            qos="reliable",
        )

    _stop_event.clear()
    _state_thread = threading.Thread(target=_publish_state_loop, name=f"{INSTANCE_NAME}-state", daemon=True)
    _state_thread.start()
    return Ok()


@provider.on_shutdown
def shutdown():
    global _driver, _state_thread
    _stop_event.set()
    if _state_thread is not None:
        _state_thread.join(timeout=2.0)
        _state_thread = None
    if _driver is not None:
        try:
            _driver.emergency_stop()
        except Exception as exc:  # noqa: BLE001
            log.warning("electronic_emergency_stop failed: %s", exc)
        _driver.disable()
        _driver.disconnect()
        _driver = None
    return Ok()


if __name__ == "__main__":
    provider.run()
