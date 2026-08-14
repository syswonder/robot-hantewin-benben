"""Return-home sequence configuration for nero_return_home skill."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ReturnHomeConfig:
    execution_mode: str
    arm_names: list[str]
    relay_joint_poses: list[list[float]]
    home_joint_pose: list[float]
    gripper_open_width_m: float
    gripper_close_width_m: float
    gripper_force_n: float
    close_gripper: bool
    open_gripper_wait_s: float
    motion_timeout_s: float
    step_pause_s: float = 1.0
    start_from: str = "relay"
    standby_joint_pose: list[float] | None = None
    standby_exit_joint_poses: list[list[float]] | None = None


DEFAULT_RETURN_HOME = ReturnHomeConfig(
    execution_mode="single",
    arm_names=["right_arm"],
    relay_joint_poses=[[-0.24, 1.12, -1.88, 0.70, 1.70, 0.11, 0.04]],
    home_joint_pose=[-0.05, 1.47, -0.04, 0.06, -0.0, -0.20, 0.04],
    gripper_open_width_m=0.10,
    gripper_close_width_m=0.0,
    gripper_force_n=1.0,
    close_gripper=True,
    open_gripper_wait_s=1.0,
    motion_timeout_s=20.0,
    step_pause_s=1.0,
)


def _cfg_str(cfg: dict, key: str, default: str = "") -> str:
    return str(cfg.get(key, default)).strip()


def _cfg_float(cfg: dict, key: str, default: float) -> float:
    value = cfg.get(key, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _cfg_bool(cfg: dict, key: str, default: bool) -> bool:
    value = cfg.get(key, default)
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _parse_joint_pose_block(cfg: dict, key: str) -> list[float]:
    raw = cfg.get(key)
    if raw is None:
        raise ValueError(f"{key} is required in nero_return_home config")
    if isinstance(raw, list):
        if len(raw) != 7:
            raise ValueError(f"{key} must have exactly 7 joint values")
        return [float(v) for v in raw]
    if isinstance(raw, dict):
        joints = raw.get("joint_pose", raw.get("joints"))
        if joints is None:
            raise ValueError(f"{key}.joint_pose is required")
        if not isinstance(joints, list) or len(joints) != 7:
            raise ValueError(f"{key}.joint_pose must have exactly 7 joint values")
        return [float(v) for v in joints]
    raise ValueError(f"{key} must be a 7-element list or {{joint_pose: [...]}}")


def _cfg_joint_pose(cfg: dict, key: str) -> list[float] | None:
    value = cfg.get(key)
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 7:
        return None
    return [float(v) for v in value]


def _parse_joint_pose_sequence(raw: object, label: str) -> list[list[float]]:
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"{label} must be a non-empty list of 7-DOF joint poses")

    if all(isinstance(v, (int, float)) for v in raw):
        if len(raw) != 7:
            raise ValueError(f"{label} must have exactly 7 joint values")
        return [[float(v) for v in raw]]

    poses: list[list[float]] = []
    for index, item in enumerate(raw):
        if not isinstance(item, list) or len(item) != 7:
            raise ValueError(f"{label}[{index}] must be a list of 7 joint values")
        poses.append([float(v) for v in item])
    return poses


def _parse_relay_joint_poses(cfg: dict) -> list[list[float]]:
    """Relay waypoints before home_joint_pose; executed in order."""
    if "relay_joint_poses" in cfg:
        return _parse_joint_pose_sequence(cfg["relay_joint_poses"], "relay_joint_poses")

    legacy = _cfg_joint_pose(cfg, "relay_joint_pose")
    if legacy is not None:
        return [legacy]

    raw = cfg.get("relay")
    if raw is None:
        raise ValueError("relay is required in nero_return_home config")

    if isinstance(raw, list):
        return _parse_joint_pose_sequence(raw, "relay")

    if isinstance(raw, dict):
        if "joint_poses" in raw:
            return _parse_joint_pose_sequence(raw["joint_poses"], "relay.joint_poses")
        if "joint_pose" in raw or "joints" in raw:
            return [_parse_joint_pose_block({"relay": raw}, "relay")]

    raise ValueError(
        "relay must be joint_poses (list of 7-DOF poses), joint_pose (single pose), "
        "or relay_joint_poses at top level"
    )


def _parse_standby_joint_pose(cfg: dict) -> list[float] | None:
    if "standby_joint_pose" in cfg:
        pose = _cfg_joint_pose(cfg, "standby_joint_pose")
        if pose is None:
            raise ValueError("standby_joint_pose must be a list of 7 joint values")
        return pose
    block = cfg.get("standby")
    if isinstance(block, dict):
        return _parse_joint_pose_block({"standby": block}, "standby")
    return None


def _parse_standby_exit_joint_poses(cfg: dict) -> list[list[float]] | None:
    if "standby_exit_joint_poses" in cfg:
        return _parse_joint_pose_sequence(cfg["standby_exit_joint_poses"], "standby_exit_joint_poses")
    block = cfg.get("standby_exit")
    if block is None:
        return None
    if isinstance(block, list):
        return _parse_joint_pose_sequence(block, "standby_exit")
    if isinstance(block, dict) and "joint_poses" in block:
        return _parse_joint_pose_sequence(block["joint_poses"], "standby_exit.joint_poses")
    raise ValueError("standby_exit must be joint_poses (list) or {joint_poses: [...]}")


def _parse_start_from(cfg: dict) -> str:
    mode = _cfg_str(cfg, "start_from", "relay").lower()
    aliases = {
        "relay": "relay",
        "standby": "standby",
        "current": "current",
        "home": "current",
        "direct": "current",
    }
    normalized = aliases.get(mode)
    if normalized is None:
        raise ValueError("start_from must be 'relay', 'standby', or 'current'")
    return normalized


def parse_return_home_config(cfg: dict | None) -> ReturnHomeConfig:
    cfg = cfg or {}
    execution_mode = _cfg_str(cfg, "execution_mode", DEFAULT_RETURN_HOME.execution_mode).lower()
    if execution_mode not in {"single", "dual"}:
        raise ValueError("execution_mode must be 'single' or 'dual'")

    arms_cfg = cfg.get("arms")
    if isinstance(arms_cfg, list) and arms_cfg:
        arm_names = [str(v).strip() for v in arms_cfg if str(v).strip()]
    elif execution_mode == "dual":
        arm_names = ["left_arm", "right_arm"]
    else:
        arm_names = [_cfg_str(cfg, "active_arm", DEFAULT_RETURN_HOME.arm_names[0])]

    if execution_mode == "single" and len(arm_names) != 1:
        raise ValueError(f"execution_mode=single requires exactly one arm, got {arm_names!r}")
    if not arm_names:
        raise ValueError("at least one arm must be configured")

    gripper_block = cfg.get("gripper") if isinstance(cfg.get("gripper"), dict) else {}

    start_from = _parse_start_from(cfg)
    standby_joint_pose = _parse_standby_joint_pose(cfg)
    standby_exit_joint_poses = _parse_standby_exit_joint_poses(cfg)
    if start_from == "standby" and standby_joint_pose is None:
        raise ValueError("start_from=standby requires standby_joint_pose or standby block")

    return ReturnHomeConfig(
        execution_mode=execution_mode,
        arm_names=arm_names,
        relay_joint_poses=_parse_relay_joint_poses(cfg),
        home_joint_pose=_parse_joint_pose_block(cfg, "home"),
        gripper_open_width_m=_cfg_float(
            gripper_block,
            "open_width_m",
            _cfg_float(cfg, "gripper_open_width_m", DEFAULT_RETURN_HOME.gripper_open_width_m),
        ),
        gripper_close_width_m=_cfg_float(
            gripper_block,
            "close_width_m",
            _cfg_float(cfg, "gripper_close_width_m", DEFAULT_RETURN_HOME.gripper_close_width_m),
        ),
        gripper_force_n=_cfg_float(
            gripper_block,
            "force_n",
            _cfg_float(cfg, "gripper_force_n", DEFAULT_RETURN_HOME.gripper_force_n),
        ),
        close_gripper=_cfg_bool(
            gripper_block,
            "close_before_return",
            _cfg_bool(cfg, "close_gripper", DEFAULT_RETURN_HOME.close_gripper),
        ),
        open_gripper_wait_s=_cfg_float(
            gripper_block,
            "open_wait_s",
            _cfg_float(cfg, "open_gripper_wait_s", DEFAULT_RETURN_HOME.open_gripper_wait_s),
        ),
        motion_timeout_s=_cfg_float(cfg, "motion_timeout_s", DEFAULT_RETURN_HOME.motion_timeout_s),
        step_pause_s=_cfg_float(cfg, "step_pause_s", DEFAULT_RETURN_HOME.step_pause_s),
        start_from=start_from,
        standby_joint_pose=standby_joint_pose,
        standby_exit_joint_poses=standby_exit_joint_poses,
    )
