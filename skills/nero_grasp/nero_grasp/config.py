"""Grasp sequence configuration for nero_grasp skill."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GraspConfig:
    execution_mode: str
    arm_names: list[str]
    relay_joint_poses: list[list[float]]
    initial_joint_pose: list[float] | None
    camera_provider_id: str | None
    detector_provider_id: str | None
    detector_settings: dict
    calib_path: str
    require_detection: bool
    grasp_target_offset_m: list[float] | None
    grasp_offset_m: list[float]
    gripper_open_width_m: float
    gripper_close_width_m: float
    gripper_force_n: float
    grasp_forward_offset_m: float
    lift_offset_m: float
    motion_timeout_s: float
    camera_frame_timeout_s: float = 5.0
    step_pause_s: float = 3.0
    return_initial: bool = True
    recover_on_failure: bool = True
    standby_joint_pose: list[float] | None = None
    target_source: str = "vlm"
    fixed_tcp_pose: list[float] | None = None


# TCP axis convention for Nero (see TEST.md):
#   index 0 = up, index 1 = backward, index 2 = right
DEFAULT_GRASP = GraspConfig(
    execution_mode="single",
    arm_names=["right_arm"],
    relay_joint_poses=[[-0.24, 1.12, -1.88, 0.70, 1.70, 0.11, 0.04]],
    initial_joint_pose=[-0.15, 0.95, -1.7, 1.4, 2.53, 0.2, 0.0],
    camera_provider_id=None,
    detector_provider_id=None,
    detector_settings={
        "detector": "vlm",
        "instruction": "抓取桌上的清洁剂",
        "depth_min_m": 0.15,
        "depth_max_m": 1.8,
        "sample_window": 15,
    },
    calib_path="calib/camera_to_robot.json",
    require_detection=False,
    grasp_target_offset_m=None,
    grasp_offset_m=[0.0, 0.0, 0.0],
    gripper_open_width_m=0.05,
    gripper_close_width_m=0.0,
    gripper_force_n=1.0,
    grasp_forward_offset_m=0.05,
    lift_offset_m=0.03,
    motion_timeout_s=20.0,
    step_pause_s=3.0,
    return_initial=True,
    recover_on_failure=True,
)


def _cfg_str(cfg: dict, key: str, default: str = "") -> str:
    value = cfg.get(key, default)
    return str(value).strip()


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


def _cfg_joint_pose(cfg: dict, key: str) -> list[float] | None:
    value = cfg.get(key)
    if value is None:
        return None
    if not isinstance(value, list):
        return None
    if len(value) != 7:
        raise ValueError(f"{key} must have exactly 7 joint values")
    return [float(v) for v in value]


def _parse_joint_pose_block(cfg: dict, key: str) -> list[float]:
    raw = cfg.get(key)
    if raw is None:
        raise ValueError(f"{key} is required in nero_grasp config")
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
    """Relay waypoints before initial_joint_pose; executed in order."""
    if "relay_joint_poses" in cfg:
        return _parse_joint_pose_sequence(cfg["relay_joint_poses"], "relay_joint_poses")

    legacy = _cfg_joint_pose(cfg, "relay_joint_pose")
    if legacy is not None:
        return [legacy]

    raw = cfg.get("relay")
    if raw is None:
        raise ValueError("relay is required in nero_grasp config")

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


def _parse_initial_joint_pose(cfg: dict) -> list[float] | None:
    """User-set initial/observe joint pose (7 rad)."""
    for key in ("initial_joint_pose", "observe_joint_pose"):
        if key in cfg:
            return _cfg_joint_pose(cfg, key)
    return DEFAULT_GRASP.initial_joint_pose


def _parse_standby_joint_pose(cfg: dict) -> list[float] | None:
    """Post-grasp standby joint pose (7 rad); optional."""
    if "standby_joint_pose" in cfg:
        return _cfg_joint_pose(cfg, "standby_joint_pose")
    if "standby" in cfg:
        return _parse_joint_pose_block(cfg, "standby")
    return None


def _parse_fixed_tcp_pose(cfg: dict) -> list[float] | None:
    """Fixed grasp TCP pose [x, y, z] or [x, y, z, roll, pitch, yaw] in robot base frame."""
    if "fixed_tcp_pose" in cfg:
        value = cfg["fixed_tcp_pose"]
        if not isinstance(value, list) or len(value) not in {3, 6}:
            raise ValueError("fixed_tcp_pose must be 3 (xyz) or 6 (xyz+rpy) floats")
        return [float(v) for v in value]
    block = cfg.get("fixed_tcp")
    if not isinstance(block, dict):
        return None
    pose = block.get("pose")
    if isinstance(pose, list) and len(pose) in {3, 6}:
        return [float(v) for v in pose]
    xyz = block.get("xyz")
    rpy = block.get("rpy")
    if isinstance(xyz, list) and len(xyz) == 3:
        if rpy is None:
            return [float(v) for v in xyz]
        if isinstance(rpy, list) and len(rpy) == 3:
            return [float(v) for v in xyz + rpy]
    raise ValueError("fixed_tcp must provide pose (3 or 6) or xyz (3) with optional rpy (3)")


def _parse_target_source(cfg: dict) -> str:
    source = _cfg_str(cfg, "target_source", DEFAULT_GRASP.target_source).lower()
    aliases = {
        "vlm": "vlm",
        "vision": "vlm",
        "detect": "vlm",
        "detection": "vlm",
        "fixed_tcp": "fixed_tcp",
        "tcp": "fixed_tcp",
        "tcp_pose": "fixed_tcp",
        "fixed": "fixed_tcp",
    }
    normalized = aliases.get(source)
    if normalized is None:
        raise ValueError("target_source must be 'vlm' or 'fixed_tcp'")
    return normalized


def _cfg_xyz_offset(cfg: dict, key: str) -> list[float] | None:
    value = cfg.get(key)
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{key} must be a list of 3 floats (metres, robot base frame)")
    return [float(v) for v in value]


def _parse_offset_xyz(cfg: dict) -> list[float]:
    offset_block = cfg.get("offset")
    if isinstance(offset_block, dict) and "xyz" in offset_block:
        xyz = offset_block["xyz"]
        if not isinstance(xyz, list) or len(xyz) != 3:
            raise ValueError("offset.xyz must be a list of 3 floats")
        return [float(v) for v in xyz]
    legacy = _cfg_xyz_offset(cfg, "grasp_target_offset_m")
    if legacy is not None:
        return legacy
    return list(DEFAULT_GRASP.grasp_offset_m)


def _merge_vlm_settings(detector_settings: dict, vlm: dict | None) -> dict:
    if not isinstance(vlm, dict) or not vlm:
        return detector_settings
    merged = dict(detector_settings)
    field_map = {
        "upstream": "vlm_base_url",
        "base_url": "vlm_base_url",
        "vlm_base_url": "vlm_base_url",
        "api_key": "vlm_api_key",
        "vlm_api_key": "vlm_api_key",
        "model": "vlm_model",
        "vlm_model": "vlm_model",
        "timeout_s": "vlm_timeout_s",
        "vlm_timeout_s": "vlm_timeout_s",
        "temperature": "vlm_temperature",
        "vlm_temperature": "vlm_temperature",
        "use_json_schema": "vlm_use_json_schema",
        "vlm_use_json_schema": "vlm_use_json_schema",
    }
    for src, dst in field_map.items():
        if src not in vlm:
            continue
        value = vlm[src]
        if value is None:
            continue
        if isinstance(value, str) and not value.strip():
            continue
        merged[dst] = value
    return merged


def parse_grasp_config(cfg: dict | None) -> GraspConfig:
    """Parse Robonix manifest config into GraspConfig."""
    cfg = cfg or {}
    execution_mode = _cfg_str(cfg, "execution_mode", DEFAULT_GRASP.execution_mode).lower()
    if execution_mode not in {"single", "dual"}:
        raise ValueError("execution_mode must be 'single' or 'dual'")

    arms_cfg = cfg.get("arms")
    if isinstance(arms_cfg, list) and arms_cfg:
        arm_names = [str(v).strip() for v in arms_cfg if str(v).strip()]
    elif execution_mode == "dual":
        arm_names = ["left_arm", "right_arm"]
    else:
        arm_names = [_cfg_str(cfg, "active_arm", DEFAULT_GRASP.arm_names[0])]

    if execution_mode == "single" and len(arm_names) != 1:
        raise ValueError(f"execution_mode=single requires exactly one arm, got {arm_names!r}")
    if not arm_names:
        raise ValueError("at least one arm must be configured")

    camera_provider = _cfg_str(cfg, "camera_provider_id", "")
    camera_provider_id = camera_provider or None
    detector_provider = _cfg_str(cfg, "detector_provider_id", "")
    detector_provider_id = detector_provider or None
    calib_path = _cfg_str(cfg, "calib_path", DEFAULT_GRASP.calib_path) or DEFAULT_GRASP.calib_path
    detector_settings = cfg.get("detector_settings")
    if not isinstance(detector_settings, dict):
        detector_settings = dict(DEFAULT_GRASP.detector_settings)

    from .vlm_client import load_pilot_vlm_from_manifest

    detector_settings = _merge_vlm_settings(detector_settings, load_pilot_vlm_from_manifest())
    skill_vlm = cfg.get("vlm")
    if isinstance(skill_vlm, dict):
        detector_settings = _merge_vlm_settings(detector_settings, skill_vlm)

    target_source = _parse_target_source(cfg)
    fixed_tcp_pose = _parse_fixed_tcp_pose(cfg)
    if target_source == "fixed_tcp" and fixed_tcp_pose is None:
        raise ValueError("target_source=fixed_tcp requires fixed_tcp_pose or fixed_tcp block")

    return GraspConfig(
        execution_mode=execution_mode,
        arm_names=arm_names,
        relay_joint_poses=_parse_relay_joint_poses(cfg),
        initial_joint_pose=_parse_initial_joint_pose(cfg),
        camera_provider_id=camera_provider_id,
        detector_provider_id=detector_provider_id,
        detector_settings=detector_settings,
        calib_path=calib_path,
        require_detection=_cfg_bool(cfg, "require_detection", DEFAULT_GRASP.require_detection),
        grasp_target_offset_m=_cfg_xyz_offset(cfg, "grasp_target_offset_m")
        if "grasp_target_offset_m" in cfg
        else DEFAULT_GRASP.grasp_target_offset_m,
        grasp_offset_m=_parse_offset_xyz(cfg),
        gripper_open_width_m=_cfg_float(cfg, "gripper_open_width_m", DEFAULT_GRASP.gripper_open_width_m),
        gripper_close_width_m=_cfg_float(cfg, "gripper_close_width_m", DEFAULT_GRASP.gripper_close_width_m),
        gripper_force_n=_cfg_float(cfg, "gripper_force_n", DEFAULT_GRASP.gripper_force_n),
        grasp_forward_offset_m=_cfg_float(cfg, "grasp_forward_offset_m", DEFAULT_GRASP.grasp_forward_offset_m),
        lift_offset_m=_cfg_float(cfg, "lift_offset_m", DEFAULT_GRASP.lift_offset_m),
        motion_timeout_s=_cfg_float(cfg, "motion_timeout_s", DEFAULT_GRASP.motion_timeout_s),
        camera_frame_timeout_s=_cfg_float(
            cfg, "camera_frame_timeout_s", DEFAULT_GRASP.camera_frame_timeout_s
        ),
        step_pause_s=_cfg_float(cfg, "step_pause_s", DEFAULT_GRASP.step_pause_s),
        return_initial=_cfg_bool(cfg, "return_initial", DEFAULT_GRASP.return_initial),
        recover_on_failure=_cfg_bool(cfg, "recover_on_failure", DEFAULT_GRASP.recover_on_failure),
        standby_joint_pose=_parse_standby_joint_pose(cfg),
        target_source=target_source,
        fixed_tcp_pose=fixed_tcp_pose,
    )
