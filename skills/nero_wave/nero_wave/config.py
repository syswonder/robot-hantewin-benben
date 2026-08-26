"""Wave motion configuration for nero_wave skill."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal


@dataclass(frozen=True)
class MotionStep:
    name: str
    kind: Literal["joint", "tcp"]
    values: list[float]
    linear: bool = False


@dataclass(frozen=True)
class WaveConfig:
    execution_mode: str
    arm_names: list[str]
    home_joint_pose: list[float]
    steps: list[MotionStep]
    motion_timeout_s: float = 20.0
    step_pause_s: float = 0.5
    start_at_home: bool = True
    default_tcp_linear: bool = False


DEFAULT_WAVE = WaveConfig(
    execution_mode="single",
    arm_names=["left_arm"],
    home_joint_pose=[-0.11, 1.42, 0.02, -0.16, 0.05, 0.04, 0.03],
    steps=[],
    motion_timeout_s=20.0,
    step_pause_s=0.5,
    start_at_home=True,
    default_tcp_linear=False,
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


def _as_float_list(value: Any, *, size: int, name: str) -> list[float]:
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} must be a list of {size} floats")
    return [float(v) for v in value]


def _parse_joint_pose(raw: dict | list | None, *, name: str) -> list[float]:
    if raw is None:
        raise ValueError(f"{name} is required in nero_wave config")
    if isinstance(raw, list):
        return _as_float_list(raw, size=7, name=name)
    if isinstance(raw, dict):
        if "joint_pose" in raw:
            return _as_float_list(raw["joint_pose"], size=7, name=f"{name}.joint_pose")
        if "joints" in raw:
            return _as_float_list(raw["joints"], size=7, name=f"{name}.joints")
    raise ValueError(f"{name} must be a 7-element list or {{joint_pose: [...]}}")


def _infer_step_kind(values: list[float], *, step_name: str) -> Literal["joint", "tcp"]:
    if len(values) == 7:
        return "joint"
    if len(values) == 3:
        return "tcp"
    raise ValueError(
        f"step {step_name!r}: pose length must be 7 (joint) or 3 (tcp xyz), got {len(values)}"
    )


def _parse_step(raw: dict | list, *, index: int) -> MotionStep:
    if isinstance(raw, list):
        values = [float(v) for v in raw]
        name = f"step_{index + 1}"
        kind = _infer_step_kind(values, step_name=name)
        return MotionStep(name=name, kind=kind, values=values)

    if not isinstance(raw, dict):
        raise ValueError(f"steps[{index}] must be a pose list or mapping")

    name = str(raw.get("name", f"step_{index + 1}")).strip() or f"step_{index + 1}"
    linear = bool(raw.get("linear", False))

    if "joint_pose" in raw:
        values = _as_float_list(raw["joint_pose"], size=7, name=f"steps[{index}].joint_pose")
        return MotionStep(name=name, kind="joint", values=values, linear=linear)
    if "tcp_pose" in raw:
        tcp_raw = raw["tcp_pose"]
        if isinstance(tcp_raw, list) and len(tcp_raw) == 6:
            values = _as_float_list(tcp_raw, size=6, name=f"steps[{index}].tcp_pose")
            return MotionStep(name=name, kind="tcp", values=values, linear=linear)
        values = _as_float_list(tcp_raw, size=3, name=f"steps[{index}].tcp_pose")
        return MotionStep(name=name, kind="tcp", values=values, linear=linear)
    if "pose" in raw:
        values = [float(v) for v in raw["pose"]]
        if len(values) == 6:
            return MotionStep(name=name, kind="tcp", values=values, linear=linear)
        kind = _infer_step_kind(values, step_name=name)
        return MotionStep(name=name, kind=kind, values=values, linear=linear)

    raise ValueError(f"steps[{index}] ({name!r}) needs pose, joint_pose, or tcp_pose")


def parse_wave_config(cfg: dict | None) -> WaveConfig:
    """Parse Robonix manifest config into WaveConfig."""
    cfg = cfg or {}
    execution_mode = _cfg_str(cfg, "execution_mode", DEFAULT_WAVE.execution_mode).lower()
    if execution_mode not in {"single", "dual"}:
        raise ValueError("execution_mode must be 'single' or 'dual'")

    arms_cfg = cfg.get("arms")
    if isinstance(arms_cfg, list) and arms_cfg:
        arm_names = [str(v).strip() for v in arms_cfg if str(v).strip()]
    elif execution_mode == "dual":
        arm_names = ["left_arm", "right_arm"]
    else:
        arm_names = [_cfg_str(cfg, "active_arm", DEFAULT_WAVE.arm_names[0])]

    if execution_mode == "single" and len(arm_names) != 1:
        raise ValueError(f"execution_mode=single requires exactly one arm, got {arm_names!r}")
    if not arm_names:
        raise ValueError("at least one arm must be configured")

    steps_raw = cfg.get("steps")
    if not isinstance(steps_raw, list) or not steps_raw:
        raise ValueError("steps must be a non-empty list in nero_wave config")

    steps = [_parse_step(item, index=i) for i, item in enumerate(steps_raw)]
    return WaveConfig(
        execution_mode=execution_mode,
        arm_names=arm_names,
        home_joint_pose=_parse_joint_pose(cfg.get("home"), name="home"),
        steps=steps,
        motion_timeout_s=_cfg_float(cfg, "motion_timeout_s", DEFAULT_WAVE.motion_timeout_s),
        step_pause_s=_cfg_float(cfg, "step_pause_s", DEFAULT_WAVE.step_pause_s),
        start_at_home=_cfg_bool(cfg, "start_at_home", DEFAULT_WAVE.start_at_home),
        default_tcp_linear=_cfg_bool(cfg, "default_tcp_linear", DEFAULT_WAVE.default_tcp_linear),
    )
