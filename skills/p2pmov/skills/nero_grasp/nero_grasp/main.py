#!/usr/bin/env python3
"""nero_grasp — pick-and-place style grasp skill for AgileX Nero arms."""
from __future__ import annotations

import logging
import time

from robonix_api import ATLAS, Err, Ok, Skill
from nero_grasp_mcp import NeroGrasp_Request, NeroGrasp_Response
from std_msgs_mcp import String

from .arm_client import ArmClient
from .calib import load_camera_to_robot
from .camera_client import CameraClient
from .config import GraspConfig, parse_grasp_config
from .sequence import run_grasp

logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
log = logging.getLogger("nero_grasp")

provider = Skill(id="nero_grasp", namespace="robonix/skill/nero_grasp")

_cfg: GraspConfig | None = None
_calib = None
_arms: list[ArmClient] = []
_camera: CameraClient | None = None


def _wait_for_primitive(provider_id: str, deadline_s: float = 60.0) -> None:
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        if ATLAS.query_primitives(id=provider_id):
            return
        time.sleep(1.0)
    raise RuntimeError(f"primitive {provider_id!r} not registered on atlas")


def _resolve_arm_clients(grasp_cfg: GraspConfig, deadline_s: float = 60.0) -> list[ArmClient]:
    consumer_id = provider.id
    resolved: dict[str, ArmClient] = {}
    deadline = time.time() + deadline_s
    for provider_id in grasp_cfg.arm_names:
        _wait_for_primitive(provider_id, deadline_s=max(0.0, deadline - time.time()))

    while time.time() < deadline and len(resolved) < len(grasp_cfg.arm_names):
        for provider_id in grasp_cfg.arm_names:
            if provider_id in resolved:
                continue
            try:
                resolved[provider_id] = ArmClient(consumer_id, provider_id)
                log.info("connected arm client: %s", provider_id)
            except Exception as exc:  # noqa: BLE001
                log.warning("arm %s not ready yet: %s", provider_id, exc)
        if len(resolved) < len(grasp_cfg.arm_names):
            time.sleep(2.0)

    missing = [name for name in grasp_cfg.arm_names if name not in resolved]
    if missing:
        raise RuntimeError(f"failed to resolve arm capabilities for: {missing}")
    return [resolved[provider_id] for provider_id in grasp_cfg.arm_names]


@provider.on_init
def init(cfg: dict):
    global _cfg, _calib
    try:
        _cfg = parse_grasp_config(cfg)
    except Exception as exc:  # noqa: BLE001
        return Err(f"invalid nero_grasp config: {exc}")

    try:
        _calib = load_camera_to_robot(_cfg.calib_path)
    except FileNotFoundError as exc:
        needs_calib = _cfg.target_source == "vlm" and _cfg.require_detection
        if needs_calib:
            return Err(str(exc))
        log.warning("%s — continuing without camera calibration", exc)
        _calib = None

    log.info(
        "config mode=%s target_source=%s arms=%s step_pause=%.1fs return_initial=%s recover_on_failure=%s "
        "camera=%s detector=%s require_detection=%s calib=%s offset=%s vlm=%s",
        _cfg.execution_mode,
        _cfg.target_source,
        _cfg.arm_names,
        _cfg.step_pause_s,
        _cfg.return_initial,
        _cfg.recover_on_failure,
        _cfg.camera_provider_id or "none",
        _cfg.detector_provider_id or _cfg.detector_settings.get("detector", "vlm"),
        _cfg.require_detection,
        _cfg.calib_path,
        [round(v, 3) for v in _cfg.grasp_offset_m],
        _cfg.detector_settings.get("vlm_model")
        or _cfg.detector_settings.get("model")
        or "from-manifest/env",
    )
    if _cfg.initial_joint_pose is None:
        log.info("initial pose: keep current joints")
    else:
        for index, pose in enumerate(_cfg.relay_joint_poses, start=1):
            log.info("relay[%d/%d]: %s", index, len(_cfg.relay_joint_poses), [round(v, 3) for v in pose])
        log.info("initial pose: %s", [round(v, 3) for v in _cfg.initial_joint_pose])
    if _cfg.standby_joint_pose is None:
        log.info("standby pose: disabled")
    else:
        log.info("standby pose: %s", [round(v, 3) for v in _cfg.standby_joint_pose])
    if _cfg.target_source == "fixed_tcp":
        log.info("fixed tcp pose: %s", [round(v, 4) for v in _cfg.fixed_tcp_pose or []])
    return Ok()


@provider.on_activate
def activate():
    global _arms, _camera
    if _cfg is None:
        return Err("nero_grasp config missing")
    try:
        _arms = _resolve_arm_clients(_cfg)
        if _cfg.target_source == "vlm":
            _camera = CameraClient(
                provider.id,
                provider_id=_cfg.camera_provider_id,
                calib=_calib,
                detector_provider_id=_cfg.detector_provider_id,
                detector_settings=_cfg.detector_settings,
                frame_timeout_s=_cfg.camera_frame_timeout_s,
            )
        else:
            _camera = None
            log.info("target_source=fixed_tcp — camera/VLM disabled")
    except Exception as exc:  # noqa: BLE001
        return Err(f"failed to connect arm primitives: {exc}")
    return Ok()


@provider.on_shutdown
def shutdown():
    global _arms, _camera
    for arm in _arms:
        arm.close()
    _arms = []
    if _camera is not None:
        _camera.close()
        _camera = None
    return Ok()


@provider.mcp("robonix/skill/nero_grasp/grasp")
def grasp(req: NeroGrasp_Request) -> NeroGrasp_Response:
    """Execute the Nero grasp sequence on configured arm(s).

    Flow: initial pose -> open gripper -> camera detect -> move_l approach (+offset) ->
    close gripper -> return initial.

    Args:
        object_name: target object label from pilot (e.g. "不锈钢保温杯").
            Empty uses manifest detector_settings.instruction.
    """
    if _cfg is None or not _arms:
        return NeroGrasp_Response(
            success=False,
            message=String(data="nero_grasp is not initialized"),
        )
    if _cfg.target_source == "vlm" and _camera is None:
        return NeroGrasp_Response(
            success=False,
            message=String(data="nero_grasp camera is not connected"),
        )
    object_name = (req.object_name or "").strip() or None
    if object_name:
        log.info("grasp object_name=%r", object_name)
    try:
        run_grasp(_arms, _cfg, _camera, object_name=object_name)
    except Exception as exc:  # noqa: BLE001
        log.exception("grasp failed")
        return NeroGrasp_Response(success=False, message=String(data=str(exc)))
    arms_text = ", ".join(a.provider_id for a in _arms)
    return NeroGrasp_Response(
        success=True,
        message=String(data=f"grasp sequence completed on [{arms_text}]"),
    )


if __name__ == "__main__":
    provider.run()
