#!/usr/bin/env python3
"""nero_return_home — return arm to home via relay after releasing gripper."""
from __future__ import annotations

import logging
import time

from robonix_api import ATLAS, Err, Ok, Skill
from nero_return_home_mcp import NeroHome_Request, NeroHome_Response
from std_msgs_mcp import String

from .arm_client import ArmClient
from .config import ReturnHomeConfig, parse_return_home_config
from .sequence import run_return_home

logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
log = logging.getLogger("nero_return_home")

provider = Skill(id="nero_return_home", namespace="robonix/skill/nero_return_home")

_cfg: ReturnHomeConfig | None = None
_arms: list[ArmClient] = []


def _wait_for_primitive(provider_id: str, deadline_s: float = 60.0) -> None:
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        if ATLAS.query_primitives(id=provider_id):
            return
        time.sleep(1.0)
    raise RuntimeError(f"primitive {provider_id!r} not registered on atlas")


def _resolve_arm_clients(cfg: ReturnHomeConfig, deadline_s: float = 60.0) -> list[ArmClient]:
    consumer_id = provider.id
    resolved: dict[str, ArmClient] = {}
    deadline = time.time() + deadline_s
    for provider_id in cfg.arm_names:
        _wait_for_primitive(provider_id, deadline_s=max(0.0, deadline - time.time()))

    while time.time() < deadline and len(resolved) < len(cfg.arm_names):
        for provider_id in cfg.arm_names:
            if provider_id in resolved:
                continue
            try:
                resolved[provider_id] = ArmClient(consumer_id, provider_id)
                log.info("connected arm client: %s", provider_id)
            except Exception as exc:  # noqa: BLE001
                log.warning("arm %s not ready yet: %s", provider_id, exc)
        if len(resolved) < len(cfg.arm_names):
            time.sleep(2.0)

    missing = [name for name in cfg.arm_names if name not in resolved]
    if missing:
        raise RuntimeError(f"failed to resolve arm capabilities for: {missing}")
    return [resolved[provider_id] for provider_id in cfg.arm_names]


@provider.on_init
def init(cfg: dict):
    global _cfg
    try:
        _cfg = parse_return_home_config(cfg)
    except Exception as exc:  # noqa: BLE001
        return Err(f"invalid nero_return_home config: {exc}")

    log.info(
        "config mode=%s start_from=%s arms=%s close_gripper=%s open_wait=%.1fs motion_timeout=%.1fs",
        _cfg.execution_mode,
        _cfg.start_from,
        _cfg.arm_names,
        _cfg.close_gripper,
        _cfg.open_gripper_wait_s,
        _cfg.motion_timeout_s,
    )
    if _cfg.start_from == "standby":
        log.info("standby: %s", [round(v, 3) for v in _cfg.standby_joint_pose or []])
        if _cfg.standby_exit_joint_poses:
            for index, pose in enumerate(_cfg.standby_exit_joint_poses, start=1):
                log.info(
                    "standby_exit[%d/%d]: %s",
                    index,
                    len(_cfg.standby_exit_joint_poses),
                    [round(v, 3) for v in pose],
                )
        else:
            log.info("standby_exit: none")
    for index, pose in enumerate(_cfg.relay_joint_poses, start=1):
        log.info("relay[%d/%d]: %s", index, len(_cfg.relay_joint_poses), [round(v, 3) for v in pose])
    log.info("home: %s", [round(v, 3) for v in _cfg.home_joint_pose])
    return Ok()


@provider.on_activate
def activate():
    global _arms
    if _cfg is None:
        return Err("nero_return_home config missing")
    try:
        _arms = _resolve_arm_clients(_cfg)
    except Exception as exc:  # noqa: BLE001
        return Err(f"failed to connect arm primitives: {exc}")
    return Ok()


@provider.on_shutdown
def shutdown():
    global _arms
    for arm in _arms:
        arm.close()
    _arms = []
    return Ok()


@provider.mcp("robonix/skill/nero_return_home/return_home")
def return_home(_req: NeroHome_Request) -> NeroHome_Response:
    """Release gripper, optionally close, then relay -> home."""
    if _cfg is None or not _arms:
        return NeroHome_Response(
            success=False,
            message=String(data="nero_return_home is not initialized"),
        )
    try:
        run_return_home(_arms, _cfg)
    except Exception as exc:  # noqa: BLE001
        log.exception("return_home failed")
        return NeroHome_Response(success=False, message=String(data=str(exc)))
    arms_text = ", ".join(a.provider_id for a in _arms)
    return NeroHome_Response(
        success=True,
        message=String(data=f"return home completed on [{arms_text}]"),
    )


if __name__ == "__main__":
    provider.run()
