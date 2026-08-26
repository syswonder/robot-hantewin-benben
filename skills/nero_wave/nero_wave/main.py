#!/usr/bin/env python3
"""nero_wave — execute configured wave motion on AgileX Nero arms."""
from __future__ import annotations

import logging
import time

from nero_wave_mcp import NeroWave_Request, NeroWave_Response
from robonix_api import ATLAS, Err, Ok, Skill
from std_msgs_mcp import String

from .arm_client import ArmClient
from .config import WaveConfig, parse_wave_config
from .sequence import run_wave

logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
log = logging.getLogger("nero_wave")

provider = Skill(id="nero_wave", namespace="robonix/skill/nero_wave")

_cfg: WaveConfig | None = None
_arms: list[ArmClient] = []


def _wait_for_primitive(provider_id: str, deadline_s: float = 60.0) -> None:
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        if ATLAS.query_primitives(id=provider_id):
            return
        time.sleep(1.0)
    raise RuntimeError(f"primitive {provider_id!r} not registered on atlas")


def _resolve_arm_clients(cfg: WaveConfig, deadline_s: float = 60.0) -> list[ArmClient]:
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
        _cfg = parse_wave_config(cfg)
    except Exception as exc:  # noqa: BLE001
        return Err(f"invalid nero_wave config: {exc}")

    log.info(
        "config mode=%s arms=%s steps=%d start_at_home=%s step_pause=%.1fs motion_timeout=%.1fs",
        _cfg.execution_mode,
        _cfg.arm_names,
        len(_cfg.steps),
        _cfg.start_at_home,
        _cfg.step_pause_s,
        _cfg.motion_timeout_s,
    )
    log.info("home: %s", [round(v, 3) for v in _cfg.home_joint_pose])
    for i, step in enumerate(_cfg.steps):
        log.info("  [%d] %s (%s)", i + 1, step.name, step.kind)
    return Ok()


@provider.on_activate
def activate():
    global _arms
    if _cfg is None:
        return Err("nero_wave config missing")
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


@provider.mcp("robonix/skill/nero_wave/wave")
def wave(_req: NeroWave_Request) -> NeroWave_Response:
    """Run the configured wave motion sequence on configured arm(s)."""
    if _cfg is None or not _arms:
        return NeroWave_Response(
            success=False,
            message=String(data="nero_wave is not initialized"),
        )
    try:
        run_wave(_arms, _cfg)
    except Exception as exc:  # noqa: BLE001
        log.exception("wave failed")
        return NeroWave_Response(success=False, message=String(data=str(exc)))
    arms_text = ", ".join(a.provider_id for a in _arms)
    return NeroWave_Response(
        success=True,
        message=String(data=f"wave sequence completed on [{arms_text}]"),
    )


if __name__ == "__main__":
    provider.run()
