# SPDX-License-Identifier: MulanPSL-2.0
"""fine_align atlas bridge — resolve upstream deps + expose the align MCP tool.

Pipeline (atlas-resolved endpoints):
    nero_grasp.detect   (MCP)  -> robot_base-frame object position
    chassis.move        (MCP)  -> forward_m / rotate_deg burst
    chassis odom (ROS /odom)   -> closed-loop displacement feedback

The loop itself lives in controller.FineAlignController; this module only
wires transport (MCP + ROS) and lifecycle.
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import threading
import time
from typing import Any, Optional

from robonix_api import ATLAS, Err, Ok, Skill
from robonix_api.ros import RosBackend

from .controller import AlignConfig, FineAlignController

logging.basicConfig(
    level=os.environ.get("FINE_ALIGN_LOG_LEVEL", "INFO"),
    format="[fine_align] %(message)s",
)
log = logging.getLogger("fine_align")

fine_align_skill = Skill(id="fine_align", namespace="robonix/skill/fine_align")

REQUIRED_INPUTS = {
    "detect": ("robonix/skill/nero_grasp/detect", "mcp"),
    "move": ("robonix/primitive/chassis/move", "mcp"),
}

# Module-level state (between on_activate and the align handler).
_state_lock = threading.Lock()
_endpoints: Optional[dict[str, str]] = None
_cfg: AlignConfig = AlignConfig()
_odom_topic: str = "/odom"

# ── MCP client plumbing (background loop, mirrors pick_skill) ───────────────
_mcp_clients: dict[str, Any] = {}
_bg_loop: Optional[asyncio.AbstractEventLoop] = None
_bg_loop_thread: Optional[threading.Thread] = None
_bg_loop_lock = threading.Lock()


def _ensure_bg_loop() -> asyncio.AbstractEventLoop:
    global _bg_loop, _bg_loop_thread
    with _bg_loop_lock:
        if _bg_loop is not None and _bg_loop.is_running():
            return _bg_loop
        loop = asyncio.new_event_loop()

        def _runner() -> None:
            asyncio.set_event_loop(loop)
            loop.run_forever()

        t = threading.Thread(target=_runner, name="fine-align-mcp-loop", daemon=True)
        t.start()
        _bg_loop = loop
        _bg_loop_thread = t
        return loop


def _mcp_call_sync(url: str, tool: str, args: dict) -> dict:
    loop = _ensure_bg_loop()

    async def _call() -> dict:
        client = _mcp_clients.get(url)
        if client is None:
            from fastmcp import Client
            client = Client(url)
            _mcp_clients[url] = client
        async with client as c:
            result = await c.call_tool(tool, args)
        if not result.content:
            return {}
        txt = result.content[0].text
        try:
            return json.loads(txt)
        except Exception:  # noqa: BLE001
            return {"raw": txt}

    fut = asyncio.run_coroutine_threadsafe(_call(), loop)
    try:
        return fut.result()
    except Exception as exc:  # noqa: BLE001
        log.warning("mcp call %s failed: %s", tool, exc)
        return {"_error": str(exc)}


# ── endpoint resolution ──────────────────────────────────────────────────────
def _resolve_inputs(deadline_s: float = 60.0) -> dict[str, str]:
    resolved: dict[str, str] = {}
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        for key, (cid, transport) in REQUIRED_INPUTS.items():
            if key in resolved:
                continue
            try:
                cap = ATLAS.find_unique_capability(contract_id=cid, transport=transport)
                ch = fine_align_skill.connect_capability(cap, cid, transport)
                ep = ch.endpoint
                try:
                    ch.close()
                except Exception:  # noqa: BLE001
                    pass
                if ep:
                    resolved[key] = ep
                    log.info("resolved %s [%s] -> %s", cid, transport, ep)
            except Exception:  # noqa: BLE001
                continue
        if len(resolved) == len(REQUIRED_INPUTS):
            break
        time.sleep(2.0)

    missing = [k for k in REQUIRED_INPUTS if k not in resolved]
    if missing:
        raise RuntimeError(
            f"fine_align cannot resolve upstream deps on atlas: missing "
            f"{[REQUIRED_INPUTS[k][0] for k in missing]}")
    return resolved


# ── odom reader (ROS subscription) ───────────────────────────────────────────
_odom_lock = threading.Lock()
_latest_odom: dict[str, float] = {}


def _on_odom(msg: Any) -> None:
    x = float(msg.pose.pose.position.x)
    y = float(msg.pose.pose.position.y)
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    with _odom_lock:
        _latest_odom["x"] = x
        _latest_odom["y"] = y
        _latest_odom["yaw"] = yaw


def _read_odom() -> dict:
    with _odom_lock:
        return dict(_latest_odom)


def _start_odom() -> None:
    ros = RosBackend.get()
    try:
        from nav_msgs.msg import Odometry
        ros.create_subscription(Odometry, _odom_topic, _on_odom, "reliable")
        log.info("odom subscription active on %s", _odom_topic)
    except Exception as exc:  # noqa: BLE001
        log.warning("odom subscription failed on %s: %s", _odom_topic, exc)


# ── I/O adapters passed to the controller ────────────────────────────────────
def _detect_fn(object_name: str | None) -> dict:
    assert _endpoints is not None
    resp = _mcp_call_sync(_endpoints["detect"], "detect", {"object_name": object_name or ""})
    if "_error" in resp:
        return {"found": False, "message": resp["_error"]}
    robot_xyz = resp.get("robot_xyz")
    if not robot_xyz or len(robot_xyz) != 3:
        return {"found": False, "message": _extract_msg(resp, "message", "no robot_xyz")}
    return {
        "found": bool(resp.get("found", True)),
        "robot_xyz": [float(v) for v in robot_xyz],
        "label": resp.get("label", "object"),
        "confidence": float(resp.get("confidence", 0.0)),
        "message": _extract_msg(resp, "message", "ok"),
    }


def _move_fn(forward_m: float, rotate_deg: float) -> None:
    assert _endpoints is not None
    args = {"command": {"forward_m": float(forward_m), "rotate_deg": float(rotate_deg)}}
    resp = _mcp_call_sync(_endpoints["move"], "move", args)
    if "_error" in resp:
        raise RuntimeError(f"chassis.move failed: {resp['_error']}")
    # Response is std_msgs/String status; tolerate either a bare string or {data: ...}.
    status = resp.get("status", resp.get("raw", "{}"))
    if isinstance(status, dict):
        status = status.get("data", json.dumps(status))
    try:
        parsed = json.loads(status)
        if parsed.get("status") not in (None, "done"):
            log.warning("chassis.move ack: %s", parsed)
    except Exception:  # noqa: BLE001
        log.warning("chassis.move ack unparsed: %s", status)


def _extract_msg(resp: dict, key: str, default: str) -> str:
    value = resp.get(key, default)
    if isinstance(value, dict):
        value = value.get("data", default)
    return str(value)


# ── config parsing ───────────────────────────────────────────────────────────
_DEFAULTS = AlignConfig()


def _xyz(v: Any, default: tuple[float, float, float]) -> tuple[float, float, float]:
    if isinstance(v, list) and len(v) == 3:
        return tuple(float(x) for x in v)
    return default


def _rot(v: Any) -> tuple[tuple[float, float, float], ...]:
    if isinstance(v, list) and len(v) == 3 and all(isinstance(r, list) and len(r) == 3 for r in v):
        return tuple(tuple(float(x) for x in r) for r in v)
    return _DEFAULTS.base_to_robot_rotation


def _parse_config(cfg: dict) -> AlignConfig:
    zone = cfg.get("graspable_zone") or {}
    center = _xyz(zone.get("center") if isinstance(zone, dict) else None, _DEFAULTS.graspable_center)
    tolerance = _xyz(zone.get("tolerance") if isinstance(zone, dict) else None, _DEFAULTS.graspable_tolerance)

    return AlignConfig(
        graspable_center=center,
        graspable_tolerance=tolerance,
        base_to_robot_rotation=_rot(cfg.get("base_to_robot_rotation")),
        base_to_robot_translation=_xyz(cfg.get("base_to_robot_translation"), _DEFAULTS.base_to_robot_translation),
        max_iterations=int(cfg.get("max_iterations", _DEFAULTS.max_iterations)),
        timeout_s=float(cfg.get("timeout_s", _DEFAULTS.timeout_s)),
        forward_gain=float(cfg.get("forward_gain", _DEFAULTS.forward_gain)),
        rotate_gain=float(cfg.get("rotate_gain", _DEFAULTS.rotate_gain)),
        forward_step_max_m=float(cfg.get("forward_step_max_m", _DEFAULTS.forward_step_max_m)),
        rotate_step_max_deg=float(cfg.get("rotate_step_max_deg", _DEFAULTS.rotate_step_max_deg)),
        forward_tol_m=float(cfg.get("forward_tol_m", _DEFAULTS.forward_tol_m)),
        lateral_tol_m=float(cfg.get("lateral_tol_m", _DEFAULTS.lateral_tol_m)),
        move_settle_s=float(cfg.get("move_settle_s", _DEFAULTS.move_settle_s)),
        odom_tolerance_m=float(cfg.get("odom_tolerance_m", _DEFAULTS.odom_tolerance_m)),
        odom_yaw_tolerance_deg=float(cfg.get("odom_yaw_tolerance_deg", _DEFAULTS.odom_yaw_tolerance_deg)),
    )


# ── MCP tool ─────────────────────────────────────────────────────────────────
from fine_align_mcp import Align_Request, Align_Response  # noqa: E402


@fine_align_skill.mcp("robonix/skill/fine_align/align")
def align(req: Align_Request) -> Align_Response:
    """Fine-tune the chassis position so a detected object enters the arm
    graspable zone (closed loop). Call this AFTER nav2/p2pmov parked the
    base near the target and BEFORE nero_grasp.grasp.

    `object_name` is forwarded to nero_grasp.detect for VLM detection.
    """
    if _endpoints is None:
        raise RuntimeError("fine_align not active (upstream deps not resolved)")

    object_name = (req.object_name or "").strip() or None
    ctrl = FineAlignController(_cfg, _detect_fn, _move_fn, _read_odom)
    result = ctrl.align(object_name)
    return Align_Response(
        success=bool(result["success"]),
        iterations=float(result["iterations"]),
        robot_xyz=[float(v) for v in result["robot_xyz"]],
        message=str(result["message"]),
    )


# ── lifecycle ────────────────────────────────────────────────────────────────
@fine_align_skill.on_init
def init(cfg: dict):
    global _cfg, _odom_topic
    try:
        _cfg = _parse_config(cfg or {})
    except Exception as exc:  # noqa: BLE001
        return Err(f"invalid fine_align config: {exc}")
    _odom_topic = str(cfg.get("odom_topic", "/odom")) if cfg else "/odom"
    log.info("CMD_INIT ok: center=%s tol=%s max_iter=%d",
             _cfg.graspable_center, _cfg.graspable_tolerance, _cfg.max_iterations)
    return Ok()


@fine_align_skill.on_activate
def activate():
    global _endpoints
    with _state_lock:
        if _endpoints is not None:
            return Ok()
        try:
            _endpoints = _resolve_inputs()
        except RuntimeError as exc:
            return Err(str(exc))
    _start_odom()
    log.info("CMD_ACTIVATE ok: %s", list(_endpoints.keys()))
    return Ok()


@fine_align_skill.on_deactivate
def deactivate():
    global _endpoints
    with _state_lock:
        _endpoints = None
        _mcp_clients.clear()
    log.info("CMD_DEACTIVATE ok")
    return Ok()


def main() -> int:
    fine_align_skill.run()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
