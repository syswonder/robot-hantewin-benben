# SPDX-License-Identifier: MulanPSL-2.0
"""p2pmov atlas bridge — capability + contract-typed MCP tools.

Tools are typed against the codegen Request/Response dataclasses for the
p2pmov/srv/* contracts (P2PMove, P2PStatus, P2PCancel). The JSON Schema
each MCP tool advertises to the LLM is derived from those classes via
codegen introspection — no hand-written schemas.

The skill has no atlas-resolved inputs: it talks to the chassis directly
through the vendor TBox SDK (task_distribution), so there is no
resolve_inputs() step like explore has for map/nav.
"""
from __future__ import annotations

import logging
import os
import sys

from robonix_api import Deferred, Err, Ok, Skill

from .controller import P2PController

logging.basicConfig(
    level=os.environ.get("P2PMOV_LOG_LEVEL", "INFO"),
    format="[p2pmov] %(levelname)s %(message)s",
)
log = logging.getLogger("p2pmov")

p2pmov_skill = Skill(id="p2pmov", namespace="robonix/skill/p2pmov")
ctrl: P2PController | None = None
# Validated config stashed by on_init; on_activate takes no args, so the
# controller is built from this in CMD_ACTIVATE.
_init_cfg: dict = {}


# ── MCP tools (typed against codegen Request/Response) ──────────────────────
from p2pmov_mcp import (  # noqa: E402
    P2PCancel_Request, P2PCancel_Response,
    P2PMove_Request, P2PMove_Response,
    P2PStatus_Request, P2PStatus_Response,
)


@p2pmov_skill.mcp("robonix/skill/p2pmov/move")
def move(req: P2PMove_Request) -> P2PMove_Response:
    """Move the robot to a point — use THIS tool when the user asks to
    go to a named point or a coordinate ("前往饮水机", "去原点",
    "前往坐标 (20.2, 2.28, -1.57)").

    Provide EITHER `point_name` (a key of the deployment `points`
    config) OR raw coordinates (`x`, `y`, `yaw`); `point_name` takes
    priority when both are present. The goal is dispatched via the TBox
    SDK task_distribution() and the chassis navigates autonomously on
    its onboard vendor map.

    IMPORTANT — do NOT handle these destinations through scene
    goal_near / goal_room + navigation/navigate: that chain runs the
    nav2 stack on the RTAB-Map coordinate frame, while this skill runs
    the vendor onboard map. The two frames do not match. scene
    goal_near / goal_room are only for scene-registered objects and
    rooms (followed by navigation/navigate).

    Returns immediately with a run_id — poll
    robonix/skill/p2pmov/move/status with that run_id to track arrival,
    cancel with robonix/skill/p2pmov/move/cancel.

    Examples:
      {"point_name": "饮水机"}
      {"x": 20.20, "y": 2.28, "yaw": -1.57}
    """
    if ctrl is None:
        raise RuntimeError("controller not initialized")
    ok, run_id, message = ctrl.move(
        point_name=(req.point_name or "").strip(),
        x=float(req.x), y=float(req.y), yaw=float(req.yaw),
    )
    return P2PMove_Response(accepted=ok, run_id=run_id, message=message)


@p2pmov_skill.mcp("robonix/skill/p2pmov/move/status")
def status(req: P2PStatus_Request) -> P2PStatus_Response:
    """Poll the navigation task started by robonix/skill/p2pmov/move.

    `run_id` is informational (the TBox SDK reports one global task
    stream); empty means the most recent task. Returns
    {known, state, task_state, run_state, distance_m, detail}; `state` is
    one of PENDING | RUNNING | SUCCEEDED | FAILED | CANCELED (terminal:
    SUCCEEDED / FAILED / CANCELED).
    """
    if ctrl is None:
        raise RuntimeError("controller not initialized")
    s = ctrl.status()
    return P2PStatus_Response(
        known=s["known"], state=s["state"],
        task_state=int(s["task_state"]), run_state=int(s["run_state"]),
        distance_m=float(s["distance_m"]), detail=s["detail"],
    )


@p2pmov_skill.mcp("robonix/skill/p2pmov/move/cancel")
def cancel(req: P2PCancel_Request) -> P2PCancel_Response:
    """Abort the navigation task started by robonix/skill/p2pmov/move
    via TBox task_control(CANCEL). Idempotent."""
    if ctrl is None:
        raise RuntimeError("controller not initialized")
    ok, message = ctrl.cancel()
    return P2PCancel_Response(ok=ok, message=message)


# ── lifecycle ────────────────────────────────────────────────────────────────
@p2pmov_skill.on_init
def init(cfg):
    """CMD_INIT: light — parse + validate config only. The TBox SDK is
    NOT touched here (authorization is heavy and may take up to ~30 s);
    it is initialized on the first CMD_ACTIVATE."""
    global _init_cfg
    token = cfg.get("token") or os.environ.get("P2PMOV_TOKEN", "")
    map_name = cfg.get("map_name") or os.environ.get("P2PMOV_MAP_NAME", "")
    if not token:
        return Err("token required: set config.token (or P2PMOV_TOKEN)")
    if not map_name:
        return Err("map_name required: set config.map_name — the onboard "
                   "TBox map file, e.g. bdyjd_27_0803.bin")
    points = cfg.get("points") or {}
    if not isinstance(points, dict):
        return Err(f"config.points must be a dict {{name: coords}}, "
                   f"got {type(points).__name__}")
    _init_cfg = {
        "token": token,
        "map_name": map_name,
        "points": points,
        "speed_ratio": int(cfg.get("speed_ratio", 3)),
    }
    log.info("CMD_INIT ok (map_name=%s, %d named points)", map_name, len(points))
    return Ok()


@p2pmov_skill.on_activate
def activate():
    """CMD_ACTIVATE: heavy — init the TBox SDK and wait for auth. The
    executor only sends this when there's a request to satisfy. Retryable:
    a failure returns Deferred so the next LLM call re-triggers CMD_ACTIVATE."""
    global ctrl
    if ctrl is not None:
        log.info("CMD_ACTIVATE — already runnable, no-op")
        return Ok()
    try:
        c = P2PController(
            token=_init_cfg["token"],
            map_name=_init_cfg["map_name"],
            points=_init_cfg["points"],
            speed_ratio=_init_cfg["speed_ratio"],
        )
        c.start_runtime()
    except Exception as exc:  # noqa: BLE001
        log.exception("CMD_ACTIVATE failed")
        return Deferred(f"TBox SDK not ready: {exc}")
    ctrl = c
    log.info("CMD_ACTIVATE ok — TBox SDK authorized, %d named points",
             len(ctrl.known_points))
    return Ok()


@p2pmov_skill.on_deactivate
def deactivate():
    """CMD_DEACTIVATE: drop the SDK handle (no deinit exists; see
    controller.stop_runtime). Safe to call repeatedly."""
    global ctrl
    if ctrl is None:
        return Ok()
    try:
        ctrl.stop_runtime()
    finally:
        ctrl = None
    log.info("CMD_DEACTIVATE ok")
    return Ok()


def main() -> int:
    code = 0
    try:
        p2pmov_skill.run()
    except BaseException as exc:  # noqa: BLE001
        log.exception("skill exited abnormally: %s", exc)
        code = 1
    finally:
        # libtbox_sdk_cpp.so has no deinit API: its static singletons are
        # destroyed at normal Python exit while its background threads are
        # still running -> use-after-free segfault. os._exit() skips C++
        # static destructors entirely (see htysdk notes).
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)


if __name__ == "__main__":
    main()
