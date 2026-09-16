# SPDX-License-Identifier: MulanPSL-2.0
"""guided_tour atlas bridge — resolve upstream deps + expose the tour MCP tools.

Pipeline (atlas-resolved endpoints, same pattern as fine_align):
    navigation/navigate  (MCP)  -> start a nav2 goal (PoseStamped)
    navigation/navigate/status (MCP) -> poll nav2 until terminal
    navigation/navigate/cancel (MCP) -> abort the active nav2 goal
    speech/speak         (MCP)  -> speak an announcement aloud

The stop-by-stop loop lives in controller.TourController; this module only
wires transport (MCP) and lifecycle, plus a small per-tour run registry so
run/status/cancel can address an individual tour.
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import re
import threading
import time
import uuid
from typing import Any, Optional

from robonix_api import ATLAS, Err, Ok, Skill

from .config import GuidedTourConfig, TourStop, parse_guided_tour_config
from .controller import TourController

logging.basicConfig(
    level=os.environ.get("GUIDED_TOUR_LOG_LEVEL", "INFO"),
    format="[guided_tour] %(message)s",
)
log = logging.getLogger("guided_tour")

guided_tour_skill = Skill(id="guided_tour", namespace="robonix/skill/guided_tour")

REQUIRED_INPUTS = {
    "navigate": ("robonix/service/navigation/navigate", "mcp"),
    "nav_status": ("robonix/service/navigation/navigate/status", "mcp"),
    "nav_cancel": ("robonix/service/navigation/navigate/cancel", "mcp"),
    "speak": ("robonix/service/speech/speak", "mcp"),
}

# Module-level state.
_state_lock = threading.Lock()
_endpoints: Optional[dict[str, str]] = None
_cfg: Optional[GuidedTourConfig] = None

_runs_lock = threading.Lock()
_runs: dict[str, TourController] = {}
_latest_run_id = ""

# ── MCP client plumbing (background loop, mirrors fine_align) ────────────────
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

        thread = threading.Thread(target=_runner, name="guided-tour-mcp-loop", daemon=True)
        thread.start()
        _bg_loop = loop
        _bg_loop_thread = thread
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
                ch = guided_tour_skill.connect_capability(cap, cid, transport)
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
            f"guided_tour cannot resolve upstream deps on atlas: missing "
            f"{[REQUIRED_INPUTS[k][0] for k in missing]}")
    return resolved


# ── upstream I/O adapters passed to the controller ───────────────────────────
def _navigate_fn(x: float, y: float, yaw: float, frame_id: str) -> dict:
    assert _endpoints is not None
    goal = {
        "header": {"frame_id": frame_id},
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": 0.0},
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(float(yaw) / 2.0),
                "w": math.cos(float(yaw) / 2.0),
            },
        },
    }
    resp = _mcp_call_sync(_endpoints["navigate"], "navigate", {"goal": goal})
    if "_error" in resp:
        return {"accepted": False, "run_id": "", "detail": resp["_error"]}
    return {
        "accepted": bool(resp.get("accepted", False)),
        "run_id": str(resp.get("run_id", "")),
        "detail": str(resp.get("detail", "")),
    }


def _status_fn(run_id: str) -> dict:
    assert _endpoints is not None
    resp = _mcp_call_sync(_endpoints["nav_status"], "status", {"run_id": run_id})
    if "_error" in resp:
        return {"state": "", "detail": resp["_error"]}
    return {"state": str(resp.get("state", "")), "detail": str(resp.get("detail", ""))}


def _cancel_nav_fn(run_id: str) -> dict:
    assert _endpoints is not None
    resp = _mcp_call_sync(_endpoints["nav_cancel"], "cancel", {"run_id": run_id})
    if "_error" in resp:
        return {"accepted": False, "detail": resp["_error"]}
    return {"accepted": bool(resp.get("accepted", False)), "detail": str(resp.get("detail", ""))}


def _speak_fn(text: str) -> None:
    assert _endpoints is not None and _cfg is not None
    resp = _mcp_call_sync(
        _endpoints["speak"], "speak",
        {"target": _cfg.speech_target, "text": text},
    )
    if "_error" in resp:
        raise RuntimeError(f"speech.speak failed: {resp['_error']}")
    if not resp.get("ok"):
        raise RuntimeError(f"speech.speak failed: {resp.get('detail', 'ok=false')}")


# ── tour run registry ────────────────────────────────────────────────────────
def _split_names(raw: str) -> list[str]:
    if not raw or not raw.strip():
        return []
    return [p for p in re.split(r"[,\s，、]+", raw.strip()) if p]


def _begin_tour(stop_names: str) -> str:
    global _latest_run_id
    if _cfg is None or _endpoints is None:
        raise RuntimeError("guided_tour not active (upstream deps not resolved)")

    stops: tuple[TourStop, ...] = _cfg.stops
    names = _split_names(stop_names)
    if names:
        by_name = {s.name: s for s in _cfg.stops}
        missing = [n for n in names if n not in by_name]
        if missing:
            raise RuntimeError(f"unknown stop names: {missing} (known: {sorted(by_name)})")
        stops = tuple(by_name[n] for n in names)
    if not stops:
        raise RuntimeError("no stops to visit")

    with _runs_lock:
        for rid, c in _runs.items():
            if c.snapshot()["state"] in ("PENDING", "RUNNING"):
                raise RuntimeError(f"a tour is already running (run_id={rid}); cancel it first")
        run_id = "tour-" + uuid.uuid4().hex[:8]
        controller = TourController(
            stops, _cfg, _navigate_fn, _status_fn, _speak_fn, _cancel_nav_fn
        )
        _runs[run_id] = controller
        _latest_run_id = run_id
        threading.Thread(
            target=controller.run, name=f"guided-tour-{run_id}", daemon=True
        ).start()
    log.info("started tour %s with %d stops", run_id, len(stops))
    return run_id


def _lookup(run_id: str) -> TourController:
    with _runs_lock:
        rid = run_id or _latest_run_id
        controller = _runs.get(rid)
    if controller is None:
        raise RuntimeError(f"unknown run_id {rid!r}")
    return controller


# ── MCP tools ────────────────────────────────────────────────────────────────
from guided_tour_mcp import (  # noqa: E402
    CancelTour_Request, CancelTour_Response,
    GetTourStatus_Request, GetTourStatus_Response,
    RunTour_Request, RunTour_Response,
)


@guided_tour_skill.mcp("robonix/skill/guided_tour/run")
def run_tour(req: RunTour_Request) -> RunTour_Response:
    """Start the guided tour (导览): visit each fixed stop in order, then speak
    its announcement. Returns immediately with a run_id — poll run/status,
    abort with run/cancel.

    `stop_names` is optional; empty means visit every configured stop in
    order, otherwise it is a whitespace/comma-separated subset of stop names.
    """
    run_id = _begin_tour(req.stop_names)
    return RunTour_Response(accepted=True, run_id=run_id, detail="tour started")


@guided_tour_skill.mcp("robonix/skill/guided_tour/run/status")
def tour_status(req: GetTourStatus_Request) -> GetTourStatus_Response:
    """Poll a tour run. Empty run_id means the most recent tour. `state` is
    PENDING | RUNNING | SUCCEEDED | FAILED | CANCELED; `phase` is
    idle | starting | navigating | announcing | finished | canceled | error."""
    snap = _lookup(req.run_id).snapshot()
    return GetTourStatus_Response(
        known=True,
        state=str(snap["state"]),
        phase=str(snap["phase"]),
        stop_index=int(snap["stop_index"]),
        total_stops=int(snap["total_stops"]),
        detail=str(snap["detail"]),
    )


@guided_tour_skill.mcp("robonix/skill/guided_tour/run/cancel")
def tour_cancel(req: CancelTour_Request) -> CancelTour_Response:
    """Abort a tour. Cancels the active nav2 goal and stops the loop. Empty
    run_id means the most recent tour."""
    _lookup(req.run_id).request_cancel()
    return CancelTour_Response(accepted=True, detail="cancel requested")


# ── lifecycle ────────────────────────────────────────────────────────────────
@guided_tour_skill.on_init
def init(cfg: dict):
    global _cfg
    try:
        _cfg = parse_guided_tour_config(cfg)
    except Exception as exc:  # noqa: BLE001
        return Err(f"invalid guided_tour config: {exc}")
    log.info(
        "CMD_INIT ok: %d stops frame_id=%s speech_target=%s",
        len(_cfg.stops), _cfg.frame_id, _cfg.speech_target or "(default)",
    )
    return Ok()


@guided_tour_skill.on_activate
def activate():
    global _endpoints
    with _state_lock:
        if _endpoints is not None:
            return Ok()
        try:
            _endpoints = _resolve_inputs()
        except RuntimeError as exc:
            return Err(str(exc))
    log.info("CMD_ACTIVATE ok: %s", list(_endpoints.keys()))
    return Ok()


@guided_tour_skill.on_deactivate
def deactivate():
    global _endpoints
    with _state_lock:
        _endpoints = None
        _mcp_clients.clear()
    log.info("CMD_DEACTIVATE ok")
    return Ok()


def main() -> int:
    guided_tour_skill.run()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
