# SPDX-License-Identifier: MulanPSL-2.0
"""Point-to-point navigation controller — TBoxClient wrapper + target resolution.

Threading:
  - The TBox SDK runs its own background threads (connection, auth,
    heartbeat, task-status callbacks). We only touch the client from MCP
    tool handlers; state updates arrive on the SDK's callback thread and
    are snapshotted under a lock.

State mapping (vendor task_state -> canonical executor state). The executor
polls move/status until a terminal value and treats unknown states as
RUNNING, so the terminal words MUST map — otherwise a finished task never
reaches SUCCEEDED/FAILED and the executor polls forever:
    0 idle    -> PENDING
    1 running -> RUNNING
    2 success -> SUCCEEDED
    3 paused  -> RUNNING   (not terminal — waiting for resume)
    4 failed  -> FAILED
    5 canceled-> CANCELED
"""
from __future__ import annotations

import logging
import threading
import uuid
from typing import Any, Optional, Tuple

log = logging.getLogger("p2pmov.controller")


def canonical_state(task_state: int) -> str:
    """Map a vendor task_state byte to a canonical executor state name."""
    return {
        0: "PENDING",     # idle
        1: "RUNNING",     # running
        2: "SUCCEEDED",   # success
        3: "RUNNING",     # paused — still active
        4: "FAILED",      # failed
        5: "CANCELED",    # canceled
    }.get(int(task_state), "RUNNING")


def normalize_point(value: Any, name: str) -> Tuple[float, float, float]:
    """Normalize a configured point to (x, y, yaw).

    Accepts ``{"x":.., "y":.., "yaw":..}`` or ``[x, y, yaw]`` / ``(x, y, yaw)``.
    Raises ValueError with the point name on malformed entries.
    """
    if isinstance(value, dict):
        try:
            return (float(value["x"]), float(value["y"]), float(value["yaw"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"point {name!r} must be {{'x':.., 'y':.., 'yaw':..}}, "
                f"got {value!r}"
            ) from exc
    try:
        xs = list(value)
    except TypeError:
        xs = None
    if xs is None or len(xs) != 3:
        raise ValueError(
            f"point {name!r} must be a dict or [x, y, yaw], got {value!r}"
        )
    try:
        return (float(xs[0]), float(xs[1]), float(xs[2]))
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"point {name!r} coordinates must be numeric: {xs!r}"
        ) from exc


class P2PController:
    """One instance per skill process; serializes goal dispatch with a lock.

    ``points`` (the deployment's named-point dict) is normalized once in
    the constructor so a misconfigured entry fails at activate time with a
    clear message instead of at the first LLM call.
    """

    def __init__(self, *, token: str, map_name: str,
                 points: Optional[dict] = None, speed_ratio: int = 3):
        self._token = token
        self._map_name = map_name
        self._points: dict[str, Tuple[float, float, float]] = {
            str(k): normalize_point(v, str(k))
            for k, v in (points or {}).items()
        }
        self._speed_ratio = int(speed_ratio)
        self._lock = threading.Lock()
        self._client: Optional[Any] = None          # TBoxClient (lazy import)
        self._last_task: Optional[Any] = None       # SDK_BACK_TASK_STATU_T snapshot
        self._task_uuid: str = ""                   # vendor task uuid from snapshot

    # ── runtime (on_activate / on_deactivate) ───────────────────────────
    def start_runtime(self, timeout_s: float = 30.0) -> None:
        """Initialize the TBox SDK and wait for authorization.

        The SDK is async: commands issued before ``wait_until_ready()`` are
        rejected (-1) with "未授权". Raising here leaves the skill in a
        retryable state — the executor re-sends CMD_ACTIVATE on the next
        LLM call.
        """
        if self._client is not None:
            return
        from .tbox_sdk import TBoxClient

        client = TBoxClient()
        client.initialize(self._token)
        if not client.wait_until_ready(timeout_s):
            raise RuntimeError(
                f"TBox SDK not authorized within {timeout_s:.0f}s "
                f"(last_login_status={client.last_login_status})"
            )
        client.register_state_callback(self._on_state)
        with self._lock:
            self._client = client
        log.info("TBox SDK ready (map_name=%s, %d named points)",
                 self._map_name, len(self._points))

    def stop_runtime(self) -> None:
        """Release the SDK handle.

        libtbox_sdk_cpp.so has NO deinit API and its static singletons
        segfault on normal Python exit (see htysdk notes), so we only drop
        our reference here; the process must exit via os._exit() — see
        atlas_bridge.main().
        """
        with self._lock:
            self._client = None

    def _on_state(self, cmd: int, payload: bytes, parsed: Any) -> None:
        """SDK state-callback thread -> snapshot under lock."""
        from .tbox_sdk.constants import PacketCommand

        if int(cmd) == int(PacketCommand.PACKET_CMD_TASK_STATE) and parsed is not None:
            with self._lock:
                self._last_task = parsed

    # ── public API (MCP tool handlers) ──────────────────────────────────
    def move(self, *, point_name: str = "", x: float = 0.0,
             y: float = 0.0, yaw: float = 0.0) -> Tuple[bool, str, str]:
        """Dispatch a task_distribution goal. Returns (accepted, run_id, message).

        ``point_name`` takes priority; otherwise the raw x/y/yaw are used
        verbatim. The chassis navigates autonomously on its onboard map.
        """
        with self._lock:
            client = self._client
        if client is None:
            return False, "", "controller not initialized (no TBox authorization)"
        try:
            if point_name:
                if point_name not in self._points:
                    known = ", ".join(sorted(self._points)) or "(none configured)"
                    return (False, "",
                            f"unknown point {point_name!r}; known points: {known}")
                tx, ty, tyaw = self._points[point_name]
                target_desc = f"named point {point_name!r}"
            else:
                tx, ty, tyaw = float(x), float(y), float(yaw)
                target_desc = "coordinate"
            client.task_distribution(
                tx, ty, tyaw,
                map_name=self._map_name,
                speed_ratio=self._speed_ratio,
            )
            run_id = "p2p-" + uuid.uuid4().hex[:8]
            message = (f"move dispatched to {target_desc} "
                       f"({tx:.3f}, {ty:.3f}, {tyaw:.3f}) on map "
                       f"{self._map_name!r}")
            log.info("run %s: %s", run_id, message)
            return True, run_id, message
        except Exception as exc:  # noqa: BLE001
            log.exception("task_distribution failed")
            return False, "", f"task_distribution failed: {exc}"

    def status(self) -> dict:
        """Latest TBox task lifecycle snapshot, mapped to canonical state."""
        with self._lock:
            task = self._last_task
            client = self._client
        if task is None:
            return {"known": False, "state": "PENDING", "task_state": 0,
                    "run_state": 0, "distance_m": -1.0,
                    "detail": "no task status reported yet"}
        ts = int(task.task_state)
        rs = int(task.run_state)
        dist = float(task.distance) if getattr(task, "distance", None) is not None \
            else -1.0
        raw_uuid = getattr(task, "uuid", b"") or b""
        task_uuid = bytes(raw_uuid).decode("utf-8", errors="replace").strip("\x00")
        return {
            "known": True,
            "state": canonical_state(ts),
            "task_state": ts,
            "run_state": rs,
            "distance_m": dist,
            "detail": f"task_state={ts} run_state={rs} "
                      f"uuid={task_uuid or 'n/a'}",
        }

    def cancel(self) -> Tuple[bool, str]:
        """Abort the active task via TBox task_control(CANCEL). Idempotent."""
        with self._lock:
            client = self._client
        if client is None:
            return False, "controller not initialized"
        try:
            from .tbox_sdk.constants import TaskControl

            client.task_control(TaskControl.CANCEL)
            return True, "task cancel sent"
        except Exception as exc:  # noqa: BLE001
            log.exception("task_control(CANCEL) failed")
            return False, f"task_control(CANCEL) failed: {exc}"

    @property
    def known_points(self) -> list:
        """Sorted named-point keys, for diagnostics / error messages."""
        return sorted(self._points)
