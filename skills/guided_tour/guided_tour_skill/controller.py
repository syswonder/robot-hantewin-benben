"""Guided-tour（导览）controller — the stop-by-stop navigation + speech loop.

The controller is transport-agnostic: it receives injectable callables
(navigate / status / cancel_nav / speak) so the loop can be unit-tested
without hardware, while atlas_bridge wires the real MCP clients.

Tour flow, per configured stop:
  1. navigate there via nav2 (robonix/service/navigation/navigate), then
  2. poll navigate/status until SUCCEEDED / FAILED / CANCELED / TIMEOUT,
  3. speak the stop's announcement via speech/speak, then settle.

State exposed by ``snapshot()`` uses the executor's canonical async state
names (PENDING / RUNNING / SUCCEEDED / FAILED / CANCELED) plus a human
``phase`` (idle / navigating / announcing / finished) and the 1-based
``stop_index``.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Optional

from .config import GuidedTourConfig, TourStop

log = logging.getLogger("guided_tour.controller")

NavigateFn = Callable[[float, float, float, str], dict]   # (x, y, yaw, frame_id) -> {accepted, run_id, detail}
StatusFn = Callable[[str], dict]                          # (run_id) -> {state, detail}
CancelNavFn = Callable[[str], dict]                       # (run_id) -> {accepted, detail}
SpeakFn = Callable[[str], None]                           # (text) -> None, raises on failure


class TourController:
    """Runs one tour over ``stops``; thread-safe for status/cancel readers."""

    def __init__(
        self,
        stops: tuple[TourStop, ...],
        cfg: GuidedTourConfig,
        navigate_fn: NavigateFn,
        status_fn: StatusFn,
        speak_fn: SpeakFn,
        cancel_nav_fn: Optional[CancelNavFn] = None,
    ) -> None:
        self.stops = stops
        self.cfg = cfg
        self._navigate = navigate_fn
        self._status = status_fn
        self._speak = speak_fn
        self._cancel_nav = cancel_nav_fn

        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._current_nav_run_id = ""
        self._state: dict[str, Any] = {
            "state": "PENDING",
            "phase": "idle",
            "stop_index": 0,
            "total_stops": len(stops),
            "detail": "",
        }

    # ── state accessors (thread-safe) ──────────────────────────────────────
    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._state)

    def request_cancel(self) -> None:
        self._cancel.set()

    def _set_state(self, **updates: Any) -> None:
        with self._lock:
            self._state.update(updates)

    def _finish(self, state: str, phase: str, detail: str) -> dict:
        self._set_state(state=state, phase=phase, detail=detail)
        return self.snapshot()

    # ── small helpers ──────────────────────────────────────────────────────
    def _sleep(self, seconds: float) -> bool:
        """Sleep in short slices; return True if a cancel arrived mid-sleep."""
        if seconds <= 0:
            return False
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._cancel.is_set():
                return True
            time.sleep(min(0.1, deadline - time.monotonic()))
        return False

    def _cancel_active_navigation(self, run_id: str) -> None:
        if not run_id or self._cancel_nav is None:
            return
        try:
            self._cancel_nav(run_id)
        except Exception as exc:  # noqa: BLE001
            log.warning("cancel nav %s failed: %s", run_id, exc)

    def _speak_announcement(self, text: str, stop_index: int) -> None:
        if not text:
            return
        total = len(self.stops)
        self._set_state(
            state="RUNNING", phase="announcing", stop_index=stop_index,
            detail=f"[{stop_index}/{total}] 播报: {text}",
        )
        self._speak(text)

    def _poll_navigation(self, run_id: str, stop: TourStop) -> str:
        """Poll nav2 until a terminal state or timeout. Returns that state."""
        deadline = time.monotonic() + self.cfg.navigation_timeout_s
        while time.monotonic() < deadline:
            if self._cancel.is_set():
                self._cancel_active_navigation(run_id)
                return "CANCELED"
            res = self._status(run_id)
            state = (res or {}).get("state", "")
            if state == "SUCCEEDED":
                return "SUCCEEDED"
            if state in ("FAILED", "CANCELED"):
                return state
            # PENDING / RUNNING / unknown -> keep polling
            if self._sleep(self.cfg.poll_interval_s):
                self._cancel_active_navigation(run_id)
                return "CANCELED"
        self._cancel_active_navigation(run_id)
        return "TIMEOUT"

    # ── main loop ──────────────────────────────────────────────────────────
    def run(self) -> dict:
        try:
            return self._run()
        except Exception as exc:  # noqa: BLE001
            log.exception("tour loop crashed")
            self._cancel_active_navigation(self._current_nav_run_id)
            return self._finish("FAILED", "error", str(exc))

    def _run(self) -> dict:
        cfg = self.cfg
        total = len(self.stops)
        self._set_state(state="RUNNING", phase="starting", detail="导览开始")

        if cfg.start_announcement:
            self._speak_announcement(cfg.start_announcement, 0)
            if self._sleep(cfg.announce_pause_s):
                return self._finish("CANCELED", "canceled", "已取消（出发前）")

        for index, stop in enumerate(self.stops, start=1):
            if self._cancel.is_set():
                return self._finish("CANCELED", "canceled", f"已取消（第 {index}/{total} 站前）")

            # 1) navigate to the stop.
            self._set_state(
                state="RUNNING", phase="navigating", stop_index=index,
                detail=f"[{index}/{total}] 前往 {stop.name} ({stop.x:.2f}, {stop.y:.2f}, {stop.yaw:.3f})",
            )
            nav = self._navigate(stop.x, stop.y, stop.yaw, cfg.frame_id)
            if not nav.get("accepted"):
                return self._finish(
                    "FAILED", "navigating",
                    f"第 {index} 站 {stop.name} 导航下发失败: {nav.get('detail', '')}",
                )
            run_id = str(nav.get("run_id", ""))
            with self._lock:
                self._current_nav_run_id = run_id

            final = self._poll_navigation(run_id, stop)
            if final != "SUCCEEDED":
                state = "CANCELED" if final == "CANCELED" else "FAILED"
                return self._finish(
                    state, "navigating",
                    f"第 {index} 站 {stop.name} 导航{self._nav_desc(final)}",
                )

            # 2) speak the stop's announcement.
            self._speak_announcement(stop.announcement, index)
            if self._sleep(cfg.announce_pause_s):
                return self._finish("CANCELED", "canceled", f"已取消（第 {index} 站播报后）")

        if cfg.finish_announcement:
            self._speak_announcement(cfg.finish_announcement, total)
        return self._finish("SUCCEEDED", "finished", f"导览完成，共 {total} 站")

    @staticmethod
    def _nav_desc(state: str) -> str:
        return {"FAILED": "失败", "CANCELED": "被取消", "TIMEOUT": "超时"}.get(state, state)
