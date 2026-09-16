# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for guided_tour config parsing + controller loop (no hardware)."""
from __future__ import annotations

import json
import threading

from guided_tour_skill.config import parse_guided_tour_config, GuidedTourConfig, TourStop
from guided_tour_skill.controller import TourController


def _cfg(**overrides):
    stops = [
        {"name": "a", "point": [0.0, 0.0, 0.0], "announcement": "到了a"},
        {"name": "b", "point": [1.0, 2.0, 1.57], "announcement": "到了b"},
    ]
    base = {"stops": stops, "announce_pause_s": 0.0}
    base.update(overrides)
    return parse_guided_tour_config(base)


def _make_controller(cfg, *, nav_results=None, speak_errors=None):
    """Build a controller with fake nav2 + speech callables."""
    nav_results = nav_results or ["SUCCEEDED"]
    spoken: list[str] = []
    nav_calls: list[tuple] = []
    status_calls: list[str] = []

    def navigate(x, y, yaw, frame_id):
        nav_calls.append((x, y, yaw, frame_id))
        return {"accepted": True, "run_id": f"run-{len(nav_calls)}", "detail": "ok"}

    def status(run_id):
        status_calls.append(run_id)
        # Pop a result per navigation; last result repeats.
        state = nav_results[min(len(status_calls) - 1, len(nav_results) - 1)]
        return {"state": state, "detail": ""}

    def speak(text):
        if speak_errors:
            raise RuntimeError(speak_errors.pop(0))
        spoken.append(text)

    ctrl = TourController(
        cfg.stops, cfg, navigate, status, speak,
        cancel_nav_fn=lambda run_id: {"accepted": True, "detail": ""},
    )
    return ctrl, spoken, nav_calls, status_calls


def test_config_parses_stops():
    cfg = _cfg()
    assert len(cfg.stops) == 2
    assert cfg.stops[0].name == "a"
    assert cfg.stops[1].yaw == 1.57
    assert cfg.frame_id == "map"


def test_config_rejects_empty_announcement():
    try:
        parse_guided_tour_config({"stops": [{"name": "a", "x": 0, "y": 0, "announcement": ""}]})
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_config_accepts_x_y_yaw_form():
    cfg = parse_guided_tour_config(
        {"stops": [{"name": "a", "x": 3, "y": 4, "yaw": 0.5, "announcement": "hi"}]}
    )
    assert cfg.stops[0].x == 3.0
    assert cfg.stops[0].yaw == 0.5


def test_config_loads_stops_file_absolute(tmp_path):
    stops_file = tmp_path / "stops.json"
    stops_file.write_text(json.dumps([
        {"name": "a", "point": [1.0, 2.0, 0.5], "announcement": "到了a"},
        {"name": "b", "point": [3.0, 4.0, 1.0], "announcement": "到了b"},
    ]), encoding="utf-8")
    cfg = parse_guided_tour_config({"stops_file": str(stops_file)})
    assert [s.name for s in cfg.stops] == ["a", "b"]
    assert cfg.stops[0].x == 1.0
    assert cfg.stops[1].yaw == 1.0


def test_config_loads_stops_file_object_form_deploy_relative(tmp_path, monkeypatch):
    monkeypatch.setenv("RBNX_INVOCATION_CWD", str(tmp_path))
    (tmp_path / "route.json").write_text(json.dumps({
        "stops": [
            {"name": "前台", "point": [-37.07, 12.59, 1.91], "announcement": "欢迎来到前台"},
        ]
    }), encoding="utf-8")
    cfg = parse_guided_tour_config({"stops_file": "route.json"})
    assert cfg.stops[0].name == "前台"
    assert cfg.stops[0].yaw == 1.91


def test_config_rejects_missing_stops_file(tmp_path):
    try:
        parse_guided_tour_config({"stops_file": str(tmp_path / "nope.json")})
        assert False, "expected FileNotFoundError"
    except FileNotFoundError:
        pass


def test_run_visits_all_stops_and_speaks():
    cfg = _cfg()
    ctrl, spoken, nav_calls, _ = _make_controller(cfg)
    result = ctrl.run()
    assert result["state"] == "SUCCEEDED"
    assert len(nav_calls) == 2
    assert spoken == ["到了a", "到了b"]


def test_run_fails_on_nav_failure():
    cfg = _cfg()
    ctrl, spoken, _, _ = _make_controller(cfg, nav_results=["FAILED"])
    result = ctrl.run()
    assert result["state"] == "FAILED"
    assert spoken == []  # never announces a stop it did not reach


def test_cancel_stops_between_stops():
    cfg = _cfg(announce_pause_s=0.05, poll_interval_s=0.05)
    ctrl, spoken, _, _ = _make_controller(cfg)

    def _cancel_later():
        # Cancel once the controller has moved past the first stop.
        while ctrl.snapshot()["stop_index"] < 2:
            if ctrl.snapshot()["state"] in ("SUCCEEDED", "FAILED", "CANCELED"):
                return
        ctrl.request_cancel()

    t = threading.Thread(target=_cancel_later, daemon=True)
    t.start()
    result = ctrl.run()
    t.join(timeout=2)
    assert result["state"] == "CANCELED"
