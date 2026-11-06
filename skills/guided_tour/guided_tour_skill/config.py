"""Guided-tour（导览）skill configuration.

Defines the tour route (a fixed, ordered list of stops) and the tunables
that drive the stop-by-stop nav2 + speech loop. The config is delivered
via ``Driver(CMD_INIT, config_json)`` from the deploy manifest's
``skill: guided_tour: config:`` block — see config.spec.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class TourStop:
    """One fixed destination the tour visits, plus what to say there."""

    name: str
    x: float
    y: float
    yaw: float          # radians; nav2 goal heading (0 = map +x)
    announcement: str   # spoken via robonix/service/speech/speak on arrival


@dataclass(frozen=True)
class GuidedTourConfig:
    stops: tuple[TourStop, ...]
    frame_id: str = "map"           # nav2 goal frame
    speech_target: str = ""         # speaker provider_id; "" = speech default
    poll_interval_s: float = 2.0    # nav2 status poll interval
    navigation_timeout_s: float = 300.0   # per-stop nav2 timeout
    announce_pause_s: float = 1.0   # settle time after each announcement
    start_announcement: str = ""    # optional, spoken before the first stop
    finish_announcement: str = ""   # optional, spoken after the last stop


def _to_str(value: Any, default: str) -> str:
    return str(value if value is not None else default).strip()


def _to_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _parse_point(value: Any, label: str) -> tuple[float, float, float]:
    """Normalize a stop coordinate to (x, y, yaw).

    Accepts ``{"x":.., "y":.., "yaw":..}`` or ``[x, y, yaw]`` / ``(x, y, yaw)``.
    """
    if isinstance(value, dict):
        try:
            x = float(value["x"])
            y = float(value["y"])
            yaw = _to_float(value.get("yaw"), 0.0)
            return x, y, yaw
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"stop {label!r} point dict must be {{'x':.., 'y':.., 'yaw':..}}, "
                f"got {value!r}"
            ) from exc
    if isinstance(value, (list, tuple)) and len(value) == 3:
        try:
            return float(value[0]), float(value[1]), float(value[2])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"stop {label!r} point coordinates must be numeric: {value!r}") from exc
    raise ValueError(
        f"stop {label!r} point must be {{x, y, yaw}} or [x, y, yaw], got {value!r}"
    )


def _parse_stop(item: Any, index: int) -> TourStop:
    if not isinstance(item, dict):
        raise ValueError(f"stops[{index}] must be a dict, got {type(item).__name__}")

    name = _to_str(item.get("name"), f"站点{index}")
    if "point" in item:
        x, y, yaw = _parse_point(item["point"], name)
    else:
        try:
            x = float(item["x"])
            y = float(item["y"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"stop {name!r} needs either 'point' ({{x,y,yaw}} or [x,y,yaw]) "
                f"or numeric 'x'/'y' (with optional 'yaw')"
            ) from exc
        yaw = _to_float(item.get("yaw"), 0.0)

    announcement = _to_str(item.get("announcement"), "")
    if not announcement:
        raise ValueError(f"stop {name!r} requires a non-empty 'announcement'")

    return TourStop(name=name, x=x, y=y, yaw=yaw, announcement=announcement)


def _deployment_root() -> Path:
    """Directory containing the active robot manifest (mirrors nav2 wrapper).

    Relative file paths in config (e.g. ``stops_file``) are resolved against
    this root, so a deployment can own its tour route file under its own
    ``config/`` directory instead of the skill package.

    ``rbnx boot`` exports ``RBNX_INVOCATION_CWD`` to the manifest dir, but a
    standalone ``rbnx start`` (or an older boot binary) may not forward it to
    the provider process. Fall back to deriving the deploy root from this
    package's location: ``<deploy>/skills/guided_tour/guided_tour_skill/config.py``
    -> ``parents[3]`` is ``<deploy>``.
    """
    raw = os.environ.get("RBNX_INVOCATION_CWD", "").strip()
    if raw:
        return Path(raw).expanduser().resolve()
    return Path(__file__).resolve().parents[3]


def _candidate_roots() -> list[Path]:
    """Ordered candidate bases for resolving a relative config file.

    A relative ``stops_file`` should point into the deploy repo, but the
    provider process does not always receive a correct ``RBNX_INVOCATION_CWD``
    (skills in particular may get the package dir instead of the manifest
    dir). Resolve against each candidate and pick the first that exists, so the
    result does not depend on which env var the spawner happened to set.
    """
    roots: list[Path] = []
    raw = os.environ.get("RBNX_INVOCATION_CWD", "").strip()
    if raw:
        roots.append(Path(raw).expanduser())
    # Deploy root via package layout: <deploy>/skills/guided_tour/guided_tour_skill/config.py
    roots.append(Path(__file__).resolve().parents[3])
    roots.append(Path.cwd().resolve())

    seen: set[Path] = set()
    uniq: list[Path] = []
    for root in roots:
        rr = root.resolve()
        if rr not in seen:
            seen.add(rr)
            uniq.append(rr)
    return uniq


def _load_stops_file(raw_path: object) -> list[Any]:
    """Load a stops list from a JSON file (deploy-relative or absolute).

    Accepts either a top-level JSON array of stop objects, or an object with a
    single ``stops`` array. Each stop keeps the same shape as the inline
    ``stops`` config: ``{name, point|x,y, announcement}``.
    """
    path = str(raw_path or "").strip()
    if not path:
        raise ValueError("stops_file must be a non-empty path")
    p = Path(path).expanduser()

    candidates = [p] if p.is_absolute() else [root / p for root in _candidate_roots()]

    resolved: Path | None = None
    for candidate in candidates:
        rc = candidate.resolve()
        if rc.is_file():
            resolved = rc
            break
    if resolved is None:
        tried = ", ".join(str(c.resolve()) for c in candidates)
        raise FileNotFoundError(f"stops_file not found: {path} (tried: {tried})")

    try:
        data = json.loads(resolved.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"stops_file {resolved} is not valid JSON: {exc}") from exc
    if isinstance(data, dict):
        data = data.get("stops")
    if not isinstance(data, list) or not data:
        raise ValueError(f"stops_file {resolved} must contain a non-empty list of stops")
    return data


def parse_guided_tour_config(cfg: dict | None) -> GuidedTourConfig:
    cfg = cfg or {}

    if cfg.get("stops_file"):
        raw_stops = _load_stops_file(cfg["stops_file"])
    else:
        raw_stops = cfg.get("stops")
        if not isinstance(raw_stops, list) or not raw_stops:
            raise ValueError(
                "stops must be a non-empty list of {name, point|x,y, announcement}, "
                "or set stops_file to a JSON file"
            )

    stops = tuple(_parse_stop(item, i + 1) for i, item in enumerate(raw_stops))
    return GuidedTourConfig(
        stops=stops,
        frame_id=_to_str(cfg.get("frame_id"), "map"),
        speech_target=_to_str(cfg.get("speech_target"), ""),
        poll_interval_s=_to_float(cfg.get("poll_interval_s"), 2.0),
        navigation_timeout_s=_to_float(cfg.get("navigation_timeout_s"), 300.0),
        announce_pause_s=_to_float(cfg.get("announce_pause_s"), 1.0),
        start_announcement=_to_str(cfg.get("start_announcement"), ""),
        finish_announcement=_to_str(cfg.get("finish_announcement"), ""),
    )
