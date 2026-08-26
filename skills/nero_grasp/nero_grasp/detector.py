"""VLM-based object detector for nero_grasp."""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Protocol

from .depth_sample import sample_xyz_at_pixel
from .rgbd_frame import CameraIntrinsics, RGBDFrame, parse_rgbd
from .vlm_client import (
    DEFAULT_USER_INSTRUCTION,
    VlmConfig,
    VlmCupPoint,
    query_cup_center,
    resolve_vlm_config,
)
from .vlm_debug import save_vlm_debug

log = logging.getLogger("nero_grasp.detector")


def resolve_user_instruction(object_name: str | None, default_instruction: str) -> str:
    """Build VLM user instruction from pilot-provided object name."""
    name = (object_name or "").strip()
    if not name:
        return default_instruction
    return f"抓取桌上的{name}"


@dataclass(frozen=True)
class CameraTarget:
    """Grasp point expressed in the depth-camera frame (metres)."""

    xyz_m: list[float]
    label: str = "object"
    confidence: float = 1.0


@dataclass(frozen=True)
class VlmDetectorConfig:
    instruction: str = DEFAULT_USER_INSTRUCTION
    depth_min_m: float = 0.15
    depth_max_m: float = 1.8
    depth_scale: float = 0.001
    sample_window: int = 15
    sample_strategy: str = "median"
    debug_dir: str | None = None
    debug_prefix: str = "latest"
    vlm_base_url: str | None = None
    vlm_api_key: str | None = None
    vlm_model: str | None = None
    vlm_timeout_s: float | None = None
    vlm_temperature: float | None = None
    vlm_use_json_schema: bool | None = None


class ObjectDetector(Protocol):
    def detect(self, rgbd: Any, *, user_instruction: str | None = None) -> CameraTarget | None:
        """Run detection on a synchronized RGB-D frame."""


class StubObjectDetector:
    def detect(self, rgbd: Any, *, user_instruction: str | None = None) -> CameraTarget | None:
        log.info("object detector stub: no algorithm configured")
        return None


def _vlm_point_to_dict(point: VlmCupPoint) -> dict:
    return {
        "found": point.found,
        "u": point.u,
        "v": point.v,
        "confidence": point.confidence,
        "note": point.note,
        "raw_text": point.raw_text,
        "class_name": point.class_name,
        "thinking_process": point.thinking_process,
        "box_center_x": point.box_center_x,
        "box_center_y": point.box_center_y,
        "box_width": point.box_width,
        "box_height": point.box_height,
    }


class VlmObjectDetector:
    """Localize grasp target via VLM + depth back-projection."""

    def __init__(self, cfg: VlmDetectorConfig | None = None) -> None:
        self.cfg = cfg or VlmDetectorConfig()

    def _resolve_vlm_config(self) -> VlmConfig:
        overrides: dict[str, Any] = {}
        if self.cfg.vlm_base_url:
            overrides["vlm_base_url"] = self.cfg.vlm_base_url
        if self.cfg.vlm_api_key:
            overrides["vlm_api_key"] = self.cfg.vlm_api_key
        if self.cfg.vlm_model:
            overrides["vlm_model"] = self.cfg.vlm_model
        if self.cfg.vlm_timeout_s is not None:
            overrides["vlm_timeout_s"] = self.cfg.vlm_timeout_s
        if self.cfg.vlm_temperature is not None:
            overrides["vlm_temperature"] = self.cfg.vlm_temperature
        if self.cfg.vlm_use_json_schema is not None:
            overrides["vlm_use_json_schema"] = self.cfg.vlm_use_json_schema
        return resolve_vlm_config(overrides=overrides)

    def _save_debug(self, bgr, point: VlmCupPoint, camera_xyz: list[float] | None) -> None:
        if not self.cfg.debug_dir:
            return
        save_vlm_debug(
            rgb=bgr,
            output_dir=self.cfg.debug_dir,
            prefix=self.cfg.debug_prefix,
            vlm_point=_vlm_point_to_dict(point),
            camera_xyz=camera_xyz,
            robot_xyz=None,
            sample_window=self.cfg.sample_window,
        )

    def detect_frame(
        self,
        frame: RGBDFrame,
        *,
        user_instruction: str | None = None,
    ) -> CameraTarget | None:
        if frame.rgb is None:
            log.warning("vlm detect: RGB image missing")
            return None

        bgr = frame.rgb
        depth_m = frame.depth_m
        h, w = depth_m.shape
        if bgr.shape[:2] != (h, w):
            log.warning(
                "vlm detect: RGB/depth size mismatch %s vs %s",
                bgr.shape[:2],
                (h, w),
            )
            return None

        intrinsics = frame.intrinsics or CameraIntrinsics.for_resolution(w, h)
        instruction = user_instruction or self.cfg.instruction
        log.info("vlm detect instruction: %s", instruction)
        point = query_cup_center(
            bgr,
            self._resolve_vlm_config(),
            user_instruction=instruction,
        )

        if not point.found:
            log.info("vlm detect: no target (%s)", point.note)
            self._save_debug(bgr, point, None)
            return None

        try:
            xyz, _pixel, valid_count = sample_xyz_at_pixel(
                depth_m,
                intrinsics,
                point.u,
                point.v,
                window=self.cfg.sample_window,
                depth_min_m=self.cfg.depth_min_m,
                depth_max_m=self.cfg.depth_max_m,
                strategy=self.cfg.sample_strategy,
            )
        except RuntimeError as exc:
            log.warning("vlm detect: depth sample failed at (%d,%d): %s", point.u, point.v, exc)
            self._save_debug(bgr, point, None)
            return None

        label = point.class_name or "object"
        target = CameraTarget(xyz_m=xyz, label=label, confidence=point.confidence)
        log.info(
            "vlm detect: pixel=(%d,%d) xyz=%s label=%s valid_px=%d",
            point.u,
            point.v,
            [round(v, 4) for v in xyz],
            label,
            valid_count,
        )
        self._save_debug(bgr, point, xyz)
        return target

    def detect(self, rgbd: Any, *, user_instruction: str | None = None) -> CameraTarget | None:
        try:
            frame = parse_rgbd(rgbd, depth_scale=self.cfg.depth_scale)
            return self.detect_frame(frame, user_instruction=user_instruction)
        except Exception as exc:  # noqa: BLE001
            log.warning("vlm detect: failed to parse rgbd: %s", exc)
            return None


def _first_vlm_str(cfg_dict: dict, *keys: str) -> str | None:
    for key in keys:
        value = cfg_dict.get(key)
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return None


def vlm_detector_config_from_dict(cfg_dict: dict | None) -> VlmDetectorConfig:
    cfg_dict = cfg_dict or {}
    debug_dir = cfg_dict.get("debug_dir")
    use_json_schema = cfg_dict.get("vlm_use_json_schema", cfg_dict.get("use_json_schema"))
    if use_json_schema is not None:
        use_json_schema = str(use_json_schema).strip().lower() not in {"0", "false", "no", "off"}

    return VlmDetectorConfig(
        instruction=str(cfg_dict.get("instruction", DEFAULT_USER_INSTRUCTION)).strip()
        or DEFAULT_USER_INSTRUCTION,
        depth_min_m=float(cfg_dict.get("depth_min_m", VlmDetectorConfig.depth_min_m)),
        depth_max_m=float(cfg_dict.get("depth_max_m", VlmDetectorConfig.depth_max_m)),
        depth_scale=float(cfg_dict.get("depth_scale", VlmDetectorConfig.depth_scale)),
        sample_window=int(cfg_dict.get("sample_window", VlmDetectorConfig.sample_window)),
        sample_strategy=str(cfg_dict.get("sample_strategy", VlmDetectorConfig.sample_strategy)),
        debug_dir=str(debug_dir).strip() if debug_dir else None,
        debug_prefix=str(cfg_dict.get("debug_prefix", VlmDetectorConfig.debug_prefix)),
        vlm_base_url=_first_vlm_str(cfg_dict, "vlm_base_url", "base_url", "upstream"),
        vlm_api_key=_first_vlm_str(cfg_dict, "vlm_api_key", "api_key"),
        vlm_model=_first_vlm_str(cfg_dict, "vlm_model", "model"),
        vlm_timeout_s=float(cfg_dict["vlm_timeout_s"])
        if cfg_dict.get("vlm_timeout_s") is not None
        else (float(cfg_dict["timeout_s"]) if cfg_dict.get("timeout_s") is not None else None),
        vlm_temperature=float(cfg_dict["vlm_temperature"])
        if cfg_dict.get("vlm_temperature") is not None
        else (float(cfg_dict["temperature"]) if cfg_dict.get("temperature") is not None else None),
        vlm_use_json_schema=use_json_schema if use_json_schema is not None else None,
    )


def create_detector(cfg_dict: dict | None = None) -> ObjectDetector:
    """Factory: ``detector: vlm`` (default) or ``detector: stub``."""
    cfg_dict = cfg_dict or {}
    kind = str(cfg_dict.get("detector", "vlm")).strip().lower()
    if kind in {"stub", "none", "off"}:
        return StubObjectDetector()
    if kind in {"cup", "segment", "segmentation"}:
        log.warning("detector=%r removed; using vlm instead (see tools.cup_detect for legacy segmentation)", kind)
    return VlmObjectDetector(vlm_detector_config_from_dict(cfg_dict))
