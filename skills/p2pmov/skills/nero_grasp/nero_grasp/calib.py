"""Load camera→robot-base calibration produced by tools/camera_arm_calib."""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path

import numpy as np

log = logging.getLogger("nero_grasp.calib")

_DEPLOY_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CALIB_PATH = _DEPLOY_ROOT / "calib" / "camera_to_robot.json"


@dataclass(frozen=True)
class CameraToRobotCalib:
    """Rigid transform: P_robot = R @ P_camera + t."""

    rotation: np.ndarray
    translation: np.ndarray
    rms_error_m: float = 0.0
    max_error_m: float = 0.0
    source_frame: str = "camera"
    target_frame: str = "robot_base"
    path: Path | None = None

    def camera_to_robot(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        p = self.rotation @ np.array([x, y, z], dtype=float) + self.translation
        return float(p[0]), float(p[1]), float(p[2])

    def camera_to_robot_xyz(self, xyz: list[float]) -> list[float]:
        x, y, z = self.camera_to_robot(float(xyz[0]), float(xyz[1]), float(xyz[2]))
        return [x, y, z]


def resolve_calib_path(path: str | Path) -> Path:
    p = Path(path)
    if p.is_absolute():
        return p
    return (_DEPLOY_ROOT / p).resolve()


def load_camera_to_robot(path: str | Path | None = None) -> CameraToRobotCalib:
    calib_path = resolve_calib_path(path or DEFAULT_CALIB_PATH)
    if not calib_path.is_file():
        raise FileNotFoundError(f"camera calibration not found: {calib_path}")

    data = json.loads(calib_path.read_text(encoding="utf-8"))
    rotation = np.asarray(data["rotation"], dtype=float)
    translation = np.asarray(data["translation"], dtype=float)
    if rotation.shape != (3, 3) or translation.shape != (3,):
        raise ValueError(f"invalid calibration matrices in {calib_path}")

    frames = data.get("frames", {})
    calib = CameraToRobotCalib(
        rotation=rotation,
        translation=translation,
        rms_error_m=float(data.get("rms_error_m", 0.0)),
        max_error_m=float(data.get("max_error_m", 0.0)),
        source_frame=str(frames.get("source", "camera")),
        target_frame=str(frames.get("target", "robot_base")),
        path=calib_path,
    )
    log.info(
        "loaded calibration %s  rms=%.1f mm  max=%.1f mm",
        calib_path,
        calib.rms_error_m * 1000.0,
        calib.max_error_m * 1000.0,
    )
    return calib
