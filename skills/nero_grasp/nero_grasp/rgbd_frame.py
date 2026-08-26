"""Decode ROS RGB-D messages / numpy frames for perception."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

_DEPTH_MM_ENCODINGS = frozenset({"16UC1", "mono16", "16UC1; jpeg compressed"})
_DEPTH_M_ENCODINGS = frozenset({"32FC1"})


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def for_resolution(cls, width: int, height: int) -> CameraIntrinsics:
        """Approximate RealSense D435 intrinsics scaled to image size."""
        fx = 615.0 * width / 640.0
        fy = 615.0 * height / 480.0
        return cls(
            fx=fx,
            fy=fy,
            cx=width / 2.0,
            cy=height / 2.0,
            width=width,
            height=height,
        )


@dataclass(frozen=True)
class RGBDFrame:
    depth_m: np.ndarray
    rgb: np.ndarray | None = None
    intrinsics: CameraIntrinsics | None = None
    depth_scale: float = 0.001


def _ros_image_to_array(msg: Any) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    encoding = str(getattr(msg, "encoding", "") or "")
    data = bytes(getattr(msg, "data", b""))
    if encoding in _DEPTH_MM_ENCODINGS:
        arr = np.frombuffer(data, dtype=np.uint16)
        if arr.size != height * width:
            raise ValueError(f"depth image size mismatch: {arr.size} vs {height}x{width}")
        return arr.reshape(height, width)
    if encoding in _DEPTH_M_ENCODINGS:
        arr = np.frombuffer(data, dtype=np.float32)
        if arr.size != height * width:
            raise ValueError(f"depth image size mismatch: {arr.size} vs {height}x{width}")
        return arr.reshape(height, width)
    if encoding in {"rgb8", "bgr8"}:
        arr = np.frombuffer(data, dtype=np.uint8)
        if arr.size != height * width * 3:
            raise ValueError(f"rgb image size mismatch: {arr.size} vs {height}x{width}x3")
        return arr.reshape(height, width, 3)
    raise ValueError(f"unsupported image encoding: {encoding!r}")


def _depth_array_to_meters(depth_raw: np.ndarray, *, encoding: str, depth_scale: float) -> np.ndarray:
    depth = depth_raw.astype(np.float32, copy=False)
    if encoding in _DEPTH_MM_ENCODINGS:
        depth = depth * depth_scale
    return depth


def intrinsics_from_camera_info(msg: Any) -> CameraIntrinsics:
    """Build pinhole intrinsics from ``sensor_msgs/CameraInfo``."""
    k = [float(v) for v in msg.k]
    if len(k) < 9:
        raise ValueError("CameraInfo.k must have 9 elements")
    width = int(msg.width)
    height = int(msg.height)
    return CameraIntrinsics(
        fx=k[0],
        fy=k[4],
        cx=k[2],
        cy=k[5],
        width=width,
        height=height,
    )


def image_msg_to_rgbd_frame(
    rgb_msg: Any,
    depth_msg: Any,
    *,
    intrinsics: CameraIntrinsics | None = None,
    depth_scale: float = 0.001,
) -> RGBDFrame:
    """Convert paired ROS Image messages into ``RGBDFrame``."""
    depth_encoding = str(depth_msg.encoding)
    depth_raw = _ros_image_to_array(depth_msg)
    depth_m = _depth_array_to_meters(depth_raw, encoding=depth_encoding, depth_scale=depth_scale)

    rgb_arr: np.ndarray | None = None
    if rgb_msg is not None and int(rgb_msg.width) > 0:
        rgb_arr = _ros_image_to_array(rgb_msg)

    h, w = depth_m.shape
    if intrinsics is None:
        intrinsics = CameraIntrinsics.for_resolution(w, h)
    return RGBDFrame(
        depth_m=depth_m,
        rgb=rgb_arr,
        intrinsics=intrinsics,
        depth_scale=depth_scale,
    )


def parse_rgbd(rgbd: Any, *, depth_scale: float = 0.001) -> RGBDFrame:
    """Normalize ROS ``camera.msg.RGBD`` or a plain dict/frame into ``RGBDFrame``."""
    if isinstance(rgbd, RGBDFrame):
        return rgbd

    if isinstance(rgbd, dict):
        depth_raw = np.asarray(rgbd["depth_m"], dtype=np.float32)
        rgb = rgbd.get("rgb")
        intr = rgbd.get("intrinsics")
        if intr is not None and not isinstance(intr, CameraIntrinsics):
            intr = CameraIntrinsics(**intr)
        return RGBDFrame(depth_m=depth_raw, rgb=rgb, intrinsics=intr, depth_scale=depth_scale)

    depth_msg = getattr(rgbd, "depth", None)
    if depth_msg is None:
        raise ValueError("rgbd message has no depth field")

    depth_encoding = str(depth_msg.encoding)
    depth_raw = _ros_image_to_array(depth_msg)
    depth_m = _depth_array_to_meters(depth_raw, encoding=depth_encoding, depth_scale=depth_scale)

    rgb_arr: np.ndarray | None = None
    rgb_msg = getattr(rgbd, "rgb", None)
    if rgb_msg is not None and int(rgb_msg.width) > 0:
        try:
            rgb_arr = _ros_image_to_array(rgb_msg)
        except ValueError:
            rgb_arr = None

    h, w = depth_m.shape
    intrinsics = CameraIntrinsics.for_resolution(w, h)
    return RGBDFrame(depth_m=depth_m, rgb=rgb_arr, intrinsics=intrinsics, depth_scale=depth_scale)


def deproject_depth(
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    *,
    valid_mask: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (points Nx3, us, vs) for valid depth pixels."""
    h, w = depth_m.shape
    if valid_mask is None:
        valid_mask = np.isfinite(depth_m) & (depth_m > 0)

    vs, us = np.nonzero(valid_mask)
    z = depth_m[vs, us].astype(np.float64)
    x = (us.astype(np.float64) - intrinsics.cx) * z / intrinsics.fx
    y = (vs.astype(np.float64) - intrinsics.cy) * z / intrinsics.fy
    points = np.stack([x, y, z], axis=1)
    return points, us.astype(np.int32), vs.astype(np.int32)
