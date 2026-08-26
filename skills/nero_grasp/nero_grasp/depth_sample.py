"""Sample metric depth and deproject to camera-frame XYZ."""
from __future__ import annotations

import numpy as np

from .rgbd_frame import CameraIntrinsics


def _clip_roi(
    u: int,
    v: int,
    *,
    half: int,
    width: int,
    height: int,
) -> tuple[slice, slice]:
    u0 = max(0, u - half)
    u1 = min(width, u + half + 1)
    v0 = max(0, v - half)
    v1 = min(height, v + half + 1)
    return slice(v0, v1), slice(u0, u1)


def sample_xyz_at_pixel(
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    u: int,
    v: int,
    *,
    window: int = 15,
    depth_min_m: float = 0.15,
    depth_max_m: float = 2.0,
    strategy: str = "median",
) -> tuple[list[float], tuple[int, int], int]:
    """Return camera-frame xyz (m) around pixel (u, v).

    strategy: ``median`` (robust) or ``min`` (closest point in window).
    """
    h, w = depth_m.shape
    u = int(np.clip(u, 0, w - 1))
    v = int(np.clip(v, 0, h - 1))
    half = max(1, window // 2)
    vs, us = _clip_roi(u, v, half=half, width=w, height=h)

    roi = depth_m[vs, us]
    valid = roi[np.isfinite(roi) & (roi >= depth_min_m) & (roi <= depth_max_m)]
    if valid.size == 0:
        raise RuntimeError(
            f"no valid depth around pixel ({u}, {v}); "
            f"range=[{depth_min_m}, {depth_max_m}] m"
        )

    if strategy == "min":
        depth = float(np.min(valid))
    else:
        depth = float(np.median(valid))

    x = (float(u) - intrinsics.cx) * depth / intrinsics.fx
    y = (float(v) - intrinsics.cy) * depth / intrinsics.fy
    return [float(x), float(y), float(depth)], (u, v), int(valid.size)
