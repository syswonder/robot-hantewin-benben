"""Save VLM detection debug images for nero_grasp."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def save_vlm_debug(
    *,
    rgb: np.ndarray,
    output_dir: str | Path,
    prefix: str,
    vlm_point: dict,
    camera_xyz: list[float] | None,
    robot_xyz: list[float] | None,
    sample_window: int,
) -> Path:
    import cv2

    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=True)

    cv2.imwrite(str(out / f"{prefix}_input.jpg"), rgb)

    overlay = rgb.copy()
    h, w = rgb.shape[:2]
    if vlm_point.get("found"):
        u, v = int(vlm_point["u"]), int(vlm_point["v"])
        half = max(1, sample_window // 2)
        cv2.drawMarker(overlay, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.rectangle(overlay, (u - half, v - half), (u + half, v + half), (0, 255, 255), 1)

        bcx = float(vlm_point.get("box_center_x", 0.0))
        bcy = float(vlm_point.get("box_center_y", 0.0))
        bw = float(vlm_point.get("box_width", 0.0))
        bh = float(vlm_point.get("box_height", 0.0))
        if bw > 0 and bh > 0 and bcx > 0 and bcy > 0:
            cx = bcx * (w - 1)
            cy = bcy * (h - 1)
            bw_px = bw * (w - 1)
            bh_px = bh * (h - 1)
            x1 = int(round(cx - bw_px / 2))
            y1 = int(round(cy - bh_px / 2))
            x2 = int(round(cx + bw_px / 2))
            y2 = int(round(cy + bh_px / 2))
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)

        class_name = str(vlm_point.get("class_name", "") or "target")
        label = f"{class_name} ({u},{v}) conf={vlm_point.get('confidence', 0):.2f}"
        cv2.putText(
            overlay,
            label,
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            label,
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
        if camera_xyz is not None:
            xyz_text = (
                f"camera xyz (m): "
                f"{camera_xyz[0]:.3f}, {camera_xyz[1]:.3f}, {camera_xyz[2]:.3f}"
            )
            cv2.putText(
                overlay,
                xyz_text,
                (8, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                overlay,
                xyz_text,
                (8, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )
        if robot_xyz is not None:
            r_text = f"robot xyz (m): {robot_xyz[0]:.3f}, {robot_xyz[1]:.3f}, {robot_xyz[2]:.3f}"
            cv2.putText(
                overlay,
                r_text,
                (8, 76),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                overlay,
                r_text,
                (8, 76),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )

    overlay_path = out / f"{prefix}_overlay.jpg"
    cv2.imwrite(str(overlay_path), overlay)

    result = {
        "vlm": vlm_point,
        "camera_xyz_m": camera_xyz,
        "robot_xyz_m": robot_xyz,
        "sample_window": sample_window,
    }
    json_path = out / f"{prefix}_result.json"
    json_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return overlay_path
