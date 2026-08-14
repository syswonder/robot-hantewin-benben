"""Camera primitive + perception client for nero_grasp.

Subscribes to RealSense camera primitive topics via atlas:

  - ``robonix/primitive/camera/rgb``        -> sensor_msgs/Image (BGR8)
  - ``robonix/primitive/camera/depth``      -> sensor_msgs/Image (16UC1, aligned)
  - ``robonix/primitive/camera/intrinsics`` -> sensor_msgs/CameraInfo (optional)
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from robonix_api import ATLAS
from robonix_api.atlas_types import Channel
from robonix_api.ros import RosBackend
from sensor_msgs.msg import CameraInfo, Image  # type: ignore

from .calib import CameraToRobotCalib
from .detector import CameraTarget, create_detector, resolve_user_instruction
from .vlm_client import DEFAULT_USER_INSTRUCTION
from .rgbd_frame import CameraIntrinsics, RGBDFrame, image_msg_to_rgbd_frame, intrinsics_from_camera_info
from .target import GraspTarget

if TYPE_CHECKING:
    from .detector import ObjectDetector

log = logging.getLogger("nero_grasp.camera_client")

DEFAULT_DEPTH_SCALE = 0.001
SYNC_TOLERANCE_NS = 50_000_000  # 50 ms


@dataclass(frozen=True)
class CameraTopics:
    rgb: str
    depth: str
    intrinsics: str | None


class CameraClient:
    """Atlas-resolved camera/perception client."""

    def __init__(
        self,
        consumer_id: str,
        *,
        provider_id: str | None = None,
        calib: CameraToRobotCalib | None = None,
        detector: ObjectDetector | None = None,
        detector_provider_id: str | None = None,
        detector_settings: dict | None = None,
        frame_timeout_s: float = 5.0,
        depth_scale: float = DEFAULT_DEPTH_SCALE,
    ) -> None:
        self.consumer_id = consumer_id
        self.provider_id = provider_id
        self.detector_provider_id = detector_provider_id
        self._calib = calib
        settings = dict(detector_settings or {})
        self._depth_scale = float(settings.get("depth_scale", depth_scale))
        self._frame_timeout_s = float(settings.get("frame_timeout_s", frame_timeout_s))
        self._detector = detector or create_detector(detector_settings)
        default_instruction = str(settings.get("instruction", DEFAULT_USER_INSTRUCTION)).strip()
        self._default_instruction = default_instruction or DEFAULT_USER_INSTRUCTION
        self._channels: list[Channel] = []
        self._topics: CameraTopics | None = None
        self._camera_ready = False
        self._ros = RosBackend.get()

        self._rgb_lock = threading.Lock()
        self._depth_lock = threading.Lock()
        self._intrinsics_lock = threading.Lock()
        self._latest_rgb: Image | None = None
        self._latest_depth: Image | None = None
        self._latest_intrinsics: CameraIntrinsics | None = None

        if provider_id:
            self._connect_camera(provider_id)

    def close(self) -> None:
        for ch in self._channels:
            try:
                ch.close()
            except Exception as exc:  # noqa: BLE001
                log.debug("camera channel close failed: %s", exc)
        self._channels.clear()
        self._camera_ready = False
        self._topics = None

    @property
    def available(self) -> bool:
        return self._camera_ready

    def _connect_topic(self, contract_id: str, *, qos: str = "best_effort") -> str:
        caps = ATLAS.find_capability(
            contract_id=contract_id,
            transport="ros2",
            provider_id=self.provider_id,
        )
        if not caps:
            raise RuntimeError(
                f"no ros2 capability {contract_id!r} for provider {self.provider_id!r}"
            )
        ch = ATLAS.connect_capability(
            consumer_id=self.consumer_id,
            provider_id=caps[0].provider_id,
            contract_id=contract_id,
            transport="ros2",
        )
        endpoint = (ch.endpoint or "").strip()
        if not endpoint:
            ch.close()
            raise RuntimeError(
                f"connect_capability returned empty endpoint for "
                f"{self.provider_id!r} {contract_id!r}"
            )
        self._channels.append(ch)
        log.info(
            "camera capability connected: provider=%s contract=%s topic=%s qos=%s",
            self.provider_id,
            contract_id,
            endpoint,
            qos,
        )
        return endpoint

    def _on_rgb(self, msg: Image) -> None:
        with self._rgb_lock:
            self._latest_rgb = msg

    def _on_depth(self, msg: Image) -> None:
        with self._depth_lock:
            self._latest_depth = msg

    def _on_intrinsics(self, msg: CameraInfo) -> None:
        try:
            intrinsics = intrinsics_from_camera_info(msg)
        except Exception as exc:  # noqa: BLE001
            log.warning("failed to parse camera intrinsics: %s", exc)
            return
        with self._intrinsics_lock:
            self._latest_intrinsics = intrinsics

    def _connect_camera(self, provider_id: str) -> None:
        """Resolve camera rgb/depth/intrinsics topics and start ROS subscriptions."""
        self.provider_id = provider_id
        try:
            rgb_topic = self._connect_topic("robonix/primitive/camera/rgb", qos="best_effort")
            depth_topic = self._connect_topic("robonix/primitive/camera/depth", qos="best_effort")
            intrinsics_topic: str | None
            try:
                intrinsics_topic = self._connect_topic(
                    "robonix/primitive/camera/intrinsics",
                    qos="reliable",
                )
            except RuntimeError:
                intrinsics_topic = None
                log.info(
                    "camera intrinsics not available for %r; using scaled defaults",
                    provider_id,
                )
        except RuntimeError as exc:
            log.info("camera primitive %r not ready: %s", provider_id, exc)
            return

        self._topics = CameraTopics(rgb=rgb_topic, depth=depth_topic, intrinsics=intrinsics_topic)
        self._ros.create_subscription(Image, rgb_topic, self._on_rgb, "best_effort")
        self._ros.create_subscription(Image, depth_topic, self._on_depth, "best_effort")
        if intrinsics_topic:
            self._ros.create_subscription(CameraInfo, intrinsics_topic, self._on_intrinsics, "reliable")

        if not self._ros.wait_for_topic(rgb_topic, Image, self._frame_timeout_s):
            log.warning(
                "no RGB frame on %s within %.1fs; fetch_rgbd may block until stream starts",
                rgb_topic,
                self._frame_timeout_s,
            )
        else:
            log.info("first RGB frame received on %s", rgb_topic)

        self._camera_ready = True
        log.info(
            "camera ready: provider=%s rgb=%s depth=%s intrinsics=%s",
            provider_id,
            rgb_topic,
            depth_topic,
            intrinsics_topic or "default",
        )

    @staticmethod
    def _stamp_ns(msg: Image | CameraInfo) -> int:
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _wait_for_frames(self, timeout_s: float) -> tuple[Image, Image] | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._rgb_lock, self._depth_lock:
                rgb = self._latest_rgb
                depth = self._latest_depth
            if rgb is not None and depth is not None:
                rgb_ns = self._stamp_ns(rgb)
                depth_ns = self._stamp_ns(depth)
                if abs(rgb_ns - depth_ns) <= SYNC_TOLERANCE_NS:
                    return rgb, depth
                # Streams are publishing but stamps diverged — use latest pair anyway.
                log.debug(
                    "rgb/depth stamp skew %.1f ms; using latest pair",
                    abs(rgb_ns - depth_ns) / 1e6,
                )
                return rgb, depth
            time.sleep(0.02)
        return None

    def fetch_rgbd(self) -> RGBDFrame | None:
        """Fetch one synchronized RGB-D frame from the camera primitive."""
        if not self._camera_ready or self._topics is None:
            log.info("fetch_rgbd skipped (camera not connected, provider=%r)", self.provider_id)
            return None

        pair = self._wait_for_frames(self._frame_timeout_s)
        if pair is None:
            log.warning(
                "fetch_rgbd timed out after %.1fs (rgb=%s depth=%s)",
                self._frame_timeout_s,
                self._topics.rgb,
                self._topics.depth,
            )
            return None

        rgb_msg, depth_msg = pair
        with self._intrinsics_lock:
            intrinsics = self._latest_intrinsics

        try:
            frame = image_msg_to_rgbd_frame(
                rgb_msg,
                depth_msg,
                intrinsics=intrinsics,
                depth_scale=self._depth_scale,
            )
        except Exception as exc:  # noqa: BLE001
            log.warning("fetch_rgbd decode failed: %s", exc)
            return None

        if frame.rgb is not None and frame.rgb.shape[:2] != frame.depth_m.shape[:2]:
            log.warning(
                "RGB %s and depth %s size mismatch after align_depth",
                frame.rgb.shape[:2],
                frame.depth_m.shape[:2],
            )
            return None

        log.info(
            "fetch_rgbd ok: %dx%d depth=%s intrinsics=%s",
            frame.depth_m.shape[1],
            frame.depth_m.shape[0],
            depth_msg.encoding,
            "camera_info" if intrinsics is not None else "default",
        )
        return frame

    def _camera_target_to_grasp_target(self, cam: CameraTarget) -> GraspTarget:
        if self._calib is None:
            raise RuntimeError("camera calibration not loaded; cannot map camera target to robot frame")
        robot_xyz = self._calib.camera_to_robot_xyz(cam.xyz_m)
        return GraspTarget(
            robot_xyz=robot_xyz,
            camera_xyz=list(cam.xyz_m),
            label=cam.label,
            confidence=cam.confidence,
        )

    def detect_object(self, *, object_name: str | None = None) -> GraspTarget | None:
        """Detect grasp target: RGB-D -> detector -> camera frame -> robot base."""
        rgbd = self.fetch_rgbd()
        if rgbd is None:
            log.info("detect skipped: no RGB-D frame")
            return None

        user_instruction = resolve_user_instruction(object_name, self._default_instruction)
        cam_target = self._detector.detect(rgbd, user_instruction=user_instruction)
        if cam_target is None:
            log.info("detect skipped: detector returned no target")
            return None

        if self._calib is None:
            log.error("detect produced camera target but calibration is missing")
            return None

        grasp_target = self._camera_target_to_grasp_target(cam_target)
        log.info(
            "detect -> camera %s  robot %s",
            [round(v, 4) for v in grasp_target.camera_xyz or []],
            [round(v, 4) for v in grasp_target.robot_xyz],
        )
        return grasp_target
