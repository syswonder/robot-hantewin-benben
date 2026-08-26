"""Atlas-resolved ROS clients for one Nero arm primitive instance."""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass

from geometry_msgs.msg import Pose  # type: ignore
from robonix_api import ATLAS
from robonix_api.atlas_types import Channel
from robonix_api.ros import RosBackend
from sensor_msgs.msg import JointState  # type: ignore
from std_msgs.msg import Float64MultiArray  # type: ignore

log = logging.getLogger("nero_wave.arm_client")

JOINT_TOLERANCE_RAD = 0.05
TCP_POS_TOLERANCE_M = 0.005
SETTLE_S = 0.0
PUBLISHER_MATCH_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class ArmTopics:
    joint_command: str
    pos_command: str
    linear_pos_command: str | None
    gripper_command: str | None
    end_pose: str
    joint_states: str


class ArmClient:
    def __init__(self, consumer_id: str, provider_id: str) -> None:
        self.consumer_id = consumer_id
        self.provider_id = provider_id
        self._channels: list[Channel] = []
        self.topics = self._resolve_topics()
        ros = RosBackend.get()
        self._joint_pub = ros.create_publisher(JointState, self.topics.joint_command, "reliable")
        self._pos_pub = ros.create_publisher(Pose, self.topics.pos_command, "reliable")
        self._linear_pos_pub = None
        if self.topics.linear_pos_command:
            self._linear_pos_pub = ros.create_publisher(
                Pose, self.topics.linear_pos_command, "reliable"
            )
        self._gripper_pub = None
        if self.topics.gripper_command:
            self._gripper_pub = ros.create_publisher(
                Float64MultiArray, self.topics.gripper_command, "reliable"
            )
        self._latest_pose: Pose | None = None
        self._latest_joints: list[float] | None = None
        self._joint_names: list[str] = []
        self._home_joint_pose: list[float] | None = None
        self._pose_lock = threading.Lock()
        self._joint_lock = threading.Lock()
        ros.create_subscription(Pose, self.topics.end_pose, self._on_pose, "reliable")
        ros.create_subscription(JointState, self.topics.joint_states, self._on_joints, "reliable")
        log.info(
            "[%s] atlas channels ready: joint_cmd=%s pos_cmd=%s linear_pos_cmd=%s end_pose=%s",
            provider_id,
            self.topics.joint_command,
            self.topics.pos_command,
            self.topics.linear_pos_command or "none",
            self.topics.end_pose,
        )

    def close(self) -> None:
        for ch in self._channels:
            try:
                ch.close()
            except Exception as exc:  # noqa: BLE001
                log.debug("[%s] channel close failed: %s", self.provider_id, exc)
        self._channels.clear()

    def _connect_topic(self, contract_id: str) -> str:
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
        return endpoint

    def _resolve_topics(self) -> ArmTopics:
        gripper_topic: str | None
        try:
            gripper_topic = self._connect_topic("robonix/primitive/arm/gripper_command")
        except RuntimeError:
            gripper_topic = None
            log.info("[%s] gripper_command not available; gripper commands disabled", self.provider_id)

        linear_pos_topic: str | None
        try:
            linear_pos_topic = self._connect_topic("robonix/primitive/arm/linear_pos_command")
        except RuntimeError:
            linear_pos_topic = None
            log.warning(
                "[%s] linear_pos_command not available; linear grasp will fall back to pos_command",
                self.provider_id,
            )

        return ArmTopics(
            joint_command=self._connect_topic("robonix/primitive/arm/joint_command"),
            pos_command=self._connect_topic("robonix/primitive/arm/pos_command"),
            linear_pos_command=linear_pos_topic,
            gripper_command=gripper_topic,
            end_pose=self._connect_topic("robonix/primitive/arm/end_pose"),
            joint_states=self._connect_topic("robonix/primitive/arm/joint_states"),
        )

    def _on_pose(self, msg: Pose) -> None:
        with self._pose_lock:
            self._latest_pose = msg

    def _on_joints(self, msg: JointState) -> None:
        if len(msg.position) < 7:
            return
        with self._joint_lock:
            self._latest_joints = [float(v) for v in msg.position[:7]]
            if msg.name and len(msg.name) >= 7:
                self._joint_names = [str(n) for n in msg.name[:7]]

    def wait_for_feedback(self, timeout_s: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._pose_lock, self._joint_lock:
                if self._latest_pose is not None and self._latest_joints is not None:
                    if self._home_joint_pose is None:
                        self._home_joint_pose = list(self._latest_joints)
                    return
            time.sleep(0.05)
        raise TimeoutError(
            f"timed out waiting for arm feedback from {self.provider_id} ({timeout_s:.1f}s)"
        )

    @staticmethod
    def _wait_for_subscribers(pub, *, label: str, timeout_s: float = PUBLISHER_MATCH_TIMEOUT_S) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                count = int(pub.get_subscription_count())
            except Exception:  # noqa: BLE001
                count = 0
            if count > 0:
                return
            time.sleep(0.05)
        raise RuntimeError(f"{label}: no ROS2 subscriber matched within {timeout_s:.1f}s")

    def move_joints(self, joints: list[float], *, timeout_s: float | None = None) -> None:
        if len(joints) != 7:
            raise ValueError(f"expected 7 joint targets, got {len(joints)}")
        target = [float(v) for v in joints]
        before = None
        with self._joint_lock:
            if self._latest_joints is not None:
                before = list(self._latest_joints)
        msg = JointState()
        if self._joint_names:
            msg.name = list(self._joint_names)
        msg.position = target
        self._wait_for_subscribers(self._joint_pub, label=f"{self.provider_id} joint_command")
        log.info("[%s] publish joint_command -> %s", self.provider_id, [round(v, 3) for v in target])
        self._joint_pub.publish(msg)
        log.info("[%s] published -> %s", self.provider_id, msg)
        self._wait_for_joint_target(target, timeout_s=timeout_s or 20.0, before=before)

    def _publish_tcp_pose(
        self,
        pose_rpy: list[float],
        *,
        pub,
        label: str,
        timeout_s: float | None,
    ) -> None:
        x, y, z, roll, pitch, yaw = pose_rpy
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        target = [float(v) for v in pose_rpy]
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        self._wait_for_subscribers(pub, label=f"{self.provider_id} {label}")
        log.info("[%s] publish %s -> %s", self.provider_id, label, [round(v, 3) for v in target[:3]])
        pub.publish(msg)
        self._wait_for_tcp_target(target, timeout_s=timeout_s or 20.0)

    def move_tcp_rpy(self, pose_rpy: list[float], *, timeout_s: float | None = None) -> None:
        if len(pose_rpy) != 6:
            raise ValueError(f"expected 6D TCP pose, got {len(pose_rpy)}")
        self._publish_tcp_pose(
            pose_rpy,
            pub=self._pos_pub,
            label="pos_command",
            timeout_s=timeout_s,
        )

    def move_tcp_linear_rpy(self, pose_rpy: list[float], *, timeout_s: float | None = None) -> None:
        if len(pose_rpy) != 6:
            raise ValueError(f"expected 6D TCP pose, got {len(pose_rpy)}")
        pub = self._linear_pos_pub or self._pos_pub
        label = "linear_pos_command" if self._linear_pos_pub is not None else "pos_command (fallback)"
        if self._linear_pos_pub is None:
            log.warning("[%s] linear_pos_command unavailable, using pos_command", self.provider_id)
        self._publish_tcp_pose(
            pose_rpy,
            pub=pub,
            label=label,
            timeout_s=timeout_s,
        )

    def set_gripper(self, width_m: float, force_n: float) -> None:
        if self._gripper_pub is None:
            log.warning("[%s] gripper command skipped (no publisher)", self.provider_id)
            return
        msg = Float64MultiArray()
        msg.data = [float(width_m), float(force_n)]
        self._gripper_pub.publish(msg)
        time.sleep(1.0)

    def go_home(self, *, timeout_s: float | None = None) -> None:
        if self._home_joint_pose is None:
            raise RuntimeError(f"{self.provider_id} has no recorded home joint pose")
        self.move_joints(self._home_joint_pose, timeout_s=timeout_s)

    @staticmethod
    def _pose_xyz(pose: Pose) -> tuple[float, float, float]:
        return float(pose.position.x), float(pose.position.y), float(pose.position.z)

    def _wait_for_joint_target(
        self,
        target: list[float],
        *,
        timeout_s: float,
        before: list[float] | None = None,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        stable_since: float | None = None
        while time.monotonic() < deadline:
            with self._joint_lock:
                joints = None if self._latest_joints is None else list(self._latest_joints)
            if joints is not None:
                error = max(abs(a - t) for a, t in zip(joints, target))
                if error <= JOINT_TOLERANCE_RAD:
                    if before is not None:
                        delta = max(abs(a - b) for a, b in zip(joints, before))
                        if delta <= JOINT_TOLERANCE_RAD:
                            log.info("[%s] already at joint target (no motion)", self.provider_id)
                            return
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= SETTLE_S:
                        log.info("[%s] joint target reached", self.provider_id)
                        return
                else:
                    stable_since = None
            time.sleep(0.05)
        raise TimeoutError(
            f"timed out waiting for {self.provider_id} to reach joint target ({timeout_s:.1f}s)"
        )

    def _wait_for_tcp_target(self, target: list[float], *, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        stable_since: float | None = None
        target_xyz = target[:3]
        while time.monotonic() < deadline:
            with self._pose_lock:
                pose = self._latest_pose
            if pose is not None:
                xyz = self._pose_xyz(pose)
                error = math.dist(xyz, target_xyz)
                if error <= TCP_POS_TOLERANCE_M:
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= SETTLE_S:
                        log.info("[%s] tcp target reached", self.provider_id)
                        return
                else:
                    stable_since = None
            time.sleep(0.05)
        raise TimeoutError(
            f"timed out waiting for {self.provider_id} to reach tcp target ({timeout_s:.1f}s)"
        )

    def get_tcp_pose_rpy(self) -> list[float]:
        with self._pose_lock:
            pose = self._latest_pose
        if pose is None:
            raise RuntimeError(f"no end_pose feedback from {self.provider_id}")
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        qx = float(pose.orientation.x)
        qy = float(pose.orientation.y)
        qz = float(pose.orientation.z)
        qw = float(pose.orientation.w)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return [x, y, z, roll, pitch, yaw]
