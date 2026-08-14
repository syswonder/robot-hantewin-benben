"""Atlas-resolved ROS clients for return-home joint + gripper commands."""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

from robonix_api import ATLAS
from robonix_api.atlas_types import Channel
from robonix_api.ros import RosBackend
from sensor_msgs.msg import JointState  # type: ignore
from std_msgs.msg import Float64MultiArray  # type: ignore

log = logging.getLogger("nero_return_home.arm_client")

JOINT_TOLERANCE_RAD = 0.05
SETTLE_S = 0.4
PUBLISHER_MATCH_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class ArmTopics:
    joint_command: str
    gripper_command: str | None
    joint_states: str


class ArmClient:
    def __init__(self, consumer_id: str, provider_id: str) -> None:
        self.consumer_id = consumer_id
        self.provider_id = provider_id
        self._channels: list[Channel] = []
        self.topics = self._resolve_topics()
        ros = RosBackend.get()
        self._joint_pub = ros.create_publisher(JointState, self.topics.joint_command, "reliable")
        self._gripper_pub = None
        if self.topics.gripper_command:
            self._gripper_pub = ros.create_publisher(
                Float64MultiArray, self.topics.gripper_command, "reliable"
            )
        self._latest_joints: list[float] | None = None
        self._joint_names: list[str] = []
        self._joint_lock = threading.Lock()
        ros.create_subscription(JointState, self.topics.joint_states, self._on_joints, "reliable")
        log.info(
            "[%s] atlas channels ready: joint_cmd=%s gripper=%s",
            provider_id,
            self.topics.joint_command,
            self.topics.gripper_command or "none",
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
            log.info("[%s] gripper_command not available", self.provider_id)
        return ArmTopics(
            joint_command=self._connect_topic("robonix/primitive/arm/joint_command"),
            gripper_command=gripper_topic,
            joint_states=self._connect_topic("robonix/primitive/arm/joint_states"),
        )

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
            with self._joint_lock:
                if self._latest_joints is not None:
                    return
            time.sleep(0.05)
        raise TimeoutError(
            f"timed out waiting for joint feedback from {self.provider_id} ({timeout_s:.1f}s)"
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
        self._wait_for_joint_target(target, timeout_s=timeout_s or 20.0, before=before)

    def set_gripper(self, width_m: float, force_n: float) -> None:
        if self._gripper_pub is None:
            log.warning("[%s] gripper command skipped (no publisher)", self.provider_id)
            return
        msg = Float64MultiArray()
        msg.data = [float(width_m), float(force_n)]
        self._gripper_pub.publish(msg)
        time.sleep(1.0)

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
