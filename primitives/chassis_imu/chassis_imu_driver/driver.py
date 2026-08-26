#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Chassis IMU primitive - ROS1 TCPROS Imu bridge.

Owns ``robonix/primitive/imu/*``. This namespace is shared with the MID-360
IMU primitive (``mid360_imu``); only one IMU primitive is active at a time -
disable ``mid360_imu`` when running this chassis IMU.

The BenBen chassis controller publishes ``sensor_msgs/Imu`` on a remote
ROS1 node (topic ``/imu_data``).
This driver subscribes to that stream via direct TCPROS (no ROS 1
environment needed on the host), bridges each frame to ROS 2, and exposes
an MCP snapshot tool for one-shot capture.

The ROS1 TCPROS bridging follows the same pattern as the chassis driver's
``/odom`` bridge and the LakiBeam1 lidar driver's ``/scan_filter`` bridge:
a background thread maintains the connection, stores the latest frame, and
a ROS 2 timer republishes it on the host DDS bus.

Capability surface:

  primitive/imu/imu      topic_out  ROS 2 Imu stream
  primitive/imu/snapshot rpc        MCP one-shot Imu capture
  primitive/imu/driver   rpc        gRPC lifecycle (Init waits for first Imu)
"""
from __future__ import annotations

import logging
import os
import signal
import socket
import struct
import subprocess
import threading
import xmlrpc.client

from robonix_api import Primitive, Ok, Err
from urllib.parse import urlparse

logging.basicConfig(
    level=os.environ.get("CHASSIS_IMU_LOG_LEVEL", "INFO"),
    format="[chassis_imu] %(message)s",
)
log = logging.getLogger("chassis_imu")

chassis_imu = Primitive(
    id="chassis_imu",
    namespace="robonix/primitive/imu",
)

# ── shared state - the latest parsed ROS1 Imu ───────────────────────────────
_imu_lock = threading.Lock()
_latest_imu: dict | None = None
_last_pub_seq: int = -1            # seq of last published Imu (dup suppression)
_imu_receiver: "_Ros1ImuReceiver | None" = None

# ── ROS 2 handles ───────────────────────────────────────────────────────────
_imu_pub = None                    # rclpy publisher -> /imu
_imu_timer = None                  # rclpy timer for periodic Imu publishing
_stp_proc: subprocess.Popen | None = None  # static_transform_publisher
_frame_id: str = "imu_link"        # configured frame_id (overrides ROS1 Imu's)
_clock = None                      # ROS 2 clock (host time, set in on_init)

# ── ROS1 Imu constants ──────────────────────────────────────────────────────
_IMU_MD5SUM = "6a62c6daae103f4ff57a132d6f95cec2"
_IMU_TYPE = "sensor_msgs/Imu"
_ROS1_CALLER_ID = "/chassis_imu_bridge"


# ── ROS1 TCPROS helpers ─────────────────────────────────────────────────────
def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("ROS1 imu connection closed by peer")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _build_tcpros_header(fields: dict[str, str]) -> bytes:
    body = b""
    for key, value in fields.items():
        entry = f"{key}={value}".encode()
        body += struct.pack("<I", len(entry)) + entry
    return struct.pack("<I", len(body)) + body


def _read_tcpros_header(sock: socket.socket) -> dict[str, str]:
    (total_len,) = struct.unpack("<I", _recv_exact(sock, 4))
    body = _recv_exact(sock, total_len)
    fields: dict[str, str] = {}
    offset = 0
    while offset < len(body):
        (field_len,) = struct.unpack_from("<I", body, offset)
        offset += 4
        entry = body[offset : offset + field_len].decode(errors="replace")
        offset += field_len
        key, _, value = entry.partition("=")
        fields[key] = value
    return fields


def _read_tcpros_message(sock: socket.socket) -> bytes:
    (msg_len,) = struct.unpack("<I", _recv_exact(sock, 4))
    return _recv_exact(sock, msg_len)


def _parse_ros_string(data: bytes, offset: int) -> tuple[str, int]:
    (length,) = struct.unpack_from("<I", data, offset)
    offset += 4
    return data[offset : offset + length].decode(errors="replace"), offset + length


def _parse_ros1_imu(data: bytes) -> dict:
    """Parse a ROS1 sensor_msgs/Imu TCPROS message into a dict.

    ROS1 Imu layout (TCPROS wire format):
      Header: seq(u32), stamp{sec,nsec}(u32×2), frame_id(string)
      orientation: x,y,z,w (float64×4)
      orientation_covariance: float64[9]        (fixed-size, no length prefix)
      angular_velocity: x,y,z (float64×3)
      angular_velocity_covariance: float64[9]   (fixed-size, no length prefix)
      linear_acceleration: x,y,z (float64×3)
      linear_acceleration_covariance: float64[9] (fixed-size, no length prefix)
    """
    offset = 0
    (seq,) = struct.unpack_from("<I", data, offset)
    offset += 4
    sec, nsec = struct.unpack_from("<II", data, offset)
    offset += 8
    frame_id, offset = _parse_ros_string(data, offset)

    orientation = struct.unpack_from("<4d", data, offset)
    offset += 4 * 8
    orientation_covariance = struct.unpack_from("<9d", data, offset)
    offset += 9 * 8

    angular_velocity = struct.unpack_from("<3d", data, offset)
    offset += 3 * 8
    angular_velocity_covariance = struct.unpack_from("<9d", data, offset)
    offset += 9 * 8

    linear_acceleration = struct.unpack_from("<3d", data, offset)
    offset += 3 * 8
    linear_acceleration_covariance = struct.unpack_from("<9d", data, offset)
    offset += 9 * 8

    return {
        "seq": seq,
        "sec": sec,
        "nsec": nsec,
        "frame_id": frame_id,
        "orientation": orientation,                            # (x, y, z, w)
        "orientation_covariance": orientation_covariance,
        "angular_velocity": angular_velocity,                  # (x, y, z)
        "angular_velocity_covariance": angular_velocity_covariance,
        "linear_acceleration": linear_acceleration,            # (x, y, z)
        "linear_acceleration_covariance": linear_acceleration_covariance,
    }


def _connect_ros1_imu(publisher_uri: str, topic: str) -> socket.socket:
    """Negotiate a TCPROS connection to a remote ROS1 Imu publisher."""
    proxy = xmlrpc.client.ServerProxy(publisher_uri)
    code, status_msg, params = proxy.requestTopic(
        _ROS1_CALLER_ID, topic, [["TCPROS"]]
    )
    if code != 1:
        raise RuntimeError(f"ROS1 requestTopic failed: {status_msg}")
    protocol, host, port = params
    if protocol != "TCPROS":
        raise RuntimeError(f"ROS1 publisher returned unsupported protocol: {protocol}")

    sock = socket.create_connection((host, int(port)), timeout=5.0)
    sock.settimeout(None)
    sock.sendall(_build_tcpros_header({
        "callerid": _ROS1_CALLER_ID,
        "topic": topic,
        "md5sum": _IMU_MD5SUM,
        "type": _IMU_TYPE,
        "tcp_nodelay": "1",
    }))
    response = _read_tcpros_header(sock)
    if "error" in response:
        sock.close()
        raise RuntimeError(f"ROS1 TCPROS handshake failed: {response['error']}")
    return sock


class _Ros1ImuReceiver(threading.Thread):
    """Keep the latest remote ROS1 Imu frame, reconnecting after failures."""

    def __init__(self, publisher_uri: str, topic: str) -> None:
        super().__init__(daemon=True, name="ros1-imu-receiver")
        self._publisher_uri = publisher_uri
        self._topic = topic
        self._stop_event = threading.Event()
        self._sock: socket.socket | None = None
        self._first_imu_event = threading.Event()

    def run(self) -> None:
        global _latest_imu
        while not self._stop_event.is_set():
            try:
                self._sock = _connect_ros1_imu(self._publisher_uri, self._topic)
                log.info("connected to ROS1 Imu publisher %s on %s",
                         self._publisher_uri, self._topic)
                while not self._stop_event.is_set():
                    imu = _parse_ros1_imu(_read_tcpros_message(self._sock))
                    with _imu_lock:
                        _latest_imu = imu
                    if not self._first_imu_event.is_set():
                        self._first_imu_event.set()
                        log.info("first Imu received: seq=%d frame=%s",
                                 imu["seq"], imu["frame_id"])
            except Exception as exc:  # noqa: BLE001
                if not self._stop_event.is_set():
                    log.warning("ROS1 Imu receive failed: %s", exc)
                    self._stop_event.wait(1.0)
            finally:
                sock, self._sock = self._sock, None
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass

    def wait_for_first_imu(self, timeout: float) -> bool:
        return self._first_imu_event.wait(timeout)

    def stop(self) -> None:
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


# ── static_transform_publisher: parent_frame -> frame_id ──────────────────
# Same pattern as the mid360 / lakibeam1 drivers: when extrinsics is present,
# spawn a static_transform_publisher so consumers (mapping, lio) see a complete
# TF tree rooted at base_link without needing chassis or soma.
def _pump_output(stream, tag: str) -> None:
    """Forward a child process's merged stdout/stderr into the package log."""
    for raw in iter(stream.readline, b""):
        line = raw.decode(errors="replace").rstrip()
        if line:
            log.info("[%s] %s", tag, line)


def _spawn_stp(cfg: dict) -> None:
    global _stp_proc
    ext = cfg.get("extrinsics")
    if not ext:
        log.info("no extrinsics in cfg; assuming chassis/soma publishes "
                 "parent_frame -> frame_id elsewhere")
        return
    parent = str(cfg.get("parent_frame", "base_link"))
    child = str(cfg.get("frame_id", "imu_link"))
    args = [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", str(float(ext.get("x", 0.0))),
        "--y", str(float(ext.get("y", 0.0))),
        "--z", str(float(ext.get("z", 0.0))),
        "--roll", str(float(ext.get("roll", 0.0))),
        "--pitch", str(float(ext.get("pitch", 0.0))),
        "--yaw", str(float(ext.get("yaw", 0.0))),
        "--frame-id", parent,
        "--child-frame-id", child,
    ]
    log.info("spawning static_transform_publisher %s -> %s @ %s",
             parent, child, ext)
    _stp_proc = subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(target=_pump_output, args=(_stp_proc.stdout, "stp"),
                     daemon=True).start()


def _kill_stp() -> None:
    p = _stp_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── ROS 2 Imu publishing ────────────────────────────────────────────────────
def _publish_imu() -> None:
    """Publish the latest ROS1 Imu as ROS2 sensor_msgs/Imu.

    Duplicate suppression: skip publishing when the latest Imu's seq
    matches the last published one, so the ROS 2 publish rate never
    exceeds the actual IMU frame rate even when the timer ticks faster.
    """
    global _last_pub_seq
    if _imu_pub is None:
        return
    try:
        from sensor_msgs.msg import Imu  # type: ignore

        with _imu_lock:
            imu = _latest_imu
        if imu is None:
            return
        if imu["seq"] == _last_pub_seq:
            return
        _last_pub_seq = imu["seq"]

        ox, oy, oz, ow = imu["orientation"]
        ax, ay, az = imu["angular_velocity"]
        lx, ly, lz = imu["linear_acceleration"]

        msg = Imu()
        # Stamp with ROS 2 host clock, not the remote ROS1 clock (same policy
        # as the chassis /odom bridge - the controller's clock may drift).
        if _clock is not None:
            msg.header.stamp = _clock.now().to_msg()
        else:
            msg.header.stamp.sec = imu["sec"]
            msg.header.stamp.nanosec = imu["nsec"]
        msg.header.frame_id = _frame_id
        msg.orientation.x = ox
        msg.orientation.y = oy
        msg.orientation.z = oz
        msg.orientation.w = ow
        msg.orientation_covariance = list(imu["orientation_covariance"])
        msg.angular_velocity.x = ax
        msg.angular_velocity.y = ay
        msg.angular_velocity.z = az
        msg.angular_velocity_covariance = list(imu["angular_velocity_covariance"])
        msg.linear_acceleration.x = lx
        msg.linear_acceleration.y = ly
        msg.linear_acceleration.z = lz
        msg.linear_acceleration_covariance = list(imu["linear_acceleration_covariance"])
        _imu_pub.publish(msg)
    except Exception as exc:  # noqa: BLE001
        log.warning("Imu publish failed: %s", exc)


# ── MCP snapshot tool (typed against codegen MCP dataclasses) ──────────────
import builtin_interfaces_mcp  # noqa: E402
import geometry_msgs_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import Imu as ImuMcp  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def _ros_to_mcp(imu: dict) -> ImuMcp:
    stamp = builtin_interfaces_mcp.Time(
        sec=int(imu["sec"]), nanosec=int(imu["nsec"])
    )
    header = std_msgs_mcp.Header(stamp=stamp, frame_id=str(imu["frame_id"]))
    ox, oy, oz, ow = imu["orientation"]
    ax, ay, az = imu["angular_velocity"]
    lx, ly, lz = imu["linear_acceleration"]
    return ImuMcp(
        header=header,
        orientation=geometry_msgs_mcp.Quaternion(
            x=float(ox), y=float(oy), z=float(oz), w=float(ow)
        ),
        orientation_covariance=[float(c) for c in imu["orientation_covariance"]],
        angular_velocity=geometry_msgs_mcp.Vector3(
            x=float(ax), y=float(ay), z=float(az)
        ),
        angular_velocity_covariance=[float(c) for c in imu["angular_velocity_covariance"]],
        linear_acceleration=geometry_msgs_mcp.Vector3(
            x=float(lx), y=float(ly), z=float(lz)
        ),
        linear_acceleration_covariance=[float(c) for c in imu["linear_acceleration_covariance"]],
    )


@chassis_imu.mcp("robonix/primitive/imu/snapshot")
def snapshot(msg: Empty) -> ImuMcp:
    """Get the latest chassis IMU sample. Returns sensor_msgs/Imu with
    orientation (quaternion), angular_velocity (rad/s), and
    linear_acceleration (m/s^2), plus the three 3x3 covariance matrices.
    Useful for "am I tilted?" / "how fast am I turning?".
    Contract: robonix/primitive/imu/snapshot."""
    _ = msg
    with _imu_lock:
        imu = _latest_imu
    if imu is None:
        raise RuntimeError("no Imu received yet")
    return _ros_to_mcp(imu)


def get_topic_publisher_port(topic, master_uri=None, publisher_name=None):
    """获取指定话题的发布者端口号并返回。

    通过 ROS Master 查询:
    1. getSystemState 找到该话题的发布者节点列表
    2. lookupNode 查询发布者节点的 XML-RPC URI
    3. 解析出端口号返回

    Args:
        topic: 话题名,如 "/odom"
        master_uri: ROS Master 的 XML-RPC 地址,默认取 ROS_MASTER_URI 环境变量,
            缺省为 http://192.168.10.1:11311
        publisher_name: 指定要查的发布者节点名(如 "/motor");留空则取第一个

    Returns:
        int | None: 发布者端口号;话题无发布者、节点不可达或解析失败时返回 None
    """
    if master_uri is None:
        master_uri = os.environ.get('ROS_MASTER_URI', 'http://192.168.10.1:11311')

    proxy = xmlrpc.client.ServerProxy(master_uri + '/RPC2')
    caller_id = '/python_xmlrpc_client'

    # 1. 获取系统状态
    code, msg, state = proxy.getSystemState(caller_id)
    if code != 1:
        print(f"错误: {msg}")
        return None

    pub_list, _sub_list, _srv_list = state

    # 2. 找到该话题的发布者节点列表
    pubs = []
    for topic_name, pub_nodes in pub_list:
        if topic_name == topic:
            pubs = pub_nodes
            break
    if not pubs:
        print(f"话题 {topic} 没有发布者")
        return None

    # 3. 选择发布者节点
    if publisher_name is not None:
        if publisher_name not in pubs:
            print(f"节点 {publisher_name} 不是话题 {topic} 的发布者,可选: {', '.join(pubs)}")
            return None
        node_name = publisher_name
    else:
        node_name = pubs[0]

    # 4. 查询节点 XML-RPC URI 并解析端口号
    code, msg, node_uri = proxy.lookupNode(caller_id, node_name)
    if code != 1:
        print(f"查询节点 {node_name} 失败: {msg}")
        return None

    try:
        # node_uri 形如 http://192.168.1.100:34567/
        return urlparse(node_uri).port
    except Exception as e:
        print(f"解析 {node_name} 的 URI {node_uri} 失败: {e}")
        return None


# ── lifecycle ────────────────────────────────────────────────────────────────
@chassis_imu.on_init
def init(cfg):
    global _imu_pub, _imu_timer, _imu_receiver, _stp_proc, _frame_id, _clock, _last_pub_seq

    # ── Config ──
    imu_topic = (
        cfg.get("imu_topic")
        or os.environ.get("CHASSIS_IMU_TOPIC", "/imu")
    )
    imu_hz = float(
        cfg.get("imu_hz")
        or os.environ.get("CHASSIS_IMU_HZ", "100")
    )
    target_ip = (
        cfg.get("ros1_target_ip")
        or os.environ.get("CHASSIS_IMU_ROS1_TARGET_IP")
        or "192.168.10.1"
    )
    ros1_imu_topic = (
        cfg.get("ros1_imu_topic")
        or os.environ.get("CHASSIS_IMU_ROS1_TOPIC")
        or "/imu_data"
    )
    _frame_id = str(
        cfg.get("frame_id")
        or os.environ.get("CHASSIS_IMU_FRAME_ID", "imu_link")
    )
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 15.0))
    if imu_hz <= 0:
        return Err("imu_hz must be greater than zero")

    # Auto-discover the ROS1 publisher port via the ROS Master, then build
    # the publisher URI (same approach as the chassis /odom bridge).
    port = get_topic_publisher_port(ros1_imu_topic)
    if port is None:
        return Err(f"Failed to get publisher port for topic {ros1_imu_topic}")
    ros1_publisher_uri = f'http://{target_ip}:{port}/'

    # ── ROS1 TCPROS receiver ──
    _imu_receiver = _Ros1ImuReceiver(ros1_publisher_uri, ros1_imu_topic)
    _imu_receiver.start()

    if not _imu_receiver.wait_for_first_imu(sentinel_timeout):
        _imu_receiver.stop()
        _imu_receiver.join(timeout=2.0)
        _imu_receiver = None
        return Err(
            f"no Imu received from {ros1_publisher_uri} on {ros1_imu_topic} "
            f"within {sentinel_timeout:.1f}s"
        )

    # ── ROS 2: Imu publisher + timer ──
    from sensor_msgs.msg import Imu  # type: ignore
    _last_pub_seq = -1
    _imu_pub = chassis_imu.create_publisher(
        "robonix/primitive/imu/imu",
        topic=imu_topic, msg_type=Imu, qos="best_effort",
    )

    from robonix_api.ros import RosBackend
    _clock = RosBackend.get().node.get_clock()
    period = 1.0 / imu_hz
    _imu_timer = RosBackend.get().node.create_timer(period, _publish_imu)

    # parent_frame -> frame_id static TF (no-op when extrinsics absent).
    try:
        _spawn_stp(cfg)
    except Exception as e:  # noqa: BLE001
        _imu_receiver.stop()
        _imu_receiver.join(timeout=2.0)
        _imu_receiver = None
        return Err(f"spawn static_transform_publisher failed: {e}")

    log.info(
        "init complete: ROS1 %s%s -> ROS2 %s @ %.1f Hz (frame=%s)",
        ros1_publisher_uri, ros1_imu_topic, imu_topic, imu_hz, _frame_id,
    )
    return Ok()


@chassis_imu.on_shutdown
def shutdown():
    global _imu_receiver, _imu_pub, _imu_timer, _latest_imu, _stp_proc

    # Stop the static_transform_publisher.
    _kill_stp()
    _stp_proc = None

    # Stop the ROS1 TCPROS receiver before dropping ROS2 handles.
    if _imu_receiver is not None:
        _imu_receiver.stop()
        _imu_receiver.join(timeout=2.0)
        _imu_receiver = None
    with _imu_lock:
        _latest_imu = None

    # Cancel imu timer.
    if _imu_timer is not None:
        try:
            _imu_timer.cancel()
        except Exception:  # noqa: BLE001
            pass
        _imu_timer = None

    _imu_pub = None
    return Ok()


if __name__ == "__main__":
    chassis_imu.run()
