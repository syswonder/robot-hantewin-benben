#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""LakiBeam1 lidar primitive - ROS1 TCPROS LaserScan bridge.

Owns ``robonix/primitive/lidar/*`` (2D planar lidar surface). The LakiBeam1
publishes ``sensor_msgs/LaserScan`` on a remote ROS1 node
(``/richbeam_lidar`` -> ``/scan_filter``). This driver subscribes to that
stream via direct TCPROS (no ROS 1 environment needed on the host), bridges
each scan to ROS 2, and exposes an MCP snapshot tool for one-shot capture.

The ROS1 TCPROS bridging follows the same pattern as the chassis driver's
``/odom`` bridge: a background thread maintains the connection, stores the
latest frame, and a ROS 2 timer republishes it on the host DDS bus.

Capability surface:

  primitive/lidar/lidar     topic_out  ROS 2 LaserScan stream
  primitive/lidar/snapshot  rpc        MCP one-shot LaserScan capture
  primitive/lidar/driver    rpc        gRPC lifecycle (Init waits for first scan)
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
    level=os.environ.get("LAKIBEAM1_LOG_LEVEL", "INFO"),
    format="[lakibeam1] %(message)s",
)
log = logging.getLogger("lakibeam1")

lakibeam1_lidar = Primitive(
    id="lakibeam1_lidar",
    namespace="robonix/primitive/lidar",
)

# ── shared state - the latest parsed ROS1 LaserScan ─────────────────────────
_scan_lock = threading.Lock()
_latest_scan: dict | None = None
_last_pub_seq: int = -1            # seq of last published scan (dup suppression)
_scan_receiver: "_Ros1LaserScanReceiver | None" = None

# ── ROS 2 handles ───────────────────────────────────────────────────────────
_scan_pub = None                   # rclpy publisher -> /scan
_scan_timer = None                 # rclpy timer for periodic scan publishing
_stp_proc: subprocess.Popen | None = None  # static_transform_publisher
_frame_id: str = "laser"           # configured frame_id (overrides ROS1 scan's)
_clock = None                      # ROS 2 clock (host time, set in on_init)

# ── ROS1 LaserScan constants ────────────────────────────────────────────────
_LASERSCAN_MD5SUM = "90c7ef2dc6895d81024acba2ac42f369"
_LASERSCAN_TYPE = "sensor_msgs/LaserScan"
_ROS1_CALLER_ID = "/lakibeam1_lidar_bridge"


# ── ROS1 TCPROS helpers ─────────────────────────────────────────────────────
def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("ROS1 lidar connection closed by peer")
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


def _parse_ros1_laserscan(data: bytes) -> dict:
    """Parse a ROS1 sensor_msgs/LaserScan TCPROS message into a dict.

    ROS1 LaserScan layout (TCPROS wire format):
      Header: seq(u32), stamp{sec,nsec}(u32×2), frame_id(string)
      angle_min, angle_max, angle_increment, time_increment,
      scan_time, range_min, range_max  (float32×7)
      ranges:     length(u32) + float32[length]
      intensities: length(u32) + float32[length]
    """
    offset = 0
    (seq,) = struct.unpack_from("<I", data, offset)
    offset += 4
    sec, nsec = struct.unpack_from("<II", data, offset)
    offset += 8
    frame_id, offset = _parse_ros_string(data, offset)

    (angle_min, angle_max, angle_increment, time_increment, scan_time,
     range_min, range_max) = struct.unpack_from("<7f", data, offset)
    offset += 7 * 4

    (ranges_len,) = struct.unpack_from("<I", data, offset)
    offset += 4
    ranges = struct.unpack_from(f"<{ranges_len}f", data, offset)
    offset += ranges_len * 4

    (intensities_len,) = struct.unpack_from("<I", data, offset)
    offset += 4
    intensities = struct.unpack_from(f"<{intensities_len}f", data, offset)
    offset += intensities_len * 4

    return {
        "seq": seq,
        "sec": sec,
        "nsec": nsec,
        "frame_id": frame_id,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": angle_increment,
        "time_increment": time_increment,
        "scan_time": scan_time,
        "range_min": range_min,
        "range_max": range_max,
        "ranges": ranges,
        "intensities": intensities,
    }


def _connect_ros1_scan(publisher_uri: str, topic: str) -> socket.socket:
    """Negotiate a TCPROS connection to a remote ROS1 LaserScan publisher."""
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
        "md5sum": _LASERSCAN_MD5SUM,
        "type": _LASERSCAN_TYPE,
        "tcp_nodelay": "1",
    }))
    response = _read_tcpros_header(sock)
    if "error" in response:
        sock.close()
        raise RuntimeError(f"ROS1 TCPROS handshake failed: {response['error']}")
    return sock


class _Ros1LaserScanReceiver(threading.Thread):
    """Keep the latest remote ROS1 LaserScan, reconnecting after failures."""

    def __init__(self, publisher_uri: str, topic: str) -> None:
        super().__init__(daemon=True, name="ros1-laserscan-receiver")
        self._publisher_uri = publisher_uri
        self._topic = topic
        self._stop_event = threading.Event()
        self._sock: socket.socket | None = None
        self._first_scan_event = threading.Event()

    def run(self) -> None:
        global _latest_scan
        while not self._stop_event.is_set():
            try:
                self._sock = _connect_ros1_scan(self._publisher_uri, self._topic)
                log.info("connected to ROS1 LaserScan publisher %s on %s",
                         self._publisher_uri, self._topic)
                while not self._stop_event.is_set():
                    scan = _parse_ros1_laserscan(_read_tcpros_message(self._sock))
                    with _scan_lock:
                        _latest_scan = scan
                    if not self._first_scan_event.is_set():
                        self._first_scan_event.set()
                        log.info("first LaserScan received: %d points, frame=%s",
                                 len(scan["ranges"]), scan["frame_id"])
            except Exception as exc:  # noqa: BLE001
                if not self._stop_event.is_set():
                    log.warning("ROS1 LaserScan receive failed: %s", exc)
                    self._stop_event.wait(1.0)
            finally:
                sock, self._sock = self._sock, None
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass

    def wait_for_first_scan(self, timeout: float) -> bool:
        return self._first_scan_event.wait(timeout)

    def stop(self) -> None:
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


# ── static_transform_publisher: parent_frame -> frame_id ──────────────────
# Same pattern as the mid360 driver: when extrinsics is present, spawn a
# static_transform_publisher so consumers (mapping, navigation) see a
# complete TF tree rooted at base_link without needing chassis or soma.
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
    child = str(cfg.get("frame_id", "laser"))
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


# ── ROS 2 LaserScan publishing ──────────────────────────────────────────────
def _publish_scan() -> None:
    """Publish the latest ROS1 LaserScan as ROS2 sensor_msgs/LaserScan.

    Duplicate suppression: skip publishing when the latest scan's seq
    matches the last published one, so the ROS 2 publish rate never
    exceeds the actual lidar frame rate even when the timer ticks faster.
    """
    global _last_pub_seq
    if _scan_pub is None:
        return
    try:
        from sensor_msgs.msg import LaserScan  # type: ignore

        with _scan_lock:
            scan = _latest_scan
        if scan is None:
            return
        if scan["seq"] == _last_pub_seq:
            return
        _last_pub_seq = scan["seq"]

        msg = LaserScan()
        # Stamp with ROS 2 host clock, not the remote ROS1 clock.
        # The ROS1 node's clock may differ from the host, causing TF
        # extrapolation errors in consumers (rtabmap, nav2).
        if _clock is not None:
            from rclpy.duration import Duration
            # Back-date by one scan period: the scan was measured over the
            # previous `scan_time`, and stamping at "now" makes nav2 look up
            # odom->base_link at a timestamp slightly beyond the latest odom
            # TF, which tf2 refuses ("extrapolation into the future" — tf2
            # does not interpolate past the newest sample).
            stamp = (_clock.now() - Duration(
                seconds=max(float(scan["scan_time"]), 0.05))).to_msg()
            msg.header.stamp = stamp
        else:
            msg.header.stamp.sec = scan["sec"]
            msg.header.stamp.nanosec = scan["nsec"]
        msg.header.frame_id = _frame_id
        msg.angle_min = float(scan["angle_min"])
        msg.angle_max = float(scan["angle_max"])
        msg.angle_increment = float(scan["angle_increment"])
        msg.time_increment = float(scan["time_increment"])
        msg.scan_time = float(scan["scan_time"])
        msg.range_min = float(scan["range_min"])
        msg.range_max = float(scan["range_max"])
        msg.ranges = [float(r) for r in scan["ranges"]]
        msg.intensities = ([float(i) for i in scan["intensities"]]
                           if len(scan["intensities"]) else [])
        _scan_pub.publish(msg)
    except Exception as exc:  # noqa: BLE001
        log.warning("LaserScan publish failed: %s", exc)


# ── MCP snapshot tool (typed against codegen MCP dataclasses) ──────────────
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import LaserScan as LaserScanMcp  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def _ros_to_mcp(scan: dict) -> LaserScanMcp:
    stamp = builtin_interfaces_mcp.Time(
        sec=int(scan["sec"]), nanosec=int(scan["nsec"])
    )
    header = std_msgs_mcp.Header(stamp=stamp, frame_id=str(scan["frame_id"]))
    intensities = ([float(x) for x in scan["intensities"]]
                   if len(scan["intensities"]) else [])
    return LaserScanMcp(
        header=header,
        angle_min=float(scan["angle_min"]),
        angle_max=float(scan["angle_max"]),
        angle_increment=float(scan["angle_increment"]),
        time_increment=float(scan["time_increment"]),
        scan_time=float(scan["scan_time"]),
        range_min=float(scan["range_min"]),
        range_max=float(scan["range_max"]),
        ranges=[float(r) for r in scan["ranges"]],
        intensities=intensities,
    )


@lakibeam1_lidar.mcp("robonix/primitive/lidar/snapshot")
def snapshot(msg: Empty) -> LaserScanMcp:
    """Get the latest planar lidar scan. Returns sensor_msgs/LaserScan;
    ``ranges[i]`` is the distance (m) at angle ``angle_min + i*angle_increment``.
    Useful for "obstacle in front?" / "where's the nearest open space?"
    Contract: robonix/primitive/lidar/snapshot."""
    _ = msg
    with _scan_lock:
        scan = _latest_scan
    if scan is None:
        raise RuntimeError("no LaserScan received yet")
    return _ros_to_mcp(scan)


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
@lakibeam1_lidar.on_init
def init(cfg):
    global _scan_pub, _scan_timer, _scan_receiver, _stp_proc, _frame_id, _clock

    # ── Config ──
    scan_topic = (
        cfg.get("scan_topic")
        or os.environ.get("LAKIBEAM1_SCAN_TOPIC", "/scan")
    )
    scan_hz = float(
        cfg.get("scan_hz")
        or os.environ.get("LAKIBEAM1_SCAN_HZ", "10")
    )
    target_ip = cfg.get("ros1_target_ip") or os.environ.get("LAKIBEAM1_ROS1_TARGET_IP") or "192.168.10.1"
    # ros1_publisher_uri = (
    #     cfg.get("ros1_publisher_uri")
    #     or os.environ.get("LAKIBEAM1_ROS1_PUBLISHER_URI")
    #     or "http://192.168.10.1:39233/"
    # )
    ros1_topic = (
        cfg.get("ros1_topic")
        or os.environ.get("LAKIBEAM1_ROS1_TOPIC")
        or "/scan_filter"
    )
    port = get_topic_publisher_port(ros1_topic)
    if port is None:
        return Err(f"Failed to get publisher port for topic {ros1_topic}")
    ros1_publisher_uri = f'http://{target_ip}:{port}/'
    _frame_id = str(
        cfg.get("frame_id")
        or os.environ.get("LAKIBEAM1_FRAME_ID", "laser")
    )
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 15.0))
    if scan_hz <= 0:
        return Err("scan_hz must be greater than zero")

    # ── ROS1 TCPROS receiver ──
    _scan_receiver = _Ros1LaserScanReceiver(ros1_publisher_uri, ros1_topic)
    _scan_receiver.start()

    if not _scan_receiver.wait_for_first_scan(sentinel_timeout):
        _scan_receiver.stop()
        _scan_receiver.join(timeout=2.0)
        _scan_receiver = None
        return Err(
            f"no LaserScan received from {ros1_publisher_uri} on {ros1_topic} "
            f"within {sentinel_timeout:.1f}s"
        )

    # ── ROS 2: LaserScan publisher + timer ──
    from sensor_msgs.msg import LaserScan  # type: ignore
    _scan_pub = lakibeam1_lidar.create_publisher(
        "robonix/primitive/lidar/lidar",
        topic=scan_topic, msg_type=LaserScan, qos="reliable",
    )

    from robonix_api.ros import RosBackend
    _clock = RosBackend.get().node.get_clock()
    period = 1.0 / scan_hz
    _scan_timer = RosBackend.get().node.create_timer(period, _publish_scan)

    # parent_frame -> frame_id static TF (no-op when extrinsics absent).
    try:
        _spawn_stp(cfg)
    except Exception as e:  # noqa: BLE001
        _scan_receiver.stop()
        _scan_receiver.join(timeout=2.0)
        _scan_receiver = None
        return Err(f"spawn static_transform_publisher failed: {e}")

    log.info(
        "init complete: ROS1 %s%s -> ROS2 %s @ %.1f Hz (frame=%s)",
        ros1_publisher_uri, ros1_topic, scan_topic, scan_hz, _frame_id,
    )
    return Ok()


@lakibeam1_lidar.on_shutdown
def shutdown():
    global _scan_receiver, _scan_pub, _scan_timer, _latest_scan, _stp_proc

    # Stop the static_transform_publisher.
    _kill_stp()
    _stp_proc = None

    # Stop the ROS1 TCPROS receiver before dropping ROS2 handles.
    if _scan_receiver is not None:
        _scan_receiver.stop()
        _scan_receiver.join(timeout=2.0)
        _scan_receiver = None
    with _scan_lock:
        _latest_scan = None

    # Cancel scan timer.
    if _scan_timer is not None:
        try:
            _scan_timer.cancel()
        except Exception:  # noqa: BLE001
            pass
        _scan_timer = None

    _scan_pub = None
    return Ok()


if __name__ == "__main__":
    lakibeam1_lidar.run()
