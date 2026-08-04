#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportArgumentType=false
"""Hantewin BenBen chassis primitive - TBox-SDK-based driver.

Owns ``robonix/primitive/chassis/*``.  Controls the physical chassis
through the TBox C++ SDK (``libtbox_sdk_cpp.so``) and bridges the controller's
remote ROS1 ``/odom`` stream to ROS2.  The SDK handles the TCP connection,
authorization handshake, and heartbeat internally; a global ``TBoxClient``
instance persists for the lifetime of the driver so that auth state is not
lost between commands.

Capability surface:

  primitive/chassis/driver         rpc        gRPC lifecycle (Capability built-in)
  primitive/chassis/move           rpc        gRPC + MCP ExecuteMoveCommand -
                                             burst-style velocity command.
  primitive/chassis/odom           topic_out  ROS 2 /odom
  primitive/chassis/odom_snapshot  rpc        MCP one-shot Odometry capture
  primitive/chassis/twist_in       topic_in   ROS 2 /cmd_vel
"""
from __future__ import annotations

import json
import logging
import math
import os
import socket
import struct
import threading
import time
import xmlrpc.client

from robonix_api import Primitive, Ok, Err, Deferred
from tbox_sdk import TBoxClient, TBoxSDKError

logging.basicConfig(
    level=os.environ.get("BENBEN_CHASSIS_LOG_LEVEL", "INFO"),
    format="[benben_chassis] %(message)s",
)
log = logging.getLogger("benben_chassis")

benben_chassis = Primitive(
    id="benben_chassis",
    namespace="robonix/primitive/chassis",
)

# ── Global TBoxClient (persistent authorization + heartbeat) ─────────────────
_client: "TBoxClient| None" = None          # TBoxClient
_client_lock = threading.Lock()          # guards _client read/write
_motion_lock = threading.Lock()          # serializes motion (move vs twist_in)
# _udp_lock = threading.Lock()             # serializes UDP sendto() calls
# ── ROS1 odometry source (TCPROS) ────────────────────────────────────────────
_ODOMETRY_MD5SUM = "cd5e73d190d741a2f92e81eda573aca7"
_ODOMETRY_TYPE = "nav_msgs/Odometry"
_ROS1_CALLER_ID = "/benben_chassis_odom_bridge"
_odom_lock = threading.Lock()
_latest_odom = None
_last_pub_seq: int = -1            # seq of last published odom (dup suppression)
_odom_receiver = None

# ── ROS 2 handles ───────────────────────────────────────────────────────────
_odom_pub = None     # rclpy publisher -> /odom
_odom_timer = None   # rclpy timer for periodic odom publishing
_tf_broadcaster = None  # tf2_ros TransformBroadcaster (odom -> base_link)
_clock = None        # ROS 2 clock (host time, set in on_init)
_odom_frame: str = "odom"       # overrides ROS1 frame_id
_base_frame: str = "base_link"  # overrides ROS1 child_frame_id

_udp_clinet: socket.socket| None = None
_udp_server_ip :str = None
_udp_server_port:int = None
# ── proto codegen (on PYTHONPATH via rbnx-build/codegen/proto_gen) ──────────
import chassis_pb2  # noqa: E402
import std_msgs_pb2  # noqa: E402


# ── ROS1 odometry receiver ──────────────────────────────────────────────────
def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("ROS1 odom connection closed by peer")
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
    fields = {}
    offset = 0
    while offset < len(body):
        (field_len,) = struct.unpack_from("<I", body, offset)
        offset += 4
        entry = body[offset:offset + field_len].decode(errors="replace")
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
    return data[offset:offset + length].decode(errors="replace"), offset + length


def _parse_ros1_odometry(data: bytes) -> dict:
    offset = 0
    (seq,) = struct.unpack_from("<I", data, offset)
    offset += 4  # ROS1 Header.seq has no ROS2 equivalent.
    sec, nsec = struct.unpack_from("<II", data, offset)
    offset += 8
    frame_id, offset = _parse_ros_string(data, offset)
    child_frame_id, offset = _parse_ros_string(data, offset)

    position_orientation = struct.unpack_from("<7d", data, offset)
    offset += 7 * 8
    pose_covariance = struct.unpack_from("<36d", data, offset)
    offset += 36 * 8
    twist = struct.unpack_from("<6d", data, offset)
    offset += 6 * 8
    twist_covariance = struct.unpack_from("<36d", data, offset)

    return {
        "seq": seq,
        "sec": sec,
        "nsec": nsec,
        "frame_id": frame_id,
        "child_frame_id": child_frame_id,
        "position_orientation": position_orientation,
        "pose_covariance": pose_covariance,
        "twist": twist,
        "twist_covariance": twist_covariance,
    }


def _connect_ros1_odom(publisher_uri: str, topic: str) -> socket.socket:
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
        "md5sum": _ODOMETRY_MD5SUM,
        "type": _ODOMETRY_TYPE,
        "tcp_nodelay": "1",
    }))
    response = _read_tcpros_header(sock)
    if "error" in response:
        sock.close()
        raise RuntimeError(f"ROS1 TCPROS handshake failed: {response['error']}")
    return sock


class _Ros1OdomReceiver(threading.Thread):
    """Keep the latest remote ROS1 /odom frame, reconnecting after failures."""

    def __init__(self, publisher_uri: str, topic: str) -> None:
        super().__init__(daemon=True, name="ros1-odom-receiver")
        self._publisher_uri = publisher_uri
        self._topic = topic
        self._stop_event = threading.Event()
        self._sock = None
        self._first_odom_event = threading.Event()

    def run(self) -> None:
        global _latest_odom
        while not self._stop_event.is_set():
            try:
                self._sock = _connect_ros1_odom(self._publisher_uri, self._topic)
                log.info("connected to ROS1 Odometry publisher %s on %s",
                         self._publisher_uri, self._topic)
                while not self._stop_event.is_set():
                    odom = _parse_ros1_odometry(_read_tcpros_message(self._sock))
                    with _odom_lock:
                        _latest_odom = odom
                    if not self._first_odom_event.is_set():
                        self._first_odom_event.set()
                        px, py, pz = odom["position_orientation"][:3]
                        log.info("first Odometry received: seq=%d frame=%s->%s "
                                 "pos=(%.3f,%.3f,%.3f)",
                                 odom["seq"], odom["frame_id"],
                                 odom["child_frame_id"], px, py, pz)
            except Exception as exc:  # noqa: BLE001
                if not self._stop_event.is_set():
                    log.warning("ROS1 odom receive failed: %s", exc)
                    self._stop_event.wait(1.0)
            finally:
                sock, self._sock = self._sock, None
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass

    def wait_for_first_odom(self, timeout: float) -> bool:
        return self._first_odom_event.wait(timeout)

    def stop(self) -> None:
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


# ── Odom publishing ─────────────────────────────────────────────────────────
def _publish_odom() -> None:
    """Publish the latest remote ROS1 nav_msgs/Odometry as ROS2 /odom.

    Duplicate suppression: skip publishing when the latest odom's seq
    matches the last published one, so the ROS 2 publish rate never
    exceeds the actual odometry frame rate even when the timer ticks faster.
    """
    global _last_pub_seq
    if _odom_pub is None:
        return
    try:
        from nav_msgs.msg import Odometry  # type: ignore

        with _odom_lock:
            odom = _latest_odom
        if odom is None:
            return
        if odom["seq"] == _last_pub_seq:
            return
        _last_pub_seq = odom["seq"]

        px, py, pz, ox, oy, oz, ow = odom["position_orientation"]
        lx, ly, lz, ax, ay, az = odom["twist"]
        msg = Odometry()
        # Stamp with ROS 2 host clock, not the remote ROS1 clock.
        if _clock is not None:
            stamp = _clock.now().to_msg()
            msg.header.stamp = stamp
        else:
            msg.header.stamp.sec = odom["sec"]
            msg.header.stamp.nanosec = odom["nsec"]
        msg.header.frame_id = _odom_frame
        msg.child_frame_id = _base_frame
        msg.pose.pose.position.x = px
        msg.pose.pose.position.y = py
        msg.pose.pose.position.z = pz
        msg.pose.pose.orientation.x = ox
        msg.pose.pose.orientation.y = oy
        msg.pose.pose.orientation.z = oz
        msg.pose.pose.orientation.w = ow
        msg.pose.covariance = list(odom["pose_covariance"])
        msg.twist.twist.linear.x = lx
        msg.twist.twist.linear.y = ly
        msg.twist.twist.linear.z = lz
        msg.twist.twist.angular.x = ax
        msg.twist.twist.angular.y = ay
        msg.twist.twist.angular.z = az
        msg.twist.covariance = list(odom["twist_covariance"])
        _odom_pub.publish(msg)

        # Broadcast odom -> base_link TF so consumers (rtabmap, nav2) have
        # a complete TF tree without needing a separate robot_state_publisher.
        if _tf_broadcaster is not None:
            from geometry_msgs.msg import TransformStamped  # type: ignore
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = _odom_frame
            tf.child_frame_id = _base_frame
            tf.transform.translation.x = px
            tf.transform.translation.y = py
            tf.transform.translation.z = pz
            tf.transform.rotation.x = ox
            tf.transform.rotation.y = oy
            tf.transform.rotation.z = oz
            tf.transform.rotation.w = ow
            _tf_broadcaster.sendTransform(tf)
    except Exception as exc:  # noqa: BLE001
        log.warning("odom publish failed: %s", exc)


def send_cmd(sock, host_ip, port, cmd):
    """
    cmd: list of 10 numbers
         [linear_x, linear_y, linear_z,
          angular_x, angular_y, angular_z,
          duration_sec, forward_m, rotate_deg,
          type]
          type = 1 is twist mode
          type = 0 is move mode 
    """
    if len(cmd) != 10:
        raise ValueError("需要10个数值")
    # 前9个float64，后1个int32
    data = struct.pack('<9di', *cmd)
    sock.sendto(data, (host_ip, port))
    # sock.close()
    # 示例：以0.5 m/s前进1米（type=0）
    # send_cmd('192.168.10.1', [0,0,0, 0,0,0, 0, 1.0, 0, 0])

    # 示例：原地顺时针旋转90度（type=0）
    # send_cmd('192.168.10.1', [0,0,0, 0,0,0, 0, 0, -360.0, 0])

# ── twist_in subscription callback ──────────────────────────────────────────
def _on_twist_in(msg) -> None:
    """Forward geometry_msgs/Twist from /cmd_vel to TBox remote_control.

    Uses non-blocking lock acquisition: if a ``move`` RPC burst is in
    progress the Twist is silently dropped rather than queueing up behind
    the motion lock.
    """
    linear_x = float(getattr(msg, "linear_x", 0.0))
    linear_y = float(getattr(msg, "linear_y", 0.0))
    linear_z = float(getattr(msg, "linear_z", 0.0))
    angular_x = float(getattr(msg, "angular_x", 0.0))
    angular_y = float(getattr(msg, "angular_y", 0.0))
    angular_z = float(getattr(msg, "angular_z", 0.0))
    data = [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, 0.0, 0,0, 0.0, 0]
    send_cmd(_udp_server_port, _udp_server_ip, _udp_server_port, data)


# ── gRPC RPC: `move` (no MCP - keep velocity primitive off the LLM tool list) ─
@benben_chassis.grpc("robonix/primitive/chassis/move")
def move(req: "chassis_pb2.ExecuteMoveCommand_Request") -> "chassis_pb2.ExecuteMoveCommand_Response":
    """Velocity-mode chassis command.  Service callers (simple_nav,
    nav2_wrapper, teleop) reach this via gRPC.  NOT exposed as MCP - the
    LLM should invoke ``service/navigation/navigate`` instead, which
    handles obstacle avoidance.

    Three modes by priority:
      1. command.forward_m != 0   drive straight by signed distance (m)
      2. command.rotate_deg != 0  in-place yaw rotation by signed degrees
      3. velocity mode (linear_x/angular_z used directly for duration_sec)

    ``forward_m`` / ``rotate_deg`` are approximate quantities - they use
    timed velocity bursts and do not provide odom-closed-loop precision.
    """
    with _client_lock:
        client = _client
    if client is None:
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(data=json.dumps({"error": "TBox client not initialized"})),
        )

    msg = req.command
    # 一下参数已由server端定义
    # speed_mps = float(os.environ.get("BENBEN_CHASSIS_SPEED_MPS", "0.4"))
    # ang_speed_rps = float(os.environ.get("BENBEN_CHASSIS_ANG_SPEED_RPS", "0.4"))
    # default_dur = float(os.environ.get("BENBEN_CHASSIS_CMD_DURATION_SEC", "1.0"))
    linear_x = float(getattr(msg, "linear_x", 0.0))
    linear_y = float(getattr(msg, "linear_y", 0.0))
    linear_z = float(getattr(msg, "linear_z", 0.0))
    angular_x = float(getattr(msg, "angular_x", 0.0))
    angular_y = float(getattr(msg, "angular_y", 0.0))
    angular_z = float(getattr(msg, "angular_z", 0.0))
    forward_m = float(getattr(msg, "forward_m", 0.0))
    rotate_deg = math.radians(float(getattr(msg, "rotate_deg", 0.0)))
    duration_sec = float(getattr(msg, "duration_sec", 0.0))
    data = [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, duration_sec, forward_m, rotate_deg, 0]

    mode = ""
    duration = 0.0
    # _client.brake_control(0) # pull brake to allow motion
    try:
        with _motion_lock:
            # Ensure chassis is in parallel-driving mode with brake released.

            try:
                if forward_m != 0.0:
                    # sign = 1.0 if forward_m > 0 else -1.0
                    # duration = abs(forward_m) / speed_mps
                    # deadline = time.monotonic() + duration
                    # while time.monotonic() < deadline:
                    #     client.remote_control(sign * speed_mps, 0.0)
                    #     remaining = deadline - time.monotonic()
                    #     if remaining > 0:
                    #         time.sleep(min(interval, remaining))
                    mode = "forward_m"
                    send_cmd(_udp_clinet, _udp_server_ip, _udp_server_port, data)
                    # tw_linear_x = ...
                    # tw_angular_z = 0.0

                elif rotate_deg != 0.0:

                    mode = "rotate_deg"
                    # tw_linear_x = 0.0
                    # tw_angular_z = sign * ang_speed_rps
                    client.rotation_control(1,rotate_deg,0.5)
                else:
                    # data[-1] = 1  # type=1 for twist mode
                    mode = "velocity" 
                    send_cmd(_udp_clinet, _udp_server_ip, _udp_server_port, data)
                    # duration = duration_sec if duration_sec > 0 else default_dur
                    # deadline = time.monotonic() + duration
                    # while time.monotonic() < deadline:
                    #     client.remote_control(linear_x, angular_z)
                    #     remaining = deadline - time.monotonic()
                    #     if remaining > 0:
                    #         time.sleep(min(interval, remaining))
                   
                
            finally:
                # Always stop after a burst.
                client.stop()
                client.stop()
                client.stop()
    except Exception as exc:  # noqa: BLE001
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(data=json.dumps({"error": str(exc)})),
        )

    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps({
            "status": "done", "mode": mode,
            "forward_m": forward_m, "rotate_deg": rotate_deg,
            "duration_sec": duration,
            "linear_x": linear_x, "angular_z": angular_z,
            "defalut_linear_x": 0.5, "deflaut_angular_z": 0.5
        })),
    )



# ── lifecycle ────────────────────────────────────────────────────────────────
@benben_chassis.on_init
def init(cfg):
    global _client, _odom_pub, _odom_timer, _odom_receiver, _last_pub_seq, _tf_broadcaster, _clock, _odom_frame, _base_frame, _udp_clinet,_udp_server_ip,_udp_server_port

    # ── Config ──
    token = cfg.get("token") or os.environ.get("BENBEN_TBOX_TOKEN", "")
    if not token:
        return Deferred("BENBEN_TBOX_TOKEN not set: cannot authenticate to TBox")
    odom_topic = cfg.get("odom_topic") or os.environ.get("BENBEN_ODOM_TOPIC", "/odom")
    twist_in_topic = cfg.get("twist_in_topic") or os.environ.get("BENBEN_CMD_VEL_TOPIC", "/cmd_vel")
    _odom_frame = str(cfg.get("odom_frame") or os.environ.get("BENBEN_ODOM_FRAME", "odom"))
    _base_frame = str(cfg.get("base_frame") or os.environ.get("BENBEN_BASE_FRAME", "base_link"))
    odom_hz = float(cfg.get("odom_hz") or os.environ.get("BENBEN_ODOM_HZ", "50"))
    _udp_server_ip = (
        cfg.get("ros1_target_ip")
        or os.environ.get("BENBEN_ROS1_CMD_VEL_PUBLISHER_URI")
        or "192.168.10.1"
    )
    _udp_server_port = int(cfg.get("ros1_target_port") or os.environ.get("BENBEN_ROS1_CMD_VEL_SOCKET_PORT") or "11451")

    ros1_odom_uri = (
        cfg.get("ros1_odom_publisher_uri")
        or os.environ.get("BENBEN_ROS1_ODOM_PUBLISHER_URI")
        or "http://192.168.10.1:32769/"
    )
    ros1_odom_topic = (
        cfg.get("ros1_odom_topic")
        or os.environ.get("BENBEN_ROS1_ODOM_TOPIC")
        or "/odom"
    )
    sentinel_timeout = float(cfg.get("odom_sentinel_timeout_s", 15.0))
    if odom_hz <= 0:
        return Err("odom_hz must be greater than zero")

    # ── Initialize TBox client ──
    # The SDK manages connection, authorization, and heartbeat internally.
    # A single global client persists auth state across all commands.
    client = TBoxClient()
    client.initialize(token)
    if not client.wait_until_ready(timeout=30.0):
        return Deferred("TBox auth not ready after 30 s")
    with _client_lock:
        _client = client
    # udp socket publisher
    _udp_clinet =  socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # ── ROS1 TCPROS odom receiver ──
    _odom_receiver = _Ros1OdomReceiver(ros1_odom_uri, ros1_odom_topic)
    _odom_receiver.start()

    if not _odom_receiver.wait_for_first_odom(sentinel_timeout):
        _odom_receiver.stop()
        _odom_receiver.join(timeout=2.0)
        _odom_receiver = None
        return Err(
            f"no Odometry received from {ros1_odom_uri} on {ros1_odom_topic} "
            f"within {sentinel_timeout:.1f}s"
        )

    # ── ROS 2: odom publisher + timer ──
    from nav_msgs.msg import Odometry  # type: ignore
    _odom_pub = benben_chassis.create_publisher(
        "robonix/primitive/chassis/odom",
        topic=odom_topic, msg_type=Odometry, qos="reliable",
    )
    _last_pub_seq = -1
    # client.brake_control(1)
    from robonix_api.ros import RosBackend
    _clock = RosBackend.get().node.get_clock()
    from tf2_ros import TransformBroadcaster  # type: ignore
    _tf_broadcaster = TransformBroadcaster(RosBackend.get().node)
    period = 1.0 / odom_hz
    _odom_timer = RosBackend.get().node.create_timer(period, _publish_odom)

    # ── ROS 2: twist_in subscription ──
    from geometry_msgs.msg import Twist  # type: ignore
    benben_chassis.create_subscription(
        "robonix/primitive/chassis/twist_in",
        topic=twist_in_topic, msg_type=Twist,
        callback=_on_twist_in, qos="reliable",
    )

    log.info(
        "init complete: ROS1 %s%s -> ROS2 %s @ %.1f Hz",
        ros1_odom_uri, ros1_odom_topic, odom_topic, odom_hz,
    )
    return Ok()


@benben_chassis.on_shutdown
def shutdown():
    global _client, _odom_pub, _odom_timer, _odom_receiver, _latest_odom, _last_pub_seq, _tf_broadcaster, _clock

    # Stop chassis and pull brake.
    with _client_lock:
        client = _client
        _client = None
    if client is not None:
        try:
            client.stop()
        except Exception:  # noqa: BLE001
            pass
        # try:
        #     from tbox_sdk.constants import BrakeControl
            
        #     # client.brake_control(BrakeControl.PULL)
        # except Exception:  # noqa: BLE001
        #     pass

    # Stop the ROS1 TCPROS receiver before dropping ROS2 handles.
    if _odom_receiver is not None:
        _odom_receiver.stop()
        _odom_receiver.join(timeout=2.0)
        _odom_receiver = None
    with _odom_lock:
        _latest_odom = None
    _last_pub_seq = -1

    # Cancel odom timer.
    if _odom_timer is not None:
        try:
            _odom_timer.cancel()
        except Exception:  # noqa: BLE001
            pass
        _odom_timer = None
    #close udp clinet
    _udp_clinet.close()
    _udp_clinet = None
    _odom_pub = None
    _tf_broadcaster = None
    _clock = None
    return Ok()


if __name__ == "__main__":
    benben_chassis.run()


# ── MCP RPC: `move` (exposed to LLM agent for direct velocity commands) ──────
# The gRPC handler above is for service-to-service calls (future nav2 wrapper).
# This MCP handler lets the pilot agent invoke chassis/move directly when no
# navigation service is deployed.  Both share the same _motion_lock so they
# are mutually exclusive.
@benben_chassis.mcp("robonix/primitive/chassis/move")
def move_mcp(req: "chassis_mcp.ExecuteMoveCommand_Request") -> "chassis_mcp.ExecuteMoveCommand_Response":
    """Chassis velocity command (agent-callable).

    Three modes by priority:
      1. forward_m != 0   - drive straight by signed distance (meters)
      2. rotate_deg != 0  - in-place yaw rotation by signed degrees
      3. velocity mode    - linear_x / angular_z for duration_sec

    Examples:
      Rotate right 90°:  {"command": {"rotate_deg": -90}}
      Drive forward 0.5m: {"command": {"forward_m": 0.5}}
      Turn slowly left:   {"command": {"linear_x": 0, "angular_z": 0.3, "duration_sec": 2}}
    """
    import chassis_mcp
    import std_msgs_mcp

    with _client_lock:
        client = _client
    if client is None:
        return chassis_mcp.ExecuteMoveCommand_Response(
            status=std_msgs_mcp.String(data=json.dumps({"error": "TBox client not initialized"})),
        )

    msg = req.command
    speed_mps = float(os.environ.get("BENBEN_CHASSIS_SPEED_MPS", "0.2"))
    ang_speed_rps = float(os.environ.get("BENBEN_CHASSIS_ANG_SPEED_RPS", "0.4"))
    default_dur = float(os.environ.get("BENBEN_CHASSIS_CMD_DURATION_SEC", "1.0"))

    forward_m = float(msg.forward_m)
    rotate_deg = float(msg.rotate_deg)
    duration_sec = float(msg.duration_sec)

    interval = 0.1
    mode = ""
    tw_linear_x = 0.0
    tw_angular_z = 0.0
    duration = 0.0

    try:
        with _motion_lock:
            try:
                if forward_m != 0.0:
                    sign = 1.0 if forward_m > 0 else -1.0
                    duration = abs(forward_m) / speed_mps
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        client.remote_control(sign * speed_mps, 0.0)
                        remaining = deadline - time.monotonic()
                        if remaining > 0:
                            time.sleep(min(interval, remaining))
                    mode = "forward_m"
                    tw_linear_x = sign * speed_mps

                elif rotate_deg != 0.0:
                    rad = math.radians(rotate_deg)
                    sign = 1.0 if rad > 0 else -1.0
                    duration = abs(rad) / ang_speed_rps
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        client.remote_control(0.0, sign * ang_speed_rps)
                        remaining = deadline - time.monotonic()
                        if remaining > 0:
                            time.sleep(min(interval, remaining))
                    mode = "rotate_deg"
                    tw_angular_z = sign * ang_speed_rps

                else:
                    linear_x = max(-1.0, min(1.0, float(msg.linear_x)))
                    angular_z = max(-1.0, min(1.0, float(msg.angular_z)))
                    duration = duration_sec if duration_sec > 0 else default_dur
                    deadline = time.monotonic() + duration
                    while time.monotonic() < deadline:
                        client.remote_control(linear_x, angular_z)
                        remaining = deadline - time.monotonic()
                        if remaining > 0:
                            time.sleep(min(interval, remaining))
                    mode = "velocity"
                    tw_linear_x = linear_x
                    tw_angular_z = angular_z
            finally:
                client.stop()
                client.stop()
                client.stop()
    except Exception as exc:  # noqa: BLE001
        return chassis_mcp.ExecuteMoveCommand_Response(
            status=std_msgs_mcp.String(data=json.dumps({"error": str(exc)})),
        )

    return chassis_mcp.ExecuteMoveCommand_Response(
        status=std_msgs_mcp.String(data=json.dumps({
            "status": "done", "mode": mode,
            "forward_m": forward_m, "rotate_deg": rotate_deg,
            "duration_sec": duration,
            "linear_x": tw_linear_x, "angular_z": tw_angular_z,
        })),
    )


# ── MCP RPC: `odom_snapshot` (one-shot Odometry capture for agent) ───────────
# Mirrors the lidar driver's ``snapshot`` MCP: lets the pilot agent query the
# current chassis pose / velocity in a single call without subscribing to the
# /odom topic.  Returns a JSON string in std_msgs/String for compatibility with
# the existing chassis MCP codegen (avoids depending on nav_msgs_mcp).
@benben_chassis.mcp("robonix/primitive/chassis/odom_snapshot")
def odom_snapshot(req: "std_msgs_mcp.Empty") -> "std_msgs_mcp.String":
    """Get the current chassis odometry.

    Returns a JSON string (std_msgs/String) with:
      seq, stamp, frame_id, child_frame_id,
      position {x, y, z}, orientation {x, y, z, w},
      linear_velocity {x, y, z}, angular_velocity {x, y, z}.
    Useful for "where am I?" / "how fast am I moving?".
    Contract: robonix/primitive/chassis/odom_snapshot.
    """
    import std_msgs_mcp

    _ = req
    with _odom_lock:
        odom = _latest_odom
    if odom is None:
        raise RuntimeError("no Odometry received yet")
    px, py, pz, ox, oy, oz, ow = odom["position_orientation"]
    lx, ly, lz, ax, ay, az = odom["twist"]
    return std_msgs_mcp.String(data=json.dumps({
        "seq": odom["seq"],
        "stamp": odom["sec"] + odom["nsec"] * 1e-9,
        "frame_id": odom["frame_id"],
        "child_frame_id": odom["child_frame_id"],
        "position": {"x": px, "y": py, "z": pz},
        "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
        "linear_velocity": {"x": lx, "y": ly, "z": lz},
        "angular_velocity": {"x": ax, "y": ay, "z": az},
    }))
