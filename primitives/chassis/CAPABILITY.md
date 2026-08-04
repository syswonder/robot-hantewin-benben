---
description: Mobile differential-drive base — all movement during navigation, exploration, and search via TBox SDK.
---

## Identity

`robonix/primitive/chassis` — the robot's mobile base, driven through the TBox
C++ SDK (`libtbox_sdk_cpp.so`). All interactive exploration, search, and
wandering flows through this capability.

## Relation to localization

For "where is the robot" queries, use `service/map/pose` (SLAM-corrected,
map-frame). The `robonix/primitive/chassis/odom` topic provides raw odometry in
the local odom frame — it is a localization *input*, not the final localization
result.

## Tool: `cmd` (`robonix/primitive/chassis/move`)

Accepts `linear_x`, `linear_y`, `linear_z`, `angular_x`, `angular_y`,
`angular_z` (all default to 0), plus `forward_m` and `rotate_deg`.

**Three exclusive modes, evaluated in priority order:**

1. **`forward_m != 0`** — drive straight by signed distance (meters).
   Speed is controlled by `BENBEN_CHASSIS_SPEED_MPS` (default 0.2 m/s).
2. **`rotate_deg != 0`** — in-place yaw rotation by signed degrees.
   Angular speed is controlled by `BENBEN_CHASSIS_ANG_SPEED_RPS` (default 0.4 rad/s).
3. **Velocity mode** (fallback) — uses `linear_x` / `angular_z` directly.
   Duration defaults to `BENBEN_CHASSIS_CMD_DURATION_SEC` (default 1.0 s).

After each burst the driver sends a zero-velocity stop command. Returns
JSON ack `{status, mode, linear_x, angular_z, duration_sec, ...}`.

**Important**: `forward_m` and `rotate_deg` are approximate quantities —
they use timed velocity bursts and do not provide odom-closed-loop
precision. For accurate goal-reaching with obstacle avoidance, use
`service/navigation/navigate`.

## Burst pattern for visual exploration

```
snapshot → reason → single `cmd` burst → snapshot → reason → ...
```

Recommended magnitudes for exploration:
- Forward: `linear_x ≈ 0.10–0.20`
- Turn: `angular_z ≈ ±0.4`

## Topic: `twist_in` (`robonix/primitive/chassis/twist_in`)

Continuous velocity stream (geometry_msgs/Twist). The navigation stack
publishes to this topic; the driver forwards each message to the TBox
`remote_control()` function. Not intended for direct LLM use.

## Topic: `odom` (`robonix/primitive/chassis/odom`)

nav_msgs/Odometry republished at 50 Hz from the remote ROS1 `/odom` publisher
(`http://192.168.10.1:32769/`) through a direct TCPROS subscription. The ROS1
header, pose, twist, and both covariance matrices are preserved in the ROS2
message.
