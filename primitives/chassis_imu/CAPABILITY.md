---
description: Chassis 6-axis IMU - orientation, angular velocity, and linear acceleration via ROS1 TCPROS bridge.
---

# Chassis IMU (`robonix/primitive/imu`)

The BenBen chassis controller's built-in 6-axis IMU. It publishes
`sensor_msgs/Imu` on a remote ROS1 node (`/imu_data`). This driver bridges
that stream to ROS 2 via direct TCPROS subscription - no ROS 1 environment
is required on the host.

## Tools

### `snapshot` - `robonix/primitive/imu/snapshot`
- input: none
- returns: `sensor_msgs/Imu` JSON with `orientation` (quaternion x/y/z/w),
  `angular_velocity` (rad/s), `linear_acceleration` (m/s^2), and the three
  3x3 covariance matrices.
- use cases:
  - "am I tilted?" -> read `orientation` (roll/pitch), or the sign of
    `linear_acceleration.z` against gravity.
  - "how fast am I turning?" -> `angular_velocity.z` (yaw rate for a
    differential-drive base).
- DO NOT use the IMU alone to estimate position; it drifts without an
  odometry or SLAM correction.

## Topic: `imu` (`robonix/primitive/imu/imu`)

Continuous `sensor_msgs/Imu` stream republished on ROS 2 `/imu` from the
remote ROS1 `/imu_data` publisher. The ROS1 TCPROS receiver runs in a
background thread; a ROS 2 timer republishes each new frame on the host DDS
bus.

## Reasoning

The chassis IMU is a fast, odometry-independent attitude/turn sensor. Use it
for tilt sanity checks and to cross-check yaw rate against wheel odometry;
prefer the mapping/odom services for position, since raw IMU integration
drifts.
