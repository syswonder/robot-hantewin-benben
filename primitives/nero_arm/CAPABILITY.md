---
description: "AgileX Nero 7-DOF arm joint/Cartesian control via pyAgxArm."
---

# AgileX Nero arm primitive

This package exposes one Nero arm as a Robonix `robonix/primitive/arm/*` provider.

## Reference frame

`end_pose` and `pos_command` use the arm base link frame configured by
`reference_frame` (default `<prefix>base_link`). Positions are meters; orientations
are unit quaternions in `geometry_msgs/Pose`.

## Joint command support

`joint_command` accepts `sensor_msgs/JointState` with either:

- seven unnamed `position` values in joint order, or
- named joints matching `joint_name_prefix` + `joint1..joint7`.

Only `position` is consumed; velocity and effort are ignored.

## Safety

`enable_on_init` defaults to `false`. The first command enables all joints.
`CMD_SHUTDOWN` triggers electronic emergency stop and disconnect.

## Dual-arm deployment

Run one manifest instance per CAN-connected arm (`left_arm`, `right_arm`) with
distinct `can_channel`, `joint_name_prefix`, and `reference_frame`. Both instances
may publish partial updates to `/joint_states` for `robot_state_publisher`.
