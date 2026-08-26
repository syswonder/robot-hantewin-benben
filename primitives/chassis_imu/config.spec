# Runtime config accepted by the BenBen chassis IMU primitive.
#
# This file documents the mapping passed as the package's `config:` value in a
# robot deployment manifest. It is not loaded by the provider. Values below are
# runtime defaults unless a field is marked as required or optional.

config:
  # string, default: /imu (env: CHASSIS_IMU_TOPIC).
  # ROS 2 topic on which sensor_msgs/Imu is published. Downstream consumers
  # (mapping, LIO, localization) subscribe to this topic.
  imu_topic: /imu

  # string, default: imu_link (env: CHASSIS_IMU_FRAME_ID).
  # ROS 2 frame_id written into Imu headers. This overrides the frame_id from
  # the remote ROS1 node so the published Imu matches the TF tree published by
  # the static_transform_publisher below.
  frame_id: imu_link

  # string, default: base_link (env: CHASSIS_IMU_PARENT_FRAME).
  # Parent frame for the IMU mount pose. Used only when extrinsics is present.
  parent_frame: base_link

  # mapping, optional; no runtime default.
  # Static pose of frame_id in parent_frame. Translation is in metres and
  # rotation is roll/pitch/yaw in radians. When present, the driver spawns
  # a static_transform_publisher (parent_frame -> frame_id) so consumers
  # (mapping, LIO) see a complete TF tree. Omit when a chassis driver / soma
  # URDF already publishes the same edge.
  # extrinsics:
  #   x: 0.0
  #   y: 0.0
  #   z: 0.0
  #   roll: 0.0
  #   pitch: 0.0
  #   yaw: 0.0

  # float (Hz), default: 100.0 (env: CHASSIS_IMU_HZ).
  # ROS 2 publish timer rate. Should be >= the chassis controller's native IMU
  # rate to avoid introducing latency. Duplicate frames are suppressed
  # automatically so setting this higher than the actual IMU rate is harmless.
  imu_hz: 100.0

  # string, default: 192.168.10.1 (env: CHASSIS_IMU_ROS1_TARGET_IP).
  # Host of the ROS1 master and IMU publisher node. The driver queries the ROS
  # master (getSystemState / lookupNode) to discover the publisher's TCPROS
  # port, then negotiates a subscription directly with that node.
  ros1_target_ip: 192.168.10.1

  # string, default: /imu_data (env: CHASSIS_IMU_ROS1_TOPIC).
  # ROS 1 topic name to subscribe to on the remote publisher.
  ros1_imu_topic: /imu_data

  # float (seconds), default: 15.0.
  # Maximum wait for the first Imu frame during startup. If no frame arrives
  # within this period the provider reports ERROR.
  sentinel_timeout_s: 15.0
