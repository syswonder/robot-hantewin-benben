# Runtime config accepted by the LakiBeam1 lidar primitive.
#
# This file documents the mapping passed as the package's `config:` value in a
# robot deployment manifest. It is not loaded by the provider. Values below are
# runtime defaults unless a field is marked as required or optional.

config:
  # string, default: /scan (env: LAKIBEAM1_SCAN_TOPIC).
  # ROS 2 topic on which sensor_msgs/LaserScan is published. Downstream
  # consumers (mapping, navigation, obstacle check) subscribe to this topic.
  scan_topic: /scan

  # string, default: laser (env: LAKIBEAM1_FRAME_ID).
  # ROS 2 frame_id written into LaserScan headers. This overrides the
  # frame_id from the remote ROS1 node so the published scan matches the
  # TF tree published by the static_transform_publisher below.
  frame_id: laser

  # string, default: base_link (env: LAKIBEAM1_PARENT_FRAME).
  # Parent frame for the lidar mount pose. Used only when extrinsics
  # is present.
  parent_frame: base_link

  # mapping, optional; no runtime default.
  # Static pose of frame_id in parent_frame. Translation is in metres and
  # rotation is roll/pitch/yaw in radians. When present, the driver spawns
  # a static_transform_publisher (parent_frame -> frame_id) so consumers
  # (mapping, navigation) see a complete TF tree. Omit when a chassis
  # driver / soma URDF already publishes the same edge.
  # extrinsics:
  #   x: 0.0
  #   y: 0.0
  #   z: 0.0
  #   roll: 0.0
  #   pitch: 0.0
  #   yaw: 0.0

  # float (Hz), default: 10 (env: LAKIBEAM1_SCAN_HZ).
  # ROS 2 publish timer rate. Should be >= the LakiBeam1's native scan rate
  # to avoid introducing latency. Duplicate scans are suppressed automatically
  # so setting this higher than the actual lidar rate is harmless.
  scan_hz: 10.0

  # string, default: http://192.168.10.1:39233/ (env: LAKIBEAM1_ROS1_PUBLISHER_URI).
  # XML-RPC URI of the remote ROS1 /richbeam_lidar node that publishes
  # /scan_filter. The driver uses this to negotiate a TCPROS connection.
  ros1_publisher_uri: "http://192.168.10.1:39233/"

  # string, default: /scan_filter (env: LAKIBEAM1_ROS1_TOPIC).
  # ROS 1 topic name to subscribe to on the remote publisher.
  ros1_topic: /scan_filter

  # float (seconds), default: 15.0.
  # Maximum wait for the first LaserScan during startup. If no scan arrives
  # within this period the provider reports ERROR.
  sentinel_timeout_s: 15.0
