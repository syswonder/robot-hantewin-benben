# Runtime config accepted by the BenBen chassis primitive.
#
# This file documents the mapping passed as the package's `config:` value in a
# robot deployment manifest. It is not loaded by the provider. Values below are
# runtime defaults unless a field is marked as required or optional.

config:
  # string, optional unless BENBEN_TBOX_TOKEN is set.
  # Authentication token for the TBox SDK. The driver initializes a global
  # TBoxClient with this token; the SDK handles the TCP connection,
  # authorization handshake, and heartbeat internally.
  token: ""

  # string, default: /odom (env: BENBEN_ODOM_TOPIC).
  # ROS 2 topic on which nav_msgs/Odometry is published. The downstream
  # localization service (mapping) subscribes to this topic.
  odom_topic: /odom

  # string, default: /cmd_vel (env: BENBEN_CMD_VEL_TOPIC).
  # ROS 2 topic from which geometry_msgs/Twist velocity commands are consumed.
  # The navigation stack publishes to this topic; the driver forwards each
  # message to the TBox SDK remote_control() function.
  twist_in_topic: /cmd_vel
