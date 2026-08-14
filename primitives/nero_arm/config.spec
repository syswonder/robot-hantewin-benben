config:
  # string, default: can0
  # SocketCAN channel used by pyAgxArm. The interface must exist and be UP
  # before CMD_INIT.
  can_channel: can0

  # string, default: socketcan
  # CAN backend passed to pyAgxArm create_agx_arm_config().
  can_interface: socketcan

  # string, default: default
  # Nero firmware selector. Supported values: default, v111, v112, v120.
  firmware_version: default

  # int, percent, default: 50; range: 1..100
  # Motion speed percent applied after connect.
  speed_percent: 50

  # bool, default: false
  # When false, joints are enabled on the first joint/pos command.
  enable_on_init: false

  # list[float], optional, length 7
  # After init (and enable), move the arm to this joint pose (radians).
  # Requires enable_on_init: true, or joints are auto-enabled for the move.
  startup_joint_pose: null

  # float, seconds, default: connect_timeout_s
  # Timeout for the startup joint move when startup_joint_pose is set.
  startup_motion_timeout_s: 30.0

  # float, seconds, default: 30.0; must be > 0
  # Maximum wait for joint enable during init when enable_on_init is true.
  connect_timeout_s: 30.0

  # string, default: ""
  # Prefix added to URDF/Soma joint names, e.g. left_ -> left_joint1..7.
  joint_name_prefix: ""

  # string, default: <prefix>base_link
  # Reference frame documented for end_pose and pos_command.
  reference_frame: base_link

  # float, Hz, default: 50.0
  publish_rate_hz: 50.0

  # string, default: /joint_states
  # Additional JointState topic used to drive robot_state_publisher TF.
  tf_joint_states_topic: /joint_states

  # list[float], optional, length 6
  # TCP offset [x, y, z, roll, pitch, yaw] in meters/radians relative to flange.
  tcp_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  # string ROS topic overrides; defaults derive from the manifest instance name.
  joint_states_topic: /left_arm/joint_states
  joint_command_topic: /left_arm/joint_command
  pos_command_topic: /left_arm/pos_command
  end_pose_topic: /left_arm/end_pose
