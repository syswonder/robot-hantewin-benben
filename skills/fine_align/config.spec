# fine_align deployment config (robonix_manifest.yaml `skill: fine_align: config:`).

# ── 上游依赖（Atlas 解析，无需配置） ──────────────────────────────────────
# detect: robonix/skill/nero_grasp/detect   (MCP)  -> object robot_xyz
# move:   robonix/primitive/chassis/move    (MCP)  -> forward_m / rotate_deg

# ── 几何（TODO 待实测，占位值） ──────────────────────────────────────────
graspable_zone:
  center: [0.0, 0.0, 0.0]      # robot_base 系下「理想抓取点」，待测
  tolerance: [0.05, 0.05, 0.03] # 各轴容差 (m)

base_to_robot_rotation:          # base_link -> robot_base 旋转 (3x3)，待测
  - [1.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0]
  - [0.0, 0.0, 1.0]
base_to_robot_translation: [0.0, 0.0, 0.0]  # 仅记录，误差规划用不到平移

# ── ROS ───────────────────────────────────────────────────────────────────
odom_topic: /odom               # 闭环反馈 odom 话题

# ── 控制参数 ─────────────────────────────────────────────────────────────
max_iterations: 8
timeout_s: 60.0
forward_gain: 1.0
rotate_gain: 1.0
forward_step_max_m: 0.30
rotate_step_max_deg: 30.0
forward_tol_m: 0.02
lateral_tol_m: 0.02
move_settle_s: 0.5
odom_tolerance_m: 0.02
odom_yaw_tolerance_deg: 3.0
