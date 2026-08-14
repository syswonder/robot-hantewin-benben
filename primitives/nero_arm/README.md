# nero_arm

AgileX Nero 7-DOF 机械臂的 Robonix primitive 驱动。通过 `pyAgxArm` 经 CAN 控制单臂，对外暴露标准 `robonix/primitive/arm/*` 能力（关节/Cartesian 指令、状态反馈、夹爪、生命周期）。

能力契约摘要见 [`CAPABILITY.md`](./CAPABILITY.md)。

---

## 运行环境与依赖

### 系统与硬件

| 项目 | 要求 |
|------|------|
| 机械臂 | AgileX Nero（7 关节） |
| CAN | Linux SocketCAN，接口在 `CMD_INIT` 前必须已创建且 `UP` |
| ROS 2 | Jazzy（`scripts/start.sh` 默认 `ROS_DISTRO=jazzy`） |
| Robonix CLI | `rbnx`（build / boot / codegen） |

### Python 包

| 包 | 用途 |
|----|------|
| `pyAgxArm` | 机械臂底层 SDK（CAN 通信、运动、状态） |
| `rclpy` + Robonix ROS 2 IDL | 由 `scripts/build.sh` 中 `colcon build` 生成并 source |
| `robonix-api` | Robonix provider 运行时（`rbnx path robonix-api`） |

`pyAgxArm` 需按 AgileX 官方方式安装到当前 Python 环境；本 primitive 的 `package_manifest.yaml` 未声明 pip 依赖，部署前请自行确认：

```bash
python3 -c "from pyAgxArm import AgxArmFactory; print('pyAgxArm OK')"
```

### CAN 接口示例

```bash
# 将 can0 设为 1Mbps 并拉起（具体比特率以硬件为准）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip link show can0
```

manifest 中 `can_channel` 需与上述接口名一致（例如 `can_piper`、`can0`）。

---

## 构建与部署

在 Robonix 项目根目录或本包目录执行：

```bash
# 生成 MCP / ROS2 绑定并编译 IDL
rbnx build primitives/nero_arm

# 或通过 manifest 启动整栈
rbnx boot
```

构建脚本 [`scripts/build.sh`](./scripts/build.sh) 会：

1. 运行 `rbnx codegen -p <pkg> --mcp --ros2`
2. 在 `rbnx-build/codegen/ros2_idl` 下 `colcon build`

启动脚本 [`scripts/start.sh`](./scripts/start.sh) 会 source ROS 2 与 IDL overlay，然后执行 `python3 -m nero_arm.main`。

修改 primitive 代码或 capability 定义后需重新 `rbnx build`，再 `rbnx boot`（或重启对应 primitive 进程）。

---

## 配置项（config）

在 `robonix_manifest.yaml` 的 primitive 实例下填写 `config`。完整说明见 [`config.spec`](./config.spec)。

### CAN 与固件

| 键 | 类型 | 默认 | 说明 |
|----|------|------|------|
| `can_channel` | string | `can0` | SocketCAN 通道名，init 前必须可用 |
| `can_interface` | string | `socketcan` | 传给 `pyAgxArm.create_agx_arm_config()` |
| `firmware_version` | string | `default` | 固件：`default` / `v111` / `v112` / `v120`（也支持 `1.10` 等别名） |
| `speed_percent` | int | `50` | 连接后设置的运动速度百分比，范围 1–100 |

### 上电与启动位姿

| 键 | 类型 | 默认 | 说明 |
|----|------|------|------|
| `enable_on_init` | bool | `false` | `true` 时在 init 阶段使能全部关节并等待 `arm_status=normal` |
| `connect_timeout_s` | float | `30.0` | `enable_on_init=true` 时等待使能/恢复的超时（秒） |
| `startup_joint_pose` | list[7] | — | 可选，init 完成后 `move_j` 到此关节角（弧度） |
| `startup_motion_timeout_s` | float | 同 `connect_timeout_s` | 启动关节运动超时 |

**建议**：生产环境通常设 `enable_on_init: true` 并配置安全的 `startup_joint_pose`，避免上电后臂处于任意姿态。

当 `enable_on_init: false` 时，**第一条** `joint_command` / `pos_command` / `linear_pos_command` / `gripper_command` 会自动触发使能。

### 坐标系与关节命名

| 键 | 类型 | 默认 | 说明 |
|----|------|------|------|
| `joint_name_prefix` | string | `""` | 关节名前缀，如 `left_` → `left_joint1`…`left_joint7` |
| `reference_frame` | string | `<prefix>base_link` | `end_pose` / `pos_command` 文档化用的基座坐标系 |
| `tcp_offset` | list[6] | 全 0 | 相对法兰的 TCP 偏置 `[x,y,z,roll,pitch,yaw]`（米/弧度） |

### 末端执行器

| 键 | 类型 | 默认 | 说明 |
|----|------|------|------|
| `effector` | string | `""` | 非空时在 connect 后初始化夹爪。支持：`agx_gripper`、`revo2`、`revo2_touch` |

未配置 `effector` 时，不会注册 `gripper_command` topic。

### 状态发布

| 键 | 类型 | 默认 | 说明 |
|----|------|------|------|
| `publish_rate_hz` | float | `50.0` | `joint_states` / `end_pose` 发布频率 |
| `tf_joint_states_topic` | string | `/joint_states` | 额外发布 JointState 供 `robot_state_publisher`；可与 `joint_states_topic` 相同 |

### ROS Topic 覆盖

未指定时，topic 默认以 manifest **实例名**为前缀（如实例 `left_arm` → `/left_arm/joint_states`）。

| 键 | 默认示例 |
|----|----------|
| `joint_states_topic` | `/<instance>/joint_states` |
| `joint_command_topic` | `/<instance>/joint_command` |
| `pos_command_topic` | `/<instance>/pos_command` |
| `linear_pos_command_topic` | `/<instance>/linear_pos_command` |
| `end_pose_topic` | `/<instance>/end_pose` |
| `gripper_command_topic` | `/<instance>/gripper_command` |

### 配置示例

```yaml
# robonix_manifest.yaml 片段
primitive:
  - name: left_arm
    path: ./primitives/nero_arm
    config:
      can_channel: can_piper
      can_interface: socketcan
      firmware_version: v120
      speed_percent: 50
      enable_on_init: true
      startup_joint_pose: [-0.05, 1.47, -0.04, 0.06, -0.00, -0.20, 0.04]
      effector: agx_gripper
      joint_name_prefix: left_
      reference_frame: left_base_link
      joint_states_topic: /left_arm/joint_states
      joint_command_topic: /left_arm/joint_command
      pos_command_topic: /left_arm/pos_command
      linear_pos_command_topic: /left_arm/linear_pos_command
      end_pose_topic: /left_arm/end_pose
      gripper_command_topic: /left_arm/gripper_command
      tf_joint_states_topic: /joint_states
```

---

## 通信接口

### Robonix 能力契约

| Contract ID | 模式 | 消息类型 | 说明 |
|-------------|------|----------|------|
| `robonix/primitive/arm/driver` | RPC | `lifecycle/srv/Driver.srv` | 生命周期：`CMD_INIT` / `CMD_SHUTDOWN` |
| `robonix/primitive/arm/joint_states` | topic_out | `sensor_msgs/JointState` | 连续关节角反馈 |
| `robonix/primitive/arm/end_pose` | topic_out | `geometry_msgs/Pose` | 连续 TCP 位姿（`reference_frame` 下） |
| `robonix/primitive/arm/joint_command` | topic_in | `sensor_msgs/JointState` | 关节空间目标 |
| `robonix/primitive/arm/pos_command` | topic_in | `geometry_msgs/Pose` | Cartesian 目标（内部 IK，`move_p`） |
| `robonix/primitive/arm/linear_pos_command` | topic_in | `geometry_msgs/Pose` | 直线 Cartesian 目标（`move_l`） |
| `robonix/primitive/arm/gripper_command` | topic_in | `std_msgs/Float64MultiArray` | 夹爪：`data[0]`=开口宽度(m)，`data[1]`=力(N，可选，默认 1.0) |

Capability 定义文件位于 [`capabilities/`](./capabilities/)。

### joint_command 格式

- **方式 A**：7 个无名 `position` 值，顺序为 joint1…joint7（弧度）
- **方式 B**：带名关节，名称须匹配 `{joint_name_prefix}joint1` … `{joint_name_prefix}joint7`

仅使用 `position` 字段；`velocity` / `effort` 被忽略。

### pos_command / linear_pos_command

- 消息为 `geometry_msgs/Pose`：位置单位米，姿态为单位四元数 `(x,y,z,w)`
- 内部转换为 `[x,y,z,roll,pitch,yaw]` 后调用 `move_p` / `move_l`
- 运动完成判定：关节角误差 ≤ 0.05 rad 或 TCP 位置误差 ≤ 5 mm（见 `arm_driver.py`）

### 参考坐标系与 TCP 轴约定

`end_pose` 与 Cartesian 指令均在配置的 `reference_frame`（基座 link）下表达。

Nero **TCP 数组**（与 `grasp_test` / skill 层一致）：

| 索引 | 轴 | 方向 |
|------|-----|------|
| 0 | up | 上 |
| 1 | backward | 后（**y 增大为向后**，不是向前） |
| 2 | right | 右 |

配置或调试 Cartesian 目标时务必按此约定理解，否则易出现 `no_ik` 或碰撞。

---

## 生命周期与安全

```
CMD_INIT  → 连接 CAN →（可选）使能 →（可选）startup_joint_pose → 开始发布状态
CMD_SHUTDOWN → 电子急停 → 失能 → 断开 CAN
```

- **`enable_on_init: false`**：首条运动指令前自动使能；适合调试，需注意首次指令前臂可能未使能。
- **`CMD_SHUTDOWN`**：调用 `emergency_stop()` 后 `disable()` 并 `disconnect()`；停止 hook 见 [`scripts/stop.sh`](./scripts/stop.sh)。
- 运动指令在 driver 层串行执行（全局锁）；并发多条 command 会排队。
- **错误传播**：`joint_command` / `pos_command` 等回调内异常仅写日志（`log.error`），**不会**向 Robonix skill 抛出 fault。上层若只订阅 topic 而不看 primitive 日志，可能表现为“超时未到位”。

---

## 双臂部署

每个 CAN 连接的臂在 manifest 中单独实例化一份 `nero_arm`，并区分：

- `can_channel`
- `joint_name_prefix` / `reference_frame`
- 各 topic 名

两实例可向同一 `tf_joint_states_topic`（如 `/joint_states`）发布**部分**关节更新，供 URDF + `robot_state_publisher` 合并 TF。

---

## arm_status 故障码

驱动在运动前后会检查控制器 `arm_status`：

| 码 | 名称 | 说明 |
|----|------|------|
| `0x00` | normal | 正常 |
| `0x01` | e-stop | 急停；会尝试 `recover_faults()` |
| `0x02` | no_ik | 逆解失败，目标不可达 |
| `0x03` | singularity | 奇异 |
| `0x04` | joint_limit | 关节限位 |
| `0x06` | brake_not_open | 抱闸未开（瞬态，会等待） |
| `0x07` | collision | 碰撞检测 |

常见处理：

1. **`no_ik`**：目标 TCP 过远、姿态不可达或轴方向理解错误；先用 `end_pose` 或日志中的 `before/after` 核对当前 TCP，再小步调整。
2. **`e-stop` / `collision`**：解除物理急停或障碍后，重新 init 或发送下一条指令（driver 会尝试恢复）。
3. **CAN 不通**：检查 `ip link`、线缆、比特率与 `can_channel` 配置。

---

## 使用注意事项

1. **先 CAN 后 boot**：CAN 接口未 `UP` 时 init 会失败并返回 `Deferred` / 连接错误。
2. **固件版本**：`firmware_version` 须与臂上实际固件一致，否则行为异常。
3. **startup 位姿**：`startup_joint_pose` 必须是安全、无碰撞的关节角；init 失败会导致整 primitive 不可用。
4. **夹爪**：需要抓取时务必配置 `effector`；否则 `gripper_command` topic 不存在。
5. **tcp_offset**：若工具长度未在 SDK/URDF 中体现，可在此补偿；与手眼标定 offset 不要重复叠加。
6. **调试工具**：不依赖 Robonix 的关节/TCP 验证可用 [`tools/grasp_test`](../../tools/grasp_test/README.md)；标定可用 [`tools/camera_arm_calib`](../../tools/camera_arm_calib/README.md)。
7. **修改后重建**：改 `capabilities/`、`config.spec` 或 Python 代码后执行 `rbnx build primitives/nero_arm` 再重启。

---

## 相关文件

| 文件 | 说明 |
|------|------|
| [`config.spec`](./config.spec) | 配置项规范（机器可读） |
| [`package_manifest.yaml`](./package_manifest.yaml) | 包元数据与 capability 列表 |
| [`nero_arm/main.py`](./nero_arm/main.py) | Provider 入口、topic 注册、回调 |
| [`nero_arm/arm_driver.py`](./nero_arm/arm_driver.py) | pyAgxArm 封装与运动/状态逻辑 |
| [`CAPABILITY.md`](./CAPABILITY.md) | 能力说明（英文摘要） |
