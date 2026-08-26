# chassis_imu

Robonix package wrapping the **Hantewin BenBen chassis controller's built-in
6-axis IMU**. The controller publishes `sensor_msgs/Imu` on a remote ROS 1
node (topic `/imu_data`). This driver bridges that stream to ROS 2 via direct
TCPROS subscription - no ROS 1 environment is required on the host - and
atlas-registers the output topic under `robonix/primitive/imu/*` so
consumers (mapping, LIO, localization) discover the topic name through atlas.

## Capability surface

| Contract                                      | Mode      | Transport | Source / handler                            |
| --------------------------------------------- | --------- | --------- | ------------------------------------------- |
| `robonix/lifecycle/driver`                    | rpc       | gRPC      | shared `Driver(CMD_INIT, config_json)` lifecycle |
| `robonix/primitive/imu/imu`           | topic_out | ROS 2     | `/imu` (sensor_msgs/Imu)                    |
| `robonix/primitive/imu/snapshot`      | rpc       | MCP       | one-shot Imu capture                        |

## Architecture

```
ROS1 device                    Host (ROS 2 Jazzy)
┌─────────────┐   TCPROS     ┌───────────────────────────────────┐
│ chassis      │─────────────│ _Ros1ImuReceiver (thread)          │
│ controller   │  xmlrpc +   │   · discover port via ROS master    │
│ /imu_data    │  socket     │   · negotiate via requestTopic      │
└─────────────┘              │   · parse binary sensor_msgs/Imu    │
                             │   · store latest in _latest_imu     │
                             │                                     │
                             │ _publish_imu() (ROS 2 timer)        │
                             │   · convert dict -> Imu msg          │
                             │   · publish on /imu (best_effort)    │
                             │                                     │
                             │ snapshot() (MCP rpc)                 │
                             │   · return _latest_imu as MCP        │
                             └─────────────────────────────────────┘
```

The ROS1 TCPROS bridging follows the same pattern as the chassis driver's
`/odom` bridge and the LakiBeam1 lidar driver's `/scan_filter` bridge: a
background thread maintains the connection (with automatic reconnect), stores
the latest frame, and a ROS 2 timer republishes each new frame on the host DDS
bus. Duplicate suppression (via the ROS1 header `seq`) ensures the ROS 2
publish rate never exceeds the actual IMU frame rate.

Unlike `/odom`, the ROS1 publisher port is not hardcoded: `init` queries the
ROS master (`getSystemState` / `lookupNode`) to discover which port the
`/imu_data` publisher is serving, then builds the publisher URI as
`http://<ros1_target_ip>:<port>/`.

## Driver-init lifecycle

`start.sh` brings up the atlas bridge process. The shared Robonix runtime
registers the lifecycle driver, then the provider blocks on heartbeat
awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init it passes the manifest's `config:` block as
JSON. The handler starts the ROS1 TCPROS receiver, waits for the first Imu
frame (sentinel), creates the ROS 2 publisher + timer, and returns ok.

## Layout

```
chassis_imu/
├── package_manifest.yaml         robonix dev-packaging spec
├── config.spec                   runtime config documentation
├── CAPABILITY.md                 capability surface description
├── chassis_imu_driver/
│   ├── __init__.py
│   └── driver.py                 ROS1 TCPROS bridge + MCP snapshot
├── scripts/
│   ├── build.sh                  rbnx codegen + ROS 2 IDL colcon build
│   └── start.sh                  source ROS, exec driver
└── README.md
```

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "imu_topic": "/imu",
  "imu_hz": 100.0,
  "frame_id": "imu_link",
  "parent_frame": "base_link",
  "ros1_target_ip": "192.168.10.1",
  "ros1_imu_topic": "/imu_data",
  "sentinel_timeout_s": 15.0
}
```

## Build / run standalone

```bash
bash scripts/build.sh           # or:  rbnx build -p .
bash scripts/start.sh           # or:  rbnx boot  -p .
```

After Init the IMU stream should appear on:

```bash
ros2 topic hz /imu              # ~100 Hz sensor_msgs/Imu
```

## Network - host-side prereqs

The chassis controller's ROS 1 master must be reachable at the configured
`ros1_target_ip` (default `192.168.10.1`, master on port `11311`). The host
must have a route to that address (the robot's controller). No ROS 1
installation is needed on the host - the driver speaks TCPROS directly over a
raw socket.

## License

MulanPSL-2.0
