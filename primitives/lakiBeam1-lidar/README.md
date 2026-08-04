# lakibeam1_lidar

Robonix package wrapping the **RichBeam LakiBeam1** 2D planar LiDAR. The
LakiBeam1 publishes `sensor_msgs/LaserScan` on a remote ROS 1 node
(`/richbeam_lidar` -> `/scan_filter`). This driver bridges that stream to
ROS 2 via direct TCPROS subscription - no ROS 1 environment is required on
the host - and atlas-registers the output topic under
`robonix/primitive/lidar/*` so consumers (mapping, navigation, obstacle
check) discover the topic name through atlas.

## Capability surface

| Contract                              | Mode      | Transport | Source / handler                            |
| ------------------------------------- | --------- | --------- | ------------------------------------------- |
| `robonix/lifecycle/driver`            | rpc       | gRPC      | shared `Driver(CMD_INIT, config_json)` lifecycle |
| `robonix/primitive/lidar/lidar`       | topic_out | ROS 2     | `/scan` (sensor_msgs/LaserScan)             |
| `robonix/primitive/lidar/snapshot`    | rpc       | MCP       | one-shot LaserScan capture                  |

## Architecture

```
ROS1 device                   Host (ROS 2 Jazzy)
┌─────────────┐   TCPROS    ┌──────────────────────────────────┐
│ /richbeam   │─────────────│ _Ros1LaserScanReceiver (thread)  │
│ _lidar      │  xmlrpc +   │   · negotiate via requestTopic    │
│ /scan_filter│  socket     │   · parse binary LaserScan        │
└─────────────┘             │   · store latest in _latest_scan  │
                            │                                  │
                            │ _publish_scan() (ROS 2 timer)     │
                            │   · convert dict -> LaserScan msg │
                            │   · publish on /scan (best_effort)│
                            │                                  │
                            │ snapshot() (MCP rpc)              │
                            │   · return _latest_scan as MCP    │
                            └──────────────────────────────────┘
```

The ROS1 TCPROS bridging follows the same pattern as the chassis driver's
`/odom` bridge: a background thread maintains the connection (with
automatic reconnect), stores the latest frame, and a ROS 2 timer
republishes each new scan on the host DDS bus. Duplicate suppression
(via the ROS1 header `seq`) ensures the ROS 2 publish rate never exceeds
the actual lidar frame rate.

## Driver-init lifecycle

`start.sh` brings up the atlas bridge process. The shared Robonix runtime
registers the lifecycle driver, then the provider blocks on heartbeat
awaiting `Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init it passes the manifest's `config:` block as
JSON. The handler starts the ROS1 TCPROS receiver, waits for the first
LaserScan (sentinel), creates the ROS 2 publisher + timer, and returns ok.

## Layout

```
lakiBeam1-lidar/
├── package_manifest.yaml         robonix dev-packaging spec
├── config.spec                   runtime config documentation
├── CAPABILITY.md                 capability surface description
├── lidar_driver/
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
  "scan_topic": "/scan",
  "scan_hz": 10.0,
  "ros1_publisher_uri": "http://192.168.10.1:39233/",
  "ros1_topic": "/scan_filter",
  "sentinel_timeout_s": 15.0
}
```

## Build / run standalone

```bash
bash scripts/build.sh           # or:  rbnx build -p .
bash scripts/start.sh           # or:  rbnx boot  -p .
```

After Init the lidar scan should appear on:

```bash
ros2 topic hz /scan             # ~10 Hz sensor_msgs/LaserScan
```

## Network - host-side prereqs

The LakiBeam1's ROS 1 node must be reachable at the configured
`ros1_publisher_uri` (default `http://192.168.10.1:39233/`). The host
must have a route to `192.168.10.1` (the robot's controller). No ROS 1
installation is needed on the host - the driver speaks TCPROS directly
over a raw socket.

## License

MulanPSL-2.0
