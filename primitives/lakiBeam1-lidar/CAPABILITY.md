---
description: 2D planar lidar (LakiBeam1) - obstacle detection and open-space sensing via ROS1 TCPROS bridge.
---

# LakiBeam1 lidar (`robonix/primitive/lidar`)

RichBeam LakiBeam1 planar lidar. The lidar publishes
`sensor_msgs/LaserScan` on a remote ROS1 node (`/richbeam_lidar` ->
`/scan_filter`). This driver bridges that stream to ROS 2 via direct
TCPROS subscription - no ROS 1 environment is required on the host.

## Tools

### `snapshot` - `robonix/primitive/lidar/snapshot`
- input: none
- returns: `sensor_msgs/LaserScan` JSON. `ranges[i]` is the distance
  (meters) at angle `angle_min + i*angle_increment`.
- use cases:
  - "is there an obstacle within X m in front of me?"  ->  scan the
    middle of `ranges[]` for the smallest value.
  - "where is the nearest open space?"  ->  argmax of `ranges[]`.
- DO NOT use lidar to localize on a map; it has no map context.

## Topic: `lidar` (`robonix/primitive/lidar/lidar`)

Continuous `sensor_msgs/LaserScan` stream republished on ROS 2 `/scan`
from the remote ROS1 `/scan_filter` publisher. The ROS1 TCPROS receiver
runs in a background thread; a ROS 2 timer republishes each new scan on
the host DDS bus.

## Reasoning

Lidar is good for "stop before hitting a wall" sanity checks. Use it as
a safety floor before committing to a chassis/cmd burst, not as the
primary sensor for finding things by visual category (use camera for
that).
