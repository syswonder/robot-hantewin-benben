# Runtime config accepted by the p2pmov skill.
#
# This file documents the mapping passed as the package's `config:` value
# in a robot deployment manifest. It is not loaded by the provider — the
# values are transmitted as JSON via Driver(CMD_INIT, config_json) to the
# on_init handler.

config:
  # string, required (or env P2PMOV_TOKEN).
  # Authentication token for the TBox SDK. Same token as the benben_chassis
  # primitive uses; the skill initializes its own TBoxClient with it.
  token: ""

  # string, required (or env P2PMOV_MAP_NAME).
  # Onboard vendor navigation map file, format 楼栋_层_日期.bin (see the
  # vendor SDK doc). Must exist on the chassis for task_distribution to
  # navigate. Example: bdyjd_27_0803.bin
  map_name: ""

  # dict[str, coords], optional (default {}).
  # Named points the LLM can target with move(point_name=...). coords is
  # either {x, y, yaw} or [x, y, yaw]. x/y are metres in the onboard map
  # frame; yaw is passed through verbatim to task_distribution (the vendor
  # spec documents degrees, the reference example used radian-like values).
  points:
    充电桩: {x: 20.12, y: 1.85, yaw: -1.59}

  # int, default 3.
  # task_distribution speed_ratio (vendor scale, higher = faster).
  speed_ratio: 3
