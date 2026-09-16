# guided_tour（导览）部署配置（robonix_manifest.yaml `skill: guided_tour: config:`）。
#
# 上游依赖通过 Atlas 解析，无需在此配置：
#   navigate:   robonix/service/navigation/navigate        (MCP) — 下发 nav2 目标
#   nav_status: robonix/service/navigation/navigate/status (MCP) — 轮询导航状态
#   nav_cancel: robonix/service/navigation/navigate/cancel (MCP) — 取消导航
#   speak:      robonix/service/speech/speak               (MCP) — 语音播报

config:
  # string，默认 "map"。nav2 目标坐标系（PoseStamped.header.frame_id）。
  frame_id: "map"

  # string，默认 ""（speech 服务的 default_speaker_provider_id）。
  # 播报使用的扬声器 primitive 的 provider_id；留空用 speech 服务默认扬声器。
  speech_target: ""

  # 以下为可调参数（均有默认值，可省略）。

  # float，默认 2.0。导航状态轮询间隔（秒）。
  poll_interval_s: 2.0

  # float，默认 300.0。单站导航超时（秒），超时判 FAILED 并取消该目标。
  navigation_timeout_s: 300.0

  # float，默认 1.0。每次播报后的稳定等待（秒）。
  announce_pause_s: 1.0

  # string，默认 ""。可选：出发前的开场播报。
  start_announcement: ""

  # string，默认 ""。可选：结束后的收尾播报。
  finish_announcement: ""

  # string，必填（与 stops 二选一，优先 stops_file）。站点 JSON 文件路径，
  # 相对部署目录（robonix_manifest.yaml 所在目录，经 RBNX_INVOCATION_CWD）或绝对路径。
  # 文件内容：顶层数组，或 {"stops": [...]}。每站字段同下方 stops 说明。
  stops_file: "config/guided_tour_stops.json"

  # list[dict]，可选（未设置 stops_file 时的回退）。固定导览路线（按顺序访问）。
  # 每站字段：
  #   name          string   站点名（run 工具的 stop_names 按它过滤）
  #   point         {x,y,yaw} 或 [x,y,yaw]  目标坐标；也可直接写 x/y/yaw
  #   x,y,yaw       float    yaw 单位为弧度（nav2 目标朝向，0 = 地图 +x）
  #   announcement  string   到达后播报的文本（必填，非空）
  # stops:
  #   - name: 前台
  #     point: [1.0, 2.0, 0.0]
  #     announcement: "欢迎来到前台"
