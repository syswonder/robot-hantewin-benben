---
description: Guided tour (导览) — visit fixed stops in order via nav2 and speak an announcement at each via speech.
---

# guided_tour — 导览 skill

Visit a fixed, ordered list of stops: at each one, navigate there with **nav2**
(`robonix/service/navigation/navigate` on the RTAB-Map frame) and then speak an
announcement aloud (`robonix/service/speech/speak`). The route is deployment
config (`stops`), not decided by the LLM at call time.

## When to use this skill

Use `robonix/skill/guided_tour/run` when the user asks to "带人参观 / 导览 /
沿固定路线走一圈并介绍". Each stop's announcement is fixed in config.

This skill drives **nav2** (the RTAB-Map `map` frame). It must NOT be routed
through `p2pmov` (vendor TBox onboard map, different frame).

## Interface (3 MCP tools)

### `robonix/skill/guided_tour/run`

Start a tour. Returns immediately with `{accepted, run_id, detail}` — **save
the exact `run_id`** and poll `run/status`.

| param       | type   | meaning                                                        |
|-------------|--------|----------------------------------------------------------------|
| `stop_names`| string | optional; empty = all stops in order, else a whitespace/comma-separated subset of stop names |

### `robonix/skill/guided_tour/run/status`

Poll a tour. Empty `run_id` = most recent. Returns
`{known, state, phase, stop_index, total_stops, detail}`. `state ∈
{PENDING | RUNNING | SUCCEEDED | FAILED | CANCELED}` (terminal: SUCCEEDED /
FAILED / CANCELED). `phase` is one of idle / starting / navigating /
announcing / finished / canceled / error; `stop_index` is 1-based.

### `robonix/skill/guided_tour/run/cancel`

Abort a tour: cancels the active nav2 goal and stops the loop. Idempotent.

## Usage pattern (thread the run_id)

1. Call `run` once. Save `run_id`.
2. Poll `run/status` with that same `run_id` until `state` is terminal.
3. To stop it, call `run/cancel`.

## Config (deployment manifest `skill:` block)

```yaml
skill:
  - name: guided_tour
    path: ./skills/guided_tour
    config:
      frame_id: "map"          # nav2 goal frame
      speech_target: ""        # speaker provider_id; "" = speech default
      poll_interval_s: 2.0
      navigation_timeout_s: 300.0
      announce_pause_s: 1.0
      start_announcement: "导览开始，请跟我来"
      finish_announcement: "本次导览到此结束，谢谢"
      stops:
        - name: 前台
          point: {x: 1.0, y: 2.0, yaw: 0.0}
          announcement: "欢迎来到前台"
        - name: 展厅
          point: [5.0, 3.0, 1.57]
          announcement: "这里是展厅"
```

`yaw` is radians in the nav2 `map` frame. See `config.spec` for the full
schema (a stop accepts `point: {x,y,yaw}` / `[x,y,yaw]` or top-level `x`/`y`/`yaw`).

## What this skill does NOT do

- No mapping / localization — that is `service/map` (RTAB-Map).
- No obstacle-avoidance tuning — that is `service/navigation` (nav2).
- No arbitrary ad-hoc destinations — the route is fixed in config.
