---
description: Point-to-point navigation on the vendor TBox map — use for "go to <named point>" / "go to (x, y, yaw)"; these destinations must NOT be routed via scene goal_near/goal_room + navigation/navigate (different map frame).
---

# p2pmov — point-to-point navigation skill

Moves the BenBen chassis to a **named point** or a **raw goal coordinate**
by dispatching a TBox SDK `task_distribution` goal. The chassis navigates
autonomously on its onboard vendor map (obstacle avoidance is the vendor
firmware, not robonix nav2).

## When to use this skill (intent routing)

Use `robonix/skill/p2pmov/move` when the user asks to go to:

- a **configured point name** — "去饮水机", "前往原点" (keys of the
  deployment `points` config);
- a **raw coordinate** — "前往坐标 (20.20, 2.28, -1.57)".

## When NOT to use it — and don't route these through scene

This skill drives the **vendor TBox onboard map** via `task_distribution`.
That is a different navigation stack and a different coordinate frame from
the robonix nav2 chain (`scene goal_near` / `goal_room` →
`navigation/navigate`, which resolves poses on the RTAB-Map frame):

- Do NOT resolve a p2pmov point name with `scene goal_near` / `goal_room`
  — the returned pose is a nav2-frame coordinate and does not match the
  vendor map.
- `scene goal_near` / `goal_room` are only for scene-registered objects
  and rooms, and they must be followed by `navigation/navigate`, never by
  this skill.

## Interface (3 MCP tools)

### `robonix/skill/p2pmov/move`

Dispatch a navigation goal. Provide **either**:

| param        | type    | meaning                                          |
|--------------|---------|--------------------------------------------------|
| `point_name` | string  | key into the deployment `config.points` dict     |
| `x`          | float64 | goal X in the onboard map frame (metres)         |
| `y`          | float64 | goal Y in the onboard map frame (metres)         |
| `yaw`        | float64 | goal heading, passed through verbatim            |

`point_name` takes priority when both are present. Returns immediately
with `{accepted, run_id, message}` — **save the exact `run_id`** and poll
`move/status` with it.

### `robonix/skill/p2pmov/move/status`

Poll the TBox task lifecycle. Returns
`{known, state, task_state, run_state, distance_m, detail}`.
`state ∈ {PENDING | RUNNING | SUCCEEDED | FAILED | CANCELED}` (terminal:
SUCCEEDED / FAILED / CANCELED). Empty `run_id` = most recent task.

### `robonix/skill/p2pmov/move/cancel`

Abort the active navigation task (TBox `task_control(CANCEL)`). Idempotent.

## Usage pattern (IMPORTANT — thread the run_id)

1. Call `move` ONCE with a `point_name` or `x`/`y`/`yaw`. Save the `run_id`.
2. Poll `move/status` with that SAME `run_id` until `state` is terminal.
   Do not call `move` again to monitor — it dispatches a new goal.
3. To stop it, call `move/cancel`.

## Config (deployment manifest `skill:` block)

```yaml
skill:
  - name: p2pmov
    path: ./skills/p2pmov
    config:
      token: "<TBox auth token — same as benben_chassis config.token>"
      map_name: "bdyjy_27_0805.bin"   # onboard vendor map file
      points:
        饮水机: {x: 20.22, y: 2.10,yaw: -1.52}
        原点: {x: 0.0, y: 0.0, yaw: 0.0}
        工位: {x: 2.29, y: 1.01, yaw: 2.89}
```

`points` is a dict `{name: coords}`; coords may be `{x,y,yaw}` or
`[x,y,yaw]`.

## What this skill does NOT do

- No SLAM / localization — that is the vendor's onboard map + robonix's
  `service/map` (RTAB-Map) for the robonix map.
- No velocity control — that is `robonix/primitive/chassis/*`.

The skill is a thin dispatcher: resolve name → coords, then hand the goal
to the vendor SDK. `yaw` is passed through verbatim — the vendor SDK spec
documents it in degrees, while the reference example
(`examples/test_p2p_move.py`) used radian-like values, so match whatever
convention the `points` config uses.
