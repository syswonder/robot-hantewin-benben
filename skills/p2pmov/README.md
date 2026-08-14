# p2pmov — point-to-point navigation skill

Point-to-point movement for the Hantewin BenBen chassis via the TBox SDK
`client.task_distribution(x, y, yaw, map_name, ...)`. Triggered by the
LLM/pilot through `rbnx ask "go to <point name>"` or
`rbnx ask "go to (x, y, yaw)"` — either intent resolves to the
`robonix/skill/p2pmov/move` MCP tool.

See `CAPABILITY.md` for the full interface + behaviour spec.

## Quickstart

```bash
# from the package root
bash scripts/build.sh   # rbnx codegen (proto stubs + MCP dataclasses)
# then register in the deploy manifest under `skill:` and `rbnx boot`
```

Manifest snippet:

```yaml
skill:
  - name: p2pmov
    path: ./skills/p2pmov
    config:
      token: "<TBox auth token — same as benben_chassis>"
      map_name: "bdyjd_27_0803.bin"
      points:
        饮水机: {x: 20.22, y: 2.18,yaw: -1.56}
        原点: {x: 0.0, y: 0.0, yaw: 0.0}
        工位: {x: 2.29, y: 1.01, yaw: 2.89}
```

The skill registers 3 MCP tools with atlas. Pilot's tool catalog will
include `robonix/skill/p2pmov/{move, move/status, move/cancel}`.

## Layout

```
p2pmov/
├── package_manifest.yaml    # 4 caps + build/start hooks
├── capabilities/            # package-local TOMLs + .srv files
│   ├── driver.v1.toml       # lifecycle entry (rbnx boot -> CMD_INIT)
│   ├── move.v1.toml         # MCP tool: dispatch goal
│   ├── status.v1.toml       # MCP tool: poll task lifecycle
│   ├── cancel.v1.toml       # MCP tool: abort task
│   └── lib/p2pmov/srv/      # P2PMove / P2PStatus / P2PCancel .srv
├── p2pmov_skill/            # python module
│   ├── atlas_bridge.py      # entrypoint: register + serve MCP + lifecycle
│   ├── controller.py        # TBoxClient wrapper, target resolution, states
│   └── tbox_sdk/            # vendored vendor SDK (libtbox_sdk_cpp.so)
├── scripts/{build,start}.sh # native host, no Docker
├── CAPABILITY.md            # LLM-readable spec (registered with atlas)
├── config.spec              # runtime config documentation
└── tests/                   # unit tests (no hardware needed)
```

## Notes

- **No deinit API**: `libtbox_sdk_cpp.so` has no deinit/stop API and its
  static singletons segfault on normal Python exit. `atlas_bridge.main()`
  therefore terminates with `os._exit()` after flushing stdout/stderr.
- **Auth gate**: all SDK commands (including `task_distribution`) are only
  valid after `wait_until_ready()` (AUTH_SUCCESS=8); the controller refuses
  to dispatch otherwise.
- The skill is ROS-agnostic (TBox SDK only) but ships with the same native
  start.sh pattern as the benben_chassis primitive.

License: MulanPSL-2.0
