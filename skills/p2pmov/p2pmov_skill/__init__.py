# SPDX-License-Identifier: MulanPSL-2.0
"""p2pmov_skill — point-to-point navigation skill.

Modules:
  - atlas_bridge: entrypoint. Registers the skill with atlas, runs the
    FastMCP server with move/status/cancel tools, drives the lifecycle.
  - controller: TBoxClient wrapper + named-point / raw-coordinate target
    resolution + vendor task_state -> canonical executor state mapping.
"""
