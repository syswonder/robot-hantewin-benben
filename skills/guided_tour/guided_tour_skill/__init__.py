# SPDX-License-Identifier: MulanPSL-2.0
"""guided_tour_skill — 导览 skill: visit fixed stops in order and speak.

Modules:
  - atlas_bridge: entrypoint. Registers the skill with atlas, resolves the
    nav2 navigation + speech/speak MCP endpoints, and serves the
    run / run/status / run/cancel MCP tools + lifecycle.
  - controller: the stop-by-stop navigation + speech loop (transport-agnostic).
  - config: tour route + tunables parsing from the deploy config.
"""
