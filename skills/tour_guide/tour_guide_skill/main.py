# SPDX-License-Identifier: MIT
"""tour_guide — 展厅导览技能.

一次调用讲一个区域: `introduce(area="前台")` 返回该区域的介绍文案, 调用方
(pilot / 模型) 原样播报给用户. 技能没有游标, 也不记得"讲到第几站": 区域之间
相互独立, 顺序写在 CAPABILITY.md 里由模型决定怎么走.

介绍文案不是代码, 是场地数据: 放在 robonix_manifest.yaml 的 `config.areas`,
由 `on_init` 解析校验 —— 换一段串词是改配置, 不是改这个文件. 配置里没有兜底:
漏配或配错让 CMD_INIT 失败, 好过拿占位场地糊弄用户.
"""
import logging

from robonix_api import Skill, Ok, Err

tour_guide = Skill(id="tour_guide", namespace="robonix/skill/tour_guide")

# Codegen output: the typed dataclasses derived from
# capabilities/lib/tour_guide/srv/IntroduceArea.srv. Available after
# `rbnx codegen --mcp` runs.
from tour_guide_mcp import IntroduceArea_Request, IntroduceArea_Response  # noqa: E402

log = logging.getLogger("tour_guide")
logging.basicConfig(level=logging.INFO, format="[tour_guide] %(levelname)s %(message)s")


# What a call that arrives before the executor has activated the skill gets.
# The executor activates lazily on the first MCP call, so this is a race, not
# a fault.
NOT_ACTIVE_TEXT = "(skill not yet activated)"

# What a caller that names an area the script doesn't have gets back. The
# names are CAPABILITY.md's to list, not the tool's: echoing them here would
# make a mistyped call look like a successful listing.
UNKNOWN_AREA_TEXT = "(没有这个区域)"


# ── module state populated by lifecycle handlers ─────────────────────
# `areas` survives deactivate: it is config, not a heavy resource, and there
# is no re-init to reload it from.
_state = {
    "areas": [],      # filled by on_init — config.areas is required
    "active": False,
}


def _validate_areas(raw) -> "list[dict] | str":
    """Normalize a `config.areas` value, or return an error string.

    The error string goes into the CMD_INIT reply as-is, so it names the
    offending entry by index — a venue script with five stops should not fail
    with "somewhere in your config".
    """
    if not isinstance(raw, list) or not raw:
        return "must be a non-empty list of {name, coordinate, description}"
    areas = []
    for i, item in enumerate(raw):
        if not isinstance(item, dict):
            return f"[{i}] must be a mapping with name, coordinate, description"
        if not all(str(item.get(k) or "").strip() for k in ("name", "description")):
            return f"[{i}] needs a non-blank name and description"
        coord = item.get("coordinate")
        if not isinstance(coord, (list, tuple)) or len(coord) != 3 or not all(
            isinstance(v, (int, float)) for v in coord
        ):
            return f"[{i}] coordinate must be three numbers (x, y, z), got {coord!r}"
        areas.append(
            {
                "name": str(item["name"]).strip(),
                "coordinate": [float(v) for v in coord],
                "description": str(item["description"]).strip(),
            }
        )
    return areas


# ── @tour_guide.mcp: the actual LLM-callable tool ────────────────────────
# Description for pilot's tool-picker comes from the docstring.
# Pilot uses it verbatim when deciding whether to dispatch this skill.
@tour_guide.mcp("robonix/skill/tour_guide/introduce")
def introduce(req: IntroduceArea_Request) -> IntroduceArea_Response:
    """导览技能: 传入区域名, 返回该区域的介绍文案, 由调用方原样播报给用户.

    一次调用只讲一个区域; 区域清单和顺序见本技能的 CAPABILITY.md.
    没有持久化状态, 同一区域可以重复讲.
    """
    # if not _state["active"]:
    #    return IntroduceArea_Response(narration=NOT_ACTIVE_TEXT)

    # Exact name match, trimmed only: the name IS the calling key, and
    # CAPABILITY.md lists it, so a caller has no other spelling to guess at.
    log.info("introduce(area=%r)", req.area)
    key = (req.area or "").strip()
    area = next((a for a in _state["areas"] if a["name"] == key), None)
    return IntroduceArea_Response(
        narration=area["description"] if area else UNKNOWN_AREA_TEXT
    )


# ── Lifecycle ────────────────────────────────────────────────────────
@tour_guide.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE. Parse and validate `config.areas` — the tour
    script is venue data, so this is where a mis-provisioned venue fails,
    before any call can reach a user."""
    areas = _validate_areas(cfg.get("areas"))
    if isinstance(areas, str):
        log.error("init failed: config.areas %s", areas)
        return Err(f"config.areas {areas}")

    _state["areas"] = areas
    log.info("init ok: %d areas", len(areas))
    return Ok()


@tour_guide.on_activate
def activate():
    """INACTIVE → ACTIVE. Where a real skill loads models and opens hardware;
    triggered by the executor lazily on the first MCP call."""
    _state["active"] = True
    return Ok()


@tour_guide.on_deactivate
def deactivate():
    """ACTIVE → INACTIVE. Keeps the config, drops the rest; a follow-up MCP
    call re-activates."""
    _state["active"] = False
    return Ok()


@tour_guide.on_shutdown
def shutdown():
    """any → TERMINATED. Last-chance cleanup."""
    return Ok()


def main() -> int:
    tour_guide.run()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
