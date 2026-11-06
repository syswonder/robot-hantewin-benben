# Runtime config accepted by the tour_guide skill.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. It is documentation, not a runtime schema: the package
# itself validates what it reads (see `_validate_areas` in
# tour_guide_skill/main.py).

config:
  # list, required, must be non-empty. The tour script, in the order the guide
  # walks it. The live copy for a deployment lives in robonix_manifest.yaml —
  # there is no default here and the package has no fallback, so a venue that
  # forgets it fails CMD_INIT instead of narrating a placeholder venue.
  #
  # Each entry is one stop:
  #
  #   name         string, required, non-blank, unique within the list.
  #                Doubles as the `area` argument to introduce(), so it is
  #                also what CAPABILITY.md lists for the model to call.
  #
  #   coordinate   [x, y, z] numbers, required. Metres, relative to the venue
  #                origin. The skill does not consume it yet — introduce()
  #                only returns narration — but it is what a walk to this
  #                stop would use, and it is where "导览技能按坐标导航" gets
  #                its data.
  #
  #   description  string, required, non-blank. Returned verbatim as
  #                `narration`; the caller plays it as-is, so write it the
  #                way it should be heard, not as notes about the area.
  areas: []
