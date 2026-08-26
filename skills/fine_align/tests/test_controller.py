"""Unit tests for fine_align's pure geometry (no hardware / transport)."""
from __future__ import annotations

import unittest

from fine_align_skill.controller import (
    AlignConfig,
    in_graspable_zone,
    plan_correction,
)


def _cfg(**kw) -> AlignConfig:
    return AlignConfig(**kw)


class InGraspableZoneTest(unittest.TestCase):
    def test_center_is_in_zone(self):
        cfg = _cfg(graspable_center=(0.0, 0.0, 0.0), graspable_tolerance=(0.05, 0.05, 0.03))
        self.assertTrue(in_graspable_zone([0.0, 0.0, 0.0], cfg))

    def test_outside_tolerance_is_not_in_zone(self):
        cfg = _cfg(graspable_center=(0.0, 0.0, 0.0), graspable_tolerance=(0.05, 0.05, 0.03))
        self.assertFalse(in_graspable_zone([0.2, 0.0, 0.0], cfg))


class PlanCorrectionTest(unittest.TestCase):
    def test_no_correction_when_in_zone(self):
        cfg = _cfg(graspable_center=(0.0, 0.0, 0.0), graspable_tolerance=(0.05, 0.05, 0.03))
        self.assertEqual(plan_correction([0.0, 0.0, 0.0], cfg), (0.0, 0.0))

    def test_forward_error_yields_forward_action(self):
        cfg = _cfg(forward_step_max_m=0.3, forward_gain=1.0)
        forward_m, rotate_deg = plan_correction([0.2, 0.0, 0.0], cfg)
        # At most one action; sign is TODO until the mount transform is measured.
        self.assertEqual(rotate_deg, 0.0)
        self.assertNotEqual(forward_m, 0.0)
        self.assertLessEqual(abs(forward_m), 0.3)

    def test_lateral_error_yields_rotate_action(self):
        cfg = _cfg(rotate_step_max_deg=30.0, rotate_gain=1.0)
        forward_m, rotate_deg = plan_correction([0.0, 0.15, 0.0], cfg)
        self.assertEqual(forward_m, 0.0)
        self.assertNotEqual(rotate_deg, 0.0)
        self.assertLessEqual(abs(rotate_deg), 30.0)


if __name__ == "__main__":
    unittest.main()
