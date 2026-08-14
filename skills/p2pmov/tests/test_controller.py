# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for p2pmov_skill.controller — no hardware or SDK needed.

The TBox SDK is imported lazily inside start_runtime(), so these tests
run without libtbox_sdk_cpp.so.
"""
from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))

from p2pmov_skill.controller import (  # noqa: E402
    P2PController,
    canonical_state,
    normalize_point,
)


class FakeTBoxClient:
    """Records task_distribution / task_control calls instead of touching
    the real SDK."""

    def __init__(self):
        self.dispatched: list = []
        self.cancels = 0

    def task_distribution(self, x, y, yaw, **kwargs):
        self.dispatched.append((x, y, yaw, kwargs))

    def task_control(self, ctrl):
        self.cancels += 1


class TestCanonicalState(unittest.TestCase):
    def test_mapping(self):
        self.assertEqual(canonical_state(0), "PENDING")
        self.assertEqual(canonical_state(1), "RUNNING")
        self.assertEqual(canonical_state(2), "SUCCEEDED")
        self.assertEqual(canonical_state(3), "RUNNING")   # paused -> not terminal
        self.assertEqual(canonical_state(4), "FAILED")
        self.assertEqual(canonical_state(5), "CANCELED")
        self.assertEqual(canonical_state(99), "RUNNING")  # unknown -> RUNNING


class TestNormalizePoint(unittest.TestCase):
    def test_dict_form(self):
        self.assertEqual(normalize_point({"x": 1, "y": 2, "yaw": 3}, "a"),
                         (1.0, 2.0, 3.0))

    def test_list_form(self):
        self.assertEqual(normalize_point([1, 2, 3], "a"), (1.0, 2.0, 3.0))

    def test_bad_forms(self):
        for bad in ({"x": 1}, [1, 2], "nope", 42, None):
            with self.assertRaises(ValueError):
                normalize_point(bad, "a")


class TestMove(unittest.TestCase):
    def setUp(self):
        self.fake = FakeTBoxClient()
        self.ctrl = P2PController(
            token="t", map_name="m.bin",
            points={
                "充电桩": {"x": 20.12, "y": 1.85, "yaw": -1.59},
                "工位A": [0.0, 0.0, 0.0],
            },
            speed_ratio=5,
        )
        self.ctrl._client = self.fake  # noqa: SLF001 — unit-test injection

    def test_move_by_name(self):
        ok, run_id, msg = self.ctrl.move(point_name="充电桩")
        self.assertTrue(ok)
        self.assertTrue(run_id.startswith("p2p-"))
        self.assertIn("充电桩", msg)
        self.assertEqual(self.fake.dispatched[0][:3], (20.12, 1.85, -1.59))
        self.assertEqual(self.fake.dispatched[0][3]["map_name"], "m.bin")
        self.assertEqual(self.fake.dispatched[0][3]["speed_ratio"], 5)

    def test_move_by_coords(self):
        ok, _, _ = self.ctrl.move(x=1.5, y=-2.5, yaw=0.25)
        self.assertTrue(ok)
        self.assertEqual(self.fake.dispatched[0][:3], (1.5, -2.5, 0.25))

    def test_name_wins_over_coords(self):
        ok, _, _ = self.ctrl.move(point_name="工位A", x=9.0, y=9.0, yaw=9.0)
        self.assertTrue(ok)
        self.assertEqual(self.fake.dispatched[0][:3], (0.0, 0.0, 0.0))

    def test_unknown_point(self):
        ok, run_id, msg = self.ctrl.move(point_name="不存在")
        self.assertFalse(ok)
        self.assertEqual(run_id, "")
        self.assertIn("unknown point", msg)
        self.assertEqual(self.fake.dispatched, [])

    def test_not_initialized(self):
        ctrl = P2PController(token="t", map_name="m.bin")
        ok, _, msg = ctrl.move(point_name="充电桩")
        self.assertFalse(ok)
        self.assertIn("not initialized", msg)

    def test_cancel(self):
        ok, _ = self.ctrl.cancel()
        self.assertTrue(ok)
        self.assertEqual(self.fake.cancels, 1)


class TestStatus(unittest.TestCase):
    def test_no_task_yet(self):
        ctrl = P2PController(token="t", map_name="m.bin")
        s = ctrl.status()
        self.assertFalse(s["known"])
        self.assertEqual(s["state"], "PENDING")

    def test_task_snapshot_mapping(self):
        task = types.SimpleNamespace(
            task_state=2, run_state=6, distance=1.5,
            uuid=b"abc123" + b"\x00" * 37,
        )
        ctrl = P2PController(token="t", map_name="m.bin")
        with ctrl._lock:  # noqa: SLF001 — unit-test access
            ctrl._last_task = task
        s = ctrl.status()
        self.assertTrue(s["known"])
        self.assertEqual(s["state"], "SUCCEEDED")
        self.assertEqual(s["task_state"], 2)
        self.assertEqual(s["run_state"], 6)
        self.assertEqual(s["distance_m"], 1.5)
        self.assertIn("abc123", s["detail"])


if __name__ == "__main__":
    unittest.main()
