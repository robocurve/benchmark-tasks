"""Task glue tests for Task G (coupled knob peak tuning): task loading, objects, sliders, zero-gate verification, and hold timing."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
sys.path.insert(0, str(HERE))

import geometry as g

try:
    import yam_arena
    from yam_arena import task_file
    from parts import PartState
except ImportError:
    yam_arena = None
    PartState = None


def make_part_state(pos, sliders=None, lin_vel=None, ang_vel=None) -> PartState:
    return PartState(
        pos=np.array(pos, dtype=float),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        lin_vel=np.array(lin_vel if lin_vel is not None else [0.0, 0.0, 0.0], dtype=float),
        ang_vel=np.array(ang_vel if ang_vel is not None else [0.0, 0.0, 0.0], dtype=float),
        low=np.zeros(3),
        high=np.zeros(3),
        sliders=dict(sliders) if sliders else {},
    )


@unittest.skipIf(yam_arena is None, "yam_arena not importable: set YAM_ARENA_SRC to a checkout")
class TestTaskGlue(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.task = task_file.load(HERE / "task.py")
        cls.mod = cls.task.module

    def setUp(self):
        self.mod._reset_scorer()

    def test_loads_and_names(self):
        expected_names = (
            "fixture",
            "knob_left",
            "knob_right",
            "needle",
        )
        self.assertEqual(self.task.names, expected_names)
        self.assertEqual(self.task.scene, "station")
        self.assertEqual(self.task.score_over, "end")
        self.assertEqual(self.task.success_hold_s, 1.0)

    def test_demo_plans_and_duration(self):
        plans = self.mod.demo()
        self.assertIn("left", plans)
        self.assertIn("right", plans)

        duration_left = sum(w.duration for w in plans["left"])
        duration_right = sum(w.duration for w in plans["right"])
        max_duration = max(duration_left, duration_right)

        # 20 Hz control rate: steps = duration * 20
        steps = max_duration * 20
        # Run limit 2,400 steps: must leave >= 25% spare margin (i.e. < 1,800 steps)
        self.assertLess(steps, 2400 * 0.75)
        self.assertGreater(steps, 200)

    def test_fixture_slider(self):
        fixture = next(obj for obj in self.task.objects if obj.name == "fixture")
        slider_names = {s.name for s in fixture.sliders}
        self.assertEqual(slider_names, {"read_btn"})
        read_btn = next(s for s in fixture.sliders if s.name == "read_btn")
        self.assertEqual(read_btn.travel, 0.008)
        self.assertEqual(read_btn.rate, 100.0)
        self.assertEqual(read_btn.damping, 4.0)

    def test_knobs_mass_and_friction(self):
        for name in ("knob_left", "knob_right"):
            knob = next(obj for obj in self.task.objects if obj.name == name)
            self.assertEqual(knob.mass, 0.080)
            self.assertEqual(knob.friction, 0.5)

    def test_initial_step_zero_gate_not_tripped(self):
        """Blocker G1 regression test: feeding declared initial poses to _update must give score 0.0 with NO gate tripped."""
        objects = {
            "fixture": make_part_state(g.BASE_POS, {"read_btn": 0.0}),
            "knob_left": make_part_state(g.START_POS["left"]),
            "knob_right": make_part_state(g.START_POS["right"]),
            "needle": make_part_state(g.needle_pos_from_r(0.0)),
        }
        res = self.mod._update(objects)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)
        self.assertFalse(res.details["zero_gate"], f"Zero gate tripped unexpectedly: {res.details.get('zero_gate_reason')}")

    def test_update_caching_per_control_step(self):
        """C4 test: success() and score() in same step must evaluate scorer exactly once."""
        objects = {
            "fixture": make_part_state(g.BASE_POS, {"read_btn": 0.0}),
            "knob_left": make_part_state(g.START_POS["left"]),
            "knob_right": make_part_state(g.START_POS["right"]),
            "needle": make_part_state(g.needle_pos_from_r(0.0)),
        }
        self.mod._reset_scorer()
        # First call (success)
        suc = self.mod.success(objects)
        # Second call in same step (score)
        sc = self.mod.score(objects)

        self.assertFalse(suc)
        self.assertEqual(sc, 0.0)
        # Ensure consecutive_success_steps was not incremented twice
        self.assertEqual(self.mod._SCORER.consecutive_success_steps, 0)

    def test_full_success_and_hold_sequence(self):
        """Verify full 1.0 ladder progression and exact 20-step (1.0 s) hold."""
        objs_start = {
            "fixture": make_part_state(g.BASE_POS, {"read_btn": 0.0}),
            "knob_left": make_part_state(g.START_POS["left"]),
            "knob_right": make_part_state(g.START_POS["right"]),
            "needle": make_part_state(g.needle_pos_from_r(0.0)),
        }
        objs_press1 = {
            "fixture": make_part_state(g.BASE_POS, {"read_btn": -0.007}),
            "knob_left": make_part_state(g.START_POS["left"]),
            "knob_right": make_part_state(g.START_POS["right"]),
            "needle": make_part_state(g.needle_pos_from_r(0.0)),
        }
        probe_l = (g.knob_cx_from_setting_mm(50.0), g.TRACK_Y["left"], g.KNOB_CZ)
        probe_r = (g.knob_cx_from_setting_mm(30.0), g.TRACK_Y["right"], g.KNOB_CZ)
        objs_probe_press2 = {
            "fixture": make_part_state(g.BASE_POS, {"read_btn": -0.007}),
            "knob_left": make_part_state(probe_l),
            "knob_right": make_part_state(probe_r),
            "needle": make_part_state(g.needle_pos_from_r(0.0)),
        }
        objs_peak_rest = {
            "fixture": make_part_state(g.BASE_POS, {"read_btn": 0.0}),
            "knob_left": make_part_state(g.PEAK_POS["left"]),
            "knob_right": make_part_state(g.PEAK_POS["right"]),
            "needle": make_part_state(g.needle_pos_from_r(1.0)),
        }

        # Step 1: Initial state -> 0.0
        r1 = self.mod._update(objs_start)
        self.assertEqual(r1.score, 0.0)

        # Step 2: Press READ at start (Reading 1) -> 0.25
        r2 = self.mod._update(objs_press1)
        self.assertEqual(r2.score, 0.25)
        self.mod._update(objs_start)

        # Step 3: Move to probe and press READ (Reading 2) -> 0.50 with distinct readings
        r3 = self.mod._update(objs_probe_press2)
        self.assertGreaterEqual(r3.score, 0.50)
        self.assertTrue(r3.details["has_distinct_readings"])

        # Step 3b: third separated reading at (65, 40) (the 1.0 rung needs three)
        objs_mid = {
            "fixture": make_part_state(g.BASE_POS, {"read_btn": -0.007}),
            "knob_left": make_part_state((g.knob_cx_from_setting_mm(65.0), g.TRACK_Y["left"], g.KNOB_CZ)),
            "knob_right": make_part_state((g.knob_cx_from_setting_mm(40.0), g.TRACK_Y["right"], g.KNOB_CZ)),
            "needle": make_part_state(g.needle_pos_from_r(0.0)),
        }
        objs_probe_release = dict(objs_probe_press2, fixture=make_part_state(g.BASE_POS, {"read_btn": 0.0}))
        self.mod._update(objs_probe_release)
        self.mod._update(objs_mid)
        self.mod._update(dict(objs_mid, fixture=make_part_state(g.BASE_POS, {"read_btn": 0.0})))

        # Step 4: Move to peak, grippers open, hold for 19 steps -> 0.75
        for step in range(19):
            # Create fresh dict reference so cache updates per step
            objs_step = dict(objs_peak_rest)
            r_hold = self.mod._update(objs_step)
            self.assertEqual(r_hold.score, 0.75)
            self.assertFalse(r_hold.success)

        # Step 20 of hold (1.0 s total hold) -> 1.0!
        r_success = self.mod._update(dict(objs_peak_rest))
        self.assertEqual(r_success.score, 1.0)
        self.assertTrue(r_success.success)


if __name__ == "__main__":
    unittest.main()
