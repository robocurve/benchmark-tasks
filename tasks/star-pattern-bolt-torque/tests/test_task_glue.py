"""Task glue tests for Task 1: task file loading, object structure, slider parameters, and zero-gate initialization."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

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
        expected_names = [
            "fixture",
            "status_lamp_red",
            "status_lamp_green",
        ]
        for c in g.CORNERS:
            for s in (1, 2, 3):
                expected_names.append(f"lamp_{c}_{s}")

        self.assertEqual(list(self.task.names), expected_names)
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

        steps = max_duration * 20
        # Run limit 2,400 steps: must leave >= 25% spare margin (i.e. < 1,800 steps)
        self.assertLess(steps, 2400 * 0.75)
        self.assertGreater(steps, 500)

    def test_fixture_sliders(self):
        fixture = next(obj for obj in self.task.objects if obj.name == "fixture")
        expected_sliders = {f"btn_{c}" for c in g.CORNERS}
        slider_names = {s.name for s in fixture.sliders}
        self.assertEqual(slider_names, expected_sliders)
        for s in fixture.sliders:
            self.assertEqual(s.travel, 0.008)
            self.assertEqual(s.rate, 100.0)
            self.assertEqual(s.damping, 4.0)
            self.assertEqual(s.mass, 0.004)

    def test_fixture_mass_and_friction(self):
        fixture = next(obj for obj in self.task.objects if obj.name == "fixture")
        self.assertEqual(fixture.mass, 6.0)
        self.assertEqual(fixture.friction, 1.0)

    def test_initial_step_zero_gate_not_tripped(self):
        objects = {
            "fixture": make_part_state(g.BASE_POS, {f"btn_{c}": 0.0 for c in g.CORNERS}),
            "status_lamp_red": make_part_state((0, 0, g.STOWED_Z)),
            "status_lamp_green": make_part_state((0, 0, g.STOWED_Z)),
        }
        for c in g.CORNERS:
            for s in (1, 2, 3):
                objects[f"lamp_{c}_{s}"] = make_part_state((0, 0, g.STOWED_Z))

        res = self.mod._update(objects)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)
        self.assertFalse(res.details.get("failed", False))


if __name__ == "__main__":
    unittest.main()
