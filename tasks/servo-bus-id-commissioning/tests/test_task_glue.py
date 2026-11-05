"""Task glue tests for Task F (servo bus commissioning): task file loading, object definitions, demo plan bounds."""

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

try:
    import yam_arena
    from yam_arena import task_file
except ImportError:
    yam_arena = None


@unittest.skipIf(yam_arena is None, "yam_arena not importable: set YAM_ARENA_SRC to a checkout")
class TestTaskGlue(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.task = task_file.load(HERE / "task.py")
        cls.mod = cls.task.module

    def test_loads_and_names(self):
        expected_names = (
            "fixture",
            "servo_red",
            "servo_green",
            "servo_blue",
            "bus_lamp_green",
            "readout_red",
            "readout_green",
            "readout_blue",
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
        self.assertLess(steps, 3000 * 0.75)  # must leave at least 25% spare margin under 3000 steps
        self.assertGreater(steps, 200)

    def test_fixture_sliders(self):
        fixture = next(obj for obj in self.task.objects if obj.name == "fixture")
        slider_names = {s.name for s in fixture.sliders}
        expected_sliders = {"plunger_0", "plunger_1", "plunger_2", "assign_btn", "reset_btn"}
        self.assertEqual(slider_names, expected_sliders)

    def test_servos_mass_and_friction(self):
        for name in ("servo_red", "servo_green", "servo_blue"):
            servo = next(obj for obj in self.task.objects if obj.name == name)
            self.assertEqual(servo.mass, 0.080)
            self.assertEqual(servo.friction, 0.8)

    def test_initial_state_score_zero(self):
        """Glue test: verify initial scene state scores 0.0 and success is False (C3)."""
        from types import SimpleNamespace
        import geometry as g

        objects = {
            "fixture": SimpleNamespace(
                pos=np.array([0.0, 0.0, 0.0]),
                quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                sliders={"assign_btn": 0.0, "reset_btn": 0.0, "plunger_0": 0.0, "plunger_1": 0.0, "plunger_2": 0.0},
            ),
            "servo_red": SimpleNamespace(
                pos=np.array(g.START_POS["red"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
            "servo_green": SimpleNamespace(
                pos=np.array(g.START_POS["green"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
            "servo_blue": SimpleNamespace(
                pos=np.array(g.START_POS["blue"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
        }
        self.mod._reset_scorer()
        self.assertFalse(self.mod.success(objects))
        self.assertEqual(self.mod.score(objects), 0.0)


if __name__ == "__main__":
    unittest.main()
