"""Task glue tests for Task 4 (blink-code module swap): task file loading, object definitions, demo plan bounds."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

HERE = Path(__file__).resolve().parent.parent
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
else:
    cand = HERE.parent.parent / "refs" / "yam-isaac-lab-arena"
    if cand.exists():
        sys.path.insert(0, str(cand))
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
            "module_1",
            "module_2",
            "module_3",
            "module_4",
            "spare",
            "lamp_amber",
            "lamp_green",
            "lamp_red",
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

        steps = max_duration * 20
        self.assertGreaterEqual(steps, 900)
        self.assertLessEqual(steps, 1300)
        self.assertLess(steps, 3000 * 0.75)  # must leave at least 25% spare margin under 3000 steps

    def test_fixture_sliders(self):
        fixture = next(obj for obj in self.task.objects if obj.name == "fixture")
        slider_names = {s.name for s in fixture.sliders}
        expected_sliders = {"plunger_0", "plunger_1", "plunger_2", "plunger_3", "diag_btn"}
        self.assertEqual(slider_names, expected_sliders)

    def test_modules_mass_and_friction(self):
        for name in ("module_1", "module_2", "module_3", "module_4", "spare"):
            mod = next(obj for obj in self.task.objects if obj.name == name)
            self.assertEqual(mod.mass, 0.080)
            self.assertEqual(mod.friction, 0.8)

    def test_initial_state_score_zero(self):
        import geometry as g

        objects = {
            "fixture": SimpleNamespace(
                pos=np.array([0.0, 0.0, 0.0]),
                quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                sliders={"diag_btn": 0.0, "plunger_0": 0.0, "plunger_1": 0.0, "plunger_2": 0.0, "plunger_3": 0.0},
            ),
            "module_1": SimpleNamespace(
                pos=np.array(g.START_POS["module_1"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
            "module_2": SimpleNamespace(
                pos=np.array(g.START_POS["module_2"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
            "module_3": SimpleNamespace(
                pos=np.array(g.START_POS["module_3"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
            "module_4": SimpleNamespace(
                pos=np.array(g.START_POS["module_4"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
            "spare": SimpleNamespace(
                pos=np.array(g.SPARE_STAGE_POS),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
        }
        self.assertEqual(self.mod.score(objects), 0.0)
        self.assertFalse(self.mod.success(objects))


if __name__ == "__main__":
    unittest.main()
