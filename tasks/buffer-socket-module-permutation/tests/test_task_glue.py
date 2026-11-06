"""Task glue tests for buffer socket module permutation: task file loading, objects, initial scorer evaluation."""

from __future__ import annotations

import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
else:
    cand = HERE.parent.parent / "refs" / "yam-isaac-lab-arena"
    if cand.exists():
        sys.path.insert(0, str(cand))

try:
    import yam_arena
    from yam_arena import task_file
except ImportError:
    yam_arena = None


@unittest.skipIf(yam_arena is None, "yam_arena not importable: set YAM_ARENA_SRC")
class TestTaskGlue(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.task = task_file.load(HERE / "task.py")
        cls.mod = cls.task.module

    def test_task_attributes_and_names(self):
        expected_names = (
            "fixture",
            "module_green",
            "module_red",
            "module_blue",
            "lamp_status_red",
            "lamp_status_green",
        )
        self.assertEqual(self.task.names, expected_names)
        self.assertEqual(self.task.scene, "station")
        self.assertEqual(self.task.score_over, "end")
        self.assertEqual(self.task.success_hold_s, 1.0)

    def test_fixture_sliders(self):
        fixture = next(obj for obj in self.task.objects if obj.name == "fixture")
        slider_names = {s.name for s in fixture.sliders}
        expected = {"plunger_0", "plunger_1", "plunger_2", "plunger_3", "commit_btn"}
        self.assertEqual(slider_names, expected)

    def test_module_mass_and_friction(self):
        for name in ("module_green", "module_red", "module_blue"):
            mod = next(obj for obj in self.task.objects if obj.name == name)
            self.assertEqual(mod.mass, 0.080)
            self.assertEqual(mod.friction, 0.8)

    def test_initial_objects_scorer_eval(self):
        import geometry as g

        objects = {
            "fixture": SimpleNamespace(
                pos=np.array([0.0, 0.0, 0.0]),
                quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                sliders={"commit_btn": 0.0, "plunger_0": 0.0, "plunger_1": 0.0, "plunger_2": 0.0, "plunger_3": 0.0},
            ),
            "module_green": SimpleNamespace(
                pos=np.array(g.START_POS["module_green"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
            "module_red": SimpleNamespace(
                pos=np.array(g.START_POS["module_red"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
            "module_blue": SimpleNamespace(
                pos=np.array(g.START_POS["module_blue"]),
                quat_xyzw=np.array([0.0, 0.0, np.sin(g.PLUG_YAW_SEATED / 2), np.cos(g.PLUG_YAW_SEATED / 2)]),
                lin_vel=np.zeros(3),
                ang_vel=np.zeros(3),
            ),
        }

        self.mod._reset_scorer()
        score_val = self.mod.score(objects)
        succ_val = self.mod.success(objects)
        self.assertEqual(score_val, 0.0)
        self.assertFalse(succ_val)


if __name__ == "__main__":
    unittest.main()
