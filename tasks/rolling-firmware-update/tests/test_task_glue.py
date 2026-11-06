"""Tests for task.py contract and glue."""

import os
import unittest
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import geometry as g
import task


class TestTaskGlue(unittest.TestCase):
    def test_module_contract(self):
        self.assertEqual(task.SCENE, "station")
        self.assertEqual(task.SCORE_OVER, "end")
        self.assertEqual(task.SUCCESS_HOLD_S, 1.0)
        self.assertIsInstance(task.INSTRUCTION, str)
        self.assertGreater(len(task.INSTRUCTION), 20)
        self.assertTrue(callable(task.success))
        self.assertTrue(callable(task.score))
        self.assertTrue(callable(task.demo))

    def test_objects_list(self):
        names = [obj.name for obj in task.OBJECTS]
        self.assertIn("fixture", names)
        self.assertIn("active_lamp_a", names)
        self.assertIn("active_lamp_b", names)
        self.assertIn("load_lamp_green", names)
        self.assertIn("load_lamp_red", names)
        for i in range(1, 5):
            self.assertIn(f"progress_a_{i}", names)
            self.assertIn(f"progress_b_{i}", names)

    def test_initial_scoring_evaluation(self):
        # Simulate object states at initial pose
        fixture_state = SimpleNamespace(
            pos=np.array(g.BASE_POS),
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            lin_vel=np.zeros(3),
            ang_vel=np.zeros(3),
            sliders={"door_a": 0.0, "door_b": 0.0, "btn_update_a": 0.0, "btn_update_b": 0.0, "btn_switch": 0.0},
        )
        objects = {"fixture": fixture_state}
        sc = task.score(objects)
        succ = task.success(objects)
        self.assertEqual(sc, 0.0)
        self.assertFalse(succ)

    def test_demo_returns_waypoint_dict(self):
        d = task.demo()
        self.assertIn("left", d)
        self.assertIn("right", d)
        self.assertGreater(len(d["left"]), 10)
        self.assertGreater(len(d["right"]), 10)


if __name__ == "__main__":
    unittest.main()
