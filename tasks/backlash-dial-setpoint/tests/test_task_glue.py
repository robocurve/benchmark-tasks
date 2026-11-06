"""Tests for task.py contract and glue."""

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
        self.assertIn("knob_left", names)
        self.assertIn("knob_right", names)
        self.assertIn("target_lamp_left", names)
        self.assertIn("target_lamp_right", names)
        self.assertIn("locked_lamp_left", names)
        self.assertIn("locked_lamp_right", names)
        for k in range(1, 11):
            self.assertIn(f"lamp_left_{k}", names)
            self.assertIn(f"lamp_right_{k}", names)

    def test_initial_scoring_evaluation(self):
        fixture_state = SimpleNamespace(
            pos=np.array(g.BASE_POS),
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            lin_vel=np.zeros(3),
            ang_vel=np.zeros(3),
            sliders={"lock_btn_left": 0.0, "lock_btn_right": 0.0},
        )
        knob_l = SimpleNamespace(
            pos=np.array(g.START_POS["left"]),
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            lin_vel=np.zeros(3),
            ang_vel=np.zeros(3),
        )
        knob_r = SimpleNamespace(
            pos=np.array(g.START_POS["right"]),
            quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            lin_vel=np.zeros(3),
            ang_vel=np.zeros(3),
        )
        objects = {
            "fixture": fixture_state,
            "knob_left": knob_l,
            "knob_right": knob_r,
        }
        sc = task.score(objects)
        succ = task.success(objects)
        self.assertEqual(sc, 0.0)
        self.assertFalse(succ)

    def test_demo_returns_waypoint_dict(self):
        d = task.demo()
        self.assertIn("left", d)
        self.assertIn("right", d)
        self.assertGreater(len(d["left"]), 5)
        self.assertGreater(len(d["right"]), 5)


if __name__ == "__main__":
    unittest.main()
