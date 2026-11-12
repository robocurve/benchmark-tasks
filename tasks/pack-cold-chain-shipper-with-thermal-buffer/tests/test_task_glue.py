"""Unit tests for task glue, object declarations, and scene integration in pack-cold-chain-shipper-with-thermal-buffer."""

from __future__ import annotations

import os
import sys
import unittest
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
TASK_DIR = os.path.dirname(HERE)
if TASK_DIR not in sys.path:
    sys.path.insert(0, TASK_DIR)

if "YAM_ARENA_SRC" in os.environ:
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))
elif os.path.exists(os.path.expanduser("~/Projects/Robocurve/work/refs/yam-isaac-lab-arena")):
    sys.path.insert(0, os.path.expanduser("~/Projects/Robocurve/work/refs/yam-isaac-lab-arena"))

import task
import geometry as g


class TestTaskGlue(unittest.TestCase):
    def test_scene_name(self):
        self.assertEqual(task.SCENE, "station")

    def test_instruction(self):
        self.assertEqual(task.INSTRUCTION, "Pack the medication shipper for transport.")

    def test_objects_declared(self):
        self.assertGreaterEqual(len(task.OBJECTS), 7)
        names = [o.name for o in task.OBJECTS]
        self.assertIn("shipper_base", names)
        self.assertIn("pad_left", names)
        self.assertIn("pad_right", names)
        self.assertIn("pad_front", names)
        self.assertIn("gel_brick", names)
        self.assertIn("buffer_pad", names)
        self.assertIn("medication_carton", names)

    def test_demo_returns_plans(self):
        d = task.demo()
        self.assertIn("left", d)
        self.assertIn("right", d)
        self.assertGreater(len(d["left"]), 0)
        self.assertGreater(len(d["right"]), 0)

    def test_update_caching(self):
        task.reset()
        obj1 = {
            "gel_brick": SimpleNamespace(pos=list(g.HOME_L), quat_xyzw=(0, 0, 0, 1), lin_vel=(0, 0, 0)),
            "buffer_pad": SimpleNamespace(pos=list(g.HOME_L), quat_xyzw=(0, 0, 0, 1), lin_vel=(0, 0, 0)),
            "medication_carton": SimpleNamespace(pos=list(g.HOME_L), quat_xyzw=(0, 0, 0, 1), lin_vel=(0, 0, 0)),
        }
        res1 = task._update(obj1)
        res2 = task._update(obj1)
        self.assertIs(res1, res2)

        # Calling score and success invokes cached _update
        self.assertEqual(task.score(obj1), res1.score)
        self.assertEqual(task.success(obj1), res1.success)


if __name__ == "__main__":
    unittest.main()
