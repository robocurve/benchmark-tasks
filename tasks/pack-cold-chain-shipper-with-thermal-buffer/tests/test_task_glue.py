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
        self.assertGreaterEqual(len(task.OBJECTS), 9)
        names = [o.name for o in task.OBJECTS]
        self.assertIn("shipper_base", names)
        self.assertIn("pad_left", names)
        self.assertIn("pad_right", names)
        self.assertIn("pad_front", names)
        self.assertIn("gel_brick", names)
        self.assertIn("buffer_pad", names)
        self.assertIn("medication_carton", names)
        self.assertIn("shipper_lid", names)
        self.assertIn("pad_lid", names)
        self.assertEqual(len(names), len(set(names)))

    def test_shipper_has_two_spring_latches(self):
        latches = task.SHIPPER_BASE.sliders
        self.assertEqual([s.name for s in latches], ["latch_pos_y", "latch_neg_y"])
        for s, axis in zip(latches, ("+Y", "-Y")):
            self.assertEqual(s.axis, axis)  # each slides outward, away from the lid
            self.assertAlmostEqual(s.rate, g.LATCH_RATE)
            self.assertAlmostEqual(s.travel, g.LATCH_TRAVEL)
            self.assertGreater(s.travel, g.LATCH_OVERLAP)  # the head can clear the lid edge
            task.SHIPPER_BASE._check()
            s._check("shipper_base")

    def test_lid_part_is_a_rigid_slab_with_a_tab(self):
        lid = task.SHIPPER_LID
        lid._check()
        self.assertEqual(lid.sliders, ())  # the lid itself has no hinge or moving part
        slab, tab = lid.boxes[0][1], lid.boxes[1][1]
        for got, want in zip(slab, (g.LID_WIDTH_X, g.LID_LENGTH_Y, g.LID_THICK_Z)):
            self.assertAlmostEqual(got, want, places=6)
        for got, want in zip(tab, (g.LID_TAB_X, g.LID_TAB_Y, g.LID_TAB_Z)):
            self.assertAlmostEqual(got, want, places=6)
        self.assertAlmostEqual(lid.mass, g.LID_MASS)
        # the lid starts on its plate: its underside 0.5 mm above the plate's top face
        self.assertAlmostEqual(lid.pos[2] - g.LID_THICK_Z / 2.0, g.PAD_SURFACE_Z + g.SPAWN_GAP, places=6)

    def test_lid_staging_differs_between_variants(self):
        a, b = task._build_lid("A"), task._build_lid("B")
        self.assertAlmostEqual(a.pos[1], g.LID_PAD_Y_A)
        self.assertAlmostEqual(b.pos[1], g.LID_PAD_Y_B)
        self.assertNotEqual(task._build_lid_pad("A").pos, task._build_lid_pad("B").pos)

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
            "shipper_lid": SimpleNamespace(pos=[g.LID_PAD_X, g.LID_PAD_Y_A, g.LID_STAGING_Z], quat_xyzw=(0, 0, 0, 1), lin_vel=(0, 0, 0)),
            "shipper_base": SimpleNamespace(pos=[g.SHIPPER_X, g.SHIPPER_Y, g.TABLE_Z], sliders={"latch_pos_y": 0.0, "latch_neg_y": 0.0}),
        }
        res1 = task._update(obj1)
        res2 = task._update(obj1)
        self.assertIs(res1, res2)

        # Calling score and success invokes cached _update
        self.assertEqual(task.score(obj1), res1.score)
        self.assertEqual(task.success(obj1), res1.success)


if __name__ == "__main__":
    unittest.main()
