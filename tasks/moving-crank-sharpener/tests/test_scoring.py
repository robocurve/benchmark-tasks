"""Exercise progression filters with measured-pose-shaped synthetic samples."""

import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
from yam_arena.task_file import load


class SharpenerTests(unittest.TestCase):
    def setUp(self):
        task = Path(__file__).resolve().parents[1] / 'scene.py'
        if not task.exists():
            task = Path(__file__).resolve().parents[2] / 'pilots/moving_crank_sharpener.py'
        self.scene = load(task).module
        self.scene.on_reset()
        self.angle = -math.pi / 2

    def frame(self, *, feed=.0033, end=1, aligned=True, cutter_lag=0.):
        scene = self.scene
        tool = np.asarray(scene.TOOL.pos)
        wheel = np.asarray(scene.WHEEL.pos)
        cutter_x = tool[0] + scene.CUTTER_OFFSET + max(0., feed) + cutter_lag
        start_face = tool[0] + scene.CUTTER_OFFSET - scene.CUTTER_HALF_THICKNESS
        pencil = np.array((start_face + feed - .070, tool[1], tool[2]))
        orientation = (0., 0., 0., 1.) if end == 1 else (0., 0., 1., 0.)
        if not aligned:
            orientation = (0., 0., math.sin(.2), math.cos(.2))
        context = {
            'body_origins': {
                'tool': tool,
                'pencil': pencil,
                'cutter': np.array((cutter_x, tool[1], tool[2])),
                'wheel': wheel,
                'knob': wheel + (.046, .035*math.cos(self.angle), .035*math.sin(self.angle)),
            },
        }
        scene.observe({'pencil': SimpleNamespace(quat_xyzw=orientation)}, context, .05)

    def revolve(self, turns, *, advance=False, feed=.0033, end=1, aligned=True):
        for _ in range(round(abs(turns)*80)):
            self.angle -= math.copysign(math.tau/80, turns)
            depth = self.scene._cut_depth_by_end[end] if advance else 0.
            self.frame(feed=feed + depth, end=end, aligned=aligned)

    def test_fixed_insertion_many_turns_below_first_score_tier(self):
        self.revolve(12)
        self.assertLess(self.scene._cut_depth_by_end[1], .006)

    def test_progressive_feed_completes_the_taper(self):
        self.revolve(8, advance=True)
        self.assertAlmostEqual(self.scene._cut_depth_by_end[1], self.scene.CUT_LENGTH)

    def test_the_opposite_end_can_also_finish(self):
        self.frame(feed=-.010, end=-1)
        self.revolve(8, advance=True, end=-1)
        self.assertAlmostEqual(self.scene._cut_depth_by_end[-1], self.scene.CUT_LENGTH)
        self.assertEqual(self.scene._cut_depth_by_end[1], 0.)

    def test_compressed_face_without_tip_contact_does_not_cut(self):
        for _ in range(80):
            self.angle -= math.tau/80
            self.frame(feed=-.002, cutter_lag=.0033)
        self.assertEqual(self.scene._cut_depth_by_end[1], 0.)

    def test_empty_cranking_and_zero_pressure_cannot_cut(self):
        self.revolve(10, feed=-.012)
        self.revolve(10, feed=0.)
        self.assertEqual(self.scene._cut_depth_by_end[1], 0.)

    def test_misalignment_and_overcompression_cannot_cut(self):
        self.revolve(10, aligned=False)
        self.revolve(10, feed=.006)
        self.assertEqual(self.scene._cut_depth_by_end[1], 0.)

    def test_rocking_crank_or_just_pushing_cannot_build_taper(self):
        for _ in range(40):
            self.revolve(.1, advance=True)
            self.revolve(-.1, advance=True)
        self.assertLess(self.scene._cut_depth_by_end[1], .001)
        cut = self.scene._cut_depth_by_end[1]
        self.frame(feed=.020)
        self.assertEqual(self.scene._cut_depth_by_end[1], cut)

    def test_opposite_ends_cut_independently(self):
        self.revolve(4, advance=True)
        self.frame(feed=-.010, end=-1)
        self.revolve(4, advance=True, end=-1)
        depths = self.scene._cut_depth_by_end
        self.assertTrue(all(.009 < depth < .010 for depth in depths.values()))
        self.assertLess(max(depths.values()), self.scene.CUT_LENGTH)

    def test_reset_restores_cut_depth_and_angle(self):
        self.revolve(8, advance=True)
        self.scene.on_reset()
        self.assertEqual(self.scene._cut_depth_by_end, {-1: 0., 1: 0.})
        self.assertEqual(self.scene._loaded_by_end, {-1: 0., 1: 0.})


if __name__ == '__main__':
    unittest.main()
