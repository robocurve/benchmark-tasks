"""Fixture geometry and free-fall checks independent of scripted controllers."""

import unittest

import mujoco as mj
import numpy as np

from tea.model import DT, Kitchen
from tea.scene import MUG


class FixtureTests(unittest.TestCase):
    def test_visible_marks_match_the_measured_water_surface(self):
        e = Kitchen()
        for name, dose in (("trigger_mark", 48), ("serving_mark", 120)):
            with self.subTest(name=name):
                for side in range(24):
                    geom = e.m.geom(f"{name}{side}")
                    self.assertAlmostEqual(geom.pos[2], 0.006 + dose * 0.078 / 260)
                self.assertLess(e.m.geom("mug_wall0").rgba[3], 0.5)

    def test_both_stop_faces_align_with_the_carriage_and_mug_axis(self):
        e = Kitchen()
        pos_a = e.d.body("receiving_hand").xpos.copy()
        carriage, stop_a, stop_b = [
            e.m.geom(name)
            for name in ("stage_main_carriage", "stage_stop1", "stage_stop0")
        ]
        x_a = e.d.geom_xpos[carriage.id, 0]
        self.assertAlmostEqual(
            x_a + carriage.size[0], e.d.geom_xpos[stop_a.id, 0] - stop_a.size[0]
        )
        e.cup, e.remaining = 48, 152
        e.move_person(0)
        e.move_person(1)
        mj.mj_forward(e.m, e.d)
        x_b = e.d.geom_xpos[carriage.id, 0]
        self.assertAlmostEqual(
            x_b - carriage.size[0], e.d.geom_xpos[stop_b.id, 0] + stop_b.size[0]
        )
        np.testing.assert_allclose(
            e.d.body("receiving_hand").xpos - pos_a, [-0.083, 0, 0], atol=1e-8
        )
        self.assertAlmostEqual(x_a - x_b, 0.083)
        self.assertIsNone(e.flag_source)

    def test_detached_drop_keeps_falling_when_source_stands_upright(self):
        e = Kitchen()
        drop, velocity, mass = MUG + [0, 0, 0.160], np.array([0.0, 0.0, -0.1]), 0.04
        e.remaining -= mass
        e.packets.append((drop.copy(), velocity.copy(), mass))
        e.flow()
        self.assertEqual(e.rate, 0)
        self.assertEqual(len(e.packets), 1)
        np.testing.assert_allclose(
            e.packets[0][0], drop + velocity * DT + [0, 0, -9.81 * DT**2 / 2]
        )
        for _ in range(100):
            e.flow()
        self.assertEqual(len(e.packets), 0)
        self.assertAlmostEqual(e.cup, mass)
