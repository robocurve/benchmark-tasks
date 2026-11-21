import math
import unittest

import numpy as np

from egg_state import Egg, LIQUID_Z, Trial, fork_outside_cavity


def puncture(egg):
    for z in (0.040, 0.026, 0.021, 0.015, 0.009):
        egg.stir(np.array([0.0, 0.0, z]))


def beat(egg):
    puncture(egg)
    for angle in np.linspace(0, 16 * math.pi, 8 * 60):
        egg.stir(np.array([0.032 * math.cos(angle), 0.032 * math.sin(angle), 0.010]))


class MixingTest(unittest.TestCase):
    def test_puncturing_and_vertical_jabs_do_not_mix(self):
        egg = Egg()
        puncture(egg)
        self.assertTrue(egg.membrane_open)
        for _ in range(40):
            egg.stir(np.array([0.0, 0.0, 0.018]))
            egg.stir(np.array([0.0, 0.0, 0.009]))
        self.assertFalse(egg.mixed)

    def test_sweeps_actually_mix_and_conserve_yolk(self):
        egg = Egg()
        initial = egg.yolk.sum()
        beat(egg)
        self.assertTrue(egg.mixed)
        self.assertAlmostEqual(egg.yolk.sum(), initial)
        self.assertTrue(((egg.yolk >= 0) & (egg.yolk <= 1)).all())

    def test_figure_eight_sweeps_can_mix(self):
        egg = Egg()
        puncture(egg)
        for angle in np.linspace(0, 16 * math.pi, 480):
            egg.stir(np.array([0.037 * math.cos(angle), 0.034 * math.sin(2 * angle), 0.010]))
        self.assertTrue(egg.mixed)

    def test_holding_still_or_moving_above_liquid_does_not_mix(self):
        egg = Egg()
        puncture(egg)
        before = egg.yolk.copy()
        for _ in range(40):
            egg.stir(np.array([0.0, 0.0, 0.009]))
        egg.stir(np.array([0.0, 0.0, 0.030]))
        for angle in np.linspace(0, 16 * math.pi, 8 * 60):
            egg.stir(np.array([0.032 * math.cos(angle), 0.032 * math.sin(angle), 0.030]))
        np.testing.assert_array_equal(before, egg.yolk)

    def test_contact_does_not_bridge_an_unobserved_tool_motion(self):
        egg = Egg()
        for _ in range(40):
            egg.stir(np.array([0.0, 0.0, 0.012]))
            egg.stir(None)
        self.assertFalse(egg.membrane_open)

    def test_circles_skimming_the_surface_do_not_mix(self):
        for clearance in (0, 0.0001, 0.0025):
            with self.subTest(clearance=clearance):
                egg = Egg()
                puncture(egg)
                egg.stir(None)
                before = egg.yolk.copy()
                for angle in np.arange(1800) * math.tau / 48:
                    egg.stir(np.array([0.032 * math.cos(angle), 0.032 * math.sin(angle), LIQUID_Z + clearance]))
                np.testing.assert_array_equal(egg.yolk, before)
                self.assertFalse(egg.mixed)

    def test_shallow_grazing_is_weaker_than_immersed_sweeps(self):
        for depth, expected in ((0.00005, False), (0.003, True)):
            with self.subTest(depth=depth):
                egg = Egg()
                puncture(egg)
                egg.stir(None)
                for angle in np.arange(1800) * math.tau / 48:
                    egg.stir(np.array([0.032 * math.cos(angle), 0.032 * math.sin(angle), LIQUID_Z - depth]))
                self.assertEqual(egg.mixed, expected)


class CavityTest(unittest.TestCase):
    def outside(self, centre, angle=0):
        c, s = math.cos(angle), math.sin(angle)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        return fork_outside_cavity(np.array(centre), rotation, np.array([[0., 0., 0.]]),
                                   np.array([[0.09, 0.002, 0.002]]), np.zeros(3), np.full(3, 0.2))

    def test_fork_held_outside_and_fork_above_housing_are_clear(self):
        self.assertTrue(self.outside((-0.10, 0.1, 0.1)))
        self.assertTrue(self.outside((0.1, 0.1, 0.25)))

    def test_tip_and_middle_intersections_count(self):
        self.assertFalse(self.outside((-0.089, 0.1, 0.1)))
        self.assertFalse(self.outside((0., 0., 0.1), -math.pi / 4))

    def test_diagonal_fork_can_pass_close_to_corner(self):
        self.assertTrue(self.outside((-0.02, -0.02, 0.1), -math.pi / 4))


class ScoreTest(unittest.TestCase):
    def setUp(self):
        self.trial = Trial()
        self.loaded = dict(closed=True, seated=True, fork_outside=True)

    def press(self, **changes):
        self.trial.update(pressed=True, spill=False, **(self.loaded | changes))

    def score(self, **changes):
        return self.trial.score(**(self.loaded | {"bowl_on_counter": False} | changes))

    def test_hold_is_zero_and_beating_without_start_is_partial(self):
        self.assertEqual(self.score(), 0.0)
        beat(self.trial.egg)
        self.assertEqual(self.score(), 0.5)
        self.assertEqual(self.score(seated=False, bowl_on_counter=True), 0.5)
        self.assertEqual(self.score(seated=False), 0.0)

    def test_safe_and_complete_start_scores_one(self):
        beat(self.trial.egg)
        self.press()
        self.assertEqual(self.score(), 1.0)

    def test_early_start_is_a_permanent_failure(self):
        self.press()
        beat(self.trial.egg)
        self.trial.update(pressed=False, spill=False, **self.loaded)
        self.press()
        self.assertTrue(self.trial.failed)
        self.assertEqual(self.score(), 0.0)

    def test_punctured_egg_is_still_unsafe_to_start(self):
        puncture(self.trial.egg)
        self.press()
        self.assertTrue(self.trial.failed)
        self.assertEqual(self.score(), 0.0)

    def test_local_reciprocation_cannot_prepare_the_egg(self):
        for stroke_width in (0.0005, 0.010):
            with self.subTest(stroke_width=stroke_width):
                self.trial = Trial()
                egg = self.trial.egg
                puncture(egg)
                for step in range(1800):
                    x = stroke_width / 2 if step % 2 else -stroke_width / 2
                    egg.stir(np.array([x, 0.0, 0.009]))
                    self.assertFalse(egg.mixed)
                self.assertTrue(egg.membrane_open)
                self.assertEqual(self.score(), 0.0)
                self.press()
                self.assertTrue(self.trial.failed)
                self.assertEqual(self.score(), 0.0)

    def test_open_door_interlock_and_new_press(self):
        self.press(closed=False)
        self.assertFalse(self.trial.started)
        self.assertFalse(self.trial.failed)
        beat(self.trial.egg)
        self.press()
        self.assertEqual(self.score(), 0.5)  # a held button is not a second press
        self.trial.update(pressed=False, spill=False, **self.loaded)
        self.press()
        self.assertEqual(self.score(), 1.0)

    def test_start_with_fork_in_cavity_or_missing_bowl_is_zero(self):
        for unsafe in ({"fork_outside": False}, {"seated": False}):
            with self.subTest(unsafe=unsafe):
                self.trial = Trial()
                beat(self.trial.egg)
                self.press(**unsafe)
                self.assertTrue(self.trial.failed)
                self.assertEqual(self.score(), 0.0)

    def test_fork_in_cavity_at_end_blocks_credit(self):
        beat(self.trial.egg)
        self.assertEqual(self.score(fork_outside=False), 0.0)
        self.press()
        self.assertEqual(self.score(fork_outside=False), 0.0)

    def test_spill_invalidates_an_otherwise_complete_trial(self):
        beat(self.trial.egg)
        self.press()
        self.trial.update(pressed=False, spill=True, **self.loaded)
        self.assertEqual(self.score(), 0.0)


if __name__ == "__main__":
    unittest.main()
