"""Regression cases for PR #1151's second scoring review (no simulator needed)."""
import math
import unittest

from endpoint_hold import EndpointHold
from fluid import AREA, Fluid
from rack import is_parked


class ScoringRevisionTests(unittest.TestCase):
    origin = (.25, 0., .8875)
    tabletop = .7475

    def purge_with_correction(self, aspirated):
        f = Fluid()
        for _ in range(40):
            f.update(0., 1., .05)
        f.update(1.55 / AREA, 1., .05)
        f.update((1.55 - aspirated) / AREA, 1., .05)
        f.update(1.55 / AREA, 1., .05)
        return f

    def test_corrective_aspiration_and_both_rack_directions_reach_full_score(self):
        for aspirated in (.199, .20, .201, .40, 1.0):
            for direction in (-1, 1):
                with self.subTest(aspirated=aspirated, direction=direction):
                    f = self.purge_with_correction(aspirated)
                    self.assertFalse(f.failed)
                    self.assertAlmostEqual(f.aspirated, aspirated)
                    self.assertAlmostEqual(f.air, 0.)
                    self.assertAlmostEqual(f.lost_sample, .05)
                    parked = is_parked(self.origin, (0., direction, 0.),
                                       (0., 0., 0.), self.origin, self.tabletop)
                    hold = EndpointHold(.6)
                    for _ in range(20):
                        self.assertEqual(hold.step(.05, f.score(True, parked, True)), .6)
                    self.assertEqual(hold.step(.05, f.score(True, parked, True)), 1.)

    def test_unpurged_corrective_air_earns_only_appropriate_partial_credit(self):
        f = self.purge_with_correction(.4)
        for remaining, expected in ((.25, .6), (.50, .3), (.80, 0.)):
            with self.subTest(remaining=remaining):
                f.update((1.55 - remaining) / AREA, 1., .05)
                self.assertFalse(f.failed)
                self.assertEqual(f.score(True, True, True), expected)

    def test_cumulative_repeated_recovery_has_no_hidden_aspiration_cap(self):
        f = self.purge_with_correction(.4)
        for _ in range(20):
            f.update(1.15 / AREA, 1., .05)
            f.update(1.55 / AREA, 1., .05)
        self.assertGreater(f.aspirated, 8.)
        self.assertFalse(f.failed)
        self.assertEqual(f.score(True, True, True), 1.)

    def test_liquid_loss_still_disqualifies_recovered_air(self):
        f = self.purge_with_correction(.4)
        f.update(2.1 / AREA, 1., .05)
        self.assertTrue(f.failed)
        f.update(1.7 / AREA, 1., .05)
        f.update(2.1 / AREA, 1., .05)
        self.assertLessEqual(f.air, .05)
        self.assertEqual(f.score(True, True, True), 0.)

    def test_recovery_still_requires_all_endpoint_gates(self):
        f = self.purge_with_correction(.4)
        for gates in ((False, True, True), (True, False, True), (True, True, False)):
            with self.subTest(gates=gates):
                self.assertEqual(f.score(*gates), 0.)

    def test_invalid_geometry_and_motion_rejected_in_both_directions(self):
        for direction in (-1, 1):
            for position, axis, velocity in (
                ((.276, 0., .8875), (0., direction, 0.), (0., 0., 0.)),
                ((.25, 0., self.tabletop + .156), (0., direction, 0.), (0., 0., 0.)),
                ((.25, 0., self.tabletop + .124), (0., direction, 0.), (0., 0., 0.)),
                (self.origin, (1., 0., 0.), (0., 0., 0.)),
                (self.origin, (0., direction * math.sqrt(1-.16**2), .16), (0., 0., 0.)),
                (self.origin, (0., direction, 0.), (.026, 0., 0.)),
                ((float('nan'), 0., .8875), (0., direction, 0.), (0., 0., 0.)),
            ):
                with self.subTest(direction=direction, position=position, axis=axis, velocity=velocity):
                    self.assertFalse(is_parked(position, axis, velocity, self.origin, self.tabletop))

    def test_volume_thresholds_are_unchanged(self):
        for air, loss, expected in ((.05, .30, 1.), (.05001, .30, .6),
                                    (.05, .30001, .6), (.30, .50, .6),
                                    (.30001, .50, .3), (.75, .50, .3),
                                    (.75001, .50, 0.), (.05, .50001, 0.)):
            with self.subTest(air=air, loss=loss):
                self.assertEqual(Fluid(air=air, lost_sample=loss, aspirated=2.).score(True, True, True), expected)


if __name__ == '__main__':
    unittest.main()
