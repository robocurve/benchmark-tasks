"""Numerical assembly criteria and controls."""

import unittest

from scoring import Assembly, LegGauge, base_supported


class RetentionTests(unittest.TestCase):
    def test_base_support_depends_on_final_footprint_height_and_tilt(self):
        def supported(position, upright=1.):
            return base_supported(position, upright, (0.,0.,.72), (1.10,1.30,.06),
                                  .75, .071, .105)

        self.assertTrue(supported((.302,0.,.855)))
        self.assertTrue(supported((.322,.06,.855)))
        self.assertFalse(supported((.49,0.,.855)))
        self.assertFalse(supported((.302,0.,.865)))
        self.assertFalse(supported((.302,0.,.855), .995))

    def test_notch_and_sleeve_are_separate(self):
        self.assertTrue(LegGauge(gap=0.0003, shoulder_overlap=.0025, sleeve_cover=.0025).retained)
        self.assertFalse(LegGauge(gap=.0003, shoulder_overlap=-.001, sleeve_cover=.0025).retained)
        self.assertFalse(LegGauge(gap=.0003, shoulder_overlap=.0025, sleeve_cover=.0005).retained)

    def test_missing_leg_with_covered_sleeves_returns_zero(self):
        assembly = Assembly()
        self.assertEqual(assembly.update(1, False, True, True, True, True), 0.)

    def test_released_stable_assembly_and_revocation(self):
        assembly = Assembly()
        self.assertEqual(assembly.update(1, True, True, True, False, True), .5)
        for tick in range(2,22):
            s = assembly.update(tick, True, True, True, True, True)
        self.assertEqual(s,1.)
        self.assertEqual(assembly.update(22, True, False, True, True, True), 0.)

    def test_missing_or_displaced_shade_cannot_reach_full_score(self):
        trial = Assembly()
        for tick in range(1, 21):
            self.assertEqual(trial.update(tick, True, True, False, True, True), .5)
        for tick in range(21, 41):
            score = trial.update(tick, True, True, True, True, True)
        self.assertEqual(score, 1.)
        self.assertEqual(trial.update(41, True, True, False, True, True), .5)

    def test_transient_invalid_pose_recovers_and_restarts_dwell(self):
        trial = Assembly()
        for tick in range(1, 20):
            self.assertEqual(trial.update(tick, True, True, True, True, True), .5)
        self.assertEqual(trial.update(20, True, True, True, True, False), 0.)
        for tick in range(21, 40):
            self.assertEqual(trial.update(tick, True, True, True, True, True), .5)
        self.assertEqual(trial.update(40, True, True, True, True, True), 1.)

    def test_repeated_steps_cannot_fill_dwell(self):
        trial = Assembly()
        for _ in range(26):
            self.assertEqual(trial.update(1,True,True,True,True,True), .5)


if __name__ == "__main__":
    unittest.main()
