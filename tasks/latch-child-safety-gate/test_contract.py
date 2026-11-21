"""CPU-only contract tests for the child-safety-gate scorer and dimensions."""
import math
import unittest
import spec
from scorer import Evidence, Scorer


def evidence(**kwargs):
    base = dict(
        door_yaw_rad=0.,
        lever_rad=0.,
        base_lock_rad=0.,
        latch_overlap_m=.006,
        door_closed=True,
        lever_locked=True,
        base_locked=True,
        latch_engaged=True,
        hands_clear=True,
        stable=True,
        inside_workspace=True,
        intact=True,
    )
    base.update(kwargs)
    return Evidence(**base)


class Contract(unittest.TestCase):
    def test_aperture_cap(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -.03745)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)

    def test_gate_dimensions_match_easy_step_class(self):
        self.assertAlmostEqual(spec.GATE_HEIGHT, .762)
        self.assertAlmostEqual(spec.DOOR_WIDTH, .406)
        self.assertAlmostEqual(spec.OPENING_WIDTH, .762)
        self.assertGreater(spec.HANDLE_SIZE[1], .05)

    def test_partial_and_full_ladder(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(door_closed=False, lever_locked=False, base_locked=False,
                                           latch_engaged=False, latch_overlap_m=0.)), 0.)
        self.assertEqual(s.update(evidence(lever_locked=False, base_locked=False,
                                           latch_engaged=False, latch_overlap_m=0.)), .25)
        self.assertEqual(s.update(evidence(base_locked=False)), .5)
        for _ in range(spec.DWELL_STEPS):
            score = s.update(evidence())
        self.assertEqual(score, 1.)
        self.assertEqual(s.dwell, spec.DWELL_STEPS)

    def test_invalid_zeros_all_tiers(self):
        s = Scorer()
        s.update(evidence())
        self.assertEqual(s.update(evidence(intact=False)), 0.)
        self.assertTrue(s.invalid)

    def test_reopen_after_release_zeros_trial(self):
        s = Scorer()
        # Closed with hands clear arms the reopen rule.
        self.assertEqual(s.update(evidence()), 1.0 if spec.DWELL_STEPS <= 1 else .5)
        for _ in range(spec.DWELL_STEPS):
            s.update(evidence())
        self.assertEqual(s.score, 1.)
        # Door swings past the reopen limit after release → permanent zero, even if re-closed.
        self.assertEqual(s.update(evidence(door_yaw_rad=-(spec.REOPEN_LIMIT + 0.01),
                                           door_closed=False, lever_locked=False,
                                           base_locked=False, latch_engaged=False,
                                           latch_overlap_m=0.)), 0.)
        self.assertTrue(s.invalid)
        self.assertEqual(s.update(evidence()), 0.)

    def test_release_inside_closed_band_is_not_a_reopen(self):
        s = Scorer()
        near_edge = -(spec.DOOR_CLOSED_TOL - math.radians(.5))
        s.update(evidence(door_yaw_rad=near_edge, lever_locked=False, base_locked=False,
                          latch_engaged=False, latch_overlap_m=0.))
        self.assertFalse(s.invalid)
        drift = -(spec.DOOR_CLOSED_TOL + math.radians(2))
        self.assertEqual(s.update(evidence(door_yaw_rad=drift, door_closed=False, lever_locked=False,
                                           base_locked=False, latch_engaged=False,
                                           latch_overlap_m=0.)), 0.)
        self.assertFalse(s.invalid)

    def test_reopen_limit_is_outside_closed_band(self):
        self.assertGreater(spec.REOPEN_LIMIT, spec.DOOR_CLOSED_TOL)
        self.assertAlmostEqual(spec.REOPEN_LIMIT, math.radians(12))

    def test_open_start_is_not_closed(self):
        self.assertGreater(abs(spec.DOOR_START_YAW - spec.DOOR_CLOSED_YAW), spec.DOOR_CLOSED_TOL)
        self.assertGreater(abs(spec.LEVER_START - spec.LEVER_LOCKED), spec.LEVER_LOCKED_TOL)
        self.assertGreater(abs(spec.BASE_START - spec.BASE_LOCKED), spec.BASE_LOCKED_TOL)
        self.assertAlmostEqual(abs(spec.DOOR_START_YAW), math.radians(35))
        self.assertAlmostEqual(spec.DOOR_CLOSED_TOL, math.radians(7))


if __name__ == '__main__':
    unittest.main()
