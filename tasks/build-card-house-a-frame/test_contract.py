"""CPU-only contract tests for the house-of-cards scorer and geometry."""
import math
import unittest
import spec
from scorer import Evidence, Scorer, is_frame


def evidence(**kwargs):
    """A card-house A-frame standing free, as the demo leaves it; override fields to break it."""
    base = dict(
        near_out=True, far_out=True,
        near_held=False, far_held=False,
        near_on_stand=False, far_on_stand=False,
        near_lean_deg=15., far_lean_deg=15.,
        near_low_m=.0004, far_low_m=.0004,
        apex_gap_m=.0008, feet_gap_m=.046, facing=True,
        inside=True, intact=True,
    )
    base.update(kwargs)
    return Evidence(**base)


START = dict(near_out=False, far_out=False, near_lean_deg=3., far_lean_deg=3., near_low_m=.005, far_low_m=.005,
             apex_gap_m=.236, feet_gap_m=.300)
HELD = dict(near_held=True, far_held=True)


def run(s, rows):
    score = 0.
    for row in rows:
        score = s.update(row)
    return score


class Contract(unittest.TestCase):
    def test_aperture_cap(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -.03745)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)

    def test_start_pose_sits_in_the_slot(self):
        for side in spec.STAND_Y:
            (x, y, z), q = spec.start_pose(side)
            self.assertAlmostEqual(y, spec.STAND_Y[side])
            bottom = z - spec.CARD_SKIN_H / 2 * math.cos(math.radians(spec.START_TILT_DEG))
            self.assertGreater(bottom, spec.STAND_TOP_Z - spec.SLOT_DEPTH - .001)
            self.assertLess(abs(x - spec.STAND_X), spec.SLOT_W / 2 + spec.CARD_H / 2 * math.sin(math.radians(spec.START_TILT_DEG)))
        self.assertLess(spec.CARD_SKIN_T, spec.SLOT_W)

    def test_demo_frame_geometry_is_a_frame(self):
        (xn, yn, zn), _ = spec.lean_pose('near')
        (xf, yf, zf), _ = spec.lean_pose('far')
        self.assertLess(xn, spec.RIDGE_X)
        self.assertGreater(xf, spec.RIDGE_X)
        self.assertAlmostEqual(zn, zf)
        feet = 2 * spec.CARD_SKIN_H * math.sin(math.radians(spec.LEAN_DEG))
        self.assertGreater(feet, spec.FEET_GAP)
        # Each card's top edge overlaps the other's by more than half a card width.
        self.assertGreater(spec.CARD_W - 2 * spec.STAGGER, spec.CARD_W / 2)

    def test_start_scores_zero(self):
        s = Scorer()
        self.assertEqual(run(s, [evidence(**START)] * 200), 0.)
        self.assertFalse(s.invalid)

    def test_ladder(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(**START, **HELD)), 0.)
        self.assertEqual(s.update(evidence(**HELD, apex_gap_m=.03, near_low_m=.01, far_low_m=.01)), .25)
        self.assertEqual(s.update(evidence(**HELD)), .5)
        self.assertEqual(run(s, [evidence()] * (spec.STAND_STEPS - 1)), .5)
        self.assertEqual(s.update(evidence()), 1.)

    def test_lifting_one_card_at_a_time_is_not_the_first_rung(self):
        s = Scorer()
        rows = [evidence(**{**START, 'near_held': True, 'near_out': True}),
                evidence(**{**START, 'far_held': True, 'far_out': True})]
        self.assertEqual(run(s, rows), 0.)

    def test_standing_count_restarts_on_any_touch(self):
        s = Scorer()
        s.update(evidence(**HELD))
        run(s, [evidence()] * (spec.STAND_STEPS - 1))
        self.assertEqual(s.update(evidence(near_held=True)), .5)
        self.assertEqual(s.standing, 0)
        self.assertEqual(run(s, [evidence()] * (spec.STAND_STEPS - 1)), .5)
        self.assertEqual(s.update(evidence()), 1.)

    def test_held_frame_never_reaches_full(self):
        s = Scorer()
        self.assertEqual(run(s, [evidence(**HELD)] * 400), .5)
        self.assertEqual(run(Scorer(), [evidence(near_held=True)] * 400), .5)

    def test_not_a_frame(self):
        cases = {
            'stacked face to face': dict(feet_gap_m=.001, apex_gap_m=0.),
            'leaning the same way': dict(facing=False),
            'tops apart': dict(apex_gap_m=.006),
            'feet too close': dict(feet_gap_m=.019),
            'off the felt': dict(near_low_m=.003),
            'too upright': dict(near_lean_deg=9.),
            'too flat': dict(far_lean_deg=36.),
            'propped on a stand': dict(far_on_stand=True),
        }
        self.assertTrue(is_frame(evidence()))
        for name, change in cases.items():
            with self.subTest(name):
                self.assertFalse(is_frame(evidence(**change)))
                s = Scorer()
                self.assertEqual(run(s, [evidence(**change)] * 300), 0.)

    def test_fallen_cards_keep_the_partial_score(self):
        s = Scorer()
        s.update(evidence(**HELD))
        self.assertEqual(run(s, [evidence(near_lean_deg=90., far_lean_deg=90., apex_gap_m=.06)] * 200), .5)

    def test_forbidden_events_zero_the_trial_for_good(self):
        for change in (dict(inside=False), dict(intact=False), dict(apex_gap_m=float('nan'))):
            s = Scorer()
            s.update(evidence(**HELD))
            self.assertEqual(s.update(evidence(**change)), 0.)
            self.assertTrue(s.invalid)
            self.assertEqual(run(s, [evidence()] * 300), 0.)


if __name__ == '__main__':
    unittest.main()
