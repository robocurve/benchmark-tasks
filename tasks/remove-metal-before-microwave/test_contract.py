import math
import unittest
import spec
from scorer import Evidence, Scorer


def evidence(**kwargs):
    base = dict(
        fork_in_cavity=False, fork_on_safe_spot=True, dish_in_cavity=True,
        button_latched=True, hands_clear=True, stable=True, inside_workspace=True,
        intact=True, fork_xy_err_m=0.01, fork_tilt_rad=0.05, button_travel_m=spec.BUTTON_TRAVEL,
    )
    base.update(kwargs)
    return Evidence(**base)


class Contract(unittest.TestCase):
    def test_aperture(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -.03745)

    def test_ladder(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(fork_on_safe_spot=False, fork_in_cavity=True, button_latched=False)), 0.)
        self.assertEqual(s.update(evidence(button_latched=False, dish_in_cavity=False)), .25)
        self.assertEqual(s.update(evidence(button_latched=False)), .5)
        for _ in range(spec.DWELL_STEPS):
            score = s.update(evidence())
        self.assertEqual(score, 1.)

    def test_metal_start_invalidates(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(fork_in_cavity=True, fork_on_safe_spot=False, button_latched=True)), 0.)
        self.assertTrue(s.invalid)
        self.assertTrue(s.metal_start)


if __name__ == '__main__':
    unittest.main()
