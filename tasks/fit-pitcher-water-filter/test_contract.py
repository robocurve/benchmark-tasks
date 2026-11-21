"""Contract tests for pitcher filter scoring ladder."""
import math
import unittest
import spec
from scorer import Evidence, Scorer


def evidence(**kw):
    base = dict(
        cartridge_seated=True, lid_closed=True, hands_clear=True, stable=True,
        inside_workspace=True, intact=True, cart_xy_err_m=.005, cart_z_err_m=.004,
        cart_tilt_rad=.05, lid_angle_rad=.05,
    )
    base.update(kw)
    return Evidence(**base)


class Contract(unittest.TestCase):
    def test_aperture(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -0.03745, places=5)
        with self.assertRaises(ValueError):
            spec.finger_joint_position(.076)

    def test_ladder(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(cartridge_seated=False, lid_closed=False)), 0.)
        self.assertEqual(s.update(evidence(lid_closed=False)), .25)
        self.assertEqual(s.update(evidence()), .5)
        for _ in range(spec.DWELL_STEPS - 1):
            s.update(evidence())
        self.assertEqual(s.update(evidence()), 1.)

    def test_invalid_zeros(self):
        s = Scorer()
        s.update(evidence())
        self.assertEqual(s.update(evidence(inside_workspace=False)), 0.)


if __name__ == '__main__':
    unittest.main()
