"""Contract tests for leash bolt-snap scoring ladder."""
import unittest
import spec
from scorer import Evidence, Scorer


def evidence(**kw):
    base = dict(
        hooked=True, gate_closed=True, hands_clear=True, stable=True,
        inside_workspace=True, intact=True, hook_xy_err_m=.005, hook_z_err_m=.004,
        gate_open_m=.001,
    )
    base.update(kw)
    return Evidence(**base)


class Contract(unittest.TestCase):
    def test_aperture(self):
        self.assertAlmostEqual(spec.finger_joint_position(.075), -0.03745, places=5)

    def test_ladder(self):
        s = Scorer()
        self.assertEqual(s.update(evidence(hooked=False, gate_closed=False)), 0.)
        self.assertEqual(s.update(evidence(gate_closed=False)), .25)
        self.assertEqual(s.update(evidence()), .5)
        for _ in range(spec.DWELL_STEPS - 1):
            s.update(evidence())
        self.assertEqual(s.update(evidence()), 1.)


if __name__ == '__main__':
    unittest.main()
