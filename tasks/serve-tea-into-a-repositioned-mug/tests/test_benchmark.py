"""Reset and input validation independent of pre-generated evidence."""
import unittest
import numpy as np
from tea.model import Kitchen, PROFILES


class EvidenceTests(unittest.TestCase):
    def test_all_profiles_reset_dry(self):
        self.assertIn("return", PROFILES)
        for name in PROFILES:
            with self.subTest(profile=name):
                k = Kitchen(name)
                s = k.report()
                self.assertEqual(s["on_person_ml"], 0.0)
                self.assertLess(np.linalg.norm(k.d.qvel), 0.02)
                self.assertIsNone(k.onset)
                self.assertTrue(np.isfinite(k.state()).all())

    def test_bad_actions_fail_before_advancing(self):
        k = Kitchen()
        for action in [[0]*13, [float("nan")]*14]:
            with self.assertRaises(ValueError):
                k.action(action)
        self.assertEqual(k.d.time, 0)
