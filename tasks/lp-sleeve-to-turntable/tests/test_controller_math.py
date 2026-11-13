"""Correction direction/limits only; no simulated or mocked physics."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('lp_scripted', Path(__file__).resolve().parents[1] / 'scripted.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

class CorrectionTests(unittest.TestCase):
    def test_withdrawal_clears_rim_before_lifting(self):
        for fraction in np.linspace(0., 1., 101):
            distance, rise, opening = module.withdrawal(fraction)
            if rise > 0:
                self.assertGreaterEqual(distance, .030)
            self.assertEqual(opening, 1.)
            self.assertGreaterEqual(rise, 0.)
        self.assertGreater(module.withdrawal(1.)[0], .027)

if __name__ == '__main__':
    unittest.main()
