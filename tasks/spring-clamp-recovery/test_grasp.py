"""Adversarial predicate tests, not simulated trials or native contact validation."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

SOURCE = Path(__file__).resolve().parent / 'grasp.py'
spec = importlib.util.spec_from_file_location('clamp_grasp', SOURCE)
grasp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(grasp)
TIPS = ('/LeftArm/tip_left', '/LeftArm/tip_right')


def contact(path, normal=(1., 0., 0.), impulse=.001, separation=0.):
    return dict(other_path=path, normal=normal, impulse_Ns=impulse, separation_m=separation)


class SpringClampGrasp(unittest.TestCase):
    def setUp(self):
        self.bilateral = [contact(TIPS[0]), contact(TIPS[1], (-1., 0., 0.))]

    def test_solver_contact_gap_boundary(self):
        rows = [contact(TIPS[0], separation=1e-5), self.bilateral[1]]
        self.assertTrue(grasp.bilateral_grasp(rows, TIPS))
        rows[0]['separation_m'] = 1.01e-5
        self.assertFalse(grasp.bilateral_grasp(rows, TIPS))

    def test_bilateral_isolated_pinch(self):
        self.assertTrue(grasp.bilateral_grasp(self.bilateral, TIPS))

    def test_right_hand_near_left_tcp_cannot_score(self):
        rows = [contact('/RightArm/tip_left'), contact('/RightArm/tip_right', (-1., 0., 0.))]
        self.assertFalse(grasp.bilateral_grasp(rows, TIPS))

    def test_single_finger_or_no_contact_cannot_score(self):
        self.assertFalse(grasp.bilateral_grasp(self.bilateral[:1], TIPS))
        self.assertFalse(grasp.bilateral_grasp([], TIPS))

    def test_auxiliary_support_cannot_score_even_with_both_tips(self):
        for actor in ('/RightArm/tip_left', '/table', '/clamp/base', '/LeftArm/gripper', '/unknown'):
            with self.subTest(actor=actor):
                self.assertFalse(grasp.bilateral_grasp(self.bilateral + [contact(actor, impulse=0.)], TIPS))

    def test_proximity_and_unloaded_contact_cannot_score(self):
        for impulse, separation in ((0., 0.), (.001, .0001)):
            with self.subTest(impulse=impulse, separation=separation):
                rows = [contact(TIPS[0], impulse=impulse, separation=separation), self.bilateral[1]]
                self.assertFalse(grasp.bilateral_grasp(rows, TIPS))

    def test_signed_native_impulses_use_magnitude(self):
        rows = [contact(TIPS[0], impulse=-.001), contact(TIPS[1], (-1., 0., 0.), impulse=-.001)]
        self.assertTrue(grasp.bilateral_grasp(rows, TIPS))

    def test_normals_must_oppose(self):
        self.assertFalse(grasp.bilateral_grasp([contact(tip) for tip in TIPS], TIPS))

    def test_invalid_data_raises(self):
        for normal in ((np.nan, 0., 0.), (0., 0., 0.), (1., 0.)):
            with self.assertRaises(ValueError):
                grasp.bilateral_grasp([contact(TIPS[0], normal)], TIPS)


if __name__ == '__main__':
    unittest.main()
