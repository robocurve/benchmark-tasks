import unittest

import numpy as np

from photo_profile import REFERENCE_MM, measure, transform


class PhotoProfileTests(unittest.TestCase):
    def record(self, dip=.8):
        # Synthetic perspective geometry, not a real photograph or task evidence.
        camera = np.array([[40., 4., 600.], [2., -35., 500.], [.003, .007, 1.]])
        pixels = lambda p: transform(camera, p).tolist()
        return {'reference_pixels': pixels(REFERENCE_MM),
                'calibration_steps_pixels': [pixels([[x, z], [x, z+.3]])
                                             for x, z in [(-6, 1), (0, 4), (6, 7)]],
                'pixel_uncertainty': .5,
                'mapping_uncertainty_mm': .05,
                'profiles': [{'name': 'synthetic', 'pixels': pixels([[-1, 4], [0, 4-dip], [1, 4.1]])}]}

    def test_perspective_profile(self):
        result = measure(self.record())
        self.assertTrue(result['calibration_passed'])
        profile = result['profiles'][0]
        self.assertAlmostEqual(profile['dip_mm'], .8)
        self.assertAlmostEqual(profile['ridge_separation_mm'], 2.)
        self.assertEqual(profile['depth_evidence'], 'depth_resolved_at_or_above_threshold')

    def test_shallow_ridge_fails(self):
        self.assertEqual(measure(self.record(.1))['profiles'][0]['depth_evidence'],
                         'depth_resolved_below_threshold')

    def test_boundary_is_unresolved(self):
        self.assertEqual(measure(self.record(.30))['profiles'][0]['depth_evidence'], 'rephotograph')

    def test_bad_independent_calibration_rejects_measurement(self):
        record = self.record()
        record['calibration_steps_pixels'][1][1][1] -= 15
        result = measure(record)
        self.assertFalse(result['calibration_passed'])
        self.assertEqual(result['profiles'][0]['depth_evidence'], 'rephotograph')

    def test_wide_stripe_cannot_certify_small_dip(self):
        record = self.record(.4)
        record['pixel_uncertainty'] = 10
        self.assertEqual(measure(record)['profiles'][0]['depth_evidence'], 'rephotograph')

    def test_collinear_references_rejected(self):
        record = self.record()
        record['reference_pixels'] = [[0, 0], [1, 0], [2, 0], [3, 0]]
        with self.assertRaises(ValueError):
            measure(record)

    def test_reference_step_outside_calibrated_area_rejected(self):
        record = self.record()
        record['calibration_steps_pixels'][0] = [[10000, 10000], [10000, 9990]]
        with self.assertRaises(ValueError):
            measure(record)

    def test_missing_reference_uncertainty_rejected(self):
        record = self.record()
        del record['mapping_uncertainty_mm']
        with self.assertRaises(KeyError):
            measure(record)

    def test_large_mapping_uncertainty_prevents_depth_credit(self):
        record = self.record()
        record['mapping_uncertainty_mm'] = .4
        self.assertEqual(measure(record)['profiles'][0]['depth_evidence'], 'rephotograph')


if __name__ == '__main__':
    unittest.main()
