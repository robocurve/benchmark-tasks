import unittest

import numpy as np

from trajectory_control import bounded_joint_pi_step, mesh_reference_step_allowance


class TrajectoryControlTests(unittest.TestCase):
    def test_all_six_joints_use_unbiased_reference_error(self):
        reference = np.array([1, -2, 3, -4, 5, -6]) * 0.0001
        actual = -reference
        result = bounded_joint_pi_step(np.zeros(6), reference, actual, 0.05)
        np.testing.assert_allclose(result, 0.1 * (reference - actual))

    def test_q3_reversal_removes_old_positive_bias(self):
        bias = np.zeros(6)
        bias[2] = 0.002067
        actual = np.zeros(6)
        actual[2] = 0.00524
        result = bounded_joint_pi_step(bias, np.zeros(6), actual, 0.05)
        self.assertAlmostEqual(result[2], bias[2] - 0.00025)
        self.assertLess(result[2], bias[2])

    def test_increment_and_total_caps(self):
        signs = np.array([1, -1, 1, -1, 1, -1])
        result = bounded_joint_pi_step(signs * 0.0199, signs, -signs, 0.1)
        np.testing.assert_allclose(result, signs * 0.02)
        result = bounded_joint_pi_step(np.zeros(6), signs, -signs, 0.1)
        np.testing.assert_allclose(result, signs * 0.00025)

    def test_disabled_and_zero_dt_do_not_alias_or_mutate(self):
        bias = np.full(6, 0.001)
        reference, actual = np.ones(6), np.zeros(6)
        for kwargs in ({"dt": 0.05, "allowed": False}, {"dt": 0}):
            result = bounded_joint_pi_step(bias, reference, actual, **kwargs)
            np.testing.assert_array_equal(result, bias)
            self.assertFalse(np.shares_memory(result, bias))
            result[0] = 42
        np.testing.assert_array_equal(bias, np.full(6, 0.001))
        np.testing.assert_array_equal(reference, np.ones(6))
        np.testing.assert_array_equal(actual, np.zeros(6))

    def test_invalid_joint_inputs_and_dt_rejected_even_when_disabled(self):
        valid = np.zeros(6)
        for allowed in (True, False):
            with self.assertRaises(ValueError):
                bounded_joint_pi_step(np.full(6, 0.020001), valid, valid, 0.05, allowed)
        for invalid in ([0] * 5, np.zeros((6, 1)), [np.nan] * 6, [np.inf] * 6):
            for index in range(3):
                args = [valid, valid, valid]
                args[index] = invalid
                with self.subTest(index=index, invalid=invalid):
                    with self.assertRaises(ValueError):
                        bounded_joint_pi_step(*args, 0.05, allowed=False)
        for dt in (-0.001, 0.10001, np.nan, np.inf, [0.05]):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                bounded_joint_pi_step(valid, valid, valid, dt)

    def test_mesh_allowance_preserves_triangle_slack(self):
        budget, error = 0.0007, 0.0003
        allowance = mesh_reference_step_allowance(budget, error)
        self.assertAlmostEqual(allowance, 0.0002)
        # Even aligned error and reference displacement leave half the slack.
        self.assertLessEqual(error + allowance, budget)
        self.assertAlmostEqual(budget - error - allowance, allowance)
        for budget, error in ((0.0003, 0.0003), (0.0002, 0.0003), (0, 0), (-0.1, 0)):
            self.assertEqual(mesh_reference_step_allowance(budget, error), 0)

    def test_invalid_mesh_inputs(self):
        for budget, error in ((np.nan, 0), (np.inf, 0), (1, np.nan), (1, np.inf), (1, -0.1), ([1], 0), (1, [0])):
            with self.subTest(budget=budget, error=error), self.assertRaises(ValueError):
                mesh_reference_step_allowance(budget, error)


if __name__ == "__main__":
    unittest.main()
