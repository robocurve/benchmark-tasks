import unittest

import numpy as np

from gripper_waypoints import gripper_waypoints


class GripperWaypointTests(unittest.TestCase):
    def test_open_close_ramps_preserve_native_targets_and_slew(self):
        right = np.linspace(1.0, 0.60, 9, dtype=np.float32)
        targets = np.column_stack((right[::-1], right))
        waypoints = gripper_waypoints(targets)
        np.testing.assert_array_equal(waypoints.astype(np.float32), targets)
        self.assertLessEqual(np.max(np.abs(np.diff(waypoints, axis=0))), 0.05)

    def test_infeasible_aperture_transition_is_rejected(self):
        with self.assertRaises(ValueError):
            gripper_waypoints(np.array([[1.0], [0.90]], dtype=np.float32))


if __name__ == "__main__":
    unittest.main()
