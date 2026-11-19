"""Regression for the sixth vise turn crossing quaternion wraparound."""
import math
import unittest
from runtime import advance_angle


class ScrewAngle(unittest.TestCase):
    def test_preserves_preload_across_one_full_turn(self):
        angle=6.16
        new=advance_angle(angle,6.16,6.16-2*math.pi)
        self.assertAlmostEqual(new,angle)
        self.assertGreater(new,6.)

    def test_real_forward_and_reverse_motion_remain_signed(self):
        self.assertAlmostEqual(advance_angle(6.16,6.16,-.08),2*math.pi-.08)
        self.assertAlmostEqual(advance_angle(6.16,6.16,6.13),6.13)


if __name__=='__main__':unittest.main()
