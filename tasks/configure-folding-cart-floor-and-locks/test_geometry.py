import math
import unittest

import numpy as np

import geometry as geo


class CartGeometryTest(unittest.TestCase):
    def test_folded_sidewalls_define_shared_front_plane(self):
        left, right = (geo.fore_hinge(sign) for sign in (1,-1))
        self.assertAlmostEqual(left[0],right[0])
        for sign in (1,-1):
            rear,mid,front = (fn(sign) for fn in (geo.rear_hinge,geo.middle_hinge,geo.fore_hinge))
            self.assertAlmostEqual(np.linalg.norm(mid-rear),geo.LENGTH)
            self.assertAlmostEqual(np.linalg.norm(front-mid),geo.LENGTH)
            self.assertLess(abs(mid[1]),abs(front[1]))

    def test_captive_slide_goes_from_clear_to_bridging(self):
        trailing, leading = geo.BAR_START + np.array([-1,1])*geo.BAR_LEN/2
        self.assertLess(leading,geo.LENGTH)
        self.assertLess(trailing+geo.BAR_TRAVEL,geo.LENGTH)
        self.assertGreater(leading+geo.BAR_TRAVEL,geo.LENGTH+.075)

    def test_rail_channel_can_slide_but_receives_independent_wall(self):
        rail = geo.folding_side()[4]
        left_side, right_side = geo.slider()[1:3]
        clear_half = abs(left_side[0][1])-left_side[1][1]/2
        self.assertGreater(clear_half,rail[1][1]/2)
        self.assertAlmostEqual(left_side[0][1],-right_side[0][1])
        overlap = geo.BAR_START+geo.BAR_TRAVEL+geo.BAR_LEN/2-geo.LENGTH
        contact_angle = math.degrees(math.asin((clear_half-rail[1][1]/2)/overlap))
        self.assertLess(contact_angle,2.)
