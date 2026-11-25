import math
import unittest
from wheel_support import clearance,support_gap,polygon_box_gap
import numpy as np
from scipy.spatial.transform import Rotation
import spec


def pose(x,z,y=0.):return [x,y,z,math.sqrt(.5),0.,0.,math.sqrt(.5)]

class WheelSupport(unittest.TestCase):
    def test_floor_sill_and_corner_allow_normal_rolling(self):
        for x,z in [(.455,spec.WHEEL_RADIUS),(.380,spec.WHEEL_RADIUS+spec.SILL_HEIGHT),
                    (.420,spec.SILL_HEIGHT+math.sqrt(spec.WHEEL_RADIUS**2-.030**2))]:
            self.assertLess(clearance(pose(x,z)),.0005)
    def test_carried_wheel_has_visible_gap(self):
        self.assertAlmostEqual(clearance(pose(.550,spec.WHEEL_RADIUS+.020)),.020,places=5)
    def test_one_wheel_support_allows_normal_rocking(self):
        self.assertLess(support_gap({'wheel_left':pose(.55,spec.WHEEL_RADIUS,.1955),
                                    'wheel_right':pose(.55,spec.WHEEL_RADIUS+.010,-.1955)}),.0005)
    def test_tilted_tire_still_has_exact_floor_support(self):
        from wheel_support import tread_points
        for yaw in [0.,30.,80.]:
            rotation=Rotation.from_euler('zyx',[yaw,0.,100.],degrees=True)
            p=[.55,0.,.2,*rotation.as_quat()]
            p[2]-=float(tread_points(p)[:,2].min())
            self.assertLess(clearance(p),1e-10)
    def test_profile_crossing_with_all_vertices_outside(self):
        diamond=np.array([[-2.,0.],[0.,-2.],[2.,0.],[0.,2.]])
        self.assertEqual(polygon_box_gap(diamond,[-3.,-.1],[3.,.1]),0.)
    def test_box_contained_in_tire_profile(self):
        self.assertEqual(polygon_box_gap(np.array([[-2.,0.],[0.,-2.],[2.,0.],[0.,2.]]),[-.1,-.1],[.1,.1]),0.)
    def test_known_gap_to_support_edge(self):
        square=np.array([[0.,2.],[1.,2.],[1.,3.],[0.,3.]])
        self.assertAlmostEqual(polygon_box_gap(square,[0.,0.],[1.,1.]),1.)
    def test_both_wheels_airborne_fail(self):
        self.assertGreater(support_gap({'wheel_left':pose(.55,spec.WHEEL_RADIUS+.01,.1955),
                                       'wheel_right':pose(.55,spec.WHEEL_RADIUS+.02,-.1955)}),spec.WHEEL_SUPPORT_GAP)

if __name__=='__main__':unittest.main()
