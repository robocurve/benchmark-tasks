"""Counterexamples and conservation checks, independent of scripted waypoints."""
import unittest,json
from pathlib import Path
import numpy as np
from surface_material_models import WetFilm,SpreadFilm

class WetTests(unittest.TestCase):
    def test_dry_same_pose_has_no_wet_success(self):
        m=WetFilm(0)
        for _ in range(20):m.step([.33,.025,.777575,0,0,0,1],.05)
        self.assertEqual(m.metrics()['wet_specimen_fraction'],0)
    def test_hover_does_not_spread_drop(self):
        m=WetFilm();old=m.water.copy()
        for _ in range(20):m.step([.33,.025,.80,0,0,0,1],.05)
        np.testing.assert_array_equal(old,m.water)
    def test_clean_flat_placement_can_succeed_without_angle_rule(self):
        m=WetFilm()
        for _ in range(80):m.step([.33,.025,.777575,0,0,0,1],.05)
        self.assertGreater(m.metrics()['wet_specimen_fraction'],.98)
        self.assertLess(m.metrics()['conservation_error_ul'],1e-9)
    def test_ring_traps_air_and_lift_reopens_vent(self):
        m=WetFilm();r=np.hypot(m.xx,m.yy);ring=(r>4)&(r<6);m.water=ring.astype(float)/ring.sum()*m.initial
        for _ in range(10):m.step([.33,.025,.777575,0,0,0,1],.05)
        self.assertGreater(m.metrics()['largest_trapped_air_mm2'],20)
        m.step([.33,.025,.79,0,0,0,1],.05)
        self.assertEqual(m.metrics()['trapped_area_mm2'],0)

class SpreadTests(unittest.TestCase):
    def test_empty_tool_cannot_paint(self):
        m=SpreadFilm(0)
        for y in np.linspace(-.02,.02,50):m.step([.33,.04,.7475,0,0,0,1],[.33,.04+y-.068,.7555,0,0,0,1],.03,.05)
        self.assertEqual(m.metrics()['wet_fraction'],0)
    def test_no_native_contact_does_not_transport(self):
        m=SpreadFilm();old=m.water.copy()
        for y in np.linspace(-.02,.02,50):m.step([.33,.04,.7475,0,0,0,1],[.33,.04+y-.068,.7555,0,0,0,1],0,.05)
        np.testing.assert_array_equal(old,m.water)
    def test_loaded_touch_without_motion_is_not_a_sweep(self):
        m=SpreadFilm();old=m.water.copy()
        for _ in range(50):m.step([.33,.04,.7475,0,0,0,1],[.33,.04-.068,.7555,0,0,0,1],.03,.05)
        np.testing.assert_array_equal(old,m.water)
    def test_real_motion_transports_finite_liquid(self):
        m=SpreadFilm();old=m.water.copy()
        for y in np.linspace(-.02,.02,100):m.step([.33,.04,.7475,0,0,0,1],[.33,.04+y-.068,.7555,0,0,0,1],.03,.05)
        self.assertGreater(np.abs(m.water-old).sum(),1)
        self.assertLess(m.metrics()['conservation_error_ul'],1e-9)

if __name__=='__main__':unittest.main()
