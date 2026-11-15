import unittest
import numpy as np
from functional_staple import Staple


class TestStaple(unittest.TestCase):
    def tool(self):
        R=np.array([[0.,-1,0],[1,0,0],[0,0,1.]])
        return -R@np.array([0,.024,.000025]),R

    def test_requires_entry_then_real_lift(self):
        m=Staple([0,0,0]);p,R=self.tool();m.step(p,R,.01)
        self.assertTrue(m.engaged);self.assertFalse(m.extracted)
        for h in np.linspace(0,.008,81):m.step(p+[0,0,h],R,.01)
        self.assertTrue(m.extracted);self.assertFalse(m.damaged)
        self.assertGreater(m.points()[:,2].min(),.001)

    def test_miss_does_not_extract(self):
        m=Staple([0,0,0]);p,R=self.tool()
        for h in np.linspace(0,.02,81):m.step(p+[.01,0,h],R,.01)
        self.assertFalse(m.engaged);self.assertFalse(m.extracted)

    def test_wedge_slides_under_crown_without_dragging_holes(self):
        m=Staple([0,0,0]);p,R=self.tool();m.step(p,R,.01)
        m.step(p+[.003,0,.001],R,.01)
        self.assertFalse(m.damaged)
        np.testing.assert_allclose(m.crown[:2],[0,0])
        self.assertFalse(m.extracted)

    def test_sideways_pull_damages(self):
        m=Staple([0,0,0]);p,R=self.tool();m.step(p,R,.01)
        m.step(p+[0,.004,.001],R,.01)
        self.assertTrue(m.damaged)


if __name__=='__main__':unittest.main()
