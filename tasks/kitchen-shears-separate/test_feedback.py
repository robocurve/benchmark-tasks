import unittest
import numpy as np
from pivot_feedback import PivotFeedback, rz


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.c=PivotFeedback()
        self.p=np.array([[.255,0.,.748],[.255,0.,.748]])
        self.r=np.array([rz(np.radians(30)),rz(np.radians(-60))])
        self.tcp=np.array([.20,.08,.79])
        self.hand=rz(np.radians(-60))

    def step(self,t):
        return self.c.update(t,self.p,self.r,self.tcp,self.hand)

    def test_yaw_turn_keeps_the_seen_pivot_fixed(self):
        self.r[1]=rz(np.radians(-70))
        self.step(0.)
        local=np.linalg.solve(self.c.rot,self.p[1]-self.c.tcp)
        self.step(.05)
        np.testing.assert_allclose(self.c.tcp+self.c.rot@local,self.p[1],atol=1e-12)

    def test_alignment_does_not_allow_immediate_lift(self):
        for t in np.arange(0,.55,.05):self.step(t)
        self.assertIsNone(self.c.lift_at)
        self.assertEqual(self.c.lift,0.)
        self.step(.55)
        self.assertAlmostEqual(self.c.lift_at,.55)
        self.step(2.55)
        self.assertAlmostEqual(self.c.lift,.010)

    def test_old_failure_angle_aligned_offset_2_48_mm_blocks_lift(self):
        self.p[1,0]+=.00248
        for t in np.arange(0,20,.05):self.step(t)
        self.assertIsNone(self.c.lift_at)
        self.assertEqual(self.c.lift,0.)

    def test_good_interval_resets_after_losing_centering(self):
        for t in np.arange(0,.4,.05):self.step(t)
        self.p[1,0]+=.001
        self.step(.4)
        self.p[1,0]-=.001
        for t in np.arange(.45,.95,.05):self.step(t)
        self.assertIsNone(self.c.lift_at)

    def test_reset_discards_gate_and_lift(self):
        for t in np.arange(0,5,.05):self.step(t)
        self.assertGreater(self.c.lift,0.)
        self.c.reset()
        self.assertIsNone(self.c.lift_at)
        self.assertIsNone(self.c.tcp)
        self.assertEqual(self.c.lift,0.)


if __name__=='__main__':unittest.main()
