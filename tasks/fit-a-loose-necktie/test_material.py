import unittest
import numpy as np
from material import Tie,score
from make_plan import yaw

def hand(point,R,opening=.1):return dict(tcp=np.asarray(point)+.008*R[:,2],rotation=R,opening=opening)

class MaterialTests(unittest.TestCase):
    def captured(self):
        t=Tie();left=t.wide_root+[0,0,-(t.wide_length-.06)];right=t.center+[-(.018+.06*t.slack),0,0]
        h=dict(left=hand(left,yaw(180)),right=hand(right,yaw(0)));t.update(h)
        self.assertEqual({g['part'] for g in t.grips.values()},{'wide','knot'})
        return t,h
    def test_no_contact_does_not_tighten(self):
        t=Tie();h={s:hand([0,0,1.],np.eye(3),1.) for s in ['left','right']}
        for _ in range(120):t.update(h)
        self.assertEqual(t.drawn,0.);self.assertEqual(score(t.measures()),0.)
    def test_one_hand_pull_moves_but_does_not_tighten(self):
        t,h=self.captured();h['right']['opening']=1.
        for _ in range(20):h['left']['tcp'][2]-=.001;t.update(h)
        self.assertAlmostEqual(t.drawn,0.);self.assertLess(t.z,1.09)
    def test_opposing_feed_and_reverse_conserve_material(self):
        t,h=self.captured()
        for _ in range(40):
            h['right']['tcp'][2]+=.001;h['left']['tcp'][2]-=.001;t.update(h)
            self.assertAlmostEqual(t.measures()['material_length_m'],1.26)
        self.assertGreater(t.drawn,.075);before=t.drawn
        for _ in range(20):h['left']['tcp'][2]+=.001;t.update(h)
        self.assertLess(t.drawn,before-.018)
    def test_release_and_fit_required(self):
        t,h=self.captured()
        for _ in range(80):h['right']['tcp'][2]+=.001;h['left']['tcp'][2]-=.00025;t.update(h)
        self.assertEqual(score(t.measures()),0.)
        for x in h.values():x['opening']=1.
        for _ in range(100):t.update(h)
        self.assertEqual(score(t.measures()),1.)
        t.y=.04;self.assertEqual(score(t.measures()),.5)
        t.wide_twist=np.pi;self.assertEqual(score(t.measures()),.5)
    def test_empty_grips_and_teleports_cannot_pull_material(self):
        t,h=self.captured();h['left']['tcp'][2]-=.2;t.update(h)
        self.assertNotIn('left',t.grips);self.assertEqual(t.drawn,0.)
if __name__=='__main__':unittest.main()
