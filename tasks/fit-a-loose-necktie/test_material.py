import unittest
import numpy as np
from material import Tie,score,LENGTH
from make_plan import yaw

def hand(p,R,opening=.1):return dict(tcp=np.asarray(p)+.008*R[:,2],rotation=R,opening=opening)
class MaterialTests(unittest.TestCase):
 def captured(self):
  t=Tie(threaded=True,wide_base=.25,wrap_base=.32);h=dict(left=hand(t.wide_root+[0,0,-(t.wide_length-.06)],yaw(180)),right=hand(t.center+[-(.018+.06*t.slack),0,0],yaw(0)))
  t.update(h);self.assertEqual({g['part'] for g in t.grips.values()},{'wide','knot'});return t,h
 def test_wide_pull_snugs_but_cannot_raise_knot(self):
  t,h=self.captured()
  for _ in range(50):h['left']['tcp'][2]-=.001;h['right']['tcp'][2]+=.001;t.update(h)
  self.assertLessEqual(t.z,1.09);self.assertGreater(t.drawn,.02);self.assertEqual(score(t.measures()),0.)
 def test_length_and_opposing_force(self):
  t,h=self.captured()
  for _ in range(40):
   h['left']['tcp'][2]-=.001;t.update(h)
   self.assertAlmostEqual(t.measures()['material_length_m'],LENGTH)
   np.testing.assert_allclose(sum(t.reaction_forces.values()),0,atol=1e-9)
  self.assertGreater(t.sliding_work_j,.1)
 def test_no_contact_no_progress(self):
  t=Tie();h={s:hand([0,0,1],np.eye(3),1.) for s in ['left','right']}
  for _ in range(120):t.update(h)
  self.assertEqual(t.drawn,0.);self.assertEqual(score(t.measures()),0.)
 def test_empty_grip_cannot_pull(self):
  t,h=self.captured();h['left']['tcp'][2]-=.2;t.update(h)
  self.assertNotIn('left',t.grips);self.assertEqual(t.drawn,0.)
 def test_release_and_blade_alignment(self):
  t=Tie(threaded=True,wide_base=.25,wrap_base=.32,z=1.17,y=0,drawn=.05,narrow_direction=np.array([0.,0.,-1.]));self.assertEqual(score(t.measures()),0.)
  t.released_s=5;self.assertEqual(score(t.measures()),1.)
  t.narrow_direction=np.array([0.,.8,-.6]);self.assertLess(score(t.measures()),1.)
 def test_fitted_but_unthreaded_is_zero(self):
  t=Tie(threaded=True,wide_base=.25,wrap_base=.32,z=1.17,y=0,drawn=.05,released_s=5)
  m=t.measures();self.assertEqual(score(m),1.);m['threaded']=False
  self.assertEqual(score(m),0.)
 def test_outside_downward_pass_is_not_threading(self):
  from ribbon_thread import LooseBlade
  b=LooseBlade(np.array([.319,.025,1.09]))
  for z in np.linspace(1.20,.81,90):
   b.advance((31,np.array([.249,.025,z])))
  self.assertFalse(b.entered);self.assertFalse(b.exited)
 def test_measurements_are_json_serializable(self):
  import json
  from ribbon_thread import LooseBlade,N
  b=LooseBlade(np.array([.319,.025,1.09]))
  b.advance((N-1,b.centre+[0,0,.04]));b.advance((N-1,b.centre+[0,0,.02]))
  self.assertTrue(b.entered)
  json.dumps({'entered':b.entered,'exited':b.exited})
if __name__=='__main__':unittest.main()
