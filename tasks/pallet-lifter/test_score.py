import unittest
from types import SimpleNamespace
import numpy as np
from pallet_lifter.scoring import History,evaluate
TOP=.7475
def state(p,size,still=True,quat=(0,0,0,1)):
 p=np.array(p);h=np.array(size)/2
 return SimpleNamespace(pos=p,low=p-h,high=p+h,resting=lambda:still,quat_xyzw=np.array(quat,dtype=float))
def objects():
 return {'pallet':state((.37,-.12,TOP+.0342),(.12,.10,.0384)),'lifter':state((.2575,.28,TOP+.0475),(.305,.08,.025))}
class Scores(unittest.TestCase):
 def test_staging_without_verified_lift_is_zero(self):self.assertEqual(evaluate(objects(),History(open_clear=True),TOP),0)
 def test_complete_and_incomplete_tool_return(self):
  h=History(tool_contact=True,lifted=True,open_clear=True);self.assertEqual(evaluate(objects(),h,TOP),1.)
  h.open_clear=False;self.assertEqual(evaluate(objects(),h,TOP),.75)
 def test_contact_then_clearance_required(self):
  o=objects();o['pallet']=state((.4,.03,TOP+.10),(.12,.1,.0384));h=History();h.update(o,TOP);self.assertFalse(h.lifted)
  h.contact('/World/lifter','/World/pallet');h.update(o,TOP);self.assertTrue(h.lifted);self.assertEqual(evaluate(o,h,TOP),.25)
 def test_direct_robot_contact_invalid(self):
  h=History(lifted=True,open_clear=True);h.contact('/World/pallet','/World/left_robot/finger');self.assertEqual(evaluate(objects(),h,TOP),0)
 def test_table_drop_or_wrong_rotation_is_not_staged(self):
  o=objects();h=History(lifted=True,open_clear=True);o['pallet']=state((.37,-.12,TOP+.0192),(.12,.1,.0384));self.assertEqual(evaluate(o,h,TOP),0)
  o=objects();o['pallet'].quat_xyzw=np.array([0.,0.,.707,.707]);self.assertEqual(evaluate(o,h,TOP),0)
 def test_dropped_tool_invalidates(self):
  o=objects();o['lifter']=state((.2,.2,TOP-.1),(.305,.08,.025));h=History(lifted=True,open_clear=True);h.update(o,TOP);self.assertEqual(evaluate(o,h,TOP),0)
class ActualRobotPrimPaths(unittest.TestCase):
    def test_physx_robot_prim_paths_are_rejected(self):
        for arm in ('LeftArm', 'RightArm'):
            history = History()
            history.contact('/World/envs/env_0/pallet', f'/World/envs/env_0/{arm}/link_6')
            self.assertTrue(history.invalid, arm)


if __name__=='__main__':unittest.main()
