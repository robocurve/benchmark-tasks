import unittest
from types import SimpleNamespace
import numpy as np
from plunger_gauge.scoring import History,evaluate
TOP=.7475
def state(p,size=(.01,.01,.0122),still=True):
 p=np.array(p);h=np.array(size)/2
 return SimpleNamespace(pos=p,low=p-h,high=p+h,resting=lambda:still)
def objects():
 return {'plunger_0':state((.35,.04,TOP+.0295)),'plunger_1':state((.35,.14,TOP+.0295)),'punch':state((.2,.27,TOP+.051),(.026,.026,.082))}
class Scores(unittest.TestCase):
 def test_untested_rest_is_zero(self):self.assertEqual(evaluate(objects(),History(open_clear=True),TOP),0)
 def test_tested_returned_and_tool_parked(self):
  h=History(tested={'plunger_0','plunger_1'},open_clear=True);self.assertEqual(evaluate(objects(),h,TOP),1.)
  h.open_clear=False;self.assertEqual(evaluate(objects(),h,TOP),.8)
  h.tested.remove('plunger_1');self.assertEqual(evaluate(objects(),h,TOP),.4)
 def test_must_hold_stroke_with_tool_provenance(self):
  o=objects();o['plunger_0']=state((.35,.04,TOP+.0219));h=History()
  for _ in range(8):h.update(o,TOP)
  self.assertFalse(h.tested);h.contact('/World/plunger_0','/World/punch')
  for _ in range(5):h.update(o,TOP)
  self.assertFalse(h.tested);h.update(o,TOP);self.assertEqual(h.tested,{'plunger_0'})
 def test_brief_stroke_does_not_accumulate(self):
  o=objects();h=History(touched={'plunger_0'})
  for _ in range(3):
   o['plunger_0']=state((.35,.04,TOP+.0219))
   for _ in range(4):h.update(o,TOP)
   o['plunger_0']=state((.35,.04,TOP+.0295));h.update(o,TOP)
  self.assertFalse(h.tested)
 def test_held_down_is_not_complete(self):
  o=objects();o['plunger_0']=state((.35,.04,TOP+.0219));h=History(tested={'plunger_0','plunger_1'},open_clear=True);self.assertEqual(evaluate(o,h,TOP),.4)
 def test_direct_hand_contact_invalidates(self):
  h=History(tested={'plunger_0','plunger_1'},open_clear=True);h.contact('/World/plunger_1','/World/left_robot/finger');self.assertEqual(evaluate(objects(),h,TOP),0)
class ActualRobotPrimPaths(unittest.TestCase):
    def test_physx_robot_prim_paths_are_rejected(self):
        for arm in ('LeftArm', 'RightArm'):
            history = History()
            history.contact('/World/envs/env_0/plunger_1', f'/World/envs/env_0/{arm}/link_6')
            self.assertTrue(history.invalid, arm)


if __name__=='__main__':unittest.main()
