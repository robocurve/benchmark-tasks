import unittest
from types import SimpleNamespace
from adaptive_clip import extend_alignment
class Alignment(unittest.TestCase):
 def test_preserves_release_and_shifts_later_servo(self):
  a=SimpleNamespace(start=1,end=3);b=SimpleNamespace(start=6,end=8)
  p={'frames':[{'action14':[i]*14} for i in range(10)],'clip_servos':[{'window':[1,3]},{'window':[6,8]}]}
  release=p['frames'][3].copy();extend_alignment(p,[('a',a),('b',b)],a,4,extra=2,maximum=12)
  self.assertEqual(p['frames'][5],release);self.assertEqual((a.end,b.start,b.end),(5,8,10));self.assertEqual(p['clip_servos'][1]['window'],[8,10])
  self.assertEqual(p['frames'][3]['action14'],[2]*14)
 def test_budget_failure_does_not_mutate_plan(self):
  a=SimpleNamespace(start=0,end=2);p={'frames':[{},{}]}
  with self.assertRaises(RuntimeError):extend_alignment(p,[('a',a)],a,3,extra=20,maximum=3)
  self.assertEqual(len(p['frames']),2);self.assertEqual(a.end,2)
