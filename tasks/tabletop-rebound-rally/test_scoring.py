"""Counterexamples for the public rally rules; no simulator required."""
import itertools
import unittest
from rally_task.scoring import RallyState,apply_contacts,score
TABLE=.7475;BOARD=.75
class RallyRules(unittest.TestCase):
 def event(self,s,k,**kwargs):
  p={'table':(.5,.25,TABLE+.02),'paddle':(.35,.25,TABLE+.15),'wall':(.7215,.25,TABLE+.15)}.get(k,(.5,.25,TABLE+.15))
  apply_contacts(s,[k],p,table=TABLE,board=BOARD,**kwargs)
 def completed(self,n):
  s=RallyState();self.event(s,'table');self.event(s,'paddle')
  for _ in range(n):
   for k in ('wall','table','paddle'):self.event(s,k)
  return s
 def test_three_cycles_need_four_held_hits(self):
  s=self.completed(3);self.assertEqual((s.hits,s.cycles,score(s),s.done),(4,3,1.,True))
 def test_partial_cycles(self):
  for n in range(3):self.assertEqual(score(self.completed(n)),n/3)
 def test_double_bounce_ends_and_cannot_recover(self):
  s=self.completed(1);self.event(s,'wall');self.event(s,'table');self.event(s,'table');self.event(s,'paddle')
  self.assertTrue(s.done);self.assertEqual(s.cycles,1)
 def test_unheld_or_handle_hit_erases_partial_credit(self):
  for kw in ({'held':False},{'blade':False},{'robot_contact':True}):
   s=self.completed(1);self.event(s,'wall');self.event(s,'table');self.event(s,'paddle',**kw)
   self.assertTrue(s.invalid);self.assertEqual(score(s),0.)
 def test_simultaneous_contact_order_cannot_create_cycle(self):
  for entries in itertools.permutations(['wall','table','paddle']):
   s=self.completed(1);apply_contacts(s,entries,(.5,.25,TABLE+.02),table=TABLE,board=BOARD)
   self.assertTrue(s.done);self.assertEqual(s.cycles,1)
 def test_robot_contact_has_priority(self):
  s=self.completed(2);apply_contacts(s,['table','other'],(.5,.25,TABLE+.02),table=TABLE,board=BOARD,robot_contact=True)
  self.assertEqual(score(s),0.);self.assertTrue(s.invalid)
 def test_back_and_side_wall_contacts_fail(self):
  for p in ((.779,.25,.9),(.7215,.41,.9),(.7215,.25,1.05)):
   s=RallyState(phase='wall');apply_contacts(s,['wall'],p,table=TABLE,board=BOARD);self.assertTrue(s.done)
 def test_success_is_absorbing(self):
  s=self.completed(3);self.event(s,'other',robot_contact=True);self.assertEqual(score(s),1.)
if __name__=='__main__':unittest.main()
