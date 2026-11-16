import unittest
import numpy as np
from config import PRODUCTS,CENTER,BOARD,PRELOADED
from scoring import PackingScorer
from test_scoring import state
class Tests(unittest.TestCase):
 def test_two_items_fully_inside_and_separated_at_reset(self):
  self.assertTrue(PRELOADED);bounds=[];origin=np.array(CENTER)-[0,0,BOARD/2]
  for _,p,size,_,_ in PRODUCTS:
   local=np.array(p)-origin;lo=local-np.array(size)/2;hi=local+np.array(size)/2;self.assertTrue(np.all(lo>=np.array([-.104,-.084,.006])));self.assertTrue(np.all(hi<=np.array([.104,.084,.092])));bounds.append((lo,hi))
  self.assertGreater(bounds[1][0][0],bounds[0][1][0])
 def test_preload_and_initial_closed_flaps_earn_no_credit(self):
  scorer=PackingScorer();s=state();self.assertEqual(scorer.update(s)['score'],0)
  s.update(time=.05,flap_angles=[0]*4,inner_flaps_under_outer=True);self.assertEqual(scorer.update(s)['score'],0)
 def test_progress_requires_connected_actual_tape_bonds(self):
  scorer=PackingScorer();s=state(time=0.,flap_angles=[0]*4,inner_flaps_under_outer=True,payout=.10,ribbon_length=.10,continuous_material=True)
  s['seam_bond'][:11]=[1.]*11;self.assertEqual(scorer.update(s)['score'],.25)
 def test_native_reset_geometry_is_closed_and_untaped(self):
  from carton_scene import layout
  from yam_arena.urdf import quat_xyzw_to_matrix
  parts={p['name']:p for p in layout()[0]}
  for side in ('north','south','east','west'):
   p=parts[side];f=parts[side+'_top'];R=quat_xyzw_to_matrix([*p['quat'][1:],p['quat'][0]]);Q=quat_xyzw_to_matrix([*f['quat'][1:],f['quat'][0]]);np.testing.assert_allclose(R,Q,atol=1e-12)
if __name__=='__main__':unittest.main()
