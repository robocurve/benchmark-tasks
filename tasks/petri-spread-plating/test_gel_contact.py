"""Damage from any contacting body, excluding safe rim/floor and shallow contact."""
import unittest
import numpy as np
import torch
from scipy.spatial.transform import Rotation
from gel_contact_witness import contact_candidates,apply_events
from surface_material_models import SpreadFilm
from surface_scoring import ScoreState

def native(points,normal,depth,rotation=None):
 r=Rotation.identity() if rotation is None else rotation
 p=torch.tensor(r.apply(np.asarray(points)/1000)+[.33,.04,.7475],dtype=torch.float32)
 n=torch.tensor(r.apply(normal),dtype=torch.float32)
 raw=(torch.ones(len(p)),p,n,-torch.tensor(depth,dtype=torch.float32)/1000,torch.tensor([[len(p)]]),torch.tensor([[0]]))
 base=torch.tensor([.33,.04,.7475,*r.as_quat()],dtype=torch.float32)
 return contact_candidates(raw,base,0,[0])
class GelContactTests(unittest.TestCase):
 def test_foreign_finger_contact_cannot_keep_full_credit(self):
  m=SpreadFilm();m.water[:]=0;m.water[m.domain]=100/m.domain.sum()
  s=ScoreState('spread_plate',[],[],[],m);s.hold=1.;self.assertEqual(s.update(),1.)
  events,stats=native([[2,1,5.7]],[[0,0,1]],[.4]);apply_events(m,events.numpy())
  self.assertEqual(stats[1],1);self.assertEqual(m.metrics()['damage_area_mm2'],1.)
  self.assertEqual(s.update(),0.)
 def test_rim_and_table_support_do_not_damage_gel(self):
  events,stats=native([[44,0,5.7],[2,1,.5]],[[1,0,0],[0,0,-1]],[1.,1.]);m=SpreadFilm();apply_events(m,events.numpy());self.assertEqual(stats[1],0);self.assertEqual(m.metrics()['damage_area_mm2'],0)
 def test_shallow_contact_is_allowed(self):
  events,stats=native([[2,1,5.9]],[[0,0,1]],[.1]);m=SpreadFilm();apply_events(m,events.numpy());self.assertEqual(stats[1],1);self.assertEqual(m.metrics()['damage_area_mm2'],0)
 def test_rotating_dish_preserves_damage_location(self):
  a,_=native([[2,1,5.7]],[[0,0,1]],[.4]);b,_=native([[2,1,5.7]],[[0,0,1]],[.4],Rotation.from_euler('xyz',[30,10,80],degrees=True));np.testing.assert_allclose(a.numpy(),b.numpy(),atol=1e-4)
 def test_damage_is_persistent(self):
  m=SpreadFilm();apply_events(m,[[2,1,.4,0]]);apply_events(m,[]);self.assertEqual(m.metrics()['damage_area_mm2'],1.)
 def test_nonfinite_contact_data_rejected(self):
  events,stats=native([[float('nan'),1,5.7]],[[0,0,1]],[.4]);self.assertGreater(stats[3],0)
if __name__=='__main__':unittest.main()
