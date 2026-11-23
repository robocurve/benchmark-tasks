import unittest
import numpy as np
from config import CELLS,CELL_LENGTH,tape_cell
from seal_geometry import bonded_coverage,strip_component,segment_box,BladeCuts,hinge_points

R=np.array([[1,0,0],[0,0,1],[0,-1,0]])
class MeasuredSealTests(unittest.TestCase):
 def strip(self):
  films={f'tape_{i}':dict(pos=np.array((-.11+(i+.5)*.005,0,.103025)),rotation=R) for i in range(44)}
  bonds={(name,'north_top'):set(range(12)) for name in films}
  return films,bonds
 def test_coverage_follows_current_bonded_material_not_recorded_path(self):
  films,bonds=self.strip()
  seam,ends=bonded_coverage(films,bonds,set(range(44)),np.zeros(3),np.eye(3))
  self.assertEqual(seam,[1.]*44);self.assertEqual(ends,[0.,0.])
  for film in films.values():film['pos'][1]+=.06
  self.assertEqual(bonded_coverage(films,bonds,set(range(44)),np.zeros(3),np.eye(3))[0],[0.]*44)
 def test_unbonded_and_other_component_material_cannot_fill_seam(self):
  films,bonds=self.strip()
  self.assertEqual(bonded_coverage(films,{},set(range(44)),np.zeros(3),np.eye(3))[0],[0.]*44)
  seam,_=bonded_coverage(films,bonds,set(range(1,44)),np.zeros(3),np.eye(3))
  self.assertEqual(seam[0],0.);self.assertEqual(seam[1:], [1.]*43)
 def test_half_width_contact_only_gets_half_width_credit(self):
  films,bonds=self.strip();bonds={key:set(range(6)) for key in bonds}
  self.assertEqual(bonded_coverage(films,bonds,set(range(44)),np.zeros(3),np.eye(3))[0],[.5]*44)
 def test_cut_and_stretched_hinge_break_material_connectivity(self):
  errors={i:0. for i in range(1,CELLS)}
  self.assertEqual(strip_component({12},errors),set(range(12,CELLS)))
  errors[50]=.0003
  self.assertEqual(strip_component({12},errors),set(range(50,CELLS)))
 def test_swept_blade_needs_force_and_whole_width_crossing(self):
  films={}
  for i in range(CELLS):
   p,y=tape_cell(i);films[f'tape_{i}']=dict(pos=np.array(p),rotation=np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]]))
  center=hinge_points(films,36).mean(axis=0)
  def cutter(z):return dict(pos=center-np.array((.062,0,-.016))+np.array((0,0,z)),rotation=np.eye(3))
  cuts=BladeCuts();cuts.step(films,cutter(-.04),{})
  self.assertFalse(cuts.step(films,cutter(.04),{}));self.assertFalse(cuts.cuts)
  forces={('tape_36','cutter'):(0,1,0)}
  cuts.step(films,cutter(-.04),forces)
  self.assertIn(36,cuts.cuts)
 def test_swept_segment_rejects_handle_height(self):
  self.assertTrue(segment_box((0,-.02,0),(0,.02,0),(.007,.00025,.004)))
  self.assertFalse(segment_box((0,-.02,.016),(0,.02,.016),(.007,.00025,.004)))
