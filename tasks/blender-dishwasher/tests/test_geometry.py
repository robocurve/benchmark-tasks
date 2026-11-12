import itertools,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from demo_runtime import StateReader

class GeometryTests(unittest.TestCase):
 def reader(self):
  r=StateReader.__new__(StateReader);r._refresh=lambda:None;r._controls_cache=None
  r.specs={'dishwasher_rack':SimpleNamespace(position=(.403,-.245,.8135)),'dishwasher':SimpleNamespace(position=(.650,-.245,.7475))}
  r.vertices={n:np.array(list(itertools.product([-.04,.04],[-.04,.04],[0,.10]))) for n in ['blender_jug','blender_lid','motor_base']}
  self.positions={'blender_jug':np.array([.38,-.18,.8218]),'blender_lid':np.array([.38,-.33,.8218]),'motor_base':np.array([.205,.18,.7475])}
  self.velocities={n:np.zeros(6) for n in r.vertices};self.tcp=np.array([0,0,2.])
  def body(n):
   pts=r.vertices[n]+self.positions[n]
   return dict(points=pts,pose=np.r_[self.positions[n],[0,0,0,1]],velocity=self.velocities[n],rotation=np.eye(3),low=pts.min(0),high=pts.max(0))
  r.body=body;r.arm=lambda side:(None,None,self.tcp,None);return r
 def test_released_supported_parts_eligible(self):
  r=self.reader();d=r.controls();self.assertTrue(all(d['eligible'].values()));self.assertFalse(d['motor_in_dishwasher'])
 def test_hovering_part_not_delivered(self):
  r=self.reader();self.positions['blender_jug'][2]+=.03;self.assertFalse(r.controls()['eligible']['blender_jug'])
 def test_in_gripper_part_not_delivered(self):
  r=self.reader();self.tcp=self.positions['blender_lid']+[0,0,.05];self.assertFalse(r.controls()['eligible']['blender_lid'])
 def test_unstable_part_not_delivered(self):
  r=self.reader();self.velocities['blender_lid'][0]=.1;self.assertFalse(r.controls()['eligible']['blender_lid'])
 def test_edge_overhang_not_delivered(self):
  r=self.reader();self.positions['blender_jug'][0]=.53;self.assertFalse(r.controls()['eligible']['blender_jug'])
 def test_motor_rack_and_cavity_entry(self):
  for pos in [[.403,-.245,.8218],[.650,-.245,.790]]:
   r=self.reader();self.positions['motor_base']=np.array(pos);self.assertTrue(r.controls()['motor_in_dishwasher'])
 def test_motor_flying_above_rack_not_loaded(self):
  r=self.reader();self.positions['motor_base']=np.array([.403,-.245,1.2]);self.assertFalse(r.controls()['motor_in_dishwasher'])

if __name__=='__main__':unittest.main()
