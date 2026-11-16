"""Actual031 housing near-miss, and the unchanged physical-stroke clearance."""
import json
from pathlib import Path
import unittest
import numpy as np
from yam_arena import ik
from yam_arena.urdf import quat_xyzw_to_matrix
from collision_geometry import FingerGeometry, minimum_separation, separating_gap
from config import SERVICE, CLIP_Z
from geometry import object_obstacles

HERE=Path(__file__).parent

class PressHousingClearanceTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.record=json.loads((HERE/'fixtures/clip031-clearance.json').read_text())
  cls.spec=json.loads((HERE/'press_point.json').read_text())
  cls.geometry=FingerGeometry()
  r=cls.record
  cls.plugs={int(k.split('_')[-1]):np.array(v) for k,v in r['plug_poses'].items()}
  clips={int(k.split('_')[-1]):np.array(v) for k,v in r['clip_poses'].items()}
  cls.obstacles=object_obstacles(cls.plugs,clips,include_wires=False)
  cls.rigid=[t for owner,name,t in cls.obstacles if (owner,name) not in ((f'clip_{SERVICE}','tab'),(f'clip_{SERVICE}','hook'))]
  cls.arm=r['right'];cls.pieces=cls.geometry.pieces(cls.arm['actual_finger_joints_m'])
  cls.bodyR=quat_xyzw_to_matrix(cls.plugs[SERVICE][3:])

 def test_recorded_failure_is_housing_not_omitted_gripper_rotation(self):
  p=np.array(self.arm['tcp']);R=np.array(self.arm['bounded_command_fk_rotation'])
  actual=[t@R.T+p for _,t in self.pieces]
  oldgap=minimum_separation(actual,self.rigid)
  self.assertAlmostEqual(oldgap,.00018722851824237718,places=12)
  wall=next(t for owner,name,t in self.obstacles if (owner,name)==('switch','side_0_m'))
  self.assertEqual(self.pieces[58][0],'tip_left')
  self.assertAlmostEqual(separating_gap(actual[58],wall),oldgap,places=12)
  self.assertLess(oldgap,.0002)
  shift=self.bodyR@(np.array(self.spec['contact'])-self.record['old_contact'])
  self.assertGreater(minimum_separation([a+shift for a in actual],self.rigid),.00065)

 def test_whole_gripper_dense_stroke_keeps_physical_depth_and_tab_footprint(self):
  # Same support vertex, wrist rotation, finite real finger positions and live
  # housing/neighbor objects. Only the intended point on the real tab changes.
  old=np.array(self.record['old_contact']);contact=np.array(self.spec['contact'])
  np.testing.assert_allclose(contact-old,[-.0005,0,0],atol=1e-18,rtol=0.)
  self.assertGreater(contact[0],-.0127);self.assertLess(abs(contact[1]),.0022)
  self.assertAlmostEqual(contact[2],CLIP_Z+.0005,places=15)
  point=np.array(self.spec['point_tcp']);point[0]+=self.arm['actual_finger_joints_m'][0]+.028
  R=ik.tcp_pose(self.record['reference_q'],(0,-.305,.75))[1]
  seed=np.array(self.record['reference_q'])
  gaps=[]
  for down in np.linspace(-.0041,.0023,41):
   p=self.plugs[SERVICE][:3]+self.bodyR@(contact-[0,0,down])-R@point
   # Certify each interval's translated swept mesh, not only endpoints.
   if gaps:
    sweep=[np.concatenate((t@R.T+previous,t@R.T+p)) for _,t in self.pieces]
    self.assertGreater(minimum_separation(sweep,self.rigid),.00095)
   gaps.append(minimum_separation([t@R.T+p for _,t in self.pieces],self.rigid))
   seed,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),seed,base_pos=(0,-.305,.75),iters=220,tol=1e-6)
   self.assertLess(pe,1e-6);self.assertLess(re,.001)
   previous=p
  self.assertGreater(min(gaps),.00095)
  witness=self.bodyR.T@(previous+R@point-self.plugs[SERVICE][:3])
  self.assertAlmostEqual(witness[2],CLIP_Z+.0005-.0023,places=12)

if __name__=='__main__':unittest.main()
