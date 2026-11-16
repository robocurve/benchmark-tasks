import unittest
from types import SimpleNamespace as S
import numpy as np
from turnover_effort import read_current,K,D,E
class Tests(unittest.TestCase):
 def make(self):
  ids=[5,3,1,4,2,0];n=8
  d=S(joint_pos=np.arange(n,dtype=float)[None,:]/10,joint_vel=np.zeros((1,n)),gravity_compensation_forces=np.arange(n+6,dtype=float)[None,:],joint_stiffness=np.zeros((1,n)),joint_damping=np.zeros((1,n)),joint_effort_limits=np.zeros((1,n)))
  d.joint_stiffness[0,ids]=K;d.joint_damping[0,ids]=D;d.joint_effort_limits[0,ids]=E
  base=S(scene={s+'_robot':S(data=d,num_joints=n)for s in ('left','right')});env=S(_ids={s+'_robot':(ids,[6,7])for s in ('left','right')})
  return base,env,d,ids
 def test_joint_mapping_and_floating_gravity_are_explicit(self):
  b,e,d,ids=self.make();r=read_current(b,e,.05);self.assertTrue(r['verified']);self.assertEqual(r['sides']['left']['own_gravity_compensation_Nm'],[float(i+6)for i in ids]);self.assertIsNone(r['sides']['left']['applied_torque_Nm'])
 def test_missing_and_mismatched_data_are_not_zero_effort(self):
  b,e,d,ids=self.make();del d.joint_vel;r=read_current(b,e,0.);self.assertFalse(r['verified']);self.assertNotIn('joint_velocity_rad_s',r['sides']['left'])
  b,e,d,ids=self.make();d.joint_stiffness[0,ids[0]]+=1.;self.assertFalse(read_current(b,e,0.)['verified'])
 def test_nonfinite_native_gravity_is_unverified(self):
  b,e,d,ids=self.make();d.gravity_compensation_forces[0,6]=np.nan;self.assertFalse(read_current(b,e,0.)['verified'])
if __name__=='__main__':unittest.main()
