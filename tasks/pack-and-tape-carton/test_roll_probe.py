"""Read-only ordinary roll-checkpoint protocol and native-force gates."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from roll_probe import opposed_contacts,FAMILY,RollPickupProbe,HOLD,BASES,grip_target,Geometry
from check_roll_probe import nominal_poses

class Tensor:
 def __init__(self,value):self.value=np.asarray(value)
 def detach(self):return self
 def cpu(self):return self
 def numpy(self):return self.value


def contacts(time=0.,force=1.):
 names=sorted(FAMILY);f=np.zeros((2,len(names),3));i=names.index('roll');f[0,i,1]=force;f[1,i,1]=-force
 return dict(time_s=time,body_names=['tip_right','tip_left'],target_names=names,normal_force_matrix_w=f)


class RollProbeTests(unittest.TestCase):
 def test_force_gate_is_name_bound_opposing_coeval_and_finite(self):
  self.assertTrue(opposed_contacts(contacts(),0.)[0]);self.assertFalse(opposed_contacts(contacts(force=0.),0.)[0])
  bad=contacts();bad['normal_force_matrix_w'][1]*=-1;self.assertFalse(opposed_contacts(bad,0.)[0])
  for change in ('stale','nonfinite','missing','duplicate'):
   x=contacts()
   if change=='stale':x['time_s']=.05
   elif change=='nonfinite':x['normal_force_matrix_w'][0,0,0]=np.nan
   elif change=='missing':x['target_names']=x['target_names'][1:]
   else:x['body_names']=['tip_left','tip_left']
   with self.assertRaises(ValueError):opposed_contacts(x,0.)

 def test_geometry_contains_all315_and_core_rim_ik_is_real(self):
  geometry=Geometry();self.assertEqual(len(geometry.raw),315)
  p,R=grip_target(nominal_poses()['roll']);q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),np.array(READY),base_pos=BASES['right'],iters=240,tol=1e-6)
  self.assertLess(pe,1e-6);self.assertLess(re,.001)
  self.assertAlmostEqual((R[:,2]@[0,0,-1]),1.)

 def test_no_contact_never_captures_and_lift_contact_loss_freezes_action(self):
  poses=nominal_poses();p,R=grip_target(poses['roll']);q,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(R),np.array(READY),base_pos=BASES['right'],iters=240,tol=1e-6)
  self.assertLess(pe,1e-6);ready=np.r_[READY,1.,q,HOLD];values={side:Tensor([[*(np.array(READY) if side=='left' else q),FINGER_OPEN if side=='left' else -.0065,FINGER_OPEN if side=='left' else -.0065]]) for side in ('left','right')}
  world=SimpleNamespace(get_pose_w=lambda n:Tensor([[*poses[n][0],*ik.matrix_to_quat_xyzw(poses[n][1])]]))
  env=SimpleNamespace(_ready=Tensor([ready]),_low=np.r_[ik.LOWER,0.,ik.LOWER,0.],_high=np.r_[ik.UPPER,1.,ik.UPPER,1.],_ids={s+'_robot':(list(range(6)),[6,7]) for s in ('left','right')},rig=SimpleNamespace(bases=BASES),_base=SimpleNamespace(arena_world=world,scene={s+'_robot':SimpleNamespace(data=SimpleNamespace(joint_pos=values[s])) for s in ('left','right')}))
  clock=[0.];force=[0.];probe=RollPickupProbe(env,lambda side:contacts(clock[0],force[0]));probe.stage='close_roll';probe.started=0.;probe.origin=poses['roll'];probe.start=(p,R);probe.start_g=HOLD
  # This tests only state-machine contracts; real full315 geometry is covered
  # by the separate nominal36-pose checker, not replaced by this mock.
  with patch.object(probe.geometry,'check',return_value={'safe':True}):
   for i in range(5):
    clock[0]=2.5+.05*i;out,report=probe.command(clock[0]);self.assertIsNone(report['failure'])
   self.assertIsNone(probe.grasp);self.assertEqual(probe.stage,'close_roll')
   force[0]=1.
   for i in range(5):clock[0]=2.8+.05*i;out,report=probe.command(clock[0])
   self.assertEqual(probe.stage,'lift_roll');self.assertIsNotNone(probe.grasp);before=out.copy()
   clock[0]=3.05;force[0]=0.;out,report=probe.command(clock[0])
   self.assertIn('contacts lost',report['failure']);np.testing.assert_array_equal(out,before)
   np.testing.assert_array_equal(out[:7],ready[:7]);self.assertFalse(probe.done)

if __name__=='__main__':unittest.main()
