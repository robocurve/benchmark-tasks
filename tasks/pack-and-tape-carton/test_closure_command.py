"""Exercise a complete closure command with native-shaped state and real guard."""
import unittest
from types import SimpleNamespace
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from robot import ClosureProbe
from test_roll_probe import Tensor
from check_roll_probe import nominal_poses
from roll_probe import BASES
class Tests(unittest.TestCase):
 def test_first_commands_keep_contact_point_and_use_full_geometry(self):
  poses=nominal_poses();ready=np.r_[READY,1.,READY,1.]
  world=SimpleNamespace(get_pose_w=lambda n:Tensor([[*poses[n][0],*ik.matrix_to_quat_xyzw(poses[n][1])]]))
  scene={s+'_robot':SimpleNamespace(data=SimpleNamespace(joint_pos=Tensor([[*READY,FINGER_OPEN,FINGER_OPEN]]))) for s in ('left','right')}
  env=SimpleNamespace(_ready=Tensor([ready]),_low=np.r_[ik.LOWER,0.,ik.LOWER,0.],_high=np.r_[ik.UPPER,1.,ik.UPPER,1.],_ids={s+'_robot':(list(range(6)),[6,7]) for s in ('left','right')},rig=SimpleNamespace(bases=BASES),_base=SimpleNamespace(arena_world=world,scene=scene))
  probe=ClosureProbe(env)
  # Native simulation is absent, but IK and all315 collision pieces are real;
  # no guard or geometric contact point is mocked out of this command call.
  self.assertEqual(len(probe.geometry.raw),315)
  for time in (0.,.05):
   action,report=probe.command(time)
   self.assertIsNone(report['failure']);self.assertEqual(action.shape,(14,));self.assertTrue(np.isfinite(action).all())
   point=report['grip_contact_target'];self.assertEqual(len(point),3);self.assertTrue(np.isfinite(point).all())
if __name__=='__main__':unittest.main()
