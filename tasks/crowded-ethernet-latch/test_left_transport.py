"""Continuity of the loaded left arm's frozen compensation during transport."""
from unittest.mock import patch
import unittest
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY
from robot import ReleaseProbe,GRIP,PRESS_GRIP
from config import SERVICE,port,CLIP_Z,RELEASE_TRAVEL
from test_robot import environment


class LeftTransportTests(unittest.TestCase):
 def probe(self):
  probe=ReleaseProbe(environment(),np.array(port(SERVICE)));reference=np.array(READY)
  probe.entry_joint_servo.latch(reference);probe.entry_joint_servo.bias[:]=[.001,-.002,.003,.004,-.005,.009]
  probe.entry_joint_servo.freeze();probe.left_unbiased_q=reference.copy()
  probe.previous[:6]=reference+probe.entry_joint_servo.bias;probe.previous[6]=GRIP
  return probe,reference

 def test_zero_distance_handoff_and_new_ik_add_frozen_bias_exactly_once(self):
  probe,reference=self.probe();probe.phase='withdraw_with_clip_pressed'
  position,rotation=ik.tcp_pose(reference,probe.env.rig.bases['left']);right=probe.arm('right')
  target=[position.copy(),rotation];bias=probe.entry_joint_servo.bias.copy()
  def targets(*args):return target[0],target[1],GRIP,right[2],right[3],PRESS_GRIP
  safe={'rigid_gap_m':.01,'other_hand_gap_m':.01,'right_non_target_rigid_gap_m':.01}
  with patch.object(probe,'release_targets',side_effect=targets),patch.object(probe,'stationary_hold_safety',return_value=safe) as safety,patch.object(probe.entry_joint_servo,'update',wraps=probe.entry_joint_servo.update) as update:
   action,_=probe.command(1.)
   np.testing.assert_allclose(action[:6],reference+bias,atol=1e-12)
   target[0]=position+[-.0005,0,0]
   with patch('robot.ik.solve',wraps=ik.solve) as solve:action,_=probe.command(1.05)
   np.testing.assert_array_equal(solve.call_args_list[0].args[2],reference)
   np.testing.assert_allclose(action[:6],probe.left_unbiased_q+bias,atol=1e-12)
   self.assertGreater(np.linalg.norm(probe.left_unbiased_q-reference),1e-4)
   np.testing.assert_array_equal(probe.entry_joint_servo.bias,bias)
   update.assert_not_called();self.assertEqual(safety.call_count,2)

 def test_bias_persists_through_release_and_retreat_until_new_episode(self):
  probe,reference=self.probe();bias=probe.entry_joint_servo.bias.copy()
  p,r=ik.tcp_pose(reference,probe.env.rig.bases['left']);right=probe.arm('right')
  targets=(p,r,GRIP,right[2],right[3],PRESS_GRIP)
  phases=('lift_press_finger','finish_withdrawal','clear_press_finger','align_empty_target','insert_empty_target',
          'retract_for_alignment','seat_and_release_clip','release_boot','retreat_hands','certify_open_clearance',
          'retreat_partially_open','open_clear_hands','retention_rest')
  with patch.object(probe,'release_targets',return_value=targets),patch.object(probe.entry_joint_servo,'update',wraps=probe.entry_joint_servo.update) as update:
   for i,phase in enumerate(phases):
    probe.phase=phase;action,_=probe.command(1.+i*.05)
    np.testing.assert_array_equal(probe.entry_joint_servo.bias,bias)
    np.testing.assert_allclose(action[:6],reference+bias,atol=1e-12)
    self.assertTrue(probe.entry_joint_servo.frozen)
   update.assert_not_called()
  fresh=ReleaseProbe(environment(),np.array(port(SERVICE)))
  self.assertIsNone(fresh.entry_joint_servo.reference);np.testing.assert_array_equal(fresh.entry_joint_servo.bias,np.zeros(6))

 def test_withdrawal_uses_reference_fk_position_and_rotation(self):
  probe,reference=self.probe();probe.phase='depress_clip'
  p,r=ik.tcp_pose(reference,probe.env.rig.bases['left']);probe.boot_goal=p+[.003,.002,.001]
  measured=[probe.arm(s) for s in ('left','right')];plug=np.r_[port(SERVICE),0,0,0,1.]
  travel=.0019
  probe.measured_clip=np.r_[plug[:3]+[0,0,CLIP_Z-travel],0,0,0,1.]
  rightp=plug[:3]+probe.local_contact-[0,0,travel]-probe.pressR@probe.point
  rightq,error,_=ik.solve(rightp,ik.matrix_to_quat_xyzw(probe.pressR),measured[1][0],base_pos=probe.env.rig.bases['right'],iters=350,tol=1e-6)
  self.assertLess(error,1e-6)
  rightp,rightr=ik.tcp_pose(rightq,probe.env.rig.bases['right'])
  measured[1]=(rightq,np.array((-.028,-.028)),rightp,rightr)
  targets=(probe.boot_goal,probe.boot_rotation,GRIP,rightp,probe.pressR,PRESS_GRIP)
  # Isolate transport handoff from trajectory tracking, retaining the real
  # measured-travel counter, actual tab footprint, and reference-FK logic.
  with patch.object(probe,'governed_prepress',return_value=(rightp,rightr,False)) as governor:
   for i in range(4):
    probe.release_targets(1.+.05*i,1.+.05*i,'depress_clip',plug,np.eye(3),travel,measured,targets)
    self.assertEqual(probe.phase,'depress_clip')
   probe.release_targets(1.2,1.2,'depress_clip',plug,np.eye(3),RELEASE_TRAVEL-.0001,measured,targets)
   self.assertEqual(probe.stable_frames,0)
   for i in range(5):
    probe.release_targets(1.25+.05*i,1.25+.05*i,'depress_clip',plug,np.eye(3),travel,measured,targets)
    self.assertEqual(probe.phase,'withdraw_with_clip_pressed' if i==4 else 'depress_clip')
   self.assertEqual(governor.call_count,10)
  self.assertEqual(probe.phase,'withdraw_with_clip_pressed');self.assertTrue(probe.entry_joint_servo.frozen)
  np.testing.assert_allclose(probe.withdraw_start,p);np.testing.assert_allclose(probe.withdraw_rotation,r)
  result=probe.release_targets(1.,0.,probe.phase,plug,np.eye(3),0.,measured,targets)
  np.testing.assert_allclose(result[0],p);np.testing.assert_allclose(result[1],r)

 def test_nonfinite_or_excess_bias_stops_before_new_joint_command(self):
  for bad in (float('nan'),.021):
   probe,reference=self.probe();probe.phase='release_boot';probe.entry_joint_servo.bias[0]=bad
   p,r=ik.tcp_pose(reference,probe.env.rig.bases['left']);right=probe.arm('right')
   with patch.object(probe,'release_targets',return_value=(p,r,GRIP,right[2],right[3],PRESS_GRIP)):
    action,report=probe.command(1.)
   self.assertTrue(np.isfinite(action).all());self.assertIn('left transport bias',report['failure'])


if __name__=='__main__':unittest.main()
