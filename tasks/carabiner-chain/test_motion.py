"""Regression for observed grasp release during replanning."""
import unittest,numpy as np
from chain_task.motion import Plan,Target,transition_grip
from yam_arena import ik,station
from yam_arena.constants import READY
class GripContinuityTests(unittest.TestCase):
 def test_contact_blocked_jaws_keep_closure_command_across_long_hold(self):
  p,r=ik.tcp_pose(np.array(READY),station.BASES['right']);old=Plan([Target(1,p,r,0.,'pinch')],'right');targets=[Target(600,p,r,0.,'hold')];new=Plan(targets,'right',grip=transition_grip(old,targets,.64));self.assertEqual(new.action7(.05)[0][6],0.);self.assertEqual(new.action7(30.)[0][6],0.)
 def test_requested_release_opens_and_initial_approach_uses_measured_state(self):
  p,r=ik.tcp_pose(np.array(READY),station.BASES['right']);old=Plan([Target(1,p,r,0.,'pinch')],'right');targets=[Target(2,p,r,1.,'release')];new=Plan(targets,'right',grip=transition_grip(old,targets,.64));self.assertGreater(new.action7(1.)[0][6],.64);self.assertEqual(new.action7(2.)[0][6],1.);self.assertEqual(transition_grip(None,targets,.8),.8)
if __name__=='__main__':unittest.main()
