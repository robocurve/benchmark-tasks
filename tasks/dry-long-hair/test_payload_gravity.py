import unittest
import numpy as np
from yam_arena import ik
from payload_gravity import gravity_torque,target_offset,STIFFNESS
class Tests(unittest.TestCase):
 def test_zero_payload_has_no_effect(self):np.testing.assert_array_equal(target_offset([0,1,1,-.6,0,0],0,[0,0,0],[0,.305,.75]),np.zeros(6))
 def test_virtual_work_matches_held_mass_potential(self):
  q=np.array([-.58,1.47,.45,.69,.96,-.04]);com=np.array([-.09,.003,-.024]);base=[0,.305,.75];mass=.483;tau=gravity_torque(q,mass,com,base);direction=np.array([.1,-.2,.3,-.1,.2,.1]);eps=1e-5
  def potential(x):p,R=ik.tcp_pose(x,base);return mass*9.81*(p+R@com)[2]
  derivative=(potential(q+eps*direction)-potential(q-eps*direction))/(2*eps);self.assertAlmostEqual(float(tau@direction),derivative,places=7);np.testing.assert_allclose(target_offset(q,mass,com,base)*STIFFNESS,tau)
 def test_ramp_starts_exactly_zero(self):np.testing.assert_array_equal(target_offset([0,1,1,-.6,0,0],.483,[0,0,.09],[0,.305,.75],0),np.zeros(6))
if __name__=='__main__':unittest.main()
