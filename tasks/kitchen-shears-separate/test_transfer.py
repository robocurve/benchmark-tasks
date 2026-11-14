import unittest
import numpy as np
from transfer import Transfer,rz

class TransferTests(unittest.TestCase):
    def make(self):
        t=Transfer(.7475)
        p=np.array([[.255,0,.748],[.255,0,.762]])
        r=np.array([rz(np.radians(36)),rz(np.radians(-54))])
        cmds=[(p[i]+r[i]@[-.082,(-1 if i==0 else 1)*.037,.02],r[i]) for i in range(2)]
        t.begin(20.,p,r,cmds)
        return t,p,r
    def test_lower_command_stays_fixed_until_upper_is_released_and_parked(self):
        t,p,r=self.make();initial=t.target(0,0.)[0]
        for elapsed in np.arange(0,16.99,.05):
            np.testing.assert_allclose(t.target(0,elapsed)[0],initial,atol=1e-12)
    def test_motion_rotations_remain_rigid_and_targets_finite(self):
        t,p,r=self.make()
        for elapsed in np.arange(0,t.duration+.1,.05):
            for i in range(2):
                q,rot,f,_=t.target(i,elapsed)
                self.assertTrue(np.isfinite(q).all())
                np.testing.assert_allclose(rot.T@rot,np.eye(3),atol=1e-9)
                self.assertAlmostEqual(np.linalg.det(rot),1.)
                self.assertGreaterEqual(f,-.012)
                self.assertLessEqual(f,0.)
    def test_final_commands_are_released_and_away_from_tray(self):
        t,p,r=self.make()
        for i,sgn in ((0,-1),(1,1)):
            q,rot,f,_=t.target(i,t.duration+1)
            np.testing.assert_allclose(q,[.17,sgn*.28,.9675])
            self.assertEqual(f,-.012)
    def test_inputs_are_not_modified(self):
        t,p,r=self.make();saved=p.copy()
        for elapsed in np.arange(0,t.duration,.1):t.target(1,elapsed)
        np.testing.assert_array_equal(p,saved)

if __name__=='__main__':unittest.main()
