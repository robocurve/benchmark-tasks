"""Constitutive invariants, not proof of calibrated paperboard mechanics."""
import sys,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from carton_probe.plasticity import CreaseMemory,STIFFNESS_NM_PER_DEG,YIELD_TORQUE_NM

class MaterialTests(unittest.TestCase):
    def make(self):
        hinge=('fold','base','panel',np.zeros(3),np.array([1.,0,0]))
        q=np.array([[0.,0,0,0,0,0,1],[0.,0,0,0,0,0,1]])
        return CreaseMemory([hinge],['base','panel'],q),q
    def at(self,q,angle):
        p=q.copy();p[1,3:]=Rotation.from_euler('x',angle,degrees=True).as_quat();return p
    def test_elastic_loading_does_not_change_rest(self):
        m,q=self.make()
        for a in [1,1.5,1,0,-1,0]:m.update(self.at(q,a))
        np.testing.assert_allclose(m.rest,[0]);self.assertEqual(m.plastic_work_J,0)
    def test_residual_fold_and_reversal_have_positive_dissipation(self):
        m,q=self.make();m.update(self.at(q,30));self.assertAlmostEqual(m.rest[0],28)
        energy=m.plastic_work_J;m.update(self.at(q,28))
        self.assertEqual(m.plastic_work_J,energy)
        m.update(self.at(q,0));self.assertAlmostEqual(m.rest[0],2)
        self.assertGreater(m.plastic_work_J,energy)
    def test_global_motion_does_not_plastically_fold(self):
        m,q=self.make();r=Rotation.from_euler('xyz',[170,86,95],degrees=True)
        q[:,:3]=r.apply(q[:,:3])+[2,3,4]
        q[:,3:]=(r*Rotation.from_quat(q[:,3:])).as_quat();m.update(q)
        np.testing.assert_allclose(m.rest,[0],atol=1e-10)
    def test_wrap_crossing_has_no_spurious_full_turn(self):
        m,q=self.make()
        for a in np.arange(0,201):m.update(self.at(q,a))
        self.assertAlmostEqual(m.previous[0],200);self.assertAlmostEqual(m.rest[0],198)
        self.assertAlmostEqual(m.plastic_rotation_deg[0],198)
    def test_force_stays_within_yield_for_arbitrary_path(self):
        m,q=self.make();rng=np.random.default_rng(42);angle=0;work=0
        for step in rng.normal(0,5,200):
            angle+=step;m.update(self.at(q,angle))
            self.assertLessEqual(abs(m.previous[0]-m.rest[0])*STIFFNESS_NM_PER_DEG,YIELD_TORQUE_NM+1e-12)
            self.assertGreaterEqual(m.plastic_work_J,work);work=m.plastic_work_J

if __name__=='__main__':unittest.main()
