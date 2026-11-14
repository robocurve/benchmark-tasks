"""Pure-Python regression checks for phase order, volume conservation and scorer bypasses."""
import unittest
from fluid import Fluid, AREA, CAPACITY, INITIAL_AIR, INITIAL_SAMPLE

class FluidChecks(unittest.TestCase):
    def check_mass(self,f):
        self.assertAlmostEqual(f.sample+f.lost_sample,INITIAL_SAMPLE,places=8)
        self.assertAlmostEqual(f.air+f.expelled_air-f.aspirated,INITIAL_AIR,places=8)
        self.assertAlmostEqual(f.air+f.sample,CAPACITY-f.last_q*AREA,places=8)
    def test_no_action_no_credit(self):
        f=Fluid()
        for _ in range(400):f.update(0,0,.05)
        self.assertEqual(f.score(True,True,True),0)
        self.check_mass(f)
    def test_raise_wait_then_purge(self):
        f=Fluid()
        for _ in range(40):f.update(0,.766,.05)
        for k in range(1,41):f.update(1.6/AREA*k/40,.766,.05)
        self.assertAlmostEqual(f.air,0,places=8)
        self.assertAlmostEqual(f.lost_sample,.1,places=8)
        self.assertEqual(f.score(True,True,True),1)
        self.check_mass(f)
    def test_horizontal_push_loses_sample(self):
        f=Fluid()
        for k in range(1,41):f.update(1.6/AREA*k/40,0,.05)
        self.assertTrue(f.failed)
        self.assertEqual(f.score(True,True,True),0)
        self.check_mass(f)
    def test_tiny_retraction_cannot_move_trapped_air_to_outlet(self):
        f=Fluid()
        f.update(.02/AREA,0,.05)
        f.update(0,0,.05)
        self.assertGreater(f.pocket,.024)
        f.update(1.6/AREA,0,.05)
        self.assertGreater(f.air,1.49)
        self.assertEqual(f.score(True,True,True),0)
        self.check_mass(f)
    def test_no_lift_no_release_no_park(self):
        f=Fluid(air=.01,sample=10.4,lost_sample=.1)
        for args in [(False,True,True),(True,False,True),(True,True,False)]:self.assertEqual(f.score(*args),0)
    def test_overpush(self):
        f=Fluid(pocket=0)
        f.update(3./AREA,1,.05)
        self.assertEqual(f.score(True,True,True),0)
        self.check_mass(f)
    def test_retraction_introduces_air(self):
        f=Fluid(pocket=0)
        f.update(1.5/AREA,1,.05);f.update(1./AREA,1,.05)
        self.assertAlmostEqual(f.air,.5)
        self.assertFalse(f.failed)
        self.assertEqual(f.score(True,True,True),.3)
        f.update(1.5/AREA,1,.05)
        self.assertEqual(f.score(True,True,True),1.)
        self.check_mass(f)
    def test_no_teleport_at_tilt(self):
        f=Fluid();f.update(0,1,.05)
        self.assertGreater(f.pocket,0)
        self.assertEqual(f.air,INITIAL_AIR)
    def test_nonfinite_fails_closed(self):
        f=Fluid();f.update(float('nan'),1,.05)
        self.assertTrue(f.failed)

if __name__=='__main__':unittest.main()
