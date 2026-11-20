import unittest
from moisture import Patch


class Tests(unittest.TestCase):
    def test_hidden_or_no_flow_patch_does_not_magically_dry(self):
        for h,exposed in [(100.,0.),(0.,1.)]:
            p=Patch(.001,.01,.00001,35.);initial=p.water
            for _ in range(100):p.step(.05,65.,h,exposed)
            self.assertEqual(initial,p.water)

    def test_evaporation_is_limited_by_delivered_energy(self):
        p=Patch(.001,.01,.00001,35.)
        removed=p.step(.05,65.,100.,1.)
        self.assertLessEqual(removed,.05*100*.001*30/2_260_000.+1e-15)
        self.assertGreater(removed,0.)

    def test_water_never_becomes_negative(self):
        p=Patch(.001,.01,.000000001,35.)
        for _ in range(100):p.step(.05,65.,1000.,1.)
        self.assertGreaterEqual(p.water,0.)

    def test_bad_exposure_is_rejected(self):
        with self.assertRaises(ValueError):Patch(.001,.01,.01).step(.05,65.,100.,1.5)

if __name__=='__main__':unittest.main()
