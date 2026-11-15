import unittest
import numpy as np
from functional_pegs import Peg, ry


class PegFailureTests(unittest.TestCase):
    def hands(self, peg, gap, displacement=(0.,0.,0.)):
        return [('right', peg.p+[-.018,0.,0.]+np.asarray(displacement), ry(np.pi/2), gap)]

    def test_only_material_in_the_jaw_is_retained_and_opening_releases(self):
        peg=Peg((.49,.10,1.09),0.)
        points=np.array([[.52,.10,1.09],[.52,.14,1.09],[.56,.10,1.09]])
        peg.update(self.hands(peg,.014),points,.004)
        peg.update(self.hands(peg,.030),points,.004)
        self.assertTrue(peg.clipped);self.assertEqual(peg.indices,[0])
        peg.update(self.hands(peg,.014),points,.004)
        self.assertFalse(peg.clipped);self.assertEqual(peg.indices,[])

    def test_a_missed_handle_does_not_carry_the_peg(self):
        peg=Peg((.49,.10,1.09),0.);before=peg.p.copy()
        peg.update(self.hands(peg,.014,(0.,.05,0.)),np.empty((0,3)),.004)
        self.assertIsNone(peg.held);np.testing.assert_array_equal(peg.p,before)

    def test_closing_away_from_line_does_not_fasten(self):
        peg=Peg((.40,.10,1.09),0.)
        points=np.array([[.43,.10,1.09]])
        peg.update(self.hands(peg,.014),points,.004)
        peg.update(self.hands(peg,.030),points,.004)
        self.assertFalse(peg.clipped);self.assertEqual(peg.indices,[])


if __name__=='__main__':unittest.main()
