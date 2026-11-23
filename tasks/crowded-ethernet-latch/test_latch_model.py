import unittest
from latch_model import Geometry,ReleaseHistory


class Tests(unittest.TestCase):
    def setUp(self):
        # Synthetic test geometry; not the eventual connector specification.
        self.g=Geometry(.018,.001,.0012,1000.,.0025)

    def test_pull_without_clip_is_invalid(self):
        h=ReleaseHistory(self.g);h.update(.014,0.)
        self.assertIsNotNone(h.invalid);self.assertFalse(h.disconnected)

    def test_real_release_must_precede_withdrawal(self):
        h=ReleaseHistory(self.g);h.update(.018,.0013);h.update(0.,.0013)
        self.assertIsNone(h.invalid);self.assertTrue(h.disconnected)

    def test_breaking_tab_is_not_a_release_solution(self):
        h=ReleaseHistory(self.g);h.update(.018,.004);h.update(0.,.004)
        self.assertIsNotNone(h.invalid)

    def test_spring_force_has_physical_units(self):
        self.assertAlmostEqual(self.g.restoring_force(.001),1.)

if __name__=='__main__':unittest.main()
