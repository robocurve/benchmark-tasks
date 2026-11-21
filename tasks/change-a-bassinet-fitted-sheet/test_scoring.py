import unittest

import numpy as np

from task import CORNER_I, SEAT, X0, X1, Y, ZTOP, Sheet, make_positions, silhouette


class ClothScoringTest(unittest.TestCase):
    def test_seated_clean_pockets_do_not_pin_the_interior(self):
        sheet = Sheet(clean=True)
        sheet.latched[:] = True
        sheet.folded[:] = False
        self.assertEqual(sheet.grip({}, set()), {})

    def test_indirect_outward_strain_can_release_a_fitted_pocket(self):
        sheet = Sheet(clean=True)
        sheet.started = True
        sheet.xyz[:] = make_positions()
        sheet.xyz[CORNER_I[0]] = SEAT[0] + (-.055, -.040, .025)
        sheet.latched[0] = True
        other = Sheet(clean=False)
        sheet.solve({}, other, other.xyz.copy(), .05 / 3)
        self.assertFalse(sheet.latched[0])

    def test_overhead_measurement_uses_triangles_above_the_mattress(self):
        vertices = make_positions()
        bounds = (X0, X1, -Y, Y)
        self.assertEqual(silhouette(vertices, bounds, ZTOP-.002).mean(), 1.)
        vertices[:, 2] -= .02
        self.assertFalse(silhouette(vertices, bounds, ZTOP-.002).any())
