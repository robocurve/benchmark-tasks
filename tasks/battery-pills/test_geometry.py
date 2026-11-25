"""Geometry checks (NumPy only): the convex pieces match the card's drawings, the start layout is collision-free."""
import math
import unittest

import numpy as np

import geometry as G


def radial(pts):
    return np.hypot(pts[:, 0], pts[:, 1]) / G.MM


class Pieces(unittest.TestCase):
    def test_dish_profile(self):
        pcs = G.dish_pieces()
        self.assertEqual(len(pcs), 1 + G.DISH_SEGMENTS)
        allp = np.vstack([p for _, p in pcs])
        self.assertAlmostEqual(allp[:, 2].max() / G.MM, G.DISH_H, places=6)
        self.assertAlmostEqual(allp[:, 2].min(), 0.0, places=9)
        rmax = radial(allp).max() * math.cos(math.pi / G.DISH_SEGMENTS)
        self.assertAlmostEqual(rmax, G.DISH_TOP_R, places=3)
        # inner wall: the card's line from (32, 4) to (57.5, 32)
        self.assertAlmostEqual(float(G.dish_inner_r(G.DISH_FLOOR_TOP)), 32.0)
        self.assertAlmostEqual(float(G.dish_inner_r(G.DISH_H)), 57.5)
        floor = pcs[0][1]
        self.assertAlmostEqual(floor[:, 2].max() / G.MM, G.DISH_FLOOR_TOP)

    def test_jar(self):
        pcs = G.jar_pieces()
        allp = np.vstack([p for _, p in pcs[1:]])
        self.assertAlmostEqual(radial(allp).min(), G.JAR_IN_R, places=6)
        self.assertAlmostEqual(allp[:, 2].max() / G.MM, G.JAR_H)
        self.assertGreaterEqual(G.JAR_IN_R, G.P['holder']['stowed_radius_mm'])

    def test_tray(self):
        pcs = dict(G.tray_pieces())
        allp = np.vstack(list(pcs.values())) / G.MM
        self.assertAlmostEqual(allp[:, 0].max() - allp[:, 0].min(), G.TRAY_X)
        self.assertAlmostEqual(allp[:, 1].max() - allp[:, 1].min(), G.TRAY_Y)
        self.assertAlmostEqual(allp[:, 2].max(), G.TRAY_H)
        self.assertAlmostEqual(pcs['floor'][:, 2].max() / G.MM, G.TRAY_FLOOR)
        inner_x = G.TRAY_X - 2 * G.TRAY_WALL
        self.assertGreaterEqual(inner_x, G.TR['served_outline_mm']['x'])

    def test_prisms(self):
        for r, h in ((G.TAB_R, G.TAB_H), (G.CELL_R, G.CELL_H)):
            p = G.prism_points(r, h)
            rr = radial(p)
            self.assertAlmostEqual(rr.max() * math.cos(math.pi / G.PRISM_SIDES), r, places=6)
            self.assertAlmostEqual((p[:, 2].max() - p[:, 2].min()) / G.MM, h)

    def test_volume(self):
        v = G.convex_volume(G.prism_points(G.TAB_R, G.TAB_H))
        self.assertAlmostEqual(v / (math.pi * (G.TAB_R * G.MM) ** 2 * G.TAB_H * G.MM), 1.0, delta=0.02)


class StartLayout(unittest.TestCase):
    def test_clear_of_each_other(self):
        for seed in range(0, 50):
            lay = G.layout(seed)
            dish, tray, jar = np.array(lay['dish']), np.array(lay['tray']), np.array(lay['jar'])
            # dish rim clear of the tray's near wall (card: >= 30.5 mm), jar clear of the tray
            self.assertGreaterEqual((tray[0] - G.TRAY_X / 2) - (dish[0] + G.DISH_TOP_R), 30.4)
            self.assertGreaterEqual(abs(jar[1]) - G.JAR_OUT_R - (abs(tray[1]) + G.TRAY_Y / 2), 0.0, (seed, jar, tray))
            discs = [(np.array(t[:2]), G.TAB_R) for t in lay['tablets']] + [(np.array(lay['cell'][:2]), G.CELL_R)]
            for i in range(len(discs)):
                for j in range(i + 1, len(discs)):
                    self.assertGreater(np.linalg.norm(discs[i][0] - discs[j][0]) - discs[i][1] - discs[j][1], 8.0)


if __name__ == '__main__':
    unittest.main()
