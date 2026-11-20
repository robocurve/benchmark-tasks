"""Authored strap-loop layout checks (offline; not physics evidence)."""
import unittest
import numpy as np
from loop_chain import chain
from removal_assets import (loop_polylines, loop_axes, pinch_point, STRAP_SEG, STRAP_W, STRAND_GAP, LOOP_ROOT)
from realistic_meshes_v32 import HEM_R


class LoopGeometryTests(unittest.TestCase):
    def test_each_loop_is_a_closed_u_of_equal_links(self):
        for s, poly in zip((1, -1), loop_polylines()):
            c = chain('loop', poly, segment=STRAP_SEG)
            nodes = np.asarray(c.nodes)
            self.assertEqual(c.segments, 15)
            self.assertTrue(np.allclose(np.linalg.norm(np.diff(nodes, axis=0), axis=1), STRAP_SEG, atol=1e-6))
            # both strand ends finish next to the cut-out, one strand gap apart
            self.assertLess(np.linalg.norm(nodes[-1] - poly[-1]), .001)
            self.assertAlmostEqual(np.linalg.norm(nodes[0, :2] - nodes[-1, :2]), STRAND_GAP, delta=.002)
            cut = np.array([.22, s * HEM_R])
            for end in (nodes[0], nodes[-1]):
                self.assertLess(np.linalg.norm(end[:2] - cut), LOOP_ROOT + STRAND_GAP)

    def test_strap_stands_on_the_mat(self):
        for poly in loop_polylines():
            self.assertTrue(np.allclose(poly[:, 2] - STRAP_W / 2, .750, atol=.001))

    def test_pinch_site_lies_between_the_two_strands(self):
        for s, poly in zip((1, -1), loop_polylines()):
            u, n = loop_axes(s)
            rel = (poly[:, :2] - pinch_point(s)[:2]) @ n[:2]
            self.assertAlmostEqual(rel.max(), STRAND_GAP / 2, delta=.001)
            self.assertAlmostEqual(rel.min(), -STRAND_GAP / 2, delta=.001)
            # the closing direction of a gripper yawed 30 degrees is across the strands
            yaw = np.radians(-30 * s)
            self.assertAlmostEqual(abs(np.dot([np.cos(yaw), np.sin(yaw)], n[:2])), 1., places=6)


if __name__ == '__main__':
    unittest.main()
