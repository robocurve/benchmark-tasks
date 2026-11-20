"""Adversarial geometry checks for spill detection in the concave, folded liner."""
import unittest
import numpy as np
from scipy.spatial import ConvexHull
from containment import closed_triangles, contains_points
from evaluate_removal import inside_bag
from realistic_meshes_v32 import dirty_mesh, BODY_R, HEM_R, RIM_Z


class NonConvexContainmentTests(unittest.TestCase):
    def test_outside_fold_is_rejected_despite_inside_convex_hull(self):
        vertices, faces, mouth = dirty_mesh()
        x, y, z = vertices.T
        inner = np.hypot(x, y) < BODY_R + 1e-6
        dent = np.clip(1 - np.abs(y) / .05, 0, 1) * np.clip(1 - np.abs(z - .07) / .05, 0, 1) * inner
        vertices[:, 0] -= .06 * dent * (x > 0)
        point = np.array([[.055, 0, .07]])
        hull = ConvexHull(vertices)
        self.assertTrue(np.all(point @ hull.equations[:, :3].T + hull.equations[:, 3] <= 0))
        self.assertFalse(inside_bag(point, vertices)[0])

    def test_folded_interior_inside_and_above_rim_outside(self):
        v, f, m = dirty_mesh()
        got = inside_bag([[0, 0, .05], [0, 0, RIM_Z - .01], [0, 0, RIM_Z + .06], [HEM_R + .02, 0, .05]], v)
        np.testing.assert_array_equal(got, [True, True, False, False])

    def test_contact_tolerance_has_a_numeric_boundary(self):
        v, f, m = dirty_mesh()
        a = np.pi / 20  # between pleats of the hanging surplus, where its radius is HEM_R
        pts = [[(HEM_R + d) * np.cos(a), (HEM_R + d) * np.sin(a), .03] for d in (.006, .011)]
        np.testing.assert_array_equal(inside_bag(pts, v), [True, False])

    def test_gap_between_liner_and_folded_cuff_is_outside(self):
        v, f, m = dirty_mesh()
        self.assertFalse(inside_bag([[(BODY_R + HEM_R) / 2, 0, .05]], v)[0])

    def test_rigid_transform_invariance(self):
        from scipy.spatial.transform import Rotation
        v, f, m = dirty_mesh()
        p = np.array([[0, 0, .1], [.2, 0, .1]])
        r = Rotation.from_euler('xyz', [.3, .6, -.2]); delta = [.4, -.7, .2]
        np.testing.assert_array_equal(contains_points(p, closed_triangles(v, f, m)),
                                      contains_points(r.apply(p) + delta, closed_triangles(r.apply(v) + delta, f, m)))


if __name__ == '__main__':
    unittest.main()
