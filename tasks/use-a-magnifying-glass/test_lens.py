"""CPU checks for the closed optical solid, independent of Isaac startup."""

import math
import unittest
from collections import Counter

from magnifier_workbench.lens import generate_lens, _rotation_quaternion


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


class LensGeometryTests(unittest.TestCase):
    def test_closed_outward_solid_for_candidate_curvatures(self):
        for curvature in (0.060, 0.075, 0.100):
            with self.subTest(curvature=curvature):
                lens = generate_lens(curvature_radius_m=curvature)
                vertices, faces = lens["vertices"], lens["faces"]
                directed = Counter((face[i], face[(i + 1) % 3])
                                   for face in faces for i in range(3))
                self.assertTrue(all(count == 1 for count in directed.values()))
                self.assertTrue(all(directed[(b, a)] == 1 for a, b in directed))
                self.assertEqual(len(vertices) - len(directed) // 2 + len(faces), 2)
                volume = sum(dot(vertices[a], cross(vertices[b], vertices[c]))
                             for a, b, c in faces) / 6
                expected = lens["meta"]["analytic_volume_m3"]
                self.assertGreater(volume, 0)
                self.assertLess(abs(volume - expected) / expected, 0.002)
                for face_index, (a, b, c) in enumerate(faces):
                    ab = tuple(y - x for x, y in zip(vertices[a], vertices[b]))
                    ac = tuple(y - x for x, y in zip(vertices[a], vertices[c]))
                    normal = cross(ab, ac)
                    self.assertGreater(dot(normal, normal), 1e-22)
                    centroid = tuple(sum(vertices[v][i] for v in (a, b, c)) / 3
                                     for i in range(3))
                    self.assertGreater(dot(normal, centroid), 0)
                    for smooth in lens["face_vertex_normals"][face_index * 3:face_index * 3 + 3]:
                        self.assertAlmostEqual(dot(smooth, smooth), 1, places=12)
                        self.assertGreater(dot(normal, smooth), 0)

    def test_dimensions_and_optical_assumptions(self):
        lens = generate_lens()
        meta = lens["meta"]
        self.assertAlmostEqual(max(p[0] for p in lens["vertices"]), 0.03)
        self.assertAlmostEqual(min(p[0] for p in lens["vertices"]), -0.03)
        self.assertAlmostEqual(max(p[2] for p in lens["vertices"]) * 2,
                               meta["center_thickness_m"])
        self.assertAlmostEqual(meta["thin_lens_focal_length_m"], 0.10)
        self.assertGreater(meta["thick_lens_effective_focal_length_m"], 0.10)
        self.assertLess(meta["thick_lens_effective_focal_length_m"], 0.103)
        self.assertIsNone(generate_lens(glass_ior=1)["meta"]["thin_lens_focal_length_m"])

    def test_geometry_inputs_reject_degenerate_or_nonfinite_values(self):
        for kwargs in ({"radius_m": 0}, {"radius_m": float("nan")},
                       {"curvature_radius_m": 0.03}, {"edge_thickness_m": -1},
                       {"rings": 0}, {"rings": 2.2}, {"segments": 3},
                       {"glass_ior": float("nan")}, {"glass_ior": 0.9}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                generate_lens(**kwargs)

    def test_rotation_matrix_and_quaternion_conventions(self):
        self.assertEqual(_rotation_quaternion(None, None), (1, 0, 0, 0))
        self.assertEqual(_rotation_quaternion((2, 0, 0, 0), None), (1, 0, 0, 0))
        q = _rotation_quaternion(None, ((1, 0, 0), (0, 0, -1), (0, 1, 0)))
        self.assertAlmostEqual(q[0], math.sqrt(0.5))
        self.assertAlmostEqual(q[1], math.sqrt(0.5))
        self.assertEqual(_rotation_quaternion(None, ((1, 0, 0), (0, -1, 0), (0, 0, -1))),
                         (0, 1, 0, 0))
        for matrix in (((-1, 0, 0), (0, 1, 0), (0, 0, 1)),
                       ((2, 0, 0), (0, 1, 0), (0, 0, 1))):
            with self.assertRaises(ValueError):
                _rotation_quaternion(None, matrix)


if __name__ == "__main__":
    unittest.main()
