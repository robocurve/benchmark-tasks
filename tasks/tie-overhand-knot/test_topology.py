"""Independent geometric fixtures for the conservative open-rope diagnostic.

These tests exercise numerical topology only.  Parametric curves are deliberately
not claimed as physically tied robot demonstrations.
"""

import json
import unittest
from unittest.mock import patch

import numpy as np

import topology
from topology import analyze_open_rope


def open_torus(p=2, q=3, samples=200, cut=0.12, tails=True):
    """T(p,q), cut near its outermost x point, with unthreaded exterior tails."""
    t = np.linspace(cut, 2*np.pi-cut, samples)
    curve = np.column_stack([
        (2+np.cos(q*t))*np.cos(p*t),
        (2+np.cos(q*t))*np.sin(p*t), np.sin(q*t),
    ])
    if tails:
        curve = np.vstack([
            np.linspace(curve[0]+[4., 0., 0.], curve[0], 24)[:-1],
            curve,
            np.linspace(curve[-1], curve[-1]+[4., 0., 0.], 24)[1:],
        ])
    return curve*0.03


class TopologyTests(unittest.TestCase):
    def test_open_trefoil_and_mirror(self):
        for mirror in (1, -1):
            points = open_torus()*[1, 1, mirror]
            rest = np.linalg.norm(np.diff(points, axis=0), axis=1)
            result = analyze_open_rope(points, .002, expected_segment_lengths=rest)
            self.assertEqual(result["status"], "overhand_candidate")
            self.assertIs(result["is_overhand"], True)
            self.assertEqual(result["observed_determinants"], [3])
            self.assertGreaterEqual(result["valid_closure_count"], 3)
            self.assertTrue(result["geometry"]["continuity_checked_against_rest_lengths"])
            self.assertFalse(result["physical_knot_proven"])
            self.assertTrue(all(v > .05 for v in result["tail_lengths_m"]))
            lo, hi = result["core_arclength_m"]
            left, right = result["tail_lengths_m"]
            self.assertAlmostEqual(left+hi-lo+right, result["geometry"]["length_m"])
            json.dumps(result, allow_nan=False)

    def test_rigid_transform_scale_and_reversed_vertex_order(self):
        # Independently selected rotation, translation and scale should not
        # change the result; view/closure samples are not fixed to a camera.
        axis = np.array([.2, -.5, .7]); axis /= np.linalg.norm(axis)
        angle = .83
        cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rotation = np.eye(3)+np.sin(angle)*cross+(1-np.cos(angle))*(cross@cross)
        points = (open_torus()[::-1] @ rotation.T)*1.7+[.4, -.25, 1.2]
        result = analyze_open_rope(points, .002*1.7)
        self.assertIs(result["is_overhand"], True)
        self.assertEqual(result["observed_determinants"], [3])

    def test_resampling_does_not_make_the_candidate(self):
        for count in (100, 260):
            self.assertIs(analyze_open_rope(open_torus(samples=count), .002)["is_overhand"], True)

    def test_crossing_envelope_tails_are_not_ruler_measurements(self):
        # Exactly the same 8 mm-diameter synthetic curve, merely rotated.
        # Its topology agrees, but finite projection envelopes cross the old
        # 4 cm gate. These estimates must never become scoring thresholds.
        points = open_torus(samples=100, tails=False)
        axis = np.array([.2, -.5, .7]); axis /= np.linalg.norm(axis)
        cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        tails = []
        for angle in (1., 2.):
            rotation = np.eye(3)+np.sin(angle)*cross+(1-np.cos(angle))*(cross@cross)
            result = analyze_open_rope(points @ rotation.T, .008)
            self.assertIs(result["is_overhand"], True)
            tails.append(min(result["tail_lengths_m"]))
        self.assertLess(tails[0], .02)
        self.assertGreater(tails[1], .045)

    def test_straight_u_and_coil_are_unknotted(self):
        t = np.linspace(.15, 2*np.pi-.15, 160)
        u = np.column_stack([.1*np.cos(t), .1*np.sin(t), np.zeros(len(t))])
        t = np.linspace(0, 6*np.pi, 200)
        coil = np.column_stack([.08*np.cos(t), .08*np.sin(t), t*.008])
        straight = np.column_stack([np.linspace(-.4, .4, 101), np.zeros(101), np.zeros(101)])
        for name, points in (("straight", straight), ("U", u), ("coil", coil)):
            with self.subTest(name=name):
                result = analyze_open_rope(points, .002)
                self.assertEqual(result["status"], "unknot")
                self.assertIs(result["is_overhand"], False)
                self.assertEqual(result["observed_determinants"], [1])

    def test_same_apparent_loops_with_wrong_threading_are_not_a_knot(self):
        points = open_torus()
        # Preserve every x/y feature but put the whole traversal in increasing
        # height.  This is a descending unknot diagram, not an overhand knot.
        points[:, 2] = np.linspace(-.06, .06, len(points))
        result = analyze_open_rope(points, .002)
        self.assertEqual(result["status"], "unknot")
        self.assertIs(result["is_overhand"], False)

    def test_five_crossing_torus_is_a_different_knot(self):
        result = analyze_open_rope(open_torus(2, 5), .002)
        self.assertEqual(result["status"], "other")
        self.assertIs(result["is_overhand"], False)
        self.assertEqual(result["observed_determinants"], [5])

    def test_determinant_three_does_not_uniquely_identify_a_trefoil(self):
        # T(3,4) is the eight-crossing knot 8_19, also with determinant 3.
        result = analyze_open_rope(open_torus(3, 4, samples=260), .001)
        self.assertEqual(result["observed_determinants"], [3])
        self.assertEqual(result["status"], "ambiguous")
        self.assertIsNone(result["is_overhand"])
        self.assertEqual(result["reason"], "invariant_consistent_but_diagram_certificate_missing")

    def test_virtual_closure_cannot_supply_missing_physical_crossings(self):
        result = analyze_open_rope(open_torus(cut=.9, tails=False), .001)
        self.assertEqual(result["observed_determinants"], [3])
        self.assertEqual(result["status"], "ambiguous")
        self.assertIsNone(result["is_overhand"])
        self.assertTrue(any(v.get("closure_trefoil_certificate") and not v["trefoil_certificate"]
                            for c in result["closures"] for v in c["projections"]))

    def test_penetration_and_exact_crossing_are_rejected(self):
        points = np.array([[-.1, -.1, 0], [.1, .1, 0], [.1, -.1, 0], [-.1, .1, 0]])
        result = analyze_open_rope(points, .008)
        self.assertEqual(result["status"], "invalid")
        self.assertIn("centreline_self_intersection", result["geometry"]["reasons"])
        points[-2:, 2] = .003
        result = analyze_open_rope(points, .008)
        self.assertEqual(result["status"], "invalid")
        self.assertIn("nonlocal_capsule_penetration", result["geometry"]["reasons"])

    def test_wrong_rest_length_and_missing_continuity_evidence(self):
        points = open_torus()
        rest = np.linalg.norm(np.diff(points, axis=0), axis=1)
        rest[20] /= 1.3
        result = analyze_open_rope(points, .002, expected_segment_lengths=rest)
        self.assertEqual(result["status"], "invalid")
        self.assertIn("segment_length_outside_tolerance", result["geometry"]["reasons"])
        result = analyze_open_rope(points, .002)
        self.assertFalse(result["geometry"]["continuity_checked_against_rest_lengths"])

    def test_singular_views_are_uncertain_not_negative(self):
        points = np.column_stack([np.linspace(-.4, .4, 101), np.zeros(101), np.zeros(101)])
        with patch.object(topology, "_VIEWS", np.tile([1., 0., 0.], (8, 1))):
            result = analyze_open_rope(points, .002)
        self.assertEqual(result["status"], "ambiguous")
        self.assertIsNone(result["is_overhand"])
        self.assertTrue(result["geometry"]["valid"])
        self.assertEqual(result["projection_failure_counts"], {"segment_projects_to_point": 40})

    def test_bad_inputs_fail_closed_without_nan_in_json(self):
        for points, diameter in (([[0, 0, 0]]*4, .01), ([[float("nan"), 0, 0]]*4, .01),
                                 (np.zeros((4, 2)), .01), (open_torus(), -.01)):
            result = analyze_open_rope(points, diameter)
            self.assertEqual(result["status"], "invalid")
            self.assertIsNone(result["is_overhand"])
            json.dumps(result, allow_nan=False)

    def test_integer_determinant_and_reidemeister_one(self):
        self.assertEqual(topology._bareiss_determinant([[2, -1], [-1, 2]]), 3)
        self.assertEqual(topology._bareiss_determinant([[0, 2], [3, 4]]), -6)
        events = [{"id": 0, "over": True, "sign": 1, "position": .2},
                  {"id": 0, "over": False, "sign": 1, "position": 1.4}]
        self.assertEqual(topology._alexander_determinant(events), 1)
        self.assertEqual(topology._reduce_diagram(events)[0], [])


if __name__ == "__main__":
    unittest.main()
