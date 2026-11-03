"""Synthetic measured-state tests for the visible v3 rubric.

These verify scoring behavior, including adversarial counterexamples. They are
not a physics simulation and do not establish feasibility or physical fidelity.
Run: python -m unittest discover -s tasks/roll-a-die -p test_scoring.py -v
"""

import math
import unittest

import numpy as np

from scoring import (
    EDGE, FACES, LIFT_HOLD_S, REST_HOLD_S, SCORE_VERSION, TABLE_TOP,
    TABLE_X, TABLE_Y, RollTracker, rotation,
)

DT = 0.05


def quaternion(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return np.r_[axis * math.sin(angle / 2), math.cos(angle / 2)]


def compose_quaternions(first, second):
    """Apply the second rotation, then the first (both XYZW)."""
    first, second = np.asarray(first), np.asarray(second)
    return np.r_[
        first[3] * second[:3] + second[3] * first[:3] + np.cross(first[:3], second[:3]),
        first[3] * second[3] - np.dot(first[:3], second[:3]),
    ]


class Trace:
    """Construct consistent cube bounds and measured hand positions per sample."""

    def __init__(self):
        self.tracker = RollTracker()
        self.time = -DT
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])

    def sample(self, *, clearance=0.0, quat=None, lin_vel=(0, 0, 0),
               ang_vel=(0, 0, 0), hand="far", x=0.25, y=0.0,
               hand_opening=None, tcp_offset=None, second_tcp_offset=None):
        self.time += DT
        if quat is not None:
            self.quat = np.asarray(quat, dtype=float)
        half_bounds = np.abs(rotation(self.quat)) @ np.full(3, EDGE / 2)
        pos = np.array([x, y, TABLE_TOP + clearance + half_bounds[2]])
        tcp = pos + np.array([0.20, 0.0, 0.20])
        opening = 1.0
        if hand in {"closed", "open_near"}:
            tcp = pos + np.array([0.0, 0.0, 0.025])
            opening = 0.0 if hand == "closed" else 1.0
        if hand_opening is not None:
            opening = hand_opening
        if tcp_offset is not None:
            tcp = pos + np.asarray(tcp_offset)
        second_tcp = pos + np.asarray(second_tcp_offset if second_tcp_offset is not None else [-0.20, 0, 0.20])
        return {
            "time": self.time, "pos": pos.tolist(),
            "low": (pos - half_bounds).tolist(), "high": (pos + half_bounds).tolist(),
            "quat_xyzw": self.quat.tolist(), "lin_vel": list(lin_vel), "ang_vel": list(ang_vel),
            "hands": [{"tcp": tcp.tolist(), "opening": opening},
                      {"tcp": second_tcp.tolist(), "opening": 1.0}],
        }

    def step(self, **kwargs):
        return self.tracker.update(self.sample(**kwargs))

    def pickup(self, **kwargs):
        self.step()
        for _ in range(math.ceil(LIFT_HOLD_S / DT) + 2):
            self.step(clearance=0.08, hand="closed", **kwargs)
        return self

    def release(self, **kwargs):
        self.step(clearance=0.08, **kwargs)
        return self

    def tumble(self, *, axis=(1, 0, 0), angle=math.pi):
        for value in np.linspace(angle / 8, angle, 8):
            self.step(clearance=0.04, quat=quaternion(axis, value))
        return self

    def rest(self, samples=None, **kwargs):
        if samples is None:
            samples = math.ceil(REST_HOLD_S / DT) + 2
        for _ in range(samples):
            self.step(**kwargs)
        return self


class RollScoringTests(unittest.TestCase):
    def test_visible_pickup_release_turnover_and_rest_reach_full_score(self):
        trace = Trace().pickup()
        self.assertEqual(trace.tracker.value, 0.25)
        trace.release()
        self.assertEqual(trace.tracker.value, 0.5)
        trace.tumble()
        self.assertTrue(trace.tracker.face_down_times)
        self.assertEqual(trace.tracker.value, 0.5)
        trace.rest()
        self.assertEqual(trace.tracker.value, 1.0)
        self.assertIsNone(trace.tracker.invalid)
        down = next(e for e in trace.tracker.events if e["event"] == "face_down")
        self.assertEqual((down["face"], down["pips"], down["color"], down["direction"]),
                         ("+Z", 1, "white", "down"))
        self.assertGreater(down["time"], down["up_time"])

    def test_visible_release_earns_half_credit_without_turnover(self):
        trace = Trace().pickup().release().rest()
        self.assertEqual(trace.tracker.value, 0.5)
        self.assertFalse(trace.tracker.face_down_times)
        self.assertEqual(trace.tracker.rest_hold, 0.0)

    def test_release_requires_both_hand_centres_and_airborne_clearance(self):
        for clearance, first, second, expected in (
            (0.01, 0.05, 0.05, 0.5), (0.0099, 0.05, 0.05, 0.25),
            (0.01, 0.0499, 0.05, 0.25), (0.01, 0.05, 0.0499, 0.25),
        ):
            with self.subTest(clearance=clearance, first=first, second=second):
                trace = Trace().pickup()
                trace.step(clearance=clearance, tcp_offset=(first, 0, 0),
                           second_tcp_offset=(-second, 0, 0))
                self.assertEqual(trace.tracker.value, expected)

    def test_opening_nearby_jaws_alone_is_not_visible_release(self):
        trace = Trace().pickup().release(hand="open_near")
        self.assertFalse(trace.tracker.released)
        self.assertEqual(trace.tracker.value, 0.25)

    def test_placing_cube_back_on_table_is_only_pickup_credit(self):
        trace = Trace().pickup()
        trace.step(hand="closed")
        trace.rest()
        self.assertFalse(trace.tracker.released)
        self.assertEqual(trace.tracker.value, 0.25)

    def test_pickup_needs_continuous_height_and_measured_grasp(self):
        for params in ({"clearance": 0.0299, "hand": "closed"},
                       {"clearance": 0.03, "hand": "open_near"},
                       {"clearance": 0.03, "hand_opening": 0.0, "tcp_offset": (0, 0, 0.035)},
                       {"clearance": 0.03, "hand_opening": 0.30, "tcp_offset": (0, 0, 0.025)}):
            with self.subTest(params=params):
                trace = Trace().rest(10, **params)
                self.assertEqual(trace.tracker.value, 0.0)
        trace = Trace().rest(4, clearance=0.03, hand="closed")
        trace.step(clearance=0.0299, hand="closed")
        trace.rest(4, clearance=0.03, hand="closed")
        self.assertFalse(trace.tracker.lifted)
        trace.step(clearance=0.03, hand="closed")
        self.assertTrue(trace.tracker.lifted)

    def test_pickup_dwell_starts_at_first_held_observation(self):
        trace = Trace()
        trace.step()
        trace.step(clearance=0.03, hand="closed")
        self.assertEqual(trace.tracker.lift_hold, 0.0)
        trace.rest(math.ceil(LIFT_HOLD_S / DT) - 1, clearance=0.03, hand="closed")
        self.assertFalse(trace.tracker.lifted)
        trace.step(clearance=0.03, hand="closed")
        self.assertTrue(trace.tracker.lifted)

    def test_pose_rubric_does_not_depend_on_velocity_or_acceleration_estimates(self):
        trace = Trace().pickup(lin_vel=(1.0, 0, 2.0)).release(lin_vel=(1.0, 0, 2.0))
        trace.tumble().rest(lin_vel=(99, -99, 99), ang_vel=(99, 99, 99))
        self.assertEqual(trace.tracker.value, 1.0)

    def test_no_pickup_means_no_credit_even_for_a_free_turnover(self):
        trace = Trace().release().tumble().rest()
        self.assertFalse(trace.tracker.lifted)
        self.assertFalse(trace.tracker.released)
        self.assertEqual(trace.tracker.value, 0.0)

    def test_held_reorientation_cannot_supply_the_upward_witness(self):
        trace = Trace().pickup()
        for angle in np.linspace(0, math.pi, 9):
            trace.step(clearance=0.08, hand="closed", quat=quaternion((1, 0, 0), angle))
        trace.release().rest()
        self.assertEqual(trace.tracker.value, 0.5)
        self.assertNotIn("+Z", trace.tracker.face_up_times)
        self.assertFalse(trace.tracker.face_down_times)
        self.assertAlmostEqual(trace.tracker.tumble, 0.0)

    def test_held_to_free_boundary_rotation_is_excluded(self):
        trace = Trace().pickup().release(quat=quaternion((1, 0, 0), math.pi)).rest()
        self.assertEqual(trace.tracker.value, 0.5)
        self.assertFalse(trace.tracker.face_down_times)
        self.assertAlmostEqual(trace.tracker.max_tilt_excursion, 0.0)

    def test_face_can_become_upward_after_release_before_turning_down(self):
        trace = Trace().pickup().release(quat=quaternion((1, 0, 0), math.pi / 4))
        self.assertFalse(trace.tracker.face_up_times)  # deliberately ambiguous release pose
        for degrees in (60, 90, 120, 180, 210, 240, 270):
            trace.step(clearance=0.04, quat=quaternion((1, 0, 0), math.radians(degrees)))
        trace.rest()
        self.assertIn("+Y", trace.tracker.face_down_times)
        self.assertEqual(trace.tracker.value, 1.0)

    def test_face_direction_threshold_is_thirty_degrees(self):
        for start, end, expected in ((30, 150, True), (30.001, 150, False), (30, 149.999, False)):
            with self.subTest(start=start, end=end):
                trace = Trace().pickup().release(quat=quaternion((1, 0, 0), math.radians(start)))
                trace.step(clearance=0.04, quat=quaternion((1, 0, 0), math.radians(end)))
                self.assertEqual("+Z" in trace.tracker.face_down_times, expected)

    def test_each_physical_face_can_supply_its_own_turnover(self):
        starts = {
            "+X": quaternion((0, 1, 0), -math.pi / 2),
            "-X": quaternion((0, 1, 0), math.pi / 2),
            "+Y": quaternion((1, 0, 0), math.pi / 2),
            "-Y": quaternion((1, 0, 0), -math.pi / 2),
            "+Z": quaternion((1, 0, 0), 0),
            "-Z": quaternion((1, 0, 0), math.pi),
        }
        for face, start in starts.items():
            with self.subTest(face=face):
                trace = Trace().pickup().release(quat=start)
                self.assertIn(face, trace.tracker.face_up_times)
                for angle in np.linspace(math.pi / 8, math.pi, 8):
                    trace.step(clearance=0.04,
                               quat=compose_quaternions(quaternion((1, 0, 0), angle), start))
                trace.rest()
                self.assertIn(face, trace.tracker.face_down_times)
                self.assertEqual(trace.tracker.value, 1.0)

    def test_up_and_down_witnesses_must_be_the_same_face_and_in_order(self):
        trace = Trace().pickup().release(quat=quaternion((1, 0, 0), math.pi))
        self.assertNotIn("+Z", trace.tracker.face_up_times)
        # +Z is first down, then up: that is not yet +Z's up-to-down turnover.
        trace.step(clearance=0.04, quat=quaternion((1, 0, 0), 0))
        self.assertNotIn("+Z", trace.tracker.face_down_times)
        self.assertIn("-Z", trace.tracker.face_down_times)

    def test_repeated_small_rocking_cannot_accumulate_turnover(self):
        trace = Trace().pickup().release()
        for degrees in [10, -10] * 20 + [0]:
            trace.step(clearance=0.04, quat=quaternion((1, 0, 0), math.radians(degrees)))
        trace.rest()
        self.assertGreater(math.degrees(trace.tracker.tumble), 360)
        self.assertFalse(trace.tracker.face_down_times)
        self.assertEqual(trace.tracker.value, 0.5)

    def test_ninety_degree_turn_is_insufficient_for_full_inversion(self):
        trace = Trace().pickup().release().tumble(angle=math.pi / 2).rest()
        self.assertFalse(trace.tracker.face_down_times)
        self.assertEqual(trace.tracker.value, 0.5)

    def test_full_revolution_counts_even_when_final_face_matches_release(self):
        trace = Trace().pickup().release().tumble(angle=2 * math.pi).rest()
        self.assertTrue(trace.tracker.face_down_times)
        np.testing.assert_allclose(rotation(trace.quat), np.eye(3), atol=1e-12)
        self.assertEqual(trace.tracker.value, 1.0)

    def test_world_yaw_of_tilted_cube_never_becomes_turnover(self):
        tilted = quaternion((1, 0, 0), math.radians(20))
        trace = Trace().pickup().release(quat=tilted)
        for angle in np.linspace(0, 2 * math.pi, 25):
            trace.step(clearance=0.04, quat=compose_quaternions(quaternion((0, 0, 1), angle), tilted))
        self.assertFalse(trace.tracker.face_down_times)
        self.assertEqual(trace.tracker.value, 0.5)
        self.assertAlmostEqual(trace.tracker.tumble, 0.0)

    def test_slow_turnover_survives_legacy_diagnostic_deadband(self):
        trace = Trace().pickup().release()
        for angle in np.arange(0.25, 180.25, 0.25):
            trace.step(clearance=0.04, quat=quaternion((1, 0, 0), math.radians(angle)))
        trace.rest()
        self.assertAlmostEqual(trace.tracker.tumble, 0.0)
        self.assertEqual(trace.tracker.value, 1.0)

    def test_quaternion_sign_changes_do_not_create_turnover(self):
        trace = Trace().pickup().release()
        for index in range(8):
            trace.step(clearance=0.04, quat=[0, 0, 0, -1 if index % 2 else 1])
        trace.rest()
        self.assertFalse(trace.tracker.face_down_times)
        self.assertEqual(trace.tracker.value, 0.5)

    def test_rest_dwell_starts_at_first_resting_pose(self):
        trace = Trace().pickup().release().tumble()
        trace.step()
        self.assertEqual(trace.tracker.rest_hold, 0.0)
        trace.rest(math.ceil(REST_HOLD_S / DT) - 1)
        self.assertEqual(trace.tracker.value, 0.5)
        trace.step()
        self.assertEqual(trace.tracker.value, 1.0)

    def test_rest_requires_face_table_contact_and_hand_withdrawal(self):
        for params in ({"clearance": 0.0031}, {"tcp_offset": (0, 0, 0.0599)},
                       {"quat": quaternion((1, 0, 0), math.radians(169.9))}):
            with self.subTest(params=params):
                trace = Trace().pickup().release().tumble().rest(**params)
                self.assertEqual(trace.tracker.value, 0.5)
                self.assertEqual(trace.tracker.rest_hold, 0.0)
                self.assertIsNone(trace.tracker.invalid)
        trace = Trace().pickup().release().tumble()
        trace.rest(clearance=0.003, tcp_offset=(0, 0, 0.06),
                   quat=quaternion((1, 0, 0), math.radians(170)))
        self.assertEqual(trace.tracker.value, 1.0)

    def test_translation_tolerance_is_against_one_fixed_pose(self):
        trace = Trace().pickup().release().tumble()
        for index in range(30):
            trace.step(x=0.25 + index * 0.0003)
            self.assertLess(trace.tracker.rest_hold, 1.0)
        self.assertEqual(trace.tracker.value, 0.5)
        trace.rest(x=0.2587)
        self.assertEqual(trace.tracker.value, 1.0)

    def test_yaw_tolerance_is_against_one_fixed_pose(self):
        trace = Trace().pickup().release().tumble()
        for index in range(30):
            trace.step(quat=compose_quaternions(quaternion((0, 0, 1), math.radians(index)),
                                               quaternion((1, 0, 0), math.pi)))
            self.assertLess(trace.tracker.rest_hold, 1.0)
        self.assertEqual(trace.tracker.value, 0.5)
        trace.rest()
        self.assertEqual(trace.tracker.value, 1.0)

    def test_small_position_and_orientation_noise_can_rest(self):
        trace = Trace().pickup().release().tumble()
        trace.step()
        for index in range(20):
            sign = -1 if index % 2 else 1
            trace.step(x=0.25 + sign * 0.001,
                       quat=compose_quaternions(quaternion((0, 0, 1), math.radians(sign * 4)),
                                                quaternion((1, 0, 0), math.pi)))
        self.assertEqual(trace.tracker.value, 1.0)

    def test_motion_after_full_score_removes_full_credit(self):
        trace = Trace().pickup().release().tumble().rest()
        self.assertEqual(trace.tracker.value, 1.0)
        trace.step(x=0.26)
        self.assertEqual(trace.tracker.value, 0.5)
        trace.rest(x=0.26)
        self.assertEqual(trace.tracker.value, 1.0)

    def test_rest_must_restart_after_losing_table_contact(self):
        trace = Trace().pickup().release().tumble().rest(12)
        trace.step(clearance=0.01)
        self.assertEqual(trace.tracker.rest_hold, 0.0)
        trace.rest(12)
        self.assertEqual(trace.tracker.value, 0.5)
        trace.rest()
        self.assertEqual(trace.tracker.value, 1.0)

    def test_each_off_table_boundary_permanently_erases_credit(self):
        for params in ({"x": TABLE_X[0]}, {"x": TABLE_X[1]},
                       {"y": TABLE_Y[0]}, {"y": TABLE_Y[1]}, {"clearance": -0.0151}):
            with self.subTest(params=params):
                trace = Trace().pickup().release().tumble().rest()
                trace.step(**params)
                self.assertEqual(trace.tracker.value, 0.0)
                trace.rest()
                self.assertEqual(trace.tracker.value, 0.0)
                self.assertIsNotNone(trace.tracker.invalid)

    def test_whole_cube_on_boundary_is_allowed(self):
        trace = Trace().pickup().release().tumble().rest(x=TABLE_X[1] - EDGE / 2)
        self.assertEqual(trace.tracker.value, 1.0)
        self.assertIsNone(trace.tracker.invalid)

    def test_open_or_closed_hand_reentry_permanently_erases_credit(self):
        for opening in (0.0, 1.0):
            with self.subTest(opening=opening):
                trace = Trace().pickup().release().tumble().rest()
                trace.step(tcp_offset=(0, 0, 0.0499), hand_opening=opening)
                self.assertEqual(trace.tracker.value, 0.0)
                trace.rest()
                self.assertEqual(trace.tracker.value, 0.0)
        trace = Trace().pickup().release().tumble()
        trace.step(tcp_offset=(0, 0, 0.05))
        self.assertIsNone(trace.tracker.invalid)

    def test_invalidity_erases_pickup_and_release_levels_too(self):
        for released in (False, True):
            trace = Trace().pickup()
            if released:
                trace.release()
            trace.step(x=TABLE_X[1])
            self.assertEqual(trace.tracker.value, 0.0)
            trace.release().tumble().rest()
            self.assertEqual(trace.tracker.value, 0.0)

    def test_summary_preserves_diagnostics_and_adds_named_face_evidence(self):
        trace = Trace().pickup().release().tumble().rest()
        summary = trace.tracker.summary()
        self.assertEqual(summary["score_version"], SCORE_VERSION)
        self.assertEqual(summary["score"], 1.0)
        self.assertTrue(summary["turned_over"])
        self.assertIn("+Z", summary["turnover_faces"])
        self.assertGreater(summary["face_down_times"]["+Z"], summary["face_up_times"]["+Z"])
        self.assertEqual(summary["tumble_degrees"], summary["cumulative_tumble_degrees"])
        self.assertEqual({face: pips for face, _, _, pips, _ in FACES},
                         {"+Z": 1, "-Z": 6, "+X": 2, "-X": 5, "+Y": 3, "-Y": 4})

    def test_bad_times_and_observation_gaps_are_rejected(self):
        for time in (float("nan"), float("inf"), float("-inf"), -0.05):
            sample = Trace().sample()
            sample["time"] = time
            with self.assertRaises(ValueError):
                RollTracker().update(sample)
        for time in (0.0, 0.10):
            trace = Trace()
            trace.step()
            sample = trace.sample()
            sample["time"] = time
            with self.assertRaises(ValueError):
                trace.tracker.update(sample)

    def test_nonfinite_or_malformed_measured_states_are_rejected(self):
        for key, value in (("pos", [0, float("nan"), 0]), ("quat_xyzw", [0, 0, 0, 0]),
                           ("quat_xyzw", [0, 0, 1]), ("low", [0, 0]),
                           ("ang_vel", [0, float("inf"), 0]), ("hands", [])):
            sample = Trace().sample()
            sample[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                RollTracker().update(sample)


if __name__ == "__main__":
    unittest.main()
