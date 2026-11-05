"""Unit tests for Task F scoring: boundary cases, score ladder, zero-gates, and fuzz test."""

from __future__ import annotations

import unittest
import numpy as np

import geometry as g
from scoring import (
    Scorer,
    SEATED_DEPTH,
    SEATED_LATERAL,
    SEATED_ANG_DEG,
    FIXTURE_MOVE_LIMIT,
    is_servo_seated,
)

# Canonical quaternion for seated plug (yaw = -pi/2)
Q_SEATED = np.array([0.0, 0.0, np.sin(-np.pi / 4), np.cos(-np.pi / 4)])


def make_servo_data(pos, quat=Q_SEATED, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)):
    return (np.array(pos, float), np.array(quat, float), np.array(lin_vel, float), np.array(ang_vel, float))


class TestTaskFScoring(unittest.TestCase):
    def setUp(self) -> None:
        self.scorer = Scorer()
        self.fixture_pos = np.array([0.28, 0.0, g.TOP])
        self.zero_sliders = {"assign_btn": 0.0, "reset_btn": 0.0}

    def start_servos(self):
        return {
            "blue": make_servo_data(g.START_POS["blue"]),
            "red": make_servo_data(g.START_POS["red"]),
            "green": make_servo_data(g.START_POS["green"]),
        }

    def test_start_scores_zero(self) -> None:
        res = self.scorer.update(self.fixture_pos, self.zero_sliders, self.start_servos(), False)
        self.assertEqual(res.score, 0.0)
        self.assertFalse(res.success)

    def test_unplug_scores_quarter(self) -> None:
        # Move green out to staging
        servos = self.start_servos()
        servos["green"] = make_servo_data(g.STAGE_POS["green"])
        res = self.scorer.update(self.fixture_pos, self.zero_sliders, servos, False)
        self.assertEqual(res.score, 0.25)
        self.assertFalse(res.success)

    def test_seated_boundary_depth(self) -> None:
        # Inside seated boundary: depth = 17.0 mm (origin at MOUTH_X - 1.0 mm)
        pos_just_in = (g.MOUTH_X - 0.001, g.SOCKET_Y[1], g.SOCKET_Z)
        self.assertTrue(is_servo_seated(pos_just_in, Q_SEATED))

        # Outside seated boundary: depth = 15.0 mm (origin at MOUTH_X - 3.0 mm; limit is 16 mm)
        pos_just_out = (g.MOUTH_X - 0.003, g.SOCKET_Y[1], g.SOCKET_Z)
        self.assertFalse(is_servo_seated(pos_just_out, Q_SEATED))

    def test_seated_boundary_lateral(self) -> None:
        # Just inside 1.5 mm lateral
        pos_in = (g.MOUTH_X, g.SOCKET_Y[1] + 0.0014, g.SOCKET_Z)
        self.assertTrue(is_servo_seated(pos_in, Q_SEATED))

        # Just outside 1.5 mm lateral
        pos_out = (g.MOUTH_X, g.SOCKET_Y[1] + 0.0016, g.SOCKET_Z)
        self.assertFalse(is_servo_seated(pos_out, Q_SEATED))

    def test_seated_boundary_angle(self) -> None:
        # Yaw -90 deg + 9.5 deg (inside 10 deg)
        yaw_in = -np.pi / 2 + np.radians(9.5)
        q_in = np.array([0.0, 0.0, np.sin(yaw_in / 2), np.cos(yaw_in / 2)])
        self.assertTrue(is_servo_seated(g.START_POS["red"], q_in))

        # Yaw -90 deg + 10.5 deg (outside 10 deg)
        yaw_out = -np.pi / 2 + np.radians(10.5)
        q_out = np.array([0.0, 0.0, np.sin(yaw_out / 2), np.cos(yaw_out / 2)])
        self.assertFalse(is_servo_seated(g.START_POS["red"], q_out))

    def test_fixture_shift_zero_gate(self) -> None:
        # Step 0: establish fixture start position
        self.scorer.update(self.fixture_pos, self.zero_sliders, self.start_servos(), False)
        self.assertFalse(self.scorer.failed)

        # 4 mm shift is OK
        shifted_ok = self.fixture_pos + np.array([0.004, 0.0, 0.0])
        res = self.scorer.update(shifted_ok, self.zero_sliders, self.start_servos(), False)
        self.assertFalse(self.scorer.failed)

        # 6 mm shift fails
        shifted_fail = self.fixture_pos + np.array([0.006, 0.0, 0.0])
        res2 = self.scorer.update(shifted_fail, self.zero_sliders, self.start_servos(), False)
        self.assertTrue(self.scorer.failed)
        self.assertEqual(res2.score, 0.0)

    def test_fixture_tilt_zero_gate(self) -> None:
        """Verify fixture tilt <= 5 deg passes, tilt > 5 deg triggers zero-gate (F21)."""
        scorer = Scorer()
        q_identity = (0.0, 0.0, 0.0, 1.0)
        scorer.update(self.fixture_pos, q_identity, self.zero_sliders, self.start_servos(), False)
        self.assertFalse(scorer.failed)

        # 4 deg tilt is OK
        th_ok = np.radians(4.0)
        q_ok = (np.sin(th_ok / 2), 0.0, 0.0, np.cos(th_ok / 2))
        scorer.update(self.fixture_pos, q_ok, self.zero_sliders, self.start_servos(), False)
        self.assertFalse(scorer.failed)

        # 6 deg tilt triggers failure
        th_fail = np.radians(6.0)
        q_fail = (np.sin(th_fail / 2), 0.0, 0.0, np.cos(th_fail / 2))
        res = scorer.update(self.fixture_pos, q_fail, self.zero_sliders, self.start_servos(), False)
        self.assertTrue(scorer.failed)
        self.assertEqual(res.score, 0.0)

    def test_button_depression_partial_credit_threshold(self) -> None:
        """Verify 2.0 mm button press triggers 0.25 partial credit (F20)."""
        scorer = Scorer()
        # 1.8 mm depression does not trigger 0.25 (remains 0.0)
        res1 = scorer.update(self.fixture_pos, {"assign_btn": -0.0018, "reset_btn": 0.0}, self.start_servos(), False)
        self.assertEqual(res1.score, 0.0)

        # 2.0 mm depression triggers 0.25
        res2 = scorer.update(self.fixture_pos, {"assign_btn": -0.0020, "reset_btn": 0.0}, self.start_servos(), False)
        self.assertEqual(res2.score, 0.25)

    def test_synthetic_shortcuts(self) -> None:
        """Construct validity: verify shortcut attempts cannot reach higher tiers without real work."""
        # Shortcut 1: Pressing ASSIGN while all 3 are plugged in
        s1 = Scorer()
        res_s1 = s1.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, self.start_servos(), True)
        self.assertEqual(res_s1.score, 0.25)
        self.assertFalse(res_s1.success)
        self.assertEqual(res_s1.details["bus_lamp"], "red")
        self.assertIsNone(res_s1.details["ids"]["red"])

        # Shortcut 2: Wrong order commissioning (green commissioned first)
        s2 = Scorer()
        servos = self.start_servos()
        servos["red"] = make_servo_data(g.STAGE_POS["red"])
        servos["blue"] = make_servo_data(g.STAGE_POS["blue"])
        # Green gets 1
        s2.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        s2.update(self.fixture_pos, self.zero_sliders, servos, False)
        # Red plugged and assigned -> red gets 2
        servos["red"] = make_servo_data(g.START_POS["red"])
        s2.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        s2.update(self.fixture_pos, self.zero_sliders, servos, False)
        # Blue plugged and assigned -> blue gets 3
        servos["blue"] = make_servo_data(g.START_POS["blue"])
        res_s2 = s2.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        # Red has 2 != 1, so tier 0.50 is NOT reached; score capped at 0.25
        self.assertEqual(res_s2.score, 0.25)
        self.assertFalse(res_s2.success)

        # Shortcut 3: All plugged but leaving RESET pressed
        s3 = Scorer()
        res_s3 = s3.update(self.fixture_pos, {"assign_btn": 0.0, "reset_btn": -0.007}, self.start_servos(), True)
        self.assertEqual(res_s3.score, 0.25)
        self.assertFalse(res_s3.success)

    def test_full_progression_to_success(self) -> None:
        servos = self.start_servos()

        # Step 1: Unplug green and blue
        servos["green"] = make_servo_data(g.STAGE_POS["green"])
        servos["blue"] = make_servo_data(g.STAGE_POS["blue"])
        res = self.scorer.update(self.fixture_pos, self.zero_sliders, servos, False)
        self.assertEqual(res.score, 0.25)

        # Step 2: Press ASSIGN -> red gets 1
        res = self.scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        self.assertEqual(res.score, 0.50)

        # Step 3: Release ASSIGN
        self.scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # Step 4: Plug green into socket 2
        servos["green"] = make_servo_data(g.START_POS["green"])
        res = self.scorer.update(self.fixture_pos, self.zero_sliders, servos, False)
        self.assertEqual(res.score, 0.50)

        # Step 5: Press ASSIGN -> green gets 2
        res = self.scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        self.assertEqual(res.score, 0.75)

        # Step 6: Release ASSIGN
        self.scorer.update(self.fixture_pos, self.zero_sliders, servos, False)

        # Step 7: Plug blue into socket 0
        servos["blue"] = make_servo_data(g.START_POS["blue"])
        res = self.scorer.update(self.fixture_pos, self.zero_sliders, servos, False)
        self.assertEqual(res.score, 0.75)

        # Step 8: Press ASSIGN -> blue gets 3
        res = self.scorer.update(self.fixture_pos, {"assign_btn": -0.007, "reset_btn": 0.0}, servos, False)
        # All 3 numbered, but grippers not open yet -> score 0.75, success False
        self.assertEqual(res.score, 0.75)
        self.assertFalse(res.success)

        # Step 9: Release ASSIGN, open grippers, still
        res = self.scorer.update(self.fixture_pos, self.zero_sliders, servos, True)
        self.assertEqual(res.score, 1.0)
        self.assertTrue(res.success)

    def test_random_state_fuzz(self) -> None:
        """Verify that Scorer.update produces defined numeric scores in [0, 1] without crashing on arbitrary states."""
        rng = np.random.default_rng(42)
        for _ in range(100):
            fpos = self.fixture_pos + rng.uniform(-0.01, 0.01, size=3)
            sliders = {
                "assign_btn": float(rng.uniform(-0.008, 0.0)),
                "reset_btn": float(rng.uniform(-0.008, 0.0)),
            }
            servos = {}
            for name in ("red", "green", "blue"):
                spos = np.array([0.25, 0.0, g.TOP]) + rng.uniform(-0.2, 0.2, size=3)
                q = rng.normal(size=4)
                q /= np.linalg.norm(q)
                lvel = rng.uniform(-0.1, 0.1, size=3)
                avel = rng.uniform(-1.0, 1.0, size=3)
                servos[name] = (spos, q, lvel, avel)

            open_both = bool(rng.choice([True, False]))
            res = self.scorer.update(fpos, sliders, servos, open_both)
            self.assertIsInstance(res.score, float)
            self.assertGreaterEqual(res.score, 0.0)
            self.assertLessEqual(res.score, 1.0)
            self.assertIsInstance(res.success, bool)


if __name__ == "__main__":
    unittest.main()
