"""Offline tests for scoring.py (stdlib unittest, numpy only).

Run:  python -m unittest discover -s <this folder> -p 'test_scoring.py' -v

The tests build world-frame states from block-frame descriptions with their own quaternion helpers (Hamilton
product and vector rotation, no rotation matrices), so they do not share code with the maths under test.
"""

from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import geometry as G  # noqa: E402
import scoring as S  # noqa: E402

# -- independent quaternion helpers (x, y, z, w) -------------------------------------------------------------------
Q_IDENT = np.array([0.0, 0.0, 0.0, 1.0])


def q_axis_angle(axis, deg):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    h = math.radians(deg) / 2.0
    return np.concatenate([a * math.sin(h), [math.cos(h)]])


def q_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )


def q_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_rotate(q, v):
    """Rotate vector ``v`` by the unit quaternion ``q``."""
    u, w = np.asarray(q[:3], dtype=float), float(q[3])
    v = np.asarray(v, dtype=float)
    return v + 2.0 * w * np.cross(u, v) + 2.0 * np.cross(u, np.cross(u, v))


SPEED_DIR = np.array([1.0, 2.0, 2.0]) / 3.0  # unit vector, so a speed of m is a vector of norm m (not axis aligned)
MOVE_DIR = np.array([0.6, 0.8, 0.0])  # unit vector in the xy plane
START_PLUG_POS = np.array([*G.PLUG_XY0, G.PLUG_Z0])
START_PLUG_Q = q_axis_angle((0, 0, 1), math.degrees(G.PLUG_YAW0))


def build(block_pos, block_q, local, rel_q=Q_IDENT, v_plug=0.0, w_plug=0.0, v_block=0.0, w_block=0.0,
          detent=0.0, grippers_open=True):
    """World-frame keyword arguments for ``Scorer.update`` from the plug's pose in the block frame."""
    bq = np.asarray(block_q, dtype=float)
    bq = bq / np.linalg.norm(bq)
    block_pos = np.asarray(block_pos, dtype=float)
    return dict(
        plug_pos=block_pos + q_rotate(bq, local),
        plug_quat_xyzw=q_mul(bq, rel_q),
        plug_lin_vel=v_plug * SPEED_DIR,
        plug_ang_vel=w_plug * SPEED_DIR,
        block_pos=block_pos.copy(),
        block_quat_xyzw=bq.copy(),
        block_lin_vel=v_block * SPEED_DIR,
        block_ang_vel=w_block * SPEED_DIR,
        detent=detent,
        grippers_open=grippers_open,
    )


def with_plug(kwargs, plug_pos, plug_q=START_PLUG_Q):
    """Copy of ``kwargs`` with the plug placed directly in the world frame."""
    out = dict(kwargs)
    out["plug_pos"] = np.asarray(plug_pos, dtype=float)
    out["plug_quat_xyzw"] = np.asarray(plug_q, dtype=float)
    return out


# -- tier / threshold cases, run for several block poses -----------------------------------------------------------
class _TierCases:
    """Every boundary case, parametrised by the block's starting pose (the maths must use the block frame)."""

    block_pos0 = np.array(G.BLOCK_POS)
    block_q0 = Q_IDENT
    level_q = Q_IDENT  # the block's yaw alone (no tilt)

    def setUp(self):
        self.start = self.world_state(START_PLUG_POS)
        r = self.fresh_scorer_result()
        self.assertEqual(r.score, 0.0)
        self.assertEqual(r.details["tier"], S.TIER_NONE)

    # -- helpers
    def fresh_scorer_result(self):
        return S.Scorer().update(**self.start)

    def fresh(self):
        s = S.Scorer()
        s.update(**self.start)
        return s

    def state(self, depth=G.NOSE_L, dx=0.0, dz=0.0, ang=0.0, axis=(0, 0, 1), moved=0.0, block_dz=0.0,
              block_q=None, **kw):
        """The plug ``depth`` metres into the socket, offset (dx, dz) from its axis and turned ``ang`` degrees
        about ``axis`` (block frame); the block displaced ``moved`` metres in xy from its start."""
        block_pos = self.block_pos0 + moved * MOVE_DIR + np.array([0.0, 0.0, block_dz])
        bq = self.block_q0 if block_q is None else block_q
        local = (dx, G.MOUTH_Y + depth - G.NOSE_L, dz)
        return build(block_pos, bq, local, q_axis_angle(axis, ang), **kw)

    def world_state(self, plug_pos, plug_q=START_PLUG_Q):
        return with_plug(build(self.block_pos0, self.block_q0, (0, 0, 0)), plug_pos, plug_q)

    def mouth(self):
        return self.block_pos0 + q_rotate(self.block_q0 / np.linalg.norm(self.block_q0), (0.0, G.MOUTH_Y, 0.0))

    def near_state(self, dist, height_gain=0.0):
        """The plug ``dist`` metres from the socket mouth (horizontally away along world -y), at the given height
        above its start."""
        mouth = self.mouth()
        z = G.PLUG_Z0 + height_gain
        horizontal = math.sqrt(dist ** 2 - (z - mouth[2]) ** 2)
        return self.world_state([mouth[0], mouth[1] - horizontal, z])

    def lift_state(self, height):
        return self.world_state([*G.PLUG_XY0, G.PLUG_Z0 + height])

    def fall_state(self, below):
        return self.world_state([*G.PLUG_XY0, G.PLUG_Z0 - below])

    def score_of(self, **kw):
        return self.fresh().update(**self.state(**kw))

    def play(self, *states):
        s = self.fresh()
        return [s.update(**st) for st in states]

    # -- the seated pose from geometry.py
    def test_geometry_seated_pose_scores_one(self):
        local = G.seated_plug_origin_in_block_frame()
        r = self.fresh().update(**build(self.block_pos0, self.block_q0, local))
        self.assertEqual((r.score, r.success), (1.0, True))
        self.assertEqual(r.details["tier"], S.TIER_SUCCESS)
        self.assertAlmostEqual(r.details["depth"], G.NOSE_L, places=9)
        self.assertLess(r.details["lateral"], 1e-9)
        self.assertLess(r.details["ang"], 1e-4)
        self.assertTrue(r.details["seated"])
        self.assertFalse(r.details["failed"])

    def test_first_update_details(self):
        r = self.fresh_scorer_result()
        self.assertEqual(r.details["block_moved"], 0.0)
        self.assertEqual(r.details["plug_height_gain"], 0.0)
        self.assertFalse(r.details["lifted_ever"])
        self.assertFalse(r.details["near"])

    # -- depth
    def test_depth_seated_boundary(self):
        for depth, score in [(0.0169, 0.5), (0.017 - 1e-6, 0.5), (0.017, 1.0), (0.017 + 1e-6, 1.0),
                             (0.0171, 1.0), (0.018, 1.0)]:
            with self.subTest(depth_mm=depth * 1000):
                r = self.score_of(depth=depth)
                self.assertEqual(r.score, score)
                self.assertEqual(r.details["seated"], score == 1.0)

    def test_depth_engaged_boundary(self):
        for depth, score in [(0.0, 0.0), (0.0059, 0.0), (0.006 - 1e-6, 0.0), (0.006, 0.5), (0.006 + 1e-6, 0.5),
                             (0.0061, 0.5), (0.0169, 0.5)]:
            with self.subTest(depth_mm=depth * 1000):
                r = self.score_of(depth=depth)
                self.assertEqual(r.score, score)
                self.assertEqual(r.details["engaged"], score == 0.5)
                self.assertFalse(r.details["seated"])

    def test_depth_is_clipped_to_nose_length(self):
        r = self.score_of(depth=G.NOSE_L + 0.004)  # origin past the mouth: depth saturates at NOSE_L
        self.assertAlmostEqual(r.details["depth"], G.NOSE_L, places=9)
        self.assertEqual(r.score, 1.0)
        r = self.score_of(depth=-0.05)  # far outside the socket
        self.assertEqual(r.details["depth"], 0.0)
        self.assertEqual(r.score, 0.0)

    # -- angle
    def test_ang_seated_boundary_about_each_axis(self):
        for axis in [(0, 0, 1), (1, 0, 0), (0, 1, 0), (1, 2, -2)]:
            for sign in (1, -1):
                for ang, score in [(2.9, 1.0), (3.0 - 1e-3, 1.0), (3.0, 1.0), (3.0 + 1e-3, 0.5), (3.1, 0.5)]:
                    with self.subTest(axis=axis, ang=sign * ang):
                        r = self.score_of(ang=sign * ang, axis=axis)
                        self.assertEqual(r.score, score)
                        self.assertAlmostEqual(r.details["ang"], ang, places=7)

    def test_ang_engaged_boundary_about_each_axis(self):
        for axis in [(0, 0, 1), (1, 0, 0), (0, 1, 0), (1, 2, -2)]:
            for ang, score in [(4.9, 0.5), (5.0 - 1e-3, 0.5), (5.0, 0.5), (5.0 + 1e-3, 0.0), (5.1, 0.0)]:
                with self.subTest(axis=axis, ang=ang, where="engaged depth"):
                    self.assertEqual(self.score_of(depth=0.010, ang=ang, axis=axis).score, score)
                with self.subTest(axis=axis, ang=ang, where="seated depth"):
                    self.assertEqual(self.score_of(ang=ang, axis=axis).score, score)

    def test_flipped_plug_scores_zero(self):
        for axis in [(0, 0, 1), (0, 1, 0), (1, 0, 0)]:
            with self.subTest(axis=axis):
                r = self.score_of(ang=180.0, axis=axis)
                self.assertEqual(r.score, 0.0)
                self.assertAlmostEqual(r.details["ang"], 180.0, places=3)  # acos is ill-conditioned at 180

    # -- lateral
    def test_lateral_seated_boundary(self):
        for axis in ("x", "z"):
            for sign in (1, -1):
                for lat, score in [(0.0014, 1.0), (0.0015 - 1e-6, 1.0), (0.0015, 1.0), (0.0015 + 1e-6, 0.5),
                                   (0.0016, 0.5)]:
                    with self.subTest(axis=axis, offset_mm=sign * lat * 1000):
                        kw = {"dx": sign * lat} if axis == "x" else {"dz": sign * lat}
                        r = self.score_of(**kw)
                        self.assertEqual(r.score, score)
                        self.assertAlmostEqual(r.details["lateral"], lat, places=9)

    def test_lateral_is_max_of_x_and_z_not_the_norm(self):
        r = self.score_of(dx=0.0015, dz=-0.0015)  # corner of the square zone: norm 2.1 mm, still seated
        self.assertEqual(r.score, 1.0)
        self.assertAlmostEqual(r.details["lateral"], 0.0015, places=9)
        r = self.score_of(dx=0.0015, dz=0.0016)
        self.assertEqual(r.score, 0.5)

    def test_lateral_engaged_boundary(self):
        for axis in ("x", "z"):
            for sign in (1, -1):
                for lat, score in [(0.0029, 0.5), (0.003 - 1e-6, 0.5), (0.003, 0.5), (0.003 + 1e-6, 0.0),
                                   (0.0031, 0.0)]:
                    with self.subTest(axis=axis, offset_mm=sign * lat * 1000):
                        kw = {"dx": sign * lat} if axis == "x" else {"dz": sign * lat}
                        self.assertEqual(self.score_of(depth=0.010, **kw).score, score)
        self.assertEqual(self.score_of(depth=0.010, dx=0.003, dz=-0.003).score, 0.5)  # corner of the square

    # -- detent
    def test_detent_boundary(self):
        for detent, score in [(0.0, 1.0), (0.0005, 1.0), (-0.0002, 1.0), (-0.0003 + 1e-6, 1.0), (-0.0003, 0.5),
                              (-0.0003 - 1e-6, 0.5), (-0.0004, 0.5), (-0.0015, 0.5)]:
            with self.subTest(detent=detent):
                r = self.score_of(detent=detent)
                self.assertEqual(r.score, score)
                self.assertEqual(r.details["seated"], score == 1.0)
                if score == 0.5:
                    self.assertEqual(r.details["tier"], S.TIER_ENGAGED)

    # -- block displacement
    def test_block_moved_boundaries(self):
        cases = [(0.0, 1.0, False), (0.0049, 1.0, False), (0.005 - 1e-6, 1.0, False), (0.005, 1.0, False),
                 (0.005 + 1e-6, 0.5, False), (0.0051, 0.5, False), (0.029, 0.5, False), (0.03 - 1e-6, 0.5, False),
                 (0.030, 0.5, False), (0.03 + 1e-6, 0.0, True), (0.031, 0.0, True), (0.2, 0.0, True)]
        for moved, score, failed in cases:
            with self.subTest(moved_mm=moved * 1000):
                r = self.score_of(moved=moved)
                self.assertEqual((r.score, r.success, r.details["failed"]), (score, score == 1.0, failed))
                self.assertAlmostEqual(r.details["block_moved"], moved, places=9)
        self.assertEqual(self.score_of(moved=0.0051).details["tier"], S.TIER_SEATED_BLOCK_MOVED)

    def test_block_moved_does_not_matter_for_engaged_tier(self):
        for moved in (0.0, 0.0049, 0.0051, 0.02, 0.03):
            with self.subTest(moved_mm=moved * 1000):
                self.assertEqual(self.score_of(depth=0.010, moved=moved).score, 0.5)
        self.assertEqual(self.score_of(depth=0.010, moved=0.031).score, 0.0)

    def test_block_moved_ignores_z(self):
        r = self.score_of(block_dz=0.05)
        self.assertEqual((r.score, r.details["failed"]), (1.0, False))
        self.assertEqual(r.details["block_moved"], 0.0)

    def test_block_moved_failure_applies_with_grippers_closed_too(self):
        r = self.score_of(moved=0.031, grippers_open=False)
        self.assertEqual((r.score, r.details["failed"]), (0.0, True))

    # -- 1.0 versus 0.75: grippers_open and still
    def test_still_and_gripper_combinations(self):
        cases = [
            ({}, 1.0),
            ({"grippers_open": False}, 0.75),
            ({"v_plug": 0.0049}, 1.0), ({"v_plug": 0.005}, 0.75), ({"v_plug": 0.0051}, 0.75),
            ({"w_plug": 0.19}, 1.0), ({"w_plug": 0.2}, 0.75), ({"w_plug": 0.21}, 0.75),
            ({"v_block": 0.0049}, 1.0), ({"v_block": 0.005}, 0.75), ({"v_block": 0.0051}, 0.75),
            ({"w_block": 0.19}, 1.0), ({"w_block": 0.2}, 0.75), ({"w_block": 0.21}, 0.75),
            ({"grippers_open": False, "v_plug": 0.0049}, 0.75),
            ({"grippers_open": True, "v_plug": 0.5, "w_plug": 3.0, "v_block": 0.5, "w_block": 3.0}, 0.75),
            ({"grippers_open": False, "v_plug": 0.5, "w_block": 3.0}, 0.75),
        ]
        for kw, score in cases:
            with self.subTest(**kw):
                r = self.score_of(**kw)
                self.assertEqual(r.score, score)
                self.assertEqual(r.success, score == 1.0)
                self.assertEqual(r.details["still"], not any(
                    kw.get(k, 0.0) >= t for k, t in [("v_plug", 0.005), ("w_plug", 0.2), ("v_block", 0.005),
                                                     ("w_block", 0.2)]))
                if score == 0.75:
                    self.assertEqual(r.details["tier"], S.TIER_SEATED_HELD)

    def test_held_tier_needs_block_within_5mm(self):
        for moved, score in [(0.005, 0.75), (0.0051, 0.5)]:
            with self.subTest(moved_mm=moved * 1000):
                self.assertEqual(self.score_of(moved=moved, grippers_open=False).score, score)
                self.assertEqual(self.score_of(moved=moved, v_plug=0.5).score, score)

    def test_still_and_grippers_do_not_matter_below_seated(self):
        for kw in ({}, {"grippers_open": False}, {"v_plug": 1.0}):
            with self.subTest(**kw):
                self.assertEqual(self.score_of(depth=0.010, **kw).score, 0.5)

    def test_success_is_not_latched(self):
        results = self.play(self.state(), self.state(grippers_open=False), self.state(), self.state(depth=0.0),
                            self.state())
        self.assertEqual([r.score for r in results], [1.0, 0.75, 1.0, 0.0, 1.0])
        self.assertEqual([r.success for r in results], [True, False, True, False, True])

    # -- near / lifted
    def test_near_boundary(self):
        for dist, score in [(0.05, 0.25), (0.099, 0.25), (0.10 - 1e-6, 0.25), (0.10, 0.25), (0.10 + 1e-6, 0.0),
                            (0.101, 0.0), (0.2, 0.0)]:
            with self.subTest(dist=dist):
                r = self.play(self.lift_state(0.05), self.near_state(dist))[1]
                self.assertEqual(r.score, score)
                self.assertEqual(r.details["near"], score == 0.25)
                self.assertAlmostEqual(r.details["near_dist"], dist, places=9)
                if score == 0.25:
                    self.assertEqual(r.details["tier"], S.TIER_LIFTED_NEAR)

    def test_lifted_boundary(self):
        for height, score in [(0.0, 0.0), (0.029, 0.0), (0.03 - 1e-6, 0.0), (0.03, 0.25), (0.03 + 1e-6, 0.25),
                              (0.031, 0.25), (0.2, 0.25)]:
            with self.subTest(lift_mm=height * 1000):
                lift_result, r = self.play(self.lift_state(height), self.near_state(0.099))
                self.assertEqual(lift_result.score, 0.0)  # far from the socket: lifting alone scores nothing
                self.assertEqual(lift_result.details["lifted_ever"], height >= 0.03)
                self.assertEqual(r.score, score)
                self.assertEqual(r.details["lifted_ever"], score == 0.25)

    def test_near_without_lift_scores_zero(self):
        r = self.play(self.near_state(0.05))[0]
        self.assertEqual((r.score, r.details["near"], r.details["lifted_ever"]), (0.0, True, False))

    def test_lift_and_near_in_the_same_step(self):
        r = self.play(self.near_state(0.05, height_gain=0.04))[0]
        self.assertEqual((r.score, r.details["lifted_ever"], r.details["near"]), (0.25, True, True))

    def test_lifted_latch_survives_the_plug_returning_to_the_table(self):
        r = self.play(self.lift_state(0.05), self.world_state([*G.PLUG_XY0, G.PLUG_Z0]), self.near_state(0.08))[2]
        self.assertEqual(r.score, 0.25)

    def test_engaged_beats_lifted_near(self):
        r = self.play(self.lift_state(0.05), self.state(depth=0.010))[1]
        self.assertEqual((r.score, r.details["tier"]), (0.5, S.TIER_ENGAGED))

    # -- latched failures
    def test_block_moved_failure_is_latched(self):
        results = self.play(self.state(), self.state(moved=0.031), self.state(), self.state(),
                            self.state(depth=0.010), self.near_state(0.05, height_gain=0.05))
        self.assertEqual([r.score for r in results], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual([r.success for r in results], [True] + [False] * 5)
        self.assertEqual([r.details["failed"] for r in results], [False] + [True] * 5)
        self.assertEqual(results[1].details["tier"], S.TIER_FAILED)
        self.assertEqual(results[1].details["fail_reason"], "block_moved")
        self.assertTrue(results[2].details["seated"])  # the quantities are still reported

    def test_block_moved_within_limit_is_not_latched(self):
        for moved in (0.029, 0.030):
            with self.subTest(moved_mm=moved * 1000):
                r = self.play(self.state(moved=moved), self.state())[1]
                self.assertEqual((r.score, r.details["failed"]), (1.0, False))

    def test_block_moved_from_start_not_from_previous_step(self):
        # 20 mm, then 40 mm from the start (20 mm from the previous step): fails on distance from the start
        r = self.play(self.state(moved=0.02), self.state(moved=0.04))[1]
        self.assertEqual((r.score, r.details["failed"]), (0.0, True))

    def test_tilt_blip_is_latched(self):
        for axis in [(1, 0, 0), (0, 1, 0), (1, 1, 0), (-1, 2, 0)]:
            for tilt, failed in [(19.0, False), (19.99, False), (20.0, False), (20.001, True), (21.0, True),
                                 (90.0, True), (180.0, True)]:
                with self.subTest(axis=axis, tilt=tilt):
                    blip_q = q_mul(q_axis_angle(axis, tilt), self.level_q)
                    results = self.play(self.state(block_q=blip_q), self.state(), self.state())
                    self.assertAlmostEqual(results[0].details["block_tilt"], tilt, places=6)
                    self.assertEqual([r.details["failed"] for r in results], [failed] * 3)
                    if failed:
                        self.assertEqual(results[0].details["fail_reason"], "block_tilted")
                        self.assertEqual([r.score for r in results], [0.0] * 3)
                        self.assertEqual([r.success for r in results], [False] * 3)
                    else:
                        self.assertEqual(results[1].score, 1.0)
                        self.assertTrue(results[1].success)

    def test_yaw_alone_is_not_tilt(self):
        for yaw in (0.0, 45.0, 90.0, 135.0, 180.0, -170.0):
            with self.subTest(yaw=yaw):
                r = self.score_of(block_q=q_axis_angle((0, 0, 1), yaw))
                self.assertAlmostEqual(r.details["block_tilt"], 0.0, places=4)
                self.assertFalse(r.details["failed"])

    def test_fallen_plug_is_latched(self):
        for below, failed in [(0.0, False), (0.0195, False), (0.02 - 1e-6, False), (0.02, False),
                              (0.02 + 1e-6, True), (0.0205, True), (0.3, True)]:
            with self.subTest(below_mm=below * 1000):
                fell, after, again = self.play(self.fall_state(below), self.state(), self.state())
                self.assertEqual(fell.score, 0.0)
                self.assertEqual(fell.details["failed"], failed)
                self.assertAlmostEqual(fell.details["plug_height_gain"], -below, places=9)
                self.assertEqual((after.score, after.success, after.details["failed"]),
                                 (0.0, False, True) if failed else (1.0, True, False))
                self.assertEqual(again.score, after.score)
                if failed:
                    self.assertEqual(fell.details["fail_reason"], "plug_fell")

    def test_failures_are_independent_and_first_reason_is_kept(self):
        blip_q = q_mul(q_axis_angle((1, 0, 0), 25.0), self.level_q)
        results = self.play(self.state(block_q=blip_q), self.state(moved=0.05), self.fall_state(0.05))
        self.assertTrue(all(r.details["failed"] for r in results))
        self.assertEqual([r.details["fail_reason"] for r in results], ["block_tilted"] * 3)

    # -- reset
    def test_reset_clears_every_latch(self):
        for name, bad in [("moved", lambda: self.state(moved=0.05)),
                          ("tilt", lambda: self.state(block_q=q_mul(q_axis_angle((1, 0, 0), 30.0), self.level_q))),
                          ("fell", lambda: self.fall_state(0.05))]:
            with self.subTest(failure=name):
                s = self.fresh()
                self.assertTrue(s.update(**bad()).details["failed"])
                self.assertTrue(s.update(**self.state()).details["failed"])
                s.reset()
                self.assertFalse(s.failed)
                s.update(**self.start)
                r = s.update(**self.state())
                self.assertEqual((r.score, r.success, r.details["failed"]), (1.0, True, False))

    def test_reset_clears_lifted_ever(self):
        s = self.fresh()
        self.assertTrue(s.update(**self.lift_state(0.05)).details["lifted_ever"])
        self.assertEqual(s.update(**self.near_state(0.05)).score, 0.25)
        s.reset()
        s.update(**self.start)
        r = s.update(**self.near_state(0.05))
        self.assertEqual((r.score, r.details["lifted_ever"]), (0.0, False))

    def test_reset_makes_the_next_update_recapture_start_poses(self):
        s = self.fresh()
        self.assertTrue(s.update(**self.state(moved=0.05)).details["failed"])
        s.reset()
        # new episode: the block starts 5 cm away from where it started before, and the plug 10 cm lower
        shifted = self.block_pos0 + 0.05 * MOVE_DIR
        new_start_z = START_PLUG_POS[2] - 0.10
        first = with_plug(build(shifted, self.block_q0, (0, 0, 0)), START_PLUG_POS - [0, 0, 0.10])
        r = s.update(**first)
        self.assertEqual((r.details["block_moved"], r.details["plug_height_gain"]), (0.0, 0.0))
        seated = build(shifted, self.block_q0, (0, G.MOUTH_Y, 0))
        r = s.update(**seated)
        self.assertEqual((r.score, r.details["failed"]), (1.0, False))
        self.assertAlmostEqual(r.details["block_moved"], 0.0, places=9)
        # the plug's height is measured from the new start: the seated plug is now > 3 cm above it ("lifted")
        self.assertAlmostEqual(r.details["plug_height_gain"], seated["plug_pos"][2] - new_start_z, places=9)
        self.assertTrue(r.details["lifted_ever"])

    def test_priorities_seated_beats_everything(self):
        r = self.play(self.lift_state(0.05), self.state())[1]
        self.assertEqual((r.score, r.details["tier"]), (1.0, S.TIER_SUCCESS))


class TierCasesIdentityBlock(_TierCases, unittest.TestCase):
    block_pos0 = np.array(G.BLOCK_POS)
    block_q0 = Q_IDENT
    level_q = Q_IDENT


class TierCasesYawedAndOffsetBlock(_TierCases, unittest.TestCase):
    """The block yawed 30 degrees and moved elsewhere in the world."""

    block_pos0 = np.array([0.33, 0.04, G.BLOCK_POS[2]])
    block_q0 = q_axis_angle((0, 0, 1), 30.0)
    level_q = block_q0


class TierCasesTiltedBlock(_TierCases, unittest.TestCase):
    """A block that rests slightly tilted (about 10 degrees, below the 20 degree failure) with an arbitrary yaw."""

    block_pos0 = np.array([0.25, 0.12, 0.79])
    level_q = q_axis_angle((0, 0, 1), -50.0)
    block_q0 = q_mul(q_axis_angle((1, 0, 0), 7.0), q_mul(q_axis_angle((0, 1, 0), -8.0), level_q))


class TierCasesBlockYawed180(_TierCases, unittest.TestCase):
    block_pos0 = np.array([0.40, -0.10, G.BLOCK_POS[2]])
    block_q0 = np.array([0.0, 0.0, 1.0, 0.0])
    level_q = block_q0


# -- the block frame is really used --------------------------------------------------------------------------------
class TestBlockFrame(unittest.TestCase):
    def test_block_yawed_90_hand_computed(self):
        # Yaw 90 maps block-frame +y to world -x, so the mouth (block y = MOUTH_Y < 0) is at +x of the block.
        block = np.array([0.30, 0.10, G.SEAT_Z])
        q = q_axis_angle((0, 0, 1), 90.0)
        mouth = block + np.array([-G.MOUTH_Y, 0.0, 0.0])
        start = with_plug(build(block, q, (0, 0, 0)), START_PLUG_POS)
        s = S.Scorer()
        s.update(**start)
        r = s.update(**with_plug(start, mouth, q))
        self.assertEqual((r.score, r.success), (1.0, True))
        np.testing.assert_allclose(r.details["p_local"], [0.0, G.MOUTH_Y, 0.0], atol=1e-12)
        # the same placement as if the block were unrotated (plug 8.5 cm to its -y side) is nowhere near
        s = S.Scorer()
        s.update(**start)
        r = s.update(**with_plug(start, block + [0.0, G.MOUTH_Y, 0.0], q))
        self.assertEqual(r.score, 0.0)

    def test_world_frame_placement_is_wrong_for_a_yawed_block(self):
        block = np.array([0.33, 0.04, G.SEAT_Z])
        q = q_axis_angle((0, 0, 1), 30.0)
        start = with_plug(build(block, q, (0, 0, 0)), START_PLUG_POS)
        naive = block + [0.0, G.MOUTH_Y, 0.0]  # right if the block were at yaw 0
        s = S.Scorer()
        s.update(**start)
        self.assertEqual(s.update(**with_plug(start, naive, q)).score, 0.0)
        s = S.Scorer()
        s.update(**start)
        self.assertEqual(s.update(**with_plug(start, naive, Q_IDENT)).score, 0.0)

    def test_relative_rotation_is_between_plug_and_block(self):
        block = np.array([0.33, 0.04, G.SEAT_Z])
        bq = q_axis_angle((0, 0, 1), 30.0)
        mouth = block + q_rotate(bq, (0.0, G.MOUTH_Y, 0.0))
        start = with_plug(build(block, bq, (0, 0, 0)), START_PLUG_POS)
        for plug_yaw, ang, score in [(30.0, 0.0, 1.0), (33.0, 3.0, 1.0), (27.0, 3.0, 1.0), (33.1, 3.1, 0.5),
                                     (35.0, 5.0, 0.5), (35.2, 5.2, 0.0), (0.0, 30.0, 0.0), (210.0, 180.0, 0.0)]:
            with self.subTest(plug_yaw=plug_yaw):
                s = S.Scorer()
                s.update(**start)
                r = s.update(**with_plug(start, mouth, q_axis_angle((0, 0, 1), plug_yaw)))
                self.assertAlmostEqual(r.details["ang"], ang, places=6)
                self.assertEqual(r.score, score)

    def test_pitched_block_lateral_axis_follows_the_block(self):
        # Block pitched 10 degrees about x. A plug offset 2 mm along the block's own z axis is 2 mm laterally off;
        # the same 2 mm along world z is not (it also moves along the socket axis).
        block = np.array([0.25, 0.12, 0.79])
        bq = q_axis_angle((1, 0, 0), 10.0)
        s = S.Scorer()
        s.update(**with_plug(build(block, bq, (0, 0, 0)), START_PLUG_POS))
        r = s.update(**build(block, bq, (0.0, G.MOUTH_Y, 0.002)))
        self.assertAlmostEqual(r.details["lateral"], 0.002, places=9)
        self.assertEqual(r.score, 0.5)
        r = s.update(**build(block, bq, (0.0, G.MOUTH_Y, 0.0)))
        self.assertEqual(r.score, 1.0)


# -- quaternion handling -------------------------------------------------------------------------------------------
class TestQuaternions(unittest.TestCase):
    def test_identity(self):
        np.testing.assert_allclose(S.quat_to_matrix([0, 0, 0, 1]), np.eye(3), atol=1e-15)

    def test_xyzw_order(self):
        # (1, 0, 0, 0) is a half turn about x (w is last); (0, 0, 0, 1) is the identity.
        np.testing.assert_allclose(S.quat_to_matrix([1, 0, 0, 0]), np.diag([1.0, -1.0, -1.0]), atol=1e-15)
        np.testing.assert_allclose(S.quat_to_matrix([0, 1, 0, 0]), np.diag([-1.0, 1.0, -1.0]), atol=1e-15)

    def test_yaw_180(self):
        np.testing.assert_allclose(S.quat_to_matrix([0, 0, 1, 0]), np.diag([-1.0, -1.0, 1.0]), atol=1e-15)
        np.testing.assert_allclose(S.quat_to_matrix(q_axis_angle((0, 0, 1), 180.0)), np.diag([-1.0, -1.0, 1.0]),
                                   atol=1e-15)

    def test_yaw_90_and_roll_90(self):
        np.testing.assert_allclose(S.quat_to_matrix(q_axis_angle((0, 0, 1), 90.0)),
                                   [[0, -1, 0], [1, 0, 0], [0, 0, 1]], atol=1e-15)
        np.testing.assert_allclose(S.quat_to_matrix(q_axis_angle((1, 0, 0), 90.0)),
                                   [[1, 0, 0], [0, 0, -1], [0, 1, 0]], atol=1e-15)

    def test_non_unit_input_is_normalised(self):
        for q in ([0, 0, 0, 5], [0, 0, 2, 0], [0.3, -0.2, 0.9, 0.1], [1e-3, 2e-3, -1e-3, 3e-3]):
            unit = np.array(q, dtype=float) / np.linalg.norm(q)
            for scale in (0.001, 0.5, 3.7, 1000.0):
                with self.subTest(q=q, scale=scale):
                    np.testing.assert_allclose(S.quat_to_matrix(scale * np.array(q, dtype=float)),
                                               S.quat_to_matrix(unit), atol=1e-12)
        np.testing.assert_allclose(S.quat_to_matrix([0, 0, 0, 5]), np.eye(3), atol=1e-15)

    def test_negated_quaternion_is_the_same_rotation(self):
        q = np.array([0.3, -0.2, 0.9, 0.1])
        np.testing.assert_allclose(S.quat_to_matrix(q), S.quat_to_matrix(-q), atol=1e-15)

    def test_matches_axis_angle_for_random_rotations(self):
        rng = np.random.default_rng(1)
        for _ in range(200):
            axis = rng.normal(size=3)
            deg = rng.uniform(-180, 180)
            v = rng.normal(size=3)
            R = S.quat_to_matrix(q_axis_angle(axis, deg))
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(R), 1.0, places=12)
            np.testing.assert_allclose(R @ v, q_rotate(q_axis_angle(axis, deg), v), atol=1e-12)
            self.assertAlmostEqual(S.rotation_angle_deg(R), abs(deg), places=6)

    def test_bad_quaternions_raise(self):
        for q in ([0, 0, 0, 0], [np.nan, 0, 0, 1], [np.inf, 0, 0, 1], [0, 0, 1], [0, 0, 0, 1, 0]):
            with self.subTest(q=q):
                with self.assertRaises(ValueError):
                    S.quat_to_matrix(q)

    def test_scorer_accepts_non_unit_quaternions(self):
        for scale_b, scale_p in [(1.0, 1.0), (3.7, 0.25), (0.01, 40.0)]:
            with self.subTest(scale_b=scale_b, scale_p=scale_p):
                bq = q_axis_angle((0.2, -0.3, 1.0), 40.0)
                kw = build([0.3, 0.1, 0.78], bq, (0.0, G.MOUTH_Y, 0.0))
                start = with_plug(kw, START_PLUG_POS)
                s = S.Scorer()
                s.update(**start)
                kw["block_quat_xyzw"] = kw["block_quat_xyzw"] * scale_b
                kw["plug_quat_xyzw"] = kw["plug_quat_xyzw"] * scale_p
                r = s.update(**kw)
                self.assertEqual(r.score, 1.0)
                self.assertTrue(r.success)

    def test_scorer_yaw_180_block_and_plug(self):
        block = np.array([0.4, -0.1, G.SEAT_Z])
        bq = np.array([0.0, 0.0, 1.0, 0.0])
        start = with_plug(build(block, bq, (0, 0, 0)), START_PLUG_POS)
        s = S.Scorer()
        s.update(**start)
        r = s.update(**build(block, bq, (0.0, G.MOUTH_Y, 0.0)))
        self.assertEqual(r.score, 1.0)
        np.testing.assert_allclose(block + [0.0, -G.MOUTH_Y, 0.0], build(block, bq, (0.0, G.MOUTH_Y, 0.0))["plug_pos"],
                                   atol=1e-12)  # mouth is on the +y side of a 180 degree yawed block
        # plug facing the opposite way (relative yaw 180)
        r = s.update(**build(block, bq, (0.0, G.MOUTH_Y, 0.0), q_axis_angle((0, 0, 1), 180.0)))
        self.assertEqual(r.score, 0.0)
        self.assertAlmostEqual(r.details["ang"], 180.0, places=3)

    def test_scorer_negated_plug_quaternion(self):
        kw = build(G.BLOCK_POS, Q_IDENT, (0.0, G.MOUTH_Y, 0.0), q_axis_angle((0, 1, 0), 2.0))
        s = S.Scorer()
        s.update(**with_plug(kw, START_PLUG_POS))
        a = s.update(**kw)
        kw["plug_quat_xyzw"] = -kw["plug_quat_xyzw"]
        b = s.update(**kw)
        self.assertEqual((a.score, b.score), (1.0, 1.0))
        self.assertAlmostEqual(a.details["ang"], b.details["ang"], places=9)


# -- module-level maths --------------------------------------------------------------------------------------------
class TestMaths(unittest.TestCase):
    def test_plug_pose_in_block_known_values(self):
        R_block = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # yaw 90
        block = np.array([1.0, 2.0, 3.0])
        plug = block + R_block @ np.array([0.01, 0.1, 0.02])  # = (0.9, 2.01, 3.02)
        np.testing.assert_allclose(plug, [0.9, 2.01, 3.02])
        p, rel = S.plug_pose_in_block(plug, R_block, block, R_block)
        np.testing.assert_allclose(p, [0.01, 0.1, 0.02], atol=1e-15)
        np.testing.assert_allclose(rel, np.eye(3), atol=1e-15)

    def test_plug_pose_in_block_relative_rotation(self):
        Rb = S.quat_to_matrix(q_axis_angle((0, 0, 1), 30.0))
        Rp = S.quat_to_matrix(q_axis_angle((0, 0, 1), 75.0))
        _, rel = S.plug_pose_in_block([0, 0, 0], Rp, [0, 0, 0], Rb)
        np.testing.assert_allclose(rel, S.quat_to_matrix(q_axis_angle((0, 0, 1), 45.0)), atol=1e-12)
        self.assertAlmostEqual(S.rotation_angle_deg(rel), 45.0, places=9)

    def test_insertion_depth(self):
        for y, depth in [(G.MOUTH_Y - G.NOSE_L, 0.0), (G.MOUTH_Y - 0.1, 0.0), (-5.0, 0.0),
                         (G.MOUTH_Y - G.NOSE_L + 0.006, 0.006), (G.MOUTH_Y - 0.001, 0.017), (G.MOUTH_Y, G.NOSE_L),
                         (G.MOUTH_Y + 0.004, G.NOSE_L), (5.0, G.NOSE_L)]:
            with self.subTest(y=y):
                self.assertAlmostEqual(S.insertion_depth([0.0, y, 0.0]), depth, places=12)

    def test_lateral_offset(self):
        self.assertEqual(S.lateral_offset([0.0, 0.5, 0.0]), 0.0)  # y does not enter
        self.assertEqual(S.lateral_offset([-0.002, 0.0, 0.001]), 0.002)
        self.assertEqual(S.lateral_offset([0.001, 0.0, -0.003]), 0.003)

    def test_rotation_angle_deg(self):
        for axis in [(1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 1)]:
            for deg in (0.0, 1.0, 2.9, 45.0, 90.0, 120.0, 179.0, 180.0):
                with self.subTest(axis=axis, deg=deg):
                    R = S.quat_to_matrix(q_axis_angle(axis, deg))
                    self.assertAlmostEqual(S.rotation_angle_deg(R), deg, places=6)
        self.assertEqual(S.rotation_angle_deg(np.eye(3)), 0.0)
        self.assertAlmostEqual(S.rotation_angle_deg(np.diag([-1.0, -1.0, 1.0])), 180.0, places=12)

    def test_rotation_angle_deg_clips_round_off(self):
        self.assertEqual(S.rotation_angle_deg(np.eye(3) * (1.0 + 1e-10)), 0.0)  # (trace - 1) / 2 slightly above 1
        self.assertAlmostEqual(S.rotation_angle_deg(np.eye(3) * -(1.0 + 1e-10) + np.diag([0, 0, 2.0])), 180.0,
                               places=12)

    def test_block_tilt_deg(self):
        self.assertEqual(S.block_tilt_deg(np.eye(3)), 0.0)
        for yaw in (30.0, 90.0, 180.0):
            self.assertAlmostEqual(S.block_tilt_deg(S.quat_to_matrix(q_axis_angle((0, 0, 1), yaw))), 0.0, places=6)
        for axis in [(1, 0, 0), (0, 1, 0), (1, 1, 0)]:
            for deg in (10.0, 20.0, 90.0, 180.0):
                R = S.quat_to_matrix(q_axis_angle(axis, deg))
                self.assertAlmostEqual(S.block_tilt_deg(R), deg, places=6)

    def test_xy_displacement(self):
        self.assertEqual(S.xy_displacement([0.3, 0.4, 9.0], [0.0, 0.0, -9.0]), 0.5)
        self.assertEqual(S.xy_displacement([1.0, 1.0, 1.0], [1.0, 1.0, 5.0]), 0.0)

    def test_mouth_world(self):
        R = S.quat_to_matrix(q_axis_angle((0, 0, 1), 90.0))
        np.testing.assert_allclose(S.mouth_world([1.0, 2.0, 3.0], R), [1.0 - G.MOUTH_Y, 2.0, 3.0], atol=1e-12)
        np.testing.assert_allclose(S.mouth_world([1.0, 2.0, 3.0], np.eye(3)), [1.0, 2.0 + G.MOUTH_Y, 3.0])

    def test_thresholds_are_consistent_with_each_other_and_with_geometry(self):
        self.assertLess(S.ENGAGED_DEPTH, S.SEATED_DEPTH)
        self.assertLessEqual(S.SEATED_DEPTH, G.NOSE_L)  # reachable: the nose cannot go deeper than NOSE_L
        self.assertGreater(G.NOSE_L - S.SEATED_DEPTH, 0.0)
        self.assertLess(S.SEATED_ANG_DEG, S.ENGAGED_ANG_DEG)
        self.assertLess(S.SEATED_LATERAL, S.ENGAGED_LATERAL)
        self.assertGreater(S.SEATED_LATERAL, G.CLEAR)  # looser than the mechanical play, so a seated plug passes
        self.assertLess(S.BLOCK_MOVED_OK, S.BLOCK_MOVED_FAIL)
        self.assertLess(S.DETENT_MIN, 0.0)
        self.assertGreater(S.LIFT_HEIGHT, G.SEAT_Z - G.PLUG_Z0)  # seating alone does not satisfy "lifted"
        self.assertLess(S.TOL, 1e-6)


# -- API behaviour -------------------------------------------------------------------------------------------------
class TestApi(unittest.TestCase):
    def seated(self, **kw):
        return build(G.BLOCK_POS, Q_IDENT, (0.0, G.MOUTH_Y, 0.0), **kw)

    def start(self):
        return with_plug(build(G.BLOCK_POS, Q_IDENT, (0, 0, 0)), START_PLUG_POS)

    def test_details_keys_and_types(self):
        s = S.Scorer()
        s.update(**self.start())
        r = s.update(**self.seated())
        self.assertIsInstance(r, S.StepResult)
        self.assertIsInstance(r.score, float)
        self.assertIsInstance(r.success, bool)
        for key in ("depth", "ang", "lateral", "block_moved", "block_tilt", "still", "lifted_ever", "near", "seated",
                    "failed", "tier"):
            self.assertIn(key, r.details)
        json.dumps(r.details)  # plain Python types only
        for key in ("still", "lifted_ever", "near", "seated", "failed", "engaged"):
            self.assertIsInstance(r.details[key], bool, key)

    def test_all_tier_names_are_reported(self):
        s = S.Scorer()
        s.update(**self.start())
        seen = {s.update(**self.seated()).details["tier"],
                s.update(**self.seated(grippers_open=False)).details["tier"],
                s.update(**build(G.BLOCK_POS + 0.01 * MOVE_DIR, Q_IDENT, (0.0, G.MOUTH_Y, 0.0))).details["tier"],
                s.update(**build(G.BLOCK_POS, Q_IDENT, (0.0, G.MOUTH_Y - 0.01, 0.0))).details["tier"],
                s.update(**with_plug(self.start(), START_PLUG_POS)).details["tier"]}
        self.assertEqual(seen, {S.TIER_SUCCESS, S.TIER_SEATED_HELD, S.TIER_SEATED_BLOCK_MOVED, S.TIER_ENGAGED,
                                S.TIER_NONE})

    def test_new_scorer_needs_no_reset_and_reset_twice_is_harmless(self):
        s = S.Scorer()
        s.reset()
        s.reset()
        s.update(**self.start())
        self.assertEqual(s.update(**self.seated()).score, 1.0)

    def test_first_update_captures_start_poses(self):
        block = np.array([0.5, -0.3, 0.8])
        s = S.Scorer()
        first = with_plug(build(block, Q_IDENT, (0, 0, 0)), [0.0, 0.0, 1.0])
        r = s.update(**first)
        self.assertEqual((r.details["block_moved"], r.details["plug_height_gain"]), (0.0, 0.0))
        r = s.update(**with_plug(build(block + [0.0049, 0.0, 0.0], Q_IDENT, (0, 0, 0)), [0.0, 0.0, 1.0]))
        self.assertAlmostEqual(r.details["block_moved"], 0.0049, places=12)
        self.assertFalse(r.details["failed"])
        r = s.update(**with_plug(build(block + [0.0, 0.031, 0.0], Q_IDENT, (0, 0, 0)), [0.0, 0.0, 1.0]))
        self.assertTrue(r.details["failed"])

    def test_first_update_may_already_be_seated(self):
        r = S.Scorer().update(**self.seated())
        self.assertEqual((r.score, r.success), (1.0, True))
        self.assertEqual(r.details["block_moved"], 0.0)

    def test_first_update_sets_the_plug_height_reference(self):
        for first_z, second_z, expect in [(1.0, 1.0 - 0.0195, "ok"), (1.0, 1.0 - 0.0205, "fell"),
                                          (1.0, 1.03, "lifted"), (1.0, 1.0 + 0.029, "ok"), (0.2, 0.2 - 0.0205, "fell"),
                                          (0.7615, 0.7615 - 0.01, "ok")]:
            with self.subTest(first_z=first_z, second_z=second_z):
                s = S.Scorer()
                s.update(**with_plug(self.start(), [0.2, -0.12, first_z]))
                r = s.update(**with_plug(self.start(), [0.2, -0.12, second_z]))
                self.assertEqual(r.details["failed"], expect == "fell")
                self.assertEqual(r.details["lifted_ever"], expect == "lifted")

    def test_inputs_are_copied_not_aliased(self):
        # a simulator often hands out views into one buffer that it overwrites each step
        block_pos = np.array([0.3, 0.1, 0.78])
        s = S.Scorer()
        s.update(**with_plug(build(block_pos, Q_IDENT, (0, 0, 0)), START_PLUG_POS) | {"block_pos": block_pos})
        block_pos += [0.0049, 0.0, 0.0]  # in place, same array object
        r = s.update(**with_plug(build(block_pos, Q_IDENT, (0, 0, 0)), START_PLUG_POS) | {"block_pos": block_pos})
        self.assertAlmostEqual(r.details["block_moved"], 0.0049, places=12)
        plug_pos = START_PLUG_POS.copy()
        s = S.Scorer()
        s.update(**self.start() | {"plug_pos": plug_pos})
        plug_pos[2] += 0.05  # lifted in place
        self.assertTrue(s.update(**self.start() | {"plug_pos": plug_pos}).details["lifted_ever"])

    def test_inputs_are_not_modified(self):
        kw = self.seated()
        before = {k: np.array(v) if not isinstance(v, (bool, float)) else v for k, v in kw.items()}
        S.Scorer().update(**kw)
        for k, v in before.items():
            np.testing.assert_array_equal(kw[k], v)

    def test_accepts_lists_tuples_and_numpy_scalars(self):
        kw = self.seated()
        listed = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in kw.items()}
        listed["detent"] = np.float64(0.0)
        listed["grippers_open"] = np.bool_(True)
        s = S.Scorer()
        s.update(**self.start())
        r = s.update(**listed)
        self.assertEqual((r.score, r.success), (1.0, True))
        self.assertIsInstance(r.success, bool)

    def test_bad_inputs_raise(self):
        base = self.seated()
        for key, bad in [("plug_pos", [0, 0]), ("plug_pos", [0, 0, np.nan]), ("block_pos", [0, 0, 0, 0]),
                         ("plug_lin_vel", [np.inf, 0, 0]), ("block_ang_vel", [0, 0]),
                         ("plug_quat_xyzw", [0, 0, 0, 0]), ("block_quat_xyzw", [0, 0, 0]),
                         ("detent", float("nan"))]:
            with self.subTest(key=key, bad=bad):
                with self.assertRaises(ValueError):
                    S.Scorer().update(**{**base, key: bad})

    def test_failed_property(self):
        s = S.Scorer()
        s.update(**self.start())
        self.assertFalse(s.failed)
        s.update(**build(G.BLOCK_POS + 0.05 * MOVE_DIR, Q_IDENT, (0.0, G.MOUTH_Y, 0.0)))
        self.assertTrue(s.failed)
        s.reset()
        self.assertFalse(s.failed)


# -- an independent reference implementation of the rules as written in the task brief ----------------------------
class _Reference:
    """The scoring rules transcribed from the task brief with literal numbers, using quaternion maths instead of
    matrices (so it shares neither code nor constants with scoring.py)."""

    def __init__(self):
        self.start = None
        self.lifted = False
        self.failed = False

    def step(self, k):
        pp, bp = np.asarray(k["plug_pos"], float), np.asarray(k["block_pos"], float)
        qp = np.asarray(k["plug_quat_xyzw"], float)
        qb = np.asarray(k["block_quat_xyzw"], float)
        qp, qb = qp / np.linalg.norm(qp), qb / np.linalg.norm(qb)
        if self.start is None:
            self.start = (pp[2], bp[:2].copy())
        loc = q_rotate(q_conj(qb), pp - bp)
        depth = min(max((loc[1] + G.NOSE_L) - G.MOUTH_Y, 0.0), G.NOSE_L)
        lateral = max(abs(loc[0]), abs(loc[2]))
        ang = math.degrees(2.0 * math.acos(min(1.0, abs(q_mul(q_conj(qb), qp)[3]))))
        moved = float(np.hypot(*(bp[:2] - self.start[1])))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, q_rotate(qb, (0, 0, 1))[2]))))
        norm = lambda key: float(np.linalg.norm(k[key]))  # noqa: E731
        still = (norm("plug_lin_vel") < 0.005 and norm("plug_ang_vel") < 0.2 and norm("block_lin_vel") < 0.005
                 and norm("block_ang_vel") < 0.2)
        if pp[2] - self.start[0] >= 0.03:
            self.lifted = True
        near = float(np.linalg.norm(pp - (bp + q_rotate(qb, (0.0, G.MOUTH_Y, 0.0))))) <= 0.10
        if moved > 0.03 or tilt > 20.0 or self.start[0] - pp[2] > 0.02:
            self.failed = True
        if self.failed:
            return 0.0, False
        seated = depth >= 0.017 and ang <= 3.0 and lateral <= 0.0015 and k["detent"] > -0.0003
        if seated and moved <= 0.005 and k["grippers_open"] and still:
            return 1.0, True
        if seated and moved <= 0.005:
            return 0.75, False
        if (seated and moved > 0.005) or (depth >= 0.006 and ang <= 5.0 and lateral <= 0.003):
            return 0.5, False
        if self.lifted and near:
            return 0.25, False
        return 0.0, False


class TestAgainstReference(unittest.TestCase):
    def test_random_episodes_match_the_reference(self):
        rng = np.random.default_rng(20240607)
        tiers_seen = set()
        n_failed = 0
        counts = {}
        for episode in range(400):
            block_pos = np.array([rng.uniform(0.1, 0.4), rng.uniform(-0.2, 0.2), rng.uniform(0.75, 0.8)])
            level_q = q_axis_angle((0, 0, 1), rng.uniform(-180, 180))
            tilt_axis = (rng.normal(), rng.normal(), 0.0)
            base_q = q_mul(q_axis_angle(tilt_axis, rng.uniform(0, 8)), level_q)
            plug_start = START_PLUG_POS + [rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05), 0.0]
            ref, scorer = _Reference(), S.Scorer()
            first = with_plug(build(block_pos, base_q, (0, 0, 0)), plug_start)
            self.assertEqual(scorer.update(**first).score, ref.step(first)[0])
            for _ in range(5):
                bq = base_q
                if rng.random() < 0.08:  # occasional big tilt, sometimes past 20 degrees
                    bq = q_mul(q_axis_angle(tilt_axis, rng.uniform(0, 30)), level_q)
                moved = rng.choice([0.0, rng.uniform(0, 0.007), rng.uniform(0, 0.035)], p=[0.5, 0.3, 0.2])
                pos = block_pos + moved * np.array([math.cos(a := rng.uniform(0, 6.28)), math.sin(a), 0.0])
                still_p = [0.75, 0.125, 0.125]
                speeds = dict(v_plug=rng.choice([0.0, 0.003, 0.01], p=still_p),
                              w_plug=rng.choice([0.0, 0.1, 0.5], p=still_p),
                              v_block=rng.choice([0.0, 0.003, 0.01], p=still_p),
                              w_block=rng.choice([0.0, 0.1, 0.5], p=still_p))
                mode = rng.random()
                if mode < 0.35:  # tight around the seated pose: mostly seated, so the upper tiers get exercised
                    kw = build(pos, bq, (rng.uniform(-0.002, 0.002), G.MOUTH_Y + rng.uniform(-0.0015, 0.0005),
                                         rng.uniform(-0.002, 0.002)),
                               q_axis_angle(rng.normal(size=3), rng.uniform(0, 4)),
                               detent=rng.choice([0.0, 0.0, 0.0, -0.0002, -0.0005]),
                               grippers_open=bool(rng.random() < 0.8), **speeds)
                elif mode < 0.65:  # loosely around the seated pose
                    kw = build(pos, bq, (rng.uniform(-0.004, 0.004), G.MOUTH_Y + rng.uniform(-0.014, 0.0005),
                                         rng.uniform(-0.004, 0.004)),
                               q_axis_angle(rng.normal(size=3), rng.uniform(0, 7)),
                               detent=rng.choice([0.0, 0.0, -0.0002, -0.0005]),
                               grippers_open=bool(rng.random() < 0.7), **speeds)
                else:  # somewhere in the neighbourhood of the mouth, any height
                    mouth = pos + q_rotate(bq / np.linalg.norm(bq), (0.0, G.MOUTH_Y, 0.0))
                    kw = with_plug(build(pos, bq, (0, 0, 0), grippers_open=bool(rng.random() < 0.7), **speeds),
                                   mouth + rng.uniform(-0.15, 0.15, size=3) + [0, 0, rng.choice([0.0, 0.05, -0.03])],
                                   q_axis_angle(rng.normal(size=3), rng.uniform(0, 20)))
                got = scorer.update(**kw)
                want_score, want_success = ref.step(kw)
                self.assertEqual((got.score, got.success), (want_score, want_success),
                                 msg=f"episode {episode}: {got.details}")
                self.assertEqual(got.details["failed"], ref.failed)
                tiers_seen.add(got.details["tier"])
                counts[got.details["tier"]] = counts.get(got.details["tier"], 0) + 1
                n_failed += got.details["failed"]
        self.assertEqual(tiers_seen, {S.TIER_SUCCESS, S.TIER_SEATED_HELD, S.TIER_SEATED_BLOCK_MOVED,
                                      S.TIER_ENGAGED, S.TIER_LIFTED_NEAR, S.TIER_NONE, S.TIER_FAILED})
        self.assertGreater(n_failed, 20)
        self.assertGreater(counts[S.TIER_SUCCESS], 20)


if __name__ == "__main__":
    unittest.main()
