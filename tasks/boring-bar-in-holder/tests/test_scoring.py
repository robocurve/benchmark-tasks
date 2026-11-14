"""Offline tests for layout.py, setscrew.py, scoring.py, offline_boring.py and task.py: synthetic bar and key states
(mechkit AssemblyStates from the task's own specs) and synthetic arm snapshots (rigstate.Arm), no simulator.
    python3 -m unittest discover -s tests -v"""

import json
import math
import pathlib
import re
import subprocess
import sys
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))


def _offline_dir():
    for p in HERE.parents:
        d = p / "tools" / "offline"
        if (d / "yam_stub" / "yam_arena").is_dir():
            return d
    return None


OFFLINE = _offline_dir()
if OFFLINE is not None:
    for _p in (OFFLINE, OFFLINE / "yam_stub"):
        if str(_p) not in sys.path:
            sys.path.insert(0, str(_p))

import layout as L  # noqa: E402
import mechkit as mk  # noqa: E402
import rigstate  # noqa: E402
import scoring as S  # noqa: E402
import setscrew as SS  # noqa: E402
import task as T  # noqa: E402
from yam_arena import demo as D  # noqa: E402

MM = L.MM
AWAY = {a: rigstate.Arm(np.array([0.28, s * 0.25, L.TOP + 0.12]), np.array([0.28, s * 0.25, L.TOP + 0.107]), 0.64)
        for a, s in (("left", 1.0), ("right", -1.0))}


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def bar_rot(flat_deg=0.0, yaw=math.pi / 2, pitch_deg=0.0):
    r = rz(yaw) @ L.rx(math.radians(flat_deg))
    if pitch_deg:  # the axis tipped up about the world's x
        r = L.rx(math.radians(pitch_deg)) @ r
    return r


def bar_state(s=46.5, flat_deg=0.0, yaw=math.pi / 2, dz=-0.0002, dx=0.0, rot=None, tip_y=None):
    r = bar_rot(flat_deg, yaw) if rot is None else rot
    a = r[:, 0]
    tip = np.array([L.X_AX * MM + dx, (L.Y_FACE + s) * MM if tip_y is None else tip_y, L.TOP + L.Z_AX * MM + dz])
    origin = tip - a * L.BAR_L / 2 * MM
    return mk.state_from_poses(T.BAR_SPEC, (*origin, *mk.matrix_to_quat(r)), np.zeros(6), source="offline")


def key_state(x=None, y=None, tip_z=None, heading=0.0, tilt_deg=0.0):
    x = L.X_AX * MM if x is None else x
    y = L.SCREW_Y[0] * MM if y is None else y
    z = L.TOP + (L.HOLDER_TOP - 10.0) * MM if tip_z is None else tip_z
    r = rz(heading)
    if tilt_deg:
        r = r @ L.rx(math.radians(tilt_deg))
    return mk.state_from_poses(T.KEY_SPEC, (x, y, z, *mk.matrix_to_quat(r)), np.zeros(6), source="offline")


def far_key():
    return key_state(L.KEY_XY[0] * MM, L.KEY_XY[1] * MM, L.TOP + L.STAND_FLOOR * MM)


THREAD_OK = {"tight": [True, True], "engaged": [False, False], "weld": True, "clamped_away": False,
             "turns": [1.5, 1.5], "record": None}


def run(tracker, bar, thread=None, arms=None, key=None, n=L.WIN_STEPS + 1):
    st = {S.BAR: bar, S.KEY: key or far_key()}
    res = None
    for _ in range(n):
        res = tracker.update(st, AWAY if arms is None else arms, THREAD_OK if thread is None else thread)
    return res


class LayoutTests(unittest.TestCase):
    def test_bore_and_shank(self):
        self.assertAlmostEqual(2 * L.BORE_R, 12.4)
        self.assertAlmostEqual(2 * L.BAR_R, 12.0)
        self.assertEqual(L.BAR_L, 150.0)
        self.assertEqual(2 * L.HOLE_R, 22.0)
        self.assertEqual(L.HOLE_DEPTH, 40.0)
        self.assertEqual(L.PITCH, 1.0)
        self.assertAlmostEqual(L.TIGHT_PAST, 0.4)

    def test_band_is_the_rule(self):
        self.assertEqual(L.S_BAND[0], L.HOLE_DEPTH + 5.0)  # reach the bottom with 5 mm of clearance
        self.assertEqual(L.S_BAND[1], 4 * 2 * L.BAR_R)  # 4 x the 12 mm diameter

    def test_rack_puts_the_axis_at_the_lathe_height(self):
        top = L.RACK_TOP
        self.assertAlmostEqual(top + math.sqrt(L.BAR_R ** 2 - L.RACK_X[0] ** 2), L.Z_AX)

    def test_shank_union_is_round_with_a_flat(self):
        rows = [r for r in L.bar_rows(0.0) if r[4] == L.STEEL]
        self.assertEqual(len(rows), 9)  # 8 rectangles and the flat's shoulders
        pts = []
        for y in np.linspace(-6.5, 6.5, 131):
            for z in np.linspace(-6.5, 6.5, 131):
                inside = False
                for _, c, size, ypr, _col in rows:
                    a = ypr[2]
                    v = np.array([y - c[1], z - c[2]])
                    u = np.array([math.cos(a), math.sin(a)])
                    w = np.array([-math.sin(a), math.cos(a)])
                    if abs(v @ u) <= size[1] / 2 and abs(v @ w) <= size[2] / 2:
                        inside = True
                        break
                pts.append((y, z, inside))
        for y, z, inside in pts:
            rr = math.hypot(y, z)
            if inside:
                self.assertLessEqual(rr, L.BAR_R + 0.01, (y, z))
                self.assertLessEqual(z, L.FLAT_Z + 1e-6, (y, z))
            elif rr <= 5.3 and z <= L.FLAT_Z - 0.05:
                self.fail(f"hole in the shank at {y:.2f}, {z:.2f}")
        top = max(z for y, z, ins in pts if ins and abs(y) < 1.0)
        self.assertAlmostEqual(top, L.FLAT_Z, delta=0.11)  # the sampling grid is 0.1 mm
        self.assertGreater(max(z for y, z, ins in pts if ins and abs(y - 5.0) < 0.1), -1)

    def test_bar_balances_in_the_holder_at_every_stick_out_of_the_band(self):
        for s in (45.0, 46.5, 48.0, 60.0):
            com_y = L.Y_FACE + s - L.BAR_L / 2
            self.assertTrue(L.Y_BACK < com_y < L.Y_FACE, s)  # the bar's middle is over the holder's bore
        self.assertLess(L.BAR_START_Y + L.BAR_L / 2, L.WP_FACE - 30.0)  # the bar's tip clears the workpiece at spawn
        self.assertGreater(L.BAR_START_Y - L.BAR_L / 2, L.Y_FACE + 10.0)  # and its plain end is in front of the holder
        self.assertTrue(L.RACK_Y[0] < L.BAR_START_Y < L.RACK_Y[1])  # the middle is over the rack

    def test_screws_are_over_the_bar_at_the_demo_stick_out(self):
        for y in L.SCREW_Y:
            self.assertTrue(L.Y_BACK + 5 < y < L.Y_FACE - 5)
            tip_y = L.Y_FACE + 46.5
            self.assertTrue(tip_y - L.BAR_L < y < tip_y)

    def test_key_fits_the_sockets(self):
        self.assertGreaterEqual((L.SOCKET - 2 * L.KEY_SHAFT_R) / 2, 0.9)
        self.assertGreater(L.STAND_HOLE, 2 * L.KEY_SHAFT_R)
        self.assertLess(L.STAND_HOLE - 2 * L.KEY_SHAFT_R, 1.0)
        # the socket's depth (from the top face to the ring): deeper than the engage depth
        self.assertGreater(L.HOLDER_TOP - L.SOCKET_STEPS[0][0], L.ENGAGE_DEPTH + 2)

    def test_arms_can_grip_what_the_demo_grips(self):
        self.assertTrue(8.0 <= 12.0 <= 80.0 and 8.0 <= 2 * L.KEY_BAR_R <= 80.0)  # the gripper's 8 to 80 mm
        self.assertLessEqual(L.BAR_MASS, 0.4)
        self.assertLessEqual(L.KEY_MASS, 0.4)

    def test_shapes_build(self):
        for spec in T.OBJECTS:
            spec._check()
        self.assertEqual(T.BAR_SPEC.mass, 0.130)
        self.assertIsNone(T.HOLDER_SPEC.mass)
        n = sum(len(s.shapes) for s in T.OBJECTS)
        self.assertLess(n, 400)


class ScrewTests(unittest.TestCase):
    def feed(self, m, bar, key_poses):
        out = []
        for (x, y, z, h, tilt) in key_poses:
            k = key_state(x, y, z, h, tilt)
            out.append(m.update(k.origin, k.rotation(), bar.origin, bar.rotation()))
        return out

    def turn_poses(self, screw=0, turns=1.5, steps_per_turn=36, x=None, tip_z=None, h0=0.0, cw=True, tilt=0.0):
        n = int(round(turns * steps_per_turn))
        sgn = -1.0 if cw else 1.0
        return [(x, L.SCREW_Y[screw] * MM, tip_z, h0 + sgn * 2 * math.pi * k / steps_per_turn, tilt)
                for k in range(n + 1)]

    def test_engaged_only_in_the_socket(self):
        k = key_state()
        self.assertTrue(SS.key_engaged(k.origin, k.rotation(), 0))
        self.assertFalse(SS.key_engaged(k.origin, k.rotation(), 1))
        for kw in ({"x": L.X_AX * MM + 0.0016}, {"y": L.SCREW_Y[0] * MM + 0.0016}, {"tilt_deg": 6.5},
                   {"tip_z": L.TOP + (L.HOLDER_TOP - 6.9) * MM}):
            k = key_state(**kw)
            self.assertFalse(SS.key_engaged(k.origin, k.rotation(), 0), kw)
        for kw in ({"x": L.X_AX * MM + 0.0014}, {"tilt_deg": 5.5}, {"tip_z": L.TOP + (L.HOLDER_TOP - 7.1) * MM}):
            k = key_state(**kw)
            self.assertTrue(SS.key_engaged(k.origin, k.rotation(), 0), kw)

    def test_clockwise_turns_tighten_after_one_point_four_turns_on_the_flat(self):
        m = SS.ScrewModel()
        bar = bar_state(dz=0.0)
        outs = self.feed(m, bar, [(None, None, None, 0.0, 0.0)] + self.turn_poses(0, 1.39)[1:])
        self.assertFalse(m.tight[0])
        self.assertFalse(any(o.fire for o in outs))
        outs = self.feed(m, bar, self.turn_poses(0, 0.1, h0=-2 * math.pi * 1.39))
        self.assertTrue(m.tight[0])
        self.assertTrue(any(o.fire for o in outs))
        self.assertTrue(m.weld)
        self.assertFalse(m.tight[1])

    def test_round_contact_is_sooner_than_flat(self):
        _, c_flat = SS.contact_depth_mm(bar_state(dz=0.0).origin, bar_state(dz=0.0).rotation(), 0)
        b = bar_state(flat_deg=90.0, dz=0.0)
        _, c_round = SS.contact_depth_mm(b.origin, b.rotation(), 0)
        self.assertAlmostEqual(c_flat, L.BACKOFF + (L.BAR_R - L.FLAT_Z), places=6)
        self.assertAlmostEqual(c_round, L.BACKOFF, places=6)
        m = SS.ScrewModel()
        self.feed(m, b, self.turn_poses(0, 0.65))
        self.assertTrue(m.tight[0])  # 0.2 + 0.4 = 0.6 turn
        out = [o for o in self.feed(SS.ScrewModel(), b, self.turn_poses(0, 0.65)) if o.fire]
        self.assertTrue(out[0].record["bad_clamp"])
        self.assertAlmostEqual(out[0].record["flat_err_deg"], 90.0, places=1)

    def test_flat_within_ten_degrees_counts_as_up(self):
        for deg, expect_bad in ((9.0, False), (11.0, True)):
            m = SS.ScrewModel()
            outs = self.feed(m, bar_state(flat_deg=deg, dz=0.0), self.turn_poses(0, 1.6))
            rec = [o.record for o in outs if o.fire][0]
            self.assertEqual(rec["bad_clamp"], expect_bad, deg)

    def test_counter_clockwise_unscrews_and_never_goes_below_zero(self):
        m = SS.ScrewModel()
        bar = bar_state(dz=0.0)
        self.feed(m, bar, self.turn_poses(0, 0.5, cw=False))
        self.assertEqual(m.turns[0], 0.0)
        self.feed(m, bar, self.turn_poses(0, 1.6))
        self.assertTrue(m.tight[0])
        outs = self.feed(m, bar, self.turn_poses(0, 0.5, h0=-2 * math.pi * 1.6, cw=False))
        self.assertTrue(any(o.release for o in outs))
        self.assertFalse(m.weld)
        self.assertFalse(m.tight[0])

    def test_no_bar_no_travel_and_no_contact_when_it_comes_later(self):
        m = SS.ScrewModel()
        far = bar_state(tip_y=(L.WP_FACE - 40.0) * MM)  # the bar not under the screws (the plain end is 15 mm out)
        far = bar_state(tip_y=(L.BAR_START_Y + 75.0) * MM, dz=0.0)
        self.feed(m, far, self.turn_poses(0, 3.0))
        self.assertLessEqual(m.depth_mm(0), L.BACKOFF + 1e-9)
        self.assertFalse(m.tight[0])
        self.assertFalse(m.weld)
        bar = bar_state(dz=0.0)
        k = key_state(tip_z=L.TOP + 0.2)
        m.update(k.origin, k.rotation(), bar.origin, bar.rotation())
        self.assertFalse(m.tight[0])  # the bar slid in after: no free lunch

    def test_turning_stops_when_tight(self):
        m = SS.ScrewModel()
        bar = bar_state(dz=0.0)
        self.feed(m, bar, self.turn_poses(0, 6.0))
        _, c = SS.contact_depth_mm(bar.origin, bar.rotation(), 0)
        self.assertAlmostEqual(m.depth_mm(0), c + L.CAP_PAST, places=6)

    def test_a_lifted_key_does_not_turn_the_screw(self):
        m = SS.ScrewModel()
        bar = bar_state(dz=0.0)
        self.feed(m, bar, self.turn_poses(0, 3.0, tip_z=L.TOP + (L.HOLDER_TOP + 2.0) * MM))
        self.assertEqual(m.turns, [0.0, 0.0])

    def test_ratchet_counts_only_engaged_steps(self):
        m = SS.ScrewModel()
        bar = bar_state(dz=0.0)
        up = L.TOP + (L.HOLDER_TOP + 2.0) * MM
        for _ in range(3):
            self.feed(m, bar, self.turn_poses(0, 0.4))  # in, clockwise 0.4 turn
            self.feed(m, bar, self.turn_poses(0, 0.4, tip_z=up, cw=False))  # out, back (not engaged: counts nothing)
        self.assertAlmostEqual(m.turns[0], 1.2, delta=0.05)

    def test_away_clamp_is_latched_in_the_model(self):
        m = SS.ScrewModel()
        bar = bar_state(yaw=-math.pi / 2, dz=0.0, tip_y=(L.Y_FACE - 40.0) * MM + 0.0)
        # the plain end out in front, the cutting end behind the holder: pointing away from the chuck
        r = bar_rot(0.0, yaw=-math.pi / 2)
        o = np.array([L.X_AX * MM, (L.Y_FACE - 28.5) * MM, L.TOP + L.Z_AX * MM])
        away = mk.state_from_poses(T.BAR_SPEC, (*o, *mk.matrix_to_quat(r)), np.zeros(6), source="offline")
        outs = self.feed(m, away, self.turn_poses(0, 1.6))
        self.assertTrue(m.clamped_away)
        self.assertTrue([o_ for o_ in outs if o_.fire][0].record["cutting_end_away"])

    def test_unreadable_pose_changes_nothing_and_releases(self):
        m = SS.ScrewModel()
        bar = bar_state(dz=0.0)
        self.feed(m, bar, self.turn_poses(0, 1.6))
        self.assertTrue(m.weld)
        out = m.update(np.full(3, np.nan), np.eye(3), bar.origin, bar.rotation())
        self.assertTrue(out.release)
        self.assertEqual(m.skipped, 1)


class ScoringTests(unittest.TestCase):
    def setUp(self):
        self.tr = S.Tracker()

    def test_full_score(self):
        r = run(self.tr, bar_state())
        self.assertEqual(r.score, 1.0)
        self.assertTrue(r.success)

    def test_needs_the_full_window_and_stillness(self):
        r = run(self.tr, bar_state(), n=L.WIN_STEPS - 1)
        self.assertEqual(r.score, 0.5)
        r = run(self.tr, bar_state(), n=2)
        self.assertEqual(r.score, 1.0)
        tr = S.Tracker()
        for k in range(L.WIN_STEPS):
            r = run(tr, bar_state(tip_y=(L.Y_FACE + 46.5) * MM + 0.00004 * k), n=1)
        self.assertEqual(r.score, 1.0)  # 0.8 mm of creep over the window
        tr = S.Tracker()
        for k in range(L.WIN_STEPS):
            r = run(tr, bar_state(tip_y=(L.Y_FACE + 46.0) * MM + 0.0003 * k), n=1)
        self.assertEqual(r.score, 0.5)  # 5.7 mm of creep

    def test_band_edges(self):
        for s, want in ((44.9, 0.5), (45.0, 1.0), (48.0, 1.0), (48.1, 0.5), (60.0, 0.5), (30.0, 0.5)):
            self.assertEqual(run(S.Tracker(), bar_state(s)).score, want, s)

    def test_flat_and_insert(self):
        self.assertEqual(run(S.Tracker(), bar_state(flat_deg=9.5)).score, 1.0)
        self.assertEqual(run(S.Tracker(), bar_state(flat_deg=10.5)).score, 0.5)
        self.assertEqual(run(S.Tracker(), bar_state(flat_deg=180.0)).score, 0.5)
        # the insert is 90 deg round from the flat, so a rolled bar fails both; a bar turned 20 deg about the vertical
        # keeps the flat up but its insert is 20 deg off the operator side
        r = run(S.Tracker(), bar_state(yaw=math.pi / 2 + math.radians(20)))
        self.assertEqual(r.score, 0.5)
        self.assertGreater(r.details["insert_err_deg"], 15.0)
        r = run(S.Tracker(), bar_state(yaw=math.pi / 2 + math.radians(10)))
        self.assertEqual(r.score, 1.0)

    def test_cutting_end_away_is_zero_even_when_clamped(self):
        thread = dict(THREAD_OK, clamped_away=True)
        self.assertEqual(run(S.Tracker(), bar_state(), thread=thread).score, 0.0)
        # clamped with the end away, then the screw model forgets: the latch holds in the tracker
        tr = S.Tracker()
        run(tr, bar_state(), thread=thread, n=1)
        self.assertEqual(run(tr, bar_state(), n=30).score, 0.0)
        tr.reset()
        self.assertEqual(run(tr, bar_state()).score, 1.0)

    def test_end_away_without_clamp_is_zero(self):
        r = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]]).T @ np.eye(3)
        rot = rz(-math.pi / 2)
        bar = bar_state(rot=rot, tip_y=(L.Y_FACE - 40.0) * MM)
        self.assertEqual(run(S.Tracker(), bar).score, 0.0)

    def test_one_screw_in_bore_is_a_quarter(self):
        thread = dict(THREAD_OK, tight=[True, False])
        self.assertEqual(run(S.Tracker(), bar_state(), thread=thread).score, 0.25)
        thread = dict(THREAD_OK, tight=[False, False], weld=False)
        self.assertEqual(run(S.Tracker(), bar_state(), thread=thread).score, 0.25)
        self.assertEqual(run(S.Tracker(), bar_state(s=60.0), thread=thread).score, 0.25)

    def test_not_in_the_bore_is_zero(self):
        thread = dict(THREAD_OK, tight=[False, False], weld=False)
        self.assertEqual(run(S.Tracker(), bar_state(dz=0.004), thread=thread).score, 0.0)  # 4 mm high
        self.assertEqual(run(S.Tracker(), bar_state(dx=0.003), thread=thread).score, 0.0)
        self.assertEqual(run(S.Tracker(), bar_state(s=1.0), thread=thread).score, 0.25 if False else 0.0)
        self.assertEqual(run(S.Tracker(), bar_state(tip_y=(L.BAR_START_Y + 75.0) * MM), thread=thread).score, 0.0)
        # tilted 5 deg: at the faces its axis is far off the bore's
        rot = bar_rot(0.0, pitch_deg=5.0)
        self.assertEqual(run(S.Tracker(), bar_state(rot=rot), thread=thread).score, 0.0)

    def test_key_still_in_a_screw_is_half(self):
        thread = dict(THREAD_OK, engaged=[False, True])
        self.assertEqual(run(S.Tracker(), bar_state(), thread=thread).score, 0.5)

    def test_grippers_must_be_clear_of_the_bar(self):
        near = {a: rigstate.Arm(np.array([0.25, 0.09, L.TOP + 0.07]), np.array([0.25, 0.09, L.TOP + 0.057]), 0.5)
                for a in ("left", "right")}
        self.assertEqual(run(S.Tracker(), bar_state(), arms=near).score, 0.5)
        r = run(S.Tracker(), bar_state(), arms={"left": AWAY["left"], "right": near["right"]})
        self.assertEqual(r.score, 0.5)
        self.assertEqual(run(S.Tracker(), bar_state(), arms=None).score, 1.0)

    def test_unreadable_arms_or_states_fail_closed(self):
        tr = S.Tracker()
        run(tr, bar_state(), n=10)
        r = tr.update({S.BAR: bar_state(), S.KEY: far_key()}, None, THREAD_OK)
        self.assertEqual(r.score, 0.0)
        self.assertEqual(run(tr, bar_state(), n=L.WIN_STEPS - 1).score, 0.5)  # the window restarted
        self.assertEqual(tr.update({}, AWAY, THREAD_OK).score, 0.0)
        nan = bar_state()
        bad = {S.BAR: mk.state_from_poses(T.BAR_SPEC, (np.nan, 0.1, 0.8, 0, 0, 0, 1), np.zeros(6), source="offline"),
               S.KEY: far_key()}
        self.assertEqual(tr.update(bad, AWAY, THREAD_OK).score, 0.0)

    def test_off_table_latches(self):
        tr = S.Tracker()
        fallen = mk.state_from_poses(T.KEY_SPEC, (0.2, 0.0, L.TOP - 0.2, 0, 0, 0, 1), np.zeros(6), source="offline")
        r = run(tr, bar_state(), key=fallen, n=1)
        self.assertEqual(r.score, 0.0)
        self.assertEqual(run(tr, bar_state(), n=30).score, 0.0)
        bar_gone = mk.state_from_poses(T.BAR_SPEC, (0.2, 0.0, L.TOP - 0.2, 0, 0, 0, 1), np.zeros(6), source="offline")
        self.assertEqual(run(S.Tracker(), bar_gone, n=1).score, 0.0)

    def test_nothing_done_scores_zero(self):
        start = mk.state_from_poses(T.BAR_SPEC, (*T.BAR_SPEC.spawn_pose()[0], *T.BAR_SPEC.spawn_pose()[1]),
                                    np.zeros(6), source="offline")
        thread = {"tight": [False, False], "engaged": [False, False], "weld": False, "clamped_away": False,
                  "turns": [0, 0], "record": None}
        self.assertEqual(run(S.Tracker(), start, thread=thread).score, 0.0)


class OfflineTaskTests(unittest.TestCase):
    def test_no_simulator_detection(self):
        self.assertTrue(T.no_simulator({S.BAR: bar_state()}))

    def test_task_hooks_on_the_start_layout(self):
        T._TRACKER.reset()
        T.OFFLINE.reset()
        self.assertFalse(T.success({S.BAR: T.BAR_SPEC.rest_state(), S.KEY: T.KEY_SPEC.rest_state()}))
        self.assertEqual(T.score({}), 0.0)

    def test_variants_and_plans(self):
        for v in T.VARIANTS + ("",):
            p = T.plans(v)
            self.assertEqual(set(p), {"left", "right"})
            for side in p:
                for w in p[side]:
                    self.assertGreater(w.duration, 0.0)
                    self.assertTrue(np.all(np.isfinite(w.pos)))
        self.assertLess(len(T.VARIANTS), 5)
        full = sum(w.duration for w in T.plans("")["right"])
        self.assertLessEqual(full * 20 + 61, 2400)  # the policy holds 3 s after the plan: inside the run limit

    def test_demo_aims_inside_the_band(self):
        self.assertTrue(L.S_BAND[0] + 1.0 <= T.S_DEMO <= L.S_BAND[1] - 1.0)
        self.assertGreater(T.S_LONG, 4 * 2 * L.BAR_R)
        left = T.plans("")["left"]
        push = [w for w in left if w.label.startswith("push the bar")][0]
        self.assertAlmostEqual(push.pos[1] / MM - T.PAD_FACE - L.Y_FACE, T.S_DEMO + T.TRIM_PUSH_Y, places=6)
        pre = [w for w in left if w.label.startswith("down behind")][0]
        self.assertLessEqual(pre.pos[1] / MM + T.PAD_FACE, L.WP_FACE - 5.0)
        self.assertGreaterEqual(T.PUSH_TIP_Z - L.RACK_TOP, 4.0)
        self.assertGreaterEqual(T.GRIP_Z - T.TIP - L.RACK_TOP, 4.0)

    def test_the_insert_is_in_the_cutting_end_not_the_middle(self):
        rows = L.bar_rows()
        ins = [r for r in rows if r[-1] == L.GOLD][0]
        self.assertAlmostEqual(ins[1][0] + ins[2][0] / 2, L.BAR_L / 2, places=6)  # flush with the cutting end's face
        # and nothing wider than the bore (6.2 mm radius) lies within 100 mm of the plain end
        for r in rows:
            if r[-1] == L.GOLD:
                self.assertGreater(r[1][0] - r[2][0] / 2, 0.0)

    def test_the_key_stand_has_its_floor_under_the_key(self):
        # the floor box is in the stand's own frame: the stand's bounds are 30 mm square about the key's x, y
        low, high = np.min([np.asarray(s.center) - s.half_extents() for s in T._STAND_SHAPES], axis=0), \
            np.max([np.asarray(s.center) + s.half_extents() for s in T._STAND_SHAPES], axis=0)
        self.assertLessEqual(float(np.max(high[:2] - low[:2])), 0.0301)
        self.assertAlmostEqual(float(low[2]), 0.0, places=6)

    def test_the_closed_pad_block_pushes_the_bar_offline(self):
        import rigstate as RS

        T.OFFLINE.reset()
        o0 = T.OFFLINE.bar_o.copy()
        tail = o0[1] + 0.075
        tips = np.array([o0[0], tail + 0.004 - 0.020, o0[2] + 0.001])  # 20 mm into the end face
        arm = RS.Arm(tcp=tips + np.array([0, 0, 0.013]), tips=tips, opening=0.0)
        far = RS.Arm(tcp=np.array([0.3, -0.3, 0.9]), tips=np.array([0.3, -0.3, 0.887]), opening=1.0)
        T.OFFLINE.step({"left": arm, "right": far}, 1.0)
        self.assertAlmostEqual(T.OFFLINE.bar_o[1], o0[1] - 0.020 + L.GPU_PUSH_LAG * L.MM, places=6)
        # pulling back does not pull the bar
        tips2 = tips + np.array([0, 0.03, 0])
        arm2 = RS.Arm(tcp=tips2 + np.array([0, 0, 0.013]), tips=tips2, opening=0.0)
        T.OFFLINE.step({"left": arm2, "right": far}, 1.1)
        self.assertAlmostEqual(T.OFFLINE.bar_o[1], o0[1] - 0.020 + L.GPU_PUSH_LAG * L.MM, places=6)
        T.OFFLINE.reset()

    def test_the_wrist_turns_clockwise_to_tighten(self):
        w = [x for x in T.plans("")["right"] if "turn the key clockwise" in x.label][0]
        r0, r1 = D.grip_rotation(T.YAW_A), D.grip_rotation(w.yaw)
        h0, h1 = math.atan2(r0[1, 0], r0[0, 0]), math.atan2(r1[1, 0], r1[0, 0])
        self.assertLess(SS.wrap_pi(h1 - h0), -math.radians(140))

    def test_the_strokes_turn_the_screw_far_enough(self):
        need = (L.BACKOFF + (L.BAR_R - L.FLAT_Z) + 0.2 + L.TIGHT_PAST * L.PITCH) / L.PITCH  # the bar 0.2 mm low in the bore
        self.assertGreater(T.STROKES * T.STROKE_DEG / 360.0, need * 1.2)

    def test_the_demo_scores_one_offline(self):
        if OFFLINE is None:
            self.skipTest("tools/offline not found")
        out = subprocess.run([sys.executable, str(OFFLINE / "check_task.py"), str(HERE), "--json", "-", "--quiet"],
                             capture_output=True, text=True, timeout=900)
        self.assertEqual(out.returncode, 0, out.stdout[-2000:] + out.stderr[-2000:])
        text = out.stdout
        start = text.index("{")
        rep = json.loads(text[start:text.rindex("}") + 1])
        self.assertEqual(rep["playback"]["proxy"]["predicted_task_score"], 1.0)


class FolderTests(unittest.TestCase):
    def test_kit_files_are_byte_identical_copies(self):
        for p in HERE.parents:
            kit = p / "tools" / "kit"
            if kit.is_dir():
                for name in ("mechkit.py", "rigstate.py"):
                    self.assertEqual((HERE / name).read_bytes(), (kit / name).read_bytes(), name)
                return
        self.skipTest("tools/kit not found")

    def test_text_rules(self):
        banned = [chr(0x2014), chr(0x2013), r"claude", r"anthropic", r"co-authored", r"T-?70", r"vihan", r"aggarwal",
                  r"/home/", r"token", r"ROBOCURVE_"]
        for name in ("task.json", "README.md", "task.py", "layout.py", "scoring.py", "setscrew.py", "offline_boring.py"):
            text = (HERE / name).read_text()
            for pat in banned:
                if name == "task.json" and pat == "token":
                    continue
                self.assertIsNone(re.search(pat, text, flags=re.I), f"{name}: {pat}")

    def test_task_json_matches_the_layout(self):
        spec = json.loads((HERE / "task.json").read_text())
        for key in ("title", "instruction", "setup", "scoring", "fidelity", "reset_notes", "context", "run_limit"):
            self.assertTrue(spec[key].strip(), key)
        self.assertEqual(spec["instruction"], T.INSTRUCTION)
        setup, scoring = spec["setup"], spec["scoring"]
        for needle in ("12.4 mm", "40 mm", "12 mm", "150 mm", "22 mm", "1.0 mm"):
            self.assertIn(needle, setup, needle)
        for needle in ("45", "48", "0.4 turn", "10 degrees", "15 degrees"):
            self.assertIn(needle, scoring, needle)
        # the instruction must not give the band or the orientation away
        for word in ("45", "48", "flat", "insert", "chuck", "mm"):
            self.assertNotIn(word, spec["instruction"].lower())
        self.assertIn("software", spec["fidelity"].lower())
        self.assertIn("N m", spec["fidelity"])


if __name__ == "__main__":
    unittest.main()
