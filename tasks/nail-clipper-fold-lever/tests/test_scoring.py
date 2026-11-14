"""Offline tests for layout.py, scoring.py, offline_clipper.py and task.py: synthetic clipper states (mechkit
AssemblyStates from the task's own spec) and synthetic arm snapshots (rigstate.Arm), no simulator.
    python3 -m unittest discover -s tests -v"""

import hashlib
import json
import math
import pathlib
import re
import subprocess
import sys
import tempfile
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
            sys.path.insert(0, str(_p))  # mechkit and task.py import yam_arena: the offline stand-in

import layout as L  # noqa: E402
import mechkit as mk  # noqa: E402
import offline_clipper as OC  # noqa: E402
import rigstate  # noqa: E402
import scoring as S  # noqa: E402
import task as T  # noqa: E402
from yam_arena import demo as D  # noqa: E402

try:
    import yamkin as K  # noqa: E402
except ImportError:  # pragma: no cover
    K = None

SC = T.CLIPPER_SPEC
WIN = int(round(L.STILL_WINDOW_S * L.CONTROL_HZ))  # 20 steps
FLOOR = np.array(L.ORIGIN, dtype=float)
AWAY = {a: rigstate.Arm(np.array([0.32, s * 0.305, L.TOP + 0.26]), np.array([0.33, s * 0.305, L.TOP + 0.25]), 1.0)
        for a, s in (("left", 1.0), ("right", -1.0))}
FIELDS = ("title", "instruction", "context", "setup", "scoring", "fidelity", "reset_notes")
META = json.loads((HERE / "task.json").read_text())
README = (HERE / "README.md").read_text()


# -- states --------------------------------------------------------------------------------------------------------


def states(psi=0.0, lever=0.0, shift=(0.0, 0.0, 0.0), rot=None):
    """The clipper's state: swivel ``psi``, lever hinge ``lever`` (rad; 0 is the cutting position), the frame at the
    start pose moved by ``shift`` (m) and turned by the matrix ``rot``."""
    o = FLOOR + np.asarray(shift, dtype=float)
    quat = mk.yaw_quat(0.0) if rot is None else mk.matrix_to_quat(rot)
    sc = mk.state_from_poses(SC, (*o, *quat), joints={S.PIN: psi, S.LEVER: lever}, source="offline")
    return {S.CLIPPER: sc}


def stowed(sign=1.0, **kw):
    """Folded: the lever to the tail, turned over, lying on the blade; the swivel half a turn either way round."""
    return states(psi=sign * math.pi, lever=L.EPS_STOW - L.E0, **kw)


def feed(tracker, st, arms, n):
    result = None
    for _ in range(n):
        result = tracker.update(st, arms)
    return result


def arm_near(point, opening=0.0):
    p = np.asarray(point, dtype=float)
    return rigstate.Arm(p + np.array([0.0, 0.0, 0.013]), p, opening)


def settled(st, arms=AWAY, n=WIN + 1):
    return feed(S.Tracker(), st, arms, n)


def surface(state, bodies, n=24):
    """Points (world) on the surface of every shape of ``bodies`` of an assembly state."""
    spec, out = state.spec, []
    for body in bodies:
        r, p = state.link_pose(body)
        off = spec.body_offset(body)
        for s in spec.body_shapes(body):
            u = np.linspace(-0.5, 0.5, n)
            if s.kind == "box":
                size = np.asarray(s.size, dtype=float)
                pts = []
                for k in range(3):
                    a, b = [i for i in range(3) if i != k]
                    uu, vv = np.meshgrid(u, u)
                    for sign in (-0.5, 0.5):
                        q = np.zeros((uu.size, 3))
                        q[:, a], q[:, b], q[:, k] = uu.ravel() * size[a], vv.ravel() * size[b], sign * size[k]
                        pts.append(q)
                local = np.vstack(pts)
            else:
                k = {"X": 0, "Y": 1, "Z": 2}[s.axis]
                ang = np.linspace(0, 2 * math.pi, 3 * n, endpoint=False)
                a, b = [i for i in range(3) if i != k]
                local = []
                for h in np.linspace(-0.5, 0.5, n):
                    q = np.zeros((ang.size, 3))
                    q[:, a], q[:, b], q[:, k] = s.radius * np.cos(ang), s.radius * np.sin(ang), h * s.height
                    local.append(q)
                for sign in (-0.5, 0.5):
                    for rr in np.linspace(0, s.radius, 6):
                        q = np.zeros((ang.size, 3))
                        q[:, a], q[:, b], q[:, k] = rr * np.cos(ang), rr * np.sin(ang), sign * s.height
                        local.append(q)
                local = np.vstack(local)
            base = (local @ s.rotation().T) + np.asarray(s.center, dtype=float) - off
            out.append(base @ r.T + p)
    return np.vstack(out)


class Geometry(unittest.TestCase):
    def test_kit_files_are_unchanged_copies(self):
        for src, names in (("tools/kit", ("mechkit.py", "rigstate.py")),
                           ("tasks/cord-lock-squeeze-and-thread", ("tiltgrip.py",))):
            d = HERE.parents[1] / src
            if not (d / names[0]).is_file():
                self.skipTest("the repository's kit and task folders are not next to this task")
            for name in names:
                a = hashlib.sha256((HERE / name).read_bytes()).hexdigest()
                b = hashlib.sha256((d / name).read_bytes()).hexdigest()
                self.assertEqual(a, b, name)

    def test_solved_numbers(self):
        deg = math.degrees
        self.assertAlmostEqual(deg(L.TH0), 18.71, places=2)
        self.assertEqual(L.TH0, L.TH180)  # the heel stands on the level seat at every swing angle
        self.assertAlmostEqual(L.LEVEL_TOP, 9.99, places=2)
        self.assertAlmostEqual(L.ZH + L.LEV * math.sin(L.TH0), 30.8, places=1)  # tip height at rest, either way
        self.assertAlmostEqual(L.LEV * math.cos(L.TH0), 54.0, places=1)  # and its reach from the pin in plan
        self.assertAlmostEqual(deg(L.BETA_STOW), 6.13, places=2)  # stowed, under level (the blade is 5.5)
        self.assertAlmostEqual(deg(L.BETA_FWD), 5.73, places=2)  # turned over without a swing, on the jaw edge
        self.assertAlmostEqual(L.stow_kink_gap(), 0.55, places=2)  # the stowed lever rests on its tip, not the kink
        under_hinge = L.ZH - L.KNUCKLE_R - L.blade_top(L.PIN_X)  # the knuckle over the seat
        self.assertAlmostEqual(under_hinge, 1.41, places=2)
        self.assertAlmostEqual(L.JAW_GAP, 2.39, places=2)
        self.assertAlmostEqual(L.LEVER_RATIO, 9.5, places=2)
        self.assertAlmostEqual(deg(L.EPS_STOW - L.TH0), 167.4, delta=0.1)  # swung half a turn, flipped 167.4 deg
        self.assertAlmostEqual(L.LEVER_UPPER, 2.922, delta=0.001)  # the lever's stowed reading: its upper stop
        self.assertEqual(L.LEVER_LOWER, 0.0)  # the heel on its seat: the pose as drawn
        self._rest_elevation_matches_the_drawn_pose()

    def test_the_stops_are_where_the_lever_meets_the_blade(self):
        """The two stops of the lever's hinge stand in for contact with the blade (self-collision is off inside one
        articulation): each must be where the real lever meets the real blade, at both ends of the swing."""
        for psi in (0.0, math.pi, -math.pi):
            self.assertAlmostEqual(L.heel_gap(L.E0 + L.LEVER_LOWER, math.cos(psi)), 0.0, places=6)
        self.assertAlmostEqual(L._stow_gap(L.EPS_STOW - math.pi), 0.0, places=6)  # stowed: tip on the slope
        fwd_mm = L.PIN_X * (math.tan(L.BETA_STOW) - math.tan(L.BETA_FWD))  # turned over at psi 0: into the jaw edge
        self.assertLess(fwd_mm, 0.15)
        self.assertGreater(fwd_mm, 0.0)
        # the stowed lever's underside clears the seat from the knuckle to the kink, then rests on its tip
        b = L.BETA_STOW
        for x in np.linspace(L.PIN_X + L.LEV_X0, L.PIN_X + L.LEV * math.cos(b) - 0.5, 60):
            under = L.ZH - (x - L.PIN_X) * math.tan(b) - (L.LEV_T / 2) / math.cos(b)
            self.assertGreaterEqual(under - L.blade_top(x), -1e-6, x)

    def _masses(self):
        total = L.M_BASE + L.M_PIN + L.M_LEVER
        self.assertAlmostEqual(total, 0.054, places=6)
        for body, m in {"base": L.M_BASE, S.PIN: L.M_PIN, S.LEVER: L.M_LEVER}.items():
            self.assertAlmostEqual(SC.body_mass(body), m, places=9, msg=body)
        steel = L.masses_g()  # the plates alone, in steel: the declared masses (with lips, pin, block) are close
        self.assertAlmostEqual(steel["lower blade"] + steel["upper blade"] + 6.0, 1000 * L.M_BASE, delta=4.0)
        self.assertAlmostEqual(steel["lever"], 1000 * L.M_LEVER, delta=2.0)

    def test_assemblies_and_joints(self):
        self.assertEqual((T.TRAY.kind, SC.kind), ("static", "articulation"))
        self.assertEqual([o.name for o in T.OBJECTS], ["tray", "clipper"])  # ONE free articulation, nothing else
        self.assertEqual(mk.rules()["bonds"], {})  # no joint between two free bodies
        self.assertEqual(mk.rules()["welds"], {})
        self.assertEqual([j.name for j in SC.joints], ["pin", "lever"])
        self.assertEqual(SC.link("lever").parent, "pin")
        for j in SC.joints:
            self.assertLessEqual(j.lower, 0.0, j.name)  # the limits contain 0: 0 is the pose as drawn
            self.assertGreaterEqual(j.upper, 0.0, j.name)
            self.assertEqual(j.spring, 0.0, j.name)  # nothing is driven
        self.assertEqual((SC.link("pin").axis, SC.link("lever").axis), ("Z", "-Y"))
        self.assertEqual((SC.link("pin").lower, SC.link("pin").upper), (-3.67, 3.67))
        self.assertEqual(SC.link("lever").lower, 0.0)
        self.assertAlmostEqual(SC.link("lever").upper, L.EPS_STOW - L.E0, places=12)
        self.assertEqual(SC.friction, (0.25, 0.2))
        SC._check()
        self._masses()

    def test_start_pose_is_clear_and_fits_the_slot(self):
        st = states(lever=math.radians(0.2))  # 0.2 deg off its stop: the heel 0.02 mm over the seat
        lever_pts = surface(st[S.CLIPPER], (S.LEVER,))
        base = mk.state_from_poses(SC, (*FLOOR, 0, 0, 0, 1), joints={S.PIN: 0.0, S.LEVER: 0.0})
        d = np.array([min(_base_distance(base, p)) for p in [lever_pts]]).ravel()
        self.assertGreater(float(d.min()), 0.0)  # the lever overlaps no part of the base
        self.assertLess(float(d.min()), 0.0005)  # and the heel is down on the seat
        self.assertAlmostEqual(L.heel_gap(L.TH0, 1.0), 0.0, places=6)
        # the upper blade clears the lower one: 0.7 at the tail end, and the jaws stand open 2.39
        z_pivot_end = L.PIV[1] + L.BLADE_S[0] * math.sin(L.ALPHA) - (L.BLADE_T / 2) / math.cos(L.ALPHA)
        self.assertAlmostEqual(z_pivot_end - L.LOW_T, 0.7, delta=0.1)
        lips = [sh for sh in L.blade_shapes() if sh[1][0] < 2.0]
        self.assertEqual(len(lips), 1)
        upper_lip_bottom = lips[0][1][2] - lips[0][2][2] / 2
        self.assertAlmostEqual(upper_lip_bottom - (L.LOW_T + L.LIP_H), L.JAW_GAP, places=9)
        self.assertAlmostEqual(L.JAW_GAP, 2.4, delta=0.05)
        # the sloping part meets the level part at the kink (within 0.1 mm)
        self.assertAlmostEqual(L.PIV[1] + L.BLADE_S[1] * math.sin(L.ALPHA), L.LEVEL_C, places=6)
        # the clipper lies in the slot with 1.5 mm of play each way
        self.assertAlmostEqual(L.SLOT_X[1] - L.LEN, 1.5, places=9)
        self.assertAlmostEqual(-L.SLOT_X[0], 1.5, places=9)
        self.assertGreater(L.SLOT_Y[1] - L.WIDTH / 2, 1.4)
        tray = T.TRAY.rest_state()
        self.assertGreater(float(tray.distance(surface(st[S.CLIPPER], ("base",))).min()), 0.0)
        self.assertLess(float(tray.distance(surface(st[S.CLIPPER], ("base",))).min()), 0.0001)  # lies on the floor

    def test_heel_stays_on_the_level_seat_and_off_the_hole(self):
        hole_corner = math.hypot(L.HOLE_Y, (L.HOLE_X[1] - L.HOLE_X[0]) / 2)  # 4.95 from the pin
        self.assertAlmostEqual(hole_corner, 4.95, places=2)
        for k in range(72):
            psi = math.radians(5.0 * k)
            th = L.rest_elevation(psi)
            self.assertAlmostEqual(th, L.TH0, places=9, msg=k)
            radius = L.HEEL_X * math.cos(th) + L.HEEL_C * math.sin(th)  # the heel's line from the pin, in plan
            self.assertGreater(radius, hole_corner + 1.0, k)
            hx, _ = L.heel_xz(th, math.cos(psi))
            self.assertLess(hx, L.KINK_X - 1.0, k)  # on the level seat
            self.assertGreater(hx, 0.0, k)

    def _rest_elevation_matches_the_drawn_pose(self):
        self.assertAlmostEqual(L.rest_elevation(0.0), L.TH0)
        self.assertAlmostEqual(L.rest_elevation(math.pi), L.TH0)
        self.assertEqual(L.E0, L.TH0)
        lever = SC.link("lever")
        self.assertEqual(tuple(round(v, 9) for v in lever.anchor), (L.PIN_X * L.MM, 0.0, L.ZH * L.MM))


def _base_distance(base_state, pts):
    """The distance (m) from world points to the base body's shapes alone (the pin and lever left out)."""
    out = []
    for sh in SC.body_shapes("base"):
        rot = sh.rotation()
        local = (pts - np.asarray(base_state.origin)) - np.asarray(sh.center, dtype=float)
        local = local @ rot
        if sh.kind == "box":
            q = np.abs(local) - np.asarray(sh.size, dtype=float) / 2
        else:
            k = {"X": 0, "Y": 1, "Z": 2}[sh.axis]
            a, b = [i for i in range(3) if i != k]
            rr = np.hypot(local[:, a], local[:, b]) - sh.radius
            q = np.stack([rr, np.abs(local[:, k]) - sh.height / 2, np.full(len(local), -1.0)], axis=1)
        out.append(np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0))
    return [np.min(np.vstack(out), axis=0)]


class Scoring(unittest.TestCase):
    def test_start_scores_zero(self):
        r = settled(states())
        self.assertEqual(r.score, 0.0)
        self.assertFalse(r.success)
        self.assertFalse(r.details["heel_away"])
        self.assertTrue(r.details["to_tail"])  # to the tail, heel toward the blade: 0
        self.assertAlmostEqual(r.details["plan_mm"], L.LEV * math.cos(L.E0), places=1)
        self._pressed_flat_scores_zero()

    def _pressed_flat_scores_zero(self):
        r = settled(states(lever=L.LEVER_LOWER))  # pressed down, the lever bears on its heel stop
        self.assertEqual(r.score, 0.0)
        self.assertFalse(r.details["heel_away"])

    def test_swing_done_scores_a_quarter_either_way_round(self):
        for sign in (1.0, -1.0):
            r = settled(states(psi=sign * math.pi, lever=L.TH180 - L.E0))
            self.assertEqual(r.score, 0.25, sign)
            self.assertTrue(r.details["past_jaws"])
        r = settled(states(psi=math.pi - math.radians(29.0), lever=L.TH180 - L.E0))
        self.assertEqual(r.score, 0.25)
        r = settled(states(psi=math.pi - math.radians(31.0), lever=L.TH180 - L.E0))
        self.assertEqual(r.score, 0.0)

    def test_flipped_forward_without_a_swing_scores_a_quarter(self):
        r = settled(states(psi=0.0, lever=L.EPS_FWD - L.E0))
        self.assertEqual(r.score, 0.25)
        self.assertTrue(r.details["heel_away"])
        self.assertFalse(r.details["to_tail"])
        self._over_the_top_but_short_scores_a_half()

    def _over_the_top_but_short_scores_a_half(self):
        r = settled(states(psi=math.pi, lever=L.EPS_STOW - L.E0 - math.radians(10.0)))
        self.assertEqual(r.score, 0.5)
        self.assertFalse(r.details["flat"])

    def test_stowed_clear_and_still_scores_one_both_directions(self):
        for sign in (1.0, -1.0):
            r = settled(stowed(sign))
            self.assertEqual(r.score, 1.0, sign)
            self.assertTrue(r.success)
            self.assertTrue(r.details["flat"] and r.details["in_slot"] and r.details["clear_now"])
            self.assertLess(abs(r.details["tip_gap_mm"]), 0.1)
        tr = S.Tracker()
        self.assertEqual(feed(tr, stowed(), AWAY, WIN).score, 0.5)  # 20 steps are not yet 20 steps of stillness
        self.assertEqual(tr.update(stowed(), AWAY).score, 1.0)

    def test_stowed_with_a_gripper_near_scores_a_half(self):
        st = stowed()
        tip = st[S.CLIPPER].link_to_world(S.LEVER, S.TIP_FACE)
        near = dict(AWAY, right=arm_near(tip + np.array([0.0, 0.0, 0.020])))
        self.assertEqual(settled(st, near).score, 0.5)
        far = dict(AWAY, right=arm_near(tip + np.array([0.0, 0.0, 0.080])))
        self.assertEqual(settled(st, far).score, 1.0)
        mid = dict(AWAY, right=arm_near(tip + np.array([0.0, 0.0, 0.040])))  # a closed finger zone is 16 mm round
        self.assertEqual(settled(st, mid).score, 0.5)
        mid = dict(AWAY, right=arm_near(tip + np.array([0.0, 0.0, 0.050])))
        self.assertEqual(settled(st, mid).score, 1.0)

    def test_stowed_but_moving_scores_a_half(self):
        tr = S.Tracker()
        r = None
        for k in range(WIN + 5):
            r = tr.update(stowed(shift=(0.0001 * k, 0.0, 0.0)), AWAY)  # slides 0.1 mm a step along the slot
        self.assertEqual(r.score, 0.5)
        self.assertGreater(r.details["motion_mm"], 1.0)
        tr = S.Tracker()
        for k in range(WIN + 5):
            r = tr.update(stowed(shift=(0.00004 * k, 0.0, 0.0)), AWAY)  # 0.8 mm over the window
        self.assertEqual(r.score, 1.0)

    def _eleven_degrees_off_in_plan_is_not_a_fold(self):
        r = settled(states(psi=math.pi + math.radians(11.0), lever=L.EPS_STOW - L.E0))
        self.assertLess(r.score, 1.0)
        self.assertEqual(r.score, 0.0)
        r = settled(states(psi=math.pi + math.radians(9.0), lever=L.EPS_STOW - L.E0))
        self.assertEqual(r.score, 1.0)  # 9 deg off is still within the 10

    def test_tip_gap_over_two_millimetres_is_not_flat(self):
        def gap_of(deg_up):
            st = states(psi=math.pi, lever=L.EPS_STOW - L.E0 - math.radians(deg_up))
            return settled(st, n=2).details["tip_gap_mm"]

        lo, hi = 0.0, 10.0
        for _ in range(40):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if gap_of(mid) < 2.5 else (lo, mid)
        up = (lo + hi) / 2
        self.assertAlmostEqual(gap_of(up), 2.5, delta=0.02)
        r = settled(states(psi=math.pi, lever=L.EPS_STOW - L.E0 - math.radians(up)))
        self.assertEqual(r.score, 0.5)
        self.assertFalse(r.details["flat"])
        r = settled(states(psi=math.pi, lever=L.EPS_STOW - L.E0 - math.radians(up * 1.5 / 2.5)))
        self.assertEqual(r.score, 1.0)  # 1.5 mm is flat enough
        self._eleven_degrees_off_in_plan_is_not_a_fold()

    def test_clipper_turned_over_proves_nothing(self):
        flip = np.diag([1.0, -1.0, -1.0])  # a half turn about x
        r = settled(states(rot=flip, shift=(0.0, 0.0, 0.020)))
        self.assertEqual(r.score, 0.0)  # the lever unfolded, in the clipper's own frame: heel toward the blade
        r = settled(stowed(rot=flip, shift=(0.0, 0.0, 0.020)))
        self.assertLess(r.score, 1.0)
        self.assertFalse(r.details["in_slot"])

    def test_clipper_out_of_the_slot_is_not_one(self):
        for dz in (0.004, -0.003, 0.002):
            r = settled(stowed(shift=(0.0, 0.0, dz)))
            self.assertEqual(r.score, 0.5, dz)
            self.assertFalse(r.details["in_slot"], dz)
        r = settled(stowed(shift=(0.0, 0.0, 0.0014)))
        self.assertEqual(r.score, 1.0)  # within 1.5 mm
        r = settled(stowed(rot=mk.axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(1.5))))
        self.assertEqual(r.score, 0.5)  # a corner 1.5 mm up and the other as far down: tilted in the slot is not flat
        self.assertFalse(r.details["in_slot"])

    def test_an_unreadable_step_starts_the_window_again(self):
        tr = S.Tracker()
        self.assertEqual(feed(tr, stowed(), AWAY, WIN + 5).score, 1.0)
        self.assertEqual(tr.update({}, AWAY).score, 0.0)  # no state at all
        self.assertEqual(tr.bad_steps, 1)
        self.assertEqual(feed(tr, stowed(), AWAY, WIN).score, 0.5)
        self.assertEqual(tr.update(stowed(), AWAY).score, 1.0)
        bad = stowed()
        self.assertEqual(tr.update({S.CLIPPER: None}, AWAY).score, 0.0)
        self.assertEqual(tr.update({"tray": bad[S.CLIPPER]}, AWAY).score, 0.0)
        nan = states(psi=math.nan)
        self.assertEqual(tr.update(nan, AWAY).score, 0.0)
        self.assertEqual(tr.update(stowed(), None).score, 0.5)  # arms unreadable: never clear
        self.assertEqual(feed(tr, stowed(), AWAY, WIN - 1).score, 0.5)  # 19 clear steps are not 20
        self.assertEqual(tr.update(stowed(), AWAY).score, 1.0)

    def test_reset_detection(self):
        tr = S.Tracker()
        feed(tr, stowed(), AWAY, WIN + 5)
        self.assertEqual(tr.resets, 0)
        r = tr.update(states(), AWAY)  # the simulator put it back on its start pose in one step
        self.assertEqual((tr.resets, tr.steps, r.score), (1, 1, 0.0))
        # a natural return to the start (it moves back in small steps) is not a reset
        tr = S.Tracker()
        feed(tr, states(lever=0.10), AWAY, 3)
        tr.update(states(), AWAY)
        self.assertEqual(tr.resets, 0)
        # the same pose with the swivel half a turn is not the start
        self.assertFalse(S.Tracker.at_start(S.read(states(psi=math.pi)[S.CLIPPER])))


class Demo(unittest.TestCase):
    PLAN = {v: T.plans(v)["right"] for v in ("", "swing_only", "press_flat")}

    @classmethod
    def setUpClass(cls):
        if K is None:
            raise unittest.SkipTest("tools/offline is not next to this task")

    def _run(self, variant):
        """Play a plan through the offline arm and clipper: (the plan, one row per control step)."""
        plan = D.ArmPlan(self.PLAN[variant], K.BASES["station"]["right"])
        off = OC.OfflineClipper()
        left = AWAY["left"]
        rows = []
        for k in range(int((plan.duration + 1.0) * L.CONTROL_HZ)):
            t = (k + 1) / L.CONTROL_HZ
            pos, rot, finger, label = plan.target(t)
            arm = rigstate.Arm(pos, pos + K.PAD_TIP_PAST_TCP * rot[:, 2], float(np.clip(finger / K.FINGER_OPEN, 0, 1)))
            off.step({"left": left, "right": arm})
            rows.append((t, label, pos, rot, arm, off.psi, off.eps))
        return plan, rows

    def test_every_waypoint_solves_for_the_right_arm(self):
        base = K.BASES["station"]["right"]
        q = np.array(K.READY, dtype=float)
        for variant, wps in self.PLAN.items():
            plan = D.ArmPlan(wps, base)
            worst = 0.0
            for k in range(int(plan.duration * L.CONTROL_HZ) + 1):
                act, _ = plan.action7(k / L.CONTROL_HZ)
                self.assertGreaterEqual(float(K.joint_margins(act[:6]).min()), 0.0, (variant, k))
                worst = max(worst, plan.last_ik_mm, plan.last_ik_deg)
            self.assertLess(plan.worst_ik_mm, 0.5, variant)
            self.assertLess(plan.worst_ik_deg, 0.5, variant)
            for w in wps:  # each waypoint's own pose, from the plan's last joints
                rot = D.grip_rotation(w.yaw)
                q, pe, re = K.solve(w.pos, K.matrix_to_quat_xyzw(rot), plan.q, base, iters=200)
                self.assertLess(pe, 1e-3, (variant, w.label))
                self.assertLess(math.degrees(re), 0.5, (variant, w.label))
        # the pitched waypoints carry a full rotation whose approach axis is down and toward the tail
        tilted = [w for w in self.PLAN[""] if hasattr(w.yaw, "rotation")]
        self.assertGreater(len(tilted), 15)
        pitches = []
        for w in tilted:
            rot = D.grip_rotation(w.yaw)
            self.assertAlmostEqual(float(rot[1, 2]), 0.0, places=9, msg=w.label)  # pitched about y alone
            self.assertAlmostEqual(float(rot[1, 0]), 0.0, places=9, msg=w.label)
            self.assertLess(float(rot[2, 2]), 0.0, w.label)  # the fingertips point down, toward the tail
            self.assertGreater(float(rot[0, 2]), -0.2, w.label)
            pitches.append(math.degrees(math.asin(float(rot[0, 2]))))
            self.assertTrue(np.allclose(rot[:, 2], L.d_vec(pitches[-1]), atol=1e-9), w.label)
            self.assertTrue(np.allclose(rot[:, 0], L.n_vec(pitches[-1]), atol=1e-9), w.label)  # face to the tail
        self.assertAlmostEqual(max(pitches), L.PITCH_DEG, places=6)  # 35 deg wherever the lever is under 145 deg
        self.assertAlmostEqual(pitches[0], L.PITCH_DEG, places=6)
        self.assertLess(pitches[-1], 0.0)  # the last is flat on the stowed lever: 6 deg the other way

    def test_no_waypoint_puts_a_pad_inside_a_shape(self):
        tray = T.TRAY.rest_state()
        for variant in self.PLAN:
            plan, rows = self._run(variant)
            ends, t = [], 0.0
            for w in plan.waypoints:
                t += w.duration
                ends.append(min(int(round(t * L.CONTROL_HZ)) - 1, len(rows) - 1))
            for k in ends:
                t, label, pos, rot, arm, psi, eps = rows[k]
                pts = world_pad_points(arm)
                sc = mk.state_from_poses(SC, (*FLOOR, 0, 0, 0, 1), joints={S.PIN: psi, S.LEVER: eps - L.E0})
                for name, state in (("tray", tray), ("clipper", sc)):
                    if variant == "press_flat" and label.startswith(("press", "hold", "done")) and name == "clipper":
                        continue  # pressing the lever on its stop is the variant: the command goes 1.8 mm past it
                    inside = int((state.distance(pts) <= 1e-9).sum())
                    self.assertEqual(inside, 0, f"{variant} {label}: {inside} pad points in the {name}")

    def test_offline_playback_gives_the_expected_scores(self):
        if OFFLINE is None:
            self.skipTest("tools/offline is not next to this task")
        expected = {"": 1.0, "swing_only": 0.25, "press_flat": 0.0}
        for variant, want in expected.items():
            with tempfile.TemporaryDirectory() as tmp:
                out = pathlib.Path(tmp) / "report.json"
                env = dict(__import__("os").environ, DEMO_VARIANT=variant)
                done = subprocess.run([sys.executable, str(OFFLINE / "check_task.py"), str(HERE), "--json", str(out)],
                                      capture_output=True, text=True, env=env)
                self.assertEqual(done.returncode, 0, done.stdout[-2000:])
                report = json.loads(out.read_text())
            self.assertEqual(report["playback"]["proxy"]["predicted_task_score"], want, variant)
            self.assertEqual(report["playback"]["proxy"]["hook_errors"], {}, variant)
            if variant == "":
                self.assertIsNotNone(report["playback"]["proxy"]["success_held_at"])
                self.assertLessEqual(report["suggested_max_steps"], L_RUN_LIMIT)

    def test_the_lever_follows_the_pads(self):
        _, rows = self._run("")
        psi = [r[5] for r in rows]
        eps = [r[6] for r in rows]
        self.assertAlmostEqual(abs(psi[-1]) % (2 * math.pi), math.pi, delta=math.radians(5.0))  # squared to the blade
        self.assertAlmostEqual(eps[-1], L.EPS_STOW, delta=0.01)
        self.assertLess(max(abs(p) for p in psi), L.PIN_LIMIT)
        self.assertTrue(all(L.TH0 - 1e-9 <= r[6] <= L.EPS_STOW + 1e-9 for r in rows))  # never past a stop
        _, rows = self._run("swing_only")
        self.assertAlmostEqual(rows[-1][6], L.TH180, delta=math.radians(0.1))
        self.assertAlmostEqual(abs(rows[-1][5]), math.pi, delta=math.radians(5.0))
        _, rows = self._run("press_flat")
        self.assertAlmostEqual(rows[-1][6], L.TH0, places=9)  # pressed: it stays on its heel
        self.assertEqual(rows[-1][5], 0.0)

    def test_the_demo_ends_clear_and_starts_clear(self):
        plan, rows = self._run("")
        t, label, pos, rot, arm, psi, eps = rows[-1]
        st = states(psi=psi, lever=eps - L.E0)
        dist = S.clipper_distance(st[S.CLIPPER])
        self.assertGreater(rigstate.hands_gap({"left": AWAY["left"], "right": arm}, dist), 0.040)
        start = rigstate.Arm(*[np.array(v) for v in (K.tcp_pose(K.READY, K.BASES["station"]["right"])[0],
                                                       K.tcp_pose(K.READY, K.BASES["station"]["right"])[0]
                                                       + np.array([0.013, 0.0, -0.007]))], 1.0)
        s0 = states()
        d0 = S.clipper_distance(s0[S.CLIPPER])
        self.assertGreater(rigstate.hands_gap({"left": AWAY["left"], "right": start}, d0), 0.150)
        self.assertEqual({a for a in T.plans("")}, {"right"})  # one arm
        self.assertEqual(T.plans("swing_only")["right"][-1].label, "hold")
        with self.assertRaises(ValueError):
            T.plans("nonsense")

    def test_demo_geometry_follows_the_spec(self):
        wps = self.PLAN[""]
        w = {x.label: x for x in wps}
        down = [x for x in wps if x.label.startswith("down beside")][0]
        self.assertAlmostEqual((down.pos[2] - L.ORIGIN[2]) / L.MM, L.SWING_TIP_Z + L.TIP_PAST_TCP, places=6)
        circle = L.swing_circle()
        for p in circle:
            self.assertAlmostEqual(math.hypot(p[0] - L.PIN_X, p[1]), L.SWING_R, places=6)  # radius 45 about the pin
        speeds = [np.linalg.norm(b - a) / x.duration for a, b, x in zip(circle, circle[1:], wps[3:])]
        for v in speeds:
            self.assertAlmostEqual(v, T.SWING_SPEED, delta=0.5 * T.SWING_SPEED + 1.0)
        edges = L.flip_edges()
        e_first = np.array([edges[0][0], edges[0][1]])
        t_tip = np.array([L.PIN_X, L.ZH]) + L.LEV * L.lever_dir(L.TH180)
        d = L.d_vec(L.PITCH_DEG)
        self.assertTrue(np.allclose(e_first, t_tip + 15.0 * np.array([d[0], d[2]])))  # E = T + 15 d
        x_end, z_end, _, p_end = edges[-1]
        self.assertAlmostEqual(p_end, -math.degrees(L.BETA_STOW), places=6)
        self.assertAlmostEqual(x_end, 47.0, delta=0.3)  # about z 10.8 at x 47
        self.assertAlmostEqual(z_end, 10.8, delta=0.4)
        self.assertEqual(w["hold"].duration, T.HOLD_S)
        self.assertTrue(all(x.finger == 0.0 for x in wps))  # closed for the whole demo


def world_pad_points(arm):
    return FLOOR + OC.pad_points(arm.tcp, arm.tips, arm.opening, FLOOR) * L.MM


L_RUN_LIMIT = 800


class Text(unittest.TestCase):
    def test_fields_and_limits(self):
        board = {"title": 85, "instruction": 490, "context": 950, "setup": 3300, "scoring": 3150, "fidelity": 490,
                 "reset_notes": 580}
        for k, n in board.items():
            self.assertLessEqual(len(META[k]), n, k)
        self.assertEqual(META["instruction"], T.INSTRUCTION)
        self.assertEqual(META["run_limit"], L_RUN_LIMIT)
        self.assertRegex(META["scores"], r"^Dream-1 run: [0-9.]+ \(800 steps\); demo: 1\.0 \([0-9]+ steps\)$")
        self.assertEqual(META["category"], "Societal relatability")
        self.assertLess(len(META["instruction"]), 300)
        for word in ("swing", "flip", "half a turn", "turn it over", "pin"):  # the goal only, not the trick
            self.assertNotIn(word, META["instruction"].lower(), word)
        self.assertNotIn("media", META)
        scoring = META["scoring"]
        for text in ("within 10 degrees", "2 mm", "1.5 mm", "30 mm", "20 steps", "1 mm", "0.5:", "0.25:", "1.0:"):
            self.assertIn(text, scoring, text)
        self.assertEqual((L.TAIL_PLAN_DEG, L.PAST_PLAN_DEG, L.FLAT_GAP, L.SLOT_TOL, L.LET_GO, L.STILL_MOVE),
                         (10.0, 30.0, 0.002, 0.0015, 0.030, 0.001))
        setup = META["setup"]
        for text in ("88 x 18 mm", "85 mm long", "54 g", "5.5 degrees", "2.4 mm", "3.5 mm pin", "stop",
                     "Run limit 800 steps (40 s)"):
            self.assertIn(text, setup, text)
        self.assertAlmostEqual(L.LEV, 57.0)

    def test_files_are_clean(self):
        files = sorted(HERE.glob("*.py")) + sorted(HERE.glob("tests/*.py")) + [HERE / "task.json", HERE / "README.md"]
        for f in files:
            text = f.read_text()
            self.assertNotIn(chr(0x2014), text, f.name)  # no em dash
            self.assertNotIn(chr(0x2013), text, f.name)  # no en dash
            self.assertTrue(text.isascii(), f.name)
            if f.suffix == ".py":
                for i, line in enumerate(text.splitlines(), 1):
                    self.assertLessEqual(len(line), 120, f"{f.name}:{i}")
        prose = json.dumps({k: v for k, v in META.items() if k != "scores"}) + README
        barred = (r"\bscaled?\b", r"\bmodel of\b", r"\banalogue\b", r"\bmock\b", r"\btrainer\b", r"stand-in",
                  r"\bstands? for\b", r"\brepresents\b", r"\bbought\b", r"\bbuy", r"\bTBD\b")
        for pat in barred:
            self.assertIsNone(re.search(pat, prose, flags=re.IGNORECASE), pat)

    def test_readme(self):
        flat = " ".join(README.split())
        self.assertLessEqual(len(README), 10000)
        for text in ("Trim 3.25 x 0.5 in", "Seki Edge SS-107", "Tweezerman", "littlemechanic.com", "compound lever",
                     "## Results in simulation", "DEMO_VARIANT=swing_only", "DEMO_VARIANT=press_flat", "TASK_TRACE",
                     "python3 -m unittest discover -s tests", "Open loop", "stops in its hinge",
                     "GPU run 2026-11-14"):
            self.assertIn(text, flat, text)
        self.assertIn("Run limit 800 steps (40 s)", META["setup"])
        self.assertIn("run limit 800 steps (40 s)", README)
        for text in ("two arms are needed", "needs two", "check_task", "tools/", "offline prediction"):
            self.assertNotIn(text, flat, text)
        section = README.split("## Results in simulation", 1)[1].split("\n## ", 1)[0]
        for token in ("nothing", "demo", "dream", "swing_only", "press_flat"):
            self.assertLessEqual(section.count(f"@@{token}@@"), 1, token)


if __name__ == "__main__":
    unittest.main()
