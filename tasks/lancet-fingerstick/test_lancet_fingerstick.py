"""Offline tests for lancet_fingerstick.py (no simulator): the site, the press and firing model, the scorer, the lazy
routes, and both arms' demo replayed through the model on each film seed by a kinematic stand-in for the sim (each arm
follows its plan's commanded TCP pose, lancet_fingerstick.TiltPlan; a gripped lancet follows the gripper's pose; the
press along the lancet's axis is the arm's compliance in series with the shield's spring and the ring finger's vertical
slide; an unheld finger gives way down its slide). Run with yam_arena on PYTHONPATH."""
import dataclasses
import json
import math
import pathlib
import types
import unittest
import warnings
import zlib

import numpy as np

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parent
from yam_arena import station  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

M = load(HERE / "lancet_fingerstick.py").module


def _tool(name):
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, HERE / "tools" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


YS = _tool("yieldsweep")
FIRE_MARGIN = 1.0                # N: every allowed press on the unheld finger at F_FIRE pushes it down by this much
#                                  more than it takes to give it DROP_MAX (with no friction; README The model)
CLI = (0, 2, 4)
N_SEEDS = 20                     # inspect-robots --seed 0..19 for the many-seed replay
N_PAIRS = 20                     # seed pairs (a, a + 20) for the open-loop replay
SIM_SEEDS = (26385, 84277, 5410, 55059, 126748, 5673, 58369, 100825)   # --seed nearest each layout corner (README)
DT = 0.05
K_ARM = 500.0                    # N/m at the TCP [EST, as the task's PRESS_DZ comment]
GRIP_READ = -(M.L_T / 2 - 0.0003)   # the finger's reading closed on the 10 mm side: 0.3 mm in [RUN, sim trace]
GRIP_R_READ = -M.HOLD_GAP_ON     # a finger closed on the ring finger reads it 0.6 mm in [EST, as HOLD_GAP_ON]
UP = np.array([0.0, 0.0, 1.0])
PRESS_LABELS = {"tilt over the site", "steady over the site", "just off the site", "steady just off the site",
                "tip on the site", "load the press", "settle the press", "press until it fires", "hold the press",
                "back off along the axis"}


def trial_seed(cli):
    return zlib.crc32(f"{cli}:None:0".encode())


def q_from_R(r):
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def st(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    return types.SimpleNamespace(pos=np.asarray(pos, float), quat_xyzw=np.asarray(quat, float),
                                 lin_vel=np.zeros(3))


def ring_link(seed, drop=0.0):
    """The ring link's frame (world pos, quat): the mechanism's frame, lowered drop down the slide."""
    hx, hy, hz, yaw = M.layout(seed)["hand"]
    return np.array([hx, hy, hz - drop]), q_from_R(rz(yaw))


def hold_grip(seed):
    """The left TCP closed on the ring finger at the demo's hold (world)."""
    return M.hold_pose(seed)[0]


def lq(seed, turn=0.0):
    """The demo's holding gripper quat (its closing direction across the finger), turned `turn` about its approach."""
    _, a, R = M.hold_pose(seed)
    c, s = math.cos(turn), math.sin(turn)
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return q_from_R((np.eye(3) + s * K + (1 - c) * K @ K) @ R)


def square_quat(n_in_world):
    """A lancet quat square to skin whose inward normal is n_in_world: its z (up the housing) along the outward
    normal, so it points along the inward one."""
    n = -np.asarray(n_in_world, float)
    perp = np.cross(n, [1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.cross(n, [0.0, 1.0, 0.0])
    return q_from_R(M.rot_from(n, perp))


def _rows(R):
    return tuple(tuple(float(x) for x in r) for r in np.asarray(R))


class Kin:
    """A kinematic stand-in for the sim, both arms, each following its TiltPlan's commanded TCP pose."""
    GRAB = 0.004

    def __init__(self, seed):
        self.seed = seed
        self.l = np.array(M.L_START, float)
        self.lR = np.eye(3)
        self.tab, self.q, self.drop = 0.0, 0.0, 0.0
        self.held, self.off = False, None
        self.r = (np.array([0.323, -0.305, 1.008]), M.grip_rotation(0.0), M.OPEN_F)
        self.lf = (np.array([0.323, 0.305, 1.008]), M.grip_rotation(0.0), M.OPEN_F)
        self.trace = []
        self.hook = None

    def _lancet_quat(self):
        return q_from_R(self.lR)

    def _left_on_finger(self):
        """(the left closed round the ring finger, its pull up on it (N), its TCP, reading and rotation as the sim
        gives them). WHY the lift: pads closed round the finger cannot move across it, so a TCP commanded over it (the
        demo's HOLD_UP) stops on the finger and pulls it up with the arm's stiffness."""
        p, Rg, f = self.lf
        hp, hq = ring_link(self.seed, 0.0)
        Rr = M._rot3(hq)
        if -f > M.FINGERS["ring"][4] + 0.0005:
            return False, 0.0, p.copy(), f, Rg
        c = Rg[:, 0]
        s = float((Rr.T @ (p - hp)) @ M.AXIS)
        d = p - (hp + Rr @ (s * M.AXIS))
        lift = float(d @ c) / float(UP @ c) if abs(float(UP @ c)) > 0.2 else 0.0
        tcp = p - lift * UP
        if not (-0.003 <= lift <= M.HOLD_UP + 0.005 and M.finger_held((tcp, GRIP_R_READ, q_from_R(Rg)), hp, hq)):
            return False, 0.0, p.copy(), f, Rg
        return True, K_ARM * max(0.0, lift), tcp, GRIP_R_READ, Rg

    def _press(self, cmd_face, a):
        """(the shield's travel, the finger's drop, the face's point) for a commanded face point pressed along a: the
        arm (K_ARM), the shield and the finger's vertical slide in series (the slide gives drop |a_z| along a); a
        finger held by the left also has the left arm's stiffness and its pull behind it."""
        hp, hq = ring_link(self.seed, 0.0)
        depth = self._depth(cmd_face, a, hp, M._rot3(hq))
        if depth <= 0.0:
            self.contact = None
            return 0.0, 0.0, cmd_face
        # WHY: the demo drives the press along press_dir, off a by the sideways give of the arm under the press; that
        # sideways part is taken up by the arm's compliance, not by the face, so it is removed here (the face stays on
        # the point it met). Any other sideways command (a stem's) still moves the face.
        if getattr(self, "contact", None) is None:
            self.contact = cmd_face.copy()
            ad = M.press_dir(self.seed)
            u = ad - (ad @ a) * a
            self.u = u / max(np.linalg.norm(u), 1e-9)
            a0, v, _ = M._press_give(self.seed)
            self.c_ax = float(v @ a0)     # WHY the task's own J K^-1 J^T: PhysX gave within 4% of it (seed 0)
        cmd_face = cmd_face - float((cmd_face - self.contact) @ self.u) * self.u
        depth = self._depth(cmd_face, a, hp, M._rot3(hq))
        on, pull, _, _, _ = self._left_on_finger()
        k_fin = M.K_RING + (K_ARM if on else 0.0)
        pre = M.RING_PRELOAD + pull
        az = abs(float(a[2]))

        def parts(f):
            q = 0.0 if f <= M.F_PRE else min(M.SHIELD_TRAVEL, (f - M.F_PRE) / M.K_SHIELD)
            drop = min(M.DROP_STOP, max(0.0, (f * az - pre) / k_fin))
            return q, drop
        lo, hi = 0.0, 200.0
        for _ in range(60):
            f = (lo + hi) / 2
            q, drop = parts(f)
            lo, hi = (f, hi) if f * self.c_ax + q + drop * az < depth else (lo, f)
        q, drop = parts(lo)
        # WHY the face stays on the skin point it met, moved down with the finger: friction holds it there and the arm
        # gives across
        return q, drop, cmd_face - depth * a - drop * UP

    def _depth(self, face, a, hp, Rh):
        """How far (m) a commanded face point sits past the trainer's surface, back along a (0 if none)."""
        def inside(t):
            s = M.site_of(Rh.T @ (face - t * a - hp))
            return s is not None and s[4] < 0.0 and "palm" not in s[0]
        top = None
        for i in range(101):
            if inside(0.0005 * i):
                top = 0.0005 * i
        if top is None:
            return 0.0
        lo, hi = top, top + 0.0005
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if inside(mid) else (lo, mid)
        return hi

    def step(self):
        p, Rg, f = self.r
        z_l = self.lR[:, 2]
        grip = self.l + z_l * M.tcp_over_lancet()
        closed = -f <= M.L_T / 2 + 0.0005
        if not self.held and closed and np.linalg.norm(p - grip) <= self.GRAB:
            self.held, self.off = True, Rg.T @ self.lR
        elif self.held and not closed:
            self.held = False
        self.q, self.drop = 0.0, 0.0
        if self.held:
            self.lR = Rg @ self.off
            a = -self.lR[:, 2]
            cmd = p + M.tcp_over_lancet() * a
            if M.in_socket(cmd) and abs(self.tab) < M.TAB_STOP:
                lyaw = math.atan2(self.lR[1, 0], self.lR[0, 0])
                if abs(lyaw) > abs(self.tab):
                    self.tab = max(-M.TAB_STOP, min(M.TAB_STOP, lyaw))
            if not M.in_socket(cmd):
                self.q, self.drop, face = self._press(cmd.copy(), a)
                self.l = face + self.q * a
            else:
                # in the stand the shield rests on the tab cap: a TCP commanded under where the lancet stands
                # compresses the shield (the arm's compliance in series), the housing stays where it stood
                depth = max(0.0, M.L_START[2] - cmd[2])
                f = (depth + M.F_PRE / M.K_SHIELD) / (1 / M.K_SHIELD + 1 / K_ARM) if depth > 0 else 0.0
                self.q = min(M.SHIELD_TRAVEL, max(0.0, (f - M.F_PRE) / M.K_SHIELD)) if f > M.F_PRE else 0.0
                self.l = np.array([cmd[0], cmd[1], max(cmd[2], M.L_START[2])])
        elif not M.in_socket(self.l) and not M.in_sharps(self.l) and self.l[2] > M.TOP + 0.0005:
            ox, oy = M.SH_OPEN[0] / 2, M.SH_OPEN[1] / 2
            if abs(self.l[0] - M.SHARPS[0]) <= ox and abs(self.l[1] - M.SHARPS[1]) <= oy:
                self.l = np.array([self.l[0], self.l[1], M.TOP + M.SH_WALL])
            else:
                self.l = np.array([self.l[0], self.l[1], M.TOP])
        hp, hq = ring_link(self.seed, self.drop)
        # a finger closed on the housing stops at it: report what the sim reads, not the command
        right = (p.copy(), min(f, GRIP_READ) if self.held else f, q_from_R(Rg))
        _, _, ltcp, lread, lR = self._left_on_finger()
        left = (ltcp, lread, q_from_R(lR))
        M.advance(M._M, DT, self.l.copy(), self._lancet_quat(), self.q, self.tab, -self.drop, hp, hq, right, left)
        M._CLOCK["dt"] = DT
        M._CLOCK["steps"] += 1
        M._M["steps"] += 1
        self.trace.append(self.objects())

    def objects(self):
        hx, hy, hz, yaw = M.layout(self.seed)["hand"]
        return {"hand": st((hx, hy, hz), q_from_R(rz(yaw))), "lancet": st(self.l, self._lancet_quat()),
                "bench": st((0.0, 0.0, M.TOP))}

    def run(self, right, left, tail=2.0):
        pr = M.TiltPlan(list(right), station.BASES["right"])
        pl = M.TiltPlan(list(left), station.BASES["left"])
        t, end = 0.0, max(pr.duration, pl.duration) + tail
        while t < end - 1e-9:
            t += DT
            self.r = tuple(pr.target(t)[:3])
            self.lf = tuple(pl.target(t)[:3])
            self.step()
            M.update(self.trace[-1])
            if self.hook:
                self.hook()
        return M.score(self.trace[-1])


def replay(seed, plan_seed=None, edit=None):
    M.reset_memory(seed)
    right, left = M.plan(0.0, seed if plan_seed is None else plan_seed)
    if edit:
        right, left = edit(right, left)
    k = Kin(seed)
    return k.run(right, left), k


def site_point(name, s, ang):
    """A point on a finger's distal pad (mechanism frame): s back from its apex, ang round from the pad's middle line
    (toward the thumb, +v, positive)."""
    v, u0, _, rd, _ = M.FINGERS[name]
    return M.hand_to_mech((u0 + s, v + rd * math.sin(ang), rd * math.cos(ang)))


def out_normal(ang):
    """The pad's outward normal (mechanism frame) at ang round from its middle line."""
    return M.R_HAND @ np.array([0.0, math.sin(ang), math.cos(ang)])


def fresh():
    M.reset_memory(0)
    M._M["tab_off"] = M._M["out"] = True
    return M._M


GOOD = site_point("ring", M.SITE_AIM, M.SITE_TURN)
OFF = GOOD + 0.020 * out_normal(M.SITE_TURN)          # 20 mm off the site along its outward normal: in the air
SQ = square_quat(M._rot3(ring_link(0)[1]) @ M.site_of(GOOD)[3])   # square to the skin at GOOD, seed 0


def lancet_pos(face, q, l_quat):
    """The lancet's frame for its shield's face at `face`, the shield pushed in q."""
    return np.asarray(face, float) - M._rot3(l_quat) @ np.array([0.0, 0.0, q])


def press_at(m, p_mech, f, drop=0.0, l_quat=None, seed=0, right=None, held=True):
    """One model step with the shield's face at p_mech (ring frame at rest) under force f, the lancet held (square to
    the skin at GOOD by default), the ring finger dropped `drop` down its slide and held by the left (held) or not."""
    l_quat = SQ if l_quat is None else l_quat
    hp, hq = ring_link(seed, drop)
    face = hp + M._rot3(hq) @ np.asarray(p_mech, float)
    q = max(0.0, (f - M.F_PRE) / M.K_SHIELD) if f > M.F_PRE else 0.0
    l_pos = lancet_pos(face, q, l_quat)
    tcp = M.grip_point(l_pos, l_quat)
    left = (hold_grip(seed) - [0, 0, drop], GRIP_R_READ, lq(seed)) if held else None
    M.advance(m, DT, l_pos, l_quat, q, M.TAB_BREAK + 0.1, -drop, hp, hq,
              right if right is not None else (tcp, GRIP_READ), left)


class Model(unittest.TestCase):
    def test_shield_force(self):
        self.assertEqual(M.shield_force(0.0), 0.0)
        self.assertAlmostEqual(M.shield_force(0.001), M.F_PRE + M.K_SHIELD * 0.001)
        q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD
        self.assertLess(q_fire, M.SHIELD_TRAVEL)    # it fires before the shield reaches its stop

    def test_demo_site_is_valid_and_its_press_square(self):
        """The demo's site is on the side of the ring's pad, and its press direction is the skin's inward normal there,
        48 deg from vertical (README Design)."""
        for seed in (0, trial_seed(2)) + tuple(M.corners()):
            site, a, _ = M.press_pose(seed)
            hp, hq = ring_link(seed)
            s = M.site_of(M._rot3(hq).T @ (site - hp))
            self.assertEqual(s[0], "ring")
            self.assertEqual(M.site_ok(s), (True, ""))
            self.assertAlmostEqual(abs(s[4]), 0.0, places=9)
            np.testing.assert_allclose(M._rot3(hq) @ s[3], a, atol=1e-9)
            self.assertAlmostEqual(math.degrees(math.acos(-a[2])), 48.4, delta=0.1)

    def test_site_rules(self):
        cases = [(("ring", 0.010, 0.0), "middle of the ring's pad"),
                 (("ring", 0.010, -math.radians(70)), "too far round"),
                 (("ring", 0.010, math.radians(65)), "too far round"),
                 (("ring", 0.020, -0.5), "off the pad"),
                 (("ring", 0.006, -0.5), "off the pad"),          # under 8 mm: on the cap, not scored
                 (("index", 0.010, -0.5), "not the side of the ring fingertip"),
                 (("ring", 0.010, -0.5), None),
                 (("ring", 0.010, 0.5), None),                  # either side (BD shades both)
                 (("ring", 0.010, math.radians(10)), "middle of the ring's pad")]
        for (name, s, ang), why in cases:
            ok, msg = M.site_ok(M.site_of(site_point(name, s, ang)))
            if why is None:
                self.assertTrue(ok, msg)
            else:
                self.assertFalse(ok)
                self.assertIn(why, msg)
        self.assertIn("not the fingertip", M.site_of(site_point("ring", 0.040, -0.5))[0])
        self.assertIsNone(M.site_of(np.array([0.0, 0.0, 0.05])))

    def test_lifted_with_tab_on_latches(self):
        M.reset_memory(0)
        lp = np.array(M.L_START) + [0, 0, M.LIFT_OUT + 0.002]
        hp, hq = ring_link(0)
        M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, math.radians(30), 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)),
                                                                                  GRIP_READ))
        self.assertIn("tab still on", M._M["fault"])

    def test_twisted_then_lifted_is_fine(self):
        M.reset_memory(0)
        hp, hq = ring_link(0)
        lp = np.array(M.L_START)
        M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, M.TAB_BREAK + 0.01, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)),
                                                                                    GRIP_READ))
        lp = lp + [0, 0, M.LIFT_OUT + 0.002]
        M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, 0.0, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)), GRIP_READ))
        self.assertIsNone(M._M["fault"])
        self.assertTrue(M._M["tab_off"] and M._M["out"])

    def test_good_press_fires_clean(self):
        m = fresh()
        for f in (2.5, 5.0, 8.5):
            press_at(m, GOOD, f)
        self.assertTrue(m["fired"])
        self.assertTrue(m["positioned"])
        self.assertIsNone(m["fault"])

    def test_placed_needs_the_finger_held(self):
        """A touch on a good site is "placed" (a trace field, not a rung) only with a gripper closed on the ring finger
        since the first touch."""
        m = fresh()
        press_at(m, GOOD, M.F_TOUCH + 0.5)
        self.assertTrue(m["positioned"])
        m = fresh()
        press_at(m, GOOD, M.F_TOUCH + 0.5, held=False)
        self.assertFalse(m["positioned"])
        press_at(m, GOOD, M.F_TOUCH + 0.5)
        self.assertFalse(m["positioned"])   # held now, but not from the first touch

    def test_touch_is_contact(self):
        """The touch is geometric, as a camera sees it: the shield's face within TOUCH_D (0.3 mm) of the ring finger's
        skin, or the shield off its stop with its face on it. 0.5 mm off with no force is not a touch."""
        self.assertAlmostEqual(M.F_TOUCH, M.F_PRE + M.K_SHIELD * M.Q_TOUCH)
        self.assertEqual(M.shield_force(M.Q_TOUCH * 0.99), 0.0)
        self.assertGreaterEqual(M.shield_force(M.Q_TOUCH * 1.01), M.F_TOUCH)
        out = out_normal(M.SITE_TURN)
        m = fresh()
        press_at(m, GOOD + 0.0005 * out, 0.0)
        self.assertIsNone(m["touch_t"], "0.5 mm off the skin, no force")
        press_at(m, GOOD + 0.0002 * out, 0.0)
        self.assertIsNotNone(m["touch_t"], "0.2 mm off the skin, no force: the camera sees contact")
        m = fresh()
        press_at(m, GOOD, M.F_TOUCH + 0.01)
        self.assertIsNotNone(m["touch_t"])

    def test_a_brush_before_the_hold_is_a_fault(self):
        """The window runs from the first touch, so the shield brushing the finger before the left holds it is a fault
        by design, even with no force."""
        m = fresh()
        press_at(m, GOOD, 0.0, held=False)
        press_at(m, OFF, 0.0)
        for f in (2.5, 5.0, 8.5):
            press_at(m, GOOD, f)
        self.assertTrue(m["fired"])
        self.assertIn("not held from first touch", m["fault"])

    def test_a_hold_near_the_press_that_also_reads_as_holding_the_lancet_counts(self):
        """Review of e233a08: a hold at the distal joint (27 mm back, half-gap 6.5 mm, tilt 30 deg) with a press 15 mm
        back, 60 deg round. That gripper is inside held_by's window for the lancet too; it must still count as holding
        the finger, and it is not the left holding the lancet."""
        seed = 0
        hp, hq = ring_link(seed)
        ang = -math.radians(59.5)   # WHY 59.5: 60 is SITE_ANG's bound, where rounding puts it either side
        site_m = site_point("ring", 0.015, ang)
        lq_ = q_from_R(M.hold_pose(seed, s=0.027)[2])
        left = (M.hold_pose(seed, s=0.027)[0], -0.0065, lq_)
        sq = square_quat(M._rot3(hq) @ M.site_of(site_m)[3])
        m = fresh()
        overlap = False
        for f in (0.0, 2.5, 5.0, 8.5, 0.0, 0.0):
            face = hp + M._rot3(hq) @ site_m
            q = (f - M.F_PRE) / M.K_SHIELD if f > M.F_PRE else 0.0
            l_pos = lancet_pos(face, q, sq)
            overlap |= M.held_by(left, l_pos, sq)
            self.assertTrue(M.finger_held(left, hp, hq))
            M.advance(m, DT, l_pos, sq, q, M.TAB_BREAK + 0.1, 0.0, hp, hq, (M.grip_point(l_pos, sq), GRIP_READ), left)
        self.assertTrue(overlap, "the case needs a hold that held_by also calls the lancet's")
        self.assertTrue(m["fired"])
        self.assertIsNone(m["fault"])

    def test_either_gripper_may_carry_the_lancet(self):
        """Review of e233a08 asked for one rule in both the sim and the real trial; the hand-off is legal in both. The
        left holding the armed lancet is no fault (the instruction assigns no gripper, the #826 rejection), and the
        real trial has no E-stop for it (README, Rig protection)."""
        m = fresh()
        l_pos = lancet_pos(GOOD + 0.020 * out_normal(M.SITE_TURN), 0.0, SQ)
        hp, hq = ring_link(0)
        grip = (M.grip_point(l_pos, SQ), GRIP_READ)
        for _ in range(3):
            M.advance(m, DT, l_pos, SQ, 0.0, M.TAB_BREAK + 0.1, 0.0, hp, hq, None, grip)
        self.assertTrue(M.held_by(grip, l_pos, SQ))
        self.assertIsNone(m["fault"])

    def test_a_touch_off_the_ring_finger_does_not_start_the_window(self):
        """Review F1: the shield pressed on the table (1.8 N, no hold), then on the index, then lifted, then a held
        press on the site fires clean. Before the fix the table touch started the window and the held fire latched
        "not held"."""
        m = fresh()
        table = np.array([0.050, 0.080, -M.HZ + M.BOARD_T])
        self.assertIsNone(M.site_of(table))
        index = site_point("index", M.SITE_AIM, 0.0)
        for p in (table, index):
            for _ in range(3):
                press_at(m, p, 1.8, held=False)
            press_at(m, p, 0.0, held=False)
        self.assertIsNone(m["touch_t"])
        self.assertIsNone(m["fault"])
        for f in (2.5, 5.0, 8.5):
            press_at(m, GOOD, f)
        self.assertTrue(m["fired"])
        self.assertIsNone(m["fault"])

    def test_hold_must_last_from_touch_to_after_the_click(self):
        """F2: the finger let go of at any step from the first touch to 0.5 s after the click scores 0 (latched at the
        click, or when let go after it); let go later, it does not count."""
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD, 5.0, held=False)
        press_at(m, GOOD, 5.0)
        self.assertIsNone(m["fault"])
        press_at(m, GOOD, 9.0)
        self.assertIn("not held from first touch", m["fault"])
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD, 9.0)
        for _ in range(int(0.3 / DT)):
            press_at(m, OFF, 0.0)
        press_at(m, OFF, 0.0, held=False)
        self.assertIn("not held from first touch", m["fault"])
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD, 9.0)
        for _ in range(int(0.6 / DT)):
            press_at(m, OFF, 0.0)
        press_at(m, OFF, 0.0, held=False)
        self.assertIsNone(m["fault"])

    def test_ring_pinned_at_its_lower_stop(self):
        """A finger pushed down onto its lower stop (rigid there) and pressed is not held still: the drop counts from
        the top stop."""
        m = fresh()
        press_at(m, GOOD, 3.0, drop=M.DROP_STOP)
        press_at(m, GOOD, 9.0, drop=M.DROP_STOP)
        self.assertIn("not held still", m["fault"])

    def test_still_is_read_at_the_fingertip_too(self):
        """B1: a ring finger that pitches on its slide (its tip down 3 mm, the slide's reading still 0) is not still:
        the fingertip mark is read against the base, as the real trial reads it."""
        m = fresh()
        hp, hq = ring_link(0)
        base = (hp.copy(), hq.copy())
        lp = hp + M._rot3(hq) @ np.asarray(GOOD, float)
        M.advance(m, DT, lp, SQ, 0.0, M.TAB_BREAK + 0.1, 0.0, hp, hq, None, None, base)
        self.assertIsNone(m["fault"])
        anchor = np.array([M.WEB_X[0], 0.0, M.SLIDE_Z])     # the slide joint's anchor, behind the knuckle
        mark = M.TIP_MARK * M.AXIS
        a = math.asin(0.003 / (anchor[0] - mark[0]))         # turned about the anchor, tip down about 3 mm
        piv = hp + M._rot3(hq) @ anchor
        Rp = M._rot3(hq) @ ry(-a) @ M._rot3(hq).T
        rq = q_from_R(Rp @ M._rot3(hq))
        rpos = piv + Rp @ (hp - piv)
        self.assertGreater(M.tip_height(hp, hq, *base) - M.tip_height(rpos, rq, *base), M.DROP_MAX)
        face = rpos + M._rot3(rq) @ np.asarray(GOOD, float)
        q = (3.0 - M.F_PRE) / M.K_SHIELD
        left = (hold_grip(0), GRIP_R_READ, lq(0))
        lp = lancet_pos(face, q, SQ)
        M.advance(m, DT, lp, SQ, q, M.TAB_BREAK + 0.1, 0.0, rpos, rq, (M.grip_point(lp, SQ), GRIP_READ), left, base)
        self.assertIn("not held still", m["fault"] or "")

    def test_fire_without_holding_the_finger_latches(self):
        """Fired with the finger still (say, a light finger under a fast jab) but no gripper on it: 0."""
        m = fresh()
        press_at(m, GOOD, 3.0, held=False)
        press_at(m, GOOD, 9.0, held=False)
        self.assertIn("no gripper holding the ring finger", m["fault"])

    def test_no_fire_under_the_fire_force(self):
        m = fresh()
        press_at(m, GOOD, M.F_FIRE - 0.2)
        self.assertFalse(m["fired"])

    def test_slide_latches(self):
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD + (M.SLIDE_MAX + 0.0005) * M.AXIS, 4.0)
        self.assertIn("slid", m["fault"])

    def test_small_creep_and_lift_reset_are_fine(self):
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD + 0.0005 * M.AXIS, 4.0)
        press_at(m, GOOD + 0.003 * M.AXIS, 0.0)   # lifted off: a new placement may start elsewhere
        press_at(m, GOOD + 0.003 * M.AXIS, 3.0)
        press_at(m, GOOD + 0.003 * M.AXIS, 9.0)
        self.assertIsNone(m["fault"])
        self.assertTrue(m["fired"])

    def test_tilted_lancet_latches(self):
        m = fresh()
        tq = q_from_R(M._rot3(SQ) @ rx(math.radians(10.5)))   # 10.5 deg off square
        press_at(m, GOOD, 3.0, l_quat=tq)
        press_at(m, GOOD, 9.0, l_quat=tq)
        self.assertIn("tilted", m["fault"])

    def test_finger_that_gives_latches(self):
        m = fresh()
        press_at(m, GOOD, 3.0, drop=M.DROP_MAX + 0.001)
        press_at(m, GOOD, 9.0, drop=M.DROP_MAX + 0.001)
        self.assertIn("not held still", m["fault"])

    def test_jab_window_runs_past_the_click(self):
        """B1: a jab that fires before the finger moves, which then gives way within 0.5 s of the click, scores 0; the
        finger moving after the window (the lancet long lifted) does not count."""
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD, 9.0)
        self.assertTrue(m["fired"])
        self.assertIsNone(m["fault"])
        for _ in range(int(0.4 / DT)):
            press_at(m, OFF, 0.0, drop=0.0)
        press_at(m, OFF, 0.0, drop=0.005)
        self.assertIn("within 0.5 s of the click", m["fault"])
        m = fresh()
        press_at(m, GOOD, 3.0)
        press_at(m, GOOD, 9.0)
        for _ in range(int(0.6 / DT)):
            press_at(m, OFF, 0.0)
        press_at(m, OFF, 0.0, drop=0.005)
        self.assertIsNone(m["fault"])

    def test_window_starts_at_first_touch_whatever_the_force(self):
        """The finger moved under a touch, before any firm press, counts too (the window starts at first touch)."""
        m = fresh()
        press_at(m, GOOD, M.F_TOUCH + 0.1)
        press_at(m, GOOD, 0.0, drop=0.004)    # lifted off, but the finger was pushed down meanwhile
        self.assertIn("not held still", m["fault"])

    def test_tab_breaks_either_way(self):
        for sign in (1, -1):
            M.reset_memory(0)
            hp, hq = ring_link(0)
            lp = np.array(M.L_START)
            M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, sign * (M.TAB_BREAK + 0.01), 0.0, hp, hq,
                      (M.grip_point(lp, (0, 0, 0, 1)), GRIP_READ))
            self.assertTrue(M._M["tab_off"], sign)
            M.reset_memory(0)
            M.advance(M._M, DT, lp, (0, 0, 0, 1), 0.0, sign * (M.TAB_BREAK - 0.01), 0.0, hp, hq,
                      (M.grip_point(lp, (0, 0, 0, 1)), GRIP_READ))
            self.assertFalse(M._M["tab_off"], sign)

    def test_finger_held_rule(self):
        """Held: a gripper closed on the ring finger between its distal joint and its base. Closed on the index finger,
        over the palm, open, or closed on nothing, it is not."""
        hp, hq = ring_link(0)
        R = M._rot3(hq)
        _, a, Rh = M.hold_pose(0)
        c = Rh[:, 0]

        def on(s, across=0.0, back=M.HOLD_BACK, name="ring"):
            v, u0 = M.FINGERS[name][0], M.FINGERS[name][1]
            return hp + R @ M.hand_to_mech((u0 + s, v, 0.0)) + across * c - back * a
        q = lq(0)
        self.assertTrue(M.finger_held((hold_grip(0), GRIP_R_READ, q), hp, hq))
        self.assertTrue(M.finger_held((on(0.040), GRIP_R_READ, q), hp, hq))
        self.assertTrue(M.finger_held((on(M.HOLD_AT, M.HOLD_ACROSS - 0.0005), GRIP_R_READ, q), hp, hq))
        self.assertTrue(M.finger_held((on(M.HOLD_AT, back=-0.008), GRIP_R_READ, q), hp, hq))   # in deeper
        self.assertFalse(M.finger_held((hold_grip(0), GRIP_R_READ), hp, hq), "no quat: closing direction unknown")
        palm = hp + R @ M.hand_to_mech((0.150, -0.030, M.PALM_W + 0.005))
        for tcp, gap, why in ((on(M.HOLD_AT, name="index"), -(M.FINGERS["index"][4] - 0.0005), "index"),
                              (palm, GRIP_R_READ, "palm"),
                              (hold_grip(0), M.OPEN_R, "open"), (hold_grip(0), -0.001, "on nothing"),
                              (on(0.012), GRIP_R_READ, "on the pad, past the distal joint"),
                              (on(M.KNUCKLE_S + 0.002), GRIP_R_READ, "behind the knuckle line"),
                              (on(M.HOLD_AT, 0.004), -0.005, "closed on air 4 mm beside the ring"),
                              (on(M.HOLD_AT, M.HOLD_ACROSS + 0.0005), GRIP_R_READ, "4.5 mm beside, reading as on it"),
                              (on(M.HOLD_AT, back=M.TIP + 0.001), GRIP_R_READ, "short of it, pad tips before the axis"),
                              (on(M.HOLD_AT, 0.026), -0.005, "stem a00: on air above the finger")):
            self.assertFalse(M.finger_held((tcp, gap, q), hp, hq), why)
        # the half-gap window: the traces' readings on the finger pass (7.84 to 7.98 mm over five layouts in PhysX);
        # wider, the pads are off the skin (review of e233a08: 8.6 and 8.7 mm used to pass). 8.1 is 0.12 mm past the
        # widest reading, so it still passes; 8.3 mm and up do not
        for gap in (7.84, 7.98, 8.1):
            self.assertTrue(M.finger_held((hold_grip(0), -gap / 1000, q), hp, hq), gap)
        for gap in (8.3, 8.6, 8.7):
            self.assertFalse(M.finger_held((hold_grip(0), -gap / 1000, q), hp, hq), gap)
        self.assertLess(M.HOLD_GAP[1], M.HOLD_GAP_ON + 0.0004)
        # turned 90 deg (closing along the finger), the same TCP on the axis is still on it: across is 0 either way,
        # but the gap it reads closed along the finger would be its length, not the 7.8 mm radius
        self.assertTrue(M.finger_held((hold_grip(0), GRIP_R_READ, lq(0, math.pi / 2)), hp, hq))

    def test_square_is_against_the_local_normal_at_every_site(self):
        """The square rule reads the lancet against the skin's inward normal where it touches, at every allowed site
        (site_of's normal, not the vertical): a lancet along that normal is square everywhere, a vertical one is off by
        the normal's own lean, and leaning 0.5 deg past TILT_MAX off the normal is not square."""
        for s, a, p, n in YS.sites(-1) + YS.sites(1):
            site = M.site_of(p)
            self.assertEqual(site[0], "ring")
            if min([abs(abs(a) - b) for b in M.SITE_ANG] + [abs(s - b) for b in M.SITE_S]) > 1e-6:   # WHY: on a bound, rounding puts it either side
                self.assertEqual(M.site_ok(site), (True, ""), (s, a))
            down = np.array([0.0, 0.0, -1.0])
            lean = math.acos(float(n @ down))
            self.assertAlmostEqual(M.tilt_of((0, 0, 0, 1), (0, 0, 0, 1), site[3]), lean, places=9)
            for extra, square in ((0.0, True), (M.TILT_MAX + math.radians(0.5), False)):
                ax = np.cross(n, [0.0, 1.0, 0.0])
                ax /= np.linalg.norm(ax)
                c, sn = math.cos(extra), math.sin(extra)
                d = c * n + sn * np.cross(ax, n)       # n turned `extra` about ax
                k = np.cross(down, d)
                ang = math.acos(max(-1.0, min(1.0, float(down @ d))))
                q = (0.0, 0.0, 0.0, 1.0) if np.linalg.norm(k) < 1e-12 else tuple(
                    list(k / np.linalg.norm(k) * math.sin(ang / 2)) + [math.cos(ang / 2)])
                self.assertEqual(M.tilt_of(q, (0, 0, 0, 1), site[3]) <= M.TILT_MAX, square, (s, a, extra))

    def test_fire_off_the_hand_latches(self):
        m = fresh()
        p = np.array([-0.05, 0.0, -M.HZ])   # on the table before the fingertips
        press_at(m, p, 3.0)
        press_at(m, p, 9.0)
        self.assertIn("not on the hand", m["fault"])

    def test_drop_outside_latches_and_sharps_does_not(self):
        m = fresh()
        hp, hq = ring_link(0)
        on_table = np.array([0.25, -0.05, M.TOP])
        for _ in range(int(M.DROP_S / DT) + 2):
            M.advance(m, DT, on_table, (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq, (np.array([0.2, -0.2, 1.0]), M.OPEN_F))
        self.assertIn("dropped", m["fault"])
        m = fresh()
        inside = np.array([M.SHARPS[0], M.SHARPS[1], M.TOP + M.SH_WALL])
        self.assertTrue(M.in_sharps(inside))
        for _ in range(40):
            M.advance(m, DT, inside, (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq, (np.array([0.2, -0.2, 1.0]), M.OPEN_F))
        self.assertIsNone(m["fault"])

    def test_grasp_across_the_width_is_held(self):
        """A gripper closed across the housing's 13 mm width holds it as one across its 10 mm thickness does: carried
        in the air for 2 s, neither is a drop. Open past the width, or closed on nothing, it is not held."""
        lp = np.array([0.20, -0.10, M.TOP + 0.10])
        hp, hq = ring_link(0)
        for gap in (M.L_T / 2 - 0.0003, M.L_W / 2 - 0.0003, M.L_W / 2 + 0.0004):
            m = fresh()
            for _ in range(int(2.0 / DT)):
                M.advance(m, DT, lp, (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)), -gap))
            self.assertIsNone(m["fault"], f"half-gap {1000 * gap:.1f} mm")
        g = M.grip_point(lp, (0, 0, 0, 1))
        for gap in (M.L_W / 2 + 0.001, 0.001):   # open past the width; closed on nothing (the demo's 1 mm command)
            self.assertFalse(M.held_by((g, -gap), lp, (0, 0, 0, 1)), f"half-gap {1000 * gap:.1f} mm")

    def test_out_of_the_stand_bounds(self):
        """Out of the stand: more than 4 mm across or 10 mm up from where it stood."""
        p0 = np.array(M.L_START)
        self.assertTrue(M.in_socket(p0 + [M.OUT_XY - 0.0005, 0, M.LIFT_OUT - 0.0005]))
        self.assertFalse(M.in_socket(p0 + [M.OUT_XY + 0.0005, 0, 0]))
        self.assertFalse(M.in_socket(p0 + [0, 0, M.LIFT_OUT + 0.0005]))

    def test_put_back_in_the_stand_and_let_go_is_a_drop(self):
        """Once out, the lancet stays out: put back in the stand and let go, it is a drop (task.json scoring)."""
        m = fresh()
        m["out"] = False
        hp, hq = ring_link(0)
        lp = np.array(M.L_START)
        M.advance(m, DT, lp + [0, 0, M.LIFT_OUT + 0.002], (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq,
                  (M.grip_point(lp + [0, 0, M.LIFT_OUT + 0.002], (0, 0, 0, 1)), GRIP_READ))
        self.assertTrue(m["out"])
        for _ in range(int(M.DROP_S / DT) + 2):
            M.advance(m, DT, lp, (0, 0, 0, 1), 0.0, 1.0, 0.0, hp, hq, (np.array([0.2, -0.2, 1.0]), M.OPEN_F))
        self.assertIn("dropped", m["fault"])

    def test_unheld_finger_gives_before_fire(self):
        """tools/yieldsweep.py over the whole site (the ring, 8 to 15 mm, 15 to 60 deg round either side), every lancet
        direction within the square rule's 10 deg of the local normal: with no friction, every press at F_FIRE pushes
        the unheld finger down by at least FIRE_MARGIN more than it takes to give it DROP_MAX. With friction 0.25 or
        more some allowed presses do not (the press is up to 70 deg from vertical): there the scorer's held rule, not
        the slide, scores a one-handed press 0 (test_fire_without_holding_the_finger_latches)."""
        k, where = YS.sweep(0.0, side=0)
        self.assertGreaterEqual(M.F_FIRE * k - YS.give_force(), FIRE_MARGIN, f"mu 0 at {where}")
        for mu in YS.MUS[1:]:
            k, where = YS.sweep(mu, side=0)
            self.assertLess(M.F_FIRE * k - YS.give_force(), 0.0, f"mu {mu} at {where}")
        # and the demo's hold covers what its press puts straight down at its peak (10 N [EST] along a) with margin
        a = M.press_pose(0)[1]
        self.assertGreater(M.RING_PRELOAD + K_ARM * M.HOLD_UP, 10.0 * abs(a[2]) + 2.0)

    def test_the_sweep_can_fail(self):
        """The check can fail: a spring five times as strong (5 N to spare) would hold the finger still under some
        allowed presses at 8 N, with no friction."""
        k, _ = YS.sweep(0.0, side=0)
        self.assertLess(M.F_FIRE * k - (5.0 + M.K_RING * M.DROP_MAX), 0.0)

    def test_fired_in_the_stand_latches(self):
        """Tab off and still in the stand, a press of F_FIRE on the shield fires it into the post: 0."""
        M.reset_memory(0)
        lp = np.array(M.L_START)
        hp, hq = ring_link(0)
        q = (M.F_FIRE + 0.1 - M.F_PRE) / M.K_SHIELD
        M.advance(M._M, DT, lp, (0, 0, 0, 1), q, M.TAB_BREAK + 0.1, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)),
                                                                                   GRIP_READ))
        self.assertTrue(M._M["fired"])
        self.assertIn("fired in the stand", M._M["fault"])
        M.reset_memory(0)   # tab still on: it does not fire
        M.advance(M._M, DT, lp, (0, 0, 0, 1), q, 0.1, 0.0, hp, hq, (M.grip_point(lp, (0, 0, 0, 1)), GRIP_READ))
        self.assertFalse(M._M["fired"])
        self.assertIsNone(M._M["fault"])

    def test_slipped_or_deep_grasp_in_the_container_is_still_held(self):
        """A gripper closed on the housing with its TCP 25 mm off the demo's grip point (slipped along the pads, or
        taken deep or high) still holds it: lowered into the container and kept there it scores 0.5, not 0.75. The
        old rule (TCP within 20 mm of the grip point) read it as let go."""
        inside = np.array([M.SHARPS[0], M.SHARPS[1], M.TOP + 0.150])
        q0 = (0.0, 0.0, 0.0, 1.0)
        g = M.grip_point(inside, q0)
        for off in (np.array([0.0, 0.0, -0.025]), np.array([0.0, 0.0, 0.020]), np.array([0.009, 0.0, -0.018])):
            tcp = g + off
            self.assertGreater(np.linalg.norm(tcp - g), 0.020)   # the old rule: let go
            m = fresh()
            m["fired"] = True
            hp, hq = ring_link(0)
            for _ in range(int(2.0 / DT)):
                M.advance(m, DT, inside, q0, 0.0, 1.0, 0.0, hp, hq, (tcp, GRIP_READ), None)
                M._CLOCK["dt"] = DT
                M._CLOCK["steps"] += 1
                m["steps"] += 1
            objs = Kin(0).objects()
            objs["lancet"] = st(inside, q0)
            self.assertTrue(M.held_by((tcp, GRIP_READ), inside, q0))
            self.assertEqual(M.stage(objs, m), 2, off)
        # closed tight on nothing, beside the lancet: not let go either; open, it is
        self.assertFalse(M.let_go((g + [0.0, 0.0, -0.030], -0.001), inside, q0))
        self.assertTrue(M.let_go((g, M.OPEN_F), inside, q0))
        self.assertTrue(M.let_go((g + [0.0, 0.0, 0.10], -0.001), inside, q0))


class Scoring(unittest.TestCase):
    def test_model_must_step(self):
        M.reset_memory(0)
        M._CLOCK["steps"] = 10
        self.assertIn("not stepping", M.failure(Kin(0).objects(), M._M))

    def test_base_dragged_latches(self):
        M.reset_memory(0)
        o = Kin(0).objects()
        o["hand"] = st(o["hand"].pos + [M.BASE_MOVED + 0.001, 0, 0])
        self.assertIn("off its layout pose", M.failure(o, M._M))

    def test_base_lifted_turned_or_tipped_latches(self):
        """The base's z, its turn and its tilt are checked as well as its x, y (the fixed base placed by seed)."""
        yaw = M.layout(0)["hand"][3]
        for dz, R, why in ((M.BASE_MOVED + 0.001, rz(yaw), "off its layout pose"),
                           (0.0, rz(yaw + M.BASE_TURN + 0.01), "off its layout pose"),
                           (0.0, rz(yaw) @ ry(M.BASE_TURN + 0.01), "off its layout pose"), (0.0, rz(yaw + 0.01), None)):
            M.reset_memory(0)
            o = Kin(0).objects()
            o["hand"] = st(o["hand"].pos + [0, 0, dz], q_from_R(R))
            got = M.failure(o, M._M)
            if why is None:
                self.assertIsNone(got)
            else:
                self.assertIn(why, got or "")

    def test_ladder_along_the_demo(self):
        M.reset_memory(trial_seed(2))
        right, left = M.plan(0.0, trial_seed(2))
        k = Kin(trial_seed(2))
        k.run(right, left)
        seen = []
        M.reset_memory(trial_seed(2))
        k = Kin(trial_seed(2))
        k.hook = lambda: seen.append(M._MEM["now"]["stage"])
        k.run(right, left)
        order = [s for i, s in enumerate(seen) if i == 0 or s != seen[i - 1]]
        self.assertEqual(order, [0, 1, 2, 3])
        self.assertEqual((M.LADDER, M.score.__doc__.count("0.75")), ({0: 0.0, 1: 0.25, 2: 0.5}, 1))


class Demo(unittest.TestCase):
    def test_demo_succeeds_on_every_film_seed(self):
        for cli in CLI:
            sc, k = replay(trial_seed(cli))
            self.assertEqual(sc, 1.0, f"--seed {cli}: {M._MEM['fail']} stage {M._MEM['now']['stage']}")
            self.assertTrue(M._M["fired"])
            self.assertLess(M._M["fire"]["slide"], M.SLIDE_MAX)

    def test_demo_succeeds_at_every_layout_corner(self):
        for c in M.corners():
            sc, _ = replay(c)
            self.assertEqual(sc, 1.0, f"corner {c}: {M._MEM['fail']} stage {M._MEM['now']['stage']}")

    def test_demo_succeeds_on_many_seeds(self):
        for cli in list(range(N_SEEDS)) + list(SIM_SEEDS):
            sc, _ = replay(trial_seed(cli))
            self.assertEqual(sc, 1.0, f"--seed {cli}: {M._MEM['fail']} stage {M._MEM['now']['stage']}")

    def test_layout_is_continuous_and_fills_its_box(self):
        """Each seed draws its own layout: inside the bounds, all distinct, every octant of the box reached, each axis
        spread to within 5% of its bounds, and the same seed always gives the same layout."""
        lays = [M.shift_turn(trial_seed(c)) for c in range(2000)]
        lim = (M.SHIFT_MAX, M.SHIFT_MAX, M.TURN_MAX)
        for lay in lays:
            for v, b in zip(lay, lim):
                self.assertLessEqual(abs(v), b)
        self.assertEqual(len(set(lays)), len(lays))
        self.assertEqual(len({tuple(v > 0 for v in lay) for lay in lays}), 8)
        for i, b in enumerate(lim):
            self.assertGreater(max(lay[i] for lay in lays), 0.95 * b)
            self.assertLess(min(lay[i] for lay in lays), -0.95 * b)
        self.assertEqual(M.shift_turn(trial_seed(7)), M.shift_turn(trial_seed(7)))
        self.assertEqual(M.shift_turn(None), (0.0, 0.0, 0.0))

    def test_turn_is_about_the_palm(self):
        """The turn leaves the point TURN_C behind the apex where the shift alone puts it."""
        for c in M.corners():
            hx, hy, _, yaw = M.layout(c)["hand"]
            p = np.array([hx, hy]) + rz(yaw)[:2, :2] @ np.array([M.TURN_C, 0.0])
            u = M.TURN_C * np.array([math.cos(M.HAND_YAW), math.sin(M.HAND_YAW)])
            np.testing.assert_allclose(p, np.array(M.HAND0) + np.array(c[:2]) + u, atol=1e-12)

    def test_open_loop_plan_fails_other_layouts(self):
        """A plan made for one seed's layout, replayed on another seed's, never reaches 1.0 (README Seeds)."""
        for a in range(N_PAIRS):
            b = a + N_PAIRS
            sc, _ = replay(trial_seed(a), plan_seed=trial_seed(b))
            self.assertLess(sc, 1.0, f"plan of --seed {b} replayed on --seed {a}")
        for c in M.corners():
            sc, _ = replay(c, plan_seed=(0.0, 0.0, 0.0))   # WHY a tuple: plan_seed=None means "the trial's own seed"
            self.assertLess(sc, 1.0, f"the unshifted plan replayed on corner {c}")

    def test_hand_clear_of_the_stand_and_the_container(self):
        """At every corner the trainer (its board, post, hand and fingers) keeps 20 mm from the stand and the sharps
        container."""
        shapes = M.support_shapes() + M.HAND_SHAPES + M.RING_SHAPES
        for c in M.corners():
            hx, hy, _, yaw = M.layout(c)["hand"]
            pts = [np.array([hx, hy]) + rz(yaw)[:2, :2] @ np.array([x, y]) for sh in shapes
                   for x in (sh.bounds()[0][0], sh.bounds()[1][0]) for y in (sh.bounds()[0][1], sh.bounds()[1][1])]
            for (cx, cy), (hxx, hyy) in ((M.STAND, (M.STAND_HALF, M.STAND_HALF)),
                                         (M.SHARPS, (M.SH_L / 2, M.SH_W / 2))):
                # the trainer's bounding corners all lie beyond one of the fixture's faces (x or y), by 20 mm
                gaps = [(cx - hxx) - max(p[0] for p in pts), min(p[0] for p in pts) - (cx + hxx),
                        (cy - hyy) - max(p[1] for p in pts), min(p[1] for p in pts) - (cy + hyy)]
                self.assertGreater(max(gaps), 0.020, f"corner {c}")

    def test_board_on_the_table(self):
        """At every corner of the layout box the base board's corners lie 14 mm or more inside the table's rear edge,
        where the C-clamps hold it, and 13 mm or more clear of the top camera stand's foot (yam_arena's station)."""
        foot = station.camera_stand(station.top_camera("calibrated"))["stand_foot"]
        (fx, fy, _), half = foot[2], foot[1][0] / 2
        edge = station.TABLE_CENTER[0] - station.TABLE_SIZE[0] / 2
        for c in M.corners():
            hx, hy, _, yaw = M.layout(c)["hand"]
            for (x0, x1), (y0, y1) in M.BOARD:
                for x in (x0, x1):
                    for y in (y0, y1):
                        p = np.array([hx, hy]) + rz(yaw)[:2, :2] @ np.array([x, y])
                        self.assertGreater(p[0] - edge, 0.014, c)
                        self.assertGreater(max(abs(p[0] - fx), abs(p[1] - fy)) - half, 0.013, c)

    def test_demo_fits_the_run(self):
        r, lf = M.plan(0.05, 0)
        self.assertLess(max(sum(w.duration for w in r), sum(w.duration for w in lf)) + 2.0, M.RUN_LIMIT_STEPS / 20)


def _no_left(right, left):
    park, rot = M.park_pose(LazyRoutes.S)
    return right, [M._w(sum(w.duration for w in right), park, M.OPEN_F, "stay parked", rot=rot)]


def _no_twist(right, left):
    return [dataclasses.replace(w, yaw=0.0 if w.yaw == M.TWIST else w.yaw) for w in right], left


def _move_press(seed, s=M.SITE_AIM, turn=M.SITE_TURN, finger="ring"):
    """The right's press moved, square, to another point: s back from a finger's apex, turn round its pad."""
    site, a, _ = M.press_pose(seed)
    touch = site - M.tcp_over_lancet() * a
    v, u0, _, rd, _ = M.FINGERS[finger]
    out = np.array([0.0, math.sin(turn), math.cos(turn)])
    site2 = M.hand_world(seed, np.array([u0 + s, v, 0.0]) + rd * out)
    a2 = -M.hand_world(seed, out, direction=True)
    R2 = M.rot_from(a2, np.cross(M.hand_world(seed, (1.0, 0.0, 0.0), direction=True), a2))
    touch2 = site2 - M.tcp_over_lancet() * a2

    def edit(right, left):
        return [dataclasses.replace(w, pos=tuple(float(x) for x in touch2 + float((np.asarray(w.pos) - touch) @ a)
                                                 * a2), rot=_rows(R2)) if w.label in PRESS_LABELS else w
                for w in right], left
    return edit


def _drop_on_table(right, left):
    return [dataclasses.replace(w, pos=(0.25, -0.08, w.pos[2])) if w.label in
            ("over the sharps container", "steady over the opening", "let go", "up clear") else w for w in right], left


def _keep_holding(right, left):
    return [dataclasses.replace(w, finger=M.GRIP_L) if w.label in ("let go", "up clear") else w for w in right], left


class LazyRoutes(unittest.TestCase):
    """Each shortcut a policy might try, and the score it gets (the README's Lazy routes table)."""
    S = trial_seed(2)

    def _run(self, edit):
        sc, _ = replay(self.S, edit=edit)
        return sc, M._MEM["fail"]

    def test_no_holding_hand(self):
        sc, why = self._run(_no_left)
        self.assertEqual(sc, 0.0)
        self.assertIn("not held still", why)

    def test_pull_without_twisting(self):
        sc, why = self._run(_no_twist)
        self.assertEqual(sc, 0.0)
        self.assertIn("tab still on", why)

    def test_press_on_the_index_fingertip(self):
        """The same press moved, square, onto the index fingertip's side (fixed to the base, so it does not give): 0."""
        sc, why = self._run(_move_press(self.S, finger="index"))
        self.assertEqual(sc, 0.0)
        self.assertIn("not the side of the ring fingertip", why)

    def test_press_on_the_pad_middle(self):
        """A press square to the pad's middle line: off the side."""
        sc, why = self._run(_move_press(self.S, turn=0.0))
        self.assertEqual(sc, 0.0)
        self.assertIn("middle of the ring's pad", why)

    def test_press_past_the_pad(self):
        sc, why = self._run(_move_press(self.S, s=M.SITE_AIM + 0.030))
        self.assertEqual(sc, 0.0)
        self.assertIn("not the fingertip", why)

    def test_drop_on_the_table(self):
        sc, why = self._run(_drop_on_table)
        self.assertEqual(sc, 0.0)
        self.assertIn("dropped", why)

    def test_keep_holding_over_the_container(self):
        sc, why = self._run(_keep_holding)
        self.assertEqual(sc, 0.5)
        self.assertIsNone(why)

    def test_sim_stems(self):
        """sim/stems.py, appended by sim/mk.sh to the stem copies (README Sim results), replayed offline on --seed 0's
        layout: each scores what the README expects, for the reason it gives."""
        src = (HERE / "sim" / "stems.py").read_text()
        want = {"a": (0.0, ("not held",)), "g": (0.0, ("not held still",)), "y": (0.0, ("not held still",)),
                "o": (0.0, ("not held still",)), "j": (0.0, ("not held still",)),
                "x": (0.0, ("not held still", "no gripper holding the ring finger")),
                "w": (0.0, ("not held still", "slid")), "k": (0.5, (None,)), "c": (1.0, (None,))}
        # WHY no f00: in PhysX its press in the stand never fires (the shield's face sinks into the tab cap and the
        # housing's front meets the cap with the shield 1.05 mm in, 5.2 N; README Sim results); this replay keeps the
        # face on the cap and would fire it. The rule itself is test_fired_in_the_stand_latches.
        seed = trial_seed(0)
        for stem, (score, why) in want.items():
            ns = dict(vars(M))
            ns["run_seed"] = lambda: seed
            exec(compile(src.replace('_STEM = "none"', f'_STEM = "{stem}"'), "stems.py", "exec"), ns)
            d = ns["demo"]()
            g, ah, rh = M.hold_pose(seed)
            if stem in "ojxw":
                park = tuple(float(x) for x in M.park_pose(seed)[0])
                self.assertTrue(all(w.pos == park for w in d["left"]))
            press = [w for w in d["right"] if w.label == "press until it fires"][0]
            tip = [w for w in d["right"] if w.label == "tip on the site"][0]
            a = M.press_pose(seed)[1]
            if stem == "j":
                self.assertEqual(press.duration, ns["_JAB_S"])
            if stem in "xay":   # 30 mm (60 mm for y00) past contact along the lancet's axis in one control step
                self.assertEqual(press.duration, DT)
                self.assertAlmostEqual(float((np.asarray(press.pos) - tip.pos) @ a), 0.060 if stem == "y" else 0.030,
                                       places=9)
            if stem == "a":   # closed to 5 mm on air above the finger: not held by the rule
                hold = [w for w in d["left"] if w.label == "keep holding through the press"][0]
                np.testing.assert_allclose(np.asarray(hold.pos) - g, ns["_AIR_ACROSS"] * rh[:, 0], atol=1e-9)
                self.assertAlmostEqual(-hold.finger, 0.005)
                hp, hq = ring_link(seed)
                self.assertFalse(M.finger_held((np.asarray(hold.pos), hold.finger, q_from_R(rh)), hp, hq))
            if stem == "g":
                hold = [w for w in d["left"] if w.label == "keep holding through the press"][0]
                np.testing.assert_allclose(hold.pos, g, atol=1e-9)
            if stem == "c":
                self.assertTrue(any(w.yaw == -M.TWIST for w in d["right"]))
            M.reset_memory(seed)
            k = Kin(seed)
            got = k.run(d["right"], d["left"])
            self.assertEqual(got, score, f"stem {stem}: {M._MEM['fail']}")
            fail = M._MEM["fail"]
            self.assertTrue(any((w is None and fail is None) or (w and fail and w in fail) for w in why),
                            f"stem {stem}: {fail}")
            if stem == "w":   # pressed 8.5 mm back, on the site, driven toward the knuckle
                f = [w for w in d["right"] if w.label == "press until it fires"][0]
                t = [w for w in d["right"] if w.label == "tip on the site"][0]
                axis = M.hand_world(seed, (1.0, 0.0, 0.0), direction=True)
                along = float((np.asarray(f.pos) - np.asarray(t.pos)) @ axis)
                demo = M.plan(0.0, seed)[0]
                df = [w for w in demo if w.label == "press until it fires"][0]
                dt_ = [w for w in demo if w.label == "tip on the site"][0]
                comp = float((np.asarray(df.pos) - np.asarray(dt_.pos)) @ axis)   # the demo's own terms, kept
                self.assertAlmostEqual(along, M.PRESS_DZ * math.tan(ns["_TILT"]) + comp, places=6)
            if stem == "c":
                self.assertLess(M._M["tab"], -M.TAB_BREAK)
            if stem == "k":
                self.assertTrue(M.in_sharps(k.l))
                self.assertTrue(M.held_by(M._M["right"], k.l, k._lancet_quat()))

    def test_do_nothing(self):
        M.reset_memory(self.S)
        k = Kin(self.S)
        park, rot = M.park_pose(self.S)
        self.assertEqual(k.run([M._w(5.0, (0.323, -0.305, 1.008), M.OPEN_F, "idle", 0.0)],
                               [M._w(5.0, park, M.OPEN_F, "idle", rot=rot)]), 0.0)


class Geometry(unittest.TestCase):
    def test_hand_clear_of_the_table(self):
        lo = min(s.bounds()[0][2] for s in M.HAND_SHAPES)
        self.assertGreater(M.HZ + lo, 0.010)

    def test_palm_up_fingertips_lowered(self):
        """Palm up (its normal 20 deg off vertical), the fingertips lower than the knuckles by -PITCH (BD step 4), the fingers
        pointing away from the arm bases (+x)."""
        self.assertAlmostEqual(math.degrees(math.acos(M.NORMAL[2])), abs(math.degrees(M.PITCH)), places=9)
        self.assertLess(M.PITCH, 0.0)
        self.assertAlmostEqual(float(M.AXIS[2]), -math.sin(M.PITCH), places=12)   # apex to knuckle runs up
        self.assertAlmostEqual(float(M.hand_world(None, (1.0, 0.0, 0.0), direction=True)[0]), -math.cos(M.PITCH),
                               places=9)
        self.assertGreater(M.site_of(np.zeros(3) + 0.002 * M.NORMAL)[4], 0.0)   # the apex is on the axis, not skin

    def test_pads_clear_the_socket_rim(self):
        self.assertGreater(M.PAD_TIP_L, M.CUP_D + 0.005)

    def test_slide_parts_fit(self):
        """The web stands behind the knuckle and beyond the holding pads' reach along the finger (16 mm either side of
        the hold); the rail is on the post's front face; the foot lands on the stop post after DROP_STOP; the block
        stays on the rail over the travel; the ring finger's base end clears the rail; and a 15 N press at the demo's
        site puts less moment on the block than its pitch rating (MP)."""
        self.assertGreater(M.WEB_X[0], M.KNUCKLE_S * math.cos(M.PITCH))
        self.assertLessEqual((M.HOLD_AT + 0.016) * math.cos(M.PITCH), M.WEB_X[0])
        self.assertAlmostEqual(M.POST_X[0] - (M.WEB_X[1] + M.RAIL["height"]), 0.0, places=12)
        self.assertAlmostEqual(M.FOOT_Z[0] - M.GUIDE_TOP, M.DROP_STOP, places=12)
        self.assertGreaterEqual(M.BLOCK_Z[0] - M.DROP_STOP, M.RAIL_Z[0])
        self.assertLessEqual(M.BLOCK_Z[1], M.RAIL_Z[1])
        rp = M.FINGERS["ring"][4]
        self.assertGreater(M.POST_X[0] - M.RAIL["rail_h"] - (M.RING_END_S * math.cos(M.PITCH) + rp * abs(math.sin(M.PITCH)) / math.cos(math.pi / 16)),
                           0.0003)
        site = M.press_pose(None)[0] - np.array(M.layout(None)["hand"][:3])
        block = np.array([-(M.WEB_X[1] + M.RAIL["height"] / 2), 0.0, (M.BLOCK_Z[0] + M.BLOCK_Z[1]) / 2])  # world, yaw 180
        self.assertLessEqual(15.0 * float(np.linalg.norm(block - site)), M.RAIL["MP"])
        self.assertTrue(M.HOLD_SEG[0] < M.HOLD_AT < M.HOLD_SEG[1])

    def test_shield_reaches_skin_before_housing(self):
        """At the fire travel the housing's front still stands off the skin (the shield takes the whole press)."""
        q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD
        self.assertGreater(M.SHIELD_OUT - q_fire, 0.001)

    def test_lancet_fits_the_opening_and_clears_the_lid(self):
        self.assertLess(math.hypot(M.L_W, M.L_T), M.SH_OPEN[1] + 0.001)
        face = M.RELEASE_Z - M.tcp_over_lancet()
        self.assertGreater(face, M.TOP + M.SH_H + 0.005)

    def test_housing_grip_friction(self):
        """The housing the right holds has yam_arena's grasped-object friction: on the 0.5/0.4 plastic its grip slid at
        5 to 6 N in PhysX, under the 8 N fire (f00, c00, t00 corners 126748 and 84277) [RUN]."""
        housing = M.lancet_parts()[0].shapes[0]
        self.assertEqual(housing.material, "housing")
        self.assertEqual(M.MATERIALS["housing"], (1.2, 1.0))

    def test_forces_under_the_finger_budget(self):
        """Every working force at or under 40 N a YAM finger, with margin (G16)."""
        grip = 5000.0 * (M.L_T / 2 + M.GRIP_L)  # N a finger: 4 mm into the housing at 5000 N/m
        self.assertLessEqual(M.F_FIRE_MAX_REAL * 2.5, 40.0)
        twist = M.T_TAB_MAX_REAL / M.L_W
        self.assertLessEqual((grip + twist) * 1.6, 40.0)
        self.assertLessEqual(M.F_FIRE / (2 * 0.34 * grip), 0.65)
        hold = 5000.0 * (M.FINGERS["ring"][4] + M.GRIP_R)   # the left's 5 mm into the ring finger
        self.assertLessEqual(hold * 1.5, 40.0)


PIN_D_TXT = "4"


class ReadmeNumbers(unittest.TestCase):
    """The README's numbers come from the code."""
    TXT = " ".join((HERE / "README.md").read_text().split())

    def test_seed_table(self):
        """Every row of the README's seed table is the layout the code draws for that --seed."""
        rows = [ln for ln in (HERE / "README.md").read_text().splitlines() if ln.startswith("| ") and ln.count("|") == 5
                and ln.split("|")[1].strip().isdigit()]
        self.assertGreaterEqual(len(rows), len(CLI) + 8)
        for ln in rows:
            cli = int(ln.split("|")[1])
            dx, dy, yaw = M.shift_turn(trial_seed(cli))
            self.assertEqual(ln.split("|")[2:4], [f" {1000 * dx:.1f}, {1000 * dy:.1f} ", f" {math.degrees(yaw):.2f} "],
                             ln)
        for cli in CLI:
            self.assertTrue(any(ln.startswith(f"| {cli} |") for ln in rows))

    def test_model_numbers(self):
        t = self.TXT
        q_fire = (M.F_FIRE - M.F_PRE) / M.K_SHIELD
        rv, ru0, ru1, rd, rp = M.FINGERS["ring"]
        drop_v = (-M.RING_PRELOAD + math.sqrt(M.RING_PRELOAD ** 2 + M.K_RING * M.RING_KG)) / M.K_RING
        for s in (f"fires at {M.F_FIRE:g} N on the shield, {1000 * q_fire:.2f} mm of travel",
                  f"breaks at {math.degrees(M.TAB_BREAK):.0f} deg of turn either way",
                  f"up to {math.degrees(M.TAB_STOP):.0f} deg either way", f"{100 * M.T_TAB:g} N-cm",
                  f"{1000 * M.SITE_S[0]:.0f} to {1000 * M.SITE_S[1]:.0f} mm back from the apex and "
                  f"{math.degrees(M.SITE_ANG[0]):.0f} to {math.degrees(M.SITE_ANG[1]):.0f} deg round from the pad's "
                  "middle line, on either side",
                  f"within {math.degrees(M.TILT_MAX):.0f} deg of the skin's inward normal",
                  f"more than {1000 * M.SLIDE_MAX:.1f} mm on the skin is a slide",
                  f"until {M.STEADY_AFTER:g} s after the click", f"loose for {M.DROP_S:g} s",
                  f"ring finger ({1000 * M.RING_KG:.0f} g:", f"A spring ({M.K_RING:g} N/m)",
                  f"with {M.RING_PRELOAD:g} N to spare over its weight; it stops {1000 * M.DROP_STOP:.0f} mm down",
                  f"gives {1000 * M.DROP_MAX:.0f} mm at {YS.give_force():.2f} N straight down",
                  f"half-gap {1000 * M.HOLD_GAP[0]:.2f} to {1000 * M.HOLD_GAP[1]:.2f} mm and its TCP within "
                  f"{1000 * M.HOLD_R:.0f} mm of the ring finger's axis between its distal joint "
                  f"({1000 * M.HOLD_SEG[0]:.1f} mm back from the apex) and its base ({1000 * M.HOLD_SEG[1]:.1f} mm back)",
                  f"at most {1000 * M.HOLD_ACROSS:.0f} mm off the axis along the gripper's closing direction",
                  f"pad tips ({1000 * M.TIP:.0f} mm past the TCP along its approach) past the axis",
                  f"from 1.5 mm under to {1000 * (M.HOLD_GAP[1] - M.HOLD_GAP_ON):.2f} mm over {1000 * M.HOLD_GAP_ON:.2f} mm",
                  f"within {1000 * M.TOUCH_D:.1f} mm of the ring finger's skin",
                  f"a fingertip mark {1000 * M.TIP_MARK:.1f} mm back from the apex",
                  f"may turn or tip {math.degrees(M.BASE_TURN):.0f} deg",
                  f"{1000 * M.BASE_MOVED:.0f} mm off its layout pose; more is a fault",
                  f"stops 25 g only after {1000 * drop_v:.1f} mm",
                  f"{1000 * M.HOLD_AT:.1f} mm back from its apex, {1000 * (M.HOLD_AT - M.SITE_AIM):.1f} mm behind the press",
                  f"its approach {math.degrees(M.HOLD_TILT):.0f} deg below the palm's plane",
                  f"its pad tips {1000 * (M.TIP - M.HOLD_BACK):.0f} mm past the finger's axis",
                  f"aims {1000 * M.HOLD_UP:.0f} mm over the finger",
                  f"pads {1000 * rp:.1f} to 14.2 mm either side",
                  f"{1000 * M.SITE_AIM:g} mm back from the ring's apex, {math.degrees(M.SITE_TURN):.0f} deg round on "
                  "the thumb's side",
                  f"sloping down {abs(math.degrees(M.PITCH)):.0f} deg", f"ring's apex {1000 * M.HZ:.0f} mm over the table",
                  f"its web {1000 * M.WEB_X[0]:.0f} mm behind the apex",
                  f"within {1000 * M.HELD_R:.0f} mm of the lancet's axis, anywhere from {-1000 * M.HELD_AX[0]:.0f} mm "
                  f"beyond its face to {1000 * M.HELD_AX[1]:.0f} mm up it",
                  f"half-gap over {1000 * M.HELD_GAP[1]:.1f} mm, or its TCP more than {1000 * M.RELEASE_R:.0f} mm",
                  f"half-gap {1000 * M.HELD_GAP[0]:.1f} to {1000 * M.HELD_GAP[1]:.1f} mm",
                  f"{M.RUN_LIMIT_STEPS:,} steps",
                  f"more than {1000 * M.OUT_XY:.0f} mm across or {1000 * M.LIFT_OUT:.0f} mm up",
                  f"each uniform within +-{1000 * M.SHIFT_MAX:.0f} mm",
                  f"turned uniform within +-{math.degrees(M.TURN_MAX):.0f} deg about a point {1000 * M.TURN_C:.0f} mm",
                  f"({M.HAND0[0]:g}, {M.HAND0[1]:g})", f"waits clear at ({M.WAIT_R[0]:.2f}, {M.WAIT_R[1]:.2f})",
                  f"{abs(math.degrees(M.HAND_YAW)):.0f} deg right of +x"):
            self.assertIn(s, t)
        self.assertAlmostEqual(rp, 0.0078)
        # the demo's press angle and the site's normals' lean range, as the README gives them
        n_lo = math.degrees(math.acos(math.cos(M.PITCH) * math.cos(M.SITE_ANG[0])))
        n_hi = math.degrees(math.acos(math.cos(M.PITCH) * math.cos(M.SITE_ANG[1])))
        self.assertIn(f"normals lean {n_lo:.0f} to {n_hi:.0f} deg from vertical", t)
        demo = math.degrees(math.acos(math.cos(M.PITCH) * math.cos(M.SITE_TURN)))
        self.assertIn(f"comes in {demo:.0f} deg from vertical", t)

    def test_run_numbers(self):
        """The plans' lengths the README gives, and the run limit's headroom over the slowest (a stem)."""
        r, lf = M.plan(0.0, trial_seed(0))
        a, b = sum(w.duration for w in r), sum(w.duration for w in lf)
        self.assertIn(f"take {a:.1f} s (right) and {b:.1f} s (left)", self.TXT)
        ik = _tool("ikcheck")
        slow = max((max(sum(w.duration for w in x) for x in ik.stem_plan(k, trial_seed(0))), k) for k in ik.STEMS)
        self.assertIn(f"the slowest stem, {slow[1]}00, {slow[0]:.1f} s", self.TXT)
        self.assertGreater(M.RUN_LIMIT_STEPS / 20, slow[0] + 15.0)
        for f in ("film.sh", "dream.sh"):
            self.assertIn(f"--max-steps {M.RUN_LIMIT_STEPS}", (HERE / "sim" / f).read_text())
        self.assertIn(f"--max-steps {M.RUN_LIMIT_STEPS}", self.TXT)

    def test_yield_table(self):
        """The README's yield table and the sweep's other numbers are the sweep's."""
        t = self.TXT
        g = YS.give_force()
        for mu, txt in zip(YS.MUS, ("0", "0.25", "0.5", "1.0")):
            k, w = YS.sweep(mu)
            at = f"{g / k:.2f}" if k > 0 else "never (pushed up)"
            self.assertIn(f"| {txt} | {k:.4f} | {at} | {M.F_FIRE * k - g:+.2f} N |", t)
        k, w = YS.sweep(0.0)
        self.assertEqual(abs(w[1]), 60.0)
        self.assertIn(f"The worst site is {w[0]:.1f} mm back, 60 deg round, on either side", t)
        self.assertIn(f"(by {M.F_FIRE * k - g:.2f} N or more)", t)
        self.assertIn("the held rule is what makes every one-handed fire score 0", t)

    def test_real_check_numbers(self):
        """Check R's numbers come from the sweep (no friction: the only case its guarantee covers)."""
        t = self.TXT
        g = YS.give_force()
        k, _ = YS.sweep(0.0)
        self.assertIn(f"1.0 to 1.4 N (the sim's {g:.2f} N plus or minus 0.2 N)", t)
        self.assertIn(f"by {M.F_FIRE * k - 1.4:.3f} N at 8 N ({k:.5f} x 8 - 1.4)", t)
        self.assertEqual(f"{M.F_FIRE * round(k, 5) - 1.4:.3f}", f"{M.F_FIRE * k - 1.4:.3f}")   # one number, either way
        self.assertNotIn("1.09 N", t)
        self.assertNotIn("1.08 N", t)
        self.assertNotIn("kitchen scale", t)   # review F2: no scale under the trainer; the touch is read on camera

    def test_rig_numbers(self):
        """The rig's sizes under Real trials are rig_scad.numbers(), which come from the model."""
        t = self.TXT
        n = _tool("rig_scad").numbers()
        g = lambda v: f"{v:g}"  # noqa: E731
        wx0, wx1, ww, _, _ = n["web"]
        sp, pin = n["spring"], n["pin"]
        bx, by, bz = n["base_size"]
        px0, px1, py0, py1, pb, ptop = n["post"]
        sx0, sx1, shy, sb, stp = n["pillar"]
        (a0, a1, ay0, ay1), (b0, b1, by0, by1) = n["board"]
        rl = M.RAIL
        for s in (f"sloping down {g(abs(n['pitch_deg']))} deg toward their tips, the ring's apex {g(n['apex_z'])} mm",
                  f"a plate x {g(a0)} to {g(a1)}, y {g(ay0)} to {g(ay1)}, under the hand and a bar x {g(b0)} to "
                  f"{g(b1)}, y {g(by0)} to {g(by1)},", f"over the bar at x {g(n['clamp_x'])}",
                  f"{bx} by {by} by {bz} mm, so it needs a bed of at least {bx + 10} by {by + 10} mm and {bz + 5} mm",
                  f"The post: x {g(px0)} to {g(px1)}, {g(py1 - py0)} mm across, z {g(pb)} to {g(ptop)}",
                  "at z " + ", ".join(g(v) for v in n["rail_holes_z"][:-1]) + f" and {g(n['rail_holes_z'][-1])}",
                  f"a pillar x {g(sx0)} to {g(sx1)}, {g(2 * shy)} mm across, z {g(sb)} to {g(stp)}",
                  f"its top at z {g(n['stop_top'])}", f"a web x {g(wx0)} to {g(wx1)}", f"{g(ww)} mm wide above",
                  f"foot z {g(n['foot'][0])} to {g(n['foot'][1])}",
                  f"at z {g(n['block_holes_z'][0])} and {g(n['block_holes_z'][1])}",
                  f"clears the rail's face by {n['finger_clear']:.1f} mm", rl["part"].split()[1],
                  f"rail {1000 * rl['rail_len']:g} mm long, {1000 * rl['rail_w']:g} wide, {1000 * rl['rail_h']:g} high",
                  f"block {1000 * rl['block_l']:g} long, {1000 * rl['block_w']:g} wide, {1000 * rl['height']:g} mm high",
                  f"MR {rl['MR']:g}, MP {rl['MP']:g}, MY {rl['MY']:g} N m",
                  f"The rail runs z {g(n['rail'][2])} to {g(n['rail'][3])}",
                  f"the block, z {g(n['block'][0])} to {g(n['block'][1])} at rest",
                  f"{g(n['lever'])} mm from the block's middle", f"at most {n['moment_15N']:.2f} N m of moment",
                  f"two {PIN_D_TXT} x {g(pin['length'])} mm steel dowel pins pressed 10 mm into the pillar's top at x "
                  f"{g(n['pins'][0])}", f"z {g(pin['bottom'])} to {g(pin['top'])}",
                  f"z {g(n['collar'][0])} to {g(n['collar'][1])}: the top stop",
                  f"music wire {g(sp['wire'])} mm", f"(OD {g(sp['od'])}, ID {g(sp['id'])})",
                  f"{g(sp['active'])} active coils and {g(sp['total'])} in all",
                  f"{g(sp['rate_each'])} N/mm, free length {sp['free']:.1f} mm, {g(sp['rest'])} mm long at rest "
                  f"({sp['force_rest_each']:.3f} N each, {2 * sp['force_rest_each']:.2f} N in all",
                  f"{g(sp['bottom'])} mm at the lower stop, solid {g(sp['solid'])} mm, peak stress {sp['tau']} MPa",
                  f"spring index is {sp['index']:.0f}", f"sit at z {g(n['seat_top'])}",
                  f"{g(n['cup_pocket'][0])} by {g(n['cup_pocket'][1])}, {g(n['cup_pocket'][2])} deep",
                  f"stops it at +-{g(n['tab_stop_deg'])} deg",
                  f"{n['hold_web_e']:.1f} mm ahead of the web's middle", f"{n['hold_web_MPa_15N']:.1f} MPa",
                  f"{n['hold_moment_15N']:.2f} N m on the guide's block"):
            self.assertIn(s, t)
        self.assertAlmostEqual(2 * sp["force_rest_each"], M.RING_PRELOAD + M.RING_KG * M.GRAVITY, places=2)
        self.assertGreater(sp["bottom"], sp["solid"])                # the springs never go solid
        self.assertLessEqual(n["moment_15N"], M.RAIL["MP"])
        self.assertLessEqual(n["hold_moment_15N"], M.RAIL["MP"] / 4)    # check H's 15 N hold bound
        self.assertLessEqual(n["hold_web_MPa_15N"], 10.0)
        self.assertGreater(n["finger_clear"], 1.0)
        self.assertGreaterEqual(pin["top"], n["collar"][1])           # the collars sit on the pins
        self.assertLess(pin["bottom"], stp)                           # and the pins sit in the pillar
        self.assertAlmostEqual(n["foot"][0] - n["stop_top"], 1000 * M.DROP_STOP, places=6)
        self.assertAlmostEqual(n["foot"][0] - n["seat_top"], 1000 * M.SPRING_REST, places=3)

    def test_instruction_is_outcomes_not_choreography(self):
        """Under 1000 characters, and no gripper assignment or grasp (the #826 rejection)."""
        ins = " ".join(M.__doc__.split())
        self.assertLess(len(ins), 1000)
        for w in ("other hand", "left", "right gripper", "your hand", "grip "):
            self.assertNotIn(w, ins.lower())
        self.assertEqual(json.loads((HERE / "task.json").read_text())["instruction"], ins)


def real_square_tilt(theta, lean_apex, lean_thumb):
    """README Real trials, Square: the lancet's angle off the skin's normal (deg) from what the trial reads. theta (deg)
    the ink patch's angle round from the pad's middle line on the template, signed toward the thumb; lean_apex and
    lean_thumb the housing top's lean (deg, signed) toward the fingertip, seen by the camera looking across the
    fingers, and toward the thumb, seen by the camera looking along them. Rig frame: x level toward the wrist, y to the
    thumb side, z up; the pad's normal there is n = (sin P cos theta, sin theta, cos P cos theta), P the hand's pitch.
    WHY no along-finger term: every site is 8 mm or more back from the apex, past the pad's radius, where the skin's
    normal has no lean along the finger."""
    p, t = M.PITCH, math.radians(theta)
    n = np.array([math.sin(p) * math.cos(t), math.sin(t), math.cos(p) * math.cos(t)])
    d = np.array([-math.tan(math.radians(lean_apex)), math.tan(math.radians(lean_thumb)), 1.0])
    return math.degrees(math.acos(float(n @ d) / (np.linalg.norm(n) * np.linalg.norm(d))))


def _quat_z_to(d):
    k = np.cross([0.0, 0.0, 1.0], d)
    sn, c = float(np.linalg.norm(k)), float(d[2])
    if sn < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    ang = math.atan2(sn, c)
    return tuple(list(k / sn * math.sin(ang / 2)) + [math.cos(ang / 2)])


class RealTrial(unittest.TestCase):
    def test_real_square_check_matches_the_scorer(self):
        """The README's two-camera squareness reading gives the scorer's own tilt (tilt_of against site_of's normal)
        over the whole site, on either side of the pad, lancets leaning up to 25 deg any way off the normal."""
        rng = np.random.default_rng(3)
        v, u0, _, r, _ = M.FINGERS["ring"]
        for _ in range(400):
            s = float(rng.uniform(*M.SITE_S))
            theta = float(rng.choice((-1.0, 1.0)) * rng.uniform(15.0, 60.0))
            t = math.radians(theta)
            p = M.hand_to_mech((u0 + s, v + r * math.sin(t), r * math.cos(t)))
            site = M.site_of(p)
            self.assertEqual(site[0], "ring")
            self.assertAlmostEqual(site[4], 0.0, places=9)
            n = M.R_HAND @ np.array([0.0, math.sin(t), math.cos(t)])
            e1 = np.cross(n, [1.0, 0.0, 0.0])
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(n, e1)
            tip, az = math.radians(float(rng.uniform(0.0, 25.0))), float(rng.uniform(-math.pi, math.pi))
            d = math.cos(tip) * n + math.sin(tip) * (math.cos(az) * e1 + math.sin(az) * e2)
            code = math.degrees(M.tilt_of(_quat_z_to(d), (0.0, 0.0, 0.0, 1.0), site[3]))
            lean_apex = math.degrees(math.atan2(-d[0], d[2]))      # the apex lies toward -x
            lean_thumb = math.degrees(math.atan2(d[1], d[2]))      # the thumb lies toward +y
            self.assertAlmostEqual(real_square_tilt(theta, lean_apex, lean_thumb), code, places=6)

    def test_real_square_check_both_sides_of_theta(self):
        """theta is signed: at sites near both ends of the band on both sides, with leans of both signs, the reading
        equals the scorer's tilt; a vertical lancet reads acos(cos P cos theta)."""
        v, u0, _, r, _ = M.FINGERS["ring"]
        for theta in (15.5, 59.5, -15.5, -59.5):   # WHY not 15 and 60 exactly: rounding puts a site on the bound
            t = math.radians(theta)
            site = M.site_of(M.hand_to_mech((u0 + 0.011, v + r * math.sin(t), r * math.cos(t))))
            self.assertEqual(M.site_ok(site), (True, ""))
            for la, lt in ((0.0, 0.0), (8.0, -12.0), (-8.0, 12.0), (0.0, 15.0), (0.0, -15.0)):
                d = np.array([-math.tan(math.radians(la)), math.tan(math.radians(lt)), 1.0])
                d /= np.linalg.norm(d)
                code = math.degrees(M.tilt_of(_quat_z_to(d), (0.0, 0.0, 0.0, 1.0), site[3]))
                self.assertAlmostEqual(real_square_tilt(theta, la, lt), code, places=6)
                if la == lt == 0.0:
                    self.assertAlmostEqual(code, math.degrees(math.acos(math.cos(M.PITCH) * math.cos(t))), places=6)

    def test_rig_scad_is_current(self):
        """rig.scad is what tools/rig_scad.py writes from the task's numbers now, and its key sizes are the model's."""
        rs = _tool("rig_scad")
        txt = (HERE / "rig.scad").read_text()
        self.assertEqual(txt, rs.text(), "rerun python3 tools/rig_scad.py")
        for a, b in ("{}", "()", "[]"):
            self.assertEqual(txt.count(a), txt.count(b))
        n = rs.numbers()
        wx0, wx1, ww, _, wt = n["web"]
        fb = n["foot"][0]
        self.assertIn(f"translate([{wx0}, {-ww / 2}, {fb}]) cube([{round(wx1 - wx0, 3)}, {ww}, {round(wt - fb, 3)}])", txt)
        # the cup's arc slot is centred on the stop pin (at 0 deg) and meets it at +-TAB_STOP either way
        half = n["tab_stop_deg"] + n["stop_half_deg"]
        self.assertIn("translate([9.0, 0, ", txt)
        self.assertIn(f"rotate([0, 0, {-round(half, 3):g}]) rotate_extrude(angle = {round(2 * half, 3):g})", txt)
        self.assertEqual(n["drop_stop"], round(1000 * M.DROP_STOP, 3))
        # the seat pillar from the board to the springs' shims, the pins pressed into its top and reaching past the
        # collars, the springs at their model rest length, and the stop post on the pillar at the model's height
        sx0, sx1, sy, bt, stp = n["pillar"]
        self.assertIn(f"translate([{sx0}, {-sy}, {bt}]) cube([{round(sx1 - sx0, 3)}, {round(2 * sy, 3)}, "
                      f"{round(stp - bt, 3)}]);", txt)
        self.assertAlmostEqual(stp, 1000 * (M.HZ + M.PILLAR_TOP), places=3)
        self.assertAlmostEqual(n["foot"][0] - n["seat_top"], 1000 * M.SPRING_REST, places=3)
        self.assertGreaterEqual(n["pin"]["top"], n["collar"][1])
        self.assertLess(n["pin"]["bottom"], stp)
        self.assertAlmostEqual(n["post"][5], round(1000 * (M.HZ + M.POST_TOP), 3), places=3)
        # the shield's face rests on the post at the model's height, and the cup's pocket is the model's socket
        face = round(1000 * (M.STAND_H + M.TAB_H), 3)
        self.assertAlmostEqual(face + 0.5, 1000 * (M.L_START[2] - M.TOP), places=6)
        self.assertIn(f"cylinder(d = {rs.POST_D}, h = {face})", txt)
        self.assertIn(f"cube([{round(1000 * (M.L_W + 2 * M.CUP_CLEAR), 3)}, {round(1000 * (M.L_T + 2 * M.CUP_CLEAR), 3)}",
                      txt)
        self.assertAlmostEqual(half - n["stop_half_deg"], math.degrees(M.TAB_STOP), places=6)   # the slot's ends
        # the springs: two in parallel are the model's K_RING, and at rest they carry the preload plus the weight
        sp = n["spring"]
        self.assertAlmostEqual(2 * sp["rate_each"] * 1000, M.K_RING, places=6)
        self.assertAlmostEqual(sp["comp"], 1000 * M.ring_target(), places=3)
        for s in ("tools/rig_scad.py", "rig.scad", "OpenSCAD is not installed"):
            self.assertIn(s, ReadmeNumbers.TXT)


if __name__ == "__main__":
    unittest.main()
