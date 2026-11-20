"""Scripted full-score demo (and negative variants) for the SO-101 joint-1 task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the bracket's and the top
horn's poses through ``jointkit.bench_state()`` (read-only, like an operator watching the parts); it never writes a
pose, a joint state or a score.

Plan (right arm; the left arm stays at READY):
1. Grip the bracket's upright (the motor-2 holder, 46 mm across) from above, fingers closing across it.
2. Lift it and carry it in front of the servo, square to the base, its two cheeks level with the two horns (the
   lower cheek and horn are out of sight under the servo) and its slot line on the slide direction.
3. Slide it home (5 mm/s) along its slots: every control step the target gripper pose is recomputed from the
   measured bracket pose (closed loop on the bracket, not on the hand), so grasp slip and arm sag are corrected.
4. Turn the seated bracket about the servo axis until its four top holes are over the top horn's holes (the horn
   starts 25 deg off; the error is read from the bracket's and the horn's poses), press the top horn into the cheek's
   lip (the lip arc is centred on the hole pattern, so this centres the holes), take the push off and open the hand
   at once. The hole offset is then read from the poses; if letting go moved the bracket off the holes, it is gripped
   and seated again. Then the hand lifts straight up and looks down at the holes.
Both arms add the remaining joint error back to their commands (integral term on their own joint readings), because
the joint PD sags under load (pattern reused from earlier demos).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'src'))

import layout as L  # noqa: E402

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
FINGER_OPEN = .04695          # m per finger at grip 1.0
GRASP_B = np.array([.0356, 0., .0300])   # TCP target on the bracket's upright, bracket frame
UPRIGHT_HALF_WIDTH = .023               # the upright is 46 mm across (bracket y)
SQUEEZE = .0025                         # fingers commanded 2.5 mm inside the part on each side
COMP_GAIN = .08                         # joint integral gain per control step
# joint stiction: 0.3 N m on 80 N m/rad (shoulder joints) = 3.75 mrad, 0.06 N m on 10 N m/rad (wrist) = 6 mrad
FINE_LIMIT = np.array([.006, .006, .006, .009, .009, .009])
PUSH_LIMIT = np.array([.03, .03, .03, .04, .04, .04])  # 'push': bounded joint integral (about 2.4 N m on a shoulder)
SLOT_COMP = 'hold'        # arm mode in the slots: 'hold' (no integral) or 'push' (bounded integral)


def grip_of(half_opening_m):
    return float(np.clip(half_opening_m / FINGER_OPEN, 0., 1.))


class Stop(Exception):
    pass


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


class Arm:
    """One arm: smooth Cartesian moves (URDF IK), each starting where the last command left it."""

    def __init__(self, name, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY
        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.offset = np.zeros(6)
        self.measured = self.q.copy()
        self.comp = True
        self.base_offset = np.zeros(6)
        self.free_j1 = False
        self.hold_j1 = None  # base joint commanded to this fixed angle (held by its stiffness and stiction)
        self.label = ''
        self.worst_mm = 0.

    @property
    def cmd(self):
        c = self.q + self.offset
        if self.free_j1:
            c[0] = self.measured[0]  # base joint left where it is: no spring, only its own stiction
        elif self.hold_j1 is not None:
            c[0] = self.hold_j1
        return np.r_[c, self.grip]

    def set_comp(self, mode):
        """True: full joint integral (free moves); 'fine': joint integral kept within FINE_LIMIT of the sag offsets
        learned so far, so each joint still gets past its stiction to its command but the hand can never push on
        a part with more than about 1 N; 'hold': offsets frozen (no integral; in contact); False: offsets decay (hand
        released)."""
        if mode in ('fine', 'push') and self.comp not in ('fine', 'push'):
            self.base_offset = self.offset.copy()
        self.comp = mode

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        if self.comp in ('fine', 'push'):
            lim = FINE_LIMIT if self.comp == 'fine' else PUSH_LIMIT
            self.offset = np.clip(self.offset + COMP_GAIN * (self.q - self.measured), self.base_offset - lim,
                                  self.base_offset + lim)
        elif self.comp == 'hold':
            pass  # offsets held as learned (no integral: nothing winds up against a part)
        elif self.comp:
            # slow integral: the YAM joints have 0.3 N m (shoulder) dry friction; a fast integrator hunts around it
            self.offset = np.clip(self.offset + COMP_GAIN * (self.q - self.measured), -.25, .25)
        else:
            self.offset *= .85

    def relax(self):
        """Command the arm to where it is (joint commands = measured angles): releases the spring force it stores
        against a part (a soft wrist flexed by a push) before the fingers open, so opening does not flick the part."""
        from yam_arena import ik
        self.free_j1 = False
        self.hold_j1 = None
        self.set_comp('hold')
        # aim at where the arm is, plus the gravity-sag offsets learned in free motion (captured when the fine-mode
        # joint integral started), so the soft wrist does not drop onto the part when the push is taken off
        self.q = self.measured.copy()
        self.offset = self.base_offset.copy()
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)

    def measured_tcp(self):
        from yam_arena import ik
        return ik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot, iters=80):
        from yam_arena import ik
        from yam_arena.constants import READY
        best = None
        for s in (self.q, np.array(READY, float), np.array([0., 1.2, 1.2, -.4, 0., 0.])):
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), s, base_pos=self.base, iters=iters)
            if best is None or pe + .1 * re < best[1] + .1 * best[2]:
                best = (q, pe, re)
            if pe < 2e-4 and re < 3e-3:
                break
        return best

    def set_pose(self, pos, rot):
        q, pe, re = self.solve(pos, rot)
        if pe > .002 or re > math.radians(3):
            raise Stop(f'{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg')
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label=''):
        from yam_arena import ik
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s))
            yield

    def set_grip(self, grip, duration=.8, label=''):
        yield from self.move(self.pos, self.rot, duration, grip, label)

    def home(self, duration=2.0, label=''):
        from yam_arena import ik
        from yam_arena.constants import READY
        self.label = label or f'{self.name} home'
        q0, q1 = self.q.copy(), np.array(READY, float)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.q = q0 + ik.smoothstep(k / n) * (q1 - q0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            yield


def wait(seconds):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


class Ctx:
    def __init__(self, emb=None):
        from yam_arena import station
        self.emb = emb
        self.left = Arm('left', station.BASES['left'])
        self.right = Arm('right', station.BASES['right'])
        self.log = []
        self.step = 0

    def pose(self, name):
        import jointkit
        return jointkit.bench_state()[name]

    def detail(self):
        import jointkit
        return jointkit.STATE['detail']

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        return Action(data=pack(self.left.cmd, self.right.cmd))

    def note(self, tag, **kw):
        d = self.detail()
        row = dict(at=tag, step=self.step, measure=d.get('measure'), level=d.get('level'), **kw)
        self.log.append(row)
        print('LOOK', json.dumps(row, default=float), flush=True)


def tcp_rot_for(r_b, arm, pos):
    """Straight-down grip, fingers closing along the bracket's y axis (either sign: least joint-6 travel)."""
    best = None
    for sgn in (1., -1.):
        x = sgn * r_b[:, 1]
        x = x - x[2] * np.array([0, 0, 1.])
        x /= np.linalg.norm(x)
        z = np.array([0, 0, -1.])
        r = np.column_stack([x, np.cross(z, x), z])
        q, pe, re = arm.solve(pos, r)
        cost = pe * 100 + re + abs(q[5]) / 2.094
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


def grasp_offset(c):
    """Measured transform bracket -> TCP (in the bracket frame): (p, R) with T_tcp = T_b * G."""
    pb, rb = c.pose('bracket')
    pt, rt = c.right.measured_tcp()
    return rb.T @ (pt - pb), rb.T @ rt


def _rotvec(r):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(r).as_rotvec()


def _rotm(v):
    from scipy.spatial.transform import Rotation
    return Rotation.from_rotvec(v).as_matrix()


def capture_grasp(c):
    """Freeze the bracket -> TCP transform (measured once, with the bracket held still). Re-measuring it every step
    feeds the arm's own dynamics back into the command and oscillates."""
    c.G = grasp_offset(c)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def track(c, target_fn, duration, label, gain=.05, free_axis=None, deadband=0., rot_gain=None):
    """Closed loop on the bracket: each step command the TCP pose that puts the bracket at target_fn(k/n) through
    the frozen grasp transform, plus slow integral corrections of the measured bracket position and orientation
    error (arm sag, grasp slip)."""
    from yam_arena import ik
    arm = c.right
    arm.label = label
    n = max(1, round(duration * CONTROL_HZ))
    gp, gr = c.G
    for k in range(1, n + 1):
        pt_b, rt_b = target_fn(ik.smoothstep(k / n))
        pb, rb = c.pose('bracket')
        err = pt_b - pb
        for ax in ([] if free_axis is None else np.atleast_2d(free_axis)):
            err = err - (err @ ax) * ax  # no integral push along these axes (contact sets the position there)
        # inside the deadband the integral stops: the arm's joints then stick where they are instead of hunting
        err = np.where(np.abs(err) < deadband, 0., err)
        c.corr_p = np.clip(c.corr_p + gain * err, -.008, .008)
        c.corr_r = np.clip(c.corr_r + (gain if rot_gain is None else rot_gain) * _rotvec(rt_b @ rb.T), -.035, .035)
        rc = _rotm(c.corr_r)
        arm.set_pose(pt_b + c.corr_p + rc @ rt_b @ gp, rc @ rt_b @ gr)
        yield


def steered(pose, yaw_rad, roll_rad):
    """Bracket pose turned by yaw_rad about its own z axis through the grasp point (the hand only turns), then by
    roll_rad about its own x axis (the slot axis) through its origin (top cheek)."""
    p, r = pose
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    rz_b = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    g = p + r @ GRASP_B
    r1 = r @ rz_b
    p1 = g - r1 @ GRASP_B
    c, s = math.cos(roll_rad), math.sin(roll_rad)
    return p1, r1 @ np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def bracket_target(yaw_deg, s_m, lift_m=0.):
    """Bracket pose when backed off ``s_m`` along its own +x (toward the robot) from fully home at yaw_deg."""
    p, r = L.bracket_home(yaw_deg)
    return p + r @ np.array([s_m, 0., 0.]) + np.array([0, 0, lift_m]), r


# --------------------------------------------------------------------------------------------------- the script
def pick(c):
    arm = c.right
    pb, rb = c.pose('bracket')
    g = pb + rb @ GRASP_B
    r = tcp_rot_for(rb, arm, g + [0, 0, .06])
    arm.set_comp(True)
    yield from arm.move(g + [0, 0, .07], r, 2.5, 1.0, 'to bracket')
    yield from arm.move(g + [0, 0, .07], r, .6, grip_of(UPRIGHT_HALF_WIDTH + .012), 'open')
    pb, rb = c.pose('bracket')
    g = pb + rb @ GRASP_B
    yield from arm.move(g, r, 2.0, None, 'descend')
    yield from arm.set_grip(grip_of(UPRIGHT_HALF_WIDTH - SQUEEZE), 1.0, 'close')
    yield from wait(.5)
    c.note('gripped')
    yield from arm.move(arm.pos + [0, 0, .04], arm.rot, 1.5, None, 'lift')
    yield from wait(.8)
    capture_grasp(c)


def approach(c, yaw_deg, s0=.040, s1=.026):
    """Carry the bracket in front of the servo (backed off s0 then s1), cheeks level with the horns."""
    arm = c.right
    gp, gr = c.G
    p, r = bracket_target(yaw_deg, s0, .03)
    yield from arm.move(p + r @ gp, r @ gr, 3.0, None, 'carry')
    yield from track(c, lambda s: bracket_target(yaw_deg, s0, .03 * (1 - s)), 2.0, 'lower to cheek height')
    yield from track(c, lambda s: bracket_target(yaw_deg, s0 + s * (s1 - s0)), 1.5, 'close in')
    yield from track(c, lambda s: bracket_target(yaw_deg, s1), 2.0, 'settle')
    arm.set_comp('fine')
    yield from track(c, lambda s: bracket_target(yaw_deg, s1), 2.0, 'settle', gain=SLIDE_GAIN,
                     deadband=SLIDE_DEADBAND_M)
    c.note('aligned', yaw=yaw_deg)


def measured(c, yaw_deg):
    """(px, py, pz) in mm of the servo axis in the bracket frame, from the current poses (same as the scorer)."""
    pb, rb = c.pose('bracket')
    rel = rb.T @ (L.O_S - pb) * 1000.
    return float(rel[0]), float(rel[1]), float(rel[2])


def boss_lateral(c, z_mm=-36.6):
    """Diagnostics: [py at the boss height (mm), servo-axis direction x / y in the bracket frame (mrad)]."""
    pb, rb = c.pose('bracket')
    rel = rb.T @ (L.O_S - pb) * 1000.
    a = rb.T @ L.R_S[:, 2]
    t = (z_mm - rel[2]) / a[2]
    return [round(float(rel[1] + t * a[1]), 3), round(float(a[0] * 1e3), 2), round(float(a[1] * 1e3), 2)]


# The arm's joints stick and slip (0.3 N m static, no dynamic friction on the shoulder joints): sideways the hand
# settles anywhere within about +-1.2 mm (joint 1), and any integral action makes it hunt by that much. So:
# * cheek ends passing the horn rims (px about -22 mm; 0.2 mm face clearance + 0.4 mm rim chamfer): the slide only
#   advances while the measured height is inside the window (the height is servoed, its error stays small);
# * just before the screw head reaches the slot mouth (px -17 to -15 mm) the hand waits until the bracket is within
#   PRE_SLOT_PY_MM of the slot line (inside that the slot-mouth corners steer the head; sideways servoing cannot do
#   better than the +-1.2 mm stick-slip band);
# * screw head and boss entering their slots (from px -15 mm; 0.35 / 0.25 mm sideways clearance, tighter than the
#   hand can be placed): no joint integral there (the hand is pushed aside through its joints' stiffness) and only
#   the slow bracket-level servo sideways; the hand pushes on (about 2 N), and the round head and boss, meeting the
#   corners of the slot mouths, steer the bracket sideways into the slots, as a person lets a part find its slot.
RIM_WINDOW = (-24.5, -19.5)
SLIDE_GAIN = .02          # bracket-level integral gain during the slide (slow: grasp slip, sag drift)
SLIDE_DEADBAND_M = .0001
Z_AX = np.array([0., 0., 1.])
SLIDE_ROT_GAIN = .1       # bracket tilt / yaw integral gain during the slide (keeps the cheeks level on the horns)
PULSE_STEPS = 10          # in the slots: push 0.5 s, pause 0.5 s
PULSE_UNTIL_PX = -6.0     # from here on the push is continuous (the bracket is level and deep in its slots)
LEAD_M = .0015            # push lead before the slots (joint integral capped at FINE_LIMIT gets past stiction)
LEAD_SLOT_M = .012        # push lead from the slot mouths on (no joint integral: about 2 N through the arm)
PRESS_M = .003            # final press of the top horn onto the lip, and kept while turning to the holes
SLIDE_PAST_M = .006       # going home the slide's push target may lead up to this far past home (arm reach)
PRE_SLOT = (-17.0, -15.0)
WIGGLE_FROM_PX = -6.0
WIGGLE_DEG = 0.  # off: with the base joint free the wrist turned the bracket about the grasp, pressing the pins into their walls
PRE_SLOT_PY_MM = 1.3
FREE_J1_PX = -11.5        # from here on the head and the boss are inside their slots (mouth corners at x -11.7 mm)
MOUTH_ALIGN_MM = (.3, .25)  # head / boss sideways offsets under which the push resumes at the slot mouths
MOUTH_WAIT_S = 8.0
# At the slot mouths (base joint free, see above) the bracket is steered sideways without moving the hand: a turn
# about the vertical through the grasp point (wrist only; 1 deg moves the screw head about 0.8 mm) puts the head on
# its slot line, a roll about the slot axis (through the top cheek) puts the boss under it. Slow integrals of the
# measured offsets (poses), clipped; they fade once both are inside their slots.
STEER_WINDOW = (-14.5, -12.0)  # ends before the slot walls start (x -11.8 mm): inside, the slots set both offsets
STEER_GAIN = .02
STEER_YAW_MAX = math.radians(2.0)
STEER_ROLL_MAX = math.radians(1.0)
BOSS_Z_MM = -36.6
SLOT_WINDOW = (-15.0, -11.0)


def slide(c, yaw_deg, s_from=.026, s_to=0.0, speed=.004, label='slide', retries=3):
    """Slide along the slots (gated, see above) with a light push until the bracket is within 0.4 mm of ``s_to``
    (0 = home, top horn on the lip); if it makes no progress for 15 s or a gate stays shut for 12 s, back off 3 mm,
    re-level and retry (up to ``retries`` times)."""
    # the push target may lead past s_to by this much: going home (s_to <= 0) the full slot lead, so the push does not
    # weaken over the last millimetres (the top horn then stops on the lip); the final press uses PRESS_M
    s_floor = s_to - (max(PRESS_M, SLIDE_PAST_M) if s_to <= 0. else .001)
    s_press = s_to - (PRESS_M if s_to <= 0. else .001)
    axis = (L.R_S @ L.rz(yaw_deg))[:, 0]
    side = (L.R_S @ L.rz(yaw_deg))[:, 1]
    s_k, attempt, held, shut_at, k_slot, k_wig, k_mouth, regrips = s_from, 0, 0, None, 0, 0, 0, 0
    steer_yaw = steer_roll = 0.
    best, stall = s_from, 0
    c.right.set_comp('fine')
    while True:
        px, py, pz = measured(c, yaw_deg)
        pb, rb = c.pose('bracket')
        back = float(axis @ (pb - L.O_S))  # how far the bracket is still out (m)
        if back <= s_to + .0004:
            break
        gate = True
        if RIM_WINDOW[0] <= px <= RIM_WINDOW[1]:
            gate = abs(pz) < .3
        elif PRE_SLOT[0] <= px < PRE_SLOT[1]:
            gate = abs(py) < PRE_SLOT_PY_MM and abs(pz) < .3
        in_slot = px >= SLOT_WINDOW[0]
        c.right.set_comp(SLOT_COMP if in_slot else 'fine')
        # once the screw head is inside its slot the base joint (vertical axis; it is what moves the hand sideways
        # across the slots, which run roughly toward the arm's base) is left free: the slots then only have to
        # overcome its 0.3 N m stiction (about 0.9 N at the hand) to steer the bracket sideways
        # (with 0.5 mm hysteresis: px dithers by a few tenths, and every re-engagement would make the joint spring
        # back to the IK angle). At the slot mouths themselves the base joint is instead held at the angle it has
        # there (so the hand stays put sideways while the bracket is steered by the wrist, see STEER_*)
        was_free = c.right.free_j1
        c.right.free_j1 = px >= FREE_J1_PX - (.5 if was_free else 0.)
        mouth = STEER_WINDOW[0] <= px and not c.right.free_j1
        if mouth and c.right.hold_j1 is None:
            c.right.hold_j1 = float(c.right.measured[0])
        elif not mouth:
            c.right.hold_j1 = None
        pyb = boss_lateral(c, BOSS_Z_MM)[0]
        aligned = abs(py) < MOUTH_ALIGN_MM[0] and abs(pyb) < MOUTH_ALIGN_MM[1]
        k_mouth = k_mouth + 1 if mouth else 0
        lead = LEAD_SLOT_M if in_slot else LEAD_M
        s_prev = s_k
        k_slot = k_slot + 1 if in_slot else 0
        if mouth and not aligned and k_mouth < MOUTH_WAIT_S * CONTROL_HZ:
            # at the slot mouths: no push until the steering has put the head and the boss on their slot lines
            s_k = max(s_k, back)
            held, shut_at = 0, None
        elif in_slot and (k_slot // PULSE_STEPS) % 2 and px < PULSE_UNTIL_PX:
            # in the slots the push is pulsed: while it pushes, the push's moment (the hand holds the bracket 3 cm
            # above the top cheek) slowly tips the bracket until the cheeks wedge on the horns; in each pause the
            # bracket is levelled again (tilt servo) before the next push
            s_k = max(s_k, back)
            held, shut_at = 0, None
        elif in_slot:
            s_k = max(s_floor, back - lead, s_prev - .002)  # push: lead reached within a few steps
            held, shut_at = 0, None
        elif gate:
            s_k = max(s_floor, s_k - (speed if not in_slot else .6 * speed) * DT)
            s_k = max(s_k, min(back - lead, s_prev))  # light push: the target leads the bracket by at most ``lead``
            held, shut_at = 0, None
        else:
            # gate shut: no push while the hand levels / centres: the target comes back to where the bracket was
            # when the gate shut (never further out, so the target cannot run away outward)
            held += 1
            shut_at = back if shut_at is None else shut_at
            s_k = max(s_k, min(back, shut_at))
        # Height: servoed to the middle of the 0.4 mm play between the horn faces (left alone, the bracket settles
        # onto the top horn and its screw head onto the groove's roof, and drags). Sideways: servoed to the slot
        # line. Both targets are reachable inside the fit, so the integrals do not wind up. Tilt: servoed level.
        free = [axis]
        if c.right.free_j1 or c.right.hold_j1 is not None:
            # no sideways integral while the base joint is free or held: it is not commanded, so the integral would only
            # wind up (to its 8 mm clip) and swing the hand sideways the moment the joint is engaged again
            free.append(side)
        # over the last 6 mm the bracket is rocked +-WIGGLE_DEG about the horn axis (1 s period) while it is pushed,
        # as a person works a snug part home
        k_wig = k_wig + 1 if px > WIGGLE_FROM_PX else 0
        yaw_k = yaw_deg + WIGGLE_DEG * math.sin(2 * math.pi * k_wig / CONTROL_HZ)
        if (mouth or c.right.free_j1) and STEER_WINDOW[0] <= px <= STEER_WINDOW[1]:
            gx = GRASP_B[0] * 1e3
            # bracket turned by d about the grasp vertical: the horn axis moves in the bracket frame by d * (gx - px)
            steer_yaw = float(np.clip(steer_yaw + STEER_GAIN * py / (px - gx), -STEER_YAW_MAX, STEER_YAW_MAX))
            # rolled by r about the slot axis: the boss-height point moves sideways by r * |BOSS_Z_MM|
            steer_roll = float(np.clip(steer_roll + STEER_GAIN * (pyb - py) / -BOSS_Z_MM, -STEER_ROLL_MAX,
                                       STEER_ROLL_MAX))
        elif px > STEER_WINDOW[1]:
            # in the slots, which now hold the bracket sideways: the corrections fade (about 2 s), so the bracket is
            # pushed square along its slots again
            steer_yaw *= .975
            steer_roll *= .975
        yield from track(c, lambda u, s_k=s_k, yaw_k=yaw_k, sy=steer_yaw, sr=steer_roll:
                         steered(bracket_target(yaw_k, s_k), sy, sr),
                         DT, f'{label} {attempt}',
                         free_axis=free, gain=SLIDE_GAIN, deadband=SLIDE_DEADBAND_M, rot_gain=SLIDE_ROT_GAIN)
        pb, rb = c.pose('bracket')
        back = float(axis @ (pb - L.O_S))
        if back < best - .0002:
            best, stall = back, 0
        else:
            stall += 1
        if held > 12 * CONTROL_HZ or stall > (6 if px > -12.0 else 15) * CONTROL_HZ:
            c.note('gate timeout' if held > 12 * CONTROL_HZ else 'stalled', s_target=s_k, s_actual=back,
                   attempt=attempt, pxyz=[px, py, pz])
            best, stall = back + .003, 0
            if px > -12.0 and regrips < REGRIPS:
                # deep in the slots: do not back out; let go for a moment (the bracket, freed of the hand's stored
                # push and twist, settles on the horns), grip it again and carry on pushing
                regrips += 1
                best, stall = back, 0
                yield from regrip(c)
                continue
            if attempt >= retries or px > -12.0:
                break  # backing out of the slots loses more than it gains: finish() pushes
            attempt += 1
            s_k = min(s_from, back + .003)
            held, shut_at = 0, None
            c.right.free_j1 = False
            c.right.set_comp('fine')
            yield from track(c, lambda u: bracket_target(yaw_deg, s_k), 1.0, 'back off', gain=SLIDE_GAIN,
                             deadband=SLIDE_DEADBAND_M, rot_gain=SLIDE_ROT_GAIN)
            yield from track(c, lambda u: bracket_target(yaw_deg, s_k), 2.0, 're-level', gain=SLIDE_GAIN,
                             deadband=SLIDE_DEADBAND_M, rot_gain=SLIDE_ROT_GAIN)
    c.right.set_comp('hold')
    c.right.free_j1 = True
    # final press: short when the bracket is already home (while the hand holds it pressed, contact chatter ratchets
    # the top horn round under it, about 2-3 deg/s)
    press_s = .25 if s_to <= 0. and out_mm(c, yaw_deg) < .3 else 1.0
    yield from track(c, lambda s: bracket_target(yaw_deg, s_press if s_to <= 0. else s_to), press_s, label + ' press',
                     gain=SLIDE_GAIN, deadband=SLIDE_DEADBAND_M, rot_gain=SLIDE_ROT_GAIN, free_axis=[axis, Z_AX, side])
    c.right.free_j1 = False
    c.note('slid', yaw=yaw_deg, s_to=s_to)


REGRIPS = 0               # (regrips let the bracket slip back; a stalled slide goes to push_home instead)


def regrip(c):
    """Relax the arm, open 3 mm, let the bracket settle, close again and re-measure the grasp."""
    arm = c.right
    arm.relax()
    yield from wait(.3)
    yield from arm.set_grip(grip_of(UPRIGHT_HALF_WIDTH + .003), .6, 'regrip open')
    yield from wait(.5)
    yield from arm.set_grip(grip_of(UPRIGHT_HALF_WIDTH - SQUEEZE), .6, 'regrip close')
    yield from wait(.3)
    capture_grasp(c)
    c.note('regripped')


def _slot_axis(yaw_deg):
    """Bracket x axis at yaw_deg (along the slots: the seated bracket's depth is set by the top horn on the lip)."""
    return (L.R_S @ L.rz(yaw_deg))[:, 0]


REGISTER_ROUNDS = 5
REGISTER_TOL_DEG = .3


def register(c, yaw_deg, rounds=REGISTER_ROUNDS, off_deg=0.):
    """Turn the seated bracket about the servo axis until its hole pattern matches the top horn's (read from poses),
    re-measuring after each turn (``off_deg``: aim that far off, for the negative case)."""
    for i in range(rounds):
        pb, rb = c.pose('bracket')
        bracket_yaw = math.degrees(math.atan2(*(L.R_S.T @ rb[:, 0])[[1, 0]]))
        err = ((bracket_yaw - horn_yaw(c) - off_deg) + 45.) % 90. - 45.
        if abs(err) < REGISTER_TOL_DEG:
            break
        target = yaw_deg - err
        y0 = yaw_deg
        # slow turn, level and centred between the horn faces, so the cheeks do not wedge on the horns and drag
        # the top horn round with them
        yield from track(c, lambda s: bracket_target(y0 + s * (target - y0), -PRESS_M), max(1.5, abs(err) / 5.),
                         f'turn to holes {i}', gain=SLIDE_GAIN, deadband=SLIDE_DEADBAND_M, rot_gain=SLIDE_ROT_GAIN,
                         free_axis=[_slot_axis(target), Z_AX])
        yield from track(c, lambda s: bracket_target(target, -PRESS_M), 1.0, 'settle turn', gain=SLIDE_GAIN,
                         deadband=SLIDE_DEADBAND_M, rot_gain=SLIDE_ROT_GAIN, free_axis=[_slot_axis(target), Z_AX])
        yaw_deg = target
    c.note('registered', yaw=yaw_deg)
    return yaw_deg


def let_go(c):
    """Take the push off (arm commanded to where it is) and open the fingers at once, 6 mm clear of the upright. Left
    gripped, the relaxed arm's wrist creeps (about 0.2 mm/s at the hand) and carries the seated bracket with it."""
    arm = c.right
    arm.relax()
    yield from arm.set_grip(grip_of(UPRIGHT_HALF_WIDTH + .006), .3, 'open')
    yield from wait(.5)
    c.note('opened', holes_mm=holes_mm(c))


def release(c, look=False, opened=False):
    """Open, lift straight up; then either go back to READY or (look=True) hold the hand 15 cm in front of and above
    the joint, the wrist camera looking down at the bracket's top arm, so its last frames show the holes."""
    arm = c.right
    if not opened:
        yield from let_go(c)
    arm.set_comp('hold')  # keep the learned sag offsets while the hand lifts clear (no drop onto the part)
    yield from arm.move(arm.pos + [0, 0, .04], arm.rot, 1.2, None, 'lift away')
    if look:
        a = np.array([.35, 0., -1.])
        a /= np.linalg.norm(a)                     # approach axis: down, leaning away from the robot
        x = np.array([1., 0., 0.])                 # fingers closing along station x; the wrist camera then sits
        x = x - (x @ a) * a                        # 8 cm to the hand's left, looking down at the joint
        x /= np.linalg.norm(x)
        r = np.column_stack([x, np.cross(a, x), a])
        target = L.O_S + np.array([0., 0., .005])
        yield from arm.move(target - .17 * a, r, 2.0, None, 'look at holes')
    else:
        yield from arm.home(2.0)
    c.note('released')


# Last stage: once screw head and boss are in their slots (with less overlap, a let-go bracket tips back on the horns
# by its own weight, up to 8 deg at 4 mm overlap, and jams), let go and push the bracket home with the closed
# fingertips against the back of its upright, low down between the cheeks. Held in the hand, the bracket is tied to the arm's
# sticky, soft joints (pressed onto the horn faces and slot walls, it drags and wedges); let go, it rests on the
# horns, the slot-mouth corners steer the screw head and boss into their slots, and only its own weight's friction
# (about 0.1 N) resists, as when a person lets a part find its seat. Pushing low (between the two horn contacts)
# keeps the push from tipping the bracket and jamming it like a drawer pushed high.
PUSH_FROM_S = 0.          # held slide all the way home (push_home kept for the partial-insertion case)
PUSH_Z_B = -.018          # fingertip height on the upright's back, bracket frame (between the cheeks)
PUSH_BACK_X_B = .0476     # the upright's back face, bracket frame
PUSH_LEAD_M = .010        # push target ahead of the bracket's measured position
PUSH_FREE_J1 = False
PUSH_SPEED = .003         # m/s


def push_home(c, yaw_deg, s_to=0.):
    """Let go, put the closed fingertips behind the upright and push the bracket along its slots until it is within
    0.4 mm of ``s_to`` (0 = home: the top horn against the lip) or it stops moving for 10 s."""
    from yam_arena.constants import PAD_TIP_PAST_TCP
    arm = c.right
    xb = (L.R_S @ L.rz(yaw_deg))[:, 0]
    arm.relax()
    yield from wait(.3)
    yield from arm.set_grip(grip_of(UPRIGHT_HALF_WIDTH + .006), 1.5, 'let go to push')
    arm.set_comp('fine')
    arm.set_comp('push')  # bounded joint integral: the fingertips push low (between the cheeks), no tipping moment
    pb, rb = c.pose('bracket')
    up = np.array([0., 0., 1.])
    pad = lambda x_b, z_b: pb + xb * x_b + up * (z_b + PAD_TIP_PAST_TCP)  # noqa: E731  (TCP for pad tips at x_b, z_b)
    yield from arm.move(arm.pos + up * .028, arm.rot, 1.2, None, 'lift off')
    yield from arm.move(pad(PUSH_BACK_X_B + .016, .060), arm.rot, 1.5, None, 'behind upright')
    yield from arm.set_grip(0., .8, 'close fingers')
    yield from arm.move(pad(PUSH_BACK_X_B + .016, PUSH_Z_B), arm.rot, 2.0, None, 'down behind upright')
    c.note('push start', yaw=yaw_deg)
    # base joint free while pushing: sideways the fingertips follow the bracket (its slots hold it) instead of the
    # base joint's stick-slip dragging them across the upright's back and turning the bracket
    arm.free_j1 = PUSH_FREE_J1
    d = float(xb @ (arm.pos - pb))  # TCP ahead of the bracket origin along +x_b, before contact
    best, stall = None, 0
    while True:
        pb, _ = c.pose('bracket')
        back = float(xb @ (pb - L.O_S))
        if back <= s_to + .0004 or stall > 10 * CONTROL_HZ:
            break
        # move in at PUSH_SPEED until touching, then keep the target PUSH_LEAD_M ahead of where contact holds it
        d = max(d - PUSH_SPEED * DT, (PUSH_BACK_X_B + .006) - PUSH_LEAD_M)
        tgt = L.O_S + xb * back + xb * d
        tgt[2] = arm.pos[2]
        arm.label = 'push home'
        arm.set_pose(tgt, arm.rot)
        yield
        if best is None or back < best - .0002:
            best, stall = back, 0
        else:
            stall += 1
    if arm.free_j1:
        arm.relax()  # base joint engaged again at the angle it has (no spring-back sideways)
    c.note('pushed', s_to=s_to)


def out_mm(c, yaw_deg):
    pb, _ = c.pose('bracket')
    return float((L.R_S @ L.rz(yaw_deg))[:, 0] @ (pb - L.O_S)) * 1000.


SEAT_S = 2.0
SEAT_TRIES = 3            # seat, let go, check the holes from the poses; grip and seat again if they are off
SEAT_OK_MM = .12          # (the scored limit is 0.15 mm)


def finish(c, yaw_deg, look=True, hold=False, turn=True, off_deg=0.):
    """After the held slide: if the bracket is not yet home, let go and push it home with the fingertips; turn it to
    the holes (held only, from the measured poses), press it into the lip, take the push off and let go (unless
    ``hold``); if letting go moved it off the holes, grip it and seat it again."""
    if out_mm(c, yaw_deg) > .4:
        yield from push_home(c, yaw_deg)
        if not hold:
            yield from retract_and_look(c, look=look)
        return
    for attempt in range(SEAT_TRIES):
        c.right.set_comp('fine')  # bounded joint integral: the hand gets past its joints' stiction (about 1 N at most)
        if turn:
            yaw_deg = yield from register(c, yaw_deg, off_deg=off_deg)
        # seat: press the top horn into the lip arc (its 0.1 mm clearance then centres the bracket sideways); the push
        # is taken off by let_go() (arm commanded to where it is) as the fingers open
        yield from track(c, lambda s: bracket_target(yaw_deg, -PRESS_M), SEAT_S, 'seat', gain=SLIDE_GAIN,
                         deadband=SLIDE_DEADBAND_M, rot_gain=SLIDE_ROT_GAIN, free_axis=[Z_AX])
        c.note('held', holes_mm=holes_mm(c))
        if hold:
            return
        yield from let_go(c)
        if not turn or off_deg or holes_mm(c) <= SEAT_OK_MM or attempt == SEAT_TRIES - 1:
            break
        # letting go moved the bracket off the holes (read from the poses): grip it again where it is and seat it again
        yield from c.right.set_grip(grip_of(UPRIGHT_HALF_WIDTH - SQUEEZE), .6, 'grip again')
        yield from wait(.3)
        capture_grasp(c)
    yield from release(c, look=look, opened=True)


def holes_mm(c):
    """Largest offset (mm) of a top hole from its horn hole, from the measured bracket and horn poses (the scorer's
    geometry, src/scoring.py)."""
    from scoring import hole_offset
    pb, rb = c.pose('bracket')
    return round(hole_offset(pb, rb, horn_yaw(c), L.O_S, L.R_S) * 1e3, 4)


def jointkit_measure(c):
    import jointkit
    return jointkit.STATE['detail'].get('measure')


def retract_and_look(c, look=True):
    """Fingertips back 3 cm and up, then (look=True) the wrist camera looks down at the top arm's holes."""
    arm = c.right
    xb_now = arm.pos - L.O_S
    xb_now[2] = 0.
    away = xb_now / max(np.linalg.norm(xb_now), 1e-9)
    yield from arm.move(arm.pos + away * .03, arm.rot, 1.2, None, 'back off')
    yield from release(c, look=look, opened=True)


def horn_yaw(c):
    _, rh = c.pose('horn_top')
    v = L.R_S.T @ rh[:, 0]
    return math.degrees(math.atan2(v[1], v[0]))


def entry_yaw(c):
    """Bracket yaw (deg, about the servo axis) to slide on at: the one of the four hole-matching angles of the top
    horn (read from its pose) nearest square-on whose whole carry / slide path the arm can reach (IK only)."""
    h = horn_yaw(c)
    base = ((h + 45.) % 90.) - 45.
    gp, gr = c.G
    for yaw in sorted((base, base - 90., base + 90.), key=abs):
        ok = True
        for s_m, lift in ((.040, .03), (.040, 0.), (.026, 0.), (0., 0.), (-.0004, 0.)):
            p, r = bracket_target(yaw, s_m, lift)
            _, pe, re = c.right.solve(p + r @ gp, r @ gr)
            ok = ok and pe < .001 and re < math.radians(1.5)
        if ok:
            c.note('entry yaw', yaw=yaw, horn_deg=h)
            return yaw
    c.note('entry yaw', yaw=0., horn_deg=h, fallback=True)
    return 0.


def demo(c):
    yield from wait(.5)
    yield from pick(c)
    yaw = entry_yaw(c)
    yield from approach(c, yaw)
    yield from slide(c, yaw, s_to=PUSH_FROM_S)
    yield from finish(c, yaw, look=True)
    yield from wait(3.0)


def neg_partial(c):
    """Slides the bracket on only until it is 6 mm from home and lets go: on both horns, not home (0.25)."""
    yield from pick(c)
    yaw = entry_yaw(c)
    yield from approach(c, yaw)
    yield from slide(c, yaw, s_to=.006, label='slide part way')
    yield from release(c)
    yield from wait(3.0)


def neg_unregistered(c):
    """Slides fully home square to the base but never turns it to the horn's holes (25 deg off): home (0.5)."""
    yield from pick(c)
    yield from approach(c, 0.)
    yield from slide(c, 0., s_to=PUSH_FROM_S)
    yield from finish(c, 0., look=False, turn=False)
    yield from wait(3.0)


def neg_offset(c):
    """Full fit, but the bracket is turned 3 deg off the horn's holes (0.37 mm at the holes: it looks lined up, but an
    M3 screw cannot start), then let go: home, holes not lined up (0.5)."""
    yield from wait(.5)
    yield from pick(c)
    yaw = entry_yaw(c)
    yield from approach(c, yaw)
    yield from slide(c, yaw, s_to=PUSH_FROM_S)
    yield from finish(c, yaw, look=False, off_deg=3.)
    yield from wait(3.0)


def neg_held(c):
    """Full fit, but the hand is never taken away (0.75: fingertips stay within 20 mm of the bracket). Same moves as
    the demo, including opening the fingers; then the hand stays where it is, around the upright, instead of lifting
    away."""
    yield from wait(.5)
    yield from pick(c)
    yaw = entry_yaw(c)
    yield from approach(c, yaw)
    yield from slide(c, yaw, s_to=PUSH_FROM_S)
    yield from finish(c, yaw, hold=True)
    yield from let_go(c)
    c.right.label = 'open (hand stays)'
    c.note('holding')
    yield from wait(4.0)


def neg_jammed(c):
    """Pushes the bracket in 1.5 mm too high: the lower cheek's end stops on the bottom horn's rim and the top
    cheek rides on the top horn's face; the hand pushes, then lets go (expected 0: not straddling)."""
    yield from pick(c)
    arm = c.right
    gp, gr = c.G
    p, r = bracket_target(0., .035, .03)
    yield from arm.move(p + r @ gp, r @ gr, 3.0, None, 'carry')
    yield from track(c, lambda s: bracket_target(0., .035, .03 - .0285 * s), 2.0, 'lower (1.5 mm high)')
    yield from track(c, lambda s: bracket_target(0., .035 - .025 * s, .0015), 4.0, 'push in high')
    c.note('jammed')
    yield from release(c)
    yield from wait(3.0)


def neg_beside(c):
    """Sets the bracket down on the table beside the base, never on the horns (0)."""
    yaw = horn_yaw(c)
    yield from pick(c)
    arm = c.right
    gp, gr = c.G
    p, r = bracket_target(yaw, .070, .01)
    p = p + np.array([0., -.06, 0.])
    yield from arm.move(p + r @ gp, r @ gr, 3.0, None, 'carry beside')
    yield from arm.move(arm.pos + [0, 0, -.009], arm.rot, 1.5, None, 'set down')
    yield from release(c)
    yield from wait(3.0)


def idle(c):
    yield from wait(8.0)


STAGES = {'demo': demo, 'neg-partial': neg_partial, 'neg-unregistered': neg_unregistered, 'neg-offset': neg_offset,
          'neg-held': neg_held,
          'neg-beside': neg_beside, 'neg-jammed': neg_jammed, 'idle': idle}


def _write_mp4(frames, path):
    import imageio
    w = imageio.get_writer(str(path), fps=CONTROL_HZ, codec='libx264', quality=8, macro_block_size=8,
                           ffmpeg_params=['-pix_fmt', 'yuv420p'])
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(task_path, out_dir, max_steps=2400, record=True, stage='demo', cam_width=320, cam_height=176, device='cpu'):
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=cam_width, cam_height=cam_height, device=device)
    stages = stage.split(',')
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f'trial{i}_{st}'
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0)
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / 'reset_check.json').write_text(json.dumps(rep, indent=2, default=float))
        print('RESETCHECK', json.dumps(rep['summary']), flush=True)
    return results


def reset_state_now():
    import jointkit
    rt = jointkit.runtime()
    poses = rt.read()
    return dict(score=jointkit.STATE['score'],
                bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
                rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()})


def reset_report(stages, resets, results):
    base = resets[0]
    rows = []
    for i, r in enumerate(resets):
        dev = max(float(np.linalg.norm(np.subtract(r['bodies'][n], base['bodies'][n]))) for n in base['bodies'])
        rdev = max(float(np.abs(np.subtract(r['rots'][n], base['rots'][n])).max()) for n in base['rots'])
        rows.append(dict(trial=i, stage=stages[i], score_after_reset=r['score'],
                         max_body_dev_from_trial0_mm=round(dev, 4), max_rot_dev_from_trial0=round(rdev, 6),
                         trial_score=results[i]['task_score'], trial_stop=results[i]['stop']))
    ok = all(x['score_after_reset'] == 0. and x['max_body_dev_from_trial0_mm'] < 0.5 for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows), max_body_dev_mm=max(x['max_body_dev_from_trial0_mm']
                                                                               for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0):
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY
    import jointkit

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id='demo', instruction='demo'))
    print(f'BENCH reset {time.time() - t1:.0f} s', flush=True)
    reset_state = reset_state_now()
    c = Ctx(emb)
    gen = STAGES[stage](c)
    frames = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, info, stop, success = 0, None, {}, None, False
    tracef = open(out / 'trace.jsonl', 'w')
    while steps < max_steps:
        if ended is None:
            try:
                next(gen)
            except StopIteration:
                ended = steps
            except Stop as e:
                stop, ended = str(e), steps
                print('STOP', stop, flush=True)
        res = emb.step(c.action())
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        c.left.observe(jp[:7])
        c.right.observe(jp[7:14])
        steps += 1
        c.step = steps
        info = res.info
        snap(res.observation)
        d = jointkit.STATE['detail']
        row = dict(step=steps, score=info.get('score'), level=jointkit.STATE['level'], detail=d,
                   labels=[c.left.label, c.right.label], nonfinite=jointkit.STATE['nonfinite'],
                   cmd=np.round(np.r_[c.left.cmd, c.right.cmd], 4).tolist(), q=np.round(jp, 4).tolist(),
                   corr_mm=np.round(np.asarray(getattr(c, 'corr_p', np.zeros(3))) * 1000, 3).tolist(),
                   comp=str(c.right.comp), boss=boss_lateral(c))
        tracef.write(json.dumps(row, default=float) + '\n')
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            m = d.get('measure', {})
            print(f"step {steps} {c.right.label} score {info.get('score')} px {m.get('px_mm')} py {m.get('py_mm')} "
                  f"pz {m.get('pz_mm')} tilt {m.get('tilt_deg')} reg {m.get('register_deg')} holes {m.get('hole_offset_mm')} "
                  f"clear {d.get('tip_clearance_mm')} boss {row['boss']}", flush=True)
        if res.terminated:
            success = res.termination_reason == 'success'
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get('score', 0.) or 0.)
    result = dict(stage=stage, task_score=score, success=success, steps=steps, stop=stop,
                  termination=getattr(res, 'termination_reason', None),
                  scorer=dict(score=jointkit.STATE['score'], level=jointkit.STATE['level']),
                  final=jointkit.STATE['detail'], nonfinite=jointkit.STATE['nonfinite'], looks=c.log,
                  wall_s=round(time.time() - t1, 1), wall_total_s=round(time.time() - t0, 1),
                  worst_ik_mm=dict(left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)),
                  physics_dt=jointkit.PHYSICS_DT, control_hz=CONTROL_HZ)
    tracef.close()
    (out / 'result.json').write_text(json.dumps(result, indent=2, default=float))
    if record:
        import imageio
        for cam, fs in frames.items():
            _write_mp4(fs, out / f'demo_{cam}.mp4')
        top = next((k for k in frames if 'top' in k), next(iter(frames), None))
        if top:
            imageio.imwrite(out / 'setup.png', frames[top][0])
            imageio.imwrite(out / 'final.png', frames[top][-1])
        for cam, fs in frames.items():
            imageio.imwrite(out / f'final_{cam}.png', fs[-1])
    print('RESULT', json.dumps({k: result[k] for k in ('stage', 'task_score', 'success', 'steps', 'stop')}), flush=True)
    return result, reset_state


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('out')
    ap.add_argument('--max-steps', type=int, default=2400)
    ap.add_argument('--no-record', action='store_true')
    ap.add_argument('--stage', default='demo')
    ap.add_argument('--cam-size', default='320x176', help='camera WxH (evidence runs use the default)')
    ap.add_argument('--device', default='cpu', help='PhysX device (cpu, or cuda:0); rendering is on the GPU either way')
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split('x'))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device)
