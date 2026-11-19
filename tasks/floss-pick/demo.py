"""Scripted demo (right arm), reading object poses (the pick's pose, measured every control step).

1. Grasp: the right hand pinches the thick grip's rear 26 mm, which overhangs the board's edge, across its 10 mm
   depth (pads on its two 6 mm-thick edges, about 15 N a pad), lifts it and rolls 90 degrees about the handle so the
   arms point down (fingers above and below the handle, the hand coming from the robot's right).
2. In through the lips: the floss 6 mm above the incisal edges, across the contact (front arm 2.4 mm in front of the
   contact, the handle out of the mouth along x).
3. Saw it through: the floss settled on the contact; then +-1 mm along the floss at 2 Hz while the floss's height is
   closed-loop on a target descending 0.5 mm/s (0.15 mm/s over the contact's last 0.3 mm), the push bounded about its
   free-space value, until the floss is 0.6 mm below the contact zone; then held 1 mm below the contact.
4. Hug and wipe: the pick shifted 0.5 mm toward the left incisor (41), so the floss wraps its facing side; one stroke
   at 2 mm/s down to 2.05 mm below the contact and back up to the contact zone's bottom; then the right incisor (31).
   No upward push is added on the way down (the lip, springing back as the front arm rises, pushes the pick up).
5. Out: centred, pulled straight up through the contact (it holds from below until pulled 3 N), lifted 1 cm above
   the incisal edges, drawn out of the mouth 5 cm, held.

    python demo.py task.py demo_out [--max-steps 3600] [--no-record] [--variant NAME] [--upto PHASE]

Variants (scorer checks): ``snap`` saws through, then drops the hand 12 mm at once (the floss snaps onto the gum);
``press`` presses the floss into the gum after passing; ``one_side`` wipes only the left incisor; ``straight`` strokes
up and down in the middle of the gap without hugging either tooth; ``leave`` does everything but bring the floss back
out; ``push`` pushes straight down without sawing. Recorded runs move the top camera to a close view of the mouth
once the floss is over the notch (``--keep-top-cam`` keeps it).
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, together, wait  # noqa: E402

DEBUG = bool(os.environ.get("FP_DEBUG"))
KEEP_ROLL = os.environ.get("FP_KEEP_ROLL", "1") == "1"
GAP_MM = 94.0                        # pads' gap at grip 1.0
SQUEEZE_MM = float(os.environ.get("FP_SQUEEZE", "6.0"))   # the fingers commanded 6 mm inside the grip: about 15 N a pad
R_TABLE = np.column_stack([[0, -1, 0], [-1, 0, 0], [0, 0, -1.0]])   # from above, closing across the handle (y)
R_WORK = np.column_stack([[0, 0, -1], [-1, 0, 0], [0, 1, 0.0]])     # from the right (approach +y), closing along z
PAD_BELOW_TCP = float(os.environ.get("FP_PAD_MM", "3.0")) / 1000.0   # the handle's mid-thickness this far past the tool point (the pads' wide part)
X_MID_ENTRY = -2.4 + g.SPAN / 2      # the floss midpoint when the front anchor is 2.4 mm in front of the contact (its
                                     # tapered tip meets the lip's rounded top edge where it is steep)
X_MID_SAW = -2.6 + g.SPAN / 2        # sawing about this (the front arm 2.6 mm in front of the contact)
X_MID_HUG = -2.6 + g.SPAN / 2        # ... 2.6 mm (hugging and stroking)
SAW_AMP, SAW_HZ = 1.0, 2.0           # mm, Hz (commanded; the arm moves the pick about half that)
PUSH_LAG = float(os.environ.get("FP_PUSH_LAG", "2.5"))   # mm: the target held this far below the pick while sawing in
LOWER_RATE = 0.5                     # mm/s while sawing in
POST_BAND_DOWN, POST_BAND_UP = 10.0, 4.0   # mm: the z correction's bound below the contact (about 2 N down, 0.8 N up)
Y_BAND = 3.0                         # mm: the sideways correction stays this close to its free-space value (about
                                     # 0.6 N), so a drift can never load the floss sideways enough to turn the pick
PUSH_BAND = 10.0                     # mm: the z correction stays this close to its free-space value (a bounded push)
ENTRY_BAND = 15.0                    # mm: ... while the floss is still above the incisal edges (the lip being wedged)
RAISE_RATE = 0.5                     # mm/s: the target rising while pulling out
HUG = float(os.environ.get("FP_HUG", "0.5"))      # mm: the pick shifted toward the tooth
STROKE_TOP, STROKE_BOT = -0.7, -2.05
STROKE_SPEED = 2.0                   # mm/s
STROKES = 1
HOLD_BELOW = -1.0                    # the floss held here after passing
HUG_Z = -1.0                         # ... and while hugging
PASSED_BELOW = 0.6                   # mm: the line between the arm tips this far below the contact zone = passed
CLEAR_Z = g.Z_EDGE + 10.0            # lifted 1 cm above the incisal edges


def grip_for(width_mm: float, squeeze: bool = True) -> float:
    return max(0.0, (width_mm - (SQUEEZE_MM if squeeze else 0.0)) / GAP_MM)


def tcp_actual(arm):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints, env

    return ik.tcp_pose(arm_joints(env(), arm.side, ARM_JOINTS), arm.base)


def pick_pose(ctx):
    """The pick's origin (the floss's midpoint, station m) and rotation."""
    s = ctx.states()["pick"]
    return np.asarray(s.pos, float), ctx.task._rot(s.quat_xyzw)


def pick_h(ctx) -> np.ndarray:
    """The controlled point (head frame, mm): the floss's midpoint in x; in y and z the floss line where it crosses
    the contact (x 0), from the pick's pose and its arm tips (the pick may sit pitched or yawed in the hand)."""
    p, R = pick_pose(ctx)
    A = g.to_head(p + R @ g.ANCHOR_A / 1000.0)
    B = g.to_head(p + R @ g.ANCHOR_B / 1000.0)
    mid = (A + B) / 2
    if B[0] - A[0] < 5.0:                      # not across the teeth (in the hand on the way): the midpoint
        return mid
    t = float(np.clip(-A[0] / (B[0] - A[0]), 0.0, 1.0))
    c = A + t * (B - A)
    return np.array([mid[0], c[1], c[2]])


def settle_to(arm, pos, rot, tol=0.0004, rounds=3, seconds=0.6):
    from demokit import wait as w

    target = np.asarray(pos, float)
    cmd = target.copy()
    yield from arm.move(cmd, rot, seconds)
    for _ in range(rounds):
        yield from w(0.3)
        err = target - tcp_actual(arm)[0]
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
        yield from arm.move(cmd, rot, 0.4)


class Holder:
    """The pick held in the right hand: maps a desired pick pose to the hand's command, through the grasp measured
    after the roll, plus a correction that closes the loop on the pick's measured position."""

    def __init__(self, ctx, arm):
        self.ctx, self.arm = ctx, arm
        self.corr = np.zeros(3)
        self.yaw_corr = 0.0
        self.yaw_frozen = False
        self.z_free = None              # the z correction settled in free space; the push is bounded about it
        self.z_band = None
        self.z_lo = self.z_hi = None    # an asymmetric bound on the z correction (after the pass)
        self.y_free = None              # the y correction settled in free space; the sideways push is bounded about it
        self.y_band = None
        self.measure()

    def measure(self):
        p_t, R_t = tcp_actual(self.arm)
        p_p, R_p = pick_pose(self.ctx)
        self.R_rel = R_t.T @ R_p
        self.p_rel = R_t.T @ (p_p - p_t)
        # the pick's roll about its handle in the hand is kept, not levelled (levelling a 10-15 degree roll puts the
        # wrist past its limit); the floss stays along x and its position is closed-loop anyway
        self.roll0 = math.atan2(-R_p[1, 2], R_p[2, 2]) if KEEP_ROLL else 0.0

    def tcp_for(self, p_h, yaw=0.0):
        """The hand's pose that puts the floss's midpoint at p_h (head frame, mm) with the pick level, yawed."""
        c, s = math.cos(yaw), math.sin(yaw)
        cr, sr = math.cos(self.roll0), math.sin(self.roll0)
        R_d = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]) @ np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_t = R_d @ self.R_rel.T
        p = g.to_world(p_h) - R_t @ self.p_rel
        return p, R_t

    def aim(self, p_h, yaw=0.0, k=(0.25, 0.25, 0.25), integrate=(True, True, True), clamp=0.02):
        """One control step toward p_h: the nominal hand pose plus the correction, the correction stepped by the
        measured error on the axes that integrate."""
        err = (np.asarray(p_h, float) - pick_h(self.ctx)) / 1000.0
        for i in range(3):
            if integrate[i]:
                self.corr[i] = float(np.clip(self.corr[i] + k[i] * err[i], -clamp, clamp))
        if self.z_band is not None:
            self.corr[2] = float(np.clip(self.corr[2], self.z_free - self.z_band, self.z_free + self.z_band))
        elif self.z_lo is not None:
            self.corr[2] = float(np.clip(self.corr[2], self.z_lo, self.z_hi))
        if self.y_band is not None:
            self.corr[1] = float(np.clip(self.corr[1], self.y_free - self.y_band, self.y_free + self.y_band))
        R_p = pick_pose(self.ctx)[1]
        yaw_meas = math.atan2(R_p[1, 0], R_p[0, 0])
        k_yaw = 0.0 if self.yaw_frozen else 0.2           # frozen once the floss is among the teeth (turning the hand
                                                          # swings the floss sideways: a pick slipping in the hand
                                                          # would be chased)
        self.yaw_corr = float(np.clip(self.yaw_corr + k_yaw * (yaw - yaw_meas), -0.2, 0.2))
        p, R = self.tcp_for(p_h, yaw + self.yaw_corr)
        self.arm.aim(p + self.corr, R)
        return err * 1000.0

    def go(self, p0, p1, seconds, yaw0=0.0, yaw1=0.0, **kw):
        n = max(1, round(seconds * 20))
        from yam_arena import ik
        for i in range(1, n + 1):
            s = ik.smoothstep(i / n)
            self.aim(np.asarray(p0) + s * (np.asarray(p1) - np.asarray(p0)), yaw0 + s * (yaw1 - yaw0), **kw)
            yield

    def hold(self, p, seconds, yaw=0.0, **kw):
        for _ in range(max(1, round(seconds * 20))):
            self.aim(p, yaw, **kw)
            yield


# -- phases ------------------------------------------------------------------------------------------------------------
def grasp(ctx, arm):
    p, R = pick_pose(ctx)
    grip_c = p + R @ g.GRIP_C / 1000.0
    tcp = np.array([grip_c[0], grip_c[1], p[2] + PAD_BELOW_TCP])
    yield from arm.move_joints(tcp + [0, 0, 0.06], R_TABLE, 2.5, grip=grip_for(g.HANDLE_W + 14, squeeze=False))
    yield from arm.move(tcp + [0, 0, 0.015], R_TABLE, 1.0)
    yield from settle_to(arm, tcp, R_TABLE)
    yield from arm.set_grip(grip_for(g.HANDLE_W), 0.8)
    yield from wait(0.4)
    report_grasp(ctx, arm, "closed")
    yield from arm.move(arm.pos + [0, 0, 0.05], R_TABLE, 1.0)
    yield from wait(0.3)
    report_grasp(ctx, arm, "lifted")


ROLL_AT = np.array([0.17, -0.12, g.TOP + 0.17])   # where the hand rolls the pick (clear of the head and table)


def roll_joints(arm, pos, rot, seconds):
    """A joint-space move to (pos, rot) through the solution nearest the current joints among a few seeds (the roll
    from pinching on the table to arms-down; checked offline: the pick stays 10 cm above the table)."""
    from yam_arena import ik

    quat = ik.matrix_to_quat_xyzw(rot)
    best = None
    for seed in ([0.3, 1.4, 0.3, 1.2, -1.2, 1.57], [0.2, 1.2, 0.3, 0.9, -1.3, 1.57], arm.q, [0.0, 0.8, 0.6, -0.6, 0.0, 1.5]):
        q, err, rerr = ik.solve(pos, quat, q_init=np.asarray(seed, float), base_pos=arm.base, iters=300)
        if err < 0.001 and rerr < math.radians(0.5):
            if best is None or np.abs(q - arm.q).sum() < np.abs(best - arm.q).sum():
                best = q
    q0 = arm.q.copy()
    n = max(1, round(seconds * 20))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        arm.q = q0 + s * (best - q0)
        yield
    arm.pos, arm.rot = np.asarray(pos, float), np.asarray(rot, float)


def report_grasp(ctx, arm, tag):
    if not DEBUG:
        return
    p_t, R_t = tcp_actual(arm)
    p_p, R_p = pick_pose(ctx)
    R_rel = R_t.T @ R_p
    # the handle's direction (pick x) in the hand frame, and the arms' direction (pick -z)
    print(f"[grasp {tag}] pick x in hand {np.round(R_rel[:, 0], 3).tolist()} pick z in hand "
          f"{np.round(R_rel[:, 2], 3).tolist()} pick origin in hand {np.round(R_t.T @ (p_p - p_t) * 1000, 2).tolist()}",
          flush=True)


def bring_in(ctx, arm):
    """Roll the arms down and bring the pick in front of the mouth, then in over the teeth."""
    ctx.flags["look"] = True
    yield from arm.move(ROLL_AT, R_TABLE, 2.0)
    report_grasp(ctx, arm, "at roll")
    yield from roll_joints(arm, ROLL_AT, R_WORK, float(os.environ.get("FP_ROLL_S", "4.0")))
    yield from wait(0.3)
    report_grasp(ctx, arm, "rolled")
    if ctx.flags.get("upto") == "roll":
        return
    # in front of the mouth (nominal grasp: the pick 12 mm deeper in the pads than the tool point)
    out_h = np.array([X_MID_SAW - 45.0, 0.0, 30.0])
    nominal = g.to_world(out_h) + np.array([-0.0525, -0.0118, 0.013])
    yield from arm.move(nominal, R_WORK, 2.5)
    yield from wait(0.5)
    h = Holder(ctx, arm)
    if DEBUG:
        print("grasp: pick in tcp frame", np.round(h.p_rel * 1000, 2).tolist(), "R_rel\n", np.round(h.R_rel, 3),
              flush=True)
    ctx.flags["holder"] = h
    p0 = pick_h(ctx)
    above = np.array([X_MID_ENTRY, 0.0, g.Z_EDGE + 6.5])
    yield from h.go(p0, np.array([X_MID_ENTRY - 45.0, 0.0, above[2]]), 1.5)
    yield from h.go(np.array([X_MID_ENTRY - 45.0, 0.0, above[2]]), above, 3.0)
    yield from h.hold(above, 1.0)


def saw_in(ctx, arm, variant):
    """Down onto the notch and through. The hand's correction is settled in free space above the notch and then frozen
    on z: from there the arm's own stiffness is the push, set by how far the target is below the pick (PUSH_LAG while
    sawing in: a bounded push however the lip and the contact resist). The straight push (``snap``) lowers the target
    at 2 mm/s without sawing."""
    h = ctx.flags["holder"]
    ctx.flags["view"] = "close"
    start = pick_h(ctx)
    above = np.array([X_MID_ENTRY, 0.0, g.Z_EDGE + 1.0])
    yield from h.go(start, above, 1.5)
    yield from h.hold(above, 2.0, k=(0.3, 0.3, 0.3))
    z_t = above[2]
    straight = variant == "push"      # a straight push (no sawing): the pick turns in the pinch long before 8 N
    fast = variant == "fast"
    log = ctx.flags.setdefault("push_log", [])

    def note(z_t):
        S = ctx.task._STATE
        log.append((S["step"], round(pick_h(ctx)[2], 3), round(z_t, 3), round(S["F_plug"], 3),
                    round(S["contact_down"], 3), S["mode"], round(S["lip"] * 1000, 2)))

    # 1. settled in free space: freeze the yaw correction and bound the push about the free-space z correction
    h.yaw_frozen = True
    h.z_free = float(h.corr[2])
    h.y_free = float(h.corr[1])
    h.y_band = Y_BAND / 1000.0
    h.z_band = None if straight else PUSH_BAND / 1000.0
    if straight:
        # the straight push: the target lowered 3 mm/s, no sawing, the arm's stiffness the push (no integral on z)
        prev = pick_h(ctx)[2]
        for _ in range(int(60 * 20)):
            z_t -= 3.0 * 0.05
            h.aim(np.array([X_MID_ENTRY, 0.0, z_t]), k=(0.25, 0.25, 0.0), integrate=(True, True, False))
            yield
            note(z_t)
            now = pick_h(ctx)[2]
            if (prev - now > 0.5 and now < g.ZONE[0]) or now < g.PAP_Z:   # the sudden drop when the contact lets go
                break
            prev = now
    else:
        # 2. down onto the notch (no sawing) until the floss rests on the contact, then saw: the amplitude ramps up over
        # 1 s; the pick's height is closed-loop on a target that waits 0.5 s on the contact and then descends
        # LOWER_RATE (the floss's push is whatever makes it creep that fast, bounded by PUSH_BAND); the descent pauses
        # while the pick lags the target by 0.8 mm
        h.z_band = ENTRY_BAND / 1000.0                  # the front arm wedging the lip forward needs a firmer push
        for _ in range(int(20 * 20)):
            if pick_h(ctx)[2] - z_t < 0.8:
                z_t -= 0.6 * 0.05
            h.aim(np.array([X_MID_ENTRY, 0.0, z_t]), k=(0.25, 0.25, 0.25))
            yield
            note(z_t)
            if pick_h(ctx)[2] <= g.ZONE[1] + 0.4:
                break
        # unwind the push just above the contact, then down onto it gently
        for _ in range(8):
            z_t = pick_h(ctx)[2] + 0.2
            h.aim(np.array([X_MID_ENTRY, 0.0, z_t]), k=(0.25, 0.25, 0.25))
            yield
            note(z_t)
        h.z_band = PUSH_BAND / 1000.0
        for _ in range(int(10 * 20)):
            if pick_h(ctx)[2] - z_t < 0.6:
                z_t -= 0.3 * 0.05
            h.aim(np.array([X_MID_ENTRY, 0.0, z_t]), k=(0.25, 0.25, 0.25))
            yield
            note(z_t)
            if pick_h(ctx)[2] <= g.ZONE[1] + 0.1:
                break
        t_saw = 0.0
        z_t = min(z_t, pick_h(ctx)[2])
        ctx.flags["phase"] = "on_notch"
        for _ in range(int(40 * 20)):
            t_saw += 0.05
            blend = min(1.0, t_saw / 1.0)
            x_t = X_MID_SAW + blend * SAW_AMP * math.sin(2 * math.pi * SAW_HZ * t_saw)
            ph = pick_h(ctx)
            if t_saw > 0.5 and ph[2] - z_t < 0.8:
                z_t -= (LOWER_RATE if ph[2] > -0.2 else 0.15) * 0.05     # slower in the contact's last 0.3 mm
            h.aim(np.array([x_t, 0.0, z_t]), k=(0.25, 0.25, 0.25))
            yield
            note(z_t)
            if pick_h(ctx)[2] < g.ZONE[0] - PASSED_BELOW:   # (the floss sags a little below the line between its
                break                                        # tips while the contact holds it)
    ctx.flags["passed_at"] = ctx.task._STATE["step"]
    ctx.flags["phase"] = "passed"
    # after the pass: the push bounded mostly downward (the floss can never be shoved back up through the contact's
    # hold from below), the floss held where it settles between the contact and the gum
    h.z_band = None
    h.z_lo, h.z_hi = h.z_free - POST_BAND_DOWN / 1000.0, h.z_free + POST_BAND_UP / 1000.0
    p = pick_h(ctx)
    hold = np.array([X_MID_HUG, 0.0, HOLD_BELOW])
    yield from h.go(np.array([X_MID_SAW, 0.0, min(max(p[2], -2.0), -0.8)]), hold, 1.5, k=(0.25, 0.25, 0.15))
    yield from h.hold(hold, 0.8, k=(0.25, 0.25, 0.15))


def stroke_to(ctx, h, cur, tgt, k, tol=0.25, dwell_max=1.0):
    """A stroke: the target moves at STROKE_SPEED. Going down it then waits (at most dwell_max) until the floss has
    reached it (the lip's friction and the soft arm make the pick lag). Going up it never presses the contact from
    below: once the floss is within 0.15 mm of the top (or 0.25 mm of the contact) the upward correction is frozen
    and the stroke ends."""
    up = tgt[2] > cur[2]
    cur = np.array(cur, float)
    saved = h.z_hi
    if not up:
        # Down: start from where the floss is (it can coast past the top) and never add upward push on the way down
        # (the lip, springing back as the front arm rises, pushes the pick up; an upward push here drove the floss
        # back into the contact).
        cur[2] = min(cur[2], pick_h(ctx)[2])
        if h.z_lo is not None:
            h.z_hi = min(h.z_hi, h.corr[2])
    n = max(1, round(abs(tgt[2] - cur[2]) / STROKE_SPEED * 20))
    from yam_arena import ik
    for i in range(1, n + 1):
        s = ik.smoothstep(i / n)
        h.aim(cur + s * (np.asarray(tgt) - cur), k=k)
        yield
        if up and pick_h(ctx)[2] > min(tgt[2] - 0.15, g.ZONE[0] - 0.25):
            break
    if up:
        h.z_hi = min(h.z_hi, h.corr[2]) if h.z_hi is not None else h.corr[2]
        yield
        h.z_hi = saved
        return
    for _ in range(round(dwell_max * 20)):
        if abs(pick_h(ctx)[2] - tgt[2]) < tol:
            break
        h.aim(tgt, k=k)
        yield
    h.z_hi = saved


STROKE_BAND = 6.0                    # mm: the bounded push while stroking (the lip's friction, not the contact)
HUG_Y_BAND = 5.0                     # mm: the sideways correction allowed while hugging (about 1 N)


def hug_strokes(ctx, arm, side: int, hug: float):
    h = ctx.flags["holder"]
    h.z_band = None
    h.y_band = HUG_Y_BAND / 1000.0
    ctx.flags["phase"] = "hug41" if side > 0 else "hug31"
    k = (0.25, 0.15, 0.25)
    c = np.array([X_MID_HUG, 0.0, HOLD_BELOW])
    q = np.array([X_MID_HUG, side * hug, HUG_Z if hug > 0 else HOLD_BELOW])
    yield from h.go(c, q, 3.0, k=k)
    yield from h.hold(q, 1.0, k=k)
    top, bot = q.copy(), q.copy()
    top[2], bot[2] = STROKE_TOP, STROKE_BOT
    cur = q
    for i in range(STROKES):
        for tgt in (bot, top):
            yield from stroke_to(ctx, h, cur, tgt, k)
            cur = tgt
            if i == STROKES - 1 and tgt is bot:
                ctx.flags["shot"] = "stroke41" if side > 0 else "stroke31"
    back = np.array([X_MID_HUG, 0.0, HOLD_BELOW])
    cur = np.array(cur, float)
    cur[2] = min(cur[2], pick_h(ctx)[2])          # from where the floss is, with no added upward push (as a down stroke)
    saved = h.z_hi
    if h.z_lo is not None:
        h.z_hi = min(h.z_hi, h.corr[2])
    yield from h.go(cur, back, 2.0, k=k)
    yield from h.hold(back, 0.5, k=k)
    h.z_hi = saved
    h.y_band = Y_BAND / 1000.0
    ctx.flags["phase"] = "centred"


def snap_jerk(ctx, arm):
    """The snap check: once the floss is through, it is brought up under the contact and the hand drops 12 mm at once,
    snapping the floss onto the gum."""
    h = ctx.flags["holder"]
    top = np.array([X_MID_HUG, 0.0, g.ZONE[0] - 0.3])          # first up under the contact (it holds from below)
    yield from h.go(pick_h(ctx), top, 1.0)
    yield from h.hold(top, 0.5)
    h.z_band = None
    h.z_lo = h.z_hi = None
    ctx.flags["phase"] = "snap"
    tgt = np.array([X_MID_HUG, 0.0, pick_h(ctx)[2] - 12.0])
    for _ in range(int(1.5 * 20)):
        h.aim(tgt, k=(0.25, 0.25, 0.0), integrate=(True, True, False))
        yield


def press(ctx, arm):
    h = ctx.flags["holder"]
    h.z_band = None
    h.z_lo = h.z_hi = None
    p = np.array([X_MID_HUG, 0.0, HOLD_BELOW])
    tgt = np.array([X_MID_HUG, 0.0, -9.0])        # far into the gum: the push builds until it clearly passes 3 N
    yield from h.go(p, tgt, 1.0, k=(0.25, 0.25, 0.25), clamp=0.05)
    yield from h.hold(tgt, 4.0, k=(0.25, 0.25, 0.25), clamp=0.05)


def saw_out(ctx, arm):
    h = ctx.flags["holder"]
    h.z_lo = h.z_hi = None
    h.z_band = PUSH_BAND / 1000.0
    p = pick_h(ctx)
    start = np.array([X_MID_SAW, 0.0, HOLD_BELOW])
    yield from h.go(p, start, 1.0, k=(0.25, 0.25, 0.15))
    z_t = start[2]
    t = 0.0
    for _ in range(int(40 * 20)):
        t += 0.05
        z_t += RAISE_RATE * 0.05          # a straight pull: the contact lets the floss back up only at its 3 N hold
        h.aim(np.array([X_MID_SAW, 0.0, z_t]), k=(0.25, 0.25, 0.0), integrate=(True, True, False))
        yield
        if pick_h(ctx)[2] > g.ZONE[1] + 0.3:
            break
    p = pick_h(ctx)
    up = np.array([X_MID_SAW, 0.0, CLEAR_Z])
    yield from h.go(p, up, 2.0)
    ctx.flags["phase"] = "out"
    out = np.array([X_MID_SAW - 50.0, 0.0, CLEAR_Z + 10.0])
    yield from h.go(up, out, 2.5)
    yield from h.hold(out, 1.0)


LOOK = os.environ.get("FP_LOOK", "0") == "1"   # park the free left arm as a camera on the mouth (off: it shows
                                                # mostly cheek, and the close view below covers the mouth)
LOOK_Q = [-0.357, 1.412, 0.396, 0.73, 0.827, 0.505]  # left wrist camera 15 cm from the contact, 35 degrees up, from the
                                                    # front left, through the mouth opening; the hand 3.7 cm or more
                                                    # from the right hand's space and clear of the head on the way


def move_q(arm, q1, seconds: float = 2.0):
    """Generator: a smooth joint-space move to the joint angles q1."""
    from yam_arena import ik

    q0, q1 = arm.q.copy(), np.asarray(q1, float)
    n = max(1, round(seconds * 20))
    for k in range(1, n + 1):
        arm.q = q0 + ik.smoothstep(k / n) * (q1 - q0)
        yield
    arm.pos, arm.rot = ik.tcp_pose(arm.q, arm.base)


def look(ctx):
    """The left arm: once the pick is on its way to the mouth, turn the wrist camera on the mouth and stay. It touches
    nothing."""
    if not LOOK:
        return
    while not ctx.flags.get("look"):
        if ctx.flags.get("script_done"):
            return
        yield
    yield from move_q(ctx.left, LOOK_Q, 3.0)


CLOSE_EYE = np.array([-0.065, 0.032, 0.007])   # the close view's eye from the contact (m): in front, to the robot's
                                                # left, just above the floss and below the pick's handle


def close_view(ctx, gen):
    """Recorded runs: when the floss reaches the notch the top camera moves to a close mouth-level view (in front of
    the mouth, to the robot's left, 73 mm from the contact, below the handle), so the video shows the pass, the wipe
    and the withdrawal. Only the recording changes; the demo reads object poses, not images."""
    import torch

    base = ctx.emb._env.unwrapped
    cam = base.scene["top_cam"]
    o = base.scene.env_origins[0:1].to(torch.float32)
    moved = False
    for _ in gen:
        yield
        if not moved and ctx.flags.get("view") == "close":
            t = np.array(g.CONTACT) + np.array([0.0, 0.0, -0.002])
            eye = torch.tensor(np.array([t + CLOSE_EYE]), dtype=torch.float32, device=o.device) + o
            tgt = torch.tensor(np.array([t]), dtype=torch.float32, device=o.device) + o
            cam.set_world_poses_from_view(eye, tgt)
            moved = True
            print("top camera: close view at step", ctx.task._STATE["step"], flush=True)


def right_script(ctx, variant: str, upto: str):
    yield from script(ctx, variant, upto)
    ctx.flags["script_done"] = True


def script(ctx, variant: str, upto: str):
    arm = ctx.right
    ctx.flags["upto"] = upto
    yield from grasp(ctx, arm)
    if upto == "grasp":
        yield from wait(2.0)
        return
    yield from bring_in(ctx, arm)
    if upto == "roll":
        yield from wait(1.0)
        return
    if upto == "bring":
        yield from ctx.flags["holder"].hold(pick_h(ctx), 3.0)
        return
    yield from saw_in(ctx, arm, variant)
    if variant == "snap":
        yield from snap_jerk(ctx, arm)
        return
    if variant == "push":
        yield from ctx.flags["holder"].hold(np.array([X_MID_HUG, 0.0, HOLD_BELOW]), 2.0)
        return
    if variant == "press":
        yield from press(ctx, arm)
        return
    if upto == "pass":
        yield from ctx.flags["holder"].hold(np.array([X_MID_HUG, 0.0, HOLD_BELOW]), 2.0)
        return
    sides = (1,) if variant == "one_side" else (1, -1)
    for side in sides:
        yield from hug_strokes(ctx, arm, side, 0.0 if variant == "straight" else HUG)
    if variant == "leave":
        yield from ctx.flags["holder"].hold(np.array([X_MID_HUG, 0.0, HOLD_BELOW]), 2.0)
        return
    yield from saw_out(ctx, arm)


def force_log(ctx):
    """Per control step: the demo's phase and the floss's largest push on each incisor in that step (to set F_wipe)."""
    S = ctx.task._STATE
    ctx.flags.setdefault("forces", []).append((S["step"], ctx.flags.get("phase", "-"), round(S.get("on41_max", 0), 3),
                                              round(S.get("on31_max", 0), 3), S["mode"]))


def closeup_cameras(ctx, out_dir):
    """Replicator cameras near the mouth (added after start; the run's own cameras are untouched). Returns a function
    that writes one still per camera: closeup_<tag>_<view>.png."""
    import imageio
    import omni.replicator.core as rep

    base = ctx.emb._env.unwrapped
    o = base.scene.env_origins[0].cpu().numpy()
    target = g.CONTACT + o + np.array([0.0, 0.0, -0.002])
    views = {"front": np.array([-0.075, -0.030, 0.030]), "above": np.array([-0.045, 0.012, 0.070]),
             "side": np.array([-0.030, 0.065, 0.020])}
    anns = {}
    for name, off in views.items():
        cam = rep.create.camera(position=tuple(target + off), look_at=tuple(target), focal_length=35.0)
        rp = rep.create.render_product(cam, (800, 560))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        anns[name] = ann
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    def shoot(tag):
        for _ in range(2):
            rep.orchestrator.step(rt_subframes=8, pause_timeline=False)
        for name, ann in anns.items():
            img = np.asarray(ann.get_data())
            if img.size:
                imageio.imwrite(out / f"closeup_{tag}_{name}.png", img[..., :3])

    shoot("start")
    return shoot


def grasp_angles(ctx):
    """The pick's turn in the hand since the grasp was measured: (roll about the handle, pitch, yaw), degrees."""
    h = ctx.flags.get("holder")
    if h is None:
        return None
    p_t, R_t = tcp_actual(h.arm)
    R_p = pick_pose(ctx)[1]
    D = (R_t @ h.R_rel).T @ R_p                      # the pick now, in the pick frame it had when measured
    roll = math.degrees(math.atan2(D[2, 1], D[2, 2]))
    pitch = math.degrees(math.asin(float(np.clip(-D[2, 0], -1, 1))))
    yaw = math.degrees(math.atan2(D[1, 0], D[0, 0]))
    return round(roll, 1), round(pitch, 1), round(yaw, 1)


def log(ctx, step, info) -> str:
    T = ctx.task
    S = T._STATE
    ph = pick_h(ctx)
    fl = T.FL
    tr, _ = tcp_actual(ctx.right)
    return (f"pickH {np.round(ph, 2).tolist()} cmdErr {np.round((tr - ctx.right.pos) * 1000, 1).tolist()} "
            f"mode {S['mode']} zs {S['zs']:.2f} Fplug {S['F_plug']:.2f} T {S['T']:.2f} "
            f"on41 {fl.on.get('t41', 0):.2f} on31 {fl.on.get('t31', 0):.2f} gum {fl.on.get('gum', 0):.2f} "
            f"press {S['press_now']:.2f} (contact {S['contact_down']:.2f}) lip {S['lip'] * 1000:.2f} "
            f"cov {{41: {S['cov']['t41']:.2f}, 31: {S['cov']['t31']:.2f}}} out {S['out']} rung {S['rung']} "
            f"land {[round(x, 2) for x in T.RU.landings][-3:]} why {S['why']} G {grasp_angles(ctx)} C {S.get('contacts')} "
            f"fnet {S.get('floss_net')}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=3600)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--variant", default="full",
                    choices=["full", "nothing", "snap", "push", "press", "one_side", "straight", "leave"])
    ap.add_argument("--upto", default="full", choices=["grasp", "roll", "bring", "pass", "full"])
    ap.add_argument("--closeup", action="store_true", help="also render close-up stills of the mouth at key phases")
    ap.add_argument("--keep-top-cam", action="store_true", help="don't move the top camera to the close view")
    a = ap.parse_args()
    print("variant", a.variant, "upto", a.upto, flush=True)
    if a.variant == "nothing":
        def scr(ctx):
            yield from wait(5.0)
    else:
        def scr(ctx):
            shoot = closeup_cameras(ctx, a.out) if a.closeup else None
            done = set()
            steps = together(right_script(ctx, a.variant, a.upto), look(ctx))
            if not (a.no_record or a.keep_top_cam):
                steps = close_view(ctx, steps)
            for _ in steps:
                force_log(ctx)
                yield
                tag = ctx.flags.get("shot") or ctx.flags.get("phase")
                if shoot and tag in ("on_notch", "passed", "stroke41", "stroke31", "out") and tag not in done:
                    done.add(tag)
                    shoot(tag)
            if DEBUG:
                for row in ctx.flags.get("forces", []):
                    print("force", *row, flush=True)
                for row in ctx.flags.get("push_log", []):
                    print("push", *row, flush=True)
    res = run(a.task, a.out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
              log_every=int(os.environ.get("FP_LOG_EVERY", "10")), hold_s=1.5)
