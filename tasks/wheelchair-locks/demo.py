"""Scripted full-score demo (reads object poses from the simulator: the chair's pose and its two wheel-lock levers'
angles).

1. The turn, both hands: the left hand pinches the near seat rail's front end (straight down, closing across the tube),
   the right hand the near push handle's foam grip; both are aimed each step at their grips on the planned pose a few
   degrees further round than the chair measures, so the chair reverses through a quarter turn about a point on its
   rear axle line 15 cm from the hubs' midpoint.
2. At 32 degrees round the right hand moves to the near desk arm's front end; at 52 degrees the left hand moves to the
   far seat rail's front end and the right hand to the near one; one hand always holds the chair.
3. On round to the reference pose (facing the bed's foot, the hubs' midpoint at (0.75, -0.045)), then a short roll
   forward so the casters trail.
4. The bed-side lock: the left hand holds the chair; the right hand, open and pointing down, lowers its rear finger into
   the gap between the knob and the tyre, closes on the knob and turns it forward about the lever's pivot, aimed well
   past it (the arm gives way under load), until the lever goes over centre.
5. The right hand pinches the bed-side seat rail's front end and holds; the left hand sets the far lock the same way.
6. Both hands open, lift and go back to the ready pose; the held test then runs.

    python demo.py task.py demo_out [--max-steps 3600] [--no-record] [--variant NAME] [--close-view] [--hold 15]

Variants (scorer checks), each printing its mode:
- ``no_lock``: the full turn, no locks set, arms back to ready;
- ``one_lock``: the full turn, only the bed-side lock set;
- ``wedge``: the chair rolled straight back against the bed's frame, no locks set;
- ``nearly``: the turn stopped 15 degrees short (the far knob comes within reach at 75 degrees), both locks set;
- ``lock_start``: a scorer check: both levers set over centre by task code at the start pose (the far knob is out of
  the arms' reach there), the arms raised and returned to ready;
- ``nothing``: the arms stay still.
Writes one MP4 per camera, setup.png (the first top-camera frame), result.json and extra.json.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, together, turn, wait  # noqa: E402

DEBUG = bool(os.environ.get("WC_DEBUG"))
TIP = 0.013                         # fingertips past the tool point
OPEN_HALF = 0.047                   # each open finger's inner face from the tool point
RATE = math.radians(float(os.environ.get("WC_RATE", "7.0")))     # turn rate, rad/s
TURN_R = float(np.linalg.norm(g.START_M - g.TURN_C))             # the planned turn's radius about the axle line
K_PATH = float(os.environ.get("WC_K_PATH", "1.0"))               # path feedback: extra aim per m off the path
HOLD_KI = float(os.environ.get("WC_HOLD_KI", "0.4"))
HOLD_KP = float(os.environ.get("WC_HOLD_KP", "2.0"))             # holding hand: aim offset per step per m pushed off
HOLD_CAP = float(os.environ.get("WC_HOLD_CAP", "0.18"))          # ... up to this far (kept within reach)
PARK_FWD = float(os.environ.get("WC_PARK_FWD", "0.04"))         # rolled this far on forward at the end, so the casters
                                                                # swing round to trail behind
LEAD = math.radians(float(os.environ.get("WC_LEAD", "8.0")))     # turn: aim up to this far ahead of the measured chair
HAND1 = math.radians(float(os.environ.get("WC_HAND1", "33")))   # right hand: push handle -> near desk arm
HAND2 = math.radians(float(os.environ.get("WC_HAND2", "52")))   # left: near rail -> far rail; right: -> near rail
PUSH_LEAD = float(os.environ.get("WC_PUSH_LEAD", "0.15"))        # lever: the aim goes up to this far past the knob
ROT_LEAD = float(os.environ.get("WC_ROT_LEAD", "20"))            # ... and the hand turns up to this many degrees ahead
PUSH_RAMP = float(os.environ.get("WC_PUSH_RAMP", "0.08"))        # m/s the aim's lead grows
KNOB_DZ = float(os.environ.get("WC_KNOB_DZ", "-0.006"))          # fingertips this far above the knob's centre
PINCH_OPEN = float(os.environ.get("WC_PINCH_OPEN", "0.050"))     # fingers this far apart coming down on the knob
TRAP = float(os.environ.get("WC_TRAP", "0.050"))                 # fingers this far apart round the knob while pushing
PITCH = math.radians(float(os.environ.get("WC_PITCH", "0")))      # the hand's tilt (negative: fingertips forward)
CONT = (9.0, 9.0, 9.0, 3.0, 3.0, 3.0)                           # continuous (rated) torques, DM4340 / DM4310, N m
KNOB_OUT = float(os.environ.get("WC_KNOB_OUT", "0.012"))         # the hand this far outboard of the knob's middle ...
KNOB_ROLL = math.radians(float(os.environ.get("WC_KNOB_ROLL", "10")))   # ... and leaning out, clear of the seat rail


def R_from(a, c) -> np.ndarray:
    """Gripper rotation with approach ``a`` (its z) and closing axis ``c`` (its x)."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def tcp_actual(arm):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints, env

    return ik.tcp_pose(arm_joints(env(), arm.side, ARM_JOINTS), arm.base)


def wrap(a: float) -> float:
    return math.remainder(a, 2 * math.pi)


class Chair:
    """The measured chair: its frame's pose, heading, how far round the turn, the levers."""

    def __init__(self, ctx):
        T = ctx.task
        p, R, _, _ = T._poses(ctx.emb._env)
        self.p, self.R = p, R
        self.yaw = math.atan2(R[1, 0], R[0, 0])
        self.m = p[:2].copy()
        self.phi = wrap(self.yaw - g.START_YAW)
        self.levers = {s: math.radians(v) for s, v in T._STATE["levers"].items()}
        self.locked = dict(T._STATE["locked"])

    def world(self, pt) -> np.ndarray:
        return self.p + self.R @ np.asarray(pt, float)


TOWARD = {"fwd": 0.0, "lft": math.pi / 2, "bck": math.pi, "rgt": -math.pi / 2}


def grip_rot(yaw: float, flip: float, tilt: float = 0.0, toward: str = "fwd") -> np.ndarray:
    """Pointing down (tipped ``tilt`` toward the chair's ``toward`` side), closing across the chair (along its y)."""
    t = yaw + TOWARD[toward]
    a = np.array([math.cos(t) * math.sin(tilt), math.sin(t) * math.sin(tilt), -math.cos(tilt)])
    c = yaw + math.pi / 2 + flip
    return R_from(a, [math.cos(c), math.sin(c), 0.0])


class Hold:
    """One hand's grip on the chair: a point in the chair frame, its closing direction and tilt."""

    def __init__(self, arm, pt, flip, tilt=0.0, toward="fwd"):
        self.arm, self.pt, self.flip, self.tilt, self.toward = arm, np.asarray(pt, float), flip, tilt, toward

    def aim_at(self, m, yaw):
        self.arm.aim(g.chair_to_world(self.pt, m, yaw), grip_rot(yaw, self.flip, self.tilt, self.toward))

    def recal(self, ctx):
        """Re-measure where the hand holds the chair (a grip can creep along a tube while the chair is turned)."""
        ch = Chair(ctx)
        self.pt = ch.R.T @ (tcp_actual(self.arm)[0] - ch.p)


def grasp(ctx, arm, pt, label, flip=0.0, tilt=0.0, toward="fwd", close=0.0, above=0.08, seconds=1.5):
    """Open, come down on the chair-frame point ``pt`` from above (where the measured chair puts it), close."""
    tilt = math.radians(tilt)
    ch = Chair(ctx)
    R = grip_rot(ch.yaw, flip, tilt, toward)
    up = -R[:, 2] * above                                   # back along the approach
    yield from arm.move_joints(ch.world(pt) + up, R, seconds, grip=1.0)
    corr = np.zeros(3)
    for _ in range(2):
        yield from wait(0.15)
        ch = Chair(ctx)
        R = grip_rot(ch.yaw, flip, tilt, toward)
        target = ch.world(pt) + up
        err = target - tcp_actual(arm)[0]
        if np.linalg.norm(err) < 0.002:
            break
        corr = corr + err
        yield from arm.move(target + corr, R, 0.3)
    yield from arm.move(ch.world(pt) + corr, R, 0.7)
    yield from arm.set_grip(close, 0.45)
    yield from wait(0.1)
    ch = Chair(ctx)
    tcp = tcp_actual(arm)[0]
    err = ch.world(pt) - tcp
    pt = ch.R.T @ (tcp - ch.p)                              # hold where the hand actually is on the chair
    ctx.flags.setdefault("grasps", []).append(dict(label=label, step=ctx.task._STATE["step"], flip=round(flip, 2),
                                                   corr_mm=np.round(corr * 1000, 1).tolist(),
                                                   err_mm=np.round(err * 1000, 1).tolist()))
    print(f"PHASE grasp {label} step {ctx.task._STATE['step']} corr_mm {np.round(corr * 1000, 1)} err_mm "
          f"{np.round(err * 1000, 1)}", flush=True)
    return Hold(arm, pt, flip, tilt, toward)


def release(arm, up=0.08):
    yield from arm.set_grip(1.0, 0.35)
    p, _ = tcp_actual(arm)
    arm.pos = p
    yield from arm.move(p - arm.rot[:, 2] * up, arm.rot, 0.5)


def turn_to(ctx, holds, phi_end, rate=None):
    """Swing the chair on round to ``phi_end``. Every step the holding hands are aimed at their grips on the planned
    pose a few degrees further round than the chair measures, shifted back toward the path by as much as the chair is
    off it (the arms give way under load, so the aim must lead)."""
    rate = RATE if rate is None else rate
    for hd in holds:
        hd.recal(ctx)
    ch = Chair(ctx)
    ref = ch.phi
    n, best, since = 0, ch.phi, 0
    while True:
        ch = Chair(ctx)
        ref = min(phi_end + LEAD / 2, ref + rate / 20.0)
        h = np.array([math.cos(ch.yaw), math.sin(ch.yaw)])
        d = ch.m - g.turn_pose(ch.phi)[0]                  # off the planned path now
        cmd = float(np.clip(ref, ch.phi - LEAD, ch.phi + LEAD))
        m_c, yaw_c = g.turn_pose(cmd)
        m_t = m_c - K_PATH * d
        for hd in holds:
            hd.aim_at(m_t, yaw_c)
        r = 0.0
        dphi = cmd - ch.phi
        yield
        n += 1
        if ch.phi > best + math.radians(0.3):
            best, since = ch.phi, 0
        else:
            since += 1
        if ch.phi >= phi_end - math.radians(1.5) or since > 40 or n > 20 * 30:
            break
        if DEBUG and n % 10 == 0:
            errs = []
            for hd in holds:
                e = hd.arm.pos - tcp_actual(hd.arm)[0]
                errs.append(f"{hd.arm.side[0]} lag {np.round(e * 1000, 0).tolist()} along -h {float(e[:2] @ -h) * 1000:.0f}")
            print(f"TURN step {ctx.task._STATE['step']} phi {math.degrees(ch.phi):.1f} ref {math.degrees(ref):.1f} "
                  f"dphi {math.degrees(dphi):.1f} r {r * 100:.1f} cm off-path {np.round(d * 1000, 1).tolist()} mm "
                  f"| {' | '.join(errs)}", flush=True)
    # hold the chair where it is
    ch = Chair(ctx)
    for _ in range(6):
        for hd in holds:
            hd.aim_at(ch.m, ch.yaw)
        yield
    ch = Chair(ctx)
    d = ch.m - g.turn_pose(ch.phi)[0]
    stalled = since > 40
    print(f"PHASE turned to {math.degrees(ch.phi):.1f} (aim {math.degrees(phi_end):.1f}) off-path "
          f"{np.round(d * 1000, 1).tolist()} mm step {ctx.task._STATE['step']}{' STALLED' if stalled else ''}",
          flush=True)
    ctx.flags.setdefault("turns", []).append(dict(to=round(math.degrees(ch.phi), 2), aim=round(math.degrees(phi_end), 1),
                                                  off_mm=np.round(d * 1000, 1).tolist(), stalled=stalled,
                                                  step=ctx.task._STATE["step"]))


def roll_to(ctx, holds, m_goal, tol=0.010, lead=0.10):
    """Roll the chair straight along its heading until its hubs' midpoint is level with ``m_goal`` (within ``tol``
    along the heading): the hands aimed up to ``lead`` beyond, heading kept."""
    for hd in holds:
        hd.recal(ctx)
    ch = Chair(ctx)
    yaw0 = ch.yaw
    h = np.array([math.cos(yaw0), math.sin(yaw0)])
    n = 0
    while n < 20 * 10:
        ch = Chair(ctx)
        e = float((m_goal - ch.m) @ h)
        if abs(e) < tol:
            break
        step = float(np.clip(e, -lead, lead))
        for hd in holds:
            hd.aim_at(ch.m + h * step, yaw0)
        yield
        n += 1
    ch = Chair(ctx)
    for _ in range(6):
        for hd in holds:
            hd.aim_at(ch.m, ch.yaw)
        yield
    e = float((m_goal - Chair(ctx).m) @ h)
    print(f"PHASE rolled: {e * 1000:.1f} mm left along the heading, step {ctx.task._STATE['step']}", flush=True)


def hold_loop(ctx, hd, flag: str):
    """While ``ctx.flags[flag]`` is set, hold the chair where it was when this began: the hand's aim is its grip there
    plus an offset that integrates how far the chair has been pushed off (the arm gives about 1 cm per newton, so a
    fixed aim would let a 20 N push carry the chair 20 cm)."""
    hd.recal(ctx)
    ch = Chair(ctx)
    m0, yaw0 = ch.m.copy(), ch.yaw
    ref = g.chair_to_world(hd.pt, m0, yaw0)
    off = np.zeros(3)
    worst = 0.0
    while ctx.flags.get(flag):
        ch = Chair(ctx)
        now = g.chair_to_world(hd.pt, ch.m, ch.yaw)
        err = ref - now
        err[2] = 0.0
        nrm_e = np.linalg.norm(err)
        err = err * max(0.0, nrm_e - 0.003) / max(nrm_e, 1e-9)    # a 3 mm deadband
        worst = max(worst, float(np.linalg.norm(ch.m - m0)))
        off = off + HOLD_KI * err
        aim_off = off + HOLD_KP * err
        nrm = np.linalg.norm(aim_off)
        if nrm > HOLD_CAP:
            aim_off *= HOLD_CAP / nrm
            off = aim_off - HOLD_KP * err
        hd.arm.aim(ref + aim_off, grip_rot(yaw0, hd.flip, hd.tilt, hd.toward))
        yield
    for _ in range(10):                                      # ease off: aim where the hand now holds the chair
        hd.recal(ctx)
        ch = Chair(ctx)
        hd.aim_at(ch.m, ch.yaw)
        yield
    ctx.flags.setdefault("holds", []).append(dict(moved_mm=round(worst * 1000, 1), offset_cm=round(float(np.linalg.norm(off)) * 100, 1),
                                                  step=ctx.task._STATE["step"]))
    print(f"PHASE hold done: chair moved at most {worst * 1000:.1f} mm, aim offset {np.linalg.norm(off) * 100:.1f} cm",
          flush=True)


def push_lever(ctx, arm, side: str):
    """Set one wheel lock: the hand, pointing down and leaning a little outboard, its fingers 5 cm apart along the
    chair's heading, comes down with the rear finger in the gap between the knob and the tyre, the knob between the
    fingers. The rear finger pushes the knob forward along its arc, the hand aimed a growing distance ahead of it (the
    arm gives way under load), until the lever goes over centre; then the hand lifts straight off."""
    s = 1.0 if side == "l" else -1.0
    ch = Chair(ctx)
    f = ch.R[:, 0].copy()
    f[2] = 0.0
    f /= np.linalg.norm(f)
    cy = ch.R[:, 1].copy()                                  # the lever turns about the chair's y axis
    out = cy * s
    a = np.array([0.0, 0.0, -math.cos(KNOB_ROLL)]) - math.sin(KNOB_ROLL) * out
    a = math.cos(PITCH) * a - math.sin(PITCH) * f           # the fingertips tipped forward: the rear finger's face
    a /= np.linalg.norm(a)                                  # meets the knob facing forward and up, so a push can't ride
                                                            # it up and over the knob
    th0 = ch.levers[side]
    from yam_arena import ik

    best, flip = None, 0.0
    for fl in (0.0, math.pi):
        Rf = R_from(a, f if fl == 0.0 else -f)
        q, err, rerr = ik.solve(ch.world(g.knob_at(th0, s)), ik.matrix_to_quat_xyzw(Rf), q_init=arm.q,
                                base_pos=arm.base, iters=200)
        key = (not (err < 0.002 and rerr < math.radians(1)), float(np.abs(q - arm.q).sum()))
        if best is None or key < best[0]:
            best, flip = (key, fl), fl
    R0 = R_from(a, f if flip == 0.0 else -f)
    knob0 = ch.world(g.knob_at(th0, s))
    # fingers 5 cm apart, the rear one's inner face 3 mm behind the knob
    start = knob0 + f * (PINCH_OPEN / 2 - g.KNOB_R - 0.003) + [0.0, 0.0, KNOB_DZ + TIP] + out * KNOB_OUT
    open_g = PINCH_OPEN / 0.094
    def moved():
        return f"chair moved {np.round((Chair(ctx).m - ch.m) * 1000, 0).tolist()} mm"

    rel = start - knob0                                     # the tool point from the knob, held as the chair moves

    cref = Chair(ctx)                                       # the chair as it stands now: a fixed reference, so a
                                                            # touch that nudges the chair doesn't drag the aim along

    def knob_now():
        return cref.world(g.knob_at(Chair(ctx).levers[side], s))

    above = knob_now() + rel + [0, 0, 0.10]
    if np.linalg.norm(tcp_actual(arm)[0] - above) < 0.30:
        arm.pos = tcp_actual(arm)[0]
        yield from arm.move(above, R0, 1.5, grip=open_g)    # straight there, nothing swept
    else:
        yield from arm.move_joints(above, R0, 1.8, grip=open_g)
    yield from wait(0.3)
    corr = np.zeros(3)
    for _ in range(2):
        e = knob_now() + rel + [0, 0, 0.10] - tcp_actual(arm)[0]
        corr = corr + (e if np.linalg.norm(e) < 0.02 else 0.0)
        arm.aim(knob_now() + rel + [0, 0, 0.10] + corr, R0)
        yield from wait(0.2)
    # servo the rear finger's inner face to 1.5 mm behind the knob, from the arm's own joint angles, 5 cm above it
    for it in range(5):
        hover = [0, 0, 0.05]
        for _ in range(8):
            arm.aim(knob_now() + rel + hover + corr, R0)
            yield
        yield from wait(0.15)
        fg = finger_geom(ctx, arm.side, side)
        ex = (-(g.KNOB_R + 0.0015)) - fg["rear_inner_x"]          # chair x: where it should be minus where it is
        ey = KNOB_OUT - fg["rear_mid_y"]
        corr = corr + cref.R @ np.array([ex, ey * s, 0.0])
        if DEBUG:
            print(f"PHASE lever {side} servo {it}: rear inner face {fg['rear_inner_x'] * 1000:.1f} mm, mid y "
                  f"{fg['rear_mid_y'] * 1000:.1f} mm, fix {ex * 1000:.1f}/{ey * 1000:.1f} mm", flush=True)
        if abs(ex) < 0.0015 and abs(ey) < 0.003:
            break
    n_down = 30
    for k in range(n_down + 6):                               # down the last 5 cm
        dz = 0.05 * max(0.0, 1.0 - (k + 1) / n_down)
        arm.aim(knob_now() + rel + [0, 0, dz] + corr, R0)
        yield
    for _ in range(2):                                        # fingertips to KNOB_DZ from the knob's centre
        yield from wait(0.15)
        fg = finger_geom(ctx, arm.side, side)
        corr = corr + np.array([0.0, 0.0, KNOB_DZ - fg["rear_z"]])
        arm.aim(knob_now() + rel + corr, R0)
    fp = finger_report(ctx, arm.side, side)
    fg = finger_geom(ctx, arm.side, side)
    ctx.flags.setdefault("finger_access", {})[side] = dict(rear_inner_mm=round(fg["rear_inner_x"] * 1000, 1),
                                                           tip_below_centre_mm=round(-fg["rear_z"] * 1000, 1),
                                                           fingers_to_tyre_mm=round(fg["tyre"] * 1000, 1))
    print(f"PHASE lever {side} down: {moved()}, opening {finger_gap(arm.side) * 1000:.1f} mm, fingers to tyre "
          f"{fg['tyre'] * 1000:.1f} mm, rear finger {fp}", flush=True)
    start, knob0 = knob_now() + rel, knob_now()
    # pinch the knob; then turn the hand with the lever about its pivot, aimed ahead of it in angle and along the
    # knob's path (the arm gives way under load), until the lever goes over centre
    yield from arm.set_grip(0.0, 0.4)
    yield from wait(0.2)

    def lever_pose(theta):
        M = np.eye(4)
        M[:3, :3] = cref.R @ g.rot_y(theta - g.TH_OFF)
        M[:3, 3] = cref.world(g.PIVOT * np.array([1.0, s, 1.0]))
        return M

    th_p = Chair(ctx).levers[side]
    tcp0, rot0 = tcp_actual(arm)
    H = np.eye(4)
    H[:3, :3], H[:3, 3] = rot0, tcp0
    in_lever = np.linalg.inv(lever_pose(th_p)) @ H            # the hand in the lever's frame, as pinched
    print(f"PHASE lever {side} pinched: lever {math.degrees(th_p):.1f} deg, opening {finger_gap(arm.side) * 1000:.1f} mm",
          flush=True)
    lead, dth, n = 0.0, 0.0, 0
    T = ctx.task
    tq_log = []
    while n < 20 * 8:
        ch2 = Chair(ctx)
        th = ch2.levers[side]
        if ch2.locked[side]:
            break
        lead = min(PUSH_LEAD, lead + PUSH_RAMP / 20.0)
        dth = min(math.radians(ROT_LEAD), dth + math.radians(ROT_LEAD) / 30.0)
        th_c = min(th + dth, g.TH_ON + math.radians(4.0))
        Tc = lever_pose(th_c) @ in_lever
        tang = cref.R @ np.array([math.cos(th), 0.0, -math.sin(th)])
        arm.aim(Tc[:3, 3] + tang * lead, Tc[:3, :3])
        yield
        n += 1
        tq = joint_torques(arm.side)
        tq_log.append(np.abs(tq))
        if DEBUG and n % 5 == 0:
            print(f"PUSH {side} n {n} lever {math.degrees(th):.1f} aim {math.degrees(th_c):.1f} lead {lead * 100:.1f} cm "
                  f"chair moved {np.linalg.norm(ch2.m - ch.m) * 1000:.0f} mm torques {np.round(tq, 2).tolist()}",
                  flush=True)
    tl = np.array(tq_log) if tq_log else np.zeros((1, 6))
    ch3 = Chair(ctx)
    ctx.flags.setdefault("levers", {})[side] = dict(steps=n, lead_cm=round(lead * 100, 1),
                                                    lever_deg=round(math.degrees(ch3.levers[side]), 2),
                                                    locked=ch3.locked[side],
                                                    peak_torque=np.round(tl.max(0), 2).tolist(),
                                                    rms_torque=np.round(np.sqrt((tl ** 2).mean(0)), 2).tolist(),
                                                    s_over_cont=[round(float((tl[:, i] > c).sum()) / 20.0, 2)
                                                                 for i, c in enumerate(CONT)])
    print(f"PHASE lever {side} done n {n} lead {lead * 100:.1f} cm locked {ch3.locked[side]} step "
          f"{T._STATE['step']} torques {json.dumps(ctx.flags['levers'][side])}", flush=True)
    p, r_now = tcp_actual(arm)
    arm.pos, arm.rot = p, r_now
    yield from arm.move(p, r_now, 0.1)
    yield from arm.set_grip(open_g, 0.3)
    yield from arm.move(p + [0, 0, 0.10], r_now, 0.6)


def finger_geom(ctx, arm_side: str, side: str) -> dict:
    """The rear finger (the one behind the knob) from the knob's centre in the chair's axes: its inner face's x (the
    face toward the knob) and its mid y (outboard +), from the arm's measured joints (forward kinematics)."""
    from scenekit import robot_link_poses

    poses = robot_link_poses(ctx.emb._env)[arm_side]
    ch = Chair(ctx)
    s = 1.0 if side == "l" else -1.0
    k = ch.world(g.knob_at(ch.levers[side], s))
    fingers = []
    for n in ("tip_left", "tip_right"):
        P = ctx.task._robot_points()[n][0]
        W = P @ poses[n][:3, :3].T + poses[n][:3, 3]
        loc = (W - k) @ ch.R
        loc[:, 1] *= s
        lo = loc[loc[:, 2] < loc[:, 2].min() + 0.016]
        fingers.append(lo)
    rear = min(fingers, key=lambda L: L[:, 0].mean())
    _, _, Tb, _ = ctx.task._poses(ctx.emb._env)
    W = np.vstack([ctx.task._robot_points()[n][0] @ poses[n][:3, :3].T + poses[n][:3, 3] for n in ("tip_left", "tip_right")])
    tyre = ctx.task.dist_to_body(W, f"wheel_{side}", Tb[f"wheel_{side}"])
    return dict(rear_inner_x=float(rear[:, 0].max()), rear_mid_y=float((rear[:, 1].min() + rear[:, 1].max()) / 2),
                rear_z=float(rear[:, 2].min()), tyre=float(tyre))


def finger_report(ctx, arm_side: str, side: str) -> str:
    """Where the fingers' lowest 2 cm are, from the knob's centre, in the chair's axes (mm): x forward, y outboard."""
    from scenekit import robot_link_poses

    poses = robot_link_poses(ctx.emb._env)[arm_side]
    ch = Chair(ctx)
    s = 1.0 if side == "l" else -1.0
    k = ch.world(g.knob_at(ch.levers[side], s))
    out = []
    for n in ("tip_left", "tip_right"):
        P = ctx.task._robot_points()[n][0]
        W = P @ poses[n][:3, :3].T + poses[n][:3, 3]
        loc = (W - k) @ ch.R
        loc[:, 1] *= s
        lo = loc[loc[:, 2] < loc[:, 2].min() + 0.02]
        out.append(f"{n} x {lo[:, 0].min() * 1000:.0f}..{lo[:, 0].max() * 1000:.0f} y {lo[:, 1].min() * 1000:.0f}.."
                   f"{lo[:, 1].max() * 1000:.0f} z {lo[:, 2].min() * 1000:.0f}")
    return "; ".join(out)


def finger_gap(side: str) -> float:
    from yam_arena.constants import FINGER_JOINTS

    from scenekit import arm_joints, env

    return float(-arm_joints(env(), side, FINGER_JOINTS).sum())


def joint_torques(side: str) -> np.ndarray:
    """The arm's joint torques now: the drives' PD effort plus the gravity feed-forward (N m), clipped to the limits."""
    from yam_arena.constants import ARM_JOINTS

    from scenekit import env

    robot = env().unwrapped.scene[f"{side}_robot"]
    ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
    t = getattr(robot.data, "applied_torque", None)
    t = getattr(t, "torch", t)
    return t[0, ids].detach().cpu().numpy().astype(float)


def gravity_torques(side: str) -> np.ndarray:
    """The torque gravity alone puts on each arm joint in the arm's present pose (N m)."""
    import torch
    import warp as wp
    from yam_arena.constants import ARM_JOINTS

    from scenekit import env

    robot = env().unwrapped.scene[f"{side}_robot"]
    ids = robot.find_joints(ARM_JOINTS, preserve_order=True)[0]
    g_ = robot.root_physx_view.get_gravity_compensation_forces()
    g_ = g_ if isinstance(g_, torch.Tensor) else wp.to_torch(g_)
    return g_.reshape(g_.shape[0], -1)[0, ids].detach().cpu().numpy().astype(float)


def lock_held(ctx, arm, side, hd):
    """One hand sets a lock while the other holds the chair."""
    ctx.flags["hold"] = True

    def push():
        for _ in push_lever(ctx, arm, side):
            if Chair(ctx).locked[side]:
                ctx.flags["hold"] = False                   # locked: the other hand can ease off
            yield
        ctx.flags["hold"] = False

    yield from together(hold_loop(ctx, hd, "hold"), push())


def go_home_both(ctx):
    yield from together(ctx.left.go_home(2.5), ctx.right.go_home(2.5))


# -- the script --------------------------------------------------------------------------------------------------------
def script(ctx, variant: str):
    print(f"MODE variant={variant}", flush=True)
    L, Rr = ctx.left, ctx.right
    T = ctx.task
    if variant == "wedge":
        # roll the free chair straight back until its rear tyres stop against the bed's frame, then let go
        hl = yield from grasp(ctx, L, g.rail_front(1.0), "left near rail front")
        hr = yield from grasp(ctx, Rr, g.handle_grip(1.0), "right near handle", flip=math.pi, tilt=20.0, toward="rgt")
        for hd in (hl, hr):
            hd.recal(ctx)
        ch = Chair(ctx)
        yaw, m0 = ch.yaw, ch.m.copy()
        back = -np.array([math.cos(yaw), math.sin(yaw)])
        best, since = 0.0, 0
        for k in range(20 * 12):
            ch = Chair(ctx)
            d = float(np.dot(ch.m - m0, back))
            for hd in (hl, hr):
                hd.aim_at(m0 + back * min(d + 0.06, 0.40), yaw)
            yield
            if d > best + 0.002:
                best, since = d, 0
            else:
                since += 1
            if since > 30 and d > 0.10:
                break
        print(f"PHASE wedged: rolled back {d * 100:.1f} cm, stopped against the bed, step {T._STATE['step']}", flush=True)
        ctx.flags["wedge_cm"] = round(d * 100, 1)
        yield from together(release(L), release(Rr))
        yield from go_home_both(ctx)
        return
    if variant == "lock_start":
        # scorer check, not a robot strategy: both levers set over centre by task code at the start pose (the far knob
        # is out of the arms' reach until the chair is 75 degrees round), the arms leave their ready pose and return
        from scenekit import env, write_joint_positions

        q = T._np(T._SUB["chair"].data.joint_pos)[0]
        for side in ("l", "r"):
            T._set_lock(env(), side, True, float(q[T._SUB["jid"][f"axle_{side}"]]))
        write_joint_positions(env(), "wheelchair", {"lock_joint_l": T.Q_ON, "lock_joint_r": T.Q_ON})
        print(f"PHASE levers set by task code at step {T._STATE['step']}", flush=True)
        up = np.array([0.0, 0.0, 0.06])
        yield from together(L.move(L.pos + up, L.rot, 1.0), Rr.move(Rr.pos + up, Rr.rot, 1.0))
        yield from go_home_both(ctx)
        return
    end = {"nearly": math.radians(75.0)}.get(variant, math.pi / 2)
    # 1. the turn, part one: left on the near rail's front end, right on the near push handle (tipped 20 degrees)
    hl = yield from grasp(ctx, L, g.rail_front(1.0), "left near rail front")
    hr = yield from grasp(ctx, Rr, g.handle_grip(1.0), "right near handle", flip=math.pi, tilt=20.0, toward="rgt")
    yield from turn_to(ctx, [hl, hr], min(end, HAND1))
    # 2. the right hand moves to the near desk arm's front end; on round
    yield from release(Rr)
    hr = yield from grasp(ctx, Rr, g.arm_front(1.0), "right near arm front")
    yield from turn_to(ctx, [hl, hr], min(end, HAND2))
    # 3. the left hand moves to the far seat rail's front end, the right to the near one; on round to the end
    yield from release(L)
    hl = yield from grasp(ctx, L, g.rail_front(-1.0), "left far rail front")
    yield from release(Rr)
    hr = yield from grasp(ctx, Rr, g.rail_front(1.0), "right near rail front")
    if end > HAND2:
        yield from turn_to(ctx, [hl, hr], end)
    m_end, yaw_end = g.turn_pose(end)
    yield from roll_to(ctx, [hl, hr], m_end + PARK_FWD * np.array([math.cos(yaw_end), math.sin(yaw_end)]))
    ctx.flags["turned"] = dict(phi=round(math.degrees(Chair(ctx).phi), 2), step=T._STATE["step"])
    if variant != "no_lock":
        # 4. the bed-side lock, the left hand holding the chair
        yield from release(Rr)
        yield from lock_held(ctx, Rr, "l", hl)
        if variant != "one_lock":
            # 5. the right hand holds the bed-side rail again; the left sets the far lock
            hr = yield from grasp(ctx, Rr, g.rail_front(1.0), "right bed-side rail front")
            yield from release(L)
            yield from lock_held(ctx, L, "r", hr)
    # 6. both hands off, back to ready
    yield from together(release(L), release(Rr))
    yield from go_home_both(ctx)
    print("FLAGS", json.dumps(ctx.flags), flush=True)


TORQUE_LOG: dict = {"left": [], "right": []}
GRAVITY_LOG: dict = {"left": [], "right": []}


def record_torques(gen):
    """Pass the script through, logging both arms' joint torques every control step."""
    for _ in gen:
        for side in ("left", "right"):
            TORQUE_LOG[side].append(np.abs(joint_torques(side)))
            try:
                GRAVITY_LOG[side].append(np.abs(gravity_torques(side)))
            except Exception as e:                  # noqa: BLE001
                GRAVITY_LOG.setdefault("error", repr(e)[:200])
        yield


def torque_summary() -> dict:
    out = {}
    for side, rows in TORQUE_LOG.items():
        if not rows:
            continue
        A = np.array(rows)
        out[side] = dict(peak=np.round(A.max(0), 2).tolist(), rms=np.round(np.sqrt((A ** 2).mean(0)), 2).tolist(),
                         s_over_cont=[round(float((A[:, i] > c).sum()) / 20.0, 2) for i, c in enumerate(CONT)],
                         longest_s_over_cont=[round(_longest(A[:, i] > c) / 20.0, 2) for i, c in enumerate(CONT)],
                         steps=len(A))
    return out


def _longest(mask) -> int:
    best = run = 0
    for v in mask:
        run = run + 1 if v else 0
        best = max(best, run)
    return best


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    c = S.get("pose", {})
    if not c:
        return ""
    return (f"yaw {math.degrees(c['yaw']):.1f} gaps {c['gap_front'] * 100:.1f}/{c['gap_rear'] * 100:.1f} head "
            f"{math.degrees(c['heading']):.1f} hip {c['hip'] * 100:.1f} pose {S['pose_tier']} levers "
            f"{ {k: round(v, 1) for k, v in S['levers'].items()} } locked {S['locked']} touch "
            f"{S['chair_touch_d'] * 1000:.0f}mm tier {S['tier']} events {S['events'][-2:]}")


def close_view(ctx, gen):
    """Recorded runs: once both locks are on, the top camera moves for 2.25 s to a close view of the far lever (from in
    front of the chair, on its right), then for 2.25 s to the bed-side lever (low, from beside the right arm's
    pedestal), then back to its own pose for the rest of the run (the arms returning to ready, the held test). Only the
    recording changes; the demo reads object poses."""
    import torch

    base = ctx.emb._env.unwrapped
    cam = base.scene["top_cam"]
    o = base.scene.env_origins[0:1].to(torch.float32)

    def _t(x):
        x = getattr(x, "torch", x)
        return x if isinstance(x, torch.Tensor) else torch.as_tensor(np.asarray(x))

    home = (_t(cam.data.pos_w).clone().to(torch.float32), _t(cam.data.quat_w_world).clone().to(torch.float32))
    stage, t0 = 0, None
    seg = 45

    def look(eye_c, at_c):
        ch = Chair(ctx)
        eye, t = ch.world(eye_c), ch.world(at_c)
        cam.set_world_poses_from_view(torch.tensor(np.array([eye]), dtype=torch.float32, device=o.device) + o,
                                      torch.tensor(np.array([t]), dtype=torch.float32, device=o.device) + o)

    def check():
        nonlocal stage, t0
        step = ctx.task._STATE["step"]
        if stage == 0 and all(ctx.task._STATE["locked"].values()):
            look([0.66, -0.62, 0.62], [0.36, -0.26, 0.43])
            stage, t0 = 1, step
            ctx.flags["close_view_step"] = step
            print("top camera: close view, far lever", flush=True)
        elif stage == 1 and step - t0 >= seg:
            look([0.75, 0.38, 0.62], [0.33, 0.26, 0.40])
            stage, t0 = 2, step
            ctx.flags["close_view_2_step"] = step
            print("top camera: close view, bed-side lever", flush=True)
        elif stage == 2 and step - t0 >= seg:
            cam.set_world_poses(home[0].to(o.device), home[1].to(o.device), convention="world")
            stage = 3
            ctx.flags["close_view_end_step"] = step
            print("top camera: back to its own pose", flush=True)

    for _ in gen:
        yield
        check()
    for _ in range(10 * 20):            # the script has ended (arms at ready): keep going through the held test
        yield
        check()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=3600)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--close-view", action="store_true")
    ap.add_argument("--hold", type=float, default=8.0, help="seconds after the script ends (inf: to the step limit)")
    ap.add_argument("--variant", default="full",
                    choices=["full", "no_lock", "one_lock", "wedge", "nearly", "lock_start", "nothing"])
    a = ap.parse_args()
    if a.variant == "nothing":
        def scr(ctx):
            print("MODE variant=nothing", flush=True)
            yield from wait(5.0)
    else:
        def scr(ctx):
            if a.close_view and not a.no_record:
                yield from close_view(ctx, record_torques(script(ctx, a.variant)))
            else:
                yield from record_torques(script(ctx, a.variant))
    res = run(a.task, a.out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
              log_every=int(os.environ.get("WC_LOG_EVERY", "20")), hold_s=a.hold)
    S = sys.modules["task"]._STATE if "task" in sys.modules else {}
    extra = {"variant": a.variant, "tier": S.get("tier"), "final": S.get("final"), "pose_tier": S.get("pose_tier"),
             "pose": S.get("pose"), "levers_deg": S.get("levers"), "locked": S.get("locked"),
             "test_record": S.get("test_record"), "events": S.get("events"), "why": S.get("why"),
             "torques": torque_summary(),
             "robot_lee_min_mm": round(S.get("robot_lee_d", math.inf) * 1000, 1),
             "chair_lee_min_mm": round(S.get("chair_lee_d", math.inf) * 1000, 1)}
    if TORQUE_LOG["left"]:
        np.savez(pathlib.Path(a.out) / "torques.npz", left=np.array(TORQUE_LOG["left"]),
                 right=np.array(TORQUE_LOG["right"]), gravity_left=np.array(GRAVITY_LOG["left"]),
                 gravity_right=np.array(GRAVITY_LOG["right"]))
        if "error" in GRAVITY_LOG:
            print("GRAVITY_LOG error", GRAVITY_LOG["error"], flush=True)
    print("EXTRA " + json.dumps(extra, default=str), flush=True)
    (pathlib.Path(a.out) / "extra.json").write_text(json.dumps(extra, indent=2, default=str))
