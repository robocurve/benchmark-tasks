"""Scripted demo for the belt task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps N] [--no-record] [--cam-width 640 --cam-height 352]

It plans from the fixed start layout and, while it runs, reads the belt's link, frame and prong poses from the
simulator (the prong starts hidden under the strap, so a camera could not see all of them) and the arms' own joint
angles. It acts only through the arms' joint and gripper commands.

DEMO_STOP=<phase> ends the script after that phase (development). DEMO_VARIANT selects a cheap strategy:
nothing | pull_letgo (pull the tail and let go) | standing (prong through the hole but left standing) | hold (keep
the tail in the gripper at the end). DEMO_HOLE=2|3|4 is the hole the script aims for (3 = the task's). The start
layout follows the reset seed (--seed; BELT_START overrides).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
END_HOLD_S = 2.0           # demo.mp4 repeats its last frame this long (video only; no simulation steps)
VARIANT = os.environ.get("DEMO_VARIANT", "")
STOP = os.environ.get("DEMO_STOP", "")


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.bias = np.zeros(6)

    def _solve(self, pos, rot):
        from yam_arena import ik

        quat = ik.matrix_to_quat_xyzw(rot)
        q, pe, re = ik.solve(pos, quat, self.q, base_pos=self.base, iters=80)
        if pe > 0.0015 or re > 0.03:
            q2, pe2, re2 = ik.solve(pos, quat, self.q, base_pos=self.base, iters=300, damping=0.02)
            if pe2 + 0.05 * re2 < pe + 0.05 * re:
                q, pe, re = q2, pe2, re2
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q + self.bias, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None):
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos, float) - p0), ik.slerp_matrix(r0, rot, s))
            yield

    def best_q(self, pos, rot, seeds=None):
        """IK from several seeds (the most comfortable exact solution: largest distance from the joint limits)."""
        from yam_arena import ik

        quat = ik.matrix_to_quat_xyzw(rot)
        seeds = seeds or ([0, 1, 1, -0.6, 0, 0], [0.5, 1.5, 1.0, -0.1, -1.0, -0.6], [-0.5, 1.5, 1.0, -0.1, 1.0, 0.6],
                          [0.6, 1.8, 1.8, -1.2, -0.4, -0.8], [-0.6, 1.8, 1.8, -1.2, 0.4, 0.8],
                          [1.2, 1.7, 1.4, -0.8, 0.9, 0.9], [-1.3, 2.1, 1.4, 0.8, -1.3, 1.8], list(self.q))
        best = None
        for sd in seeds:
            q, pe, re = ik.solve(pos, quat, np.array(sd, float), base_pos=self.base, iters=300)
            margin = float(np.min(np.minimum(q - ik.LOWER, ik.UPPER - q)))
            key = (pe > 0.002 or re > 0.03, -margin)
            if best is None or key < best[0]:
                best = (key, q)
        return best[1]

    def goto(self, pos, rot, duration: float, grip: float | None = None, q_goal=None):
        """Move in joint space to a seeded IK solution (choosing the arm's configuration branch)."""
        from yam_arena import ik

        qg = self.best_q(pos, rot) if q_goal is None else np.asarray(q_goal, float)
        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (qg - q0)
            self.grip = g0 + s * (g1 - g0)
            self.cmd = np.r_[self.q + self.bias, self.grip]
            yield
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)

    def arc(self, centre, axis, angle: float, duration: float, grip: float | None = None):
        """Turn the hand (position and orientation) by ``angle`` about the line through ``centre`` along ``axis``."""
        from yam_arena import ik
        from yam_arena.urdf import axis_angle_matrix

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        ax = np.asarray(axis, float) / np.linalg.norm(axis)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            Rk = axis_angle_matrix(ax, s * angle)
            self.grip = g0 + s * (g1 - g0)
            self._solve(centre + Rk @ (p0 - centre), Rk @ r0)
            yield

    def set_grip(self, grip: float, duration: float = 0.6):
        yield from self.move(self.pos, self.rot, duration, grip)

    def stiffen(self, measured, rounds: int = 3, gain: float = 0.8):
        """Hold the commanded pose under load (commanded minus measured joint angles, the robot's own readings)."""
        for _ in range(rounds):
            yield from wait(0.25)
            err = self.q - measured(self.name)
            if np.max(np.abs(err)) < 0.003:
                break
            self.bias = np.clip(self.bias + gain * err, -0.4, 0.4)
            self.cmd = np.r_[self.q + self.bias, self.grip]

    def relax(self):
        self.bias = np.zeros(6)
        self.cmd = np.r_[self.q, self.grip]


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


def grip_rot(approach, close) -> np.ndarray:
    """Hand rotation: gripper z along ``approach`` (palm to fingertips), fingers closing along ``close``."""
    z = np.asarray(approach, float)
    z /= np.linalg.norm(z)
    x = np.asarray(close, float)
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def lean(direction, deg: float) -> np.ndarray:
    """Approach pointing down, leaned ``deg`` toward the horizontal ``direction``."""
    t = math.radians(deg)
    d = np.asarray(direction, float)
    d = d / np.linalg.norm(d)
    return d * math.sin(t) + np.array([0.0, 0.0, -math.cos(t)])


def _env_f(name, default):
    return float(os.environ.get(name, default))


def script(task, ctx):
    """Round 2: pull past, ease back, the prong hooks itself.
    0 left takes the tail's free end (face pinch, straight-line approach); 1 left pulls hole 3 past the turned-back
    prong's tip; 2 right presses the strap down just past the tip with closed fingertips; 3 both ease the strap back
    (loosening): hole 3 drops over the tip and its far edge pushes the tip up (pawl); 4 right lifts off; left carries
    the strap up and forward round the hinge (pull kept across the prong, a little toward its root), dragging the
    prong upright and over; 5 left draws the tail back flat over the fold (hole slides to the root, frame lies down);
    6 release."""
    import beltkit as B
    import geom as G
    import measure as M
    from yam_arena.urdf import axis_angle_matrix

    L, R = ctx.left, ctx.right
    st0 = ctx.belt()
    hx = st0.R("frame")[:, 1] / np.linalg.norm(st0.R("frame")[:, 1])     # hinge axis (world -x or +x)
    v0 = st0.R("frame") @ np.array([-1.0, 0.0, 0.0])                    # flat frame direction, hinge -> front bar
    v0[0] = 0.0
    v0 /= np.linalg.norm(v0)
    hinge0 = st0.p("frame").copy()
    if VARIANT == "nothing":
        while True:
            yield

    def finish(tag):
        """Hands are off: wait for the scorer (1 s hands off, 1 s pull, 0.5 s settle) and end the run."""
        for k in range(6):
            yield from wait(1.0)
            s_ = ctx.belt()
            Tm = getattr(getattr(ctx, "emb", None), "task", None)
            ctx.note(f"{tag} hands off {k}: {meas(s_)} score {Tm and Tm.module._S.get('score')} "
                     f"phase {Tm and Tm.module._S.get('phase')}")
        ctx.abort = True
        while True:
            yield

    def release_left_and_park():
        yield from L.set_grip(0.35, 0.5)
        yield from L.move(L.pos - np.array([0.07, 0.0, 0.0]) + np.array([0.0, 0.0, 0.03]), L.rot, 1.0)
        yield from L.goto(np.array([0.18, 0.16, 1.07]), grip_rot(lean([0, -1, 0], 20), [1, 0, 0]), 1.6)

    def halt(name):
        if VARIANT == "pull_letgo" and name == "pull":
            ctx.note(f"variant pull_letgo: letting go after {name}: {meas(ctx.belt())}")
            yield from release_left_and_park()
            yield from finish("pull_letgo")
        if STOP == name:
            ctx.note(f"stop after {name}")
            while True:
                yield

    def ang(s, v):
        return math.degrees(math.atan2(np.dot(np.cross(v0, v), hx), np.dot(v0, v)))   # same sense as dirn()

    def prong_angle(s):
        a, b, c = B.prong_segments()
        return ang(s, s.R("prong") @ (c - a)) % 360

    def prong_rel(s):
        """Prong angle relative to the fold (link 0): 0 = forward flat over the frame, ~160 = turned back on the fold."""
        a, b, c = B.prong_segments()
        v = s.R("prong") @ (c - a)
        f = -s.R("link_00")[:, 0]
        return math.degrees(math.atan2(np.dot(np.cross(f, v), hx), np.dot(f, v))) % 360

    def frame_angle(s):
        return ang(s, s.R("frame") @ np.array([-1.0, 0.0, 0.0]))

    def tip_of_prong(s):
        a_, b_, c_ = B.prong_segments()
        return s.p("prong") + s.R("prong") @ c_

    def dirn(phi_deg):
        """Unit vector in the belt plane at phi from the flat frame direction (phi 90 = straight up from the hinge)."""
        return axis_angle_matrix(hx, math.radians(phi_deg)) @ v0

    HOLE_T = int(os.environ.get("DEMO_HOLE", "3"))                # the hole the script aims for
    h3 = f"link_{G.HOLE_LINK[HOLE_T]:02d}"
    # the tail link to hold: the first past hole 1 that is clearly off the form (20 mm, room for the lower finger)
    kt = os.environ.get("TAIL_LINK", "auto")
    if kt == "auto":
        kt = next((i for i in range(G.HOLE_LINK[1] + 2, G.N_LINKS - 1)
                   if M.gap_to_trousers(st0.p(f"link_{i:02d}")) >= 0.020), G.N_LINKS - 4)
    kt = int(kt)
    held = f"link_{kt:02d}"
    ctx.note(f"holding tail link {kt} ({M.gap_to_trousers(st0.p(held)) * 1000:.0f} mm off the form)")
    grip_ref = {}

    def meas(s):
        """Numbers for the log: prong angle, engaged hole, hole 3 past the tip (along the strap), belt rotation
        (hinge moved along y), slip of the held link in the left hand (mm)."""
        tip = tip_of_prong(s)
        past = float(np.dot(s.p(h3) - tip, -v0)) * 1000
        rot = (s.p("frame")[1] - hinge0[1]) * 1000
        slip = None
        if grip_ref:
            Rh, ph = L.rot, L.pos
            loc = Rh.T @ (s.p(held) - ph)
            slip = round(float(np.linalg.norm(loc - grip_ref["loc"])) * 1000, 1)
        return (f"prong {prong_angle(s):.0f} rel {prong_rel(s):.0f} frame {frame_angle(s):.0f} hole {M.hole_engaged(s)} h3_past_tip "
                f"{past:.1f}mm belt_rot {rot:.1f}mm slipL {slip}")

    # ---------------------------------------------------------------- 0. left hand takes the tail's free end
    Rk, p = st0.R(held), st0.p(held)
    rotT0 = grip_rot(np.array([1.0, 0.0, 0.0]), Rk[:, 2])
    yield from L.goto(p + np.array([-0.13, 0.0, 0.06]), rotT0, 2.5, grip=0.30)       # well clear of the belt
    yield from L.move(p + np.array([-0.07, 0.0, 0.0]), rotT0, 1.0)
    yield from L.move(p + np.array([-0.035, 0.0, 0.0]), rotT0, 0.6)
    yield from L.move(p, rotT0, 1.0)
    yield from L.set_grip(0.0, 0.6)
    yield from wait(0.3)
    s = ctx.belt()
    grip_ref["loc"] = L.rot.T @ (s.p(held) - L.pos)
    ctx.note(f"tail taken: {meas(s)}")
    yield from halt("take")

    # ---------------------------------------------------------------- 1. pull hole 3 past the tip (tightening)
    PAST = _env_f("PAST_MM", "9") / 1000
    pull_dir = np.array([0.0, 1.0, _env_f("PULL_UP", "0.25")])
    pull_dir /= np.linalg.norm(pull_dir)
    for it in range(12):
        s = ctx.belt()
        tip = tip_of_prong(s)
        past = float(np.dot(s.p(h3) - tip, -v0))
        ctx.note(f"pull {it}: {meas(s)}")
        if past >= PAST:
            break
        step = min(0.015, max(0.004, (PAST - past) * 1.1))
        yield from L.move(L.pos + pull_dir * step, L.rot, 0.6)
    yield from wait(0.5)
    ctx.note(f"pulled past: {meas(ctx.belt())}")
    # lay the tail down round the form's side so the strap lies on the fold past hole 3 (r5: the straight strap from
    # hole 3 to the raised hand held hole 3's link up level, 4-17 mm over the fold, so it could not drop onto the tip)
    LAY = _env_f("LAY_MM", "28") / 1000
    for k in range(4):
        yield from L.move(L.pos + np.array([0.0, 0.0, -LAY / 4]), L.rot, 0.5)
    yield from wait(0.4)
    ctx.note(f"laid down: {meas(ctx.belt())}")
    yield from halt("pull")

    # ---------------------------------------------------------------- 2. right presses the strap down past the tip
    PRESS_OFF = _env_f("PRESS_OFF_MM", "14") / 1000       # press point beyond the tip, along the strap
    PRESS_DEPTH = _env_f("PRESS_DEPTH_MM", "5") / 1000
    LEAN_R = _env_f("PRESS_LEAN", "35")
    s = ctx.belt()
    tip = tip_of_prong(s)
    upn = dirn(90.0)                                      # 'up' from the fold, in the belt plane
    # strap's top surface above the press point: from the link under it
    target = tip - v0 * PRESS_OFF
    if os.environ.get("PRESS_REF", "hole") == "hole":
        # press just beyond hole 3's far edge, so the tip can come up through the hole without meeting the fingers
        R3_, p3_ = s.R(h3), s.p(h3)
        t3_ = R3_[:, 0] if R3_[1, 0] > 0 else -R3_[:, 0]
        target = p3_ + t3_ * _env_f("PRESS_PAST_MM", "9") / 1000
    near = min(range(G.HOLE_LINK[5], G.N_LINKS), key=lambda i: np.linalg.norm(s.p(f"link_{i:02d}") - target))
    Rn, pn = s.R(f"link_{near:02d}"), s.p(f"link_{near:02d}")
    nz = Rn[:, 2] if Rn[:, 2] @ upn > 0 else -Rn[:, 2]
    surf = target + nz * (float(np.dot(pn - target, nz)) + G.T / 2)
    surf[0] = G.X_B
    appP = lean([0, 1, 0], LEAN_R)
    rotP = grip_rot(appP, [1, 0, 0])
    tipoff = appP * 0.0135
    yield from R.goto(surf - tipoff - appP * 0.06, rotP, 2.5, grip=0.0)
    yield from R.move(surf - tipoff - appP * 0.015, rotP, 1.0)
    yield from R.move(surf - tipoff + nz * -PRESS_DEPTH, rotP, 0.8)
    yield from wait(0.3)
    ctx.note(f"pressed (link {near}): {meas(ctx.belt())}")
    yield from halt("press")

    # ---------------------------------------------------------------- 3. ease back (loosening): hole 3 onto the tip
    EASE = _env_f("EASE_MM", "50") / 1000
    STEP_E = 0.002
    # ease direction: back along the strap at the press point (r3: easing along the tilted frame direction lifted the
    # pressing fingertips 3.3 mm off the strap)
    tang = Rn[:, 0] if Rn[1, 0] > 0 else -Rn[:, 0]
    tang = tang.copy()
    tang[0] = 0.0
    easedir = -tang / np.linalg.norm(tang)
    hooked = False
    extra = 0.0
    s = ctx.belt()
    rel_pre = prong_rel(s)
    HANDOFF = _env_f("HANDOFF_DEG", "100")      # prong angle vs the fold at which the press hand lets go
    LAT_GAIN = _env_f("LAT_GAIN", "0.8")
    after = 0
    for it in range(int(round(EASE / STEP_E)) + 45):
        s = ctx.belt()
        p3, R3 = s.p(h3), s.R(h3)
        loc = R3.T @ (tip_of_prong(s) - p3)
        # centre the hole over the prong sideways (the strap has ~1.5 mm play in the frame window; the hole only
        # +-0.9 mm round the prong): the pressing fingertips carry the strap across
        lat = np.clip(R3[:, 1] * loc[1] * LAT_GAIN, -0.001, 0.001)
        lat[1:] = 0.0
        d = easedir * STEP_E + lat
        yield from both(L.move(L.pos + d, L.rot, 0.25), R.move(R.pos + d, R.rot, 0.25))
        s = ctx.belt()
        p3, R3 = s.p(h3), s.R(h3)
        loc = R3.T @ (tip_of_prong(s) - p3)
        # the hole is over the tip but the strap has not dropped onto it (the raised front bar can hold the hole
        # link up): press deeper, 1 mm at a time, up to PRESS_EXTRA_MM, until the tip is within 2.3 mm of the hole's
        # mid-plane or the hook is detected
        if (not hooked and abs(loc[0]) < 0.003 and loc[2] < -0.0023 and extra < _env_f("PRESS_EXTRA_MM", "4") / 1000):
            extra += 0.001
            yield from R.move(R.pos - nz * 0.001, R.rot, 0.25)
            yield from wait(0.15)
            s = ctx.belt()
            p3, R3 = s.p(h3), s.R(h3)
            loc = R3.T @ (tip_of_prong(s) - p3)
            ctx.note(f"ease {it}: pressed {extra*1000:.0f} mm deeper: tip@h3 {loc[0]*1000:.1f}/{loc[1]*1000:.1f}/{loc[2]*1000:.1f}mm")
        rise = rel_pre - prong_rel(s)
        if it % 2 == 1 or hooked:
            ctx.note(f"ease {it}: tip@h3 {loc[0]*1000:.1f}/{loc[1]*1000:.1f}/{loc[2]*1000:.1f}mm rise {rise:.0f} {meas(s)}")
        # the tip is in the hole's opening (its top above the strap's underside) or riding its far edge: keep easing
        # (B2 A: the far edge rode the tip at normal -2.2 mm and the 25-step cap ended the loop before the prong rose)
        tip_in = abs(loc[0]) < G.HOLE_Y / 2 and abs(loc[1]) < G.HOLE_X / 2 and loc[2] > -(G.T / 2 + G.PRONG_R * 0.5)
        if M.hole_engaged(s) == 3 or rise > 8 or tip_in:
            hooked = True
        if hooked:
            after += 1
            # keep the pawl pushing until the prong stands about upright, or the fingertips near the hinge
            fing = float(np.dot(R.pos + tipoff - s.p("frame"), -easedir))
            if prong_rel(s) <= HANDOFF or after >= 45 or fing < 0.012:
                ctx.note(f"pawl push done: rel {prong_rel(s):.0f} fingers {fing*1000:.0f}mm past hinge after {after}")
                break
        elif loc[0] > 0.006 or it >= int(round(EASE / STEP_E)):
            break
    yield from wait(0.3)
    ctx.note(f"eased back (hooked {hooked}): {meas(ctx.belt())}")
    yield from halt("ease")
    if not hooked and os.environ.get("END_IF_UNHOOKED", "1") == "1":
        ctx.note("not hooked: ending this run early")
        ctx.abort = True
        return
    if VARIANT == "standing":
        # cheap strategy: leave the prong standing through the hole, hands off
        yield from R.move(R.pos - appP * 0.03 + upn * 0.02, R.rot, 0.6)
        yield from R.goto(np.array([0.18, -0.16, 1.07]), grip_rot(lean([0, 1, 0], 20), [1, 0, 0]), 1.6)
        yield from release_left_and_park()
        ctx.note(f"variant standing: hands off: {meas(ctx.belt())}")
        yield from finish("standing")

    # ---------------------------------------------------------------- 4. press hand lifts off
    park = np.array([0.18, -0.16, 1.07])
    yield from R.move(R.pos - appP * 0.03 + upn * 0.02, R.rot, 0.6)
    yield from R.goto(park, grip_rot(lean([0, 1, 0], 20), [1, 0, 0]), 1.6)
    ctx.note(f"press released: {meas(ctx.belt())}")

    # ---------------------------------------------------------------- 5. carry the strap up, over and forward
    # The tail hand first goes up and over the hinge along a path that keeps the strap slack (closer to hole 3 than
    # the strap is long), so it never drags the prong back; only in front of the hinge does it take up the strap and
    # pull forward across the prong, laying it over toward the front bar.
    ZMIN = G.ZC + G.B1 + 0.03
    yy = -v0                                            # +y-ish along the belt at the buckle

    def carry_rot(hand, s):
        u = hand - tip_of_prong(s)
        u[0] = 0.0
        u /= max(np.linalg.norm(u), 1e-6)
        return grip_rot(np.array([1.0, 0.0, 0.0]), np.array([0.0, -u[2], u[1]]))

    s = ctx.belt()
    hinge = s.p("frame")
    for k, w in enumerate([hinge + upn * 0.075 + yy * 0.06, hinge + upn * 0.085 + yy * 0.01,
                           hinge + upn * 0.07 - yy * 0.05]):
        w = w.copy()
        w[0] = L.pos[0]
        yield from L.move(w, carry_rot(w, ctx.belt()), 1.0)
        ctx.note(f"carry {k}: {meas(ctx.belt())}")
    FWD = _env_f("FWD_DONE_DEG", "40")
    for it in range(14):
        s = ctx.belt()
        hinge = s.p("frame")
        goal = hinge + upn * 0.035 - yy * 0.115
        goal[0] = L.pos[0]
        goal[2] = max(goal[2], ZMIN)
        step = goal - L.pos
        if np.linalg.norm(step) > 0.012:
            step = step / np.linalg.norm(step) * 0.012
        w = L.pos + step
        yield from L.move(w, carry_rot(w, s), 0.4)
        s = ctx.belt()
        ctx.note(f"forward {it}: {meas(s)}")
        if prong_rel(s) <= FWD:
            break
    yield from halt("carry")

    # ---------------------------------------------------------------- 6. tail back over the top, without tightening
    # (r7: drawing the tail back under tension lifted the frame and prong together, 11 -> 36 deg)
    def frame_rel(s):
        f = -s.R("link_00")[:, 0]
        v = s.R("frame") @ np.array([-1.0, 0.0, 0.0])
        return math.degrees(math.atan2(np.dot(np.cross(f, v), hx), np.dot(f, v)))

    s = ctx.belt()
    hinge = s.p("frame")
    rotB = grip_rot(np.array([1.0, 0.0, 0.0]), upn)
    for k, w in enumerate([hinge + upn * 0.055 - yy * 0.03, hinge + upn * 0.05 + yy * 0.03,
                           hinge + upn * 0.035 + yy * 0.07]):
        w = w.copy()
        w[0] = L.pos[0]
        yield from L.move(w, rotB, 1.0)
        s = ctx.belt()
        ctx.note(f"back {k}: {meas(s)} frame/fold {frame_rel(s):.0f} prong/fold {prong_rel(s):.0f}")
        if k >= 1 and frame_rel(s) > _env_f("BACK_STOP_DEG", "20"):
            break
    yield from halt("back")

    # ---------------------------------------------------------------- 7. lay the tail down and let go
    TAIL_HOLD = os.environ.get("TAIL_HOLD", "0") == "1" or VARIANT == "hold"

    def let_go_tail():
        if TAIL_HOLD:            # take the compression off hole 3: draw the tail back a little, low along the fold
            yield from L.move(L.pos + yy * 0.006 - upn * 0.006, L.rot, 0.6)
        yield from L.move(L.pos - upn * 0.015, L.rot, 0.6)
        yield from L.set_grip(0.35, 0.5)
        yield from L.move(L.pos - np.array([0.07, 0.0, 0.0]) + upn * 0.03, L.rot, 1.0)
        yield from L.goto(np.array([0.18, 0.16, 1.07]), grip_rot(lean([0, -1, 0], 20), [1, 0, 0]), 1.6)
        s_ = ctx.belt()
        ctx.note(f"tail let go: {meas(s_)} frame/fold {frame_rel(s_):.0f} prong/fold {prong_rel(s_):.0f} "
                 f"frame_vs_strap {M.frame_angle_deg(s_)[0]:.0f} fb {M.front_bar_over_strap(s_)}")

    if not TAIL_HOLD:
        yield from let_go_tail()
    s = ctx.belt()

    # ---------------------------------------------------------------- 8. lay the frame flat (fingertip on the front
    # bar, where the prong rests on it), then lift off
    if os.environ.get("FRAME_PRESS", "0") == "1":
        appF = lean([0, 1, 0], 30)
        rotF = grip_rot(appF, [1, 0, 0])
        tipF = appF * 0.0135
        fbar = s.p("frame") + s.R("frame") @ np.array([-G.FRAME_L, 0.0, 0.0])
        yield from R.goto(fbar - tipF + upn * 0.05, rotF, 1.6, grip=0.0)
        yield from R.move(fbar - tipF + upn * 0.012, rotF, 0.8)
        for k in range(8):
            s = ctx.belt()
            fbar = s.p("frame") + s.R("frame") @ np.array([-G.FRAME_L, 0.0, 0.0])
            hinge = s.p("frame")
            # push the bar down round the hinge: aim a little below where the flat frame's front bar would be
            flat = hinge + (-s.R("link_00")[:, 0]) * G.FRAME_L
            # on top of the bar, a little toward the hinge (r8: fingertips 9 mm from the bar, in front of it)
            aim = (fbar + 0.6 * (flat - fbar) + yy * _env_f("PRESS_AIM_Y_MM", "8") / 1000
                   + upn * (G.BAR_R + G.PRONG_R * 2 - _env_f("FRAME_PRESS_MM", "2") / 1000))
            yield from R.move(aim - tipF, rotF, 0.5)
            s = ctx.belt()
            ctx.note(f"frame press {k}: {meas(s)} frame/fold {frame_rel(s):.0f} prong/fold {prong_rel(s):.0f} "
                     f"frame_vs_strap {M.frame_angle_deg(s)[0]:.0f} fb {M.front_bar_over_strap(s)}")
            if frame_rel(s) < 6:
                break
        yield from wait(_env_f("PRESS_HOLD_S", "0.5"))
        P2 = _env_f("PRESS2_MM", "0") / 1000
        if P2 > 0:
            # second press: slide the fingertips back over the prong toward the hinge and press the prong (and the strap
            # under it at hole 3) down - the strap springing up under the prong is what lifts it (r9)
            for k in range(3):
                s = ctx.belt()
                hinge = s.p("frame")
                fwd = s.R("frame") @ np.array([-1.0, 0.0, 0.0])
                frame_up = s.R("frame") @ np.array([0.0, 0.0, 1.0])
                aim2 = hinge + fwd * P2 + frame_up * (G.PRONG_R * 2 + 0.004 - 0.002 * k)
                yield from R.move(aim2 - tipF, rotF, 0.5)
                s = ctx.belt()
                ctx.note(f"press2 {k}: {meas(s)} frame/fold {frame_rel(s):.0f} prong/fold {prong_rel(s):.0f} "
                         f"frame_vs_strap {M.frame_angle_deg(s)[0]:.0f} fb {M.front_bar_over_strap(s)}")
            yield from wait(_env_f("PRESS_HOLD_S", "0.5"))
        # lift straight off, drifting back over the frame toward the hinge (r8 backed out along the approach,
        # i.e. forward and up past the front bar, and the frame came up 11 -> 21 deg as the hand left)
        yield from R.move(R.pos + upn * 0.012 + yy * 0.003, rotF, 0.8)
        s = ctx.belt()
        ctx.note(f"lifting off: {meas(s)} frame/fold {frame_rel(s):.0f} frame_vs_strap {M.frame_angle_deg(s)[0]:.0f} fb {M.front_bar_over_strap(s)}")
        yield from R.move(R.pos + upn * 0.04 + yy * 0.01, rotF, 0.8)
        yield from R.goto(park, grip_rot(lean([0, 1, 0], 20), [1, 0, 0]), 1.6)
    if VARIANT == "hold":
        # cheap strategy: keep the tail in the left gripper to the end
        for k in range(6):
            yield from wait(1.0)
            s = ctx.belt()
            Tm = getattr(getattr(ctx, "emb", None), "task", None)
            ctx.note(f"variant hold {k}: {meas(s)} score {Tm and Tm.module._S.get('score')} phase {Tm and Tm.module._S.get('phase')}")
        ctx.abort = True
        return
    if TAIL_HOLD:
        yield from let_go_tail()
    for k in range(6):
        yield from wait(1.0)
        s = ctx.belt()
        Tm = getattr(getattr(ctx, "emb", None), "task", None)
        ctx.note(f"hands off {k}: {meas(s)} frame/fold {frame_rel(s):.0f} prong/fold {prong_rel(s):.0f} "
                 f"frame_vs_strap {M.frame_angle_deg(s)[0]:.0f} fb {M.front_bar_over_strap(s)} "
                 f"score {Tm and Tm.module._S.get('score')} phase {Tm and Tm.module._S.get('phase')}")
    ctx.abort = True          # the scorer's hands-off second, pull test and settle are over by now
    return


class Ctx:
    def __init__(self, emb, out):
        from yam_arena import station

        self.emb = emb
        self.out = out
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.steps = 0
        self.obs = None

    def materials(self):
        """Print the physics materials bound to the robots' finger colliders (pad friction check)."""
        try:
            import isaaclab.sim as sim
            from pxr import UsdPhysics, UsdShade
            stage = sim.get_current_stage()
            seen = 0
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if "right_robot" not in path or not prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                if not any(k in path.lower() for k in ("finger", "link_7", "link_8", "tip", "gripper")):
                    continue
                mat = UsdShade.MaterialBindingAPI(prim).GetDirectBinding("physics").GetMaterial()
                fr = None
                if mat:
                    m = UsdPhysics.MaterialAPI(mat.GetPrim())
                    fr = (m.GetStaticFrictionAttr().Get(), m.GetDynamicFrictionAttr().Get(), str(mat.GetPath()))
                print(f"PADMAT {path} {fr}", flush=True)
                seen += 1
                if seen > 12:
                    break
            sc = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.MaterialAPI)][:20]
            for p in sc:
                m = UsdPhysics.MaterialAPI(p)
                print(f"MAT {p.GetPath()} {m.GetStaticFrictionAttr().Get()} {m.GetDynamicFrictionAttr().Get()}", flush=True)
        except Exception as ex:  # noqa: BLE001
            print("PADMAT failed", ex, flush=True)

    def joints(self, arm: str) -> np.ndarray:
        return np.asarray(self.emb._env.unwrapped.scene[f"{arm}_robot"].data.joint_pos[0].cpu(), float)[:6]

    def belt(self):
        import beltkit as B
        return B.read(self.emb._env, B._SPECS["belt"])

    def note(self, what: str) -> None:
        import measure as M
        T = self.emb.task.module
        s = self.belt()
        art = self.emb._env.unwrapped.scene.articulations["belt"]
        jn = list(art.joint_names)
        jp = art.data.joint_pos
        jp = (jp.torch if hasattr(jp, "torch") else jp)[0].cpu().numpy()
        import partkit
        gp = partkit.gripper_points(self.emb._env)
        gaps = {k: round(v[1] * 1000, 1) for k, v in gp.items()}
        print(f"NOTE {what} step {self.steps}: gaps {gaps} frame {math.degrees(jp[jn.index('frame_hinge')]):.0f} prong "
              f"{math.degrees(jp[jn.index('prong_hinge')]):.0f} hole {M.hole_engaged(s)} threaded {M.threaded(s)} "
              f"tip {np.round(s.p(f'link_{len(s.poses) - 3:02d}'), 3).tolist()} tcpL {np.round(self.left.pos, 3).tolist()} "
              f"tcpR {np.round(self.right.pos, 3).tolist()} ik_worst L {self.left.worst_mm:.1f} R {self.right.worst_mm:.1f} "
              f"raw {T._S.get('raw')} {T._S.get('last')}", flush=True)
        try:      # positions for offline side views (development)
            dump = {"step": self.steps, "what": what,
                    "bodies": {k: [np.asarray(p).round(5).tolist(), np.asarray(R).round(4).tolist()]
                               for k, (p, R) in s.poses.items()},
                    "fingers": {k: [np.asarray(q).round(5).tolist() for q in v[0]] for k, v in gp.items()}}
            with open(self.out / "notes.jsonl", "a") as fh:
                fh.write(json.dumps(dump) + "\n")
        except Exception as ex:  # noqa: BLE001
            print("dump failed", ex)
        if self.obs is not None:
            import imageio
            tag = "".join(ch if ch.isalnum() else "_" for ch in what)
            im = np.hstack([np.asarray(self.obs.images[c]) for c in sorted(self.obs.images)])
            imageio.imwrite(self.out / f"note_{self.steps:04d}_{tag}.png", im)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, seed: int | None = None) -> dict:
    from types import SimpleNamespace

    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.scoring import task_score

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    task = emb.task.module
    ctx = Ctx(emb, out)
    ctx.obs = obs
    import beltkit as B
    idx = B.start_index()
    intended, _ = B.layout("threaded", idx)
    st = ctx.belt()
    err = max(float(np.linalg.norm(st.p(n) - intended[n][1])) for n in intended) * 1000
    print(f"START CHECK: seed {seed} layout {idx}: largest body position error vs the intended layout {err:.1f} mm "
          f"(after the reset settle)", flush=True)
    ctx.note("start")
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, res = 0, None
    with (out / "actions.jsonl").open("w") as log:
        while steps < max_steps:
            try:
                next(gen)
            except StopIteration:
                pass
            act = ctx.action()
            res = emb.step(act)
            ctx.obs = res.observation
            steps = ctx.steps = steps + 1
            log.write(json.dumps({"step": steps, "action": np.asarray(act.data, float).round(6).tolist()}) + "\n")
            snap(res.observation)
            if steps % 100 == 0:
                print(f"step {steps} score {res.info.get('score')} phase {task._S['phase']} {task._S['last']}",
                      flush=True)
            if res.terminated or getattr(ctx, "abort", False):
                break
    scored = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    print(f"task_score: {scored.value:g} ({scored.explanation}) after {steps} steps", flush=True)
    result = {"task_score": scored.value, "explanation": scored.explanation, "steps": steps, "variant": VARIANT,
              "seed": seed, "layout": idx, "hole_aimed": int(os.environ.get("DEMO_HOLE", "3")),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        cams = sorted(frames, key=lambda c: ("left" not in c, "top" not in c))
        video = [np.hstack([frames[c][i] for c in cams]) for i in range(len(frames[cams[0]]))]
        video += [video[-1]] * int(END_HOLD_S * CONTROL_HZ)     # the episode ends at success; still frame of the end
        _write_mp4(video, out / "demo.mp4")
        imageio.imwrite(out / "setup.png", np.hstack([frames[c][0] for c in cams]))
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--cam-width", type=int, default=320)
    ap.add_argument("--cam-height", type=int, default=176)
    ap.add_argument("--seed", type=int, default=int(os.environ["DEMO_SEED"]) if os.environ.get("DEMO_SEED") else None)
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=a.cam_width, cam_height=a.cam_height,
        seed=a.seed)
