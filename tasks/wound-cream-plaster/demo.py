"""The scripted reference demo: the right hand creams the wound with the applicator and bins it; then both hands carry
the plaster by its liner tabs and lay it centred on the wound, the right hand pulls its liner out, bins it and presses
its wing down, and the left hand does the same on its side.

Closed loop on what a camera sees: the applicator's pose (its rigid body), the liner tabs' positions (their rigid
bodies) and the strip's ends, read from `view()` before each stage; plus the fixed scene (the forearm, the wound, the
cup, the pot) and the robot's own joints. It never reads the scorer, who holds what, the cream's thickness, the tack,
the peel fronts or any contact force. The applicator and the tabs are real bodies held by the fingers' friction.

TASK_DEMO modes: `full` (default), `none` (do nothing), and the failure modes: `face` (a fingertip touches the pad's
face, bared by pulling the tabs a little apart in the air), `one_liner` (the left liner is left on), `cream_wing`
(cream smeared where a wing will lie), `drop` (the tabs pulled right apart over the wound: the bared strip falls),
`bud_rim` (the bud wiped on the cup's rim after the dip), `unheld` (the left hand lets go of its tab before the right
liner is pulled: nothing holds the strip against the liner's peel force, so the pull drags it off the wound).

Each liner peels against its release force, which pulls back on the tab and, at the peel front, drags the strip toward
the tab: the right liner is pulled with the left hand holding its own tab still (the strip's pull comes onto that tab),
the left one with the right wing tacked and the right hand pressing the pad down beside the wound. Both pulls go out low
and slowly, the tabs pinched shut.
"""

from __future__ import annotations

import math

import numpy as np
from yam_arena.urdf import axis_angle_matrix as aam

import demokit as dk
import scene as S

TOP = dk.TABLE_TOP  # world z of the table top; scene heights are above it
SIGN = {"left": 1.0, "right": -1.0}
STICK_W = 2 * S.APP_STICK_R  # what the fingers close on: the applicator's shaft
TAB_W = S.TAB_THICK  # ... and a liner tab


def W(p):
    """Scene point (z above the table) to world."""
    p = np.asarray(p, float).copy()
    p[2] += TOP
    return p


def rot(f, c):
    f = np.asarray(f, float)
    f = f / np.linalg.norm(f)
    c = np.asarray(c, float)
    c = c - f * (c @ f)
    c = c / np.linalg.norm(c)
    return np.stack([c, np.cross(f, c), f], 1)


def grip(R):
    return dk._as_grip(np.asarray(R, float))


def tab_grip(side, beta=30.0):
    """Fingers pointing in along the strip, tilted down by beta, the jaw closing about vertically."""
    s = SIGN[side]
    b = math.radians(beta)
    f = np.array([0.0, -s * math.cos(b), -math.sin(b)])
    R = rot(f, (0, 0, 1))
    return grip(R if side == "left" else aam(f, math.pi) @ R)


PRESS_TILT = math.radians(4.0)  # the closed fingers tipped about the TCP's y axis, so that one pad (tip_left) leads
LEAD_PAD_X = -0.0032  # that pad's middle along the closing axis, in the TCP frame (m)


def press_grip(arm, p, n, depth):
    """A closed press at skin point p with one fingertip pad: fingers into the skin along the normal n tilted back toward
    vertical as far as the arm needs, tipped PRESS_TILT so that the leading pad bears the press, and shifted so that
    pad's middle is on p; of the turns about the fingers' axis it reaches well, first those with the jaw's closing axis
    along the arm, the one nearest the grip it has now. Returns (grip, TCP over the point, TCP pressing)."""
    good = []
    for rolls in ((0, 180), tuple(range(-150, 180, 30))):
        _press_candidates(arm, p, n, depth, rolls, good)
        if good:
            break
    _, R, over, push = min(good, key=lambda g: g[0])
    return grip(R), over, push


def _press_candidates(arm, p, n, depth, rolls, good):
    from yam_arena import ik

    for frac in (1.0, 0.6, 0.3, 0.0):
        m = np.asarray(n, float) * frac + np.array([0, 0, 1.0]) * (1 - frac)
        m /= np.linalg.norm(m)
        f = -m
        R0 = rot(f, (0, -1, 0))
        for roll in rolls:
            R = aam(f, math.radians(roll)) @ R0
            R = R @ aam(np.array([0.0, 1.0, 0.0]), PRESS_TILT)
            lead = -R[:, 0] * LEAD_PAD_X
            over, push = W(p + m * 0.030) + lead, W(p + m * (0.0128 - depth)) + lead
            ok = True
            for pos in (over, push):
                q, pe, re_ = ik.solve(pos, ik.matrix_to_quat_xyzw(R), arm.q, base_pos=arm.base, iters=100)
                ok &= pe < 0.002 and re_ < 0.03
            if ok:
                good.append((float(np.linalg.norm(ik._rot_error(R, arm.grip.rot))) + 0.3 * (1 - frac), R, over, push))
        if good:
            break


def script(ctx, view):
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "none":
        yield from l.wait(1.0)
        return
    yield from cream(ctx, view, r)
    yield from plaster(ctx, view, l, r)


# -- part 1: the cream ------------------------------------------------------------------------------------------------------
def _tip_offset(arm, v):
    """The bud's centre relative to the TCP, as seen now (world)."""
    return v["app_tip"] - arm.tcp()


def cream(ctx, view, r):
    v = view()
    b, d = v["app_back"], v["app_dir"]
    g = b + d * 0.030  # hold the stick 30 mm from its back end, out over the tray's edge
    dg = dk.down(0.0)  # closing along y: across the stick (which lies along x)
    yield from r.move(g + [0, 0, 0.08], grip=dg, finger=dk.FINGER_OPEN, t=2.5, label="applicator: over")
    yield from r.move(g + [0, 0, 0.01], t=0.8, label="applicator: down")
    yield from r.move(g, t=0.6, label="applicator: in")
    yield from r.close(width=STICK_W, t=1.2, label="applicator: close")
    yield from r.move(g + [0, 0, 0.16], t=1.5, label="applicator: lift")
    # turn the stick bud-down: about the jaw's closing axis, the fingers end up pointing forward (+x)
    Rd = dg.rot
    cands = [aam(Rd[:, 0], a) @ Rd for a in (math.pi / 2, -math.pi / 2)]
    Rv = max(cands, key=lambda R: R[0, 2])
    yield from r.move(r.at(), grip=grip(Rv), t=2.0, label="applicator: turn bud down")
    yield from r.wait(0.3)
    # dip into the dollop in the cup
    off = _tip_offset(r, view())
    cup = W((S.CUP_C[0], S.CUP_C[1], S.CUP_Z))
    yield from r.move(cup + [0, 0, 0.07] - off, t=2.0, label="dip: over the cup")
    yield from r.move(cup + [0, 0, 0.0055] - off, t=1.5, label="dip: down")
    for a in np.linspace(0, 2 * math.pi, 9)[1:]:
        yield from r.move(cup + [0.002 * math.cos(a), 0.002 * math.sin(a), 0.0055] - off, t=0.15, label="dip: stir")
    yield from r.move(cup + [0, 0, 0.07] - off, t=1.2, label="dip: up")
    if mode_is(ctx, "bud_rim"):
        # wipes the excess off the bud on the cup's rim (the near side), then goes on as in the full demo
        rim = W((S.CUP_C[0] - S.CUP_R_TOP + 0.001, S.CUP_C[1], S.CUP_Z + S.CUP_H))
        yield from r.move(rim + [0, 0, S.APP_TIP_R + 0.012] - off, t=1.2, label="rim: over")
        yield from r.move(rim + [0, 0, S.APP_TIP_R - 0.0005] - off, t=0.8, label="rim: down")
        yield from r.move(rim + [-0.004, 0, S.APP_TIP_R - 0.0005] - off, t=0.6, label="rim: wipe")
        yield from r.move(rim + [-0.004, 0, 0.03] - off, t=0.8, label="rim: up")
    if mode_is(ctx, "cream_wing"):
        # a careless wipe out along the arm, where the right wing will lie
        for y in (-0.024, -0.034):
            p, n = S.ARM.point(y, 0.0)
            yield from r.move(W(p + n * 0.012) - off, t=1.5, label="smear: over")
            yield from r.move(W(p + n * (S.APP_TIP_R - 0.0005)) - off, t=0.8, label="smear: down")
            p2, n2 = S.ARM.point(y - 0.008, 0.0)
            yield from r.move(W(p2 + n2 * (S.APP_TIP_R - 0.0005)) - off, t=0.8, label="smear: wipe")
            yield from r.move(W(p2 + n2 * 0.012) - off, t=0.6, label="smear: up")
    # spread: rows along the arm, stepping round it, within the marked margin
    hy, hs = S.WOUND_HALF[0] + S.MARGIN, S.WOUND_HALF[1] + S.MARGIN
    rows = np.linspace(-hs + 0.0012, hs - 0.0012, 8)
    path = []
    for k, s in enumerate(rows):
        half = (hy - 0.0005) * math.sqrt(max(0.0, 1 - (s / (hs - 0.0003)) ** 2))
        ys = (-half, half) if k % 2 == 0 else (half, -half)
        path += [(ys[0], s), (ys[1], s)]
    squash = 0.0006

    def on_skin(y, s, lift=0.0):
        p, n = S.ARM.point(y, s)
        return W(p + n * (S.APP_TIP_R - squash + lift))

    # the stick is held only by friction and can turn a little in the fingers: where the bud is relative to the hand is
    # read again (from the applicator's visible pose) before each stroke
    off = _tip_offset(r, view())
    yield from r.move(on_skin(*path[0], lift=0.015) - off, t=2.0, label="spread: over")
    off = _tip_offset(r, view())
    yield from r.servo(on_skin(*path[0], lift=0.015) - off, tol=0.001, timeout=2.0, label="spread: over")
    off = _tip_offset(r, view())
    yield from r.move(on_skin(*path[0]) - off, t=1.0, label="spread: touch")
    prev = path[0]
    for q in path[1:]:
        dist = math.hypot(q[0] - prev[0], q[1] - prev[1])
        off = _tip_offset(r, view())
        yield from r.move(on_skin(*q) - off, t=max(0.3, dist / 0.012), label="spread")
        prev = q
    yield from r.move(on_skin(*prev, lift=0.03) - off, t=1.0, label="spread: lift")
    # into the waste pot, stick upright; let go with its middle over the pot's mouth
    v = view()
    mid_off = (v["app_back"] + v["app_dir"] * S.APP_LEN / 2) - r.tcp()
    pot = W((S.POT_C[0], S.POT_C[1], S.POT_H + 0.09))
    yield from r.move(r.at() + [0, 0, 0.05], t=1.0, label="bin: up")
    yield from r.move(pot - mid_off, t=2.5, label="bin: over the pot")
    yield from r.open(t=1.0, label="bin: drop")
    yield from r.move(W((0.20, -0.25, 0.20)), grip=dk.down(0.0), t=2.0, label="right: aside")


ASIDE = {"left": (0.18, 0.26, 0.20), "right": (0.18, -0.26, 0.20)}  # clear of the forearm, in the overview camera's view


def mode_is(ctx, m):
    return ctx.mode == m


# -- part 2: the plaster -------------------------------------------------------------------------------------------------------
def plaster(ctx, view, l, r):
    v = view()
    gl, gr = tab_grip("left"), tab_grip("right")
    tl, tr = v["tab"]["left"], v["tab"]["right"]
    out = {"left": np.array([0, 0.03, 0.03]), "right": np.array([0, -0.03, 0.03])}
    yield from ctx.together(l.move(tl + out["left"], grip=gl, finger=dk.FINGER_OPEN, t=3.0, label="tab: over"),
                            r.move(tr + out["right"], grip=gr, finger=dk.FINGER_OPEN, t=3.0, label="tab: over"))
    v = view()
    tl, tr = v["tab"]["left"], v["tab"]["right"]
    yield from ctx.together(l.move(tl, t=1.2, label="tab: in"), r.move(tr, t=1.2, label="tab: in"))
    yield from ctx.together(l.close(width=TAB_W, t=1.2, label="tab: close"), r.close(width=TAB_W, t=1.2, label="tab: close"))
    lift = np.array([0, 0, 0.10])
    yield from ctx.together(l.move(tl + lift, t=1.5, label="carry: lift"), r.move(tr + lift, t=1.5, label="carry: lift"))
    if mode_is(ctx, "face"):
        # the tabs pulled a little apart in the air (both liners peel back off the middle of the pad), then the right
        # hand lets go and takes the dangling strip by its middle: the pad's face is touched
        yield from ctx.together(l.move(l.at() + [0, 0.012, 0], t=1.5, label="face: pull apart"),
                                r.move(r.at() + [0, -0.012, 0], t=1.5, label="face: pull apart"))
        yield from r.open(label="face: let go")
        yield from r.move(r.at() + [0, -0.03, 0.03], t=0.8, label="face: away")
        g = l.at() - np.array([0, 0, S.TAB_GRIP + S.PL_LEN / 2])
        yield from r.move(g + [-0.05, 0, 0], grip=dk.down(90.0), t=2.0, label="face: beside the strip")
        yield from r.move(g, t=1.2, label="face: round the strip")
        yield from r.close(t=1.2, label="face: take it by the pad")
        yield from r.wait(1.0)
        return
    # over the wound, then down: each hand out past its end of the strip and above the skin (3 out to 4 up), as far from
    # where that end should lie as the tab runs from its root to the fingers (seen: the tab's root and the hand), so that
    # the strip settles on the skin centred, both tabs just taut
    wound, _ = S.ARM.point(S.WOUND_C[0], S.WOUND_C[1])
    v = view()
    run = {"left": l, "right": r}
    lay = {}
    for k in ("left", "right"):
        tl = float(np.linalg.norm(run[k].tcp() - v["tab_root"][k])) + 0.0005
        lay[k] = W(wound + [0, SIGN[k] * S.PL_LEN / 2, 0]) + np.array([0, SIGN[k] * 0.6, 0.8]) * tl
    yield from ctx.together(l.move(lay["left"] + [0, 0, 0.06], t=3.0, label="carry: over the wound"),
                            r.move(lay["right"] + [0, 0, 0.06], t=3.0, label="carry: over the wound"))
    if mode_is(ctx, "drop"):
        # the tabs pulled right apart over the wound (the liners peel off the bared strip), then let go there: the strip
        # drops onto the forearm from 6 cm up
        yield from ctx.together(l.move(l.at() + [0, 0.06, 0], t=2.0, label="drop: pull apart"),
                                r.move(r.at() + [0, -0.06, 0], t=2.0, label="drop: pull apart"))
        yield from ctx.together(l.open(t=1.0, label="drop: let go"), r.open(t=1.0, label="drop: let go"))
        yield from ctx.together(l.move(W(ASIDE["left"]), grip=dk.down(0.0), t=2.5, label="left: aside"),
                                r.move(W(ASIDE["right"]), grip=dk.down(0.0), t=2.5, label="right: aside"))
        yield from l.wait(2.0)
        return
    yield from ctx.together(l.move(lay["left"], t=2.0, label="lay"), r.move(lay["right"], t=2.0, label="lay"))
    yield from ctx.together(l.wait(0.5), r.wait(0.5))
    if mode_is(ctx, "unheld"):
        # the left hand lets go of its tab and moves off; the right pulls its liner with nothing holding the strip
        yield from l.open(t=1.0, label="unheld: let go of the left tab")
        yield from l.move(l.at() + [0, 0.03, 0.06], t=1.2, label="unheld: left away")
        yield from pull_liner(r, "right")
        yield from r.open(t=1.0, label="unheld: let go")
        yield from r.move(W(ASIDE["right"]), grip=dk.down(0.0), t=2.0, label="right: aside")
        yield from l.move(W(ASIDE["left"]), grip=dk.down(0.0), t=2.0, label="left: aside")
        yield from l.wait(2.0)
        return
    yield from peel_bin_press(ctx, view, r, "right")
    if mode_is(ctx, "one_liner"):
        yield from l.open(label="leave the left liner on")
        yield from l.move(l.at() + [0, 0.05, 0.08], t=1.5, label="left: away")
        yield from r.wait(1.5)
        return
    # the right hand presses the pad down beside the wound (its right half, where no left liner is) while the left liner
    # is pulled
    v = view()
    centre_y = (v["ends"]["left"][1] + v["ends"]["right"][1]) / 2
    p, n = S.ARM.point(centre_y - 0.014, 0.0)
    g, over, push = press_grip(r, p, n, 0.007)
    yield from r.move(over, grip=g, t=1.2, label="pad: over")
    yield from r.move(push, t=0.6, label="pad: hold down")
    yield from r.wait(0.3)
    yield from pull_liner(l, "left")
    yield from r.move(over, t=0.5, label="pad: up")
    yield from r.move(W(ASIDE["right"]), grip=dk.down(0.0), t=1.5, label="right: aside")
    yield from bin_press(view, l, "left")
    yield from l.wait(2.0)


def pull_liner(arm, side):
    """Pull the tab straight out along the strip, low over the skin and slowly (the liner peels at half the tab's
    travel)."""
    s = SIGN[side]
    start = arm.at()
    for out, up in ((0.030, 0.018), (0.060, 0.020), (0.095, 0.024), (0.135, 0.030)):
        tgt = start + np.array([0, s * (out - 0.010), up - 0.016])
        yield from arm.move(tgt, t=1.2 if out < 0.05 else 1.6, label=f"{side} liner: pull")


def peel_bin_press(ctx, view, arm, side):
    yield from pull_liner(arm, side)
    yield from bin_press(view, arm, side)


def bin_press(view, arm, side):
    s = SIGN[side]
    yield from arm.move(arm.at() + [0, 0, 0.08], t=1.2, label=f"{side} liner: up")
    pot = W((S.POT_C[0], S.POT_C[1] + 0.012 * s, S.POT_H + 0.07))
    yield from arm.move(pot, grip=dk.down(0.0), t=2.5, label=f"{side} liner: over the pot")
    yield from arm.open(t=1.0, label=f"{side} liner: drop")
    # press the wing down, where the strip is seen to lie: 5 stations along the wing, 3 round the arm, and its corners
    yield from arm.close(t=1.0, label=f"{side} wing: close the fingers")
    v = view()
    centre_y = (v["ends"]["left"][1] + v["ends"]["right"][1]) / 2
    stations = [(y, sc) for y in (0.022, 0.029, 0.036, 0.043, 0.050) for sc in (-0.015, 0.0, 0.015)]
    stations += [(0.023, -0.019), (0.023, 0.019)]  # the wing's inner corners, next to the pad, once more
    for y, sc in stations:
        p, n = S.ARM.point(centre_y + s * y, sc)
        g, over, push = press_grip(arm, p, n, 0.007)
        yield from arm.move(over, grip=g, t=0.8, label=f"{side} wing: over")
        yield from arm.move(push, t=0.6, label=f"{side} wing: press")
        yield from arm.wait(0.3)
        yield from arm.move(over, t=0.4, label=f"{side} wing: up")
    yield from arm.move(arm.at() + [0, 0, 0.03], t=0.6, label=f"{side} wing: up")
    yield from arm.move(W(ASIDE[side]), grip=dk.down(0.0), t=1.2, label=f"{side}: aside")
