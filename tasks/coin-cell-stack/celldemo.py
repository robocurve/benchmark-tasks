"""The scripted reference demo: closed loop on the poses of the parts and tools (which the cameras see), the tweezers'
and the plunger's joints (seen as how far the tips stand apart and how far the button is down), and the arms' own joints.

Left hand: the tweezers (held sideways across both legs, 80 mm above the tips, squeezed to close the tips) and the
pipette's body. Right hand: the pipette's plunger (pressed with the underside of the closed fingers) and the cap press.
"""

from __future__ import annotations

import math
import os

import numpy as np

import cellparts as cp
from demokit import down, level

G = level(0.0)  # sideways: fingers along +x, closing along y
T = None  # the task's constants, set by script()
VERBOSE = os.environ.get("TASK_DEMO_LOG") == "1"


def log(ctx, msg):
    print(f"[demo] t={ctx.t:6.2f} {msg}", flush=True)


# -- the tweezers, as seen ---------------------------------------------------------------------------------------------


def tip_points(ctx):
    """The corners of both tip pads now, world frame: leg a's 8, then leg b's 8 (leg b turns about the head). Each pad's
    corners run (x -/+) x (inner bottom, outer bottom, inner top, outer top)."""
    tw = ctx.state("tweezers")
    th = tw.joint("leg_b")
    c, s_ = math.cos(th), math.sin(th)
    a = cp.tip_pad_points(1.0)
    b = cp.tip_pad_points(-1.0) - np.array([0, 0, cp.TW_L])
    b = np.c_[b[:, 0], c * b[:, 1] - s_ * b[:, 2], s_ * b[:, 1] + c * b[:, 2]] + np.array([0, 0, cp.TW_L])
    return tw.to_world(np.vstack([a, b]))


def tip_bottom(ctx):
    return float(tip_points(ctx)[:, 2].min())


def tip_gap(ctx):
    pts = tip_points(ctx)
    return float(np.linalg.norm(pts[[0, 4]].mean(axis=0)[:2] - pts[[8, 12]].mean(axis=0)[:2]))


def tip_centre(ctx):
    return tip_points(ctx).mean(axis=0)


def width(arm):
    return -2 * arm.finger


def set_width(arm, w, t=0.6):
    yield from arm.move(arm.at(), finger=-w / 2, t=t, label="width")


def set_gap(ctx, arm, gap, t=0.6, iters=12):
    """Set the pads' width, then trim it until the tips stand `gap` apart, never opening so far that the legs go slack."""
    w = cp.grip_width_for(gap)
    wmax = cp.grip_width_for(cp.TW_REST_GAP - 0.005)
    w = min(w, wmax)
    yield from set_width(arm, w, t)
    for _ in range(iters):
        err = gap - tip_gap(ctx)
        if abs(err) < 0.0003:
            break
        w = min(w + 0.45 * err, wmax)
        yield from set_width(arm, w, 0.15)


def part_centre(ctx, name):
    s = ctx.state(name)
    h = {"cap": cp.CAP_H}.get(name, 0.0005)
    return s.pos + s.rotation()[:, 2] * h / 2


def go_tips(ctx, l, xy, z_tip, t=None, label="go"):
    """Move so the tips' centre is over `xy`, their lowest point at `z_tip`."""
    for _ in range(2):
        tc = tip_centre(ctx)
        tcp = l.tcp()
        goal = l.at() + np.array([xy[0] - tc[0], xy[1] - tc[1], z_tip - tip_bottom(ctx)]) + (l.at() - tcp) * 0.0
        yield from l.move(goal, t=t, label=label)
        t = 0.5


def press_tips(ctx, arm, z_goal, tries=4):
    """Lower until the tips' lowest point is at `z_goal`: they press into the soft lining. A vertical dither that dies
    away lets the arm's joints past their friction."""
    goal = arm.at()
    for _ in range(tries):
        err = z_goal - tip_bottom(ctx)
        if abs(err) < 0.0001:
            break
        goal = goal + np.array([0.0, 0.0, err])
        yield from dither_to(arm, goal, amp=0.001, t=1.0, hz=4.0, vertical=True)
        yield from arm.wait(0.2)


def take_tweezers(ctx, l):
    tw = ctx.state("tweezers")
    grip = tw.to_world(np.array([[0.0, 0.0, cp.TW_GRIP_Z]]))[0]
    yield from l.move(grip + np.array([-0.10, 0, 0.03]), grip=G, label="tweezers: approach")
    yield from l.move(grip + np.array([-0.10, 0, 0.0]), t=0.6, label="tweezers: level")
    yield from l.move(grip, t=1.2, label="tweezers: in")
    yield from l.servo(grip, tol=0.0015, timeout=1.0, label="tweezers: in")
    yield from set_width(l, cp.grip_width_for(0.020), 0.8)
    yield from l.wait(0.3)
    yield from l.move(l.at() + np.array([0, 0, 0.040]), t=1.2, label="tweezers: lift")
    log(ctx, f"tweezers held: tip gap {tip_gap(ctx) * 1000:.1f} mm, pads {width(l) * 1000:.2f} mm apart")


def stow_tweezers(ctx, l):
    sx, sy, stop = T.LAYOUT.stand
    yield from set_gap(ctx, l, 0.020)
    yield from go_tips(ctx, l, (sx, sy), stop + 0.010, label="tweezers: over the stand")
    for _ in range(3):  # square the tips over the slot (a dither lets the joints past their friction)
        tc = tip_centre(ctx)
        err = np.array([sx - tc[0], sy - tc[1], stop + 0.008 - tip_bottom(ctx)])
        if np.linalg.norm(err[:2]) < 0.0004:
            break
        yield from dither_to(l, l.at() + err, amp=0.0015, t=1.0)
    floor = stop - cp.TW_SLOT[2]
    yield from l.move(l.at() + np.array([0, 0, floor + 0.0015 - tip_bottom(ctx)]), t=1.4, label="tweezers: into the slot")
    yield from l.wait(0.3)
    log(ctx, f"tweezers in the slot: tips {(tip_bottom(ctx) - floor) * 1000:.1f} mm over its floor, {np.round((tip_centre(ctx)[:2] - [sx, sy]) * 1000, 1)} mm off")
    yield from l.open(t=0.6)
    yield from l.move(l.at() + np.array([-0.10, 0, 0.0]), t=1.0, label="tweezers: let go")
    yield from l.move(l.at() + np.array([0, 0, 0.05]), t=0.6, label="tweezers: away")
    log(ctx, f"tweezers back in the stand, tilt {ctx.state('tweezers').tilt_deg():.1f} deg")


# part: (tip gap to come down at, how far past the part's edge the tips are squeezed)
GAPS = {"zinc_1": (0.016, 0.004), "zinc_2": (0.016, 0.004), "separator": (0.0235, 0.0015), "spacer": (0.0205, 0.0035),
        "spring": (0.0195, 0.0035), "cap": (0.0250, 0.003)}
DIAM = {"zinc_1": cp.ZINC_D, "zinc_2": cp.ZINC_D, "separator": cp.SEP_D, "spacer": cp.SPACER_D, "spring": cp.SPRING_D, "cap": cp.CAP_GASKET_D}
THICK = {"zinc_1": cp.ZINC_T, "zinc_2": cp.ZINC_T, "separator": cp.SEP_T, "spacer": cp.SPACER_T, "spring": cp.SPRING_H, "cap": cp.CAP_H}


def stack_top(ctx, upto):
    """The top of what already lies in the can, below `upto`."""
    order = ("zinc_1", "separator", "zinc_2", "spacer", "spring")
    top = T.FLOOR
    for n in order[:order.index(upto)] if upto in order else order:
        s = ctx.state(n)
        if np.linalg.norm(s.pos[:2] - np.asarray(T.CAN_XY)) < 0.009 and s.pos[2] < T.RIM + 0.001:
            top = max(top, s.pos[2] + THICK[n])
    return top


def watch(ctx, name, gen, label):
    k = 0
    for _ in gen:
        if VERBOSE and k % 3 == 0:
            q = ctx.state(name)
            tc = tip_centre(ctx)
            log(ctx, f"{name}: {label}: gap {tip_gap(ctx) * 1000:.2f} mm, tips {(tip_bottom(ctx) - T.LINER) * 1000:+.2f} mm, centre off {np.round((tc[:2] - q.pos[:2]) * 1000, 2)}, part {(q.pos[2] - T.LINER) * 1000:+.3f} mm tilt {q.tilt_deg():.1f}")
        k += 1
        yield


def dither_to(arm, goal, amp=0.0025, t=1.5, hz=3.0, start=None, vertical=False):
    """Settle the arm at `goal` (from `start`, if given, along a smooth path) through a circular dither in xy that dies
    away: joint friction would otherwise hold the arm a millimetre or two short, then let it jump."""
    n = max(1, int(t * 20))
    goal = np.asarray(goal, float)
    start = goal if start is None else np.asarray(start, float)
    for k in range(n + 1):
        a = amp * (1.0 - k / n) ** 2
        ph = 2 * math.pi * hz * k / 20
        u = min(1.0, 1.6 * k / n)
        p = start + (goal - start) * (3 * u * u - 2 * u ** 3)
        d = np.array([0.0, 0.0, a * math.sin(ph)]) if vertical else np.array([a * math.cos(ph), a * math.sin(ph), 0.0])
        arm._solve(p + arm.trim + d, arm.grip.rot)
        yield
    arm._solve(goal + arm.trim, arm.grip.rot)
    yield


def settle_flat(ctx, l, name, tries=2):
    """If a part came to rest tilted on the can's wall, press its raised edge down with the closed tips (re-seating a
    part with the tweezers, as a person would)."""
    for _ in range(tries):
        s = ctx.state(name)
        if s.tilt_deg() < 1.0:
            return
        up = s.rotation()[:, 2]
        high = -up[:2] / max(np.linalg.norm(up[:2]), 1e-9)  # the normal leans away from the raised edge
        r = {"separator": cp.SEP_D, "cap": cp.CAP_GASKET_D}.get(name, 0.012) / 2 - 0.0025
        target = s.pos[:2] + high * r
        log(ctx, f"{name}: tilted {s.tilt_deg():.1f} deg: pressing its raised edge down")
        yield from set_gap(ctx, l, 0.002)  # the tips together, to press one point
        yield from go_tips(ctx, l, target, T.RIM + 0.006, label=f"{name}: over the raised edge")
        seated_top = stack_top(ctx, name) + THICK[name]
        yield from l.move(l.at() + np.array([0, 0, seated_top + 0.0006 - tip_bottom(ctx)]), t=1.2, label=f"{name}: press the edge")
        yield from l.wait(0.3)
        yield from l.move(l.at() + np.array([0, 0, 0.02]), t=0.8, label=f"{name}: up")
        log(ctx, f"{name}: after the press: tilt {ctx.state(name).tilt_deg():.1f} deg; tweezers' tips {tip_gap(ctx) * 1000:.1f} mm apart")


def move_part(ctx, l, name):
    """Pick `name` off its dish with the tweezers and lay it in the can, centred."""
    d_open, over = GAPS[name]
    for attempt in range(3):
        p = ctx.state(name).pos
        yield from set_gap(ctx, l, d_open)
        yield from go_tips(ctx, l, p[:2], p[2] + max(THICK[name], 0.001) + 0.004, label=f"{name}: over")
        yield from watch(ctx, name, go_tips(ctx, l, ctx.state(name).pos[:2], T.LINER + 0.0005, t=0.8, label=f"{name}: down"), "down")
        for _ in range(2):  # square the tips over the part (a dither lets the joints past their friction)
            tc, q = tip_centre(ctx), ctx.state(name).pos
            yield from dither_to(l, l.at() + np.array([q[0] - tc[0], q[1] - tc[1], T.LINER + 0.0006 - tip_bottom(ctx)]), amp=0.0012, t=1.0)
        yield from watch(ctx, name, press_tips(ctx, l, T.LINER - 0.0005), "press")
        yield from l.wait(0.2)
        # squeeze, slowly, until the tips would stand `over` inside the part's edge
        w0, g0 = width(l), tip_gap(ctx)
        target = w0 - (g0 - (DIAM[name] - over)) * 0.345
        n = 32
        for k in range(1, n + 1):
            yield from set_width(l, w0 + (target - w0) * k / n, 0.05)
            if VERBOSE and k % 4 == 0:
                q = ctx.state(name)
                log(ctx, f"{name}: squeeze {k}/{n}: gap {tip_gap(ctx) * 1000:.2f} mm, tips {(tip_bottom(ctx) - T.LINER) * 1000:+.2f} mm, part {(q.pos[2] - T.LINER) * 1000:+.3f} mm tilt {q.tilt_deg():.1f}")
        yield from l.wait(0.3)
        yield from l.move(l.at() + np.array([0, 0, 0.010]), t=1.0, label=f"{name}: lift")
        if ctx.state(name).pos[2] > T.LINER + 0.005:
            break
        log(ctx, f"{name}: not taken (tip gap {tip_gap(ctx) * 1000:.2f} mm): again")
        yield from l.move(l.at() + np.array([0, 0, 0.010]), t=0.5, label=f"{name}: up")
    yield from l.move(l.at() + np.array([0, 0, 0.020]), t=0.6, label=f"{name}: lift")
    held = ctx.state(name).pos[2] - T.LINER
    log(ctx, f"{name}: held {held * 1000:.1f} mm over the lining, tip gap {tip_gap(ctx) * 1000:.2f} mm, tilt {ctx.state(name).tilt_deg():.1f} deg")
    # carry: the part's centre over the can's axis, then down
    axis = np.asarray(T.CAN_XY)
    if ctx.mode == "offcentre" and name == "zinc_2":  # a careless check: the upper zinc 1.5 mm off the axis
        axis = axis + np.array([0.0015, 0.0])
    c = part_centre(ctx, name)
    yield from l.move(l.at() + np.array([axis[0] - c[0], axis[1] - c[1], T.RIM + 0.012 - ctx.state(name).pos[2]]), t=1.6, label=f"{name}: carry")
    if ctx.state(name).pos[2] < T.RIM:
        log(ctx, f"{name}: dropped on the way")
        return
    under = ctx.state(name).pos[2] - tip_bottom(ctx)  # the tips reach this far below the part's bottom
    outer = tip_gap(ctx) + 2 * cp.TW_TIP_T
    if outer < cp.CAN_ID - 0.001:  # the tips fit in the can: set the part down just over what lies there
        bottom = stack_top(ctx, name) + max(0.0002, under + 0.0003)
    else:  # they do not: let go with the tips just over the rim
        bottom = max(T.RIM, stack_top(ctx, name)) + max(0.0002, under + 0.0003)
    def mean_centre(n=6):
        cs = []
        for _ in range(n):
            cs.append(np.r_[part_centre(ctx, name)[:2], ctx.state(name).pos[2]])
            yield
        return np.mean(cs, axis=0)

    # centre the part over the axis 4 mm up, then come down to just over the lowest safe point and let go. The arm's
    # joints stick (joint friction) by a millimetre or two, so every move settles through a dither that dies away
    for label, height, tries in (("centre", bottom + 0.004, 6), ("down", bottom + (0.0015 if outer < cp.CAN_ID - 0.001 else 0.0002), 1 if outer < cp.CAN_ID - 0.001 else 4)):
        c = yield from mean_centre()
        goal = l.at() + np.array([axis[0] - c[0], axis[1] - c[1], height - c[2]])
        start = l.at()
        inside = label == "down" and outer < cp.CAN_ID - 0.001
        yield from dither_to(l, goal, start=start, t=2.0, amp=0.0008 if inside else 0.0025)
        for k in range(tries):
            c = yield from mean_centre()
            err = np.array([axis[0] - c[0], axis[1] - c[1], height - c[2]])
            if VERBOSE:
                log(ctx, f"{name}: {label} {k}: off {np.round(err * 1000, 2)} mm")
            if np.linalg.norm(err[:2]) < (0.0001 if label == "centre" else 0.00015) and abs(err[2]) < 0.0004:
                break
            if label == "down" and outer < cp.CAN_ID - 0.001:
                break  # inside the can: let go now, before the tips can touch what lies there
            if label == "down":
                err[2] = max(err[2], 0.0)  # never lower: the part is near what it will lie on
            goal = goal + err * 0.9
            yield from dither_to(l, goal, amp=0.002)
    c = part_centre(ctx, name)
    log(ctx, f"{name}: let go {np.linalg.norm(c[:2] - axis) * 1000:.2f} mm off the axis, {(ctx.state(name).pos[2] - stack_top(ctx, name)) * 1000:.2f} mm up")
    if outer < cp.CAN_ID - 0.001:
        yield from set_width(l, width(l) + 0.0015, 0.4)
    else:  # over the rim: open slowly, so that neither tip flicks the part as it lets go
        yield from set_width(l, width(l) + 0.0012, 1.2)
    yield from l.wait(0.5)
    yield from l.move(l.at() + np.array([0, 0, 0.015]), t=0.6, label=f"{name}: up")
    yield from l.move(l.at() + np.array([0, 0, 0.020]), t=0.5, label=f"{name}: up")
    yield from settle_flat(ctx, l, name)
    s = ctx.state(name)
    off = np.linalg.norm(part_centre(ctx, name)[:2] - axis) * 1000
    log(ctx, f"{name}: in the can {off:.2f} mm off the axis, tilt {s.tilt_deg():.2f} deg, {(s.pos[2] - T.FLOOR) * 1000:.2f} mm over the floor")


# -- the pipette -------------------------------------------------------------------------------------------------------


def press(ctx, r, target, l=None, tip_goal=None, rate=0.004, label="press"):
    """Push the plunger with the underside of the closed right fingers until its travel is `target`; the left hand
    meanwhile holds the tip at `tip_goal` (the left arm gives under the push, so it servoes)."""
    for _ in range(200):
        travel = -ctx.state("pipette").joint("plunger")
        err = target - travel
        tip_err = np.zeros(3) if l is None else np.asarray(tip_goal) - ctx.state("pipette").pos
        if abs(err) < 0.00015 and np.linalg.norm(tip_err) < 0.0005:
            break
        step = float(np.clip(0.5 * err, -rate / 10, rate / 10))
        dl = np.clip(0.25 * tip_err, -0.0005, 0.0005)
        gens = [r.move(r.at() + np.array([0, 0, -step + dl[2]]), t=0.05, label=label)]
        if l is not None:
            gens.append(l.move(l.at() + dl, t=0.05, label=label))
        yield from ctx.together(*gens)
    log(ctx, f"{label}: plunger {-ctx.state('pipette').joint('plunger') * 1000:.2f} mm down, tip at {np.round((ctx.state('pipette').pos - [0, 0, T.TOP]) * 1000, 1)}")


def tip_to(ctx, l, goal, t=None, r=None, label="tip"):
    """Move the pipette's tip to `goal` with the left hand; the right hand, if given, moves with it."""
    for k in range(2):
        d = np.asarray(goal) - ctx.state("pipette").pos
        gens = [l.move(l.at() + d, t=t if k == 0 else 0.5, label=label)]
        if r is not None:
            gens.append(r.move(r.at() + d, t=t if k == 0 else 0.5, label=label))
        yield from ctx.together(*gens)


def button(ctx):
    return ctx.state("pipette").to_world(np.array([[0.0, 0.0, cp.PIP_BUTTON_Z]]))[0]


def electrolyte(ctx, l, r):
    p = ctx.state("pipette")
    grip = p.pos + np.array([0, 0, cp.PIP_GRIP_Z])
    yield from l.move(grip + np.array([-0.06, 0, 0.05]), grip=G, label="pipette: approach")
    yield from l.move(grip + np.array([-0.06, 0, 0.0]), t=0.8, label="pipette: level")
    yield from l.move(grip, t=1.2, label="pipette: in")
    yield from l.servo(grip, tol=0.0015, timeout=1.0, label="pipette: in")
    yield from l.close(width=0.018, force=30.0)
    yield from l.move(l.at() + [0, 0, 0.015], t=0.8, label="pipette: off the cradle")
    yield from l.move(l.at() + [-0.05, 0, 0.01], t=1.0, label="pipette: out")
    tube = np.array([*T.RACK_XY, T.Z0 + cp.TUBE_SEAT + cp.TUBE_H + 0.012])
    clear = np.array([0.37, -0.05, tube[2]])  # round the far side of the tweezer stand and the mould
    yield from tip_to(ctx, l, clear, t=1.6, label="pipette: past the stand")
    yield from tip_to(ctx, l, tube, t=2.0, label="pipette: over the tube")
    # the right hand comes over the button from above and behind, fingers closed
    yield from r.move(r.at() + np.array([-0.05, 0, 0.13]), grip=G, finger=0.0, label="right: up")
    b = button(ctx)
    yield from r.move(np.array([b[0] - 0.06, b[1], b[2] + 0.05]), label="right: behind")
    yield from r.move(b + np.array([0, 0, 0.045]), label="right: over the button")
    yield from r.move(b + np.array([0, 0, 0.022]), t=1.0, label="right: on the button")
    hold = ctx.state("pipette").pos.copy()
    yield from press(ctx, r, cp.PIP_FIRST_STOP + 0.0002, l=l, tip_goal=hold, label="to the first stop")
    dip = np.array([*T.RACK_XY, T.LIQUID_Z - 0.008])
    yield from tip_to(ctx, l, dip, t=1.5, r=r, label="pipette: dip")
    yield from press(ctx, r, 0.0, l=l, tip_goal=dip, rate=0.002, label="draw")
    yield from l.wait(0.5)
    log(ctx, f"drawn: tip holds {T.TRIAL.pip.tip_ul} uL")
    yield from r.move(r.at() + np.array([0, 0, 0.015]), t=0.6, label="right: off the button")
    yield from tip_to(ctx, l, tube + np.array([0, 0, 0.01]), t=1.2, r=r, label="pipette: out of the tube")
    sep = ctx.state("separator")
    over = np.array([*T.CAN_XY, max(sep.pos[2], T.FLOOR + cp.ZINC_T) + cp.SEP_T + 0.0035])
    yield from tip_to(ctx, l, over + np.array([0, 0, 0.02]), t=2.0, r=r, label="pipette: to the can")
    yield from tip_to(ctx, l, over, t=1.0, r=r, label="pipette: over the separator")
    b = button(ctx)
    yield from r.move(b + np.array([0, 0, 0.022]), t=0.8, label="right: on the button")
    yield from press(ctx, r, cp.PIP_FIRST_STOP + 0.0002, l=l, tip_goal=over, label="dispense")
    # withdraw the tip from the wetted separator with the plunger still held, then let it up away from the liquid
    lifted = over + np.array([0, 0, 0.015])
    yield from tip_to(ctx, l, lifted, t=0.8, r=r, label="pipette: withdraw, plunger held")
    yield from press(ctx, r, 0.0, l=l, tip_goal=lifted, rate=0.004, label="let up")
    log(ctx, f"dispensed: {T.TRIAL.pip.dispenses}")
    yield from r.move(r.at() + np.array([0, 0, 0.04]), t=0.8, label="right: off")
    yield from r.move(r.at() + np.array([-0.08, -0.05, 0.0]), t=1.0, label="right: away")
    # back to the holder: over the cradle's open side, in, down onto the cradle
    home = np.asarray(T.PIP_HOME)
    yield from tip_to(ctx, l, np.array([0.37, 0.0, home[2] + 0.03]), t=1.6, label="pipette: past the stand")
    yield from tip_to(ctx, l, home + np.array([-0.05, 0, 0.03]), t=1.6, label="pipette: to the holder")
    yield from tip_to(ctx, l, home + np.array([0.002, 0, 0.012]), t=1.2, label="pipette: over the cradle")
    yield from tip_to(ctx, l, home + np.array([0.002, 0, 0.0005]), t=0.8, label="pipette: down")
    yield from l.open(t=0.6)
    yield from l.move(l.at() + np.array([-0.06, 0, 0.0]), t=1.0, label="pipette: let go")
    yield from l.move(l.at() + np.array([0, 0, 0.04]), t=0.6, label="left: away")
    log(ctx, f"pipette home: {np.round((ctx.state('pipette').pos - home) * 1000, 2)} mm from home")


def press_cap(ctx, r):
    c = ctx.state("cap")
    top = c.pos[2] + cp.CAP_H
    over = np.array([*T.CAN_XY, top + 0.013 + 0.05])
    yield from r.move(over, grip=down(90), finger=0.0, label="right: over the cap")
    yield from r.move(over - np.array([0, 0, 0.048]), t=1.2, label="right: onto the cap")
    yield from r.move(r.at() - np.array([0, 0, 0.0032]), t=1.0, label="right: press")
    yield from r.wait(0.5)
    log(ctx, f"cap pressed: {(ctx.state('cap').pos[2] - T.RIM) * 1000:+.2f} mm against the rim, pressed {T.TRIAL.judge.pressed}")
    yield from r.move(r.at() + np.array([0, 0, 0.08]), t=1.2, label="right: off the cap, up clear")
    yield from r.move(r.at() + np.array([-0.06, -0.08, 0.0]), t=1.0, label="right: away")


def script(ctx, task):
    global T
    T = task
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "only_pipette":  # development: the pipetting alone
        yield from electrolyte(ctx, l, r)
        yield from l.wait(1.0)
        return
    if mode == "touch":  # a careless check: the right gripper's fingertips push the second zinc disc on its dish
        z2 = ctx.pos("zinc_2")
        yield from r.move(z2 + np.array([0.012, 0.0, 0.03]), grip=down(90), finger=0.0, label="right: over the zinc")
        yield from r.move(z2 + np.array([0.012, 0.0, 0.0128]), t=1.0, label="right: down")
        yield from r.move(r.at() + np.array([-0.008, 0.0, 0.0]), t=1.0, label="right: push")
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.05]), t=0.8, label="right: up")
        yield from r.move(r.at() + np.array([-0.05, -0.05, 0.0]), t=0.8, label="right: away")
        log(ctx, f"touched: {T.TRIAL.judge.gripper_moved[:2]}")
    yield from take_tweezers(ctx, l)
    if mode.startswith("only_"):  # development: one part only
        yield from move_part(ctx, l, mode[5:])
        return
    yield from move_part(ctx, l, "zinc_1")
    yield from move_part(ctx, l, "separator")
    yield from stow_tweezers(ctx, l)
    yield from electrolyte(ctx, l, r)
    yield from take_tweezers(ctx, l)
    for name in ("zinc_2", "spacer", "spring", "cap"):
        if mode == "skip_spacer" and name == "spacer":  # a careless check: a part left out
            continue
        yield from move_part(ctx, l, name)
    yield from stow_tweezers(ctx, l)
    if mode != "no_press":  # a careless check: the cap never pressed
        yield from press_cap(ctx, r)
    log(ctx, f"done: {T.TRIAL.report}")
    yield from l.wait(2.0)  # both hands away: the trial succeeds once the fingers have been clear of the tools and the cell for 1 s
