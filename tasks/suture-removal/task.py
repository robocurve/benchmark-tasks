"""Take out an interrupted suture: forceps lift the knot, a stitch cutter cuts the far leg flush, then the stitch is drawn out toward the wound.

A suture practice pad (silicone skin with a closed incision) sits on a block; one interrupted 3-0 nylon stitch crosses the incision, its knot on one
side. Dissecting forceps stand tips-down in a foam rest at the left; a single-use stitch cutter (a steel blade with a hook, no handle) stands hook-down
in a foam rest at the right; a gauze square lies on the table.

Sim (stated, all in code): the pad's knot-side half rides on a stiff spring across the incision, so a pull away from the wound opens it. The stitch is a
rigid strand (drawn 0.6 mm thick, no collider) on a lift joint normal to the skin and a hinge about the centre of its buried arc; the forceps' tips, closed
on the knot, set the lift (while uncut) and the draw (once cut), gated on geometry. A cut is the leg entering the cutter's notch through its mouth under
the lifted loop and pressed onto its sharp inner edge, read in the cutter's frame; its height above the skin is recorded. The forceps' arms are a sprung hinge that the gripper's squeeze closes.

    python demokit.py task.py OUT --mode scene --stills 1
    python demokit.py task.py OUT --mode full
    python check_score.py
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Take out the stitch: lift the knot with the forceps, cut the leg away from the knot flush with the skin with the stitch cutter, "
               "draw the stitch out toward the wound, put it on the gauze and the instruments back in their trays, and take your hands away from the pad.")
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 40  # 2 s at 20 Hz

PAD_C = np.array([0.32, 0.0])
BLOCK_H, SKIN_T = 0.022, 0.008
BLOCK = (0.14, 0.10)  # the block under the pad, x by y
SKIN = TOP + BLOCK_H + SKIN_T  # the skin's surface
R_ARC = 0.004  # the stitch's bite: it leaves the skin 4 mm either side of the incision
KNOT_Y = 0.0045
FORCEPS_REST = np.array([0.34, 0.16])
CUTTER_REST = np.array([0.30, -0.15])
GAUZE = np.array([0.24, 0.20])
TIP_L = 0.07  # the instruments' tips beyond where the hand holds them
FOOT_H, NOTCH_H = 0.0008, 0.0035  # the cutter's hook: its foot 0.8 mm thick, the notch above it 3.5 mm high
DIG = 0.002  # the blunt foot pressed this far into the soft silicone goes through it
BAND = 0.0009  # the leg within this of the blade's plane: the strand touching the 1.2 mm steel
NOTCH_X0, NOTCH_X1, SPINE_X1 = -0.002, 0.001, 0.003  # across the cutter (its x): the notch's open mouth, its sharp inner edge, the spine's back
TRAY = (0.16, 0.09)  # the steel tray under each foam rest
FORCEPS_PRE = 0.08  # rad: the forceps' arms spring open against their stop with this much preload

LOOKS = {"skin": m.Look(color=m.rgb(0.86, 0.66, 0.56), roughness=0.8), "block": m.Look(color=m.rgb(0.25, 0.25, 0.27), roughness=0.7),
         "slit": m.Look(color=m.rgb(0.45, 0.20, 0.18), roughness=0.8), "nylon": m.Look(color=m.rgb(0.05, 0.10, 0.45), roughness=0.4),
         "steel": m.Look(color=m.rgb(0.80, 0.81, 0.83), metallic=1.0, roughness=0.25), "foam": m.Look(color=m.rgb(0.20, 0.35, 0.60), roughness=0.95),
         "gauze": m.Look(color=m.rgb(0.97, 0.97, 0.95), roughness=0.95), "tray": m.Look(color=m.rgb(0.75, 0.78, 0.80), metallic=0.6, roughness=0.4)}


def _pad() -> m.Part:
    """The block and the pad's fixed half (-y), its knot-side half (+y) on a stiff slide across the incision, and the stitch on that half."""
    z0 = BLOCK_H + SKIN_T / 2
    frame = [("block", m.box((0.14, 0.10, BLOCK_H), (0.0, 0.0, BLOCK_H / 2)), "block"),
             ("skin_a", m.box((0.12, 0.04, SKIN_T), (0.0, -0.0201, z0)), "skin"),
             ("slit", m.box((0.05, 0.0006, 0.0004), (0.0, 0.0, BLOCK_H + SKIN_T + 0.0001)), "slit")]
    cols = [m.col_box("c_block", (0.14, 0.10, BLOCK_H), (0.0, 0.0, BLOCK_H / 2)), m.col_box("c_skin_a", (0.12, 0.04, SKIN_T), (0.0, -0.0201, z0))]
    half = m.slider("half", "Y", 0.0, 0.004, [("skin_b", m.box((0.12, 0.04, SKIN_T), (0.0, 0.0201, z0)), "skin")],
                    [m.col_box("c_skin_b", (0.12, 0.04, SKIN_T), (0.0, 0.0201, z0))], mass=0.05, spring=3000.0, damping=20.0, friction=0.0, effort=200.0)
    zs = BLOCK_H + SKIN_T
    lift = m.slider("lift", "Z", 0.0, 0.006, [("lift_mark", m.box((0.0005, 0.0005, 0.0005), (0.0, KNOT_Y, zs - 0.002)), "skin")], [], parent="half",
                    mass=0.002, spring=200.0, damping=2.0, friction=0.0, effort=50.0)
    # the stitch: knot, the loop over the incision, the two legs into the skin, and the buried arc (a semicircle under the incision)
    parts = [("knot", m.box((0.0025, 0.0025, 0.002), (0.0, KNOT_Y, zs + 0.0012)), "nylon"),
             ("loop", m.box((0.0006, 2 * R_ARC + 0.001, 0.0006), (0.0, 0.0, zs + 0.0008)), "nylon"),
             ("near_leg", m.box((0.0006, 0.0006, 0.0012), (0.0, R_ARC, zs + 0.0002)), "nylon"),
             ("far_leg", m.box((0.0006, 0.0006, 0.0012), (0.0, -R_ARC, zs + 0.0002)), "nylon")]
    for k, a in enumerate(np.linspace(math.radians(195), math.radians(345), 9)):
        parts.append((f"arc_{k}", m.box((0.0006, 0.0018, 0.0006), (0.0, R_ARC * math.cos(a), zs + R_ARC * math.sin(a))), "nylon"))
    draw = m.hinge("draw", "X", (0.0, 0.0, zs), 0.0, 200.0, parts, [], parent="lift", mass=0.002, spring=2.0, damping=0.05, friction=0.0, effort=20.0)
    return m.mechanism("pad", (PAD_C[0], PAD_C[1], TOP), [half, lift, draw], LOOKS, frame_parts=frame, frame_colliders=cols, friction=(1.0, 0.9))


def _forceps() -> m.Part:
    """Dissecting forceps, 12 cm, standing tips-down in its rest; origin where the hand holds them, 7 cm above the tips. Arm A is the frame; arm B turns
    about the joined back end and springs open (tips 4 mm apart) against a stop; a squeeze closes it."""
    back = 0.05
    frame = [("arm_a", m.box((0.006, 0.0018, back + TIP_L), (0.0, 0.0029, (back - TIP_L) / 2)), "steel"),
             ("butt", m.box((0.006, 0.008, 0.008), (0.0, 0.0, back - 0.004)), "steel")]
    cols = [m.col_box("c_arm_a", (0.006, 0.0018, back + TIP_L), (0.0, 0.0029, (back - TIP_L) / 2))]
    arm_b = m.hinge("arm_b", "X", (0.0, 0.0, back), 0.0, math.degrees(0.034), [("arm_b", m.box((0.006, 0.0018, back + TIP_L), (0.0, -0.0029, (back - TIP_L) / 2)), "steel")],
                    [m.col_box("c_arm_b", (0.006, 0.0018, back + TIP_L), (0.0, -0.0029, (back - TIP_L) / 2))], mass=0.008, spring=1.0, damping=0.02,
                    friction=0.0, effort=10.0)
    z = TOP + 0.004 + TIP_L + 0.0005  # tips on the rest's floor
    return m.mechanism("forceps", (FORCEPS_REST[0], FORCEPS_REST[1], z), [arm_b], LOOKS, frame_parts=frame, frame_colliders=cols, free_mass=0.012,
                       friction=(0.9, 0.8), frame_inertia=(1.5e-5, 1.5e-5, 1e-6))


def _cutter() -> m.Part:
    """A single-use stitch cutter: a flat steel blade standing hook-down, its origin where the hand pinches its flat body (faces along y). At its end, in
    the blade's own plane, a hooked notch opens toward -x: a foot 0.8 mm thick runs under it, and its inner edge (at x 1 mm) is the sharp one."""
    zb = -TIP_L + FOOT_H + NOTCH_H  # the notch's roof: the body's lower end
    parts = [("blade", m.box((0.006, 0.0012, 0.02 - zb), (0.0, 0.0, (0.02 + zb) / 2)), "steel"),
             ("spine", m.box((SPINE_X1 - NOTCH_X1, 0.0012, NOTCH_H + FOOT_H), ((SPINE_X1 + NOTCH_X1) / 2, 0.0, -TIP_L + (NOTCH_H + FOOT_H) / 2)), "steel"),
             ("foot", m.box((SPINE_X1 - NOTCH_X0, 0.0012, FOOT_H), ((SPINE_X1 + NOTCH_X0) / 2, 0.0, -TIP_L + FOOT_H / 2)), "steel")]
    cols = [m.col_box("c_blade", (0.006, 0.002, 0.02 - (-TIP_L + 0.006)), (0.0, 0.0, (0.02 + (-TIP_L + 0.006)) / 2))]  # ends 6 mm above the tip
    return m.free("cutter", (CUTTER_REST[0], CUTTER_REST[1], TOP + 0.004 + TIP_L + 0.0005), parts, cols, LOOKS, mass=0.006)


def _rests() -> m.Part:
    shapes = []
    for nm, (x, y), half_w in (("rest_f", FORCEPS_REST, 0.0085), ("rest_c", CUTTER_REST, 0.0055)):
        for k, dy in enumerate((-half_w, half_w)):  # two foam cheeks either side of the slot the instrument stands in (10 mm for the forceps, 4 mm for the cutter)
            shapes.append((f"{nm}_{k}", (0.04, 0.007, 0.03), (x, y + dy, TOP + 0.015), "foam"))
        shapes.append((f"{nm}_floor", (0.04, 0.02, 0.004), (x, y, TOP + 0.002), "foam"))
    for nm, (x, y) in (("tray_f", FORCEPS_REST), ("tray_c", CUTTER_REST)):
        shapes.append((nm, (TRAY[0], TRAY[1], 0.002), (x, y, TOP + 0.001), "tray"))
        for k, (dx, dy, sx, sy) in enumerate(((TRAY[0] / 2, 0, 0.002, TRAY[1]), (-TRAY[0] / 2, 0, 0.002, TRAY[1]), (0, TRAY[1] / 2, TRAY[0], 0.002), (0, -TRAY[1] / 2, TRAY[0], 0.002))):
            shapes.append((f"{nm}_rim{k}", (sx, sy, 0.012), (x + dx, y + dy, TOP + 0.006), "tray"))
    shapes.append(("gauze", (0.075, 0.075, 0.002), (GAUZE[0], GAUZE[1], TOP + 0.001), "gauze"))
    parts = [(nm, m.box(s, c), look) for nm, s, c, look in shapes]
    cols = [m.col_box(f"c_{nm}", s, c) for nm, s, c, look in shapes]
    parts.append(("suture_out", m.box((0.0006, 0.022, 0.0006), (GAUZE[0], GAUZE[1], TOP + 0.0025)), "nylon"))
    parts.append(("knot_out", m.box((0.0025, 0.0025, 0.002), (GAUZE[0], GAUZE[1] + 0.011, TOP + 0.003)), "nylon"))
    return m.fixed("rests", (0.0, 0.0, 0.0), parts, cols, LOOKS, friction=(1.0, 0.9))


OBJECTS = [_pad(), _forceps(), _cutter(), _rests()]


# -- the gates and the scorer ---------------------------------------------------------------------------------------------------------------------

def forceps_tips(env):
    f = m.state(env, "forceps")
    return f.to_world(np.array([[0.0, 0.0, -TIP_L]]))[0], f, float(f.joint("arm_b"))


def hook_point(env):
    """The middle of the notch's floor, in the station frame, and the cutter's state."""
    c = m.state(env, "cutter")
    return c.to_world(np.array([[(NOTCH_X0 + NOTCH_X1) / 2, 0.0, -TIP_L + FOOT_H]]))[0], c


def cutter_low_corners(c):
    """The cutter's lowest corners: the foot's bottom and the blade body's lower end (neither has a collider)."""
    zb = -TIP_L + FOOT_H + NOTCH_H
    pts = [(x, y, -TIP_L) for x in (NOTCH_X0, SPINE_X1) for y in (-0.0006, 0.0006)] + [(x, y, zb) for x in (-0.003, 0.003) for y in (-0.0006, 0.0006)]
    return c.to_world(np.array(pts))


def hook_beneath(st, h, lift, gap):
    """Whether the cutter's hook is under the stitch, reached the way a hook can get there: slid in through a side of the region under the stitch (from
    beside the strand or from beyond a leg) at the height under the lifted loop, never down from above it, at any speed (the strand has no collider, so
    a hook could otherwise pass down through it). The region: within 3 mm of the strand's line across, from 2 mm beyond the far leg to 2 mm beyond the
    near leg (its outline), from 2 mm into the silicone up to 0.3 mm under the lifted loop. h: the hook relative to the pad's middle (x, y) and its
    height. st: (beneath, inside, the last h, above): above is set while the hook is over the outline higher than the loop, and cleared only when the
    hook leaves the outline. Returns the new st."""
    beneath, was_in, prev, above = st
    hx, hy, hz = h
    top = SKIN + lift + 0.0003
    in_outline = lambda x, y: bool(abs(x) < 0.003 and -R_ARC - 0.002 <= y <= R_ARC + gap + 0.002)  # noqa: E731
    inside = in_outline(hx, hy) and SKIN - DIG <= hz <= top
    if inside and not was_in:  # entering: through a side, level, and not after being over the loop
        beneath = bool(prev is not None and not in_outline(prev[0], prev[1]) and prev[2] <= top + 0.0005 and not above)
    if not inside:
        beneath = False
    if in_outline(hx, hy):
        above = above or hz > top
    else:
        above = False
    return beneath, inside, (hx, hy, hz), above


def notch_step(st, pts_local, ready):
    """One leg and the cutter's notch, read in the cutter's own frame. pts_local: points up the leg from the skin to the loop, in the cutter's frame.
    In the band: within 0.9 mm of the blade's plane (half its 1.2 mm thickness and the strand's radius: the strand touching the steel) and between the
    notch's floor and roof. Entered: a point inside the notch (between its mouth and
    its inner edge) the step after the leg was beyond the mouth, on the open side, while `ready` (the hook beneath the lifted stitch). Cut: entered,
    then the leg pressed onto the inner edge (within 0.4 mm of it). Moving off the leg (no point in the band near the notch) undoes the entry.
    st: (the leg beyond the mouth last step, entered). Returns (st, the index of the point cut, or None)."""
    prev_open, entered = st
    p = np.asarray(pts_local)
    band = (np.abs(p[:, 1]) <= BAND) & (p[:, 2] >= -TIP_L + FOOT_H) & (p[:, 2] <= -TIP_L + FOOT_H + NOTCH_H)
    inside = band & (p[:, 0] >= NOTCH_X0) & (p[:, 0] <= NOTCH_X1)
    open_side = band & (p[:, 0] < NOTCH_X0) & (p[:, 0] > NOTCH_X0 - 0.004)
    edge = band & (p[:, 0] >= NOTCH_X1 - 0.0004) & (p[:, 0] <= SPINE_X1)
    if not entered and inside.any() and prev_open and ready:
        entered = True
    if not (band & (p[:, 0] > NOTCH_X0 - 0.004) & (p[:, 0] <= SPINE_X1)).any():
        entered = False
    cut = int(np.flatnonzero(edge)[0]) if entered and edge.any() else None
    return (bool(open_side.any()), entered), cut


def let_go(cut, drawn, tip):
    """The forceps opened. Over the gauze (tips within 3.5 cm of its middle, under 5 cm up) with the stitch drawn out, the stitch lands on it. After the
    cut, opened anywhere else, the stitch drops where the tips open: it can no longer be picked up at the knot's old place (it is not there) or carried
    to the gauze. Before the cut the knot is still in the skin, and may be taken again. Returns (on the gauze, dropped)."""
    on = bool(drawn >= 0.95 and abs(tip[0] - GAUZE[0]) < 0.035 and abs(tip[1] - GAUZE[1]) < 0.035 and tip[2] < TOP + 0.05)
    return on, bool(cut and not on)


def at_rest(points, tips, floor_z, footprint, gauze):
    """An instrument back in its tray: let go (every fingertip of both hands 3 cm or more from points along it), lying or standing on the tray (its
    lowest point within 1.5 cm of the tray's floor), its middle inside the tray's outline and not on the gauze. points: (n, 3) along the instrument;
    footprint and gauze: (x, y, half x, half y)."""
    points, tips = np.asarray(points), np.asarray(tips)
    let_go = float(np.min(np.linalg.norm(tips[:, None, :] - points[None, :, :], axis=2))) >= 0.03
    mid = points.mean(axis=0)
    inside = lambda r: abs(mid[0] - r[0]) < r[2] and abs(mid[1] - r[1]) < r[3]  # noqa: E731
    return bool(let_go and float(points[:, 2].min()) < floor_z + 0.015 and inside(footprint) and not inside(gauze))


class Stitch:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.held, self.lifted_max, self.broke, self.cut, self.cut_h, self.cut_side = 0, False, 0.0, False, False, None, None
        self.notch, self.gap_max, self.drawn, self.on_gauze, self.dug, self.hold, self.full, self.last = None, 0.0, 0.0, False, False, 0, False, {}
        self.shown, self.hook_st, self.open_near, self.stitch_dropped = None, (False, False, None, False), -1000, False
        self.notch = {"far": (False, False), "near": (False, False)}
        self.floor_min = {"far": 1.0, "near": 1.0}

    def step(self, env):
        self.steps += 1
        m.set_target(env, "forceps", "arm_b", -FORCEPS_PRE)  # the arms' spring: rest beyond the open stop, a preload
        pad = m.state(env, "pad")
        lift, draw, gap = float(pad.joint("lift")), float(pad.joint("draw")), float(pad.joint("half"))
        knot0 = np.array([PAD_C[0], PAD_C[1] + KNOT_Y + gap, SKIN + 0.0012])
        knot = knot0 + np.array([0.0, 0.0, lift])
        tip, f, fb = forceps_tips(env)
        closed = fb > 0.8 * 0.034
        if not closed and np.linalg.norm(tip - knot) < 0.005:
            self.open_near = self.steps  # open, the tips at the knot
        if closed and not self.held and np.linalg.norm(tip - knot) < 0.0025 and self.steps - self.open_near <= 10 and not self.on_gauze and not self.stitch_dropped:  # closed on the knot within 0.5 s of being open there, not touched to it shut
            self.held = True
        if self.held and not closed:
            self.held = False
            on, dropped = let_go(self.cut, self.drawn, tip)
            self.on_gauze = self.on_gauze or on
            self.stitch_dropped = self.stitch_dropped or dropped
        # the knot follows the closed tips: lift while uncut (a pull away from the wound opens it), the draw once cut
        rel = tip - knot0
        if self.held and not self.cut:
            up = float(np.clip(rel[2], 0.0, 0.006))
            if rel[2] > 0.0045:
                self.broke = True  # pulled up past what the stitch takes before it is cut: torn through the skin
            m.set_target(env, "pad", "lift", up)
            m.set_target(env, "pad", "half", float(np.clip(rel[1] - 0.0005, 0.0, 0.004)))
        elif self.held and self.cut and self.drawn < 0.95:  # once drawn out, the stitch is free of the skin (carried to the gauze, it would open the wound)
            away = float(np.clip(rel[1] - 0.0005, 0.0, 0.004))  # pulled away from the wound, the gap opens
            m.set_target(env, "pad", "half", away)
            progress = float(np.clip(np.linalg.norm(rel - np.array([0.0, 0.0, lift])) / 0.012, 0.0, 1.0)) if rel[1] <= 0.001 else self.drawn
            if rel[1] > -0.003:
                progress = min(progress, 0.9)  # drawn out only once the tips have gone 3 mm toward the wound
            self.drawn = max(self.drawn, progress)
            m.set_target(env, "pad", "draw", math.radians(180.0) * self.drawn)
        else:
            m.set_target(env, "pad", "lift", 0.0 if not self.cut else lift)
            m.set_target(env, "pad", "half", 0.0)
            m.set_target(env, "pad", "draw", math.radians(180.0) * self.drawn)
        self.lifted_max = max(self.lifted_max, lift) if not self.cut else self.lifted_max
        self.gap_max = max(self.gap_max, gap)
        # the cut: the hook under the stitch, the leg into its notch through the mouth and onto its inner edge (notch_step)
        hook, c = hook_point(env)
        hx, hy, hz = hook[0] - PAD_C[0], hook[1] - (PAD_C[1] + gap * 0.0), hook[2]
        low = cutter_low_corners(c)
        on_skin = (np.abs(low[:, 0] - PAD_C[0]) < 0.06) & (np.abs(low[:, 1] - PAD_C[1]) < 0.04)
        if (on_skin & (low[:, 2] < SKIN - DIG)).any():
            self.dug = True  # any of the cutter's lowest corners more than 2 mm into the silicone, anywhere on the skin: pressed through it
        self.hook_st = hook_beneath(self.hook_st, (hx, hy, hz), lift, gap)
        if self.cut_side != "both":
            ready = bool(self.hook_st[0] and lift >= 0.001)
            for leg, ly in (("far", PAD_C[1] - R_ARC), ("near", PAD_C[1] + R_ARC + gap)):
                if leg == self.cut_side:
                    continue
                pw = np.array([[PAD_C[0], ly, z] for z in np.linspace(SKIN, SKIN + lift + 0.0008, 6)])
                self.notch[leg], k = notch_step(self.notch[leg], c.to_local(pw), ready)
                self.floor_min[leg] = min(self.floor_min[leg], hz) if self.notch[leg][1] else 1.0  # the notch's lowest while the leg is in it
                if k is None:
                    continue
                if self.cut:
                    self.cut_side = "both"  # the other leg cut too: the buried part is left in the skin
                else:
                    self.cut, self.cut_h, self.cut_side = True, max(float(self.floor_min[leg] - SKIN), 0.0), leg  # cut just over the foot, flush at best
                try:
                    m.set_visible(f"{m.ENV_PATH}/pad/draw/visual/{leg}_leg", False)
                except Exception:
                    pass
        if self.shown != self.on_gauze:
            self.shown = self.on_gauze
            for nm in ("knot", "loop", "near_leg", "far_leg") + tuple(f"arc_{k}" for k in range(9)):
                try:
                    m.set_visible(f"{m.ENV_PATH}/pad/draw/visual/{nm}", not self.on_gauze and not (nm.endswith("_leg") and self.cut and nm.startswith(self.cut_side or "x")))
                except Exception:
                    pass
            for nm in ("suture_out", "knot_out"):
                try:
                    m.set_visible(f"{m.ENV_PATH}/rests/visual/{nm}", self.on_gauze)
                except Exception:
                    pass
        tips = m.fingertips(env)
        gz = (GAUZE[0], GAUZE[1], 0.0375, 0.0375)
        along = np.linspace(0.0, 1.0, 5)[:, None]
        f_pts = f.to_world(along * np.array([[0.0, 0.0, -TIP_L]]) + (1 - along) * np.array([[0.0, 0.0, 0.05]]))  # the forceps, back end to tips
        c_pts = c.to_world(along * np.array([[0.0, 0.0, -TIP_L]]) + (1 - along) * np.array([[0.0, 0.0, 0.02]]))  # the cutter, top to hook
        in_rest_f = at_rest(f_pts, tips, TOP + 0.004, (FORCEPS_REST[0], FORCEPS_REST[1], TRAY[0] / 2, TRAY[1] / 2), gz)
        in_rest_c = at_rest(c_pts, tips, TOP + 0.004, (CUTTER_REST[0], CUTTER_REST[1], TRAY[0] / 2, TRAY[1] / 2), gz)
        out_x = np.maximum(np.abs(tips[:, 0] - PAD_C[0]) - BLOCK[0] / 2, 0.0)
        out_y = np.maximum(np.abs(tips[:, 1] - PAD_C[1]) - BLOCK[1] / 2, 0.0)
        clear = bool(np.min(np.hypot(out_x, out_y)) > 0.04)  # every fingertip of both hands 4 cm outside the pad block's outline
        self.last = {"t": round(self.steps * float(env.unwrapped.step_dt), 2), "held": self.held, "closed": bool(closed), "forceps_mm": round(fb * 120, 2),
                     "lift_mm": round(lift * 1000, 2), "lifted_max_mm": round(self.lifted_max * 1000, 2), "gap_mm": round(gap * 1000, 2), "gap_max_mm": round(self.gap_max * 1000, 2),
                     "cut": self.cut, "cut_h_mm": None if self.cut_h is None else round(self.cut_h * 1000, 2), "cut_side": self.cut_side,
                     "drawn": round(self.drawn, 3), "draw_deg": round(math.degrees(draw), 1), "on_gauze": self.on_gauze, "broke": self.broke, "dug": self.dug,
                     "in_rests": [in_rest_f, in_rest_c], "clear": clear, "stitch_dropped": self.stitch_dropped,
                     "forceps_pos_mm": [round(float(v) * 1000, 1) for v in (f.pos - np.array([FORCEPS_REST[0], FORCEPS_REST[1], TOP]))],
                     "cutter_pos_mm": [round(float(v) * 1000, 1) for v in (c.pos - np.array([CUTTER_REST[0], CUTTER_REST[1], TOP]))],
                     "tip_from_knot_mm": [round(float(v) * 1000, 1) for v in (tip - knot)], "hook_mm": [round(hx * 1000, 1), round(hy * 1000, 1), round((hz - SKIN) * 1000, 2)],
                     "cutter_low_mm": round(float(low[:, 2].min() - SKIN) * 1000, 2), "notch": {k: list(v) for k, v in self.notch.items()}}
        self.hold = self.hold + 1 if _tier(self.last) == 1.0 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.last["hold"] = self.hold
        if self.steps % 10 == 0:
            print("[su] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


def _tier(r) -> float:
    if not r or r["broke"] or r["dug"]:
        return 0.0
    s = 0.0
    if r["lifted_max_mm"] >= 1.0:
        s = 0.25
    if s and r["cut"] and r["cut_side"] in ("far", "both") and r["cut_h_mm"] is not None and r["cut_h_mm"] <= 1.0:
        s = 0.5
        if r["cut_side"] == "both":
            return s  # both legs cut: the buried part stays in the skin
        if r["drawn"] >= 0.95 and r["gap_max_mm"] < 2.0:
            s = 0.75
            if r["on_gauze"] and all(r["in_rests"]) and r["clear"]:
                s = 1.0
    return s


STITCH = Stitch()
m.on_step(STITCH.step)
m.on_reset(STITCH.reset)


def score(objects) -> float:
    return 1.0 if STITCH.full else min(_tier(STITCH.last), 0.75)


def success(objects) -> bool:
    return STITCH.full


# -- the scripted demo ---------------------------------------------------------------------------------------------------------------------------

FORCEPS_W = 0.0076  # the forceps' arms across, where the hand holds them


def demo_script(ctx):
    """Modes: full, the solution. nolift (failure check): the forceps never take the knot (closing on it lifts it about 1.4 mm), the cutter goes for the
    far leg as in the demo. away (failure check): drawn out away from the wound. above (failure check): the hook brought straight down onto the far
    leg through the lifted loop, then drawn across it. preclosed (failure check): the forceps squeezed shut before they reach the knot, then pressed
    to it and lifted. hover (failure check): the demo, but the instruments are held over their trays at the end, never let go. regrasp (failure check): drawn out, the
    stitch let go over the pad, the forceps closed again at the knot's old place and carried to the gauze."""
    env = ctx.emb._env
    L, R = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "scene":
        yield from L.wait(1.5)
        print("[su] END " + json.dumps({"mode": mode, **STITCH.last}), flush=True)
        return
    g0 = d.down(0.0)
    f0, c0 = m.state(env, "forceps").pos.copy(), m.state(env, "cutter").pos.copy()
    # both instruments out of their rests: the forceps held lightly (under its arms' preload, so it stays open), the cutter pinched by its flat body
    yield from ctx.together(L.move(f0 + np.array([0.0, 0.0, 0.08]), grip=g0, finger=d.FINGER_OPEN, t=2.0, label="over the forceps"),
                            R.move(c0 + np.array([0.0, 0.0, 0.08]), grip=g0, finger=d.FINGER_OPEN, t=2.0, label="over the cutter"))
    yield from ctx.together(L.move(f0, t=1.0, label="fingers either side of the forceps"), R.move(c0, t=1.0, label="fingers either side of the cutter"))
    yield from ctx.together(L.close(FORCEPS_W, force=1.4, label="hold the forceps lightly"), R.close(label="pinch the cutter"))
    yield from ctx.together(L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="forceps out"), R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="cutter out"))
    tip = lambda: forceps_tips(env)[0]  # noqa: E731
    hook = lambda: hook_point(env)[0]  # noqa: E731
    pad = m.state(env, "pad")
    knot = np.array([PAD_C[0], PAD_C[1] + KNOT_Y, SKIN + 0.0012])
    gl = d.tilted(45.0, -90.0)  # the forceps pointing down toward -y, from the knot's side
    # the forceps' tips over the knot, then onto it, squeezed shut, and the knot lifted 2 mm
    yield from L.move(knot + np.array([0.0, 0.0, 0.012]) - TIP_L * L.approach_axis(gl), grip=gl, t=2.0, label="the forceps' tips over the knot")
    if mode == "preclosed":
        yield from L.close(0.003, force=6.0, label="squeeze the forceps shut before they reach the knot")
    yield from L.servo(knot + np.array([0.0, 0.0, 0.010]), tol=0.001, timeout=1.5, gain=0.4, read=tip, label="tips 10 mm over the knot")
    if mode != "nolift":
        yield from L.servo(knot, tol=0.0008, timeout=1.5, gain=0.4, read=tip, label="tips on the knot")
        if mode != "preclosed":
            yield from L.close(0.003, force=6.0, label="squeeze the forceps shut on the knot")
        print("[su] grasped: " + json.dumps(STITCH.last), flush=True)
        yield from L.servo(knot + np.array([0.0, 0.0, 0.0022]), tol=0.0005, timeout=1.5, gain=0.4, read=tip, label="lift the knot 2 mm")
    print("[su] lifted: " + json.dumps(STITCH.last), flush=True)
    # the cutter held upright, its notch opening toward -x: slid in across the far leg at the skin, the leg into the notch, then pressed onto its edge
    far = np.array([PAD_C[0], PAD_C[1] - R_ARC, SKIN + 0.0006])  # the notch's floor 0.6 mm over the skin line (the foot 0.2 mm into it)
    to_hand = np.array([-(NOTCH_X0 + NOTCH_X1) / 2, 0.0, TIP_L - FOOT_H])  # from the notch's floor to where the hand holds the cutter
    if mode == "above":  # the hook straight down through the loop onto the far leg: the strand has no collider, so only the scorer stops this
        yield from R.move(far + np.array([0.0, 0.0, 0.012]) + to_hand, grip=g0, t=2.0, label="the hook over the far leg")
        yield from R.servo(far, tol=0.0005, timeout=3.0, gain=0.06, read=hook, label="the hook down onto the far leg, through the loop")
        yield from R.servo(lambda s: far - np.array([min(s / 1.5, 1.0) * 0.0015, 0.0, 0.0]), tol=0.0005, timeout=2.5, gain=0.06, read=hook,
                           label="press it across the leg")
        print("[su] cut: " + json.dumps(STITCH.last), flush=True)
        yield from R.move(R.at() + np.array([0.012, 0.0, 0.04]), t=1.0, label="the cutter away")
        yield from L.wait(1.0)
        print("[su] END " + json.dumps({"mode": mode, **STITCH.last}), flush=True)
        return
    yield from R.move(far + np.array([0.012, 0.0, 0.012]) + to_hand, grip=g0, t=2.0, label="the hook beside the far leg, 12 mm out")
    # the hook's servos are slow (gain 0.06), so the hand's lag does not swing the hook across the blade's plane, out of the notch's band
    yield from R.servo(far + np.array([0.010, 0.0, 0.003]), tol=0.0006, timeout=2.5, gain=0.08, read=hook, label="the hook 3 mm over the skin, 10 mm out")
    yield from R.servo(far + np.array([0.010, 0.0, 0.0]), tol=0.0003, timeout=2.5, gain=0.06, read=hook, label="the hook down level with the leg's foot")
    # one servo from 10 mm out to 2.5 mm past the leg on a smooth (cosine) profile, so the hand starts and stops gently and the hook stays in the
    # blade's plane
    ease = lambda s: 0.5 - 0.5 * math.cos(math.pi * min(s / 4.0, 1.0))  # noqa: E731
    for attempt in range(3):  # if the leg slipped past the notch (the hook wanders about 1 mm across), back out 10 mm and slide in again
        if attempt:
            print(f"[su] no cut: the hook backed out for try {attempt + 1}", flush=True)
            yield from R.servo(lambda s: far + np.array([-0.0025 + ease(s * 2.0) * 0.0125, 0.0, 0.0]), tol=0.0005, timeout=2.5, gain=0.06, read=hook,
                               label="the hook backed out 10 mm across the leg")
            yield from R.servo(far + np.array([0.010, 0.0, 0.0]), tol=0.0003, timeout=1.5, gain=0.06, read=hook, label="the hook level with the leg's foot again")
        yield from R.servo(lambda s: far + np.array([0.010 - ease(s) * 0.0125, 0.0, 0.0]), tol=0.0005, timeout=5.5, gain=0.06, read=hook,
                           label="slide the hook in across the leg, the leg into the notch, and press it onto the notch's edge: cut")
        if STITCH.cut or mode == "nolift":
            break
    print("[su] cut: " + json.dumps(STITCH.last), flush=True)
    yield from R.move(R.at() + np.array([0.0, 0.0, 0.03]), t=0.8, label="the cutter up off the skin")
    yield from R.move(R.at() + np.array([0.02, -0.02, 0.02]), t=0.8, label="the cutter away")
    # the stitch drawn out by the knot, toward the wound (or away, the cheat check)
    d_out = np.array([0.0, 0.010, 0.008]) if mode == "away" else np.array([0.0, -0.010, 0.008])
    k1 = tip()
    yield from L.servo(lambda s: k1 + min(s / 2.0, 1.0) * d_out, tol=0.001, timeout=3.0, gain=0.4, read=tip, label="draw the stitch out toward the wound")
    print("[su] drawn: " + json.dumps(STITCH.last), flush=True)
    if mode == "regrasp":  # the stitch let go over the pad, the forceps closed again at the knot's old place, then carried to the gauze as in the demo
        yield from L.close(FORCEPS_W, force=1.4, label="let the stitch go over the pad")
        yield from L.servo(knot + np.array([0.0, 0.0, 0.0024]), tol=0.0008, timeout=1.5, gain=0.4, read=tip, label="the tips back at the knot's old place")
        yield from L.close(0.003, force=6.0, label="close them there again")
        print("[su] re-grasped: " + json.dumps(STITCH.last), flush=True)
    # the stitch on the gauze, the instruments back in their rests
    yield from ctx.together(L.move(np.array([GAUZE[0], GAUZE[1], TOP + 0.03]) - TIP_L * L.approach_axis(gl), t=2.0, label="over the gauze"),
                            R.move(c0 + np.array([0.0, 0.0, 0.08]), grip=g0, t=2.0, label="the cutter over its rest"))
    yield from ctx.together(L.close(FORCEPS_W, force=1.4, label="let the stitch go on the gauze"), R.move(c0 + np.array([0.0, 0.0, 0.002]), t=1.0, label="the cutter into its rest"))
    print("[su] on gauze: " + json.dumps(STITCH.last), flush=True)
    if mode != "hover":
        yield from R.open(label="let go of the cutter")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.05]), t=0.8, label="the forceps up off the gauze")
    yield from L.close(0.003, force=6.0, label="hold the forceps firmly, shut, to carry it back")  # held lightly, it can leave the hand on the way back
    yield from ctx.together(L.move(f0 + np.array([0.0, 0.0, 0.08]), grip=g0, t=3.0, label="the forceps over its rest"), R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="right up"))  # slowly, so it stays in the grip
    yield from L.move(f0 + np.array([0.0, 0.0, 0.012]), t=1.0, label="the forceps into its rest")
    floor = np.array([FORCEPS_REST[0], FORCEPS_REST[1], TOP + 0.004 + 0.002])  # the rest's floor, under the slot
    yield from L.servo(floor, tol=0.0015, timeout=1.5, gain=0.4, read=tip, label="the forceps' tips down to the rest's floor (they may have slid in the hand)")
    if mode != "hover":
        yield from L.open(label="let go of the forceps")
    yield from ctx.together(L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="left straight up"), R.move(R.at() + np.array([0.0, 0.0, 0.04]), t=1.0, label="right up"))
    yield from ctx.together(L.move(L.at() + np.array([-0.06, 0.04, 0.0]), t=1.0, label="left clear"), R.move(R.at() + np.array([-0.06, -0.04, 0.0]), t=1.0, label="right clear"))
    yield from L.wait(2.5)
    print("[su] END " + json.dumps({"mode": mode, **STITCH.last}), flush=True)
