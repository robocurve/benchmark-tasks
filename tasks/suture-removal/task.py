"""Take out an interrupted suture: forceps lift the knot, a stitch cutter cuts the far leg flush, then the stitch is drawn out toward the wound.

A suture practice pad (silicone skin with a closed incision) sits on a block; one interrupted 3-0 nylon stitch crosses the incision, its knot on one
side. Dissecting forceps stand tips-down in a foam rest at the left; a single-use stitch cutter (a steel blade with a hook, no handle) stands hook-down
in a foam rest at the right; a gauze square lies on the table.

Sim (stated, all in code): the pad's knot-side half rides on a stiff spring across the incision, so a pull away from the wound opens it. The stitch is a
rigid strand with colliders (the knot, 3 x 2.2 x 2.5 mm, and the loop) that only the forceps' jaws can move, by friction: it rides a lift joint normal to
the skin (sprung) and a circular track (a hinge, radius 26 mm) that carries it up and back toward the wound; the track is locked until the cut, and after
it resists the pull with a set force (0.3 N, assumed). The cutter's hook has colliders, and each leg resists it on a spring along the cutting direction.
A cut is a rule: the cutter's hook slid in under the lifted loop from beside it, the leg entering the notch through its mouth and pressed onto its sharp
inner edge, read in the cutter's frame, with a push on the leg of 1.0 N held 0.2 s (a simulation-only estimate, 0.5 to 2 N; the real trial is judged from video). Drawn out the whole track, the strand gives way to a free
rigid strand in the jaws, which the forceps carry and let go: it lands where it falls. The forceps' arms are a sprung hinge that the gripper's squeeze
closes; a pinch on them resists turning over a 4 mm patch.

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
KZ = 0.0015  # the knot's centre over the skin
RC = 0.026  # the radius of the track the strand is drawn out along: it rises 13 mm and comes 3.5 mm toward the wound over 30 degrees
DRAW_FULL, DRAW_LIM = 30.0, 33.0  # degrees: drawn out, and the track's end
F_CUT = 1.0  # N: the push on the leg that cuts it (ASSUMED: a sharp blade through 3-0 nylon, 0.2 mm; range 0.5 to 2 N, basis in the README)
K_LEG = 667.0  # N/m: the leg's spring on its slide, so F_CUT is 1.5 mm of push
LEG_STOP = 0.006  # m: the leg's slide ends here (4 N)
LEG_Z0, LEG_Z1 = 0.0015, 0.0030  # m over the skin: the leg's column the cutter's notch takes (above the foot; low enough that, where the leaning blade's
# plane crosses it, it is under the notch's roof: higher, it met the blade body's front edge, 1 mm ahead of the mouth, and was pushed without entering)
PRESS_HOLD = 4  # control steps (0.2 s) the push must stay at F_CUT or more
LIGHT_N = 0.5  # N: the light-touch check presses the leg to this (half of F_CUT) and holds it
F_R = 0.3  # N: the pull the strand's track resists once cut (ASSUMED: a 3-0 nylon in a closed pad track; range 0.1 to 1 N)
THREAD_PARK = np.array([0.60, 0.0])  # where the free strand waits, hidden, until it is drawn out
THREAD_LEN = 0.020
SWAP_UP = 0.0035  # the free strand's knot over the drawn knot's place, between the same jaws
SWAP_HOLD = 6  # control steps (0.3 s) that the drawn strand keeps its colliders after the swap
FORCEPS_REST = np.array([0.34, 0.16])
CUTTER_REST = np.array([0.30, -0.15])
GAUZE = np.array([0.24, 0.20])
TIP_L = 0.07  # the instruments' tips beyond where the hand holds them
FOOT_H, NOTCH_H = 0.0008, 0.0035  # the cutter's hook: its foot 0.8 mm thick, the notch above it 3.5 mm high
DIG = 0.002  # the blunt foot pressed this far into the soft silicone goes through it
BAND = 0.0009  # the leg within this of the blade's plane: the strand touching the 1.2 mm steel
NOTCH_X0, NOTCH_X1, SPINE_X1 = -0.002, 0.001, 0.003  # across the cutter (its x): the notch's open mouth, its sharp inner edge, the spine's back
TRAY = (0.16, 0.09)  # the steel tray under each foam rest
JAW_Y, JAW = 0.0054, 0.067  # the forceps' arms 3.4 mm off the middle (the tips 9 mm apart open), and the arm's turn that closes them
FORCEPS_PRE = 0.08  # rad: the forceps' arms spring open against their stop with this much preload

LOOKS = {"skin": m.Look(color=m.rgb(0.86, 0.66, 0.56), roughness=0.8), "block": m.Look(color=m.rgb(0.25, 0.25, 0.27), roughness=0.7),
         "slit": m.Look(color=m.rgb(0.45, 0.20, 0.18), roughness=0.8), "nylon": m.Look(color=m.rgb(0.05, 0.10, 0.45), roughness=0.4),
         "steel": m.Look(color=m.rgb(0.80, 0.81, 0.83), metallic=1.0, roughness=0.25), "foam": m.Look(color=m.rgb(0.20, 0.35, 0.60), roughness=0.95),
         "gauze": m.Look(color=m.rgb(0.97, 0.97, 0.95), roughness=0.95), "tray": m.Look(color=m.rgb(0.75, 0.78, 0.80), metallic=0.6, roughness=0.4)}


def _pad() -> m.Part:
    """The block and the pad's fixed half (-y), its knot-side half (+y) on a stiff slide across the incision, and the stitch on that half: a lift slide (the
    knot under the forceps' pull, sprung) carrying the strand on a circular track (a hinge), locked until the cut. The strand's colliders (knot, loop)
    meet only what is outside the pad: the forceps' jaws."""
    z0 = BLOCK_H + SKIN_T / 2
    frame = [("block", m.box((0.14, 0.10, BLOCK_H), (0.0, 0.0, BLOCK_H / 2)), "block"),
             ("skin_a", m.box((0.12, 0.04, SKIN_T), (0.0, -0.0201, z0)), "skin"),
             ("slit", m.box((0.05, 0.0006, 0.0004), (0.0, 0.0, BLOCK_H + SKIN_T + 0.0001)), "slit")]
    cols = [m.col_box("c_block", (0.14, 0.10, BLOCK_H), (0.0, 0.0, BLOCK_H / 2)), m.col_box("c_skin_a", (0.12, 0.04, SKIN_T), (0.0, -0.0201, z0))]
    half = m.slider("half", "Y", 0.0, 0.004, [("skin_b", m.box((0.12, 0.04, SKIN_T), (0.0, 0.0201, z0)), "skin")],
                    [m.col_box("c_skin_b", (0.12, 0.04, SKIN_T), (0.0, 0.0201, z0))], mass=0.05, spring=3000.0, damping=20.0, friction=0.0, effort=200.0)
    zs = BLOCK_H + SKIN_T
    lift = m.slider("lift", "Z", 0.0, 0.008, [("lift_mark", m.box((0.0005, 0.0005, 0.0005), (0.0, KNOT_Y, zs - 0.002)), "skin")], [], parent="half",
                    mass=0.002, spring=200.0, damping=2.0, friction=0.0, effort=50.0)
    # the stitch: knot, the loop over the incision, the two legs into the skin, and the buried arc (a semicircle under the incision)
    parts = [("knot", m.box((0.0030, 0.0022, 0.0025), (0.0, KNOT_Y, zs + KZ)), "nylon"),
             ("loop", m.box((0.0006, 2 * R_ARC + 0.001, 0.0006), (0.0, 0.0, zs + 0.0008)), "nylon"),
             ("near_leg", m.box((0.0006, 0.0006, 0.0012), (0.0, R_ARC, zs + 0.0002)), "nylon"),
             ("far_leg", m.box((0.0006, 0.0006, 0.0012), (0.0, -R_ARC, zs + 0.0002)), "nylon")]
    for k, a in enumerate(np.linspace(math.radians(195), math.radians(345), 9)):
        parts.append((f"arc_{k}", m.box((0.0006, 0.0018, 0.0006), (0.0, R_ARC * math.cos(a), zs + R_ARC * math.sin(a))), "nylon"))
    scols = [m.col_box("c_knot", (0.0030, 0.0022, 0.0025), (0.0, KNOT_Y, zs + KZ)),
             m.col_box("c_loop", (0.0008, 2 * R_ARC - 0.0016, 0.0008), (0.0, 0.0008, zs + 0.0008))]  # begins 1.5 mm past the far leg: the cutter's blade stands beside the leg, not on the loop
    draw = m.hinge("draw", "X", (0.0, KNOT_Y - RC, zs + KZ), 0.0, DRAW_LIM, parts, scols, parent="lift", mass=0.002, spring=0.0, damping=0.05, friction=0.0, effort=20.0)
    # each leg's cutting resistance: a collider on the leg, on a spring slide along x (the cutter's cutting direction is -x), so the cutter's edge must push it
    # F = K_LEG x; the cut counts only after that push reaches F_CUT and is held
    legs = [m.slider(f"cut_{nm}", "X", -LEG_STOP, 0.0, [(f"cut_{nm}_vis", m.box((0.0006, 0.0006, LEG_Z1 - LEG_Z0), (0.0, y, zs + (LEG_Z0 + LEG_Z1) / 2)), "nylon")],
                     [m.col_box(f"c_cut_{nm}", (0.0006, 0.0006, LEG_Z1 - LEG_Z0), (0.0, y, zs + (LEG_Z0 + LEG_Z1) / 2))], parent="half", mass=0.0005,
                     spring=K_LEG, damping=1.0, friction=0.0, effort=50.0) for nm, y in (("far", -R_ARC), ("near", R_ARC))]
    return m.mechanism("pad", (PAD_C[0], PAD_C[1], TOP), [half, lift, draw] + legs, LOOKS, frame_parts=frame, frame_colliders=cols, friction=(1.0, 0.9))


def _thread() -> m.Part:
    """The strand once drawn out: a free rigid body, a knot and 20 mm of strand along -y from it (as the loop runs from the knot), waiting hidden at the side
    until it comes out of the pad."""
    parts = [("t_knot", m.box((0.0030, 0.0022, 0.0025)), "nylon"), ("t_strand", m.box((0.0007, THREAD_LEN, 0.0007), (0.0, -(THREAD_LEN / 2 + 0.0011), 0.0)), "nylon")]
    cols = [m.col_box("c_t_knot", (0.0030, 0.0022, 0.0025)), m.col_box("c_t_strand", (0.0007, THREAD_LEN, 0.0007), (0.0, -(THREAD_LEN / 2 + 0.0011), 0.0))]
    return m.free("thread", (THREAD_PARK[0], THREAD_PARK[1], TOP + 0.002), parts, cols, LOOKS, mass=0.001, friction=(0.9, 0.8), torsional_patch=None, solver_iters=64)


def _forceps() -> m.Part:
    """Dissecting forceps, 12 cm, standing tips-down in its rest, the arms side by side along x; origin where the hand holds them, 7 cm above the tips.
    Arm A is the frame; arm B turns about the joined back end and springs open (tips 9 mm apart) against a stop; a squeeze closes it. A pinch on the
    arms resists turning about the pinch axis over a 4 mm patch (the finger pad on the 6 mm arm)."""
    back = 0.05
    frame = [("arm_a", m.box((0.006, 0.0018, back + TIP_L), (0.0, JAW_Y, (back - TIP_L) / 2)), "steel"),
             ("butt", m.box((0.006, 0.008, 0.008), (0.0, 0.0, back - 0.004)), "steel")]
    cols = [m.col_box("c_arm_a", (0.006, 0.0018, back + TIP_L), (0.0, JAW_Y, (back - TIP_L) / 2))]
    arm_b = m.hinge("arm_b", "X", (0.0, 0.0, back), 0.0, math.degrees(JAW), [("arm_b", m.box((0.006, 0.0018, back + TIP_L), (0.0, -JAW_Y, (back - TIP_L) / 2)), "steel")],
                    [m.col_box("c_arm_b", (0.006, 0.0018, back + TIP_L), (0.0, -JAW_Y, (back - TIP_L) / 2))], mass=0.008, spring=1.0, damping=0.02,
                    friction=0.0, effort=10.0)
    z = TOP + 0.004 + TIP_L + 0.0005  # tips on the rest's floor
    return m.mechanism("forceps", (FORCEPS_REST[0], FORCEPS_REST[1], z), [arm_b], LOOKS, frame_parts=frame, frame_colliders=cols, free_mass=0.012,
                       friction=(0.9, 0.8), frame_inertia=(1.5e-5, 1.5e-5, 1e-6), yaw_deg=90.0, torsional_patch=0.004)


def _cutter() -> m.Part:
    """A single-use stitch cutter: a flat steel blade standing hook-down, its origin where the hand pinches its flat body (faces along y). At its end, in
    the blade's own plane, a hooked notch opens toward -x: a foot 0.8 mm thick runs under it, and its inner edge (at x 1 mm) is the sharp one."""
    zb = -TIP_L + FOOT_H + NOTCH_H  # the notch's roof: the body's lower end
    parts = [("blade", m.box((0.006, 0.0012, 0.02 - zb), (0.0, 0.0, (0.02 + zb) / 2)), "steel"),
             ("spine", m.box((SPINE_X1 - NOTCH_X1, 0.0012, NOTCH_H + FOOT_H), ((SPINE_X1 + NOTCH_X1) / 2, 0.0, -TIP_L + (NOTCH_H + FOOT_H) / 2)), "steel"),
             ("foot", m.box((SPINE_X1 - NOTCH_X0, 0.0012, FOOT_H), ((SPINE_X1 + NOTCH_X0) / 2, 0.0, -TIP_L + FOOT_H / 2)), "steel")]
    cols = [m.col_box("c_blade", (0.006, 0.0012, 0.02 - zb), (0.0, 0.0, (0.02 + zb) / 2)),  # the blade body down to the notch's roof
            m.col_box("c_spine", (SPINE_X1 - NOTCH_X1, 0.0012, NOTCH_H + FOOT_H), ((SPINE_X1 + NOTCH_X1) / 2, 0.0, -TIP_L + (NOTCH_H + FOOT_H) / 2)),  # the hook's back, with the inner edge
            m.col_box("c_foot", (SPINE_X1 - NOTCH_X0, 0.0012, FOOT_H), ((SPINE_X1 + NOTCH_X0) / 2, 0.0, -TIP_L + FOOT_H / 2))]  # the foot under the notch: the real tip
    return m.free("cutter", (CUTTER_REST[0], CUTTER_REST[1], TOP + 0.004 + TIP_L + 0.0005), parts, cols, LOOKS, mass=0.006)


def _rests() -> m.Part:
    shapes = []
    for nm, (x, y), half_w in (("rest_f", FORCEPS_REST, 0.0095), ("rest_c", CUTTER_REST, 0.0055)):
        for k, dy in enumerate((-half_w, half_w)):  # two foam cheeks either side of the slot the instrument stands in (12 mm for the forceps, 4 mm for the cutter)
            shapes.append((f"{nm}_{k}", (0.04, 0.007, 0.03), (x, y + dy, TOP + 0.015), "foam"))
        shapes.append((f"{nm}_floor", (0.04, 0.02, 0.004), (x, y, TOP + 0.002), "foam"))
    for nm, (x, y) in (("tray_f", FORCEPS_REST), ("tray_c", CUTTER_REST)):
        shapes.append((nm, (TRAY[0], TRAY[1], 0.002), (x, y, TOP + 0.001), "tray"))
        for k, (dx, dy, sx, sy) in enumerate(((TRAY[0] / 2, 0, 0.002, TRAY[1]), (-TRAY[0] / 2, 0, 0.002, TRAY[1]), (0, TRAY[1] / 2, TRAY[0], 0.002), (0, -TRAY[1] / 2, TRAY[0], 0.002))):
            shapes.append((f"{nm}_rim{k}", (sx, sy, 0.012), (x + dx, y + dy, TOP + 0.006), "tray"))
    shapes.append(("gauze", (0.075, 0.075, 0.002), (GAUZE[0], GAUZE[1], TOP + 0.001), "gauze"))
    parts = [(nm, m.box(s, c), look) for nm, s, c, look in shapes]
    cols = [m.col_box(f"c_{nm}", s, c) for nm, s, c, look in shapes]
    return m.fixed("rests", (0.0, 0.0, 0.0), parts, cols, LOOKS, friction=(1.0, 0.9))


OBJECTS = [_pad(), _forceps(), _cutter(), _thread(), _rests()]


# -- the gates and the scorer ---------------------------------------------------------------------------------------------------------------------

def forceps_tips(env):
    """The point midway between the jaws' inner faces at the tips (arm B's tip has moved 12 cm x its turn toward arm A), the forceps' state and arm B's turn."""
    f = m.state(env, "forceps")
    fb = float(f.joint("arm_b"))
    return f.to_world(np.array([[0.0, 0.06 * math.sin(fb), -TIP_L]]))[0], f, fb


def hook_point(env):
    """The middle of the notch's floor, in the station frame, and the cutter's state."""
    c = m.state(env, "cutter")
    return c.to_world(np.array([[(NOTCH_X0 + NOTCH_X1) / 2, 0.0, -TIP_L + FOOT_H]]))[0], c


def cutter_low_corners(c):
    """The cutter's lowest corners: the foot's bottom and the blade body's lower end."""
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
    edge = band & (p[:, 0] >= NOTCH_X1 - 0.0009) & (p[:, 0] <= SPINE_X1)
    if not entered and inside.any() and prev_open and ready:
        entered = True
    if not (band & (p[:, 0] > NOTCH_X0 - 0.004) & (p[:, 0] <= SPINE_X1)).any():
        entered = False
    cut = int(np.flatnonzero(edge)[0]) if entered and edge.any() else None
    return (bool(open_side.any()), entered), cut


def press_step(count, at_edge, force):
    """The cut's force gate: the control steps in a row the leg has sat on the notch's sharp edge with the push on it at F_CUT or more. Returns the new
    count and whether the leg is cut (PRESS_HOLD steps, 0.2 s). A lighter push, or a push anywhere but on the edge, does not cut and restarts the count."""
    count = count + 1 if (at_edge and force >= F_CUT) else 0
    return count, count >= PRESS_HOLD


def thread_on_gauze(knot, tail, speed):
    """The free strand lying on the gauze: its knot end and its far end both inside the gauze's outline (3.5 cm half side), within 6 mm of the table, at rest."""
    pts = np.array([knot, tail])
    inside = bool(np.all(np.abs(pts[:, 0] - GAUZE[0]) < 0.035) and np.all(np.abs(pts[:, 1] - GAUZE[1]) < 0.035) and np.all(pts[:, 2] < TOP + 0.006))
    return bool(inside and speed < 0.02)


def at_rest(points, tips, floor_z, footprint, gauze):
    """An instrument back in its tray: let go (every fingertip of both hands 3 cm or more from points along it), lying or standing on the tray (its
    lowest point within 1.5 cm of the tray's floor), its middle inside the tray's outline and not on the gauze. points: (n, 3) along the instrument;
    footprint and gauze: (x, y, half x, half y)."""
    points, tips = np.asarray(points), np.asarray(tips)
    let_go = float(np.min(np.linalg.norm(tips[:, None, :] - points[None, :, :], axis=2))) >= 0.03
    mid = points.mean(axis=0)
    inside = lambda r: abs(mid[0] - r[0]) < r[2] and abs(mid[1] - r[1]) < r[3]  # noqa: E731
    return bool(let_go and float(points[:, 2].min()) < floor_z + 0.015 and inside(footprint) and not inside(gauze))


def _drive(env, link, stiff, effort, target=None):
    """A joint of the pad's drive gains and target, as `mechkit.lock` writes them (reset gives them back)."""
    asset, j = m._joint(env, "pad", link)
    m._TOUCHED.add(("pad", link))
    asset.write_joint_effort_limit_to_sim(m._tensor(asset, effort), joint_ids=[j])
    asset.write_joint_stiffness_to_sim(m._tensor(asset, stiff), joint_ids=[j])
    if target is not None:
        asset.set_joint_position_target(m._tensor(asset, target), joint_ids=[j])


def _collide(path, on):
    import isaaclab.sim as sim_utils
    from pxr import UsdPhysics

    prim = sim_utils.get_current_stage().GetPrimAtPath(path)
    UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(bool(on))


def draw_pose(env):
    """The strand's body (the one on the track): its origin and rotation, in the station frame."""
    base = env.unwrapped
    a = base.scene["pad"]
    i = a.find_bodies("draw")[0][0]
    pose = m._np(a.data.body_link_pose_w)[0, i]
    return pose[:3] - m._np(base.scene.env_origins[0]), m.quat_matrix(pose[3:7])


KNOT_LOCAL = np.array([0.0, KNOT_Y, BLOCK_H + SKIN_T + KZ])  # in the pad's frame, the pad's origin on the table


class Stitch:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.grasped, self.lifted_max, self.broke, self.cut, self.cut_h, self.cut_side = 0, False, 0.0, False, False, None, None
        self.notch, self.gap_max, self.drawn, self.on_gauze, self.dug, self.hold, self.full, self.last = None, 0.0, 0.0, False, False, 0, False, {}
        self.hook_st, self.swapped, self.armed = (False, False, None, False), False, False
        self.notch = {"far": (False, False), "near": (False, False)}
        self.floor_min = {"far": 1.0, "near": 1.0}
        self.force_now = {"far": 0.0, "near": 0.0}
        self.force_max, self.press, self.cut_force = {"far": 0.0, "near": 0.0}, {"far": 0, "near": 0}, {}
        self.pull_max, self.knot0, self.swap_step, self.drawn_off = 0.0, None, 0, False
        if env is not None:  # at import the simulator is not up: touching the stage would load its modules too early
            for nm in ("t_knot", "t_strand"):
                m.set_visible(f"{m.ENV_PATH}/thread/visual/{nm}", False)

    def _swap(self, env):
        """The strand is out of the pad: its body on the track gives way to the free strand, put where it is, with the jaws still on the knot."""
        base = env.unwrapped
        t, R = draw_pose(env)
        knot = t + R @ KNOT_LOCAL
        a = base.scene["thread"]
        pose = a.data.root_link_pose_w
        pose = getattr(pose, "torch", pose).clone()
        # the free strand's knot goes 3.5 mm above the drawn knot, still between the jaws' faces (they stand 12 cm tall): put in the drawn knot's own place,
        # the two knots overlap for a step before the drawn one's collider goes, and the light strand is thrown out of the jaws
        pose[0, :3] = pose.new_tensor((knot + np.array([0.0, 0.0, SWAP_UP]) + m._np(base.scene.env_origins[0])).tolist())
        pad = base.scene["pad"]
        q = pad.data.body_link_pose_w[0, pad.find_bodies("draw")[0][0], 3:7]
        pose[0, 3:7] = getattr(q, "torch", q).clone()  # turned as the strand on the track is: the strand runs from the knot along the loop's line
        a.write_root_pose_to_sim(pose)
        import torch
        a.write_root_velocity_to_sim(torch.zeros((1, 6), device=pose.device))
        for nm in ("knot", "loop", "near_leg", "far_leg") + tuple(f"arc_{k}" for k in range(9)):
            try:
                m.set_visible(f"{m.ENV_PATH}/pad/draw/visual/{nm}", False)
            except Exception:
                pass
        for nm in ("t_knot", "t_strand"):
            m.set_visible(f"{m.ENV_PATH}/thread/visual/{nm}", True)
        self.swapped, self.swap_step = True, self.steps

    def _drop_drawn(self):
        """A few steps after the swap, the drawn strand's colliders go: the jaws, held open by it, then close the last fraction of a millimetre onto the
        free strand's knot. Taken away at the swap itself, the jaws slam shut through the light knot."""
        for nm in ("c_knot", "c_loop"):
            _collide(f"{m.ENV_PATH}/pad/draw/{nm}", False)
        self.drawn_off = True

    def step(self, env):
        self.steps += 1
        m.set_target(env, "forceps", "arm_b", -FORCEPS_PRE)  # the arms' spring: rest beyond the open stop, a preload
        pad = m.state(env, "pad")
        lift, draw, gap = float(pad.joint("lift")), float(pad.joint("draw")), float(pad.joint("half"))
        if not self.armed:
            m.lock(env, "pad", "draw")  # the strand cannot be drawn through the skin until its leg is cut
            self.armed = True
        if self.swapped and not self.drawn_off and self.steps - self.swap_step >= SWAP_HOLD:
            self._drop_drawn()
        t, R = draw_pose(env)
        knot = t + R @ KNOT_LOCAL
        if self.swapped:
            knot = m.state(env, "thread").pos
        if self.knot0 is None:
            self.knot0 = knot.copy()
        tip, f, fb = forceps_tips(env)
        closed = fb > 0.8 * JAW
        near_knot = bool(np.linalg.norm(tip - knot) < 0.003)
        # caught: the jaws shut on something the width of the knot (they stop short of fully closed), tips at the knot
        if near_knot and 0.02 < fb < 0.054 and not self.swapped:
            self.grasped = True
        if not self.cut:
            if lift > 0.005:
                self.broke = True  # pulled up past what the stitch takes before it is cut: torn through the skin
            self.lifted_max = max(self.lifted_max, lift)
        self.gap_max = max(self.gap_max, gap)
        # after the cut the track gives, against a brake (the pull resistance F_R); the lift joint goes slack
        if self.cut and not self.swapped:
            _drive(env, "draw", 100.0, F_R * RC, target=draw)
            _drive(env, "lift", 0.0, 50.0)
            self.drawn = float(np.clip(math.degrees(draw) / DRAW_FULL, 0.0, 1.0))
            self.pull_max = max(self.pull_max, 0.0)
            if self.drawn >= 0.97:
                self._swap(env)
        # the cut: the hook under the stitch, the leg into its notch through the mouth, pressed onto its inner edge (notch_step)
        hook, c = hook_point(env)
        hx, hy, hz = hook[0] - PAD_C[0], hook[1] - PAD_C[1], hook[2]
        low = cutter_low_corners(c)
        on_skin = (np.abs(low[:, 0] - PAD_C[0]) < 0.06) & (np.abs(low[:, 1] - PAD_C[1]) < 0.04)
        if (on_skin & (low[:, 2] < SKIN - DIG)).any():
            self.dug = True  # any of the cutter's lowest corners more than 2 mm into the silicone, anywhere on the skin: pressed through it
        self.hook_st = hook_beneath(self.hook_st, (hx, hy, hz), lift, gap)
        if self.cut_side != "both" and not self.swapped:
            ready = bool(self.hook_st[0] and lift >= 0.001)
            for leg, ly in (("far", PAD_C[1] - R_ARC), ("near", PAD_C[1] + R_ARC + gap)):
                if leg == self.cut_side:
                    continue
                push = -float(pad.joint(f"cut_{leg}"))  # how far the cutter has pushed the leg along -x
                force = K_LEG * push
                self.force_max[leg] = max(self.force_max[leg], force)
                self.force_now[leg] = force
                pw = np.array([[PAD_C[0] - push, ly, z] for z in np.linspace(SKIN + LEG_Z0, SKIN + LEG_Z1, 24)])
                self.notch[leg], k = notch_step(self.notch[leg], c.to_local(pw), ready)
                self.floor_min[leg] = min(self.floor_min[leg], hz) if self.notch[leg][1] else 1.0  # the notch's lowest while the leg is in it
                # the cut needs the leg in the notch against the edge and the push at F_CUT or more, held PRESS_HOLD steps: a lighter press does not cut
                self.press[leg], through = press_step(self.press[leg], k is not None, force)
                if not through:
                    continue
                self.cut_force[leg] = round(force, 2)
                _drive(env, f"cut_{leg}", 0.0, 50.0)
                _collide(f"{m.ENV_PATH}/pad/cut_{leg}/c_cut_{leg}", False)  # cut through: the leg's two ends are free
                if self.cut:
                    self.cut_side = "both"  # the other leg cut too: the buried part is left in the skin
                else:
                    self.cut, self.cut_h, self.cut_side = True, max(float(self.floor_min[leg] - SKIN), 0.0), leg  # cut just over the foot, flush at best
                    _drive(env, "draw", 100.0, F_R * RC, target=draw)
                try:
                    m.set_visible(f"{m.ENV_PATH}/pad/cut_{leg}/visual/cut_{leg}_vis", False)
                    m.set_visible(f"{m.ENV_PATH}/pad/draw/visual/{leg}_leg", False)
                except Exception:
                    pass
        th = m.state(env, "thread")
        tail = th.to_world(np.array([[0.0, -(THREAD_LEN + 0.0011), 0.0]]))[0]
        self.on_gauze = bool(self.swapped and thread_on_gauze(th.pos, tail, float(np.linalg.norm(th.lin_vel))))
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
        self.last = {"t": round(self.steps * float(env.unwrapped.step_dt), 2), "grasped": self.grasped, "closed": bool(closed), "forceps_mm": round(fb * 120, 2),
                     "lift_mm": round(lift * 1000, 2), "lifted_max_mm": round(self.lifted_max * 1000, 2), "gap_mm": round(gap * 1000, 2), "gap_max_mm": round(self.gap_max * 1000, 2),
                     "push_max_N": {k: round(v, 2) for k, v in self.force_max.items()}, "push_N": {k: round(v, 2) for k, v in self.force_now.items()}, "press": dict(self.press), "cut_force_N": self.cut_force, "cut": self.cut, "cut_h_mm": None if self.cut_h is None else round(self.cut_h * 1000, 2), "cut_side": self.cut_side,
                     "drawn": round(self.drawn, 3), "draw_deg": round(math.degrees(draw), 1), "swapped": self.swapped, "on_gauze": self.on_gauze, "broke": self.broke, "dug": self.dug,
                     "in_rests": [in_rest_f, in_rest_c], "clear": clear,
                     "thread_pos_mm": [round(float(v) * 1000, 1) for v in (th.pos - np.array([GAUZE[0], GAUZE[1], TOP]))],
                     "forceps_pos_mm": [round(float(v) * 1000, 1) for v in (f.pos - np.array([FORCEPS_REST[0], FORCEPS_REST[1], TOP]))],
                     "cutter_pos_mm": [round(float(v) * 1000, 1) for v in (c.pos - np.array([CUTTER_REST[0], CUTTER_REST[1], TOP]))],
                     "tip_from_knot_mm": [round(float(v) * 1000, 1) for v in (tip - knot)], "hook_mm": [round(hx * 1000, 1), round(hy * 1000, 1), round((hz - SKIN) * 1000, 2)],
                     "cutter_low_mm": round(float(low[:, 2].min() - SKIN) * 1000, 2), "cutter_tilt": round(float(c.tilt_deg()), 1), "notch": {k: list(v) for k, v in self.notch.items()}}
        self.hold = self.hold + 1 if _tier(self.last) == 1.0 else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        self.last["hold"] = self.hold
        if self.steps % 10 == 0:
            print("[su] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


def _tier(r) -> float:
    if not r or r["broke"] or r["dug"]:
        return 0.0
    s = 0.0
    if r["lifted_max_mm"] >= 2.0 and r.get("grasped", True):
        s = 0.25
    if s and r["cut"] and r["cut_side"] in ("far", "both") and r["cut_h_mm"] is not None and r["cut_h_mm"] <= 2.0:
        s = 0.5
        if r["cut_side"] == "both":
            return s  # both legs cut: the buried part stays in the skin
        if r["drawn"] >= 0.95 and r["gap_max_mm"] < 3.0:
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

FORCEPS_W = 0.0126  # the forceps' arms across, where the hand holds them
CUT_TILT = 50.0  # degrees: the cutter leans back toward the right arm's side (-y), its blade's plane still across the strand's line
# the right hand's fingers close along x, across the cutter's 6 mm face (its two long edges), so the push on the hook (along x) turns it about y, which
# the pads' length resists; pinched across the 1.2 mm thickness, the push turned it about the pinch axis (README 24)
CUT_YAW = 90.0
AIM_DY = 0.0012  # m: the hook's notch aimed this far toward the knot from the far leg
PRESS_GAIN = 0.10  # the slide-and-press servo's gain; it runs up to 10 s, as the push on the leg builds slowly through the arm
HOOK_Z = 0.0011  # m: the notch's floor over the skin as the hook slides in (the foot touches the skin at about 1.0)


def demo_script(ctx):
    """Modes: full, the solution. light (failure check): the demo, but the press on the leg stops once it reads 0.5 N (half the cutting force) and is held there 3 s. nolift (failure check): the forceps never take the knot, the cutter goes for the far leg as in the demo. away
    (failure check): drawn out away from the wound. above (failure check): the hook brought straight down onto the far leg through the lifted loop,
    then drawn across it. preclosed (failure check): the forceps squeezed shut before they reach the knot, then pressed to it and lifted. hover
    (failure check): the demo, but the instruments are held over their trays at the end, never let go. drop (failure check): the stitch drawn out,
    then let go over the pad: it falls on the pad and the hands go on to the gauze with nothing in the jaws.
    The forceps stay upright in the left hand, the jaws closing along x; the cutter leans 50 degrees toward -y in the right hand, so the two hands
    work the one stitch from either side of it."""
    env = ctx.emb._env
    L, R = ctx.left, ctx.right
    mode = ctx.mode
    if mode == "scene":
        yield from L.wait(1.5)
        print("[su] END " + json.dumps({"mode": mode, **STITCH.last}), flush=True)
        return
    # left fingers along x; right fingers along x too, on the cutter's two long edges (across its 6 mm face), then the hand leaning back toward -y
    t = math.radians(CUT_TILT)
    gF, gC0 = d.down(-90.0), d.down(CUT_YAW)
    gC = d._as_grip(np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]]) @ gC0.rot)
    f0, c0 = m.state(env, "forceps").pos.copy(), m.state(env, "cutter").pos.copy()
    # both instruments out of their rests: the forceps held lightly (under its arms' preload, so it stays open), the cutter pinched by its flat blade
    yield from ctx.together(L.move(f0 + np.array([0.0, 0.0, 0.08]), grip=gF, finger=d.FINGER_OPEN, t=2.0, label="over the forceps"),
                            R.move(c0 + np.array([0.0, 0.0, 0.08]), grip=gC0, finger=d.FINGER_OPEN, t=2.0, label="over the cutter"))
    yield from ctx.together(L.move(f0, t=1.0, label="fingers either side of the forceps"), R.move(c0, t=1.0, label="fingers either side of the cutter"))
    yield from ctx.together(L.close(FORCEPS_W, force=0.8, label="hold the forceps lightly"), R.close(label="pinch the cutter"))
    yield from ctx.together(L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="forceps out"), R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="cutter out"))
    tip = lambda: forceps_tips(env)[0]  # noqa: E731
    hook = lambda: hook_point(env)[0]  # noqa: E731
    knot = np.array([PAD_C[0], PAD_C[1] + KNOT_Y, SKIN + KZ])
    # open jaws aimed so that arm A's inner face stands 0.5 mm off the knot's side (arm B then closes the 9 mm opening onto the other side)
    aim = lambda dz: knot + np.array([0.0, 0.0, dz]) + m.state(env, "forceps").rotation() @ np.array([0.0, -0.0025, 0.0])  # noqa: E731
    yield from L.move(aim(0.012) + np.array([0.0, 0.0, TIP_L]), grip=gF, t=2.0, label="the forceps' tips over the knot")
    if mode == "preclosed":
        yield from L.close(0.003, force=6.0, label="squeeze the forceps shut before they reach the knot")
    yield from L.servo(lambda s: aim(0.010), tol=0.0004, timeout=3.0, gain=0.4, read=tip, label="tips 10 mm over the knot")
    yield from L.servo(lambda s: aim(0.004), tol=0.0003, timeout=2.0, gain=0.3, read=tip, label="tips 4 mm over the knot, either side of it")
    if mode != "nolift":
        yield from L.servo(lambda s: aim(-0.0003), tol=0.0003, timeout=2.5, gain=0.3, read=tip, label="tips down either side of the knot")
        if mode != "preclosed":
            yield from L.close(0.003, force=6.0, label="squeeze the forceps shut on the knot")
        print("[su] grasped: " + json.dumps(STITCH.last), flush=True)
        yield from L.servo(knot + np.array([0.0, 0.0, 0.0030]), tol=0.0004, timeout=2.0, gain=0.3, read=tip, label="lift the knot 3 mm")
    print("[su] lifted: " + json.dumps(STITCH.last), flush=True)
    # the cutter leaning back toward -y, its notch opening toward -x: slid in across the far leg at the skin, the leg into the notch, then pressed onto
    # its edge; the hook steered by its own observed place every step
    far = np.array([PAD_C[0], PAD_C[1] - R_ARC + AIM_DY, SKIN + HOOK_Z])  # the notch's floor HOOK_Z over the skin line, the leaning foot's lowest corner
    # just clear of it (aimed lower, the servo pressed the foot into the skin harder each step, and its friction stalled the slide), and AIM_DY toward the
    # knot from the far leg, so the leaning blade's plane crosses the leg about 2 mm up, in the notch's middle
    to_hand = (gC.rot @ gC0.rot.T) @ np.array([-(NOTCH_X0 + NOTCH_X1) / 2, 0.0, TIP_L - FOOT_H])  # from the notch's floor to where the hand holds the cutter
    if mode == "above":  # the hook straight down through the loop onto the far leg: only the scorer stops this
        yield from R.move(far + np.array([0.0, 0.0, 0.012]) + to_hand, grip=gC, t=2.0, label="the hook over the far leg")
        yield from R.servo(far, tol=0.0005, timeout=3.0, gain=0.06, read=hook, label="the hook down onto the far leg, through the loop")
        yield from R.servo(lambda s: far - np.array([min(s / 1.5, 1.0) * 0.0015, 0.0, 0.0]), tol=0.0005, timeout=2.5, gain=0.06, read=hook,
                           label="press it across the leg")
        print("[su] cut: " + json.dumps(STITCH.last), flush=True)
        yield from R.move(R.at() + np.array([0.0, -0.02, 0.04]), t=1.0, label="the cutter away")
        yield from L.wait(1.0)
        print("[su] END " + json.dumps({"mode": mode, **STITCH.last}), flush=True)
        return
    knot_now = lambda: (lambda tr: tr[0] + tr[1] @ KNOT_LOCAL)(draw_pose(env))  # noqa: E731  the knot's own observed place

    def hold_knot():  # the left keeps the knot 3 mm up, steered by the knot's observed height, while the right works under it
        while True:
            yield from L.servo(knot + np.array([0.0, 0.0, 0.0030]), tol=0.0003, timeout=1.0, gain=0.2, read=knot_now, label="hold the knot 3 mm up")

    def cut():
        yield from R.move(far + np.array([0.012, 0.0, 0.012]) + to_hand, grip=gC, t=2.0, label="the hook beside the far leg, 12 mm out")
        # the hook's servos are slow (gain 0.06), so the hand's lag does not swing the hook across the blade's plane, out of the notch's band
        yield from R.servo(far + np.array([0.010, 0.0, 0.003]), tol=0.0006, timeout=2.5, gain=0.08, read=hook, label="the hook 3 mm over the skin, 10 mm out")
        yield from R.servo(far + np.array([0.010, 0.0, 0.0]), tol=0.0003, timeout=2.5, gain=0.06, read=hook, label="the hook down level with the leg's foot")
        ease = lambda s: 0.5 - 0.5 * math.cos(math.pi * min(s / 4.0, 1.0))  # noqa: E731
        for attempt in range(3):  # if the leg slipped past the notch, back out 10 mm and slide in again
            if attempt:
                print(f"[su] no cut: the hook backed out for try {attempt + 1}", flush=True)
                yield from R.servo(lambda s: far + np.array([-0.0025 + ease(s * 2.0) * 0.0125, 0.0, 0.0]), tol=0.0005, timeout=2.5, gain=0.06, read=hook,
                                   label="the hook backed out 10 mm across the leg")
                yield from R.servo(far + np.array([0.010, 0.0, 0.0]), tol=0.0003, timeout=1.5, gain=0.06, read=hook, label="the hook level with the leg's foot again")
            press = R.servo(lambda s: far + np.array([0.010 - ease(s) * 0.0140, 0.0, 0.0]), tol=0.0005, timeout=10.0, gain=PRESS_GAIN, read=hook,
                            label="slide the hook in across the leg, the leg into the notch, and press it onto the notch's edge: cut")
            for _ in press:
                if mode == "light" and STITCH.force_now["far"] >= LIGHT_N:  # the light touch: the press stops at half the cutting force and stays there
                    print(f"[su] light: pressed to {STITCH.force_now['far']:.2f} N, held 3 s", flush=True)
                    yield from R.wait(3.0)
                    break
                yield
            if STITCH.cut or mode in ("nolift", "light"):
                break
        print("[su] cut: " + json.dumps(STITCH.last), flush=True)
        # back out along the notch's opening first, steered by the hook: the press left the hand's command ahead of the hook, and a move from it lurched
        # the hook 2 mm on into the leg
        yield from R.servo(far + np.array([0.004, 0.0, 0.0]), tol=0.0008, timeout=2.0, gain=0.1, read=hook, label="the hook back out through the notch's mouth")
        yield from R.move(R.at() + np.array([0.0, -0.01, 0.03]), t=0.8, label="the cutter up off the skin")
        yield from R.move(R.at() + np.array([0.0, -0.04, 0.03]), t=0.8, label="the cutter away")

    done, hold, run = object(), hold_knot(), cut()
    while next(run, done) is not done:
        if mode != "nolift":
            next(hold)
        yield
    # the stitch drawn out by the knot along its track: up and back toward the wound (or away from it, the cheat check)
    k1 = tip()
    if mode == "away":
        d_out = np.array([0.0, 0.010, 0.008])
        yield from L.servo(lambda s: k1 + min(s / 2.0, 1.0) * d_out, tol=0.001, timeout=3.0, gain=0.4, read=tip, label="draw the stitch out away from the wound")
    else:
        ctr = np.array([0.0, -RC, 0.0])
        circ = lambda th: np.array([0.0, RC * math.cos(th), RC * math.sin(th)]) + ctr  # noqa: E731
        yield from L.servo(lambda s: k1 + circ(math.radians(DRAW_FULL + 2.0) * min(s / 5.0, 1.0)) - circ(0.0), tol=0.0008, timeout=7.0, gain=0.4, read=tip,
                           label="draw the stitch out along its track, up and back toward the wound")
    print("[su] drawn: " + json.dumps(STITCH.last), flush=True)
    if mode == "drop":
        yield from L.close(FORCEPS_W, force=0.8, label="let the stitch go over the pad")
        yield from L.wait(1.0)
    # the stitch carried in the jaws to the gauze and let go a little over it; the cutter laid in its tray beside the foam rest, still leaning as it cut,
    # its hook steered by its own observed place to 4 mm over the tray's floor (held by its edges, it leans freely about the pinch, so lowering it upright
    # into the rest's 4 mm slot struck the foam and threw it)
    over_gauze = np.array([GAUZE[0], GAUZE[1] + 0.010, TOP + 0.025])
    lay = np.array([CUTTER_REST[0] + 0.045, CUTTER_REST[1] + 0.035, TOP + 0.006])
    yield from ctx.together(L.move(over_gauze + np.array([0.0, 0.0, 0.02 + TIP_L]), grip=gF, t=2.5, label="over the gauze"),
                            R.move(lay + np.array([0.0, 0.0, 0.04]) + to_hand, grip=gC, t=2.5, label="the cutter over its tray, beside the rest"))
    yield from ctx.together(L.servo(over_gauze, tol=0.002, timeout=1.5, gain=0.4, read=tip, label="the jaws low over the gauze"),
                            R.servo(lay, tol=0.0015, timeout=2.5, gain=0.3, read=hook, label="the cutter's hook down to 4 mm over the tray's floor"))
    yield from L.close(FORCEPS_W, force=0.8, label="let the stitch go on the gauze")
    yield from L.wait(0.5)
    print("[su] on gauze: " + json.dumps(STITCH.last), flush=True)
    if mode != "hover":
        yield from R.open(label="let go of the cutter")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.05]), t=0.8, label="the forceps up off the gauze")
    yield from L.close(0.003, force=6.0, label="hold the forceps firmly, shut, to carry it back")
    yield from ctx.together(L.move(f0 + np.array([0.0, 0.0, 0.08]), grip=gF, t=3.0, label="the forceps over its rest"), R.move(R.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="right up"))
    # the tips steered by their own observed place: over the slot, above the foam, then straight down into it to the rest's floor
    yield from L.servo(np.array([FORCEPS_REST[0], FORCEPS_REST[1], TOP + 0.045]), tol=0.001, timeout=2.0, gain=0.4, read=tip, label="the forceps' tips over the rest's slot")
    floor = np.array([FORCEPS_REST[0], FORCEPS_REST[1], TOP + 0.004 + 0.002])  # the rest's floor, under the slot
    yield from L.servo(floor, tol=0.0015, timeout=2.5, gain=0.3, read=tip, label="the forceps' tips down into the slot to the rest's floor")
    if mode != "hover":
        yield from L.open(label="let go of the forceps")
    yield from ctx.together(L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="left straight up"), R.move(R.at() + np.array([0.0, 0.0, 0.04]), t=1.0, label="right up"))
    yield from ctx.together(L.move(L.at() + np.array([-0.06, 0.04, 0.0]), t=1.0, label="left clear"), R.move(R.at() + np.array([-0.06, -0.04, 0.0]), t=1.0, label="right clear"))
    yield from L.wait(2.5)
    print("[su] END " + json.dumps({"mode": mode, **STITCH.last}), flush=True)
