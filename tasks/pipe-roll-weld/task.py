"""Butt-weld two aluminium pipes lying in V-blocks: slide the loose pipe home against the fixed one, squeeze the
spring clamp open on its stand, put it over the joint and let it close, MIG-tack the joint through the slots in its
bands (turning the pipes in their V-blocks), squeeze it open and set it back on its stand, then weld the rest of the
joint all round, turning the pipes; finish with the gun on its rest and both hands clear.

Scene (world frame: x forward from the arm bases, y left, z up; positions in this file's constants):
- two 1 1/2 in Schedule 40 aluminium pipes (48.3 mm, 30 cm), each level in two steel 90-degree V-blocks, end stops
  past their outer ends; the fixed pipe carries roll marks 1-3 near its outer end;
- a JTLD pipe-welding spring clamp (stainless, 0.5 kg): two sprung halves on a pivot whose jaws end in slotted
  half-ring bands; normally shut, squeezed to open; clamped on a pipe-stub stand;
- MIG gun on its rest (front right);
- both grippers wear fiberglass welding-blanket covers: a sleeve on each finger and a jacket over the gripper.

Run the scripted demo: python rundemo.py task.py OUT_DIR --max-steps 7200 (PRW_DEMO=noclamp: welds without the clamp;
PRW_DEMO=tacks: stops after the tacks; PRW_DEBUG=FILE: logs the wire tip on every weld move).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402

import mechkit as mk  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = (
    "Slide the loose pipe along its V-blocks until its end meets the other pipe's end. The welding clamp is clamped on "
    "its stand, its spring holding its jaws shut: squeeze its handles together to open the jaws, lift it off, put it "
    "over the joint and let go so it grips both pipes. With the clamp on, tack the joint through the slots in the "
    "clamp's bands, at least 1 s of arc on each of at least three 10-degree stretches, turning the pipes in their "
    "V-blocks as needed but never tipping the clamp more than about 30 degrees from upright (further, it slips round "
    "the pipe and falls over). Then set the clamp upright, squeeze its handles to open it, lift it off and let it close"
    " on its stand again. Weld the rest of the joint all the way round, turning the pipes to bring each part on top. "
    "Finish with the gun on its rest and both hands clear."
)

# -- colours ---------------------------------------------------------------------------------------------------------
JIG_GREY = (0.35, 0.36, 0.38)
STEEL = (0.52, 0.53, 0.55)
PIPE = (0.66, 0.67, 0.69)  # bare aluminium pipe
BRIGHT = (0.86, 0.86, 0.88)  # wire-brushed bright at the bevels
ALU = (0.80, 0.81, 0.83)
STAINLESS = (0.70, 0.71, 0.73)
LEVER = (0.08, 0.08, 0.08)
RED = (0.80, 0.10, 0.08)
PLY = (0.72, 0.58, 0.38)
GUN_BLACK = (0.10, 0.10, 0.11)
COPPER = (0.72, 0.45, 0.20)
MARK_COLORS = ((0.95, 0.95, 0.95), (0.95, 0.80, 0.10), (0.15, 0.35, 0.85))  # marks 1, 2, 3
BEAD = (0.95, 0.42, 0.08)  # a welded segment, drawn orange
SLEEVE = (0.93, 0.91, 0.86)  # fiberglass finger sleeves (off-white)

# -- geometry (m) ----------------------------------------------------------------------------------------------------
AX_X = 0.19  # pipe axis, x, at the start (each roll moves the pipe ~3.5 cm along +x)
PIPE_R = 0.02415  # 1 1/2 in NPS Schedule 40 aluminium (6061-T6): 48.3 mm outside, 3.7 mm wall
PIPE_L = 0.30
PIPE_MASS = 0.42  # 1.39 kg/m (aluminium, 2.7 g/cm3)
AX_Z = TOP + PIPE_R * math.sqrt(2) + 0.0003  # pipe axis height: in the 90 deg V-blocks (V's apex at the table;
# the pipes' bottoms ~10 mm over it)
VBLOCK_Y = (-0.26, 0.26)  # the V-blocks' centres (y): 2-4 cm in from the pipes' outer ends, so the pipes can't run
# off them
VBLOCK_L = 0.040  # their length along the pipes
VBLOCK_IN_Y = (-0.10, 0.10)  # a second pair, 10 cm either side of the joint: each pipe rests level in two V-blocks
# (in one, near its outer end, it tipped nose-down onto the table and the two pipes met 5 deg out of line)
END_STOP_Y = (-0.336, 0.306)  # steel end stops just past the loose pipe's starting outer end and behind the fixed
# pipe's outer end: the pipes can't creep off their V-blocks
JOINT_Y = 0.0  # the joint, once the loose pipe is home
GAP0 = 0.03  # loose pipe's starting gap
FIX_Y = (JOINT_Y, JOINT_Y + PIPE_L)  # fixed pipe's ends
LOOSE_Y = (JOINT_Y - GAP0 - PIPE_L, JOINT_Y - GAP0)
MARK_Y = FIX_Y[1] - 0.006  # roll marks: on the fixed pipe beside its square end

# JTLD pipe-welding spring clamp (after the reference photos): two stainless halves on a pivot, each a jaw ending in a
# half-ring band and a handle on the same side, 0.5 kg in all. The bands wrap the pipe from either side, each from 5 to
# 135 deg from the top, 44 mm wide along it and centred on the joint. Each band is slotted from its tip to 40 deg from
# the top (a 28 mm slot along the joint: room for a 16 mm nozzle with the clamp a few mm off centre), so the joint shows through both sides and through the gap between the bands'
# tips underneath; only its top 80 deg, under the clamp's head, is covered. Clamp frame = seated: origin on the pipes'
# axis at the joint, y along the pipes, z up; the clamp stands in the x-z plane over the joint.
# A spring holds the jaws shut: it presses the bands onto the pipe and grips it with nobody holding the clamp.
# Squeezing the handles together swings both jaws open (each band out about the pivot; at 10 deg each its tip is ~5 mm
# clear of the pipe's widest point), so the clamp goes on and comes off straight down and up; let go, and the spring
# closes the bands on the pipe again. Nothing snaps into place: the bands grip and centre the pipes only by contact
# (their 135 deg wrap pushes the pipe ends to the jaws' centre and into line). With nothing in the jaws they close
# past shut to a stop at -8 deg.
CLAMP_MASS, LEVER_MASS = 0.25, 0.25  # the two halves, 0.5 kg in all
PY_F, PY_L, PLATE_T = 0.0045, -0.0045, 0.008  # the halves' planes (y) and their thickness
ZP = 0.052  # the pivot, over the pipes' axis
BAND_Y, SLOT_Y = 0.022, 0.014  # the bands' half-width and their slots' half-width, along y
SLOT_FROM = 40  # the slots run from the bands' tips to this far (deg) from the top
PAD_IN, PAD_T = 0.0, 0.005  # the bands' inner faces sit on the pipe with the jaws shut; their thickness
FIXED_BAND, LEVER_BAND = (5, 135), (-5, -135)  # deg from the top (+: toward +x)
OPEN_DEG = 28.0  # the handles squeezed together as far as they go: the jaws 28 deg open (14 deg each side)
SHUT_STOP = -8.0  # with nothing in the jaws the spring closes them to this (deg)
LEVER_K, LEVER_REST = 3.7, -25.0  # closing spring (N*m/rad), aimed 25 deg past shut: shut on the pipe it presses
# each band on with ~22 N; holding the jaws CARRY_DEG open takes ~34 N at the grips (85 mm up the
# handles: the long handles give the hand the leverage)
GRIP_DEG = (-2.5, 1.5)  # jaws within this of shut (deg): the bands are pressed onto a 48.3 mm pipe
HANDLE_OUT = 0.034  # the handles' outer faces, x +-34 mm with the jaws shut
Z1 = ZP + 0.085  # where a hand pinches the handles: to carry, open and release them
GRIP_FRICTION = (0.8, 0.7)  # knurled handles
GRIP_W, GRIP_YC, GRIP_Z = 0.036, 0.0, (ZP + 0.068, ZP + 0.102)  # the handles' grips: width, centre (y), extent (z)
GRIPPER_K = 5000.0  # the gripper is a position servo: N per m short of where it's told to close (50 N at most)
CARRY_DEG = 20.0  # a hand squeezes the handles to hold the jaws this far open (bands' tips ~5 mm clear of the pipe)
CARRY_FORCE = 34.0  # N: the spring's push on the handles at CARRY_DEG, which that hand balances (the YAM gripper
# squeezes up to ~41 N)
ON_JOINT_Y, ON_JOINT_R, ON_JOINT_ANG = 0.008, 0.003, 5.0  # the clamp counts as on the joint (for tacks) with its
# bands' centre within 8 mm of the joint along the pipes (the joint inside the 28 mm slots), its pivot within 3 mm of
# its seated distance from the pipes' axis and its plane within 5 deg of square to the axis; measured, never enforced
CLAMP_REST = (AX_X, 0.42)  # its stand (a pipe stub it is clamped on), past the left pipe's end
COVERED = SLOT_FROM + 5  # the gun keeps this far (deg) from the top of the clamp's bands

# Marks 1, 2, 3 at 0, -120 and -240 deg round the fixed pipe: each comes on top by turning the pipe +120 deg, its
# top away from the robot (the way a hand rolls it reliably; dragged toward the robot it slipped)
MARK_ANGLES = (0.0, -2 * math.pi / 3, -4 * math.pi / 3)
N_SEG = 12  # the joint's 12 segments of 30 deg, numbered from the top at the start (mark 1), turning about +y
SEG = 2 * math.pi / N_SEG
WELD_REACH = 0.004  # the wire tip within 4 mm of the joint line heats the segment under it
WELD_THIRD_S = 1.0  # a segment is welded once the tip has spent at least 1 s over each of its three 10 deg thirds ...
WELD_MIN_S = 3.0  # ... and 3 s over it in all ...
WELD_MAX_S = 5.0  # ... and burned through (a hole: it doesn't count, ever) once it has had more than 5 s
BUTT_GAP = 0.003  # the pipe ends butted: root gap under 3 mm (an open-root butt weld bridges 1.6-3.2 mm; pushed
# home, the ends sit ~1.5 mm apart in the sim)
JOIN_SEGMENTS = 3  # the pipes are joined once this many tacks (or welded segments) are laid (and butted)

GUN_HANDLE = (0.032, 0.15, 0.032)
GUN_TIP = np.array([0.0, 0.20, -0.062])  # wire tip from the handle's centre (gun frame = handle centre, y along)
GUN_REST = (0.10, -0.30)  # handle centre on its rest (x, y)
GUN_Z = TOP + 0.075  # handle centre height on the rest
GUN_MASS = 0.6

kit = mk.Kit()


def rq(axis, angle):
    """xyzw quaternion turning ``angle`` rad about ``axis``."""
    return tuple(float(c) for c in mk.mat_to_quat(mk.axis_angle(axis, angle)))


# -- pipes (free) ----------------------------------------------------------------------------------------------------
kit.part("pipe_fixed", (AX_X, (FIX_Y[0] + FIX_Y[1]) / 2, AX_Z), [
    mk.cyl(PIPE_R, PIPE_L - 0.03, (0, 0.015, 0), axis="y"),
    mk.cyl(PIPE_R, 0.015, (0, -PIPE_L / 2 + 0.0075, 0), axis="y", color=BRIGHT),  # ground bright at the bevel
    mk.cyl(PIPE_R, 0.015, (0, -PIPE_L / 2 + 0.0225, 0), axis="y", color=BRIGHT),
    # roll marks 1, 2, 3: paint, drawn only (as colliders, a mark at the bottom chocked the pipe on the table)
    *[mk.box((0.006, 0.004, 0.001), (PIPE_R * math.sin(a), MARK_Y - (FIX_Y[0] + FIX_Y[1]) / 2,
                                      PIPE_R * math.cos(a)), color=c, quat=rq((0, 1, 0), a), collide=False)
      for a, c in zip(MARK_ANGLES, MARK_COLORS)],
    # the weld bead: 12 segments of 30 deg round the joint, drawn only, shown as each is welded
    *[mk.box((0.016, 0.007, 0.0025), (PIPE_R * math.sin(a), -PIPE_L / 2, PIPE_R * math.cos(a)), color=BEAD,
             quat=rq((0, 1, 0), a), collide=False, hidden=True)
      for a in ((k + 0.5) * SEG for k in range(N_SEG))],
], mass=PIPE_MASS, color=PIPE, friction=(0.35, 0.3))
BEAD0 = len(kit.parts["pipe_fixed"].boxes) - N_SEG  # box index of bead segment 0
kit.part("pipe_loose", (AX_X, JOINT_Y - PIPE_L / 2, AX_Z), [  # declared home (butted), spawned 3 cm short
    mk.cyl(PIPE_R, PIPE_L - 0.03, (0, -0.015, 0), axis="y"),
    mk.cyl(PIPE_R, 0.03, (0, PIPE_L / 2 - 0.015, 0), axis="y", color=BRIGHT),
], mass=PIPE_MASS, color=PIPE, friction=(0.35, 0.3),
         spawn=((AX_X, (LOOSE_Y[0] + LOOSE_Y[1]) / 2, AX_Z), (0.0, 0.0, 0.0, 1.0)))
kit.latch("pipe_weld", "pipe_fixed", "pipe_loose")  # joined where they are once tacked (engage_here; no snap)

# -- welding clamp (free), declared seated over the joint, shut; it starts clamped on its stand -------------------
def arc_pad(a0, a1, y0, y1):
    """A piece of band: the arc from a0 to a1 deg (from the top, toward +x), over y0..y1."""
    a, span = math.radians((a0 + a1) / 2), math.radians(abs(a1 - a0))
    rc = PIPE_R + PAD_IN + PAD_T / 2
    return mk.box((2 * rc * math.sin(span / 2) + 0.001, y1 - y0, PAD_T),
                  (rc * math.sin(a), (y0 + y1) / 2, rc * math.cos(a)), quat=rq((0, 1, 0), a))


def band(a0, a1):
    """A jaw's band from a0 to a1 deg: full width near the top, then two prongs either side of the slot."""
    s = 1 if a1 > a0 else -1
    edges = list(range(a0, a1, s * 10)) + [a1]
    out = []
    for b0, b1 in zip(edges[:-1], edges[1:]):
        if abs(b0 + b1) / 2 < SLOT_FROM - 5:
            out.append(arc_pad(b0, b1, -BAND_Y, BAND_Y))
        else:
            out += [arc_pad(b0, b1, SLOT_Y, BAND_Y), arc_pad(b0, b1, -BAND_Y, -SLOT_Y)]
    return out


def link(p0, p1, y, width, color=None):
    """A flat bar in the clamp's plane (at y) from (x, z) p0 to p1."""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    return mk.box((width, PLATE_T, math.hypot(dx, dz)), ((p0[0] + p1[0]) / 2, y, (p0[1] + p1[1]) / 2),
                  quat=rq((0, 1, 0), math.atan2(dx, dz)), color=color)


def band_top(deg):
    r, a = PIPE_R + PAD_IN + PAD_T, math.radians(deg)
    return (r * math.sin(a), r * math.cos(a))


def half(side, py):
    """One half of the clamp: its band and its handle both on the ``side`` (+1: +x) of the pipe; squeezing the
    handles together swings the bands apart."""
    s = side
    return band(*(FIXED_BAND if s > 0 else LEVER_BAND)) + [
        link(band_top(8 * s), (0.006 * s, ZP - 0.004), py, 0.010),  # the jaw, from its band up to the pivot
        mk.cyl(0.008, PLATE_T, (0.0, py, ZP), axis="y"),  # pivot boss
        link((0.004 * s, ZP), (0.030 * s, ZP + 0.016), py, 0.009),
        link((0.030 * s, ZP + 0.016), (0.030 * s, ZP + 0.110), py, 2 * (HANDLE_OUT - 0.030)),  # handle, 26-34 mm out
        mk.box((0.004, GRIP_W, GRIP_Z[1] - GRIP_Z[0]),
               (s * (HANDLE_OUT - 0.002), GRIP_YC, (GRIP_Z[0] + GRIP_Z[1]) / 2)),  # its knurled grip
    ]


def _com(boxes):
    v = np.array([np.prod(b.size) for b in boxes])
    return tuple(float(c) for c in (v[:, None] * np.array([b.at for b in boxes])).sum(0) / v.sum())


_rest_pivot = np.array([CLAMP_REST[0], CLAMP_REST[1], AX_Z + ZP])
CLAMP_SPAWN = ((CLAMP_REST[0], CLAMP_REST[1], AX_Z + 0.0003), (0.0, 0.0, 0.0, 1.0))
clamp_boxes = half(+1, PY_F)
kit.part("clamp", (AX_X, JOINT_Y, AX_Z), clamp_boxes, mass=CLAMP_MASS, color=STAINLESS, friction=GRIP_FRICTION,
         com=_com(clamp_boxes), spawn=CLAMP_SPAWN)
lever_boxes = half(-1, PY_L)
kit.part("lever", (AX_X, JOINT_Y, AX_Z), lever_boxes, mass=LEVER_MASS, color=STAINLESS, friction=GRIP_FRICTION,
         com=_com(lever_boxes))
kit.hinge("lever_pivot", "clamp", "lever", (AX_X, JOINT_Y, AX_Z + ZP), (0, 1, 0), SHUT_STOP, OPEN_DEG,
          start_deg=0.0, stiffness=LEVER_K, rest_deg=LEVER_REST, damping=0.05)  # +: the jaws open

# -- V-blocks (fixed): steel, 90 deg V, a relief slot along the V's bottom; one under each pipe's outer end --------
def vblock_boxes():
    """A 90 deg V-block in its own frame (origin at the V's apex on the table): its two V faces, each a 12 mm thick
    plate whose top face is the V, and two cheeks down to the table."""
    out = []
    for side in (-1, 1):
        u = np.array([side, 1.0]) / math.sqrt(2)  # along the face, up and out
        n = np.array([side, -1.0]) / math.sqrt(2)  # into the block, below the face
        c = 0.026 * u + 0.006 * n
        out.append(mk.box((0.040, VBLOCK_L, 0.012), (float(c[0]), 0.0, float(c[1])), quat=rq((0, 1, 0), -side * math.pi / 4)))
        out.append(mk.box((0.016, VBLOCK_L, 0.034), (side * 0.040, 0.0, 0.017)))  # cheek
    return out


for i, vy in enumerate(VBLOCK_Y + VBLOCK_IN_Y):
    kit.part(f"vblock{i}", (AX_X, vy, TOP), vblock_boxes(), color=STEEL, friction=(0.6, 0.55))  # aluminium on steel
for i, sy in enumerate(END_STOP_Y):
    out = 1.0 if sy > 0 else -1.0
    kit.part(f"end_stop{i}", (AX_X, sy, TOP), [mk.box((0.040, 0.006, 0.060), (0.0, 0.0, 0.030)),
                                              mk.box((0.060, 0.030, 0.006), (0.0, out * 0.012, 0.003))], color=STEEL)

# -- rests (fixed) ---------------------------------------------------------------------------------------------------
# The clamp's stand: a 10 cm stub of the same pipe on a steel foot; the clamp starts clamped on it. A thin ridge
# along its top sits in the gap between the bands' tops.
kit.part("clamp_rest", (CLAMP_REST[0], CLAMP_REST[1], AX_Z), [
    mk.cyl(PIPE_R, 0.10, (0.0, 0.0, 0.0), axis="y"),
    mk.box((0.003, 0.090, 0.005), (0.0, 0.0, PIPE_R + 0.0015)),  # the ridge
    mk.box((0.030, 0.080, AX_Z - TOP - PIPE_R + 0.002), (0.0, 0.0, -(AX_Z - TOP + PIPE_R - 0.002) / 2)),
], color=PIPE)
gun_bottom = GUN_Z - GUN_HANDLE[2] / 2
kit.part("gun_rest", (GUN_REST[0], GUN_REST[1], TOP), [
    mk.box((0.03, 0.02, gun_bottom - TOP), (0.0, -0.05, (gun_bottom - TOP) / 2)),
    mk.box((0.03, 0.02, gun_bottom - TOP), (0.0, 0.04, (gun_bottom - TOP) / 2)),
    mk.box((0.09, 0.14, 0.006), (0.0, -0.005, 0.003)),
], color=STEEL)

# -- MIG gun (free): handle centre is its frame; nozzle forward along +y and down -------------------------------------
neck_len = math.hypot(0.125, 0.02)
neck_tilt = -math.atan2(0.02, 0.125)  # about x: +y toward -z
kit.part("gun", (GUN_REST[0], GUN_REST[1], GUN_Z), [
    mk.box(GUN_HANDLE),
    mk.box((0.014, neck_len, 0.014), (0.0, 0.075 + 0.0625, -0.010),
           quat=rq((1, 0, 0), neck_tilt)),
    mk.cyl(0.008, 0.032, (0.0, 0.20, -0.036), axis="z", color=COPPER),  # nozzle
    mk.box((0.002, 0.002, 0.010), (0.0, 0.20, -0.057), color=COPPER),  # wire stick-out
], mass=GUN_MASS, color=GUN_BLACK)

OBJECTS = [*kit.objects()]


def seat_frame(objects):
    """Where the clamp sits when seated: on the joint's centre, upright, along the pipes; however far the round pipes
    have turned (that doesn't matter to the clamp)."""
    c, r = joint_frame(objects)
    y = r[:, 1]
    z = np.array([0.0, 0.0, 1.0]) - y[2] * y
    z /= np.linalg.norm(z)
    return c, np.column_stack([np.cross(y, z), y, z])


def clamp_seat_error(objects) -> tuple[float, float]:
    """The clamp's distance (m) from seated over the joint, and its skew (deg) off the pipes' axis. Its turn about
    that axis doesn't count: its collars are round, they hold the pipes in line at any turn."""
    c, rs = seat_frame(objects)
    p, q = kit.pose(objects, "clamp")
    d = float(np.linalg.norm(np.asarray(p) - c))
    cy = mk.quat_to_mat(q)[:, 1]
    return d, math.degrees(math.acos(max(-1.0, min(1.0, abs(float(cy @ rs[:, 1]))))))


def lever_deg(objects) -> float:
    """How far the movable jaw is open (deg): 0 shut, OPEN_DEG fully open."""
    return kit.q_deg(objects, "lever_pivot")


def gripping() -> bool:
    """The clamp is squeezed shut on the joint (as measured at the last control step)."""
    return bool(mk.MEMORY.get("gripping"))


def clamp_on_joint(objects) -> bool:
    """Measured, never enforced: the jaws are squeezed onto a pipe and the clamp sits over the joint (ON_JOINT_*)."""
    q = lever_deg(objects)
    if not GRIP_DEG[0] < q < GRIP_DEG[1]:
        return False
    c, r = joint_frame(objects)
    v = pivot(objects) - c
    along = float(r[:, 1] @ v)
    radial = float(np.linalg.norm(v - r[:, 1] * along))
    _, skew = clamp_seat_error(objects)
    return abs(along) < ON_JOINT_Y and abs(radial - ZP) < ON_JOINT_R and skew < ON_JOINT_ANG


def pivot(objects):
    return kit.point(objects, "clamp", (0.0, 0.0, ZP))


def _grip_step(objects) -> None:
    mk.MEMORY["gripping"] = clamp_on_joint(objects)


kit.on_step(_grip_step)


def joint_frame(objects):
    """The joint's centre (world) and the fixed pipe's rotation (its +y is the pipes' axis)."""
    pos, quat = kit.pose(objects, "pipe_fixed")
    r = mk.quat_to_mat(quat)
    return np.asarray(pos) + r @ np.array([0.0, -PIPE_L / 2, 0.0]), r


def wire_tip(objects) -> np.ndarray:
    return kit.point(objects, "gun", tuple(GUN_TIP))


def third_at(objects, p):
    """The joint's 10 deg third (0-35; segment = third // 3) a point is within WELD_REACH of, or None."""
    c, r = joint_frame(objects)
    v = r.T @ (np.asarray(p) - c)  # in the fixed pipe's frame: y along the axis, angle about y from its +z
    if abs(v[1]) > WELD_REACH or abs(math.hypot(v[0], v[2]) - PIPE_R) > WELD_REACH:
        return None
    return int(((math.atan2(v[0], v[2]) % (2 * math.pi)) // (SEG / 3))) % (3 * N_SEG)


def segment_at(objects, p):
    """The joint segment (0-11) a point is within WELD_REACH of, or None."""
    t = third_at(objects, p)
    return None if t is None else t // 3


def heat() -> np.ndarray:
    """Seconds the wire tip has spent over each of the joint's 36 thirds (with the pipes butted)."""
    return mk.MEMORY.setdefault("heat", np.zeros(3 * N_SEG))


def burned() -> set:
    """Segments burned through (over WELD_MAX_S): never counted as welded."""
    return mk.MEMORY.setdefault("burned", set())


def welded() -> set:
    return mk.MEMORY.setdefault("welded", set())


def tacked() -> set:
    """Tacks: the 10 deg thirds (0-35) that got their full WELD_THIRD_S while the clamp gripped the joint."""
    return mk.MEMORY.setdefault("tacked", set())


def joined_by_weld() -> bool:
    """Three tacks (or three welded segments) hold the pipes together."""
    return len(tacked()) >= JOIN_SEGMENTS or len(welded()) >= JOIN_SEGMENTS


_SHOWN: dict = {}


def _weld_step(objects) -> None:
    """Weld rule: with the pipes butted (root gap under 3 mm: a bead can't bridge a wider one), each control step the wire
    tip spends at the joint heats the 10 deg third under it. A segment is welded (shown orange) once each of its thirds
    has had 1 s and the segment 3 s; over 5 s it is burned through and never counts. Welded with the clamp gripping
    the joint, each third that gets its full 1 s is a tack (a 4 mm bead). Three tacks (or three welded segments) join the
    pipes."""
    t = third_at(objects, wire_tip(objects))
    w = welded()
    if t is not None and abs(pipe_gap(objects)) < BUTT_GAP:
        h = heat()
        h[t] += DT
        if h[t] >= WELD_THIRD_S - 1e-9 and gripping():
            tacked().add(t)
        k = t // 3
        th = h[3 * k:3 * k + 3]
        if th.sum() > WELD_MAX_S + 1e-9:
            if k not in burned():
                print(f"    weld: segment {k} burned through ({th.sum():.2f} s)", flush=True)
            burned().add(k)
            w.discard(k)
            tacked().difference_update({3 * k, 3 * k + 1, 3 * k + 2})
        elif k not in w and th.min() >= WELD_THIRD_S - 1e-9 and th.sum() >= WELD_MIN_S - 1e-9:
            w.add(k)
    for i in range(N_SEG):
        if _SHOWN.get(i) != (i in w):
            mk.show_box("pipe_fixed", BEAD0 + i, i in w)
            _SHOWN[i] = i in w
    if joined_by_weld() and not kit.engaged("pipe_weld") and abs(pipe_gap(objects)) < BUTT_GAP:
        kit.engage_here("pipe_weld", objects)


kit.on_step(_weld_step)


def pipe_gap(objects) -> float:
    """Gap (m) between the loose pipe's end and the fixed pipe's end, along the fixed pipe's axis."""
    c, r = joint_frame(objects)
    pl, _ = kit.pose(objects, "pipe_loose")
    return float(r[:, 1] @ (c - (np.asarray(pl) + r[:, 1] * PIPE_L / 2)))


# -- fiberglass finger sleeves (welding-blanket cloth, 540 C): a duck-bill cover on each finger (2 mm wall), added to the -----
# -- robots' finger links ----------------------------------------------------------------------------------------
# Finger link frames (from the YAM meshes): the finger runs along -z to its tip at z -0.091; its plate spans
# x 0.004-0.044 (tip_right: -0.044 to -0.004) and y -0.0545 to -0.0445 (tip_right: +0.0445 to +0.0545).
SLEEVE_WALL = 0.002
SLEEVE_FRICTION = (0.7, 0.6)  # woven fiberglass cloth
_SLEEVE_BOXES = {  # (centre, size) in the link frame: the full-width sleeve, then the duck-bill tip
    "tip_left": [((0.024, -0.0495, -0.0525), (0.040 + 2 * SLEEVE_WALL, 0.010 + 2 * SLEEVE_WALL, 0.045)),
                 ((0.024, -0.0476, -0.083), (0.016 + 2 * SLEEVE_WALL, 0.0062 + 2 * SLEEVE_WALL, 0.016 + SLEEVE_WALL))],
    "tip_right": [((-0.024, 0.0495, -0.0525), (0.040 + 2 * SLEEVE_WALL, 0.010 + 2 * SLEEVE_WALL, 0.045)),
                  ((-0.024, 0.0476, -0.083), (0.016 + 2 * SLEEVE_WALL, 0.0062 + 2 * SLEEVE_WALL, 0.016 + SLEEVE_WALL))],
}


_JACKET_BOXES = [((0.0, 0.0, 0.047), (0.110, 0.071, 0.040)), ((0.0, 0.0, 0.010), (0.064, 0.064, 0.044))]


def _add_sleeves(stage, env_path) -> None:
    import isaaclab.sim as sim_utils
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    mat = f"{env_path}/sleeve_material"
    sim_utils.RigidBodyMaterialCfg(static_friction=SLEEVE_FRICTION[0], dynamic_friction=SLEEVE_FRICTION[1]).func(
        mat, sim_utils.RigidBodyMaterialCfg(static_friction=SLEEVE_FRICTION[0], dynamic_friction=SLEEVE_FRICTION[1]))
    look = f"{env_path}/sleeve_look"
    cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(c ** 2.2 for c in SLEEVE), roughness=0.8)
    cfg.func(look, cfg)
    n = 0
    # the finger links: rigid bodies named tip_left / tip_right under each arm (e.g. .../LeftArm/Geometry/.../tip_left)
    fingers = [p for p in stage.Traverse() if str(p.GetPath()).startswith(env_path + "/")
               and p.GetName() in _SLEEVE_BOXES and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for prim in fingers:
            path = str(prim.GetPath())
            for i, (c, s) in enumerate(_SLEEVE_BOXES[prim.GetName()]):
                cube = UsdGeom.Cube.Define(stage, f"{path}/sleeve{i}")
                cube.CreateSizeAttr(1.0)
                xf = UsdGeom.Xformable(cube)
                xf.AddTranslateOp().Set(Gf.Vec3d(*c))
                xf.AddScaleOp().Set(Gf.Vec3f(*s))
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                pc = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
                pc.CreateContactOffsetAttr(0.004)
                pc.CreateRestOffsetAttr(0.0)
                sim_utils.bind_physics_material(f"{path}/sleeve{i}", mat, stage=stage)
                sim_utils.bind_visual_material(f"{path}/sleeve{i}", look, stage=stage)
                n += 1
    # The fiberglass jacket over each gripper's housing and wrist collar: drawn only (gripper link frame, approach
    # along +z; housing x +-51, y +-32.5 mm, z 30-64 mm)
    grippers = [p for p in stage.Traverse() if str(p.GetPath()).startswith(env_path + "/")
                and p.GetName() == "gripper" and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for prim in grippers:
        for i, (c, s) in enumerate(_JACKET_BOXES):
            cube = UsdGeom.Cube.Define(stage, f"{prim.GetPath()}/jacket{i}")
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube)
            xf.AddTranslateOp().Set(Gf.Vec3d(*c))
            xf.AddScaleOp().Set(Gf.Vec3f(*s))
            sim_utils.bind_visual_material(str(cube.GetPath()), look, stage=stage)
    # A gripper's two sleeves overlap when it closes (the pads touch): the real cloth squashes, so the two fingers
    # of each gripper don't collide with each other.
    by_parent = {}
    for prim in fingers:
        by_parent.setdefault(str(prim.GetParent().GetPath()), {})[prim.GetName()] = prim
    for pair in by_parent.values():
        if len(pair) == 2:
            UsdPhysics.FilteredPairsAPI.Apply(pair["tip_left"]).CreateFilteredPairsRel().AddTarget(
                pair["tip_right"].GetPath())
    print(f"[pipe-roll-weld] finger sleeves: {n} boxes on {len(fingers)} fingers, {len(by_parent)} grippers",
          flush=True)
    bc = UsdGeom.BBoxCache(0, ["default", "render", "proxy", "guide"])
    for prim in fingers[:2]:
        fb = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        print(f"    {prim.GetPath()}: finger world box {fb.GetMin()} - {fb.GetMax()}", flush=True)
        for i in range(2):
            sb = bc.ComputeWorldBound(stage.GetPrimAtPath(f"{prim.GetPath()}/sleeve{i}")).ComputeAlignedRange()
            print(f"      sleeve{i} world box {sb.GetMin()} - {sb.GetMax()}", flush=True)
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        print(f"      link frame origin {xf.ExtractTranslation()}", flush=True)


mk.on_author(_add_sleeves)


# -- scoring -----------------------------------------------------------------------------------------------------------
SCORE_REST_POS = 0.02  # the clamp / the gun back within 2 cm of where they started ...
GUN_REST_DEG = 20.0  # ... the gun turned under 20 deg
HANDS_CLEAR = 0.06  # both hands (gripper centre points) this far from the pipes, the clamp and the gun
HOLD_STEPS = 20  # all of that, everything still, for 1 s (20 control steps)
IN_LINE_MM, IN_LINE_DEG = 0.8, 1.0  # the pipes' axes within 0.8 mm and 1 deg of each other
ON_BLOCK = 0.015  # a pipe's axis within 15 mm of where its V-block holds it, at the block, its end on the block


def pipes_in_line(objects) -> bool:
    off, ang = line_error(objects)
    return ang < IN_LINE_DEG and off * 1000 < IN_LINE_MM


def line_error(objects):
    """The loose pipe's end off the fixed pipe's axis (m) and the angle between their axes (deg)."""
    c, rf = joint_frame(objects)
    pl, ql = kit.pose(objects, "pipe_loose")
    yf, yl = rf[:, 1], mk.quat_to_mat(ql)[:, 1]
    ang = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(yf @ yl))))))
    end = np.asarray(pl) + yl * PIPE_L / 2  # the loose pipe's joint end, on its axis
    off = (end - c) - yf * float(yf @ (end - c))  # its offset across the fixed pipe's axis
    return float(np.linalg.norm(off)), ang


def _on_block(objects, part, by) -> bool:
    """The pipe still lies in its V-block: its axis, at the block, within ON_BLOCK of where the block holds it."""
    p, q = kit.pose(objects, part)
    ya = mk.quat_to_mat(q)[:, 1]
    if abs(ya[1]) < 0.9:
        return False
    a = np.asarray(p) + ya * (by - p[1]) / ya[1]
    reach = abs(by - p[1]) <= PIPE_L / 2 + VBLOCK_L / 2 - 0.010  # the pipe's end on the block by at least 1 cm
    return reach and np.linalg.norm(a - np.array([AX_X, by, AX_Z])) < ON_BLOCK


def dropped(objects) -> bool:
    """A pipe off its V-block, or the clamp or the gun fallen off the table."""
    if not (_on_block(objects, "pipe_loose", VBLOCK_Y[0]) and _on_block(objects, "pipe_fixed", VBLOCK_Y[1])):
        return True
    return any(objects[n].pos[2] < TOP - 0.02 for n in ("clamp", "lever", "gun"))


def put_away(objects) -> bool:
    """The clamp back on its stand and the gun on its rest."""
    pc, _ = kit.pose(objects, "clamp")
    clamp = np.linalg.norm(np.asarray(pc) - kit.parts["clamp"].spawn[:3, 3]) < SCORE_REST_POS
    pg, qg = kit.pose(objects, "gun")
    r = mk.quat_to_mat(qg)
    turned = math.degrees(math.acos(max(-1.0, min(1.0, float(r[1, 1])))))  # its handle off the rest's line (y)
    gun = np.linalg.norm(np.asarray(pg) - np.array([GUN_REST[0], GUN_REST[1], GUN_Z])) < SCORE_REST_POS
    return clamp and gun and turned < GUN_REST_DEG


_SCORED = ("pipe_fixed", "pipe_loose", "clamp", "lever", "gun")


STILL_SPEED, STILL_SPIN = 0.01, 0.2  # m/s, rad/s, averaged over a control step (a contact's few-micron hum doesn't
# count as moving)
DT = 0.05  # s per control step


def _still(objects) -> bool:
    """Every scored object moved less than STILL_SPEED and STILL_SPIN since the last control step."""
    now = {n: kit.pose(objects, n) for n in _SCORED}
    last, mk.MEMORY["last_poses"] = mk.MEMORY.get("last_poses"), now
    if last is None:
        return False
    for n, (p, q) in now.items():
        p0, q0 = last[n]
        r = mk.quat_to_mat(q) @ mk.quat_to_mat(q0).T
        turn = math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))
        if np.linalg.norm(np.asarray(p) - np.asarray(p0)) > STILL_SPEED * DT or turn > STILL_SPIN * DT:
            return False
    return True


def _finished_now(objects) -> bool:
    still = _still(objects)  # (kept up to date every step)
    return (still and len(welded()) == N_SEG and len(tacked()) >= JOIN_SEGMENTS and pipes_in_line(objects)
            and put_away(objects) and all(mk.released(objects[n], HANDS_CLEAR) for n in _SCORED))


def _finish_step(objects) -> None:
    mk.MEMORY["held"] = mk.MEMORY.get("held", 0) + 1 if _finished_now(objects) else 0


kit.on_step(_finish_step)


def score(objects):
    """0 if a pipe leaves its V-block or the clamp or the gun falls off the table. Otherwise 0.9 x the fraction of the
    joint's 12 segments welded (rounded to 0.001), capped at 0.5 unless at least 3 were welded with the clamp
    gripping (tacks) and the pipes are in line. 1.0 with all 12 welded, the clamp back on its stand, the gun on its
    rest, both hands clear and everything still, held for 1 s."""
    if dropped(objects):
        return 0.0
    s = round(0.9 * len(welded()) / N_SEG, 3)
    if len(tacked()) < JOIN_SEGMENTS or not pipes_in_line(objects):
        s = min(s, 0.5)
    if mk.MEMORY.get("held", 0) >= HOLD_STEPS:
        return 1.0
    return s


def success(objects):
    return score(objects) == 1.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
YAW_HOME = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers closing along world x
CARRY_Z = TOP + 0.15


def _log(ctx, tag):
    st = ctx.states()
    d, a = clamp_seat_error(st)
    print(f"[demo t={ctx.t:5.1f}] {tag}: gap={pipe_gap(st) * 1000:.1f} mm clamp_off={d * 1000:.1f} mm/{a:.1f} deg "
          f"jaw={lever_deg(st):.1f} deg gripping={gripping()} "
          f"welded={sorted(welded())} fixed={np.round((kit.pose(st, 'pipe_fixed')[0] - np.array([AX_X, (FIX_Y[0] + FIX_Y[1]) / 2, AX_Z])) * 1000, 1).tolist()} mm "
          f"line={np.round(np.array(line_error(st)) * (1000, 1), 2).tolist()} joined={kit.engaged('pipe_weld')} tacks={sorted(tacked())} score={score(st)}", flush=True)


WINDOW_PASS_DEG = (-55.0, 55.0)  # with the clamp on: clear of its bridge (-75 to -90 deg)
PASS_DEG = (-68.0, 68.0)  # each full pass: the top third of the joint and 8 deg either side, so the passes overlap
# and a turn a little off still leaves no gap (the nozzle clears the pipe by ~1 mm at 68 deg)
CARRY_HIGH = TOP + 0.24  # carrying the clamp by the handles (its bands then ~2 cm over the pipes)
TILT_DEG = -25.0  # with the clamp on, the pipes are turned this far each way (at 45 the clamp's weight beat the
# V-blocks' grip on the pipes and they rolled over) (first: clockwise seen from the left,
# the handles tipping toward the robot) to weld each side
HIGH = TOP + 0.19  # travel height: clears the clamp and its handle (the arms reach ~20 cm here)


def mark_angle(objects) -> float:
    """Where mark 1 is (deg from straight up, about +y; mark n is on top at +(n-1)*120)."""
    _, r = joint_frame(objects)
    up = r[:, 2]  # the fixed pipe's local +z (mark 1) in the world
    return math.degrees(math.atan2(up[0], up[2]))


def demo_script(ctx):
    L, R = ctx.left, ctx.right
    _log(ctx, "start")
    yield from hold(ctx)  # the left hand holds the left pipe while the right slides the other home
    yield from push_home(ctx)
    _log(ctx, "pipe pushed home")
    yield from unhold(ctx)
    import os
    if os.environ.get("PRW_DEMO") == "noclamp":  # test variant: weld the whole joint without the clamp (capped 0.5)
        gun = yield from take_gun(ctx)
        yield from weld_round(ctx, gun)
        return
    yield from clamp_on(ctx)  # left: squeeze the clamp open on its stand, lower it onto the joint, let go: it grips
    _log(ctx, "clamp on")
    gun = yield from take_gun(ctx)
    for tilt in (TILT_DEG, -TILT_DEG):  # turn the pipes and clamp so the handles tip down to one side; tack the
        held = yield from tilt_hold(ctx, tilt)  # other side, now on top, through its slot (the handles clear of the
        yield from sweep(ctx, gun, PASS_DEG)  # gun); then the same the other way
        yield from park_gun(ctx)
        _log(ctx, f"tilted {tilt:+.0f}, side tacked")
        yield from tilt_back(ctx, held)
    st = ctx.states()  # make sure the clamp is upright again (it comes off straight up); only now, tacked: turned
    yield from turn_pipe(ctx, tol_deg=10.0, target=mark_angle(st) - clamp_turn(st))  # before, the pipes came apart
    if os.environ.get("PRW_DEMO") == "tacks":
        return
    yield from clamp_off(ctx)  # left: squeeze it open, lift it straight off, set it on its stand and let go
    _log(ctx, "clamp off, on its stand")
    yield from weld_round(ctx, gun)


def weld_round(ctx, gun):
    """Weld whatever is left of the joint, turning the pipes to bring each unwelded stretch on top; then the gun back
    on its rest and both hands clear."""
    L = ctx.left
    for _ in range(6):  # left grips the joined pipes and turns them to bring unwelded segments on top; right welds
        if len(welded()) >= N_SEG:
            break
        yield from turn_pipe(ctx)
        yield from rest_on_pipe(ctx)
        yield from sweep(ctx, gun, PASS_DEG)
        yield from park_gun(ctx)
        yield from unhold(ctx)
        _log(ctx, "pass")
    yield from return_gun(ctx, gun)
    _log(ctx, "done")
    st = ctx.states()
    pc, _ = kit.pose(st, "clamp")
    pg, qg = kit.pose(st, "gun")
    print(f"    finish: clamp {np.round((np.asarray(pc) - kit.parts['clamp'].spawn[:3, 3]) * 1000, 1)} mm, jaws "
          f"{lever_deg(st):.1f} deg; gun {np.round((np.asarray(pg) - np.array([GUN_REST[0], GUN_REST[1], GUN_Z])) * 1000, 1)} mm "
          f"r11 {mk.quat_to_mat(qg)[1, 1]:.3f}; clear {[n for n in _SCORED if not mk.released(st[n], HANDS_CLEAR)]} "
          f"in line {pipes_in_line(st)}; held {mk.MEMORY.get('held', 0)} steps", flush=True)
    yield from L.wait(3.0)


HOLD_LOCAL = (0.0, 0.05, 0.0)  # the left hand holds the fixed pipe here (20 cm from the joint, clear of its
# V-block), fingers across it


def hold(ctx, part="pipe_fixed", local=HOLD_LOCAL, side="left"):
    """Pinch a pipe across its sides (fingertips just over the table) to hold it steady."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    arm, yaw = (ctx.left, YAW_HOME["left"]) if side == "left" else (ctx.right, YAW_HOME["right"])
    p = kit.point(ctx.states(), part, local)
    z = float(p[2]) + 0.004  # fingertips (13 mm below the TCP, sleeves 0.5 mm more) ~3 mm over the table
    yield from arm.move((float(p[0]), float(p[1]), z + 0.05), yaw=yaw, finger=FINGER_OPEN, duration=1.0)
    yield from arm.move((float(p[0]), float(p[1]), z), yaw=yaw, duration=0.6)
    yield from arm.grip(FINGER_CLOSED, duration=0.5)


def rest_on_pipe(ctx):
    """Left hand rests lightly on the left pipe's top (closed fingertips 2 mm into it) to steady it."""
    from yam_arena.constants import FINGER_CLOSED

    L, yl = ctx.left, YAW_HOME["left"]
    p = kit.point(ctx.states(), "pipe_fixed", HOLD_LOCAL)
    z = float(p[2]) + PIPE_R + 0.013 - 0.002
    yield from L.move((float(p[0]), float(p[1]), z + 0.04), yaw=yl, finger=FINGER_CLOSED, duration=1.0)
    yield from L.move((float(p[0]), float(p[1]), z), yaw=yl, duration=0.6)


def park_gun(ctx):
    """Right hand, gun in hand, moves right, out of the left hand's way."""
    yield from ctx.right.move((0.05, -0.32, HIGH), yaw=YAW_HOME["right"], duration=1.0)  # nozzle well clear of the
    # clamp's grip plates (parked nearer, it hung over them and the left hand ran into it)


def unhold(ctx, side="left", park=True):
    from yam_arena.constants import FINGER_OPEN

    arm, yaw = (ctx.left, YAW_HOME["left"]) if side == "left" else (ctx.right, YAW_HOME["right"])
    yield from arm.grip(FINGER_OPEN, duration=0.4)
    c = mk.tcps()[0 if side == "left" else 1]
    yield from arm.move((float(c[0]), float(c[1]), HIGH), yaw=yaw, duration=0.6)
    if park:
        yield from arm.move((0.12, 0.25 if side == "left" else -0.25, HIGH), yaw=yaw, duration=0.8)


def clamp_target(objects, local):
    """Where a point of the clamp goes when it is seated over the joint (seat_frame)."""
    c, rs = seat_frame(objects)
    return rs @ np.asarray(local) + c


def push_home(ctx):
    """Right hand: grip the loose pipe near its outer end and slide it along the table until it meets the other,
    which the left hand holds."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    R, yr = ctx.right, YAW_HOME["right"]
    p, _ = kit.pose(ctx.states(), "pipe_loose")
    x0, y0 = float(p[0]), float(p[1]) - PIPE_L / 2 + 0.13  # 13 cm in from its outer end (clear of its V-block)
    z_grip = float(p[2]) + 0.004
    yield from R.move((x0, y0, z_grip + 0.07), yaw=yr, finger=FINGER_OPEN, duration=1.5)
    yield from R.move((x0, y0, z_grip), yaw=yr, duration=0.8)
    yield from R.grip(FINGER_CLOSED)
    for _ in range(8):  # slide until butted (closed loop on the gap; the soft arm needs aiming past it to push)
        gap = pipe_gap(ctx.states())
        if gap < 0.0005:
            break
        tcp = mk.tcps()[1]
        print(f"    push: gap {gap * 1000:.1f} mm, hand y {tcp[1]:.4f}", flush=True)
        yield from R.move((x0, float(tcp[1]) + gap + 0.015, z_grip), yaw=yr, duration=max(0.6, gap * 60))
        yield from R.wait(0.2)
    yield from R.grip(FINGER_OPEN, duration=0.4)
    tcp = mk.tcps()[1]
    yield from R.move((float(tcp[0]), float(tcp[1]), HIGH), yaw=yr, duration=0.8)
    yield from R.move((0.12, -0.25, HIGH), yaw=yr, duration=1.0)


def reach_down(ctx, arm, target, yaw, finger):
    """Stop 5 cm over ``target``, measure the hand's own error there, correct it, then go down: going straight down,
    the left hand landed 3 cm off and its fingertips came down on the clamp's grip plates."""
    i = 0 if arm is ctx.left else 1
    up = np.asarray(target, dtype=float) + np.array([0.0, 0.0, 0.05])
    yield from arm.move(tuple(up), yaw=yaw, finger=finger, duration=1.2)
    yield from arm.wait(0.4)
    err = up - np.asarray(mk.tcps()[i])
    err[2] = 0.0
    yield from arm.move(tuple(up + err), yaw=yaw, duration=0.5)
    yield from arm.move(tuple(np.asarray(target) + err), yaw=yaw, duration=0.8)
    print(f"    reach: corrected {np.round(err * 1000, 1)} mm; now off "
          f"{np.round((np.asarray(target) - np.asarray(mk.tcps()[i])) * 1000, 1)} mm", flush=True)


def finger_for(width):
    """Finger command that sets the sleeved pads ``width`` (m) apart."""
    from yam_arena.constants import FINGER_OPEN

    return FINGER_OPEN * min(1.0, max(0.0, (width + 2 * SLEEVE_WALL) / 0.094))


def squeeze_finger(width, force):
    """Finger command that squeezes something ``width`` wide with ``force`` (N; the gripper is a position servo)."""
    return finger_for(max(0.0, width - 2 * force / GRIPPER_K))


def handle_half_width(open_deg, z=Z1):
    """Half the width across the handles' outer faces at height ``z`` (clamp frame) with the jaws ``open_deg`` open,
    each half swung open_deg / 2 about the pivot (the handles toward each other)."""
    h = math.radians(open_deg / 2)
    return HANDLE_OUT * math.cos(h) - (z - ZP) * math.sin(h)


CARRY_W = 2 * handle_half_width(CARRY_DEG)


def handle_points(objects, z=Z1):
    """World points on the outer faces a hand pinches at height ``z`` (clamp frame): the +x half's handle and the
    -x half's."""
    return (kit.point(objects, "clamp", (HANDLE_OUT, GRIP_YC, z)),
            kit.point(objects, "lever", (-HANDLE_OUT, GRIP_YC, z)))


def _ypr(objects):
    """The clamp's yaw, pitch, roll (deg) about world z, y, x."""
    r = mk.quat_to_mat(kit.pose(objects, "clamp")[1])
    return (round(math.degrees(math.atan2(r[1, 0], r[0, 0])), 1), round(math.degrees(-math.asin(r[2, 0])), 1),
            round(math.degrees(math.atan2(r[2, 1], r[2, 2])), 1))


def clamp_turn(objects) -> float:
    """The clamp's turn about the pipes' axis (deg from upright, +: their top toward +x)."""
    r = mk.quat_to_mat(kit.pose(objects, "clamp")[1])
    return math.degrees(math.atan2(r[0, 2], r[2, 2]))


PRE = 0.045  # carried clamp stops this far over their seat, to measure and correct their offset before going down
FINE = 0.008  # and again this far over it


def carry_to(ctx, dest_pivot, settle=0.002):
    """Left hand, open clamp in hand: carry it high, then lower it until its pivot is at ``dest_pivot`` (+settle),
    correcting for how it hangs in the hand and the arm's sag on the way down: at 4.5 cm and again at 8 mm over the
    seat it waits for the hanging clamp to stop swinging, measures its miss and corrects, until within 2 mm."""
    L, yl = ctx.left, YAW_HOME["left"]
    st = ctx.states()
    off = np.asarray(mk.tcps()[0]) - pivot(st)  # TCP from the pivot, as the clamp hangs
    dest = np.asarray(dest_pivot, float)
    yield from L.move((dest[0] + off[0], dest[1] + off[1], CARRY_HIGH), yaw=yl, duration=2.0)
    err = np.zeros(3)
    for h, dur in ((PRE, 1.0), (FINE, 0.8)):
        yield from L.move(tuple(dest + off + err + np.array([0, 0, h])), yaw=yl, duration=dur)
        for _ in range(4):
            yield from L.wait(0.6)
            miss = dest + np.array([0, 0, h]) - pivot(ctx.states())
            miss[2] = 0.0
            if np.linalg.norm(miss) < 0.002:
                break
            err = err + np.clip(miss, -0.02, 0.02)
            yield from L.move(tuple(dest + off + err + np.array([0, 0, h])), yaw=yl, duration=0.5)
    yield from L.move(tuple(dest + off + err + np.array([0, 0, settle])), yaw=yl, duration=0.8)
    yield from L.wait(0.3)
    print(f"    clamp lowered: pivot off {np.round((dest - pivot(ctx.states())) * 1000, 1)} mm "
          f"(corrected {np.round(err * 1000, 1)} mm), ypr {_ypr(ctx.states())}, open {lever_deg(ctx.states()):.1f} deg",
          flush=True)


def take_clamp(ctx):
    """Left hand: reach down to the clamp's handles and squeeze them together: the jaws open (CARRY_DEG) and let go
    of the pipe; keep squeezing."""
    from yam_arena.constants import FINGER_OPEN

    L, yl = ctx.left, YAW_HOME["left"]
    a, b = handle_points(ctx.states())
    yield from reach_down(ctx, L, (a + b) / 2, yl, FINGER_OPEN)
    a, b = handle_points(ctx.states())
    t = np.asarray(mk.tcps()[0])
    yield from L.move((float((a[0] + b[0]) / 2), t[1], t[2]), yaw=yl, finger=squeeze_finger(CARRY_W, CARRY_FORCE),
                      duration=1.0)
    yield from L.wait(0.3)
    print(f"    clamp squeezed open: {lever_deg(ctx.states()):.1f} deg", flush=True)


def release_clamp(ctx):
    """Left hand, the open clamp seated: open the fingers slowly; the spring closes the bands onto the pipe. Lift
    clear."""
    from yam_arena.constants import FINGER_OPEN

    L, yl = ctx.left, YAW_HOME["left"]
    yield from L.grip(finger_for(0.5 * (CARRY_W + 2 * HANDLE_OUT)), duration=0.8)
    yield from L.grip(FINGER_OPEN, duration=0.5)
    yield from L.wait(0.3)
    st = ctx.states()
    print(f"    clamp let go: jaws {lever_deg(st):.1f} deg, on the joint {clamp_on_joint(st)}, "
          f"off {np.round(clamp_seat_error(st), 4)}, ypr {_ypr(st)}", flush=True)
    t = mk.tcps()[0]
    yield from L.move((t[0], t[1], HIGH), yaw=yl, duration=0.6)


def lift_clear(ctx):
    L = ctx.left
    t = np.asarray(mk.tcps()[0])
    yield from L.move((t[0], t[1], t[2] + 0.04), yaw=YAW_HOME["left"], duration=0.8)  # straight up
    yield from L.move((t[0], t[1], CARRY_HIGH), yaw=YAW_HOME["left"], duration=0.6)


def clamp_on(ctx):
    """Left hand: squeeze the clamp open on its stand, lift it off, carry it over the joint, lower it straight down
    onto the pipes and let go: the spring closes the bands on the pipes."""
    yield from take_clamp(ctx)
    yield from lift_clear(ctx)
    c, _ = seat_frame(ctx.states())
    yield from carry_to(ctx, c + np.array([0.0, 0.0, ZP]))
    yield from release_clamp(ctx)


def clamp_off(ctx):
    """Left hand: squeeze the clamp open on the joint, lift it straight off, lower it onto its stand and let go."""
    yield from take_clamp(ctx)
    yield from lift_clear(ctx)
    yield from carry_to(ctx, _rest_pivot)
    yield from release_clamp(ctx)


def tilt_hold(ctx, deg):
    """Left hand: grip the left pipe and turn the pipes (and the clamp gripping them) ``deg`` about their axis, in
    their V-blocks (the blocks' grip helps hold the clamp's weight), and keep holding. Returns what tilt_back needs."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import grip_rotation

    L, yl = ctx.left, YAW_HOME["left"]
    p = kit.point(ctx.states(), "pipe_fixed", HOLD_LOCAL)
    g0 = grip_rotation(yl)
    c = np.array([float(p[0]), float(p[1]), float(p[2]) + 0.002])  # TCP on the pipe's axis
    yield from L.move(tuple(c + np.array([0, 0, 0.05])), yaw=yl, finger=FINGER_OPEN, duration=1.0)
    yield from L.move(tuple(c), yaw=yl, duration=0.6)
    yield from L.grip(FINGER_CLOSED, duration=0.5)
    off = np.zeros(3)
    for s in (0.25, 0.5, 0.75, 1.0):
        off = yield from _turn_on_axis(ctx, c, off, _ry(deg * s) @ g0)
    st = ctx.states()
    print(f"    tilted: mark 1 at {mark_angle(st):+.1f}, clamp at {clamp_turn(st):+.1f} deg, axis off "
          f"{np.round(_axis_miss(st) * 1000, 2)} mm", flush=True)
    return c, g0, deg, off


def _axis_miss(objects):
    """Where the fixed pipe's axis should be (held level in its V-blocks) minus where it is, at the hand."""
    a = kit.point(objects, "pipe_fixed", HOLD_LOCAL)
    return np.array([AX_X - a[0], 0.0, AX_Z - a[2]])


def _turn_on_axis(ctx, c, off, rot, duration=0.5):
    """One step of a turn about the pipes' axis: TCP at c + off, then nudge off so the gripped pipe's axis stays where
    its V-blocks hold it (the soft arm, turning, lifted it out of line)."""
    L = ctx.left
    yield from move_rot(L, c + off, rot, duration=duration)
    for _ in range(2):
        yield from L.wait(0.15)
        off = np.clip(off + 0.8 * _axis_miss(ctx.states()), -0.015, 0.015)
        yield from move_rot(L, c + off, rot, duration=0.2)
    return off


def tilt_back(ctx, held):
    """Left hand, still holding the pipe from tilt_hold: turn it back upright in its V-blocks and let go."""
    from yam_arena.constants import FINGER_OPEN

    L, yl = ctx.left, YAW_HOME["left"]
    c, g0, deg, off = held
    for s in (0.75, 0.5, 0.25, 0.0):
        off = yield from _turn_on_axis(ctx, c, off, _ry(deg * s) @ g0)
    yield from L.grip(FINGER_OPEN, duration=0.4)
    yield from L.move((float(c[0]), float(c[1]), HIGH), yaw=yl, duration=0.8)
    st = ctx.states()
    print(f"    tilted back: mark 1 at {mark_angle(st):+.1f}, clamp at {clamp_turn(st):+.1f} deg", flush=True)


def _ry(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def move_rot(arm, pos, rot, finger=None, duration=1.0):
    """Like Arm.move, but to any TCP orientation ``rot`` (3x3): position straight, orientation slerped, IK each step."""
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.ik import matrix_to_quat_xyzw

    p0, r0 = ik.tcp_pose(arm.q, arm.base)
    q0, q1 = np.asarray(matrix_to_quat_xyzw(r0)), np.asarray(matrix_to_quat_xyzw(rot))
    if q0 @ q1 < 0:
        q1 = -q1
    f0 = arm.finger
    f1 = f0 if finger is None else finger
    n = max(1, round(duration * 20))
    for k in range(1, n + 1):
        s = k / n
        om = math.acos(min(1.0, abs(float(q0 @ q1))))
        qs = q0 if om < 1e-6 else (math.sin((1 - s) * om) * q0 + math.sin(s * om) * q1) / math.sin(om)
        p = p0 + s * (np.asarray(pos) - p0)
        qj, _, _ = ik.solve(p, qs / np.linalg.norm(qs), arm.q, base_pos=arm.base, iters=60)
        arm.q = qj
        f = f0 + s * (f1 - f0)
        arm.cmd = np.r_[qj, float(np.clip(f / FINGER_OPEN, 0.0, 1.0))]
        yield
    arm.finger = f1


TURN_STEP = 45.0  # deg per twist of the wrist (the left arm reaches -60..+45 deg here)
TURN_LIFT = 0.03  # the gripped end is lifted this much first, so the tilted fingers clear the table


def unwelded_target(objects):
    """The pipe's turn (mark-1 angle, deg) that puts the middle of the biggest run of unwelded segments on top."""
    w = welded()
    todo = [k for k in range(N_SEG) if k not in w]
    if not todo:
        return None
    runs, run = [], []
    for k in range(2 * N_SEG):  # runs of consecutive unwelded segments, round the circle
        if (k % N_SEG) in todo:
            run.append(k)
        else:
            if run:
                runs.append(run)
            run = []
    best = max(runs, key=len) if runs else [todo[0]]
    best = best[:N_SEG]
    centre = (best[0] + best[-1] + 1) / 2 * math.degrees(SEG)  # pipe-frame angle of the run's middle
    return (-centre + 180.0) % 360.0 - 180.0


def turn_pipe(ctx, tol_deg=12.0, target=None):
    """Left hand grips the joined pipes directly and turns them until the biggest unwelded run is on top: it grips the
    left pipe near its outer end, lifts that end 3 cm (the far end stays on the table), twists its wrist about the
    pipes' axis by up to 45 deg, lowers it back onto the table, lets go, and grips again for the next twist."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import grip_rotation

    L, yl = ctx.left, YAW_HOME["left"]
    if target is None:
        target = unwelded_target(ctx.states())
    if target is None:
        return
    for _ in range(6):
        st = ctx.states()
        rem = (target - mark_angle(st) + 180.0) % 360.0 - 180.0
        if abs(rem) < tol_deg:
            break
        step = max(-TURN_STEP, min(TURN_STEP, rem))
        p = kit.point(st, "pipe_fixed", HOLD_LOCAL)
        g0 = grip_rotation(yl)
        z = float(p[2]) + 0.002  # TCP on the axis (fingertips just over the table)
        yield from L.move((float(p[0]), float(p[1]), z + 0.05), yaw=yl, finger=FINGER_OPEN, duration=1.0)
        yield from L.move((float(p[0]), float(p[1]), z), yaw=yl, duration=0.6)
        yield from L.grip(FINGER_CLOSED, duration=0.5)
        yield from L.move((float(p[0]), float(p[1]), z + TURN_LIFT), yaw=yl, duration=0.6)
        c = np.array([float(p[0]), float(p[1]), z + TURN_LIFT])
        for s in (0.5, 1.0):  # twist about the world y axis through the TCP (the pipes' axis)
            yield from move_rot(L, c, _ry(step * s) @ g0, duration=0.8)
        yield from L.wait(0.3)
        print(f"    turn: twisted {step:+.0f} deg, mark 1 now at {mark_angle(ctx.states()):+.1f} (target {target:+.0f})",
              flush=True)
        yield from move_rot(L, c - np.array([0, 0, TURN_LIFT - 0.004]), _ry(step) @ g0, duration=0.6)  # back down
        yield from move_rot(L, c - np.array([0, 0, TURN_LIFT - 0.004]), _ry(step) @ g0, finger=FINGER_OPEN,
                            duration=0.4)
        back = _ry(step) @ np.array([0.0, 0.0, 0.05])  # out along the tilted approach
        yield from move_rot(L, c + back, _ry(step) @ g0, duration=0.5)
        yield from L.move((float(c[0]), float(c[1]), HIGH), yaw=yl, duration=0.8)
    _log(ctx, f"turned (mark 1 at {mark_angle(ctx.states()):+.1f} deg, target {target:+.0f})")


# -- the gun -----------------------------------------------------------------------------------------------------------
def take_gun(ctx):
    """Right hand takes the gun off its rest (fingers across the handle) and lifts it. Returns how it hangs in the
    hand: (rotation, TCP minus handle centre, wire-tip correction), the correction measured over the joint."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    R, yr = ctx.right, YAW_HOME["right"]
    h = kit.point(ctx.states(), "gun", (0.0, 0.0, 0.0))
    yield from R.move((h[0], h[1], h[2] + 0.06), yaw=yr, finger=FINGER_OPEN, duration=1.2)
    yield from R.move((h[0], h[1], h[2]), yaw=yr, duration=0.8)
    yield from R.wait(0.2)
    yield from R.grip(FINGER_CLOSED)
    yield from R.move((h[0], h[1], HIGH), yaw=yr, duration=0.8)
    yield from R.wait(0.4)
    g_pos, g_quat = kit.pose(ctx.states(), "gun")
    rg = mk.quat_to_mat(g_quat)
    hold = np.asarray(mk.tcps()[1]) - np.asarray(g_pos)
    gun = {"rg": rg, "hold": hold, "err": np.zeros(3)}
    # beside the joint (the clamp's head is over it), the wire tip 2 cm over the loose pipe: measure how far off it is
    # there, to correct every move after
    beside = tip_at(ctx, 0.0, PIPE_R + 0.02) + np.array([0.0, -DIP_Y, 0.0])
    yield from R.move(tuple(hand_for(gun, beside + np.array([0, 0, 0.03]))), yaw=yr, duration=1.5)
    yield from R.move(tuple(hand_for(gun, beside)), yaw=yr, duration=0.6)
    yield from R.wait(0.4)
    gun["err"] = beside - wire_tip(ctx.states())  # the gun sags and turns a little in the fingers
    print(f"    gun: wire tip off by {np.round(gun['err'] * 1000, 1)} mm beside the joint", flush=True)
    return gun


def tip_at(ctx, deg, r):
    """World point at radius r from the pipes' axis in the joint's plane, ``deg`` from straight up (+x positive)."""
    c, _ = joint_frame(ctx.states())
    a = math.radians(deg)
    return c + np.array([r * math.sin(a), 0.0, r * math.cos(a)])


def hand_for(gun, tip):
    """TCP that puts the wire tip at ``tip``, with the gun as it hangs in the hand."""
    return tip - gun["rg"] @ GUN_TIP + gun["hold"] + gun["err"]


def exposed(objects, deg) -> bool:
    """Whether the joint at ``deg`` (from straight up, +x positive) is open to the gun: anywhere with the clamp off;
    with it on, not under their bands' unslotted tops."""
    if not gripping():
        return True
    a = (deg - clamp_turn(objects) + 180.0) % 360.0 - 180.0
    return abs(a) >= COVERED


DIP_Y = 0.05  # the gun moves between stretches of joint this far to the side (-y), clear of the clamp's bands


LIFT_R = 0.009  # over a third that already has its heat, the tip runs this far off the pipe (out of reach)


def _pass_move(ctx, gun, d0, d):
    """Where the tip goes for the move from ``d0`` to ``d`` deg, and how long it takes, by the heat the thirds it
    crosses (sampled near its start, middle and end) already have: slow (0.65 s per 5 deg, ~1.3 s per third) over a
    fresh third, quicker over a partly heated one, lifted clear and quick when all are done (so no segment goes past
    WELD_MAX_S)."""
    st = ctx.states()
    need = []
    for a in (d0 + 0.2 * (d - d0), (d0 + d) / 2, d - 0.2 * (d - d0)):
        t = third_at(st, tip_at(ctx, a, PIPE_R + 0.001))
        k = None if t is None else t // 3
        if t is not None and k not in welded() and heat()[t] < WELD_THIRD_S and heat()[3 * k:3 * k + 3].sum() < 4.3:
            need.append(float(heat()[t]))
    if not need:
        return tip_at(ctx, d, PIPE_R + LIFT_R), 0.25
    return tip_at(ctx, d, PIPE_R + 0.001), (0.65 if min(need) < 0.5 else 0.45)


def sweep(ctx, gun, span_deg, step_deg=5.0):
    """A pass from span_deg[0] to span_deg[1] along the joint, in stretches the clamp leaves open: for each, the wire
    tip comes in from the side (-y) over the start, goes down to the joint, runs along it and goes back out."""
    R, yr = ctx.right, YAW_HOME["right"]
    st = ctx.states()
    degs = [float(d) for d in np.arange(span_deg[0], span_deg[1] + 1e-6, step_deg)]
    runs, run = [], []
    for d in degs:
        if exposed(st, d):
            run.append(d)
        elif run:
            runs.append(run)
            run = []
    if run:
        runs.append(run)
    side = np.array([0.0, -DIP_Y, 0.0])
    for run in runs:
        # closed loop: the soft arm's error depends on its pose, so after each move the wire tip's measured miss is
        # fed back into the next (corr, capped at 25 mm)
        corr = np.zeros(3)

        def fix(target):
            nonlocal corr
            corr = np.clip(corr + 0.8 * (target - wire_tip(ctx.states())), -0.025, 0.025)

        yield from R.move(tuple(hand_for(gun, tip_at(ctx, run[0], PIPE_R + 0.02) + side)), yaw=yr, duration=1.0)
        target = tip_at(ctx, run[0], PIPE_R + LIFT_R)
        for _ in range(3):
            yield from R.move(tuple(hand_for(gun, target) + corr), yaw=yr, duration=0.5)
            fix(target)
        low = False
        for d0, d in zip(run, run[1:]):
            tip, dur = _pass_move(ctx, gun, d0, d)
            lifted = dur <= 0.25 + 1e-9
            if lifted and low:  # off the joint straight away, before moving on over stretches already done
                up = tip_at(ctx, d0, PIPE_R + LIFT_R)
                yield from R.move(tuple(hand_for(gun, up) + corr), yaw=yr, duration=0.15)
            yield from R.move(tuple(hand_for(gun, tip) + corr), yaw=yr, duration=dur)
            fix(tip)
            low = not lifted
            if os.environ.get("PRW_DEBUG"):
                st = ctx.states()
                c, r = joint_frame(st)
                v = r.T @ (wire_tip(st) - c)
                print(f"      tip at {d:+.0f}: along {v[1] * 1000:+.1f} mm, radial "
                      f"{(math.hypot(v[0], v[2]) - PIPE_R) * 1000:+.1f} mm, asked {np.round((tip - wire_tip(st)) * 1000, 1)} "
                      f"mm off, gap {pipe_gap(st) * 1000:.1f}", flush=True)
                if abs(d - 52) < 1e-6 and "dumped" not in mk.MEMORY:
                    import pickle
                    mk.MEMORY["dumped"] = True
                    pickle.dump({k: kit.pose(st, k) for k in ("gun", "clamp", "lever", "pipe_fixed", "pipe_loose")}
                                | {"tcps": [np.asarray(t) for t in mk.tcps()]},
                                open(os.environ["PRW_DEBUG"], "wb"))
        yield from R.move(tuple(hand_for(gun, tip_at(ctx, run[-1], PIPE_R + 0.02)) + corr), yaw=yr, duration=0.5)
        yield from R.move(tuple(hand_for(gun, tip_at(ctx, run[-1], PIPE_R + 0.02) + side) + corr), yaw=yr, duration=0.6)
    h = heat()
    print(f"    pass {span_deg}: stretches {[(r[0], r[-1]) for r in runs]}, welded {sorted(welded())}, burned "
          f"{sorted(burned())}, segment heat {[round(float(h[3 * k:3 * k + 3].sum()), 1) for k in range(N_SEG)]}",
          flush=True)


def return_gun(ctx, gun):
    from yam_arena.constants import FINGER_OPEN

    R, yr = ctx.right, YAW_HOME["right"]
    rest = np.array([GUN_REST[0], GUN_REST[1], GUN_Z]) + gun["hold"]
    yield from R.move((rest[0], rest[1], HIGH), yaw=yr, duration=1.5)
    yield from R.move((rest[0], rest[1], rest[2] + 0.004), yaw=yr, duration=1.0)
    yield from R.grip(FINGER_OPEN, duration=0.5)
    yield from R.move((rest[0], rest[1], HIGH), yaw=yr, duration=0.6)
    yield from R.move((0.12, -0.25, HIGH), yaw=yr, duration=0.8)


if __name__ == "__main__":  # offline check: the kit builds and the starting poses are sane
    for o in OBJECTS:
        lo = np.array(o.pos) - np.array(o.size) / 2
        print(f"{o.name:12s} free={o.free!s:5s} centre=({o.pos[0]:.3f}, {o.pos[1]:.3f}, {o.pos[2] - TOP:.3f})"
              f" size=({o.size[0]:.3f}, {o.size[1]:.3f}, {o.size[2]:.3f}) bottom={lo[2] - TOP:+.4f}")
    print(f"pipe axis {(AX_Z - TOP) * 100:.2f} cm up; wire tip at weld: handle at "
          f"({AX_X:.3f}, {JOINT_Y - GUN_TIP[1]:.3f}, {(AX_Z + PIPE_R + 0.010 - GUN_TIP[2] - TOP) * 100:.1f} cm)")
