"""A 165 mm Japanese kitchen knife (a santoku with a round "wa" handle) lies on an end-grain cutting board next to a
1000-grit whetstone held in a non-slip rubber holder. Sharpening it the trained way means holding the blade at 15 to
20 degrees to the stone and sweeping it across edge first so the stone runs from heel to tip, working each face until
a burr runs the whole edge, then laying the knife back down flat on the board with its edge facing away.
"""

import importlib.abc
import importlib.util
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled knife, stone and board; nothing on disk changes)
import guardkit as gk  # noqa: E402  (the gripper's own shape, for holding and letting go of the knife)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Sharpen the knife on the whetstone, then put it back on the chopping block."


def _dirs(deg):
    """(u, v) for a heading: v along the knife from handle to tip, u across it toward the edge (both horizontal)."""
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0])


# --- The knife, in its own frame: lying flat, handle axis along +y (handle at -y, tip at +y), edge toward +x,
# origin on the handle axis at the bolster (where the blade meets the handle).
HANDLE_Y = (-0.095, 0.0)
HANDLE_W, HANDLE_T = 0.020, 0.018  # across (in the blade's plane) and thickness of the octagonal handle
BLADE_Y = (0.0, 0.165)  # 165 mm blade
SPINE_X, EDGE_X = -0.005, 0.040  # at the heel the blade is 45 mm tall; its straight edge 40 mm from the handle axis
SPINE_CURVE = 0.07  # m past the bolster: the spine runs straight to here, then curves down to the tip (santoku shape)
TIP_RISE = (0.135, 0.006)  # the edge is straight to 13.5 cm past the bolster, then rises 6 mm over the last 3 cm
BEVEL_W = 0.006  # the edge bevel's width (the lighter strip along the edge)
BLADE_T = 0.003  # collider thickness (the steel is 2 mm at the spine)
EDGE_KNOTS = np.linspace(TIP_RISE[0], BLADE_Y[1], 5)  # the rising edge as 4 straight pieces (each its own collider)
EDGE_KNOTS_X = EDGE_X - TIP_RISE[1] * ((EDGE_KNOTS - TIP_RISE[0]) / (BLADE_Y[1] - TIP_RISE[0])) ** 2


def edge_x(y):
    """The edge line in the knife's frame: its distance from the handle axis `y` m past the bolster."""
    return np.interp(y, np.r_[0.0, EDGE_KNOTS], np.r_[EDGE_X, EDGE_KNOTS_X])


def spine_x(y):
    """The spine line: straight to SPINE_CURVE, then curving down to meet the edge at the tip."""
    t = np.clip((np.asarray(y, float) - SPINE_CURVE) / (BLADE_Y[1] - SPINE_CURVE), 0.0, 1.0)
    return SPINE_X + (EDGE_KNOTS_X[-1] - SPINE_X) * (1.0 - np.sqrt(1.0 - t ** 1.6))


GRIP_Y = -0.045  # where the demo holds the handle: 4.5 cm behind the bolster, the fingertips 3.2 cm short of the blade,
#                  as close to the blade as the hand reaches: its fingers are 7 cm long, and closer in the palm would
#                  meet the end of the 9.5 cm handle (2 cm clear here). The closer the grip, the shorter the lever the
#                  stone's drag on the blade has to turn the knife in the hand
KNIFE_MASS = 0.14  # kg: a 165 mm santoku with a magnolia wa handle, 46 mm tall at the heel, is listed at 140 g
#                   (Sakai Takayuki VG5 hammered santoku 165 mm, Seisuke Knife product page)
EDGE_SAMPLES = np.linspace(0.005, 0.162, 16)  # points along the edge the scorer checks: about 1 cm apart, from 0.5 cm
#                                                past the heel to 16.2 cm, 3 mm short of the point, where the edge
#                                                curves up to meet the spine (the last one 4.9 mm up the tip's rise)

# --- The board and the knife's starting place: the knife lies flat on the board, edge away from the robot, turned
# 15 degrees, its handle over the board's edge so a hand can close round it.
PARK_HEADING = 15.0
PARK_TCP = np.array([0.20, -0.05])  # the handle's grip point at the start
BOARD = (0.305, 0.305, 0.0762)  # John Boos CCB121203 end-grain maple chopping block, 12 x 12 x 3 in: across x along x
#                                  height (johnboos.com, CCB series)
BOARD_OFF = -0.09  # m across the knife (toward the spine) from the grip line to the block's middle: the 30.5 cm block
#                   sits mostly on the robot's side of the knife, so its far edge stays 1.7 cm clear of the stone's holder
P_U, P_V = _dirs(PARK_HEADING)
BOARD_EDGE = -0.015  # m past the bolster: the board's near edge is under the handle 1.5 cm behind the bolster, so the
#                     rest of the handle reaches past it and the lower finger closes round it 1.7 cm clear of the board
BOARD_POS = PARK_TCP + P_V[:2] * (BOARD_EDGE - GRIP_Y + BOARD[1] / 2) + P_U[:2] * BOARD_OFF
KNIFE_START = np.r_[PARK_TCP - GRIP_Y * P_V[:2], TOP + BOARD[2] + BLADE_T / 2]

# --- The stone, fixed in its holder; its long side along the stroke (u at STONE_HEADING).
STONE_HEADING = -15.0
STONE = (0.185, 0.063, 0.025)  # King KW65 1000/6000 combination waterstone, 185 x 63 x 25 mm (Japanwelt, My Good Knife)
RISER = (0.253, 0.080, 0.053)  # Shapton glass-stone field holder base, 253 x 80 x 53 mm (MTC Kitchen, Hodge Products):
#                               the stand the holder sits on, so the YAM's wrist clears the bench
HOLDER = (0.288, 0.076, 0.036)  # Naniwa A-902 whetstone holder, 288 x 76 x 36 mm overall (Sharpening Supplies)
HOLDER_H = 0.032  # it holds the stone's underside 32 mm (1-1/4 in) above what it stands on
STONE_TOP = RISER[2] + HOLDER_H + STONE[2]  # 11 cm above the bench
S_U, S_V = _dirs(STONE_HEADING)
STROKE_TCP_Y = -0.20  # where the hand works: the stone sits under the blade's middle
STONE_V = 0.15  # m from the grip point (the stroke's reference) to the stone's middle along the knife
STONE_SHIFT = 0.04  # m along the stone, away from the robot: the 28.8 cm holder's near clamp then ends where the
#                    hand reaches for the knife's handle no closer than before, so it does not block the grip
STONE_POS = np.array([0.355, STROKE_TCP_Y]) + STONE_V * S_V[:2] + STONE_SHIFT * S_U[:2]

# --- Scoring
BAND = (13.0, 22.0)  # deg: the 15-20 degree sharpening angle, plus 2 degrees each way for a hand-held angle
STEEP = 22.0  # deg: steeper than this the stroke works a steeper, more obtuse micro-bevel instead of the 15-20 degree
#               edge: it is not counted (a real trial sees it on the angle card)
TOUCH = 0.0005  # m: contact: the lowest edge point over the stone within 0.5 mm of its top. The knife's collider
#                 rests on the stone's (rest offset 0), so a knife really on the stone reads 0 give or take the
#                 solver's jitter; a knife held even 1 mm up does not touch
CONTACT = 0.001  # m: with the knife touching, an edge point within 1 mm of the stone's top counts as on it (that part
#                  of the edge bears on the stone; 1 mm covers the jitter and the blade's flex under a light press)
EDGE_ON = 3  # edge points (about 1 cm apart, each standing for about 1 cm of edge) on the stone for a stroke to count:
#               3 cm of edge, half the stone's 6 cm width; a tip or heel dragged alone sharpens nothing
COVER = 12  # of the 16 edge points (each about 1 cm of the 16.5 cm edge): full credit needs each face to have worked
#             at least 12 (three quarters of the edge, "most of it"), so strokes on the same few cm do not count as a
#             sharpened knife
COVER_TRAVEL = 0.10  # m: a point counts as worked once it has been on the stone for 10 cm of in-band, edge-leading
#                      travel on that face (a floor, like PER_SIDE)
LEVEL = 5.0  # deg: the blade's length (handle to tip) within this of level while stroking; a whetstone stroke keeps
#               the whole edge on the stone, and a pitch above 5 deg lifts the far end of a 6 cm contact 5 mm off it
STROKE = 0.05  # m: edge-leading travel across the edge in one pass, on the stone and in band, that makes a stroke
#                (the demo's strokes run 11 cm along the stone)
REVERSE = 0.005  # m: moving back this far against a pass ends it (a back-and-forth stroke's turn); less is jitter
STEEP_TRAVEL = 0.02  # m: a touch with 2 cm of travel steeper than STEEP is a too-steep stroke: none of it counts
PER_SIDE = 20  # strokes on each face, from the one published count we found measured on the edge itself: Verhoeven
#               ("Experiments on Knife Sharpening", 2004, section 4) hand-sharpened HRC 60 blades at very
#               light pressure on 6000 and 8000 grit waterstones and examined the edges in an electron microscope:
#               10 four-stroke cycles (20 strokes per face) removed every remnant of the old edge, 4 cycles (8 per
#               face) did not reliably. The guides give no fixed count (a face is worked until a burr runs heel to
#               tip). The sim cannot feel a burr; the real trial's edge tester decides
OFF_END = 0.003  # m: an edge point this far below the stone's top just past an end, within 0.5 s of the edge leaving
#                  the stone: the stroke rolled off the end instead of being lifted, and it is not counted
OFF_END_STEPS = 10  # control steps (0.5 s at 20 Hz) after lift-off in which a roll-off cancels the stroke just ended
# Held: a gripper with both fingers touching the handle (gk.gripping: the gripper's own shape, palm and fingers at
# their present opening, within 2 mm of it, the solver's resting gap plus the fit of its boxes to the meshes) and
# their opening stopped by it: from its 1.8 cm thickness less 15 % of full opening to its 2.0 cm width plus 3 %
# (0.10 to 0.24; 0 closed, 1 open). An empty fist, an open hand round it or a hand merely near it does not hold it.
HANDLE_GRIP = (gk.band(HANDLE_T)[0], gk.band(HANDLE_W)[1])
FLAT = 20.0  # deg: lying flat on a face (not on its spine or edge)
EDGE_AWAY = 0.5  # the edge's horizontal direction has at least this much +x (facing away from the robot, < 60 deg)
GRIPPERS = ("left_gripper", "right_gripper")

STEEL = kit.Look((0.58, 0.60, 0.63), 0.35, metallic=0.85)  # satin-finished blade
BEVEL = kit.Look((0.95, 0.96, 0.97), 0.2, metallic=0.3)  # the freshly ground edge bevel: a bright strip
WOOD = kit.Look((0.55, 0.36, 0.20), 0.6)  # magnolia handle
HORN = kit.Look((0.10, 0.09, 0.09), 0.4)  # buffalo-horn ferrule
STONE_LOOK = kit.Look((0.85, 0.45, 0.22), 0.9)  # orange 1000-grit stone
RUBBER = kit.Look((0.08, 0.08, 0.09), 0.9)
MAPLE = kit.Look((0.86, 0.72, 0.52), 0.7)
MAPLE_DARK = kit.Look((0.74, 0.58, 0.40), 0.7)


def _strip(rows, look):
    """A closed strip along the blade from (y, x0, x1, half-thickness at x0, at x1) cross-sections."""
    pts, counts, idx = [], [], []
    for y, x0, x1, t0, t1 in rows:
        pts += [(x0, y, t0), (x1, y, t1), (x1, y, -t1), (x0, y, -t0)]
    for i in range(len(rows) - 1):
        a, b = 4 * i, 4 * (i + 1)
        for e0, e1 in ((0, 1), (1, 2), (2, 3), (3, 0)):
            counts.append(4)
            idx += [a + e0, a + e1, b + e1, b + e0]
    for end, ring in ((0, (3, 2, 1, 0)), (len(rows) - 1, (0, 1, 2, 3))):
        counts.append(4)
        idx += [4 * end + k for k in ring]
    return kit._mesh(pts, counts, idx, look)


def _blade_colliders():
    """Boxes following the outline: three from the heel to where the edge starts to rise, each from the spine (at its
    middle) to the edge; one turned box per straight piece of the rising edge, 8 mm deep; two filling the tip."""
    out = []
    for y0, y1 in ((0.0, SPINE_CURVE), (SPINE_CURVE, 0.105), (0.105, TIP_RISE[0])):
        x0 = float(spine_x((y0 + y1) / 2))
        out.append(kit.Collider(((x0 + EDGE_X) / 2, (y0 + y1) / 2, 0.0), size=(EDGE_X - x0, y1 - y0, BLADE_T)))
    deep = 0.008
    for i in range(len(EDGE_KNOTS) - 1):
        (y0, y1), (x0, x1) = EDGE_KNOTS[i:i + 2], EDGE_KNOTS_X[i:i + 2]
        n = math.hypot(x1 - x0, y1 - y0)
        inward = np.array([-(y1 - y0), x1 - x0]) / n  # (x, y) unit normal pointing from the edge toward the spine
        c = np.array([(x0 + x1) / 2, (y0 + y1) / 2]) + inward * deep / 2
        out.append(kit.Collider((float(c[0]), float(c[1]), 0.0), size=(deep, n, BLADE_T),
                                rot_z=-math.degrees(math.atan2(x1 - x0, y1 - y0))))
    for y0, y1 in ((TIP_RISE[0], 0.15), (0.15, 0.16)):
        x0, x1 = float(spine_x((y0 + y1) / 2)), float(edge_x(y1)) - deep
        out.append(kit.Collider(((x0 + x1) / 2, (y0 + y1) / 2, 0.0), size=(x1 - x0, y1 - y0, BLADE_T)))
    return tuple(out)


def _knife():
    h0, h1 = HANDLE_Y
    ys = np.unique(np.r_[np.linspace(0.0, BLADE_Y[1], 34), EDGE_KNOTS, BLADE_Y[1] - np.geomspace(0.0005, 0.02, 8)])
    sp, ed = spine_x(ys), edge_x(ys)
    inner = np.maximum(ed - BEVEL_W, sp)  # where the bevel starts (it takes the whole tip where the blade narrows)
    meshes = (
        kit.cylinder((0.0, (h0 + h1 - 0.015) / 2, 0.0), HANDLE_T / 2, h1 - h0 - 0.015, WOOD, axis="y", n=8),
        kit.cylinder((0.0, -0.0075, 0.0), HANDLE_T / 2 + 0.0005, 0.015, HORN, axis="y", n=8),
        _strip([(y, a, b, 0.001, 0.001) for y, a, b in zip(ys, sp, inner)], STEEL),  # 2 mm flat steel
        _strip([(y, a, b, 0.001, 0.00015) for y, a, b in zip(ys, inner, ed)], BEVEL),  # ground to 0.3 mm at the edge
    )
    colliders = (kit.Collider((0.0, (h0 + h1) / 2, 0.0), size=(HANDLE_W, h1 - h0, HANDLE_T)),) + _blade_colliders()
    return kit.Prop("knife", pos=tuple(float(c) for c in KNIFE_START), size=(0.05, 0.26, HANDLE_T), mass=KNIFE_MASS,
                    yaw=math.radians(PARK_HEADING), meshes=meshes, colliders=colliders, com=(0.012, 0.02, 0.0),
                    report=True)  # PhysX contact reports on the knife: the scorer reads the stone's push on it


HANDLE_SHAPE = gk.Block((0.0, sum(HANDLE_Y) / 2, 0.0), (HANDLE_W, HANDLE_Y[1] - HANDLE_Y[0], HANDLE_T))


def _knife_shape():
    """The knife's colliders as guardkit boxes in its frame; each turned box along the tip's rise is replaced by the box
    that bounds it (at most 2.2 mm longer and wider), which only makes the let-go check a little stricter there."""
    out = [HANDLE_SHAPE]
    for c in _blade_colliders():
        a = math.radians(c.rot_z)
        w = abs(math.cos(a)) * c.size[0] + abs(math.sin(a)) * c.size[1]
        h = abs(math.sin(a)) * c.size[0] + abs(math.cos(a)) * c.size[1]
        out.append(gk.Block(c.center, (w, h, c.size[2])))
    return tuple(out)


KNIFE_SHAPE = _knife_shape()


def _stone():
    hx, hy, lip = HOLDER  # the holder's end clamps reach its full 36 mm up (2.1 cm below the stone's top)
    r = RISER[2]
    parts = [  # (centre, size, look)
        ((0, 0, r / 2), RISER, WOOD),
        ((0, 0, r + HOLDER_H / 2), (hx - 0.024, hy, HOLDER_H), RUBBER),
        ((0, 0, r + HOLDER_H + STONE[2] / 2), STONE, STONE_LOOK),
        (((hx - 0.012) / 2, 0, r + lip / 2), (0.012, hy, lip), RUBBER),
        ((-(hx - 0.012) / 2, 0, r + lip / 2), (0.012, hy, lip), RUBBER),
    ]
    meshes = tuple(kit.box(c, sz, lk) for c, sz, lk in parts)
    colliders = tuple(kit.Collider(c, size=sz) for c, sz, _ in parts)
    return kit.Prop("stone", pos=(float(STONE_POS[0]), float(STONE_POS[1]), TOP), size=(hx, hy, STONE_TOP),
                    yaw=math.radians(STONE_HEADING), meshes=meshes, colliders=colliders)


def _board():
    meshes = (kit.box((0, 0, BOARD[2] / 2), BOARD, MAPLE),
              kit.box((0, 0, BOARD[2] + 0.0002), (BOARD[0] - 0.02, BOARD[1] - 0.02, 0.0004), MAPLE_DARK))
    return kit.Prop("board", pos=(float(BOARD_POS[0]), float(BOARD_POS[1]), TOP), size=BOARD, yaw=math.radians(PARK_HEADING),
                    meshes=meshes, colliders=(kit.Collider((0, 0, BOARD[2] / 2), size=BOARD),))


OBJECTS = [_board(), _stone(), _knife()]


# --- Geometry from the knife's pose.
def _rot(q):
    x, y, z, w = (float(c) for c in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _pose(objects):
    k = objects["knife"]
    return np.asarray(k.pos, float), _rot(k.quat_xyzw)


def blade_angle(objects):
    """Degrees between the blade's faces and the horizontal stone top (0 = lying flat, 90 = on its edge)."""
    _, r = _pose(objects)
    return math.degrees(math.acos(min(1.0, abs(r[2, 2]))))


def sharpen_angle(objects):
    """The sharpening angle: degrees the blade's width (spine to edge, knife +x) tilts below the level stone top,
    edge down. Measured across the blade, so pitching the knife tip-down about its width does not count as angle."""
    _, r = _pose(objects)
    return math.degrees(math.asin(max(-1.0, min(1.0, -r[2, 0]))))


def level(objects):
    """The blade's length axis (knife +y, handle to tip) within LEVEL degrees of horizontal."""
    _, r = _pose(objects)
    return abs(r[2, 1]) < math.sin(math.radians(LEVEL))


def side(objects):
    """Which face is down: 'A' (the face that lies on the board at the start) or 'B'."""
    _, r = _pose(objects)
    return "A" if r[2, 2] > 0 else "B"


def _edge_points(objects):
    """World points along the edge's lower corner (both corners of the collider; the lower one is kept per point)."""
    p, r = _pose(objects)
    pts = []
    for y in EDGE_SAMPLES:
        a = p + r @ np.array([edge_x(y), y, BLADE_T / 2])
        b = p + r @ np.array([edge_x(y), y, -BLADE_T / 2])
        pts.append(a if a[2] < b[2] else b)
    return np.array(pts)


def _stone_local(pts):
    """(along the stone's length, across it, height above its top) for world points."""
    d = pts[:, :2] - STONE_POS
    return d @ S_U[:2], d @ S_V[:2], pts[:, 2] - (TOP + STONE_TOP)


def on_stone(objects):
    """The edge touching the stone: its lowest point over the stone's top within TOUCH of it (0.5 mm: resting on it,
    not hovering), with the edge lower than the spine."""
    _, r = _pose(objects)
    if r[2, 0] >= 0:  # the edge (knife +x) is not the low side: the spine or the face is what touches
        return False
    a, c, h = _stone_local(_edge_points(objects))
    over = (np.abs(a) <= STONE[0] / 2) & (np.abs(c) <= STONE[1] / 2)
    return bool(np.any(over)) and float(h[over].min()) <= TOUCH


def edge_mask(objects):
    """Which EDGE_SAMPLES points are over the stone's top and within CONTACT of it."""
    a, c, h = _stone_local(_edge_points(objects))
    return (np.abs(a) <= STONE[0] / 2) & (np.abs(c) <= STONE[1] / 2) & (h <= CONTACT)


def edge_on_stone(objects):
    """How many of the EDGE_SAMPLES points are over the stone's top and within CONTACT of it."""
    return int(np.sum(edge_mask(objects)))


def _off_end(objects):
    """An edge point just past an end of the stone (within 4 cm), across its width, below its top by OFF_END."""
    a, c, h = _stone_local(_edge_points(objects))
    past = (np.abs(a) > STONE[0] / 2) & (np.abs(a) < STONE[0] / 2 + 0.04) & (np.abs(c) <= STONE[1] / 2)
    return bool(np.any(past & (h < -OFF_END)))


def _edge_dir(objects):
    """The horizontal unit direction the edge faces (knife +x projected on the table)."""
    _, r = _pose(objects)
    e = r[:, 0].copy()
    e[2] = 0.0
    n = np.linalg.norm(e)
    return e / n if n > 1e-6 else e


def held(objects):
    """A gripper with both fingers touching the handle and their opening stopped by it (see HANDLE_GRIP)."""
    return bool(gk.gripping(objects, "knife", HANDLE_SHAPE, HANDLE_GRIP))


def _released(objects):
    """Let go: no part of either gripper touching the knife, handle or blade (gk.touching, within 2 mm)."""
    return not gk.touching(objects, "knife", KNIFE_SHAPE)


# --- Verified contact: the stone's push on the knife, read from the simulator's contact solver. PhysX reports the
# contact force between the knife's body and the stone's (only that pair: the hand and the board are not in the
# view) after every physics substep; the largest in each control step is kept. A stroke needs this force AND the
# geometry above (the edge the low side, resting on the stone's top), so a knife held just above the stone, or
# touching it with its spine or face, does not count.
STONE_PUSH = 0.5  # N: the least push from the stone that counts as touching it. The knife alone weighs 1.4 N, and
#                   the demo presses 2.4 to 3.3 N (median); a knife hovering a fraction of a mm off the stone reads 0
# Kept on the yamkit module, so it is shared if the task file is loaded more than once in a process (the physics
# hook stays the first load's; the step hook is the latest's; both use this one record).
_CONTACT = kit.__dict__.setdefault("_KNIFE_CONTACT", {"view": None, "peak": 0.0, "force": None, "err": None,
                                                     "first": None, "polls": 0})


def stone_force():
    """The largest push (N) from the stone on the knife in any physics substep of the last control step; None when
    no simulator reports it (offline, until a test sets it)."""
    return _CONTACT["force"]


def _touching_stone():
    f = stone_force()
    return f is not None and f >= STONE_PUSH


def _np(x):
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _contact_processing_on():
    """Isaac Lab turns PhysX contact processing off unless a ContactSensor exists (this embodiment builds none);
    switch it back on, as Isaac Lab's own ContactSensor does (and as move-live-wire-with-stick does)."""
    try:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    except Exception:  # pragma: no cover  (outside Isaac Sim)
        pass


def _make_view(mgr):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sv = mgr.get_physics_sim_view()
    if sv is None:
        return None
    knife = stone = None
    for prim in get_current_stage().Traverse():
        p = prim.GetPath().pathString
        if "/env_0/" in p and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            if p.endswith("/knife"):
                knife = p
            elif p.endswith("/stone"):
                stone = p
    if knife is None or stone is None:
        raise RuntimeError(f"knife contact view: knife body {knife}, stone body {stone}")
    _contact_processing_on()
    view = sv.create_rigid_contact_view([knife], filter_patterns=[[stone]], max_contact_data_count=64)
    print(f"[knife] contact view: {knife} <- {stone}", flush=True)
    return view


def _poll(mgr):
    """After every physics substep: the stone's push on the knife this substep; the control step keeps the largest."""
    try:
        if _CONTACT["view"] is None:
            _CONTACT["view"] = _make_view(mgr)
            if _CONTACT["view"] is None:
                return
        fm = _np(_CONTACT["view"].get_contact_force_matrix(dt=mgr.get_physics_dt())).reshape(-1, 3)
    except Exception as e:  # pragma: no cover  (reported once; with no reading no stroke counts, so it shows as 0)
        if _CONTACT["err"] is None:
            _CONTACT["err"] = repr(e)
            print(f"[knife] contact view error: {e!r}", flush=True)
        return
    f = float(np.linalg.norm(fm.sum(0)))
    _CONTACT["polls"] += 1
    _CONTACT["peak"] = max(_CONTACT["peak"], f)
    if f >= STONE_PUSH and _CONTACT["first"] is None:
        _CONTACT["first"] = f
        print(f"[knife] first contact with the stone: {f:.2f} N", flush=True)


def _control_step(base, objects):
    """Once per control step (yamkit hook, before success() and score()): the step's largest stone push."""
    _CONTACT["force"] = _CONTACT["peak"] if _CONTACT["polls"] else None
    _CONTACT["peak"], _CONTACT["polls"] = 0.0, 0
    if os.environ.get("KNIFE_CONTACT_DEBUG"):
        print(f"[knife] stone push {_CONTACT['force']}", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod):
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_knife_hooked", False):
        return
    base_cfg, base_step = mgr._configure_physics.__func__, mgr.step.__func__

    def _configure_physics(cls):
        base_cfg(cls)
        _contact_processing_on()

    def step(cls):
        base_step(cls)
        _poll(cls)

    step._knife_hooked = True
    mgr._configure_physics, mgr.step = classmethod(_configure_physics), classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    """Patches Isaac Lab's PhysxManager when it is first imported (it needs Kit, so not at task import)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


if _PHYSX_MGR in sys.modules:
    _patch_manager(sys.modules[_PHYSX_MGR])
elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
    sys.meta_path.insert(0, _MgrFinder())
kit.STEP_HOOKS[:] = [h for h in kit.STEP_HOOKS if getattr(h, "__name__", "") != "_control_step"] + [_control_step]


def on_board(objects):
    """The knife lying flat on the board: its middle over the board, its lowest edge point within 1 cm of the board's
    top, its faces within FLAT of level."""
    p, r = _pose(objects)
    mid = p + r @ np.array([0.0, 0.05, 0.0])
    d = mid[:2] - BOARD_POS
    over = abs(d @ P_U[:2]) <= BOARD[0] / 2 and abs(d @ P_V[:2]) <= BOARD[1] / 2
    low = _edge_points(objects)[:, 2].min() - (TOP + BOARD[2])
    return over and -0.005 <= low <= 0.01 and blade_angle(objects) <= FLAT


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.strokes = {"A": 0, "B": 0}  # strokes counted per face (kept: not too steep, not rolled off the end)
        self.worked = {"A": np.zeros(len(EDGE_SAMPLES)), "B": np.zeros(len(EDGE_SAMPLES))}  # per face, per edge
        #                                       point: in-band travel across the edge (m) while it was on the stone
        self.run = None  # the touch in progress (see _new_run) or None
        self.ended = None  # the touch that just ended, kept until OFF_END_STEPS show it did not roll off the end
        self.since_contact = 99  # control steps since the edge was last on the stone
        self.last_pos = None


_H = _History()


def _new_run(s):
    """A touch: its face, its steep travel, the edge-leading passes already finished in it (strokes, worked travel) and
    the pass in progress: its direction across the edge (+1 edge leading, -1 edge trailing, 0 not yet moved), its net
    in-band travel that way, the most it has reached, and its net travel per edge point while on the stone."""
    n = len(EDGE_SAMPLES)
    return {"side": s, "steep": 0.0, "strokes": 0, "worked": np.zeros(n),
            "dir": 0, "band": 0.0, "peak": 0.0, "pass": np.zeros(n)}


def _end_pass(run):
    """Close the pass in progress. An edge-leading pass is a stroke if it reached STROKE, and its net travel per point
    (never below 0) is kept; an edge-trailing pass (the light return of a back-and-forth stroke) counts for nothing."""
    if run["dir"] > 0:
        run["strokes"] += int(run["peak"] >= STROKE)
        run["worked"] += np.maximum(run["pass"], 0.0)
    run["dir"], run["band"], run["peak"], run["pass"] = 0, 0.0, 0.0, np.zeros(len(EDGE_SAMPLES))


def _pass_step(run, lead, mask):
    """One in-band step of `lead` m across the edge (+ edge leading). Going back more than REVERSE against the pass
    ends it, and the next pass starts from there the other way, so each edge-leading pass of a back-and-forth stroke
    kept on the stone counts on its own, as each pass with the lifted return does. Net travel is counted, so a knife
    jiggling in place on the stone adds nothing."""
    if run["dir"] == 0:
        run["dir"] = 1 if lead > 0 else -1 if lead < 0 else 0
    d = run["dir"]
    run["band"] += d * lead
    run["pass"][mask] += d * lead
    run["peak"] = max(run["peak"], run["band"])
    if run["band"] < run["peak"] - REVERSE:  # turned back: the pass ended at its peak
        back = run["peak"] - run["band"]
        _end_pass(run)
        run["dir"], run["band"], run["peak"] = -d, back, back


def _run_totals(run):
    """(strokes, worked travel per point) of a touch, the pass in progress included if it is edge leading."""
    if run["dir"] <= 0:
        return run["strokes"], run["worked"].copy()
    return (run["strokes"] + int(run["peak"] >= STROKE),
            run["worked"] + np.maximum(run["pass"], 0.0))


def _keeps(run):
    """A touch that counts: not too steep (under STEEP_TRAVEL of travel steeper than STEEP)."""
    return run is not None and run["steep"] < STEEP_TRAVEL


def _commit(run):
    """Add a finished touch's strokes and worked travel to its face's totals, unless it was too steep."""
    if _keeps(run):
        n, w = _run_totals(run)
        _H.strokes[run["side"]] += n
        _H.worked[run["side"]] += w


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    p, _ = _pose(objects)
    move = np.zeros(3) if _H.last_pos is None else p - _H.last_pos
    _H.last_pos = p
    contact = (on_stone(objects) and _touching_stone()  # the edge down on the stone, and the stone pushing back
               and held(objects))  # a knife pushed or dragged without a hand on it is not a stroke
    if contact:
        if _H.ended is not None:  # back on the stone before the roll-off window closed: the last touch stands
            _commit(_H.ended)
            _H.ended = None
        ang, s = sharpen_angle(objects), side(objects)
        lead = float(move @ _edge_dir(objects))  # this step's travel across the edge (+ the way the edge faces)
        if _H.run is not None and _H.run["side"] != s:
            _commit(_H.run)
            _H.run = None
        if _H.run is None:
            _H.run = _new_run(s)
        if ang > STEEP:
            _H.run["steep"] += abs(lead)
        elif BAND[0] <= ang and level(objects) and edge_on_stone(objects) >= EDGE_ON:
            _pass_step(_H.run, lead, edge_mask(objects))
        _H.since_contact = 0
    else:
        if _H.run is not None:  # lifting off ends the touch; it is held back until it did not roll off the end
            _H.ended, _H.run = _H.run, None
        _H.since_contact += 1
        if _H.ended is not None:
            if held(objects) and _off_end(objects):
                _H.ended = None  # rolled off the end: that stroke does not count
            elif _H.since_contact > OFF_END_STEPS:
                _commit(_H.ended)
                _H.ended = None


def _totals():
    """Strokes and worked travel per face so far: the committed touches plus the one in progress or just ended (each
    counted only while it still stands)."""
    strokes = dict(_H.strokes)
    worked = {f: w.copy() for f, w in _H.worked.items()}
    for run in (_H.run, _H.ended):
        if _keeps(run):
            n, w = _run_totals(run)
            strokes[run["side"]] += n
            worked[run["side"]] += w
    return strokes, worked


def coverage(face):
    """How many of the 16 edge points (about 1 cm each, the last 3 mm short of the point) have been worked on `face`:
    on the stone for 10 cm of in-band, edge-leading travel, in strokes that count."""
    return int(np.sum(_totals()[1][face] >= COVER_TRAVEL))


def put_down_safely(objects):
    """Let go of, lying flat on the board, edge facing away from the robot."""
    return (on_board(objects) and _released(objects) and not held(objects)
            and float(_edge_dir(objects) @ np.array([1.0, 0.0, 0.0])) >= EDGE_AWAY)


def _job(objects):
    strokes = _totals()[0]
    a, b = strokes["A"], strokes["B"]
    if min(a, b) >= PER_SIDE:
        if min(coverage("A"), coverage("B")) < COVER:
            return 0.5  # both faces stroked, but on a face less than three quarters of the edge was worked
        return 1.0 if put_down_safely(objects) else 0.75
    if max(a, b) >= PER_SIDE:
        return 0.25  # one side only: the edge is pushed over to one side (a wire edge), not sharp
    return 0.0


def _score(objects):
    return _job(objects)


def success(objects):
    """Both faces sharpened along most of the edge, and the knife put down safely and still (where the
    arms end is not scored). Called first each step, so it also records the history."""
    _update(objects)
    still = np.linalg.norm(np.asarray(objects["knife"].lin_vel, float)) < 0.01
    return _job(objects) == 1.0 and still


def score(objects):
    """A stroke: the knife held, its edge touching the stone (verified contact: the simulator's contact solver reports the
    stone pushing on the knife with 0.5 N or more, with the edge lower than the spine and its lowest point over the
    stone within 0.5 mm of its top) with at least 3 cm of edge within 1 mm of it, and moved 5 cm edge-leading with its faces 13
    to 22 degrees to the stone and its length level, in one pass (lifting off or moving back 5 mm ends it; the
    edge-trailing return counts for nothing; travel is net). An edge point is worked on a
    face once it has been on the stone for 10 cm of such travel. A touch that does not count at all: one with 2 cm of
    travel steeper than 22 degrees (it works a steeper micro-bevel), or one whose edge drops 3 mm below the stone's top
    just past an end within 0.5 s of leaving it (rolled off the end). 0 with under 20 strokes on both faces; 0.25 for
    20+ strokes on one face only; 0.5 for 20+ on each face with under 12 of the 16 edge points (about 1 cm apart, 0.5 to
    16.2 cm along the 16.5 cm edge) worked on a face; 0.75 for 20+ strokes and 12+ points worked on each face; 1.0
    for that and the knife let go, lying flat on the board, edge facing away. Where the arms end is not scored.
    Possible scores: 0, 0.25, 0.5, 0.75, 1."""
    return _score(objects)


# --- Scripted demo (right arm only; the left arm stays home).
def _axrot(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def _hand(heading, spin):
    """The right hand's rotation for a side grip on the handle: fingers closing across the handle, approach along the
    knife (v at `heading`), turned `spin` degrees about the handle; spin -90 holds the knife flat (face A down),
    -90 + a tilts the edge down by a degrees, +90 - a turns it over (face B down) with the edge down by a."""
    import yam_arena.demo as demo
    r0 = _axrot((0, 0, 1), math.radians(heading)) @ kit._rot_xyz(math.pi / 2, 0.0) @ demo.grip_rotation(math.pi / 2)
    return tuple((_axrot(r0 @ np.array([0.0, 0.0, 1.0]), math.radians(spin)) @ r0).ravel())


DEMO_ANGLE = 17.0  # deg: the angle the demo aims for on the stone
DEMO_TRIM = {"A": 4.0, "B": -3.5}  # deg: under the press the knife sits low on face A and high on B: the arm's
#                                     joints give about 1.5 deg per N on A and 0.6 on B (J K^-1 J^T), plus the grip
GRIP_OPEN = 0.45 * FINGER_OPEN  # part open: room round the 18 mm handle without the lower finger hitting the bench
DEMO_FORCE = {"A": 3.5, "B": 2.5}  # N: the press the aim is planned for, per face. The knife's weight rests on the
#                  stone; each newton more needs the edge aimed deeper by the arm's vertical give at the edge (_give,
#                  scaled by GIVE_SIM). Planned alike for 3.5 N, the stone took a median 2.4 N on face A (pushed) and
#                  3.3 N on face B (pulled: the stone's drag holds the hand back), and at 3.3 N the drag turned the
#                  knife in the hand about 2 deg a stroke on face B (0.1 on A), so face B is planned lighter to press
#                  about as A does. Planned for 4.5 N, the edge pressed 6 to 7 N and came off the stone
GIVE_SIM = 0.6  # the arm's give at the edge in the sim against _give's model (that 4.5 N plan pressed 6 to 7 N)
DEMO_HEEL, DEMO_TIP = 0.025, 0.145  # m past the bolster: the part of the blade over the stone's middle line as each
#                                     stroke starts (the heel: the 0.5 cm scorer point 1.5 cm onto the 6 cm stone)
#                                     and as it ends (the tip: the last scorer point, 16.2 cm, 1.7 cm onto it)
DEMO_SWEEP_V = 0.125  # m: where the sweep along the blade ends; the last DEMO_DWELL of the stroke brings the tip on
DEMO_RUN = {"A": (-0.07, 0.04), "B": (0.04, -0.07)}  # m along the stone from its middle (+ away from the robot): where
#                    the edge starts and ends each stroke, 11 cm of the stone's 18.5 cm (the same places on the bench as
#                    before the stone moved STONE_SHIFT along). Face A's edge faces away, so its
#                    edge-leading stroke pushes; face B's faces the robot, so it pulls. Both go heel to tip. A face-A
#                    pull heel to tip is out of reach: drawing toward the robot and sliding toward the tip both turn
#                    the wrist (joint 4) further, and it reached its 90 deg limit 6 cm into the stroke (IK check)
DEMO_DWELL = 0.02  # m of each stroke with the heel (at the start) and with the tip (at the end) on the stone, before
#                    and after the sweep, so the ends of the edge get their share of the stone
DEMO_SETS = (("A", 11), ("B", 11), ("A", 11), ("B", 11))  # strokes per set: the faces alternated in equal sets, 22
#                    per face in all (against PER_SIDE 20). At each change of face the knife is laid on the board and
#                    gripped again where it was first held, so a turn of the knife in the hand does not build up
DEMO_REGRIP_YAW = {"A": 4.6, "B": 3.7}  # deg, by the face just worked: at a regrip the open hand turns this far about
#                    the vertical before closing again, to square itself to the knife. The stone's drag turns the knife
#                    in the hand (about the fingers' closing axis): in a telemetry run of this demo 0.9 deg over a set
#                    of 11 on face A and 7.3 deg over 11 on face B. After face A the hand turns 0.9 deg plus 3.7, so
#                    the knife starts face B turned 3.7 deg the other way and the drag brings it through square
#                    halfway through the set; after face B it turns 3.7 deg, and face A starts square
DEMO_TIP_YAW = -10.0  # deg: during the sweep the knife turns 10 deg on the stone (tip further from the robot). Held
#                       square to the stone, the tip end puts the wrist at its limit (joint 5 at -90 deg, IK misses by
#                       up to 17 mm); turned 10 deg, joint 4 stays 2.6 deg inside its limit. A sharpener lifts the
#                       handle a little for the tip; the demo does not: 2.5 deg of lift sent the wrist (joints 4 and
#                       6) swinging 60 deg through its singularity in 0.4 s. Without it the last scorer points, up
#                       the tip's 6 mm rise, sit 1 to 2 mm above the stone and may not count (12 of 16 are needed)


def _give(pos, rot, point):
    """The arm's vertical give (m per N) at `point` (world) for a pushed-up held tool, the hand at (`pos`, `rot`):
    J K^-1 J^T from the joint servos' stiffness (constants.py), the joints from the arm's own IK."""
    from yam_arena import demo
    from yam_arena.constants import SHOULDER, WRIST
    from yam_arena.station import BASES

    base = np.asarray(BASES["right"], float)
    q, _, _ = demo.ik.solve(np.asarray(pos), demo.ik.matrix_to_quat_xyzw(np.asarray(rot).reshape(3, 3)),
                            q_init=None, base_pos=base)

    def fk(qq):
        p, r = demo.ik.tcp_pose(qq, base)
        r = np.asarray(r)
        return np.asarray(p), r.reshape(3, 3) if r.size == 9 else demo.ik.quat_xyzw_to_matrix(r)

    p0, r0 = fk(q)
    off = r0.T @ (np.asarray(point) - p0)
    jz = np.zeros(6)
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-6
        (pp, rp), (pm, rm) = fk(q + dq), fk(q - dq)
        jz[i] = ((pp + rp @ off) - (pm + rm @ off))[2] / 2e-6
    k = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
    return float(np.sum(jz * jz / k))


def _stroke_pose(face, v, u, turn=0.0, press=True):
    """The hand's (position, rotation) with blade point `v` (m past the bolster) over the stone's middle line and the
    edge `u` m along the stone from its middle, at the demo's angle for `face`; `turn`: the share of DEMO_TIP_YAW the
    knife is turned on the stone (0 to 1, as the stroke nears the tip). `press`: the edge aimed into the stone for
    DEMO_FORCE (else just on it)."""
    ang = DEMO_ANGLE + DEMO_TRIM[face]
    heading = STONE_HEADING + turn * DEMO_TIP_YAW
    su, sv = _dirs(heading)
    rot0 = np.array(_hand(heading, -90.0 + ang if face == "A" else 90.0 - ang)).reshape(3, 3)
    reach = math.cos(math.radians(ang)) * EDGE_X  # the edge's distance across from the handle axis, seen from above
    c = np.r_[STONE_POS, 0.0] - (v - GRIP_Y) * np.r_[sv[:2], 0.0]  # the grip point with point v on the middle line
    a = c + su * (u - (reach if face == "A" else -reach))  # face A's edge faces +u (away), face B's -u
    a[2] = TOP + STONE_TOP + math.sin(math.radians(ang)) * EDGE_X
    if press:
        edge = a + (v - GRIP_Y) * sv + su * (reach if face == "A" else -reach)
        edge[2] = TOP + STONE_TOP
        a[2] -= (DEMO_FORCE[face] - KNIFE_MASS * 9.81) * GIVE_SIM * _give(a, rot0, edge)
    return a, tuple(rot0.ravel())


def demo():
    """Right arm: slide the open hand sideways round the handle (fingers above and below it), lift, and sharpen in
    sweeping heel-to-tip strokes, edge leading: each sets the heel on the stone, draws the knife along the stone while
    sliding it along its own length so the stone runs from heel to tip (the knife turning 10 deg as the tip comes on),
    then lifts and goes back. The faces alternate in sets of 11: face A (edge away from the robot) pushed, face B
    (edge toward the robot, the knife turned over) pulled toward the robot, A, then B, 22 strokes per face. At each
    change of face the knife is laid flat on the board, let go and gripped again where it was first held. Lay it back
    on the board edge-away, let go, back out and go home."""
    from yam_arena.constants import FINGER_CLOSED
    T = kit.Tilted
    C = FINGER_CLOSED
    z_board = TOP + BOARD[2] + BLADE_T / 2
    pu = P_U
    grip = np.r_[PARK_TCP, z_board]
    beside = grip + 0.06 * pu  # past the spine-to-edge line, clear of the blade (which starts 6 cm further along)
    flat = _hand(PARK_HEADING, -90.0)
    right = [T(1.4, tuple(beside + [0, 0, 0.03]), GRIP_OPEN, "beside the handle", rot=flat),
             T(1.0, tuple(beside), GRIP_OPEN, "down beside it", rot=flat),
             T(1.2, tuple(grip), GRIP_OPEN, "slide round the handle", rot=flat),
             T(0.8, tuple(grip), C, "grip", rot=flat),
             T(1.0, tuple(grip + [0, 0, 0.06]), C, "lift", rot=flat)]
    centre = np.r_[STONE_POS, 0.0] - STONE_V * np.r_[S_V[:2], 0.0]  # the grip point with the blade's middle on the stone
    high = centre + [0, 0, TOP + STONE_TOP + 0.07]
    upright = _hand(STONE_HEADING, 0.0)
    up = 0.015  # m the hand lifts between strokes

    def strokes(face, n):
        u0, u1 = DEMO_RUN[face]
        d = DEMO_DWELL if u1 > u0 else -DEMO_DWELL
        start, rot = _stroke_pose(face, DEMO_HEEL, u0)
        heel2, _ = _stroke_pose(face, DEMO_HEEL, u0 + d)
        swept, rot_swept = _stroke_pose(face, DEMO_SWEEP_V, u1 - d, turn=1.0)
        end, _ = _stroke_pose(face, DEMO_TIP, u1, turn=1.0)
        start_on, _ = _stroke_pose(face, DEMO_HEEL, u0, press=False)  # the edge just on the stone, for lifts
        end_on, _ = _stroke_pose(face, DEMO_TIP, u1, turn=1.0, press=False)
        out = [T(1.0, tuple(start_on + [0, 0, 0.03]), C, "over the stone, heel on it", rot=rot)]
        for _ in range(n):
            out += [T(0.4, tuple(start), C, "heel on the stone", rot=rot),
                    T(0.3, tuple(heel2), C, "stroke: heel", rot=rot),
                    T(1.2, tuple(swept), C, "stroke: sweep heel to tip", rot=rot_swept),
                    T(0.4, tuple(end), C, "stroke: tip", rot=rot_swept),
                    T(0.3, tuple(end_on + [0, 0, up]), C, "lift", rot=rot_swept),
                    T(0.6, tuple(start_on + [0, 0, up]), C, "back", rot=rot)]
        return out

    def off_stone(face):
        """Lift straight up from the stroke's end; from face B turn the knife back upright above the stone."""
        last = right[-1]
        out = [T(0.6, tuple(np.array(last.pos) + [0, 0, 0.04]), C, "lift clear of the stone", rot=last.rot)]
        if face == "B":
            out.append(T(0.9, tuple(high), C, "turn the knife back upright", rot=upright))
        return out

    def regrip(face):
        """Lay the knife flat on the board where it was picked up, open the hand, close it again round the handle
        where it was first gripped, and lift; from there to face B the knife is turned over above the stone."""
        out = off_stone(face) + [T(1.2, tuple(grip + [0, 0, 0.03]), C, "over the board", rot=flat),
                                 T(0.6, tuple(grip + [0, 0, 0.004]), C, "lay it on the board", rot=flat),
                                 T(0.3, tuple(grip + [0, 0, 0.004]), GRIP_OPEN, "let go to regrip", rot=flat)]
        yaw = DEMO_REGRIP_YAW[face]
        turn = _axrot((0, 0, 1), math.radians(yaw))
        rot = tuple((turn @ np.array(flat).reshape(3, 3)).ravel())
        back = grip + turn @ (beside - grip)  # the approach point beside the handle, turned with the hand
        out += [T(0.4, tuple(beside + [0, 0, 0.004]), GRIP_OPEN, "slide out", rot=flat),
                T(0.4, tuple(back), GRIP_OPEN, "square the open hand to the handle", rot=rot),
                T(0.7, tuple(grip), GRIP_OPEN, "slide round the handle again", rot=rot),
                T(0.5, tuple(grip), C, "grip again", rot=rot),
                T(0.5, tuple(grip + [0, 0, 0.06]), C, "lift", rot=flat)]
        if face == "A":  # next is face B: turn it over above the stone
            out.append(T(1.2, tuple(high), C, "turn the knife over", rot=upright))
        return out

    sets = DEMO_SETS
    for k, (face, n) in enumerate(sets):
        if k:
            right += regrip(sets[k - 1][0])
        elif face == "B":  # it is picked up face A down: a first set on face B needs the knife turned over
            right += [T(1.2, tuple(high), C, "turn the knife over", rot=upright)]
        right += strokes(face, n)
    right += off_stone(sets[-1][0])
    right += [T(1.2, tuple(grip + [0, 0, 0.06]), C, "back over the board", rot=flat),
              T(0.8, tuple(grip + [0, 0, 0.004]), C, "lay it down", rot=flat),
              T(0.5, tuple(grip + [0, 0, 0.004]), GRIP_OPEN, "let go", rot=flat),
              T(0.8, tuple(beside + [0, 0, 0.004]), GRIP_OPEN, "slide out", rot=flat),
              T(0.6, tuple(beside + [0, 0, 0.06]), GRIP_OPEN, "up", rot=flat),
              kit.Home(1.5)]
    total = sum(w.duration for w in right)
    left = [kit.Home(total - 1.5), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
