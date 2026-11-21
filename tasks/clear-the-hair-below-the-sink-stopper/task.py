"""A bathroom vanity stands on the station table, a little in front of the robot: a 31 x 22 in stone countertop with
an oval white china basin mounted under it, a single-hole faucet and a wall behind. The basin's chrome pop-up drain
has its stopper standing open, the domed chrome head a little above the drain's flange. Below the stopper, out of
sight inside the drain, a clump of hair is caught round the stopper's stem and its pivot rod. An orange hair-clog
tool, a hand-held handle with a thin flexible 12-inch wand ending in a dark hooked pad, lies along the front edge of
the countertop, in front of the basin: its wand points to the robot's right and its handle reaches past the left end of
the countertop. Both arms start at their ready pose with the grippers open.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
from functools import lru_cache

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import parts  # noqa: E402  (before lots: its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.1  # the 1 s release-and-still hold is counted inside score()
INSTRUCTION = "Clear the hair from under the sink stopper, then leave the tool and the hair on the counter."
MM = 1e-3
EPS = 1e-9

# -- geometry (m; x away from the arms, y to the robot's left, z above the table top) ------------------------------
BX, BY = 0.360, -0.020  # the drain's axis
ZF = 0.055  # top of the drain's flange
FLANGE_T, FLANGE_R = 0.004, 0.027  # K-7114 flange: 54 mm across, 4 mm proud of the basin's bottom
ZB = ZF - FLANGE_T  # the basin's bottom round the drain
BORE = 0.016  # drain throat and body: 32 mm bore
TUBE_OUT = 0.0195
ZP = ZF - 0.050  # pivot rod's axis, 50 mm below the flange top
ROD_R = 0.0024  # 3/16 in pivot rod
WATER_DEPTH = 0.162  # Caxton K-2205: 6-3/8 in from the rim to the bottom
COUNTER_T = 0.030
CT = ZB + WATER_DEPTH + COUNTER_T  # countertop surface, 243 mm above the table
BOWL_A, BOWL_B = 0.178, 0.216  # basin opening: 356 mm front to back (x), 432 mm side to side (y), an oval
BOTTOM_A, BOTTOM_B = 0.070, 0.090  # flat of the bowl's bottom round the drain (estimate)
CTR_FRONT, CTR_BACK = BX - 0.255, BX + 0.304  # 22 in (559 mm) deep top
CTR_L, CTR_R = BY + 0.3935, BY - 0.3935  # 31 in (787 mm) wide top
# stopper, standing open (estimates from the K-7114 drawing: 54 mm flange, 1-1/4 in tailpiece)
HEAD_R, HEAD_T, LIFT = 0.021, 0.007, 0.020  # 42 mm domed head, its underside 20 mm above the flange top (open)
STEM_R = 0.0045
FIN_HALF, FIN_T = 0.0150, 0.002  # four guide fins, 30 mm across, 2 mm thick
PSI = math.radians(-105.0)  # seen from above, the wand comes to the drain from this side: its handle is left, a little back
FIN_AT = PSI + math.pi + math.pi / 4  # the fins stand 45 degrees either side of the wand's way in (an estimate of how
# the stopper happens to sit; any one of its four gaps takes the wand)
FIN_TOP, FIN_BOT = ZF + LIFT - 0.002, ZF - 0.030
# the hair clump: caught round the stopper's stem and the pivot rod from 22 to 52 mm below the flange top
HAIR_TOP, HAIR_BOT = ZF - 0.022, ZF - 0.052

CHROME = (0.80, 0.81, 0.83)
CHINA = (0.95, 0.95, 0.94)
STONE = (0.78, 0.77, 0.74)
WOOD = (0.55, 0.42, 0.30)
WALL = (0.86, 0.85, 0.82)
DARK = (0.18, 0.18, 0.20)
HAIR = (0.24, 0.16, 0.10)
ORANGE = (0.95, 0.47, 0.10)
ORANGE_D = (0.80, 0.33, 0.06)
WAND_C = (0.34, 0.35, 0.38)  # grey wand, dark against the pale stone
PAD_C = (0.10, 0.10, 0.12)
SLICK = (0.30, 0.25)  # chrome and plastic, wet (static, dynamic; estimate)


def _b(lo, hi, color, collide=True, yaw=0.0):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(hi - lo), at=tuple((lo + hi) / 2), color=color, collide=collide, yaw=yaw)


def _ellipse_r(a, b, phi):
    return 1.0 / math.sqrt((math.cos(phi) / a) ** 2 + (math.sin(phi) / b) ** 2)


def _vanity_shapes():
    """World-aligned, the part's origin at the drain's axis on the table top."""
    s = []

    def b(x0, x1, y0, y1, z0, z1, color, **kw):
        s.append(_b((x0 - BX, y0 - BY, z0), (x1 - BX, y1 - BY, z1), color, **kw))

    # countertop: four slabs round the oval cut-out, plus corner fillers that round the cut-out's corners
    z0, z1 = CT - COUNTER_T, CT
    b(CTR_FRONT, BX - BOWL_A, CTR_R, CTR_L, z0, z1, STONE)
    b(BX + BOWL_A, CTR_BACK, CTR_R, CTR_L, z0, z1, STONE)
    b(BX - BOWL_A, BX + BOWL_A, BY + BOWL_B, CTR_L, z0, z1, STONE)
    b(BX - BOWL_A, BX + BOWL_A, CTR_R, BY - BOWL_B, z0, z1, STONE)
    n = 24  # each corner: strips that fill between the oval and the rectangle
    for qx in (-1, 1):
        for qy in (-1, 1):
            for i in range(n):
                xa, xb = BOWL_A * i / n, BOWL_A * (i + 1) / n
                ye = BOWL_B * math.sqrt(max(0.0, 1 - (xb / BOWL_A) ** 2))
                if BOWL_B - ye < 0.001:
                    continue
                xs = sorted((qx * xa, qx * xb))
                ys = sorted((qy * ye, qy * BOWL_B))
                b(BX + xs[0], BX + xs[1], BY + ys[0], BY + ys[1], z0, z1, STONE)
    # basin: a flat oval bottom round the drain hole and a faceted bowl wall up to the countertop's underside
    zb0 = ZB - 0.008
    hole = 0.022  # 1-3/4 in drain hole
    k = 24
    for i in range(k):
        phi0, phi1 = 2 * math.pi * i / k, 2 * math.pi * (i + 1) / k
        ph = (phi0 + phi1) / 2
        r0 = _ellipse_r(BOTTOM_A, BOTTOM_B, ph)
        r1 = _ellipse_r(BOWL_A, BOWL_B, ph)
        # bottom: a wedge from the hole out to the bottom's edge, as a box along the radius
        w_in = 2 * hole * math.tan(math.pi / k) + 0.004
        w_out = 2 * r0 * math.tan(math.pi / k) + 0.006
        mid = (hole + r0) / 2
        s.append(parts.box((r0 - hole, max(w_in, w_out), 0.008),
                           at=(mid * math.cos(ph), mid * math.sin(ph), ZB - 0.004), color=CHINA, yaw=ph))
        # wall: a panel from the bottom's edge up to the rim, sloping out
        dr, dz = r1 - r0, (CT - COUNTER_T) - ZB
        length = math.hypot(dr, dz)
        ang = math.atan2(dz, dr)  # panel's slope above the horizontal
        width = 2 * r1 * math.tan(math.pi / k) + 0.012
        c = ((r0 + r1) / 2, (ZB + CT - COUNTER_T) / 2)
        # the panel's own x runs up the slope; its thickness is 8 mm, set 4 mm behind the bowl's inner surface
        cr, cz = c[0] + 0.004 * math.sin(ang), c[1] - 0.004 * math.cos(ang)
        rot = parts._rot((0, 0, 1), ph) @ parts._rot((0, 1, 0), -ang)
        s.append(parts.box((length, width, 0.008), at=(cr * math.cos(ph), cr * math.sin(ph), cz), color=CHINA,
                           rot=rot))
    # cabinet under the countertop, cut off at the table top: sides, back and a closed face frame with two doors
    cf, cb = CTR_FRONT + 0.020, CTR_BACK - 0.005
    cl, cr = CTR_L - 0.0125, CTR_R + 0.0125
    b(cf, cb, cl - 0.018, cl, 0.0, z0, WOOD)
    b(cf, cb, cr, cr + 0.018, 0.0, z0, WOOD)
    b(cb - 0.016, cb, cr + 0.018, cl - 0.018, 0.0, z0, WOOD)
    b(cf, cf + 0.019, cr + 0.018, cl - 0.018, 0.0, z0, WOOD)
    # wall and backsplash behind the countertop
    b(CTR_BACK, CTR_BACK + 0.020, CTR_R - 0.10, CTR_L + 0.10, 0.0, CT + 0.45, WALL)
    b(CTR_BACK - 0.012, CTR_BACK, CTR_R, CTR_L, CT, CT + 0.100, STONE)
    # single-hole faucet behind the basin: a round body and a spout reaching out over the bowl
    fx = BX + BOWL_A + 0.040
    s.append(parts.cylinder(0.025, 0.020, at=(fx - BX, 0.0, CT + 0.010), color=CHROME))
    s.append(parts.cylinder(0.016, 0.110, at=(fx - BX, 0.0, CT + 0.075), color=CHROME))
    s.append(parts.cylinder(0.011, 0.120, at=(fx - BX - 0.060, 0.0, CT + 0.125), color=CHROME, axis="x"))
    s.append(parts.cylinder(0.012, 0.020, at=(fx - BX - 0.112, 0.0, CT + 0.112), color=CHROME))
    s.append(parts.box((0.012, 0.010, 0.050), at=(fx - BX + 0.012, 0.0, CT + 0.150), color=CHROME))
    return s


VANITY = parts.Part("vanity", pos=(BX, BY, TOP), shapes=_vanity_shapes(), contact_offset=0.003, friction=(0.6, 0.5))


def _staves(r_in, r_out, z0, z1, n, color, cut=None):
    out = []
    w = 2 * r_out * math.tan(math.pi / n) + 0.0005
    rc = (r_in + r_out) / 2
    for i in range(n):
        a = 2 * math.pi * (i + 0.5) / n
        spans = [(z0, z1)]
        if cut is not None and abs(math.remainder(a - cut[0], 2 * math.pi)) < cut[1]:
            spans = [(z0, cut[2]), (cut[3], z1)]
        for lo, hi in spans:
            if hi - lo > 1e-4:
                out.append(parts.box((r_out - r_in, w, hi - lo), at=(rc * math.cos(a), rc * math.sin(a), (lo + hi) / 2),
                                     color=color, yaw=a))
    return out


def _drain_shapes():
    """Kohler K-7114 pop-up drain body, its axis the part's z axis: flange, body and tailpiece (cut off at the table
    top), the pivot rod through the stopper's eye, and the rod's port and nut on the body's back."""
    s = []
    s += _staves(BORE, FLANGE_R, ZB, ZF, 20, CHROME)  # flange
    s += _staves(BORE, TUBE_OUT, 0.0, ZB, 20, CHROME, cut=(0.0, 0.25, ZP - 0.006, ZP + 0.006))
    # pivot rod: from its port on the body's back, across the bore and through the stopper's eye
    s.append(parts.cylinder(ROD_R, 0.075, at=(0.0375 - 0.008, 0.0, ZP), color=(0.55, 0.56, 0.58), axis="x"))
    s.append(parts.cylinder(0.009, 0.022, at=(TUBE_OUT + 0.011, 0.0, ZP), color=CHROME, axis="x"))  # port
    s.append(parts.cylinder(0.011, 0.010, at=(TUBE_OUT + 0.026, 0.0, ZP), color=CHROME, axis="x"))  # nut
    s += _staves(TUBE_OUT, 0.024, ZB - 0.018, ZB - 0.008, 16, (0.65, 0.58, 0.40))  # locknut under the basin
    return s


DRAIN = parts.Part("drain", pos=(BX, BY, TOP), shapes=_drain_shapes(), contact_offset=0.002, friction=SLICK)


def _stopper_shapes():
    """The pop-up stopper, standing open: its head 20 mm above the flange, four guide fins in the bore, a stem down to
    the eye the pivot rod passes through."""
    s = [parts.cylinder(HEAD_R, HEAD_T, at=(0.0, 0.0, ZF + LIFT + HEAD_T / 2), color=CHROME),
         parts.cylinder(0.014, 0.004, at=(0.0, 0.0, ZF + LIFT + HEAD_T + 0.002), color=CHROME),
         parts.cylinder(0.018, 0.003, at=(0.0, 0.0, ZF + LIFT - 0.0015), color=DARK)]  # gasket
    for a in (FIN_AT, FIN_AT + math.pi / 2):
        s.append(parts.box((2 * FIN_HALF, FIN_T, FIN_TOP - FIN_BOT), at=(0.0, 0.0, (FIN_TOP + FIN_BOT) / 2),
                           color=DARK, yaw=a))
    s.append(parts.cylinder(STEM_R, FIN_TOP - (ZP - 0.008), at=(0.0, 0.0, (FIN_TOP + ZP - 0.008) / 2), color=DARK))
    s.append(parts.box((0.008, 0.010, 0.016), at=(0.0, 0.0, ZP), color=DARK, collide=False))  # the eye round the rod
    return s


STOPPER = parts.Part("stopper", pos=(BX, BY, TOP), shapes=_stopper_shapes(), contact_offset=0.002, friction=SLICK)


def _hair_shapes():
    """Hair wound round the stem and the rod, with strands hanging below: drawn only (no collider)."""
    s = []
    for i in range(10):
        a = 2 * math.pi * i / 10
        r = 0.0085
        s.append(parts.box((0.006, 0.010, 0.020), at=(r * math.cos(a), r * math.sin(a), 0.0), color=HAIR,
                           collide=False, yaw=a + math.pi / 2))
    for dx, dy in ((0.010, 0.004), (-0.006, 0.009), (0.003, -0.011), (-0.010, -0.004)):
        s.append(parts.box((0.002, 0.002, 0.014), at=(dx, dy, -0.016), color=HAIR, collide=False))
    s.append(parts.box((0.022, 0.009, 0.009), at=(0.012, 0.0, ZP - (ZF - 0.030)), color=HAIR, collide=False))
    return s


HAIR_AT = np.array([BX, BY, TOP + ZF - 0.030])  # the clump's origin: round the stem, 30 mm below the flange top
HAIR_PART = parts.Part("hair", pos=tuple(float(v) for v in HAIR_AT), mass=0.010, shapes=_hair_shapes(),
                       gravity=False)

# -- the tool: FlexiSnake Drain Weasel PLUS, reusable handle with its 12-inch wand -------------------------------------
# Tool frame: x along the tool toward the wand's end, origin where the wand enters the handle; the wand's hinges turn
# about the frame's y axis. The handle runs back from x = 0 to x = -0.133.
HANDLE_L = 0.133
SLEEVE_L, SLEEVE_W = 0.045, 0.030
GRIP_R = 0.013
WAND_L, WAND_R = 0.305, 0.002
N_LINKS, LINK_L = 8, 0.016
RIGID_L = WAND_L - N_LINKS * LINK_L  # 177 mm of wand held straight by the handle's socket and its own stiffness
PAD_LINKS = 2  # the hooked pad covers the last 32 mm
GRIP_AT = -0.095  # where the demo takes the handle (tool frame x)
BEND_K = 0.20  # N m/rad at each wand joint (estimate: a 3 mm polypropylene strip)
BEND_LIMIT = math.radians(75)


def _tool_shapes():
    return [
        parts.box((SLEEVE_L, SLEEVE_W, SLEEVE_W), at=(-SLEEVE_L / 2, 0.0, 0.0), color=ORANGE_D),
        parts.cylinder(GRIP_R, HANDLE_L - SLEEVE_L - 0.008, at=(-(SLEEVE_L + HANDLE_L - 0.008) / 2, 0.0, 0.0),
                       color=ORANGE, axis="x"),
        parts.cylinder(0.015, 0.008, at=(-HANDLE_L + 0.004, 0.0, 0.0), color=ORANGE_D, axis="x"),
        parts.cylinder(WAND_R, RIGID_L, at=(RIGID_L / 2, 0.0, 0.0), color=WAND_C, axis="x"),
    ]


def _link(i):
    x0 = RIGID_L + i * LINK_L
    pad = i >= N_LINKS - PAD_LINKS
    shape = (parts.box((LINK_L, 0.006, 0.005), at=(x0 + LINK_L / 2, 0.0, 0.0), color=PAD_C) if pad else
             parts.cylinder(WAND_R, LINK_L, at=(x0 + LINK_L / 2, 0.0, 0.0), color=WAND_C, axis="x"))
    return parts.Hinge(f"w{i}", shapes=(shape,), pivot=(x0, 0.0, 0.0), axis=(0.0, 1.0, 0.0), lower=-BEND_LIMIT,
                       upper=BEND_LIMIT, mass=0.002, spring=BEND_K, damping=0.001, max_effort=2.0,
                       parent="base" if i == 0 else f"w{i - 1}")


LAST = f"w{N_LINKS - 1}"
LYING = (0.0, 0.0, -math.sqrt(0.5), math.sqrt(0.5))  # wand toward the robot's right, hinges' axis away, frame z up
OVERHANG = 0.045  # lot A: the wand meets the handle 45 mm in from the countertop's left end (the sleeve on the stone,
# the grip beyond the top's end)
TOOL_X, TOOL_Y = CTR_FRONT + 0.030, CTR_L - OVERHANG
REST_Z = TOP + CT + SLEEVE_W / 2  # the lying tool's axis: 15 mm above the stone
LIFT_GAP = 0.0005  # placed 0.5 mm above the stone
TOOL = parts.Part("tool", pos=(TOOL_X, TOOL_Y, REST_Z + LIFT_GAP), mass=0.040, quat=LYING,
                  shapes=_tool_shapes(), joints=tuple(_link(i) for i in range(N_LINKS)), contact_offset=0.005,
                  self_collision=False, friction=(0.5, 0.4))

OBJECTS = [VANITY, DRAIN, STOPPER, HAIR_PART, TOOL]
if os.environ.get("HAIR_PROBE"):  # trace-only probe: two plain cubes resting on the stone, to check its collider height
    OBJECTS += [parts.Part("probe_back", pos=(0.58, 0.32, TOP + CT + 0.0155), mass=0.05,
                           shapes=[parts.box((0.03, 0.03, 0.03), color=(0.2, 0.6, 0.2))]),
                parts.Part("probe_front", pos=(0.14, -0.30, TOP + CT + 0.0155), mass=0.05,
                           shapes=[parts.box((0.03, 0.03, 0.03), color=(0.2, 0.6, 0.2))])]
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A"),
    lots.Lot("B", place={TOOL.name: (TOOL_X, TOOL_Y - 0.020, REST_Z + LIFT_GAP)}),
)


def _R(q):
    return parts._quat_to_matrix(q)


def _finite(*vals):
    return all(np.all(np.isfinite(np.asarray(v, float))) for v in vals)


# -- reading the state ------------------------------------------------------------------------------------------------


def tool_points(tool):
    """World points along the tool: the handle's two ends and every wand joint and the wand's end."""
    p, r = np.asarray(tool.pos, float), _R(tool.quat_xyzw)
    pts = [p + r @ np.array([-HANDLE_L, 0.0, 0.0]), p, p + r @ np.array([RIGID_L, 0.0, 0.0])]
    for i in range(N_LINKS):
        lp, lq = tool.links[f"w{i}"]
        lr = _R(lq)
        pts.append(np.asarray(lp, float) + lr @ np.array([LINK_L, 0.0, 0.0]))
    return np.array(pts)


def pad_centre(tool):
    """World centre of the hooked pad (the last two links)."""
    lp, lq = tool.links[f"w{N_LINKS - PAD_LINKS}"]
    return np.asarray(lp, float) + _R(lq) @ np.array([PAD_LINKS * LINK_L / 2, 0.0, 0.0])


def in_hair(p):
    """A point in the clump: inside the drain's bore, between 22 and 52 mm below the flange top."""
    r = math.hypot(p[0] - BX, p[1] - BY)
    return r <= BORE + EPS and TOP + HAIR_BOT - EPS <= p[2] <= TOP + HAIR_TOP + EPS


def in_bowl_xy(p, margin=0.0):
    return ((p[0] - BX) / (BOWL_A + margin)) ** 2 + ((p[1] - BY) / (BOWL_B + margin)) ** 2 < 1.0


SAG = 0.010  # "out of the basin": no lower than 10 mm under the countertop's surface


def on_counter(p):
    """A point over the stone (outside the basin's opening, inside the top's outline) no lower than 3 mm under its
    surface."""
    return (CTR_FRONT - EPS <= p[0] <= CTR_BACK + EPS and CTR_R - EPS <= p[1] <= CTR_L + EPS
            and not in_bowl_xy(p) and p[2] >= TOP + CT - 0.003 - EPS)


def at_counter_height(p):
    """A point inside the vanity's outline at the countertop's height: no more than 10 mm under the stone's surface
    (over the stone, or reaching past the basin's rim without hanging into it)."""
    return (CTR_FRONT - EPS <= p[0] <= CTR_BACK + EPS and CTR_R - EPS <= p[1] <= CTR_L + EPS
            and p[2] >= TOP + CT - SAG - EPS)


def hair_points(hair):
    p, r = np.asarray(hair.pos, float), _R(hair.quat_xyzw)
    lo, hi = [], []
    for s in HAIR_PART.shapes:
        c = p + r @ np.asarray(s.at)
        ext = np.abs(r @ s.turn) @ s.own_half
        lo.append(c - ext)
        hi.append(c + ext)
    return np.min(lo, 0), np.max(hi, 0)


def hair_out_of_drain(hair):
    """The clump is out of the drain: its centre 10 mm or more above the flange's top, or outside the flange (more than
    27 mm from the drain's axis, on the basin's bottom or anywhere else)."""
    p = np.asarray(hair.pos, float)
    return bool(p[2] >= TOP + ZF + 0.010 - EPS or math.hypot(p[0] - BX, p[1] - BY) > FLANGE_R + EPS)


def hair_on_counter(hair):
    """The clump at the countertop's height inside the vanity's outline (out of the basin)."""
    return at_counter_height(np.asarray(hair.pos, float))


def in_footprint(p):
    return CTR_FRONT - EPS <= p[0] <= CTR_BACK + EPS and CTR_R - EPS <= p[1] <= CTR_L + EPS


def tool_on_counter(tool):
    """The whole tool out of the basin, lying on the countertop: the sleeve (where the wand meets the handle) over the
    stone, and every point along the tool between 10 mm below and 40 mm above the stone's surface, none lower inside the
    basin's opening (the handle may reach past the top's end, as it starts; the wand's end may reach over the rim)."""
    pts = tool_points(tool)
    if not on_counter(pts[1]):
        return False
    return all(TOP + CT - SAG - EPS <= p[2] <= TOP + CT + 0.040 + EPS for p in pts)


def off_vanity(tool):
    """The tool has dropped off the vanity: any point along it more than 20 mm below the countertop's surface outside
    the vanity's outline (inside the outline, below the countertop, is the basin)."""
    return any(p[2] < TOP + CT - 0.020 - EPS and not in_footprint(p) for p in tool_points(tool))


def tier(objects):
    """The state's value before the release gate: 0.25 the hair out of the drain; 0.5 also out of the basin, resting
    over the stone; 1.0 also the whole tool resting over the stone."""
    hair, tool = objects[HAIR_PART.name], objects[TOOL.name]
    if not REC["caught"] or not hair_out_of_drain(hair):
        return 0.0
    if not hair_on_counter(hair):
        return 0.25
    return 1.0 if tool_on_counter(tool) else 0.5


# -- hands clear ---------------------------------------------------------------------------------------------------
CLEAR = 13.0 * MM  # 10 mm clear plus 3 mm for the 3 mm face sampling


def scored_boxes(objects):
    tool = objects[TOOL.name]
    poses = {"base": (np.asarray(tool.pos, float), _R(tool.quat_xyzw))}
    poses.update({k: (np.asarray(v[0], float), _R(v[1])) for k, v in tool.links.items()})
    return [(c, r, s) for _, c, r, s in TOOL.world_boxes(poses)]


def hands_clear(clouds, objects):
    """True only when every sampled gripper point is more than CLEAR from every box of the tool."""
    if not clouds:
        return False
    pts = []
    for p in clouds:
        p = np.asarray(p, float)
        if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
            return False
        pts.append(p)
    pts = np.concatenate(pts)
    for c, r, s in scored_boxes(objects):
        local = np.abs((pts - c) @ r) - s / 2
        if np.linalg.norm(np.maximum(local, 0.0), axis=1).min() <= CLEAR + EPS:
            return False
    return True


STILL_V, DRIFT_POS = 3.0 * MM, 1.0 * MM
HOLD_S = 1.0
CATCH_S = 0.5  # the pad must stay in the clump this long for its hooks to snag the hair
PULL = 0.010  # and then be drawn back up this far from its deepest point in the clump for the snagged hair to come away


def calm(objects):
    tool = objects[TOOL.name]
    return float(np.linalg.norm(tool.lin_vel)) <= STILL_V + EPS


def drifted(ref, now):
    return ref is None or float(np.linalg.norm(now - ref, axis=1).max()) > DRIFT_POS + EPS


# -- the trial record: written only by the simulator's step and reset hooks -----------------------------------------


def _fresh():
    return dict(known=False, invalid=False, t=None, held=0.0, ref=None, clear=False, still=False, value=None,
                dropped=False, caught=False, in_hair=0.0, deepest=math.inf, stamp=None)


REC = _fresh()


def stamp(objects):
    """The tool's pose as this step observed it, so score() reads the state the record was built from. (The hair is not
    in it: the declared rule moves the clump onto the pad after each observation, before the step is scored.)"""
    tool = objects[TOOL.name]
    out = [*np.round(np.asarray(tool.pos, float), 9), *np.round(np.asarray(tool.quat_xyzw, float), 9)]
    return tuple(float(v) for v in out)


def observe(objects, clouds, t):
    """Update the trial record from one control step's state and gripper surface samples (None: unknown)."""
    try:
        tool, hair = objects[TOOL.name], objects[HAIR_PART.name]
        pts = tool_points(tool)
        ok = _finite(tool.pos, tool.quat_xyzw, tool.lin_vel, hair.pos, pts)
    except (KeyError, TypeError, ValueError, AttributeError):
        ok = False
    dt = None if REC["t"] is None else float(t) - REC["t"]
    if not ok or clouds is None or (dt is not None and not 0.0 < dt <= 0.2):
        REC.update(known=False, invalid=True, held=0.0, t=float(t), stamp=None)
        return
    if off_vanity(tool):
        REC["dropped"] = True
    if not REC["caught"] and dt is not None:
        pad = pad_centre(tool)
        if in_hair(pad):
            REC["in_hair"] += dt
            REC["deepest"] = min(REC["deepest"], float(pad[2]))
        if (REC["in_hair"] >= CATCH_S - EPS and pad[2] >= REC["deepest"] + PULL - EPS
                and math.hypot(pad[0] - BX, pad[1] - BY) <= BORE + EPS):
            REC["caught"] = True
    value = tier(objects)
    clear = bool(hands_clear(clouds, objects))
    now = pts
    still = calm(objects)
    if clear and still and value == REC["value"] and dt is not None and not drifted(REC["ref"], now):
        REC["held"] += dt
    else:
        REC["held"], REC["ref"] = 0.0, now
    REC.update(known=True, t=float(t), value=value, clear=clear, still=still, stamp=stamp(objects))


def score(objects):
    """0 for good once the tool has dropped off the vanity or the record is invalid. Otherwise, with both grippers
    clear of the tool and the tool still for the last 1 s: 1.0 the hair caught on the tool's pad and the tool and the
    hair resting on the countertop outside the basin; 0.5 the hair caught and resting on the countertop but some of
    the tool in the basin; 0.25 the hair caught and out of the drain but left in the basin; 0 otherwise."""
    try:
        if not REC["known"] or REC["invalid"] or REC["dropped"] or REC["stamp"] != stamp(objects):
            return 0.0
        if not (REC["clear"] and REC["still"] and REC["held"] >= HOLD_S - EPS):
            return 0.0
        return float(REC["value"])
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


# -- simulator hooks -------------------------------------------------------------------------------------------------


@lru_cache(maxsize=1)
def hand_samples():
    """Points on every collision face of each gripper link (link frame), at most 3 mm apart."""
    from yam_arena import analysis

    out = {}
    for name in ("gripper", "tip_left", "tip_right"):
        cloud = []
        for h in analysis.link_hulls().get(name, []):
            cloud.extend(h.vertices)
            for pl in np.unique(h.planes.round(7), axis=0):
                face = h.vertices[np.abs(h.vertices @ pl[:3] - pl[3]) < 2e-6]
                if len(face) < 3:
                    continue
                c = face.mean(0)
                u = face[0] - c
                u /= np.linalg.norm(u)
                v = np.cross(pl[:3], u)
                face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
                for a, b in zip(face, np.roll(face, -1, axis=0)):
                    n = max(1, math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / 0.003))
                    for i in range(n + 1):
                        for j in range(n - i + 1):
                            cloud.append(c + (a - c) * (i / n) + (b - c) * (j / n))
        if not cloud:
            raise ValueError("missing hand geometry")
        out[name] = np.unique(np.round(np.asarray(cloud), 4), axis=0)
    return out


def hand_clouds(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    clouds = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        body = parts._np(asset.data.body_link_pose_w)[0]
        if body.ndim != 2 or body.shape[1] != 7 or not np.isfinite(body).all():
            raise ValueError("unknown robot body poses")
        for name, pts in hand_samples().items():
            pose = body[asset.body_names.index(name)]
            clouds.append(pts @ _R(pose[3:]).T + pose[:3] - origin)
    return clouds


HAIR_ON_PAD = np.array([PAD_LINKS * LINK_L / 2 - LINK_L, 0.0, 0.0])  # the clump's origin on the pad's front link


def hair_pose(objects):
    """Where the declared rule puts the clump: at its place in the drain until caught, then on the pad."""
    if not REC["caught"]:
        return HAIR_AT, np.array([0.0, 0.0, 0.0, 1.0])
    lp, lq = objects[TOOL.name].links[LAST]
    r = _R(lq)
    return np.asarray(lp, float) + r @ HAIR_ON_PAD, np.asarray(lq, float)


def _put_hair(sim, objects):
    import torch

    base = sim.env.unwrapped
    asset = base.scene[HAIR_PART.name]
    origin = parts._np(base.scene.env_origins[0])
    p, q = hair_pose(objects)
    pose = [*(np.asarray(p) + origin), *q]
    asset.write_root_pose_to_sim(torch.tensor([pose], dtype=torch.float32, device=base.device))
    asset.write_root_velocity_to_sim(torch.zeros((1, 6), dtype=torch.float32, device=base.device))


@parts.on_reset
def _reset(sim):
    REC.clear()
    REC.update(_fresh())


@parts.on_step
def _step(sim):
    objects = sim.states()
    try:
        clouds = hand_clouds(sim)
    except (ValueError, AttributeError, KeyError, IndexError):
        clouds = None
    observe(objects, clouds, float(sim.t))
    try:
        _put_hair(sim, objects)
    except Exception as e:  # noqa: BLE001
        print(f"[hair] {e}", file=sys.stderr, flush=True)
    log = (str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("hair.jsonl")) if os.environ.get("TRACE_OUT")
           else os.environ.get("HAIR_LOG"))  # per-step log for traces only
    if log:
        tool, hair = objects[TOOL.name], objects[HAIR_PART.name]
        pts = tool_points(tool)
        row = dict(t=round(float(sim.t), 3), lot=lots.current().name, pad=np.round(pad_centre(tool) - [0, 0, TOP], 4).tolist(),
                   tip=np.round(pts[-1] - [0, 0, TOP], 4).tolist(), handle=np.round(pts[0] - [0, 0, TOP], 4).tolist(),
                   joints={k: round(float(v), 3) for k, v in tool.joints.items()},
                   hair=np.round(np.asarray(hair.pos) - [0, 0, TOP], 4).tolist(), caught=REC["caught"],
                   in_hair=round(REC["in_hair"], 2), value=REC["value"], held=round(REC["held"], 2),
                   clear=REC["clear"], still=REC["still"], dropped=REC["dropped"], score=score(objects),
                   rp=np.round(np.asarray(tool.pos) - [0, 0, TOP], 5).tolist(),
                   rq=np.round(np.asarray(tool.quat_xyzw), 5).tolist())
        try:
            row["scored"] = score(sim.states())  # what the step's scorer reads, after the clump is moved
        except Exception as e:  # noqa: BLE001  trace only
            row["scored_err"] = str(e)
        try:
            from yam_arena.constants import TCP_OFFSET
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            rob = base.scene["left_robot"]
            pose = parts._np(rob.data.body_link_pose_w)[0, rob.body_names.index("gripper")]
            row["tcp"] = np.round(pose[:3] - origin + _R(pose[3:]) @ np.array(TCP_OFFSET) - [0, 0, TOP], 4).tolist()
            row["gq"] = np.round(pose[3:], 5).tolist()
            row["fingers"] = np.round(parts._np(rob.data.joint_pos)[0, -2:], 4).tolist()
        except Exception as e:  # noqa: BLE001  trace only
            row["tcp_err"] = str(e)
        with open(log, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.item() if hasattr(o, "item") else str(o)) + "\n")




# -- trace-only diagnostics: the tool's pose during the reset's settling steps (TRACE_OUT set by gpu_run.sh trace) -----


def _debug_settle(module):
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_tool_dbg", False):
        return
    ensure = cls._ensure_sim

    def patched(self, *a, **k):
        out = ensure(self, *a, **k)
        env = self._env
        if env is not None and not getattr(env, "_tool_dbg", False):
            step, n = env.step, [0]

            def logged(action):
                res = step(action)
                n[0] += 1
                if n[0] <= 60:
                    try:
                        base = env.unwrapped
                        tool = base.scene[TOOL.name]
                        origin = parts._np(base.scene.env_origins[0])
                        pose = parts._np(tool.data.root_link_pose_w)[0]
                        for pn in ("probe_back", "probe_front"):
                            if pn in base.scene.keys():
                                pp = parts._np(base.scene[pn].data.root_link_pose_w)[0]
                                with pathlib.Path(os.environ["TRACE_OUT"]).with_name("settle.jsonl").open("a") as f:
                                    f.write(json.dumps(dict(n=n[0], probe=pn, z=round(float(pp[2] - origin[2] - TOP), 4))) + "\n")
                        row = dict(n=n[0], pos=np.round(pose[:3] - origin - [0, 0, TOP], 4).tolist(),
                                   quat=np.round(pose[3:], 3).tolist(), q=np.round(parts._np(tool.data.joint_pos)[0], 3).tolist(),
                                   v=np.round(parts._np(tool.data.root_link_lin_vel_w)[0], 3).tolist())
                    except Exception as e:  # noqa: BLE001
                        row = dict(n=n[0], err=str(e))
                    with pathlib.Path(os.environ["TRACE_OUT"]).with_name("settle.jsonl").open("a") as f:
                        f.write(json.dumps(row) + "\n")
                return res

            env.step = logged
            env._tool_dbg = True
        return out

    cls._ensure_sim = patched
    cls._tool_dbg = True


if os.environ.get("TRACE_OUT"):
    _name = "yam_arena.inspect_embodiment"
    _prev = parts._PATCHES[_name]
    parts._PATCHES[_name] = lambda m: (_prev(m), _debug_settle(m))
    if _name in sys.modules:
        _debug_settle(sys.modules[_name])


# -- scripted demo: open loop from the published layout of this trial ---------------------------------------------------
# The left arm takes the handle's grip from above, leaning 45 degrees forward, closing across it. It lifts the tool and
# turns it so the wand points down to the right at 55 degrees from the horizontal, toward the drain, the hinged end
# of the wand bending in the upright plane through the drain. It feeds the wand's end in under the left edge of the
# open stopper's head, then pushes down so the end curls down the drain's bore beside the stopper, into the hair, holds
# it there, and draws the wand back out along its own line. Last it turns the tool back to how it lay, lays it on the
# countertop where it was taken from, lets go and lifts clear. The right arm stays at rest.
GRASP_TILT = math.radians(30.0)
SQUEEZE = -(GRIP_R - 0.005)  # 5 mm past each side of the 26 mm grip
ELEV = math.radians(45.0)  # the wand's slope while it goes in
ENTRY = np.array([BX, BY, TOP + ZF]) - 0.0070 * np.array([math.cos(PSI), math.sin(PSI), 0.0])  # the straight wand's
# line crosses the flange's top here, 7 mm from the drain's axis
ARM = "left"


def _unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def grip_rot(a, c):
    """TCP rotation with approach ``a`` and fingers closing along ``c`` (the stock straight-down grip's column order)."""
    a = _unit(a)
    c = _unit(np.asarray(c, float) - a * (np.dot(c, a)))
    return np.column_stack([c, np.cross(a, c), a])


_A0 = np.array([math.sin(GRASP_TILT), 0.0, -math.cos(GRASP_TILT)])
R_GRASP = grip_rot(_A0, -np.cross(_A0, _R(LYING)[:, 0]))  # leaning forward, closing across the handle
TOOL_IN_HAND = R_GRASP.T @ _R(LYING)  # the tool's frame in the gripper's frame once it is held


def tool_rot_inserting():
    x = np.array([math.cos(PSI) * math.cos(ELEV), math.sin(PSI) * math.cos(ELEV), -math.sin(ELEV)])  # down to the drain
    y = np.array([-math.sin(PSI), math.cos(PSI), 0.0])  # the wand's hinges, level and across its slope
    return np.column_stack([x, y, np.cross(x, y)])


# Measured on the GPU (trace, line-up pose): under its own weight the held tool sits 4.4 degrees off the grip's pose,
# turned about this axis of the tool's frame (mostly its hinges' axis: the wand's end droops). The demo aims that much
# the other way, so the wand points where intended.
GRIP_SLIP_DEG, GRIP_SLIP_AXIS = 4.4, (0.085, 0.855, 0.51)
_SLIP = parts._rot(GRIP_SLIP_AXIS, math.radians(GRIP_SLIP_DEG))
R_INSERT = tool_rot_inserting() @ _SLIP.T @ TOOL_IN_HAND.T
# Laying it back, the wand's end meets the stone first and the tool turns about 8 degrees in the grip, swinging the
# wand's end back toward the basin (measured on the GPU); the demo lays it down turned 3 degrees the other way.
PLACE_YAW = -3.0
R_PLACE = parts._rot((0.0, 0.0, 1.0), math.radians(PLACE_YAW)) @ R_GRASP


def tip_at(r, z):
    """World point ``r`` from the drain's axis on the wand's side, ``z`` above the flange's top."""
    u = -np.array([math.cos(PSI), math.sin(PSI), 0.0])
    return np.array([BX, BY, TOP + ZF + z]) + u * r


# Measured on the GPU (trace, line-up pose): held at this slope the hinged end of the wand droops under its weight, its
# end 20 mm below the straight wand's line (tool frame). The demo aims the wand's end that much higher.
TIP_DROOP = np.array([-0.0017, 0.0, -0.020])


def tcp_for_tip(tip):
    """TCP that puts the (drooping) end of the wand at ``tip``."""
    R = tool_rot_inserting()
    return np.asarray(tip, float) - R @ TIP_DROOP - R[:, 0] * (WAND_L - GRIP_AT)


def tcp_inserting(s, down=0.0):
    """TCP with the straight wand's end ``s`` metres past ENTRY along its slope, then ``down`` lower."""
    t = tool_rot_inserting()[:, 0]
    return ENTRY + t * (s - WAND_L + GRIP_AT) + np.array([0.0, 0.0, -down])


def _w(d, p, finger, label, rot):
    return Waypoint(d, tuple(float(v) for v in p), finger, label, tilt._TiltedYaw(0.0, rot))


# The wand's end, as if the wand stayed straight: (r, z) = mm from the drain's axis on the wand's side, mm above the
# flange's top. It comes down on the flange outside the head, slides in under the head's edge to the bore, then the
# hand feeds the wand along its own slope so its end curls down the bore beside the stem.
TOUCH = (0.024, 0.012)  # over the flange, outside the head
PRESS = (0.024, -0.002)  # resting on the flange, pressed 2 mm
SLID = (0.012, -0.002)  # slid in under the head's edge, past the bore's edge
FEED = 0.068  # then this far along the wand's slope


# Laying back, the tool comes round with its wand's end tipped up 8 degrees, so the drooping end crosses the basin's rim
# above the stone instead of catching on the rim's inside edge (it did on the GPU, turning the tool in the grip).
LIFT_PITCH = -8.0
R_LIFT = parts._rot((1.0, 0.0, 0.0), math.radians(LIFT_PITCH)) @ R_PLACE


def _fed(s):
    t = tool_rot_inserting()[:, 0]
    return tip_at(*SLID) + t * s


def demo():
    lot = lots.current()
    start = np.asarray(lot.pos(TOOL.name), float)
    start[2] = REST_Z  # it lies on the stone by the time the trial starts
    side = _R(LYING)[:, 0]  # along the lying tool, from the handle toward the wand's end
    grip = start + _R(LYING) @ np.array([GRIP_AT, 0.0, 0.0]) + np.array([0.0, 0.0, 0.002])
    slide_in = grip - side * 0.060  # beyond the handle's end, level with it
    lift = grip + np.array([0.070, -0.040, 0.050])
    pre = tcp_for_tip(ENTRY - tool_rot_inserting()[:, 0] * 0.020)  # the wand's end just in under the head's edge
    plan = [
        _w(2.0, np.array([0.250, 0.400, TOP + 0.330]), FINGER_OPEN, "up and over to the left", R_GRASP),
        _w(1.5, slide_in + np.array([0.035, 0.016, 0.040]), FINGER_OPEN, "past the handle's end", R_GRASP),
        _w(1.5, slide_in + np.array([0.0, 0.0, 0.006]), FINGER_OPEN, "beside the handle's end", R_GRASP),
        _w(1.0, slide_in, FINGER_OPEN, "level with the handle", R_GRASP),
        _w(2.0, grip, FINGER_OPEN, "slide onto the grip", R_GRASP),
        _w(1.0, grip, SQUEEZE, "close on the grip", R_GRASP),
        _w(1.5, grip + np.array([0.020, 0.0, 0.015]), SQUEEZE, "lift off", R_GRASP),
        _w(2.0, lift, SQUEEZE, "lift", R_GRASP),
        _w(3.5, tcp_for_tip(tip_at(0.055, 0.020)), SQUEEZE, "turn the wand toward the drain", R_INSERT),
        _w(1.5, tcp_for_tip(tip_at(0.034, 0.010)), SQUEEZE, "end in low beside the flange", R_INSERT),
        _w(1.5, pre, SQUEEZE, "line up", R_INSERT),
    ]
    t = tool_rot_inserting()[:, 0]
    plan += [
        _w(2.0, tcp_for_tip(ENTRY + t * 0.012), SQUEEZE, "in under the stopper's edge", R_INSERT),
        _w(2.0, tcp_for_tip(ENTRY + t * 0.035), SQUEEZE, "feed down the bore", R_INSERT),
        _w(2.0, tcp_for_tip(ENTRY + t * FEED), SQUEEZE, "feed into the hair", R_INSERT),
        _w(1.5, tcp_for_tip(ENTRY + t * FEED), SQUEEZE, "hold in the hair", R_INSERT),
        _w(2.5, tcp_for_tip(ENTRY + t * 0.012), SQUEEZE, "draw back", R_INSERT),
        _w(2.0, tcp_for_tip(tip_at(0.036, 0.010)), SQUEEZE, "out of the drain", R_INSERT),
        _w(3.5, lift, SQUEEZE, "turn back over the counter, the wand's end tipped up", R_LIFT),
        _w(2.0, grip + np.array([0.020, 0.0, 0.015]), SQUEEZE, "over the counter's end", R_PLACE),
        _w(2.0, grip + np.array([0.0, 0.0, 0.004]), SQUEEZE, "lower onto the counter", R_PLACE),
        _w(1.0, grip + np.array([0.0, 0.0, 0.004]), FINGER_OPEN, "let go", R_PLACE),
        _w(2.0, slide_in + np.array([0.0, 0.0, 0.004]), FINGER_OPEN, "slide off the handle's end", R_PLACE),
        _w(1.5, slide_in + np.array([0.040, 0.030, 0.010]), FINGER_OPEN, "clear", R_PLACE),
    ]
    return {ARM: plan}
