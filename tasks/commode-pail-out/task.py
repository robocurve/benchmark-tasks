"""Take a used pail out from under a bedside commode's seat and set it on the floor in front of the commode without spilling it.

Drive's 11135-1 drop-arm bedside commode (a tube frame on four legs, its seat and lid, two pail brackets under the seat) and its 12 qt pail (Drive 11108) with its fold-down
plastic bail, free on two pivot lugs at the pail's ends. The pail hangs by its rim flange on the two brackets, under the seat, so it cannot be lifted out: as the manual has it, it
slides out the front (world -x, toward the robot), between the front legs. The frame's front tube keeps the bail from standing up over its pivots until the pail is 0.18 m out,
while its centre passes the brackets' ends at 0.12 m: held by a level bail alone it pitches forward there, so the other hand holds the rim's front up. The pail is one rigid body
of 1.25 kg plus the 0.05 kg bail, its contents part of it (no fluid): a spill is scored from its tilt and its landing speed.

    python demokit.py task.py OUT --no-record --mode full,none,one_hand,one_hand_rim,drag,leave,drop,undo,hold_on,drop_low15,drop_low19,drag2,tip_release
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Take the used pail out from under the commode seat and set it down on the floor in front of the commode without spilling it."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself
SCORE_OVER = "end"
TOP = d.TABLE_TOP  # the bench top is the floor

CX = float(os.environ.get("CP_CX", "0.70"))  # the commode's centre, set this far forward
CY = float(os.environ.get("CP_CY", "0.0"))  # and this far to the left. Its front (the pail's way out) faces the robot: local -y is world -x, local +x is world -y
YAW = -90.0
LOOKS = {
    "steel": m.Look(m.rgb(0.90, 0.91, 0.92), metallic=0.1, roughness=0.5),  # light grey powder coat
    "seat": m.Look(m.rgb(0.42, 0.44, 0.45), metallic=0.0, roughness=0.55),
    "seat2": m.Look(m.rgb(0.47, 0.49, 0.50), metallic=0.0, roughness=0.5),
    "pad": m.Look(m.rgb(0.30, 0.32, 0.34), metallic=0.0, roughness=0.7),
    "pail": m.Look(m.rgb(0.86, 0.86, 0.84), metallic=0.0, roughness=0.5),
    "lid": m.Look(m.rgb(0.35, 0.37, 0.40), metallic=0.0, roughness=0.5),
    "contents": m.Look(m.rgb(0.55, 0.42, 0.18), metallic=0.0, roughness=0.15),
    "foot": m.Look(m.rgb(0.25, 0.26, 0.27), roughness=0.9),
}

# -- shapes: rounded outlines, lofts and tubes (visuals; every collider is listed in the README) ----------------------------------------------------------------------------------------
def rrect(w, dp, r, n=6, cx=0.0, cy=0.0):
    """A rounded rectangle w x dp, corner radius r, counter-clockwise, 4 n points."""
    pts = []
    for k, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        for i in range(n):
            a = k * math.pi / 2 + (math.pi / 2) * i / (n - 1)
            pts.append((cx + sx * (w / 2 - r) + r * math.cos(a), cy + sy * (dp / 2 - r) + r * math.sin(a)))
    return np.array(pts)


def two_sided(mesh):
    return m.Mesh(mesh.v, np.vstack([mesh.f, mesh.f[:, ::-1]]))


def loft(rings, cap0=True, cap1=True):
    """Join rings of equal point count (each [n, 3]) side by side; cap the first and last with a fan."""
    n = len(rings[0])
    v = np.vstack(rings)
    f = []
    for k in range(len(rings) - 1):
        for i in range(n):
            j = (i + 1) % n
            f += [(k * n + i, k * n + j, (k + 1) * n + j), (k * n + i, (k + 1) * n + j, (k + 1) * n + i)]
    extra = []
    for cap, k in ((cap0, 0), (cap1, len(rings) - 1)):
        if cap:
            c = len(v) + len(extra)
            extra.append(rings[k].mean(axis=0))
            f += [(c, k * n + (i + 1) % n, k * n + i) for i in range(n)]
    if extra:
        v = np.vstack([v, np.array(extra)])
    return two_sided(m.Mesh(v, np.array(f)))


def at_z(outline, z):
    return np.c_[outline, np.full(len(outline), z)]


def plate(outer, inner, z0, z1):
    """A flat plate between two outlines of equal point count (inner may be None: a solid plate)."""
    if inner is None:
        return loft([at_z(outer, z0), at_z(outer, z1)])
    return m.join(two_sided(_ring(outer, inner, z1)), two_sided(_ring(outer, inner, z0)), loft([at_z(outer, z0), at_z(outer, z1)], False, False), loft([at_z(inner, z0), at_z(inner, z1)], False, False))


def _ring(outer, inner, z):
    n = len(outer)
    v = np.vstack([at_z(outer, z), at_z(inner, z)])
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [(i, j, n + j), (i, n + j, n + i)]
    return m.Mesh(v, np.array(f))


def seg(name, p0, p1, r, look="steel"):
    """A straight tube with its own convex collider."""
    mesh = m.tube([p0, p1], r, sides=12)
    return [(name, mesh, look)], [m.col_hull(name + "_c", m.tube([p0, p1], r, sides=8))]


def bent(name, pts, r, look="steel", bend=0.03):
    """A bent tube through corner points, rounded at each corner; one convex collider per straight run."""
    pts = [np.asarray(p, float) for p in pts]
    path = [pts[0]]
    for a, b, c in zip(pts[:-2], pts[1:-1], pts[2:]):
        u, w = (a - b) / np.linalg.norm(a - b), (c - b) / np.linalg.norm(c - b)
        for t in np.linspace(0.0, 1.0, 5):
            q0, q1 = b + bend * u, b + bend * w
            path.append((1 - t) ** 2 * q0 + 2 * (1 - t) * t * b + t ** 2 * q1)
    path.append(pts[-1])
    parts = [(name, m.tube(path, r, sides=12), look)]
    cols = [m.col_hull(f"{name}_c{k}", m.tube([a, b], r, sides=8)) for k, (a, b) in enumerate(zip(pts[:-1], pts[1:]))]
    return parts, cols


# -- the commode: Drive 11135-1 deluxe bariatric drop-arm commode, as in its manual's figure 1 (spec sheet: 26.25 in outside, 25.75 in between arms, seat 23.25 x 18.5 in,
# seat height 17.5 to 22 in; Drive's page: 26 in between legs; retailers: 21 in deep, 33 in high). Frame fixed (rubber tips on the floor). Heights above the floor, seat at its lowest.
# Local y: - is the front, where the pail slides out. Sizes not published are estimates.
FRAME_W, FRAME_D = 0.667, 0.533  # outside width 26.25 in, depth 21 in
TUBE = 0.0125  # 25 mm steel tube (estimate)
LX, LY = FRAME_W / 2 - TUBE, FRAME_D / 2 - TUBE  # leg centres: 0.642 apart across, 0.508 front to back
SEAT_W, SEAT_D = 0.591, 0.470  # 23.25 x 18.5 in
FZ = 0.4125  # the seat frame's tubes, 0.40 to 0.425 up
RING_Z0, RING_Z1 = 0.425, 0.445  # the seat, 20 mm thick (estimate), top at 17.5 in
LID_T = 0.015
RAIL_X, RAIL_W = 0.157, 0.020  # pail brackets under the flange's side strips (estimates)
RAIL_Y0, RAIL_Y1 = -0.230, LY - TUBE  # from 5 mm behind the seat's front edge to the back tube; open at their front ends
RAIL_Z0, RAIL_Z1 = 0.365, 0.375  # 10 mm flat bars: the bail's pivot lugs pass under them
ARM_Z = 0.640  # arm tubes, up (estimate)
ARM_X = 0.352  # arm centres: 25.75 in (0.654) between the arms' pads, 50 mm across (pad size an estimate)
BACK_Z1 = 0.838  # 33 in overall height


def commode() -> m.Part:
    parts, cols = [], []

    def add(pc):
        parts.extend(pc[0])
        cols.extend(pc[1])

    for s in (-1.0, 1.0):
        x = s * LX
        q = "p" if s > 0 else "n"  # prim names: no minus sign
        for yy, tag in ((-LY, "f"), (LY, "b")):
            add(seg(f"leg{tag}{q}", (x, yy, 0.03), (x, yy, FZ), TUBE))  # leg (an inner extension tube in the real frame, locked by a snap button)
            parts.append((f"tip{tag}{q}", m.cylinder(0.017, 0.035, (x, yy, 0.0175)), "foot"))
            cols.append(m.col_cylinder(f"tip{tag}{q}_c", 0.017, 0.035, (x, yy, 0.0175)))
        add(seg(f"side{q}", (x, -LY, FZ), (x, LY, FZ), TUBE))  # side tube of the seat frame
        add(seg(f"bracef{q}", (x, -LY, 0.20), (x, -0.06, FZ - 0.01), 0.009))  # diagonal side braces
        add(seg(f"braceb{q}", (x, LY, 0.20), (x, 0.06, FZ - 0.01), 0.009))
        add(seg(f"tie{q}", (x, -LY, 0.10), (x, LY, 0.10), 0.009))  # low side tie
        # pail bracket: a flat steel bar along y, hung from the side tube by two cross arms
        parts.append((f"rail{q}", m.box((RAIL_W, RAIL_Y1 - RAIL_Y0, RAIL_Z1 - RAIL_Z0), (s * RAIL_X, (RAIL_Y0 + RAIL_Y1) / 2, (RAIL_Z0 + RAIL_Z1) / 2)), "steel"))
        cols.append(m.col_box(f"rail{q}_c", (RAIL_W, RAIL_Y1 - RAIL_Y0, RAIL_Z1 - RAIL_Z0), (s * RAIL_X, (RAIL_Y0 + RAIL_Y1) / 2, (RAIL_Z0 + RAIL_Z1) / 2)))
        for k, yy in enumerate((0.0, 0.20)):
            x0, x1 = RAIL_X + RAIL_W / 2, LX
            sz = (x1 - x0, 0.020, RAIL_Z1 - RAIL_Z0)
            c = (s * (x0 + x1) / 2, yy, (RAIL_Z0 + RAIL_Z1) / 2)
            parts.append((f"rarm{q}{k}", m.box(sz, c), "steel"))
            cols.append(m.col_box(f"rarm{q}{k}_c", sz, c))
            sz2, c2 = (0.012, 0.020, FZ - RAIL_Z1), (s * LX, yy, (FZ + RAIL_Z1) / 2)
            parts.append((f"rstrap{q}{k}", m.box(sz2, c2), "steel"))
            cols.append(m.col_box(f"rstrap{q}{k}_c", sz2, c2))
        # drop arm, up: a bent tube loop from the seat frame, with its foam pad
        ax = s * ARM_X
        add(bent(f"arm{q}", [(ax, -LY + 0.03, FZ), (ax, -LY + 0.03, ARM_Z), (ax, LY - 0.02, ARM_Z), (ax, LY - 0.02, FZ)], TUBE))
        pad = m.cylinder(0.025, 0.26, (ax, -0.06, ARM_Z + 0.012), axis="y")
        parts.append((f"pad{q}", pad, "pad"))
        cols.append(m.col_hull(f"pad{q}_c", pad))
    add(seg("frontbar", (-LX, -LY, FZ), (LX, -LY, FZ), TUBE))  # the seat frame's front tube, over the pail's way out
    add(seg("backbar", (-LX, LY, FZ), (LX, LY, FZ), TUBE))
    add(seg("tieb", (-LX, LY, 0.10), (LX, LY, 0.10), 0.009))
    # back rest: the back legs run on up into a loop, with a half-moon pad
    add(bent("backloop", [(-LX, LY, FZ), (-LX, LY + 0.02, BACK_Z1 - TUBE), (LX, LY + 0.02, BACK_Z1 - TUBE), (LX, LY, FZ)], TUBE, bend=0.06))
    th = np.linspace(0.0, math.pi, 13)
    moon = np.array([(0.26 * math.cos(t), 0.13 * math.sin(t)) for t in -th])  # from (0.26, 0) round the bottom to (-0.26, 0): straight along the top
    back = plate(moon, None, -0.015, 0.015).turned(90, "x").moved((0.0, LY + 0.035, BACK_Z1 - 0.05))
    parts.append(("backpad", back, "seat"))
    cols.append(m.col_hull("backpad_c", back))
    # the moulded seat with its round opening, and its lid, closed
    so = rrect(SEAT_W, SEAT_D, 0.045, n=8)
    oc = np.array([0.0, -0.06])
    ang = np.arctan2(so[:, 1] - oc[1], so[:, 0] - oc[0])
    hole = np.c_[oc[0] + 0.125 * np.cos(ang), oc[1] + 0.135 * np.sin(ang)]
    parts.append(("seat", plate(so, hole, RING_Z0, RING_Z1), "seat"))
    cols.append(m.col_box("seat_c", (SEAT_W - 0.02, SEAT_D - 0.02, RING_Z1 - RING_Z0), (0.0, 0.0, (RING_Z0 + RING_Z1) / 2)))  # one box: its opening lies under the closed lid
    lid = rrect(SEAT_W - 0.02, SEAT_D - 0.03, 0.045, n=8)
    parts.append(("lid", plate(lid, None, RING_Z1, RING_Z1 + LID_T), "seat2"))
    cols.append(m.col_box("lid_c", (SEAT_W - 0.02, SEAT_D - 0.03, LID_T), (0.0, 0.0, RING_Z1 + LID_T / 2)))
    return m.fixed("commode", (CX, CY, TOP), parts, cols, LOOKS, yaw_deg=YAW)


# -- the pail: Drive's 12 qt commode bucket (retail listing 9.8 x 13.7 x 8.5 in; its shape, the handle and the mass are not published, so these are estimates), tapered and round-cornered
# as in the manual's figure. Origin at the middle of its bottom's underside, local x across the commode, local -y the handle's side (the front)
BODY_X, BODY_Y, WALL = 0.144, 0.0945, 0.004  # outer half sizes of the body at its rim: 0.288 x 0.189; 0.24 x 0.15 at the bottom
BOT_X, BOT_Y = 0.120, 0.075
FL_X, FL_Y, FL_T = 0.174, 0.125, 0.010  # flange: 0.348 x 0.25 over its rim, 10 mm thick
DEPTH = 0.190
PAIL_MASS = 1.3  # 0.5 kg empty (the bail's 0.05 kg of it) + 0.8 kg of contents
PAIL_Y = -0.110  # the pail's centre in the commode (local y): its rim's front flush with the seat's front edge
START = np.array([CX + PAIL_Y, CY, TOP + RAIL_Z1 + FL_T - DEPTH])  # world  # the flange rests on the brackets: its top is 10 mm over them

# -- its handle: the 11108's fold-down plastic bail (Drive's photos): a moulded band that pivots on two lugs at the pail's ends, below the rim's end tabs, with a ridged grip at its middle.
# Free to swing about the line through the lugs (the pail's local x, across the commode); its swing stops where the band meets the pail's wall, set as the joint's limits (a link does not
# collide with its own pail). Sizes from the photos, scaled by the listed 13.7 in length: estimates.
PIV_Z = 0.150  # the lugs' axis, 40 mm under the rim's top
PIV_X = 0.185  # the bail's ends, just outside the rim's end tabs (0.174)
BAIL_H = 0.150  # from the lugs' axis to the grip's centre
BAIL_W, BAIL_T = 0.014, 0.006  # the band: 14 mm deep (in its plane), 6 mm thick
GRIP_R, GRIP_L = 0.008, 0.12  # the ridged centre grip: 16 mm round, 120 mm long
FOLD = 142.0  # at rest the bail lies folded forward, 38 degrees off straight down, its grip against the pail's front wall (joint zero)
BAIL_MASS = 0.05


def bail_line(n=17):
    """The bail's centreline standing up (in the pail's x-z plane), a squared arch from one lug to the other."""
    t = np.linspace(0.0, math.pi, n)
    c, s = np.cos(t), np.sin(t)
    return np.c_[PIV_X * np.sign(c) * np.abs(c) ** (2 / 3), np.zeros(n), PIV_Z + BAIL_H * np.abs(s) ** (2 / 3)]


def folded(mesh, deg=FOLD):
    """Turn a part of the bail about the lugs' axis, from standing up to `deg` toward the front (local -y)."""
    return mesh.moved((0.0, 0.0, -PIV_Z)).turned(deg, "x").moved((0.0, 0.0, PIV_Z))


def grip_local(joint_rad: float) -> np.ndarray:
    """The grip's centre in the pail's frame, with the bail's joint at `joint_rad` (0: folded forward at rest; -FOLD degrees: standing up)."""
    th = math.radians(FOLD) + joint_rad
    return np.array([0.0, -BAIL_H * math.sin(th), PIV_Z + BAIL_H * math.cos(th)])


def bail() -> m.Link:
    pts = bail_line()
    parts, cols = [], []
    for k, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
        if abs(0.5 * (a[0] + b[0])) < GRIP_L / 2 - 0.02:
            continue  # the grip
        L = float(np.linalg.norm(b - a)) + 0.004
        ang = math.degrees(math.atan2(b[2] - a[2], b[0] - a[0]))
        seg_ = m.box((L, BAIL_T, BAIL_W), (0.0, 0.0, 0.0)).turned(-ang, "y").moved(0.5 * (a + b))
        parts.append((f"band{k}", folded(seg_), "pail"))
        cols.append(m.col_hull(f"band{k}_c", folded(seg_)))
    top = (0.0, 0.0, PIV_Z + BAIL_H)
    g = m.cylinder(GRIP_R, GRIP_L, top, axis="x", segments=16)
    parts.append(("grip", folded(g), "pail"))
    cols.append(m.col_hull("grip_c", folded(g)))
    for i in range(7):  # the grip's ridges
        parts.append((f"ridge{i}", folded(m.cylinder(GRIP_R + 0.0015, 0.004, (-0.042 + 0.014 * i, 0.0, PIV_Z + BAIL_H), axis="x", segments=16)), "pail"))
    com = folded(m.Mesh(pts, np.zeros((0, 3), int))).v.mean(axis=0)
    return m.hinge("bail", "X", (0.0, 0.0, PIV_Z), -284.0, 0.0, parts, cols, mass=BAIL_MASS, gravity=True, com=tuple(com), damping=0.002, friction=0.002,
                   inertia=(4e-4, 2e-4, 4e-4))


def pail() -> m.Part:
    out_b, out_t = rrect(2 * BOT_X, 2 * BOT_Y, 0.03), rrect(2 * BODY_X, 2 * BODY_Y, 0.035)
    in_b, in_t = rrect(2 * BOT_X - 2 * WALL, 2 * BOT_Y - 2 * WALL, 0.026), rrect(2 * BODY_X - 2 * WALL, 2 * BODY_Y - 2 * WALL, 0.031)
    body = loft([at_z(out_b, 0.0), at_z(out_t, DEPTH - FL_T), at_z(in_t, DEPTH - FL_T), at_z(in_b, WALL)])
    flange = plate(rrect(2 * FL_X, 2 * FL_Y, 0.03), out_t, DEPTH - FL_T, DEPTH)
    cf = rrect(2 * BOT_X - 2 * WALL - 0.006, 2 * BOT_Y - 2 * WALL - 0.006, 0.024)
    parts = [("body", body, "pail"), ("flange", flange, "pail"), ("contents", plate(cf, None, WALL, WALL + 0.016), "contents")]
    hull = loft([at_z(out_b, 0.0), at_z(out_t, DEPTH - FL_T)])
    cols = [m.col_hull("body_c", hull)]  # the body as one convex solid (uniform density; the contents are not a separate mass)
    for n, sz, c in (("fl_f", (2 * FL_X, FL_Y - BODY_Y + WALL, FL_T), (0, (FL_Y + BODY_Y - WALL) / 2, DEPTH - FL_T / 2)),
                     ("fl_b", (2 * FL_X, FL_Y - BODY_Y + WALL, FL_T), (0, -(FL_Y + BODY_Y - WALL) / 2, DEPTH - FL_T / 2)),
                     ("fl_l", (FL_X - BODY_X + WALL, 2 * BODY_Y - 2 * WALL, FL_T), (-(FL_X + BODY_X - WALL) / 2, 0, DEPTH - FL_T / 2)),
                     ("fl_r", (FL_X - BODY_X + WALL, 2 * BODY_Y - 2 * WALL, FL_T), ((FL_X + BODY_X - WALL) / 2, 0, DEPTH - FL_T / 2))):
        cols.append(m.col_box(n + "_c", sz, c))
    for s, q in ((-1.0, "n"), (1.0, "p")):  # the pivot lugs, moulded on the end walls under the rim's end tabs
        lug = m.cylinder(0.008, PIV_X - 0.004 - 0.136, (s * (0.136 + PIV_X - 0.004) / 2, 0.0, PIV_Z), axis="x", segments=16)
        parts.append((f"lug{q}", lug, "pail"))
        cols.append(m.col_hull(f"lug{q}_c", lug))
    return m.mechanism("pail", tuple(START), [bail()], LOOKS, frame_parts=parts, frame_colliders=cols, yaw_deg=YAW, friction=(0.6, 0.5), free_mass=PAIL_MASS - BAIL_MASS,
                       frame_inertia=(0.010, 0.016, 0.019))


def cover() -> m.Part:
    """The pail's cover, off, on the floor beside the commode's right side, to be put on once the pail is out."""
    o = rrect(2 * FL_X, 2 * FL_Y, 0.03)
    parts = [("cover", plate(o, None, 0.0, 0.012), "pail"), ("knob", m.box((0.06, 0.02, 0.015), (0.0, 0.0, 0.0195)), "pail")]
    cols = [m.col_box("cover_c", (2 * FL_X, 2 * FL_Y, 0.012), (0.0, 0.0, 0.006))]
    return m.fixed("cover", (CX, CY - 0.52, TOP), parts, cols, LOOKS, yaw_deg=YAW)


OBJECTS = [commode(), pail(), cover()]

# -- the scorer ---------------------------------------------------------------------------------------------------------------------------------------------------------------------
SPLASH_DEG, LAND_SPEED, DOWN_DEG = 15.0, 0.30, 10.0
TOUCH = 0.010  # a corner of the bottom is armed again once it is over 10 mm up
CONTACT = 0.002  # first floor contact: a corner of the bottom under 2 mm from the floor
DOWN_STEPS = 20  # 1 s at 20 Hz
RAIL_END = CX + RAIL_Y0  # the brackets' open front ends (world x)
ZONE = RAIL_END - 0.04  # DOWN: the flange's back edge (world x) in front of this line
CLEAR = 0.030
fl = [(sx * FL_X, sy * FL_Y, DEPTH) for sx in (-1, 1) for sy in (-1, 1)]
PAIL_PTS = np.array(fl + [(sx * BOT_X, sy * BOT_Y, 0.0) for sx in (-1, 1) for sy in (-1, 1)] + [(sx * FL_X, 0.0, DEPTH) for sx in (-1, 1)] + [(0.0, sy * FL_Y, DEPTH) for sy in (-1, 1)]
                    + [(0.0, 0.0, 0.0)])
BAIL_PTS = bail_line(33)  # the bail's centreline standing up; posed by its joint for the hands-clear check


def bail_world(h) -> np.ndarray:
    """The bail's centreline in the world, from the pail's pose and its joint."""
    th = math.radians(FOLD) + float(h.joints.get("bail_joint", 0.0))
    c, s = math.cos(th), math.sin(th)
    p = BAIL_PTS - np.array([0.0, 0.0, PIV_Z])
    p = p @ np.array([[1, 0, 0], [0, c, -s], [0, s, c]]).T + np.array([0.0, 0.0, PIV_Z])
    return h.to_world(p)
FLANGE_PTS = np.array(fl)


def _hull_surface(step=0.004) -> np.ndarray:
    """Points 4 mm apart over the whole outer surface of the pail's solid envelope (rim plane to bottom), in its own frame: a hand's distance to the pail, not to 13 points."""
    top = [(sx * FL_X, sy * FL_Y) for sx in (-1, 1) for sy in (-1, 1)]
    bot = [(sx * BOT_X, sy * BOT_Y) for sx in (-1, 1) for sy in (-1, 1)]
    pts = []

    def quad(a, b, c, dq):
        a, b, c, dq = (np.asarray(x, float) for x in (a, b, c, dq))
        n = max(2, int(max(np.linalg.norm(b - a), np.linalg.norm(dq - a)) / step))
        for u in np.linspace(0, 1, n):
            for v in np.linspace(0, 1, n):
                pts.append(a * (1 - u) * (1 - v) + b * u * (1 - v) + c * u * v + dq * (1 - u) * v)
    T = lambda i: (*top[i], DEPTH)
    B = lambda i: (*bot[i], 0.0)
    # corners: 0 (-,-), 1 (-,+), 2 (+,-), 3 (+,+). Faces: bottom, rim plane, four slanted sides
    quad(B(0), B(1), B(3), B(2))
    quad(T(0), T(1), T(3), T(2))
    for i, j in ((0, 1), (1, 3), (3, 2), (2, 0)):
        quad(B(i), B(j), T(j), T(i))
    return np.array(pts)


BODY_SURF = _hull_surface()


def body_distance(h, tips) -> float:
    """The least distance from any point of a whole gripper to the pail's whole outer surface (flange, walls, bottom)."""
    loc = h.to_local(tips)
    return float(min(np.min(np.linalg.norm(loc[i:i + 32, None, :] - BODY_SURF[None, :, :], axis=2)) for i in range(0, len(loc), 32)))
BOTTOM_PTS = np.array([(sx * BOT_X, sy * BOT_Y, 0.0) for sx in (-1, 1) for sy in (-1, 1)])


class Trial:
    def reset(self, env=None) -> None:
        # only failures latch (splash, dropped, tipped, hard; moved is only a precondition for them). drawn, out and down are read from the state now, so undoing a step takes the credit back
        self.moved, self.splash, self.dropped, self.tipped, self.hard = (gk.Latch(n) for n in ("moved", "splash", "dropped", "tipped", "hard"))
        self.out = self.drawn = False
        self.held = 0  # running count of steps the end state has held
        self.dist, self.still, self.low = 9.0, False, 9.0
        self.down, self.n = False, 0
        self.tilt, self.peak = 0.0, 0.0
        self.substeps, self.warned, self.land = 0, False, 0.0
        self._pv, self._armed = None, np.ones(4, dtype=bool)
        self.parts = ((9.0, 9.0), (9.0, 9.0))
        if env is not None:
            self.step(env)

    def step(self, env) -> None:
        h = m.state(env, "pail")
        ft, hb = m.fingertips(env), m.hand_body(env)
        nb = len(hb) // 2
        hands = (np.concatenate([ft[:12], hb[:nb]]), np.concatenate([ft[12:], hb[nb:]]))  # the whole gripper of each arm: fingertips, pads, palm, housing and the wrist link
        pts = h.to_world(PAIL_PTS)
        tilt = h.tilt_deg()
        self.tilt = tilt
        self.moved.trip(float(np.linalg.norm(h.pos - START)) > 0.005)
        if bool(self.moved):
            self.peak = max(self.peak, tilt)
            self.splash.trip(tilt > SPLASH_DEG)
        self.tipped.trip(tilt > 70.0)
        self.drawn = bool(START[0] - h.pos[0] >= 0.050 and tilt < 70.0)
        fw = h.to_world(FLANGE_PTS)
        self.out = bool(float(np.max(fw[:, 0])) < RAIL_END - 0.002)
        bot = h.to_world(BOTTOM_PTS)
        low = float(np.min(bot[:, 2]))
        bw = bail_world(h)  # hands clear of the pail's whole body and of its bail
        dh = (body_distance(h, hands[0]), body_distance(h, hands[1]))  # left, right
        db = tuple(float(np.min(np.linalg.norm(hp[:, None, :] - bw[None, :, :], axis=2))) for hp in hands)
        self.parts = (dh, db)
        dist = min(dh + db)
        # a landing faster than 0.3 m/s splashes. The fall is read after every physics step (see _hook) and here at each control step
        self._fall(h, low)
        self._hook(env)
        on_rails = abs(h.pos[2] - START[2]) < 0.004 and tilt < 3.0 and not bool(self.out)
        if bool(self.moved) and not on_rails and dist > 0.025 and low > TOP + 0.020 and float(h.lin_vel[2]) < -0.10:
            self.dropped.trip(True)
        still = float(np.linalg.norm(h.lin_vel)) < 0.01 and float(np.linalg.norm(h.ang_vel)) < 0.2
        self.dist, self.still, self.low = dist, still, low - TOP
        ok = low <= TOP + 0.005 and float(np.max(fw[:, 0])) < ZONE and tilt <= DOWN_DEG and dist >= CLEAR and still
        self.held = self.held + 1 if (ok and bool(self.moved)) else 0
        self.down = self.held >= DOWN_STEPS  # DOWN holds for the last 1 s of the run so far; a pail lifted again, or a hand back on it, drops it at once
        self.n += 1
        if self.n % 20 == 0 and os.environ.get("CP_TRACE"):
            print(f"[cp] step {self.n}: pos {h.pos[0]:.3f} {h.pos[1]:.3f} {h.pos[2] - TOP:.3f}, tilt {tilt:.1f}, low {low - TOP:.3f}, hand {dist:.3f}, "
                  f"flags m{bool(self.moved)} d{self.drawn} o{self.out} S{bool(self.splash)} D{bool(self.dropped)} T{bool(self.tipped)} down{self.down}", flush=True)

    def _fall(self, h, low) -> None:
        """HARD: the speed at first floor contact. Once moved, each bottom corner is armed while it is over 10 mm up; the first step it is under 2 mm from the floor, its downward speed (the pail's linear velocity plus its angular velocity cross the corner's arm; the larger of that step's and the step before's, as contact already slows it) over LAND_SPEED is a hard landing. A pail let go tipped on one edge that slaps flat is HARD; speed higher up does not count."""
        if not bool(self.moved):
            return
        arm = h.to_world(BOTTOM_PTS) - h.pos
        z = h.to_world(BOTTOM_PTS)[:, 2] - TOP
        vz = h.lin_vel[2] + np.cross(h.ang_vel, arm)[:, 2]
        prev = vz if self._pv is None else self._pv
        hit = self._armed & (z < CONTACT)
        if bool(np.any(hit)):
            v = float(-np.min(np.minimum(vz, prev)[hit]))
            self.land = max(self.land, v)
            if v > LAND_SPEED:
                self.hard.trip(True)
                self.splash.trip(True)
        self._armed = (self._armed & ~(z < CONTACT)) | (z > TOUCH)
        self._pv = vz

    def _hook(self, env) -> None:
        """Read the fall after every physics step too (a control step holds several): a low drop lasts a few tens of ms and slips between 20 Hz samples."""
        scene = env.unwrapped.scene
        if getattr(scene, "_cp_hooked", None) is self:
            return
        orig = scene.update
        trial = self

        def update(*a, **k):
            r = orig(*a, **k)
            try:
                hh = m.state(env, "pail")
                trial._fall(hh, float(np.min(hh.to_world(BOTTOM_PTS)[:, 2])))
                trial.substeps += 1
            except Exception as e:  # the extra read must not break the run
                if not trial.warned:
                    trial.warned = True
                    print(f"[cp] substep read failed: {e!r}", flush=True)
            return r

        scene.update = update
        scene._cp_hooked = self

    def tier(self) -> float:
        """The tier of the state now: failures latch (splash, dropped, tipped, hard); drawn, out and down do not."""
        if bool(self.dropped) or bool(self.tipped) or bool(self.hard):
            return 0.0
        if self.down and not bool(self.splash):
            return 1.0
        if self.down:
            return 0.4  # set down after a splash
        if self.out and not bool(self.splash):
            return 0.5  # drawn clear and held level, not yet down
        if self.drawn:
            return 0.25
        return 0.0


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.tier() >= 1.0


def score(objects) -> float:
    return TRIAL.tier()


# -- the demo -----------------------------------------------------------------------------------------------------------------------------------------------------------------------
X, Y, Z = np.eye(3)
FLOOR_C = np.array([CX - 0.415, CY])  # the pail's centre when set down: its back 20 mm in front of the DOWN line
D_LEFT = float(os.environ.get("CP_DLEFT", "0.09"))  # drawn this far by the bail, the rim's front strip is clear of the frame's front tube: the left takes it
D_RAISE = float(os.environ.get("CP_DRAISE", "0.18"))  # drawn this far, the lugs are in front of the front tube and the bail can stand up over them
D_OUT = float(os.environ.get("CP_DOUT", "0.31"))  # drawn this far, the flange's back has passed under the front tube (OUT from 0.247)
RROLL = float(os.environ.get("CP_RROLL", "90"))  # jaws closing up and down on the grip; the wrist camera below the hand, clear of the front tube
LROLL = float(os.environ.get("CP_LROLL", "90"))
OFF = float(os.environ.get("CP_OFF", "0.010"))  # while swinging the bail up, the grip rides on the lower jaw: the hand this far over the grip's centre
BACK = float(os.environ.get("CP_BACK", "0.004"))  # and this far in front of it, so the fingertips stay off the pail's wall
GAIN = float(os.environ.get("CP_GAIN", "0.01"))
GRV = d._as_grip(d.level(0.0).rot @ d.axis_angle_matrix(Z, math.radians(RROLL)))
GLV = d._as_grip(d.level(0.0).rot @ d.axis_angle_matrix(Z, math.radians(LROLL)))
PINCH_F = np.array([-0.11, -(FL_Y + BODY_Y - WALL) / 2, DEPTH - FL_T / 2])  # the left hand's pinch: the rim's front strip, left of the bail's grip
OPEN_R, OPEN_L = -0.020, -0.016  # jaws 40 and 32 mm apart going in, under the front tube and over the bail's band


def report(tag):
    print(f"[cp] {tag}: peak tilt {TRIAL.peak:.1f}, tilt {TRIAL.tilt:.1f}, moved {bool(TRIAL.moved)} drawn {TRIAL.drawn} out {TRIAL.out} splash {bool(TRIAL.splash)} dropped {bool(TRIAL.dropped)} tipped {bool(TRIAL.tipped)} hard {bool(TRIAL.hard)} landing {TRIAL.land:.3f} down {TRIAL.down} (hand {TRIAL.dist:.3f} [hull L,R {TRIAL.parts[0][0]:.3f},{TRIAL.parts[0][1]:.3f} bail L,R {TRIAL.parts[1][0]:.3f},{TRIAL.parts[1][1]:.3f}] still {TRIAL.still} low {TRIAL.low:.3f})", flush=True)


def demo_script(ctx):
    """Modes: full (the right hand swings the bail up level under the front tube and draws the pail by it; the left takes the rim's front before the pail's centre reaches the brackets'
    ends and keeps it level; both draw it clear and set it down); none; one_hand (the right alone, by the bail, standing it up as soon as the front tube allows); drag (as one_hand, then dragged
    along the floor); one_hand_rim (the right draws it 90 mm by the bail and lets the bail go; the left alone takes the rim's front strip, draws it out and sets it down); leave (drawn 100 mm by the bail and let go on the brackets); drop (as full, let go 12 cm over the floor); drop_low15 and drop_low19 (as full, let go 15 and 19 mm over the floor); hold_on (as full, but only the right lets go: the left keeps its pinch on the rim); tip_gentle (as tip_release with a 5 mm lift; must score 1.0); slow_late (as full, but lowered fast to 40 mm up, braked to a stop, set down at 0.05 m/s at most, then eased level like full before letting go; must score 1.0); drag2 (as full to the set-down, both still holding, then dragged 10 cm along the floor, then let go); undo (as full to the set-down; both let go for 0.5 s, then the left takes
    the rim again and lifts it 8 cm and 20 cm back toward the brackets)."""
    if ctx.mode not in ("full", "none", "one_hand", "one_hand_rim", "drag", "leave", "drop", "undo", "hold_on", "drop_low15", "drop_low19", "drag2", "tip_release", "tip_gentle", "slow_late"):
        raise ValueError(f"unknown mode {ctx.mode!r}")
    l, r = ctx.left, ctx.right
    if ctx.mode == "none":
        yield from l.wait(3.0)
        return
    P = lambda: ctx.state("pail")
    th = lambda: math.degrees(math.radians(FOLD) + P().joint("bail"))  # the bail's angle from standing up, toward the front
    grip_at = lambda deg: P().to_world(grip_local(math.radians(deg - FOLD)))[0]
    pivot = lambda: P().to_world(np.array([0.0, 0.0, PIV_Z]))[0]
    pinch_f = lambda: P().to_world(PINCH_F)[0]
    low = lambda: float(np.min(P().to_world(BOTTOM_PTS)[:, 2])) - TOP
    drawn = lambda: START[0] - P().pos[0]
    # 1. the right hand goes in under the folded bail's grip, jaws apart, and swings the bail up and forward to level, the grip riding on its lower jaw
    ride = lambda deg: grip_at(deg) - BACK * X + OFF * Z
    yield from r.move(ride(FOLD) - 0.10 * X, grip=GRV, finger=OPEN_R, t=3.0, label="R in front of the bail")
    yield from r.servo(ride(FOLD) - 0.10 * X, timeout=1.0)
    yield from r.move(ride(FOLD), t=1.5, label="R under the grip")
    yield from r.servo(ride(FOLD), timeout=1.0)
    n = round(2.5 * d.CONTROL_HZ)
    for k in range(1, n + 1):
        deg = FOLD + (90.0 - FOLD) * d.ik.smoothstep(k / n)
        r._solve(ride(deg) + r.trim, r.grip.rot)
        yield
    print(f"[cp] bail swung to {th():.1f} deg", flush=True)
    yield from r.close(2 * GRIP_R, force=40.0)
    print(f"[cp] bail held at {th():.1f} deg", flush=True)
    if ctx.mode == "leave":
        yield from r.move(r.at() - 0.10 * X, t=3.0, label="R draws it forward")
        yield from r.open()
        yield from r.move(r.at() - 0.08 * X, t=1.0)
        yield from r.wait(2.0)
        return
    if ctx.mode == "one_hand_rim":
        # 2b. one hand by the rim: the right draws it 90 mm by the bail, lets the bail go and moves away; the left alone takes the rim's front strip,
        # draws it out off the brackets and sets it down, with no hand on the bail
        yield from r.move(r.at() - D_LEFT * X, t=3.0 + 20 * D_LEFT, label="R draws it forward")
        yield from r.open()
        yield from r.move(r.at() - 0.08 * X, t=1.0, label="R lets the bail go")
        yield from r.move(r.at() - 0.10 * X - 0.10 * Y + 0.05 * Z, t=1.5, label="R away")
        report("bail let go")
        aim = pinch_f()
        yield from l.move(aim - 0.10 * X, grip=GLV, finger=OPEN_L, t=3.5, label="L in front of the rim")
        yield from l.servo(aim - 0.10 * X, timeout=1.0)
        yield from l.move(pinch_f(), t=1.5, label="L on the rim")
        yield from l.servo(pinch_f(), timeout=1.0)
        yield from l.close(FL_T, force=40.0)
        report("left on the rim (one hand)")
        yield from l.move(l.at() - (D_OUT - drawn()) * X, t=4.5, label="L draws it clear")
        report("clear (one hand by the rim)")
        yield from l.move(l.at() + np.array([FLOOR_C[0] - P().pos[0], 0.0, -(low() - 0.002)]), t=5.0, label="L sets it down")
        report("set down (one hand by the rim)")
        yield from l.open()
        yield from l.move(l.at() - 0.045 * X, t=1.5, label="L backs off")
        yield from l.move(l.at() + 0.08 * Z, t=1.5, label="L clears")
        yield from l.wait(2.0)
        report("end (one hand by the rim)")
        return
    if ctx.mode in ("one_hand", "drag"):
        # 2a. the right alone draws it until the lugs pass the front tube, stands the bail up over them, lifts it off and carries it out and down
        yield from r.move(r.at() - D_RAISE * X, t=3.0 + 20 * D_RAISE, label="R draws it forward")
        report("drawn (one hand)")
        yield from r.arc(pivot(), Y, th(), t=2.5, label="R stands the bail up")
        report("bail up (one hand)")
        yield from r.move(r.at() + 0.006 * Z, t=1.0, label="R lifts it off")
        yield from r.move(r.at() - (D_OUT - drawn()) * X, t=3.0, label="R draws it clear")
        yield from r.move(r.at() + np.array([FLOOR_C[0] - P().pos[0], 0.0, -(low() - 0.002)]), t=5.0, label="R sets it down")
        if ctx.mode == "drag":
            yield from r.move(r.at() - 0.10 * X, t=2.0, label="R drags it")
        report("set down (one hand)")
        yield from r.open()
        yield from r.move(r.at() + 0.10 * Z, t=1.5, label="R clears")
        yield from r.wait(2.0)
        report("end (one hand)")
        return
    # 2. the right draws it until the rim's front strip is clear of the front tube; the left takes the strip
    yield from r.move(r.at() - D_LEFT * X, t=3.0 + 20 * D_LEFT, label="R draws it forward")
    report("drawn")
    aim = pinch_f()
    yield from l.move(aim - 0.10 * X, grip=GLV, finger=OPEN_L, t=3.5, label="L in front of the rim")
    yield from l.servo(aim - 0.10 * X, timeout=1.0)
    yield from l.move(pinch_f(), t=1.5, label="L on the rim")
    yield from l.servo(pinch_f(), timeout=1.0)
    yield from l.close(FL_T, force=40.0)
    report("both on")
    # 3. both draw it on, level, under the front tube and off the brackets' ends (its centre passes them on the way; the left keeps the front up, the bail holds the lugs)
    yield from carry(ctx, -(D_OUT - drawn()), 0.0, 4.5, P)
    report("clear")
    gap = {"drop": 0.12, "drop_low15": 0.015, "drop_low19": 0.019}.get(ctx.mode, 0.002)
    if ctx.mode == "slow_late":  # as full to 80 mm up, then lowered fast to 40 mm up (about 0.3 m/s), braked to a stop, and set down at 0.05 m/s at most
        yield from carry(ctx, FLOOR_C[0] - P().pos[0], -(low() - 0.08), 4.0, P)
        yield from carry(ctx, 0.0, -(low() - 0.04), 0.25, P)
        yield from l.wait(0.3)
        yield from carry(ctx, 0.0, -(low() - gap), 1.2, P)
    else:
        dz = -(low() - gap)
        yield from carry(ctx, FLOOR_C[0] - P().pos[0], dz, 5.0, P)
    report("set down")
    if ctx.mode not in ("drop", "drop_low15", "drop_low19"):  # the left's levelling trim pressed the pail's near corner into the floor (it rests 5 degrees rolled): ease the left off 1 mm a step until the pail lies flat, still held
        z = 0.0
        pl0 = l.at()
        if ctx.mode == "slow_late":  # the fast descent leaves the pail's front up on the left's hold (pitched about 3 degrees): press the left down on it until the pail's pitch is under 1 degree
            while abs(math.degrees(math.asin(float(np.clip(P().rotation()[2, 1], -1, 1))))) > 1.0 and z > -0.03:
                z -= 0.001
                l._solve(pl0 + np.array([0.0, 0.0, z]), l.grip.rot)
                yield
            yield from l.wait(0.3)
            report(f"pressed {-z * 1000:.0f} mm")
            z, pl0 = 0.0, l.at()
        while P().tilt_deg() > 1.0 and z < 0.04:
            z += 0.001
            l._solve(pl0 + np.array([0.0, 0.0, z]), l.grip.rot)
            yield
        yield from l.wait(0.3)
        report(f"eased {z * 1000:.0f} mm")
    if ctx.mode in ("drop_low15", "drop_low19"):  # the pail's lowest corner touched the floor on the way (it is a little pitched): lift it until that corner is exactly `gap` up, then let go
        pl0, pr0, z = l.at(), r.at(), 0.0  # the hands' targets are below the pail (pressed on the floor): raise both 2 mm a step until the pail's lowest corner is `gap` up
        while low() < gap and z < 0.12:
            z += 0.002
            l._solve(pl0 + np.array([0.0, 0.0, z]) + l.trim, l.grip.rot)
            r._solve(pr0 + np.array([0.0, 0.0, z]) + r.trim, r.grip.rot)
            yield
        yield from l.wait(0.1)
        report(f"held {low() * 1000:.0f} mm up")
    if ctx.mode in ("tip_release", "tip_gentle"):  # the left lifts the pail's front so it rests on its back edge (tip_release: 60 mm, tip_gentle: 5 mm, both under the splash limit), then both let go: the tipped pail slaps flat (HARD); the gentle one settles
        pl0 = l.at()
        lift = 0.060 if ctx.mode == "tip_release" else 0.005
        for k in range(1, 31):
            l._solve(pl0 + np.array([0.0, 0.0, lift * k / 30]) + l.trim, l.grip.rot)
            yield
        yield from l.wait(0.3)
        report(f"held tipped, pitch {math.degrees(math.asin(float(np.clip(P().rotation()[2, 1], -1, 1)))):.1f}")
    if ctx.mode == "drag2":  # still holding it on the floor: drag it 10 cm, then let go and back off as full does
        yield from carry(ctx, -0.10, 0.0, 2.0, P)
        report("dragged")
    if ctx.mode == "hold_on":  # only the right lets go and goes; the left keeps its pinch on the rim
        yield from r.open()
        yield from r.move(r.at() - 0.04 * X, t=1.5, label="R backs off")
        yield from r.move(r.at() + 0.08 * Z, t=1.5, label="R clears")
        yield from l.wait(4.0)
        report("end (left still on the rim)")
        return
    yield from l.open()
    yield from r.open()
    yield from r.wait(0.5)
    report("let go")
    if ctx.mode == "undo":  # within the 1 s DOWN needs (a run ends once it scores 1.0): the left, still at the rim, takes it again and lifts it back toward the brackets
        yield from l.close(FL_T, force=40.0)
        yield from l.move(l.at() + 0.08 * Z, t=2.0, label="L lifts it")
        yield from l.move(l.at() + 0.20 * X, t=3.0, label="L back toward the brackets")
        yield from l.wait(1.0)
        report("end (undo)")
        return
    yield from ctx.together(l.move(l.at() - 0.045 * X, t=1.5, label="L backs off"), r.move(r.at() - 0.04 * X, t=1.5, label="R backs off"))
    yield from ctx.together(l.move(l.at() + 0.08 * Z, t=1.5, label="L clears"), r.move(r.at() + 0.12 * Z, t=1.5, label="R clears"))  # the right grips the bail 40 mm under the rim plane: 0.12 m up puts its fingers over 30 mm clear of the pail
    yield from r.wait(2.0)
    report("end")


def carry(ctx, dx: float, dz: float, t: float, P):
    """Both hands move together by (dx, 0, dz) over t seconds. The pail's pitch is levelled every step by the left hand's height on the rim's front."""
    l, r = ctx.left, ctx.right
    pl0, pr0 = l.at(), r.at()
    cl = 0.0
    n = max(1, round(t * d.CONTROL_HZ))
    back_up = lambda: math.asin(float(np.clip(P().rotation()[2, 1], -1, 1)))  # + when the pail's back (local +y) is up, its front down
    for k in range(1, n + 1):
        s = d.ik.smoothstep(k / n)
        cl = float(np.clip(cl + GAIN * back_up(), -0.04, 0.04))
        if k % 10 == 0:
            print(f"[cp] carry {k}/{n}: pitch {math.degrees(back_up()):.1f}, tilt {P().tilt_deg():.1f}, cl {cl * 1000:.1f} mm, z {P().pos[2] - TOP:.3f}, x {P().pos[0]:.3f}, bail {math.degrees(math.radians(FOLD) + P().joint('bail')):.0f}", flush=True)
        off = np.array([dx * s, 0.0, dz * s])
        l._solve(pl0 + off + np.array([0.0, 0.0, cl]) + l.trim, l.grip.rot)
        r._solve(pr0 + off + r.trim, r.grip.rot)
        yield
