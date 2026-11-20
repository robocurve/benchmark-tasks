"""Plate up a canteen meal on a hand-held dinner plate: the left hand takes the top 26 cm melamine plate off a short stack by its rim (the right hand
first presses the plate's far rim down so its near rim tips up off the plate below) and holds it up; the right
hand lifts a steak, a hot dog in its bun, two whole chicken wings and a bread roll out of two steam-table pans and lays each on the held plate, all inside
the plate's outer edge. Then the plate is delivered to the pick-up zone on the pass, for the guest to collect.

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Take the top plate off the stack and, holding it up the whole time, serve the steak, the hot dog, both chicken wings and the "
               "bread roll onto it from the pans, with nothing sticking out past the plate's edge. Then deliver the served plate to the pick-up zone "
               "on the pass and let go.")
SUCCESS_HOLD_S = 0.0
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the plate: Carlisle Dallas Ware 4350002, 10 1/4 in (26 cm) melamine; origin at the centre of its underside's rest plane
R_EDGE = 0.130
PLATE_MASS = 0.38
# the left hand's grasp: fingers level, pointing GRIP_PHI from +x at the plate's centre, the jaws closing up and down across the near rim
GRIP_PHI = float(os.environ.get("PLATE_PHI", "0"))
GRIP_R = float(os.environ.get("PLATE_GRIP_R", "0.110"))  # the TCP this far in from the plate's centre (radius); the fingertips 13 mm further in
PLATE_START = np.array([float(os.environ.get("PLATE_SX", "0.30")), float(os.environ.get("PLATE_SY", "0.10"))])
FOOT_R, FOOT_H = 0.080, 0.006
# (radius, z) of the top surface and of the underside, from the centre out to the edge
TOP_PROFILE = [(0.0, 0.012), (0.092, 0.012), (0.105, 0.0135), (0.118, 0.0175), (0.130, 0.0215)]
UNDER_PROFILE = [(0.0, 0.0065), (0.092, 0.0065), (0.105, 0.0085), (0.118, 0.0112), (0.130, 0.0145)]
PLATE_H = 0.0215
# the stack: N_FIXED identical plates under the free top one, nested PITCH apart (the foot of each on the floor of the one below; a Carlisle case of 48
# is four stacks of 12 in a 6.38 in tall box, about 1.2 cm a plate); the stack is fixed, the top plate is free
N_FIXED = 7
PITCH = TOP_PROFILE[0][1]
SEAT_Z = N_FIXED * PITCH  # the free plate's rest plane above the counter
FIX_Z = SEAT_Z - PITCH  # the top fixed plate's rest plane
PRESS_AZ = float(os.environ.get("PLATE_PAZ", "-45"))  # the right hand presses the far rim here (deg from +x at the plate's centre)

# -- the pans: Vollrath 30222, half size, 2 1/2 in deep
PAN_X, PAN_Y, PAN_H = 0.325, 0.265, 0.065
PAN_W = 0.002
PAN_FLOOR = 0.003
PANS = {"pan_a": (0.38, -0.30), "pan_b": (0.38, -0.57)}

# -- the pick-up zone on the pass: a 35 x 35 x 1 cm stainless shelf pad, outlined on the counter by a 1.5 cm yellow tape line round it; the zone is
# the area inside the line (the pad), sized for one plate
MAT_C = np.array([0.38, 0.44])  # on the counter just left of the stack, where the loaded left arm still holds the plate up (farther out it sags 10 cm)
SET_AT = MAT_C + np.array([-0.01, -0.01])  # the demo sets the plate's centre here: the level rim grip reaches the counter there
MAT_H = 0.01
MAT_HALF = 0.175
TAPE_W = 0.015

# -- the food: (length along x, width, height, mass kg, start x, start y); masses are cooked portions (estimates)
FOOD = {
    "steak": (0.150, 0.082, 0.022, 0.270, 0.38, -0.245),  # a sirloin steak, 15 x 8.2 x 2.2 cm (7.1 cm at its narrow end, 9.3 at its wide end), its fat edge along one side
    "hotdog": (0.160, 0.060, 0.050, 0.100, 0.38, -0.36),
    "wing_a": (0.095, 0.043, 0.030, 0.060, 0.37, -0.49),  # whole wings: drumette, flat and tip joined, 9.5 x 4.3 cm; near the pans' shared wall
    "wing_b": (0.095, 0.043, 0.030, 0.060, 0.32, -0.575),
    "roll": (0.076, 0.076, 0.050, 0.045, 0.27, -0.497),  # a round crusty bread roll, 7.6 cm across
}
STEAK_TAPER = 0.13  # the steak's width grows this fraction from its centre to its +x end (and shrinks as much to its -x end)

# -- the serving tongs: 12 in (30.5 cm) stainless spring utility tongs, scalloped tips (Vollrath 4781230 class); their own frame has the hinge at
# the origin, the arms along -y, opening along x, the tips' width along z
ARM_T, ARM_W, TIP_W = 0.0025, 0.020, 0.030  # arms 20 mm wide, modelled 2.5 mm thick (real about 1 mm); scalloped tips 30 mm wide
ROOT = (0.0075, -0.006)  # where each arm leaves the hinge block
TIP_C = (0.054, -0.290)  # the centre of each tip's rounded end, the tongs open: tips 10.8 cm apart (the maker's open width); 30.5 cm long
TIP_LEN = 0.055  # the flat scalloped tip
HINGE_R = 0.010
TONG_MASS = (0.090, 0.080)  # the arm with the hinge, and the other: 170 g
SPRING = 8.0  # N m/rad at the hinge (estimate, as the 9 in tongs of hotdog-roller-tongs): about 9 N at the tips squeezed shut
CLOSE_DEG = 18.5  # the free arm's travel toward closed: at the stop the tips meet (the arms' own contact, as a joint limit)
GRIP_Y = -0.115  # the hand holds the tongs by the handle, 11.5 cm from the hinge: the arms 5.5 cm across there, open
TONGS_AT = np.array([0.46, -0.10])  # the hinge, the tongs lying on their arms' edges, tips toward -x
TONGS_YAW = -90.0
TOWEL = ((0.135, 0.485), (-0.163, -0.037), 0.008)  # a folded cotton bar towel under them, 35 x 12.6 cm: the open tongs lie wholly on it

# -- the scorer's settings
CLEAR_H = 0.05  # the plate's lowest point at least this high: held clear
ON_H = 0.008  # an item's centre this high above the plate's floor plane
LET_GO = 0.008  # every gripper point (fingers, palm, housing, wrist) at least 8 mm off the plate and the items
REST_TOL = 0.003  # a plate point within 3 mm of a surface rests on it
HOLD_TOUCH = 0.006  # held: a gripper point within 6 mm of the plate's modelled surface (a pinched rim reads 4.4 mm: the rim's rounded section)
HAND_TOUCH = 0.004  # a gripper point within 4 mm of an item's surface: the hand touched the food, which then never counts as served
STILL_V = 0.03  # m/s
FLAT_DEG = 6.0
DROP_V = 0.45  # m/s: a free fall from 1 cm

LOOKS = {
    "melamine": m.Look(color=m.rgb(0.93, 0.93, 0.90), roughness=0.35),
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.9, roughness=0.35),
    "mat": m.Look(color=m.rgb(0.55, 0.57, 0.60), metallic=0.8, roughness=0.5),
    "tape": m.Look(color=m.rgb(0.95, 0.78, 0.05), roughness=0.6),
    "steak_side": m.Look(color=m.rgb(0.43, 0.21, 0.12), roughness=0.6),
    "steak_sear": m.Look(color=m.rgb(0.29, 0.14, 0.06), roughness=0.5),
    "char": m.Look(color=m.rgb(0.09, 0.05, 0.03), roughness=0.7),
    "fat": m.Look(color=m.rgb(0.90, 0.78, 0.52), roughness=0.45),
    "bun": m.Look(color=m.rgb(0.80, 0.58, 0.30), roughness=0.7),
    "sausage": m.Look(color=m.rgb(0.62, 0.20, 0.12), roughness=0.5),
    "mustard": m.Look(color=m.rgb(0.90, 0.75, 0.10), roughness=0.5),
    "wing": m.Look(color=m.rgb(0.84, 0.54, 0.20), roughness=0.4),
    "wing_dark": m.Look(color=m.rgb(0.62, 0.33, 0.11), roughness=0.45),
    "bone": m.Look(color=m.rgb(0.93, 0.88, 0.76), roughness=0.6),
    "crust": m.Look(color=m.rgb(0.70, 0.43, 0.17), roughness=0.75),
    "flour": m.Look(color=m.rgb(0.90, 0.84, 0.70), roughness=0.95),
    "crumb": m.Look(color=m.rgb(0.95, 0.89, 0.72), roughness=0.9),
    "tong": m.Look(color=m.rgb(0.72, 0.73, 0.75), metallic=0.9, roughness=0.3),
    "brushed": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.85, roughness=0.45),
    "towel": m.Look(color=m.rgb(0.90, 0.90, 0.86), roughness=0.95),
    "stripe": m.Look(color=m.rgb(0.20, 0.35, 0.65), roughness=0.95),
}


# -- shapes ------------------------------------------------------------------------------------------------------------------------------------------

def _round_outline(lx, wy, n=20, p=3.0):
    """A convex superellipse outline, counter-clockwise, length lx along x and width wy along y."""
    th = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    c, s = np.cos(th), np.sin(th)
    return np.c_[0.5 * lx * np.sign(c) * np.abs(c) ** (2 / p), 0.5 * wy * np.sign(s) * np.abs(s) ** (2 / p)]


def _slab(lx, wy, h, bevel=0.6):
    """A rounded slab, bottom at z 0: a full-size lower body and a smaller upper one (a sloped edge)."""
    a = m.prism(_round_outline(lx, wy), 0.0, h * bevel)
    b = m.prism(_round_outline(lx - 0.012, wy - 0.012), h * bevel * 0.999, h)
    return m.join(a, b)


def _plate_parts():
    top = [(0.0, TOP_PROFILE[0][1])] + [(r, z) for r, z in TOP_PROFILE[1:]]
    under = [(r, z) for r, z in UNDER_PROFILE[1:]][::-1]
    prof = [(0.0, 0.0), (FOOT_R, 0.0), (FOOT_R, FOOT_H), (0.092, UNDER_PROFILE[1][1])]
    prof += [(r, z) for r, z in UNDER_PROFILE[2:]] + [(R_EDGE, TOP_PROFILE[-1][1])] + [(r, z) for r, z in TOP_PROFILE[:-1]][::-1]
    prof = [p for p in prof]
    prof[-1] = (0.0, TOP_PROFILE[0][1])
    return m.lathe(prof, 48)


def _plate_colliders(dz=0.0, tag=""):
    cols = [m.col_cylinder(f"{tag}foot", FOOT_R, FOOT_H, (0, 0, dz + FOOT_H / 2)),
            m.col_cylinder(f"{tag}floor", 0.094, TOP_PROFILE[0][1] - UNDER_PROFILE[0][1], (0, 0, dz + (TOP_PROFILE[0][1] + UNDER_PROFILE[0][1]) / 2))]
    n = 24
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pts = []
        for r, z in ((0.090, UNDER_PROFILE[1][1]), (0.105, UNDER_PROFILE[2][1]), (0.118, UNDER_PROFILE[3][1]), (0.130, UNDER_PROFILE[4][1]),
                     (0.130, TOP_PROFILE[4][1]), (0.118, TOP_PROFILE[3][1]), (0.105, TOP_PROFILE[2][1]), (0.090, TOP_PROFILE[1][1])):
            for a in (a0, a1):
                pts.append((r * math.cos(a), r * math.sin(a), z + dz))
        # a thin wedge: the hull of its two end profiles
        v = np.array(pts)
        cx = v.mean(axis=0)
        f = _hull_faces(v)
        cols.append(m.col_hull(f"{tag}rim{k}", m.Mesh(v, f)))
    return cols


def _hull_planes(v):
    from scipy.spatial import ConvexHull

    return ConvexHull(v).equations


def hull_gap(planes, pts):
    """Each point's distance outside a convex hull (0 inside): the largest plane distance, never more than the true distance to the surface."""
    return np.clip((pts @ planes[:, :3].T + planes[:, 3]).max(axis=1), 0.0, None)


def _hull_faces(v):
    from scipy.spatial import ConvexHull

    h = ConvexHull(v)
    return h.simplices


def _plate():
    mesh = _plate_parts()
    return m.free("plate", (PLATE_START[0], PLATE_START[1], TOP + SEAT_Z + 0.0005), [("plate", mesh, "melamine")], _plate_colliders(), LOOKS, mass=PLATE_MASS,
                  friction=(0.8, 0.7), torsional_patch=0.02)


def _stack():
    """The fixed stack under the free plate: N_FIXED plates (the top one with the plate's own colliders, the rest one hull)."""
    parts = [(f"p{k}", _plate_parts().moved((0, 0, k * PITCH)), "melamine") for k in range(N_FIXED)]
    th = np.linspace(0.0, 2 * math.pi, 32, endpoint=False)
    ring = lambda r, z: [(r * math.cos(t), r * math.sin(t), z) for t in th]
    v = np.array(ring(FOOT_R, 0.0) + ring(R_EDGE, UNDER_PROFILE[-1][1]) + ring(R_EDGE, FIX_Z - PITCH + TOP_PROFILE[-1][1]))
    cols = _plate_colliders(FIX_Z, "t") + [m.col_hull("below", m.Mesh(v, _hull_faces(v)))]
    return m.fixed("stack", (PLATE_START[0], PLATE_START[1], TOP), parts, cols, LOOKS, friction=(0.8, 0.7))


def _pan(name):
    cx, cy = PANS[name]
    hx, hy = PAN_X / 2, PAN_Y / 2
    pieces = [((-hx, hx), (-hy, hy), (0.0, PAN_FLOOR)),
              ((-hx, -hx + PAN_W), (-hy, hy), (0.0, PAN_H)), ((hx - PAN_W, hx), (-hy, hy), (0.0, PAN_H)),
              ((-hx, hx), (-hy, -hy + PAN_W), (0.0, PAN_H)), ((-hx, hx), (hy - PAN_W, hy), (0.0, PAN_H))]
    parts, cols = [], []
    for k, (x, y, z) in enumerate(pieces):
        parts.append((f"w{k}", m.box((x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2)), "steel"))
        cols.append(m.col_box(f"c{k}", (x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2)))
    return m.fixed(name, (cx, cy, TOP), parts, cols, LOOKS, friction=(0.4, 0.3))


def _mat():
    """The pick-up zone: the shelf pad and, round it on the counter, the tape line that marks it (seen only: 1 mm, no collider)."""
    mesh = m.box((2 * MAT_HALF, 2 * MAT_HALF, MAT_H), (0, 0, MAT_H / 2))
    o, t = MAT_HALF + TAPE_W / 2, 2 * MAT_HALF + 2 * TAPE_W
    tape = [(f"tape{k}", m.box(size, (cx, cy, 0.0005)), "tape") for k, (size, cx, cy) in enumerate(
        [((t, TAPE_W, 0.001), 0.0, o), ((t, TAPE_W, 0.001), 0.0, -o), ((TAPE_W, t, 0.001), o, 0.0), ((TAPE_W, t, 0.001), -o, 0.0)])]
    return m.fixed("mat", (MAT_C[0], MAT_C[1], TOP), [("mat", mesh, "mat")] + tape, [m.col_box("c", (2 * MAT_HALF, 2 * MAT_HALF, MAT_H), (0, 0, MAT_H / 2))],
                   LOOKS, friction=(0.8, 0.7))


FOOD_V: dict = {}  # each item's collider vertices in its own frame: the on-plate test reads these
FOOD_PLANES: dict = {}  # each item's collider hull as planes (n, d), n.x + d <= 0 inside: the hands-clear test measures to its surface


def _clip(poly, a, b, keep_above):
    """The part of a convex polygon on one side of the line y = a + b x."""
    out = []
    n = len(poly)
    side = lambda q: (q[1] - (a + b * q[0])) * (1 if keep_above else -1)
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        sp, sq = side(p), side(q)
        if sp >= 0:
            out.append(p)
        if sp * sq < 0:
            t = sp / (sp - sq)
            out.append(p + t * (q - p))
    return np.array(out)


def steak_outline(lx, wy, inset=0.0, n=48):
    """The steak's outline: a superellipse whose width grows linearly along x (wider at +x)."""
    o = _round_outline(lx - 2 * inset, wy - 2 * inset, n, 3.0)
    return np.c_[o[:, 0], o[:, 1] * (1.0 + STEAK_TAPER * o[:, 0] / (lx / 2))]


def _steak(lx, wy, h):
    """A seared rump steak: brown sides, a darker seared top with grill marks, a cream fat edge along -y."""
    out = steak_outline(lx, wy)
    hw = 0.5 * wy
    a, b = -(hw - 0.008), -(hw - 0.008) * STEAK_TAPER / (lx / 2)  # the fat's inner edge, 8 mm in from the -y side, following the taper
    meat, fat = _clip(out, a, b, True), _clip(out, a, b, False)
    top = _clip(steak_outline(lx, wy, 0.006), a + 0.002, b, True)
    parts = [("meat", m.prism(meat, 0.0, 0.72 * h), "steak_side"), ("sear", m.prism(top, 0.72 * h - 0.0005, h), "steak_sear"),
             ("fat", m.prism(fat, 0.0, 0.86 * h), "fat")]
    for k, (x, L) in enumerate(((-0.045, 0.045), (-0.015, 0.060), (0.015, 0.064), (0.045, 0.054))):
        parts.append((f"mark{k}", m.box((L, 0.004, 0.0008), (0, 0, 0)).turned(35.0).moved((x, 0.006, h + 0.0002)), "char"))
    return parts, m.join(*[p[1] for p in parts[:3]]).v


def _wing():
    """A whole chicken wing, bottom at z 0: the drumette (club-shaped, -x), the flat (+x) and the tip, joined and slightly bent."""
    drum = m.lathe([(0.0, 0.0), (0.006, 0.0), (0.0085, 0.006), (0.0105, 0.016), (0.014, 0.029), (0.0148, 0.038), (0.012, 0.045), (0.0, 0.048)], 20)
    drum = drum.turned(90, "y").turned(-8.0).moved((-0.047, 0.008, 0.0148))
    flat = _slab(0.048, 0.030, 0.018, 0.6).turned(10.0).moved((0.012, -0.003, 0.0))
    tip = m.lathe([(0.0, 0.0), (0.0055, 0.0), (0.006, 0.007), (0.0038, 0.016), (0.0, 0.020)], 14).turned(90, "y").turned(40.0).moved((0.030, 0.003, 0.006))
    knob = m.sphere(0.0062, (-0.0485, 0.0082, 0.0148), 6, 12)  # the drumette's bare knuckle of bone
    joint = m.sphere(0.0105, (-0.004, 0.003, 0.0118), 6, 14)  # the browned elbow between drumette and flat
    parts = [("drumette", drum, "wing"), ("flat", flat, "wing"), ("tip", tip, "wing_dark"), ("knob", knob, "bone"), ("joint", joint, "wing_dark")]
    return parts, m.join(drum, flat, tip, knob, joint).v


def _hotdog(lx, wy, h):
    """A hot dog in its bun with a line of mustard; the collider takes in the frank on top of the bun."""
    body = _slab(lx - 0.012, wy, h * 0.72, 0.75)
    dog = m.cylinder(0.013, lx + 0.01, (0, 0, h * 0.72 + 0.002), axis="x")
    mus = m.box((lx * 0.8, 0.006, 0.004), (0, 0, h * 0.72 + 0.014))
    return [("bun", body, "bun"), ("dog", dog, "sausage"), ("mustard", mus, "mustard")], m.join(body, dog).v


ROLL_PROFILE = [(0.0, 0.0), (0.032, 0.0), (0.036, 0.005), (0.038, 0.014), (0.036, 0.026), (0.030, 0.037), (0.020, 0.045), (0.009, 0.049), (0.0, 0.050)]


def _roll():
    """A round crusty bread roll: a floury base, a golden-brown domed crust, and a split across the top showing the crumb."""
    base = m.lathe([(0.0, 0.0)] + ROLL_PROFILE[1:4] + [(0.0, 0.014)], 28)
    dome = m.lathe([(0.0, 0.0135)] + ROLL_PROFILE[3:], 28)
    rr, zz = np.array([q[0] for q in ROLL_PROFILE[3:]]), np.array([q[1] for q in ROLL_PROFILE[3:]])
    xs = np.linspace(-0.026, 0.026, 9)
    split = m.tube([(float(x), 0.0, float(np.interp(abs(x), rr[::-1], zz[::-1])) - 0.001) for x in xs], 0.0042, 10)
    parts = [("base", base, "flour"), ("crust", dome, "crust"), ("split", split, "crumb")]
    return parts, m.join(base, dome).v


def _food(name):
    lx, wy, h, mass, x, y = FOOD[name]
    if name == "steak":
        parts, hull = _steak(lx, wy, h)
    elif name == "hotdog":
        parts, hull = _hotdog(lx, wy, h)
    elif name.startswith("wing"):
        parts, hull = _wing()
    else:  # the bread roll
        parts, hull = _roll()
    z = TOP + PAN_FLOOR + 0.0005
    FOOD_V[name] = np.asarray(hull, float)
    FOOD_PLANES[name] = _hull_planes(FOOD_V[name])
    return m.free(name, (x, y, z), parts, [m.col_hull(f"{name}_c", m.Mesh(FOOD_V[name], _hull_faces(FOOD_V[name])))], LOOKS, mass=mass,
                  friction=(0.5, 0.45), torsional_patch=0.01)


# -- the tongs: a free mechanism, the frame one arm with the hinge, the other arm on a sprung hinge -------------------------------------------------

ARM_ANGLE = math.atan2(TIP_C[0] - ROOT[0], -(TIP_C[1] - ROOT[1]))  # each arm's angle off the tongs' axis, open
ARM_S = math.hypot(TIP_C[0] - ROOT[0], TIP_C[1] - ROOT[1])  # from the hinge block to the tip's round end's centre
TIP_R = TIP_W / 2


def _arm_local(side: float):
    """One arm in the tongs' frame at joint zero, as meshes. side -1: the frame's arm (tip at -x), +1: the free one."""
    end = ARM_S + TIP_R
    plate0 = end - TIP_LEN
    pieces = []
    for a, b in ((0.0, 0.08), (0.08, 0.16), (0.16, plate0 + 0.002)):
        pieces.append(m.box((ARM_T, b - a, ARM_W), (0.0, -(a + b) / 2, 0.0)))
    th = np.linspace(math.pi / 2, 3 * math.pi / 2, 13)
    outline = [(-plate0, -TIP_R), (-plate0, TIP_R)] + [(-ARM_S + TIP_R * math.cos(t), TIP_R * math.sin(t)) for t in th]
    tip = m.prism(outline, -ARM_T / 2, ARM_T / 2)
    pieces.append(m.Mesh(tip.v[:, [2, 0, 1]], tip.f))
    beta = side * ARM_ANGLE
    return [pc.turned(math.degrees(beta), "z").moved((side * ROOT[0], ROOT[1], 0.0)) for pc in pieces]


def tip_local(side: float, q: float = 0.0, pts=None) -> np.ndarray:
    """Points on a tip's inner face in the tongs' frame, the free arm turned `q` rad: by default a grid over the whole flat tip and its round end."""
    if pts is None:
        plate0 = ARM_S + TIP_R - TIP_LEN
        pts = [(s, z) for s in np.linspace(plate0, ARM_S, 7) for z in np.linspace(-TIP_R, TIP_R, 7)]
        pts += [(ARM_S + TIP_R * math.cos(a), TIP_R * math.sin(a)) for a in np.linspace(-1.3, 1.3, 9)]
    beta = side * ARM_ANGLE
    dvec = np.array([math.sin(beta), -math.cos(beta)])
    nvec = np.array([math.cos(beta), math.sin(beta)])
    root = np.array([side * ROOT[0], ROOT[1]])
    out = np.array([(*(root + s * dvec - side * nvec * (ARM_T / 2)), z) for s, z in pts])
    if side > 0 and q:
        c, sn = math.cos(q), math.sin(q)
        out = out @ np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]]).T
    return out


def _tongs():
    a_pieces, b_pieces = _arm_local(-1.0), _arm_local(1.0)
    loop = m.cylinder(HINGE_R, TIP_W, (0.0, 0.0, 0.0))
    ring = m.tube([(0.0, 0.004, -0.008), (0.0, 0.020, -0.006), (0.0, 0.026, 0.0), (0.0, 0.020, 0.006), (0.0, 0.004, 0.008)], 0.0015, 8)
    frame_parts = [(f"arm_a{i}", pc, "tong") for i, pc in enumerate(a_pieces)] + [("hinge", loop, "brushed"), ("lock_ring", ring, "tong")]
    frame_cols = [m.col_hull(f"arm_a{i}_c", pc) for i, pc in enumerate(a_pieces)] + [m.col_cylinder("hinge_c", HINGE_R, TIP_W)]
    com_b = tuple(tip_local(1.0, 0.0, [(0.5 * ARM_S, 0.0)])[0])
    jaw = m.hinge("jaw", "Z", (0.0, 0.0, 0.0), -CLOSE_DEG, 0.0, [(f"arm_b{i}", pc, "tong") for i, pc in enumerate(b_pieces)],
                  [m.col_hull(f"arm_b{i}_c", pc) for i, pc in enumerate(b_pieces)], mass=TONG_MASS[1], spring=SPRING, damping=0.01, effort=5.0,
                  friction=0.0, armature=1e-4, gravity=True, com=com_b, inertia=(4.0e-4, 8.0e-6, 4.0e-4))
    z0 = TOWEL[2] + TIP_R
    return m.mechanism("tongs", (TONGS_AT[0], TONGS_AT[1], TOP + z0), [jaw], LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols,
                       friction=(0.8, 0.7), solver_iters=64, free_mass=TONG_MASS[0], frame_inertia=(5.0e-4, 1.0e-5, 5.0e-4), yaw_deg=TONGS_YAW)


def _towel():
    (x0, x1), (y0, y1), h = TOWEL
    parts = [("towel", m.box((x1 - x0, y1 - y0, h), ((x0 + x1) / 2, (y0 + y1) / 2, h / 2)), "towel")]
    for k, yy in enumerate((y0 + 0.012, y1 - 0.012)):
        parts.append((f"stripe{k}", m.box((x1 - x0 + 0.001, 0.008, h + 0.001), ((x0 + x1) / 2, yy, h / 2)), "stripe"))
    return m.fixed("towel", (0.0, 0.0, TOP), parts, [m.col_box("towel_c", (x1 - x0, y1 - y0, h), ((x0 + x1) / 2, (y0 + y1) / 2, h / 2))], LOOKS,
                   friction=(0.9, 0.8))


def tong_points(tongs) -> np.ndarray:
    """Points over the whole tongs (both arms' faces and edges every 7 mm, both tips, the hinge), station frame (z from the floor)."""
    q = tongs.joint("jaw")
    arm = [(s, z) for s in np.linspace(0.0, ARM_S + TIP_R, 44) for z in (-ARM_W / 2, 0.0, ARM_W / 2)]
    loc = np.vstack([tip_local(-1.0, 0.0, arm), tip_local(1.0, q, arm), tip_local(-1.0), tip_local(1.0, q), [[0.0, 0.0, 0.0], [0.0, 0.026, 0.0]]])
    return tongs.to_world(loc)


def tip_ref(tongs) -> np.ndarray:
    """The midpoint of the two tips' round-end centres, station frame: what the demo steers."""
    q = tongs.joint("jaw")
    a = tongs.to_world(tip_local(-1.0, 0.0, [(ARM_S, 0.0)]))[0]
    b = tongs.to_world(tip_local(1.0, q, [(ARM_S, 0.0)]))[0]
    return (a + b) / 2


OBJECTS = [_plate(), _stack(), _pan("pan_a"), _pan("pan_b"), _mat(), _towel(), _tongs()] + [_food(n) for n in FOOD]



# -- the scorer -------------------------------------------------------------------------------------------------------------------------------------

def _plate_samples():
    th = np.linspace(0.0, 2 * math.pi, 72, endpoint=False)
    edge = [(R_EDGE * math.cos(t), R_EDGE * math.sin(t), UNDER_PROFILE[-1][1]) for t in th]
    foot = [(FOOT_R * math.cos(t), FOOT_R * math.sin(t), 0.0) for t in th]
    return np.array(edge + foot + [(0.0, 0.0, 0.0)])


PLATE_PTS = _plate_samples()


def _seg_dist(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def _plate_outline():
    """The plate's section (r, z) as a closed polyline."""
    pts = [(0.0, 0.0), (FOOT_R, 0.0), (FOOT_R, FOOT_H)] + [(r, z) for r, z in UNDER_PROFILE[1:]] + [(r, z) for r, z in TOP_PROFILE[::-1]]
    return np.array(pts)


OUTLINE = _plate_outline()


def plate_gap(plate, pts):
    """Each point's distance to the plate's surface (its whole axisymmetric section: foot, underside, rim, edge and top), metres."""
    loc = plate.to_local(pts)
    rz = np.c_[np.hypot(loc[:, 0], loc[:, 1]), loc[:, 2]]
    a, b = OUTLINE, np.roll(OUTLINE, -1, axis=0)
    ab = b - a
    ap = rz[:, None, :] - a[None]
    t = np.clip((ap * ab).sum(-1) / (ab * ab).sum(-1), 0.0, 1.0)
    return np.linalg.norm(ap - t[..., None] * ab, axis=2).min(axis=1)


def gripper_points(opening: float) -> np.ndarray:
    """Points on one gripper in its `gripper` body's frame: the palm and housing (from the collision meshes' size) and both fingers from root to tip."""
    # the palm and housing box's whole surface on a grid of about 6 mm, so a touch anywhere on it reads under 4.5 mm (the let-go gap is 8 mm)
    xs, ys, zs = np.linspace(-0.05, 0.05, 17), np.linspace(-0.033, 0.033, 12), np.linspace(-0.01, 0.064, 13)
    palm = [(x, y, z) for x in xs for y in ys for z in zs if x in (xs[0], xs[-1]) or y in (ys[0], ys[-1]) or z in (zs[0], zs[-1])]
    fingers = [(sx * (opening + 0.0045), y, z) for sx in (-1.0, 1.0) for y in np.linspace(-0.018, 0.018, 13) for z in np.linspace(0.06, 0.147, 30)]  # a 3 mm grid: a pad on the rim reads under 2.2 mm
    return np.array(palm + fingers)


def hand_points(env) -> np.ndarray:
    """Both grippers (palm, housing, fingers) and each wrist link down to the gripper, as points in the station's frame."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        P = m._np(robot.data.body_link_pose_w)[0]
        g = P[robot.find_bodies("gripper")[0][0]]
        w = P[robot.find_bodies("link5")[0][0], :3] - origin
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        out.append(gripper_points(opening) @ m.quat_matrix(g[3:7]).T + g[:3] - origin)
        out.append(np.array([w + (g[:3] - origin - w) * t for t in np.linspace(0.0, 1.0, 6)]))
    return np.vstack(out)


def _support_points():
    """Points about 5 mm apart on every surface the plate could be propped on besides the counter and the zone: each pan's wall tops, the stack's top
    plate (its well, rim and edge: 144 directions x 6 radii) and the stack's outer side (144 directions, every 5 mm up). The plate rests on one when
    any of its points lies within 3 mm of the plate's real surface (its whole section, foot to edge)."""
    pts = []
    for cx, cy in PANS.values():
        hx, hy = PAN_X / 2 - PAN_W / 2, PAN_Y / 2 - PAN_W / 2
        for s in np.linspace(-hx, hx, int(2 * hx / 0.005) + 1):
            pts += [(cx + s, cy - hy, TOP + PAN_H), (cx + s, cy + hy, TOP + PAN_H)]
        for s in np.linspace(-hy, hy, int(2 * hy / 0.005) + 1):
            pts += [(cx - hx, cy + s, TOP + PAN_H), (cx + hx, cy + s, TOP + PAN_H)]
    th = np.linspace(0.0, 2 * math.pi, 144, endpoint=False)
    for r in (0.0, 0.046, 0.092, 0.105, 0.118, R_EDGE):
        for t in (th if r > 0 else th[:1]):
            pts.append((PLATE_START[0] + r * math.cos(t), PLATE_START[1] + r * math.sin(t), TOP + stack_top_z(r)))
    for z in np.arange(UNDER_PROFILE[-1][1], FIX_Z + TOP_PROFILE[-1][1], 0.005):
        for t in th:
            pts.append((PLATE_START[0] + R_EDGE * math.cos(t), PLATE_START[1] + R_EDGE * math.sin(t), TOP + z))
    return np.array(pts)


def stack_top_z(r: float) -> float:
    """The height above the counter of the top fixed plate's upper surface at radius r (the hull below it outside R_EDGE: none)."""
    rs, zs = zip(*TOP_PROFILE)
    return FIX_Z + float(np.interp(r, rs, zs))


SUPPORT_PTS = _support_points()
SUPPORT_STACK = np.hypot(SUPPORT_PTS[:, 0] - PLATE_START[0], SUPPORT_PTS[:, 1] - PLATE_START[1]) <= R_EDGE + 1e-6


def on_stack(p, tol: float) -> bool:
    """A point within `tol` of the stack's top surface, or inside the stack."""
    r = math.hypot(p[0] - PLATE_START[0], p[1] - PLATE_START[1])
    return r <= R_EDGE + tol and p[2] - TOP <= stack_top_z(min(r, R_EDGE)) + tol


def plate_rests(plate):
    """What the plate rests on: (anything, on the stack). Anything: one of its points within 3 mm of the counter or the zone, or a support point
    (stack, pan wall tops) within 3 mm of its surface."""
    w = plate.to_world(PLATE_PTS)
    h = w[:, 2] - TOP
    in_mat = (np.abs(w[:, 0] - MAT_C[0]) <= MAT_HALF) & (np.abs(w[:, 1] - MAT_C[1]) <= MAT_HALF)
    low = bool((h <= REST_TOL).any() or (in_mat & (np.abs(h - MAT_H) <= REST_TOL)).any())
    near = np.linalg.norm(SUPPORT_PTS - plate.pos, axis=1) <= R_EDGE + 0.01  # only points the plate could reach
    hit = np.zeros(len(SUPPORT_PTS), bool)
    if near.any():
        hit[near] = plate_gap(plate, SUPPORT_PTS[near]) <= REST_TOL
    stack = bool((hit & SUPPORT_STACK).any())
    return bool(low or hit.any()), stack


def in_pan_xy(p) -> bool:
    return any(abs(p[0] - cx) <= PAN_X / 2 + 0.005 and abs(p[1] - cy) <= PAN_Y / 2 + 0.005 for cx, cy in PANS.values())


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.void_why = ""
        self.void = gk.Latch("food was on the plate while the plate rested on something, before all five were served onto it held clear")
        self.served = False
        self.lost = {n: gk.Latch(f"{n} fell to the counter or the stack after leaving its pan") for n in FOOD}
        self.handled = {n: gk.Latch(f"a gripper touched the {n}: served by hand, not with the tongs") for n in FOOD}
        self.end = gk.EndHold(HOLD_STEPS)
        self.score = 0.0
        self.report: dict = {}

    def step(self, env) -> None:
        self.steps += 1
        plate = m.state(env, "plate")
        hands = hand_points(env)
        pw = plate.to_world(PLATE_PTS)
        low = float(pw[:, 2].min() - TOP)
        rests, on_stk = plate_rests(plate)
        tilt = plate.tilt_deg()

        tp = tong_points(m.state(env, "tongs"))
        on = {}
        gaps = {}
        tg = {}
        anything = False
        for n in FOOD:
            it = m.state(env, n)
            w = it.to_world(FOOD_V[n])
            loc = plate.to_local(w)
            rad = np.hypot(loc[:, 0], loc[:, 1])
            ctr = plate.to_local(it.pos + it.rotation() @ np.array([0.0, 0.0, FOOD[n][2] / 2]))[0]
            inside = bool(rad.max() <= R_EDGE and ctr[2] >= ON_H and np.hypot(ctr[0], ctr[1]) <= R_EDGE)
            d = float(hull_gap(FOOD_PLANES[n], it.to_local(hands)).min())
            gaps[n] = float(d)
            tg[n] = float(hull_gap(FOOD_PLANES[n], it.to_local(tp)).min())  # the tongs still on it: not yet let go
            # dropped: low over the counter, outside every pan, and not on the plate
            low_it = float(w[:, 2].min() - TOP)
            anything = anything or inside
            gone = not inside and ((low_it <= 0.012 and not in_pan_xy(it.pos)) or any(on_stack(q, 0.012) for q in w))
            self.lost[n].trip(bool(gone))
            self.handled[n].trip(bool(d <= HAND_TOUCH))
            speed = float(np.linalg.norm(it.lin_vel))
            on[n] = bool(inside and not self.lost[n] and not self.handled[n] and d >= LET_GO and tg[n] >= LET_GO and speed <= 0.2)
        # an item under the plate that touches it holds the plate up: the plate rests on it (a plate propped on the food in a pan is not held)
        rests = rests or any(float(plate.to_local(m.state(env, n).pos[None])[0][2]) < 0.0 and plate_gap(plate, m.state(env, n).to_world(FOOD_V[n])).min() <= REST_TOL
                             for n in FOOD if not on[n])
        pg = plate_gap(plate, hands)
        # held clear: resting on nothing, its lowest point at least 5 cm up, and a gripper touching it (within 6 mm)
        clear = bool(low >= CLEAR_H and not rests and pg.min() <= HOLD_TOUCH)
        all_on = all(on.values())

        self.served = self.served or bool(all_on and clear)
        # the plate let fall: any point of it (the whole edge and foot, so a plate that tips off a finger counts too) going down faster
        # than a free fall from 1 cm gives (h = v^2 / 2g), held or not
        v_pts = plate.lin_vel + np.cross(plate.ang_vel, pw - plate.pos)
        dropped = bool(float(v_pts[:, 2].min()) < -DROP_V)
        if not self.void and ((rests and anything and not self.served) or dropped):
            self.void_why = f"step {self.steps}: " + ("dropped, %.2f m/s down" % -float(v_pts[:, 2].min()) if dropped else "food on it while it rests, on_stack %s, low %.1f mm" % (on_stk, 1000 * low))
        self.void.trip(bool((rests and anything and not self.served) or dropped))
        failed = bool(self.void)

        pc = plate.pos
        # off the stack: held clear, or touching no part of the stack with its centre a plate's width (26 cm) from the stack's
        off_stack = bool(clear or (not on_stk and math.hypot(pc[0] - PLATE_START[0], pc[1] - PLATE_START[1]) >= 2 * R_EDGE))
        # in the zone: the whole plate inside the tape line (seen from above), resting on the pad, flat
        in_zone = bool(abs(pc[0] - MAT_C[0]) <= MAT_HALF - R_EDGE and abs(pc[1] - MAT_C[1]) <= MAT_HALF - R_EDGE and low <= MAT_H + REST_TOL and tilt <= FLAT_DEG)
        # let go: both grippers and the tongs 8 mm or more off the plate and every item
        tong_gap = min([float(plate_gap(plate, tp).min())] + list(tg.values()))
        let_go = bool(pg.min() >= LET_GO and min(gaps.values()) >= LET_GO and tong_gap >= LET_GO)
        still = bool(max([float(np.linalg.norm(plate.lin_vel))] + [float(np.linalg.norm(m.state(env, n).lin_vel)) for n in FOOD]) <= STILL_V)

        if failed:
            tier = 0.0
        elif all_on and in_zone and let_go and still:
            tier = 0.75
        elif all_on and off_stack:
            tier = 0.5
        elif off_stack:
            tier = 0.25
        else:
            tier = 0.0
        self.end.step(tier == 0.75)
        held = self.end.count >= HOLD_STEPS  # the delivered state now, and unbroken for the last 2 s
        self.score = 1.0 if held else tier
        self.report = {"step": self.steps, "score": self.score, "tier": tier, "low_mm": round(1000 * low, 1), "tilt": round(tilt, 1), "rests": rests,
                       "on": on, "lost": [n for n in FOOD if self.lost[n]], "handled": [n for n in FOOD if self.handled[n]], "tong_gap_mm": round(1000 * tong_gap, 1), "void": bool(self.void), "void_why": self.void_why, "in_zone": in_zone, "off_stack": off_stack, "on_stack": on_stk, "let_go": let_go,
                       "tip_gap_mm": round(1000 * float(pg.min()), 1), "down_mps": round(-float(v_pts[:, 2].min()), 2)}
        if self.steps % 20 == 0:
            d = os.environ.get("PLATE_SCORE_DIR", "plate_scores")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, f"{os.environ.get('TASK_DEMO', 'run')}.json"), "w") as f:
                json.dump(self.report, f, indent=0, default=str)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.end.count >= HOLD_STEPS


def score(objects) -> float:
    return TRIAL.score


# -- the demo ---------------------------------------------------------------------------------------------------------------------------------------

def rim_grip(phi_deg: float, pitch_deg: float = 0.0, normal=None):
    """Fingers pointing `phi_deg` from +x, `pitch_deg` below level, the jaws closing up and down: one pad under the rim, one on top of it.
    With `normal` (the plate's), the jaws roll about the fingers to close along it, so both pads lie flat across a rim that leans sideways."""
    import demokit as d

    phi, p = math.radians(phi_deg), math.radians(pitch_deg)
    a = np.array([math.cos(phi) * math.cos(p), math.sin(phi) * math.cos(p), -math.sin(p)])
    up = np.array([0.0, 0.0, 1.0]) if normal is None else np.asarray(normal, float)
    xc = up - up.dot(a) * a
    xc /= np.linalg.norm(xc)
    return d._as_grip(np.c_[xc, np.cross(a, xc), a])


GAP = float(os.environ.get("PLATE_GAP", "0.026"))  # jaws open this far round the rim (pad to pad)
GRIP_Z = float(os.environ.get("PLATE_GRIP_Z", "0.016"))  # the TCP this high above the counter: the lower pad's face just under the rim's underside
ROLL = os.environ.get("PLATE_ROLL", "1") == "1"  # the left jaws roll to the tipped plate's lean (see rim_grip)
LIFT = float(os.environ.get("PLATE_LIFT", "0.065"))  # the plate is lifted this far straight up off the stack and held there for loading
HIGH = float(os.environ.get("PLATE_HIGH", "0.24"))  # the right hand carries each item this high above the counter: the held plate's rim is at about 16 cm
LEVEL = os.environ.get("PLATE_LEVEL", "1") == "1"  # the left wrist levels the held plate by its observed tilt, before loading and after each item
CARRY_Y = float(os.environ.get("PLATE_CARRY_Y", "-0.12"))  # after the lift, the left hand brings the plate this far along y, toward the serving hand
HOLD_DOWN = float(os.environ.get("PLATE_HOLD_DOWN", "0.0"))  # and lowers it this far: none, so a press on the held plate does not bring its foot down onto the stack
CARRY_UP = float(os.environ.get("PLATE_CARRY_UP", "0.08"))  # the loaded plate is raised this far before it is carried to the mat
SET_P1 = float(os.environ.get("PLATE_SET_P1", "10"))  # deg: the fingers pitch down this far, pinched, so the far edge meets the mat first
SET_P2 = float(os.environ.get("PLATE_SET_P2", "25"))  # deg: and this far, open, by the time the plate lies flat, so the housing stays off the counter
BACK = 0.035  # the fingertips start this far outside where they pinch: just outside the plate's edge


def level_plate(ctx, arm, steps=3, tol_deg=1.5):
    """Read the held plate's tilt and turn the wrist about the pinch to bring the plate level again (it sags in the pinch as the load grows)."""
    import demokit as d

    for _ in range(steps):
        n = ctx.state("plate").rotation()[:, 2]
        ax = np.cross(n, np.array([0.0, 0.0, 1.0]))
        s = float(np.linalg.norm(ax))
        if math.degrees(math.asin(min(1.0, s))) < tol_deg:
            break
        rot = d.axis_angle_matrix(ax / s, min(0.8 * math.asin(min(1.0, s)), math.radians(4.0))) @ arm.grip.rot
        yield from arm.move(arm.at(), grip=d._as_grip(rot), t=0.4, label="left: level the plate")
        yield from arm.wait(0.3)


MODES = ("full", "none", "set_down", "stack_prop", "idle_server", "roll_over_edge", "dropped_wing", "no_handoff", "drop_on_mat", "bare_hands",
         "last_in_tongs", "latch_then_rest")


def serve_bare(ctx, mode):
    """The old bare-gripper serve (a lazy route now: the fingers touch the food)."""
    import demokit as d

    l, r = ctx.left, ctx.right
    up = np.array([0.0, 0.0, 1.0])
    dr = d.down(0)
    # to lay an item on the plate, the fingers lean RT toward +y (about x), the jaws still closing along y: a straight-down grip does not reach there
    lean = d._as_grip(d.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(float(os.environ.get("PLATE_RT", "25")))) @ dr.rot)
    # the steak and the hot dog side by side across v, each finger of each lay clear of the item beside it; the bread roll (far, wide end) and one wing
    # (near, narrow end) on the steak's top; the other wing on the floor past the hot dog. Items are held off their centre where the jaws reach across:
    # the steak by its narrow end (7.1 cm across there; the jaws open 9.4 cm), a wing by its drumette
    WINGV = float(os.environ.get("PLATE_WINGV", "0.059"))
    TOPZ = 0.012 + FOOD["steak"][2]
    # the leaning fingers carry each item 1 to 3 cm toward +v of the target (measured on GPU), so the targets sit that much toward -v of where
    # the items land: the steak at v -0.058, the hot dog 0.038, the roll -0.062 and one wing -0.058 on the steak, the other wing 0.088
    ITEMS = [  # name, jaw width, TCP height above the item's bottom, target (u, v, z of the item's bottom in the plate's frame), grip yaw, release margin,
        # the grip point in the item's frame (x, y), the squeeze per finger (N)
        ("steak", 0.084, 0.014, (0.010, -0.079, 0.012), 0.0, 0.005, (-0.050, 0.0), 40.0),
        ("hotdog", 0.062, 0.030, (0.010, 0.020, 0.012), 0.0, 0.004, (0.0, 0.0), 22.0),
        ("roll", 0.072, 0.028, (0.058, -0.071, TOPZ), 0.0, 0.008, (0.0, 0.0), 22.0),
        ("wing_b", 0.030, 0.018, (-0.028, -0.082, TOPZ), 0.0, 0.006, (-0.025, 0.008), 22.0),
        ("wing_a", 0.030, 0.018, (0.0, WINGV, 0.012), 0.0, 0.001, (-0.025, 0.008), 22.0),
    ]
    if mode == "roll_over_edge":  # the roll set on the rim, half past the plate's edge: 0.25
        ITEMS = [it if it[0] != "roll" else ("roll", 0.072, 0.028, (0.0, -0.125, 0.0215), 0.0, 0.008, (0.0, 0.0), 22.0) for it in ITEMS]
    if mode in ("set_down", "stack_prop"):
        ITEMS = ITEMS[:2]
    for name, w, hh, (u, v, zt), yaw, gap, (gx, gy), force in ITEMS:
        dr = d.down(yaw)
        lean = d._as_grip(d.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(float(os.environ.get("PLATE_RT", "25")))) @ dr.rot)
        grip_pt = lambda: ctx.state(name).to_world(np.array([[gx, gy, 0.0]]))[0]
        p = grip_pt()
        if mode == "dropped_wing" and name == "wing_a":  # the wing dropped on the counter on its way, then picked up again and served: 0.25
            yield from r.move(np.array([p[0], p[1], TOP + 0.12]), grip=dr, finger=d.FINGER_OPEN, t=2.0, label="right: over the wing")
            yield from r.move(np.array([p[0], p[1], p[2] + hh]), t=1.5, label="right: down to the wing")
            yield from r.close(w, force, t=0.7, label="right: take the wing")
            yield from r.move(np.array([0.12, -0.40, TOP + 0.10]), t=1.5, label="right: off the pan")
            yield from r.move(np.array([0.12, -0.40, TOP + 0.025]), t=1.0, label="right: drop it on the counter")
            yield from r.open(t=0.5, label="right: drop it")
            yield from r.move(r.at() + 0.08 * up, t=0.6, label="right: up")
            yield from r.wait(0.5)
            p = grip_pt()
        yield from r.move(np.array([p[0], p[1], TOP + 0.12]), grip=dr, finger=d.FINGER_OPEN, t=2.0, label=f"right: over the {name}")
        yield from r.move(np.array([p[0], p[1], p[2] + hh]), t=1.5, label=f"right: down to the {name}")
        yield from r.close(w, force, t=0.7, label=f"right: take the {name}")
        yield from r.move(np.array([p[0], p[1], TOP + HIGH]), t=1.2, label=f"right: lift the {name}")
        tw = ctx.state("plate").to_world(np.array([[u + gx, v + gy, zt]]))[0]
        yield from r.move(np.array([tw[0], tw[1], max(TOP + HIGH, tw[2] + hh + 0.045)]), grip=lean, t=2.0, label="right: over the plate")  # above the held rim
        yield from r.move(np.array([tw[0], tw[1], tw[2] + hh + 0.045]), t=1.0, label="right: down to the plate")
        tw = ctx.state("plate").to_world(np.array([[u + gx, v + gy, zt]]))[0]
        yield from r.move(np.array([tw[0], tw[1], tw[2] + hh + 0.006]), t=1.5, label=f"right: lay the {name}")
        yield from r.move(r.at(), finger=-(w / 2 + gap), t=0.5, label=f"right: let the {name} go")  # open just clear of it: wide fingers sweep the next item
        yield from r.move(r.at() + np.array([0.0, 0.0, 0.05]), t=0.8, label="right: up and clear")
        if LEVEL:
            yield from level_plate(ctx, l)
        print(f"[plate] {name} laid at {np.round(ctx.state('plate').to_local(ctx.state(name).pos[None])[0], 3)}: tilt {ctx.state('plate').tilt_deg():.1f}", TRIAL.report, flush=True)


# -- the tongs in the hand ----------------------------------------------------------------------------------------------------------------------------

GRIP_LOCAL = np.array([0.0, GRIP_Y, 0.006])  # the TCP on the tongs, in their frame: between the arms, 11.5 cm from the hinge
REF_LOCAL = np.array([0.0, TIP_C[1], 0.0])  # the tips' round-end centres' midpoint, on the tongs' axis


def _rot(axis, deg):
    a = math.radians(deg)
    c, s_ = math.cos(a), math.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]]), "y": np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]]),
            "z": np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])}[axis]


LIE = _rot("z", TONGS_YAW)  # the tongs where they lie: tips toward -x, opening along y
TURN = _rot("y", -90.0)  # in the hand, the tongs turned from where they lay to tips straight down, opening along y
PICK_TILT = 20.0  # the grip on the tongs where they lie: tilted this far from straight down toward their tips, the fingers closing across the arms
# (the gripper turned over about its own axis: the unturned grip is past the right wrist's limits here; IK checked from READY)


def pick_grip():
    import demokit as d

    return d.tilted(PICK_TILT, 180.0, flip=True)


def held_grip():
    """The grip that holds the tongs tips down: the pick turned with them (the fingers then point +x, 20 degrees down)."""
    import demokit as d

    return d._as_grip(TURN @ pick_grip().rot)


REL = {"v": None}  # the tips' midpoint from the commanded TCP, in the held grip's frame, measured once the tongs are in hand


def half_outer(tip_half: float) -> float:
    """Half the arms' outside width at the grip when the tips' inner faces are `tip_half` off the axis (the finger command, negated)."""
    f = (-GRIP_Y + ROOT[1]) / (-(TIP_C[1] - ROOT[1]))
    return ROOT[0] + (tip_half + ARM_T / 2 - ROOT[0]) * f + ARM_T / 2


def tcp_for(ref):
    """The commanded TCP that puts the tips' midpoint at `ref`, the tongs held tips down (from where they sit in the hand, as measured)."""
    g = held_grip().rot
    rel = REL["v"] if REL["v"] is not None else g.T @ (TURN @ LIE @ (REF_LOCAL - GRIP_LOCAL))
    return np.asarray(ref, float) - g @ rel


def take_tongs(ctx, r):
    """Down over the tongs where they lie, the fingers either side of the arms' handles 11.5 cm from the hinge; squeeze them shut; lift; turn
    them tips down."""
    t0 = ctx.state("tongs")
    gp = t0.pos + t0.rotation() @ GRIP_LOCAL
    print("[plate] tongs lie: hinge", np.round(t0.pos - [0, 0, TOP], 3), "tips", np.round(tip_ref(t0) - [0, 0, TOP], 3), "jaw", round(t0.deg("jaw"), 1), flush=True)
    up_z = TOP + SEAT_Z + PLATE_H + 0.08  # clear of the stack's rim: from the press, go up first, then across, then down over the towel
    yield from r.move(np.array([r.at()[0], r.at()[1], up_z]), t=1.0, label="right: up clear of the stack")
    yield from r.move(np.array([gp[0], gp[1] - 0.02, up_z]), t=1.5, label="right: across over the towel")
    yield from r.move(gp + [0.0, 0.0, 0.045], grip=pick_grip(), finger=-(half_outer(TIP_C[0]) + 0.012), t=2.5, label="right: over the tongs' handles")
    yield from r.move(gp, t=1.4, label="right: fingers either side of the handles")
    yield from r.move(r.at(), finger=-max(0.0, half_outer(ARM_T / 2) - 0.004), t=1.0, label="right: squeeze the handles: the tips shut")
    yield from r.wait(0.3)
    print("[plate] tongs squeezed: jaw", round(ctx.state("tongs").deg("jaw"), 1), "hinge", np.round(ctx.state("tongs").pos - [0, 0, TOP], 3),
          "tcp", np.round(r.tcp() - [0, 0, TOP], 3), flush=True)
    yield from r.move(gp + [0.0, 0.0, 0.04], t=1.0, label="right: lift the tongs")
    yield from r.move(tcp_for(np.array([0.24, -0.30, TOP + 0.12])), grip=held_grip(), t=2.5, label="right: turn the tongs tips down")
    yield from r.wait(0.4)
    REL["v"] = held_grip().rot.T @ (tip_ref(ctx.state("tongs")) - r.at())
    print("[plate] tongs in hand: tips at", np.round(tip_ref(ctx.state("tongs")) - [0, 0, TOP], 3), "jaw", round(ctx.state("tongs").deg("jaw"), 1),
          "rel", np.round(REL["v"], 3), flush=True)


# the tongs' serve: (name, the item's half width across the jaws at the grip point, the tips' lower end above the item's bottom, target (u, v, z
# of the item's bottom in the plate's frame), the grip point in the item's frame (x, y), the squeeze past contact at the tips (m))
TONG_ITEMS = [
    ("steak", 0.041, 0.004, (0.010, -0.058, 0.012), (0.0, 0.0), 0.008),  # across its middle (8.2 cm): the open tips span 9.6 cm of their 10.8
    ("hotdog", 0.030, 0.006, (0.010, 0.038, 0.012), (0.0, 0.0), 0.006),
    ("roll", 0.038, 0.012, (0.050, -0.062, 0.012 + 0.022), (0.0, 0.0), 0.008),
    ("wing_b", 0.016, 0.006, (-0.035, -0.058, 0.012 + 0.022), (-0.008, 0.004), 0.008),
    ("wing_a", 0.016, 0.008, (0.0, 0.090, 0.012), (-0.008, 0.004), 0.008),
]
TONG_OPEN = float(os.environ.get("PLATE_TOPEN", "0.007"))  # the tips this far clear of each side of an item, coming down round it
TONG_RELEASE = float(os.environ.get("PLATE_TREL", "0.004"))  # and eased this far clear to let it go
TONG_HIGH = float(os.environ.get("PLATE_THIGH", "0.18"))  # the tips' lower end this high above the counter over the plate
TONG_LOW = float(os.environ.get("PLATE_TLOW", "0.105"))  # and this high from the pan to beside the plate (the item clear of the pan walls)


def serve_tongs(ctx, mode):
    """Each item: tips opened round it, lowered beside it, closed on it, lifted, carried over the held plate, lowered, eased open, lifted."""
    l, r = ctx.left, ctx.right
    items = TONG_ITEMS
    if mode == "roll_over_edge":  # the roll set on the rim, half past the plate's edge: 0.25
        items = [it if it[0] != "roll" else ("roll", 0.038, 0.012, (0.0, -0.125, 0.0215), (0.0, 0.0), 0.008) for it in items]
    if mode in ("set_down", "stack_prop"):
        items = items[:2]
    live = lambda: tip_ref(ctx.state("tongs"))  # noqa: E731
    for name, hw, lowz, (u, v, zt), (gx, gy), sq in items:
        def grip_ref():
            it = ctx.state(name)
            p = it.to_world(np.array([[gx, gy, 0.0]]))[0]
            return np.array([p[0], p[1], p[2] + lowz + TIP_R])  # the round ends' centres: the lower end lowz above the item's bottom
        for attempt in range(2 if (mode == "dropped_wing" and name == "wing_a") else 1):
            g = grip_ref()
            yield from r.move(tcp_for(g + [0.0, 0.0, 0.10]), finger=-half_outer(hw + TONG_OPEN), t=2.0, label=f"right: tongs over the {name}")
            yield from r.servo(g + [0.0, 0.0, 0.10], read=live, tol=0.002, timeout=1.2, label=f"right: tips over the {name}")
            yield from r.move(r.at() - [0.0, 0.0, 0.10], t=1.5, label=f"right: tips down round the {name}")
            yield from r.servo(g, read=live, tol=0.0015, timeout=1.2, label=f"right: tips beside the {name}")
            yield from r.move(r.at(), finger=-(half_outer(hw) - sq), t=0.8, label=f"right: close the tongs on the {name}")
            yield from r.wait(0.3)
            yield from r.move(r.at() + [0.0, 0.0, TOP + TONG_LOW - g[2] + TIP_R], t=1.2, label=f"right: lift the {name}")
            if mode == "dropped_wing" and name == "wing_a" and attempt == 0:  # dropped on the counter on the way, picked up again: 0.25
                yield from r.move(tcp_for(np.array([0.14, -0.40, TOP + 0.05])), t=2.0, label="right: over the counter")
                yield from r.move(r.at(), finger=-half_outer(hw + TONG_OPEN), t=0.4, label="right: the wing drops")
                yield from r.move(r.at() + [0.0, 0.0, 0.08], t=0.8, label="right: up")
                yield from r.wait(0.8)
        tw = ctx.state("plate").to_world(np.array([[u + gx, v + gy, zt]]))[0]
        tgt = np.array([tw[0], tw[1], tw[2] + lowz + TIP_R + 0.004])
        pc = ctx.state("plate").pos
        yield from r.move(tcp_for(np.array([tgt[0], pc[1] - 0.20, TOP + TONG_LOW + TIP_R])), t=1.8, label=f"right: the {name} beside the plate")
        high = TONG_HIGH + (0.05 if name == "wing_a" else 0.0)  # the last wing crosses the hot dog (its top about 19 cm up) on the way to the far side
        yield from r.move(tcp_for(np.array([tgt[0], pc[1] - 0.20, TOP + high + TIP_R])), t=1.0, label=f"right: the {name} up over the rim")
        yield from r.move(tcp_for(np.array([tgt[0], tgt[1], TOP + high + TIP_R])), t=1.5, label=f"right: the {name} over the plate")
        yield from r.servo(np.array([tgt[0], tgt[1], TOP + high + TIP_R]), read=live, tol=0.003, timeout=1.0, label="right: over its place")
        tw = ctx.state("plate").to_world(np.array([[u + gx, v + gy, zt]]))[0]
        tgt = np.array([tw[0], tw[1], tw[2] + lowz + TIP_R + 0.004])
        last = name == items[-1][0]
        if last and mode == "latch_then_rest":  # the last item held 1 cm above its place, still in the tongs, then lifted away: 0
            yield from r.move(r.at() + (tgt + [0.0, 0.0, 0.010] - live()), t=2.0, label=f"right: hold the {name} just over its place")
            yield from r.wait(1.0)
            yield from r.move(r.at() + [0.0, 0.0, 0.10], t=1.2, label=f"right: lift the {name} away, still in the tongs")
            yield from r.move(tcp_for(np.array([tgt[0], pc[1] - 0.20, TOP + high + TIP_R])), t=1.5, label=f"right: the {name} beside the plate")
            return
        yield from r.move(r.at() + (tgt - live()), t=2.0, label=f"right: lay the {name}")
        if last and mode == "last_in_tongs":  # the last item on the plate, the tongs never let it go: 0.25
            print(f"[plate] {name} left in the tongs", TRIAL.report, flush=True)
            return
        yield from r.move(r.at(), finger=-half_outer(hw + TONG_RELEASE), t=0.5, label=f"right: ease the tongs off the {name}")
        yield from r.move(r.at() + [0.0, 0.0, 0.08], t=1.0, label="right: tips up and clear")
        if LEVEL:
            yield from level_plate(ctx, l)
        print(f"[plate] {name} laid at {np.round(ctx.state('plate').to_local(ctx.state(name).pos[None])[0], 3)}: tilt {ctx.state('plate').tilt_deg():.1f}", TRIAL.report, flush=True)


def demo_script(ctx):
    """Right: press the top plate's far rim down so its near rim tips up off the plate below. Left: level jaws round the raised near rim, pinch, lift.
    Right: takes the tongs and serves five items from the pans onto the held plate with them. Left: set the plate in the zone."""
    import demokit as d

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}")
    for _ in range(5):
        yield
    if mode == "none":
        yield from l.wait(2.0)
        return
    up = np.array([0.0, 0.0, 1.0])
    ua = np.array([math.cos(math.radians(PRESS_AZ)), math.sin(math.radians(PRESS_AZ)), 0.0])
    a0 = np.array([math.cos(math.radians(GRIP_PHI)), math.sin(math.radians(GRIP_PHI)), 0.0])
    pl = ctx.state("plate")
    far = pl.pos + 0.120 * ua + np.array([0.0, 0.0, TOP_PROFILE[-2][1]])
    yield from r.move(far + 0.06 * up, grip=d.down(0), finger=d.FINGER_CLOSED, t=2.5, label="right: over the far rim")
    yield from r.move(far + 0.015 * up, t=1.0, label="right: onto the far rim")
    yield from r.press(-up, force=float(os.environ.get("PLATE_PF", "15")), max_travel=0.03, label="right: press the far rim down")
    print("[plate] tipped", round(ctx.state("plate").tilt_deg(), 1), flush=True)
    # the left hand: fingers level, the jaws closing up and down across the raised near rim, centred on the rim's height in the jaws
    g = rim_grip(GRIP_PHI, float(os.environ.get("PLATE_PITCH", "0")), ctx.state("plate").rotation()[:, 2] if ROLL else None)
    a = g.rot[:, 2]
    def pinch_point():
        p = ctx.state("plate")
        under = p.to_world(np.array([[-0.107 * a0[0], -0.107 * a0[1], UNDER_PROFILE[2][1]]]))[0]
        top = p.to_world(np.array([[-R_EDGE * a0[0], -R_EDGE * a0[1], TOP_PROFILE[-1][1]]]))[0]
        edge = p.to_world(np.array([[-GRIP_R * a0[0], -GRIP_R * a0[1], 0.0]]))[0]
        return np.array([edge[0], edge[1], 0.5 * (under[2] + top[2]) + float(os.environ.get("PLATE_ZOFF", "0"))])
    tcp0 = pinch_point()
    yield from l.move(tcp0 - BACK * a + 0.03 * up, grip=g, finger=-GAP / 2, t=3.0, label="left: beside the near rim")
    yield from l.move(tcp0 - BACK * a, t=1.0, label="left: level with the rim")
    yield from l.servo(tcp0 - BACK * a, tol=0.0015, timeout=1.0, label="left: level with the rim")
    tcp0 = pinch_point()
    yield from l.move(tcp0, t=1.5, label="left: fingers round the rim")
    yield from l.servo(tcp0, tol=0.0015, timeout=1.0, label="left: fingers round the rim")
    yield from l.close(0.0, float(os.environ.get("PLATE_SQ", "40")), t=0.8, label="left: pinch the rim")
    yield from l.wait(0.3)
    print("[plate] pinched", ctx.state("plate").pos, round(ctx.state("plate").tilt_deg(), 1), flush=True)
    yield from r.move(r.tcp() + 0.06 * up, t=1.0, label="right: off the rim")  # from where the hand is: the press left its command below the rim
    yield from l.wait(0.3)
    yield from l.move(l.at() + LIFT * up, t=2.0, label="left: lift off the stack")
    yield from l.wait(0.5)
    if mode != "bare_hands":  # the right hand takes the tongs off the towel and turns them tips down
        yield from take_tongs(ctx, r)
    if CARRY_Y:  # bring the held plate toward the serving hand
        yield from l.move(l.at() + np.array([0.0, CARRY_Y, -HOLD_DOWN]), t=1.5, label="left: bring the plate to the pans")
        yield from l.wait(0.5)
    if LEVEL:  # the held plate sags and leans in the pinch: bring it level before the food goes on
        yield from level_plate(ctx, l, steps=4)
    print("[plate] held", ctx.state("plate").pos, round(ctx.state("plate").tilt_deg(), 1), TRIAL.report, flush=True)
    if mode == "idle_server":  # the plate held up, the other hand never serves: 0.25
        yield from l.wait(5.0)
        return
    if mode == "set_down":  # the plate put back on the stack and loaded there (as a one-armed server must), then lifted: void, 0
        yield from l.move(l.at() - (LIFT - 0.002) * up, t=2.0, label="left: set the plate back on the stack")
        yield from l.wait(0.5)
    if mode == "stack_prop":  # held, but its foot propped on the stack's rim while it is loaded, then lifted: 0.25 (the food slides off the tilted plate; void if it stays on)
        pc = ctx.state("plate").pos
        yield from l.move(l.at() + np.array([0.0, PLATE_START[1] - (FOOT_R + R_EDGE - 0.012) - pc[1], 0.0]), t=1.5, label="left: the plate beside the stack")
        z0 = l.at()[2]
        for _ in range(40):
            if TRIAL.report.get("on_stack"):
                break
            yield from l.move(l.at() - 0.002 * up, t=0.1, label="left: lower the foot onto the stack's rim")
        yield from l.move(l.at() - 0.003 * up, t=0.3, label="left: lean on the rim")
        yield from l.wait(0.5)
        print("[plate] propped", TRIAL.report, flush=True)

    # -- the right hand: five items from the pans onto the held plate, with the tongs
    if mode == "bare_hands":  # the bare fingers serve all five (the hand touches the food, which then never counts): 0
        yield from serve_bare(ctx, mode)
    else:
        yield from serve_tongs(ctx, mode)
    if mode == "last_in_tongs":  # held to the end, the last item still in the tongs on the plate
        yield from l.wait(4.0)
        return
    if mode != "latch_then_rest":
        yield from r.move(np.array([0.22, -0.30, TOP + 0.30]), t=1.5, label="right: tongs up and clear")
    if mode == "set_down":
        yield from l.move(l.at() + LIFT * up, t=2.0, label="left: lift the loaded plate")
    if mode == "stack_prop":
        yield from l.move(np.array([l.at()[0], l.at()[1], z0]), t=2.0, label="left: lift the loaded plate off the rim")
        yield from l.wait(4.0)
        return
    if mode in ("roll_over_edge", "dropped_wing", "no_handoff"):  # held to the end, never set on the mat
        yield from l.wait(4.0)
        return
    # -- the left hand sets the plate on the mat
    # up first: the left gripper's housing hangs below the jaws, beside the stack's top plates, and would drag across them
    yield from l.move(l.at() + CARRY_UP * up, t=1.5, label="left: up off the stack")
    pc = ctx.state("plate").pos
    span = np.array([SET_AT[0] - pc[0], SET_AT[1] - pc[1], 0.0])
    n = int(math.ceil(np.linalg.norm(span) / 0.05))
    for _ in range(n):  # in 5 cm steps, each solved from the last, levelled after each: the loaded arm lags as its base turns past the plate
        yield from l.move(l.at() + span / n, t=1.8, label="left: carry over the mat")
        yield from level_plate(ctx, l, steps=3)
    yield from level_plate(ctx, l)
    # the palm and housing reach 5 cm below the jaws' axis: lowered level, they meet the counter before the plate meets the mat and the plate tips over.
    # So the plate goes down far edge first: the wrist pitches the fingers down about the pinch, the far edge settles on the mat, the jaws open to the
    # approach gap so the near rim rests on the lower finger, and the hand pitches further while it sinks, so the rim comes down on the finger.
    def low_of_plate():
        return float(ctx.state("plate").to_world(PLATE_PTS)[:, 2].min() - TOP)

    def pitched(deg):
        a_h = l.grip.rot[:, 2] * np.array([1.0, 1.0, 0.0])
        a_h = a_h / np.linalg.norm(a_h)
        return d._as_grip(d.axis_angle_matrix(np.cross(up, a_h), math.radians(deg)) @ l.grip.rot)

    if mode == "drop_on_mat":  # let go 4 cm above the mat, a drop: 0
        for _ in range(160):
            if low_of_plate() <= MAT_H + 0.04:
                break
            yield from l.move(l.at() - np.array([0.0, 0.0, 0.002]), t=0.1, label="left: lower")
        yield from l.move(l.at(), finger=-GAP / 2, t=0.3, label="left: let go")
    else:
        yield from l.move(l.at(), grip=pitched(SET_P1), t=1.0, label="left: far edge down")
        for _ in range(160):
            if low_of_plate() <= MAT_H + 0.002:
                break
            yield from l.move(l.at() - np.array([0.0, 0.0, 0.002]), t=0.1, label="left: lower the far edge to the mat")
        print(f"[plate] far edge down: tilt {ctx.state('plate').tilt_deg():.1f}, low {low_of_plate() * 1000:.0f} mm", flush=True)
        # the loaded arm sags under the plate: let go in place, it springs up and the lower finger flicks the near rim up. So the hand goes down by
        # its sag while the jaws open, and stays where it is
        sag = max(0.0, float((l.at() + l.trim - l.tcp())[2]) - 0.012)
        yield from l.move(l.at() - np.array([0.0, 0.0, sag]), finger=-GAP / 2, t=0.8, label="left: open to the approach gap")
        g1 = l.grip
        for k in range(160):
            if ctx.state("plate").tilt_deg() < 1.5 and float(ctx.state("plate").pos[2] - TOP) <= MAT_H + 0.002:
                break
            if l.at()[2] - TOP < 0.0:
                break
            f = min(1.0, (k + 1) / 15)
            l.grip = g1
            yield from l.move(l.at() - np.array([0.0, 0.0, 0.002]), grip=pitched(f * (SET_P2 - SET_P1)), t=0.1, label="left: sink the near rim")
        print(f"[plate] set: tilt {ctx.state('plate').tilt_deg():.1f}, centre {(ctx.state('plate').pos[2] - TOP) * 1000:.0f} mm", flush=True)
    yield from l.wait(0.3)
    back = l.grip.rot[:, 2] * np.array([1.0, 1.0, 0.0])
    yield from l.move(l.at() - 0.10 * back / np.linalg.norm(back), t=1.5, label="left: draw back")  # fingertips well outside the rim before going up
    yield from l.move(l.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="left: up")
    if mode == "latch_then_rest":  # the last item laid only once the plate rests on the mat: void, 0
        name, hw, lowz, (u, v, zt), (gx, gy), sq = TONG_ITEMS[-1]
        tw = ctx.state("plate").to_world(np.array([[u + gx, v + gy, zt]]))[0]
        tgt = np.array([tw[0], tw[1], tw[2] + lowz + TIP_R + 0.004])
        hi = float(tip_ref(ctx.state("tongs"))[2])
        yield from r.move(tcp_for(np.array([tgt[0], tgt[1], hi])), t=2.5, label=f"right: the {name} over the plate on the mat")
        yield from r.move(r.at() + (tgt - tip_ref(ctx.state("tongs"))), t=2.0, label=f"right: lay the {name} on the resting plate")
        yield from r.move(r.at(), finger=-half_outer(hw + TONG_RELEASE), t=0.5, label=f"right: ease the tongs off the {name}")
        yield from r.move(r.at() + [0.0, 0.0, 0.08], t=1.0, label="right: tips up and clear")
    yield from l.wait(3.0)
    print("[plate] end:", TRIAL.report, flush=True)
