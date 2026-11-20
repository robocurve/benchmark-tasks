"""Pull the hot dog past its hold time off a running roller grill with spring tongs, and drop it in the discard pan.

An Avantco RG1830NS roller grill (11 non-stick rollers, 58.1 x 46.4 x 22.2 cm) runs on the counter, its rollers turning at 3 rpm.
Twelve franks lie in grooves 2, 4, 6 and 8; a red hold-time tag on the front rail marks the one in groove 2 at the right end, whose
hold time is up. 9 in stainless utility tongs lie on a folded bar towel beside the grill, and a 1/3-size stainless food pan stands
between them and the grill as the discard pan. The right hand takes the tongs, closes their tips on the tagged frank at its middle while the
rollers keep it turning, lifts it straight up out of its groove, carries it to the pan, lets it go there, and lays the tongs back on the towel (not scored).

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --stills 40 --mode full
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
INSTRUCTION = "Use the tongs to take the tagged hot dog off the roller grill and drop it in the discard pan."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold: score() and success() agree
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the roller grill: Avantco RG1830NS, station metres (x forward from the arm bases, y left, z up from the counter, the table top) --
GRILL_X = (0.26, 0.724)  # 46.4 cm deep (18 1/4 in)
GRILL_Y = (-0.18, 0.401)  # 58.1 cm long (22 7/8 in)
GRILL_H = 0.222  # 22.2 cm tall (8 3/4 in), the end housings' tops
N_ROLLERS = 11
ROLLER_R = 0.0125  # 25 mm across (assumed; range 25 to 28)
PITCH = 0.034  # (assumed; range 32 to 35 mm)
ROLLER_X0 = 0.29  # the front roller's axis
ROLLER_TOP = 0.19  # (estimate; range 17 to 21 cm)
ROLLER_Z = ROLLER_TOP - ROLLER_R
ROLLER_Y = (-0.148, 0.348)  # each 49.6 cm long, between the end housings
RPM = 3.0  # (assumed; range 2 to 6): every roller turns the same way, its top surface moving toward the back
OMEGA = RPM * 2 * math.pi / 60
ROLLERS_X = [ROLLER_X0 + k * PITCH for k in range(N_ROLLERS)]

# -- the franks: standard beef franks, 8 to the pound, as rigid capsules 2.2 cm across and 15 cm long
FRANK_R, FRANK_L, FRANK_M = 0.011, 0.150, 0.057
FRANK_Z = ROLLER_Z + math.sqrt((ROLLER_R + FRANK_R) ** 2 - (PITCH / 2) ** 2)  # its centre, resting on two rollers: 3.7 mm above their tops
GROOVES = (2, 4, 6, 8)
FRANK_Y = (-0.070, 0.095, 0.260)


def groove_x(k: int) -> float:
    return ROLLER_X0 + (k - 0.5) * PITCH


TAGGED = "frank_2a"  # groove 2, at the right end: its hold time is up
FRANKS = {f"frank_{k}{'abc'[i]}": (groove_x(k), y) for k in GROOVES for i, y in enumerate(FRANK_Y)}

# -- the tongs: 9 in stainless utility tongs, scalloped tips; their own frame has the hinge at the origin, the arms along -y, opening along x
ARM_T, ARM_W, TIP_W = 0.0025, 0.020, 0.025  # arms 20 mm wide, modelled 2.5 mm thick (real about 1 mm); tips 25 mm wide
ROOT = (0.0075, -0.006)  # where each arm leaves the hinge block
TIP_C = (0.035, -0.2085)  # the centre of each tip's rounded end, the tongs open: tips 7 cm apart
TIP_LEN = 0.050  # the flat scalloped tip
HINGE_R = 0.009  # the hinge block and its spring, 18 mm across
TONG_MASS = (0.060, 0.050)  # the arm with the hinge, and the other: 110 g
SPRING = 8.0  # N m/rad at the hinge: about 4 N at the tips held 5 cm apart, 8 N closed on a frank (estimate)
CLOSE_DEG = 18.5  # the free arm's travel toward closed: at the stop the tips meet (the arms' own contact, as a joint limit)
GRIP_Y = -0.110  # the hand holds the tongs 11 cm from the hinge
TONGS_AT = np.array([0.53, -0.46])  # the hinge, the tongs lying on their arms' edges, tips toward -x
TONGS_YAW = -90.0
TOWEL = ((0.27, 0.57), (-0.51, -0.41), 0.010)  # a folded cotton bar towel under them, 30 x 10 cm

# -- the discard pan: a 1/3-size stainless steam-table pan, 12 3/4 x 6 7/8 x 4 in deep, long side along x
PAN_C = np.array([0.44, -0.30])
PAN_HX, PAN_HY, PAN_H = 0.162, 0.0875, 0.102
PAN_WALL, PAN_MASS = 0.004, 0.50

# -- the scorer's thresholds
TOUCH = 0.001  # a tong tip within 1 mm of the frank's surface is in contact with it
LIFT = 0.04  # the frank's centre 4 cm above where it lay
TONGS_UP = 0.05  # the tongs 5 cm above where they lay, a finger on them
FINGER_NEAR = 0.006  # a gripper finger point within 6 mm of the frank's surface: the hand is on the frank, not the tongs
SQUEEZE_COS = -0.9  # the two tips' contact points lie on opposite sides of the frank's axis: at least 154 degrees apart round it
NEAR = 0.005  # a part of the tongs within 5 mm of the frank touches it
FALL = 0.03  # the tagged frank let go and falling 3 cm, anywhere but into the pan
G = 9.81
FREE_A = 0.6  # a point speeding up downward at 0.6 g or more over a physics step: nothing holds it up
FREE_N = 3  # physics steps (12.5 ms) in a row of that before a fall counts
LAND_LOSS = 0.5  # a falling point that loses more than half its downward speed in one physics step has landed
CHAIN_S = 10  # steps (0.5 s) the tips may be off the frank between its lift and its reaching the pan: a carry, not a set-back and a push
STILL_V = 0.005  # m/s
SETTLE_STEPS = 20  # the franks' start places are read at 1 s, once they have settled in their grooves

LOOKS = {
    "steel": m.Look(color=m.rgb(0.70, 0.71, 0.73), metallic=0.9, roughness=0.3),
    "brushed": m.Look(color=m.rgb(0.62, 0.63, 0.65), metallic=0.85, roughness=0.45),
    "roller": m.Look(color=m.rgb(0.10, 0.10, 0.11), metallic=0.3, roughness=0.35),
    "dark": m.Look(color=m.rgb(0.06, 0.06, 0.07), roughness=0.6),
    "frank": m.Look(color=m.rgb(0.58, 0.24, 0.13), roughness=0.4),
    "old": m.Look(color=m.rgb(0.33, 0.13, 0.08), roughness=0.6),
    "red": m.Look(color=m.rgb(0.85, 0.08, 0.08), roughness=0.5),
    "white": m.Look(color=m.rgb(0.95, 0.95, 0.95), roughness=0.6),
    "towel": m.Look(color=m.rgb(0.90, 0.90, 0.86), roughness=0.95),
    "stripe": m.Look(color=m.rgb(0.20, 0.35, 0.65), roughness=0.95),
    "knob": m.Look(color=m.rgb(0.08, 0.08, 0.08), roughness=0.4),
}


def _bx(x, y, z) -> m.Mesh:
    return m.box((x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def _cbox(name, x, y, z) -> str:
    return m.col_box(name, (x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def _col_axis(kind: str, name: str, radius: float, height: float, center, axis: str = "Y") -> str:
    """An implicit cylinder or capsule collider along `axis` (mechkit's are along z): smooth, so a frank rolls on it."""
    text = m.col_cylinder(name, radius, height, center).replace('uniform token axis = "Z"', f'uniform token axis = "{axis}"')
    return text.replace('def Cylinder', 'def Capsule') if kind == "capsule" else text


# -- the grill: housing (fixed) and rollers (a mechanism, each roller a driven joint) -----------------------------------------------

def _grill():
    x, y = GRILL_X, GRILL_Y
    base_top = 0.150
    parts = [("base", _bx(x, y, (0.012, base_top)), "steel"),
             ("drip", _bx((0.275, 0.65), (ROLLER_Y[0], ROLLER_Y[1]), (base_top, base_top + 0.004)), "dark"),
             ("end_r", _bx(x, (y[0], ROLLER_Y[0] - 0.004), (base_top, GRILL_H)), "brushed"),
             ("end_l", _bx(x, (ROLLER_Y[1] + 0.004, y[1]), (base_top, GRILL_H)), "brushed"),
             ("rail", _bx((x[0], 0.272), (ROLLER_Y[0] - 0.004, ROLLER_Y[1] + 0.004), (base_top, 0.178)), "steel"),
             ("back", _bx((0.652, x[1]), (ROLLER_Y[0] - 0.004, ROLLER_Y[1] + 0.004), (base_top, 0.200)), "steel")]
    for k, yy in enumerate((y[0] + 0.03, y[1] - 0.03)):  # rubber feet
        for xx in (x[0] + 0.03, x[1] - 0.03):
            parts.append((f"foot{k}{int(xx * 100)}", m.cylinder(0.012, 0.012, (xx, yy, 0.006)), "dark"))
    for k, zz in enumerate((0.075, 0.115)):  # the two heat zones' controls, on the right end's front face
        parts.append((f"knob{k}", m.cylinder(0.011, 0.012, (x[0] - 0.006, -0.12, zz), axis="x"), "knob"))
    parts.append(("switch", _bx((x[0] - 0.006, x[0]), (-0.06, -0.04), (0.085, 0.105)), "red"))
    # the hold-time tag: a red polypropylene clip tag, 50 x 38 mm, clipped on the front rail in line with the tagged frank
    ty = FRANKS[TAGGED][1]
    parts.append(("tag", _bx((x[0] - 0.004, x[0] - 0.002), (ty - 0.025, ty + 0.025), (0.140, 0.178)), "red"))
    parts.append(("tag_label", _bx((x[0] - 0.0045, x[0] - 0.004), (ty - 0.018, ty + 0.018), (0.150, 0.168)), "white"))
    parts.append(("tag_clip", _bx((x[0] - 0.004, 0.272), (ty - 0.008, ty + 0.008), (0.174, 0.180)), "dark"))
    cols = [_cbox("base_c", x, y, (0.0, base_top)), _cbox("end_r_c", x, (y[0], ROLLER_Y[0] - 0.004), (base_top, GRILL_H)),
            _cbox("end_l_c", x, (ROLLER_Y[1] + 0.004, y[1]), (base_top, GRILL_H)),
            _cbox("rail_c", (x[0], 0.272), (ROLLER_Y[0] - 0.004, ROLLER_Y[1] + 0.004), (base_top, 0.178)),
            _cbox("back_c", (0.652, x[1]), (ROLLER_Y[0] - 0.004, ROLLER_Y[1] + 0.004), (base_top, 0.200))]
    return m.fixed("grill", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.4))


def _rollers():
    length = ROLLER_Y[1] - ROLLER_Y[0]
    yc = (ROLLER_Y[0] + ROLLER_Y[1]) / 2
    mass = 0.25
    links = []
    for k, rx in enumerate(ROLLERS_X):
        c = (rx, yc, ROLLER_Z)
        links.append(m.hinge(f"r{k}", "Y", c, -3.6e6, 3.6e6, [("tube", m.cylinder(ROLLER_R, length, c, axis="y", segments=40), "roller")],
                             [_col_axis("cylinder", f"r{k}_c", ROLLER_R, length, c)], mass=mass, damping=4.0, effort=3.0, friction=0.0,
                             armature=1e-3, inertia=(mass * length ** 2 / 12, mass * ROLLER_R ** 2 / 2, mass * length ** 2 / 12)))
    anchor = [("anchor", _bx((0.66, 0.67), (0.0, 0.01), (0.152, 0.160)), "dark", True)]
    return m.mechanism("rollers", (0.0, 0.0, TOP), links, LOOKS, frame_parts=anchor, friction=(0.5, 0.4), solver_iters=32)


# -- the franks -------------------------------------------------------------------------------------------------------------------

def _frank(name, xy):
    h = FRANK_L - 2 * FRANK_R
    prof = [(0.0, -FRANK_L / 2)] + [(FRANK_R * math.sin(a), -h / 2 - FRANK_R * math.cos(a)) for a in np.linspace(0.3, math.pi / 2, 5)] + \
           [(FRANK_R * math.sin(a), h / 2 + FRANK_R * math.cos(a)) for a in np.linspace(math.pi / 2, 0.3, 5)] + [(0.0, FRANK_L / 2)]
    body = m.lathe(prof, 24).turned(-90.0, "x")  # along y
    look = "old" if name == TAGGED else "frank"
    return m.free(name, (xy[0], xy[1], TOP + FRANK_Z + 0.0003), [("body", body, look)], [_col_axis("capsule", "c", FRANK_R, h, (0.0, 0.0, 0.0))],
                  LOOKS, mass=FRANK_M, friction=(0.6, 0.5), torsional_patch=None)


# -- the tongs: a free mechanism, the frame one arm with the hinge, the other arm on a sprung hinge -----------------------------------

ARM_ANGLE = math.atan2(TIP_C[0] - ROOT[0], -(TIP_C[1] - ROOT[1]))  # each arm's angle off the tongs' axis, open
ARM_S = math.hypot(TIP_C[0] - ROOT[0], TIP_C[1] - ROOT[1])  # from the hinge block to the tip's round end's centre
TIP_R = TIP_W / 2


def _arm_local(side: float):
    """One arm in the tongs' frame at joint zero: (visual pieces, colliders as meshes). side -1: the frame's arm (tip at -x), +1: the free one."""
    end = ARM_S + TIP_R
    plate0 = end - TIP_LEN
    pieces = []
    for a, b in ((0.0, 0.065), (0.065, 0.125), (0.125, plate0 + 0.002)):
        pieces.append(m.box((ARM_T, b - a, ARM_W), (0.0, -(a + b) / 2, 0.0)))
    # the scalloped tip: a flat plate 25 mm wide with a rounded end, built in (y, z) and extruded across x
    th = np.linspace(math.pi / 2, 3 * math.pi / 2, 13)
    outline = [(-plate0, -TIP_R), (-plate0, TIP_R)] + [(-ARM_S + TIP_R * math.cos(t), TIP_R * math.sin(t)) for t in th]
    plate = m.prism(outline, -ARM_T / 2, ARM_T / 2)
    plate = m.Mesh(plate.v[:, [2, 0, 1]], plate.f)
    pieces.append(plate)
    beta = side * ARM_ANGLE

    def place(mesh):
        return mesh.turned(math.degrees(beta), "z").moved((side * ROOT[0], ROOT[1], 0.0))
    return [place(p) for p in pieces]


def tip_local(side: float, q: float = 0.0, pts=None) -> np.ndarray:
    """Points on a tip's inner face in the tongs' frame, the free arm turned `q` rad: by default a grid over the whole flat tip, from the
    plate's back (50 mm from the end) to its round end, edge to edge, and points round the round end."""
    if pts is None:
        plate0 = ARM_S + TIP_R - TIP_LEN
        pts = [(s, z) for s in np.linspace(plate0, ARM_S, 6) for z in (-TIP_R, -0.009, -0.004, 0.0, 0.004, 0.009, TIP_R)]
        pts += [(ARM_S + 0.006, z) for z in (-0.009, -0.004, 0.0, 0.004, 0.009)]
        pts += [(ARM_S + TIP_R * math.cos(a), TIP_R * math.sin(a)) for a in np.linspace(-1.2, 1.2, 7)]
    beta = side * ARM_ANGLE
    d = np.array([math.sin(beta), -math.cos(beta)])
    n = np.array([math.cos(beta), math.sin(beta)])  # local +x of the arm
    root = np.array([side * ROOT[0], ROOT[1]])
    out = []
    for s, z in pts:
        p = root + s * d - side * n * (ARM_T / 2)  # the inner face, toward the other arm
        out.append((p[0], p[1], z))
    out = np.array(out)
    if side > 0 and q:
        c, sn = math.cos(q), math.sin(q)
        out = out @ np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]]).T
    return out


def arm_line_local(side: float, q: float = 0.0, n: int = 12) -> np.ndarray:
    """Points along an arm's centre line, hinge to tip."""
    pts = tip_local(side, q, [(s, 0.0) for s in np.linspace(0.0, ARM_S + TIP_R, n)])
    return pts


def _tongs():
    a_pieces, b_pieces = _arm_local(-1.0), _arm_local(1.0)
    loop = m.cylinder(HINGE_R, TIP_W, (0.0, 0.0, 0.0))
    ring = m.tube([(0.0, 0.004, -0.008), (0.0, 0.020, -0.006), (0.0, 0.026, 0.0), (0.0, 0.020, 0.006), (0.0, 0.004, 0.008)], 0.0015, 8)
    frame_parts = [(f"arm_a{i}", p, "steel") for i, p in enumerate(a_pieces)] + [("hinge", loop, "brushed"), ("lock_ring", ring, "steel")]
    frame_cols = [m.col_hull(f"arm_a{i}_c", p) for i, p in enumerate(a_pieces)] + [m.col_cylinder("hinge_c", HINGE_R, TIP_W)]
    com_b = tuple(arm_line_local(1.0)[6])
    jaw = m.hinge("jaw", "Z", (0.0, 0.0, 0.0), -CLOSE_DEG, 0.0, [(f"arm_b{i}", p, "steel") for i, p in enumerate(b_pieces)],
                  [m.col_hull(f"arm_b{i}_c", p) for i, p in enumerate(b_pieces)], mass=TONG_MASS[1], spring=SPRING, damping=0.01, effort=5.0,
                  friction=0.0, armature=1e-4, gravity=True, com=com_b, inertia=(2.0e-4, 5.0e-6, 2.0e-4))
    z0 = TOWEL[2] + TIP_R
    return m.mechanism("tongs", (TONGS_AT[0], TONGS_AT[1], TOP + z0), [jaw], LOOKS, frame_parts=frame_parts, frame_colliders=frame_cols,
                       friction=(0.8, 0.7), solver_iters=64, free_mass=TONG_MASS[0], frame_inertia=(2.5e-4, 6.0e-6, 2.5e-4), yaw_deg=TONGS_YAW)


def _towel():
    (x0, x1), (y0, y1), h = TOWEL
    parts = [("towel", _bx((x0, x1), (y0, y1), (0.0, h)), "towel")]
    for k, yy in enumerate((y0 + 0.02, y1 - 0.02)):
        parts.append((f"stripe{k}", _bx((x0 - 0.0005, x1 + 0.0005), (yy - 0.006, yy + 0.006), (0.0, h + 0.0005)), "stripe"))
    return m.fixed("towel", (0.0, 0.0, TOP), parts, [_cbox("towel_c", (x0, x1), (y0, y1), (0.0, h))], LOOKS, friction=(0.9, 0.8))


def _pan():
    hx, hy, h, w = PAN_HX, PAN_HY, PAN_H, PAN_WALL
    parts = [("floor", _bx((-hx + 0.012, hx - 0.012), (-hy + 0.012, hy - 0.012), (0.0, 0.0015)), "steel")]
    walls = {"wx0": ((-hx, -hx + w), (-hy, hy)), "wx1": ((hx - w, hx), (-hy, hy)), "wy0": ((-hx, hx), (-hy, -hy + w)), "wy1": ((-hx, hx), (hy - w, hy))}
    cols = [_cbox("floor_c", (-hx, hx), (-hy, hy), (0.0, 0.003))]
    for n, (x, y) in walls.items():
        parts.append((n, _bx((x[0] + 0.001, x[1] - 0.001), y, (0.0, h)), "steel"))
        cols.append(_cbox(f"{n}_c", x, y, (0.0, h)))
    parts.append(("floor_rim", _bx((-hx, hx), (-hy, hy), (0.0, 0.001)), "brushed"))
    for n, (x, y) in {"fx0": ((-hx - 0.008, -hx), (-hy - 0.008, hy + 0.008)), "fx1": ((hx, hx + 0.008), (-hy - 0.008, hy + 0.008)),
                      "fy0": ((-hx, hx), (-hy - 0.008, -hy)), "fy1": ((-hx, hx), (hy, hy + 0.008))}.items():
        parts.append((n, _bx(x, y, (h - 0.0015, h)), "steel"))  # the flange
    return m.free("pan", (PAN_C[0], PAN_C[1], TOP + 0.0005), parts, cols, LOOKS, mass=PAN_MASS, friction=(0.5, 0.4))


OBJECTS = [_grill(), _rollers(), _tongs(), _towel(), _pan()] + [_frank(n, xy) for n, xy in FRANKS.items()]


# -- the scorer -------------------------------------------------------------------------------------------------------------------

def capsule_gap(points: np.ndarray, frank) -> np.ndarray:
    """Each point's distance outside a frank's surface."""
    loc = frank.to_local(points)
    h = FRANK_L / 2 - FRANK_R
    c = np.c_[np.zeros(len(loc)), np.clip(loc[:, 1], -h, h), np.zeros(len(loc))]
    return np.linalg.norm(loc - c, axis=1) - FRANK_R


def roller_gap(points: np.ndarray) -> np.ndarray:
    """Each point's distance outside the nearest roller (station frame, z above the counter)."""
    rx = np.array(ROLLERS_X)
    dx = points[:, None, 0] - rx[None, :]
    dz = points[:, None, 2] - ROLLER_Z
    r = np.sqrt(dx ** 2 + dz ** 2).min(axis=1) - ROLLER_R
    inside_y = (points[:, 1] >= ROLLER_Y[0]) & (points[:, 1] <= ROLLER_Y[1])
    return np.where(inside_y, r, np.inf)


def squeezed(pa: np.ndarray, pb: np.ndarray) -> bool:
    """Two tip contact points, as (across, up) offsets from the frank's axis: on opposite sides of it (at least 154 degrees apart round it)
    and one frank diameter apart, so a jaw closed onto the frank. Two tips side by side under it, in a cradle, fail both."""
    ca = float(np.dot(pa, pb)) / (float(np.linalg.norm(pa)) * float(np.linalg.norm(pb)) + 1e-9)
    sep = float(np.linalg.norm(pa - pb))
    return bool(ca <= SQUEEZE_COS and 2 * FRANK_R - 0.002 <= sep <= 2 * FRANK_R + 0.004)


def tong_points(tongs) -> dict:
    """The tongs' tips and arm lines in the station frame (z above the counter)."""
    q = tongs.joint("jaw")
    lift = np.array([0.0, 0.0, TOP])
    w = lambda p: tongs.to_world(p) - lift  # noqa: E731
    return {"a": w(tip_local(-1.0)), "b": w(tip_local(1.0, q)),
            "a_end": w(tip_local(-1.0, 0.0, [(ARM_S, 0.0)]))[0], "b_end": w(tip_local(1.0, q, [(ARM_S, 0.0)]))[0],
            "lines": np.vstack([w(arm_line_local(-1.0)), w(arm_line_local(1.0, q))])}


HINGE_LOCAL = np.array([(HINGE_R * math.cos(a), HINGE_R * math.sin(a), z) for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)
                        for z in (-TIP_R, TIP_R)] + [(0.0, 0.0275, 0.0)])  # the hinge block's rim and the pull ring's end


def tong_outline(tongs) -> np.ndarray:
    """Points over the whole tongs (both arms' centre line and two edges every 6 mm, both tips' faces, the hinge block's rim and the pull ring),
    station frame, z above the counter."""
    q = tongs.joint("jaw")
    t = tong_points(tongs)
    arm = [(s, z) for s in np.linspace(0.0, ARM_S + TIP_R, 40) for z in (-ARM_W / 2, 0.0, ARM_W / 2)]
    edges = np.vstack([tip_local(-1.0, 0.0, arm), tip_local(1.0, q, arm)])
    return np.vstack([tongs.to_world(edges) - [0.0, 0.0, TOP], t["a"], t["b"], tongs.to_world(HINGE_LOCAL) - [0.0, 0.0, TOP]])


def tip_ref(tongs) -> np.ndarray:
    """The midpoint of the two tips' round-end centres, station frame with z from the floor: what the demo steers."""
    t = tong_points(tongs)
    return (t["a_end"] + t["b_end"]) / 2 + [0.0, 0.0, TOP]


def moment(c: dict) -> float:
    """The scoring text on one moment's facts: three tiers, each read from the state now; only the dropped frank latches."""
    if c["failed"]:
        return 0.0
    if c["in_pan"] and c["tongs_off"] and c["carried"] and c["still"]:  # the frank carried there in the tongs, lying still, nothing on it
        return 1.0
    if c["in_pan"] and c["carried"]:
        return 2 / 3
    if c["closed_on"] and c["lifted"]:
        return 2 / 3
    if c["tongs_up"]:
        return 1 / 3
    return 0.0


class Trial:
    def __init__(self) -> None:
        self.env = None
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.dropped = gk.Latch("tagged frank dropped")
        self.carried = False  # the tongs closed on the tagged frank, lifted it and carried it to the pan, the tips never off it for over 0.5 s
        self.off_n = 0  # steps since the tips were last closed on the tagged frank
        self.start: dict = {}
        self.tongs_z0 = 0.0
        self.hold_n = 0  # steps in a row the last tier has held
        self.score = 0.0
        self.report: dict = {}
        self.why = ""
        # the drop read, after every physics step: the tagged frank's centre
        self.phys_n = 0
        self.prev = None  # (z, downward speed) at the last physics step
        self.free_n = 0
        self.free_z = 0.0
        self.frank_fall = 0.0  # the worst fall so far, from contact loss to landing (m)
        if env is not None and hasattr(env, "unwrapped"):
            self.env = env
            self._hook(env)

    def _hook(self, env) -> None:
        """Run `motion` after every physics step: the harness steps physics 12 times per control step (240 Hz) and updates the scene's data
        after each."""
        scene = env.unwrapped.scene
        if getattr(scene, "_hotdog_hooked", False):
            return
        update = scene.update

        def updated(*a, **k):
            out = update(*a, **k)
            if TRIAL.env is not None:
                TRIAL.motion(TRIAL.env)
            return out

        scene.update = updated
        scene._hotdog_hooked = True

    def motion(self, env) -> None:
        """One physics step's read of the tagged frank's fall, from its centre. It falls free once it speeds up downward at 0.6 g or more for
        3 physics steps in a row (nothing holds it up); its fall runs from that contact loss to its landing, when it loses over half its
        downward speed in one physics step: the height it lost in free flight plus the drop its downward speed at contact loss is worth
        (v^2 / 2g). A fall of 3 cm or more anywhere but over the pan voids the run."""
        self.phys_n += 1
        dt = float(getattr(env.unwrapped, "physics_dt", 1.0 / 240.0))
        tag = m.state(env, TAGGED)
        z = float(tag.pos[2]) - TOP
        if self.prev is not None:
            z1, vd1 = self.prev
            vd = (z1 - z) / dt  # downward speed over this physics step
            if vd1 is not None:
                free = vd - vd1 >= FREE_A * G * dt
                landed = vd1 > 0.05 and vd1 - vd > LAND_LOSS * vd1
                if self.free_n == 0 and free:  # contact loss: where it was, and how fast it was already going down
                    self.free_z = z1 + max(0.0, vd1) ** 2 / (2 * G)
                if self.free_n >= FREE_N and (landed or free):
                    h = self.free_z - z
                    pan_loc = m.state(env, "pan").to_local(tag.pos[None, :])[0]
                    over_pan = abs(pan_loc[0]) <= PAN_HX and abs(pan_loc[1]) <= PAN_HY
                    if not over_pan:
                        self.frank_fall = max(self.frank_fall, h)
                    self._fail(h >= FALL and not over_pan, f"({1000 * h:.1f} mm, at {np.round(1000 * (tag.pos - [0, 0, TOP]), 0).tolist()})")
                self.free_n = self.free_n + 1 if free else 0
            self.prev = (z, vd)
        else:
            self.prev = (z, None)

    def _fail(self, cond: bool, detail: str = "") -> None:
        if cond and not self.dropped:
            self.why = self.why or f"tagged frank dropped at step {self.steps} {detail}".strip()
            print(f"[hotdog] {self.why}", flush=True)
        self.dropped.trip(cond)

    def step(self, env) -> None:
        self.steps += 1
        if env is not None and hasattr(env, "unwrapped"):
            asset = env.unwrapped.scene["rollers"]
            ids = [asset.find_joints(f"r{k}_joint")[0][0] for k in range(N_ROLLERS)]
            import torch

            asset.set_joint_velocity_target(torch.full((1, N_ROLLERS), OMEGA, device=asset.device), joint_ids=ids)
        tag, tongs, pan = m.state(env, TAGGED), m.state(env, "tongs"), m.state(env, "pan")
        tagp = tag.pos - [0.0, 0.0, TOP]
        if self.steps <= SETTLE_STEPS:  # where the tagged frank and the tongs lie once settled
            self.start = tagp.copy()
            self.tongs_z0 = float(tongs.pos[2])
        tp = tong_points(tongs)
        tips = m.fingertips(env) - [0.0, 0.0, TOP]
        outline = tong_outline(tongs)

        # 1. the tongs picked up: a finger on them, 5 cm above where they lay
        held_t = float(np.linalg.norm(tips[:, None, :] - outline[None, :, :], axis=2).min()) <= 0.015
        tongs_up = bool(held_t and float(tongs.pos[2]) - self.tongs_z0 >= TONGS_UP)

        # 2. closed on: both tips in contact (within 1 mm) with the frank, on opposite sides of its axis and facing each other along it
        gap_a, gap_b = capsule_gap(tp["a"] + [0.0, 0.0, TOP], tag), capsule_gap(tp["b"] + [0.0, 0.0, TOP], tag)
        la, lb = tag.to_local(tp["a"][np.argmin(gap_a)][None, :] + [0.0, 0.0, TOP])[0], tag.to_local(tp["b"][np.argmin(gap_b)][None, :] + [0.0, 0.0, TOP])[0]
        pa, pb = la[[0, 2]], lb[[0, 2]]  # across the frank's axis (its own y)
        finger_on = bool(float(capsule_gap(tips + [0.0, 0.0, TOP], tag).min()) <= FINGER_NEAR)
        closed_on = bool(gap_a.min() <= TOUCH and gap_b.min() <= TOUCH and squeezed(pa, pb) and abs(la[1] - lb[1]) <= 0.02
                         and abs(la[1]) <= FRANK_L / 2 and abs(lb[1]) <= FRANK_L / 2 and not finger_on)
        lifted = bool(tagp[2] - self.start[2] >= LIFT)

        # 3. in the pan, lying still, no part of the tongs within 5 mm of it
        pan_loc = pan.to_local(tag.pos[None, :])[0]
        over_pan = abs(pan_loc[0]) <= PAN_HX and abs(pan_loc[1]) <= PAN_HY
        in_pan = bool(over_pan and 0.0 <= pan_loc[2] <= PAN_H)
        gap_all = float(capsule_gap(outline + [0.0, 0.0, TOP], tag).min())
        still = bool(max(float(np.linalg.norm(tag.lin_vel)), float(np.linalg.norm(pan.lin_vel))) <= STILL_V)

        # carried: closed on it and lifted it, then held it in the closed tips until it was over the pan (gaps of up to 0.5 s allowed: a
        # regrip). Set down off the pan and left, it is no longer being carried, so a frank set back and then pushed or scooped in scores
        # nothing past the tongs. A carry starts only from a lift off the grill, not over the pan.
        self.off_n = 0 if closed_on else self.off_n + 1
        if finger_on:  # a gripper finger on the frank at any time ends the carry: the hand, not the tongs, is holding it
            self.carried = False
        elif closed_on and lifted and not over_pan:
            self.carried = True
        elif self.carried and not over_pan and self.off_n > CHAIN_S:
            self.carried = False
        c = {"failed": bool(self.dropped), "in_pan": in_pan, "tongs_off": gap_all > NEAR, "carried": self.carried, "still": still,
             "closed_on": closed_on, "lifted": lifted, "tongs_up": tongs_up}
        mo = moment(c)
        self.hold_n = self.hold_n + 1 if mo == 1.0 else 0  # held 1 s, read from this moment backward: nothing latches
        self.score = 1.0 if self.hold_n >= HOLD_STEPS else min(mo, 2 / 3)
        self.report = {"step": self.steps, "score": self.score, "moment": mo, "tagged_mm": np.round(1000 * tagp, 1).tolist(),
                       "gap_all_mm": round(1000 * gap_all, 1), "gap_a_mm": round(1000 * float(gap_a.min()), 1), "gap_b_mm": round(1000 * float(gap_b.min()), 1),
                       "jaw_deg": round(tongs.deg("jaw"), 1), "tongs_up": tongs_up, "closed_on": closed_on, "lifted": lifted, "carried": self.carried,
                       "in_pan": in_pan, "still": still, "hold_n": self.hold_n, "dropped": bool(self.dropped), "why": self.why,
                       "phys_steps": self.phys_n, "frank_fall_mm": round(1000 * self.frank_fall, 1)}
        if self.steps % 20 == 0:
            d = os.environ.get("HOTDOG_SCORE_DIR", "hotdog_scores")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, f"{os.environ.get('TASK_DEMO', 'run')}.json"), "w") as f:
                json.dump(self.report, f, indent=0, default=str)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


def size_check() -> list[tuple[str, float, float, str]]:
    """The scene's key sizes against the published ones (cm): name, in the scene, published, source."""
    return [("grill length (y)", 100 * (GRILL_Y[1] - GRILL_Y[0]), 58.1, "Avantco RG1830NS sheet, 22 7/8 in"),
            ("grill depth (x)", 100 * (GRILL_X[1] - GRILL_X[0]), 46.4, "sheet, 18 1/4 in"),
            ("grill height", 100 * GRILL_H, 22.2, "sheet, 8 3/4 in"),
            ("rollers", float(N_ROLLERS), 11.0, "sheet"),
            ("frank length", 100 * FRANK_L, 15.0, "standard 8-to-the-pound frank (estimate)"),
            ("frank diameter", 100 * 2 * FRANK_R, 2.2, "standard frank (estimate)"),
            ("tongs length", 100 * (HINGE_R + ARM_S + TIP_R), 22.9, "9 in utility tongs"),
            ("tongs tip opening", 100 * 2 * TIP_C[0], 7.0, "estimate"),
            ("pan length (x)", 100 * 2 * PAN_HX, 32.4, "1/3-size pan, 12 3/4 in"),
            ("pan width (y)", 100 * 2 * PAN_HY, 17.5, "6 7/8 in"),
            ("pan depth", 100 * PAN_H, 10.2, "4 in")]


# -- the demo ---------------------------------------------------------------------------------------------------------------------

GRIP_LOCAL = np.array([0.0, GRIP_Y, 0.006])  # the TCP on the tongs, in their frame: between the arms, 11 cm from the hinge
REF_LOCAL = np.array([0.0, TIP_C[1], 0.0])  # the tips' round-end centres' midpoint, on the tongs' axis
PICK = (25.0, 0.0, False)  # the grip on the tongs where they lie: demokit tilted(theta, phi, flip), the fingers closing across the open arms
PINCH_DOWN, PINCH_YAW, PAN_YAW = 50.0, 0.0, -30.0  # the tongs at the pinch: tips 50 degrees down, reaching along the frank's axis toward -y, closing across it; turned 30 degrees for the pan


def _rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def _half_outer(tip_half: float) -> float:
    """Half the arms' outside width at the grip when the tips' centre lines are `tip_half` off the axis."""
    f = (-GRIP_Y + ROOT[1]) / (-(TIP_C[1] - ROOT[1]))
    return ROOT[0] + (tip_half - ROOT[0]) * f + ARM_T / 2


def demo_script(ctx):
    """The right hand comes down over the tongs where they lie, its fingers tilted 25 degrees forward and either side of the arms 11 cm from the
    hinge, and squeezes them shut. It lifts them and turns its wrist so the tongs reach down 50 degrees along the tagged frank's axis, eases the
    squeeze so the tips spring 3.4 cm apart, and lowers them beside the frank until their lower edges are 1.5 mm below its middle, steered by the
    tongs' own pose. It closes them on it with about 25 N a finger against the spring, lifts it 8 cm straight up, carries it over the pan, turning
    the tongs 30 degrees, lowers it and eases the squeeze, so the tips spring apart and the frank drops in. It lifts the tongs out, squeezes them
    shut, lays them back on the towel and lets go, so they spring open where they lie.

    Check modes (TASK_DEMO): none; gripper_direct (the bare gripper takes the frank); high_pinch (the tips closed 10 mm higher, on its upper
    half); push_off (closed tongs push it off the grill's front); wrong_frank (the frank beside it in groove 2 taken instead); hover (the open
    tips held round it, never closed); push (shut tips push it along its groove toward the pan); scoop (shut tips sweep it up and over the pan); undo (lifted, then set back in its groove and let go); regrip (set back, let go for 1 s, closed on again and carried on as in full);
    scoop_pinch (carried over the pan at under 4 cm and let go there, then pinched in the pan and lifted a little); tongs_in_pan (the frank let go in the pan, the tongs held in the pan with an arm on it, tips clear);
    set_back_held (lifted 8 cm, set back in its groove, pinched to the end); top_touch (the tips narrower than the frank, resting on its top); no_release (held over the pan to the end); cradle (tips 1.6 cm apart swept under the frank and tipped into the pan); two_hand_shadow (the left hand lifts it bare and carries it as far as it reaches while the right tips only touch it); release_touching (set on the pan floor, the tips eased to a touch); drag_pinched (lifted, set back pinched, then carried 4.5 cm up over the housing)."""
    import demokit as d

    r = ctx.right
    mode = ctx.mode or "full"
    live = ctx.obs is not None
    pick = d.tilted(*PICK)
    lie = _rot("z", TONGS_YAW)  # the tongs' rotation where they lie
    in_hand = _rot("z", PINCH_YAW) @ _rot("x", PINCH_DOWN)  # and at the pinch
    pinch = d._as_grip(in_hand @ lie.T @ pick.rot)
    in_hand2 = _rot("z", PAN_YAW) @ _rot("x", PINCH_DOWN)  # carried over the pan
    pinch2 = d._as_grip(in_hand2 @ lie.T @ pick.rot)

    tongs0 = np.array([TONGS_AT[0], TONGS_AT[1], TOP + TOWEL[2] + TIP_R])
    f_open = -0.040
    f_hold = -(_half_outer(0.017) - 0.0006)  # the tips held 3.4 cm apart, against the tongs' spring: 5 mm clear of the frank each side
    f_release = -(_half_outer(0.021) - 0.0006)  # 4.2 cm apart: the frank drops out
    f_carry = -max(0.0, _half_outer(ARM_T / 2) - 0.004)  # squeezed shut, the tips together: about 20 N a finger
    f_close = -(_half_outer(FRANK_R + ARM_T / 2) - 30.0 / 5000.0)  # closed on a frank: about 25 N a finger against the spring

    def tcp_for(ref, rot_turn=in_hand):
        """The TCP that puts the tips' midpoint at `ref`, the tongs held as picked and turned by `rot_turn`."""
        return np.asarray(ref, float) - rot_turn @ (REF_LOCAL - GRIP_LOCAL)

    def ref_now():
        return tip_ref(ctx.state("tongs"))

    def frank_at(name):
        p = ctx.state(name).pos if live else np.array([*FRANKS[name], TOP + FRANK_Z])
        return np.array(p, float)

    for _ in range(5):
        yield
    grip_pt = tongs0 + lie @ GRIP_LOCAL
    if live:  # where the tongs lie now
        t0 = ctx.state("tongs")
        grip_pt = t0.pos + t0.rotation() @ GRIP_LOCAL
    if mode == "none":
        yield from r.wait(4.0)
        return
    if mode == "gripper_direct":
        f = frank_at(TAGGED)
        yield from r.move(f + [0.0, 0.0, 0.10], grip=d.down(90), finger=-0.03, t=3.0, label="right: over the frank, bare")
        yield from r.move(f + [0.0, 0.0, 0.004], t=1.5, label="right: fingers round it")
        yield from r.close(2 * FRANK_R, 10.0, t=0.6, label="right: grip it")
        yield from r.move(f + [0.0, 0.0, 0.10], t=1.5, label="right: lift")
        yield from r.move(np.array([PAN_C[0], PAN_C[1], TOP + 0.16]), t=2.0, label="right: over the pan")
        yield from r.open(t=0.5, label="right: drop it")
        yield from r.wait(2.0)
        return

    # 1. take the tongs: straight down over them, the fingers either side of the open arms, and squeeze
    yield from r.move(grip_pt + [0.0, 0.0, 0.10], grip=pick, finger=f_open, t=3.0, label="right: over the tongs")
    yield from r.move(grip_pt, t=1.4, label="right: fingers either side of the arms")
    yield from r.move(r.at(), finger=f_carry, t=1.0, label="right: squeeze the tongs shut: a firm hold to carry them")
    yield from r.move(grip_pt + [0.0, 0.0, 0.12], t=1.4, label="right: lift the tongs")
    if live:
        print(f"[hotdog] tongs lifted: frame {np.round(ctx.state('tongs').pos - [0, 0, TOP], 3).tolist()}, tcp {np.round(r.tcp() - [0, 0, TOP], 3).tolist()}, jaw {ctx.state('tongs').deg('jaw'):.1f}", flush=True)

    target = frank_at("frank_2b" if mode == "wrong_frank" else TAGGED)
    low = target + [0.0, 0.0, -0.0015 + TIP_R]  # the tips' lower edges 1.5 mm below the frank's middle
    if mode == "high_pinch":
        low = low + [0.0, 0.0, 0.010]
    above = low + [0.0, 0.0, 0.06]
    # 2. tilt and bring the tips over the frank, then steer them by the tongs' own pose
    yield from r.move(tcp_for(above), grip=pinch, t=2.6, label="right: tilt, tips over the frank")
    if mode == "push_off":  # the tips still squeezed shut: lowered behind the frank and pushed toward the front, off the grill
        yield from r.move(tcp_for(above + [0.035, 0.0, 0.0]), t=1.0, label="right: shut tips behind the frank")
        yield from r.move(r.at() + [0.0, 0.0, -0.055], t=1.2, label="right: down behind it")
        yield from r.move(r.at() + [-0.16, 0.0, 0.0], t=2.5, label="right: push it off the front")
        yield from r.wait(2.0)
        return
    if mode == "push":  # the shut tips push the frank along its groove toward the pan, without closing on it
        f = frank_at(TAGGED)
        yield from r.move(tcp_for(np.array([f[0], f[1] + 0.09, f[2] + 0.05])), t=1.2, label="right: shut tips past its far end")
        yield from r.move(r.at() + [0.0, 0.0, -0.045], t=1.0, label="right: down to its end")
        yield from r.move(r.at() + [0.0, -0.20, 0.0], t=3.0, label="right: push it along the groove toward the pan")
        yield from r.wait(2.0)
        return
    if mode == "cradle":  # tips 1.6 cm apart, narrower than the frank, slid under its side and swept up and over the pan: it rests on their edges
        yield from r.move(r.at(), finger=-(_half_outer(0.0105) - 0.0006), t=0.6, label="right: the tips narrower than the frank")
        yield from r.move(tcp_for(low + [0.020, 0.0, 0.03]), t=1.2, label="right: tips behind the frank")
        yield from r.move(r.at() + [0.0, 0.0, -0.032], t=1.0, label="right: down behind it")
        yield from r.move(r.at() + [-0.03, 0.0, 0.05], t=0.8, label="right: scoop it up on the tips")
        yield from r.move(tcp_for(np.array([PAN_C[0], PAN_C[1], low[2] + 0.06])), t=2.0, label="right: over the pan")
        yield from r.move(r.at() + [0.0, 0.0, -0.12], t=1.5, label="right: down into the pan")
        yield from r.move(r.at() + [0.0, 0.0, 0.0], finger=-(_half_outer(0.021) - 0.0006), t=0.6, label="right: tip it off")
        yield from r.wait(3.0)
        print(f"[hotdog] cradle: {TRIAL.report}", flush=True)
        return
    if mode == "scoop":  # the shut tips slid under the frank's side and swept up and over toward the pan, never closed on it
        yield from r.move(tcp_for(low + [0.020, 0.0, 0.03]), t=1.2, label="right: shut tips behind the frank")
        yield from r.move(r.at() + [0.0, 0.0, -0.032], t=1.0, label="right: down behind it")
        yield from r.move(r.at() + [-0.03, 0.0, 0.05], t=0.8, label="right: scoop it up")
        yield from r.move(tcp_for(np.array([PAN_C[0], PAN_C[1], low[2] + 0.06])), t=2.0, label="right: over the pan")
        yield from r.move(r.at() + [0.0, 0.0, -0.12], t=1.5, label="right: down into the pan")
        yield from r.wait(2.0)
        return
    yield from r.move(r.at(), finger=f_hold, t=0.8, label="right: ease the squeeze: the tips spring 3.4 cm apart")
    if live:
        yield from r.servo(above, read=ref_now, tol=0.002, timeout=1.5, label="right: tips over the frank's middle")
    yield from r.move(r.at() + (low - above), t=1.6, label="right: lower the open tips beside it")
    if live:
        yield from r.servo(low, read=ref_now, tol=0.0015, timeout=1.5, label="right: tips at the frank's middle")
    if mode == "top_touch":  # the tips held 1.85 cm apart come down on the frank's top, resting on it, never beside it
        yield from r.move(r.at(), finger=-(_half_outer(0.0105) - 0.0006), t=0.6, label="right: the tips narrower than the frank")
        yield from r.move(r.at() + [0.0, 0.0, -0.02], t=1.5, label="right: let them settle on its top")
        yield from r.wait(3.0)
        print(f"[hotdog] top_touch: {TRIAL.report}", flush=True)
        return
    if mode == "hover":
        yield from r.wait(4.0)
        return
    if mode == "two_hand_shadow":  # the right hand holds the tips round the frank, touching and not squeezing; the left hand takes the frank bare and carries it to the pan
        yield from r.move(r.at(), finger=-(_half_outer(FRANK_R + ARM_T / 2) - 0.0006), t=0.8, label="right: the tips just touching it, no squeeze")
        l = ctx.left
        f = frank_at(TAGGED) + [0.0, -0.05, 0.0]
        yield from l.move(f + [0.0, 0.0, 0.10], grip=d.down(90), finger=-0.03, t=3.0, label="left: over the frank's end, bare")
        yield from l.move(f + [0.0, 0.0, 0.004], t=1.5, label="left: fingers round it")
        yield from l.close(2 * FRANK_R, 10.0, t=0.6, label="left: grip it")
        yield from l.move(f + [0.0, 0.0, 0.10], t=1.5, label="left: lift")
        yield from l.move(f + [-0.05, -0.10, 0.12], t=2.0, label="left: carry it toward the pan as far as the arm reaches")
        yield from r.wait(3.0)
        print(f"[hotdog] two_hand_shadow: {TRIAL.report}", flush=True)
        return
    # 3. close on it, then lift it straight up out of its groove
    yield from r.move(r.at(), finger=f_close, t=0.8, label="right: close the tips on the frank")
    yield from r.wait(0.3)
    print(f"[hotdog] closed: {TRIAL.report}", flush=True)
    up = 0.037 if mode == "scoop_pinch" else 0.08  # scoop_pinch: lifted under 4 cm, so no carry starts
    yield from r.move(r.at() + [0.0, 0.0, up], t=1.6, label="right: lift it straight up")
    if live:
        yield from r.servo(low + [0.0, 0.0, up], read=ref_now, tol=0.003, timeout=1.0, label="right: up")
    if mode == "set_back_held":  # lifted 8 cm, set back down in its groove with the tips still closed on it, held so to the end
        yield from r.move(r.at() + [0.0, 0.0, -0.08], t=1.8, label="right: set it back in its groove, still pinched")
        yield from r.wait(4.0)
        print(f"[hotdog] set_back_held: {TRIAL.report}", flush=True)
        return
    if mode in ("undo", "regrip"):
        yield from r.move(r.at() + [0.0, 0.0, -0.08], t=1.8, label="right: set it back in its groove")
        yield from r.move(r.at(), finger=f_hold, t=0.6, label="right: let it go")
        if mode == "undo":
            yield from r.move(r.at() + [0.0, 0.0, 0.08], t=1.0, label="right: tips up")
            yield from r.wait(2.0)
            return
        # regrip: the tips off it for 1 s, then closed on it again where it lies and the task carried on as in full
        yield from r.move(r.at() + [0.0, 0.0, 0.03], t=0.8, label="right: tips up off it")
        yield from r.wait(1.0)
        low = frank_at(TAGGED) + [0.0, 0.0, -0.0015 + TIP_R]
        if live:
            yield from r.servo(low + [0.0, 0.0, 0.03], read=ref_now, tol=0.002, timeout=1.5, label="right: tips over it again")
        yield from r.move(r.at() + [0.0, 0.0, -0.03], t=1.0, label="right: lower the open tips beside it again")
        if live:
            yield from r.servo(low, read=ref_now, tol=0.0015, timeout=1.5, label="right: tips at its middle")
        yield from r.move(r.at(), finger=f_close, t=0.8, label="right: close on it again")
        yield from r.wait(0.3)
        yield from r.move(r.at() + [0.0, 0.0, 0.08], t=1.6, label="right: lift it again")
        if live:
            yield from r.servo(low + [0.0, 0.0, 0.08], read=ref_now, tol=0.003, timeout=1.0, label="right: up 8 cm")
    if mode == "drag_pinched":  # lifted 8 cm, set back down on the grill (still pinched), then lifted 4.5 cm only and dragged over the housing into the pan
        yield from r.move(r.at() + [0.0, 0.0, -0.08], t=1.8, label="right: set it back in its groove, still pinched")
        yield from r.move(r.at() + [0.0, 0.0, 0.045], t=1.0, label="right: up 4.5 cm")
        up = 0.045
    # 4. carry it over the pan, lower it into the pan and let it go
    over = np.array([PAN_C[0], PAN_C[1], low[2] + up])
    drop = np.array([PAN_C[0], PAN_C[1], TOP + 0.060])
    yield from r.move(tcp_for(over, in_hand2) + r.trim, grip=pinch2, t=2.6, label="right: carry it over the pan, turning the tongs")
    yield from r.move(tcp_for(drop, in_hand2) + r.trim, t=2.0, label="right: lower it into the pan")
    if live:
        yield from r.servo(drop, read=ref_now, tol=0.004, timeout=1.0, label="right: tips over the pan's middle")
    if mode == "no_release":
        yield from r.wait(3.0)
        return
    if mode == "release_touching":  # lowered to the pan's floor and eased open only to a touch: the frank in the pan with a tip on it
        floor = np.array([PAN_C[0], PAN_C[1], TOP + 0.0005 + 0.003 + FRANK_R - 0.0015 + TIP_R + 0.002])
        yield from r.move(tcp_for(floor, in_hand2) + r.trim, t=1.5, label="right: down onto the pan's floor")
        yield from r.move(r.at(), finger=-(_half_outer(FRANK_R + ARM_T / 2) - 0.0006), t=0.6, label="right: ease to a touch, tips still on it")
        yield from r.wait(3.0)
        print(f"[hotdog] release_touching: {TRIAL.report}", flush=True)
        return
    yield from r.move(r.at(), finger=f_release, t=0.6, label="right: ease the squeeze: the tips spring apart and the frank drops")
    yield from r.wait(0.6)
    if mode == "scoop_pinch":  # carried over the pan at under 4 cm and let go (no carry), then the tips opened round it in the pan, closed on it, lifted a little and held
        yield from r.move(r.at() + [0.0, 0.0, 0.10], t=0.8, label="right: tips up")
        yield from r.move(r.at(), finger=f_hold, t=0.8, label="right: open the tips")
        f = frank_at(TAGGED)
        lowp = f + [0.0, 0.0, -0.0015 + TIP_R]
        yield from r.move(tcp_for(lowp + [0.0, 0.0, 0.05], in_hand2) + r.trim, grip=pinch2, t=1.5, label="right: tips over the frank in the pan")
        if live:
            yield from r.servo(lowp + [0.0, 0.0, 0.05], read=ref_now, tol=0.003, timeout=1.5, label="right: over it")
        yield from r.move(r.at() + [0.0, 0.0, -0.05], t=1.2, label="right: down beside it")
        yield from r.move(r.at(), finger=f_close, t=0.8, label="right: close on it in the pan")
        yield from r.wait(0.3)
        yield from r.move(r.at() + [0.0, 0.0, 0.03], t=1.0, label="right: lift it a little")
        print(f"[hotdog] scoop_pinch: {TRIAL.report}", flush=True)
        yield from r.wait(3.0)
        return
    # 5. lift the tongs out, squeeze them shut, and lay them back on the towel, where they spring open
    if mode == "tongs_in_pan":  # the frank let go, the tongs shifted back along their axis, lowered so an arm lies on it, the tips clear, held there
        yield from r.move(r.at() + [0.0, 0.0, 0.03], t=0.8, label="right: tips up")
        yield from r.move(r.at() + in_hand2 @ np.array([0.0, 0.0, 0.0]) + [0.04, 0.0, 0.0], t=1.0, label="right: tongs back so an arm is over the frank")
        for _ in range(16):  # lower in 5 mm steps until some part of the tongs touches the frank
            if TRIAL.report.get("gap_all_mm", 99.0) < 4.0:
                break
            yield from r.move(r.at() + [0.0, 0.0, -0.005], t=0.4, label="right: lower until they rest on it")
        yield from r.wait(1.0)
        print(f"[hotdog] tongs_in_pan: {TRIAL.report}", flush=True)
        yield from r.wait(2.0)
        return
    yield from r.move(r.at() + [0.0, 0.0, 0.14], t=1.4, label="right: tongs up out of the pan")
    yield from r.move(r.at(), finger=f_carry, t=0.6, label="right: squeeze them shut")
    yield from r.move(grip_pt + [0.0, 0.0, 0.10], grip=pick, t=2.6, label="right: back over the towel")
    yield from r.move(grip_pt + [0.0, 0.0, 0.008], t=1.4, label="right: down to the towel")
    yield from r.move(r.at(), finger=f_open, t=0.8, label="right: let go: the tongs spring open on the towel")
    yield from r.move(grip_pt + [0.0, 0.0, 0.12], t=1.2, label="right: hand up")
    yield from r.wait(2.0)
    print(f"[hotdog] end: {TRIAL.report}", flush=True)
