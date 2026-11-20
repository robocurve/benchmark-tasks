"""Fit a 13-inch plastic wheel cover onto a car's steel wheel, its valve notch over the valve stem, every retaining clip snapped in.

The wheel is mounted as it is on a car: a 13 x 4.5J steel wheel with a 155/80 R13 tyre, upright on its hub with the axle 27.2 cm above
the ground (the tyre's loaded radius), its outboard face towards the robot, under a wheel arch in the car's body side. The ground is the
surface the arms stand on. The rubber valve stem is at the top of the rim. The wheel cover (hubcap) lies face up on the ground in front of
the wheel, where it was put down when it came off, with its valve notch towards the robot's right.

Model (declared in code):
- Wheel ("wheel", fixed mechanism): the rim, drawn as a revolved 3 mm steel section, collides through rings of convex segments: the
  flange (where the cover's lip seats), the bead seat's inner face (the lip the clips ride over), the groove behind it (where the clips
  catch), the well's shoulder, and the centre disc. The tyre is drawn round it and collides through convex segments of its section; its
  footprint is flattened where it stands on the ground. Behind the disc: the brake drum and four wheel nuts. Its one link is the valve
  stem: a rubber TR414-class snap-in stem, on a hinge at its base about the tangent to the rim, sprung for the rubber's bending stiffness
  (0.5 N m/rad).
- Car ("car", fixed): the body side round the wheel arch (painted steel sheet with a rolled arch lip, 2.5 cm inboard of the tyre's
  sidewall), the arch's plastic liner over the tyre, and the sill below the body.
- Cover ("cover", free mechanism, 0.45 kg): a dished ABS cover 36.0 cm across with a valve notch 3.35 cm wide and eight through slots
  between eight spokes, colliding through convex pieces (the notch and the slots are real gaps). Its eight retaining clips are links:
  each a sprung slider (the steel retaining ring's spring, 1,500 N/m) with a ramped head. Pushed onto the rim, the head's ramp meets the
  bead seat's lip and cams the clip in; 2.2 cm further the head clears the lip and springs out into the groove behind it, where its flat
  face catches behind the lip.
- Scoring (code, each control step), all measured in the wheel's frame: a clip is seated when its catching face is behind the lip's back
  edge, at least 1.5 mm out past the lip's inner face (hooked behind it), and the cover's lip beside it is within 1.0 mm of the flange.
  The stem is through the notch when its cap is within 1.1 cm of the notch's centre line, inside the notch's length and within the
  cover's depth. A stem bent more than 15 degrees at any time scores 0 for the trial (cleared only on reset).
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the wheel cover on the wheel."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 40  # 2 s at 20 Hz
TOP = TABLE_TOP[SCENE]  # the ground the car and the arms stand on

# -- where the wheel is ------------------------------------------------------------------------------------------------------------------
# Wheel frame: origin on the wheel's axis at the inboard flange, z along the axis outboard (towards the robot), y towards the valve stem.
# World = W0 + R_W @ local: local z -> world -x (the face looks at the robot), local y -> world +z (the stem at the top), local x -> -y.
HUB_H = 0.272  # the axle above the ground: a 155/80 R13 tyre's static loaded radius
Z_F = 0.126  # the outboard flange's face, from the inboard flange (overall rim width with flanges 12.6 cm)
X_FLANGE = 0.420  # where the outboard flange's face is (world x)
W0 = np.array([X_FLANGE + Z_F, 0.0, TOP + HUB_H])
R_W = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def quat_xyzw(r) -> tuple:
    r = np.asarray(r, float)
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    if w > 1e-6:
        return (float((r[2, 1] - r[1, 2]) / (4 * w)), float((r[0, 2] - r[2, 0]) / (4 * w)), float((r[1, 0] - r[0, 1]) / (4 * w)), float(w))
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + r[i, i] - r[j, j] - r[k, k])) * 2
    q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (r[j, i] + r[i, j]) / s, (r[k, i] + r[i, k]) / s
    return tuple(float(v) for v in q)


def to_wheel(p) -> np.ndarray:
    """World points in the wheel's frame."""
    return (np.atleast_2d(p) - W0) @ R_W


def from_wheel(p) -> np.ndarray:
    return np.atleast_2d(p) @ R_W.T + W0


# -- the rim -----------------------------------------------------------------------------------------------------------------------------
R_LIP = 0.1625  # the bead seat's inner face (the lip the clips ride over): 13 in bead seat diameter 330 mm, less 2.5 mm
Z_LIP_BOT = Z_F - 0.022  # the lip's back edge: behind it, the groove
R_GROOVE = R_LIP + 0.004  # the groove's floor (the safety hump's inside)
R_FLANGE = R_GROOVE + 0.0165
SEG = 40  # collider segments round the rim and the tyre

# -- the valve stem: at the rim's top (wheel frame +y), rubber, hinged at its base about the rim's tangent (wheel frame x)
STEM_R, STEM_Z0, STEM_Z1 = 0.142, 0.080, 0.138
STEM_BENT_DEG = 15.0

# -- the cover (cover frame: origin on its axis in the plane of the lip's underside; z out of its face; the notch on +x)
R_COVER = 0.180
NOTCH_HALF_DEG = math.degrees(0.01675 / STEM_R)  # 3.35 cm wide at the stem's radius
R_NOTCH_IN = 0.125
SLOT_R0, SLOT_R1, SPOKE_W = 0.072, 0.112, 0.024  # eight through slots 4 cm long radially, between eight parallel-sided spokes 2.4 cm wide
SPOKE_ANGLES = [22.5 + 45.0 * k for k in range(8)]  # the spokes' centre lines (cover frame), beside the clips
CLIP_ANGLES = [22.5 + 45.0 * k for k in range(8)]
CLIP_K = 1500.0  # N/m per clip
CLIP_TOP = -0.0225  # the head's catching face, 0.5 mm behind the lip's back edge when seated (the cover's play when hooked)
CLIP_OUT = R_LIP + 0.003  # the head's outer face at rest: 3 mm out past the lip
CLIP_DEPTH = 0.037  # the clips' heads reach this far behind the lip plane
COVER_MASS = 0.45
START_YAW = -67.5  # the notch's direction on the ground (world, from +x): towards the robot's right and away from it
START_XY = (0.200, 0.0)
NOTCH_TOL = 0.011  # m: the stem's cap from the notch's centre line
ENGAGE_GAP = 0.0010  # m: lip to flange beside a clip
HOOK = 0.0010  # m: a seated clip's catching face at least this far out past the lip's inner face (3 mm at rest)
BEHIND_TOL = 0.0005  # m: and behind the lip's back edge, bearing on it (contact allowance)
CLEAR = 0.03  # m: every fingertip point this far from the cover

LOOKS = {
    "steel": m.Look(color=m.rgb(0.13, 0.13, 0.14), metallic=0.6, roughness=0.45),  # black-painted steel wheel
    "steel_dark": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.6),
    "zinc": m.Look(color=m.rgb(0.55, 0.55, 0.52), metallic=0.8, roughness=0.35),
    "drum": m.Look(color=m.rgb(0.22, 0.20, 0.19), metallic=0.5, roughness=0.7),
    "tyre": m.Look(color=m.rgb(0.035, 0.035, 0.035), roughness=0.9),
    "cover": m.Look(color=m.rgb(0.72, 0.73, 0.75), metallic=0.55, roughness=0.32),  # silver-painted ABS
    "cover_dark": m.Look(color=m.rgb(0.16, 0.16, 0.17), roughness=0.5),
    "clip": m.Look(color=m.rgb(0.10, 0.10, 0.10), roughness=0.6),
    "rubber": m.Look(color=m.rgb(0.04, 0.04, 0.04), roughness=0.85),
    "brass": m.Look(color=m.rgb(0.75, 0.62, 0.30), metallic=0.9, roughness=0.3),
    "paint": m.Look(color=m.rgb(0.10, 0.20, 0.42), metallic=0.5, roughness=0.3),  # the car's paint
    "liner": m.Look(color=m.rgb(0.06, 0.06, 0.06), roughness=0.8),
}


def revolve(section, a0_deg=0.0, a1_deg=360.0, n=64) -> m.Mesh:
    """A solid of revolution about z from a closed (r, z) section, counter-clockwise in the (r, z) plane. A partial sweep gets end caps."""
    sec = np.asarray(section, float)
    k = len(sec)
    full = abs(a1_deg - a0_deg) >= 359.999
    th = np.radians(np.linspace(a0_deg, a1_deg, n, endpoint=not full))
    rows = len(th)
    v = np.vstack([np.c_[sec[:, 0] * math.cos(t), sec[:, 0] * math.sin(t), sec[:, 1]] for t in th])
    f = []
    for i in range(rows if full else rows - 1):
        i2 = (i + 1) % rows
        for j in range(k):
            j2 = (j + 1) % k
            a, b, c, d = i * k + j, i * k + j2, i2 * k + j2, i2 * k + j
            f += [(a, d, c), (a, c, b)]
    if not full:
        c0 = sec.mean(axis=0)
        for i, sgn in ((0, -1), (rows - 1, 1)):
            t = th[i]
            ci = len(v)
            v = np.vstack([v, [c0[0] * math.cos(t), c0[0] * math.sin(t), c0[1]]])
            for j in range(k):
                j2 = (j + 1) % k
                f.append((ci, i * k + j2, i * k + j) if sgn < 0 else (ci, i * k + j, i * k + j2))
    return m.Mesh(v, np.array(f))


def _offset(centre, t):
    """A closed section from a polyline: offset t/2 to both sides."""
    p = np.asarray(centre, float)
    d = np.gradient(p, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True)
    nrm = np.c_[d[:, 1], -d[:, 0]]
    return np.vstack([p + nrm * t / 2, (p - nrm * t / 2)[::-1]])


def hull(pts) -> m.Mesh:
    """Points as a mesh for a convex-hull collider: a fan over every point, so that all of them are referenced."""
    p = np.asarray(pts, float)
    return m.Mesh(p, np.array([(0, i, i + 1) for i in range(1, len(p) - 1)]))


def hexa(c) -> m.Mesh:
    """A closed solid from 8 corners: the four of one face (in order round it), then the four opposite them."""
    v = np.asarray(c, float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    mid = v.mean(axis=0)
    out = []
    for a, b, k in f:  # each face counter-clockwise seen from outside
        n = np.cross(v[b] - v[a], v[k] - v[a])
        out.append((a, b, k) if np.dot(n, v[a] - mid) >= 0 else (a, k, b))
    return m.Mesh(v, np.array(out))


def _wedge(name, r0, r1, z0, z1, a0, a1):
    """A ring segment collider: from r0 to r1, z0 to z1, angles a0 to a1 (degrees)."""
    pts = []
    for a in np.radians(np.linspace(a0, a1, 3)):
        for r in (r0, r1):
            for z in (z0, z1):
                pts.append((r * math.cos(a), r * math.sin(a), z))
    return m.col_hull(name, hull(pts))


# -- the wheel --------------------------------------------------------------------------------------------------------------------------

RIM_LINE = [(0.181, 0.007), (0.181, 0.0015), (0.166, 0.0015), (0.164, 0.006), (0.164, 0.022), (0.167, 0.027), (0.167, 0.032), (0.147, 0.042),
            (0.147, 0.072), (0.168, 0.086), (0.168, 0.1025), (0.164, 0.1055), (0.164, 0.1245), (0.181, 0.1245), (0.181, 0.118)]
DISC_LINE = [(0.029, 0.088), (0.065, 0.088), (0.080, 0.076), (0.110, 0.076), (0.128, 0.083), (0.143, 0.083), (0.1475, 0.068)]
PCD_R = 0.050  # 4 x 100 mm
# 155/80 R13: 15.5 cm section width, 12.4 cm section height (28.9 cm free radius), on the rim's centre line (z 0.063); counter-clockwise
TYRE_SECTION = [(0.183, 0.0), (0.200, -0.007), (0.225, -0.0145), (0.250, -0.013), (0.270, -0.005), (0.283, 0.006), (0.289, 0.021),
                (0.289, 0.105), (0.283, 0.120), (0.270, 0.131), (0.250, 0.139), (0.225, 0.1405), (0.200, 0.133), (0.183, 0.126)]
GROUND_LOCAL_Y = -HUB_H + 0.002  # the ground in the wheel's frame (2 mm clear): the tyre's footprint is flattened here


def _flatten(points):
    p = np.array(points, float)
    p[:, 1] = np.maximum(p[:, 1], GROUND_LOCAL_Y)
    return p


def _tyre():
    mesh = revolve(TYRE_SECTION, n=120)
    mesh = m.Mesh(_flatten(mesh.v), mesh.f)
    cols = []
    da = 360.0 / SEG
    for i in range(SEG):
        pts = [(r * math.cos(a), r * math.sin(a), z) for a in np.radians(np.linspace(i * da, (i + 1) * da, 3)) for r, z in TYRE_SECTION]
        cols.append(m.col_hull(f"tyre_{i}", hull(_flatten(pts))))
    return mesh, cols


def _wheel_frame():
    parts = [("rim", revolve(_offset(RIM_LINE, 0.003), n=96), "steel"), ("disc", revolve(_offset(DISC_LINE, 0.004), n=96), "steel")]
    tyre, tyre_cols = _tyre()
    parts.append(("tyre", tyre, "tyre"))
    parts.append(("drum", m.cylinder(0.105, 0.13, (0, 0, 0.005), segments=48), "drum"))
    parts.append(("hub_cap", m.cylinder(0.024, 0.016, (0, 0, 0.096), segments=24), "zinc"))
    for i in range(4):
        a = math.radians(45 + 90 * i)
        parts.append((f"nut_{i}", m.cylinder(0.0105, 0.020, (PCD_R * math.cos(a), PCD_R * math.sin(a), 0.100), segments=6), "zinc"))
    for i in range(4):  # the disc's vent holes
        a = math.radians(90 * i)
        parts.append((f"vent_{i}", m.cylinder(0.014, 0.0012, (0.095 * math.cos(a), 0.095 * math.sin(a), 0.0786), segments=20), "steel_dark"))
    cols = list(tyre_cols)
    da = 360.0 / SEG
    for i in range(SEG):
        a0, a1 = i * da, (i + 1) * da
        cols.append(_wedge(f"lip_{i}", R_LIP, R_GROOVE, Z_LIP_BOT, Z_F, a0, a1))
        cols.append(_wedge(f"flange_{i}", R_GROOVE, R_FLANGE, Z_F - 0.006, Z_F, a0, a1))
        cols.append(_wedge(f"groove_{i}", R_GROOVE, R_GROOVE + 0.0045, Z_F - 0.039, Z_LIP_BOT, a0, a1))
        cols.append(_wedge(f"shoulder_{i}", 0.147, R_GROOVE + 0.0045, Z_F - 0.051, Z_F - 0.039, a0, a1))
    cols.append(m.col_cylinder("disc_c", 0.125, 0.016, (0, 0, 0.080)))
    cols.append(m.col_cylinder("nuts_c", 0.065, 0.020, (0, 0, 0.100)))
    return parts, cols


def _stem_link():
    x, y = 0.0, STEM_R
    body = m.lathe([(0.0, STEM_Z0), (0.0072, STEM_Z0), (0.0072, STEM_Z0 + 0.004), (0.0057, STEM_Z0 + 0.006), (0.0047, STEM_Z1 - 0.019), (0.0, STEM_Z1 - 0.019)], 24)
    core = m.cylinder(0.0030, 0.007, (0, 0, STEM_Z1 - 0.0155), segments=16)
    cap = m.lathe([(0.0, STEM_Z1 - 0.012), (0.0042, STEM_Z1 - 0.012), (0.0042, STEM_Z1 - 0.001), (0.0035, STEM_Z1), (0.0, STEM_Z1)], 20)
    parts = [("stem_body", body.moved((x, y, 0)), "rubber"), ("stem_core", core.moved((x, y, 0)), "brass"), ("stem_cap", cap.moved((x, y, 0)), "rubber")]
    cols = [m.col_cylinder("stem_c", 0.0055, STEM_Z1 - STEM_Z0 - 0.006, (x, y, (STEM_Z0 + 0.006 + STEM_Z1) / 2))]
    return m.hinge("stem", "X", (x, y, STEM_Z0), -70.0, 70.0, parts, cols, mass=0.02, spring=0.5, damping=0.01, friction=0.0, effort=20.0,
                   armature=0.0005, inertia=(2.5e-5, 2.5e-5, 1e-6))


_wf, _wc = _wheel_frame()
WHEEL = m.mechanism("wheel", tuple(W0), [_stem_link()], LOOKS, frame_parts=_wf, frame_colliders=_wc, friction=(0.5, 0.4), start_quat=quat_xyzw(R_W))

# -- the car's body round the wheel arch (world-aligned; origin on the ground below the axle, in the body side's outer face) ---------------
X_BODY = X_FLANGE + 0.010  # the body side's outer face: 2.5 cm inboard of the tyre's sidewall, 1 cm inboard of the rim's flange
R_ARCH = 0.345  # the arch's edge from the axle: 5.6 cm over the tyre
Z_SILL, Z_BODY_TOP, Y_BODY = 0.170, 0.720, 0.600  # the body's lower edge (ground clearance), the top of what is drawn, half its length
SHEET = 0.004
LINER_DEPTH = 0.22


def _car():
    parts, cols = [], []
    yo, zo = 0.0, HUB_H
    for s in (1, -1):  # the body side ahead of and behind the arch
        y0, y1 = sorted((s * R_ARCH, s * Y_BODY))
        box = m.box((SHEET, y1 - y0, Z_BODY_TOP - Z_SILL), (SHEET / 2, (y0 + y1) / 2, (Z_SILL + Z_BODY_TOP) / 2))
        parts.append((f"side_{'f' if s > 0 else 'r'}", box, "paint"))
        cols.append(m.col_box(f"side_{'f' if s > 0 else 'r'}_c", (SHEET, y1 - y0, Z_BODY_TOP - Z_SILL), (SHEET / 2, (y0 + y1) / 2, (Z_SILL + Z_BODY_TOP) / 2)))
        sill = m.box((0.06, y1 - y0, 0.03), (0.03, (y0 + y1) / 2, Z_SILL + 0.015))
        parts.append((f"sill_{'f' if s > 0 else 'r'}", sill, "liner"))
    n = 24
    arc = []
    for i in range(n):  # above the arch: strips from the arch's edge to the top
        a0, a1 = math.pi * i / n, math.pi * (i + 1) / n
        p0 = (yo + R_ARCH * math.cos(a0), zo + R_ARCH * math.sin(a0))
        p1 = (yo + R_ARCH * math.cos(a1), zo + R_ARCH * math.sin(a1))
        quad = [(0.0, p0[0], p0[1]), (0.0, p1[0], p1[1]), (0.0, p1[0], Z_BODY_TOP), (0.0, p0[0], Z_BODY_TOP)]
        c = quad + [(SHEET, q[1], q[2]) for q in quad]
        parts.append((f"over_{i}", hexa(c), "paint"))
        cols.append(m.col_hull(f"over_{i}_c", hull(c)))
        r0, r1 = R_ARCH + 0.004, R_ARCH + 0.008  # the arch liner, inboard of the body side
        lq = [(SHEET, yo + r0 * math.cos(a0), zo + r0 * math.sin(a0)), (SHEET, yo + r0 * math.cos(a1), zo + r0 * math.sin(a1)),
              (SHEET, yo + r1 * math.cos(a1), zo + r1 * math.sin(a1)), (SHEET, yo + r1 * math.cos(a0), zo + r1 * math.sin(a0))]
        lc = lq + [(LINER_DEPTH, q[1], q[2]) for q in lq]
        parts.append((f"liner_{i}", hexa(lc), "liner"))
        cols.append(m.col_hull(f"liner_{i}_c", hull(lc)))
        arc.append((-0.004, p0[0], p0[1]))
    arc.append((-0.004, yo - R_ARCH, zo))
    lip = [(-0.004, R_ARCH, Z_SILL)] + arc + [(-0.004, -R_ARCH, Z_SILL)]
    parts.append(("arch_lip", m.tube(np.array(lip), 0.006, sides=10), "paint"))
    return parts, cols


_carp, _carc = _car()
CAR = m.fixed("car", (X_BODY, 0.0, TOP), _carp, _carc, LOOKS)

# -- the cover --------------------------------------------------------------------------------------------------------------------------
TOP_LINE = [(0.180, 0.006), (0.174, 0.015), (0.162, 0.023), (0.145, 0.029), (0.125, 0.033), (0.100, 0.037), (0.050, 0.041), (0.0, 0.042)]


def _top(r):
    """The cover's face height above its lip's underside at radius r."""
    rs, zs = zip(*TOP_LINE[::-1])
    return float(np.interp(r, rs, zs))


def _band(r0, r1):
    """The dish's closed (r, z) section from r0 to r1: flat underside at 4 mm, the top line above."""
    mid = [p for p in TOP_LINE if r0 < p[0] < r1]
    return [(r0, 0.004), (r1, 0.004), (r1, _top(r1))] + mid + [(r0, _top(r0))]


def _sector_hull(name, r0, r1, a0, a1, outer=False):
    sec = [(R_NOTCH_IN, 0.004), (0.165, 0.0), (R_COVER, 0.0)] + [p for p in TOP_LINE if p[0] >= R_NOTCH_IN] if outer else _band(r0, r1)
    pts = [(r * math.cos(a), r * math.sin(a), z) for a in np.radians(np.linspace(a0, a1, 3)) for r, z in sec]
    return m.col_hull(name, hull(pts))


def _cover_frame():
    outer = [(R_NOTCH_IN, 0.004), (0.165, 0.0), (R_COVER, 0.0)] + [p for p in TOP_LINE if p[0] >= R_NOTCH_IN]
    parts = [("hub", revolve(_band(0.0, SLOT_R0), n=72), "cover"), ("ring", revolve(_band(SLOT_R1, R_NOTCH_IN), n=72), "cover"),
             ("rim", revolve(outer, NOTCH_HALF_DEG, 360.0 - NOTCH_HALF_DEG, n=72), "cover"),
             ("cap", m.cylinder(0.032, 0.003, (0, 0, 0.0425), segments=32), "cover_dark")]
    cols = [m.col_hull("hub_c", revolve(_band(0.0, SLOT_R0), n=24))]
    for i, a in enumerate(SPOKE_ANGLES):  # the spokes between the slots
        lo = SLOT_R0 - 0.004
        hgt = _top(SLOT_R1) - 0.004
        spoke = m.box((SLOT_R1 + 0.004 - lo, SPOKE_W, hgt), ((SLOT_R1 + 0.004 + lo) / 2, 0.0, 0.004 + hgt / 2)).turned(a, "z")
        parts.append((f"spoke_{i}", spoke, "cover"))
        cols.append(m.col_hull(f"spoke_{i}_c", spoke))
    for i in range(16):
        cols.append(_sector_hull(f"ring_{i}_c", SLOT_R1, R_NOTCH_IN, 22.5 * i, 22.5 * (i + 1)))
    n = 35
    span = 360.0 - 2 * NOTCH_HALF_DEG
    for i in range(n):
        cols.append(_sector_hull(f"ann_{i}", 0, 0, NOTCH_HALF_DEG + span * i / n, NOTCH_HALF_DEG + span * (i + 1) / n, outer=True))
    return parts, cols


def clip_axis(theta_deg):
    c, s = math.cos(math.radians(theta_deg)), math.sin(math.radians(theta_deg))
    return ("X", 1.0 if c > 0 else -1.0) if abs(c) > abs(s) else ("Y", 1.0 if s > 0 else -1.0)


def _area(sec):
    return 0.5 * float(np.sum(sec[:, 0] * np.roll(sec[:, 1], -1) - np.roll(sec[:, 0], -1) * sec[:, 1]))


def _clip_link(k, theta):
    u = np.array([math.cos(math.radians(theta)), math.sin(math.radians(theta)), 0.0])
    t = np.array([-u[1], u[0], 0.0])
    w = 0.008

    def pts(rz):
        return np.array([r * u + s * w * t + np.array([0, 0, z]) for r, z in rz for s in (-1, 1)])

    head = pts([(CLIP_OUT, CLIP_TOP), (CLIP_OUT, CLIP_TOP - 0.001), (R_LIP - 0.0045, CLIP_TOP - 0.014), (R_LIP - 0.0085, CLIP_TOP - 0.014), (R_LIP - 0.0085, CLIP_TOP)])
    tab = pts([(R_LIP - 0.0075, CLIP_TOP), (R_LIP - 0.004, CLIP_TOP), (R_LIP - 0.004, -0.001), (R_LIP - 0.0075, -0.001)])
    half = math.degrees(w / R_LIP)

    def solid(p):  # the clip as seen: a closed solid through the same points
        sec = np.array([(np.dot(q, u), q[2]) for q in p[::2]])
        return revolve(sec[::-1] if _area(sec) < 0 else sec, theta - half, theta + half, n=3)

    parts = [(f"clip{k}_head", solid(head), "clip"), (f"clip{k}_tab", solid(tab), "clip")]
    cols = [m.col_hull(f"clip{k}_head_c", hull(head)), m.col_hull(f"clip{k}_tab_c", hull(tab))]
    axis, sgn = clip_axis(theta)
    lo, hi = (-0.008, 0.0) if sgn > 0 else (0.0, 0.008)
    return m.slider(f"clip{k}", axis, lo, hi, parts, cols, mass=0.01, spring=CLIP_K, damping=4.0, friction=0.0, effort=200.0, armature=0.001)


_cf, _cc = _cover_frame()
COVER = m.mechanism("cover", (START_XY[0], START_XY[1], TOP + CLIP_DEPTH + 0.002), [_clip_link(k, a) for k, a in enumerate(CLIP_ANGLES)], LOOKS,
                    frame_parts=_cf, frame_colliders=_cc, yaw_deg=START_YAW, friction=(0.4, 0.35), free_mass=COVER_MASS - 0.08,
                    frame_inertia=(0.0036, 0.0036, 0.0073), solver_iters=48)

# -- distractors (SimReady, loose, on the ground) ---------------------------------------------------------------------------------------
RAG = SimReady("rag", pos=(0.33, 0.34, TOP), size=(0.26, 0.233, 0.021), mass=0.05, package="cleaning_cloth_a01", usd="sm_cleaning_cloth_a01_01.usd",
               yaw=math.radians(15.0))
GLOVES = SimReady("gloves", pos=(0.35, -0.35, TOP), size=(0.247, 0.231, 0.016), mass=0.06, package="ppe_gloves_lightDuty_a01",
                  usd="sm_ppe_gloves_lightDuty_a01_01.usd", yaw=math.radians(-20.0))

OBJECTS = [WHEEL, CAR, COVER, RAG, GLOVES]

# -- scoring ----------------------------------------------------------------------------------------------------------------------------


def stem_tip(stem_rad: float) -> np.ndarray:
    """The stem's cap (world) at hinge angle `stem_rad` (about the rim's tangent through its base)."""
    L = STEM_Z1 - STEM_Z0
    return from_wheel([(0.0, STEM_R - L * math.sin(stem_rad), STEM_Z0 + L * math.cos(stem_rad))])[0]


def _axis_vec(k):
    axis, _ = clip_axis(CLIP_ANGLES[k])
    return np.array([1.0, 0.0, 0.0]) if axis == "X" else np.array([0.0, 1.0, 0.0])


def clip_heads(cover) -> np.ndarray:
    """Each clip's catching face (its outer top corner) as it is now, its slider's travel included, in the world."""
    pts = [np.array([CLIP_OUT * math.cos(math.radians(a)), CLIP_OUT * math.sin(math.radians(a)), CLIP_TOP]) + cover.joint(f"clip{k}") * _axis_vec(k)
           for k, a in enumerate(CLIP_ANGLES)]
    return cover.to_world(np.array(pts))


def lip_points(cover) -> np.ndarray:
    """The cover's lip underside beside each clip, at the flange's middle (r 17.5 cm), in the world."""
    r = 0.5 * (R_GROOVE + R_FLANGE)
    return cover.to_world(np.array([(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), 0.0) for a in CLIP_ANGLES]))


def cover_gap(cover, tips) -> float:
    q = cover.to_local(tips)
    dr = np.maximum(np.hypot(q[:, 0], q[:, 1]) - R_COVER, 0.0)
    dz = np.maximum(np.maximum(q[:, 2] - 0.043, -CLIP_DEPTH - q[:, 2]), 0.0)
    return float(np.min(np.hypot(dr, dz)))


def now(env) -> dict:
    cv, wh = m.state(env, "cover"), m.state(env, "wheel")
    stem = wh.joint("stem")
    q = cv.to_local(stem_tip(stem)[None])[0]
    lateral = abs(q[1]) if q[0] > 0 else 1.0  # off the notch's centre line (the cover's +x)
    in_notch = bool(lateral <= NOTCH_TOL and R_NOTCH_IN <= math.hypot(q[0], q[1]) <= R_COVER and -0.030 <= q[2] <= 0.020)
    heads = to_wheel(clip_heads(cv))  # in the wheel's frame
    hook = np.hypot(heads[:, 0], heads[:, 1]) - R_LIP  # how far each catching face is out past the lip's inner face
    behind = Z_LIP_BOT - heads[:, 2]  # how far it is behind the lip's back edge
    gaps = to_wheel(lip_points(cv))[:, 2] - Z_F  # the cover's lip off the flange beside each clip
    seated = [bool(hook[k] >= HOOK and behind[k] >= -BEHIND_TOL and abs(gaps[k]) <= ENGAGE_GAP) for k in range(8)]
    tips = m.fingertips(env)
    gap = cover_gap(cv, tips)
    ax = to_wheel(cv.to_world(np.array([[0.0, 0.0, 0.0]])))[0]
    return {"stem_deg": math.degrees(stem), "lateral": float(lateral), "in_notch": in_notch, "hook": hook.tolist(), "behind": behind.tolist(),
            "gaps": gaps.tolist(), "seated": seated, "n": int(sum(seated)), "clear": gap >= CLEAR, "hand_gap": gap,
            "centre": [float(ax[0]), float(ax[1]), float(ax[2] - Z_F)]}


class Trial:
    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the bent-stem record is cleared
        self.bent = False
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.last = {}

    def step(self, env):
        c = now(env)
        self.last = c
        if abs(c["stem_deg"]) > STEM_BENT_DEG:
            self.bent = True
        ok = c["n"] == 8 and c["in_notch"] and c["clear"] and not self.bent
        self.hold = self.hold + 1 if ok else 0
        self.full = ok and self.hold >= HOLD_STEPS  # the trial ends on success, so this is never carried past a later change
        if self.bent:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["in_notch"]:
            self.score = 0.2 + 0.075 * c["n"]
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the scripted demo (demokit, closed loop on the cover's pose; never the scorer's state) ----------------------------------------------

R_GRIP = 0.092  # where the fingers straddle a spoke
GRIP_Z = -0.012  # fingertips this far behind the cover's lip plane while gripping a spoke: the pads take the spoke's whole depth
PAD = 0.013  # fingertip pads past the TCP
LIFT_H = 0.190  # TCP height when the cover is lifted flat (the highest a straight-down grip reaches here)
STAND_H = HUB_H + 0.005  # TCP height once the cover is stood up: it rises while it turns, so its lower edge stays 10 cm off the ground
PUSH = 0.050  # m: the grips driven this far past where the cover's clips meet the rim
PRESS_R = 0.158
PRESS_DEPTH = 0.025  # m: pads this far past the seated cover's face
DEMO_DEBUG = False


def _cover_axes(st):
    R = st.rotation()
    return st.pos, R[:, 0], R[:, 1], R[:, 2]  # origin, notch direction, its normal in the face, out of the face


def _swing(arm, center, axis, deg, rise, t, label):
    """Swing the TCP and the grip together about a line through `center` along `axis`, while the whole rises by `rise`."""
    from yam_arena import ik
    from yam_arena.urdf import axis_angle_matrix

    import demokit as d

    axis, center = np.asarray(axis, float), np.asarray(center, float)
    p0, r0 = ik.tcp_pose(arm.q, arm.base)
    n = max(1, round(t * 20))
    worst = (0.0, 0.0)
    rot = r0
    for k in range(1, n + 1):
        u = ik.smoothstep(k / n)
        a = axis_angle_matrix(axis, math.radians(deg) * u)
        rot = a @ r0
        mm, de = arm._solve(center + a @ (p0 - center) + np.array([0.0, 0.0, rise * u]), rot)
        worst = (max(worst[0], mm), max(worst[1], de))
        yield
    arm.grip = d._as_grip(rot)
    arm._note(*worst, label)


def _force_probe(ctx, steps):
    """Debugging only: the push each arm's joint drives exert along +x (from their position errors), printed at the end."""
    from yam_arena import ik
    from yam_arena.constants import SHOULDER, WRIST

    K = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
    cap = np.array([SHOULDER["effort"]] * 3 + [WRIST["effort"]] * 3)
    peak = {"left": 0.0, "right": 0.0}
    for _ in range(steps):
        for a in (ctx.left, ctx.right):
            q = np.array(ctx.observed(a.side)[:6], float)
            tau = np.clip(K * (a.q - q), -cap, cap)
            p0 = ik.tcp_pose(q, a.base)[0]
            J = np.zeros((3, 6))
            for i in range(6):
                dq = q.copy()
                dq[i] += 1e-6
                J[:, i] = (ik.tcp_pose(dq, a.base)[0] - p0) / 1e-6
            f = np.linalg.lstsq(J.T, tau, rcond=None)[0]
            peak[a.side] = max(peak[a.side], float(f[0]))
        yield
    print("[wheel-demo] peak push along +x (N):", {k: round(v, 1) for k, v in peak.items()}, flush=True)


def demo_script(ctx):
    """Both arms straddle the two spokes on either side of the cover (along the robot's left-right line) with open fingers down through
    its slots, grip them and lift the cover to the axle's height. They stand it up (turning both wrists about the line through the two
    grips, face towards the robot), then turn it about its own axis until the notch is at the top, over the stem. They carry it onto the
    wheel's axis, push it on (the upper grip first, then the lower, then both), let go and back off, then press the cover's rim in over
    each clip in mirrored pairs, and move away.
    Failure checks (--mode): none does nothing; noturn stands the cover up and pushes it on without turning it; onearm uses the right arm
    only (grip, lift, stand up beside the wheel as far as one arm goes, push)."""
    import demokit as d
    from yam_arena.constants import FINGER_CLOSED as FC

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "noturn", "onearm"):
        raise ValueError(mode)
    for _ in range(10):
        yield
    if mode == "none":
        for _ in range(60):
            yield
        return
    OPEN_W = -0.019  # finger joint: just wider than a spoke

    def cover():
        return ctx.state("cover")

    o, ex, ey, ez = _cover_axes(cover())
    z_grip = o[2] + GRIP_Z + PAD

    yaw = math.degrees(math.atan2(ex[1], ex[0]))  # the notch's direction, as seen

    def spoke(local_deg):
        a = math.radians(yaw + local_deg)
        return np.array([o[0] + R_GRIP * math.cos(a), o[1] + R_GRIP * math.sin(a), z_grip])

    # the left hand on the spoke at the robot's left, the right hand on the spoke nearest it on its own side (their wrists' turning
    # range decides which: the right wrist could not follow the later turns from the spoke opposite the left hand's)
    gl, gr = spoke(157.5), spoke(292.5)
    up = np.array([0.0, 0.0, 0.07])
    arms = [(l, gl, d.down(-90)), (r, gr, d.down(45))] if mode != "onearm" else [(r, gr, d.down(45))]
    yield from ctx.together(*[a.move(p + up, grip=g, finger=OPEN_W, t=2.0, label="over a spoke") for a, p, g in arms])
    yield from ctx.together(*[a.move(p, finger=OPEN_W, t=1.2, label="straddle the spoke") for a, p, g in arms])
    yield from ctx.together(*[a.move(p, finger=FC, t=0.8, label="grip") for a, p, g in arms])
    lift = np.array([0.0, 0.0, TOP + LIFT_H - z_grip])
    yield from ctx.together(*[a.move(p + lift, t=2.0, label="lift") for a, p, g in arms])
    c = cover().pos
    if mode == "full":  # turn it flat, notch away from the robot: stood up, the notch is then at the top
        yield from ctx.together(*[_swing(a, c, (0.0, 0.0, 1.0), -yaw, 0.0, 4.0, "turn the notch") for a, p, g in arms])
    c = cover().pos
    yield from ctx.together(*[_swing(a, c, (0.0, 1.0, 0.0), -90.0, TOP + HUB_H - c[2], 4.0, "stand it up") for a, p, g in arms])
    if mode == "onearm":
        target = from_wheel([(0.0, 0.0, Z_F + 0.03)])[0]
        o = cover().pos
        yield from r.move(r.at() + (target - o), t=2.5, label="to the wheel")
        yield from r.move(r.at() + np.array([0.03 + PUSH, 0.0, 0.0]), t=2.5, label="push")
        yield from r.open()
        yield from r.move(r.at() - np.array([0.10, 0.0, 0.0]), t=1.5, label="back")
        yield from ctx.until(lambda: False, timeout=4.0, label="wait")
        return
    def relax():
        """Take each arm's command to where it is: the two hands on one rigid cover otherwise pull against each other."""
        for a in (l, r):
            a.q = np.array(ctx.observed(a.side)[:6], float)
            a.cmd = np.r_[a.q, a.cmd[6]]
        yield

    # onto the wheel's axis 7 cm out, the notch turned square over the stem, then in to 3 cm out (the clips' ramps just short of the lip)
    for k, out in enumerate((0.07, 0.07, 0.07, 0.03, 0.03, 0.03, 0.03)):
        target = from_wheel([(0.0, 0.0, Z_F + out)])[0]
        dlt = target - cover().pos
        if DEMO_DEBUG:
            _R = cover().rotation()
            print("[wheel-demo] align", k, np.round(to_wheel(cover().pos[None])[0], 4), "ex", np.round(_R[:, 0], 3), "ez", np.round(_R[:, 2], 3), "tcp", np.round(l.tcp(), 3), np.round(l.at(), 3), np.round(r.tcp(), 3), np.round(r.at(), 3), flush=True)
        yield from ctx.together(l.move(l.at() + dlt, t=2.0 if k in (0, 3) else 0.8, label="to the wheel"),
                                r.move(r.at() + dlt, t=2.0 if k in (0, 3) else 0.8, label="to the wheel"))
        if k in (0, 1):  # face it square to the wheel, then (unless skipping the turn) turn the notch square over the stem
            st = cover()
            ez = st.rotation()[:, 2]
            want = np.array([-1.0, 0.0, 0.0])
            axis = np.cross(ez, want)
            ang = math.degrees(math.asin(min(1.0, float(np.linalg.norm(axis)))))
            if ang > 0.3:
                yield from ctx.together(l.arc(st.pos, axis, ang, t=1.2, label="face the wheel"), r.arc(st.pos, axis, ang, t=1.2, label="face the wheel"))
            st = cover()
            ex = st.rotation()[:, 0]
            err = math.degrees(math.atan2(-ex[1], ex[2]))  # the notch's angle from the top, clockwise as the robot sees it
            if mode == "full" and abs(err) > 0.3:
                yield from ctx.together(l.arc(st.pos, (1.0, 0.0, 0.0), -err, t=1.0, label="square the notch"),
                                        r.arc(st.pos, (1.0, 0.0, 0.0), -err, t=1.0, label="square the notch"))
    if DEMO_DEBUG:
        st = cover()
        print("[wheel-demo] at the wheel:", to_wheel(st.pos[None])[0], flush=True)
    # push it on: the upper grip first (its side's clips hook on), then the lower, then both
    ax = np.array([1.0, 0.0, 0.0])
    yield from ctx.together(l.move(l.at() + 0.030 * ax, t=1.5, label="to the clips"), r.move(r.at() + 0.030 * ax, t=1.5, label="to the clips"))
    probe = [_force_probe(ctx, 70)] if DEMO_DEBUG else []
    yield from ctx.together(l.move(l.at() + PUSH * ax, t=2.5, label="push it on"), r.move(r.at() + PUSH * ax, t=2.5, label="push it on"), *probe)
    yield from ctx.together(l.wait(1.0), r.wait(1.0))
    if DEMO_DEBUG:
        st = cover()
        print("[wheel-demo] pushed:", to_wheel(st.pos[None])[0], flush=True)
    yield from ctx.together(l.open(), r.open())
    yield from ctx.together(l.move(l.at() - 0.08 * ax, t=1.0, label="back off"), r.move(r.at() - 0.08 * ax, t=1.0, label="back off"))
    # press round: closed fingers pointing at the wheel, over each clip, in mirrored pairs
    st = cover()
    o, ex, ey, ez = _cover_axes(st)
    face = d.level(0.0)
    pairs = [(22.5, -22.5), (67.5, -67.5), (112.5, -112.5), (157.5, -157.5)]
    for al, ar in pairs:
        pts = {}
        for side, a in (("left", al), ("right", ar)):
            # the cover's local angle with its +y on the world's +y side (the left arm's)
            sgn = 1.0 if ey[1] >= 0 else -1.0
            u = math.cos(math.radians(a)) * ex + sgn * math.sin(math.radians(a)) * ey
            surf = o + PRESS_R * u + _top(PRESS_R) * ez
            pts[side] = surf
        gol = {s: p + ez * (PAD + 0.035) for s, p in pts.items()}  # 3.5 cm out from the face
        pre = {s: p + ez * (PAD - PRESS_DEPTH) for s, p in pts.items()}
        yield from ctx.together(l.move(gol["left"], grip=face, finger=FC, t=1.2, label="over a clip"),
                                r.move(gol["right"], grip=face, finger=FC, t=1.2, label="over a clip"))
        yield from ctx.together(l.move(pre["left"], t=1.2, label="press"), r.move(pre["right"], t=1.2, label="press"))
        yield from ctx.together(l.wait(0.4), r.wait(0.4))
        yield from ctx.together(l.move(gol["left"], t=0.6, label="lift off"), r.move(gol["right"], t=0.6, label="lift off"))
    yield from ctx.together(l.move((0.20, 0.20, TOP + 0.30), t=1.5, label="away"), r.move((0.20, -0.20, TOP + 0.30), t=1.5, label="away"))
    if DEMO_DEBUG:
        st = cover()
        print("[wheel-demo] end:", to_wheel(st.pos[None])[0], flush=True)
    yield from ctx.until(lambda: False, timeout=4.0, label="wait")
