"""Put away three jars of home-canned tomato sauce: the one whose lid did not seal goes in the fridge.

Three Ball half-pint jars of tomato sauce, canned yesterday and cooled overnight, stand in a row on the counter with
their screw bands off. One lid did not seal. Nothing shows it except that its centre gives and springs back when
pressed (the National Center for Home Food Preservation's test). A wooden pantry shelf (a 10 cm riser) and a small
mini fridge with its door open stand on the counter; on the fridge floor is a tray labelled USE IN 1 WEEK.
Each jar stands on a pocket scale (the real trial reads the press on it). The instruction is only to put the jars away
for the winter. The safe way: press the centre of each lid (1.5 N or more, 0.3 s), put the one that gives in the fridge
tray and the other two on the shelf. The unsealed jar let go on (or under) the pantry shelf scores 0 for the whole run.

Which jar is unsealed is the only thing that changes between seeds (layout.unsealed_index). Frame: x forward from the
arm bases, y left, z up, metres. Every size and threshold is in layout.py.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies are simulated)
import layout as L  # noqa: E402
from bodykit import Body, BoxShape, CylShape  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
CONTROL_HZ = 20.0

SAUCE, GLASS, LID_C, DOT = (0.62, 0.10, 0.06), (0.82, 0.88, 0.86), (0.78, 0.76, 0.70), (0.97, 0.97, 0.95)
BAMBOO, FRIDGE_C, FRIDGE_IN_C = (0.80, 0.66, 0.42), (0.93, 0.93, 0.93), (0.86, 0.88, 0.90)
JARS = [f"jar{i + 1}" for i in range(L.N_JARS)]
LIDS = [f"lid{i + 1}" for i in range(L.N_JARS)]


def _ring(z, r_in, r_out, h, n, color, collide=True) -> list:
    """An annulus of n flat staves between r_in and r_out, h thick, centred at height z."""
    r, w = (r_in + r_out) / 2, r_out - r_in
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        out.append(BoxShape((r * math.cos(a), r * math.sin(a), z), (w, 2 * math.pi * r / n * 1.10, h), color, yaw=a,
                            collide=collide))
    return out


def _jar(i: int) -> Body:
    """The glass jar with its sauce, neck and the lid's outer ring (the sealing ring under the rim); identical for all
    three jars. A white 12 mm dot on the far side carries the jar's number in a real trial."""
    shapes = [CylShape((0, 0, L.SAUCE_TOP / 2), L.JAR_R, L.SAUCE_TOP, SAUCE),
              CylShape((0, 0, (L.SAUCE_TOP + L.JAR_BODY_TOP) / 2), L.JAR_R, L.JAR_BODY_TOP - L.SAUCE_TOP, GLASS),
              CylShape((0, 0, (L.JAR_BODY_TOP + L.NECK_TOP) / 2), L.NECK_R, L.NECK_TOP - L.JAR_BODY_TOP, GLASS)]
    shapes += _ring(L.NECK_TOP + L.LID_T / 2, L.BUTTON_R, L.LID_R, L.LID_T, 24, LID_C)
    shapes.append(BoxShape((L.JAR_R + 0.0004, 0.0, 0.050), (0.0008, 0.012, 0.012), DOT, collide=False))
    return Body.build(JARS[i], pos=(L.JAR_X, L.JAR_YS[i], TOP + L.JAR_Z0), mass=L.JAR_MASS, shapes=shapes, color=GLASS,
                      friction=L.GLASS_MU, com=L.JAR_COM, solver_iterations=32)


def _lid(i: int) -> Body:
    """The lid's centre panel: a disc flush with the lid ring, on a slider down the jar's axis."""
    z0 = TOP + L.JAR_Z0 + L.LID_TOP - L.BUTTON_T
    return Body.build(LIDS[i], pos=(L.JAR_X, L.JAR_YS[i], z0), mass=L.BUTTON_MASS,
                      shapes=[CylShape((0, 0, L.BUTTON_T / 2), L.BUTTON_R, L.BUTTON_T, LID_C)], color=LID_C,
                      friction=(0.6, 0.5), solver_iterations=32)


for _i in range(L.N_JARS):
    bodykit.slider(f"press{_i + 1}", JARS[_i], LIDS[_i], (L.JAR_X, L.JAR_YS[_i], TOP + L.JAR_Z0 + L.LID_TOP - L.BUTTON_T),
                   axis="Z",
                   limits=(-L.TRAVEL, 0.0), stiffness=L.SEALED_K, damping=L.SEALED_DAMP, target=0.0, max_force=1.0e4)


# stroke letters in a 0.6 x 1 box (u to the reader's right, v up), for the printed labels
_STROKES = {
    "P": [((0, 0), (0, 1)), ((0, 1), (0.55, 1)), ((0.55, 1), (0.55, 0.5)), ((0.55, 0.5), (0, 0.5))],
    "A": [((0, 0), (0.3, 1)), ((0.3, 1), (0.6, 0)), ((0.12, 0.4), (0.48, 0.4))],
    "N": [((0, 0), (0, 1)), ((0, 1), (0.6, 0)), ((0.6, 0), (0.6, 1))],
    "T": [((0, 1), (0.6, 1)), ((0.3, 1), (0.3, 0))],
    "R": [((0, 0), (0, 1)), ((0, 1), (0.55, 1)), ((0.55, 1), (0.55, 0.5)), ((0.55, 0.5), (0, 0.5)),
          ((0.2, 0.5), (0.6, 0))],
    "Y": [((0, 1), (0.3, 0.5)), ((0.6, 1), (0.3, 0.5)), ((0.3, 0.5), (0.3, 0))],
    "F": [((0, 0), (0, 1)), ((0, 1), (0.6, 1)), ((0, 0.55), (0.45, 0.55))],
    "I": [((0.3, 0), (0.3, 1)), ((0.1, 1), (0.5, 1)), ((0.1, 0), (0.5, 0))],
    "D": [((0, 0), (0, 1)), ((0, 1), (0.35, 1)), ((0.35, 1), (0.6, 0.75)), ((0.6, 0.75), (0.6, 0.25)),
          ((0.6, 0.25), (0.35, 0)), ((0.35, 0), (0, 0))],
    "G": [((0.6, 1), (0, 1)), ((0, 1), (0, 0)), ((0, 0), (0.6, 0)), ((0.6, 0), (0.6, 0.45)), ((0.6, 0.45), (0.3, 0.45))],
    "E": [((0, 0), (0, 1)), ((0, 1), (0.6, 1)), ((0, 0.5), (0.45, 0.5)), ((0, 0), (0.6, 0))],
    "U": [((0, 1), (0, 0)), ((0, 0), (0.6, 0)), ((0.6, 0), (0.6, 1))],
    "S": [((0.6, 1), (0, 1)), ((0, 1), (0, 0.5)), ((0, 0.5), (0.6, 0.5)), ((0.6, 0.5), (0.6, 0)), ((0.6, 0), (0, 0))],
    "W": [((0, 1), (0.15, 0)), ((0.15, 0), (0.3, 0.6)), ((0.3, 0.6), (0.45, 0)), ((0.45, 0), (0.6, 1))],
    "H": [((0, 0), (0, 1)), ((0.6, 0), (0.6, 1)), ((0, 0.5), (0.6, 0.5))],
    "K": [((0, 0), (0, 1)), ((0.6, 1), (0, 0.45)), ((0.2, 0.62), (0.6, 0))],
    "1": [((0.3, 0), (0.3, 1)), ((0.3, 1), (0.1, 0.8)), ((0.1, 0), (0.5, 0))],
    " ": [],
}


def _label(text: str, x: float, y_mid: float, z0: float, size: float, color) -> list:
    """Visual-only stroke letters on a face looking toward the robot (-x), read left (+y) to right (-y)."""
    pitch = 0.85 * size
    y0 = y_mid + pitch * len(text) / 2 - 0.1 * size
    out = []
    for k, ch in enumerate(text):
        for (u1, v1), (u2, v2) in _STROKES[ch]:
            out.append(bodykit.segment((x, y0 - k * pitch - u1 * size, z0 + v1 * size),
                                       (x, y0 - k * pitch - u2 * size, z0 + v2 * size), 0.12 * size, color,
                                       collide=False))
    return out


def _flat_label(text: str, x0: float, y_mid: float, z: float, size: float, color) -> list:
    """Visual-only stroke letters lying on a level face, read from the robot's side: letters' tops away from the robot
    (+x), read left (+y) to right (-y)."""
    pitch = 0.85 * size
    y0 = y_mid + pitch * len(text) / 2 - 0.1 * size
    out = []
    for k, ch in enumerate(text):
        for (u1, v1), (u2, v2) in _STROKES[ch]:
            out.append(bodykit.segment((x0 + v1 * size, y0 - k * pitch - u1 * size, z),
                                       (x0 + v2 * size, y0 - k * pitch - u2 * size, z), 0.12 * size, color,
                                       collide=False))
    return out


INK, CARD, TIN, TIN_LID = (0.10, 0.10, 0.12), (0.96, 0.95, 0.90), (0.75, 0.20, 0.15), (0.70, 0.70, 0.72)


def _shelf() -> Body:
    """The riser (top board on two side panels) with the pantry's look: a backboard with a printed PANTRY card, two
    small tins at its far left end and two cans of tomatoes under the board (all visual only; real cardboard and
    real tins in a trial, clear of every place the jars go)."""
    w, d, h, t = L.SHELF_W, L.SHELF_D, L.SHELF_H, L.SHELF_T
    shapes = [BoxShape((0, 0, h - t / 2), (d, w, t), BAMBOO)]
    for s in (1.0, -1.0):
        shapes.append(BoxShape((0, s * (w / 2 - t / 2), (h - t) / 2), (d, t, h - t), BAMBOO))
    bx, bh = d / 2 + 0.004, h + 0.090
    shapes.append(BoxShape((bx + 0.003, 0, bh / 2), (0.006, w, bh), BAMBOO, collide=False))
    shapes.append(BoxShape((bx - 0.0005, 0, h + 0.045), (0.001, 0.25, 0.060), CARD, collide=False))
    shapes += _label("PANTRY", bx - 0.0015, 0.0, h + 0.025, 0.040, INK)
    for x in (-0.055, 0.055):  # two 50 mm spice tins on the board's far left end
        shapes.append(CylShape((x, 0.158, h + 0.030), 0.025, 0.060, TIN, collide=False))
        shapes.append(CylShape((x, 0.158, h + 0.061), 0.025, 0.002, TIN_LID, collide=False))
    for y in (-0.110, 0.110):  # two 8 oz cans (68 x 83 mm) under the board
        shapes.append(CylShape((0.0, y, 0.0415), 0.034, 0.083, TIN, collide=False))
    return Body.build("shelf", pos=(L.SHELF_XY[0], L.SHELF_XY[1], TOP), mass=1.0, shapes=shapes, color=BAMBOO,
                      kinematic=True, friction=L.WOOD_MU)


def _fridge() -> Body:
    """Cabinet walls round the listed inside space, and the door swung open on its right-hand hinge. Origin at the
    middle of the cabinet's front face, on the table."""
    dx, wy, hz = L.FRIDGE_OUT
    ix, iy, iz = L.FRIDGE_IN
    fl = L.FRIDGE_FLOOR
    side = (wy - iy) / 2
    top_t = hz - fl - iz
    shapes = [BoxShape((dx / 2, 0, fl / 2), (dx, wy, fl), FRIDGE_C),  # base and floor
              BoxShape((dx / 2, 0, hz - top_t / 2), (dx, wy, top_t), FRIDGE_C),  # top
              BoxShape(((ix + dx) / 2, 0, fl + iz / 2), (dx - ix, iy, iz), FRIDGE_C)]  # back wall (cooler and fan)
    for s in (1.0, -1.0):
        shapes.append(BoxShape((dx / 2, s * (iy / 2 + side / 2), fl + iz / 2), (dx, side, iz), FRIDGE_C))
    shapes.append(BoxShape((ix / 2, 0, fl + 0.0004), (ix - 0.004, iy - 0.004, 0.0008), FRIDGE_IN_C, collide=False))
    # the door: 40 mm deep, the cabinet's full width and height, hinged at the front right corner, open 110 degrees
    a = math.radians(L.DOOR_OPEN_DEG)
    hinge = np.array([0.0, -wy / 2])
    along = np.array([-math.sin(a), math.cos(a)])  # closed, the door runs from the hinge toward +y
    out = np.array([-math.cos(a), -math.sin(a)])  # its thickness, toward the outside face
    c = hinge + along * wy / 2 + out * L.DOOR_T / 2
    shapes.append(BoxShape((c[0], c[1], hz / 2), (L.DOOR_T, wy, hz), FRIDGE_C, yaw=a))
    # the fridge's look (visual only): grey door seal round the opening, the handle on the door's outer face, a lit
    # ceiling, the blue control display on top and the printed word FRIDGE above the opening
    seal = (0.35, 0.36, 0.38)
    for s in (1.0, -1.0):
        shapes.append(BoxShape((-0.0005, s * (iy / 2 + 0.004), fl + iz / 2), (0.001, 0.008, iz), seal, collide=False))
    for z in (fl - 0.004, fl + iz + 0.004):
        shapes.append(BoxShape((-0.0005, 0, z), (0.001, iy + 0.016, 0.008), seal, collide=False))
    hp = hinge + along * (wy - 0.025) + out * (L.DOOR_T + 0.010)
    shapes.append(BoxShape((hp[0], hp[1], hz / 2), (0.012, 0.016, 0.150), seal, yaw=a, collide=False))
    shapes.append(BoxShape((ix / 2, 0, fl + iz - 0.0006), (0.060, 0.080, 0.001), (1.0, 0.98, 0.85), collide=False))
    shapes.append(BoxShape((0.040, 0, hz + 0.001), (0.030, 0.070, 0.002), (0.15, 0.35, 0.85), collide=False))
    shapes += _label("FRIDGE", -0.0015, 0.0, fl + iz + 0.008, 0.020, (0.10, 0.30, 0.75))
    # the "use first" tray on the floor (a flat plastic tray with a 6 mm rim) and its printed card, which hangs from the
    # tray's front lip over the fridge's front edge, below the opening (card visual only; tray walls collide)
    tx0, td, tw, tf, tr, tt = L.TRAY_X0 - L.FRIDGE_FRONT_X, L.TRAY_D, L.TRAY_W, L.TRAY_FLOOR, L.TRAY_RIM, L.TRAY_WALL
    tray_c = (0.30, 0.62, 0.40)
    shapes.append(BoxShape((tx0 + td / 2, 0, fl + tf / 2), (td, tw, tf), tray_c))
    for s in (1.0, -1.0):
        shapes.append(BoxShape((tx0 + td / 2, s * (tw / 2 - tt / 2), fl + tf + tr / 2), (td, tt, tr), tray_c))
    for xw in (tx0 + tt / 2, tx0 + td - tt / 2):
        shapes.append(BoxShape((xw, 0, fl + tf + tr / 2), (tt, tw, tr), tray_c))
    shapes.append(BoxShape((-0.0006, 0, fl - 0.002 - L.LABEL_H / 2), (0.001, L.LABEL_W, L.LABEL_H), CARD, collide=False))
    shapes += _label(L.TRAY_LABEL, -0.0016, 0.0, fl - 0.002 - L.LABEL_H / 2 - L.LABEL_LETTER / 2, L.LABEL_LETTER, INK)
    cx0, cx1, cw = L.TOP_CARD
    shapes.append(BoxShape(((cx0 + cx1) / 2, 0, hz + 0.0003), (cx1 - cx0, cw, 0.0006), CARD, collide=False))
    shapes += _flat_label("TRAY", cx0 + 0.040, 0.0, hz + 0.0012, L.LABEL_LETTER, INK)
    shapes += _flat_label(L.TRAY_LABEL, cx0 + 0.010, 0.0, hz + 0.0012, L.LABEL_LETTER, INK)
    return Body.build("fridge", pos=(L.FRIDGE_FRONT_X, L.FRIDGE_Y, TOP), mass=1.9, shapes=shapes, color=FRIDGE_C,
                      kinematic=True, friction=L.FRIDGE_MU)


def _scales() -> Body:
    """Three pocket scales (Brecknell EPB-3000g, 140 x 107 x 20 mm, 100 x 100 mm steel pan), one under each jar's start
    place, display toward the robot. Fixed in simulation: the press force is read from the lid centre's spring, the
    same push the real scale shows as a rise in grams."""
    shapes = []
    for y in L.JAR_YS:
        xc = L.JAR_X + L.SCALE_PAN / 2 - L.SCALE_L / 2
        shapes.append(BoxShape((xc, y, L.SCALE_H / 2 - 0.001), (L.SCALE_L, L.SCALE_W, L.SCALE_H - 0.002),
                               (0.12, 0.12, 0.13)))
        shapes.append(BoxShape((L.JAR_X, y, L.SCALE_H - 0.001), (L.SCALE_PAN, L.SCALE_PAN, 0.002), (0.78, 0.79, 0.80)))
        shapes.append(BoxShape((L.JAR_X - L.SCALE_PAN / 2 - 0.020, y, L.SCALE_H + 0.0003), (0.016, 0.040, 0.0006),
                               (0.70, 0.80, 0.72), collide=False))
    return Body.build("scales", pos=(0.0, 0.0, TOP), mass=1.0, shapes=shapes, color=(0.12, 0.12, 0.13), kinematic=True,
                      friction=(0.5, 0.4))


OBJECTS = [_shelf(), _fridge(), _scales()] + [_jar(i) for i in range(L.N_JARS)] + [_lid(i) for i in range(L.N_JARS)]
NAMES = [o.name for o in OBJECTS]
LOOSE = tuple(JARS + LIDS)

# -- episode state ------------------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["JARS_DEBUG"], "w") if os.environ.get("JARS_DEBUG") else None


def _blank() -> None:
    keep = {k: _EP.get(k) for k in ("start", "unsealed", "seed")}
    _EP.clear()
    _EP.update(step=0, hands=None, report={}, now=None, breach=None, travel=[0.0] * L.N_JARS, max_tilt=[0.0] * L.N_JARS,
               free_z=[None] * L.N_JARS, moved=[False] * L.N_JARS,
               press_travel=[0.0] * L.N_JARS, touch_run=[0] * L.N_JARS, pressed=[False] * L.N_JARS,
               force_max=[0.0] * L.N_JARS, tip_force_max=[0.0] * L.N_JARS,
               tip_gap=[None] * L.N_JARS, **keep)


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def tilt_deg(state) -> float:
    return math.degrees(state.tilt())


def travel(jar, lid) -> float:
    """How far (m) the lid's centre panel is pushed down from its rest, in the jar's frame."""
    return (L.LID_TOP - L.BUTTON_T) - float(jar.to_local(np.asarray(lid.pos, float))[2])


def lid_force(i: int, t: float) -> float:
    """The push (N) on jar i's lid centre, from its spring: stiffness x how far it is pushed in (the unsealed lid's spring
    already holds it up with 1.5 N, so any travel means at least that). No travel, no push."""
    if not math.isfinite(t) or t <= 1.0e-5:
        return 0.0
    if i == _EP.get("unsealed"):
        return L.UNSEALED_K * (L.UNSEALED_PRELOAD + t)
    return L.SEALED_K * t


def finger_points(p, r, half: float) -> list:
    """Points on both fingers of a gripper (TCP at p, rotation r, pads half m from the centre line)."""
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> list | None:
    """Finger points of both grippers, or None if any reading is missing or not finite (rules then fail closed)."""
    from bodykit import as_numpy
    from yam_arena import ik, station

    pts = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        pts += finger_points(p, r, half)
    if not pts or not all(np.all(np.isfinite(q)) for q in pts):
        return None
    return pts


def clearance(state, pts) -> float:
    """Smallest distance (m) from the points to a jar's outline (a cylinder from its base to the lid top)."""
    best = float("inf")
    for p in pts:
        loc = state.to_local(p)
        dr = max(0.0, math.hypot(loc[0], loc[1]) - L.JAR_R)
        dz = max(0.0, -loc[2], loc[2] - L.LID_TOP)
        best = min(best, math.hypot(dr, dz))
    return best


def at_start(i: int, jar) -> bool:
    """The jar still stands where it started: centre within 10 mm, base within 5 mm of the counter, upright."""
    p0 = _EP["start"][JARS[i]][0]
    p = np.asarray(jar.pos, float)
    return bool(np.linalg.norm(p[:2] - p0[:2]) <= L.AT_START_XY and abs(p[2] - p0[2]) <= L.AT_START_Z
                and tilt_deg(jar) < L.UPRIGHT)


def tip_on_lid(jar, pts) -> float | None:
    """Height (m) above the lid's top of the lowest finger point over the lid's centre (within 15 mm of the jar's
    axis), or None if no finger point is over it within 2 cm."""
    best = None
    for p in pts:
        loc = jar.to_local(p)
        dz = float(loc[2]) - L.LID_TOP
        if math.hypot(loc[0], loc[1]) <= L.PRESS_R and -0.006 <= dz <= 0.020:
            best = dz if best is None else min(best, dz)
    return best


def in_pantry(jar) -> bool:
    """The jar's centre over the shelf's footprint, standing on its top board or under it on the table."""
    p = np.asarray(jar.pos, float)
    over = abs(p[0] - L.SHELF_XY[0]) <= L.SHELF_D / 2 and abs(p[1] - L.SHELF_XY[1]) <= L.SHELF_W / 2
    return bool(over and (abs(p[2] - L.SHELF_TOP) <= L.REST_M or abs(p[2] - TOP) <= L.REST_M))


def on_shelf(jar) -> bool:
    """Standing on the shelf's top board, all of its base on the board, upright."""
    p = np.asarray(jar.pos, float)
    inside = (abs(p[0] - L.SHELF_XY[0]) <= L.SHELF_D / 2 - L.JAR_R
              and abs(p[1] - L.SHELF_XY[1]) <= L.SHELF_W / 2 - L.JAR_R)
    return bool(inside and abs(p[2] - L.SHELF_TOP) <= L.REST_M and tilt_deg(jar) < L.UPRIGHT)


def in_fridge(jar) -> bool:
    """Standing on the fridge floor, all of it behind the door plane and inside the side walls, upright."""
    p = np.asarray(jar.pos, float)
    ix, iy, _ = L.FRIDGE_IN
    inside = (L.FRIDGE_FRONT_X + L.JAR_R <= p[0] <= L.FRIDGE_FRONT_X + ix - L.JAR_R + 0.001
              and abs(p[1] - L.FRIDGE_Y) <= iy / 2 - L.JAR_R + 0.001)
    return bool(inside and abs(p[2] - L.FRIDGE_FLOOR_TOP) <= L.REST_M and tilt_deg(jar) < L.UPRIGHT)


def in_tray(jar) -> bool:
    """Standing on the use-first tray's floor, all of its base inside the tray's rim, upright."""
    p = np.asarray(jar.pos, float)
    x0 = L.TRAY_X0 + L.TRAY_WALL + L.JAR_R - 0.001
    x1 = L.TRAY_X0 + L.TRAY_D - L.TRAY_WALL - L.JAR_R + 0.001
    inside = x0 <= p[0] <= x1 and abs(p[1] - L.FRIDGE_Y) <= L.TRAY_W / 2 - L.TRAY_WALL - L.JAR_R + 0.001
    return bool(inside and abs(p[2] - L.TRAY_FLOOR_TOP) <= L.REST_M and tilt_deg(jar) < L.UPRIGHT)


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["now"] = st
    k = _EP["step"]
    try:
        _EP["hands"] = _hand_points(env.unwrapped)
    except Exception as err:  # noqa: BLE001  (fail closed: no reading counts as a breach and as not released)
        _EP["hands"] = None
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    pts = _EP["hands"]
    if _EP.get("start") is not None and k >= 2:
        for i, (jn, ln) in enumerate(zip(JARS, LIDS)):
            j = st[jn]
            _EP["travel"][i] = max(_EP["travel"][i], travel(j, st[ln]))
            tl = tilt_deg(j)
            _EP["max_tilt"][i] = max(_EP["max_tilt"][i], tl)
            z = float(j.pos[2])
            touched = pts is not None and clearance(j, pts) < L.TOUCH_M
            if not _EP["moved"][i] and not at_start(i, j):
                _EP["moved"][i] = True
            if not _EP["moved"][i]:  # the press test counts only while the jar still stands at its start place
                tr = travel(j, st[ln])
                _EP["press_travel"][i] = max(_EP["press_travel"][i], tr)
                f = lid_force(i, tr)
                _EP["force_max"][i] = max(_EP["force_max"][i], f)
                gap = tip_on_lid(j, pts) if pts is not None else None
                if gap is not None and (_EP["tip_gap"][i] is None or gap < _EP["tip_gap"][i]):
                    _EP["tip_gap"][i] = gap
                if gap is not None and gap <= L.PRESS_GAP:
                    _EP["tip_force_max"][i] = max(_EP["tip_force_max"][i], f)
                if gap is not None and gap <= L.PRESS_GAP and f >= L.PRESS_FORCE:
                    _EP["touch_run"][i] += 1
                    if _EP["touch_run"][i] >= L.PRESS_STEPS:
                        _EP["pressed"][i] = True
                else:
                    _EP["touch_run"][i] = 0
            if touched:
                _EP["free_z"][i] = None
            elif _EP["free_z"][i] is None:
                _EP["free_z"][i] = z
            if _EP["breach"] is not None:
                continue
            if pts is None:
                _EP["breach"] = (k, "gripper reading missing")
            elif tl > L.FALL_TILT:
                _EP["breach"] = (k, f"jar {i + 1} tipped past 30 degrees", round(tl, 1))
            elif z < TOP - L.OFF_TABLE:
                _EP["breach"] = (k, f"jar {i + 1} fell off the table")
            elif _EP["free_z"][i] is not None and _EP["free_z"][i] - z > L.DROP_M:
                _EP["breach"] = (k, f"jar {i + 1} dropped", round(_EP["free_z"][i] - z, 4))
            elif i == _EP["unsealed"] and not touched and in_pantry(j):
                _EP["breach"] = (k, f"unsealed jar {i + 1} let go on the pantry shelf")
    _EP["step"] += 1
    if _DBG is not None and k % 4 == 0 and _EP.get("start") is not None:
        _debug(st)


# -- scoring --------------------------------------------------------------------------------------------------------
def report(objects) -> dict:
    out: dict = {"score": 0.0}
    if _EP.get("start") is None:
        return out
    pts = _EP.get("hands")
    jars = [objects[n] for n in JARS]
    clear = min(clearance(j, pts) for j in jars) if pts is not None else -1.0
    u = _EP["unsealed"]
    where = []
    for j in jars:
        where.append("tray" if in_tray(j) else "fridge" if in_fridge(j) else "shelf" if on_shelf(j)
                     else "pantry" if in_pantry(j) else "other")
    found = _EP["press_travel"][u] >= L.PRESS_TRAVEL  # the unsealed lid pressed down 0.8 mm before it was moved
    pts_sum = 0.0
    for i, w in enumerate(where):
        if i == u and w == "tray":
            pts_sum += L.POINTS["unsealed_fridge"]
        elif i != u and w == "shelf" and _EP["pressed"][i]:
            pts_sum += L.POINTS["sealed_shelf"]
    if not found:
        pts_sum = 0.0
    out.update(unsealed_jar=u + 1, where={f"jar{i + 1}": w for i, w in enumerate(where)}, breach=_EP["breach"],
               hands_clear_m=round(clear, 3), lid_travel_mm=[round(1000 * t, 2) for t in _EP["travel"]],
               tilt=[round(tilt_deg(j), 1) for j in jars], max_tilt=[round(t, 1) for t in _EP["max_tilt"]],
               points=round(pts_sum, 2), unsealed_found_by_press=bool(found),
               press_travel_at_start_mm=[round(1000 * t, 2) for t in _EP["press_travel"]],
               lid_pressed_at_start=list(_EP["pressed"]),
               lid_force_max_at_start_n=[round(f, 2) for f in _EP["force_max"]],
               fingertip_on_lid_force_max_n=[round(f, 2) for f in _EP["tip_force_max"]],
               tip_gap_mm=[None if g is None else round(1000 * g, 1) for g in _EP["tip_gap"]])
    if _EP["breach"] is not None or clear < L.HANDS_CLEAR:
        return out
    out["score"] = round(pts_sum, 2)
    return out


def score(objects) -> float:
    """0 for the whole run if the unsealed jar's lid centre was never pressed down at least 0.8 mm while that jar still
    stood at its start place (the press test; a blind guess scores 0), or if the unsealed jar is let go on or under the
    pantry shelf, any jar tips past 30 degrees,
    falls off the table or drops more than 20 mm with no gripper on it, or a gripper reading is missing; 0 if a gripper
    is within 3 cm of any jar at the end. Else 0.4 for the unsealed jar upright in the fridge's use-first tray, all of its
    base inside the rim, plus 0.3 for each sealed jar upright with all of its base on the shelf's top board whose lid
    centre a fingertip pressed (within 15 mm of the axis, within 3 mm of the lid top, pushing 1.5 N or more, 0.3 s in a
    row) while it stood at its start place (1.0 in all)."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    u = _EP.get("unsealed") or 0
    return float(np.linalg.norm(np.asarray(objects[JARS[u]].pos, float)[:2] - np.asarray(L.FRIDGE_SPOT)))


def _debug(st) -> None:
    rep = _EP.get("report") or {}
    parts = []
    for i, (jn, ln) in enumerate(zip(JARS, LIDS)):
        j = st[jn]
        parts.append(f"j{i + 1}=({j.pos[0]:.4f},{j.pos[1]:.4f},{j.pos[2]:.4f}) t={tilt_deg(j):4.1f} "
                     f"tr={1000 * travel(j, st[ln]):.2f}")
    _DBG.write(f"{_EP['step']:4d} " + " ".join(parts) + f" breach={_EP['breach']} score={rep.get('score')}"
               f" lid_force_max_N={[round(f, 2) for f in _EP['force_max']]}"
               f" fingertip_on_lid_force_max_N={[round(f, 2) for f in _EP['tip_force_max']]}"
               f" pressed={_EP['pressed']}\n")
    _DBG.flush()


# -- the start: every object at one place; only the unsealed jar is chosen by the seed --------------------------------
@bodykit.on_reset_env
def _lay_out(emb, seed):
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    base = emb._base
    u = L.unsealed_index(seed)
    for i in range(L.N_JARS):
        if i == u:
            bodykit.set_slider_drive(f"press{i + 1}", target=L.UNSEALED_PRELOAD, stiffness=L.UNSEALED_K,
                                     damping=L.UNSEALED_DAMP)
        else:
            bodykit.set_slider_drive(f"press{i + 1}", target=0.0, stiffness=L.SEALED_K, damping=L.SEALED_DAMP)
    for o in OBJECTS:
        if o.name not in LOOSE:
            continue
        obj = base.scene[o.name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.array(o.pos) + np.array([0, 0, 0.0005]), dtype=pose.dtype,
                                   device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor((0.0, 0.0, 0.0, 1.0), dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(max(1.0, getattr(ie, "SETTLE_S", 0.5)) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    st = _scene.object_states(emb._env, NAMES)
    _EP["start"] = {n: (np.asarray(st[n].pos, float).copy(), np.asarray(st[n].quat_xyzw, float).copy()) for n in LOOSE}
    _EP["unsealed"], _EP["seed"] = u, seed
    _blank()
    if _DBG is not None:
        _DBG.write(f"start seed={seed} unsealed=jar{u + 1} " + " ".join(
            f"{n}=({v[0][0]:.4f},{v[0][1]:.4f},{v[0][2]:.4f})" for n, v in _EP["start"].items()) + "\n")
    print(f"jardemo: seed {seed}, unsealed jar {u + 1}", flush=True)
    return emb._observation()


# -- the scripted demo (reads the jars' poses and the lids' travel from the simulator to plan; moves the jars and lids
#    only through the grippers, by contact) -------------------------------------------------------------------------
VARIANT = os.environ.get("DEMO_VARIANT", "")  # "", pantry_all, fridge_only, fixed_jar, fridge_unpressed, touch_only,
# nothing
PROBE = os.environ.get("JARS_PROBE", "")  # "press": press the three lids only (M1 probe)
TIP = 0.013  # the closed pads' tips are this far past the TCP
PRESS_DEPTH = 0.020  # the fingertip target goes this far below the lid's top (the soft arm stops on a sealed lid)
GRIP_H = 0.066  # top-down grips close on the body this far above the base (pads 52 to 79 mm up)
LEVEL_H = 0.060  # level grips close at this height above the base (pads 46 to 74 mm up)
CARRY_UP = 0.045  # a jar's base is carried this far above the shelf's top (28 mm over a jar still on its scale)
TOUCH_DEPTH = 0.0005  # touch_only: the fingertip target 0.5 mm below the lid top (3 mm pushed a sealed lid with 4 N)
OPEN_HALF = L.JAR_R + 0.008  # fingers open round a jar
LET_GO_HALF = L.JAR_R + 0.004  # fingers open to let go (inside the fridge: 118 mm over the fingers, opening 135)
SINK = 0.002
LOOKS = 2


def _wp(sec, pos, f, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(float(v) for v in pos), float(f), label, np.asarray(rot, float))


def _rot(x, zt) -> np.ndarray:
    x, zt = np.asarray(x, float), np.asarray(zt, float)
    return np.column_stack([x, np.cross(zt, x), zt])


def ready_pose(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, sec: float, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return _wp(sec, p, FINGER_OPEN, label, r)


def _best(arm: str, tcp, groups) -> np.ndarray:
    """Of groups of rotations in order of preference (straightest first), the first group with a rotation the arm
    reaches within 2 mm; inside it the rotation it reaches best. If none does, the best one overall."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    best = None
    tcps = np.atleast_2d(np.asarray(tcp, float))  # one rotation for every point of a carry (the jar stays upright)
    for rots in groups:
        for r in rots:
            pe, re, q5 = 0.0, 0.0, 0.0
            for p in tcps:
                q, e, f = ik.solve(p, ik.matrix_to_quat_xyzw(r), READY, base_pos=station.BASES[arm], iters=300)
                pe, re, q5 = max(pe, e), max(re, f), max(q5, abs(float(q[5])))
            key = (round(pe + 0.05 * re, 4), q5)
            if best is None or key < best[0]:
                best = (key, r, pe)
        if best[2] < 0.002:
            break
    return best[1]


DOWN = np.array([0.0, 0.0, -1.0])
UP = np.array([0.0, 0.0, 1.0])
TILTS = (0.0, 15.0, 30.0, 40.0)


def _toward(arm: str, tcp) -> np.ndarray:
    from yam_arena import station

    v = np.atleast_2d(np.asarray(tcp, float))[0, :2] - np.asarray(station.BASES[arm], float)[:2]
    return v / max(1e-9, float(np.linalg.norm(v)))


def _down_rot(arm, tcp) -> np.ndarray:
    """Pointing down, pads closing across the line from the arm's base; tilted (its top toward the base) by up to 40
    degrees where the arm cannot reach straight down."""
    u = _toward(arm, tcp)
    side = np.array([-u[1], u[0], 0.0])
    groups = []
    for t in np.radians(TILTS):
        zt = np.array([math.sin(t) * u[0], math.sin(t) * u[1], -math.cos(t)])
        groups.append([_rot(side, zt), _rot(-side, zt), _rot([u[0], u[1], 0.0], zt), _rot([-u[0], -u[1], 0.0], zt)])
    return _best(arm, tcp, groups)


def _level_rot(arm, tcp) -> np.ndarray:
    """Pointing forward (+x), pads closing across (y); pitched down by up to 40 degrees where the arm cannot hold it
    level."""
    groups = []
    for p in np.radians(TILTS):
        zt = np.array([math.cos(p), 0.0, -math.sin(p)])
        groups.append([_rot([0, 1, 0], zt), _rot([0, -1, 0], zt)])
    return _best(arm, tcp, groups)


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


def _pad(plans: dict) -> dict:
    """Make both arms' plans equally long, the shorter one keeping its last pose."""
    t = max(_dur(ws) for ws in plans.values())
    for arm, ws in plans.items():
        gap = t - _dur(ws)
        if gap > 1e-9:
            last = ws[-1] if ws else park(arm, 0.1, "wait")
            ws.append(Waypoint(gap, last.pos, last.finger, "wait", last.yaw))
    return plans


def press_test(arm: str, base, depth: float = PRESS_DEPTH) -> list:
    """Closed fingertips straight down on the lid's centre (target ``depth`` below its top), held, then lifted off."""
    lid = np.asarray(base, float) + np.array([0.0, 0.0, L.LID_TOP])
    touch = lid + (TIP + 0.006) * UP
    rot = _down_rot(arm, [touch, lid + (TIP + 0.035) * UP, lid + (TIP - depth) * UP])
    return [_wp(1.6, lid + (TIP + 0.035) * UP, 0.0, "over the lid", rot),
            _wp(1.0, touch, 0.0, "line up over the lid", rot),
            _wp(0.4, touch, 0.0, "line up over the lid", rot),
            _wp(0.8, lid + (TIP - depth) * UP, 0.0, "press the lid centre", rot),
            _wp(0.5, lid + (TIP - depth) * UP, 0.0, "hold the press", rot),
            _wp(0.4, touch, 0.0, "press released", rot),
            _wp(0.6, lid + (TIP + 0.035) * UP, 0.0, "lift off the lid", rot)]


def to_shelf(arm: str, base, spot) -> list:
    """Top-down grip on the jar's body, lifted clear of the other jars and the shelf, stood on the shelf, let go."""
    base = np.asarray(base, float)
    tcp = base + GRIP_H * UP
    lifted = np.array([base[0], base[1], L.SHELF_TOP + CARRY_UP + GRIP_H])
    over = np.array([spot[0], spot[1], lifted[2]])
    place = np.array([spot[0], spot[1], L.SHELF_TOP - SINK + GRIP_H])
    rot = _down_rot(arm, [tcp, lifted, over, place])
    return [_wp(2.0, lifted, -OPEN_HALF, "over the jar", rot),
            _wp(1.0, tcp + 0.06 * UP, -OPEN_HALF, "down to the jar", rot),
            _wp(1.2, tcp, -OPEN_HALF, "down round the jar", rot),
            _wp(0.7, tcp, 0.0, "grip the jar", rot),
            _wp(0.3, tcp, 0.0, "grip the jar", rot),
            _wp(1.6, lifted, 0.0, "lift the jar upright", rot),
            _wp(2.2, over, 0.0, "carry it over the shelf", rot),
            _wp(1.5, place, 0.0, "stand it on the shelf", rot),
            _wp(0.4, place, 0.0, "stand it on the shelf", rot),
            _wp(0.6, place, -OPEN_HALF, "let go of the jar", rot),
            _wp(1.4, place + 0.100 * UP, -OPEN_HALF, "back off up", rot)]


def to_fridge(arm: str, base) -> list:
    """Level grip from the front, lifted just above the fridge floor, carried in through the opening, stood on the
    floor, let go, and the hand backed straight out."""
    base = np.asarray(base, float)
    tcp = base + LEVEL_H * UP
    behind = tcp + np.array([-0.080, 0.0, 0.0])
    lift_z = L.FRIDGE_FLOOR_TOP + 0.040 + LEVEL_H  # 40 mm over the floor (32 over the tray's rim): the arm sags under it
    lifted = np.array([tcp[0], tcp[1], lift_z])
    door = np.array([L.FRIDGE_FRONT_X - 0.070, L.FRIDGE_Y, lift_z])
    inside = np.array([L.FRIDGE_SPOT[0], L.FRIDGE_SPOT[1], lift_z])
    place = np.array([inside[0], inside[1], L.TRAY_FLOOR_TOP - SINK + LEVEL_H])
    out = np.array([L.FRIDGE_FRONT_X - 0.065, L.FRIDGE_Y, place[2] + 0.015])
    rot = _level_rot(arm, [tcp, lifted, door, inside, place])
    return [_wp(2.2, behind + 0.04 * UP, -OPEN_HALF, "behind the jar", rot),
            _wp(0.8, behind, -OPEN_HALF, "behind the jar", rot),
            _wp(1.3, tcp, -OPEN_HALF, "round the jar", rot),
            _wp(0.7, tcp, 0.0, "grip the jar", rot),
            _wp(0.3, tcp, 0.0, "grip the jar", rot),
            _wp(2.2, lifted, 0.0, "lift the jar upright", rot),  # slow, so the sag correction keeps up
            _wp(0.4, lifted, 0.0, "lift the jar upright", rot),
            _wp(3.0, door, 0.0, "carry it to the fridge door", rot),
            _wp(2.0, inside, 0.0, "carry it into the fridge", rot),
            _wp(1.2, place, 0.0, "stand it in the fridge tray", rot),
            _wp(0.4, place, 0.0, "stand it in the fridge tray", rot),
            _wp(0.6, place, -LET_GO_HALF, "let go of the jar", rot),
            _wp(1.6, out, -LET_GO_HALF, "back off out of the fridge", rot)]


def stage_press(start) -> dict:
    """The left hand presses jar 1 while the right hand presses jar 3; once the left hand is parked, the right hand
    presses jar 2."""
    def depth(i):  # touch_only: the sealed lids only touched; the unsealed one (read from the simulator) pressed
        return TOUCH_DEPTH if VARIANT == "touch_only" and i != _EP["unsealed"] else PRESS_DEPTH

    left = press_test("left", start[JARS[0]][0], depth(0)) + [park("left", 1.6)]
    right = press_test("right", start[JARS[2]][0], depth(2))
    right.append(park("right", max(0.1, _dur(left) - _dur(right)), "wait"))
    right += press_test("right", start[JARS[1]][0], depth(1)) + [park("right", 1.6)]
    return _pad({"right": right, "left": left})


def decide() -> int | None:
    """The jar whose lid went down more than 0.8 mm under the press (read from the simulator)."""
    tr = _EP["travel"]
    i = int(np.argmax(tr))
    print("jardemo: lid travel under the press (mm) " + ", ".join(f"jar {k + 1}: {1000 * t:.2f}" for k, t in
                                                               enumerate(tr)), flush=True)
    return i if tr[i] > L.UNSEALED_TRAVEL else None


def stage_move(now) -> dict:
    """The left hand stands the sealed jars on the shelf, one at a time; then the right hand carries the unsealed jar
    into the fridge. Each arm waits parked while the other works."""
    pos = {jn: np.asarray(now[jn].pos, float) for jn in JARS}
    if VARIANT == "pantry_all":
        shelf, fridge = [0, 1, 2], None
    elif VARIANT == "fixed_jar":
        shelf, fridge = [0, 2], 1
    elif VARIANT == "fridge_unpressed":  # no press: the unsealed jar (read from the simulator) to the fridge
        u = _EP["unsealed"]
        shelf, fridge = [i for i in range(L.N_JARS) if i != u], u
    else:
        u = decide()
        print(f"jardemo: unsealed by the press test: jar {None if u is None else u + 1}", flush=True)
        fridge = u
        shelf = [] if VARIANT == "fridge_only" else [i for i in range(L.N_JARS) if i != u]
    left: list = []
    for k, i in enumerate(shelf):
        left += to_shelf("left", pos[JARS[i]], L.SHELF_SPOTS[k])
    if left:
        left.append(park("left", 2.0))
    right: list = []
    if fridge is not None:
        right = [park("right", max(_dur(left), 0.1), "wait")] + to_fridge("right", pos[JARS[fridge]])
        right.append(park("right", 2.4))
    if not left:
        left = [park("left", 0.5, "wait")]
    if not right:
        right = [park("right", 0.5, "wait")]
    return _pad({"left": left, "right": right})


def plan_stage(stage: int, img, module) -> dict:
    if VARIANT == "nothing":
        return {}
    if stage == 0:
        start = _EP["start"]
        print(f"jardemo: variant '{VARIANT or 'full'}' probe '{PROBE}'", flush=True)
        if VARIANT in ("pantry_all", "fixed_jar", "fridge_unpressed"):
            return {arm: [park(arm, 0.5, "wait")] for arm in ("left", "right")}
        return stage_press(start)
    if PROBE == "press":
        decide()
        return {}
    return stage_move(_EP["now"])


def placeholder_plans() -> dict:
    return {arm: [park(arm, 1.0, "look")] for arm in ("left", "right")}


def demo():
    return placeholder_plans()


import jardemo  # noqa: E402  (the demo's control hooks for yam_scripted)

jardemo.install()
jardemo_task = True
