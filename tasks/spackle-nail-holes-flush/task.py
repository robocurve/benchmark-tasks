"""On the station table, in front of the arms: an offcut of half-inch drywall stands in a bench stand, leaning back 20
degrees, its paper face toward the robot. Three holes are in it, the sizes nails, a picture hook and a plastic anchor leave:
3 mm across and 6 mm deep, 4 mm by 8 mm and 6 mm by 10 mm. To the right, an open half-pint tub of pink spackle stands
in a holder, and a 1.5 in. putty knife lies on the bench in front of the panel.

The job is to patch the holes so they are ready to paint: each one packed full of spackle and struck off flush with the
face, with no ridge or coat of spackle left standing on the wall, and the knife set down afterward.

The paste is pastekit's height field on the panel's face (1 mm cells). A held knife sliding with its edge on the face
works the cells its edge crosses; its angle to the face sets what it leaves: a low blade (40 degrees or less; 35 in
the demo) packs the holes and leaves a coat 0.8 mm thick, a steep blade (65 degrees in the demo) bridges the holes and
scrapes the coat back onto the knife, leaving 0.1 mm. Only a blade steeper than about 54 degrees leaves 0.3 mm or
less, so packing and striking off are at least 14 degrees apart: two near-identical strokes can't do both. So packing then striking off fills a hole flush; a steep pass alone leaves
the holes open; a low pass alone leaves a coat standing proud. Every threshold below appears once, with the same
value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import guardkit as gk  # noqa: E402
import pastekit as pk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Patch the nail holes in the wall panel with the putty knife so they're ready to paint."

# -- the panel (world frame: x away from the arms, y to the left, z up; m) --------------------------------------------
LEAN = math.radians(20.0)  # the panel leans back 20 degrees from vertical in its stand
PANEL_T = 0.0127  # half-inch drywall
PANEL_W = 0.36  # along y
PANEL_H = 0.28  # up the face
PANEL_YC = -0.04
FACE_X0 = 0.370  # x of the face's bottom edge
FACE_Z0 = TOP + PANEL_T * math.sin(LEAN)  # the bottom edge's back corner rests on the table
UP = np.array([math.sin(LEAN), 0.0, math.cos(LEAN)])  # up the face
NORMAL = np.array([-math.cos(LEAN), 0.0, math.sin(LEAN)])  # out of the face, toward the robot
R_PANEL = np.array([[math.cos(LEAN), 0.0, math.sin(LEAN)], [0.0, 1.0, 0.0], [-math.sin(LEAN), 0.0, math.cos(LEAN)]])
PANEL_POS = np.array([FACE_X0, PANEL_YC, FACE_Z0]) + R_PANEL @ np.array([PANEL_T / 2, 0.0, PANEL_H / 2])
PAPER_RGB = (0.90, 0.89, 0.86)  # drywall's face paper
HOLE_RGB = (0.25, 0.24, 0.23)  # the dark of an open hole

# Each hole: (name, height up the face from its bottom edge, y, diameter, depth); drilled to depth in the real build.
HOLE_SPECS = (("finish nail", 0.160, -0.150, 0.003, 0.006),
              ("picture-hook nail", 0.165, -0.085, 0.004, 0.008),
              ("pulled plastic anchor", 0.170, -0.020, 0.006, 0.010))
FACE_V0 = PANEL_YC - PANEL_W / 2  # y of the face's right edge (surface v = 0)


def face_point(s: float, y: float, d: float = 0.0) -> np.ndarray:
    """World point `s` m up the face from its bottom edge, at `y`, `d` m out from the face (negative: into it)."""
    return np.array([FACE_X0, y, FACE_Z0]) + s * UP + d * NORMAL


PANEL_LOOK = kit.Look(PAPER_RGB, 0.9)
PANEL_FRICTION = (0.3, 0.25)  # static, dynamic: a steel blade on drywall paper wet with spackle
PANEL = kit.Prop(
    "panel", pos=tuple(PANEL_POS), size=(PANEL_T, PANEL_W, PANEL_H),
    tilt_xyzw=(0.0, math.sin(LEAN / 2), 0.0, math.cos(LEAN / 2)),
    meshes=(kit.box((0, 0, 0), (PANEL_T, PANEL_W, PANEL_H), PANEL_LOOK),
            *(kit.cylinder((-PANEL_T / 2 - 0.0001, y - PANEL_YC, s - PANEL_H / 2), dia / 2, 0.0004,
                           kit.Look(HOLE_RGB, 0.9), axis="x") for _, s, y, dia, _ in HOLE_SPECS)),
    colliders=(kit.Collider((0, 0, 0), size=(PANEL_T, PANEL_W, PANEL_H)),), friction=PANEL_FRICTION)

# The stand: a pine 1x2 stop strip in front of the bottom edge, and a 2x2 brace behind (sim: fixed, like the bench
# stand's screws).
PINE = (0.80, 0.66, 0.46)
STOP_SIZE = (0.019, PANEL_W, 0.038)  # a 1x2 on edge: 19 x 38 mm
STOP = Box("stop", pos=(FACE_X0 - STOP_SIZE[0] / 2 - 0.001, PANEL_YC, TOP + STOP_SIZE[2] / 2), size=STOP_SIZE,
           color=PINE)
_BRACE_TOP = face_point(0.22, PANEL_YC, -PANEL_T - 0.019)
_BRACE_BOT = np.array([_BRACE_TOP[0] + 0.12, PANEL_YC, TOP + 0.019])
_BRACE_LEN = float(np.linalg.norm(_BRACE_TOP - _BRACE_BOT))
_BRACE_ANG = math.atan2(_BRACE_TOP[2] - _BRACE_BOT[2], _BRACE_BOT[0] - _BRACE_TOP[0])
BRACE = kit.Prop(
    "brace", pos=tuple((_BRACE_TOP + _BRACE_BOT) / 2), size=(_BRACE_LEN, 0.038, 0.038),
    tilt_xyzw=(0.0, math.sin(_BRACE_ANG / 2), 0.0, math.cos(_BRACE_ANG / 2)),
    meshes=(kit.box((0, 0, 0), (_BRACE_LEN, 0.038, 0.038), kit.Look(PINE, 0.7)),),
    colliders=(kit.Collider((0, 0, 0), size=(_BRACE_LEN, 0.038, 0.038)),))

# -- the spackle tub (fixed in a plywood holder ring) ------------------------------------------------------------------
TUB_C = (0.300, -0.220)
TUB_ID, TUB_H, TUB_WALL = 0.090, 0.044, 0.0025  # a half-pint tub: 9.0 cm inside, 4.4 cm tall
PASTE_Z = 0.039  # the paste's surface above the table: 3.7 cm of paste (235 mL, 8 fl oz) on a 2 mm floor, 5 mm below the rim
SPACKLE_RGB = (0.95, 0.55, 0.70)  # pink spackle (the kind that goes on pink and dries white)
_TUB_LOOK = kit.Look((0.96, 0.96, 0.95), 0.5)
_N_WALL = 16
TUB = kit.Prop(
    "tub", pos=(TUB_C[0], TUB_C[1], TOP), size=(TUB_ID + 2 * TUB_WALL, TUB_ID + 2 * TUB_WALL, TUB_H),
    meshes=(kit.lathe([(TUB_ID / 2, 0.002), (TUB_ID / 2 + TUB_WALL, 0.0), (TUB_ID / 2 + TUB_WALL, TUB_H),
                       (TUB_ID / 2, TUB_H), (TUB_ID / 2, 0.002)], _TUB_LOOK),
            kit.cylinder((0, 0, PASTE_Z / 2), TUB_ID / 2 - 0.0002, PASTE_Z, kit.Look(SPACKLE_RGB, 0.8))),
    colliders=(kit.Collider((0, 0, 0.001), size=(TUB_ID, TUB_ID, 0.002)),
               *(kit.Collider(((TUB_ID / 2 + TUB_WALL / 2) * math.cos(2 * math.pi * i / _N_WALL),
                               (TUB_ID / 2 + TUB_WALL / 2) * math.sin(2 * math.pi * i / _N_WALL), TUB_H / 2),
                              radius=TUB_WALL, height=TUB_H) for i in range(_N_WALL))))

# -- the putty knife (knife frame: origin at the blade's root, +x along the blade to its edge, y across, z up) ----------
BLADE_LEN, BLADE_W = 0.089, 0.038  # 3.5 in. of blade, 1.5 in. wide
BLADE_T = 0.001  # collider; the steel is 0.6 mm
HANDLE = ((-0.059, 0.0, 0.0), (0.098, 0.028, 0.018))  # center, size: a 3.9 in. plastic handle, 28 x 18 mm
FERRULE = ((-0.005, 0.0, 0.0), (0.010, 0.030, 0.012))
BLADE_Z = -HANDLE[1][2] / 2 + BLADE_T / 2  # the blade's mid-plane: flush with the handle's flat underside
BLADE_BOX = ((BLADE_LEN / 2, 0.0, BLADE_Z), (BLADE_LEN, BLADE_W, BLADE_T))
KNIFE_MASS = 0.070  # kg
GRIP_LOCAL = np.array([-0.035, 0.0, 0.0])  # where the demo holds the handle, just behind the ferrule
KNIFE_START = (0.270, -0.065, TOP + HANDLE[1][2] / 2)  # lying flat on the bench, blade toward +y (left)
KNIFE_YAW = math.pi / 2
STEEL = kit.Look((0.74, 0.75, 0.77), 0.3, metallic=0.9)
KNIFE = kit.Prop(
    "knife", pos=KNIFE_START, yaw=KNIFE_YAW, mass=KNIFE_MASS, size=(0.197, BLADE_W, HANDLE[1][2]),
    meshes=(kit.box(*HANDLE, kit.Look((0.95, 0.75, 0.10), 0.5)), kit.box(*FERRULE, STEEL),
            kit.box(BLADE_BOX[0], (BLADE_LEN, BLADE_W, 0.0006), STEEL)),
    colliders=(kit.Collider(*HANDLE), kit.Collider(*FERRULE), kit.Collider(*BLADE_BOX)))
KNIFE_PARTS = (gk.Block(*HANDLE), gk.Block(*FERRULE))  # what a hand holds

OBJECTS = [PANEL, STOP, BRACE, TUB, KNIFE]

# -- the paste model (pastekit) and thresholds (each with its reason in the README and task.json) -----------------------
# contact 0.6 mm; slide 1.5 mm in 0.15 s; coat 0.8 mm at <= 40 deg, 0.1 mm at >= 60 deg, linear between (0.3 mm at
# 54.3 deg); packs only at <= 40 deg, where the coat is 0.8 mm. So no packing angle leaves a patch within the 0.3 mm
# proud limit, and the strike-off must be at least 14 degrees steeper than the packing pass.
LAW = pk.Law(low_deg=40.0, pack_max_deg=40.0)
BLADE = pk.Blade(edge_x=BLADE_LEN, width=BLADE_W, thick=BLADE_T, z=BLADE_Z)
HOLES = tuple(pk.Recess(s, y - FACE_V0, dia / 2, depth, name) for name, s, y, dia, depth in HOLE_SPECS)
FACE = pk.Surface("face", "panel", origin=(-PANEL_T / 2, -PANEL_W / 2, -PANEL_H / 2), u=(0, 0, 1), v=(0, 1, 0),
                  size=(PANEL_H, PANEL_W), cell=0.001, recesses=HOLES)
POT = pk.Pot("spackle", "tub", center=(0.0, 0.0, PASTE_Z), radius=TUB_ID / 2 - 0.002, depth=0.02)
FLUSH = 0.0003  # m: a hole is filled when its deepest point is at most 0.3 mm below the face ...
PROUD = 0.0003  # m: ... and spackle more than 0.3 mm above the face is a ridge or coat left on ...
PROUD_AREA = 10e-6  # m^2: ... allowed over at most 10 mm^2 in all (about a 3 x 3 mm speck)
SET_DOWN_Z = TOP + 0.06  # the knife's origin below this: lying on the bench or across the tub
PANEL_CLEAR = 0.002  # m: every corner of the knife's handle, ferrule and blade at least this far in front of the face
GOUGE_CAP = 0.5  # a corner gouge (a proxy for torn face paper) caps the score at this
STILL_STEP = 0.0005  # m per control step (1 cm/s) ...
STILL_TURN = math.radians(0.5)  # ... and under 0.5 degree a step
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps = 1 s
GRIPPERS = ("left_gripper", "right_gripper")


def _pose(o):
    return np.asarray(o.pos, float), np.asarray(o.quat_xyzw, float)


FIXED_POSES = {"panel": (PANEL_POS, pk.quat(R_PANEL)), "tub": (np.array([TUB_C[0], TUB_C[1], TOP]), np.array([0, 0, 0, 1.0]))}


_KNIFE_CORNERS = np.array([np.asarray(c) + np.asarray(sz) / 2 * np.array([sx, sy, sz_])
                           for c, sz in (HANDLE, FERRULE, BLADE_BOX)
                           for sx in (-1, 1) for sy in (-1, 1) for sz_ in (-1, 1)])


def knife_clear_of_panel(kp, kq) -> bool:
    """Every corner of the knife's handle, ferrule and blade is at least PANEL_CLEAR in front of the panel's face
    (so no part of it rests on the wall, not just its edge)."""
    pts = _KNIFE_CORNERS @ pk.rotm(kq).T + kp
    o = face_point(0.0, PANEL_YC)
    return bool(np.min((pts - o) @ NORMAL) >= PANEL_CLEAR)


def knife_held(objects) -> bool:
    """A gripper's own shape (palm and fingers, opened as they are) touches the knife's handle or ferrule (guardkit's
    contact check, within 2 mm): contact, not nearness."""
    return bool(gk.touching(objects, "knife", KNIFE_PARTS, grippers=GRIPPERS))


class Ledger:
    """One trial's paste and history, advanced once per control step (success() and score() share it)."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.paste = pk.Paste([FACE], [POT], BLADE, LAW)
        self.last = None
        self.good = 0  # consecutive steps at the 0.75 state with the knife set down

    def depths(self):
        return [self.paste.remaining_depth("face", i) for i in range(len(HOLES))]

    def filled(self) -> list:
        return [d <= FLUSH + 1e-12 for d in self.depths()]

    def proud_area(self) -> float:
        return self.paste.proud_area("face", PROUD)

    def tier(self) -> float:
        """0.25 per hole filled; capped at GOUGE_CAP if the corner gouged the face (a proxy for torn paper)."""
        t = (0.0, 0.25, 0.5, 0.75)[sum(self.filled())]
        return min(t, GOUGE_CAP) if self.paste.gouged("face") else t


_L = Ledger()


def _update(objects):
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    held = knife_held(objects)
    poses = FIXED_POSES  # the panel and tub are fixed: their known poses (no reliance on a tilted prop's reported quat)
    kp, kq = _pose(objects["knife"])
    _L.paste.step(kp, kq, held, poses)
    r = pk.rotm(kq)
    still = False
    if _L.last is not None:
        dp = float(np.linalg.norm(kp - _L.last[0]))
        dr = math.acos(max(-1.0, min(1.0, (float(np.trace(_L.last[1].T @ r)) - 1) / 2)))
        still = dp <= STILL_STEP and dr <= STILL_TURN
    _L.last = (kp, r)
    down = (not held and still and float(kp[2]) <= SET_DOWN_Z and not _L.paste.touching("face")
            and knife_clear_of_panel(kp, kq))
    ok = (_L.tier() == 0.75 and not _L.paste.gouged("face") and _L.proud_area() <= PROUD_AREA + 1e-15 and down)
    _L.good = _L.good + 1 if ok else 0
    _TILES.draw(objects, _L.paste, poses)


def success(objects):
    """All three holes filled flush, nothing proud, the knife set down, released and still: this step counts toward
    the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """0.25 per hole filled (its deepest point at most 0.3 mm below the face): 0.25, 0.5, 0.75; and 1.0 when all
    three are filled, no spackle stands more than 0.3 mm proud of the face over more than 10 mm^2 in all, and the knife
    has been set down, released and still for 1 s, every part of it at least 2 mm clear of the panel's face. A corner
    gouge (the edge turned more than 15 degrees, corner dragged 5 mm or more) caps the score at 0.5. Doing nothing
    scores 0."""
    _update(objects)
    s = _L.tier()
    if s == 0.75 and _L.good >= HOLD_STEPS:
        s = 1.0
    return s


# -- the spackle the cameras see ----------------------------------------------------------------------------------------
class _Tiles:
    """pastekit.Tiles on the panel's face, placed in the env's frame (found from the panel's prim once)."""

    def __init__(self):
        self.tiles, self.origin = pk.Tiles(k=2), None

    def draw(self, objects, paste, poses):
        if self.tiles.off:
            return
        if self.origin is None:
            try:
                import omni.usd
                from pxr import UsdGeom

                stage, cache = omni.usd.get_context().get_stage(), UsdGeom.XformCache()
                self.origin = np.zeros(3)
                for prim in stage.Traverse():
                    if prim.GetName() == "panel":
                        w = np.array(cache.GetLocalToWorldTransform(prim).ExtractTranslation(), float)
                        self.origin = w - np.asarray(objects["panel"].pos, float)
                        break
            except Exception:  # noqa: BLE001 (no simulator: unit tests)
                self.tiles.off = True
                return
        self.tiles.draw(paste, "face", paste._frame(poses, FACE), self.origin)


_TILES = _Tiles()

# -- scripted demo: open loop from the fixed layout ---------------------------------------------------------------------
# The right hand takes the knife by the handle from above (fingers across its 28 mm width), turns it to point at the
# panel, dips its tip into the tub (blade tipped 15 degrees down) and draws it 4 cm through the paste to load it. For
# each hole it packs with a low pass (blade at 35 degrees to the face, drawn down across the hole, handle leading),
# backs off, then strikes off with two steep passes (65 degrees: the knife tipped 5 degrees nose up, as the face leans
# back 20 degrees) that start above and end below the coat, one STRIKE_SHIFT to each side of the packing pass, so the
# two overlap it on both sides (the hand sits a mm or two differently at 35 and 65 degrees). Then it lays the knife
# back where it was and lets go.
# Holding the knife at the panel, the hand settles about 4 degrees turned about the blade's normal from where it is
# sent, which leaves only one end of the edge on the face; the passes aim the knife EDGE_YAW the other way so the whole
# edge lies on it.
FILL_DEG, STRIKE_DEG = 35.0, 65.0  # blade-to-face angles of the demo's two passes
EDGE_YAW = math.radians(-4.3)  # the knife's turn about its blade's normal during the passes
DIP_PITCH = math.radians(15.0)  # tip 15 degrees down into the tub
PRESS = (0.006, 0.004)  # m: the edge is aimed this far into the face when packing and striking off (the collider stops it)
OFF = 0.02  # m out from the face before and after a pass
BACK = 0.06  # m out from the face while the knife turns between passes
FILL_SPAN = (0.012, -0.010)  # m up the face from the hole's center: the packing pass ...
STRIKE_SPAN = (0.024, -0.022)  # ... and the strike-off, which overruns the coat at both ends
STRIKE_SHIFT = 0.003  # m sideways (along the edge) of each of the two strike-off passes from the packing pass's line
SPEED = 0.015  # m/s along the face
DIP_DEPTH = 0.004  # m the tip goes under the paste's surface
DIP_SPAN = (-0.005, 0.035)  # m along x from the tub's center
LIFT = 0.10  # m above the bench when carrying
TURN_X = 0.23  # m: where the knife is turned to point at the panel, so its 12.4 cm reach clears the panel


def knife_rot(pitch: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    """The knife's rotation when the hand that took it at rest (grip yaw KNIFE_YAW) turns to `yaw` and tilts by
    `pitch` about world y (negative: nose up, for a knife pointing +x)."""
    return kit._rot_xyz(0.0, pitch) @ pk_rz(yaw - KNIFE_YAW) @ pk_rz(KNIFE_YAW)


def pk_rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pitch_for(theta_deg: float) -> float:
    """Grip pitch that sets the blade at theta_deg to the face (knife pointing +x, nose up)."""
    return -math.radians(90.0 - theta_deg - math.degrees(LEAN))


EDGE_LOCAL = np.array([BLADE_LEN, 0.0, BLADE_Z])


def tcp_for_edge(edge, pitch: float, yaw: float = 0.0) -> tuple:
    """The tool point that puts the middle of the knife's edge at `edge`."""
    r = knife_rot(pitch, yaw)
    return tuple(float(v) for v in np.asarray(edge, float) - r @ (EDGE_LOCAL - GRIP_LOCAL))


def _pass(plan, s0, s1, y, theta, press, label):
    p, yw = pitch_for(theta), EDGE_YAW
    T = lambda d, s: tcp_for_edge(face_point(s, y, d), p, yw)  # noqa: E731
    plan += [kit.Tilted(2.0, T(BACK, s0), FINGER_CLOSED, f"square up: {label}", yaw=yw, pitch=p),
             kit.Tilted(1.0, T(OFF, s0), FINGER_CLOSED, f"in close: {label}", yaw=yw, pitch=p),
             kit.Tilted(0.8, T(-press, s0), FINGER_CLOSED, f"edge on: {label}", yaw=yw, pitch=p),
             kit.Tilted(abs(s0 - s1) / SPEED, T(-press, s1), FINGER_CLOSED, label, yaw=yw, pitch=p),
             kit.Tilted(0.8, T(OFF, s1), FINGER_CLOSED, f"off: {label}", yaw=yw, pitch=p),
             kit.Tilted(1.0, T(BACK, s1), FINGER_CLOSED, f"back: {label}", yaw=yw, pitch=p)]


def demo():
    """Right hand: take the knife, load it in the tub, pack and strike off each hole, lay the knife back down."""
    g0 = np.asarray(KNIFE_START) + knife_rot(0.0, KNIFE_YAW) @ GRIP_LOCAL
    gx, gy, gz = (float(v) for v in g0)
    plan = [Waypoint(2.0, (gx, gy, gz + 0.08), FINGER_OPEN, "above the handle", KNIFE_YAW),
            Waypoint(1.5, (gx, gy, gz), FINGER_OPEN, "around the handle", KNIFE_YAW),
            Waypoint(0.8, (gx, gy, gz), FINGER_CLOSED, "grip", KNIFE_YAW),
            Waypoint(1.2, (gx, gy, TOP + LIFT), FINGER_CLOSED, "lift", KNIFE_YAW),
            Waypoint(1.0, (TURN_X, gy, TOP + LIFT), FINGER_CLOSED, "back from the panel", KNIFE_YAW),
            Waypoint(1.5, (TURN_X, gy, TOP + LIFT), FINGER_CLOSED, "point the blade at the panel", 0.0)]
    tub = np.array([TUB_C[0], TUB_C[1], TOP + PASTE_Z])
    d0 = tub + [DIP_SPAN[0], 0.0, -DIP_DEPTH]
    d1 = tub + [DIP_SPAN[1], 0.0, -DIP_DEPTH]
    above = lambda p: tcp_for_edge(np.asarray(p) + [0.0, 0.0, 0.05], DIP_PITCH)  # noqa: E731
    plan += [kit.Tilted(2.0, above(d0), FINGER_CLOSED, "over the tub", pitch=DIP_PITCH),
             kit.Tilted(1.2, tcp_for_edge(d0, DIP_PITCH), FINGER_CLOSED, "tip into the spackle", pitch=DIP_PITCH),
             kit.Tilted((DIP_SPAN[1] - DIP_SPAN[0]) / 0.02, tcp_for_edge(d1, DIP_PITCH), FINGER_CLOSED,
                        "draw through the spackle", pitch=DIP_PITCH),
             kit.Tilted(1.2, above(d1), FINGER_CLOSED, "out of the tub", pitch=DIP_PITCH)]
    for name, s, y, _, _ in HOLE_SPECS:
        _pass(plan, s + FILL_SPAN[0], s + FILL_SPAN[1], y, FILL_DEG, PRESS[0], f"pack the {name} hole")
        for side, dy in (("right", -STRIKE_SHIFT), ("left", STRIKE_SHIFT)):
            _pass(plan, s + STRIKE_SPAN[0], s + STRIKE_SPAN[1], y + dy, STRIKE_DEG, PRESS[1],
                  f"strike off the {name} hole ({side} side)")
    plan += [Waypoint(2.0, (TURN_X, gy, TOP + LIFT), FINGER_CLOSED, "back over the bench", 0.0),
             Waypoint(1.5, (TURN_X, gy, TOP + LIFT), FINGER_CLOSED, "turn the knife back", KNIFE_YAW),
             Waypoint(1.0, (gx, gy, TOP + LIFT), FINGER_CLOSED, "over its place", KNIFE_YAW),
             Waypoint(1.5, (gx, gy, gz + 0.001), FINGER_CLOSED, "lay it down", KNIFE_YAW),
             Waypoint(0.8, (gx, gy, gz + 0.001), FINGER_OPEN, "let go", KNIFE_YAW),
             Waypoint(1.2, (gx, gy, gz + 0.08), FINGER_OPEN, "lift clear", KNIFE_YAW),
             Waypoint(2.0, (gx, gy, gz + 0.08), FINGER_OPEN, "wait", KNIFE_YAW)]
    return {"right": plan}
