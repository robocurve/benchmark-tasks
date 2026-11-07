"""fingerkit: the spare soft gripper, its fin-ray fingers, fork plates, tray and bin for this task file, on yam_arena.

A `yam_arena` task file declares rigid primitives. This task's objects are textured meshes with simple colliders, so `FingerPart` subclasses `Box` so yam_arena's task-file
checks accept it, and importing this module hooks `yam_arena.scene` (it modifies nothing on disk) so each part spawns from a USD file written at start-up and `object_states`
reports the task's state.

The gripper is the published UMI gripper, from its printed parts' meshes in umi_parts/ (see import_umi.py), with M3 screws, nuts and a linear rail added at its real holes. On a real one three screws through a fork hold each soft finger. A straight-down grip cannot turn a screw, so here the worn finger's three screws start out, standing in a block; they
drop into the fork's upper plate, which rests loose on the finger, and are not threaded.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import json
import math
import os
import pathlib
import sys
import tempfile
import time
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, ObjectState

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from usdkit import Look, Mesh, box, collider, cylinder, hull_collider, join, prism, write_usda  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HERE = pathlib.Path(__file__).resolve().parent
GRIPPER_XY = (0.285, 0.0)  # where the fingers are pinched lies on this line, either side of the middle
ROOT_Y = 0.04  # each finger's root from the middle: the real gripper's rack over its gear
BED_Z = 0.04  # the cradle's top, under a finger
FIN_T = 0.0258  # a finger's thickness, lying flat, from its mesh
FIN_HALF_Y = 0.02
FIN_BACK = 0.0137  # the finger's far end, past the point where it is pinched
FIN_TIP = 0.1097  # its tip, the other way
FIN_MIDDLE = 0.048
FIN_MASS = 0.056  # 47 cm3 of TPU
Z_MID = BED_Z + FIN_T / 2  # the fork's middle plane
PINCH_Y_IN_FINGER = 0.048  # along the finger's mesh
HOLDER_X = 0.00802 + 0.0024  # along x, a holder's origin past the pinch point; the screws' holes line up, less 2.4 mm so that a finger clears the web going in
CLEAR = 0.0035  # between a finger and its pocket's walls, each side
LIP_H = 0.005  # the pocket's low walls: a pinch's fingertips pass above them
LIP_T = 0.004
CLAMP = (0.0205, 0.04, 0.0059)  # the part of the upper fork plate that is square: along x, across, thick
CLAMP_X = -0.00375 + HOLDER_X  # its centre, past the pinch point
CLAMP_PINCH_X = 0.001 + HOLDER_X  # where its sides stay parallel
CLAMP_MASS = 0.03
WEB_X = 0.00675 + HOLDER_X  # the web joining the holder's two fork plates: it stops a finger
PLATE_X = 0.05  # the body's front edge past the pinch point: far enough that a hand on the fork clears the body's mirror wings
PLATE_Z = Z_MID - 0.00115  # the bottom plate's underside
COVER_Z = PLATE_Z + 0.0263  # the top cover's underside
FIXTURE = (-0.13, 0.155, 0.1, 0.012)  # the steel plate: from x, to x, half across, thick
TRAY_LIP = 0.005
TRAY_FLOOR = 0.006
NEW_XY = (0.44, 0.185)  # where each spare is pinched, in the tray
USED_XY = (0.44, 0.275)
TRAY_XY = (0.392, 0.23)
TRAY_SIZE = (0.148, 0.155, TRAY_FLOOR)
BIN_XY = (0.22, 0.30)
BIN_SIZE = (0.16, 0.16, 0.06)
BIN_WALL = 0.004
SCREW_LEN = 0.035  # an M3 x 35 screw
SCREW_HEAD = (0.008, 0.008, 0.0035)  # its head and washer, as a collider: 8 mm across, 1 mm wider each side than the 5.5 mm head so that it stops on the plate
SCREW_STUB = (0.003, 0.003, 0.0055)  # the part of its shank that collides: the rest would have to pass through the finger
SCREW_MASS = 0.004
HOLE_HALF = 0.003  # a hole's half width as a collider: the real pocket's size
SCREW_BLOCK_XY = (0.215, 0.145)
SCREW_BLOCK = (0.08, 0.03, 0.036)
SCREW_BLOCK_HOLES = (-0.024, 0.0, 0.024)  # along x from its centre
SPAWN_GAP = 0.0005
ROBOTS = ("left_robot", "right_robot")
TIP_IN_GRIPPER = 0.147

STEEL = dict(texture="steel_color.jpg", rough_map="steel_rough.jpg", normal_map="steel_normal.jpg", tile=0.12, metallic=1.0)
LOOKS = {
    "steel": Look(color=(0.85, 0.85, 0.87), **{**STEEL, "metallic": 0.55}),
    "rail": Look(color=(0.72, 0.72, 0.74), metallic=1.0, roughness=0.25),
    "bolt": Look(color=(0.55, 0.55, 0.57), metallic=1.0, roughness=0.32),
    "socket": Look(color=(0.01, 0.01, 0.012), roughness=0.7),
    "tag": Look(color=(0.012, 0.012, 0.012), roughness=0.6),
    "hole": Look(color=(0.02, 0.02, 0.022), roughness=0.9),
    "printed": Look(texture="print_white.jpg", tile=0.03, roughness=0.6),
    "tpu": Look(texture="tpu_orange.jpg", normal_map="pla_normal.jpg", tile=0.03, roughness=0.42),
    "tpu_worn": Look(texture="tpu_worn.jpg", normal_map="pla_normal.jpg", tile=0.03, roughness=0.8),
    "strip": Look(color=(0.35, 0.35, 0.35), texture="rubber_color.jpg", normal_map="rubber_normal.jpg", tile=0.04, roughness=0.7),
    "strip_worn": Look(texture="worn_rubber.jpg", normal_map="rubber_normal.jpg", tile=0.04, roughness=0.92),
    "black": Look(color=(0.035, 0.035, 0.04), metallic=0.5, roughness=0.45, normal_map="steel_normal.jpg", tile=0.12),
    "screw": Look(color=(0.02, 0.02, 0.022), metallic=0.8, roughness=0.35),
    "tray": Look(color=(0.16, 0.17, 0.18), rough_map="plastic_rough.jpg", normal_map="plastic_normal.jpg", tile=0.1),
    "red": Look(color=(0.62, 0.035, 0.03), rough_map="plastic_rough.jpg", normal_map="plastic_normal.jpg", tile=0.1),
}


@dataclass(frozen=True)
class FingerPart(Box):
    """One prim of the scene. A `Box` only so that yam_arena's checks accept it: `pos` and `size` are bounds."""

    kind: str = ""
    args: dict = field(default_factory=dict)


@dataclass(frozen=True)
class FingerState(ObjectState):
    report: dict = field(default_factory=dict)


def seat_xy(side: float) -> tuple[float, float]:
    """Where a seated finger is pinched; side +1 is the left finger."""
    return (GRIPPER_XY[0], GRIPPER_XY[1] + side * ROOT_Y)


def clamp_xy(side: float) -> tuple[float, float]:
    return (GRIPPER_XY[0] + CLAMP_X, GRIPPER_XY[1] + side * ROOT_Y)


SEATED_Z = TOP + BED_Z + FIN_T / 2
CLAMP_Z = TOP + BED_Z + FIN_T + CLAMP[2] / 2
SCREW_START_Z = TOP + SCREW_BLOCK[2]  # a screw's origin is under its head
TRAY_Z = TOP + TRAY_FLOOR + FIN_T / 2


def parts() -> list[FingerPart]:
    """The task file's OBJECTS. The left finger, at +y, is the worn one."""
    fin = (FIN_TIP + FIN_BACK, 2 * FIN_HALF_Y, FIN_T)
    return [
        FingerPart("spare_gripper", pos=(GRIPPER_XY[0] + 0.02, GRIPPER_XY[1], TOP + 0.07), size=(0.27, 0.2, 0.14), kind="gripper"),
        FingerPart("spare_tray", pos=(*TRAY_XY, TOP + 0.005), size=(TRAY_SIZE[0], TRAY_SIZE[1], 0.01), kind="tray"),
        FingerPart("red_bin", pos=(*BIN_XY, TOP + BIN_SIZE[2] / 2), size=BIN_SIZE, kind="bin"),
        FingerPart("worn_finger", pos=(*seat_xy(1.0), SEATED_Z + SPAWN_GAP), size=fin, kind="finger", args={"worn": True, "side": 1.0}),
        FingerPart("good_finger", pos=(*seat_xy(-1.0), SEATED_Z + SPAWN_GAP), size=fin, kind="finger", args={"worn": False, "side": -1.0}),
        FingerPart("new_finger", pos=(*NEW_XY, TRAY_Z + SPAWN_GAP), size=fin, kind="finger", args={"worn": False, "side": 1.0}),
        FingerPart("used_finger", pos=(*USED_XY, TRAY_Z + SPAWN_GAP), size=fin, kind="finger", args={"worn": True, "side": 1.0}),
        FingerPart("clamp_left", pos=(*clamp_xy(1.0), CLAMP_Z + 2 * SPAWN_GAP), size=CLAMP, kind="clamp", args={"side": 1.0}),
        FingerPart("clamp_right", pos=(*clamp_xy(-1.0), CLAMP_Z + 2 * SPAWN_GAP), size=CLAMP, kind="clamp", args={"side": -1.0}),
        FingerPart("screw_block", pos=(*SCREW_BLOCK_XY, TOP + SCREW_BLOCK[2] / 2), size=SCREW_BLOCK, kind="block"),
        *(FingerPart(f"screw_{i}", pos=(SCREW_BLOCK_XY[0] + dx, SCREW_BLOCK_XY[1], SCREW_START_Z + SPAWN_GAP), size=(0.008, 0.008, 0.04), kind="screw") for i, dx in enumerate(SCREW_BLOCK_HOLES)),
    ]


_NAMES = {p.name for p in parts()}


# -- the models ----------------------------------------------------------------------------


# The printed parts lie in their files with x across the gripper, y through its thickness and z along the fingers.
CAD_TO_WORLD = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def _load(name: str):
    return np.load(HERE / "umi_parts" / f"{name}.npz")


def _placed(name: str, rotation=CAD_TO_WORLD, offset=(0.0, 0.0, 0.0), keep=None) -> Mesh:
    """A printed part from umi_parts/, turned and moved. `keep` picks triangles by the corners' place in the file."""
    d = _load(name)
    v, f, n = d["v"].astype(float), d["f"], d["n"].astype(float) / 127.0
    if keep is not None:
        mask = keep(v[f].mean(axis=1))
        f, n = f[mask], n[mask]
    return Mesh(v @ rotation.T + np.asarray(offset, float), f, n @ rotation.T)


def _holes(name: str) -> np.ndarray:
    return _load(name)["holes"].astype(float)


def _finger_rotation(side: float) -> np.ndarray:
    return np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]]) if side > 0 else np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def _hexagon(radius: float) -> np.ndarray:
    return np.array([(radius * math.cos(math.radians(60 * k)), radius * math.sin(math.radians(60 * k))) for k in range(6)])


def _socket_screw(name: str, at, height: float = 0.003):
    """An M3 socket-head cap screw's head standing on `at`."""
    x, y, z = at
    return [(name, cylinder(0.00275, height, (x, y, z + height / 2), segments=14, bevel=0.0003), "bolt"),
            (f"{name}_socket", prism(_hexagon(0.00145), z + height - 0.00005, z + height + 0.00005).moved((x, y, 0.0)), "socket")]


def _nut(name: str, at, height: float = 0.0024):
    """An M3 hex nut on `at`, with the screw's end in it."""
    x, y, z = at
    return [(name, prism(_hexagon(0.00317), z, z + height, 0.0003).moved((x, y, 0.0)), "bolt"),
            (f"{name}_end", cylinder(0.0015, height + 0.0012, (x, y, z + (height + 0.0012) / 2), segments=12), "steel")]


def _finger_parts(worn: bool, side: float):
    """The real fin-ray finger lying flat, origin where it is pinched, its tip towards -x and its straight gripping face towards the other finger. A new finger is bright, with a black grip
    strip. A worn one is faded, with ribs torn out, its tip gone and its strip mostly gone."""
    rot = _finger_rotation(side)
    off = rot @ np.array([0.0, -PINCH_Y_IN_FINGER, 0.0])

    def keep(c):
        out = np.ones(len(c), bool)
        if worn:
            for (x0, x1, y0, y1) in ((-0.008, 0.009, 0.006, 0.013), (-0.012, 0.001, -0.013, -0.006), (-0.015, -0.004, -0.031, -0.026), (-0.03, 0.03, -0.07, -0.052)):
                out &= ~((c[:, 0] > x0) & (c[:, 0] < x1) & (c[:, 1] > y0) & (c[:, 1] < y1))
        return out

    mesh = _placed("finger", rot, off, keep)
    out = [("soft", mesh, "tpu_worn" if worn else "tpu")]
    t = 0.0015
    y = -side * (FIN_HALF_Y + t / 2)
    spans = ((-0.05, -0.043), (-0.075, -0.066)) if worn else ((-FIN_TIP + 0.003, -0.03),)
    for k, (a, b) in enumerate(spans):
        look = "strip_worn" if worn else "strip"
        out.append((f"strip{k}", box((b - a, t, FIN_T - (0.008 if worn else 0.001)), ((a + b) / 2, y, 0.0)), look))
    return out


def _finger_colliders(side: float, root: str):
    d = _load("finger")
    rot = _finger_rotation(side)
    v = d["hull_v"].astype(float) @ rot.T + rot @ np.array([0.0, -PINCH_Y_IN_FINGER, 0.0])
    # A pinch closes on the root's two sides, which this box keeps parallel, within 1 mm of the mesh, so that the finger does not turn in the hand.
    root_box = collider("c_root", (0.0217, 2 * FIN_HALF_Y, FIN_T), (0.00285, 0.0, 0.0), root)
    return [hull_collider("c_soft", Mesh(v, d["hull_f"]), root), root_box]


def _holder_placement(side: float):
    """The holder's part name and its origin, in the gripper's frame. The holder named right is the one on the robot's left."""
    name = "holder_right" if side > 0 else "holder_left"
    return name, np.array([HOLDER_X, side * ROOT_Y + side * 0.00255, Z_MID])


def fork_holes(side: float) -> list[tuple[float, float]]:
    """The three screw holes in a fork's upper plate, in the plate's frame, x and y."""
    name, origin = _holder_placement(side)
    centre = np.array([CLAMP_X, side * ROOT_Y, 0.0])
    out = []
    for hx, hz, _ in _holes(name):
        if hz <= 0.00675:
            p = CAD_TO_WORLD @ np.array([hx, 0.0, hz]) + origin - centre
            out.append((float(p[0]), float(p[1])))
    return out


def _screw_parts():
    """An M3 x 35 socket-head screw and its washer, origin under the washer, shank down."""
    out = [("washer", cylinder(0.0035, 0.0005, (0.0, 0.0, 0.00025), segments=16), "bolt"), ("shank", cylinder(0.0015, SCREW_LEN, (0.0, 0.0, -SCREW_LEN / 2), segments=10), "bolt")]
    out += _socket_screw("head", (0.0, 0.0, 0.0005))
    return out


def _screw_colliders(root: str):
    hz = SCREW_HEAD[2]
    sz = SCREW_STUB[2]
    return [collider("c_head", SCREW_HEAD, (0.0, 0.0, hz / 2), root, contact_offset=0.002), collider("c_stub", SCREW_STUB, (0.0, 0.0, -sz / 2), root, contact_offset=0.002)]


def _clamp_parts(side: float):
    """The real fork plate that holds a finger's root, with a marker tag, origin at the centre of its square part. The worn finger's plate has its three holes empty; the good finger's has its screws in."""
    name, origin = _holder_placement(side)
    centre = np.array([CLAMP_X, side * ROOT_Y, Z_MID + FIN_T / 2 + CLAMP[2] / 2])
    out = [("fork", _placed(name + "_fork", offset=origin - centre), "printed")]
    for i, (x, y) in enumerate(fork_holes(side)):
        if side < 0:
            top = CLAMP[2] / 2
            out.append((f"washer{i}", cylinder(0.0035, 0.0005, (x, y, top + 0.00025), segments=16), "bolt"))
            out += _socket_screw(f"screw{i}", (x, y, top + 0.0005))
        else:
            out.append((f"hole{i}", cylinder(0.0018, 0.0002, (x, y, CLAMP[2] / 2 + 0.0001), segments=12), "hole"))
    # A printed marker tag on the plate, as on the real gripper: black cells on the white plate.
    cell = 0.0026
    pattern = ("111111", "101001", "100111", "110101", "101101", "111111")
    cx, cy = -0.0022, 0.0
    top = 0.019 - FIN_T / 2 - CLAMP[2] / 2 + 0.0001  # just above the mesh's top face, 19 mm up in its file: level with it, the cells flicker
    for r, row in enumerate(pattern):
        for c, bit in enumerate(row):
            if bit == "1":
                out.append((f"tag{r}_{c}", box((cell, cell, 0.0002), (cx + (r - 2.5) * cell, cy + (c - 2.5) * cell, top)), "tag"))
    return out


def _boxes_minus_holes(prefix: str, x0, x1, y0, y1, holes, half, z0, z1, root: str, contact_offset: float = 0.002) -> list[str]:
    """Box colliders covering a rectangle, leaving square holes."""
    inside = [(hx, hy) for hx, hy in holes if x0 < hx < x1 and y0 < hy < y1]
    xs = sorted({x0, x1, *(hx - half for hx, _ in inside), *(hx + half for hx, _ in inside)})
    ys = sorted({y0, y1, *(hy - half for _, hy in inside), *(hy + half for _, hy in inside)})
    out = []
    for i, (a, b) in enumerate(zip(xs[:-1], xs[1:])):
        for j, (c, d) in enumerate(zip(ys[:-1], ys[1:])):
            mx, my = (a + b) / 2, (c + d) / 2
            if any(abs(mx - hx) < half and abs(my - hy) < half for hx, hy in inside):
                continue
            out.append(collider(f"{prefix}_{i}_{j}", (b - a, d - c, z1 - z0), (mx, my, (z0 + z1) / 2), root, contact_offset=contact_offset))
    return out


def _clamp_colliders(side: float, root: str) -> list[str]:
    """The plate's square part and its narrower front, as boxes. The worn finger's plate has its three holes; the good finger's has its screws in."""
    holes = fork_holes(side) if side > 0 else []
    z = CLAMP[2] / 2
    x_join = -0.014 + HOLDER_X - CLAMP_X
    x_front = -0.039 + HOLDER_X - CLAMP_X
    rear = _boxes_minus_holes("c_rear", -CLAMP[0] / 2, CLAMP[0] / 2, -CLAMP[1] / 2, CLAMP[1] / 2, holes, HOLE_HALF, -z, z, root)
    y0, y1 = sorted((side * 0.0045, side * 0.02))
    front = _boxes_minus_holes("c_front", x_front, x_join, y0, y1, holes, HOLE_HALF, -z, z, root)
    return rear + front


def _block_holes() -> list[tuple[float, float]]:
    return [(dx, 0.0) for dx in SCREW_BLOCK_HOLES]


def _block_parts():
    sz = SCREW_BLOCK[2]
    out = [("block", box(SCREW_BLOCK, (0.0, 0.0, sz / 2), bevel=0.0012, round_xy=0.002), "printed")]
    for i, (x, y) in enumerate(_block_holes()):
        out.append((f"hole{i}", cylinder(0.0022, 0.0002, (x, y, sz + 0.0001), segments=12), "hole"))
    return out


def _pocket(side: float) -> dict:
    """A finger's pocket walls, in the gripper's frame: name: (size, centre)."""
    yc = side * ROOT_Y
    hy = FIN_HALF_Y + CLEAR
    x_front = -FIN_TIP - CLEAR
    x_back = WEB_X
    xm, lx = (x_back + x_front) / 2, x_back - x_front
    z = BED_Z + LIP_H / 2
    return {
        "front": ((LIP_T, 2 * hy + 2 * LIP_T, LIP_H), (x_front - LIP_T / 2, yc, z)),
        "outer": ((lx, LIP_T, LIP_H), (xm, yc + side * (hy + LIP_T / 2), z)),
        "inner": ((lx, LIP_T, LIP_H), (xm, yc - side * (hy + LIP_T / 2), z)),
    }


def _plate_offset() -> np.ndarray:
    return np.array([PLATE_X + 0.0457, 0.0, PLATE_Z + 0.01315])


def _cover_offset() -> np.ndarray:
    return np.array([PLATE_X + 0.04572, 0.0, COVER_Z + 0.0291])


def _gripper_parts():
    """The spare gripper lying flat on a printed cradle in a steel fixture, its soft fingers towards the robot. Its bottom plate, top cover and finger holders are the real parts; the linear rail
    and the M3 screws and nuts stand where the real ones do. In the gripper's frame."""
    fx0, fx1, fy, ft = FIXTURE
    out = [
        ("fixture", box((fx1 - fx0, 2 * fy, ft), ((fx0 + fx1) / 2, 0.0, ft / 2), bevel=0.001), "steel"),
        ("cradle", box((PLATE_X + FIN_TIP + CLEAR + LIP_T, 0.15, BED_Z - ft), ((PLATE_X - FIN_TIP - CLEAR - LIP_T) / 2, 0.0, (BED_Z + ft) / 2), bevel=0.001), "printed"),
        ("riser", box((0.0914, 0.172, PLATE_Z - ft), (PLATE_X + 0.0457, 0.0, (PLATE_Z + ft) / 2), bevel=0.001), "printed"),
        ("bottom_plate", _placed("bottom_plate", offset=_plate_offset()), "printed"),
        ("top_cover", _placed("top_cover", offset=_cover_offset()), "printed"),
    ]
    rail_x, rail_z = 0.033, Z_MID + 0.0115
    out.append(("rail", box((0.009, 0.155, 0.0065), (rail_x, 0.0, rail_z), bevel=0.0004), "rail"))
    for k in range(-5, 6):
        out.append((f"rail_screw{k + 5}", cylinder(0.00175, 0.0004, (rail_x, k * 0.014, rail_z + 0.0034), segments=10), "socket"))
    for side, tag in ((1.0, "left"), (-1.0, "right")):
        name, origin = _holder_placement(side)
        # All of the holder but its upper fork plate, which is the clamp.
        out.append((f"{tag}_holder", _placed(name, offset=origin), "printed"))
        for key, (size, c) in _pocket(side).items():
            out.append((f"{tag}_{key}", box(size, c, bevel=0.0006), "printed"))
        out.append((f"{tag}_carriage", box((0.018, 0.022, 0.0075), (rail_x, side * ROOT_Y, rail_z + 0.0005), bevel=0.0006), "black"))
        for i, (hx, hz, _) in enumerate(_holes(name)):
            p = CAD_TO_WORLD @ np.array([hx, 0.0, hz]) + origin
            if hz > 0.00675:
                out += _socket_screw(f"{tag}_rail_screw{i}", (p[0], p[1], Z_MID + 0.016))
    for i, (hx, hz, _) in enumerate(_holes("top_cover")):
        p = CAD_TO_WORLD @ np.array([hx, 0.0, hz]) + _cover_offset()
        out += _nut(f"cover_nut{i}", (p[0], p[1], COVER_Z + 0.0048))
    for i, (x, y) in enumerate(((fx0 + 0.008, -fy + 0.008), (fx0 + 0.008, fy - 0.008), (fx1 - 0.008, -fy + 0.008), (fx1 - 0.008, fy - 0.008))):
        out += _socket_screw(f"fixture_screw{i}", (x, y, ft), 0.003)
    return out


def _gripper_colliders(root: str):
    fx0, fx1, fy, ft = FIXTURE
    x0 = -FIN_TIP - CLEAR - LIP_T
    wing_top = COVER_Z + 0.0582
    out = [
        collider("c_fixture", (fx1 - fx0, 2 * fy, ft), ((fx0 + fx1) / 2, 0.0, ft / 2), root),
        collider("c_cradle", (PLATE_X - x0, 0.15, BED_Z), ((x0 + PLATE_X) / 2, 0.0, BED_Z / 2), root),
        collider("c_body", (0.0914, 0.172, COVER_Z + 0.005), (PLATE_X + 0.0457, 0.0, (COVER_Z + 0.005) / 2), root),
        collider("c_fork_mount", (0.038, 0.03, COVER_Z + 0.0367), (PLATE_X + 0.0724, 0.0, (COVER_Z + 0.0367) / 2), root),
    ]
    for side, tag in ((1.0, "left"), (-1.0, "right")):
        out.append(collider(f"c_wing_{tag}", (0.0759, 0.036, wing_top), (PLATE_X + 0.03795, side * 0.068, wing_top / 2), root))
        # Up to just under a finger's top, to stop the finger without catching the fork plate that rests on it.
        out.append(collider(f"c_holder_{tag}", (PLATE_X - WEB_X, 0.045, Z_MID + 0.0125), ((PLATE_X + WEB_X) / 2, side * ROOT_Y, (Z_MID + 0.0125) / 2), root))
        for key, (size, c) in _pocket(side).items():
            out.append(collider(f"c_{tag}_{key}", size, c, root))
    return out


def _tray_walls() -> dict:
    """The tray's base and a low wall round each spare, in the tray's frame (origin on the table)."""
    sx, sy, sz = TRAY_SIZE
    out = {"base": ((sx, sy, sz), (0.0, 0.0, sz / 2))}
    z = sz + TRAY_LIP / 2
    for i, xy in enumerate((NEW_XY, USED_XY)):
        rx, ry = xy[0] - TRAY_XY[0], xy[1] - TRAY_XY[1]
        hy = FIN_HALF_Y + CLEAR
        x_back = rx + FIN_BACK + CLEAR
        x_front = rx - FIN_TIP - CLEAR
        xm, lx = (x_back + x_front) / 2, x_back - x_front
        out[f"p{i}_back"] = ((LIP_T, 2 * hy + 2 * LIP_T, TRAY_LIP), (x_back + LIP_T / 2, ry, z))
        out[f"p{i}_front"] = ((LIP_T, 2 * hy + 2 * LIP_T, TRAY_LIP), (x_front - LIP_T / 2, ry, z))
        out[f"p{i}_a"] = ((lx, LIP_T, TRAY_LIP), (xm, ry + hy + LIP_T / 2, z))
        out[f"p{i}_b"] = ((lx, LIP_T, TRAY_LIP), (xm, ry - hy - LIP_T / 2, z))
    return out


def _bin_walls() -> dict:
    sx, sy, sz = BIN_SIZE
    w = BIN_WALL
    return {
        "floor": ((sx, sy, w), (0.0, 0.0, w / 2)),
        "front": ((w, sy, sz), (-sx / 2 + w / 2, 0.0, sz / 2)),
        "back": ((w, sy, sz), (sx / 2 - w / 2, 0.0, sz / 2)),
        "left": ((sx - 2 * w, w, sz), (0.0, sy / 2 - w / 2, sz / 2)),
        "right": ((sx - 2 * w, w, sz), (0.0, -sy / 2 + w / 2, sz / 2)),
    }


def _bin_parts():
    """A moulded parts bin with a rolled rim."""
    sx, sy, sz = BIN_SIZE
    out = [(k, box(size, c, bevel=0.0005), "red") for k, (size, c) in _bin_walls().items()]
    rim = 0.006
    for k, size, c in (("rim_a", (rim, sy + 0.004, 0.004), (sx / 2 - rim / 2 + 0.002, 0.0, sz - 0.002)), ("rim_b", (rim, sy + 0.004, 0.004), (-sx / 2 + rim / 2 - 0.002, 0.0, sz - 0.002)),
                       ("rim_c", (sx + 0.004, rim, 0.004), (0.0, sy / 2 - rim / 2 + 0.002, sz - 0.002)), ("rim_d", (sx + 0.004, rim, 0.004), (0.0, -sy / 2 + rim / 2 - 0.002, sz - 0.002))):
        out.append((k, box(size, c, bevel=0.0012), "red"))
    return out


# -- spawning ----------------------------------------------------------------------------


def _usd_dir() -> str:
    path = os.path.join(tempfile.gettempdir(), f"replace_soft_finger_{os.getpid()}")
    os.makedirs(path, exist_ok=True)
    return path


def _arena_object(spec: FingerPart):
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    d = _usd_dir()
    path = os.path.join(d, f"{spec.name}.usda")
    if spec.kind == "gripper":
        usd = write_usda(path, "SpareGripper", _gripper_parts(), LOOKS, _gripper_colliders("SpareGripper"), friction=(0.5, 0.4))
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=(*GRIPPER_XY, TOP)))
    if spec.kind == "tray":
        walls = _tray_walls()
        meshes = [(k, box(size, c, bevel=0.0006), "tray") for k, (size, c) in walls.items()]
        usd = write_usda(path, "SpareTray", meshes, LOOKS, [collider(f"c_{k}", size, c, "SpareTray") for k, (size, c) in walls.items()], friction=(0.5, 0.4))
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=(*TRAY_XY, TOP)))
    if spec.kind == "bin":
        usd = write_usda(path, "RedBin", _bin_parts(), LOOKS, [collider(f"c_{k}", size, c, "RedBin") for k, (size, c) in _bin_walls().items()], friction=(0.5, 0.4))
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=(*BIN_XY, TOP)))
    if spec.kind == "block":
        sx, sy, sz = SCREW_BLOCK
        cs = _boxes_minus_holes("c", -sx / 2, sx / 2, -sy / 2, sy / 2, _block_holes(), HOLE_HALF, 0.0, sz, "ScrewBlock")
        usd = write_usda(path, "ScrewBlock", _block_parts(), LOOKS, cs, friction=(0.5, 0.4))
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=(*SCREW_BLOCK_XY, TOP)))
    root = "".join(w.capitalize() for w in spec.name.split("_"))
    if spec.kind == "screw":
        usd = write_usda(path, root, _screw_parts(), LOOKS, _screw_colliders(root), mass=SCREW_MASS, friction=(1.2, 1.0), angular_damping=5.0)
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd, initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos)))
    if spec.kind == "finger":
        usd = write_usda(path, root, _finger_parts(spec.args["worn"], spec.args["side"]), LOOKS, _finger_colliders(spec.args["side"], root), mass=FIN_MASS, friction=(1.2, 1.0))
    else:
        usd = write_usda(path, root, _clamp_parts(spec.args["side"]), LOOKS, _clamp_colliders(spec.args["side"], root), mass=CLAMP_MASS, friction=(1.0, 0.8))
    return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd, initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos)))


# -- the score ---------------------------------------------------------------------------


def _t(x):
    return getattr(x, "torch", x)


def _rotation(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def task_score(good_moved: bool, off_table: bool, worn_lifted: bool, worn_in_jaw: bool, worn_in_bin: bool, new_seated: bool, plate_back: bool, full: bool) -> float:
    """The scoring text, line by line, given the trial's latches and the state at this step."""
    if good_moved or off_table or not worn_lifted or worn_in_jaw:
        return 0.0
    if not worn_in_bin:
        return 0.25
    if not new_seated:
        return 0.5
    if not plate_back:
        return 0.75
    return 1.0 if full else 0.85


class _Tracker:
    """Keeps the trial's latches and the state; writes it to finger_scores/ every 10 steps and when the score changes, because the rollout's log keeps only the
    final score."""

    def __init__(self, env):
        self.base = env.unwrapped
        self.dt = float(self.base.step_dt)
        self.fingers = {r: (self.base.scene[r].find_bodies("gripper")[0][0], self.base.scene[r].find_joints("joint7")[0][0], self.base.scene[r].find_joints("joint8")[0][0]) for r in ROBOTS}
        self.dir = os.environ.get("LIFT_SCORE_DIR", "finger_scores")
        self.step_file = os.environ.get("LIFT_STEP_FILE")
        self.last_buf = 0
        self.holes = fork_holes(1.0)
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.good_moved = False
        self.good_start = None
        self.off_table = False
        self.worn_lifted = False
        self.held = 0
        self.marks: list[float] = []
        self.worst_30 = 0.0
        self.trace: list[dict] = []
        self.last_score = None
        self.path = None

    def _body(self, name: str) -> dict:
        s = self.base.scene[name]
        o = self.base.scene.env_origins[0]
        q = _t(s.data.root_quat_w)[0].cpu().numpy().astype(float)
        R = _rotation(q)
        return {"p": (_t(s.data.root_pos_w)[0] - o).cpu().numpy().astype(float), "R": R, "tilt": float(np.degrees(math.acos(min(1.0, abs(R[2, 2]))))),
                "v": _t(s.data.root_lin_vel_w)[0].cpu().numpy().astype(float), "w": _t(s.data.root_ang_vel_w)[0].cpu().numpy().astype(float)}

    @staticmethod
    def _still(b: dict) -> bool:
        return bool(np.linalg.norm(b["v"]) < 0.01 and np.linalg.norm(b["w"]) < 0.5)

    def _finger_points(self) -> np.ndarray:
        o = self.base.scene.env_origins[0]
        pts = []
        for r, (g, j7, j8) in self.fingers.items():
            robot = self.base.scene[r]
            p = (_t(robot.data.body_link_pos_w)[0, g] - o).cpu().numpy().astype(float)
            R = _rotation(_t(robot.data.body_link_quat_w)[0, g].cpu().numpy().astype(float))
            q = _t(robot.data.joint_pos)[0]
            for sgn, j in ((1.0, j7), (-1.0, j8)):
                pts.append(p + R @ np.array([-sgn * float(q[j]), 0.0, TIP_IN_GRIPPER]))
        return np.array(pts)

    @staticmethod
    def _gap_to_box(points: np.ndarray, b: dict, half, centre=(0.0, 0.0, 0.0)) -> float:
        """The least distance from the points to the part's box."""
        local = (points - b["p"]) @ b["R"] - np.asarray(centre)
        return float(np.linalg.norm(np.maximum(np.abs(local) - np.asarray(half), 0.0), axis=1).min())

    @staticmethod
    def _below_table(b: dict) -> bool:
        """Off the table: its origin over 5 cm below the table's top."""
        return bool(b["p"][2] < TOP - 0.05)

    @classmethod
    def _lifted_in_hand(cls, b: dict, pts: np.ndarray) -> bool:
        """Lifted by a hand: both fingertips of one gripper within 1 cm of the finger's box, and its root at least 3 cm above a seated finger's height."""
        if b["p"][2] < SEATED_Z + 0.03:
            return False
        half, centre = ((FIN_TIP + FIN_BACK) / 2, FIN_HALF_Y, FIN_T / 2), ((FIN_BACK - FIN_TIP) / 2, 0.0, 0.0)
        return any(max(cls._gap_to_box(pts[i:i + 1], b, half, centre), cls._gap_to_box(pts[i + 1:i + 2], b, half, centre)) <= 0.01 for i in (0, 2))

    @staticmethod
    def _in_jaw(b: dict, side: float) -> bool:
        """Lying in a jaw's pocket: its root over the seat within 1 cm along x and y, and within 1.5 cm of a seated finger's height."""
        x, y = seat_xy(side)
        return bool(abs(b["p"][0] - x) <= 0.01 and abs(b["p"][1] - y) <= 0.01 and b["p"][2] <= SEATED_Z + 0.015)

    @staticmethod
    def _yaw(b: dict) -> float:
        """Degrees between the finger's tip and the direction of the robot, about the vertical."""
        return float(np.degrees(math.atan2(b["R"][1, 0], b["R"][0, 0])))

    @classmethod
    def _seated(cls, b: dict, side: float) -> bool:
        """Home: its root within 4.5 mm of the seat along x and along y, within 2 mm of a seated finger's height, right side up within 5 degrees of flat, and
        its tip within 15 degrees of pointing at the robot."""
        x, y = seat_xy(side)
        level = b["R"][2, 2] >= math.cos(math.radians(5.0))
        return bool(abs(b["p"][0] - x) <= 0.0045 and abs(b["p"][1] - y) <= 0.0045 and abs(b["p"][2] - SEATED_Z) <= 0.002 and level and abs(cls._yaw(b)) <= 15.0)

    @staticmethod
    def _clamped(b: dict, side: float) -> bool:
        """In its place: within 4 mm along x, 7 mm across and 2 mm of height, within 5 degrees of level."""
        x, y = clamp_xy(side)
        return bool(abs(b["p"][0] - x) <= 0.004 and abs(b["p"][1] - y) <= 0.007 and abs(b["p"][2] - CLAMP_Z) <= 0.002 and b["tilt"] <= 5.0)

    def _screws_in(self, b: dict) -> int:
        """How many of the left plate's three holes hold a screw: over the hole within 3 mm in the plate's frame, its head within 2 mm of the plate's top, within 15 degrees of upright."""
        plate = b["clamp_left"]
        filled = 0
        for hx, hy in self.holes:
            for n in ("screw_0", "screw_1", "screw_2"):
                local = (b[n]["p"] - plate["p"]) @ plate["R"]
                if abs(local[0] - hx) <= 0.003 and abs(local[1] - hy) <= 0.003 and abs(local[2] - CLAMP[2] / 2) <= 0.002 and b[n]["tilt"] <= 15.0:
                    filled += 1
                    break
        return filled

    @staticmethod
    def _in_bin(b: dict) -> bool:
        """The finger's middle between the bin's walls and below the rim."""
        hx, hy = BIN_SIZE[0] / 2 - BIN_WALL, BIN_SIZE[1] / 2 - BIN_WALL
        c = b["p"] + b["R"] @ np.array([-FIN_MIDDLE, 0.0, 0.0])
        return bool(abs(c[0] - BIN_XY[0]) < hx and abs(c[1] - BIN_XY[1]) < hy and c[2] < TOP + BIN_SIZE[2])

    def step(self) -> dict:
        buf = int(self.base.episode_length_buf[0])
        if buf < self.last_buf:
            self.reset()
        self.last_buf = buf
        self.marks.append(time.monotonic())
        if len(self.marks) > 30:
            self.worst_30 = max(self.worst_30, self.marks[-1] - self.marks[-31])
        self.steps += 1
        t = self.steps * self.dt
        names = ("worn_finger", "good_finger", "new_finger", "used_finger", "clamp_left", "clamp_right", "screw_0", "screw_1", "screw_2")
        b = {n: self._body(n) for n in names}
        if self.good_start is None:
            self.good_start = b["good_finger"]["p"].copy()
        good_shift = float(np.linalg.norm(b["good_finger"]["p"] - self.good_start))
        if good_shift > 0.003:
            self.good_moved = True
        worn_in_jaw = self._in_jaw(b["worn_finger"], 1.0) or self._in_jaw(b["worn_finger"], -1.0)
        worn_in_bin = self._in_bin(b["worn_finger"])
        new_seated = self._seated(b["new_finger"], 1.0)
        plate_back = self._clamped(b["clamp_left"], 1.0) and self._clamped(b["clamp_right"], -1.0)
        screws = self._screws_in(b)
        pts = self._finger_points()
        if any(self._below_table(b[n]) for n in names):
            self.off_table = True
        if self._lifted_in_hand(b["worn_finger"], pts):
            self.worn_lifted = True
        fin_half, fin_centre = ((FIN_TIP + FIN_BACK) / 2, FIN_HALF_Y, FIN_T / 2), ((FIN_BACK - FIN_TIP) / 2, 0.0, 0.0)
        gap = min(self._gap_to_box(pts, b[n], fin_half, fin_centre) for n in ("new_finger", "good_finger"))
        gap = min(gap, *(self._gap_to_box(pts, b[n], np.array(CLAMP) / 2) for n in ("clamp_left", "clamp_right")))
        gap = min(gap, *(self._gap_to_box(pts, b[n], np.array(SCREW_HEAD) / 2, (0.0, 0.0, SCREW_HEAD[2] / 2)) for n in ("screw_0", "screw_1", "screw_2")))
        still = all(self._still(b[n]) for n in names)
        ready = bool(self.worn_lifted and worn_in_bin and new_seated and plate_back and screws == 3 and gap >= 0.05 and still and not self.good_moved and not self.off_table)
        self.held = self.held + 1 if ready else 0
        full = self.held * self.dt >= 1.0
        score = task_score(self.good_moved, self.off_table, self.worn_lifted, worn_in_jaw, worn_in_bin, new_seated, plate_back, full)
        rep = {"t": round(t, 2), "score": score, "ready": ready, "good_moved": self.good_moved, "good_shift_mm": round(good_shift * 1000, 2), "off_table": self.off_table, "worn_lifted": self.worn_lifted, "worn_in_jaw": worn_in_jaw, "worn_in_bin": worn_in_bin,
               "new_seated": new_seated, "new_yaw_deg": round(self._yaw(b["new_finger"]), 2), "used_seated": self._seated(b["used_finger"], 1.0), "plate_back": plate_back, "screws_in": screws, "gripper_gap_m": round(gap, 3), "still": still,
               **{n: [round(float(v), 4) for v in b[n]["p"]] for n in names}, "slowest_30_steps_s": round(self.worst_30, 3)}
        if self.step_file:
            with open(self.step_file, "w") as f:
                f.write(f"{t:.2f}")
        if self.steps % 10 == 0 or self.last_score != score:
            self.trace.append(rep)
            os.makedirs(self.dir, exist_ok=True)
            self.path = self.path or os.path.join(self.dir, f"score_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json")
            with open(self.path, "w") as f:
                json.dump({"final": rep, "trace": self.trace}, f, indent=0)
        self.last_score = score
        return rep


_tracker: _Tracker | None = None


def _patch_scene(scene) -> None:
    if getattr(scene, "_fingerkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, FingerPart) else orig_task_object(spec)

    def object_states(env, names):
        global _tracker
        if _tracker is None or _tracker.base is not env.unwrapped:
            _tracker = _Tracker(env)
        rep = _tracker.step()
        others = [n for n in names if n not in _NAMES]
        out = orig_states(env, others) if others else {}
        zero = np.zeros(3)
        for n in names:
            if n in _NAMES:
                out[n] = FingerState(pos=zero, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=zero, ang_vel=zero, low=zero, high=zero, report=rep)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._fingerkit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, orig_exec = spec.loader, spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena.scene (now if already imported, else when it is). Idempotent."""
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
