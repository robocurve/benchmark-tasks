"""printkit: the printer, print plates, rests and printed part for this task file, on yam_arena.

A `yam_arena` task file declares rigid primitives. This task's objects are modelled meshes with textures and simple box colliders,
and its lamp changes colour, so this module does what `cablekit`-style helpers do for cables: `PrintPart` subclasses `Box` so
yam_arena's task-file checks accept it, and importing this module hooks `yam_arena.scene` (it modifies nothing on disk) so each
part spawns from a USD file written at start-up and `object_states` reports the task's state. The hook also runs the bed's cooling
timer once per control step, for any policy: the screen shows the bed falling from 60 to 35 C, with a hot-surface symbol until it gets there,
and the lamp shows red, orange from 5 s and green at 10 s of simulated time.

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
from usdkit import Look, Mesh, box, collider, cylinder, join, lathe, physics_material, prism, tube, write_usda  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BED_XY = (0.41, 0.0)
BED_SIZE = (0.24, 0.24, 0.03)
BED_TOP = TOP + BED_SIZE[2]
PLATE_SIDE = 0.18
PLATE_T = 0.004
PLATE_MASS = 0.6  # 4 mm aluminium, a 0.5 mm spring-steel PEI sheet and two lugs
LUG = (0.024, 0.03)  # along x, along y
LUG_RISE = 0.025  # the lug's top above the plate's top
LUG_Y = PLATE_SIDE / 2 + LUG[1] / 2
PRINT_MASS = 0.06  # PLA at 30 % infill
PRINT_H = 0.04
REST_SIZE = (0.23, 0.22, 0.02)
REST_TOP = TOP + REST_SIZE[2]
GREY_XY = (0.175, -0.135)
BLUE_XY = (0.175, 0.135)
SPAWN_GAP = 0.0005
PIN_RADIUS = 0.004
PIN_HEIGHT = 0.012
PIN_GAP = 0.006  # between a centred plate's edge and a pin
FRAME_X = 0.145  # the printer's rear frame, from the bed's centre
FRAME_H = 0.29
GREEN_AT = 10.0  # s of simulated time
ORANGE_AT = 5.0
SCREEN_SIZE = (0.15, 0.0855)  # wide, tall: about 7 inches, large for a desktop printer so that it reads on camera
SCREEN_TILT = 45.0  # degrees back from upright, facing the top camera
SCREEN_CELLS = (3, 4)  # columns, rows of textures/screen_atlas.png: one picture per second of cooling
MOVED_EARLY = 0.01
LIFTED = 0.01
OFF_BED = 0.05
ROBOTS = ("left_robot", "right_robot")
TIP_IN_GRIPPER = 0.147

STEEL = dict(texture="steel_color.jpg", rough_map="steel_rough.jpg", normal_map="steel_normal.jpg", tile=0.12, metallic=1.0)
LOOKS = {
    "pei": Look(texture="pei_color.jpg", normal_map="pei_normal.jpg", tile=0.05, metallic=0.35, roughness=0.5),
    "alu": Look(color=(0.95, 0.95, 0.97), **STEEL),
    "steel": Look(color=(0.72, 0.72, 0.74), **STEEL),
    "black_alu": Look(color=(0.05, 0.05, 0.055), metallic=0.6, roughness=0.42, normal_map="steel_normal.jpg", tile=0.12),
    "extrusion": Look(color=(0.035, 0.035, 0.04), metallic=0.5, roughness=0.5, normal_map="steel_normal.jpg", tile=0.12),
    "bed_top": Look(color=(0.018, 0.018, 0.02), metallic=0.3, roughness=0.35, normal_map="plastic_normal.jpg", tile=0.1),
    "plastic": Look(color=(0.03, 0.03, 0.033), roughness=0.6, rough_map="plastic_rough.jpg", normal_map="plastic_normal.jpg", tile=0.1),
    "brass": Look(color=(0.78, 0.58, 0.24), metallic=1.0, roughness=0.3),
    "ptfe": Look(color=(0.9, 0.9, 0.88), roughness=0.4),
    "pla": Look(texture="pla_color.jpg", normal_map="pla_normal.jpg", tile=0.04, roughness=0.45),
    "filament": Look(color=(0.75, 0.16, 0.012), roughness=0.35),
    "hole": Look(color=(0.1, 0.03, 0.005), roughness=0.9),
    "grey_rest": Look(color=(0.27, 0.28, 0.29), roughness=0.55, normal_map="plastic_normal.jpg", tile=0.1),
    "blue_rest": Look(color=(1.0, 1.0, 1.0), texture="blue_paint.jpg", roughness=0.45, tile=0.2, normal_map="plastic_normal.jpg"),
    "rubber": Look(color=(0.02, 0.02, 0.02), roughness=0.85),
    "screen": Look(color=(0.0, 0.0, 0.0), roughness=0.25, emissive_map="screen_atlas.png", emissive_gain=1.0),
    "lens_red": Look(color=(0.9, 0.03, 0.02), roughness=0.3, emissive=(1.2, 0.02, 0.01)),
    "lens_orange": Look(color=(1.0, 0.35, 0.02), roughness=0.3, emissive=(1.0, 0.1, 0.0)),
    "lens_green": Look(color=(0.05, 0.9, 0.12), roughness=0.3, emissive=(0.0, 0.8, 0.04)),
}


@dataclass(frozen=True)
class PrintPart(Box):
    """One prim of the scene. A `Box` only so that yam_arena's checks accept it: `pos` and `size` are bounds."""

    kind: str = ""
    args: dict = field(default_factory=dict)


@dataclass(frozen=True)
class PrintState(ObjectState):
    report: dict = field(default_factory=dict)


def plate_z_on(surface_top: float) -> float:
    """A plate's origin, the middle of its 4 mm slab, when it rests on a surface."""
    return surface_top + PLATE_T / 2


USED_START = (BED_XY[0], BED_XY[1], plate_z_on(BED_TOP) + SPAWN_GAP)
CLEAN_START = (BLUE_XY[0], BLUE_XY[1], plate_z_on(REST_TOP) + SPAWN_GAP)
PRINT_START = (BED_XY[0], BED_XY[1], USED_START[2] + PLATE_T / 2 + SPAWN_GAP)  # the print's origin is the centre of its base


def parts() -> list[PrintPart]:
    """The task file's OBJECTS."""
    bx, by = BED_XY
    return [
        PrintPart("printer", pos=(bx, by, TOP + FRAME_H / 2), size=(0.4, 0.5, FRAME_H), kind="printer"),
        PrintPart("grey_rest", pos=(*GREY_XY, TOP + REST_SIZE[2] / 2), size=REST_SIZE, kind="rest", args={"look": "grey_rest", "xy": GREY_XY}),
        PrintPart("blue_rest", pos=(*BLUE_XY, TOP + REST_SIZE[2] / 2), size=REST_SIZE, kind="rest", args={"look": "blue_rest", "xy": BLUE_XY}),
        PrintPart("used_plate", pos=USED_START, size=(PLATE_SIDE, PLATE_SIDE + 2 * LUG[1], PLATE_T), kind="plate"),
        PrintPart("clean_plate", pos=CLEAN_START, size=(PLATE_SIDE, PLATE_SIDE + 2 * LUG[1], PLATE_T), kind="plate"),
        PrintPart("print_part", pos=PRINT_START, size=(0.06, 0.06, PRINT_H), kind="print"),
    ]


_NAMES = {p.name for p in parts()}


# -- the models ----------------------------------------------------------------------------


def _plate_parts():
    """An aluminium plate with a gold PEI sheet on top and a machined lifting lug on each side. Origin: the slab's middle."""
    h, t = PLATE_SIDE, PLATE_T
    lug_h = t + LUG_RISE
    out = [
        ("slab", box((h, h, t - 0.0008), (0.0, 0.0, -0.0004), round_xy=0.006), "alu"),
        ("sheet", box((h - 0.001, h - 0.001, 0.0008), (0.0, 0.0, t / 2 - 0.0004), round_xy=0.006), "pei"),
    ]
    for sgn, name in ((1.0, "left"), (-1.0, "right")):
        lug = box((LUG[0], LUG[1], lug_h), (0.0, sgn * LUG_Y, -t / 2 + lug_h / 2), bevel=0.0015, round_xy=0.002)
        out.append((f"lug_{name}", lug, "alu"))
        for x in (-0.03, 0.03):
            out.append((f"screw_{name}_{x > 0}", cylinder(0.0025, 0.001, (x, sgn * (PLATE_SIDE / 2 - 0.006), t / 2 + 0.0004), segments=10), "steel"))
    return out


def _plate_colliders(root: str):
    t = PLATE_T
    lug_h = t + LUG_RISE
    out = [collider("c_slab", (PLATE_SIDE, PLATE_SIDE, t), (0.0, 0.0, 0.0), root, "SlabMaterial")]
    for sgn, name in ((1.0, "left"), (-1.0, "right")):
        out.append(collider(f"c_lug_{name}", (LUG[0], LUG[1], lug_h), (0.0, sgn * LUG_Y, -t / 2 + lug_h / 2), root))
    return out


def _print_parts():
    """A motor bracket as it comes off the plate: a square foot with four screw holes, and a round boss with a bore."""
    foot = box((0.06, 0.06, 0.012), (0.0, 0.0, 0.006), bevel=0.0008, round_xy=0.005)
    boss = lathe([(0.021, 0.012), (0.021, PRINT_H - 0.001), (0.020, PRINT_H), (0.012, PRINT_H), (0.011, PRINT_H - 0.001), (0.011, 0.016), (0.0, 0.016)], 40)
    out = [("foot", foot, "pla"), ("boss", boss, "pla")]
    for i, (x, y) in enumerate(((-0.022, -0.022), (-0.022, 0.022), (0.022, -0.022), (0.022, 0.022))):
        out.append((f"hole{i}", cylinder(0.0035, 0.0004, (x, y, 0.0122), segments=14), "hole"))
    return out


def _extrusion(w: float, d: float, length: float) -> Mesh:
    """A black T-slot extrusion along z, with a groove down the middle of each face."""
    g, a = 0.003, 0.0045  # half width and depth of a groove
    hx, hy = w / 2, d / 2
    outline = [(hx, -hy), (hx, -g), (hx - a, -g), (hx - a, g), (hx, g), (hx, hy), (g, hy), (g, hy - a), (-g, hy - a), (-g, hy), (-hx, hy), (-hx, g), (-hx + a, g),
               (-hx + a, -g), (-hx, -g), (-hx, -hy), (-g, -hy), (-g, -hy + a), (g, -hy + a), (g, -hy)]
    return prism(outline, 0.0, length)


def _printer_parts():
    """A desktop bed-slinger printer with its bed run forward, in the printer's frame: the origin is on the table under the bed's centre,
    x away from the robot. Sizes follow common 220 mm desktop printers, its frame shortened to stay in the top camera's view."""
    fx, fh = FRAME_X, FRAME_H
    bh = BED_SIZE[2]
    out = [
        ("bed_carriage", box((0.24, 0.24, bh - 0.007), (0.0, 0.0, (bh - 0.007) / 2), bevel=0.002, round_xy=0.01), "black_alu"),
        ("bed_top", box((0.24, 0.24, 0.007), (0.0, 0.0, bh - 0.0035), bevel=0.001, round_xy=0.01), "bed_top"),
        ("belt_cover", box((0.03, 0.16, 0.02), (0.128, 0.0, 0.01)), "plastic"),
    ]
    for i, (x, y) in enumerate(((-0.11, -0.11), (-0.11, 0.11), (0.11, -0.11), (0.11, 0.11))):
        out.append((f"bed_screw{i}", cylinder(0.0045, 0.002, (x, y, bh + 0.001), segments=14, bevel=0.0005), "steel"))
    for i, (x, y) in enumerate(_pins()):
        pin = lathe([(0.0, bh), (PIN_RADIUS, bh), (PIN_RADIUS, bh + PIN_HEIGHT - 0.0015), (PIN_RADIUS - 0.0015, bh + PIN_HEIGHT), (0.0, bh + PIN_HEIGHT)], 16)
        out.append((f"pin{i}", pin.moved((x, y, 0.0)), "steel"))
    # The frame.
    for sgn, name in ((1.0, "left"), (-1.0, "right")):
        out.append((f"upright_{name}", _extrusion(0.02, 0.04, fh).moved((fx, sgn * 0.175, 0.0)), "extrusion"))
        out.append((f"motor_{name}", box((0.042, 0.042, 0.04), (fx - 0.032, sgn * 0.175, 0.02), bevel=0.003), "black_alu"))
        out.append((f"motor_cap_{name}", box((0.043, 0.043, 0.008), (fx - 0.032, sgn * 0.175, 0.03), bevel=0.001), "alu"))
        out.append((f"leadscrew_{name}", cylinder(0.004, fh - 0.06, (fx - 0.032, sgn * 0.175, 0.04 + (fh - 0.06) / 2), segments=12), "brass"))
        out.append((f"z_block_{name}", box((0.05, 0.03, 0.03), (fx - 0.025, sgn * 0.16, 0.19), bevel=0.003), "plastic"))
    out.append(("base_bar", _extrusion(0.02, 0.04, 0.39).turned(90.0, "x").moved((fx, 0.195, 0.02)), "extrusion"))
    out.append(("top_bar", _extrusion(0.02, 0.02, 0.39).turned(90.0, "x").moved((fx, 0.195, fh - 0.01)), "extrusion"))
    for z, name in ((0.175, "low"), (0.205, "high")):
        out.append((f"rail_{name}", cylinder(0.004, 0.33, (fx - 0.03, 0.0, z), axis="y", segments=12), "steel"))
    # The print head, parked at the middle of the gantry.
    hx = fx - 0.04
    out += [
        ("head_body", box((0.04, 0.06, 0.06), (hx, 0.0, 0.19), bevel=0.003, round_xy=0.004), "plastic"),
        ("head_fan", cylinder(0.019, 0.008, (hx - 0.024, 0.0, 0.19), axis="x", segments=24, bevel=0.001), "black_alu"),
        ("head_hub", cylinder(0.007, 0.003, (hx - 0.029, 0.0, 0.19), axis="x", segments=16), "alu"),
        ("heat_sink", cylinder(0.011, 0.018, (hx, 0.0, 0.151), segments=20), "alu"),
        ("heat_block", box((0.02, 0.022, 0.011), (hx, 0.0, 0.137), bevel=0.0008), "alu"),
        ("nozzle", lathe([(0.0, 0.122), (0.002, 0.122), (0.005, 0.127), (0.005, 0.132), (0.0, 0.132)], 14).moved((hx, 0.0, 0.0)), "brass"),
        ("ptfe", tube([(hx, 0.0, 0.22), (hx, 0.0, 0.25), (hx + 0.05, -0.08, 0.28), (hx + 0.10, -0.22, 0.25), (fx + 0.04, -0.28, 0.21)], 0.002, 8), "ptfe"),
    ]
    # The spool on its stand, beside the frame.
    sx, sy, sz = 0.19, -0.31, 0.11
    flange = lambda y: cylinder(0.10, 0.005, (sx, y, sz), axis="y", segments=40, bevel=0.001)
    out += [
        ("spool_flange_a", flange(sy - 0.033), "plastic"),
        ("spool_flange_b", flange(sy + 0.033), "plastic"),
        ("spool_filament", cylinder(0.086, 0.061, (sx, sy, sz), axis="y", segments=40), "filament"),
        ("spool_axle", cylinder(0.006, 0.12, (sx, sy, sz), axis="y", segments=14), "steel"),
    ]
    for sgn in (1.0, -1.0):
        foot_y = sy + sgn * 0.055
        out.append((f"stand_leg_a{sgn > 0}", tube([(sx - 0.06, foot_y, 0.0), (sx, foot_y, sz)], 0.005, 8), "extrusion"))
        out.append((f"stand_leg_b{sgn > 0}", tube([(sx + 0.06, foot_y, 0.0), (sx, foot_y, sz)], 0.005, 8), "extrusion"))
    # The screen on the top bar, leaning back towards the top camera.
    out += _screen_parts()
    # The lamp on the left upright.
    ly = 0.175
    lamp_z = fh + 0.018
    out += [
        ("lamp_post", cylinder(0.007, 0.022, (fx, ly, fh + 0.009), segments=16), "alu"),
        ("lamp_collar", cylinder(0.018, 0.006, (fx, ly, lamp_z - 0.001), segments=24, bevel=0.001), "plastic"),
    ]
    dome = lathe([(0.0, 0.0), (0.016, 0.0), (0.0185, 0.012), (0.0185, 0.03), (0.015, 0.041), (0.008, 0.047), (0.0, 0.049)], 28).moved((fx, ly, lamp_z))
    out.append(("lens_red", dome, "lens_red"))
    out.append(("lens_orange", dome, "lens_orange", True))
    out.append(("lens_green", dome, "lens_green", True))
    return out


def _screen_axes():
    """The screen's centre, and its right, up and outward directions as seen from the robot."""
    t = math.radians(SCREEN_TILT)
    right = np.array([0.0, -1.0, 0.0])
    up = np.array([math.sin(t), 0.0, math.cos(t)])
    out = np.array([-math.cos(t), 0.0, math.sin(t)])
    return np.array([FRAME_X + 0.012, 0.0, FRAME_H + 0.036]), right, up, out


def _screen_cell(seconds_left: int) -> Mesh:
    """The screen's face with `seconds_left` of cooling to go: a quad over that cell of the picture."""
    c, right, up, out = _screen_axes()
    w, h = SCREEN_SIZE
    c = c + out * 0.0006
    v = np.array([c - right * w / 2 - up * h / 2, c + right * w / 2 - up * h / 2, c + right * w / 2 + up * h / 2, c - right * w / 2 + up * h / 2])
    cols, rows = SCREEN_CELLS
    k = 10 - seconds_left
    u0, u1 = (k % cols) / cols, (k % cols + 1) / cols
    v1, v0 = 1.0 - (k // cols) / rows, 1.0 - (k // cols + 1) / rows
    uv = np.array([[[u0, v0], [u1, v0], [u1, v1]], [[u0, v0], [u1, v1], [u0, v1]]])
    return Mesh(v, np.array([[0, 1, 2], [0, 2, 3]]), uv)


def _screen_parts():
    c, _, _, out_dir = _screen_axes()
    w, h = SCREEN_SIZE
    bezel = box((0.008, w + 0.012, h + 0.012), bevel=0.0015).turned(SCREEN_TILT, "y").moved(c - out_dir * 0.004)
    stand = tube([(FRAME_X, 0.0, FRAME_H - 0.002), c - out_dir * 0.006], 0.009, 12)
    out = [("screen_bezel", bezel, "plastic"), ("screen_stand", stand, "extrusion")]
    for left in range(11):
        out.append((f"screen_{left:02d}", _screen_cell(left), "screen", left != 10))
    return out


def _pins() -> list[tuple[float, float]]:
    """Two pins behind the plate and one at its left side, in the printer's frame."""
    a = PLATE_SIDE / 2 + PIN_GAP + PIN_RADIUS
    return [(a, 0.05), (a, -0.05), (-0.06, a)]


def _printer_colliders(root: str):
    fx, fh, bh = FRAME_X, FRAME_H, BED_SIZE[2]
    out = [collider("c_bed", (0.24, 0.24, bh), (0.0, 0.0, bh / 2), root)]
    for i, (x, y) in enumerate(_pins()):
        out.append(collider(f"c_pin{i}", (2 * PIN_RADIUS, 2 * PIN_RADIUS, PIN_HEIGHT), (x, y, bh + PIN_HEIGHT / 2), root))
    for sgn, name in ((1.0, "left"), (-1.0, "right")):
        out.append(collider(f"c_upright_{name}", (0.02, 0.04, fh), (fx, sgn * 0.175, fh / 2), root))
        out.append(collider(f"c_motor_{name}", (0.042, 0.042, 0.04), (fx - 0.032, sgn * 0.175, 0.02), root))
    out += [
        collider("c_base_bar", (0.02, 0.39, 0.04), (fx, 0.0, 0.02), root),
        collider("c_top_bar", (0.02, 0.39, 0.02), (fx, 0.0, fh - 0.01), root),
        collider("c_gantry", (0.03, 0.33, 0.05), (fx - 0.03, 0.0, 0.19), root),
        collider("c_head", (0.05, 0.06, 0.10), (fx - 0.04, 0.0, 0.175), root),
        collider("c_spool", (0.20, 0.08, 0.21), (0.19, -0.31, 0.105), root),
        collider("c_screen", (0.075, 0.162, 0.075), (FRAME_X + 0.012, 0.0, FRAME_H + 0.036), root),
    ]
    return out


def _rest_parts(look: str):
    sx, sy, sz = REST_SIZE
    out = [("slab", box((sx, sy, sz), (0.0, 0.0, sz / 2), bevel=0.002, round_xy=0.006), look)]
    for i, (x, y) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        out.append((f"bumper{i}", cylinder(0.006, 0.0012, (x * (sx / 2 - 0.012), y * (sy / 2 - 0.012), sz + 0.0006), segments=14), "rubber"))
    return out


# -- spawning ----------------------------------------------------------------------------


def _usd_dir() -> str:
    path = os.path.join(tempfile.gettempdir(), f"swap_print_plate_{os.getpid()}")
    os.makedirs(path, exist_ok=True)
    return path


def _arena_object(spec: PrintPart):
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    d = _usd_dir()
    if spec.kind == "printer":
        usd = write_usda(os.path.join(d, "printer.usda"), "Printer", _printer_parts(), LOOKS, _printer_colliders("Printer"), friction=(0.5, 0.4))
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=(BED_XY[0], BED_XY[1], TOP)))
    if spec.kind == "rest":
        root = spec.name.capitalize()
        sx, sy, sz = REST_SIZE
        usd = write_usda(os.path.join(d, f"{spec.name}.usda"), root, _rest_parts(spec.args["look"]), LOOKS, [collider("c_slab", (sx, sy, sz), (0.0, 0.0, sz / 2), root)],
                         friction=(0.5, 0.4))
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=(*spec.args["xy"], TOP)))
    if spec.kind == "plate":
        root = "".join(w.capitalize() for w in spec.name.split("_"))
        # The lugs are rough for a pinch. The slab's PEI face grips a print as PLA grips PEI once the bed is cold.
        usd = write_usda(os.path.join(d, f"{spec.name}.usda"), root, _plate_parts(), LOOKS, _plate_colliders(root), mass=PLATE_MASS, friction=(1.2, 1.0),
                         extra_materials=physics_material("SlabMaterial", 0.5, 0.45))
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd, initial_pose=Pose(position_xyz=tuple(spec.pos)))
    root = "PrintedPart"
    colliders = [collider("c_foot", (0.06, 0.06, 0.012), (0.0, 0.0, 0.006), root), collider("c_boss", (0.038, 0.038, PRINT_H - 0.012), (0.0, 0.0, 0.012 + (PRINT_H - 0.012) / 2), root)]
    usd = write_usda(os.path.join(d, "print.usda"), root, _print_parts(), LOOKS, colliders, mass=PRINT_MASS, friction=(0.5, 0.45))
    return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd, initial_pose=Pose(position_xyz=tuple(spec.pos)))


# -- the score ---------------------------------------------------------------------------


def _t(x):
    return getattr(x, "torch", x)


def _rotation(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def task_score(early: bool, print_off: bool, dropped: bool, used_off_bed: bool, used_on_grey: bool, clean_first: bool, clean_on_bed: bool, full: bool) -> float:
    """The scoring text, line by line, given the trial's latches and the state at this step."""
    if early or print_off or dropped or not used_off_bed:
        return 0.0
    if not used_on_grey:
        return 0.25
    if clean_first or not clean_on_bed:
        return 0.5
    return 1.0 if full else 0.75


class _Tracker:
    """Runs the lamp and keeps the trial's latches; writes the state to swap_scores/ every 10 steps and when the score changes, because the
    rollout's log keeps only the final score."""

    def __init__(self, env):
        import isaaclab.sim as sim_utils

        self.base = env.unwrapped
        self.stage = sim_utils.get_current_stage()
        self.dt = float(self.base.step_dt)
        self.fingers = {r: (self.base.scene[r].find_bodies("gripper")[0][0], self.base.scene[r].find_joints("joint7")[0][0], self.base.scene[r].find_joints("joint8")[0][0]) for r in ROBOTS}
        self.dir = os.environ.get("LIFT_SCORE_DIR", "swap_scores")
        self.step_file = os.environ.get("LIFT_STEP_FILE")
        self.last_buf = 0
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.early = self.print_off = self.dropped = self.clean_first = False
        self.used_was_on_grey = False
        self.t_lifted = None
        self.start = None
        self.held = 0
        self.marks: list[float] = []
        self.worst_30 = 0.0
        self.trace: list[dict] = []
        self.last_score = None
        self.path = None
        self.green = False
        self.lamp = None
        self.screen = None
        self._show_status(0.0)

    def _show_status(self, t: float) -> None:
        """The lamp's colour and the screen's second for simulated time t; changes only what has changed."""
        from pxr import UsdGeom

        root = "/World/envs/env_0/printer/visual"

        def show(name: str, on: bool) -> None:
            prim = self.stage.GetPrimAtPath(f"{root}/{name}")
            if prim.IsValid():
                UsdGeom.Imageable(prim).MakeVisible() if on else UsdGeom.Imageable(prim).MakeInvisible()

        lamp = "green" if t >= GREEN_AT else "orange" if t >= ORANGE_AT else "red"
        if lamp != self.lamp:
            if self.lamp is None:
                for name in ("red", "orange", "green"):
                    show(f"lens_{name}", name == lamp)
            else:
                show(f"lens_{self.lamp}", False)
                show(f"lens_{lamp}", True)
            self.lamp = lamp
        left = max(0, math.ceil(GREEN_AT - t))
        if left != self.screen:
            if self.screen is None:
                for k in range(11):
                    show(f"screen_{k:02d}", k == left)
            else:
                show(f"screen_{self.screen:02d}", False)
                show(f"screen_{left:02d}", True)
            self.screen = left

    def _body(self, name: str) -> dict:
        s = self.base.scene[name]
        o = self.base.scene.env_origins[0]
        q = _t(s.data.root_quat_w)[0].cpu().numpy().astype(float)
        return {"p": (_t(s.data.root_pos_w)[0] - o).cpu().numpy().astype(float), "q": q, "R": _rotation(q),
                "v": _t(s.data.root_lin_vel_w)[0].cpu().numpy().astype(float), "w": _t(s.data.root_ang_vel_w)[0].cpu().numpy().astype(float)}

    @staticmethod
    def _plate_shape(b: dict) -> dict:
        a = PLATE_SIDE / 2
        corners = b["p"] + (b["R"] @ np.array([[x, y, -PLATE_T / 2] for x in (-a, a) for y in (-a, a)]).T).T
        return {"low": float(corners[:, 2].min()), "high": float(corners[:, 2].max()), "tilt": float(np.degrees(math.acos(min(1.0, abs(b["R"][2, 2])))))}

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
    def _gap_to_plate(points: np.ndarray, b: dict) -> float:
        """The least distance from the points to the plate's box, its lugs included."""
        local = (points - b["p"]) @ b["R"]
        half = np.array([PLATE_SIDE / 2, PLATE_SIDE / 2 + LUG[1], (PLATE_T + LUG_RISE) / 2])
        centre = np.array([0.0, 0.0, LUG_RISE / 2])
        return float(np.linalg.norm(np.maximum(np.abs(local - centre) - half, 0.0), axis=1).min())

    def step(self) -> dict:
        buf = int(self.base.episode_length_buf[0])
        if buf < self.last_buf:
            self.reset()
        self.last_buf = buf
        now = time.monotonic()
        self.marks.append(now)
        if len(self.marks) > 30:
            self.worst_30 = max(self.worst_30, self.marks[-1] - self.marks[-31])
        self.steps += 1
        t = self.steps * self.dt
        self.green = t >= GREEN_AT
        self._show_status(t)
        used, clean, part = self._body("used_plate"), self._body("clean_plate"), self._body("print_part")
        if self.start is None:
            self.start = {"used": used["p"].copy(), "clean": clean["p"].copy()}
        su, sc = self._plate_shape(used), self._plate_shape(clean)
        moved = float(np.linalg.norm(used["p"] - self.start["used"]))
        clean_moved = float(np.linalg.norm(clean["p"] - self.start["clean"]))
        if t < GREEN_AT and moved > MOVED_EARLY:
            self.early = True
        local = used["R"].T @ (part["p"] - used["p"])
        part_tilt = float(np.degrees(math.acos(min(1.0, abs(_rotation(part["q"])[2, 2])))))
        on_plate = bool(abs(local[0]) <= PLATE_SIDE / 2 and abs(local[1]) <= PLATE_SIDE / 2 and -0.003 < local[2] - PLATE_T / 2 < 0.01 and part_tilt < 30.0)
        if not on_plate:
            self.print_off = True
        for b, s in ((used, su), (clean, sc)):
            if s["low"] < TOP + 0.008 or s["tilt"] > 30.0 or b["v"][2] < -0.8:
                self.dropped = True
        off_bed = bool(np.linalg.norm(used["p"][:2] - self.start["used"][:2]) >= OFF_BED or su["low"] >= BED_TOP + LIFTED)
        on_grey = bool(abs(used["p"][0] - GREY_XY[0]) <= 0.02 and abs(used["p"][1] - GREY_XY[1]) <= 0.02 and su["low"] >= REST_TOP - 0.002 and su["high"] <= REST_TOP + 0.002)
        self.used_was_on_grey = self.used_was_on_grey or on_grey
        if clean_moved > 0.02 and not self.used_was_on_grey:
            self.clean_first = True
        centred = bool(abs(clean["p"][0] - BED_XY[0]) <= 0.012 and abs(clean["p"][1] - BED_XY[1]) <= 0.012)
        on_bed = centred and sc["low"] >= BED_TOP - 0.002 and sc["low"] <= BED_TOP + 0.005
        flat = on_bed and sc["high"] <= BED_TOP + 0.002
        fingers = self._finger_points()
        gap = min(self._gap_to_plate(fingers, used), self._gap_to_plate(fingers, clean))
        still = self._still(used) and self._still(clean) and self._still(part)
        ready = bool(on_grey and on_plate and flat and gap >= 0.05 and still and not (self.early or self.print_off or self.dropped or self.clean_first))
        self.held = self.held + 1 if ready else 0
        full = self.held * self.dt >= 1.0
        score = task_score(self.early, self.print_off, self.dropped, off_bed, on_grey, self.clean_first, on_bed, full)
        rep = {"t": round(t, 2), "score": score, "ready": ready, "lamp": self.lamp, "screen_s_left": self.screen, "early": self.early, "print_off": self.print_off, "dropped": self.dropped,
               "clean_first": self.clean_first, "used_off_bed": off_bed, "used_on_grey": on_grey, "clean_on_bed": on_bed, "clean_flat": flat, "gripper_gap_m": round(gap, 3), "still": still,
               "used": [round(float(v), 4) for v in used["p"]], "clean": [round(float(v), 4) for v in clean["p"]], "print": [round(float(v), 4) for v in part["p"]], "print_on_plate_xy": [round(float(v), 4) for v in local[:2]],
               "used_low_high": [round(su["low"], 4), round(su["high"], 4)], "clean_low_high": [round(sc["low"], 4), round(sc["high"], 4)], "used_tilt_deg": round(su["tilt"], 2),
               "slowest_30_steps_s": round(self.worst_30, 3)}
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
    if getattr(scene, "_printkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, PrintPart) else orig_task_object(spec)

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
                out[n] = PrintState(pos=zero, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=zero, ang_vel=zero, low=zero, high=zero, report=rep)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._printkit_patched = True


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
