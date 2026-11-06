"""dropkit: the glass pipette, test tube, cork, rack and holder for this task file, on yam_arena.

A `yam_arena` task file declares rigid primitives. Here the pipette's rubber bulb must give under a gripper, so `DropPart` subclasses `Box` so yam_arena's task-file checks accept it, and
importing this module hooks `yam_arena.scene` (it modifies nothing on disk) so each part spawns from a USD file written at start-up and `object_states` reports the task's state.

The pipette is a free articulation: a stem, and under the rubber two side pads on sprung sliders, which the gripper squeezes. The rubber's shape is drawn from the pads' position. There is no
liquid simulation: one full squeeze and release makes one drop, drawn swelling at the tip and falling. The tube's level and the pipette's level stay as they are, so the scene shows no
count. The cork is a truncated cone that centres itself in the tube's bore and stays by its weight. The held tube shakes from side to side, like a bell, and that also leaves no count in view.
A straight-down plan cannot tilt a grip, so `Lean` carries a tilt through a waypoint's `yaw`, and importing this module wraps `yam_arena.demo.grip_rotation` to read it. It modifies nothing on disk.

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
from usdkit import Look, Mesh, _material_usda, _mesh_usda, box, collider, cylinder, hull_collider, join, lathe, physics_material, write_usda  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
TUBE_XY = (0.22, 0.02)  # the rack's middle hole: both arms reach it, with about 1 cm of height to spare for the left
HOLDER_XY = (0.24, 0.14)
CORK_XY = (0.26, -0.16)  # near the right wrist, out of the left arm's reach

TUBE_OD = 0.02
TUBE_WALL = 0.001
TUBE_H = 0.10
LIQUID_H = 0.07
OVER_TUBE = TUBE_OD / 2 - TUBE_WALL - 0.001  # a tip this near the axis, along x and along y, drops into the bore
TUBE_MASS = 0.045
TUBE_HULL_TOP = 0.06  # round to here, and four flat walls above, to grip and to hold the cork
LABEL_Z = (0.044, 0.068)

CORK_R_TOP = 0.0115
CORK_R_BOTTOM = 0.0075
CORK_H = 0.025
CORK_MASS = 0.005
CORK_SEATS_AT = 0.0094  # above its bottom, where 18 mm of the cone fills the bore: the top of the plug, which lands on the rim

RACK = (0.034, 0.11, 0.04)  # along x, along y, the top plate's top
RACK_PLATE = 0.002
RACK_PITCH = 0.033
RACK_HOLES = (-RACK_PITCH, 0.0, RACK_PITCH)  # along y from the tube
RACK_HOLE_HALF = 0.014
RACK_ROD_R = 0.0025
STAND_BASE = (0.10, 0.07, 0.006)  # the holder's base plate: along x, along y, thick
STAND_BASE_X = 0.012  # its centre past the ring's axis
STAND_POST_X = 0.04  # the post, past the ring's axis
STAND_POST_R = 0.004
RING_TOP = 0.08  # the ring's top above the table
RING_THICK = 0.003
RING_HOLE_R = 0.0075
RING_OUT_R = 0.012

# The pipette, with its origin at the middle of the bulb, where it is gripped, and the tip below.
TIP_BELOW = 0.085
TIP_R = 0.001
STEM_R = 0.0035
TAPER_TOP = 0.06  # where the taper ends, below the origin
BULB_HALF_LEN = 0.015
BULB_R = 0.007
SHOULDER = (0.02, 0.02, 0.002)  # the bulb's rubber flange, which sits on the holder's ring
SHOULDER_Z = -0.016
STEM_MASS = 0.02
PAD_MASS = 0.002
PAD = (0.012, 0.0035, 0.024)  # along x, across the bulb, along the pipette
PAD_TRAVEL = 0.0035
PAD_STIFFNESS = 3000.0  # N/m
PAD_DAMPING = 15.0
LIQUID_TOP = -0.035  # the pipette's column, below the origin

WIDTH_REST = 2 * BULB_R
WIDTH_DROP = 0.008  # a drop falls at this width or less
WIDTH_ARMED = 0.011  # and needs the bulb back at this width or more since the last drop
TILT_OVER = 30.0
TILT_DROPPED = 60.0
TILT_BACK = 10.0
START_Z = TOP + RING_TOP - (SHOULDER_Z - SHOULDER[2] / 2) + 0.0005
ROBOTS = ("left_robot", "right_robot")
TIP_IN_GRIPPER = 0.147
N_DROPS = 3
N_SWINGS = 4
SWING_DEG = 20.0  # a tilt this far from upright counts as reaching a side
SWING_COS = 0.5  # the opposite side: within 60 degrees of straight opposite, so a sideways tilt does not count
TUBE_START_Z = TOP + RACK_PLATE + 0.0005

LOOKS = {
    "glass": Look(color=(0.85, 0.92, 0.92), roughness=0.03, opacity=0.09),
    "liquid": Look(color=(0.015, 0.10, 0.62), roughness=0.12),
    "rubber": Look(color=(0.50, 0.045, 0.03), roughness=0.55, normal_map="rubber_normal.jpg", tile=0.04),
    "steel": Look(color=(0.5, 0.51, 0.53), metallic=0.8, roughness=0.5, normal_map="steel_normal.jpg", tile=0.12),
    "stand": Look(color=(0.02, 0.022, 0.026), roughness=0.5),
    "label": Look(color=(0.92, 0.92, 0.88), roughness=0.8),
    "cork": Look(texture="cork_color.jpg", rough_map="cork_rough.jpg", normal_map="cork_normal.jpg", tile=0.04),
}


@dataclass(frozen=True)
class DropPart(Box):
    """One prim of the scene. A `Box` only so that yam_arena's checks accept it: `pos` and `size` are bounds."""

    kind: str = ""


@dataclass(frozen=True)
class DropState(ObjectState):
    report: dict = field(default_factory=dict)


class Lean(float):
    """A waypoint's yaw of 0 that also tilts the grip by `degrees` about a horizontal `axis` through the TCP."""

    axis: tuple[float, float, float] = (1.0, 0.0, 0.0)
    degrees: float = 0.0


def lean(axis, degrees: float) -> Lean:
    out = Lean(0.0)
    out.axis, out.degrees = tuple(float(v) for v in axis), float(degrees)
    return out


def _wrap_grip_rotation() -> None:
    import yam_arena.demo as demo
    from yam_arena.urdf import axis_angle_matrix

    if getattr(demo.grip_rotation, "_reads_lean", False):
        return
    plain = demo.grip_rotation

    def grip_rotation(yaw=0.0):
        if isinstance(yaw, Lean):
            return axis_angle_matrix(np.array(yaw.axis), math.radians(yaw.degrees)) @ plain(0.0)
        return plain(yaw)

    grip_rotation._reads_lean = True
    demo.grip_rotation = grip_rotation


def parts() -> list[DropPart]:
    """The task file's OBJECTS."""
    return [
        DropPart("rack", pos=(*TUBE_XY, TOP + TUBE_H / 2), size=(RACK[0], RACK[1], TUBE_H), kind="rack"),
        DropPart("holder", pos=(HOLDER_XY[0] + STAND_BASE_X, HOLDER_XY[1], TOP + RING_TOP / 2), size=(STAND_BASE[0], STAND_BASE[1], RING_TOP), kind="holder"),
        DropPart("pipette", pos=(*HOLDER_XY, START_Z), size=(2 * BULB_R, 2 * BULB_R, TIP_BELOW + BULB_HALF_LEN), kind="pipette"),
        DropPart("tube", pos=(*TUBE_XY, TUBE_START_Z), size=(TUBE_OD, TUBE_OD, TUBE_H), kind="tube"),
        DropPart("cork", pos=(*CORK_XY, TOP + CORK_H / 2 + 0.0005), size=(2 * CORK_R_TOP, 2 * CORK_R_TOP, CORK_H), kind="cork"),
    ]


_NAMES = {p.name for p in parts()}


# -- the models ----------------------------------------------------------------------------


def _bulb_mesh(state: int) -> Mesh:
    """The rubber bulb, more squeezed at higher states: flatter across the bulb, wider along x."""
    h = BULB_HALF_LEN
    r = BULB_R
    prof = [(STEM_R + 0.0008, -h), (r * 0.82, -h + 0.002), (r, -h + 0.006), (r, h - 0.008), (r * 0.9, h - 0.0035), (r * 0.55, h - 0.0008), (0.0, h)]
    across = {0: 1.0, 1: 0.86, 2: 0.74, 3: 0.62, 4: 0.5}[state]
    along = 1.0 + (1.0 - across) * 0.35
    m = lathe(prof, 24)
    return Mesh(m.v * np.array([along, across, 1.0]), m.f)


def _bead(radius: float, z: float) -> Mesh:
    prof = [(0.0, -radius)] + [(radius * math.cos(a), radius * math.sin(a)) for a in np.radians((-60, -30, 0, 30, 60))] + [(0.0, radius)]
    return lathe(prof, 12).moved((0.0, 0.0, z))


def _pipette_parts():
    """What is drawn on the stem. Bulb states and drops start hidden, apart from the bulb at rest."""
    z_tip = -TIP_BELOW
    z_taper = -TAPER_TOP
    z_top = -BULB_HALF_LEN + 0.004
    glass = lathe([(0.0, z_tip), (TIP_R, z_tip), (STEM_R, z_taper), (STEM_R, z_top), (0.0, z_top)], 18)
    inner = 0.0008
    liquid = lathe([(0.0, z_tip + 0.001), (max(TIP_R - inner, 0.0004), z_tip + 0.001), (STEM_R - inner, z_taper), (STEM_R - inner, LIQUID_TOP), (0.0, LIQUID_TOP)], 14)
    flange = cylinder(SHOULDER[0] / 2, SHOULDER[2], (0.0, 0.0, SHOULDER_Z), segments=24, bevel=0.0005)
    out = [("glass", glass, "glass"), ("column", liquid, "liquid"), ("flange", flange, "rubber")]
    for s in range(5):
        out.append((f"bulb_{s}", _bulb_mesh(s), "rubber", s != 0))
    out.append(("bead_hang0", _bead(0.0012, z_tip - 0.0009), "liquid", True))
    out.append(("bead_hang1", _bead(0.0021, z_tip - 0.0018), "liquid", True))
    out.append(("bead_fall0", _bead(0.0023, z_tip - 0.012), "liquid", True))
    out.append(("bead_fall1", _bead(0.0023, z_tip - 0.049), "liquid", True))
    return out


def _joint(name: str, pad: str, y0: float, lower: float, upper: float, root: str) -> str:
    # A joint without a DriveAPI gets no drive in PhysX, and the actuator's gains then have no effect.
    return f"""
    def PhysicsPrismaticJoint "{name}" (
        prepend apiSchemas = ["PhysicsDriveAPI:linear"]
    )
    {{
        uniform token drive:linear:physics:type = "force"
        float drive:linear:physics:stiffness = {PAD_STIFFNESS}
        float drive:linear:physics:damping = {PAD_DAMPING}
        float drive:linear:physics:maxForce = 50
        float drive:linear:physics:targetPosition = 0
        rel physics:body0 = </{root}/stem>
        rel physics:body1 = </{root}/{pad}>
        uniform token physics:axis = "Y"
        point3f physics:localPos0 = (0, {y0:.6f}, 0)
        quatf physics:localRot0 = (1, 0, 0, 0)
        point3f physics:localPos1 = (0, 0, 0)
        quatf physics:localRot1 = (1, 0, 0, 0)
        float physics:lowerLimit = {lower:.6f}
        float physics:upperLimit = {upper:.6f}
    }}
"""


def write_pipette_usda(path: str) -> str:
    root = "Pipette"
    meshes = _pipette_parts()
    materials = "".join(_material_usda(m, root, LOOKS[m]) for m in sorted({p[2] for p in meshes}))
    drawn = "".join(_mesh_usda(p[0], p[1], p[2], root, LOOKS[p[2]], 35.0, indent="            ", hidden=len(p) > 3 and p[3]) for p in meshes)
    z_wide = (-TAPER_TOP + SHOULDER_Z + SHOULDER[2] / 2) / 2
    h_wide = (SHOULDER_Z + SHOULDER[2] / 2) - (-TAPER_TOP)
    colliders = "".join((
        collider("c_tip", (0.003, 0.003, TIP_BELOW - TAPER_TOP), (0.0, 0.0, -(TIP_BELOW + TAPER_TOP) / 2), root, contact_offset=0.002),
        collider("c_stem", (2 * STEM_R, 2 * STEM_R, h_wide), (0.0, 0.0, z_wide), root, contact_offset=0.002),
        collider("c_shoulder", SHOULDER, (0.0, 0.0, SHOULDER_Z), root, contact_offset=0.002),
    ))
    y_pad = BULB_R - PAD[1] / 2
    text = f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}"
{{
{physics_material("PhysicsMaterial", 1.2, 1.0)}

    def Scope "Looks"
    {{{materials}
    }}

    def Xform "stem" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI", "PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
    )
    {{
        float physics:mass = {STEM_MASS}
        bool physxArticulation:enabledSelfCollisions = 0
        int physxArticulation:solverPositionIterationCount = 32
        int physxArticulation:solverVelocityIterationCount = 4
        float physxRigidBody:maxDepenetrationVelocity = 1
{colliders}

        def Xform "visual"
        {{{drawn}
        }}
    }}

    def Xform "pad_left" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
    )
    {{
        float physics:mass = {PAD_MASS}
        float physxRigidBody:maxDepenetrationVelocity = 1
        double3 xformOp:translate = (0, {y_pad:.6f}, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
{collider("c_pad", PAD, (0.0, 0.0, 0.0), root, contact_offset=0.002)}
    }}

    def Xform "pad_right" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
    )
    {{
        float physics:mass = {PAD_MASS}
        float physxRigidBody:maxDepenetrationVelocity = 1
        double3 xformOp:translate = (0, {-y_pad:.6f}, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
{collider("c_pad", PAD, (0.0, 0.0, 0.0), root, contact_offset=0.002)}
    }}
{_joint("pad_left_joint", "pad_left", y_pad, -PAD_TRAVEL, 0.0, root)}
{_joint("pad_right_joint", "pad_right", -y_pad, 0.0, PAD_TRAVEL, root)}
}}
"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as out:
        out.write(text)
    return path


def _holed_plate(sx: float, t: float, z0: float, ys, pitch: float, r: float, seg: int = 28) -> Mesh:
    """A plate across x with round holes through it, one per cell of `pitch` along y, from z0 up by t."""
    hx, hy = sx / 2, pitch / 2
    corners = [math.atan2(s * hy, c * hx) % (2 * math.pi) for c, s in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    angles = sorted({round(k * 2 * math.pi / seg, 9) for k in range(seg)} | {round(a, 9) for a in corners})
    v, f = [], []
    for yc in ys:
        n0 = len(v)
        for a in angles:
            c, s = math.cos(a), math.sin(a)
            scale = min(hx / max(abs(c), 1e-9), hy / max(abs(s), 1e-9))
            for z in (z0 + t, z0):
                v.append((r * c, yc + r * s, z))
                v.append((scale * c, yc + scale * s, z))
        m = len(angles)
        for k in range(m):
            i, j = n0 + 4 * k, n0 + 4 * ((k + 1) % m)
            it, ot, ib, ob = i, i + 1, i + 2, i + 3
            jt, pt, jb, pb = j, j + 1, j + 2, j + 3
            f += [(it, ot, pt), (it, pt, jt), (ib, pb, ob), (ib, jb, pb), (it, jt, jb), (it, jb, ib)]
    y0, y1 = min(ys) - hy, max(ys) + hy
    n0 = len(v)
    v += [(hx, y0, z0), (hx, y1, z0), (hx, y1, z0 + t), (hx, y0, z0 + t), (-hx, y0, z0), (-hx, y1, z0), (-hx, y1, z0 + t), (-hx, y0, z0 + t)]
    f += [(n0, n0 + 1, n0 + 2), (n0, n0 + 2, n0 + 3), (n0 + 4, n0 + 6, n0 + 5), (n0 + 4, n0 + 7, n0 + 6)]
    return Mesh(np.array(v, float), np.array(f))


def _rack_parts():
    """An open steel test-tube rack, origin on the table under the middle hole: a thin top plate with holes, on four rods, over a base plate."""
    sx, sy, sz = RACK
    t = RACK_PLATE
    ends = (sy - 3 * RACK_PITCH) / 2
    y_end = 1.5 * RACK_PITCH + ends / 2
    plate = join(_holed_plate(sx, t, sz - t, RACK_HOLES, RACK_PITCH, RACK_HOLE_HALF),
                 box((sx, ends, t), (0.0, y_end, sz - t / 2)), box((sx, ends, t), (0.0, -y_end, sz - t / 2)))
    out = [("base", box((sx, sy, t), (0.0, 0.0, t / 2), bevel=0.0004), "steel"), ("top", plate, "steel")]
    for i, (a, b) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        x, y = a * (sx / 2 - 0.004), b * (sy / 2 - 0.004)
        out.append((f"rod{i}", cylinder(RACK_ROD_R, sz, (x, y, sz / 2), segments=12), "steel"))
    return out


def _boxes_minus_holes(prefix: str, x0, x1, y0, y1, holes, half, z0, z1, root: str, contact_offset: float = 0.002) -> list[str]:
    """Box colliders covering a rectangle, leaving square holes."""
    xs = sorted({x0, x1, *(hx - half for hx, _ in holes), *(hx + half for hx, _ in holes)})
    ys = sorted({y0, y1, *(hy - half for _, hy in holes), *(hy + half for _, hy in holes)})
    out = []
    for i, (a, b) in enumerate(zip(xs[:-1], xs[1:])):
        for j, (c, d) in enumerate(zip(ys[:-1], ys[1:])):
            mx, my = (a + b) / 2, (c + d) / 2
            if any(abs(mx - hx) < half and abs(my - hy) < half for hx, hy in holes):
                continue
            out.append(collider(f"{prefix}_{i}_{j}", (b - a, d - c, z1 - z0), (mx, my, (z0 + z1) / 2), root, contact_offset=contact_offset))
    return out


def _rack_colliders(root: str):
    sx, sy, sz = RACK
    t = RACK_PLATE
    holes = [(0.0, dy) for dy in RACK_HOLES]
    out = [collider("c_base", (sx, sy, t), (0.0, 0.0, t / 2), root, contact_offset=0.002)]
    out += _boxes_minus_holes("c_top", -sx / 2, sx / 2, -sy / 2, sy / 2, holes, RACK_HOLE_HALF, sz - t, sz, root)
    for i, (a, b) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        out.append(collider(f"c_rod{i}", (2 * RACK_ROD_R, 2 * RACK_ROD_R, sz), (a * (sx / 2 - 0.004), b * (sy / 2 - 0.004), sz / 2), root, contact_offset=0.002))
    return out


def _tube_profile():
    r_out, h = TUBE_OD / 2, TUBE_H
    r_in = r_out - TUBE_WALL
    return r_out, r_in, h


def _tube_parts():
    """The clear glass tube, origin under its middle, with its chemical and a paper label round part of it, facing the robot."""
    r_out, r_in, h = _tube_profile()
    tube = lathe([(0.0, 0.0), (r_out * 0.72, 0.001), (r_out, r_out * 0.72), (r_out, h - 0.002), (r_out + 0.0007, h - 0.0008), (r_out + 0.0007, h), (r_in, h), (r_in, r_out * 0.8), (r_in * 0.7, 0.0022), (0.0, 0.0012)], 32)
    liquid = lathe([(0.0, 0.0013), (r_in * 0.68, 0.0023), (r_in - 0.0002, r_out * 0.8), (r_in - 0.0002, LIQUID_H), (0.0, LIQUID_H)], 24)
    arc = np.radians(np.linspace(110.0, 250.0, 15))
    r = r_out + 0.0002
    v = [(r * math.cos(a), r * math.sin(a), z) for a in arc for z in LABEL_Z]
    f = []
    for k in range(len(arc) - 1):
        i = 2 * k
        f += [(i, i + 2, i + 3), (i, i + 3, i + 1)]
    label = Mesh(np.array(v), np.array(f))
    return [("tube", tube, "glass"), ("tube_liquid", liquid, "liquid"), ("label", label, "label")]


def _tube_colliders(root: str):
    r_out, r_in, h = _tube_profile()
    z0 = TUBE_HULL_TOP
    hz = h - z0
    z = z0 + hz / 2
    w = r_out - r_in
    hull = lathe([(0.0, 0.0), (r_out * 0.9, 0.0), (r_out, 0.003), (r_out, z0), (0.0, z0)], 16)  # a flat bottom, so that it stands upright in the rack
    return [
        hull_collider("c_lower", hull, root),
        collider("c_wall_a", (w, 2 * r_out, hz), (r_out - w / 2, 0.0, z), root, contact_offset=0.002),
        collider("c_wall_b", (w, 2 * r_out, hz), (-r_out + w / 2, 0.0, z), root, contact_offset=0.002),
        collider("c_wall_c", (2 * r_in, w, hz), (0.0, r_out - w / 2, z), root, contact_offset=0.002),
        collider("c_wall_d", (2 * r_in, w, hz), (0.0, -r_out + w / 2, z), root, contact_offset=0.002),
    ]


def _cork_colliders(root: str):
    """Two boxes, a plug in the bore and a top that lands flat on the rim. One convex hull of the cone would wedge in the bore and never come to rest."""
    h = CORK_H / 2
    plug = CORK_SEATS_AT
    top = CORK_H - plug
    return [
        collider("c_plug", (0.012, 0.012, plug), (0.0, 0.0, -h + plug / 2), root, contact_offset=0.002),
        collider("c_top", (0.0206, 0.0206, top), (0.0, 0.0, -h + plug + top / 2), root, contact_offset=0.002),
    ]


def _cork_mesh() -> Mesh:
    """A cork, a truncated cone, origin at its middle, its narrow end down."""
    h = CORK_H / 2
    return lathe([(0.0, -h), (CORK_R_BOTTOM, -h), (CORK_R_TOP, h), (0.0, h)], 24)


def _holder_parts():
    """The holder, a lab stand, origin on the table under the ring's axis: a ring on a post over a black base plate. The pipette's flange rests on the ring, and its stem hangs in the open."""
    sx, sy, sz = STAND_BASE
    z0 = RING_TOP - RING_THICK
    ring = lathe([(RING_HOLE_R, z0), (RING_OUT_R, z0), (RING_OUT_R, RING_TOP), (RING_HOLE_R, RING_TOP), (RING_HOLE_R, z0)], 32)
    arm_len = STAND_POST_X - RING_OUT_R + 0.001
    return [
        ("base", box(STAND_BASE, (STAND_BASE_X, 0.0, sz / 2), bevel=0.001, round_xy=0.006), "stand"),
        ("post", cylinder(STAND_POST_R, RING_TOP - sz, (STAND_POST_X, 0.0, (RING_TOP + sz) / 2), segments=16), "steel"),
        ("ring", ring, "steel"),
        ("arm", box((arm_len, 0.006, RING_THICK), (RING_OUT_R - 0.001 + arm_len / 2, 0.0, RING_TOP - RING_THICK / 2)), "steel"),
    ]


def _holder_colliders(root: str):
    sx, sy, sz = STAND_BASE
    a = RING_HOLE_R
    o = RING_OUT_R
    side = o - a
    z = RING_TOP - RING_THICK / 2
    t = RING_THICK
    return [
        collider("c_base", STAND_BASE, (STAND_BASE_X, 0.0, sz / 2), root, contact_offset=0.002),
        collider("c_post", (2 * STAND_POST_R, 2 * STAND_POST_R, RING_TOP - sz), (STAND_POST_X, 0.0, (RING_TOP + sz) / 2), root, contact_offset=0.002),
        collider("c_arm", (STAND_POST_X - o, 0.006, t), ((STAND_POST_X + o) / 2, 0.0, z), root, contact_offset=0.002),
        collider("c_a", (side, 2 * o, t), (a + side / 2, 0.0, z), root, contact_offset=0.002),
        collider("c_b", (side, 2 * o, t), (-a - side / 2, 0.0, z), root, contact_offset=0.002),
        collider("c_c", (2 * a, side, t), (0.0, a + side / 2, z), root, contact_offset=0.002),
        collider("c_d", (2 * a, side, t), (0.0, -a - side / 2, z), root, contact_offset=0.002),
    ]


# -- spawning ----------------------------------------------------------------------------


def _usd_dir() -> str:
    path = os.path.join(tempfile.gettempdir(), f"count_drops_{os.getpid()}")
    os.makedirs(path, exist_ok=True)
    return path


def _arena_object(spec: DropPart):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    path = os.path.join(_usd_dir(), f"{spec.name}.usda")
    if spec.kind == "rack":
        usd = write_usda(path, "TubeRack", _rack_parts(), LOOKS, _rack_colliders("TubeRack"), friction=(0.5, 0.4))
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=(*TUBE_XY, TOP)))
    if spec.kind == "holder":
        usd = write_usda(path, "Holder", _holder_parts(), LOOKS, _holder_colliders("Holder"), friction=(0.5, 0.4))
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=Pose(position_xyz=(*HOLDER_XY, TOP)))

    if spec.kind == "tube":
        usd = write_usda(path, "TestTube", _tube_parts(), LOOKS, _tube_colliders("TestTube"), mass=TUBE_MASS, friction=(1.0, 0.8), angular_damping=5.0)
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd, initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos)))
    if spec.kind == "cork":
        usd = write_usda(path, "Cork", [("cork", _cork_mesh(), "cork")], LOOKS, _cork_colliders("Cork"), mass=CORK_MASS, friction=(1.0, 0.8), angular_damping=5.0)
        return Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd, initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos)))
    usd = write_pipette_usda(path)
    actuators = {"pads": ImplicitActuatorCfg(joint_names_expr=["pad_.*"], stiffness=PAD_STIFFNESS, damping=PAD_DAMPING, joint_effort_limit=50.0, armature=1e-5)}

    class Sprung(Object):  # Arena's own articulation object has no actuators
        def _generate_articulation_cfg(self):
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self._get_spawn_cfg(activate_contact_sensors=False),
                                  init_state=ArticulationCfg.InitialStateCfg(joint_pos={".*": 0.0}), actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return Sprung(name=spec.name, object_type=ObjectType.ARTICULATION, usd_path=usd, initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos)))


# -- the score ---------------------------------------------------------------------------


def _t(x):
    return getattr(x, "torch", x)


def _rotation(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def task_score(failed: bool, brought_over: bool, in_tube: int, back: bool, cork_ok: bool, lifted_ok: bool, swings: int, in_rack: bool, full: bool) -> float:
    """The scoring text, line by line, given the trial's latches and the state at this step."""
    if failed or not brought_over:
        return 0.0
    if in_tube < N_DROPS:
        return 0.1
    if not back:
        return 0.25
    if not cork_ok:
        return 0.4
    if not lifted_ok:
        return 0.55
    if swings != N_SWINGS:
        return 0.7
    if not in_rack:
        return 0.85
    return 1.0 if full else 0.95


class _Tracker:
    """Counts the drops, draws the bulb and drops, and keeps the trial's latches; writes the state to drop_scores/ every 10 steps and when the score changes, because the rollout's log
    keeps only the final score."""

    def __init__(self, env):
        import isaaclab.sim as sim_utils

        self.base = env.unwrapped
        self.stage = sim_utils.get_current_stage()
        self.dt = float(self.base.step_dt)
        self.fingers = {r: (self.base.scene[r].find_bodies("gripper")[0][0], self.base.scene[r].find_joints("joint7")[0][0], self.base.scene[r].find_joints("joint8")[0][0]) for r in ROBOTS}
        pipette = self.base.scene["pipette"]
        self.j_left, self.j_right = pipette.find_joints("pad_left_joint")[0][0], pipette.find_joints("pad_right_joint")[0][0]
        self.dir = os.environ.get("LIFT_SCORE_DIR", "drop_scores")
        self.step_file = os.environ.get("LIFT_STEP_FILE")
        self.last_buf = 0
        self.reset()

    def reset(self) -> None:
        self.steps = 0
        self.armed = True
        self.in_tube = 0
        self.spilled = 0
        self.fourth = False
        self.dropped = False
        self.brought_over = False
        self.tube_dropped = False
        self.cork_lost = False
        self.cork_was_seated = False
        self.cork_counts = False
        self.early_lift = False
        self.lifted_ok = False
        self.was_lifted = False
        self.lift_counts = False
        self.first_side = None
        self.at_first_side = False
        self.through_upright = False
        self.swings = 0
        self.swing_times: list[float] = []
        self.drops: list[dict] = []
        self.held = 0
        self.marks: list[float] = []
        self.worst_30 = 0.0
        self.trace: list[dict] = []
        self.last_score = None
        self.path = None
        self.shown = {"bulb": 0, "bead": None}
        self.bead_left = 0

    def _show(self, name: str, on: bool) -> None:
        from pxr import UsdGeom

        prim = self.stage.GetPrimAtPath(f"/World/envs/env_0/pipette/stem/visual/{name}")
        if prim.IsValid():
            UsdGeom.Imageable(prim).MakeVisible() if on else UsdGeom.Imageable(prim).MakeInvisible()

    def _draw(self, width: float, fell: bool) -> None:
        """The bulb's shape from its width, and a drop: swelling while the bulb is squeezed, then falling for two frames."""
        state = 0 if width >= 0.0125 else 1 if width >= WIDTH_ARMED else 2 if width >= 0.0095 else 3 if width >= WIDTH_DROP else 4
        if state != self.shown["bulb"]:
            self._show(f"bulb_{self.shown['bulb']}", False)
            self._show(f"bulb_{state}", True)
            self.shown["bulb"] = state
        if fell:
            self.bead_left = 2
        if self.bead_left:
            bead = "bead_fall0" if self.bead_left == 2 else "bead_fall1"
            self.bead_left -= 1
        elif self.armed and width < 0.0095:
            bead = "bead_hang1"
        elif self.armed and width < WIDTH_ARMED:
            bead = "bead_hang0"
        else:
            bead = None
        if bead != self.shown["bead"]:
            if self.shown["bead"]:
                self._show(self.shown["bead"], False)
            if bead:
                self._show(bead, True)
            self.shown["bead"] = bead

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
    def _gap_to_box(points: np.ndarray, p, R, half, centre=(0.0, 0.0, 0.0)) -> float:
        """The least distance from the points to the part's box."""
        local = (points - p) @ R - np.asarray(centre)
        return float(np.linalg.norm(np.maximum(np.abs(local) - np.asarray(half), 0.0), axis=1).min())

    def _pose(self, name: str):
        s = self.base.scene[name]
        o = self.base.scene.env_origins[0]
        p = (_t(s.data.root_pos_w)[0] - o).cpu().numpy().astype(float)
        R = _rotation(_t(s.data.root_quat_w)[0].cpu().numpy().astype(float))
        v = _t(s.data.root_lin_vel_w)[0].cpu().numpy().astype(float)
        w = _t(s.data.root_ang_vel_w)[0].cpu().numpy().astype(float)
        return p, R, v, w

    @staticmethod
    def _tilt(R: np.ndarray) -> float:
        return float(np.degrees(math.acos(min(1.0, abs(R[2, 2]))))) if R[2, 2] >= 0.0 else 180.0

    def _one_hand(self, pts: np.ndarray, p, R, half, centre) -> bool:
        """Both fingertips of one gripper within 1 cm of the box."""
        return any(max(self._gap_to_box(pts[i:i + 1], p, R, half, centre), self._gap_to_box(pts[i + 1:i + 2], p, R, half, centre)) <= 0.01 for i in (0, 2))

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
        s = self.base.scene["pipette"]
        o = self.base.scene.env_origins[0]
        p = (_t(s.data.root_pos_w)[0] - o).cpu().numpy().astype(float)
        R = _rotation(_t(s.data.root_quat_w)[0].cpu().numpy().astype(float))
        v = _t(s.data.root_lin_vel_w)[0].cpu().numpy().astype(float)
        w = _t(s.data.root_ang_vel_w)[0].cpu().numpy().astype(float)
        q = _t(s.data.joint_pos)[0]
        press_left, press_right = abs(float(q[self.j_left])), abs(float(q[self.j_right]))
        width = WIDTH_REST - press_left - press_right
        tilt = float(np.degrees(math.acos(min(1.0, abs(R[2, 2]))))) if R[2, 2] >= 0.0 else 180.0
        tip = p + R @ np.array([0.0, 0.0, -TIP_BELOW])
        tp, tR, tv, tw = self._pose("tube")
        cp, cR, cv, cw = self._pose("cork")
        ttilt = self._tilt(tR)
        rim = tp + tR @ np.array([0.0, 0.0, TUBE_H])
        over = bool(abs(tip[0] - tp[0]) <= OVER_TUBE and abs(tip[1] - tp[1]) <= OVER_TUBE and tilt <= TILT_OVER)
        cork_local = (cp - rim) @ tR
        cork_top_over_rim = float(cork_local[2]) + CORK_H / 2
        cork_seated = bool(abs(cork_local[0]) <= 0.005 and abs(cork_local[1]) <= 0.005 and 0.01 <= cork_top_over_rim <= 0.02 and float(cR[:, 2] @ tR[:, 2]) >= math.cos(math.radians(15.0)))
        if not over or cork_seated:
            over_for_drop = False
        else:
            over_for_drop = True
        fell = False
        if self.armed and width <= WIDTH_DROP:
            self.armed = False
            fell = True
            if over_for_drop:
                self.in_tube += 1
            else:
                self.spilled += 1
            self.drops.append({"t": round(t, 2), "in_tube": over_for_drop, "tip": [round(float(v_), 4) for v_ in tip]})
        elif not self.armed and width >= WIDTH_ARMED:
            self.armed = True
        if self.in_tube > N_DROPS:
            self.fourth = True
        pts = self._finger_points()
        g, _, _ = self.fingers["left_robot"]
        hand_R = _rotation(_t(self.base.scene["left_robot"].data.body_link_quat_w)[0, g].cpu().numpy().astype(float))
        hand_tilt = float(np.degrees(math.acos(min(1.0, abs(hand_R[2, 2])))))
        half, centre = (BULB_R, BULB_R, BULB_HALF_LEN), (0.0, 0.0, 0.0)
        hand = self._one_hand(pts, p, R, half, centre)
        if over and hand:
            self.brought_over = True
        in_holder_xy = bool(abs(tip[0] - HOLDER_XY[0]) <= 0.02 and abs(tip[1] - HOLDER_XY[1]) <= 0.02)
        if tilt > TILT_DROPPED or v[2] < -0.8 or (tip[2] < TOP + 0.01 and not in_holder_xy):
            self.dropped = True
        back = bool(abs(tip[0] - HOLDER_XY[0]) <= 0.005 and abs(tip[1] - HOLDER_XY[1]) <= 0.005 and tip[2] <= TOP + STAND_BASE[2] + 0.02 and tilt <= TILT_BACK)
        # The tube, its cork and its shakes.
        in_rack_xy = bool(abs(tp[0] - TUBE_XY[0]) <= RACK[0] / 2 and abs(tp[1] - TUBE_XY[1]) <= RACK[1] / 2)
        if ttilt > TILT_DROPPED or tv[2] < -0.8 or (tp[2] < TOP + 0.01 and not in_rack_xy):
            self.tube_dropped = True
        if cork_seated and not self.cork_was_seated:
            self.cork_was_seated = True
            self.cork_counts = back and self.in_tube == N_DROPS
        if self.cork_was_seated and np.linalg.norm(cp - rim) > 0.03:
            self.cork_lost = True
        tube_half, tube_centre = (TUBE_OD / 2, TUBE_OD / 2, TUBE_H / 2), (0.0, 0.0, TUBE_H / 2)
        tube_hand = self._one_hand(pts, tp, tR, tube_half, tube_centre)
        lifted = bool(tp[2] >= TOP + RACK[2] + 0.01 and tube_hand)
        if lifted and not self.was_lifted:
            self.lift_counts = bool(cork_seated and self.cork_counts)
            if self.lift_counts:
                self.lifted_ok = True
            elif not self.lifted_ok:
                self.early_lift = True
            self.first_side = None
            self.at_first_side = False
            self.through_upright = False
        self.was_lifted = lifted
        if lifted and self.lift_counts and cork_seated:
            if ttilt < SWING_DEG:
                self.through_upright = True
            else:
                side = tR[:2, 2] / max(float(np.linalg.norm(tR[:2, 2])), 1e-9)
                if self.first_side is None:
                    self.first_side = side
                    self.at_first_side = True
                    self.through_upright = False
                facing = float(side @ self.first_side)
                if not self.at_first_side and facing >= SWING_COS and self.through_upright:
                    self.at_first_side = True
                    self.through_upright = False
                elif self.at_first_side and facing <= -SWING_COS and self.through_upright:
                    self.at_first_side = False
                    self.through_upright = False
                    self.swings += 1
                    self.swing_times.append(round(t, 2))
        in_rack = bool(abs(tp[0] - TUBE_XY[0]) <= 0.005 and abs(tp[1] - TUBE_XY[1]) <= 0.005 and tp[2] <= TOP + RACK_PLATE + 0.01 and ttilt <= 10.0)
        failed = bool(self.spilled or self.fourth or self.dropped or self.tube_dropped or self.cork_lost)
        cork_half = (CORK_R_TOP, CORK_R_TOP, CORK_H / 2)
        gap = min(self._gap_to_box(pts, p, R, (BULB_R, BULB_R, (TIP_BELOW + BULB_HALF_LEN) / 2), (0.0, 0.0, (BULB_HALF_LEN - TIP_BELOW) / 2)),
                  self._gap_to_box(pts, tp, tR, tube_half, tube_centre), self._gap_to_box(pts, cp, cR, cork_half))
        still = all(np.linalg.norm(a) < 0.01 and np.linalg.norm(b) < 0.5 for a, b in ((v, w), (tv, tw), (cv, cw)))
        cork_ok = bool(cork_seated and self.cork_counts)
        lifted_ok = bool(self.lifted_ok and not self.early_lift)
        ready = bool(not failed and self.brought_over and self.in_tube == N_DROPS and back and cork_ok and lifted_ok and self.swings == N_SWINGS and in_rack and gap >= 0.05 and still)
        self.held = self.held + 1 if ready else 0
        full = self.held * self.dt >= 1.0
        score = task_score(failed, self.brought_over, self.in_tube, back, cork_ok, lifted_ok, self.swings, in_rack, full)
        self._draw(width, fell)
        rep = {"t": round(t, 2), "score": score, "ready": ready, "in_tube": self.in_tube, "spilled": self.spilled, "fourth": self.fourth, "dropped": self.dropped, "brought_over": self.brought_over,
               "back": back, "over": over, "hand": hand, "width_mm": round(width * 1000, 2), "press_mm": [round(press_left * 1000, 2), round(press_right * 1000, 2)], "tilt_deg": round(tilt, 2), "left_hand_tilt_deg": round(hand_tilt, 2),
               "tip": [round(float(x), 4) for x in tip], "pipette": [round(float(x), 4) for x in p], "gripper_gap_m": round(gap, 3), "still": still, "speeds": [round(float(np.linalg.norm(a)), 4) for a in (v, w, tv, tw, cv, cw)], "tube": [round(float(x), 4) for x in tp], "tube_tilt_deg": round(ttilt, 2), "cork": [round(float(x), 4) for x in cp], "cork_top_over_rim_mm": round(cork_top_over_rim * 1000, 1),
               "cork_seated": cork_seated, "cork_counts": self.cork_counts, "lifted": lifted, "tube_hand": tube_hand, "lifted_ok": lifted_ok, "early_lift": self.early_lift, "tube_dropped": self.tube_dropped, "cork_lost": self.cork_lost,
               "swings": self.swings, "swing_times": self.swing_times, "in_rack": in_rack, "drops": self.drops, "slowest_30_steps_s": round(self.worst_30, 3)}
        if self.step_file:
            with open(self.step_file, "w") as f:
                f.write(f"{t:.2f}")
        if self.steps % 10 == 0 or self.last_score != score or fell or self.swings != getattr(self, "swings_logged", 0):
            self.swings_logged = self.swings
            self.trace.append({k: rep[k] for k in rep if k not in ("drops", "swing_times")})
            os.makedirs(self.dir, exist_ok=True)
            self.path = self.path or os.path.join(self.dir, f"score_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json")
            with open(self.path, "w") as f:
                json.dump({"final": rep, "trace": self.trace}, f, indent=0)
        self.last_score = score
        return rep


_tracker: _Tracker | None = None


def _patch_scene(scene) -> None:
    if getattr(scene, "_dropkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        return _arena_object(spec) if isinstance(spec, DropPart) else orig_task_object(spec)

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
                out[n] = DropState(pos=zero, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=zero, ang_vel=zero, low=zero, high=zero, report=rep)
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._dropkit_patched = True


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
    """Hook yam_arena.scene (now if already imported, else when it is), and let a plan tilt its grip. Idempotent."""
    _wrap_grip_rotation()
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
