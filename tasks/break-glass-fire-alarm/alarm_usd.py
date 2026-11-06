"""The fire alarm on its wall section, and the fire, written as plain-text USD (no pxr needed). All of it is ours; there are no third-party meshes.

Coordinates are metres on the table: x forward, y left, z above the table. The wall section's front slopes at 30 degrees, so a straight-down fingertip can reach the box and the top camera sees its face square on.
The glass has no collider: it is seen until a fingertip goes through it, and then the shards are seen. The box's floor and the raised frame round the window are solid.
"""

from __future__ import annotations

import math
import os
import pathlib
from dataclasses import dataclass

import numpy as np
from alarm_mesh import Mesh, block, lathe, tube, usd_mesh

HERE = pathlib.Path(__file__).resolve().parent


@dataclass(frozen=True)
class Look:
    color: tuple[float, float, float] = (0.5, 0.5, 0.5)
    metallic: float = 0.0
    roughness: float = 0.5
    color_file: str = ""
    rough_file: str = ""
    normal_file: str = ""
    opacity: float = 1.0
    ior: float = 1.5
    glow: tuple[float, float, float] = (0.0, 0.0, 0.0)
    shine: float = 0.5  # 0: no highlight at all, as smoke has none


LOOKS = {
    "wall": Look((0.46, 0.49, 0.53), 0.0, 0.85),
    "alarm_front": Look((1.0, 1.0, 1.0), 0.0, 0.4, color_file="alarm_front.png"),
    "alarm_red": Look((0.55, 0.03, 0.025), 0.0, 0.4),
    "alarm_floor": Look((1.0, 1.0, 1.0), 0.0, 0.5, color_file="alarm_floor.png"),
    "arrow": Look((0.95, 0.70, 0.05), 0.0, 0.5, glow=(1.5, 1.05, 0.06), shine=0.0),  # glow-in-the-dark, as emergency signs are: it sits in the window's shadow
    "bezel": Look((0.03, 0.03, 0.03), 0.0, 0.4),
    "recess": Look((0.015, 0.015, 0.015), 0.0, 0.6),
    "handle": Look((0.95, 0.72, 0.05), 0.0, 0.35, glow=(0.5, 0.35, 0.02)),
    "glass": Look((0.82, 0.90, 0.94), 0.0, 0.05, opacity=0.06, shine=0.1),
    "glint": Look((0.95, 0.97, 1.0), 0.0, 0.1, opacity=0.65),
    "shard": Look((0.85, 0.92, 0.96), 0.0, 0.08, opacity=0.70),
    "lamp_off": Look((0.30, 0.03, 0.02), 0.0, 0.25),
    "lamp_on": Look((1.0, 0.25, 0.15), 0.0, 0.25, glow=(40.0, 8.0, 4.0)),
    "halo": Look((1.0, 0.2, 0.1), 0.0, 1.0, opacity=0.35, glow=(9.0, 1.0, 0.5), shine=0.0),
    "char": Look((0.018, 0.016, 0.015), 0.0, 0.9),
    "scorch": Look((0.035, 0.03, 0.028), 0.0, 0.95),
    "flame_red": Look((0.8, 0.14, 0.01), 0.0, 0.6, opacity=0.62, glow=(1.5, 0.16, 0.008), shine=0.0),
    "flame_orange": Look((1.0, 0.38, 0.02), 0.0, 0.6, opacity=0.72, glow=(2.3, 0.52, 0.02), shine=0.0),
    "flame_yellow": Look((1.0, 0.8, 0.25), 0.0, 0.6, opacity=0.85, glow=(3.2, 1.9, 0.25), shine=0.0),
    "smoke": Look((0.05, 0.05, 0.055), 0.0, 1.0, opacity=0.30, glow=(0.09, 0.09, 0.10), shine=0.0),
}

STROBE_INTENSITY = 60000.0
WALL_Y = -0.12  # the middle of the wall section, so the right arm is the nearer
WALL_HALF = 0.17
SLOPE_FOOT, SLOPE_TOP = (0.205, 0.055), (0.395, 0.165)  # the sloped front, from its foot to its top, in x and z
BOX = (0.10, 0.06, 0.032)  # the box: half its length up the slope, half its width, its depth
WINDOW = (-0.062, 0.033, 0.045)  # the window: from and to, along the slope; half its width
RIM = 0.063  # the raised frame's top, out of the slope, where the glass sits
PANE_THICK = 0.002
BAR_X, BAR_Z, BAR_R, BAR_HALF = -0.002, 0.054, 0.0065, 0.034  # the handle's bar: along and out of the slope, its radius, half its length
TRAVEL = 0.03  # the handle slides this far down the slope, towards the robot, to its stop
WALL_BACK = 0.47  # the wall section's back, deep enough for the strobes to stand on its top
STROBE_R = 0.032
STROBES = ((0.4325, WALL_Y + 0.13, 0.20), (0.4325, WALL_Y, 0.20), (0.4325, WALL_Y - 0.13, 0.20))  # x, y and the height each stands on: all on the wall section's top
_ANGLE = math.atan2(SLOPE_TOP[1] - SLOPE_FOOT[1], SLOPE_TOP[0] - SLOPE_FOOT[0])
ORIGIN = (0.5 * (SLOPE_FOOT[0] + SLOPE_TOP[0]), WALL_Y, 0.5 * (SLOPE_FOOT[1] + SLOPE_TOP[1]))  # the slope frame's origin on the table
UP_SLOPE = np.array([math.cos(_ANGLE), 0.0, math.sin(_ANGLE)])
OUT = np.array([-math.sin(_ANGLE), 0.0, math.cos(_ANGLE)])
SLOPE_WXYZ = (math.cos(_ANGLE / 2), 0.0, -math.sin(_ANGLE / 2), 0.0)


def on_table(x: float, y: float, z: float) -> np.ndarray:
    """A point given along the slope, across it and out of it, as x, y and height on the table."""
    return np.array(ORIGIN) + x * UP_SLOPE + y * np.array([0.0, 1.0, 0.0]) + z * OUT


def _R() -> np.ndarray:
    return np.column_stack([UP_SLOPE, np.array([0.0, 1.0, 0.0]), OUT])  # columns: up the slope, y, out of the face


def on_slope(m: Mesh) -> Mesh:
    """A mesh built with x up the slope and z out of the face, moved to the middle of the slope."""
    return m.moved(ORIGIN, rot=_R())


def _quad(x0, x1, y0, y1, z, uv=None) -> Mesh:
    v = [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z)]
    t = [uv(q[0], q[1]) if uv else (q[0] * 20.0, q[1] * 20.0) for q in v]
    return Mesh().add(v, [(0.0, 0.0, 1.0)] * 4, t, [[0, 1, 2], [0, 2, 3]])


def prism_y(poly, y0: float, y1: float) -> Mesh:
    """A convex polygon in x and z, swept along y."""
    m = Mesh()
    c = np.mean(np.array(poly), axis=0)
    for side, y in ((-1.0, y0), (1.0, y1)):
        v = [(p[0], y, p[1]) for p in poly]
        t = [(p[0] * 10.0, p[1] * 10.0) for p in poly]
        f = [[0, i, i + 1] if side < 0 else [0, i + 1, i] for i in range(1, len(poly) - 1)]
        m.add(v, [(0.0, side, 0.0)] * len(poly), t, f)
    for a, b in zip(poly, poly[1:] + poly[:1]):
        d = np.array(b) - np.array(a)
        n = np.array([d[1], -d[0]])
        n /= np.linalg.norm(n)
        if n @ (np.array(a) - c) < 0:
            n = -n
        v = [(a[0], y0, a[1]), (b[0], y0, b[1]), (b[0], y1, b[1]), (a[0], y1, a[1])]
        t = [(0.0, 0.0), (1.0, 0.0), (1.0, 3.0), (0.0, 3.0)]
        m.add(v, [(n[0], 0.0, n[1])] * 4, t, [[0, 1, 2], [0, 2, 3]])
    return m


def wall_parts():
    y0, y1 = WALL_Y - WALL_HALF, WALL_Y + WALL_HALF
    fx, fz = SLOPE_FOOT
    tx, tz = SLOPE_TOP
    wall = prism_y([(fx, 0.0), (tx, 0.0), (tx, tz), (fx, fz)], y0, y1)
    wall += prism_y([(tx, 0.0), (WALL_BACK, 0.0), (WALL_BACK, 0.20), (tx, 0.20)], y0, y1)
    return [("wall", wall, "wall")]


def box_parts():
    hl, hw, d = BOX
    w0, w1, ww = WINDOW
    uv = lambda x, y: (0.5 - y / (2 * hw), 0.5 + x / (2 * hl))  # noqa: E731
    out = [("body", on_slope(block(0.0, 0.0, 2 * hl, 2 * hw, 0.0, d - 0.0003, r=0.006, chamfer=0.0015)), "alarm_red")]
    face = _quad(w1, hl, -hw, hw, d, uv)
    face += _quad(-hl, w0, -hw, hw, d, uv)
    face += _quad(w0, w1, ww, hw, d, uv)
    face += _quad(w0, w1, -hw, -ww, d, uv)
    out.append(("face", on_slope(face), "alarm_front"))
    t = 0.0035
    bezel = block(w1 + t / 2, 0.0, t, 2 * ww + 2 * t, d, RIM, r=0.0008, chamfer=0.0003)
    bezel += block(w0 - t / 2, 0.0, t, 2 * ww + 2 * t, d, RIM, r=0.0008, chamfer=0.0003)
    bezel += block(0.5 * (w0 + w1), ww + t / 2, w1 - w0, t, d, RIM, r=0.0008, chamfer=0.0003)
    bezel += block(0.5 * (w0 + w1), -ww - t / 2, w1 - w0, t, d, RIM, r=0.0008, chamfer=0.0003)
    out.append(("bezel", on_slope(bezel), "bezel"))
    floor_uv = lambda x, y: (0.5 - y / (2 * ww), (x - w0) / (w1 - w0))  # noqa: E731
    out.append(("floor", on_slope(_quad(w0, w1, -ww, ww, d + 0.0002, floor_uv)), "alarm_floor"))
    out.append(("arrow", on_slope(_arrow(d + 0.0004)), "arrow"))
    return out


def _arrow(z: float) -> Mesh:
    """A big arrow on the box's floor, pointing the way the handle slides: down the slope, towards the robot."""
    shaft, head = 0.0105, 0.031
    top, neck, tip = 0.012, -0.033, -0.060
    v = [(top, -shaft, z), (neck, -shaft, z), (neck, shaft, z), (top, shaft, z), (neck, -head, z), (tip, 0.0, z), (neck, head, z)]
    return Mesh().add(v, [(0.0, 0.0, 1.0)] * 7, [(0.0, 0.0)] * 7, [[0, 1, 2], [0, 2, 3], [4, 5, 6]])


def box_colliders(root: str) -> str:
    """The box's body, its floor and the raised frame, in the slope's frame."""
    hl, hw, d = BOX
    w0, w1, ww = WINDOW
    t = 0.0035
    mid = 0.5 * (w0 + w1)
    high = RIM - d
    inner = [
        box(root, "body", (0.0, 0.0, 0.5 * d), (2 * hl, 2 * hw, d)),
        box(root, "rim_up", (w1 + t / 2, 0.0, d + high / 2), (t, 2 * ww + 2 * t, high)),
        box(root, "rim_down", (w0 - t / 2, 0.0, d + high / 2), (t, 2 * ww + 2 * t, high)),
        box(root, "rim_left", (mid, ww + t / 2, d + high / 2), (w1 - w0, t, high)),
        box(root, "rim_right", (mid, -ww - t / 2, d + high / 2), (w1 - w0, t, high)),
    ]
    return _slope_xform("".join(inner))


def _slope_xform(inner: str) -> str:
    o = ORIGIN
    q = SLOPE_WXYZ
    return f"""
    def Xform "slope"
    {{
        double3 xformOp:translate = ({o[0]:.6f}, {o[1]:.6f}, {o[2]:.6f})
        quatf xformOp:orient = ({q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]{inner}
    }}"""


def wall_colliders(root: str) -> str:
    """The sloped front as a thick slab under the slope, and the block behind it. Only their surfaces matter: nothing reaches inside."""
    slab = box(root, "slab", (0.0, 0.0, -0.05), (0.2196, 2 * WALL_HALF, 0.10))
    back = box(root, "back", (0.5 * (SLOPE_TOP[0] + WALL_BACK), WALL_Y, 0.10), (WALL_BACK - SLOPE_TOP[0], 2 * WALL_HALF, 0.20))
    return _slope_xform(slab) + back


def pane_parts():
    w0, w1, ww = WINDOW
    zg = RIM + PANE_THICK
    out = [("glass", on_slope(_quad(w0, w1, -ww, ww, zg)), "glass")]
    glint = Mesh()
    for x0, y0 in ((w0 + 0.06, -0.03), (w0 + 0.072, -0.018), (w0 + 0.012, 0.016)):
        v = [(x0, y0, zg + 0.0002), (x0 + 0.02, y0 + 0.02, zg + 0.0002), (x0 + 0.018, y0 + 0.023, zg + 0.0002), (x0 - 0.002, y0 + 0.003, zg + 0.0002)]
        glint.add(v, [(0.0, 0.0, 1.0)] * 4, [(0.0, 0.0)] * 4, [[0, 1, 2], [0, 2, 3]])
    out.append(("glint", on_slope(glint), "glint"))
    return out


def handle_parts():
    """The T handle in the slope's frame."""
    bar = tube([(BAR_X, -BAR_HALF, BAR_Z), (BAR_X, BAR_HALF, BAR_Z)], BAR_R, 14)
    bar += tube([(BAR_X, 0.0, BOX[2] + 0.001), (BAR_X, 0.0, BAR_Z)], 0.0055, 12)
    turn = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
    for s in (-1.0, 1.0):
        bar += lathe([(0.0, -BAR_R), (BAR_R, -BAR_R), (BAR_R, BAR_R), (0.0, BAR_R)], 14).moved((BAR_X, s * BAR_HALF, BAR_Z), rot=turn)
    return [("handle", bar, "handle")]


def write_handle_usda(path: str) -> str:
    """The handle on one prismatic joint along the slope, with no gravity. Joint zero is the pose written here, and the root is spawned in the slope's frame."""
    root = "AlarmHandle"
    parts = handle_parts()
    collider = box(root, "bar", (BAR_X, 0.0, BAR_Z), (2 * BAR_R, 2 * BAR_HALF, 2 * BAR_R), material="Grip", contact=0.001)
    return _write(path, f"""{_header(root)}
def Xform "{root}" (
    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
)
{{
    bool physxArticulation:enabledSelfCollisions = 0
    int physxArticulation:solverPositionIterationCount = 32
    int physxArticulation:solverVelocityIterationCount = 1
{_physics_materials()}{_looks(root, {p[2] for p in parts})}
    def Xform "frame" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    )
    {{
        float physics:mass = 1
        float3 physics:diagonalInertia = (1, 1, 1)
    }}

    def Xform "slider" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
    )
    {{
        float physics:mass = 0.06
        float3 physics:diagonalInertia = (0.0001, 0.0001, 0.0001)
        bool physxRigidBody:disableGravity = 1
        float physxRigidBody:maxDepenetrationVelocity = 1
{collider}

        def Xform "visual"
        {{{_collect(parts, root)}
        }}
    }}

    def PhysicsFixedJoint "world_joint"
    {{
        rel physics:body1 = </{root}/frame>
    }}

    def PhysicsPrismaticJoint "slider_joint" (
        prepend apiSchemas = ["PhysicsDriveAPI:linear"]
    )
    {{
        uniform token drive:linear:physics:type = "force"
        float drive:linear:physics:stiffness = 40
        float drive:linear:physics:damping = 5
        float drive:linear:physics:maxForce = 20
        float drive:linear:physics:targetPosition = 0
        rel physics:body0 = </{root}/frame>
        rel physics:body1 = </{root}/slider>
        uniform token physics:axis = "X"
        point3f physics:localPos0 = (0, 0, 0)
        quatf physics:localRot0 = (1, 0, 0, 0)
        point3f physics:localPos1 = (0, 0, 0)
        quatf physics:localRot1 = (1, 0, 0, 0)
        float physics:lowerLimit = {-TRAVEL:.6f}
        float physics:upperLimit = 0
    }}
}}
""")


def shard_parts():
    w0, w1, ww = WINDOW
    return [("shards", on_slope(_shards(w0, w1, ww, RIM + PANE_THICK)), "shard")]


def _shards(w0: float, w1: float, ww: float, z: float) -> Mesh:
    """What stays of the glass: jagged teeth round the window's edge, and bits below it."""
    rng = np.random.default_rng(7)
    m = Mesh()

    def tooth(a, b, tip):
        v = [(a[0], a[1], z), (b[0], b[1], z), (tip[0], tip[1], z)]
        m.add(v, [(0.0, 0.0, 1.0)] * 3, [(0.0, 0.0)] * 3, [[0, 1, 2]])

    n = 9
    for i in range(n):
        for edge in range(4):
            q0, q1 = i / n, (i + 1) / n
            depth = 0.006 + 0.016 * rng.random()
            mid = 0.5 * (q0 + q1) + 0.03 * (rng.random() - 0.5)
            if edge == 0:
                tooth((w0 + (w1 - w0) * q0, -ww), (w0 + (w1 - w0) * q1, -ww), (w0 + (w1 - w0) * mid, -ww + depth))
            elif edge == 1:
                tooth((w0 + (w1 - w0) * q0, ww), (w0 + (w1 - w0) * q1, ww), (w0 + (w1 - w0) * mid, ww - depth))
            elif edge == 2:
                tooth((w0, -ww + 2 * ww * q0), (w0, -ww + 2 * ww * q1), (w0 + depth, -ww + 2 * ww * mid))
            else:
                tooth((w1, -ww + 2 * ww * q0), (w1, -ww + 2 * ww * q1), (w1 - depth, -ww + 2 * ww * mid))
    return m


def strobe_parts():
    """The alarm's strobes, unlit: red domes on black bases, standing on the wall section's top."""
    base, dome = Mesh(), Mesh()
    r = STROBE_R
    for x, y, z in STROBES:
        base += lathe([(0.0, 0.0), (r + 0.003, 0.0), (r + 0.003, 0.012), (0.0, 0.012)], 28).moved((x, y, z))
        dome += _dome(1.0).moved((x, y, z + 0.012))
    return [("bases", base, "bezel"), ("domes", dome, "lamp_off")]


def _dome(grow: float) -> Mesh:
    r = STROBE_R * grow
    return lathe([(r * math.cos(a), r * 1.05 * math.sin(a)) for a in np.linspace(0.0, math.pi / 2, 9)] + [(0.0, r * 1.05)], 28)


def strobes_lit_parts():
    """What shows while the strobes flash: bright domes, each with a glow round it."""
    dome, halo = Mesh(), Mesh()
    for x, y, z in STROBES:
        dome += _dome(1.01).moved((x, y, z + 0.012))
        halo += _smoke_ball(x, y, z + 0.03, 2.1 * STROBE_R)
    return [("domes", dome, "lamp_on"), ("glow", halo, "halo")]


def strobe_lights() -> str:
    """Red light cast on the scene by each strobe, shown and hidden with the bright domes. These are props: the station's own lights are not touched."""
    out = ""
    for i, (x, y, z) in enumerate(STROBES):
        out += f"""
        def SphereLight "light_{i}"
        {{
            float inputs:intensity = {STROBE_INTENSITY}
            float inputs:radius = 0.03
            color3f inputs:color = (1, 0.12, 0.06)
            double3 xformOp:translate = ({x:.4f}, {y:.4f}, {z + 0.09:.4f})
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}"""
    return out


def strobe_colliders(root: str) -> str:
    r = STROBE_R + 0.003
    out = ""
    for i, (x, y, z) in enumerate(STROBES):
        out += box(root, f"strobe_{i}", (x, y, z + 0.025), (2 * r, 2 * r, 0.05))
    return out


def _tongue(cx: float, cy: float, r: float, h: float, seed: int, z0: float = 0.0) -> Mesh:
    """One flame: wide low down, drawn to a wavering point."""
    rng = np.random.default_rng(seed)
    rows, seg = 18, 18
    p1, p2 = rng.uniform(0, 6.28, 2)
    amp = rng.uniform(0.15, 0.45) * r
    v, n, t, f = [], [], [], []
    th = np.linspace(0.0, 2 * np.pi, seg + 1)
    for j in range(rows + 1):
        q = j / rows
        rad = r * (q / 0.22) ** 0.6 if q < 0.22 else r * (1.0 - (q - 0.22) / 0.78) ** 1.35  # wide low down, drawn to a point
        ox = amp * q * math.sin(5.0 * q + p1)
        oy = amp * q * math.sin(4.0 * q + p2)
        for a in th:
            d = np.array([math.cos(a), math.sin(a), 0.2])
            lump = 1.0 + 0.16 * math.sin(3.0 * a + 7.0 * q + p1)
            v.append((cx + ox + rad * lump * math.cos(a), cy + oy + rad * lump * math.sin(a), z0 + h * q))
            n.append(d / np.linalg.norm(d))
            t.append((0.0, 0.0))
    k = seg + 1
    for j in range(rows):
        for i in range(seg):
            f += [[j * k + i, j * k + i + 1, (j + 1) * k + i + 1], [j * k + i, (j + 1) * k + i + 1, (j + 1) * k + i]]
    return Mesh().add(v, n, t, f)


def z_limit(x: float) -> float:
    """The most a thing at x can rise above the table and stay in the top camera's frame."""
    return 0.944 - 0.716 * (x + 0.166)


def fire_parts():
    rng = np.random.default_rng(20261106)
    red, orange, yellow, char, smoke = Mesh(), Mesh(), Mesh(), Mesh(), Mesh()
    spots = [(float(rng.uniform(0.60, 0.84)), float(y + rng.uniform(-0.03, 0.03))) for y in np.linspace(-0.50, 0.50, 17)]
    spots += [(0.56, 0.36), (0.54, 0.47), (0.58, 0.24)]
    spots += [(float(rng.uniform(0.58, 0.86)), float(rng.uniform(-0.52, 0.52))) for _ in range(26)]
    for i, (x, y) in enumerate(spots):
        h = min(0.30, 0.82 * z_limit(x)) * float(rng.uniform(0.72, 1.0))
        r = float(rng.uniform(0.028, 0.06))
        red += _tongue(x, y, r, h, 100 + i)
        orange += _tongue(x, y, 0.72 * r, 0.76 * h, 200 + i)
        yellow += _tongue(x, y, 0.42 * r, 0.48 * h, 300 + i)
        for a in rng.uniform(0.0, 3.14, 3):
            d = np.array([math.cos(a), math.sin(a), 0.0]) * 0.07
            char += tube([np.array([x, y, 0.012]) - d, np.array([x, y, 0.016]) + d], 0.012, 10)
        for j in range(4):
            zs = min(z_limit(x) - 0.01, h + 0.02 + 0.035 * j)
            smoke += _smoke_ball(x + float(rng.uniform(-0.06, 0.06)), y + float(rng.uniform(-0.06, 0.06)), zs, float(rng.uniform(0.035, 0.08)))
    scorch = Mesh()
    scorch += block(0.72, 0.0, 0.34, 1.12, 0.0, 0.0012, r=0.05, chamfer=0.0002)
    scorch += block(0.55, 0.38, 0.14, 0.34, 0.0, 0.0012, r=0.05, chamfer=0.0002)
    return [("scorch", scorch, "scorch"), ("logs", char, "char"), ("flames_red", red, "flame_red"), ("flames_orange", orange, "flame_orange"), ("flames_yellow", yellow, "flame_yellow"), ("smoke", smoke, "smoke")]


def _smoke_ball(x: float, y: float, z: float, r: float) -> Mesh:
    """A soft puff: a sphere with smooth normals and a lumpy surface."""
    rng = np.random.default_rng(int(1e6 * abs(x + 3 * y + 7 * z)) % 100000)
    rows, seg = 18, 26
    p = rng.uniform(0.0, 6.28, 3)
    v, n, t, f = [], [], [], []
    for j in range(rows + 1):
        a = math.pi * j / rows
        for i in range(seg + 1):
            b = 2 * math.pi * i / seg
            d = np.array([math.sin(a) * math.cos(b), math.sin(a) * math.sin(b), -math.cos(a)])
            lump = 1.0 + 0.07 * math.sin(3.0 * b + p[0]) * math.sin(3.0 * a + p[1])
            v.append(np.array([x, y, z]) + r * lump * d)
            n.append(d)
            t.append((0.0, 0.0))
    k = seg + 1
    for j in range(rows):
        for i in range(seg):
            f += [[j * k + i, j * k + i + 1, (j + 1) * k + i + 1], [j * k + i, (j + 1) * k + i + 1, (j + 1) * k + i]]
    return Mesh().add(v, n, t, f)


def _looks(root: str, used) -> str:
    out = ""
    for name in sorted(used):
        look = LOOKS[name]
        base = f"/{root}/Looks/{name}"
        c = look.color
        nodes = ""

        def reader(shader: str, file: str, space: str, extra: str, output: str) -> str:
            return f"""
            def Shader "{shader}"
            {{
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @{HERE / "textures" / file}@
                float2 inputs:st.connect = <{base}/st.outputs:result>
                token inputs:sourceColorSpace = "{space}"
                token inputs:wrapS = "repeat"
                token inputs:wrapT = "repeat"{extra}
                {output}
            }}"""

        if look.color_file:
            tint = ""
            color_line = f"color3f inputs:diffuseColor.connect = <{base}/color.outputs:rgb>"
            nodes += reader("color", look.color_file, "sRGB", tint, "float3 outputs:rgb")
        else:
            color_line = f"color3f inputs:diffuseColor = ({c[0]}, {c[1]}, {c[2]})"
        if look.rough_file:
            rough_line = f"float inputs:roughness.connect = <{base}/rough.outputs:r>"
            nodes += reader("rough", look.rough_file, "raw", "", "float outputs:r")
        else:
            rough_line = f"float inputs:roughness = {look.roughness}"
        normal_line = ""
        if look.normal_file:
            normal_line = f"normal3f inputs:normal.connect = <{base}/normal.outputs:rgb>"
            remap = """
                float4 inputs:scale = (0.7, 0.7, 2, 1)
                float4 inputs:bias = (-0.35, -0.35, -1, 0)"""
            nodes += reader("normal", look.normal_file, "raw", remap, "float3 outputs:rgb")
        st = f"""
            def Shader "st"
            {{
                uniform token info:id = "UsdPrimvarReader_float2"
                string inputs:varname = "st"
                float2 outputs:result
            }}""" if nodes else ""
        out += f"""
        def Material "{name}"
        {{
            token outputs:surface.connect = <{base}/Shader.outputs:surface>
            def Shader "Shader"
            {{
                uniform token info:id = "UsdPreviewSurface"
                {color_line}
                color3f inputs:emissiveColor = ({look.glow[0]}, {look.glow[1]}, {look.glow[2]})
                float inputs:metallic = {look.metallic}
                float inputs:opacity = {look.opacity}
                int inputs:useSpecularWorkflow = 1
                color3f inputs:specularColor = ({0.06 * look.shine}, {0.06 * look.shine}, {0.06 * look.shine})
                float inputs:ior = {look.ior}
                {rough_line}
                {normal_line}
                token outputs:surface
            }}{st}{nodes}
        }}"""
    return f"""
    def Scope "Looks"
    {{{out}
    }}"""


def _physics_materials() -> str:
    def one(name: str, mu: float) -> str:
        return f"""
    def Material "{name}" (
        prepend apiSchemas = ["PhysicsMaterialAPI"]
    )
    {{
        float physics:staticFriction = {mu}
        float physics:dynamicFriction = {mu}
        float physics:restitution = 0
    }}"""

    return one("Slick", 0.25) + one("Grip", 1.0)


COLLIDER = """prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]"""


def _quat_z(deg: float) -> str:
    h = math.radians(deg) / 2
    return f"({math.cos(h):.6f}, 0, 0, {math.sin(h):.6f})"


def _offsets(contact: float) -> str:
    return f"""float physxCollision:contactOffset = {contact}
        float physxCollision:restOffset = 0"""


def box(root: str, name: str, centre, size, yaw_deg: float = 0.0, material: str = "Slick", contact: float = 0.002) -> str:
    c, s = centre, size
    return f"""
    def Cube "{name}" (
        {COLLIDER}
    )
    {{
        rel material:binding:physics = </{root}/{material}>
        {_offsets(contact)}
        uniform token purpose = "guide"
        double size = 1
        double3 xformOp:translate = ({c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f})
        quatf xformOp:orient = {_quat_z(yaw_deg)}
        float3 xformOp:scale = ({s[0]:.6f}, {s[1]:.6f}, {s[2]:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }}"""


def cyl(root: str, name: str, x: float, y: float, z0: float, z1: float, r: float, material: str = "Slick", contact: float = 0.002) -> str:
    return f"""
    def Cylinder "{name}" (
        {COLLIDER}
    )
    {{
        rel material:binding:physics = </{root}/{material}>
        {_offsets(contact)}
        uniform token purpose = "guide"
        uniform token axis = "Z"
        double height = {z1 - z0:.6f}
        double radius = {r:.6f}
        double3 xformOp:translate = ({x:.6f}, {y:.6f}, {0.5 * (z0 + z1):.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}"""



def _collect(parts: list[tuple[str, Mesh, str]], root: str) -> str:
    return "".join(usd_mesh(n, m, f"/{root}/Looks/{look}", "        ") for n, m, look in parts)


def _write(path: str, text: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path


def _header(root: str) -> str:
    return f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)
"""


def _static(path: str, root: str, colliders: str, parts) -> str:
    return _write(path, f"""{_header(root)}
def Xform "{root}"
{{{_physics_materials()}{_looks(root, {p[2] for p in parts})}
{colliders}
    def Xform "visual"
    {{{_collect(parts, root)}
    }}
}}
""")


def write_part(path: str, root: str, parts, colliders: str = "", hidden: bool = False, extra: str = "") -> str:
    """A fixed part. Without colliders of its own it gets one small one under the table, so that the simulator accepts it. A hidden part starts invisible, for the kit to show. `extra` is more USD inside the part's visual."""
    written = _static(path, root, colliders or box(root, "base", (0.0, 0.0, -0.0005), (0.01, 0.01, 0.001)), parts)
    if hidden or extra:
        with open(written) as f:
            text = f.read()
        head = 'def Xform "visual"\n    {'
        text = text.replace(head, head + ('\n        token visibility = "invisible"' if hidden else "") + extra, 1)
        _write(written, text)
    return written
