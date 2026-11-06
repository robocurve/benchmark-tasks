"""The pulley, its screws, the bench stand, the rack and the mats, written as plain-text USD (no pxr needed).

Each part's frame: z up, x and y as the world's. The pulley's origin is on its axis at its underside. A screw's origin is on its axis at its head's underside,
with the shank pointing down. The stand, rack and mats have their origins on the table.

The pulley is the BCN3D Moveo arm's base timing pulley, unchanged, from moveo/1M1.STL (MIT, moveo/LICENSE). Everything else is ours. The rack is a comb: a plate on two legs, with slots open towards the
robot, so a hanging screw's shank is in plain view below its head.

Colliders are boxes and convex shapes in their own prims, kept out of the render (purpose "guide"). The pulley's six holes are 4.3 mm across as seen, and collide as
rings of eight boxes with flats 7 mm apart. The stand's openings are 8 mm square. Hole depth is set by the floor boxes under the stand's openings; what
is seen of every hole is the same black dot.
"""

from __future__ import annotations

import math
import os
import pathlib
from dataclasses import dataclass

import numpy as np
from refit_mesh import Mesh, block, creased_mesh, decal, knurled_wall, lathe, plate, read_stl, rounded_rect, socket_screw, usd_mesh

HERE = pathlib.Path(__file__).resolve().parent

# -- sizes, metres ----------------------------------------------------------------------------

PULLEY_R = 0.080
PULLEY_THICK = 0.0255
PULLEY_MASS = 0.405  # 326 cm3 of PLA
HOLE_CIRCLE = 0.068
HOLE_DEG = {"far_left": 63.8, "pin_left": 90.0, "near_left": 116.2, "near_right": -116.2, "pin_right": -90.0, "far_right": -63.8}  # from +x, anticlockwise
THROAT = 0.0035  # half the flats' distance in a pulley hole's collider
RING = 0.003
RIM_IN, RIM_OUT = 0.058, 0.0785
HUB_R = 0.016
STAND = (0.17, 0.17, 0.04)
OPENING = 0.004  # half the square opening's side in the stand
DEEP, SHALLOW = 0.0365, 0.0065
PIN_R, PIN_H = 0.002, 0.008
SHANK_R = 0.002
HEAD_R, HEAD_H = 0.007, 0.008
LONG, SHORT = 0.060, 0.030
SCREW_MASS = {LONG: 0.016, SHORT: 0.013}
RACK_TOP_H, RACK_PLATE = 0.07, 0.006  # the plate's top above the table, and its thickness
RACK_FRONT, RACK_BACK, RACK_END = -0.012, 0.018, 0.135  # the plate's extent, from the slots' axes along x, and along y
SLOT = 0.005  # half a slot's width; it runs from the plate's front edge to its rounded end round the axis
SLOTS_Y = (0.11, 0.05, -0.05, -0.11)
MAT_R, MAT_H = 0.10, 0.003
DOT_R = 0.00215  # what is seen of a hole in the stand


def hole_xy(name: str) -> tuple[float, float]:
    """A hole on the pulley's frame. The rectangle of the four screw holes is 60 mm along x and 122 mm along y."""
    a = math.radians(HOLE_DEG[name])
    return HOLE_CIRCLE * math.cos(a), HOLE_CIRCLE * math.sin(a)


SCREW_HOLES = ("far_left", "near_left", "far_right", "near_right")
PIN_HOLES = ("pin_left", "pin_right")
DEEP_HOLES = ("far_left", "near_right")


# -- USD pieces -------------------------------------------------------------------------------


@dataclass(frozen=True)
class Look:
    color: tuple[float, float, float] = (0.5, 0.5, 0.5)
    metallic: float = 0.0
    roughness: float = 0.5
    color_file: str = ""
    rough_file: str = ""
    normal_file: str = ""


ALU = dict(metallic=0.8, color_file="machined_aluminium_color.jpg", rough_file="machined_aluminium_roughness.jpg", normal_file="machined_aluminium_normal.jpg")
LOOKS = {
    "aluminium": Look(**ALU),
    "new_print": Look((0.05, 0.14, 0.40), 0.0, 0.42, color_file="printed_lines.jpg"),
    "worn_print": Look((0.40, 0.42, 0.46), 0.0, 0.7, color_file="printed_worn.jpg"),
    "brass": Look((0.95, 0.66, 0.16), 1.0, 0.38),
    "steel": Look((0.66, 0.67, 0.68), 1.0, 0.22),
    "black_oxide": Look((0.03, 0.03, 0.032), 0.75, 0.42),
    "rubber": Look((0.20, 0.21, 0.22), 0.0, 0.85),
    "bore": Look((0.008, 0.008, 0.009), 0.0, 0.8),
}


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
            tint = "" if name == "aluminium" else f"\n                float4 inputs:scale = ({c[0]}, {c[1]}, {c[2]}, 1)"
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
                float inputs:metallic = {look.metallic}
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


def _hull(root: str, name: str, pts: list, faces: list[int], material: str, contact: float) -> str:
    points = ", ".join(f"({x:.6f}, {y:.6f}, {z:.6f})" for x, y, z in pts)
    counts = ", ".join("3" for _ in range(len(faces) // 3))
    return f"""
    def Mesh "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"]
    )
    {{
        rel material:binding:physics = </{root}/{material}>
        uniform token physics:approximation = "convexHull"
        {_offsets(contact)}
        uniform token purpose = "guide"
        int[] faceVertexCounts = [{counts}]
        int[] faceVertexIndices = [{", ".join(str(i) for i in faces)}]
        point3f[] points = [{points}]
        uniform token subdivisionScheme = "none"
    }}"""


def round_hull(root: str, name: str, x: float, y: float, profile, material: str = "Slick", seg: int = 12, contact: float = 0.001) -> str:
    """A convex hull of a (radius, z) profile about a vertical axis at (x, y)."""
    a = np.linspace(0.0, 2 * np.pi, seg, endpoint=False)
    pts = [(x + r * math.cos(t), y + r * math.sin(t), z) for r, z in profile for t in a]
    n = len(profile)
    faces: list[int] = []
    for k in range(n - 1):
        for i in range(seg):
            j = (i + 1) % seg
            faces += [k * seg + i, k * seg + j, (k + 1) * seg + j, k * seg + i, (k + 1) * seg + j, (k + 1) * seg + i]
    for i in range(1, seg - 1):
        faces += [0, i + 1, i, (n - 1) * seg, (n - 1) * seg + i, (n - 1) * seg + i + 1]
    return _hull(root, name, pts, faces, material, contact)


def prism(root: str, name: str, poly, z0: float, z1: float, material: str = "Slick", contact: float = 0.002) -> str:
    """A convex hull of a convex polygon (counter-clockwise) between two heights."""
    m = len(poly)
    pts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    faces: list[int] = []
    for i in range(m):
        j = (i + 1) % m
        faces += [i, j, m + j, i, m + j, m + i]
    for i in range(1, m - 1):
        faces += [0, i + 1, i, m, m + i, m + i + 1]
    return _hull(root, name, pts, faces, material, contact)


def blocks_with_openings(root: str, prefix: str, x0, x1, y0, y1, z0, z1, openings) -> str:
    """Boxes filling a block, leaving square openings (cx, cy, half side, floor height) open down to their floors."""
    xs = sorted({x0, x1, *(c[0] - c[2] for c in openings), *(c[0] + c[2] for c in openings)})
    out, n = "", 0
    for a, b in zip(xs[:-1], xs[1:]):
        mid = 0.5 * (a + b)
        cuts = sorted((c[1] - c[2], c[1] + c[2]) for c in openings if abs(c[0] - mid) < c[2])
        y = y0
        for lo, hi in [*cuts, (y1, y1)]:
            if lo > y + 1e-9:
                out += box(root, f"{prefix}_{n}", (mid, 0.5 * (y + lo), 0.5 * (z0 + z1)), (b - a, lo - y, z1 - z0))
                n += 1
            y = hi
    for i, (cx, cy, h, floor) in enumerate(openings):
        if floor > z0:
            out += box(root, f"{prefix}_floor{i}", (cx, cy, 0.5 * (z0 + floor)), (2 * h, 2 * h, floor - z0))
    return out


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


# -- the stand, rack and mats -------------------------------------------------------------------


def write_stand_usda(path: str) -> str:
    root = "Stand"
    sx, sy, sz = STAND
    openings = []
    for name in SCREW_HOLES:
        x, y = hole_xy(name)
        openings.append((x, y, OPENING, sz - (DEEP if name in DEEP_HOLES else SHALLOW)))
    colliders = blocks_with_openings(root, "body", -sx / 2, sx / 2, -sy / 2, sy / 2, 0.0, sz, openings)
    for name in PIN_HOLES:
        x, y = hole_xy(name)
        colliders += round_hull(root, f"{name}_collider", x, y, [(PIN_R, sz), (PIN_R, sz + PIN_H - 0.0025), (0.001, sz + PIN_H)], contact=0.001)
    body = block(0.0, 0.0, sx, sy, 0.0, sz, r=0.006, chamfer=0.001)
    dots = Mesh()
    for name in SCREW_HOLES:
        x, y = hole_xy(name)
        dots += lathe([(DOT_R, sz + 0.0002), (0.0, sz + 0.0002)], 24).moved((x, y, 0.0))
    pins = Mesh()
    for name in PIN_HOLES:
        x, y = hole_xy(name)
        pins += lathe([(PIN_R, sz), (PIN_R, sz + PIN_H - 0.0025), (0.001, sz + PIN_H), (0.0, sz + PIN_H)], 18).moved((x, y, 0.0))
    heads, sockets = Mesh(), Mesh()
    for qx in (-1, 1):
        for qy in (-1, 1):
            h, s = socket_screw(qx * 0.075, qy * 0.075, sz, 0.0035)
            heads += h
            sockets += s
    return _static(path, root, colliders, [("body", body, "aluminium"), ("dots", dots, "bore"), ("pins", pins, "steel"), ("corner_screws", heads, "black_oxide"), ("corner_sockets", sockets, "bore")])


def _rack_outline() -> np.ndarray:
    """The comb's outline, counter-clockwise: round the back, then along the front edge through each slot."""
    pts = [(RACK_FRONT, -RACK_END), (RACK_BACK, -RACK_END), (RACK_BACK, RACK_END), (RACK_FRONT, RACK_END)]
    for yc in sorted(SLOTS_Y, reverse=True):
        pts.append((RACK_FRONT, yc + SLOT))
        for a in np.radians(np.arange(90.0, -90.1, -22.5)):
            pts.append((SLOT * math.cos(a), yc + SLOT * math.sin(a)))
        pts.append((RACK_FRONT, yc - SLOT))
    return np.array(pts)


def write_rack_usda(path: str) -> str:
    root = "Rack"
    z1, z0 = RACK_TOP_H, RACK_TOP_H - RACK_PLATE
    zm, zs = 0.5 * (z0 + z1), z1 - z0
    colliders = box(root, "back", (0.5 * (SLOT + RACK_BACK), 0.0, zm), (RACK_BACK - SLOT, 2 * RACK_END, zs))
    edges = sorted(SLOTS_Y)
    spans = [(-RACK_END, edges[0] - SLOT)] + [(a + SLOT, b - SLOT) for a, b in zip(edges[:-1], edges[1:])] + [(edges[-1] + SLOT, RACK_END)]
    for i, (lo, hi) in enumerate(spans):
        colliders += box(root, f"finger{i}", (0.5 * (RACK_FRONT + SLOT), 0.5 * (lo + hi), zm), (SLOT - RACK_FRONT, hi - lo, zs))
    legs = Mesh()
    for tag, y in (("p", RACK_END - 0.006), ("m", -RACK_END + 0.006)):
        colliders += box(root, f"leg_{tag}", (0.5 * (RACK_FRONT + RACK_BACK), y, 0.5 * z0), (RACK_BACK - RACK_FRONT, 0.012, z0))
        legs += block(0.5 * (RACK_FRONT + RACK_BACK), y, RACK_BACK - RACK_FRONT, 0.012, 0.0, z0 + 0.0002, r=0.002, chamfer=0.0005)
    body = plate(_rack_outline(), [], z0, z1, 0.0005)[0]
    screw_heads, sockets = Mesh(), Mesh()
    for y in (RACK_END - 0.006, -RACK_END + 0.006):
        h, q = socket_screw(0.003, y, z1, 0.0028)
        screw_heads += h
        sockets += q
    return _static(path, root, colliders, [("plate", body, "aluminium"), ("legs", legs, "aluminium"), ("leg_screws", screw_heads, "black_oxide"), ("leg_sockets", sockets, "bore")])


def write_mat_usda(path: str, root: str) -> str:
    colliders = cyl(root, "mat", 0.0, 0.0, 0.0, MAT_H, MAT_R)
    mat = lathe([(0.0, 0.0), (MAT_R, 0.0), (MAT_R, MAT_H - 0.0006), (MAT_R - 0.0008, MAT_H), (MAT_R - 0.008, MAT_H), (MAT_R - 0.0085, MAT_H - 0.0005), (MAT_R - 0.0095, MAT_H - 0.0005), (MAT_R - 0.01, MAT_H), (0.0, MAT_H)], 72)
    return _static(path, root, colliders, [("mat", mat, "rubber")])


# -- the pulley -----------------------------------------------------------------------------------


_TRI: dict[str, np.ndarray] = {}


def pulley_triangles() -> np.ndarray:
    """The real pulley's triangles, in metres, in the pulley's frame. The rotation keeps the mesh's handedness."""
    if "t" not in _TRI:
        t = read_stl(str(HERE / "moveo" / "1M1.STL"))
        x, y, z = t[..., 0], t[..., 1], t[..., 2]
        _TRI["t"] = np.stack([(x - 80.0) / 1000.0, -(z - 80.0) / 1000.0, y / 1000.0], axis=-1)
    return _TRI["t"]


WORN_TEETH_DEG = ((-30.0, -4.0), (150.0, 171.0), (24.0, 37.0))


def worn_teeth(tri: np.ndarray) -> np.ndarray:
    """Flatten the teeth over a few arcs: the rim's outer points there come in to the teeth's root."""
    out = tri.copy()
    r = np.hypot(out[..., 0], out[..., 1])
    a = np.degrees(np.arctan2(out[..., 1], out[..., 0]))
    root_r = 0.0772
    mask = np.zeros(r.shape, bool)
    for lo, hi in WORN_TEETH_DEG:
        mask |= (a > lo) & (a < hi) & (r > root_r)
    k = np.where(mask, root_r / np.maximum(r, 1e-9), 1.0)
    out[..., 0] *= k
    out[..., 1] *= k
    return out


def _pulley_colliders(root: str) -> str:
    z0, z1 = 0.0, PULLEY_THICK
    zm, zs = 0.5 * (z0 + z1), z1 - z0
    out = cyl(root, "hub", 0.0, 0.0, z0, z1, HUB_R, "Grip")
    for k, deg in enumerate((45.0, 135.0, 225.0, 315.0)):  # the spokes, widening from the hub to the rim
        u = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])
        v = np.array([-u[1], u[0]])
        poly = [tuple(0.014 * u - 0.005 * v), tuple(0.060 * u - 0.013 * v), tuple(0.060 * u + 0.013 * v), tuple(0.014 * u + 0.005 * v)]
        out += prism(root, f"spoke{k}", poly, z0, z1)
    half_deg = 8.0
    taken = sorted((d - half_deg, d + half_deg) for d in HOLE_DEG.values())
    arcs, a = [], -180.0
    for lo, hi in [*taken, (180.0, 180.0)]:
        if lo > a + 0.5:
            arcs.append((a, lo))
        a = hi
    n = 0
    for lo, hi in arcs:
        parts = max(1, int(math.ceil((hi - lo) / 28.0)))
        for i in range(parts):
            a0, a1 = math.radians(lo + (hi - lo) * i / parts), math.radians(lo + (hi - lo) * (i + 1) / parts)
            mid = 0.5 * (a0 + a1)
            poly = [(RIM_IN * math.cos(a0), RIM_IN * math.sin(a0)), (RIM_OUT * math.cos(a0), RIM_OUT * math.sin(a0)), (RIM_OUT * math.cos(mid), RIM_OUT * math.sin(mid)),
                    (RIM_OUT * math.cos(a1), RIM_OUT * math.sin(a1)), (RIM_IN * math.cos(a1), RIM_IN * math.sin(a1))]
            out += prism(root, f"rim{n}", poly, z0, z1)
            n += 1
    out_r = THROAT + RING
    w = 2 * out_r * math.tan(math.pi / 8)
    tangent = HOLE_CIRCLE * math.sin(math.radians(half_deg))
    for name, deg in HOLE_DEG.items():
        t = math.radians(deg)
        u, v = np.array([math.cos(t), math.sin(t)]), np.array([-math.sin(t), math.cos(t)])
        c = HOLE_CIRCLE * u
        for i in range(8):
            b = i * 45.0
            d = (THROAT + 0.5 * RING) * np.array([math.cos(t + math.radians(b)), math.sin(t + math.radians(b))])
            out += box(root, f"{name}_ring{i}", (c[0] + d[0], c[1] + d[1], zm), (RING, w, zs), deg + b)
        inner, outer = HOLE_CIRCLE - RIM_IN - out_r, RIM_OUT - HOLE_CIRCLE - out_r
        for tag, shift, size in (("in", -(out_r + 0.5 * inner), inner), ("out", out_r + 0.5 * outer, outer)):
            q = c + shift * u
            out += box(root, f"{name}_{tag}", (q[0], q[1], zm), (size, 2 * tangent, zs), deg)
        side = tangent - out_r
        for tag, sign in (("p", 1), ("m", -1)):
            q = c + sign * (out_r + 0.5 * side) * v
            out += box(root, f"{name}_{tag}", (q[0], q[1], zm), (2 * out_r, side, zs), deg)
    return out


def write_pulley_usda(path: str, worn: bool) -> str:
    root = "PulleyWorn" if worn else "PulleyNew"
    tri = pulley_triangles()
    if worn:
        tri = worn_teeth(tri)
    parts = [("pulley", creased_mesh(tri), "worn_print" if worn else "new_print")]
    return _write(path, f"""{_header(root)}
def Xform "{root}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
)
{{
    float physics:mass = {PULLEY_MASS}
    point3f physics:centerOfMass = (0, 0, {0.5 * PULLEY_THICK:.5f})
    float3 physics:diagonalInertia = (0.0009, 0.0009, 0.0017)
    int physxRigidBody:solverPositionIterationCount = 32
    float physxRigidBody:maxDepenetrationVelocity = 1
{_physics_materials()}{_looks(root, {p[2] for p in parts})}
{_pulley_colliders(root)}
    def Xform "visual"
    {{{_collect(parts, root)}
    }}
}}
""")


# -- the screws -----------------------------------------------------------------------------------


def screw_parts(length: float) -> list[tuple[str, Mesh, str]]:
    collar = lathe([(0.0035, -0.0001), (0.0045, 0.0), (0.0045, 0.0012), (HEAD_R - 0.0006, 0.0012)], 28)
    head = knurled_wall(HEAD_R, 0.0012, HEAD_H - 0.0008, teeth=30, depth=0.00045)
    top = lathe([(HEAD_R, HEAD_H - 0.0008), (HEAD_R - 0.0008, HEAD_H), (0.0, HEAD_H)], 32)
    slot = decal(-0.0048, 0.0048, -0.0006, 0.0006, HEAD_H + 0.0001)
    shank = [(SHANK_R, 0.0)]
    z, pitch = -0.003, 0.0014
    shank.append((SHANK_R, z))
    while z > -length + 0.0045:
        shank += [(SHANK_R - 0.00028, z - 0.45 * pitch), (SHANK_R, z - pitch)]
        z -= pitch
    shank += [(SHANK_R, -length + 0.003), (0.0012, -length), (0.0, -length)]
    return [("collar", collar, "brass"), ("head", head, "brass"), ("top", top, "brass"), ("slot", slot, "bore"), ("shank", lathe(shank[::-1], 10), "black_oxide")]


def write_screw_usda(path: str, length: float) -> str:
    root = "Screw"
    colliders = cyl(root, "head_collider", 0.0, 0.0, 0.0, HEAD_H, HEAD_R, "Grip", contact=0.001)
    colliders += round_hull(root, "shank_collider", 0.0, 0.0, [(0.0012, -length), (SHANK_R, -length + 0.003), (SHANK_R, 0.0)], contact=0.001)
    mass = SCREW_MASS[length]
    parts = screw_parts(length)
    com = (0.5 * HEAD_H * 0.0097 - 0.5 * length * (mass - 0.0097)) / mass
    return _write(path, f"""{_header(root)}
def Xform "{root}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
)
{{
    float physics:mass = {mass}
    point3f physics:centerOfMass = (0, 0, {com:.5f})
    float3 physics:diagonalInertia = (0.000006, 0.000006, 0.000001)
    int physxRigidBody:solverPositionIterationCount = 32
    float physxRigidBody:maxDepenetrationVelocity = 0.5
{_physics_materials()}{_looks(root, {p[2] for p in parts})}
{colliders}
    def Xform "visual"
    {{{_collect(parts, root)}
    }}
}}
""")
