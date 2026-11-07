"""kitchenkit: modelled and textured props for a yam_arena task file.

A `yam_arena` task file declares rigid primitives. This task's props are lathed and swept meshes, some with generated textures, a few
simple colliders that stay separate from what is seen, and a handle on a joint. `Part` subclasses `Box` so that yam_arena's
task-file checks accept each prop, and importing this module hooks `yam_arena.scene` and `yam_arena.inspect_embodiment` (it
changes nothing on disk) so that a `Part` spawns from a USD file written at start-up.

Callbacks registered with `on_step` run after every control step, before the task file is scored, and those registered with
`on_reset` run after every reset. Both get the Isaac Lab environment.

Everything is plain-text USD written from Python, so the only binary files are the textures in `textures/`.
Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys
import tempfile
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np
from yam_arena.task_file import Box, ObjectState

HERE = pathlib.Path(__file__).resolve().parent
TEX = HERE / "tex"
ENV_PATH = "/World/envs/env_0"

# -- looks ---------------------------------------------------------------------------------


def srgb(r: float, g: float, b: float) -> tuple[float, float, float]:
    """sRGB (0..1) to the linear values a USD material takes."""

    def one(c: float) -> float:
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

    return (one(r), one(g), one(b))


@dataclass(frozen=True)
class Look:
    """A surface. `color` and `emissive` are linear. Each `*_map` is a file under `TEX`, which this task's parts do not use: their materials and textures are in `beer_usd.py`. `tint` multiplies the colour map."""

    name: str
    color: tuple[float, float, float] = (0.5, 0.5, 0.5)
    metallic: float = 0.0
    roughness: float = 0.5
    opacity: float = 1.0
    emissive: tuple[float, float, float] | None = None
    color_map: str | None = None
    rough_map: str | None = None
    normal_map: str | None = None
    emissive_map: str | None = None
    emissive_gain: float = 1.0
    tint: tuple[float, float, float] = (1.0, 1.0, 1.0)
    glass: tuple[float, float, float] | None = None  # the OmniGlass material, with this colour


def _texture(mat: str, node: str, file: str, space: str, out: str, scale=(1, 1, 1, 1), bias=(0, 0, 0, 0)) -> str:
    return f"""
            def Shader "{node}"
            {{
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @{TEX / file}@
                float2 inputs:st.connect = <{mat}/ST.outputs:result>
                token inputs:sourceColorSpace = "{space}"
                token inputs:wrapS = "repeat"
                token inputs:wrapT = "repeat"
                float4 inputs:scale = ({scale[0]}, {scale[1]}, {scale[2]}, {scale[3]})
                float4 inputs:bias = ({bias[0]}, {bias[1]}, {bias[2]}, {bias[3]})
                {out}
            }}"""


def look_usda(root: str, look: Look) -> str:
    mat = f"/{root}/Looks/{look.name}"
    if look.glass is not None:
        c = look.glass
        return f"""
        def Material "{look.name}"
        {{
            token outputs:mdl:surface.connect = <{mat}/Shader.outputs:out>
            token outputs:mdl:displacement.connect = <{mat}/Shader.outputs:out>
            token outputs:mdl:volume.connect = <{mat}/Shader.outputs:out>

            def Shader "Shader"
            {{
                uniform token info:implementationSource = "sourceAsset"
                uniform asset info:mdl:sourceAsset = @OmniGlass.mdl@
                uniform token info:mdl:sourceAsset:subIdentifier = "OmniGlass"
                color3f inputs:glass_color = ({c[0]}, {c[1]}, {c[2]})
                float inputs:glass_ior = 1.15
                float inputs:frosting_roughness = 0
                bool inputs:thin_walled = 1
                token outputs:out
            }}
        }}"""
    nodes, inputs = [], []
    if look.color_map:
        t = look.tint
        nodes.append(_texture(mat, "Color", look.color_map, "sRGB", "float3 outputs:rgb", scale=(t[0], t[1], t[2], 1)))
        inputs.append(f"color3f inputs:diffuseColor.connect = <{mat}/Color.outputs:rgb>")
    else:
        c = look.color
        inputs.append(f"color3f inputs:diffuseColor = ({c[0]:.5f}, {c[1]:.5f}, {c[2]:.5f})")
    if look.rough_map:
        nodes.append(_texture(mat, "Rough", look.rough_map, "raw", "float outputs:r",
                              scale=(look.roughness, look.roughness, look.roughness, 1)))
        inputs.append(f"float inputs:roughness.connect = <{mat}/Rough.outputs:r>")
    else:
        inputs.append(f"float inputs:roughness = {look.roughness}")
    if look.normal_map:
        nodes.append(_texture(mat, "Normal", look.normal_map, "raw", "normal3f outputs:rgb", scale=(2, 2, 2, 1), bias=(-1, -1, -1, 0)))
        inputs.append(f"normal3f inputs:normal.connect = <{mat}/Normal.outputs:rgb>")
    if look.emissive_map:
        g = look.emissive_gain
        nodes.append(_texture(mat, "Emissive", look.emissive_map, "sRGB", "float3 outputs:rgb", scale=(g, g, g, 1)))
        inputs.append(f"color3f inputs:emissiveColor.connect = <{mat}/Emissive.outputs:rgb>")
    elif look.emissive:
        e = look.emissive
        inputs.append(f"color3f inputs:emissiveColor = ({e[0]:.4f}, {e[1]:.4f}, {e[2]:.4f})")
    inputs.append(f"float inputs:metallic = {look.metallic}")
    if look.opacity < 1.0:
        inputs.append(f"float inputs:opacity = {look.opacity}")
    body = "\n                ".join(inputs)
    return f"""
        def Material "{look.name}"
        {{
            token outputs:surface.connect = <{mat}/PBR.outputs:surface>

            def Shader "PBR"
            {{
                uniform token info:id = "UsdPreviewSurface"
                {body}
                token outputs:surface
            }}

            def Shader "ST"
            {{
                uniform token info:id = "UsdPrimvarReader_float2"
                string inputs:varname = "st"
                float2 outputs:result
            }}{''.join(nodes)}
        }}"""


# -- meshes --------------------------------------------------------------------------------


@dataclass
class Mesh:
    name: str
    look: str
    points: np.ndarray
    normals: np.ndarray
    st: np.ndarray
    counts: np.ndarray
    indices: np.ndarray
    visible: bool = True


def _merge(name: str, look: str, parts: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]], visible: bool = True) -> Mesh:
    pts, nrm, st, quads, off = [], [], [], [], 0
    for p, n, s, q in parts:
        pts.append(p), nrm.append(n), st.append(s), quads.append(q + off)
        off += len(p)
    quads_all = np.concatenate(quads)
    return Mesh(name, look, np.concatenate(pts), np.concatenate(nrm), np.concatenate(st), np.full(len(quads_all), 4), quads_all.reshape(-1), visible)


def _grid(n_rows: int, n_cols: int, wrap: bool = False) -> np.ndarray:
    """Quads of a grid of points indexed row * n_cols + col, facing along (d col) x (d row)."""
    i, j = np.meshgrid(np.arange(n_rows - 1), np.arange(n_cols - (0 if wrap else 1)), indexing="ij")
    j1 = (j + 1) % n_cols if wrap else j + 1
    a, b, c, d = i * n_cols + j, i * n_cols + j1, (i + 1) * n_cols + j1, (i + 1) * n_cols + j
    return np.stack([a, b, c, d], axis=-1).reshape(-1, 4)


def lathe(name: str, look: str, sections: list[list[tuple[float, float]]], n: int = 64, tile: float = 0.2, center=(0.0, 0.0, 0.0),
          visible: bool = True) -> Mesh:
    """Revolve profiles about +z. Each section is a smooth run of (r, z) points, with a hard edge between sections. Traverse the
    profile with the solid on the left (up the outside, down the inside, inward on a top face): the normals follow from that.
    Texture u runs round the axis and v along the profile, `tile` metres per repeat."""
    th = np.linspace(0.0, 2 * np.pi, n + 1)
    parts = []
    v0 = 0.0
    for sec in sections:
        prof = np.asarray(sec, dtype=float)
        d = np.diff(prof, axis=0)
        seg = np.linalg.norm(d, axis=1)
        t = np.zeros_like(prof)
        t[:-1] += d
        t[1:] += d
        t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
        nr, nz = t[:, 1], -t[:, 0]
        v = v0 + np.r_[0.0, np.cumsum(seg)] / tile
        v0 = float(v[-1])
        k = max(1, round(2 * np.pi * prof[:, 0].max() / tile))
        rows, cols = len(prof), n + 1
        c, s = np.cos(th)[None, :], np.sin(th)[None, :]
        r, z = prof[:, 0][:, None], prof[:, 1][:, None]
        P = np.stack([r * c + center[0], r * s + center[1], np.broadcast_to(z + center[2], (rows, cols))], axis=-1).reshape(-1, 3)
        N = np.stack([nr[:, None] * c, nr[:, None] * s, np.broadcast_to(nz[:, None], (rows, cols))], axis=-1).reshape(-1, 3)
        U = np.stack([np.broadcast_to(th[None, :] / (2 * np.pi) * k, (rows, cols)), np.broadcast_to(v[:, None], (rows, cols))], axis=-1).reshape(-1, 2)
        Q = _grid(rows, cols)[:, [0, 1, 2, 3]]
        parts.append((P, N, U, Q))
    return _merge(name, look, parts, visible)


def box_mesh(name: str, look: str, center, size, tile: float = 0.2, visible: bool = True) -> Mesh:
    """A box with its faces flat-shaded, and each face's texture at `tile` metres per repeat."""
    c, h = np.asarray(center, dtype=float), 0.5 * np.asarray(size, dtype=float)
    parts = []
    for axis in range(3):
        for sign in (1.0, -1.0):
            u_ax, v_ax = (axis + 1) % 3, (axis + 2) % 3
            if sign < 0:
                u_ax, v_ax = v_ax, u_ax
            corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], dtype=float)
            P = np.zeros((4, 3))
            P[:, axis] = sign * h[axis]
            P[:, u_ax] = corners[:, 0] * h[u_ax]
            P[:, v_ax] = corners[:, 1] * h[v_ax]
            N = np.zeros((4, 3))
            N[:, axis] = sign
            U = np.stack([P[:, u_ax], P[:, v_ax]], axis=-1) / tile + 0.5
            parts.append((P + c, N, U, np.array([[0, 1, 2, 3]])))
    return _merge(name, look, parts, visible)


def swept(name: str, look: str, path, half_w, half_h, side=(1.0, 0.0, 0.0), n: int = 20, tile: float = 0.2, power: float = 4.0, visible: bool = True) -> Mesh:
    """A bar swept along `path`, with a rounded rectangular section (superellipse of exponent `power`) of half-widths given per
    path point, the width along `side`. Both ends are capped."""
    pts = np.asarray(path, dtype=float)
    side = np.asarray(side, dtype=float)
    w, h = np.broadcast_to(half_w, len(pts)).astype(float), np.broadcast_to(half_h, len(pts)).astype(float)
    t = np.zeros_like(pts)
    t[:-1] += np.diff(pts, axis=0)
    t[1:] += np.diff(pts, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    a = np.linspace(0.0, 2 * np.pi, n + 1)
    ca, sa = np.cos(a), np.sin(a)
    ex, ey = np.sign(ca) * np.abs(ca) ** (2 / power), np.sign(sa) * np.abs(sa) ** (2 / power)
    rows, cols = len(pts), n + 1
    P, N, U = np.zeros((rows, cols, 3)), np.zeros((rows, cols, 3)), np.zeros((rows, cols, 2))
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    for i in range(rows):
        s = side - (side @ t[i]) * t[i]
        s /= np.linalg.norm(s)
        up = np.cross(t[i], s)
        P[i] = pts[i] + np.outer(w[i] * ex, s) + np.outer(h[i] * ey, up)
        nn = np.outer(np.sign(ca) * np.abs(ca) ** (2 - 2 / power) / max(w[i], 1e-6), s) + np.outer(np.sign(sa) * np.abs(sa) ** (2 - 2 / power) / max(h[i], 1e-6), up)
        N[i] = nn / np.maximum(np.linalg.norm(nn, axis=1, keepdims=True), 1e-9)
        U[i, :, 0] = a * max(w.max(), h.max()) / tile
        U[i, :, 1] = arc[i] / tile
    parts = [(P.reshape(-1, 3), N.reshape(-1, 3), U.reshape(-1, 2), _grid(rows, cols)[:, [0, 3, 2, 1]])]
    for i, sign in ((0, -1.0), (rows - 1, 1.0)):  # caps: a fan as quads from the centre
        C = np.vstack([pts[i][None], P[i]])
        M = np.tile(sign * t[i], (len(C), 1))
        q = np.array([[0, j + 1, j + 2, 0] for j in range(n)]) if sign > 0 else np.array([[0, j + 2, j + 1, 0] for j in range(n)])
        parts.append((C, M, np.zeros((len(C), 2)), q))
    return _merge(name, look, parts, visible)


def silhouette(file: str, threshold: float = 0.35, rows: int = 28) -> np.ndarray:
    """A picture's outline, from its alpha: rows of (v, u left, u right), bottom up, along the largest solid run in each row."""
    from PIL import Image

    alpha = np.asarray(Image.open(TEX / file).convert("RGBA"))[:, :, 3] / 255.0
    h, w = alpha.shape
    out = []
    for row in np.linspace(h - 1, 0, rows).astype(int):
        solid = np.flatnonzero(alpha[row] > threshold)
        if len(solid) == 0:
            if out:
                break
            continue
        runs = np.split(solid, np.flatnonzero(np.diff(solid) > 1) + 1)
        run = max(runs, key=len)
        out.append((1.0 - row / (h - 1), run[0] / (w - 1), run[-1] / (w - 1)))
    return np.array(out)


def cards(name: str, look: str, outline: np.ndarray, quads: list[tuple], visible: bool = True) -> Mesh:
    """Upright flat shapes: `outline` from `silhouette`, placed by each of `quads`: (x, y, angle about z, width, z0, height, flip the
    picture). The shape is real geometry, so nothing needs an opacity map."""
    parts = []
    v, ul, ur = outline[:, 0], outline[:, 1], outline[:, 2]
    n = len(v)
    us = np.stack([ul, 0.5 * (ul + ur), ur], axis=1)
    eps = 0.0004  # the renderer lights emissive faces on their front only, so each shape is two faces back to back, this far apart
    for x, y, a, w, z0, h, flip in quads:
        d = np.array([np.cos(a), np.sin(a), 0.0])
        o = np.array([x, y, z0])
        side = (0.5 - us) if flip else (us - 0.5)
        P = np.zeros((n, 3, 3))
        for j in range(3):
            P[:, j] = o + (w * side[:, j])[:, None] * d + (h * v)[:, None] * np.array([0.0, 0.0, 1.0])
        front = np.cross(P[0, 2] - P[0, 0], np.array([0.0, 0.0, 1.0]))
        front /= np.linalg.norm(front)
        U = np.stack([us.reshape(-1), np.repeat(v, 3)], axis=-1)
        Q = _grid(n, 3)
        parts.append((P.reshape(-1, 3) + eps * front, np.tile(front, (3 * n, 1)), U, Q))
        parts.append((P.reshape(-1, 3) - eps * front, np.tile(-front, (3 * n, 1)), U, Q[:, [1, 0, 3, 2]]))
    return _merge(name, look, parts, visible)


def flat_rect(name: str, look: str, center, size, tile: float = 0.2, visible: bool = True) -> Mesh:
    """One flat rectangle facing up, at `center`, of `size` (x, y)."""
    cx, cy, z = center
    hx, hy = 0.5 * size[0], 0.5 * size[1]
    P = np.array([[cx - hx, cy - hy, z], [cx + hx, cy - hy, z], [cx + hx, cy + hy, z], [cx - hx, cy + hy, z]])
    return _merge(name, look, [(P, np.tile(np.array([0.0, 0.0, 1.0]), (4, 1)), P[:, :2] / tile, np.array([[0, 1, 2, 3]]))], visible)


def flat_disc(name: str, look: str, r0: float, r1: float, z: float, n: int = 96, tile: float = 0.2, center=(0.0, 0.0), visible: bool = True) -> Mesh:
    """A flat ring (or a disc, with r0 = 0) facing up."""
    return lathe(name, look, [[(r1, z), (r0 if r0 > 0 else 1e-4, z)]], n=n, tile=tile, center=(center[0], center[1], 0.0), visible=visible)


def mesh_usda(m: Mesh, root: str, indent: str = "        ") -> str:
    def f3(a):
        return "[" + ",".join(f"({x:.5f},{y:.5f},{z:.5f})" for x, y, z in a) + "]"

    st = "[" + ",".join(f"({u:.4f},{v:.4f})" for u, v in m.st) + "]"
    vis = "" if m.visible else f'\n{indent}    token visibility = "invisible"'
    return f"""
{indent}def Mesh "{m.name}" (
{indent}    prepend apiSchemas = ["MaterialBindingAPI"]
{indent})
{indent}{{{vis}
{indent}    rel material:binding = </{root}/Looks/{m.look}>
{indent}    uniform bool doubleSided = 1
{indent}    uniform token subdivisionScheme = "none"
{indent}    int[] faceVertexCounts = [{",".join(map(str, m.counts.tolist()))}]
{indent}    int[] faceVertexIndices = [{",".join(map(str, m.indices.tolist()))}]
{indent}    point3f[] points = {f3(m.points)}
{indent}    normal3f[] normals = {f3(m.normals)} (
{indent}        interpolation = "vertex"
{indent}    )
{indent}    texCoord2f[] primvars:st = {st} (
{indent}        interpolation = "vertex"
{indent}    )
{indent}}}"""


# -- colliders: simple shapes, never rendered -------------------------------------------------


def col_box(name: str, center, size, orient_wxyz=None, material: str | None = None) -> str:
    rel = f"\n            rel material:binding:physics = <{material}>" if material else ""
    api = '"PhysicsCollisionAPI", "MaterialBindingAPI"' if material else '"PhysicsCollisionAPI"'
    ops = ["xformOp:translate", "xformOp:scale"]
    orient = ""
    if orient_wxyz is not None:
        w, x, y, z = orient_wxyz
        orient = f"\n            quatd xformOp:orient = ({w:.6f}, {x:.6f}, {y:.6f}, {z:.6f})"
        ops = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    order = ", ".join(f'"{o}"' for o in ops)
    return f"""
        def Cube "{name}" (
            prepend apiSchemas = [{api}]
        )
        {{{rel}
            token visibility = "invisible"
            double size = 1
            double3 xformOp:translate = ({center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}){orient}
            float3 xformOp:scale = ({size[0]:.6f}, {size[1]:.6f}, {size[2]:.6f})
            uniform token[] xformOpOrder = [{order}]
        }}"""


def col_cylinder(name: str, center, radius: float, height: float, material: str | None = None) -> str:
    rel = f"\n            rel material:binding:physics = <{material}>" if material else ""
    api = '"PhysicsCollisionAPI", "MaterialBindingAPI"' if material else '"PhysicsCollisionAPI"'
    return f"""
        def Cylinder "{name}" (
            prepend apiSchemas = [{api}]
        )
        {{{rel}
            token visibility = "invisible"
            uniform token axis = "Z"
            double height = {height:.6f}
            double radius = {radius:.6f}
            double3 xformOp:translate = ({center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}"""


def physics_material(root: str, friction: tuple[float, float]) -> str:
    return f"""
    def Material "PhysicsMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI"]
    )
    {{
        float physics:staticFriction = {friction[0]}
        float physics:dynamicFriction = {friction[1]}
        float physics:restitution = 0
    }}"""


def usda_file(root: str, body: str, looks: list[Look], root_api: str = "", attrs: str = "") -> str:
    api = f"(\n    prepend apiSchemas = [{root_api}]\n)" if root_api else ""
    mats = "".join(look_usda(root, k) for k in looks)
    return f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}" {api}
{{{attrs}
    def Scope "Looks"
    {{{mats}
    }}
{body}
}}
"""


# -- parts of a task file ------------------------------------------------------------------


@dataclass(frozen=True)
class Part(Box):
    """A prop spawned from USD text. `pos` places the file's origin; `size` is nominal. `build()` returns the text at spawn.

    `role`: "fixed" (static colliders), "free" (one rigid body, with `mass`), or "jointed" (a fixed articulation; `drives` maps
    joint name to (stiffness, damping, effort))."""

    role: str = "fixed"
    build: Callable[[], str] | None = None
    drives: tuple = ()
    start_joints: tuple = ()  # (joint name, radians): the articulation's joint positions at a reset; others start at 0
    friction: tuple[float, float] = (1.2, 1.0)
    torsional_patch: float | None = None
    solver_iters: int = 16
    yaw_deg: float = 0.0

    def _check(self) -> None:
        if self.role not in ("fixed", "free", "jointed"):
            raise ValueError(f"{self.name}: unknown role {self.role!r}")
        if self.role == "free" and self.mass is None:
            raise ValueError(f"{self.name}: a free part needs a mass")


@dataclass(frozen=True)
class PartState(ObjectState):
    """A part's origin pose and velocities, and its joint positions."""

    joints: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        return quat_matrix(self.quat_xyzw)

    def tilt_deg(self) -> float:
        return float(np.degrees(np.arccos(np.clip(self.rotation()[2, 2], -1.0, 1.0))))

    def to_local(self, points: np.ndarray) -> np.ndarray:
        """World points in this part's frame."""
        return (np.atleast_2d(points) - self.pos) @ self.rotation()

    def to_world(self, points: np.ndarray) -> np.ndarray:
        return np.atleast_2d(points) @ self.rotation().T + self.pos


def quat_matrix(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


_SPECS: dict[str, Part] = {}


def _usd_dir() -> str:
    path = os.path.join(tempfile.gettempdir(), f"kitchenkit_{os.getpid()}")
    os.makedirs(path, exist_ok=True)
    return path


def _arena_object(spec: Part):
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    usd = os.path.join(_usd_dir(), f"{spec.name}.usda")
    with open(usd, "w") as f:
        f.write(spec.build())
    h = math.radians(spec.yaw_deg) / 2
    pose = Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(h), math.cos(h)))
    if spec.role == "fixed":
        return Object(name=spec.name, object_type=ObjectType.BASE, usd_path=usd, initial_pose=pose)
    if spec.role == "jointed":
        drives = dict(spec.drives)

        class Jointed(Object):  # Arena's own articulation object has neither actuators nor initial joint positions
            def _generate_articulation_cfg(self):
                actuators = {n: ImplicitActuatorCfg(joint_names_expr=[n], stiffness=g[0], damping=g[1], joint_effort_limit=g[2],
                                                    friction=g[3] if len(g) > 3 else 0.0, armature=g[4] if len(g) > 4 else 0.0)
                             for n, g in drives.items()}
                start = {n: 0.0 for n in drives} | dict(spec.start_joints)
                cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self._get_spawn_cfg(activate_contact_sensors=False),
                                      init_state=ArticulationCfg.InitialStateCfg(joint_pos=start), actuators=actuators)
                return self._add_initial_pose_to_cfg(cfg)

        return Jointed(name=spec.name, object_type=ObjectType.ARTICULATION, usd_path=usd, initial_pose=pose)
    material = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction[0], dynamic_friction=spec.friction[1])
    collision: dict[str, Any] = {"contact_offset": 0.005, "rest_offset": 0.0}
    if spec.torsional_patch:
        collision.update(torsional_patch_radius=spec.torsional_patch, min_torsional_patch_radius=0.8 * spec.torsional_patch)
    addon = {"physics_material": material, "collision_props": sim_utils.CollisionPropertiesCfg(**collision),
             "rigid_props": sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=spec.solver_iters),
             "mass_props": sim_utils.MassPropertiesCfg(mass=spec.mass)}
    obj = Object(name=spec.name, object_type=ObjectType.RIGID, usd_path=usd, spawn_cfg_addon=addon)
    obj.set_initial_pose(pose)
    return obj


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _state(environment, spec: Part) -> PartState:
    h = math.radians(spec.yaw_deg) / 2
    if spec.role == "fixed":
        pos = np.asarray(spec.pos, float)
        return PartState(pos=pos, quat_xyzw=np.array([0.0, 0.0, math.sin(h), math.cos(h)]), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos)
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0]
    joints = {}
    if spec.role == "jointed":
        q = _np(asset.data.joint_pos)[0]
        joints = {n: float(q[i]) for i, n in enumerate(asset.joint_names)}
    pos = pose[:3] - origin
    return PartState(pos=pos, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6], low=pos, high=pos, joints=joints)


def state(environment, name: str) -> PartState:
    """A part's state now."""
    return _state(environment, _SPECS[name])


def fingertips(environment) -> np.ndarray:
    """Points on both arms' fingers (station frame): tip and 3 cm up it, at the middle and both edges of each finger, opened as
    the fingers are now. Shape (24, 3)."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = _np(robot.data.body_link_pose_w)[0, i]
        R = quat_matrix(pose[3:7])
        q = _np(robot.data.joint_pos)[0]
        opening = abs(float(q[robot.find_joints("joint7")[0][0]]))
        for sx in (-1.0, 1.0):
            for dy in (-0.018, 0.0, 0.018):
                for dz in (0.147, 0.117):
                    out.append(pose[:3] - origin + R @ np.array([sx * (opening + 0.0045), dy, dz]))
    return np.array(out)


def set_visible(path: str, visible: bool) -> None:
    """Show or hide a prim (only when that changes), for anything seen that follows the task's state."""
    import isaaclab.sim as sim_utils
    from pxr import UsdGeom

    if _VISIBLE.get(path) == visible:
        return
    _VISIBLE[path] = visible
    prim = sim_utils.get_current_stage().GetPrimAtPath(path)
    if prim.IsValid():
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if visible else img.MakeInvisible()


_VISIBLE: dict[str, bool] = {}

# -- hooking yam_arena -----------------------------------------------------------------------

_STEP: list = []
_RESET: list = []


def on_step(fn):
    """Run ``fn(env)`` after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Run ``fn(env)`` after every reset, once the scene has settled."""
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_kitchenkit", False):
        return
    task_object, object_states = scene.task_object, scene.object_states

    def task_object_hook(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return task_object(spec)

    def object_states_hook(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = object_states(environment, plain) if plain else {}
        out.update({n: _state(environment, _SPECS[n]) for n in names if n in _SPECS})
        return {n: out[n] for n in names}

    scene.task_object, scene.object_states = task_object_hook, object_states_hook
    scene._kitchenkit = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_kitchenkit", False):
        return
    task_step, reset = cls._task_step, cls.reset

    def task_step_hook(self, *a, **k):
        for fn in _STEP:
            fn(self._env)
        return task_step(self, *a, **k)

    def reset_hook(self, *a, **k):
        obs = reset(self, *a, **k)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = task_step_hook, reset_hook
    cls._kitchenkit = True


class _Finder(importlib.abc.MetaPathFinder):
    """Patches yam_arena's scene and embodiment modules as they are imported (they need Isaac Sim, which starts after the task
    file is loaded)."""

    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        exec_module = loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            patch(module)

        loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    """Hook yam_arena now for modules already imported, and on import for the rest. Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
