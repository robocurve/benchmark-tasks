"""Everyday-looking items for the bag: a hardcover book, a plastic water bottle, a drink can and a juice box.

Each item is a rigid body whose collider is the same primitive the scene used before (a box or a cylinder of the item's
size, centred on the body's origin, cylinder axis z), kept out of the render (purpose "guide"), and whose look comes from
visual-only meshes: a lathe profile for the bottle and the can, boxes for the book and the juice box. So the physics is
unchanged and only the graphics differ.
"""

from __future__ import annotations

import os

import numpy as np

Mesh = tuple[np.ndarray, np.ndarray]  # (vertices [n, 3], triangles [m, 3])

# name: (diffuse colour, metallic, roughness)
MATERIALS = {
    "cover": ((0.10, 0.28, 0.21), 0.0, 0.6),
    "pages": ((0.93, 0.90, 0.80), 0.0, 0.9),
    "gilt": ((0.78, 0.62, 0.25), 0.6, 0.4),
    "bottle": ((0.72, 0.86, 0.95), 0.0, 0.12),
    "label": ((0.12, 0.42, 0.78), 0.0, 0.5),
    "cap": ((0.10, 0.30, 0.70), 0.0, 0.4),
    "can_metal": ((0.78, 0.79, 0.80), 0.9, 0.3),
    "can_print": ((0.78, 0.06, 0.08), 0.3, 0.35),
    "carton": ((0.97, 0.58, 0.10), 0.0, 0.6),
    "carton_top": ((0.96, 0.95, 0.92), 0.0, 0.6),
    "straw": ((0.98, 0.98, 0.97), 0.0, 0.5),
    "foil": ((0.80, 0.80, 0.82), 0.8, 0.35),
}


def box(size, center=(0.0, 0.0, 0.0)) -> Mesh:
    sx, sy, sz = (0.5 * np.asarray(size, float))
    v = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)]) + np.asarray(center, float)
    f = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return v, f


def lathe(profile: list[tuple[float, float]], segments: int = 40) -> Mesh:
    """Revolve a (radius, z) profile about the z axis; a profile point of radius 0 closes the surface there."""
    th = np.linspace(0.0, 2 * np.pi, segments, endpoint=False)
    rings, verts = [], []
    for r, z in profile:
        if r == 0.0:
            rings.append([len(verts)])
            verts.append((0.0, 0.0, z))
        else:
            rings.append(list(range(len(verts), len(verts) + segments)))
            verts += [(r * np.cos(t), r * np.sin(t), z) for t in th]
    faces = []
    for a, b in zip(rings[:-1], rings[1:]):
        if len(a) == 1 and len(b) > 1:
            faces += [(a[0], b[(i + 1) % segments], b[i]) for i in range(segments)]
        elif len(b) == 1 and len(a) > 1:
            faces += [(a[i], a[(i + 1) % segments], b[0]) for i in range(segments)]
        elif len(a) > 1:
            for i in range(segments):
                j = (i + 1) % segments
                faces += [(a[i], a[j], b[j]), (a[i], b[j], b[i])]
    return np.array(verts), np.array(faces)


def cylinder_along(p0, p1, r: float, segments: int = 12) -> Mesh:
    """A closed cylinder of radius r from p0 to p1."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    v, f = lathe([(0.0, 0.0), (r, 0.0), (r, 1.0), (0.0, 1.0)], segments)
    axis = p1 - p0
    length = np.linalg.norm(axis)
    w = axis / length
    u = np.cross(w, [0.0, 0.0, 1.0] if abs(w[2]) < 0.9 else [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    R = np.column_stack([u, np.cross(w, u), w])
    v = v * [1.0, 1.0, length]
    return v @ R.T + p0, f


def book_parts(size) -> list[tuple[str, Mesh, str]]:
    """A hardcover standing on its bottom edge: covers normal to x, spine at -y, a cream page block inset 3 mm."""
    sx, sy, sz = size
    t, inset = 0.002, 0.003
    return [
        ("front_cover", box((t, sy, sz), (0.5 * sx - 0.5 * t, 0.0, 0.0)), "cover"),
        ("back_cover", box((t, sy, sz), (-0.5 * sx + 0.5 * t, 0.0, 0.0)), "cover"),
        ("spine", box((sx, t, sz), (0.0, -0.5 * sy + 0.5 * t, 0.0)), "cover"),
        ("pages", box((sx - 2 * t, sy - t - inset, sz - 2 * inset), (0.0, -0.5 * sy + t + 0.5 * (sy - t - inset), 0.0)), "pages"),
        ("title_band", box((0.6 * sx, 0.0006, 0.012), (0.0, -0.5 * sy - 0.0003, 0.30 * sz)), "gilt"),
        ("base_band", box((0.6 * sx, 0.0006, 0.004), (0.0, -0.5 * sy - 0.0003, -0.38 * sz)), "gilt"),
    ]


def bottle_parts(r: float, h: float) -> list[tuple[str, Mesh, str]]:
    """A plastic water bottle: body, shoulder and neck, a screw cap, and a printed label band."""
    z0, z1 = -0.5 * h, 0.5 * h
    cap_h, neck_r, cap_r = 0.022, 0.5 * r, 0.56 * r
    shoulder0, shoulder1 = z1 - 0.36 * h, z1 - cap_h - 0.012
    body = lathe([(0.0, z0), (0.85 * r, z0), (r, z0 + 0.008), (r, shoulder0), (0.85 * r, shoulder0 + 0.35 * (shoulder1 - shoulder0)),
                  (0.55 * r, shoulder1 - 0.004), (neck_r, shoulder1), (neck_r, z1 - cap_h), (0.0, z1 - cap_h)])
    cap = lathe([(0.0, z1 - cap_h), (cap_r, z1 - cap_h), (cap_r, z1), (0.0, z1)], 32)
    label = lathe([(1.006 * r, z0 + 0.28 * h), (1.006 * r, z0 + 0.55 * h)], 48)
    return [("body", body, "bottle"), ("cap", cap, "cap"), ("label", label, "label")]


def can_parts(r: float, h: float) -> list[tuple[str, Mesh, str]]:
    """A drink can: an aluminium shell with a tapered top and bottom, a printed band, and a pull tab on the lid."""
    z0, z1 = -0.5 * h, 0.5 * h
    shell = lathe([(0.0, z0 + 0.003), (0.78 * r, z0), (0.9 * r, z0 + 0.002), (r, z0 + 0.007), (r, z1 - 0.008), (0.86 * r, z1 - 0.002),
                   (0.88 * r, z1), (0.80 * r, z1), (0.80 * r, z1 - 0.0015), (0.0, z1 - 0.0015)], 48)
    band = lathe([(1.004 * r, z0 + 0.010), (1.004 * r, z1 - 0.011)], 48)
    tab = box((0.012, 0.007, 0.0008), (0.004, 0.0, z1 - 0.001))
    return [("shell", shell, "can_metal"), ("print", band, "can_print"), ("tab", tab, "can_metal")]


def juice_parts(size) -> list[tuple[str, Mesh, str]]:
    """A juice box: an orange carton with a white top band, a foil straw hole and the wrapped straw on its side."""
    sx, sy, sz = size
    top_h = 0.009
    fv, ff = lathe([(0.0, 0.5 * sz), (0.004, 0.5 * sz), (0.004, 0.5 * sz + 0.0006), (0.0, 0.5 * sz + 0.0006)], 16)
    straw0 = (0.5 * sx + 0.0022, -0.38 * sy, -0.40 * sz)
    straw1 = (0.5 * sx + 0.0022, 0.38 * sy, 0.30 * sz)
    return [
        ("carton", box((sx, sy, sz - top_h), (0.0, 0.0, -0.5 * top_h)), "carton"),
        ("top", box((sx, sy, top_h), (0.0, 0.0, 0.5 * sz - 0.5 * top_h)), "carton_top"),
        ("foil", (fv + [0.004, 0.008, 0.0], ff), "foil"),
        ("straw", cylinder_along(straw0, straw1, 0.002), "straw"),
    ]


def item_parts(spec: dict) -> list[tuple[str, Mesh, str]]:
    name = spec["name"]
    if name == "book":
        return book_parts(spec["size"])
    if name == "bottle":
        return bottle_parts(spec["radius"], spec["height"])
    if name == "can":
        return can_parts(spec["radius"], spec["height"])
    if name == "juice":
        return juice_parts(spec["size"])
    raise ValueError(f"no visual for item {name!r}")


def _mesh_usda(name: str, mesh: Mesh, mat: str, root: str) -> str:
    v, f = mesh
    points = ", ".join(f"({x:.5f}, {y:.5f}, {z:.5f})" for x, y, z in v)
    indices = ", ".join(str(int(i)) for i in np.asarray(f).reshape(-1))
    counts = ", ".join("3" for _ in range(len(f)))
    return f"""
        def Mesh "{name}" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {{
            rel material:binding = </{root}/Looks/{mat}>
            uniform bool doubleSided = 1
            int[] faceVertexCounts = [{counts}]
            int[] faceVertexIndices = [{indices}]
            point3f[] points = [{points}]
            uniform token subdivisionScheme = "none"
        }}"""


def cube_collider(name: str, size, center, root: str) -> str:
    """A box collider (not rendered: purpose "guide") of `size`, centred at `center` in the body frame."""
    sx, sy, sz = size
    cx, cy, cz = center
    return f"""
    def Cube "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
    )
    {{
        rel material:binding:physics = </{root}/PhysicsMaterial>
        uniform token purpose = "guide"
        double size = 1
        double3 xformOp:translate = ({cx:.6f}, {cy:.6f}, {cz:.6f})
        float3 xformOp:scale = ({sx:.6f}, {sy:.6f}, {sz:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}"""


def cylinder_collider(name: str, radius: float, height: float, axis: str, center, root: str) -> str:
    cx, cy, cz = center
    return f"""
    def Cylinder "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
    )
    {{
        rel material:binding:physics = </{root}/PhysicsMaterial>
        uniform token purpose = "guide"
        uniform token axis = "{axis}"
        double height = {height:.6f}
        double radius = {radius:.6f}
        double3 xformOp:translate = ({cx:.6f}, {cy:.6f}, {cz:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}"""


def write_rigid_usda(path: str, root: str, mass: float, colliders: list[str], parts: list[tuple[str, Mesh, str]],
                     materials: dict, friction: tuple[float, float], combine: str | None = None) -> str:
    """Write a rigid body: collider prims (see cube_collider / cylinder_collider) and visual meshes bound to
    UsdPreviewSurface materials (name: (colour, metallic, roughness))."""
    used = sorted({m for _, _, m in parts})
    looks = "".join(f"""
        def Material "{m}"
        {{
            token outputs:surface.connect = </{root}/Looks/{m}/Shader.outputs:surface>
            def Shader "Shader"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = ({materials[m][0][0]}, {materials[m][0][1]}, {materials[m][0][2]})
                float inputs:metallic = {materials[m][1]}
                float inputs:roughness = {materials[m][2]}
                token outputs:surface
            }}
        }}""" for m in used)
    meshes = "".join(_mesh_usda(n, mesh, m, root) for n, mesh, m in parts)
    combine_line = f'\n        uniform token physxMaterial:frictionCombineMode = "{combine}"' if combine else ""
    schemas = '["PhysicsMaterialAPI", "PhysxMaterialAPI"]' if combine else '["PhysicsMaterialAPI"]'
    text = f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{{
    float physics:mass = {mass:.4f}

    def Material "PhysicsMaterial" (
        prepend apiSchemas = {schemas}
    )
    {{
        float physics:staticFriction = {friction[0]:.4f}
        float physics:dynamicFriction = {friction[1]:.4f}
        float physics:restitution = 0{combine_line}
    }}
{"".join(colliders)}

    def Scope "Looks"
    {{{looks}
    }}

    def Xform "visual"
    {{{meshes}
    }}
}}
"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path


def write_item_usda(path: str, spec: dict, mass: float, friction: tuple[float, float] = (0.7, 0.6)) -> str:
    """Write one item as a rigid body: an invisible collider primitive of the item's size plus its visual meshes."""
    root = spec["name"].capitalize()
    if spec["shape"] == "box":
        collider = cube_collider("collider", spec["size"], (0.0, 0.0, 0.0), root)
    else:
        collider = cylinder_collider("collider", spec["radius"], spec["height"], "Z", (0.0, 0.0, 0.0), root)
    return write_rigid_usda(path, root, mass, [collider], item_parts(spec), MATERIALS, friction)
