"""A steel industrial shelf as a visual-only USD: a perforated galvanised deck framed by edge beams, standing on four
slotted-angle uprights with foot plates and a tie beam on each side near the floor. The environment adds the colliders
(invisible boxes), so the holes are only seen, never felt."""

from __future__ import annotations

import os

import numpy as np
import trimesh

# (diffuse colour, metallic, roughness)
MATERIALS = {
    "Deck": ((0.60, 0.62, 0.64), 0.8, 0.35),  # galvanised steel
    "Beam": ((0.80, 0.33, 0.05), 0.3, 0.5),  # orange edge and tie beams
    "Upright": ((0.07, 0.20, 0.42), 0.3, 0.5),  # blue slotted angle
    "Foot": ((0.12, 0.12, 0.13), 0.5, 0.6),
}


def _intervals(length: float, centres: np.ndarray, size: float) -> list[tuple[float, float]]:
    """The solid intervals of [-length/2, length/2] left between holes of width `size` at `centres`."""
    out, lo = [], -length / 2
    for c in centres:
        if c - size / 2 > lo:
            out.append((lo, c - size / 2))
        lo = c + size / 2
    if length / 2 > lo:
        out.append((lo, length / 2))
    return out


def _centres(length: float, pitch: float, hole: float, margin: float) -> np.ndarray:
    n = int((length - 2 * margin - hole) // pitch) + 1
    return (np.arange(n) - (n - 1) / 2) * pitch if n > 0 else np.zeros(0)


def perforated_plate(lu: float, lv: float, t: float, pitch: tuple[float, float], hole: tuple[float, float],
                     margin: float) -> trimesh.Trimesh:
    """A plate of lu x lv x t (u along x, v along y, thickness along z) centred at the origin, with rectangular holes
    of hole[0] x hole[1] on a pitch[0] x pitch[1] grid, at least `margin` from the edges. Built from boxes."""
    cu = _centres(lu, pitch[0], hole[0], margin)
    cv = _centres(lv, pitch[1], hole[1], margin)
    boxes = []
    for v0, v1 in _intervals(lv, cv, hole[1]):  # full-width strips between the hole rows
        boxes.append(trimesh.creation.box((lu, v1 - v0, t), trimesh.transformations.translation_matrix((0, (v0 + v1) / 2, 0))))
    for v in cv:  # the bars between the holes of each row
        for u0, u1 in _intervals(lu, cu, hole[0]):
            boxes.append(trimesh.creation.box((u1 - u0, hole[1], t), trimesh.transformations.translation_matrix(((u0 + u1) / 2, v, 0))))
    return trimesh.util.concatenate(boxes)


def _place(mesh: trimesh.Trimesh, axes: tuple[int, int, int], center) -> trimesh.Trimesh:
    """Map the local u, v, w axes onto world axes `axes` (e.g. (0, 2, 1): u -> x, v -> z, w -> y) and move to `center`."""
    m = mesh.copy()
    T = np.eye(4)
    T[:3, :3] = 0.0
    for local, world in enumerate(axes):
        T[world, local] = 1.0
    if np.linalg.det(T[:3, :3]) < 0:  # keep a proper rotation (no mirrored faces): flip w, the plate is symmetric in it
        T[:3, 2] *= -1.0
    m.apply_transform(T)
    m.apply_translation(center)
    return m


def _box(size, center) -> trimesh.Trimesh:
    return trimesh.creation.box(size, trimesh.transformations.translation_matrix(center))


def shelf_parts(x: tuple[float, float], y: tuple[float, float], top: float, board: float, floor: float) -> dict:
    """The shelf's meshes by material, in the robot root frame. `top` is the deck's upper surface; `board` the depth of
    the frame under it; `floor` the floor height."""
    x0, x1 = x
    y0, y1 = y
    xc, yc, d, w = 0.5 * (x0 + x1), 0.5 * (y0 + y1), x1 - x0, y1 - y0
    t_deck, t_steel, flange = 0.003, 0.003, 0.045
    parts = {name: [] for name in MATERIALS}
    # Deck: perforated plate, 15 mm square holes on a 30 mm grid.
    deck = perforated_plate(d - 2 * t_steel, w - 2 * t_steel, t_deck, (0.03, 0.03), (0.015, 0.015), margin=0.02)
    parts["Deck"].append(_place(deck, (0, 1, 2), (xc, yc, top - t_deck / 2)))
    # Edge beams round the deck: a skirt `board` deep, with a 2 cm return flange under it.
    zb = top - board / 2
    for yy in (y0 + t_steel / 2, y1 - t_steel / 2):
        parts["Beam"].append(_box((d, t_steel, board), (xc, yy, zb)))
    for xx in (x0 + t_steel / 2, x1 - t_steel / 2):
        parts["Beam"].append(_box((t_steel, w, board), (xx, yc, zb)))
    for xx, s in ((x0, 1), (x1, -1)):
        parts["Beam"].append(_box((0.02, w, t_steel), (xx + s * 0.01, yc, top - board + t_steel / 2)))
    # Uprights: slotted angle (two 45 mm flanges, 10 x 25 mm slots on a 50 mm pitch) at each corner, outside faces flush.
    h = top - board - floor
    zc = floor + h / 2
    slot = perforated_plate(flange, h, t_steel, (0.05, 0.05), (0.01, 0.025), margin=0.012)
    for cx, sx in ((x0, 1), (x1, -1)):
        for cy, sy in ((y0, 1), (y1, -1)):
            # flange in the x-z plane (on the long side), and in the y-z plane (on the short side)
            parts["Upright"].append(_place(slot, (0, 2, 1), (cx + sx * flange / 2, cy + sy * t_steel / 2, zc)))
            parts["Upright"].append(_place(slot, (1, 2, 0), (cx + sx * t_steel / 2, cy + sy * flange / 2, zc)))
            parts["Foot"].append(_box((0.08, 0.08, 0.006), (cx + sx * 0.035, cy + sy * 0.035, floor + 0.003)))
    # Tie beams 15 cm above the floor, between the uprights on every side.
    zt = floor + 0.15
    for yy in (y0 + 0.012, y1 - 0.012):
        parts["Beam"].append(_box((d - 2 * flange, 0.02, 0.04), (xc, yy, zt)))
    for xx in (x0 + 0.012, x1 - 0.012):
        parts["Beam"].append(_box((0.02, w - 2 * flange, 0.04), (xx, yc, zt)))
    return {name: trimesh.util.concatenate(ms) for name, ms in parts.items() if ms}


def write_shelf_usda(path: str, parts: dict, root_name: str = "Shelf") -> str:
    """Write the parts as Mesh prims, each bound to a UsdPreviewSurface material. Visual only (no physics APIs)."""
    mats, meshes = [], []
    for name, (rgb, metallic, rough) in MATERIALS.items():
        mats.append(f"""
    def Material "{name}Mat"
    {{
        token outputs:surface.connect = </{root_name}/{name}Mat/Shader.outputs:surface>
        def Shader "Shader"
        {{
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor = ({rgb[0]}, {rgb[1]}, {rgb[2]})
            float inputs:metallic = {metallic}
            float inputs:roughness = {rough}
            token outputs:surface
        }}
    }}""")
    for name, mesh in parts.items():
        points = ", ".join(f"({x:.5f}, {y:.5f}, {z:.5f})" for x, y, z in mesh.vertices)
        indices = ", ".join(str(int(i)) for i in mesh.faces.reshape(-1))
        counts = ", ".join("3" for _ in range(len(mesh.faces)))
        meshes.append(f"""
    def Mesh "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        rel material:binding = </{root_name}/{name}Mat>
        uniform bool doubleSided = 1
        int[] faceVertexCounts = [{counts}]
        int[] faceVertexIndices = [{indices}]
        point3f[] points = [{points}]
        uniform token subdivisionScheme = "none"
    }}""")
    text = f"""#usda 1.0
(
    defaultPrim = "{root_name}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root_name}"
{{{"".join(mats)}{"".join(meshes)}
}}
"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path
