"""Write a triangle mesh as a plain-text USD file (no pxr needed), shaped for Isaac Lab's deformable spawner."""

from __future__ import annotations

import os

import trimesh


def write_mesh_usda(mesh: trimesh.Trimesh, path: str, root_name: str = "Root") -> str:
    """Write `mesh` as an Xform `root_name` holding one Mesh child, in metres with +z up. Returns `path`."""
    points = ", ".join(f"({x:.6f}, {y:.6f}, {z:.6f})" for x, y, z in mesh.vertices)
    indices = ", ".join(str(int(i)) for i in mesh.faces.reshape(-1))
    counts = ", ".join("3" for _ in range(len(mesh.faces)))
    text = f"""#usda 1.0
(
    defaultPrim = "{root_name}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root_name}"
{{
    def Mesh "mesh"
    {{
        uniform bool doubleSided = 1
        int[] faceVertexCounts = [{counts}]
        int[] faceVertexIndices = [{indices}]
        point3f[] points = [{points}]
        uniform token subdivisionScheme = "none"
    }}
}}
"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path
