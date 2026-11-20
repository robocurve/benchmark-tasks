"""Write the five food stand-ins as STL files in millimetres, for printing the real trial's replicas at the simulated sizes.

    python export_food_stl.py OUT_DIR        (in the harness's Python, as for demokit.py)

One file per item (steak, hotdog, wing, roll; print the wing twice). Each is the item's visible shape as in the scene, its parts as
overlapping closed shells, which a slicer merges. Print in PETG at the infill that gives the mass in task.json's setup; weigh each print.
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import task as t  # noqa: E402


def write_stl(path: pathlib.Path, meshes) -> None:
    with open(path, "w") as f:
        f.write(f"solid {path.stem}\n")
        for mesh in meshes:
            v = np.asarray(mesh.v, float) * 1000.0
            for a, b, c in np.asarray(mesh.f, int):
                n = np.cross(v[b] - v[a], v[c] - v[a])
                n = n / (np.linalg.norm(n) or 1.0)
                f.write(f"facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n outer loop\n")
                for p in (v[a], v[b], v[c]):
                    f.write(f"  vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
                f.write(" endloop\nendfacet\n")
        f.write(f"endsolid {path.stem}\n")


def main() -> None:
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "food_stl")
    out.mkdir(parents=True, exist_ok=True)
    lx, wy, h = t.FOOD["steak"][:3]
    items = {"steak": t._steak(lx, wy, h)[0], "hotdog": t._hotdog(*t.FOOD["hotdog"][:3])[0], "wing": t._wing()[0], "roll": t._roll()[0]}
    for name, parts in items.items():
        meshes = [p[1] for p in parts if not p[0].startswith("mark")]  # grill marks are paint, not shape
        write_stl(out / f"{name}.stl", meshes)
        v = np.vstack([mm.v for mm in meshes])
        print(name, "size (mm):", np.round((v.max(0) - v.min(0)) * 1000, 1).tolist())


if __name__ == "__main__":
    main()
