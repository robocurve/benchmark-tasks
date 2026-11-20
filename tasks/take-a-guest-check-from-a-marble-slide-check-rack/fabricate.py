"""Generate print-ready ASCII STL files (in mm) for the kitchen pass shelf stand that mounts an off-the-shelf
San Jamar CK6512A 12-inch slide check rack 195 mm above the station tabletop, matching the exact scene dimensions.

The check rack (San Jamar CK6512A), guest checks (NCCO G3632 green / T3624-SP tan) and clipless black check tray
(Winco WDM018-301) are commercial off-the-shelf products. This script only fabricates the rigid mounting stand parts
(apron shelf top, two rear support legs and base foot) so any lab can position the CK6512A reproducibly behind the
hanging checks without obstructing the grasp region.

Run as a script (writes no files on import):
    python3 fabricate.py --out ./stl_out
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def _tri_normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    n = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (nx / n, ny / n, nz / n) if n > 1e-12 else (0.0, 0.0, 0.0)


def _box_tris(x0, x1, y0, y1, z0, z1):
    v = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]
    faces = [
        (0, 3, 2, 1), (4, 5, 6, 7),
        (0, 1, 5, 4), (2, 3, 7, 6),
        (0, 4, 7, 3), (1, 2, 6, 5),
    ]
    tris = []
    for a, b, c, d in faces:
        tris.append((v[a], v[b], v[c]))
        tris.append((v[a], v[c], v[d]))
    return tris


def _cyl_z_tris(cx, cy, r, z0, z1, sides=32):
    tris = []
    bot_c, top_c = (cx, cy, z0), (cx, cy, z1)
    ring_b = [(cx + r * math.cos(2 * math.pi * i / sides), cy + r * math.sin(2 * math.pi * i / sides), z0) for i in range(sides)]
    ring_t = [(p[0], p[1], z1) for p in ring_b]
    for i in range(sides):
        j = (i + 1) % sides
        tris.append((bot_c, ring_b[j], ring_b[i]))
        tris.append((top_c, ring_t[i], ring_t[j]))
        tris.append((ring_b[i], ring_b[j], ring_t[j]))
        tris.append((ring_b[i], ring_t[j], ring_t[i]))
    return tris


def _write_stl(path: Path, solid_name: str, tris) -> None:
    lines = [f"solid {solid_name}"]
    for a, b, c in tris:
        nx, ny, nz = _tri_normal(a, b, c)
        lines.append(f"  facet normal {nx:.6f} {ny:.6f} {nz:.6f}")
        lines.append("    outer loop")
        for p in (a, b, c):
            lines.append(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append(f"endsolid {solid_name}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_parts(out_dir: Path) -> list[Path]:
    """Write ASCII STLs (mm) for the pass shelf stand mounting the San Jamar CK6512A at z = 195 mm."""
    out_dir.mkdir(parents=True, exist_ok=True)
    written = []

    # 1. Pass shelf + front mounting apron (all dimensions in mm matching scene constants)
    # Apron: 10 (x) x 305 (y) x 51 (z) at front; Shelf top: 105 (x) x 320 (y) x 15 (z) behind apron
    apron_shelf = (
        _box_tris(0.0, 10.0, -152.5, 152.5, 0.0, 51.0)
        + _box_tris(10.0, 115.0, -160.0, 160.0, 36.0, 51.0)
    )
    p1 = out_dir / "pass_shelf_apron.stl"
    _write_stl(p1, "pass_shelf_apron", apron_shelf)
    written.append(p1)

    # 2. Rear support leg (print two; 22 mm diameter x 219 mm tall; bolts between base and shelf underside)
    leg = _cyl_z_tris(0.0, 0.0, 11.0, 0.0, 219.0, sides=36)
    p2 = out_dir / "pass_stand_leg.stl"
    _write_stl(p2, "pass_stand_leg", leg)
    written.append(p2)

    # 3. Weighted base foot (70 x 320 x 12 mm; clamp or double-side tape to tabletop at x = 425 mm)
    base = _box_tris(-35.0, 35.0, -160.0, 160.0, 0.0, 12.0)
    p3 = out_dir / "pass_stand_base.stl"
    _write_stl(p3, "pass_stand_base", base)
    written.append(p3)

    return written


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=Path("stl_out"), help="Output directory for ASCII STL files")
    args = ap.parse_args()
    files = build_parts(args.out)
    for f in files:
        print(f"wrote {f} ({f.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
