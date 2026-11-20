"""Print-ready ASCII STL files (mm) for optional real-world setup, scoring, and calibration aids for the
door reinforcement lock task, built from the scene's exact dimensions.

Run:  python3 fabricate.py [out_dir]      (default: ./stl next to this file)

The lock mechanism itself is an off-the-shelf commercial Prime-Line U 10827 Satin Nickel Door Reinforcement Lock
mounted on a standard lumber door-and-jamb L-mock-up; no custom 3D-printed part carries the mechanism. These three
optional prints speed up repeatable bench setup and overhead-photo scoring:
  1. jamb_protractor_plate.stl : top-mounted angle & slide-travel reference plate (0, 20, 78, 84, and 90 deg radial
     ribs around the hinge pin axis) that sits flush atop the jamb mounting plate for unambiguous top-camera scoring.
  2. reveal_spacer_jig.stl     : 3.0 mm door-to-jamb reveal spacer with a 125.0 mm lock-centerline step to align the
     door slab, jamb post, and Prime-Line U 10827 mounting plate repeatably on the lumber mock-up.
  3. block_clearance_gauge.stl : 76.2 x 35.0 x 25.0 mm full-size U 10827 stop-block envelope check gauge with 66.2 mm
     recessed top/bottom finger grooves (29.0 mm rear pocket, 3.0 mm shoulders) for verifying gripper alignment.

Standard library only; importing this file writes nothing.
"""

from __future__ import annotations

import math
import os
import sys

# ---- dimensions, mm (matched to unlock_door_reinforcement_lock.py) --------------------------------------------------
REVEAL_GAP = 3.0
LOCK_CENTER_Z = 125.0

BLOCK_H = 76.2      # 3.0 in
BLOCK_W = 35.0      # 1-3/8 in
BLOCK_D = 25.0      # 1.0 in projection
GROOVE_SPAN = 66.2  # 5.0 mm deep top and bottom grooves
SHOULDER_W = 3.0    # 3.0 mm rear y-end shoulders -> 29.0 mm pocket around pad tips

PROTRACTOR_R_OUT = 42.0
PROTRACTOR_R_IN = 6.5
PROTRACTOR_T = 2.5


# ---- watertight mesh helpers ----------------------------------------------------------------------------------------
def box_tris(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float):
    """Watertight outward-facing axis-aligned box triangles."""
    v = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]
    faces = [
        (0, 2, 1), (0, 3, 2),  # -z
        (4, 5, 6), (4, 6, 7),  # +z
        (0, 1, 5), (0, 5, 4),  # -y
        (2, 3, 7), (2, 7, 6),  # +y
        (0, 4, 7), (0, 7, 3),  # -x
        (1, 2, 6), (1, 6, 5),  # +x
    ]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def extrude_poly_z(poly: list[tuple[float, float]], z0: float, z1: float):
    """Watertight z-extrusion of a simple counter-clockwise 2D polygon (fan-triangulated for star/convex shapes,
    ear-clipped for general simple polygons)."""
    # Ensure CCW orientation
    area2 = sum(
        poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
        for i in range(len(poly))
    )
    pts = list(poly) if area2 > 0 else list(reversed(poly))

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def in_tri(p, a, b, c):
        c1 = cross(a, b, p)
        c2 = cross(b, c, p)
        c3 = cross(c, a, p)
        return c1 >= -1e-9 and c2 >= -1e-9 and c3 >= -1e-9

    idx = list(range(len(pts)))
    tri_idx = []
    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        clipped = False
        n = len(idx)
        for k in range(n):
            ia, ib, ic = idx[(k - 1) % n], idx[k], idx[(k + 1) % n]
            a, b, c = pts[ia], pts[ib], pts[ic]
            if cross(a, b, c) <= 1e-9:
                continue
            if any(in_tri(pts[j], a, b, c) for j in idx if j not in (ia, ib, ic)):
                continue
            tri_idx.append((ia, ib, ic))
            idx.pop(k)
            clipped = True
            break
        if not clipped:
            raise ValueError("polygon ear-clipping failed")
    if len(idx) == 3:
        tri_idx.append((idx[0], idx[1], idx[2]))

    bot = [(x, y, z0) for x, y in pts]
    top = [(x, y, z1) for x, y in pts]
    tris = []
    for a, b, c in tri_idx:
        tris.append((bot[a], bot[c], bot[b]))
        tris.append((top[a], top[b], top[c]))
    m = len(pts)
    for i in range(m):
        j = (i + 1) % m
        tris.append((bot[i], bot[j], top[j]))
        tris.append((bot[i], top[j], top[i]))
    return outward(tris)


def volume(tris) -> float:
    return (
        sum(
            p[0] * (q[1] * r[2] - q[2] * r[1])
            - p[1] * (q[0] * r[2] - q[2] * r[0])
            + p[2] * (q[0] * r[1] - q[1] * r[0])
            for p, q, r in tris
        )
        / 6.0
    )


def outward(tris):
    return tris if volume(tris) > 0 else [(p, r, q) for p, q, r in tris]


def edges_ok(tris) -> bool:
    seen: dict = {}
    for t in tris:
        for a, b in ((t[0], t[1]), (t[1], t[2]), (t[2], t[0])):
            k = (tuple(round(c, 4) for c in a), tuple(round(c, 4) for c in b))
            seen[k] = seen.get(k, 0) + 1
    return all(c == 1 and seen.get((b, a)) == 1 for (a, b), c in seen.items())


def bounds(tris):
    lo = [min(v[i] for t in tris for v in t) for i in range(3)]
    hi = [max(v[i] for t in tris for v in t) for i in range(3)]
    return lo, hi


def write_stl(path: str, name: str, tris) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"solid {name}\n")
        for p, q, r in tris:
            ux, uy, uz = (q[i] - p[i] for i in range(3))
            vx, vy, vz = (r[i] - p[i] for i in range(3))
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            m = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(f"  facet normal {nx / m:.6e} {ny / m:.6e} {nz / m:.6e}\n    outer loop\n")
            for v in (p, q, r):
                f.write(f"      vertex {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def jamb_protractor_plate():
    """90-degree annular sector protractor plate (0..90 deg around hinge axis, r 6.5..42.0 mm, thickness 2.5 mm)
    with notched radial index ticks at 20 deg (Milestone C), 78 deg (0.50), and 84 deg (1.00)."""
    # Angles in hinge frame: 0 deg locked = +y (90 deg polar) to 90 deg open = -x (180 deg polar)
    n_arc = 90
    outer = []
    for k in range(n_arc + 1):
        th = float(k)  # lock angle 0..90 deg
        ang = math.radians(90.0 + th)
        # 2.5 mm radial notch at scoring marks 20, 78, 84 deg
        r = PROTRACTOR_R_OUT - (2.5 if th in (20.0, 78.0, 84.0) else 0.0)
        outer.append((r * math.cos(ang), r * math.sin(ang)))
    inner = []
    for k in range(n_arc, -1, -1):
        ang = math.radians(90.0 + float(k))
        inner.append((PROTRACTOR_R_IN * math.cos(ang), PROTRACTOR_R_IN * math.sin(ang)))
    return extrude_poly_z(outer + inner, 0.0, PROTRACTOR_T)


def reveal_spacer_jig():
    """L-shaped bench alignment jig: 3.0 mm reveal blade (45 mm deep) joined to a 125.0 mm height reference bar
    for positioning the door slab, jamb post, and Prime-Line U 10827 centerline."""
    # 2D profile in (x = blade thickness / bar width, y = height 0..125 mm), extruded 25 mm along z
    poly = [
        (0.0, 0.0),
        (18.0, 0.0),
        (18.0, 12.0),
        (REVEAL_GAP, 12.0),
        (REVEAL_GAP, LOCK_CENTER_Z),
        (0.0, LOCK_CENTER_Z),
    ]
    return extrude_poly_z(poly, 0.0, 25.0)


def block_clearance_gauge():
    """Full-size 76.2 x 35.0 x 25.0 mm Prime-Line U 10827 stop-block envelope gauge with 66.2 mm recessed top/bottom
    finger grooves (29.0 mm flat recess, 3.0 mm y-end shoulders) for dry-run gripper clearance checks."""
    # Profile in (y = 0..35 mm, z = 0..76.2 mm), extruded 25.0 mm along depth
    g_depth = 0.5 * (BLOCK_H - GROOVE_SPAN)  # 5.0 mm
    poly = [
        (0.0, 0.0),
        (SHOULDER_W, 0.0),
        (SHOULDER_W, g_depth),
        (BLOCK_W - SHOULDER_W, g_depth),
        (BLOCK_W - SHOULDER_W, 0.0),
        (BLOCK_W, 0.0),
        (BLOCK_W, BLOCK_H),
        (BLOCK_W - SHOULDER_W, BLOCK_H),
        (BLOCK_W - SHOULDER_W, BLOCK_H - g_depth),
        (SHOULDER_W, BLOCK_H - g_depth),
        (SHOULDER_W, BLOCK_H),
        (0.0, BLOCK_H),
    ]
    return extrude_poly_z(poly, 0.0, BLOCK_D)


PARTS = (
    ("jamb_protractor_plate", jamb_protractor_plate),
    ("reveal_spacer_jig", reveal_spacer_jig),
    ("block_clearance_gauge", block_clearance_gauge),
)


def main(out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    for name, builder in PARTS:
        tris = builder()
        assert edges_ok(tris), f"{name}: mesh is not watertight"
        assert volume(tris) > 0.0, f"{name}: non-positive volume"
        lo, hi = bounds(tris)
        ext = [hi[i] - lo[i] for i in range(3)]
        path = os.path.join(out_dir, f"{name}.stl")
        write_stl(path, name, tris)
        print(f"wrote {path}  ({len(tris)} tris, {ext[0]:.1f} x {ext[1]:.1f} x {ext[2]:.1f} mm)")


if __name__ == "__main__":
    default_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
    main(sys.argv[1] if len(sys.argv) > 1 else default_dir)
