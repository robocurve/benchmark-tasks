"""The task's parts as USD files in build/: the telescope (fixed), its accessory case (fixed), the front solar filter
with its thumbscrew (an articulation), the finder's cap, and the eyepiece with the old "SUN" eyepiece filter (one
articulation: the filter hangs from the eyepiece on six free joints, which the thread rule in task.py drives once the
filter's thread has caught).

Sizes are from real products: a 70 mm f/5.7 travel refractor (400 mm focal length) with a 6x30 finder and a 1.25-inch
star diagonal holding a 1.25-inch eyepiece, a slip-on full-aperture film filter for an 84 mm tube held by one nylon
1/4-20 thumbscrew with felt opposite, a slip-on finder cap and a 1.25-inch eyepiece filter (male M28.5 thread). Colliders are convex hulls of the visual shapes (rings are rings
of hull segments), so the visual and the collider differ by well under 2 mm where anything touches.

Frames (metres): the telescope's origin is on the table under its mount's axis line, x forward, y along the tube
(the front toward +y), z up. The filter cell and the finder cap have their axis along local y, their closed (film)
end toward +y, their origin on the axis at mid-depth. The eyepiece's origin is on its axis at its shoulder (where it rests on the
holder), z up; the eyepiece filter's origin is on its axis at the face its thread stands from, z along the thread.

Pure Python and NumPy.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

import usdkit as U

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build"

# -- telescope ---------------------------------------------------------------------------------------------------------
AX = 0.100                      # tube axis above the table
TUBE_R = 0.038
TUBE_Y = (-0.110, 0.110)
DEW_R = 0.042                   # dew shield outside radius (8.4 cm across)
DEW_Y = (0.110, 0.200)          # its front rim at y +0.20
FOCUSER_Y = (-0.152, -0.110)
DIAG_Y = -0.175                 # the star diagonal's eyepiece axis
HOLDER_Z = (0.110, 0.145)       # the 1.25-inch holder's bore: floor (the diagonal's box below) .. top
HOLDER_RI = 0.0162              # bore corner radius (24 segments: 32.1 mm across the flats, 32.4 at the corners)
HOLDER_RO = 0.019
EYEPIECE_R = 0.018
EYEPIECE_TOP = 0.185
FZ = 0.175                      # finder axis above the table
FINDER_Y = (-0.030, 0.140)      # eyepiece end .. objective cell's front
FOBJ_R = 0.0195                 # finder objective cell (3.9 cm across)
FOBJ_Y = (0.112, 0.140)

# -- filter cell -------------------------------------------------------------------------------------------------------
CELL_RI, CELL_RO = 0.043, 0.0455        # skirt inside (8.6 cm, felt lined) and outside (9.1 cm)
CELL_FLARE = 0.003                      # the lead-in: the mouth's last 5 mm open 3 mm wider
CELL_Y0, CELL_Y1 = -0.016, 0.014        # skirt from its open mouth to the lip
LIP_RI = 0.038                          # clear aperture 7.6 cm
LIP_Y1 = 0.016
CELL_SEAT_Y = DEW_Y[1] - CELL_Y1        # the cell's origin (world y) when its lip rests on the dew shield's rim
SCREW_ANGLES = (90.0,)                  # one thumbscrew, at the top (local +z), felt opposite
SCREW_Y = -0.001                        # at the skirt's mid-depth
SHANK_R, HEAD_R, HEAD_H = 0.003175, 0.008, 0.008      # 1/4-20 shank
PITCH = 0.00127                         # 1/4-20: 1.27 mm per turn
SCREW_TRAVEL = 0.005                    # the tip can go 5 mm in from flush
HEAD_R0 = CELL_RO + 0.006               # the head's inner face, 6 mm out from the skirt
HEAD_TOP = HEAD_R0 + HEAD_H             # 5.95 cm from the axis
CELL_MASS = 0.090                       # the cell with its thumbscrew
SCREW_MASS = 0.005

# -- finder cap --------------------------------------------------------------------------------------------------------
CAP_RI, CAP_RO = 0.0205, 0.0245         # 1 mm radial clearance over the objective cell; a thick moulded cap
CAP_FLARE = 0.003                       # an inside chamfer at the mouth: 3 mm of lead-in
CAP_Y0, CAP_Y1, CAP_END = -0.009, 0.006, 0.009
CAP_SEAT_Y = FOBJ_Y[1] - CAP_Y1         # the cap's origin (world y) when its end touches the objective cell
CAP_MASS = 0.020

# -- eyepiece filter (the decoy) ---------------------------------------------------------------------------------------
DECOY_R, DECOY_H = 0.01525, 0.004    # the filter's cell: 30.5 mm across, 4 mm tall
SPIGOT_R, SPIGOT_H = 0.0140, 0.0035     # its male thread (sim stand-in for M28.5): 28.0 mm across, 3.5 mm long
DECOY_MASS = 0.020

# -- loose eyepiece (1.25 inch), frame at its shoulder --------------------------------------------------------------
BARREL_R, BARREL_L = 0.015875, 0.026    # 31.75 mm barrel, 2.6 cm long below the shoulder
BORE_R, BORE_D, BORE_CH = 0.0145, 0.004, 0.001   # the filter thread at the barrel's end: 29 mm, 4 mm deep, 1 mm lead-in
EP_BODY_R, EP_BODY_TOP, EP_TOP = 0.018, 0.023, 0.040   # body to 2.3 cm above the shoulder, eyecup to 4.0 cm
EP_MASS = 0.120
SEAT_Z = HOLDER_Z[1]                    # the shoulder rests on the holder's top

# -- case ----------------------------------------------------------------------------------------------------------------
CASE_C = (0.15, 0.0)
CASE_SIZE = (0.16, 0.28, 0.06)          # x, y, foam top height
FOAM_Z0 = 0.030
CELL_SLOT = dict(x=(0.132, 0.168), y=(0.012, 0.128), floor=FOAM_Z0)            # the cell stands on edge here
CAP_SLOT = dict(x=(0.138, 0.162), y=(-0.0565, -0.0035), floor=0.0515)         # a shallow cradle for the cap
DECOY_SLOT = dict(x=(0.1345, 0.1655), y=(-0.1105, -0.0795), floor=0.057)      # a 31 mm square recess, 3 mm deep
CELL_STEP = dict(x=(0.132, 0.1605), h=0.003)   # the slot's floor is cut 3 mm deeper under the flared mouth, so the cell stands level
CELL_START = (0.150, 0.070, CELL_SLOT["floor"] + CELL_STEP["h"] + CELL_RO)
CAP_START = (0.150, -0.030, CAP_SLOT["floor"] + CAP_RO)
DECOY_START = (0.150, -0.095, DECOY_SLOT["floor"] + DECOY_H)    # the filter's origin (its thread's face), thread up

LOOKS = {
    "white": U.Look(color=(0.82, 0.83, 0.82), roughness=0.35),
    "black": U.Look(color=(0.025, 0.025, 0.028), roughness=0.45),
    "black_anod": U.Look(color=(0.03, 0.03, 0.033), metallic=0.5, roughness=0.38),
    "rubber": U.Look(color=(0.02, 0.02, 0.02), roughness=0.85),
    "felt": U.Look(color=(0.012, 0.012, 0.012), roughness=1.0),
    "grey_alu": U.Look(color=(0.55, 0.56, 0.58), metallic=0.8, roughness=0.35),
    "glass": U.Look(color=(0.02, 0.05, 0.05), metallic=0.3, roughness=0.04),
    "decoy_glass": U.Look(color=(0.015, 0.05, 0.025), metallic=0.2, roughness=0.03),
    "film": U.Look(color=(1.0, 1.0, 1.0), metallic=1.0, roughness=0.12, texture="film.jpg", tile=0.05),
    "nylon": U.Look(color=(0.88, 0.87, 0.83), roughness=0.5),
    "nylon_dark": U.Look(color=(0.30, 0.30, 0.29), roughness=0.6),
    "foam": U.Look(color=(1.0, 1.0, 1.0), roughness=0.95, texture="foam.jpg", tile=0.04),
    "case_alu": U.Look(color=(1.0, 1.0, 1.0), metallic=0.85, roughness=0.32, texture="brushed.jpg", tile=0.10),
    "decoy_top": U.Look(color=(1.0, 1.0, 1.0), metallic=0.3, roughness=0.35, texture="decoy_top.png",
                        origin=(-DECOY_R, -DECOY_R, 0.0), tile_uv=(2 * DECOY_R, 2 * DECOY_R)),
}


# -- shapes ------------------------------------------------------------------------------------------------------------
def lathe_y(profile, segments: int = 48) -> U.Mesh:
    """Revolve a (radius, y) profile about the y axis."""
    return U.lathe(profile, segments).turned(-90.0, "x")


def cyl_points(r: float, a0: float, a1: float, axis: str = "y", n: int = 24, center=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Corners of an n-gon prism (vertices on radius r) from a0 to a1 along `axis`."""
    th = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    c, s = r * np.cos(th), r * np.sin(th)
    pts = []
    for a in (a0, a1):
        if axis == "y":
            pts += [(ci, a, si) for ci, si in zip(c, s)]
        elif axis == "x":
            pts += [(a, ci, si) for ci, si in zip(c, s)]
        else:
            pts += [(ci, si, a) for ci, si in zip(c, s)]
    return np.array(pts) + np.asarray(center, float)


def ring_hulls(name: str, root: str, ri0: float, ro0: float, ri1: float, ro1: float, y0: float, y1: float, n: int,
               center=(0.0, 0.0, 0.0), half_turn: bool = False) -> list[str]:
    """A ring about the y axis as n convex segments; radii (ri0, ro0) at y0 and (ri1, ro1) at y1. half_turn turns the
    segments by half a segment, so that a flat (not a seam between two segments) faces each of local +-x and +-z:
    where fingers pinch across the ring."""
    out = []
    c = np.asarray(center, float)
    s = -np.pi / n if half_turn else 0.0
    for k in range(n):
        a0, a1 = s + 2 * np.pi * k / n, s + 2 * np.pi * (k + 1) / n
        pts = []
        for (ri, ro, y) in ((ri0, ro0, y0), (ri1, ro1, y1)):
            for r in (ri, ro):
                for a in (a0, a1):
                    pts.append((r * math.cos(a), y, r * math.sin(a)))
        out.append(U.hull_collider(f"{name}_{k:02d}", np.array(pts) + c, root))
    return out


def ring_hulls_z(name: str, root: str, ri0: float, ro0: float, ri1: float, ro1: float, z0: float, z1: float, n: int,
                 center=(0.0, 0.0, 0.0), material: str = "PhysicsMaterial") -> list[str]:
    """A ring about the z axis as n convex segments; radii (ri0, ro0) at z0 and (ri1, ro1) at z1."""
    out = []
    c = np.asarray(center, float)
    for k in range(n):
        a0, a1 = 2 * np.pi * k / n, 2 * np.pi * (k + 1) / n
        pts = [(r * math.cos(a), r * math.sin(a), z) for (ri, ro, z) in ((ri0, ro0, z0), (ri1, ro1, z1)) for r in (ri, ro)
               for a in (a0, a1)]
        out.append(U.hull_collider(f"{name}_{k:02d}", np.array(pts) + c, root, material))
    return out


def _hull(name: str, points, root: str, material: str = "PhysicsMaterial") -> str:
    """usdkit's convex-hull collider, with its faces from scipy's ConvexHull (usdkit lists every coplanar triangle,
    which bloats a many-sided prism); falls back to usdkit's own if scipy is missing."""
    try:
        from scipy.spatial import ConvexHull
    except ImportError:
        return _usdkit_hull(name, points, root, material)
    p = np.asarray(points, float)
    c = p.mean(axis=0)
    faces = []
    for tri in ConvexHull(p).simplices:
        i, j, k = (int(v) for v in tri)
        nrm = np.cross(p[j] - p[i], p[k] - p[i])
        faces.append((i, j, k) if (c - p[i]) @ nrm < 0 else (i, k, j))
    return f"""
    def Mesh "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"]
    )
    {{
        rel material:binding:physics = </{root}/{material}>
        uniform token purpose = "guide"
        uniform token physics:approximation = "convexHull"
        float physxCollision:contactOffset = 0.004
        float physxCollision:restOffset = 0
        int[] faceVertexCounts = [{", ".join("3" for _ in faces)}]
        int[] faceVertexIndices = [{", ".join(str(i) for t in faces for i in t)}]
        point3f[] points = [{U._fmt(p, 6)}]
    }}"""


_usdkit_hull = U.hull_collider
U.hull_collider = _hull


def turn_pts(p: np.ndarray, deg: float, axis: str) -> np.ndarray:
    """Points turned as Mesh.turned turns a mesh's vertices."""
    return U.Mesh(np.asarray(p, float), np.zeros((0, 3), int)).turned(deg, axis).v


def box_collider(name: str, root: str, x, y, z) -> str:
    return U.collider(name, (x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2), root)


def box_mesh(x, y, z, bevel: float = 0.0, round_xy: float = 0.0) -> U.Mesh:
    return U.box((x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2), bevel, round_xy)


# -- telescope -----------------------------------------------------------------------------------------------------------
def _telescope() -> str:
    root = "telescope"
    parts, cols = [], []
    up = (0.0, 0.0, AX)
    # Dew shield: a thin white tube with a rolled front rim; the objective lens sits 8 cm inside it.
    parts.append(("dew_shield", lathe_y([(DEW_R - 0.002, DEW_Y[0] + 0.004), (DEW_R, DEW_Y[0]), (DEW_R, DEW_Y[1] - 0.0015),
                                         (DEW_R - 0.0008, DEW_Y[1]), (DEW_R - 0.0022, DEW_Y[1]), (DEW_R - 0.0025, DEW_Y[1] - 0.002),
                                         (DEW_R - 0.0025, DEW_Y[0] + 0.004)]).moved(up), "white"))
    parts.append(("dew_inside", lathe_y([(DEW_R - 0.0026, DEW_Y[1] - 0.002), (DEW_R - 0.0026, DEW_Y[0] + 0.006),
                                         (0.0, DEW_Y[0] + 0.006)]).moved(up), "felt"))
    parts.append(("objective_cell", lathe_y([(0.0395, 0.116), (0.0395, 0.124), (0.0345, 0.124), (0.0345, 0.121)]).moved(up), "black"))
    parts.append(("objective", U.cylinder(0.0346, 0.002, (0.0, 0.121, AX), axis="y", segments=48), "glass"))
    cols.append(U.hull_collider("dew_shield_col", cyl_points(DEW_R, DEW_Y[0], DEW_Y[1], n=24, center=up), root))
    # Tube: white, closed at the back by a black cell.
    parts.append(("tube", lathe_y([(TUBE_R, TUBE_Y[0] + 0.004), (TUBE_R, TUBE_Y[1])]).moved(up), "white"))
    parts.append(("tube_rear", lathe_y([(0.0, TUBE_Y[0]), (TUBE_R + 0.001, TUBE_Y[0]), (TUBE_R + 0.001, TUBE_Y[0] + 0.006)]).moved(up), "black"))
    cols.append(U.hull_collider("tube_col", cyl_points(TUBE_R, TUBE_Y[0], TUBE_Y[1], n=20, center=up), root))
    # Focuser: a rack-and-pinion body, its drawtube and two focus knobs.
    parts.append(("focuser", lathe_y([(0.0, -0.155), (0.0185, -0.155), (0.0185, -0.150), (0.017, -0.150), (0.017, -0.126),
                                      (0.024, -0.126), (0.024, -0.110), (0.0, -0.110)]).moved(up), "black"))
    parts.append(("rack_box", box_mesh((-0.012, 0.012), (-0.126, -0.112), (AX - 0.034, AX - 0.018), bevel=0.001), "black"))
    for sx in (-1, 1):
        parts.append((f"knob_{'l' if sx > 0 else 'r'}", U.cylinder(0.008, 0.012, (sx * 0.020, -0.119, AX - 0.026), axis="x", segments=16, bevel=0.001), "rubber"))
    cols.append(U.hull_collider("focuser_col", cyl_points(0.024, FOCUSER_Y[0], FOCUSER_Y[1], n=16, center=up), root))
    # Star diagonal, its eyepiece holder facing up, and a 20 mm eyepiece with a rubber eyecup.
    dz = (AX - 0.020, AX + 0.020)
    parts.append(("diagonal", box_mesh((-0.018, 0.018), (DIAG_Y - 0.020, DIAG_Y + 0.020), dz, bevel=0.004, round_xy=0.004), "black"))
    parts.append(("diag_nose", U.cylinder(0.0165, 0.006, (0.0, DIAG_Y + 0.023, AX), axis="y", segments=24), "black"))
    # The 1.25-inch eyepiece holder: a bore down into the diagonal's body; its side thumbscrew backed off (no mechanism).
    parts.append(("holder", U.lathe([(HOLDER_RO, dz[1]), (HOLDER_RO, HOLDER_Z[1] - 0.0008), (HOLDER_RO - 0.0008, HOLDER_Z[1]),
                                     (HOLDER_RI + 0.001, HOLDER_Z[1]), (HOLDER_RI, HOLDER_Z[1] - 0.001),
                                     (HOLDER_RI, HOLDER_Z[0] + 0.0005), (0.0, HOLDER_Z[0] + 0.0005)], 48).moved((0.0, DIAG_Y, 0.0)), "black"))
    parts.append(("holder_screw", U.cylinder(0.004, 0.008, (HOLDER_RO + 0.0045, DIAG_Y, 0.136), axis="x", segments=12), "black"))
    cols.append(box_collider("diagonal_col", root, (-0.018, 0.018), (DIAG_Y - 0.020, DIAG_Y + 0.020), (dz[0], HOLDER_Z[0])))
    cols += ring_hulls_z("holder_col", root, HOLDER_RI, HOLDER_RO, HOLDER_RI, HOLDER_RO, HOLDER_Z[0], HOLDER_Z[1] - 0.001, 24,
                         center=(0.0, DIAG_Y, 0.0))
    cols += ring_hulls_z("holder_mouth", root, HOLDER_RI, HOLDER_RO, HOLDER_RI + 0.001, HOLDER_RO, HOLDER_Z[1] - 0.001,
                         HOLDER_Z[1], 24, center=(0.0, DIAG_Y, 0.0))
    # Finder: a white 6x30 finder in a black two-ring bracket on a shoe on the tube's top.
    fc = (0.0, 0.0, FZ)
    parts.append(("finder_tube", lathe_y([(0.0115, -0.002), (0.016, 0.0), (0.016, FOBJ_Y[0])]).moved(fc), "white"))
    parts.append(("finder_cell", lathe_y([(0.016, FOBJ_Y[0]), (FOBJ_R, FOBJ_Y[0] + 0.004), (FOBJ_R, FOBJ_Y[1] - 0.001),
                                          (FOBJ_R - 0.001, FOBJ_Y[1]), (0.0172, FOBJ_Y[1]), (0.0170, FOBJ_Y[1] - 0.002),
                                          (0.0170, 0.128)]).moved(fc), "white"))
    parts.append(("finder_lens", U.cylinder(0.0171, 0.0005, (0.0, 0.128, FZ), axis="y", segments=32), "glass"))
    parts.append(("finder_ep", lathe_y([(0.0, FINDER_Y[0]), (0.012, FINDER_Y[0]), (0.012, -0.012), (0.0115, -0.002)]).moved(fc), "black"))
    parts.append(("finder_ep_ring", lathe_y([(0.0125, -0.022), (0.0125, -0.014)]).moved(fc), "rubber"))
    for i, ry in enumerate((0.020, 0.085)):
        parts.append((f"finder_ring_{i}", lathe_y([(0.016, ry - 0.004), (0.0225, ry - 0.004), (0.0225, ry + 0.004), (0.016, ry + 0.004)]).moved(fc), "black"))
        for a in (90.0, 210.0, 330.0):
            d = np.array([math.cos(math.radians(a)), 0.0, math.sin(math.radians(a))])
            p = np.array(fc) + np.array([0.0, ry, 0.0]) + d * 0.0255
            parts.append((f"finder_screw_{i}_{int(a)}", U.cylinder(0.0022, 0.008, (0, 0, 0), axis="z", segments=10)
                          .turned(math.degrees(math.atan2(d[0], d[2])), "y").moved(p), "nylon"))
        cols.append(U.hull_collider(f"finder_ring_col_{i}", cyl_points(0.0225, ry - 0.004, ry + 0.004, n=16, center=fc), root))
    parts.append(("finder_post", box_mesh((-0.006, 0.006), (0.010, 0.095), (0.140, FZ - 0.020)), "black"))
    parts.append(("finder_shoe", box_mesh((-0.013, 0.013), (0.000, 0.105), (0.134, 0.141), bevel=0.0015), "black"))
    cols.append(U.hull_collider("finder_col", cyl_points(0.016, FINDER_Y[0], FOBJ_Y[0], n=16, center=fc), root))
    cols.append(U.hull_collider("finder_cell_col", cyl_points(FOBJ_R, FOBJ_Y[0], FOBJ_Y[1], n=24, center=fc), root))
    cols.append(box_collider("finder_post_col", root, (-0.013, 0.013), (0.000, 0.105), (0.134, FZ - 0.018)))
    # Tabletop alt-az mount: a round base, a column, the head and a dovetail clamp under the tube.
    mb = (0.0, 0.020)
    parts.append(("mount_base", U.cylinder(0.070, 0.020, (mb[0], mb[1], 0.010), axis="z", segments=48, bevel=0.003), "black"))
    parts.append(("mount_column", U.cylinder(0.018, 0.022, (mb[0], mb[1], 0.031), axis="z", segments=32), "grey_alu"))
    parts.append(("mount_head", box_mesh((-0.026, 0.026), (mb[1] - 0.030, mb[1] + 0.030), (0.040, 0.056), bevel=0.002, round_xy=0.004), "black"))
    parts.append(("mount_knob", U.cylinder(0.009, 0.014, (0.033, mb[1], 0.048), axis="x", segments=16, bevel=0.001), "rubber"))
    parts.append(("dovetail", box_mesh((-0.016, 0.016), (-0.050, 0.070), (0.056, AX - TUBE_R + 0.002), bevel=0.001), "grey_alu"))
    cols.append(U.hull_collider("mount_base_col", cyl_points(0.070, 0.0, 0.020, axis="z", n=24, center=(mb[0], mb[1], 0.0)), root))
    cols.append(box_collider("mount_head_col", root, (-0.026, 0.026), (mb[1] - 0.030, mb[1] + 0.030), (0.020, AX - TUBE_R)))
    path = BUILD / "telescope.usda"
    U.write_usda(str(path), root, parts, LOOKS, cols, mass=1.0, friction=(0.5, 0.45))  # loaded kinematic (fixed)
    return path.relative_to(HERE).as_posix()


# -- articulations ------------------------------------------------------------------------------------------------------
DUMMY = dict(mass=0.0003, com=(0.0, 0.0, 0.0), inertia=(1e-8, 1e-8, 1e-8), parts=[], cols=[])


def _write_artic(name: str, root: str, links: list[dict], joints: list[dict], friction, materials: str = "",
                 self_collide: bool = False, solver_iters: int = 32) -> str:
    """One articulation. links[0] is its root, a free body. Each link: name, pos (where it is with every joint at 0),
    mass, com, inertia, and its meshes and colliders in its own frame. Each joint: name, kind ("Prismatic" or
    "Revolute"), axis, body0, body1, pos0 (the joint's place on body0; on body1 it is at the origin), limits (m or
    degrees) or None. The joints carry drive schemas; their gains come from task.py."""
    used = sorted({p[2] for lk in links for p in lk["parts"]})
    looks = "".join(U._material_usda(k, root, LOOKS[k]) for k in used)
    body = ""
    for i, lk in enumerate(links):
        api = '"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"'
        art = ""
        if i == 0:
            api += ', "PhysicsArticulationRootAPI", "PhysxArticulationAPI"'
            art = f"""
        bool physxArticulation:enabledSelfCollisions = {int(self_collide)}
        int physxArticulation:solverPositionIterationCount = {solver_iters}
        int physxArticulation:solverVelocityIterationCount = 4
        float physxArticulation:sleepThreshold = 0
        float physxArticulation:stabilizationThreshold = 0"""
        meshes = "".join(U._mesh_usda(p[0], p[1], p[2], root, LOOKS[p[2]], 35.0, indent="            ") for p in lk["parts"])
        vis = f"""

        def Xform "visual"
        {{{meshes}
        }}""" if lk["parts"] else ""
        cols = "".join(c.replace("\n    ", "\n        ") for c in lk["cols"])
        px, py, pz = lk.get("pos", (0.0, 0.0, 0.0))
        cx, cy, cz = lk["com"]
        ix, iy, iz = lk["inertia"]
        body += f"""
    def Xform "{lk['name']}" (
        prepend apiSchemas = [{api}]
    )
    {{
        double3 xformOp:translate = ({px:.6f}, {py:.6f}, {pz:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]
        float physics:mass = {lk['mass']:.6f}
        point3f physics:centerOfMass = ({cx:.6f}, {cy:.6f}, {cz:.6f})
        float3 physics:diagonalInertia = ({ix:.4e}, {iy:.4e}, {iz:.4e})
        float physxRigidBody:maxDepenetrationVelocity = 1
        float physxRigidBody:sleepThreshold = 0
        float physxRigidBody:stabilizationThreshold = 0{art}
{cols}{vis}
    }}
"""
    jt = ""
    for j in joints:
        drive = "linear" if j["kind"] == "Prismatic" else "angular"
        lim = j.get("limits")
        lim_txt = "" if lim is None else f"""
            float physics:lowerLimit = {lim[0]}
            float physics:upperLimit = {lim[1]}"""
        x, y, z = j.get("pos0", (0.0, 0.0, 0.0))
        jt += f"""
        def Physics{j['kind']}Joint "{j['name']}" (
            prepend apiSchemas = ["PhysicsDriveAPI:{drive}", "PhysxJointAPI"]
        )
        {{
            uniform token physics:axis = "{j['axis']}"
            rel physics:body0 = </{root}/{j['body0']}>
            rel physics:body1 = </{root}/{j['body1']}>
            point3f physics:localPos0 = ({x:.6f}, {y:.6f}, {z:.6f})
            quatf physics:localRot0 = (1, 0, 0, 0)
            point3f physics:localPos1 = (0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0){lim_txt}
            float physxJoint:armature = 0.00001
            float physxJoint:maxJointVelocity = 1000000
            uniform token drive:{drive}:physics:type = "force"
            float drive:{drive}:physics:targetPosition = 0
            float drive:{drive}:physics:stiffness = 0
            float drive:{drive}:physics:damping = 0
        }}
"""
    text = f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}"
{{{U.physics_material("PhysicsMaterial", friction[0], friction[1])}{materials}

    def Scope "Looks"
    {{{looks}
    }}
{body}
    def Scope "joints"
    {{{jt}
    }}
}}
"""
    path = BUILD / f"{name}.usda"
    rel = os.path.relpath(HERE / "textures", path.parent).replace(os.sep, "/")
    text = text.replace(f"@{(HERE / 'textures').as_posix()}/", f"@{rel}/")
    path.write_text(text)
    return path.relative_to(HERE).as_posix()


def _dummy(name: str, pos) -> dict:
    return dict(DUMMY, name=name, pos=pos)


# -- filter cell and its thumbscrew -------------------------------------------------------------------------------------
def _cell() -> str:
    """The cell is the root; the thumbscrew hangs from it on a slide (`j_adv`, + outward along the screw) and a hinge
    (`j_spin`, + anticlockwise seen from the head). Its tip is flush with the felt at j_adv = 0."""
    root = "solar_filter"
    parts, cols = [], []
    y0, y1, f = CELL_Y0, CELL_Y1, CELL_FLARE
    # Skirt: outside black anodised, with the flared mouth; inside lined with black felt.
    parts.append(("skirt", lathe_y([(CELL_RI + f, y0), (CELL_RO + f, y0), (CELL_RO, y0 + 0.005), (CELL_RO, y1),
                                    (CELL_RO, LIP_Y1 - 0.0005), (CELL_RO - 0.0005, LIP_Y1), (LIP_RI + 0.001, LIP_Y1),
                                    (LIP_RI, LIP_Y1 - 0.001), (LIP_RI, y1)], 64), "black_anod"))
    parts.append(("felt", lathe_y([(LIP_RI, y1), (CELL_RI, y1), (CELL_RI, y0 + 0.005), (CELL_RI + f, y0)], 64), "felt"))
    parts.append(("film", U.cylinder(LIP_RI + 0.0005, 0.0006, (0.0, LIP_Y1 - 0.0012, 0.0), axis="y", segments=64), "film"))
    parts.append(("boss", U.cylinder(0.0055, 0.0025, (0.0, SCREW_Y, CELL_RO + 0.001), axis="z", segments=24), "black_anod"))
    cols += ring_hulls("skirt", root, CELL_RI, CELL_RO, CELL_RI, CELL_RO, y0 + 0.005, y1, 24)
    cols += ring_hulls("lead_in", root, CELL_RI + f, CELL_RO + f, CELL_RI, CELL_RO, y0, y0 + 0.005, 24)
    cols += ring_hulls("lip", root, LIP_RI, CELL_RO, LIP_RI, CELL_RO, y1, LIP_Y1, 24)
    m_cell = CELL_MASS - SCREW_MASS
    cell = dict(name="cell", pos=(0.0, 0.0, 0.0), mass=m_cell, com=(0.0, 0.002, 0.0),
                inertia=(m_cell * CELL_RO ** 2 / 2 + m_cell * 0.03 ** 2 / 12, m_cell * CELL_RO ** 2,
                         m_cell * CELL_RO ** 2 / 2 + m_cell * 0.03 ** 2 / 12), parts=parts, cols=cols)
    shank = U.cylinder(SHANK_R, HEAD_R0 - CELL_RI, (0, 0, (CELL_RI + HEAD_R0) / 2), axis="z", segments=12)
    head = U.cylinder(HEAD_R, HEAD_H, (0, 0, HEAD_R0 + HEAD_H / 2), axis="z", segments=18, bevel=0.0012)
    knurl = [U.box((0.0012, 0.0012, HEAD_H - 0.003), (HEAD_R * math.cos(a), HEAD_R * math.sin(a), HEAD_R0 + HEAD_H / 2))
             for a in np.linspace(0, 2 * np.pi, 9, endpoint=False)]
    sp = [("shank", shank, "nylon"), ("head", head, "nylon"), ("knurl", U.join(*knurl), "nylon_dark")]
    sc = [U.hull_collider("screw_head", cyl_points(HEAD_R, HEAD_R0, HEAD_TOP, axis="z", n=16), root),
          U.hull_collider("screw_shank", cyl_points(SHANK_R, CELL_RI, HEAD_R0, axis="z", n=8), root)]
    zc = (HEAD_R0 + HEAD_TOP) / 2
    screw = dict(name="screw", pos=(0.0, SCREW_Y, 0.0), mass=SCREW_MASS, com=(0.0, 0.0, zc),
                 inertia=(2.5e-7, 2.5e-7, 1.6e-7), parts=sp, cols=sc)
    joints = [dict(name="j_adv", kind="Prismatic", axis="Z", body0="cell", body1="l_adv", pos0=(0.0, SCREW_Y, 0.0),
                   limits=(-SCREW_TRAVEL, 0.0005)),
              dict(name="j_spin", kind="Revolute", axis="Z", body0="l_adv", body1="screw", limits=None)]
    return _write_artic("solar_filter", root, [cell, _dummy("l_adv", (0.0, SCREW_Y, 0.0)), screw], joints,
                        friction=(0.5, 0.45), solver_iters=48)


# -- finder cap ----------------------------------------------------------------------------------------------------------
def _cap() -> str:
    root = "finder_cap"
    f = CAP_FLARE
    parts = [("cap", lathe_y([(CAP_RI + f, CAP_Y0), (CAP_RO, CAP_Y0), (CAP_RO, CAP_END - 0.001),
                              (CAP_RO - 0.001, CAP_END), (0.0, CAP_END)], 48), "black"),
             ("cap_inside", lathe_y([(0.0, CAP_Y1), (CAP_RI, CAP_Y1), (CAP_RI, CAP_Y0 + 0.004), (CAP_RI + f, CAP_Y0)], 48), "black")]
    cols = ring_hulls("skirt", root, CAP_RI, CAP_RO, CAP_RI, CAP_RO, CAP_Y0 + 0.004, CAP_Y1, 24, half_turn=True)
    cols += ring_hulls("mouth", root, CAP_RI + f, CAP_RO, CAP_RI, CAP_RO, CAP_Y0, CAP_Y0 + 0.004, 24, half_turn=True)
    cols.append(U.hull_collider("end", cyl_points(CAP_RO, CAP_Y1, CAP_END, n=24), root))
    path = BUILD / "finder_cap.usda"
    U.write_usda(str(path), root, parts, LOOKS, cols, mass=CAP_MASS, friction=(0.6, 0.5), solver_iters=48)
    return path.relative_to(HERE).as_posix()


# -- eyepiece and the SUN filter -----------------------------------------------------------------------------------------
CHAIN = [("j_x", "Prismatic", "X", "eyepiece", "l_x", (-0.6, 0.6)), ("j_y", "Prismatic", "Y", "l_x", "l_y", (-0.6, 0.6)),
         ("j_z", "Prismatic", "Z", "l_y", "l_z", (-0.4, 0.4)), ("j_a", "Revolute", "X", "l_z", "l_a", (-170.0, 170.0)),
         ("j_b", "Revolute", "Y", "l_a", "l_b", (-85.0, 85.0)), ("j_s", "Revolute", "Z", "l_b", "filter", None)]
CHAIN_JOINTS = [c[0] for c in CHAIN]


def _eyepiece(filter_offset) -> str:
    """The eyepiece is the root (origin at its shoulder, on its axis). The SUN filter hangs from it on three slides and
    three hinges, with no drive while free; at every joint at 0 the filter is `filter_offset` from the eyepiece,
    upright. Self-collision is on, so the filter and eyepiece meet by contact."""
    root = "eyepiece"
    L = BARREL_L
    ep_parts = [("barrel", U.cylinder(BARREL_R, L, (0.0, 0.0, -L / 2), axis="z", segments=40), "grey_alu"),
                ("bore", U.cylinder(BORE_R, 0.0003, (0.0, 0.0, -L - 0.0002), axis="z", segments=40), "black"),
                ("body", U.lathe([(0.0, 0.0), (EP_BODY_R, 0.0), (EP_BODY_R, EP_BODY_TOP), (0.0, EP_BODY_TOP)], 40), "black_anod"),
                ("eyecup", U.lathe([(0.0, EP_BODY_TOP), (EP_BODY_R + 0.0005, EP_BODY_TOP), (EP_BODY_R + 0.0005, EP_TOP - 0.001),
                                    (EP_BODY_R - 0.001, EP_TOP), (0.010, EP_TOP), (0.009, EP_TOP - 0.003),
                                    (0.0, EP_TOP - 0.003)], 40), "rubber"),
                ("eye_lens", U.cylinder(0.0089, 0.0005, (0.0, 0.0, EP_TOP - 0.0028), axis="z", segments=24), "glass")]
    ep_cols = ring_hulls_z("bore_mouth", root, BORE_R + BORE_CH, BARREL_R, BORE_R, BARREL_R, -L, -L + BORE_CH, 16)
    ep_cols += ring_hulls_z("bore_wall", root, BORE_R, BARREL_R, BORE_R, BARREL_R, -L + BORE_CH, -L + BORE_D, 16)
    ep_cols += [U.hull_collider("barrel", cyl_points(BARREL_R, -L + BORE_D, 0.0, axis="z", n=16), root),
                U.hull_collider("body", cyl_points(EP_BODY_R, 0.0, EP_TOP, axis="z", n=16), root)]
    eyepiece = dict(name="eyepiece", pos=(0.0, 0.0, 0.0), mass=EP_MASS, com=(0.0, 0.0, 0.006),
                    inertia=(5.2e-5, 5.2e-5, 1.8e-5), parts=ep_parts, cols=ep_cols)
    h = DECOY_H
    f_parts = [("cell", U.lathe([(0.0, -h), (DECOY_R - 0.0004, -h), (DECOY_R, -h + 0.0004), (DECOY_R, -0.0004)], 48), "black_anod"),
               ("top", U.lathe([(DECOY_R, -0.0004), (DECOY_R - 0.0004, 0.0), (SPIGOT_R, 0.0)], 48), "decoy_top"),
               ("thread", U.lathe([(SPIGOT_R, 0.0), (SPIGOT_R, SPIGOT_H - 0.0005), (SPIGOT_R - 0.0005, SPIGOT_H),
                                   (0.0125, SPIGOT_H), (0.0125, 0.0)], 48), "black_anod"),
               ("thread_lines", U.join(*[U.lathe([(SPIGOT_R + 0.00005, z), (SPIGOT_R + 0.00005, z + 0.0002)], 48)
                                         for z in np.arange(0.0004, SPIGOT_H - 0.0006, 0.0006)]), "grey_alu"),
               ("glass", U.cylinder(0.0126, 0.0016, (0.0, 0.0, -0.0012), axis="z", segments=48), "decoy_glass")]
    # The cell bears on its rim: the glass sits 0.3 mm up inside it, as in a real filter cell.
    f_cols = ring_hulls_z("filter_rim", root, 0.0125, DECOY_R, 0.0125, DECOY_R, -h, 0.0, 20, material="FilterMaterial")
    f_cols += [U.hull_collider("filter_cell", cyl_points(0.0126, -h + 0.0003, 0.0, axis="z", n=20), root, "FilterMaterial"),
              U.hull_collider("filter_thread", np.vstack([cyl_points(SPIGOT_R, 0.0, SPIGOT_H - 0.0005, axis="z", n=20),
                                                          cyl_points(SPIGOT_R - 0.0005, SPIGOT_H, SPIGOT_H, axis="z", n=20)[:20]]),
                                root, "FilterMaterial")]
    filt = dict(name="filter", pos=tuple(filter_offset), mass=DECOY_MASS, com=(0.0, 0.0, -0.001),
                inertia=(1.3e-6, 1.3e-6, 2.3e-6), parts=f_parts, cols=f_cols)
    links = [eyepiece] + [_dummy(c[4], tuple(filter_offset)) for c in CHAIN[:-1]] + [filt]
    joints = [dict(name=n, kind=k, axis=a, body0=b0, body1=b1, limits=lim,
                   pos0=tuple(filter_offset) if b0 == "eyepiece" else (0.0, 0.0, 0.0)) for n, k, a, b0, b1, lim in CHAIN]
    return _write_artic("eyepiece", root, links, joints, friction=(0.3, 0.25),
                        materials=U.physics_material("FilterMaterial", 0.6, 0.5), self_collide=True, solver_iters=32)


# -- case ----------------------------------------------------------------------------------------------------------------
def _foam_boxes() -> list[tuple]:
    """The foam's top layer (FOAM_Z0 to the top) with three cut-outs, as (x, y, z) box ranges; the floors under the
    shallow cut-outs are boxes too."""
    cx, cy = CASE_C
    sx, sy, top = CASE_SIZE
    rim = 0.006
    X = (cx - sx / 2 + rim, cx + sx / 2 - rim)
    Y = (cy - sy / 2 + rim, cy + sy / 2 - rim)
    holes = [CELL_SLOT, CAP_SLOT, DECOY_SLOT]
    cuts = sorted({Y[0], Y[1], *[v for h in holes for v in h["y"]]})
    boxes = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        hole = next((h for h in holes if h["y"][0] <= a and b <= h["y"][1]), None)
        if hole is None:
            boxes.append((X, (a, b), (FOAM_Z0, top)))
            continue
        boxes.append(((X[0], hole["x"][0]), (a, b), (FOAM_Z0, top)))
        boxes.append(((hole["x"][1], X[1]), (a, b), (FOAM_Z0, top)))
        if hole["floor"] > FOAM_Z0 + 1e-6:
            boxes.append((hole["x"], (a, b), (FOAM_Z0, hole["floor"])))
    boxes.append((CELL_STEP["x"], CELL_SLOT["y"], (CELL_SLOT["floor"], CELL_SLOT["floor"] + CELL_STEP["h"])))
    return boxes


def _case() -> str:
    root = "case"
    cx, cy = CASE_C
    sx, sy, top = CASE_SIZE
    parts, cols = [], []
    X = (cx - sx / 2, cx + sx / 2)
    Y = (cy - sy / 2, cy + sy / 2)
    w = 0.006
    for i, (xr, yr) in enumerate((((X[0], X[0] + w), Y), ((X[1] - w, X[1]), Y), (X, (Y[0], Y[0] + w)), (X, (Y[1] - w, Y[1])))):
        parts.append((f"wall_{i}", box_mesh(xr, yr, (0.0, top + 0.003), bevel=0.0015), "case_alu"))
        cols.append(box_collider(f"wall_col_{i}", root, xr, yr, (0.0, top + 0.003)))
    for i, (px, py) in enumerate(((X[0], Y[0]), (X[0], Y[1]), (X[1], Y[0]), (X[1], Y[1]))):
        parts.append((f"corner_{i}", box_mesh((px - 0.010, px + 0.010), (py - 0.010, py + 0.010), (-0.0, top + 0.005), bevel=0.003, round_xy=0.004), "black"))
    parts.append(("foam_base", box_mesh((X[0] + w, X[1] - w), (Y[0] + w, Y[1] - w), (0.0, FOAM_Z0)), "foam"))
    cols.append(box_collider("foam_base_col", root, (X[0] + w, X[1] - w), (Y[0] + w, Y[1] - w), (0.0, FOAM_Z0)))
    for i, (xr, yr, zr) in enumerate(_foam_boxes()):
        parts.append((f"foam_{i}", box_mesh(xr, yr, zr), "foam"))
        cols.append(box_collider(f"foam_col_{i}", root, xr, yr, zr))
    path = BUILD / "case.usda"
    U.write_usda(str(path), root, parts, LOOKS, cols, mass=1.0, friction=(0.7, 0.6))  # loaded kinematic (fixed)
    return path.relative_to(HERE).as_posix()


def build(eyepiece_xyz, filter_xyz) -> dict[str, str]:
    """eyepiece_xyz, filter_xyz: where the eyepiece's and the SUN filter's origins start (station frame); the filter
    hangs from the eyepiece at their difference."""
    BUILD.mkdir(exist_ok=True)
    off = np.asarray(filter_xyz, float) - np.asarray(eyepiece_xyz, float)
    return {"telescope": _telescope(), "solar_filter": _cell(), "finder_cap": _cap(), "eyepiece": _eyepiece(off),
            "case": _case()}


if __name__ == "__main__":
    print(build((0.32, DIAG_Y, 0.7475 + SEAT_Z), (0.15, -0.095, 0.7475 + DECOY_START[2])))
