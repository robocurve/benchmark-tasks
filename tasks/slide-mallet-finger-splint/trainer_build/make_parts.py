"""Generates the printable parts of the mallet-finger trainer (hinged index fingertip) and the splint.

Run with:  pip install trimesh numpy manifold3d && python make_parts.py
Writes fingertip.stl, finger_stub.stl, splint.stl and prints a collision sweep of the hinge.
Frame (mm): x across the finger, y along it toward the fingertip, z up (nail side); the DIP hinge axis is the x axis
through the origin. Dimensions are the ones used in the simulation (task.json, setup).
"""
import math
import numpy as np
import trimesh
from trimesh.creation import box, cylinder

def bx(x0, x1, y0, y1, z0, z1):
    m = box(extents=(x1 - x0, y1 - y0, z1 - z0)); m.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)); return m

def cyl_x(r, x0, x1, y=0.0, z=0.0, sections=64):
    m = cylinder(radius=r, height=x1 - x0, sections=sections)
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0]))
    m.apply_translation(((x0 + x1) / 2, y, z)); return m

def cyl_z(r, z0, z1, x=0.0, y=0.0, sections=32):
    m = cylinder(radius=r, height=z1 - z0, sections=sections); m.apply_translation((x, y, (z0 + z1) / 2)); return m

def union(*ms): return trimesh.boolean.union(list(ms), engine="manifold")
def diff(a, *bs): return trimesh.boolean.difference([a] + list(bs), engine="manifold")

PIN_D = 3.2          # bore for an M3 screw
W_TIP, T_TIP = 18.0, 15.0      # distal segment width x thickness (sim: 18 x 15)
W_MID, T_MID = 19.0, 16.0      # middle segment (sim: 19 x 16)
APEX = 26.6          # DIP axis to the fingertip apex (sim: 26.6 mm)
ARM_W = 4.0          # clevis arms
SLOT = W_MID - 2 * ARM_W       # 11 mm slot: 6 mm tongue + 3 mm spring-leg gap + clearances
TONGUE_X = (-5.3, 0.7)         # tongue sits against the left arm (x), 6 mm wide
R_ARM = T_MID / 2

# ---------- fingertip (distal segment) ----------
body = union(bx(-W_TIP / 2, W_TIP / 2, 9.0, APEX - T_TIP / 2, -T_TIP / 2, T_TIP / 2),
             cyl_x(T_TIP / 2, -W_TIP / 2, W_TIP / 2, y=APEX - T_TIP / 2))           # rounded apex, nail side up
tongue = union(bx(TONGUE_X[0], TONGUE_X[1], 0, 9.5, -T_TIP / 2, T_TIP / 2),
               cyl_x(T_TIP / 2, TONGUE_X[0], TONGUE_X[1]))
tip = union(body, tongue)
tip = diff(tip, cyl_x(PIN_D / 2, -12, 12))
# blind hole in the tongue's right face for the spring's moving leg (radius 5.5 mm, dorsal)
tip = diff(tip, cyl_x(0.5, TONGUE_X[1] - 2.0, TONGUE_X[1] + 0.1, y=0.0, z=5.5, sections=16))

# ---------- finger stub (proximal part carrying the clevis) ----------
stub = union(bx(-W_MID / 2, W_MID / 2, -60, 0, -R_ARM, R_ARM), cyl_x(R_ARM, -W_MID / 2, W_MID / 2))
stub = diff(stub, bx(-W_MID / 2 + ARM_W, W_MID / 2 - ARM_W, -9.0, 12, -12, 12))      # slot between the arms
stub = diff(stub, cyl_x(PIN_D / 2, -12, 12))
for yy in (-30.0, -48.0):                                                           # two M3 clearance holes to bolt to the hand
    stub = diff(stub, cyl_z(1.7, -12, 12, x=0.0, y=yy))
# arc slot (1.6 mm wide, radius 5.5) through the right arm for the spring's moving leg: world angles 17..98 deg
cutters = []
for ang_deg in np.linspace(17, 98, 41):
    ang = math.radians(ang_deg)
    c = box(extents=(ARM_W + 0.4, 1.6, 1.6))
    c.apply_translation((W_MID / 2 - ARM_W / 2, 5.5 * math.cos(ang), 5.5 * math.sin(ang)))
    cutters.append(c)
stub = diff(stub, union(*cutters))
# anchor hole for the spring's fixed leg: blind 2 mm, right arm outer face, world angle 143 deg, radius 5.5
ang = math.radians(143)
stub = diff(stub, cyl_x(0.5, W_MID / 2 - 2.0, W_MID / 2 + 0.1, y=5.5 * math.cos(ang), z=5.5 * math.sin(ang), sections=16))

# ---------- splint (stack-type, 1.5 mm wall; dimensions from the simulation setup) ----------
W_O, H_O, L_O, W_I, H_I, WALL = 23.5, 20.0, 47.5, 20.5, 17.0, 1.5
HOOD = 31.0                      # closed hood over the fingertip and the DIP joint (y = 0 is the closed fingertip end)
SIDE = 8.5                       # side walls beyond the hood
outer = bx(-W_O / 2, W_O / 2, 0.0, L_O, 0.0, H_O)
cavity = bx(-W_I / 2, W_I / 2, WALL, L_O + 1.0, WALL, WALL + H_I)               # open at the rear (y = L_O), closed at y = 0
splint = diff(outer, cavity)
splint = diff(splint, bx(-W_O, W_O, HOOD, L_O + 1.0, WALL + SIDE, H_O + 1.0))   # beyond the hood: floor + 8.5 mm side walls only

# ---------- checks ----------
parts = {"fingertip": tip, "finger_stub": stub, "splint": splint}
for n, m in parts.items():
    m.export(f"{n}.stl")
    print(f"{n:12s} watertight={m.is_watertight} volume={m.volume:8.1f} mm3 bounds_mm={np.round(m.bounds[1]-m.bounds[0],1).tolist()}")
print("hinge collision sweep (tip rotated about the DIP axis; 0 = straight, positive = bent toward the palm):")
worst = 0.0
for flex in (-5, 0, 20, 37, 40, 55, 70):
    R = trimesh.transformations.rotation_matrix(-math.radians(flex), [1, 0, 0])
    t = tip.copy(); t.apply_transform(R)
    inter = trimesh.boolean.intersection([t, stub], engine="manifold")
    v = float(inter.volume) if len(inter.vertices) else 0.0
    worst = max(worst, v); print(f"  flex {flex:4d} deg: tip/stub overlap {v:6.3f} mm3")
print("max overlap mm3:", round(worst, 3), "(0 means the joint turns freely through the full range)")
