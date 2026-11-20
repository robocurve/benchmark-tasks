"""Printable parts for the repeatable X-ray sensor holder: bite block, aiming ring and grip hub (millimetres).

Run with:  pip install trimesh numpy manifold3d && python make_parts.py [prong_hole_mm]
Writes bite_block.stl, aiming_ring.stl, grip_hub.stl and prints a check of each part (watertight, volume, size).
The default prong hole is 2.95 mm for the 3.0 mm steel dowel prongs: a 0.05 mm interference (light press fit) in a
printed part. (The simulation's rigid bodies cannot interfere, so the simulation uses a 3.1 mm hole.)
Frame: bite block x along the pocket, y across it, z up (the connector is on top of the bite plate); ring flat on z = 0.
"""
import math
import sys
import numpy as np
import trimesh
from trimesh.creation import box, cylinder

HOLE_D = float(sys.argv[1]) if len(sys.argv) > 1 else 2.95

def bx(x0, x1, y0, y1, z0, z1):
    m = box(extents=(x1 - x0, y1 - y0, z1 - z0)); m.apply_translation(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)); return m

def cyl_z(r, z0, z1, x=0.0, y=0.0, sections=48):
    m = cylinder(radius=r, height=z1 - z0, sections=sections); m.apply_translation((x, y, (z0 + z1) / 2)); return m

def union(*ms): return trimesh.boolean.union(list(ms), engine="manifold")
def diff(a, *bs): return trimesh.boolean.difference([a] + list(bs), engine="manifold")

# ---------------- bite block ----------------
PAD = (35.0, 45.0, 4.0)                     # paddle the sensor lies on (x, y, thickness)
RAIL_Y, RAIL = 22.0, (35.0, 3.0, 9.0)       # side rails
LIP_Y, LIP = 19.5, (35.0, 4.0, 2.0)         # lips over the sensor's edges
BP = (4.0, 20.0, 45.0)                      # bite plate
BP_X = PAD[0] / 2 + BP[0] / 2
CONN = (12.0, 20.0, 20.0)                   # connector block on top of the bite plate
PRONG_DY = 4.0                              # prong centres at y = +-4 mm
LEAD = (8.0, 3.0)                           # lead-in: 8 mm opening over the top 3 mm

parts = [bx(-PAD[0] / 2, PAD[0] / 2, -PAD[1] / 2, PAD[1] / 2, 0, PAD[2])]
for sy in (-1, 1):
    parts.append(bx(-RAIL[0] / 2, RAIL[0] / 2, sy * RAIL_Y - RAIL[1] / 2, sy * RAIL_Y + RAIL[1] / 2, PAD[2], PAD[2] + RAIL[2]))
    parts.append(bx(-LIP[0] / 2, LIP[0] / 2, sy * LIP_Y - LIP[1] / 2, sy * LIP_Y + LIP[1] / 2, PAD[2] + 6.5, PAD[2] + 6.5 + LIP[2]))
parts.append(bx(BP_X - BP[0] / 2, BP_X + BP[0] / 2, -BP[1] / 2, BP[1] / 2, PAD[2], PAD[2] + BP[2]))
z0 = PAD[2] + BP[2]
parts.append(bx(BP_X - CONN[0] / 2, BP_X + CONN[0] / 2, -CONN[1] / 2, CONN[1] / 2, z0, z0 + CONN[2]))
block = union(*parts)
cuts = [cyl_z(HOLE_D / 2, z0 - 8.0, z0 + CONN[2] + 1.0, x=BP_X, y=sy * PRONG_DY) for sy in (-1, 1)]
cuts.append(bx(BP_X - LEAD[0] / 2, BP_X + LEAD[0] / 2, -PRONG_DY - LEAD[0] / 2, PRONG_DY + LEAD[0] / 2, z0 + CONN[2] - LEAD[1], z0 + CONN[2] + 1.0))
block = diff(block, *cuts)

# ---------------- aiming ring (flat on z = 0) ----------------
R0, R1, T = 23.0, 30.0, 5.0
LUG, SLOT, SLEEVE, TAB_H, TAB_L = 48.0, (3.5, 6.0), 22.0, 12.0, 42.0
ring = diff(cyl_z(R1, 0, T), cyl_z(R0, -1, T + 1))
sleeve = bx(R1 - 1.0, LUG + 7.0, -7.0, 7.0, 0, SLEEVE)
tab = bx(LUG + 7.0, LUG + TAB_L, -3.0, 3.0, 0, TAB_H)
ring = union(ring, sleeve, tab)
ring = diff(ring, bx(LUG - SLOT[0] / 2, LUG + SLOT[0] / 2, -SLOT[1] / 2, SLOT[1] / 2, -1, SLEEVE + 1))   # slot for the 2.5 x 5 mm upright

# ---------------- grip hub on the upright (10 x 14 x 20 with a 2.7 x 5.2 mm slot) ----------------
hub = diff(bx(-5.0, 5.0, -7.0, 7.0, 0, 20.0), bx(-1.35, 1.35, -2.6, 2.6, -1, 21))

for name, m in (("bite_block", block), ("aiming_ring", ring), ("grip_hub", hub)):
    m.export(f"{name}.stl")
    print(f"{name:12s} watertight={m.is_watertight} volume={m.volume:9.1f} mm3 size_mm={np.round(m.bounds[1]-m.bounds[0],1).tolist()}")
print(f"prong holes: {HOLE_D} mm for 3.0 mm prongs ({(3.0-HOLE_D)/2:.3f} mm interference per side)" if HOLE_D < 3.0 else f"prong holes: {HOLE_D} mm for 3.0 mm prongs ({(HOLE_D-3.0)/2:.3f} mm clearance per side)")
