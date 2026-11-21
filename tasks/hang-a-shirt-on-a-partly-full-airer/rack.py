"""The airer as one fixed body (its feet are clamped to the table), and the fabric sheets it hosts. Pure Python.

Each end frame is pine: a 30 x 18 mm top crossbar the rails pass through, two splayed 18 x 18 mm legs and an
18 x 45 mm foot batten, in the plane y = +-FRAME_Y. The rails are 12 mm beech dowels.
"""

from __future__ import annotations

import bodykit
import clothkit
import layout as L
from bodykit import Body, BoxShape, CylShape, segment

PINE = (0.80, 0.64, 0.42)
BEECH = (0.86, 0.71, 0.52)
bodykit.FINISH[PINE] = (0.62, 0.0)
bodykit.FINISH[BEECH] = (0.45, 0.0)

RAIL_FRICTION = (0.5, 0.45)  # cotton on lacquered beech
X0, X1 = L.RAIL_X["A"] - 0.03, L.RAIL_X["C"] + 0.03  # the crossbar's ends


def rack_body() -> Body:
    shapes = []
    zc = L.RAIL_Z - L.TOP  # rail axis above the table (body frame origin on the table, at y = 0)
    xm = (X0 + X1) / 2
    for side in (1, -1):
        y = side * L.FRAME_Y
        shapes.append(BoxShape((xm, y, zc), (X1 - X0, L.FRAME_T, 0.030), PINE))  # crossbar, rails through it
        shapes.append(BoxShape((xm, y, 0.009), (X1 - X0 + 0.12, 0.045, 0.018), PINE))  # foot batten
        for x_top, x_foot in ((X0 + 0.02, X0 - 0.04), (X1 - 0.02, X1 + 0.04)):
            shapes.append(segment((x_foot, y, 0.018), (x_top, y, zc - 0.015), 0.018, PINE))
    for x in L.RAIL_X.values():
        shapes.append(CylShape((x, 0.0, zc), L.RAIL_R, L.RAIL_LEN + 0.02, BEECH, axis="Y"))
    return Body.build("airer", pos=(0.0, 0.0, L.TOP), mass=1.5, shapes=shapes, kinematic=True,
                      friction=RAIL_FRICTION, contact_offset=0.003)


RACK = rack_body()

# -- fabric ------------------------------------------------------------------------------------------------------------
TEE_COLORS = ((0.86, 0.36, 0.30), (0.96, 0.95, 0.92))  # coral jersey, cream rib trim
CLOTH_COLORS = {"B": ((0.62, 0.78, 0.90), (0.95, 0.96, 0.97)),  # pale blue terry, white hem
                "C": ((0.97, 0.88, 0.55), (0.95, 0.96, 0.97))}  # pale yellow terry, white hem

TEE_MESH = L.tshirt()
TEE = clothkit.sheet(name="tshirt", host="airer", rest=TEE_MESH.rest, start=TEE_MESH.start, faces=TEE_MESH.faces,
                     face_kind=TEE_MESH.face_kind, colors=TEE_COLORS, mass=L.TEE_MASS, thickness=L.TEE_T,
                     youngs=4.0e5, poisson=0.3, bend=1.0e-4, friction=(0.6, 0.5), iterations=48)
CLOTH_MESH = {r: L.washcloth(r) for r in L.LOADED}
CLOTHS = {r: clothkit.sheet(name=L.LOADED[r], host="airer", rest=m.rest, start=m.start, faces=m.faces,
                            face_kind=m.face_kind, colors=CLOTH_COLORS[r], mass=L.CLOTH_MASS, thickness=L.CLOTH_T,
                            youngs=2.0e5, poisson=0.3, bend=1.0e-3, friction=(0.7, 0.6), iterations=32)
          for r, m in CLOTH_MESH.items()}
