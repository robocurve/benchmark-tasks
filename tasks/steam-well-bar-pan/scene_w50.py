"""Steam-well task, W50 scene module (claude-cake, codex 0522). Import from a task file AFTER labkit.floor_bay().
Stock: Avantco W50 / 177W50 countertop food warmer, used UNPLUGGED, cold, dry, empty: 14.5 x 22.5 x 9 in outside
(368.3 x 571.5 x 228.6 mm), 12 x 20 x 6.25 in inside (304.8 x 508.0 x 158.75 mm), the rim is its top edge (Avantco and
Webstaurant listings). Support: Regency stainless equipment stand with undershelf, 24 in (610 mm) work-surface height,
STAND_W x STAND_D top (600ES3048S-class 30 x 48 in by default so pans can be staged beside the warmer; listings).
The W50 rests on the stand top (modelled fixed: an 18 lb appliance on a stand; disclosed). Its walls are solid blocks
(the real unit is sheet metal over a heating element; only the rim, opening and inner depth matter here)."""
import os

import labkit as _lk
from labkit import part
from yam_arena.station import FLOOR_Z

FLOOR = FLOOR_Z
STAND_H = 0.610
STAND_X0 = 0.13                                     # stand front edge: clears the arm pedestals' 0.24 m feet (x <= 0.12)
STAND_W = float(os.environ.get("SW_STAND_W", "0.762"))   # along x (30 in)
STAND_D = float(os.environ.get("SW_STAND_D", "1.219"))   # along y (48 in)
TOP_T = 0.02
STAND_TOP = FLOOR + STAND_H
W_OUT = (0.3683, 0.5715, 0.2286)
W_IN = (0.3048, 0.5080, 0.15875)
WC = (float(os.environ.get("SW_WCX", "0.36")), float(os.environ.get("SW_WCY", "0.0")))
RIM = STAND_TOP + W_OUT[2]
STEEL = (0.74, 0.75, 0.77)
BODY = (0.62, 0.63, 0.65)
GALV = (0.58, 0.59, 0.60)
# render-only looks (v2): brushed stainless; detail shapes below have NO collider (labkit.VISUAL_ONLY)
WELL_LINER = (0.50, 0.51, 0.53)       # slightly darker, duller well interior
ROLL = (0.80, 0.81, 0.83)             # rolled outer top edge
KNOB = (0.06, 0.06, 0.07)             # black thermostat knob
KNOB_MARK = (0.92, 0.92, 0.92)        # white pointer on the knob
LAMP = (0.98, 0.60, 0.12)             # amber power indicator
VENT = (0.12, 0.12, 0.13)             # louvre slots
FOOT = (0.05, 0.05, 0.05)             # black rubber feet
for _c, _m, _r in ((STEEL, 1.0, 0.33), (BODY, 1.0, 0.38), (GALV, 1.0, 0.45), (WELL_LINER, 1.0, 0.5), (ROLL, 1.0, 0.25)):
    _lk.METAL[_c] = (_m, _r)
for _c in (WELL_LINER, ROLL, KNOB, KNOB_MARK, LAMP, VENT, FOOT):
    _lk.VISUAL_ONLY.add(_c)


def _stand_boxes():
    cx, cy = STAND_X0 + STAND_W / 2, 0.0
    b = [((cx, cy, STAND_H - TOP_T / 2), (STAND_W, STAND_D, TOP_T), STEEL),
         ((cx, cy, 0.25), (STAND_W - 0.06, STAND_D - 0.06, 0.012), GALV)]           # undershelf
    for sx in (-1, 1):
        for sy in (-1, 1):
            b.append(((cx + sx * (STAND_W / 2 - 0.02), cy + sy * (STAND_D / 2 - 0.02), (STAND_H - TOP_T) / 2),
                      (0.04, 0.04, STAND_H - TOP_T), GALV))
    return tuple(b)


def _w50_boxes():
    ox, oy, oz = W_OUT
    ix, iy, iz = W_IN
    wx, wy = (ox - ix) / 2, (oy - iy) / 2
    floor_t = oz - iz
    cx, cy = WC
    b = [((cx, cy, floor_t / 2), (ox, oy, floor_t), BODY)]                         # base below the inner well
    for sx in (-1, 1):
        b.append(((cx + sx * (ix / 2 + wx / 2), cy, oz / 2), (wx, oy, oz), BODY))   # long walls (rim 31.75 mm)
    for sy in (-1, 1):
        b.append(((cx, cy + sy * (iy / 2 + wy / 2), oz / 2), (ix, wy, oz), BODY))  # end walls (rim 31.75 mm)
    # ---- visual only (no collider) ----
    t = 0.0008  # liner plates 0.8 mm thick, 0.2 mm inside the inner walls and floor
    b.append(((cx, cy, floor_t + 0.0002 + t / 2), (ix - 0.001, iy - 0.001, t), WELL_LINER))
    for sx in (-1, 1):
        b.append(((cx + sx * (ix / 2 - 0.0002 - t / 2), cy, floor_t + iz / 2), (t, iy - 0.001, iz - 0.001), WELL_LINER))
    for sy in (-1, 1):
        b.append(((cx, cy + sy * (iy / 2 - 0.0002 - t / 2), floor_t + iz / 2), (ix - 0.001, t, iz - 0.001), WELL_LINER))
    # louvre slots on both end panels (the +-y faces), 6 x 70 mm x 4 mm, standing 0.5 mm proud
    for sy in (-1, 1):
        for k in range(6):
            b.append(((cx + (k - 2.5) * 0.045, cy + sy * (oy / 2 + 0.0005), 0.07), (0.035, 0.001, 0.005), VENT))
    # front panel (the -x face): amber lamp
    b.append(((cx - ox / 2 - 0.0015, cy + 0.12, 0.10), (0.003, 0.010, 0.010), LAMP))
    return tuple(b)


def _w50_cyls():
    ox, oy, oz = W_OUT
    cx, cy = WC
    c = []
    # thermostat knob on the front panel, axis x, 40 mm across, 18 mm deep, white pointer
    c.append(((cx - ox / 2 - 0.009, cy + 0.17, 0.10), 0.020, 0.018, KNOB, "X"))
    c.append(((cx - ox / 2 - 0.0185, cy + 0.17, 0.112), 0.0025, 0.001, KNOB_MARK, "X"))
    # rolled outer top edge: 4 mm round bead along the four outer top edges, top flush with the rim plane
    r = 0.004
    for sx in (-1, 1):
        c.append(((cx + sx * (ox / 2 - r), cy, oz - r), r, oy, ROLL, "Y"))
    for sy in (-1, 1):
        c.append(((cx, cy + sy * (oy / 2 - r), oz - r), r, ox, ROLL, "X"))
    # four rubber feet, 12 mm, visible just outside the body footprint corners (the body itself rests on the stand)
    for sx in (-1, 1):
        for sy in (-1, 1):
            c.append(((cx + sx * (ox / 2 + 0.004), cy + sy * (oy / 2 - 0.03), 0.006), 0.009, 0.012, FOOT, "Z"))
    return tuple(c)


STAND = part("equipment_stand", pos=(0.0, 0.0, FLOOR), fixed=True, boxes=_stand_boxes(), friction=0.6)
WARMER = part("food_warmer_w50", pos=(0.0, 0.0, STAND_TOP), fixed=True, boxes=_w50_boxes(), cylinders=_w50_cyls(), friction=0.6)
