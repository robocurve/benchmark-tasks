"""Geometry and station layout for hot-swap-failed-drive-sled. Pure Python.

World frame: x forward, y left, z up, metres.
Arm bases: Left (0, +0.305, 0.75), Right (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Contents:
- Rack block at x = 0.400 (front face x = 0.330), 3 bays along y at -0.120, 0.000, +0.120:
  Channel clearance 12 mm per side, 5 mm lead-in wedges at mouth.
- Three sleds: 150 (x) x 80 (y) x 35 (z) mm; front handle 20 (x) x 45 (y) x 30 (z) mm.
  Bays:
    Bay 0 (y = +0.120): Healthy, green status lamp
    Bay 1 (y =  0.000): Failed, red status lamp (middle bay)
    Bay 2 (y = -0.120): Healthy, green status lamp
- Per-bay latch buttons on rack top at front edge (x = 0.355): 40 x 40 x 12 mm caps.
- Status lamps: 25 mm diameter markers above bays on rack top.
- Spare stand on the left at (0.280, +0.300):
  Two spares:
    Blue handle (correct keying) at y = +0.250
    Orange handle (wrong keying) at y = +0.350
- Discard pad on the right at (0.280, -0.300), 130 x 130 mm.
- Left arm: holds the latch and fetches/inserts the spare. Right arm: pulls the old sled and parks it on the pad.
"""

from __future__ import annotations

TOP = 0.7475

# Colours (sRGB)
DARK_GREY = (0.22, 0.22, 0.24)
RACK_GREY = (0.35, 0.35, 0.38)
SLED_METAL = (0.70, 0.70, 0.72)
HANDLE_BLACK = (0.15, 0.15, 0.15)
HANDLE_BLUE = (0.15, 0.35, 0.85)
HANDLE_ORANGE = (0.90, 0.45, 0.10)
LAMP_RED = (0.85, 0.15, 0.15)
LAMP_GREEN = (0.15, 0.85, 0.25)
BUTTON_LATCH = (0.80, 0.75, 0.20)  # metallic brass/gold latch button
PAD_RUBBER = (0.18, 0.18, 0.20)

# Rack dimensions
RACK_SIZE = (0.160, 0.400, 0.060)
RACK_POS = (0.410, 0.000, TOP + RACK_SIZE[2] / 2)  # front face at x = 0.330
RACK_FRONT_X = 0.330
PLATE_FRONT_X = 0.355   # the top plate starts 35 mm behind the front face so the gripper palm can reach the handles
RACK_TOP_Z = TOP + RACK_SIZE[2]

# Three bays along y
BAY_Y = {
    "bay_0": +0.120,  # healthy (left)
    "bay_1":  0.000,  # failed (middle)
    "bay_2": -0.120,  # healthy (right)
}
FAILED_BAY = "bay_1"

# Sled dimensions
SLED_SIZE = (0.150, 0.080, 0.025)
SLED_MASS = 0.050
HANDLE_SIZE = (0.030, 0.045, 0.030)   # 30 mm deep so the fingers get a firm hold in front of the rack
HANDLE_OFFSET_X = -SLED_SIZE[0] / 2 - HANDLE_SIZE[0] / 2  # on front face of sled

# Bay mouth and seated positions
BAY_FLOOR_Z = TOP + 0.010
SLED_REST_Z = BAY_FLOOR_Z + SLED_SIZE[2] / 2
# When seated, sled back is at x = 0.480, front at x = 0.330, center at x = 0.405
SLED_SEATED_X = 0.405
HANDLE_SEATED_X = SLED_SEATED_X + HANDLE_OFFSET_X  # ~0.320

# Latch buttons on rack top at front edge
BUTTON_SIZE = (0.040, 0.040, 0.012)
BUTTON_TRAVEL = 0.008      # 8 mm downward
BUTTON_REST_Z = RACK_TOP_Z + BUTTON_TRAVEL + BUTTON_SIZE[2] / 2   # cap bottom sits one stroke above the rack top
BUTTON_CAP_TOP_REST_Z = BUTTON_REST_Z + BUTTON_SIZE[2] / 2
LATCH_BUTTON_X = 0.405     # on the top plate, 75 mm back from the rack front face
LATCH_BUTTON_DY = 0.055    # cap sits 55 mm to the left of its bay's centre line, clear of the handle the right arm grips


def latch_xy(bay: str) -> tuple[float, float]:
    return (LATCH_BUTTON_X, BAY_Y[bay] + LATCH_BUTTON_DY)

LATCH_THRESHOLD = -0.0064  # pressed if disp <= -6.4 mm

# Status lamps (markers) on rack top
LAMP_DIAMETER = 0.025
LAMP_SIZE = (LAMP_DIAMETER, LAMP_DIAMETER, 0.004)
LAMP_X = 0.455
LAMP_Z = RACK_TOP_Z + LAMP_SIZE[2] / 2
LAMP_ON_Z = LAMP_Z
LAMP_OFF_Z = LAMP_Z - 0.050  # hidden

# Spare stand on the left at (0.280, +0.300)
STAND_SIZE = (0.180, 0.180, 0.020)
STAND_POS = (0.280, +0.300, TOP + STAND_SIZE[2] / 2)
STAND_TOP_Z = TOP + STAND_SIZE[2]

SPARE_POS = {
    "blue": (0.280, +0.250, STAND_TOP_Z + SLED_SIZE[2] / 2),
    "orange": (0.280, +0.350, STAND_TOP_Z + SLED_SIZE[2] / 2),
}

# Discard pad on the right at (0.280, -0.300)
DISCARD_PAD_SIZE = (0.130, 0.130, 0.010)
DISCARD_PAD_POS = (0.280, -0.300, TOP + DISCARD_PAD_SIZE[2] / 2)
DISCARD_TOP_Z = TOP + DISCARD_PAD_SIZE[2]
DISCARD_SPOT = (0.280, -0.300, DISCARD_TOP_Z + SLED_SIZE[2] / 2)

# Navigation and Grasp Heights
HOVER_Z = TOP + 0.100                                  # ~0.8475 m
GRASP_Z = SLED_REST_Z                                  # ~0.775 m
LATCH_PRESS_Z = BUTTON_CAP_TOP_REST_Z - 0.014 + 0.013  # fingertip 14 mm below the rest cap top (PAD_TO_TCP = 0.013)

PAD_TO_TCP = 0.013
MAX_ARM_REACH = 0.50
LEFT_BASE = (0.0, +0.305)
RIGHT_BASE = (0.0, -0.305)



def rack_world_boxes() -> list[tuple]:
    """The rack as an open-front shell (world frame): floor, back wall, two dividers, two outer walls and a top plate.

    Each bay is a 104 mm wide slot (12 mm clearance per side around the 80 mm sled) and the top plate leaves 12 mm above
    the sled. A solid block would overlap the sleds and throw them out of the rack at start-up.
    """
    x0, x1 = RACK_FRONT_X, RACK_POS[0] + RACK_SIZE[0] / 2
    half_w = RACK_SIZE[1] / 2
    xc, xl = (x0 + x1) / 2, x1 - x0
    floor_t, back_t, plate_t = 0.010, 0.009, 0.008
    wall_len = xl - back_t
    wall_xc = x0 + wall_len / 2
    wall_zc = TOP + floor_t + (RACK_SIZE[2] - floor_t) / 2
    wall_h = RACK_SIZE[2] - floor_t
    slot_half = SLED_SIZE[1] / 2 + 0.012
    div_w = (BAY_Y["bay_0"] - BAY_Y["bay_1"]) - 2 * slot_half
    out_w = half_w - (BAY_Y["bay_0"] + slot_half)
    boxes = [
        ((xc, 0.0, TOP + floor_t / 2), (xl, RACK_SIZE[1], floor_t), RACK_GREY),
        ((x1 - back_t / 2, 0.0, TOP + RACK_SIZE[2] / 2), (back_t, RACK_SIZE[1], RACK_SIZE[2]), RACK_GREY),
        (((PLATE_FRONT_X + x1) / 2, 0.0, RACK_TOP_Z - plate_t / 2), (x1 - PLATE_FRONT_X, RACK_SIZE[1], plate_t), RACK_GREY),
    ]
    for sign in (+1, -1):
        boxes.append(((wall_xc, sign * (slot_half + div_w / 2), wall_zc), (wall_len, div_w, wall_h), RACK_GREY))
        boxes.append(((wall_xc, sign * (BAY_Y["bay_0"] + slot_half + out_w / 2), wall_zc), (wall_len, out_w, wall_h), RACK_GREY))
    return boxes
