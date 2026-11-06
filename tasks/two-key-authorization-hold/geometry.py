"""Geometry and layout for two-key-authorization-hold task. Pure Python.

World frame (the station's): x forward from the arm bases, y left, z up, metres.
The left arm base is at (0, +0.305, 0.75), the right at (0, -0.305, 0.75).
Table top z = 0.7475 m (TOP).

Station contents:
- Heavy controller fixture base: 240 x 680 x 20 mm, mass 8.0 kg, friction 1.0,
  centered at (0.300, 0.000, TOP + 0.010 = 0.7575).
- Left key: yellow button at (0.300, +0.280), 40 x 40 x 12 mm cap, 8 mm travel (Z down), 100 N/m.
- Right key: yellow button at (0.300, -0.280), 40 x 40 x 12 mm cap, 8 mm travel (Z down), 100 N/m.
- CONFIRM button: white button at (0.300, 0.000), 40 x 40 x 12 mm cap, 8 mm travel (Z down), 100 N/m.
- Left guard: 90 x 90 x 25 mm red sliding lid over left key, slides along +Y outward (travel 70 mm).
- Right guard: 90 x 90 x 25 mm red sliding lid over right key, slides along -Y outward (travel 70 mm).
- Lamps:
  * ARM PROGRESS strip: 4 cyan lamps (25 mm diameter) at x = 0.400, y in [-0.045, -0.015, +0.015, +0.045]
  * ARMED lamp: green 30 mm disc at (0.360, 0.000)
  * FAULT lamp: red 30 mm disc at (0.360, +0.060)
"""

from __future__ import annotations

TOP = 0.7475  # table top height in metres

# Colours (sRGB 0..1)
DARK_GREY = (0.22, 0.22, 0.24)
GREY = (0.55, 0.56, 0.58)
WHITE = (0.92, 0.92, 0.92)
YELLOW_CAP = (0.95, 0.80, 0.10)
WHITE_CAP = (0.92, 0.92, 0.92)
RED_LID = (0.85, 0.15, 0.15)
LAMP_OFF = (0.18, 0.18, 0.20)
LAMP_CYAN = (0.10, 0.85, 0.95)
LAMP_GREEN = (0.10, 0.95, 0.20)
LAMP_RED = (0.95, 0.10, 0.10)

# Fixture dimensions
FIXTURE_MASS = 8.0       # kg
FIXTURE_FRICTION = 1.0   # high friction
BASE_SIZE = (0.240, 0.680, 0.020)
BASE_POS = (0.300, 0.000, TOP + BASE_SIZE[2] / 2)  # (0.300, 0.000, 0.7575)
FIXTURE_TOP_Z = TOP + BASE_SIZE[2]                  # 0.7675 m

# Buttons
BUTTONS = ("key_left", "key_right", "confirm")

BUTTON_POS_XY: dict[str, tuple[float, float]] = {
    "key_left":  (0.300, +0.280),
    "key_right": (0.300, -0.280),
    "confirm":   (0.300,  0.000),
}

BUTTON_ARM: dict[str, str] = {
    "key_left":  "left",
    "key_right": "right",
    "confirm":   "right",
}

BUTTON_CAP_COLORS: dict[str, tuple[float, float, float]] = {
    "key_left":  YELLOW_CAP,
    "key_right": YELLOW_CAP,
    "confirm":   WHITE_CAP,
}

BUTTON_CAP_SIZE = (0.040, 0.040, 0.012)
BUTTON_TRAVEL = 0.008
BUTTON_SPRING_K = 100.0
BUTTON_MASS = 0.005
BUTTON_DAMPING = 4.0

BUTTON_REST_BOTTOM_Z = FIXTURE_TOP_Z + BUTTON_TRAVEL
BUTTON_REST_Z = BUTTON_REST_BOTTOM_Z + BUTTON_CAP_SIZE[2] / 2
BUTTON_REST_TOP_Z = BUTTON_REST_BOTTOM_Z + BUTTON_CAP_SIZE[2]

BUTTON_REST_POS: dict[str, tuple[float, float, float]] = {
    b: (BUTTON_POS_XY[b][0], BUTTON_POS_XY[b][1], BUTTON_REST_Z) for b in BUTTONS
}

BUTTON_LOCAL_POS: dict[str, tuple[float, float, float]] = {
    b: (BUTTON_POS_XY[b][0] - BASE_POS[0], BUTTON_POS_XY[b][1] - BASE_POS[1], BUTTON_REST_Z - BASE_POS[2])
    for b in BUTTONS
}

BUTTON_PRESS_THRESHOLD = -0.0064    # -6.4 mm
BUTTON_RELEASE_THRESHOLD = -0.0035  # -3.5 mm

BUTTON_PRESS_DEPTH_CMD = 0.014
PAD_TIP_PAST_TCP = 0.013
BUTTON_PRESS_TCP_Z = BUTTON_REST_TOP_Z - BUTTON_PRESS_DEPTH_CMD + PAD_TIP_PAST_TCP

# Guards
GUARDS = ("guard_left", "guard_right")
GUARD_SIZE = (0.090, 0.090, 0.025)
GUARD_TRAVEL = 0.080  # 80 mm travel outward
GUARD_OPEN_THRESHOLD = 0.060  # 60 mm required to count as open
GUARD_MASS = 0.150

# Guard rest positions: centered over the buttons, elevated to clear button rest top
GUARD_REST_Z = BUTTON_REST_TOP_Z + 0.002 + GUARD_SIZE[2] / 2  # 2 mm gap above button top

GUARD_REST_POS: dict[str, tuple[float, float, float]] = {
    "guard_left":  (0.300, +0.280, GUARD_REST_Z),
    "guard_right": (0.300, -0.280, GUARD_REST_Z),
}

GUARD_LOCAL_POS: dict[str, tuple[float, float, float]] = {
    g_name: (
        GUARD_REST_POS[g_name][0] - BASE_POS[0],
        GUARD_REST_POS[g_name][1] - BASE_POS[1],
        GUARD_REST_POS[g_name][2] - BASE_POS[2],
    )
    for g_name in GUARDS
}

# Guard slide axes: left slides +Y, right slides -Y
GUARD_AXIS: dict[str, str] = {
    "guard_left":  "+Y",
    "guard_right": "-Y",
}

# Lamps
ARMED_LAMP_POS = (0.360, 0.000)
FAULT_LAMP_POS = (0.360, +0.060)

# 4 Progress lamps along y at x = 0.400
PROGRESS_LAMP_Y = [-0.045, -0.015, +0.015, +0.045]
PROGRESS_LAMP_X = 0.400

LAMP_DIAMETER_STATUS = 0.030     # 30 mm
LAMP_DIAMETER_PROGRESS = 0.025   # 25 mm

LAMP_ON_Z = FIXTURE_TOP_Z + 0.001
LAMP_OFF_Z = TOP - 0.100
STOWED_Z = LAMP_OFF_Z

# Arm home / hover
HOVER_Z = TOP + 0.120
HOME_L = (0.200, +0.280, HOVER_Z)
HOME_R = (0.200, -0.280, HOVER_Z)


def fixture_boxes() -> list[tuple]:
    """Returns collision boxes for fixture base plate."""
    return [
        (BASE_POS, BASE_SIZE, DARK_GREY),
    ]
