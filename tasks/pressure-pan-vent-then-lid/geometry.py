"""Geometry for pressure_pan_vent_then_lid (metres; the table top stands for a hob top).

Frames: x forward, y left from the midpoint between the arm bases; z of a part is its height above the table top.
The pan part's origin is the centre of the pan's base on the table; the lid's origin is the centre of its underside.
"""

from __future__ import annotations

TABLE_TOP = 0.7475

COLOR_PAN = (0.604, 0.627, 0.659)
COLOR_HANDLE = (0.15, 0.15, 0.17)
COLOR_POST = (0.38, 0.40, 0.43)
COLOR_VENT = (0.839, 0.157, 0.157)
COLOR_LOCK = (0.55, 0.56, 0.58)
COLOR_LID = (0.725, 0.745, 0.769)
COLOR_KNOB = (0.10, 0.10, 0.11)
COLOR_VALVE = (0.45, 0.46, 0.48)
COLOR_TRIVET = (0.424, 0.478, 0.537)
COLOR_STEAM = (0.93, 0.95, 0.97)

# Pan: steel, fixed in place (stands in for a 25 kg pan locked to the hob), 18 cm across, 8 cm tall.
PAN_NAME = "pp_pot"
PAN_XY = (0.28, 0.0)
PAN_RADIUS_M = 0.09
PAN_HEIGHT_M = 0.08
PAN_MASS_KG = 25.0
PAN_FRICTION = 1.2
PAN_CYL = ((0.0, 0.0, PAN_HEIGHT_M / 2), PAN_RADIUS_M, PAN_HEIGHT_M)
PAN_HANDLE = ((0.0, 0.13, 0.045), (0.035, 0.08, 0.03))                      # y 0.09..0.17, z 0.03..0.06
_P = 0.0764                                                                  # four hob support posts on the diagonals,
PAN_POSTS = tuple(((sx * _P, sy * _P, 0.043), (0.015, 0.015, 0.086)) for sx in (1, -1) for sy in (1, -1))  # 8 mm off the lid
PAN_KEY_PIN = ((-0.103, 0.0, 0.046), (0.01, 0.01, 0.092))                    # key post at the -x edge, 8 mm off the lid

# Vent assembly: ONE sprung slide along +x carried by the pan (the pan part owns the slider, so its own handle never
# collides with it). Button face 4 cm wide x 6 cm tall on the handle's -x side, its pushing face at x = PAN_X - 0.0295.
VENT_NAME = "vent"
VENT_TRAVEL_M = 0.028
VENT_SPRING_N_PER_M = 8.0
VENT_DAMPING = 5.0
VENT_MASS_KG = 0.05
VENT_REST = (-0.0235, 0.165, 0.075)
VENT_BUTTON = ((-0.0235, 0.165, 0.075), (0.012, 0.04, 0.06))               # z 0.045..0.105 (rides above the handle top)
VENT_STEM = ((0.0, 0.165, 0.0475), (0.035, 0.015, 0.015))                    # through the handle
VENT_CROSS = ((0.01, 0.14625, 0.0475), (0.015, 0.0525, 0.015))               # y 0.12..0.1725
VENT_POST = ((0.01, 0.1275, 0.0745), (0.015, 0.015, 0.039))                  # up to the lock arm
VENT_ARM = ((0.01, 0.114, 0.098), (0.03, 0.042, 0.008))                      # lock arm x -0.005..0.025, y 0.093..0.135 (just outside the lid's rim), z 0.094..0.102
VENT_BUTTON_FACE_X = PAN_XY[0] - 0.0295                                      # button's pushing face at rest

# Lid: steel disc 18 cm across, 4 mm thick, 450 g; round knob 4 cm across and 5 cm tall at the centre; lug under the lock
# arm at the +y rim; a grey valve disc 3.5 cm from the knob toward +x (the steam outlet).
LID_NAME = "pp_lid"
LID_REST_Z_M = 0.0802                    # underside 0.2 mm above the pan rim at rest
LID_MASS_KG = 0.45
LID_FRICTION = 0.4
LID_DISC = ((0.0, 0.0, 0.002), 0.09, 0.004)
LID_KNOB = ((0.0, 0.0, 0.029), 0.02, 0.05)                                    # z 0.004..0.054 above the underside
LID_VALVE = ((0.035, 0.0, 0.009), 0.01, 0.01)
LID_LUG = ((0.0, 0.10, 0.003), (0.03, 0.02, 0.006))                           # x +-0.015, y 0.09..0.11, z 0..0.006
LID_KNOB_GRIP_Z_M = 0.032                 # commanded gripper centre above the lid underside while holding the knob
VALVE_XY = (PAN_XY[0] + LID_VALVE[0][0], PAN_XY[1])

# Trivet: grey silicone 22 x 22 x 1 cm, fixed (25 kg), 5 mm clear of the pan's rim circle.
TRIVET_NAME = "pp_trivet"
TRIVET_XY = (0.28, -0.205)
TRIVET_SIZE = (0.22, 0.22, 0.01)
TRIVET_MASS_KG = 25.0
TRIVET_FRICTION = 1.2

# Plume (display only) and its keep-out zone.
PLUME_STEPS = 100                         # 5 s at 20 Hz
PLUME_TRIGGER_M = 0.005                   # vent pressed this far starts the plume
KEEPOUT_RADIUS_M = 0.045
KEEPOUT_FINGER_MARGIN_M = 0.03            # a gripper centre this much outside the radius still has a finger in the zone
KEEPOUT_Z_MIN_M = 0.084
KEEPOUT_Z_MAX_M = 0.25

# Scoring tolerances (the same numbers are in task.json).
UNLOCK_PRESS_M = 0.022
UNLOCK_HOLD_STEPS = 20
LIFT_MIN_M = 0.03
LIFT_TILT_DEG = 15.0
BYPASS_RISE_M = 0.01
TRIVET_TOP_M = TRIVET_SIZE[2]
SEAT_MAX_Z_ERR_M = 0.003
SEAT_MAX_OFFCENTRE_M = 0.02
SEAT_MAX_TILT_DEG = 3.0
VENT_RELEASED_M = 0.003
PAN_MAX_SHIFT_M = 0.005
ABOVE_PAN_RADIUS_M = 0.12
RELEASE_MIN_DIST_M = 0.03
TCP_BELOW_LINK_M = 0.138
FINGERS_OPEN_FRACTION = 0.9
STILL_SPEED_M_S = 0.01
SUCCESS_HOLD_TIME_S = 1.0
