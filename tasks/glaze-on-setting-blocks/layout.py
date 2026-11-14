"""Every dimension of the task, in one place (metres, kilograms, the station's world frame).

World frame: x forward from the line through the two arm bases, y to the left, z up. The arm bases stand at
x 0, y +-0.305; the station's table top is at z 0.7475.

The window: a complete small fixed-light frame (sill, two jambs, head) of 20 x 60 mm members standing on the table,
its room side facing the arms (-x) and its outside away from them (+x). A fixed outside stop (the rebate upstand)
runs round the opening on the outside; the room side is open, because its glazing bead has been taken off (it lies
on the table beside the frame). The unit goes in from the room side and ends up standing on two setting blocks
against the outside stop.
"""

TOP = 0.7475  # station table top

# -- the frame (fixed, aluminium grey) -------------------------------------------------------------------------
FRAME_Y = 0.0  # the opening is centred on the midline
MEMBER = 0.020  # sill height, jamb width and head height (20 x 60 mm sections)
FRAME_ROOM_X = 0.276  # the frame's room-side face (x)
FRAME_DEPTH = 0.060  # room face to outside face
FRAME_OUT_X = FRAME_ROOM_X + FRAME_DEPTH  # 0.336
REBATE = 0.037  # the rebate's depth: room face to the outside stop's face (a 24 mm unit and a 12 mm bead, 1 mm play)
STOP_X = FRAME_ROOM_X + REBATE  # 0.313: the outside stop's room-facing face; the unit ends against it
STOP_COVER = 0.015  # the outside stop stands 15 mm into the opening all round
OPEN_HALF_L = 0.120  # the opening is 240 mm between the jambs (y -0.12 to +0.12)
SILL_TOP = TOP + MEMBER  # the rebate floor, TOP + 0.020

# -- the two quarter-point marks (thin white plates on the rebate floor) --------------------------------------
MARK_SIZE = (0.030, 0.030, 0.001)
MARK_X = 0.294  # a block on its mark runs from x 0.279 to 0.309: under the whole unit's thickness when it stands
#                 against the stop (x 0.288 to 0.312), and 9 mm room-side of it, where the unit first stands on it
MARK_YS = (FRAME_Y - 0.050, FRAME_Y + 0.050)  # right mark, left mark: a 200 mm unit's quarter points
MARK_TOP = SILL_TOP + MARK_SIZE[2]

# -- the glass unit and where it ends ----------------------------------------------------------------------
UNIT_SIZE = (0.024, 0.200, 0.080)  # thickness (along x), length (along y), height
UNIT_MASS = 0.20
UNIT_X, UNIT_Y = 0.22, 0.29  # standing on its long edge on the table, left of the frame; its nearer end at y +0.19
BLOCK_SIZE = (0.030, 0.030, 0.008)  # 30 mm along x (across the rebate), 30 mm along y, 8 mm tall
BLOCK_MASS = 0.009  # neoprene, 1.23 g/cm3: 7.2 cm3 is 9 g
SET_BLOCK_TOP = MARK_TOP + BLOCK_SIZE[2]  # TOP + 0.029: a block lying on its mark
UNIT_PLAY = 0.001  # standing upright on the blocks the unit's outside face is this far from the stop
UNIT_SET_X = STOP_X - UNIT_PLAY - UNIT_SIZE[0] / 2  # 0.300: the unit's centre, standing in the rebate
HEAD_GAP = 0.006  # the head's soffit is 6 mm over a unit standing on the blocks
SOFFIT = SET_BLOCK_TOP + UNIT_SIZE[2] + HEAD_GAP  # TOP + 0.115
HEAD_TOP = SOFFIT + MEMBER  # TOP + 0.135


def _box(x0, x1, y0, y1, z0, z1):
    """(centre, size) of an axis-aligned box from its bounds."""
    return ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0)


_JY = OPEN_HALF_L + MEMBER  # the jambs' outer faces, y +-0.140
# name: (centre, size). Boxes touch but never overlap.
FRAME_BOXES = {
    "sill": _box(FRAME_ROOM_X, FRAME_OUT_X, -_JY, _JY, TOP, SILL_TOP),
    "jamb_right": _box(FRAME_ROOM_X, FRAME_OUT_X, -_JY, -OPEN_HALF_L, SILL_TOP, SOFFIT),
    "jamb_left": _box(FRAME_ROOM_X, FRAME_OUT_X, OPEN_HALF_L, _JY, SILL_TOP, SOFFIT),
    "head": _box(FRAME_ROOM_X, FRAME_OUT_X, -_JY, _JY, SOFFIT, HEAD_TOP),
    "stop_sill": _box(STOP_X, FRAME_OUT_X, -OPEN_HALF_L, OPEN_HALF_L, SILL_TOP, SILL_TOP + STOP_COVER),
    "stop_head": _box(STOP_X, FRAME_OUT_X, -OPEN_HALF_L, OPEN_HALF_L, SOFFIT - STOP_COVER, SOFFIT),
    "stop_jamb_right": _box(STOP_X, FRAME_OUT_X, -OPEN_HALF_L, -OPEN_HALF_L + STOP_COVER, SILL_TOP + STOP_COVER,
                            SOFFIT - STOP_COVER),
    "stop_jamb_left": _box(STOP_X, FRAME_OUT_X, OPEN_HALF_L - STOP_COVER, OPEN_HALF_L, SILL_TOP + STOP_COVER,
                           SOFFIT - STOP_COVER),
}
SILL_BOX = FRAME_BOXES["sill"]

# -- the bead that was taken off: a prop lying on the table to the left of the frame (fixed; not scored) --------
BEAD_SIZE = (0.240, 0.012, 0.015)  # 240 mm long (the bottom bead), 12 x 15 mm section, lying along x
BEAD_POS = (0.42, 0.17, TOP + BEAD_SIZE[2] / 2)

# -- the setting blocks' tray (right of the frame, beside its right jamb) ----------------------------------------
TRAY_SIZE = (0.090, 0.090, 0.004)
TRAY_X, TRAY_Y = 0.32, -0.24
TRAY_TOP = TOP + TRAY_SIZE[2]
BLOCK_STARTS = ((TRAY_X - 0.020, TRAY_Y), (TRAY_X + 0.020, TRAY_Y))  # 40 mm apart, centre to centre

# -- rule thresholds (scoring.py) --------------------------------------------------------------------------
SILL_TOUCH_GAP = 0.0  # 'touched the sill': the unit and the sill (or a mark) touching or pressed into each other;
#                       any air gap, however small, is not a touch (a chalk print needs contact)
REBATE_SLACK = 0.001  # 'in the rebate': every corner inside the opening's outline + this
STOP_NEAR = 0.003  # ... and its outside face within this of the outside stop
STAND_TILT_DEG = 3.0  # 'standing': the unit's height axis within this of vertical
DOWN_TILT_DEG = 30.0  # more than this at the end: lying down (or stood on end), 0
ON_BLOCK_GAP = 0.001  # 'resting on a block': the unit's underside within this of the block's top, above or below
ON_BLOCK_SHARE = 0.5  # ... with at least this share of the block's top under the unit
UNIT_CENTRE_TIGHT = 0.008  # the unit's centre along y from the opening's centre: 1.0 (12 mm or more to each jamb)
UNIT_CENTRE_LOOSE = 0.015  # 0.75
BLOCK_TIGHT = 0.008  # a block's centre from its mark's centre: 'set' (1.0, 0.25)
BLOCK_LOOSE = 0.020  # 0.75
BLOCK_YAW_DEG = 15.0  # a block's sides within this of the opening's
BLOCK_FLAT_DEG = 10.0  # a block lies flat: its 8 mm axis within this of vertical (either broad face down)
BLOCK_LOW_BAND = (SILL_TOP - 0.001, MARK_TOP + 0.001)  # a block 'on the sill': its lowest point within 1 mm of
#                                                        the sill's or a mark's top (1 mm below to 2 mm above the sill)
OFF_TABLE_DROP = 0.010  # an object's lowest point this far below the table top: it fell off the table
CONTROL_HZ = 20.0  # the program's runs use -E control_hz=20
# Let go and still (fail closed): every rule that credits the unit or a block as resting needs both.
WINDOW_S = 1.0  # 'for the last 1 s': every one of the last 20 control steps, the current one included
WINDOW_STEPS = round(WINDOW_S * CONTROL_HZ)  # 20
LET_GO_CLEAR = 0.030  # 'let go': both grippers' finger zones (rigstate.py) at least 30 mm from the object
STILL_MOVE = 0.002  # 'still': each of the last 20 poses within 2 mm of the current one ...
STILL_TURN_DEG = 2.0  # ... and turned no more than 2 degrees from it
STILL_SPEED, STILL_SPIN = 0.01, 0.5  # 'at rest' on the starting layout (reset detection only): 1 cm/s, 0.5 rad/s
GRAVITY = 9.81
RESET_JUMP = 0.08  # an object landing on its starting spot having moved this much further in one control step
#                    than its last velocity and gravity could carry it was teleported: a new episode began
START_TOL = 0.045  # ... onto its starting spot: within this of it (covers a seeded reset's +-3 cm jitter)

# Colours (sRGB).
ALUMINIUM = (0.60, 0.62, 0.65)
WHITE = (0.95, 0.95, 0.95)
RUBBER = (0.08, 0.08, 0.08)
TRAY_GREY = (0.30, 0.30, 0.32)
GLASS = (0.75, 0.85, 0.90)
BEAD_GREY = (0.45, 0.47, 0.50)
