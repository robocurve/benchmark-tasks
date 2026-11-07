"""Dimensions and layout of the tape task: four LTO cartridges standing on edge in a tote, and a five-slot tape
magazine clamped to the table, its slots facing the arms.

Station frame: metres from the midpoint between the arm bases, x forward, y to the robot's left, z up; heights here
are above the table top (TOP), and task.py adds TOP. A cartridge's own frame: x from its label face (the rear end)
toward its front end, z up from its bottom edge, y across its thickness.
"""

from __future__ import annotations

TOP = 0.7475                      # the station's table top

# -- an LTO cartridge, standing on edge ---------------------------------------------------------------------------------
TAPE_L = 0.102                    # label face to front end (x)
TAPE_H = 0.1054                   # standing height (z)
TAPE_T = 0.0215                   # thickness (y)
TAPE_MASS = 0.20
BARCODES = ("AI0001L9", "AI0002L9", "AI0003L9", "AI0004L9")

# -- the tote: four bays, robot's left to right, holding the cartridges upright ------------------------------------------
TOTE_X = 0.120                    # the cartridges' near faces
TOTE_FLOOR = 0.004
BAY_Y = (0.105, 0.035, -0.035, -0.105)
TOTE_ORDER = (3, 1, 2, 4)         # which cartridge stands in each bay
TURNED = 2                        # this one stands turned round: its label faces the magazine, away from the arms
TOTE_WALL = 0.030
DIVIDER_H = 0.024
TOTE_HALF_W = 0.14

# -- the magazine: five slots, slot 1 on the robot's left ------------------------------------------------------------------
MAG_X = 0.360                     # the slots' open front
MAG_FLOOR = 0.006
MAG_D = 0.110                     # deep enough for a cartridge, with a stop at the back
SLOT_PITCH = 0.030
SLOT_W = 0.026                    # 2.25 mm either side of a cartridge
SLOTS = 5
SLOT_H = 0.116                    # 10.6 mm above a cartridge
NOTCH = 0.015                     # the top stops this far behind the front: room for fingers pushing a cartridge home
SLOT_IN = 0.000                   # a cartridge pushed home stands with its label face this far inside the front


def slot_y(k: int) -> float:
    return (SLOTS + 1) / 2 * SLOT_PITCH - k * SLOT_PITCH            # slot 1 at +0.06 ... slot 5 at -0.06


# -- where a cartridge is turned round: on the table between the tote and the magazine ------------------------------------
TURN_X, TURN_Y = 0.290, 0.0

# -- scoring --------------------------------------------------------------------------------------------------------------
HOME_TOL = 0.003                  # pushed home: its label face within 3 mm of home
IN_SLOT_TOL = 0.006               # in a slot: within 6 mm of the slot's middle, across
LABEL_OUT = 0.9                   # label out: its label face points toward the arms (cos of the angle to -x)
CLEAR = 0.01                      # let go: every part of both arms at least 1 cm from every cartridge in a slot
