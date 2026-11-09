"""A red fire extinguisher stands on the station table: a cylinder body with a carry bar on top, a
small pressure-gauge dial on its front face, and a steel retaining pin resting in a rail bracket
raised above the body's top, its pull end reaching toward the right arm. NSPIRE fire-extinguisher
check: the retaining pin must be in place and the gauge must read in the green - an undercharged or
unpinned extinguisher will not work when it is needed.

The check has two halves: pull the retaining pin out of its bracket (at least 2 cm, the tamper seal
broken), read the gauge, and file the finding by leaving the white report tag on the matching pad:
green for the gauge reads in the green, red for the gauge reads out of range.

Run it (per-trial gauge state via the FAULT_MODE environment variable, 'charged' or 'undercharged'):
    FAULT_MODE=charged inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=extinguisher_pin_check.py -E scene=station -E object_jitter=0 \
      -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "pull the extinguisher's retaining pin, check the gauge and file the report" \
      --scorer task_score --max-steps 900 --store-frames --log-dir ~/yam-eval/ext-demo
"""

import os

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"

TOP = TABLE_TOP[SCENE]

FAULT_MODE = os.environ.get("FAULT_MODE", "charged")
if FAULT_MODE not in ("charged", "undercharged"):
    raise ValueError(f"FAULT_MODE must be charged | undercharged, got {FAULT_MODE!r}")

# Extinguisher body, carry bar and gauge dial on its front face.
BX, BY = 0.28, -0.08
BODY = Cylinder("extinguisher", pos=(BX, BY, TOP + 0.05), radius=0.05, height=0.10,
                color=(0.75, 0.12, 0.10))
CARRY = Box("carry_bar", pos=(BX, BY, TOP + 0.1075), size=(0.09, 0.015, 0.012),
            color=(0.25, 0.26, 0.28))
DIAL = Box("gauge_dial", pos=(BX, BY - 0.048, TOP + 0.075), size=(0.04, 0.005, 0.04),
           color=(0.92, 0.92, 0.94))
ZONE_GREEN = Box("gauge_green", pos=(BX, BY - 0.0525, TOP + 0.0875), size=(0.008, 0.004, 0.012),
                 color=(0.20, 0.65, 0.30))
ZONE_RED = Box("gauge_red", pos=(BX - 0.012, BY - 0.0525, TOP + 0.0625), size=(0.008, 0.004, 0.012),
               color=(0.80, 0.15, 0.12))
NEEDLE_POS = {"charged": (BX, BY - 0.0525, TOP + 0.081),
              "undercharged": (BX - 0.012, BY - 0.0525, TOP + 0.0725)}
NEEDLE = Box("gauge_needle", pos=NEEDLE_POS[FAULT_MODE], size=(0.006, 0.004, 0.010),
             color=(0.15, 0.15, 0.17))

# The retaining pin rides a plate raised on a riser above the body's top, so the arm approaches it
# horizontally well clear of the body. The stage matches the proven slide-bolt's exactly: a wide
# plate, two guide rails 2.2 cm apart, and the pin = the bolt's cross-section (1.6 x 1.1 cm).
RISER = Box("pin_riser", pos=(BX, -0.09, TOP + 0.1275), size=(0.024, 0.03, 0.05),
            color=(0.45, 0.47, 0.50))
SHELF = Box("pin_shelf", pos=(BX, -0.14, TOP + 0.1525), size=(0.10, 0.14, 0.005),
            color=(0.45, 0.47, 0.50))
RAIL_L = Box("rail_left", pos=(BX - 0.011, -0.14, TOP + 0.157), size=(0.004, 0.12, 0.008),
             color=(0.30, 0.31, 0.33))
RAIL_R = Box("rail_right", pos=(BX + 0.011, -0.14, TOP + 0.157), size=(0.004, 0.12, 0.008),
             color=(0.30, 0.31, 0.33))
PIN_Y = -0.105   # seated at the bracket's back edge; the pull draws it out the front (-y)
PIN_Z = TOP + 0.168
PIN = Box("retaining_pin", pos=(BX, PIN_Y, PIN_Z), size=(0.016, 0.07, 0.011), mass=0.06,
          color=(0.70, 0.72, 0.75))

# Report station: two pads and the white tag.
PAD_OK = Box("pad_ok", pos=(0.14, 0.05, TOP + 0.006), size=(0.07, 0.075, 0.012), color=(0.20, 0.65, 0.30))
PAD_FAULT = Box("pad_fault", pos=(0.14, -0.05, TOP + 0.006), size=(0.07, 0.075, 0.012),
                color=(0.80, 0.15, 0.12))
TAG = Box("report_tag", pos=(0.20, -0.10, TOP + 0.015), size=(0.03, 0.03, 0.03), mass=0.06,
          color=(0.95, 0.95, 0.95))

OBJECTS = [BODY, CARRY, DIAL, ZONE_GREEN, ZONE_RED, NEEDLE, RISER, SHELF, RAIL_L, RAIL_R, PIN,
           PAD_OK, PAD_FAULT, TAG]

PADS = {"charged": PAD_OK, "undercharged": PAD_FAULT}
PULL_MIN = 0.02                 # the pin's centre pulled this far -y of its start = un-pinned
RAIL_TOL = 0.012                # the pin still rides its rails
PIN_Z_MAX = TOP + 0.18          # ...and still on the bracket
REPORT_ON = 0.025               # tag centre within this of a pad's centre = filed on it


def _pin_pulled(objects):
    """The pin slid out of its seat along its travel direction and is at rest (on the bracket or
    fallen off it - a removed pin is removed wherever it lies)."""
    p = objects["retaining_pin"]
    return bool(PIN_Y - p.pos[1] >= PULL_MIN and p.resting())


def _tag_on_pad(objects):
    """Which pad the tag is filed on, or None."""
    tag = objects["report_tag"]
    if not tag.resting():
        return None
    for mode, pad in PADS.items():
        s = objects[pad.name]
        if abs(tag.pos[0] - s.pos[0]) < REPORT_ON and abs(tag.pos[1] - s.pos[1]) < REPORT_ON:
            return mode
    return None


def score(objects):
    """0.5 for the retaining pin pulled at least 2 cm out along its rails and at rest;
    +0.5 for the tag filed on the pad matching the gauge's actual state. 1.0 for the complete
    check. Do-nothing: 0."""
    pulled = 0.5 if _pin_pulled(objects) else 0.0
    filed = 0.5 if _tag_on_pad(objects) == FAULT_MODE else 0.0
    import csv, os as _os
    if _os.path.exists("/tmp/ext_dbg_on"):
        p = objects["retaining_pin"]
        t = objects["report_tag"]
        row = [round(p.pos[0], 4), round(p.pos[1], 4), round(p.pos[2], 4), p.resting(),
               round(t.pos[0], 4), round(t.pos[1], 4), round(t.pos[2], 4), t.resting(), pulled, filed]
        with open(f"/tmp/ext_{_os.getpid()}.csv", "a") as f:
            csv.writer(f).writerow(row)
    return pulled + filed


def success(objects):
    return score(objects) >= 1.0


SCORE_OVER = "best"     # a pulled pin stays pulled; the filed tag stays filed


def demo():
    """The right arm descends on the pin from above (a horizontal approach would ram it with the
    wrist camera), closes on it, draws it out the front in slow stages, releases - then carries the
    tag to the pad matching the gauge's state."""
    open_wide, grip = -0.047, -0.0115   # wide descent (no drag), match-the-pin squeeze to draw
    right = [
        Waypoint(1.4, (BX, PIN_Y, PIN_Z + 0.09), open_wide, "hover above the pin", yaw=0.0),
        Waypoint(1.0, (BX, PIN_Y, PIN_Z), open_wide, "descend over the pin", yaw=0.0),
        Waypoint(0.8, (BX, PIN_Y, PIN_Z), grip, "close on the pin", yaw=0.0),
        Waypoint(1.4, (BX, PIN_Y - 0.025, PIN_Z), grip, "draw the pin out 1/2", yaw=0.0),
        Waypoint(1.4, (BX, PIN_Y - 0.05, PIN_Z), grip, "draw the pin out 2/2", yaw=0.0),
        Waypoint(0.6, (BX, PIN_Y - 0.05, PIN_Z), open_wide, "release the pin", yaw=0.0),
        Waypoint(1.2, (BX, PIN_Y - 0.10, PIN_Z), open_wide, "back off along the bracket", yaw=0.0),
        Waypoint(1.2, (BX, PIN_Y - 0.10, PIN_Z + 0.10), open_wide, "retreat from the bracket", yaw=0.0),
    ]
    # file the tag on the matching pad
    pad = PADS[FAULT_MODE]
    place = (pad.pos[0], pad.pos[1], TIPS_ON_TABLE + pad.size[2] + TAG.size[2] / 2 + 0.004)
    right += pick_place_waypoints((TAG.pos[0], TAG.pos[1], TAG.pos[2]), place,
                                  travel_z=TIPS_ON_TABLE + 0.14, speed=0.8)
    right += [Waypoint(1.0, (TAG.pos[0], TAG.pos[1] - 0.10, TIPS_ON_TABLE + 0.16), -0.040, "back off")]
    return {
        "right": right,
        "left": [Waypoint(sum(w.duration for w in right), (0.20, 0.20, TOP + 0.08), -0.040, "parked")],
    }
