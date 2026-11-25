"""Geometry and mechanism of the garden-hose quick-connect scene on the yam_arena station table, shared by
build_assets.py, task.py and demo.py. Pure Python, so the task file can import it before Isaac Sim starts.

World frame (yam_arena's): x forward from the arms, y to the robot's left, z up, metres; heights are above the table
top (TOP) unless they say "world".

A garden tap on a standpipe post has a brass quick-connect coupler on its spout, mouth toward the robot. The coupler has
a sprung collar and three locking pins. A green hose, clamped to the table off to the robot's right, ends in a brass
plug resting on the table. To connect: pull the collar back toward the tap and hold it, push the plug in until its
flange meets the mouth, then let the collar spring forward over the pins, which drop into the plug's groove.
"""

from __future__ import annotations

import math

TOP = 0.7475                            # yam_arena.task_file.TABLE_TOP["station"]

# -- the tap and its coupler -----------------------------------------------------------------------------------------
# Coupler frame (the asset's): origin at the centre of the mouth, x into the bore (toward the tap), z up.
MOUTH_X, MOUTH_Y, AXIS_Z = 0.32, -0.04, 0.11        # the mouth in the world
BORE_YAW = math.radians(-15)            # the bore runs into the tap along this heading (the spout angled to the right)
BORE_R = 0.0090                         # 18 mm bore (1.25 mm clearance round the 15.5 mm shaft)
LEAD_IN = ((0.002, 0.0130), (0.004, 0.0115), (0.006, 0.0100))   # the chamfered mouth: steps to the bore
BODY_R, BODY_LEN = 0.015, 0.040         # brass body, 30 mm across
PIN_X = 0.008                           # the locking pins, this far in from the mouth
PIN_ANGLES = (90.0, 210.0, 330.0)       # degrees about the bore axis, from +y toward +z (one on top)
PIN_IN = 0.0060                         # a pin's tip this far from the axis when down (locked)
PIN_TRAVEL = 0.0035                     # it can rise this far, to 9.5 mm: well clear of the 7.75 mm shaft
PIN_BALL = 0.002                        # the locking balls: 4 mm steel balls
BALL_FRICTION = 0.1                     # greased
PIN_K = 40.0                            # N/m, the light spring pressing each pin in
COLLAR_R_IN, COLLAR_R_OUT = 0.0165, 0.0200   # the collar: 40 mm across
COLLAR_X0, COLLAR_LEN = -0.003, 0.030   # it reaches 3 mm in front of the mouth at rest
COLLAR_TRAVEL = 0.009                   # it slides back toward the tap this far
COLLAR_K = 300.0                        # N/m; preloaded by COLLAR_PRELOAD: 1.2 N at rest, 3.9 N at full travel
COLLAR_PRELOAD = 0.004
RELEASE = 0.006                         # pulled back this far, the collar's relief is over the pins: they can rise
COLLAR_MASS, BODY_MASS, PIN_MASS = 0.04, 0.30, 0.002
POST_BACK = BODY_LEN + 0.045            # the standpipe post the tap stands on, this far behind the mouth along the bore
POST_SIZE = (0.05, 0.05)

# -- the hose and its plug -------------------------------------------------------------------------------------------
# Hose frame (the asset's): origin at the clamp, x along the hose as it leaves the clamp. Links alternate yaw (about z)
# and pitch (about y) joints at their ends; the plug is the last link, its frame at its flange face, x toward its nose.
ANCHOR_X, ANCHOR_Y = -0.04, -0.48
ANCHOR_HEADING = math.radians(160)
HOSE_LINKS, HOSE_LINK_L, HOSE_R = 16, 0.045, 0.0115   # 0.72 m of 23 mm hose
HOSE_LINK_MASS = 0.012
HOSE_BEND = math.radians(-20)            # the start: each yaw joint bent this much, so the hose lies in a gentle curve
HOSE_K, HOSE_DAMPING = 0.001, 0.002     # N m/rad, N m s/rad per joint: a soft, flexible hose
SHAFT_R, SHAFT_LEN = 0.00775, 0.026     # 15.5 mm shaft
GROOVE = (0.006, 0.012)                 # the groove, this far ahead of the flange face
GROOVE_R = 0.0055                       # its core, 11 mm
NOSE_LEN = 0.008                        # the nose: a cone to a point, so it lifts the balls
FLANGE_R, FLANGE_LEN = 0.012, 0.006
PLUG_BODY_R, PLUG_BODY_LEN = 0.011, 0.040     # the hex body the hand takes
BOOT_LEN = 0.022
PLUG_MASS = 0.10
PLUG_GRIP_X = -(FLANGE_LEN + 0.85 * PLUG_BODY_LEN)  # the hand takes the hex near its hose end, in the plug's frame

# -- checks ----------------------------------------------------------------------------------------------------------
SEATED = 0.003                          # the flange within 3 mm of the mouth
LOCKED_COLLAR = 0.0015                  # the collar back within 1.5 mm of rest
PIN_DOWN = 0.0008                       # each pin within 0.8 mm of down
ENTERED = 0.015                         # the plug's nose this far into the bore: past the locking balls (8 mm in)
LET_GO = 0.03                           # every finger pad at least this far from coupler, collar and plug
STILL = 0.003                           # the plug moved less than this over the last second: still
