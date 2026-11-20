"""Every size, mass, friction and position of the spray-nozzle-off scene, with its source. The task file and the
scorer both read from here. Frame: x forward from the arm bases, y left, z up, metres; positions on the table are from
the midpoint between the two arm bases.

Sources (opened 2026-11-20):
- Trigger sprayer: Citadel Packaging TS1356-MD, "28-400 Black Ribbed Trigger Sprayer, All Plastic, Spray/Stream/Off
  (9.25" Dip Tube) 1.2cc", https://www.citadelpackaging.com/products/28-400-black-trigger-sprayer-9-25-dip-tube and its
  1:1 drawing (TS1356-MD_28-400_Black_Trigger_Sprayer_9.25inch_Dip_Tube.pdf): 89.39 mm tip to tail, 38.31 mm shroud
  height, closure 31.53 mm across. The nozzle cap's size (15.2 mm square, 13.4 mm long) and the trigger's place are
  measured from that 1:1 drawing.
- Nozzle positions and their order: OFF, SPRAY, OFF, STREAM, a quarter turn apart (patent US7017833B2, "Trigger
  sprayer spray, off, stream, off indexing nozzle assembly": "rotated from the off position directly to either of the
  spray or stream positions by rotating the cap one-quarter turn in opposite directions").
- Bottle: Citadel Packaging "32 oz White 28-400 HDPE Carafe Style Round Ringed Neck Plastic Bottle": height 9.84 in,
  diameter 3.45 in.
- Caddy: HUBERT 52663 "Plastic 2 Compartment Cleaning Caddy - 16 1/2"L x 12"W x 8 3/4"H", each side compartment
  14"L x 4 3/4"W x 5 3/4"H (hubert.com).
Estimates (no page gives them): the nozzle's detent torque, the shoulder shape of the bottle, wall and floor
thicknesses, the caddy's mass, frictions.
"""

from __future__ import annotations

import numpy as np

TOP = 0.7475  # the station table's top (yam_arena)

# -- the bottle (frame: origin at the middle of its base, z up, the nozzle pointing toward -x) -------------------------
BOTTLE_R = 0.0438  # 3.45 in across
BOTTLE_H = 0.250  # 9.84 in to the top of the neck
BOTTLE_MASS = 0.60  # with 540 mL of water (the real trial's fill)
BOTTLE_COM = (0.0, 0.0, 0.062)  # water 90 mm deep in the base; bottle 60 g, sprayer 21 g higher up
CLOSURE_R, CLOSURE_Z = 0.01577, (0.236, 0.252)  # 31.53 mm ribbed closure
NOZ_AXIS_Z = 0.282  # the spray axis: 32 mm above the top of the neck
NOZ_FRONT_X = -0.0537  # the nozzle's front face, ahead of the bottle's axis (89.39 mm tip to tail)
NOZ_LEN, NOZ_HALF = 0.0134, 0.0076  # the nozzle cap: 13.4 mm long, 15.2 mm square
SHROUD_TOP = 0.292
SHROUD_HW = 0.016  # the shroud's half width (estimate, as wide as the closure)
TRIGGER = ((-0.036, 0.0, 0.272), (-0.052, 0.0, 0.234), 0.020, 0.006)  # its front face: top end, bottom end, width, thickness
BOTTLE_FRICTION = (0.5, 0.4)  # HDPE on a laminate counter and on the caddy's plastic (estimate)

# -- the nozzle joint ---------------------------------------------------------------------------------------------------
NOZ_MASS = 0.004
NOZ_INERTIA = (2e-7, 2e-7, 2e-7)  # kg m^2 (a 4 g, 15 mm cube is 1.5e-7)
DETENT_TORQUE = 0.04  # N m: the detent's peak resisting torque, 22.5 degrees from a click (estimate)
NOZ_FRICTION_DAMP = 2e-5  # N m s/deg: the O-ring's drag
OFF_WITHIN = 20.0  # degrees: the nozzle counts as OFF within 20 degrees of an OFF click
# nozzle angle 0 = SPRAY (the start); +-90 = OFF; 180 = STREAM. The word on the TOP face shows the setting.

# -- where things stand (fixed layout) --------------------------------------------------------------------------------
BOTTLE_XY = (0.30, 0.08)  # the bottle to put away, nozzle on SPRAY, pointing toward the robot (-x)
CADDY_XY = (0.40, -0.20)  # the caddy, its long side along y
CADDY_L, CADDY_W = 0.419, 0.305  # 16 1/2 in along y, 12 in along x
COMP_L, COMP_W, COMP_H = 0.356, 0.121, 0.146  # each compartment, inside: 14 x 4 3/4 x 5 3/4 in
CADDY_WALL, CADDY_FLOOR = 0.004, 0.004  # estimates
HANDLE_TOP = 0.222  # 8 3/4 in overall
CADDY_MASS = 0.90  # estimate (the shop's 12 lb is the shipping weight)
CENTER_W = CADDY_W - 2 * COMP_W - 2 * CADDY_WALL  # the middle divider with the handle: 55 mm
SLOT_XY = (CADDY_XY[0] - (CENTER_W / 2 + COMP_W / 2), CADDY_XY[1])  # the free (near) compartment's middle
FULL_XY = (CADDY_XY[0] + (CENTER_W / 2 + COMP_W / 2), CADDY_XY[1])  # the far compartment: a second bottle, nozzle OFF
CUP_XY, CUP_R, CUP_H = (0.47, 0.22), 0.037, 0.095  # a child's plastic tumbler with a toothbrush (estimate)

# -- the demo's grasps (TCP poses, world) -------------------------------------------------------------------------------
NOZ_TCP_BACK = 0.004  # the TCP 4 mm in front of the nozzle's face: the fingertips (13 mm past the TCP) cover 9 mm of it
LIFT = 0.17  # lift the bottle so its base clears the caddy's 146 mm wall


def nozzle_tcp(xy=BOTTLE_XY) -> np.ndarray:
    return np.array([xy[0] + NOZ_FRONT_X - NOZ_TCP_BACK, xy[1], TOP + NOZ_AXIS_Z])


def neck_tcp(xy=BOTTLE_XY) -> np.ndarray:
    """The right hand closes across the closure from the -y side (fingers along +y, closing along x)."""
    return np.array([xy[0], xy[1] - 0.004, TOP + sum(CLOSURE_Z) / 2])
