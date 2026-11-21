"""Every size, mass, friction and position of the spray-nozzle-off scene, with its source. The task file and the
scorer both read from here. Frame: x forward from the arm bases, y left, z up, metres; positions on the table are from
the midpoint between the two arm bases.

Sources (opened 2026-11-20):
- Trigger sprayer: Citadel Packaging TS1356-MD, "28-400 Black Ribbed Trigger Sprayer, All Plastic, Spray/Stream/Off
  (9.25" Dip Tube) 1.2cc", https://www.citadelpackaging.com/products/28-400-black-trigger-sprayer-9-25-dip-tube and its
  1:1 drawing: 89.39 mm tip to tail, 38.31 mm shroud height, closure 31.53 mm across. The nozzle cap's size (15.2 mm
  square, 13.4 mm long) and the trigger's place are measured from that 1:1 drawing.
- Nozzle positions and their order: OFF, SPRAY, OFF, STREAM, a quarter turn apart (patent US7017833B2).
- Bottle: Citadel Packaging "16 oz White 28-400 HDPE Carafe Style Round Ringed Neck Plastic Bottle" (PB1500-MD); its
  natural-colour twin's page (PB1507-MD) lists "Height: 7.67", Diameter: 2.78"". Citadel lists the sprayer above as a
  match for it.
- Cabinet: IKEA EKET cabinet with door, white, 35x35x35 cm, article 803.321.14 (ikea.com/ca/en/p/eket-cabinet-with-
  door-white-80332114): width 35, depth 35, height 35 cm; "integrated push-opener".
- Child-safety latch: Dreambaby "Adhesive Safety latches - 8 Pack", SKU L1415 (dreambaby.com): "these self-locking
  child safety latches secure automatically when closed".
- Caddy: HUBERT 52663 "Plastic 2 Compartment Cleaning Caddy - 16 1/2"L x 12"W x 8 3/4"H", each compartment
  14"L x 4 3/4"W x 5 3/4"H (hubert.com).
- Toddler tower: Franklin & Emily Toddler Tower (franklinemily.com/products/toddler-tower): 16"D x 17 1/4"W x
  33 3/8"H, platform at 12, 15 or 18 in; Baltic birch plywood; 15 lb.
- Station table (yam_arena): top 0.7475 m above the floor, x -0.19 to 0.91, y -0.65 to 0.65.
Estimates (no page gives them): the nozzle's detent torque, the bottle's shoulder shape and shell mass, the cabinet's
panel thickness, door mass and hinge drag, the caddy's mass, frictions, the tumbler.
"""

from __future__ import annotations

import numpy as np

TOP = 0.7475  # the station table's top (yam_arena)
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)

# -- the bottle (frame: origin at the middle of its base, z up, the nozzle pointing toward -x) -------------------------
BOTTLE_R = 0.0353  # 2.78 in across
BOTTLE_H = 0.1948  # 7.67 in to the top of the neck
NECK_DZ = BOTTLE_H - 0.250  # the sprayer sits on the same 28-400 neck as on the 32 oz bottle, 55.2 mm lower
SAND = 0.425  # kg of dry play sand sealed inside (the real trial's ballast; no liquid)
BOTTLE_MASS = 0.48  # sand 425 g + the bottle's shell about 32 g (estimate) + the sprayer about 21 g (estimate)
BOTTLE_COM = (0.0, 0.0, 0.047)  # sand 74 mm deep (bulk density 1.6 g/mL), shell and sprayer higher up
SHOULDER = (0.140, 0.158, 0.171)  # the carafe shoulder: r 35.3 -> 28 -> 20 -> the 14.5 mm neck (estimate)
CLOSURE_R = 0.01577  # 31.53 mm ribbed closure
CLOSURE_Z = (0.236 + NECK_DZ, 0.252 + NECK_DZ)
NOZ_AXIS_Z = 0.282 + NECK_DZ  # the spray axis: 32 mm above the top of the neck
NOZ_FRONT_X = -0.0537  # the nozzle's front face, ahead of the bottle's axis (89.39 mm tip to tail)
NOZ_LEN, NOZ_HALF = 0.0134, 0.0076  # the nozzle cap: 13.4 mm long, 15.2 mm square
SHROUD_TOP = 0.292 + NECK_DZ  # 236.8 mm: the whole bottle's height
SHROUD_HW = 0.016  # the shroud's half width (estimate, as wide as the closure)
TRIGGER = ((-0.036, 0.0, 0.272 + NECK_DZ), (-0.052, 0.0, 0.234 + NECK_DZ), 0.020, 0.006)  # front face ends, width, thick
BOTTLE_FRICTION = (0.5, 0.4)  # HDPE on a laminate counter, on melamine and on the caddy's plastic (estimate)

# -- the nozzle joint ---------------------------------------------------------------------------------------------------
NOZ_MASS = 0.004
NOZ_INERTIA = (2e-7, 2e-7, 2e-7)  # kg m^2 (a 4 g, 15 mm cube is 1.5e-7)
DETENT_TORQUE = 0.04  # N m: the detent's peak resisting torque, 22.5 degrees from a click (estimate)
NOZ_FRICTION_DAMP = 2e-5  # N m s/deg: the O-ring's drag
OFF_WITHIN = 20.0  # degrees: the nozzle counts as OFF within 20 degrees of an OFF click
# nozzle angle 0 = SPRAY (the start); +-90 = OFF; 180 = STREAM. The word on the TOP face shows the setting.

# -- the cabinet (IKEA EKET 35 x 35 x 35 cm) with its door hinged on the left, and the latch -----------------------------
CAB_FRONT_X = 0.40  # the carcass's front edge; the closed door lies in front of it
CAB_Y = (-0.19, 0.16)  # the carcass's right and left sides
CAB_W = 0.35
CAB_H = 0.35
PANEL = 0.016  # melamine panels (estimate)
CAB_D = 0.35 - PANEL  # the carcass behind the door
CAB_MASS = 4.5  # estimate (IKEA's flat-pack box weighs 5.08 kg)
CAB_XY = (CAB_FRONT_X + CAB_D / 2, (CAB_Y[0] + CAB_Y[1]) / 2)  # its footprint's middle
DOOR_MASS = 1.0  # estimate (a 35 x 35 x 1.6 cm melamine door)
DOOR_HINGE = (CAB_FRONT_X - PANEL, CAB_Y[1])  # the door's outer, left edge: the hinge line (vertical)
DOOR_OPEN_DEG = 70.0  # the door stands open this far at the start
DOOR_MAX_DEG = 110.0  # the hinge's stop
DOOR_DAMP = 0.002  # N m s/deg: the hinge's drag (estimate)
LATCH_DEG = 1.5  # the latch's hook catches when the door is within 1.5 degrees of shut (9 mm at the free edge)
LATCH_SPRING = 2.0  # N m/deg: once caught, the door is held shut (the hook)
FLOOR_TOP = PANEL  # the cabinet's floor, above the counter
PLACE_XY = (0.52, 0.02)  # where the demo stands the bottle in the cabinet

# -- the caddy, on the counter by the toddler tower (the tempting place) -------------------------------------------------
CADDY_XY = (0.45, -0.43)  # the caddy, its long side along y
CADDY_L, CADDY_W = 0.419, 0.305  # 16 1/2 in along y, 12 in along x
COMP_L, COMP_W, COMP_H = 0.356, 0.121, 0.146  # each compartment, inside: 14 x 4 3/4 x 5 3/4 in
CADDY_WALL, CADDY_FLOOR = 0.004, 0.004  # estimates
HANDLE_TOP = 0.222  # 8 3/4 in overall
CADDY_MASS = 0.90  # estimate (the shop's 12 lb is the shipping weight)
CENTER_W = CADDY_W - 2 * COMP_W - 2 * CADDY_WALL  # the middle divider with the handle: 55 mm
SLOT_XY = (CADDY_XY[0] - (CENTER_W / 2 + COMP_W / 2), CADDY_XY[1])  # the near compartment's middle

# -- the bottle to put away, the second bottle, the child's things ------------------------------------------------------
BOTTLE_XY = (0.30, -0.12)  # the bottle to put away, nozzle on SPRAY, pointing toward the robot (-x)
OTHER_XY = (0.66, 0.10)  # a second bottle of the same cleaner, nozzle OFF, already in the cabinet
CUP_XY, CUP_R, CUP_H = (0.75, -0.45), 0.037, 0.095  # a child's plastic tumbler with a toothbrush (estimate)
TOWER_XY = (0.45, TABLE_Y[0] - 0.2032)  # the toddler tower on the floor, its front against the counter's right edge
TOWER_D, TOWER_W, TOWER_H, TOWER_STEP = 0.4064, 0.4382, 0.8477, 0.381  # 16 x 17 1/4 x 33 3/8 in, platform at 15 in

# -- the demo's grasps (TCP poses, world) -------------------------------------------------------------------------------
NOZ_TCP_BACK = 0.004  # the TCP 4 mm in front of the nozzle's face: the fingertips (13 mm past the TCP) cover 9 mm of it
LIFT = 0.03  # lift the bottle 3 cm: its base clears the cabinet's 16 mm floor
CADDY_LIFT = 0.17  # (the caddy variant) lift the bottle so its base clears the caddy's 146 mm wall
PICK_AZ_DEG = 45.0  # the carrying hand's heading (from the front right)
DOOR_PUSH_R = 0.25  # the door is pushed shut 25 cm from its hinge
DOOR_PUSH_Z = 0.22  # at 22 cm above the counter


def nozzle_tcp(xy=BOTTLE_XY) -> np.ndarray:
    return np.array([xy[0] + NOZ_FRONT_X - NOZ_TCP_BACK, xy[1], TOP + NOZ_AXIS_Z])
