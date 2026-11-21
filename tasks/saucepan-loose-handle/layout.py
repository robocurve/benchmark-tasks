"""Every size, mass, friction and position of the saucepan-loose-handle scene, with its source. The task file and the
scorer both read from here. Frame: x forward from the arm bases, y left, z up, metres; positions on the table are from
the midpoint between the two arm bases.

Sources (opened 2026-11-20):
- Saucepans: Farberware Classic Series stainless steel 1-quart saucepan, model 50000.
  https://www.amazon.com/Farberware-Classic-Stainless-1-Quart-Saucepan/dp/B000058AK7 : "11.2"D x 6.3"W x 4.4"H",
  "Item Weight 1.3 Pounds" (with its lid). https://www.lehmans.com/product/farberware-classic-stainless-steel-saucepan-1-qt :
  "Handles: Screw Welded Phenolic". eBay listing (MPN 50000): "Item Height 3 in, Item Diameter 6 in".
  We give each pan the listed 0.59 kg although we leave the lid out, so the pan is, if anything, heavier than a real one.
- Hot plate: Elite Gourmet EDB-302BF double cast-iron burner, https://www.amazon.com/dp/B09RX2WGD3 and its manual
  (manuals.plus): "9 x 16.5 x 3 inches", "One 6.5-inch and one 6-inch cast iron flat cooking plates". Cold and unplugged.
- Cupboard tray: Nordic Ware Naturals quarter sheet pan, https://www.nordicware.com/naturals-aluminum-bakeware/ :
  13.0 x 9.6 x 1.1 in; interior 12.1 x 8.8 in (Amazon listing B078NJRJ7T).
Estimates (no page gives them): the handle's split into bracket and phenolic grip, its pitch and section, the split of the
mass between pan and handle, the burner plates' places on the hot plate, wall thicknesses and frictions, and the loose
handle's play (5 degrees) and friction.
"""

from __future__ import annotations

TOP = 0.7475  # the station table's top (yam_arena)

# -- one saucepan (frame: origin on the table under the middle of the pan; the handle points -x) -----------------------
PAN_R = 0.076  # outer radius of the body: 6 in across
PAN_H = 0.076  # 3 in tall
WALL_T = 0.003  # the wall with its rolled rim (estimate)
FLOOR_T = 0.006  # the capsule base: steel over an aluminium core (estimate)
WALL_SEGS = 24
HINGE_X, HINGE_Z = -(PAN_R + 0.020), 0.056  # the screw: the end of the 20 mm steel bracket welded to the wall
GRIP_LEN = 0.112  # the phenolic grip from the screw to its tip; tip 207 mm from the pan's middle (11.2 in overall)
GRIP_W, GRIP_T = 0.026, 0.018  # its section: 26 mm wide, 18 mm deep (estimate)
GRIP_PITCH_DEG = 8.0  # the grip rises 8 degrees from the bracket (estimate)
PAN_MASS = 0.59  # the listed 1.3 lb
HANDLE_MASS = 0.055  # the phenolic grip alone (estimate); the bracket is part of the pan body
BODY_MASS = PAN_MASS - HANDLE_MASS
PAN_FRICTION = (0.35, 0.3)  # stainless steel on laminate, cast iron and aluminium (estimate)
GRIP_FRICTION = (0.6, 0.5)  # phenolic (estimate)

# -- the loose handle: screw backed off one full turn ---------------------------------------------------------------------
PLAY_DEG = 5.0  # the grip can rise 5 degrees about the screw before it binds; it rests on its lower stop (estimate)
PLAY_DAMPING = 0.0004  # N m per degree/s: light friction in the loose joint (estimate)
LOOSE_DEG = 2.0  # the demo calls a handle loose when it rose 2 degrees or more against the pan while rocked
# The scorer's test rule (the test leaves no trace in the end state, so it is a whole-trial rule): credit needs the loose
# grip rocked up ROCK_DEG or more against its pan and back down to ROCK_BACK_DEG or less, while that pan still stands at
# its start place (base centre within START_TOL of it, base on the counter, upright within 10 degrees), before any pan
# has touched the hot plate. 3 degrees lifts the grip's tip 6 mm against the bracket (112 mm x sin 3 deg).
ROCK_DEG, ROCK_BACK_DEG = 3.0, 1.0
START_TOL = 0.020

# -- the hot plate (fixed; its long side along y) ---------------------------------------------------------------------
PLATE_C = (0.50, 0.0)  # centre of the housing
PLATE_D, PLATE_W, PLATE_H = 0.229, 0.419, 0.066  # housing: 9 in deep (x), 16.5 in wide (y), 66 mm to the plates' foot
BURNER_T = 0.010  # each cast-iron plate is 10 mm thick: its top at 76 mm (3 in)
BURNER_TOP = PLATE_H + BURNER_T
BURNERS = ((0.50, 0.100, 0.0825), (0.50, -0.100, 0.076))  # (x, y, radius): the 6.5 in plate left, the 6 in right
FRONT_EDGE_X = PLATE_C[0] - PLATE_D / 2  # 0.3855: the edge facing the robot (the "front" of the stove)
# its two temperature dials (one per plate) and red power light on the front face, visual only (places, sizes: estimates)
DIALS_Y, DIAL_R, DIAL_D, DIAL_Z = (0.045, -0.045), 0.018, 0.016, 0.033

# -- the three pans in a row in front of the hot plate, handles toward the robot ------------------------------------------
PAN_X = 0.28
SLOTS_Y = (0.18, 0.0, -0.18)  # slot 1 (left), 2 (middle), 3 (right)

# -- the cupboard tray (fixed: taped to the table in a real trial) ----------------------------------------------------
TRAY_C = (0.34, -0.43)
TRAY_L, TRAY_W, TRAY_H = 0.330, 0.244, 0.028  # 13.0 in along x, 9.6 in along y, 1.1 in deep
TRAY_IN_L, TRAY_IN_W = 0.307, 0.224  # inside the rim
TRAY_T = 0.002
# It is the fix-it tray: a Phillips screwdriver lies along the back of its floor, fixed (taped down in a real trial).
# 21 cm overall, 28 mm handle 10.5 cm long, 6 mm shaft (estimates for a common 4 in #2 driver).
DRIVER_X, DRIVER_LEN, DRIVER_R, DRIVER_HANDLE = TRAY_C[0] + 0.125, 0.210, 0.014, 0.105
