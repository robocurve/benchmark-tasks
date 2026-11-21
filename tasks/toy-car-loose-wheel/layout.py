"""Every size, mass, friction and position of the toy-car-loose-wheel scene, with its source. The task file and the
scorer both read from here. Frame: x forward from the arm bases, y left, z up, metres; positions on the table are from
the midpoint between the two arm bases.

Sources (opened 2026-11-20):
- Wheels: Woodpeckers "1-1/4" Treaded Wheel", https://woodpeckerscrafts.com/products/1-1-4-treaded-wheel: 1-1/4"
  diameter, 1/4" hole, "1/2" thick wheels are made of premium hardwood" (the page names no species).
- Axle pegs: Woodpeckers "1-1/4" Axle", https://woodpeckerscrafts.com/products/1-1-4-axle: length 1-1/4", head diameter
  3/8", tenon diameter 3/16", birch; it "fits smoothly inside the 1/4" hole" of their wheels and can be sanded "down to
  size". The peg is glued into the car body and the
  wheel turns on it ("glue the peg into the axle hole in the toy body rather than into the wheel", American Woodcrafters
  Supply, https://www.americanwoodcrafterssupply.com/catalog/wooden-wheels-wooden-toy-parts).
- Car body: a small beech toy car of the common size, 10 cm long (Ostheimer "Car small natural", beech, 10 x 5 x 6 cm,
  https://www.waldorfshop.eu/en/wooden-toys/wooden-car); our body is 100 x 38 x 38 mm with four pegged wheels.
- Toy basket: IKEA TROFAST storage box, light green, 16 1/2 x 11 3/4 x 4 in (42 x 30 x 10 cm).
- Repair box: IKEA TROFAST storage box, grey, 7 3/4 x 11 3/4 x 4 in (20 x 30 x 10 cm).
- Pull a secure wheel must survive: 16 CFR 1500.51 (10 lbf, children 18 months or less) and 1500.52 (15 lbf, 18 to 36
  months), https://www.law.cornell.edu/cfr/text/16/1500.51.
Estimates (no page gives them): the body's shape and mass, the loose peg's sliding friction (3 N; a real trial measures
it on the real peg with a 500 g spring balance), wall thicknesses and frictions.
"""

from __future__ import annotations

TOP = 0.7475  # the station table's top (yam_arena)

# -- one toy car (frame: origin on the table under the middle of the body, x toward the front, y left, z up) -----------
WHEEL_R, WHEEL_W = 0.015875, 0.0127  # 1-1/4 in across, 1/2 in thick (birch)
AXLE_X = 0.032  # front and rear axle pegs at x = +-32 mm (64 mm wheelbase)
BODY_L, BODY_W = 0.100, 0.038  # the beech body: 100 mm long, 38 mm wide
BODY_Z = (0.008, 0.030)  # the lower body, 8 to 30 mm above the table (the pegs at 15.9 mm go into it)
CAB_L, CAB_W, CAB_Z, CAB_X = 0.050, 0.034, (0.030, 0.046), -0.008  # the cab on top
SPACER = 0.0015  # gap between the body's side and the wheel (a washer's thickness)
HEAD_R, HEAD_T = 0.00476, 0.003  # the peg's round head outside the wheel: 3/8 in across
PEG_R, PEG_IN = 0.00238, 0.0145  # the peg's tenon: 3/16 in (4.8 mm), 14.5 mm of it inside the body
WHEEL_Y = BODY_W / 2 + SPACER + WHEEL_W / 2  # the wheel's middle plane: 26.9 mm from the car's middle
CAR_W = 2 * (WHEEL_Y + WHEEL_W / 2 + HEAD_T)  # 72.6 mm over the peg heads
CAR_H = CAB_Z[1]
BODY_MASS = 0.075  # beech, about 50 cm^3 at 0.72 g/cm^3, shaped and drilled (estimate)
WHEEL_MASS = 0.0075  # one birch wheel (10 cm^3 at 0.65 g/cm^3) and its peg
CAR_MASS = BODY_MASS + 4 * WHEEL_MASS  # 105 g
CAR_FRICTION = (0.45, 0.4)  # oiled wood on a laminate table and on the boxes' plastic (estimate)

# -- the loose wheel (one wheel of the defective car: its peg pulled and refitted dry) ---------------------------------
LOOSE_FRICTION_N = 3.0  # the dry peg slides in its hole at about 3 N (the idea's figure; estimate)
LOOSE_DAMPING = 3000.0  # N s/m below that force (it creeps 0.3 mm/s under a 1 N touch)
LOOSE_STOP = 0.012  # the simulator stops the peg 12 mm out (2.5 mm still in the hole); a real peg can come out
SECURE_PULL_N = 66.7  # 15 lbf: the pull a glued wheel must survive (16 CFR 1500.52, 18 to 36 months)
SLID_MM = 4.0  # the demo calls a wheel loose when it slid 4 mm or more under its tug
FOUND_MM = 4.0  # scoring: car 1's loose wheel must stand at least 4 mm out at the end (the proof it was tested)

# -- where things stand ------------------------------------------------------------------------------------------------
CAR_X = 0.27
SLOTS_Y = (0.13, 0.0, -0.13)  # car 1 (left), car 2 (middle), car 3 (right), each pointing +x
BASKET_XY = (0.55, -0.12)  # the green toy basket: 30 cm along x, 42 cm along y
BASKET_L, BASKET_W, BASKET_H = 0.42, 0.30, 0.10  # outer: along y, along x, height
BOX_XY = (0.55, 0.22)  # the grey repair box: 30 cm along x, 20 cm along y
BOX_L, BOX_W, BOX_H = 0.20, 0.30, 0.10
WALL, FLOOR = 0.003, 0.003  # polypropylene walls and floor (estimate)
BASKET_MASS, BOX_MASS = 0.62, 0.30  # estimates
BASKET_SPOTS = ((0.52, -0.21), (0.52, -0.05))  # where the demo sets the two good cars
BOX_SPOT = (0.52, 0.22)
# the repair box's paper sign "STAFF / REPAIR" (visual only): an A5 card (20 x 14 cm) taped inside the back wall,
# 10 cm of it above the rim, leaning back 15 degrees; red letters 35 mm high, 5 mm strokes
SIGN_W, SIGN_H, SIGN_BOTTOM, SIGN_LEAN_DEG = 0.20, 0.14, 0.06, 15.0
LETTER_H, LETTER_W, STROKE = 0.035, 0.022, 0.005

# -- the demo's grasps --------------------------------------------------------------------------------------------------
TUG_TILT_DEG = 25.0  # each hand comes down tilted 25 degrees outward over its wheel
TUG_PULL = 0.010  # each hand pulls its wheel 10 mm outward along the axle
TCP_ABOVE_AXLE = 0.004  # the pinch's TCP 4 mm above the axle (back along the approach)
PICK_TCP_Z = 0.025  # straight-down body grip: TCP 25 mm above the table, fingertips at 12 mm
LIFT_Z = 0.17  # carry height of the TCP above the table (the car's underside clears the 10 cm walls)
DROP_Z = 0.045  # set-down height of the TCP above the box floor
