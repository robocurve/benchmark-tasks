"""Every size, mass, friction and place of the spirit-burner scene, in one module. The task file and the scorer read
it. Frame: x forward from the arm bases, y left, z up, metres; the table top is z = TOP.

Sources (opened 2026-11-20):
- Spirit burner: American Scientific "Glass Alcohol Burner" (Avogadro Lab Supply listing): "A flint glass alcohol
  burner ... globe shaped base ... Also included ... the snuff cap", "Height 10.16 cm / 4.0"", "Width 8.25 cm /
  3.25"", "Capacity 150 ml", "Empty Weight 188 g / 6.6 oz". https://www.avogadro-lab-supply.com/products/glass-alcohol-lamp-burner
  The globe's profile, neck, wick holder, wick and cap sizes are estimates fitted to that height and width.
- Heat mat: The Lab Warehouse "MAT CERAMIC FIBRE 225MM SQUARE X APPROX 5MM THICK".
  https://www.thelabwarehouse.com/products/mat-ceramic-fibre-225mm-square-x-approx-5mm-thick
- Tray: Henley's IT300GY grey polypropylene tray, "12 x 10 x 2"". https://www.henleysmed.com/products/polypropylene-trays
- Matchbox: Swan Vestas safety matches box, 80 x 47 x 16 mm (Hackney Museum record of a box).
  https://museum-collection.hackney.gov.uk/object-1992-195 ; its mass is an estimate.
"""

from __future__ import annotations

TOP = 0.7475  # the station's table top (yam_arena.task_file.TABLE_TOP["station"])

# -- the spirit burner: one rigid glass body; origin at the centre of its base, z up its axis --------------------------
BODY_R = 0.04125  # the globe's widest band, 82.5 mm across (listed width)
GLOBE = ((0.032, 0.000, 0.008), (0.038, 0.008, 0.018), (BODY_R, 0.018, 0.044), (0.038, 0.044, 0.054),
         (0.031, 0.054, 0.062), (0.022, 0.062, 0.068))  # (radius, z from, z to): the globe in six bands
SEAT_Z = 0.068  # the flat ring round the neck's foot where the cap's rim comes to rest
NECK_R, NECK_TOP = 0.012, 0.078  # glass neck, 24 mm across
COLLAR_R, COLLAR_TOP = 0.011, 0.081  # wick holder seated in the neck
TUBE_R, TUBE_TOP = 0.003, 0.089  # its wick tube
WICK_R, WICK_TOP = 0.0025, 0.092  # cotton wick end
PROP_R, PROP_TOP = 0.0035, 0.098  # stand-in flame: a short rigid orange silicone tip on the wick end
BURNER_MASS = 0.188  # empty (listed)
BURNER_COM = (0.0, 0.0, 0.028)  # the glass is thickest at the base: the centre of mass sits low (estimate)
GLASS_MU = (0.55, 0.45)  # glass on ceramic fibre / on polypropylene (estimate)

# -- the snuff cap: a cup, open end down; origin at the centre of its rim ------------------------------------------
CAP_R, CAP_WALL, CAP_H, CAP_TOPT = 0.016, 0.0015, 0.034, 0.002  # 32 mm across, 34 mm tall
CAP_IN_R = CAP_R - CAP_WALL  # 14.5 mm: 2.5 mm round the 24 mm neck
CAP_MASS = 0.010
CAP_MU = (0.50, 0.40)
CAP_STAVES = 18
# seated, the cap's top is SEAT_Z + CAP_H = 102 mm above the burner's base (listed height 101.6 mm)

# -- the heat mat (fixed): ceramic-fibre board on the bench ---------------------------------------------------------
MAT_SIZE = (0.225, 0.225, 0.005)
MAT_XY = (0.350, -0.050)
MAT_TOP = TOP + MAT_SIZE[2]

# -- the burner and cap at the start -----------------------------------------------------------------------------------
BURNER_XY = MAT_XY  # standing at the mat's centre, inside the ring drawn on the mat
CAP_XY = (0.215, -0.160)  # standing open end down on the bench, just in front of the mat's front-right corner

# -- the grey tray (fixed): polypropylene, open top ------------------------------------------------------------------
TRAY_SIZE = (0.305, 0.254, 0.051)  # outside length (x), width (y), height: 12 x 10 x 2 in
TRAY_WALL, TRAY_FLOOR = 0.003, 0.003
TRAY_XY = (0.380, 0.235)
TRAY_FLOOR_TOP = TOP + TRAY_FLOOR
TRAY_IN = (TRAY_SIZE[0] / 2 - TRAY_WALL, TRAY_SIZE[1] / 2 - TRAY_WALL)  # inner half length, half width
TRAY_MU = (0.45, 0.40)

# -- the box of safety matches, flat in the tray's far half (its outline is drawn on the tray floor) ---------------
MATCH_SIZE = (0.047, 0.080, 0.016)  # x, y, z: lying flat, long side across the tray
MATCH_XY = (0.465, 0.235)
MATCH_MASS = 0.025
MATCH_MU = (0.50, 0.45)

# -- where the demo stands the burner: the tray's near half --------------------------------------------------------
DROP_XY = (0.305, 0.235)

# -- scoring thresholds (the scoring text, number for number) -----------------------------------------------------
SEATED_GAP = 0.003  # cap rim within 3 mm of the seat ...
SEATED_TILT = 10.0  # ... and its axis within 10 degrees of the burner's
RESTING_GAP = 0.010  # 3 to 10 mm above the seat, axis within 20 degrees: on the burner, not seated
RESTING_TILT = 20.0
MOVE_M = 0.010  # the burner's base centre this far (horizontally) from its start spot: moved
LIFT_M = 0.005  # or its base this far above its start height: lifted
FALL_TILT = 30.0  # tipped this far at any time: fallen
UPRIGHT = 10.0  # standing upright: tilt under 10 degrees
REST_M = 0.005  # base within 5 mm of the tray floor
FLAME_R = 0.010  # flame zone: within 10 mm of the wick axis ...
FLAME_ABOVE = 0.060  # ... from the wick holder's top to 60 mm above the wick end, while the cap is not seated
MATCH_MOVE = 0.005  # matchbox outside its drawn outline: moved 5 mm or turned 5 degrees
MATCH_TURN = 5.0
MATCH_TOUCH = 0.002  # burner closer than 2 mm to the matchbox: touching
HANDS_CLEAR = 0.030  # every finger point at least 3 cm from the burner and the cap at the end
POINTS = {"full": 1.0, "tray_off": 0.6, "capped": 0.4, "resting": 0.2}
