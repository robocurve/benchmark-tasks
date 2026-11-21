"""Every size, mass, friction and place of the canning-jar scene, in one module. The task file and the scorer read it.
Frame: x forward from the arm bases, y left, z up, metres; the table top is z = TOP. Only one thing changes between
seeds: which of the three jars has the unsealed lid (``unsealed_index``). Everything else is at one stated place.

Sources (opened 2026-11-20):
- Jar: Ball 60000ZFP 8 oz half-pint regular-mouth glass canning jar (WebstaurantStore listing): "Height: 3 15/16
  Inches", "Top Diameter: 2 3/4 Inches", "Capacity: 8 oz.", "Shipping Weight: 5.5 lb." for "12/Case".
  https://www.webstaurantstore.com/ball-60000zfp-8-oz-half-pint-regular-mouth-glass-canning-jar-with-silver-metal-lid-and-band-case/52760000.html
  The body width (72 mm), the lid's flexing centre (48 mm) and the filled mass (0.45 kg: about 0.21 kg of glass and
  lid, from the case weight, plus 8 oz of sauce) are estimates.
- Lid test: National Center for Home Food Preservation, "Cooling Jars and Testing Jar Seals": "Press the middle of the
  lid with a finger or thumb. If the lid springs up when you release your finger, the lid is unsealed."
  https://nchfp.uga.edu/how/can/general-information/cooling-jars-and-testing-jar-seals
  No source gives the travel or force of an unsealed lid's centre: 1.5 mm at about 5 N is an estimate.
- Mini fridge: Cooluli Classic 4 L, inner 5.3 x 5.7 x 8.3 in (135 x 145 x 210 mm), outer 8 x 10.2 x 11 in (maker's
  page and retailer listings). https://cooluli.com/products/classic-4-liter-compact-mini-fridge
  The wall thicknesses, the floor height (35 mm) and the door's 40 mm depth are estimates fitted to those sizes.
- Pantry shelf: a wooden display riser, 4 in high x 15 in wide x 10 in deep (102 x 381 x 254 mm), as Wayfair's
  "Wrentham Riser Wood Lg" listing gives it (seen in search results). Board thickness (12 mm) is an estimate. A
  6 1/4 in bamboo cabinet shelf (Lipper International) was tried first: its top is out of the arms' reach for a jar.
- Scales (one under each jar, the real trial's press indicator): Brecknell EPB-3000g digital pocket scale, "3000 g x
  0.1 g", "Pan Dimensions: 4.00" x 4.00" / 100 x 100 mm", "140 x 107 x 20 mm" overall (ScalesPlus
  listing). https://www.scalesplus.com/brecknell-epb-3000g-digital-pocket-scale-3000-g-x-0-1-g/
- Unsealed jars: NCHFP FAQ "May canned food be reprocessed if the lid does not seal?": "Due to loss of quality,
  reprocessing of jars is not recommended." ... "To preserve quality of food where jars failed to seal (and discovered
  within 24 hours of initial canning), refrigerate the jars and consume contents within one week."
  https://nchfp.uga.edu/faqs/general-canning/category/faq-canning  -> the fridge tray's label: USE IN 1 WEEK.
"""

from __future__ import annotations

TOP = 0.7475  # the station's table top (yam_arena.task_file.TABLE_TOP["station"])

# -- the three jars (rigid glass, filled with sauce); origin at the centre of the base, z up -----------------------------
JAR_R = 0.036  # body, 72 mm across (estimate between the listed 2 5/16 in and 2 3/4 in)
JAR_BODY_TOP = 0.082  # the straight body ends at the shoulder
NECK_R = 0.0335  # threaded neck under the lid
NECK_TOP = 0.0975  # the glass rim the lid sits on
LID_R = 0.035  # flat lid, 70 mm across (regular mouth, listed 2 3/4 in)
LID_T = 0.0015
LID_TOP = NECK_TOP + LID_T  # 99 mm: the listed 3 15/16 in (100 mm) is with the band, which is off
SAUCE_TOP = 0.078  # the sauce level behind the glass (headspace above it, as canning guides require)
JAR_MASS = 0.45
JAR_COM = (0.0, 0.0, 0.040)
GLASS_MU = (0.55, 0.45)  # glass on wood and on the fridge's plastic floor (estimate)

# -- the lid's centre panel: a disc on a slider in every jar -------------------------------------------------------------
BUTTON_R = 0.024  # the panel inside the sealing ring, 48 mm across (estimate)
BUTTON_T = 0.0015
BUTTON_MASS = 0.004
TRAVEL = 0.0015  # an unsealed lid's centre gives 1.5 mm (estimate) ...
UNSEALED_K = 3000.0  # ... against a spring of 3,000 N/m ...
UNSEALED_PRELOAD = 0.0005  # ... held up against its stop with 1.5 N: 1.5 N to start it, 6 N at the bottom
UNSEALED_DAMP = 3.0
SEALED_K = 2.0e4  # a sealed lid barely gives: 1.5 N moves it 0.075 mm, 6 N 0.3 mm (stiff spring, so its force can be read)
SEALED_DAMP = 20.0

# -- where the jars stand at the start: a row across the table, 130 mm apart -----------------------------------------------
JAR_X = 0.300
JAR_YS = (0.130, 0.000, -0.130)  # start places 1 (left, from the robot), 2, 3; their stickers number the places only
N_JARS = 3

# -- a pocket scale under each jar (fixed in simulation): Brecknell EPB-3000g, 140 x 107 x 20 mm, pan 100 x 100 mm --------
SCALE_L, SCALE_W, SCALE_H = 0.140, 0.107, 0.020  # along x (display toward the robot), across (y), high
SCALE_PAN = 0.100  # the pan, centred under the jar; the 40 mm display strip is in front of it
JAR_Z0 = SCALE_H  # each jar's base starts on its scale's pan


def unsealed_index(seed) -> int:
    """Which jar (0, 1, 2 = jar 1, 2, 3) has the unsealed lid: the episode seed modulo 3 (CLI seeds 0, 1, 2 give jars
    2, 3, 1). Unseeded runs use jar 2."""
    return 1 if seed is None else int(seed) % 3


# -- the pantry shelf (fixed): a wooden riser, a top board on two side panels --------------------------------------------
SHELF_W, SHELF_D, SHELF_H = 0.381, 0.254, 0.102  # across (y), deep (x), high: 15 x 10 x 4 in
SHELF_T = 0.012
SHELF_XY = (0.330, 0.400)
SHELF_TOP = TOP + SHELF_H
SHELF_SPOTS = ((0.275, 0.440), (0.385, 0.372), (0.275, 0.305))  # 1st, 2nd, 3rd jar: far spots first, so no
# carried jar passes over one already standing
WOOD_MU = (0.55, 0.45)

# -- the mini fridge (fixed): door open; its opening faces the robot ------------------------------------------------------
FRIDGE_FRONT_X = 0.400  # the cabinet's front face (the door plane, door open)
FRIDGE_Y = -0.290
FRIDGE_OUT = (0.219, 0.203, 0.279)  # cabinet behind the door plane (x), width (y), height; with the door 259 mm deep
FRIDGE_IN = (0.145, 0.135, 0.210)  # inside depth, width, height (listed)
FRIDGE_FLOOR = 0.035  # inside floor above the table (estimate)
FRIDGE_FLOOR_TOP = TOP + FRIDGE_FLOOR
DOOR_T = 0.040
DOOR_OPEN_DEG = 110.0  # hinged on its right (-y) side, swung open to 110 degrees
FRIDGE_MU = (0.50, 0.40)
FRIDGE_SPOT = (FRIDGE_FRONT_X + 0.047, FRIDGE_Y)  # where the demo stands the jar: its front 11 mm inside the door plane
# the "use first" tray on the fridge floor (fixed): a flat plastic tray, 104 deep x 124 wide, 2 mm floor, 6 mm rim of 2 mm
# walls, its front 1 mm behind the door plane; a printed card hangs from its front lip over the fridge's front edge,
# as wide as the fridge, so the cameras can read it
TRAY_D, TRAY_W, TRAY_FLOOR, TRAY_RIM, TRAY_WALL = 0.104, 0.124, 0.002, 0.006, 0.002
TRAY_X0 = FRIDGE_FRONT_X + 0.001  # the tray's front edge
TRAY_FLOOR_TOP = FRIDGE_FLOOR_TOP + TRAY_FLOOR
TRAY_LABEL = "USE IN 1 WEEK"  # the NCHFP limit for an unsealed jar kept in the fridge ("within one week")
LABEL_W, LABEL_H, LABEL_LETTER = 0.203, 0.030, 0.018  # the card: 20.3 x 3.0 cm, 18 mm letters
# the same words on a card lying on the fridge's top (the arms hide the front edge from the top camera): TRAY over
# USE IN 1 WEEK, 18 mm letters, on a 20 x 7 cm card 7.5 to 14.5 cm behind the front face
TOP_CARD = (0.075, 0.145, 0.200)  # from x, to x (behind the front face), width

# -- scoring thresholds (the scoring text, number for number) ---------------------------------------------------------------
UNSEALED_TRAVEL = 0.0005  # the demo calls a lid unsealed when its centre went down more than 0.5 mm (sealed: 0.1)
UPRIGHT = 15.0  # standing upright: tilt under 15 degrees
FALL_TILT = 30.0  # tipped this far at any time: fallen
REST_M = 0.005  # base within 5 mm of the surface it stands on
OFF_TABLE = 0.020  # base more than 20 mm below the table top: fallen off
DROP_M = 0.020  # a jar no gripper touches that goes down more than 20 mm: dropped
TOUCH_M = 0.015  # a finger point within 15 mm of a jar's outline: touching it
HANDS_CLEAR = 0.030  # every finger point at least 3 cm from every jar at the end
POINTS = {"unsealed_fridge": 0.4, "sealed_shelf": 0.3}
# the press evidence (a whole-trial rule; the lid springs back, so the end state shows nothing)
PRESS_TRAVEL = 0.0008  # the unsealed lid's centre must go down at least 0.8 mm (the demo's press: 1.44 mm; sealed 0.22)
PRESS_R = 0.015  # a fingertip on the lid's centre: within 15 mm of the jar's axis ...
PRESS_GAP = 0.003  # ... and within 3 mm of the lid's top (above it or pressed into it)
PRESS_STEPS = 6  # ... for 6 control steps in a row (0.3 s)
PRESS_FORCE = 1.5  # ... pushing the lid centre down with at least 1.5 N (a rise of 153 g on the scale under the jar)
AT_START_XY = 0.010  # a jar still at its start place: centre within 10 mm of where it started ...
AT_START_Z = 0.005  # ... base within 5 mm of the counter, and upright (tilt under 15 degrees)
