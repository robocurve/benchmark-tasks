"""Every size, mass, friction and position of the sparkling-cork scene, with its source. The task file and the scorer
both read from here. Frame: x forward from the arm bases, y left, z up, metres; positions on the table are from the
midpoint between the two arm bases.

Sources (opened 2026-11-20):
- Bottle: Hauser Packaging "Champagne 375ml" (hauserpack.com/catalog/beer/champagne-375/): "Crown 29mm" finish,
  500 g, 243.5 mm tall, 70.6 mm across. The same bottle is Saverglass Champenoise STANDARD 37.5 cl, reference 0725
  (saverglass.com/en/our-products/catalog-sparkling-wine/standard-375-cl-couronne-champ-29): 17.5 mm bore.
- Stopper: MoreBeer W460 "Champagne Stoppers - White Plastic" (morebeer.com/products/champagne-stoppers-white-plastic):
  "Height: 46.50mm, Head diameter: 30.50mm, Stem height: 30.50mm, Stem Diameter: 20mm"; the shop's answer on the page:
  "These fit in both 26mm and 29mm tops in champagne bottles". The ribbed 20 mm stem is squeezed into the 17.5 mm bore:
  the ribs' grip is the fit.
- Guidance: American Academy of Ophthalmology, https://www.aao.org/eye-health/tips-prevention/injuries-champagne:
  "hold the bottle at a 45 degree angle ... and point it away from yourself and any bystanders".
- Coaster: Libbey 763804 stainless wine bottle coaster, 4.75 in diameter, 1 in height (katom.com/192-763804.html).
- Dish: Iconikal round white ceramic dipping dish, 4 in across, 0.8 in tall (Amazon B0B4BJGPL4).
- Shade: IKEA RINGSTA lamp shade, white, width 19 cm, height 15 cm, polystyrene/PET (904.040.92).
- Mannequin: Mannequin Mall MM-9020 realistic sitting female, height 4 ft 3 in on its 19 in stool.
- Chair: IKEA STEFAN chair, brown-black (002.110.88): width 42, depth 49, height 90, seat width 42, seat depth 37,
  seat height 45 cm.
- Station table (yam_arena): top 0.7475 m above the floor, x -0.19 to 0.91, y -0.65 to 0.65.
Estimates (no page gives them): the bottle's shoulder profile and centre of mass, the stopper's knob width and its
pull-out force (12 N; a real trial measures it with a luggage scale, see README), the plate and flute sizes, the masses of the coaster, dish, plate and flute, frictions.
"""

from __future__ import annotations

import math

TOP = 0.7475  # the station table's top (yam_arena)
TABLE_X, TABLE_Y = (-0.19, 0.91), (-0.65, 0.65)

# -- the bottle (frame: origin at the middle of its base, z up along its axis) -----------------------------------------
BOTTLE_R = 0.0353  # 70.6 mm across
BOTTLE_H = 0.2435  # 243.5 mm to the lip
BOTTLE_MASS = 0.500  # 500 g, empty
BOTTLE_COM = (0.0, 0.0, 0.085)  # estimate: thick base and punt, thin walls
BODY_TOP = 0.135  # the straight body ends here (estimate, from the maker's drawing proportions)
SHOULDER = ((0.150, 0.029), (0.165, 0.021), (0.180, 0.0155))  # (height, radius) steps of the shoulder (estimate)
NECK_R = 0.0145  # 29 mm crown finish
LIP_R, LIP_Z = 0.0150, (0.232, BOTTLE_H)  # the crown ring at the top of the neck
BOTTLE_FRICTION = (0.6, 0.5)  # glass on a gripper pad / on a stainless coaster (estimate)

# -- the stopper (frame: origin on the axis at the lip, z out of the bottle) --------------------------------------------
SHANK_R, SHANK_LEN = 0.0100, 0.025  # MoreBeer W460: 20 mm ribbed stem, 30.5 mm long. The ribs are squeezed to the
# 17.5 mm bore; the model's collider is the rigid 17.0 mm core and the squeezed ribs' grip is the fit. MODELLED 25 mm
# long: with the full 30.5 mm the fingers lost the stopper 5 mm into the draw (runs f-b1, g-b1, h-b1, all flew)
BORE_R, BORE_DEPTH = 0.00875, 0.030  # the 17.5 mm bore (Saverglass), modelled 30 mm deep
FLANGE_R, FLANGE_T = 0.01525, 0.004  # W460: 30.5 mm head; its 4 mm flange rests on the lip (flange thickness estimate)
KNOB_R, KNOB_H = 0.0105, 0.014  # 21 mm knob above the flange, domed on top. The page's head is 16 mm tall (46.5 - 30.5);
# the model's is 18 mm (4 + 14): the 2 mm longer knob gives the gripper's pads the grip length the demo was proven with
STOPPER_MASS = 0.010  # about 10 g (estimate)
PULL_N = 12.0  # N to start it moving, seated (estimate: "go in easily by hand"); falls with the length left inside
TWIST_DAMP = 0.01  # N m s/rad: the shank's drag when it is twisted in the bore (estimate)
EXIT_AT = SHANK_LEN  # the stopper is out when its shank's end passes the lip

# -- the coaster, the dish, the guest's place, the lamp, the wall ------------------------------------------------------
COASTER_XY = (0.34, 0.02)
COASTER_R, COASTER_RIM_H, COASTER_FLOOR, COASTER_WALL = 0.0603, 0.0254, 0.004, 0.002  # Libbey 763804: 4 3/4 in, 1 in
DISH_XY, DISH_R, DISH_H, DISH_FLOOR = (0.20, -0.30), 0.0508, 0.0203, 0.004  # Iconikal white ceramic dish, 4 x 0.8 in
PLATE_XY, PLATE_R, PLATE_H = (0.68, 0.48), 0.130, 0.020  # 26 cm dinner plate, in front of the child
FLUTE_XY, FLUTE_BASE_R, FLUTE_BOWL_R, FLUTE_H = (0.50, 0.54), 0.035, 0.032, 0.105  # the child's drinking glass
# (the scorer's name "flute" is kept): a 7 cm tumbler 10.5 cm tall (estimate)
# The guest is a seated child at the table's LEFT side (chair seat's middle at GUEST_XY, the seat's front at the table
# edge y 0.65), facing the table. Only a child's head fits the top camera: an adult's seated head (41 cm over the table)
# is above the camera's view at every seat round this table (measured from the station camera's pose and lens).
GUEST_XY = (0.66, 0.835)
HEAD_XY = (0.66, 0.73)  # the child sits up to the table, chest 3 cm from its edge, head over the chest
SIT_H = 0.64  # crown above the seat: a 6-7-year-old's sitting height (estimate from growth tables)
HEAD_R = 0.085
SEAT_Z, SEAT_W, SEAT_D, CHAIR_H = 0.45, 0.42, 0.37, 0.90  # IKEA STEFAN chair (brown-black)
HEAD_Z = SEAT_Z + SIT_H - HEAD_R - TOP  # head centre 25.8 cm over the table top (crown 109 cm above the floor)
SHOULDER_Z = SEAT_Z + 0.42  # 87 cm above the floor: 12 cm over the table top
TRIM_XY = (0.47, 0.14)  # the removed foil capsule and wire hood, lying on the table beside the coaster
LAMP_XY = (0.26, -0.08)  # the pendant lamp, over the bottle's mouth (8 cm nearer the robot, 10 cm to its right)
LAMP_R, LAMP_BOTTOM, LAMP_TALL = 0.095, 0.60, 0.15  # 19 x 15 cm drum shade (IKEA RINGSTA size), its rim 60 cm (24 in) above the table:
# hung lower than the usual 30-36 in so that the top camera (60 degrees down, from behind the arms) sees its rim and red
# band; the rest of the shade and the rod are above the picture
BAND = 0.02  # the red clearance marks: a 2 cm paper band hung below the shade's rim, a 2 cm-thick foam sleeve on the rod
WALL_Y = -0.70  # the room wall, 5 cm past the table's right edge; nothing stands on that side

# -- the gas (a disclosed rule, simulation only) ------------------------------------------------------------------------
# Once the fit has given (the stopper has slipped GIVE_AT along the neck) and no gripper has held it for FREE_STEPS control
# steps, the stopper leaves along the neck axis at FLY_SPEED: AAO, "can fly up to 34 mph" (15.2 m/s).
GIVE_AT = 0.0003
FREE_STEPS = 2
FLY_SPEED = 15.2

# -- the score ------------------------------------------------------------------------------------------------------------
TILT_RANGE = (30.0, 60.0)  # degrees from vertical at the moment the stopper leaves (AAO: 45)
LAMP_CONE = 25.0  # degrees: the axis within this of the line from the lip to the shade's middle = aimed at the lamp
GUEST_CONE = 45.0  # degrees of heading (seen from above) either side of the line from the lip to the child's head
SELF_CONE = 45.0  # degrees of heading either side of the line from the lip back to the midpoint of the arm bases
UPRIGHT_DEG = 10.0
LAMP_TOUCH = 0.02  # a finger point within 20 mm of the lamp (shade or rod), at any time: 0
CLEAR = 0.05  # every finger point at least 50 mm from the bottle and the stopper at the end, for 1 s
GUEST_MOVE = 0.02  # the plate and the flute moved at most 20 mm (and the flute tilted at most 10 degrees)


def heading_deg(dx: float, dy: float) -> float:
    return math.degrees(math.atan2(dy, dx))
