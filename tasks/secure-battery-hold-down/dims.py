"""Sizes and places for the secure-battery-hold-down task, shared by task.py and the
script that builds the assets. Metres; positions in the station frame (x forward from the arms, y left), heights from the table top.

Sources (see README.md):
- Battery: BCI Group 35, the BCI group size chart's maximum 230 x 175 x 225 mm (L x W x H); flooded, about 17 kg
  (Marathon 35 lb, Interstate MTP-35 about 38 lb). It sits in its tray from the start (too heavy to lift).
- J-bolts: 1/4 in (6.35 mm) steel rod, 1/4-20, 12 in long (Lawson 97542; Grote 12 in J-style set): a 10 in bolt
  would not reach above the bar from a hook that rests on the floor under the tray (see BOLT_TOP below).
- Hold-down bar: steel, 1 in (25.4 mm) wide (NAPA/Grote frame width 1 in); 228 mm long and 4 mm thick, its slotted
  ends taking the bolts a few mm closer or further apart.
- Tray, its slots, the bracket under it and the engine-bay well: sized to the parts above.
"""

MM = 0.001

# -- the battery ------------------------------------------------------------------------------------------------
BAT_L, BAT_W, BAT_H = 0.230, 0.175, 0.225  # along x, y, z
BAT_X, BAT_Y = 0.300, 0.000  # its centre
BAT_TOP = 0.040  # its top above the table top: it sits in a well cut into the table (the engine bay)

# -- the tray and the bracket it sits on ------------------------------------------------------------------------
PLATE_T = 0.002  # steel tray floor
BRACKET_H = 0.0195  # the bracket holds the tray's floor this far above the well's floor: a hook resting on the
# floor has its tip 1.5 mm under the tray's floor, so a hooked bolt let go can't tip more than a few degrees
PLATE_TOP = BAT_TOP - BAT_H  # the battery stands on the tray's floor
PLATE_BOTTOM = PLATE_TOP - PLATE_T
FLOOR = PLATE_BOTTOM - BRACKET_H  # the well's floor: -0.2065
PLATE_X = (BAT_X - BAT_L / 2 - 0.005, BAT_X + BAT_L / 2 + 0.005)
BRACKET_Y = 0.080  # the bracket spans |y| <= this (under the battery), leaving room under the tray's sides
SLOT_LEN, SLOT_W = 0.028, 0.010  # each hook slot: along x, across (y)
SLOT_IN = 0.090  # its inner edge (|y|): 2.5 mm outside the battery's side
SLOT_OUT = SLOT_IN + SLOT_W  # 0.100
TRAY_LEDGE = 0.006  # the tray's floor runs on this far beyond the slot to its side wall
WALL_IN = SLOT_OUT + TRAY_LEDGE  # 0.106, the side wall's inner face
SLOT_X = (BAT_X - SLOT_LEN / 2, BAT_X + SLOT_LEN / 2)  # one slot each side, halfway along the battery
SLOT_YC = (SLOT_IN + SLOT_OUT) / 2  # 0.095
TRAY_WALL_T, TRAY_WALL_H = 0.002, 0.040  # the tray's side and end walls, above its floor

# -- the well cut in the table (the engine bay) -----------------------------------------------------------------
HOLE_X = (0.130, 0.470)
HOLE_Y = (-0.140, 0.140)
WELL_WALL_T = 0.005

# -- the J-bolts ------------------------------------------------------------------------------------------------
ROD_D = 6.35 * MM
ROD_R = ROD_D / 2
J_LEN = 0.305  # 12 in, from the hook's underside to the top
J_GAP = 6.0 * MM  # inside the J, between the shank and the hook's tip
J_TIP_H = 18.0 * MM  # the tip's top above the hook's underside (resting on the well's floor, it is 1.5 mm under the tray's floor)
J_TIP_X = ROD_D + J_GAP  # the tip's axis from the shank's (along +x of the bolt's frame)
J_MASS = 0.080  # kg: 6.35 mm steel rod, about 330 mm in all
BOLT_TOP = FLOOR + J_LEN  # a hooked bolt resting on the well's floor: its top this high (+0.0985, 54.5 mm above the bar)

# -- the hold-down bar ------------------------------------------------------------------------------------------
BAR_LEN, BAR_W, BAR_T = 0.228, 0.0254, 0.004
BAR_HOLE = 0.010  # each end's slot: 10 mm wide...
BAR_SLOT = 0.020  # ... and 20 mm long, along the bar, so it takes the bolts a little more or less than 192 mm apart
BAR_HOLE_Y = 0.096  # the slots' centres from the bar's middle, along it (a let-go bolt's top leans out a few mm)
BAR_MASS = 0.170  # kg: 228 x 25.4 x 4 mm steel less the slots

# -- the nuts: 1/4-20 steel flange nuts (9/16 in tall, 1 in flange, as McMaster's 1/4-20 flange nuts) --------------
NUT_H = 0.0143  # overall height
NUT_FLANGE_D, NUT_FLANGE_T = 0.0254, 0.002  # the flange it bears on the bar with
NUT_BODY_D = 0.017  # the body above the flange
NUT_BORE = 0.008  # the bore round the 1/4 in bolt
NUT_MASS = 0.025  # kg
NUT_X, NUT_Y = 0.170, 0.200  # each lies flange down on the table beside the bay, one each side (+-y)

# -- the holders the new bolts stand in, at the front corners of the bay -----------------------------------------
HOLDER = (0.030, 0.030, 0.060)  # x, y, z: a block on the well's floor with a slot the hook stands in
HOLDER_SLOT = (0.020, 0.007)  # along x, y
HOLDER_X = 0.155
HOLDER_Y = 0.112
