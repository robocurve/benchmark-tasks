"""On the station table, a small bench lathe seen from its front. A grey cast bed (15 x 35 cm, 3 cm tall) runs left to
right in front of the arms; on the left a grey headstock (12 x 10.5 cm, 17.5 cm tall) carries a dark chuck (8 cm
across) holding a steel bar 8 mm across, its axis 12 cm above the table, sticking out 40 mm to the right.

The tool approaches the bar from the robot's side: a steel tool bit (8 mm square) in a black tool holder, its
cutting edge 3 mm short of the bar and 0.75 mm above the bar's centre line (too small to see). Two cranks with yellow
knobs (16 mm across, 30 mm tall) turn the tool's screws: the cross-feed crank on a grey pedestal in front of the
carriage moves the tool toward the bar (2.54 mm per clockwise turn), and the height crank on top of the tool holder
lowers the tool (1.06 mm per clockwise turn, raising it when turned the other way). Each knob spins freely on its
crank arm.

In front of the right arm, a 150 mm steel rule (13 mm wide, 0.5 mm thick, a red top end) stands upright in a slot in
a blue block. Trapped lightly between the cutting edge and the bar, the rule leans toward the robot when the edge is
below centre and away from it when the edge is above centre; it stands vertical when the edge is on centre.
"""

from yam_arena.task_file import Box

from lathe_centre_height import design as d

SCENE = "station"
BLUE = (0.10, 0.30, 0.80)


def _stand():
    sx, sy = d.RULE_STAND_XY
    sxw, syw, sh = d.STAND
    jaw = (sxw - d.SLOT_W) / 2
    parts = [Box("stand_floor", pos=(float(sx), float(sy), d.TOP + 0.005), size=(sxw, syw, 0.010), color=BLUE)]
    for i, s in enumerate((-1, 1)):
        parts.append(Box(f"stand_jaw_{i}", pos=(float(sx + s * (d.SLOT_W / 2 + jaw / 2)), float(sy),
                                                d.TOP + (0.010 + sh) / 2), size=(jaw, syw, sh - 0.010), color=BLUE))
    return parts


OBJECTS = _stand()


def success(objects):
    return False  # scored by the embodiment yam_lathe_centre_height (lathe_centre_height/scoring.py)
