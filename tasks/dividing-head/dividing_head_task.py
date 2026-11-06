"""On the station table, in front of the left arm: a dividing head, the machinist's tool for turning a workpiece
through exact fractions of a turn. Its dark-blue housing (15 x 31 cm, 5 cm tall) is clamped to the table by two
dark steel clamp blocks. A 40-tooth worm wheel inside it (not visible) turns the spindle one-fortieth of a turn for
each full turn of the index crank.

On the housing, in front of the left arm, a grey steel index plate (14 cm across) lies flat with two circles of
holes: 20 holes on the inner circle (42 mm radius) and 21 holes on the outer circle (56 mm radius). A red zero line
on the plate runs through hole 0 of both circles (both red). The orange index crank turns about the plate's centre
above it. Its plunger rides in a slot along the crank arm: a steel pin, pressed down by a spring, with a yellow knob
(20 mm across, 36 mm tall) on top. Pulling the knob up about 5 mm lifts the pin out of its hole; the crank then turns
freely, and the plunger can slide along the arm between the two circles. Released over a hole, the pin drops in and
locks the crank. The knob spins freely on the pin. At the start the pin is in hole 0 of the 20-hole circle, on the
zero line.

On the housing nearer the centre line, the spindle carries the workpiece: an aluminium disc (100 mm across, 16 mm
thick) lying flat, with a red reference line painted from its centre to its rim, pointing at a fixed green pointer.
The workpiece turns only through the crank (the worm cannot be turned backward): pushing the disc does nothing.
"""

from yam_arena.task_file import Box

from dividing_head import design as d

SCENE = "station"
CLAMP_COLOR = (0.22, 0.22, 0.24)
CLAMP = (0.030, 0.040, 0.015)


def _clamps():
    y = d.CRANK_XY[1] + (d.HOUSE_Y[0] + d.HOUSE_Y[1]) / 2
    xs = (d.CRANK_XY[0] + d.HOUSE_X[0] - CLAMP[0] / 2, d.CRANK_XY[0] + d.HOUSE_X[1] + CLAMP[0] / 2)
    return [Box(f"clamp_{i}", pos=(float(x), float(y), d.TOP + CLAMP[2] / 2), size=CLAMP, color=CLAMP_COLOR)
            for i, x in enumerate(xs)]


OBJECTS = _clamps()


def success(objects):
    return False  # scored by the embodiment yam_dividing_head (dividing_head/scoring.py), from the head's part poses
