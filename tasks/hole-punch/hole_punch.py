"""A desk two-hole punch stands on the table, its throat facing you. A yellow A6 card lies flat on a grey foam stand
in front of it, level with the punch's bed. Punch the card's two filing holes on their two black crosshair marks.

The punch: black steel, its bed 2 cm above the table and a 6 mm throat whose front face is at x 26.2 cm; two 6 mm
punches 80 mm apart at x 27.0 cm, y -4 and +4 cm, 12 mm in front of a back stop at x 28.2 cm. A steel lever, pivoted
behind the punches at x 29.5 cm, reaches forward over the punch head on its left side to a red handle knob at x
19.5-22.5 cm, y 8-12.5 cm, about 6 cm above the table. Pressing the knob down about 3 cm drives the punches through
the card into the dies; a return spring lifts it back.

The card: 148 x 105 mm, its long edge facing the punch 4 cm in front of the back stop, centred at y -1.5 cm. Its
marks are 12 mm in from that edge and 80 mm apart, so the card is on its marks when its edge is against the back
stop and its middle is on the punch's centre line (y 0, the white line on the bed).

Run with the yam_hole_punch embodiment (pip install -e this folder). It builds the punch, lever, head, stand and card
and scores the holes.
"""

from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
# A fixed dark green cutting mat under the punch and stand (2 mm). Every other part is built by the yam_hole_punch
# embodiment (punch_kit/assets.py).
OBJECTS = [Box("desk_mat", pos=(0.21, 0.0, TOP + 0.001), size=(0.28, 0.30, 0.002), color=(0.10, 0.30, 0.20))]


def success(objects):
    return False  # the embodiment scores the holes (punch_kit/scoring.py)
