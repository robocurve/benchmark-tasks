"""Punch a card's two filing holes on their marks with the desk two-hole punch: each hole's centre within 1.5 mm of
its crosshair. The card lies on the foam stand in front of the punch; the punch is fixed to the table.

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
