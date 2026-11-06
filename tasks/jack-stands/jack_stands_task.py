"""On the station table, a tabletop trainer of jacking up an axle and setting it on jack stands.

A grey solid-steel beam (10 cm deep, 40 cm long, 6 cm tall, 23 kg: far too heavy for one arm) lies across the table
in front of the arms, on two dark end blocks 5 cm tall. Its lift point, in the middle, is marked yellow; its two
support points, 12 cm either side of the middle, are marked cyan (on the top and on the near face).

A blue floor jack is fixed under the lift point. Its black saddle sits 3 mm below the beam. Pump the jack by pressing
its green lever pad (in front of the beam, on the centre line) all the way down and letting it spring back up: each
full stroke raises the saddle 7 mm. The yellow release knob (on a post in front of the right side) lowers the saddle
slowly when it is turned counterclockwise, seen from above: it starts to open at 10 degrees and is fully open at
50 degrees (about 12 mm/s). While it is open, pumping does nothing.

Two red jack stands, fixed height (their tops 8 cm above the table), stand 10 cm in front of the support points.
Each has a long skid with a dark-red grip block at its near end, so it can be slid forward by the block until its
column is under the beam. An orange wrench (the work) lies on the table under the beam, its grip block just past
the beam's near edge.

Never put a gripper under the beam, or within 6 cm of its edges below its underside, and never move the wrench,
unless the beam rests on both stands.
"""

from yam_arena.task_file import Box, Cylinder

from jack_stands import design as d

SCENE = "station"
JACK_BLUE = (0.10, 0.30, 0.90)


def _fixed():
    (jx, jy), (sx, sy, sz) = d.JACK_BODY
    top = d.TOP
    parts = [Box("jack_body", pos=(jx, jy, top + sz / 2), size=(sx, sy, sz), color=JACK_BLUE)]
    cx, cy, cz = d.CHEEK
    for s, n in ((1, "lever_cheek_left"), (-1, "lever_cheek_right")):
        parts.append(Box(n, pos=(d.CHEEK_X, s * d.CHEEK_Y, top + cz / 2), size=(cx, cy, cz), color=JACK_BLUE))
    r, h = d.KNOB_POST
    parts.append(Cylinder("knob_post", pos=(float(d.KNOB_XY[0]), float(d.KNOB_XY[1]), top + h / 2), radius=r,
                          height=h, color=(0.15, 0.15, 0.17)))
    return parts


OBJECTS = _fixed()


def success(objects):
    return False  # scored by the embodiment yam_jack_stands (jack_stands/scoring.py), from the measured part poses
