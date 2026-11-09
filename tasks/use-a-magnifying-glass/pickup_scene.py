"""Fixed starting layout for the magnifier task."""
import math
from yam_arena.task_file import Box

SCENE = "station"
OBJECTS = [
    Box("magnifier", pos=(.20, -.27, .757), size=(.080, .020, .018),
        yaw=-math.pi / 2, mass=.120, color=(.10, .23, .40)),
    Box("target_plate", pos=(.35, .05, .75125), size=(.120, .100, .0075),
        color=(.95, .95, .95)),
]
