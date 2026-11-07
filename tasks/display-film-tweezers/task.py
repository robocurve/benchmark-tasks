"""Remove the protective film from the display module in its loose assembly carrier.
Use the passive tweezers to grasp the recessed starter tab. Lay the detached film
entirely outside the carrier, return the tweezers upright to their stand, and
withdraw both hands. Throughout the trial the carrier must stay within 30 mm
horizontally, 5 mm vertically, and 10 degrees of its starting orientation.
"""
from pathlib import Path
import sys

# yam_arena loads task files by path, without adding their directory to sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from yam_arena.task_file import Box
from geometry import FILM_NAMES, CARRIER_POS, CARRIER_SIZE, STRIP_SIZE, TAB_POS, TAB_SIZE, strip_pos
from filmkit import install, trial

SCENE = 'station'
# Trial counts the full second itself so a timeout cannot bypass the hold.
SUCCESS_HOLD_S = 0.0
SCORE_OVER = 'end'
OBJECTS = [Box(name='carrier', pos=tuple(map(float, CARRIER_POS)), size=tuple(map(float, CARRIER_SIZE)),
               mass=.250, color=(.12, .15, .18)),
           Box(name='pull_tab', pos=tuple(map(float, TAB_POS)), size=tuple(map(float, TAB_SIZE)),
               mass=.003, color=(.03, .2, .85))]
OBJECTS += [Box(name=name, pos=tuple(map(float, strip_pos(i))), size=tuple(map(float, STRIP_SIZE)),
                mass=.0003, color=(.03, .2, .85)) for i, name in enumerate(FILM_NAMES[1:])]
from tool_geometry import tool_objects
OBJECTS += tool_objects()
install(Path(__file__).resolve())


def score(objects):
    return trial.score(objects)


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    from geometry import N_BONDED, PITCH
    return (N_BONDED - trial.peeled) * PITCH



from motion import demo
