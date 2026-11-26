"""Turn the open 2-in-1 laptop into a stable tent display for viewing.

The single laptop is on the bare wood worktop, keyboard toward the arms.
Its hinge runs along y at x = 360 mm and starts open 112 degrees from closed.
All four feet are on the worktop. The left case lip extends 19 mm beyond
the desk's side, where it can be pinched from below. The YAM bases sit at
y = +520 and -420 mm.
The fixed side view is published as top_cam. The wrist views retain their
usual camera names. Work on the case edges. You can use the desktop for
support and change grips. Leave the display facing out and both free edges
on the desk, then release both hands.
"""

import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from yam_arena.task_file import Box

from arena_adapter import install
from demo import LaptopController
from geometry import MASS, RESET_HINGE, WIDTH


SCENE = 'tabletop'
LAPTOP_TENT_TASK = True
CONTROL = os.getenv('LAPTOP_TENT_CONTROL', 'normal')
if CONTROL not in ('normal', 'flat', 'held'):
    raise ValueError('LAPTOP_TENT_CONTROL must be normal, flat or held')

OBJECTS = [Box('laptop', RESET_HINGE, size=(0.218, WIDTH, 0.01745), mass=sum(MASS))]


def success(_objects):
    """The scene adapter evaluates success from contact and motion history."""
    return False


def demo():
    return LaptopController(CONTROL)


install()
