"""Pick up and roll a marked die on the station table.

The only loose object is a 16 mm cube, mass 5 g, with coloured pip-marked faces.
Its collision geometry remains one cube; run.py installs render-only paint.
Its centre starts
at world x=0.20 m, y=0.16 m, z=0.7555 m on the oak-coloured table. World x points
away from the arm bases, y left, z up. Pick it up, release it so it tumbles freely,
and leave it on the tabletop until it stops. No final numbered face is requested.
Use run.py: the scorer needs measured gripper states and a fresh tracker per reset.
"""

from yam_arena.task_file import Box, TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DIE = Box("die", pos=(0.20, 0.16, TOP + 0.008), size=(0.016,)*3, mass=0.005, color=(0.95, 0.95, 0.92))
OBJECTS = [DIE]
SUCCESS_HOLD_S = 0.0  # the temporal scorer itself verifies the full one-second rest
SCORE_OVER = "end"   # a later violation must erase previously earned partial credit

_tracker = None


def success(objects):
    if _tracker is None:
        raise RuntimeError("Use run.py; stock yam_arena does not supply the required robot-state audit")
    return _tracker.value == 1.0


def score(objects):
    return _tracker.value if _tracker is not None else 0.0
