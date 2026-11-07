"""The scissors_handoff.py scene and scoring with an empty plan: with --policy yam_scripted, both arms hold their
READY pose and do nothing. This records the zero-action video that stands in for the Dream-1 run, because the
Dream API refused the instruction (DA-4000, unsafe instruction) and sent no actions.
"""

import importlib.util
from pathlib import Path

_spec = importlib.util.spec_from_file_location("scissors_handoff", Path(__file__).with_name("scissors_handoff.py"))
_task = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_task)

SCENE = _task.SCENE
OBJECTS = _task.OBJECTS
SUCCESS_HOLD_S = _task.SUCCESS_HOLD_S
success = _task.success
score = _task.score
distance = _task.distance


def demo():
    return {}
