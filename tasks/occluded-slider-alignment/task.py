"""An orange sliding handle projects from a fixed covered service fixture. Its black
index and the green target can be inspected through the side opening. Adjust the
slider until the index aligns with the target, then release it and leave it still. Keep the
fixture intact and avoid unintended contact with its housing or the other arm.
"""

from pathlib import Path
import sys

# The official task loader imports by file location, without adding its directory.
_HERE = str(Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from slider_task.layout import OBJECTS

SCENE = 'station'
SCORE_OVER = 'end'
# The evaluator performs the 20 consecutive samples itself; no double hold.
SUCCESS_HOLD_S = 0


def _evaluation(objects):
    if '__evaluation__' not in objects:
        raise RuntimeError('Use launch.py: stock task descriptors lack joints/contact history. Scoring fails closed.')
    return objects['__evaluation__']


def success(objects):
    return _evaluation(objects)['complete']


def distance(objects):
    return _evaluation(objects)['error_m']


def score(objects):
    return _evaluation(objects)['score']


def demo():
    # Privileged, open-loop feasibility oracle; not a vision policy or Dream run.
    from slider_task.oracle import manipulation_waypoints
    return manipulation_waypoints()
