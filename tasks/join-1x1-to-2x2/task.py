"""Join the red upper block to the blue lower block and demonstrate retention."""
from yam_arena.task_file import Box
from config import CONFIG

SCENE = "station"
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
OBJECTS = [Box(role, pos=(*CONFIG[role + '_xy'], CONFIG['table_top'] + CONFIG['body_height'] / 2),
               size=(*(n * CONFIG['pitch'] - .0002 for n in CONFIG[role + '_shape']), CONFIG['body_height']),
               mass=CONFIG[role + '_mass'], color=CONFIG[role + '_color'])
           for role in ('lower', 'upper')]
# run.py replaces these placeholders with compound hollow rigid bricks before
# simulator construction; the stock runner must not be used for this task.
_tracker = None

def success(objects):
    if _tracker is None:
        raise RuntimeError('Use run.py for compound brick geometry and measured hand scoring')
    return _tracker.score == 1.0

def score(objects):
    return _tracker.score if _tracker is not None else 0.0
