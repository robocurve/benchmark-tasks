"""Cut the bottom tape, open four bottom flaps, and flatten the intact carton.

The empty carton starts upright with its top flaps folded back. Roll it upside down first. Cut the centre seam and taped end folds with the guarded cutter,
unfold every bottom flap, collapse along the existing wall creases and leave
it flat. No handles or pre-cut bottom tape. Use run.py for native joints.
"""
from yam_arena.task_file import Box
from scene import layout
from config import *
SCENE='station';SUCCESS_HOLD_S=0.;SCORE_OVER='end'
PARTS,HINGES=layout()
OBJECTS=[Box(p['name'],pos=starting_pose(p,upright=True)[0],size=p['size'],mass=p['mass'],color=(.58,.36,.17)) for p in PARTS]
OBJECTS += [Box('cutter_pad',pos=CUTTER_PAD_START,size=(.17,.07,.002),color=(.85,.30,.08)),Box('cutter',pos=CUTTER_START,size=(.11,.025,.025),mass=.055,color=(1.,.30,.02))]
_tracker=None
_diagnostic=True
def score(objects):return 0. if _tracker is None or _diagnostic else _tracker.value
def success(objects):return score(objects)==1.
