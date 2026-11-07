"""Attempt installation while grasping the root across x instead of squeezing the retaining legs."""
from dataclasses import replace
from pathlib import Path
import math
from yam_arena.task_file import load
_base=load(Path(__file__).with_name('task.py'))
SCENE=_base.scene
OBJECTS=_base.objects
SUCCESS_HOLD_S=1.
SCORE_OVER='end'
score=_base.score
success=_base.success
distance=_base.distance
observe=_base.module.observe
reset_history=_base.module.reset_history
def demo():
    plans=_base.demo()
    plans['right']=[replace(w,yaw=math.pi/2) for w in plans['right']]
    return plans
