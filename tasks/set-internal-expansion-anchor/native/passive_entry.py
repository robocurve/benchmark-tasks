"""Both arms hold the ready pose: a passive reset check of the scene. The untouched scene scores 0."""
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from task_runtime import OBJECTS,SCENE,SCORE_OVER,SUCCESS_HOLD_S,CONTROL_HZ,score,success
from yam_arena.demo import Waypoint
from yam_arena import ik,station
from yam_arena.constants import READY,FINGER_OPEN

def demo():
 return {s:[Waypoint(6.,tuple(ik.tcp_pose(READY,station.BASES[s])[0]),FINGER_OPEN,'passive_reset')]for s in ['left','right']}
