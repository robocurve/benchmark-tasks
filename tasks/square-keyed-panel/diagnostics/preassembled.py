"""Physics diagnostic only; parts start at their assembled goal."""
import importlib.util
from pathlib import Path
from dataclasses import replace
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
spec=importlib.util.spec_from_file_location('square_api_task',Path(__file__).resolve().parents[1]/'task.py')
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
SCENE=t.SCENE
OBJECTS=[replace(obj,pos=(*goal,t.TOP+t.HEIGHT/2),yaw=0.0) for obj,goal in zip(t.OBJECTS[:4],t.GOAL)]+t.OBJECTS[4:]
score,success,distance=t.score,t.success,t.distance
def demo(): return {'left':[Waypoint(5.0,(.12,.30,t.TOP+.12),FINGER_OPEN,'wait clear')]}
