"""Reattach a live left-wrist camera. Requires wrist_adapter; not a standalone object-only scene."""
import json
from pathlib import Path
import numpy as np
from yam_arena.task_file import Asset,Box,TABLE_TOP
from yam_arena.demo import pick_place_waypoints,Waypoint
from yam_arena.constants import PAD_TIP_PAST_TCP,FINGER_OPEN
SCENE='station';TOP=TABLE_TOP[SCENE]
LAYOUT=json.loads(Path(__file__).with_name('layout_prototype.json').read_text())
PRESENT=np.array(LAYOUT['left_present_joints']);GOAL=np.array(LAYOUT['camera_center_world'])
CAMERA=Box('loose_camera',pos=(.36,-.19,TOP+.025),size=(.04,.036,.028),mass=.055,color=(.08,.1,.13))
from yam_arena import station
OBJECTS=[CAMERA,Asset('apple','apple_01_objaverse_robolab',(.30,.28),.066,mass=.12,color=station.APPLE_RED,convex=True),Asset('orange','orange_02_fruits_veggies_robolab',(.40,.26),.064,mass=.11,color=station.ORANGE)]
def demo():
 # Full-rotation waypoints require motion.PosePlan, used by run_demo.py.
 from motion import make_plan
 return {'right':make_plan().waypoints}
