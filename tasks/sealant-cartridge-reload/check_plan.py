"""Offline kinematics only; does not demonstrate native feasibility."""
import argparse,json
from inspect_robots import Scene
from sealant_reload.controller import ScriptedPolicy
from sealant_reload.config import INSTRUCTION
p=argparse.ArgumentParser();p.add_argument('--mode',default='release');a=p.parse_args()
policy=ScriptedPolicy(a.mode);policy.reset(Scene(id='offline',instruction=INSTRUCTION))
for i in range(round(policy.duration*20)+1): policy.action_at(i/20)
print(json.dumps(dict(scope='offline IK only',mode=a.mode,duration_s=policy.duration,worst_position_m=policy.worst_position_error,worst_rotation_rad=policy.worst_rotation_error)))
