"""Two sliding keys sit beside a fallen panel. A glove and cuff protrude on the far side."""
from pathlib import Path
import sys, json, os
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from peg_return import STIFFNESS, DAMPING, LINK_BOUNDS, ReturnClutch, grippers_clear
from fixture import BODIES, WORLD, INSTRUCTION, SIDES
from scoring import Evaluator, Frame
from scenekit import UsdBody, on_reset, on_step, _np

SCENE="station"
SCORE_OVER="end"
SUCCESS_HOLD_S=0.
joints=[b for b in BODIES if b[3]]
OBJECTS=[UsdBody("rescue_trainer",pos=WORLD,size=(.42,.25,.16),usd="assets/trainer.usda",
                 articulation=True,joint_pos=tuple((b[3],0.) for b in joints),
                 joint_actuators=tuple((b[3],b[8],b[9]) for b in joints),
                 joint_actuator_limits=tuple((b[3],50.,2.) for b in joints),self_collisions=True,
                 friction=(.55,.55),solver_iters=64)]
meter=Evaluator()
clutch=ReturnClutch()
reader=None
trace=None
last_row=None
def demo():
    from reference import plan
    return plan()

def set_return_drives(articulation, engaged):
    ids=[articulation.joint_names.index("advance_"+s) for s in SIDES]
    k=STIFFNESS if engaged else 0.
    d=DAMPING if engaged else 80.
    if hasattr(articulation,"write_joint_stiffness_to_sim_index"):
        articulation.write_joint_stiffness_to_sim_index(stiffness=k,joint_ids=ids)
        articulation.write_joint_damping_to_sim_index(damping=d,joint_ids=ids)
    else:
        # Earlier Isaac Lab torch interface accepts scalar gains for this slice.
        articulation.write_joint_stiffness_to_sim(k,joint_ids=ids)
        articulation.write_joint_damping_to_sim(d,joint_ids=ids)

@on_reset
def reset(env):
    global reader,meter,trace,last_row,clutch
    last_row=None
    from contacts import ContactReader
    if reader is not None:reader.close()
    if trace is not None:trace.close()
    meter=Evaluator()
    clutch=ReturnClutch()
    set_return_drives(env.unwrapped.scene['rescue_trainer'],False)
    reader=ContactReader(env)
    log=os.environ.get("PANEL_TRACE")
    trace=open(log,"x",encoding="utf8",buffering=1) if log else None

@on_step
def step(env):
    global last_row
    a=env.unwrapped.scene["rescue_trainer"]
    q=dict(zip(a.joint_names,map(float,_np(a.data.joint_pos)[0])))
    v=dict(zip(a.joint_names,map(float,_np(a.data.joint_vel)[0])))
    digit,seat,hit,touch,landing,key_touch=reader.read()
    from yam_arena import YAM_URDF,ik,station
    from yam_arena.constants import ALL_JOINTS
    from scenekit import arm_joints
    poses=[]
    for side in SIDES:
        j=arm_joints(env,side,ALL_JOINTS)
        fk=ik.forward_kinematics(YAM_URDF,dict(zip(ALL_JOINTS,j)))
        offset=np.asarray(station.BASES[side])-WORLD
        for name in LINK_BOUNDS:
            pose=fk[name]
            poses.append((name,pose[:3,3]+offset,pose[:3,:3]))
    clear=grippers_clear(poses)
    f=Frame(q,v,digit,seat,hit,touch,clear,landing,key_touch,clutch.active)
    if clutch.update(q,digit,seat,touch,clear):
        set_return_drives(a,True)
    meter.observe(f)
    last_row=meter.record(f)
    if trace:trace.write(json.dumps(last_row)+"\n")

def score(objects):return meter.score
def success(objects):return meter.score==1.
