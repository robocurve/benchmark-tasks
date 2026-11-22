"""A garment caught between a stationary padded roller and an ironer's pressure shoe."""
from pathlib import Path
import sys,json,os
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from fixture import WORLD,BODIES,COUNT,INSTRUCTION
from measure import observe,glove_geometry
from withdrawal import native_arm_state
from scoring import Evaluator
from scenekit import UsdBody,on_reset,on_step,_np
SCENE="station"
SCORE_OVER="end"
SUCCESS_HOLD_S=0.
joints=[b for b in BODIES if b[3]]
OBJECTS=[UsdBody("ironer",pos=WORLD,size=(.38,.41,.20),usd="assets/ironer.usda",articulation=True,
                 joint_pos=tuple((b[3],0.) for b in joints),joint_actuators=tuple((b[3],b[8],b[9]) for b in joints),
                 joint_actuator_limits=tuple((b[3],60.,12.) for b in joints),self_collisions=True,solver_iters=64)]
meter=Evaluator()
reader=trace=last_row=None
def demo():
    from reference import plan
    return plan()
@on_reset
def reset(env):
    global meter,reader,trace,last_row
    from contacts import Reader
    if reader:reader.close()
    if trace:trace.close()
    meter=Evaluator();reader=Reader(env);last_row=None
    log=os.environ.get("IRONER_TRACE")
    trace=open(log,"x",encoding="utf8",buffering=1) if log else None
@on_step
def step(env):
    global last_row
    rig=env.unwrapped.scene["ironer"]
    q=dict(zip(rig.joint_names,map(float,_np(rig.data.joint_pos)[0])))
    v=dict(zip(rig.joint_names,map(float,_np(rig.data.joint_vel)[0])))
    if not np.all(np.isfinite(list(q.values())+list(v.values()))):raise ValueError("Nonfinite fixture telemetry")
    from measure import link_poses
    poses=link_poses(q)
    origin=_np(env.unwrapped.scene.env_origins)[0]
    xyz=_np(rig.data.body_pos_w)[0]-origin-WORLD
    for i,name in enumerate(rig.body_names):
        if name in poses:
            if np.linalg.norm(xyz[i]-poses[name][0])>.005:
                raise RuntimeError(f"Fixture pose differs from forward kinematics at {name}")
            poses[name]=(xyz[i],poses[name][1])
    left,_=native_arm_state(env,"left")
    right,limits=native_arm_state(env,"right")
    arms=[left,right]
    f=observe(q,v,poses,reader.read(),arms,bool(reader.surface_contacts),limits)
    meter.observe(f)
    if meter.fault=="wrist_overload":
        idx=rig.joint_names.index("flag_travel")
        import torch
        target=torch.full((1,1),.036,device=str(env.unwrapped.device),dtype=torch.float32)
        if hasattr(rig,"set_joint_position_target_index"):
            rig.set_joint_position_target_index(target=target,joint_ids=[idx])
        else:
            rig.set_joint_position_target(target,joint_ids=[idx])
    last_row=dict(**meter.record(f),**reader.approach.record(),**glove_geometry(poses),q=q,sleeve_pinch_substeps=reader.sleeve_steps,sleeve_pad_peak_n=reader.sleeve_pad_peak,opposed_forearm_contact=reader.opposed,robot_glove_contact=reader.glove_touch,
                  opposed_knob=reader.opposed_knob,surface_pairs=reader.surface_contacts,
                  left_outboard_m=arms[0][2],right_outboard_m=arms[1][2])
    if trace:trace.write(json.dumps(last_row)+"\n")
def score(objects):return meter.score
def success(objects):return meter.score==1.
