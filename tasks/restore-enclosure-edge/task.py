"""Powered enclosure guard with a turned-out clip-base edge and calibrated reversal probe."""
import sys,os,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from fixture import WORLD,BODIES,INSTRUCTION
from logic import Machine,Evaluator,Sample
from scenekit import UsdBody,on_reset,on_step,_np,robot_links,arm_joints
from yam_arena.constants import FINGER_OPEN
SCENE="station"
SCORE_OVER="end"
SUCCESS_HOLD_S=0.
joints=[b for b in BODIES if b[3]]
OBJECTS=[UsdBody("guard",pos=WORLD,size=(.465,.516,.240),usd="assets/guard.usda",articulation=True,
                 joint_pos=tuple((b[3],0.) for b in joints),joint_actuators=tuple((b[3],b[8],b[9]) for b in joints),
                 joint_actuator_limits=tuple((b[3],120.,12.) for b in joints),self_collisions=True,solver_iters=64)]
machine=Machine()
meter=Evaluator()
reader=trace=last_row=None
def demo():
    from reference import plan
    return plan()
@on_reset
def reset(env):
    global machine,meter,reader,trace,last_row
    from contacts import Reader
    if reader:reader.close()
    if trace:trace.close()
    machine,meter,last_row=Machine(),Evaluator(),None
    reader=Reader(env)
    path=os.environ.get("EDGE_TRACE")
    trace=open(path,"x",encoding="utf8",buffering=1) if path else None
@on_step
def step(env):
    global last_row
    import torch
    rig=env.unwrapped.scene["guard"]
    q=dict(zip(rig.joint_names,map(float,_np(rig.data.joint_pos)[0])))
    edge,touch,sweep,operator,peak=reader.read()
    links=robot_links(env)
    parked=all(all(sign*links[side][n][1]>.202 for n in ("tip_left","tip_right","gripper"))
               and min(arm_joints(env,side,("joint7","joint8"))/FINGER_OPEN)>.78
               for side,sign in (("left",1),("right",-1)))
    s=Sample(**q,probe_edge=edge,robot_touch=touch,hazardous_contact=sweep and machine.phase in ("closing","reversing","return_limit"),
             parked=bool(parked),peak_compression=peak,operator_contact=operator)
    targets=machine.step(s)
    for name,value in targets.items():
        index=rig.joint_names.index(name)
        target=torch.full((1,1),value,device=str(env.unwrapped.device),dtype=torch.float32)
        if hasattr(rig,"set_joint_position_target_index"):rig.set_joint_position_target_index(target=target,joint_ids=[index])
        else:rig.set_joint_position_target(target,joint_ids=[index])
    meter.observe(s,machine)
    last_row=dict(sample=vars(s),phase=machine.phase,cycle=machine.cycle,tick=meter.tick,
                  score=meter.score,fault=meter.fault,dwell=meter.dwell,signal=machine.signal)
    if trace:trace.write(json.dumps(last_row)+"\n")
def score(objects):return meter.score
def success(objects):return meter.score==1.
