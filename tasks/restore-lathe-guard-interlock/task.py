"""Lathe interlock recommissioning task for the YAM station."""
import sys, os, json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import fixture as g
from camera_layout import camera_rig
from logic import Machine, Evaluator, Sample, read_encoder
from sensing import clearance
import fixture_reset
from scenekit import UsdBody, on_reset, on_step, _np, robot_links

INSTRUCTION = g.INSTRUCTION
SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.
joints = [b for b in g.BODIES if b[3]]
OBJECTS = [
    UsdBody("guard",pos=g.WORLD,size=(.93,.42,.68),usd="assets/station.usda",articulation=True,
            joint_pos=tuple((b[3],fixture_reset.START_JOINTS[b[3]]) for b in joints),
            joint_actuators=tuple((b[3],b[8],b[9]) for b in joints),
            joint_actuator_limits=tuple((b[3],.26 if b[3]=="shaft" else 100.,40. if b[3]=="shaft" else 12.) for b in joints),
            self_collisions=True,solver_iters=64),
    UsdBody("insert",pos=tuple(np.array(g.WORLD)+np.array(g.KEY)),size=g.KEY_SIZE,
            usd="assets/service_insert.usda",mass=.070,friction=(.85,.85)),
]
machine,meter = Machine(),Evaluator()
reader = trace = None

def demo():
    from reference import plan
    return plan()

reset_report = None

def close_episode():
    global reader, trace, reset_report
    if reader: reader.close()
    if trace: trace.close()
    reader = trace = reset_report = None

def initialize_fixture(env):
    global reset_report
    import torch
    base = env.unwrapped
    reset_report = {"before_fixture_reset": fixture_reset.readback(base),
                    "requested_joints": fixture_reset.START_JOINTS.copy()}
    fixture_reset.initialize(base,torch)

@on_reset
def reset(env):
    global machine, meter, reader, trace
    from contacts import Reader
    path = os.environ.get("LATHE_TRACE")
    if reset_report is None:
        raise RuntimeError("YAM did not execute the pre-settle lathe fixture reset")
    measured = fixture_reset.readback(env.unwrapped)
    failures = fixture_reset.settled_failures(measured)
    reset_report.update(after_settle=measured, failures=failures)
    fixture_reset.record(path, reset_report)
    if failures:
        raise RuntimeError("Lathe fixture invalid after settling: " + " | ".join(failures))
    machine, meter = Machine(), Evaluator()
    reader = Reader(env)
    trace = open(path,"x",encoding="utf8",buffering=1) if path else None

@on_step
def step(env):
    import torch
    base = env.unwrapped
    rig = base.scene["guard"]
    q = dict(zip(rig.joint_names,map(float,_np(rig.data.joint_pos)[0])))
    speed = float(_np(rig.data.joint_vel)[0,rig.joint_names.index("shaft")])
    insert = _np(base.scene["insert"].data.root_link_pose_w)[0,:3]-_np(base.scene.env_origins)[0]-g.WORLD
    links = {arm:{n:p-g.WORLD for n,p in poses.items()} for arm,poses in robot_links(env).items()}
    person,shield,tray,touch,pinch,dropped,exposed = reader.read()
    s = Sample(q["shield_angle"],q["cam_angle"],q["pad_z"],q["button_z"],q["shaft"],speed,
               *map(float,insert),bool(tray),bool(touch),clearance(links),float(person),float(shield),float(exposed),bool(pinch),q["seat_z"],bool(dropped))
    if meter.tick == 0:
        reset_report["first_action"] = vars(s)
        fixture_reset.record(os.environ.get("LATHE_TRACE"), reset_report)
    motor = machine.step(s)
    for kind,name,value in (("velocity","shaft",motor),("position","cam_angle",machine.detent)):
        idx = [rig.joint_names.index(name)]
        target = torch.full((1,1),value,device=str(base.device),dtype=torch.float32)
        setter = getattr(rig,"set_joint_"+kind+"_target_index",None)
        if setter: setter(target=target,joint_ids=idx)
        else: getattr(rig,"set_joint_"+kind+"_target")(target,joint_ids=idx)
    encoder = read_encoder(reader.shaft_angles, 12, s.shaft)
    meter.observe(s,machine,encoder)
    if trace:
        row = dict(tick=meter.tick,sample=vars(s),shaft_encoder_rad=encoder,score=meter.score,fault=meter.fault,
            dwell=meter.dwell,requests=machine.requests,starts=machine.starts,open_test=meter.open_test,powered=meter.powered)
        if reader.operator_peak:
            row["operator_peak"] = reader.operator_peak
        trace.write(json.dumps(row)+"\n")

def score(objects): return meter.score
def success(objects): return meter.score == 1.
