"""Isolate the guard interlock and spindle response on the frozen native USD in CPU PhysX."""
import argparse
import hashlib
import json
from math import degrees, radians
from pathlib import Path
import sys
import tempfile
import numpy as np
import ovstage
from ovphysx import PhysX
from ovphysx.types import SimObjectType as Type
from ovphysx.population import physics_population_descs
from ovphysx.samples.python_samples.schema_support import physx_schema_paths

TASK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TASK))
from fixture import WORLD, SHIELD_UP, KEY, TRAY
from logic import Machine, Sample

def joints(sim):
    q={}
    with sim.read_sync(Type.ARTICULATION_JOINT,["jointPosition"]) as op:
        for group in op.groups:
            names=op.dictionary.get_path_strings(group.prim_list)
            q.update((n.rsplit("/",1)[-1],float(v)) for n,v in zip(names,group.tensors[0].numpy().reshape(-1)))
    for name in ("shield_angle","cam_angle","shaft"): q[name]=radians(q[name])
    return q

def write(sim,attr,values):
    for name,v in values.items():
        with sim.write_sync(Type.ARTICULATION_JOINT,attr,pattern="/Rig/joints/"+name) as op:
            if len(op.groups)!=1:raise RuntimeError("No joint match for "+name)
            group=op.groups[0]
            value=degrees(v) if name in ("shield_angle","cam_angle","shaft") else v
            group.tensors[0].assign(np.full(group.tensors[0].shape,value,np.float32))
            op.commit_sync(group)
def cycle(mode,output):
    asset=TASK/"assets/station.usda"
    layer=f'''#usda 1.0
(metersPerUnit=1
 upAxis="Z")
def PhysicsScene "world" {{
 vector3f physics:gravityDirection=(0,0,-1)
 float physics:gravityMagnitude=9.81
}}
def Xform "Rig" (prepend references=@{asset.as_posix()}@) {{
 double3 xformOp:translate={WORLD}
 uniform token[] xformOpOrder=["xformOp:translate"]
}}
'''
    xyz=KEY if mode=="faulted" else TRAY
    pos=tuple(float(a+b) for a,b in zip(WORLD,xyz))
    insert=TASK/"assets/service_insert.usda"
    layer+=f'''def Xform "Insert" (prepend references=@{insert.as_posix()}@) {{
 double3 xformOp:translate={pos}
 uniform token[] xformOpOrder=["xformOp:translate"]
}}
'''
    rows=[]
    machine=Machine()
    with tempfile.TemporaryDirectory(dir=output) as tmp:
        file=Path(tmp)/"world.usda"
        file.write_text(layer)
        stage=ovstage.Stage(mode)
        ovstage.population.open_usd_with_desc(stage,str(file),ordinal=1,time=ovstage.population.DEFAULT_TIME,descs=physics_population_descs(rendering=True))
        stage.advance_write_floor(ordinal=1).wait()
        sim=PhysX()
        sim.wait_task(sim.attach_ovstage(stage,read_ordinal=1))
        try:
            write(sim,"jointPosition",{"shield_angle":0. if mode=="closed" else SHIELD_UP})
            for tick in range(70):
                q=joints(sim)
                s=Sample(q["shield_angle"],q["cam_angle"],q["pad_z"],-.009 if tick==20 else 0.,
                    q["shaft"],0.,0.,0.,0.,False,False,0.,0.,0.,0.,False,q["seat_z"],False)
                target=machine.step(s)
                write(sim,"jointVelocityTarget",{"shaft":target})
                rows.append(dict(tick=tick,permit=machine.permit,starts=machine.starts,**q))
                for _ in range(12): sim.step_sync(1/240)
        finally:
            sim.wait_task(sim.detach_ovstage())
            sim.destroy()
            stage.destroy()
    start=rows[20]
    angles=np.unwrap([q["shaft"] for q in rows])
    travel=float(np.max(np.abs(angles-angles[20])))
    expected=mode!="inhibited"
    valid=(machine.requests==1 and machine.starts==int(expected) and
           ((travel>6.28) if expected else (travel<.05)) and
           (abs(start["shield_angle"])<.044 if mode=="closed" else start["shield_angle"]>.69))
    summary=dict(mode=mode,passed=bool(valid),engine="CPU PhysX fixture only; configured initial pose and electrical start pulse",
                 tongue_mm=-1000*start["seat_z"],pad_mm=1000*start["pad_z"],guard_deg=degrees(start["shield_angle"]),shaft_turns=travel/(2*np.pi),
                 requests=machine.requests,starts=machine.starts,
                 usd_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (asset,insert)})
    (output/(mode+".json")).write_text(json.dumps(summary,indent=2))
    (output/(mode+".jsonl")).write_text("".join(json.dumps(row)+"\n" for row in rows))
    print(summary,flush=True)
    return valid

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("out",type=Path)
    args=p.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    ovstage.population.register_usd_schemas([str(v) for v in physx_schema_paths()])
    PhysX.set_cpu_mode(True)
    if not all([cycle(m,args.out) for m in ("inhibited","faulted","closed")]):raise SystemExit(2)
