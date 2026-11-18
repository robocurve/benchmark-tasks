"""CPU material probe. Prescribed nozzle motion is NOT a robotic demonstration."""
import argparse
import json
import hashlib
from pathlib import Path
import time
import numpy as np
from material import Icing
from scoring import assess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--dx",type=float,default=.002)
    a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    t=time.monotonic()
    m=Icing(dx=a.dx)
    down=np.diag((1.,-1.,-1.))
    trace=[]
    # One pressure rise, dwell and taper. This is a constitutive-model probe,
    # deliberately not a six-shell target-following script or claimed demo.
    for step in range(240):
        elapsed=step/240
        piston=.012*min(elapsed/.25,1.) if elapsed<.55 else max(0.,.012-(elapsed-.55)*.06)
        y=-.015+.03*elapsed
        reaction=m.step(1/240,piston_m=piston,nozzle_pos=(.235,y,.8065),nozzle_rotation=down)
        if step%12==0:trace.append(dict(t=elapsed,piston_m=piston,reaction_n=reaction,
                                      pressure_pa=m.pressure_pa,flow_ml_s=m.last_flow_m3_s*1e6))
    for _ in range(120):m.step(1/240,piston_m=0.,nozzle_pos=(.235,.015,.83),nozzle_rotation=down)
    m.save(a.out/"material.npz")
    report={"status":"CPU constitutive probe only; not acceptance evidence",
            "robot_demo":False,"physics_seconds":m.time_s,"wall_seconds":time.monotonic()-t,
            "source_sha256":{name:hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                             for name in ('probe.py','material.py','scoring.py')},
            "parameters":vars(m.p),"cell_size_m":a.dx,"balance":m.balance(),"trace":trace,
            "draft_assessment":assess(m.x,m.y,m.volume,dx=m.dx,emitted_m3=m.emitted_m3,
                                       lost_m3=m.lost_m3,airborne_m3=float(m.air_volume.sum()),
                                       damage_depth_m=m.cake_damage_depth_m)}
    (a.out/"probe.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps({k:v for k,v in report.items() if k not in ("trace","draft_assessment")},indent=2))


if __name__=="__main__":main()
