"""Inspect static fits, sampled actuation, and rack clearance on the shipped STEP solids."""
import argparse
import hashlib
from itertools import combinations
import json
from pathlib import Path

import cadquery as cq

SLIDE = {"brake_tab", "locking_yoke"}
TURN = {"tap_rotor", "tap_crank"}
PRESS_FIT = {"tap_rotor", "tap_liner"}
EXPECTED = {"carriage","rack","brake_tab","locking_yoke","pin_housing","pin_housing_cap",
            "lift_neck","tap_body","tap_rotor","tap_crank","tap_liner"}


def overlap(a,b):
    A,B=a.BoundingBox(),b.BoundingBox()
    if any(getattr(A,axis+"min") >= getattr(B,axis+"max")-1e-8
           or getattr(B,axis+"min") >= getattr(A,axis+"max")-1e-8 for axis in "xyz"):
        return 0.0
    return a.intersect(b).Volume()


def scan(folder):
    paths={p.stem:p for p in sorted(folder.glob("*.step"))}
    if set(paths)!=EXPECTED:
        raise ValueError("STEP part inventory differs; classify new or missing bodies")
    parts={name:cq.importers.importStep(str(path)).val() for name,path in paths.items()}
    if not all(p.isValid() and len(p.Solids())==1 for p in parts.values()):
        raise ValueError("invalid or disconnected STEP solid")
    worst={}
    intentional=[]
    count=0

    def test(a,b,p,q,state):
        nonlocal count
        count+=1
        size=overlap(p,q)
        if size<=1e-4:
            return
        if {a,b}==PRESS_FIT:
            intentional.append(size)
            return
        key=f"{a}/{b}"
        if not key in worst or size>worst[key]["overlap_mm3"]:
            worst[key]={"overlap_mm3":round(size,5), "state":state}

    # Conservative swept rack bounding prism covers all teeth and their base at every carriage
    # height. The retracted yoke must clear the full prism. The seated pin is checked on actual teeth.
    x=parts['rack'].BoundingBox()
    rack_prism=cq.Solid.makeBox(x.xlen,x.ylen,1000,cq.Vector(x.xmin,x.ymin,-500))
    for name,p in parts.items():
        if name in {"rack"} | TURN:
            continue
        stroke=8 if name in SLIDE else 0
        test(name,"rack travel envelope",p.translate((stroke,0,0)),rack_prism,{"pin_mm":stroke})
    for name in SLIDE:
        for zero in (940,964,1060,1084):
            for stroke in range(9):
                test(name,"rack",parts[name].translate((stroke,0,0)),parts["rack"].translate((0,0,-zero)),
                     {"zero_mm":zero,"pin_mm":stroke})
    for a,b in combinations(sorted(EXPECTED-{"rack"}),2):
        sliding=(a in SLIDE) != (b in SLIDE)
        turning=(a in TURN) != (b in TURN)
        for angle in (range(0,91,5) if turning else [0]):
            for stroke in (range(9) if sliding else [0]):
                def pose(n):
                    p=parts[n]
                    if n in SLIDE:
                        return p.translate((stroke,0,0))
                    if n in TURN:
                        return p.rotate((345,-85,-50),(400,-85,-50),-angle)
                    return p
                test(a,b,pose(a),pose(b),{"pin_mm":stroke,"tap_deg":angle})
    for name in TURN:
        for deg in range(0,91,5):
            p=parts[name].rotate((345,-85,-50),(400,-85,-50),-deg)
            test(name,"rack travel envelope",p,rack_prism,{"tap_deg":deg})
    return {"kind":"CAD interference computation, no physical measurements",
            "step_sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths.values()},
            "scope":"11 manufactured solids, nominal CAD tolerances; actual fasteners, bought-in guides, hoses and flex require bench inspection. Checks simplified rigid sliding and rotary axes, finite rotary/withdrawal samples and retracted swept rack envelope.",
            "pair_pose_checks":count,"rotary_sample_step_deg":5,"pin_sample_step_mm":1,
            "ignored_ideal_face_touch_and_boolean_tolerance_mm3":0.0001,
            "intentional_rotor_liner_interference_max_mm3":round(max(intentional, default=0.0),4),
            "unexpected_positive_volume":worst,"pass":not worst}


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cad",type=Path)
    parser.add_argument("report",type=Path)
    args=parser.parse_args()
    result=scan(args.cad)
    args.report.write_text(json.dumps(result,indent=2)+"\n")
    raise SystemExit(0 if result["pass"] else 1)
