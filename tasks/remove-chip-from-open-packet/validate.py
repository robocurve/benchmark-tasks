"""Offline scorer counterexamples and official 20 Hz YAM kinematic preflight."""
from dataclasses import replace
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import design as d
from scoring import Score,Sample

ROOT=Path(__file__).resolve().parent
TABLE=.7475


def start():
    p=(*d.PACKET_XY,TABLE+.002)
    return Sample(0.,p,d.IDENTITY,tuple(tuple(np.array(p)+c) for c in d.CHIP_ORIGINS),
                  (d.IDENTITY,)*3,(0.,)*3,(0.,)*3,.1)


def move(s,index,p):
    positions=list(s.chip_pos);positions[index]=p
    return replace(s,chip_pos=tuple(positions))


def controls():
    initial=start()
    lifted=move(replace(initial,t=.05),0,(*d.PACKET_XY,TABLE+.080))
    plated=move(replace(lifted,t=.1),0,(*d.PLATE_XY,TABLE+.005))
    stable=[replace(plated,t=.1+i*.05) for i in range(25)]
    valid=[initial,lifted,*stable]
    cases={
        'untouched':([initial],0.),
        'extracted_held':([initial,lifted],.5),
        'released_on_plate':(valid,1.),
        'placed_without_observed_extraction':([initial,*stable],0.),
        'still_held':([initial,lifted,*[replace(s,hand_clearance=.01) for s in stable]],.5),
        'too_short_settle':([initial,lifted,*stable[:10]],.5),
        'moving_on_plate':([initial,lifted,*[replace(s,chip_speed=(.1,0,0)) for s in stable]],.5),
        'spinning_on_plate':([initial,lifted,*[replace(s,chip_spin=(1.,0,0)) for s in stable]],.5),
        'hovering_above_plate':([initial,lifted,*[move(s,0,(*d.PLATE_XY,TABLE+.05)) for s in stable]],.5),
        'plate_edge_overhang':([initial,lifted,*[move(s,0,(d.PLATE_XY[0]+.045,d.PLATE_XY[1],TABLE+.005)) for s in stable]],.5),
        'second_chip_removed':([initial,lifted,*[move(s,1,(.45,-.1,TABLE+.1)) for s in stable]],0.),
        'second_chip_returned':([initial,lifted,move(replace(lifted,t=.1),1,(.45,-.1,TABLE+.1)),*[replace(s,t=s.t+.05) for s in stable]],0.),
        'tipped_packet':([initial,replace(lifted,packet_quat=(1.,0.,0.,0.)),*stable],0.),
        'carried_packet':([initial,replace(lifted,packet_pos=(.5,0.,TABLE+.1)),*stable],0.),
        'dropped_below_table':([initial,move(lifted,0,(.4,-.1,TABLE-.01)),*stable],0.),
        'invalid_quaternion':([initial,replace(lifted,packet_quat=(0.,)*4),*stable],0.),
        'nan_telemetry':([initial,replace(lifted,hand_clearance=float('nan')),*stable],0.),
        'missing_frames':([initial,replace(lifted,t=.2)],0.),
        'wrong_start':([plated],0.),
    }
    result={}
    for name,(samples,expected) in cases.items():
        scorer=Score(TABLE)
        for s in samples:scorer.update(s)
        assert scorer.value==expected,(name,scorer.value,expected)
        result[name]=dict(expected=expected,actual=scorer.value)
    # Any of the three chips may be chosen. No visual identity classification.
    for index in (1,2):
        s1=move(replace(initial,t=.05),index,(*d.PACKET_XY,TABLE+.080))
        s2=move(replace(s1,t=.1),index,(*d.PLATE_XY,TABLE+.005))
        scorer=Score(TABLE)
        for s in [initial,s1,*[replace(s2,t=.1+i*.05) for i in range(25)]]:scorer.update(s)
        assert scorer.value==1.
        result[f'choose_chip_{index}']=dict(expected=1.,actual=scorer.value)
    return result


def preflight(source):
    sys.path[:0]=[str(source),str(ROOT/'runtime')]
    import task
    from yam_arena import ik
    from yam_arena.demo import ArmPlan
    from yam_arena.constants import READY
    from yam_arena.station import BASES
    for spec in task.OBJECTS:spec._check()
    results={}
    for arm,waypoints in task.demo().items():
        plan=ArmPlan(waypoints,BASES[arm],READY)
        q=np.array(READY,dtype=float);worst_mm=0.;worst_deg=0.
        times=sorted(set(np.arange(0,plan.duration,.05).tolist()+np.cumsum([w.duration for w in waypoints]).tolist()))
        for t in times:
            pos,rot,_,_=plan.target(float(t))
            q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),q,base_pos=BASES[arm],iters=60)
            worst_mm=max(worst_mm,pe*1000);worst_deg=max(worst_deg,float(np.degrees(re)))
        results[arm]=dict(duration_s=plan.duration,sampled_targets=len(times),position_mm=worst_mm,
                          orientation_deg=worst_deg,pass_3mm_5deg=bool(worst_mm<=3 and worst_deg<=5))
        print(arm,results[arm],flush=True)
    return dict(hz=20,geometry='passed',arms=results)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--yam-source',type=Path);args=parser.parse_args()
    result=dict(scorer_checks=controls(),isaac_simulation='NOT RUN',demo_score=None,dream_score=None,
                fracture_validation='NOT IMPLEMENTED',novelty='NOT CLEARED')
    if args.yam_source:result['official_preflight']=preflight(args.yam_source)
    result['source_sha256']={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in ROOT.rglob('*') if p.suffix in ('.py','.sh','.toml') and 'dist' not in p.parts}
    (ROOT/'local_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
