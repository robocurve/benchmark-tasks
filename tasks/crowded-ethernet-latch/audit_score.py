"""Independently recompute the Ethernet score from the saved native poses."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from config import CLIP_Z, SERVICE, TARGET, USED, PLUG_X, port
from feedback import retaining_face
from scoring import EthernetScorer
from yam_arena.urdf import quat_xyzw_to_matrix


def audit(root):
    root=Path(root)
    outcome=json.loads((root/'result.json').read_text())
    if outcome.get('external_bench_forces') is True or outcome.get('task_score')!=1:
        raise ValueError('Successful scored run required')
    scorer=EthernetScorer()
    trace=root/'trajectory.jsonl'
    digest=hashlib.sha256()
    events={}
    peak=0.
    biggest_neighbor=0.
    min_neighbor_depth=1.
    last=None
    minfinalclear=float('inf')
    finalrows=0
    source=np.asarray(port(SERVICE));target=np.asarray(port(TARGET))
    for i,raw in enumerate(trace.open('rb')):
        digest.update(raw);row=json.loads(raw)
        t=row['time']
        assert abs(t-(i+1)/20)<1e-6
        assert row.get('pull_N',0)==row.get('press_N',0)==0
        cmd=np.asarray(row['action_14d'])
        assert cmd.shape==(14,) and np.isfinite(cmd).all()
        if 'poses' in row:
            poses=row['poses']
            plugs={j:np.asarray(poses[f'plug_{j}']) for j in USED}
            clips={j:np.asarray(poses[f'clip_{j}']) for j in USED}
        else:
            plugs={j:np.asarray(row['plug_poses'][str(j)]) for j in USED}
            clips={j:np.asarray(row['clip_poses'][str(j)]) for j in USED}
        plug=plugs[SERVICE];clip=clips[SERVICE]
        r=quat_xyzw_to_matrix(plug[3:]);tip=plug[:3]+.012*r[:,0]
        travel=CLIP_Z-(r.T@(clip[:3]-plug[:3]))[2]
        def ins(mid):
            return tip[0]-.3985 if np.linalg.norm(plug[1:3]-mid[1:3])<.007 and r[0,0]>.8 else -.020
        neighbour=max(max(0.,PLUG_X-plugs[j][0]) for j in USED if j!=SERVICE)
        min_neighbor_depth=min(min_neighbor_depth,min(plugs[j][0]+.012*quat_xyzw_to_matrix(plugs[j][3:])[0,0]-.3985 for j in USED if j!=SERVICE))
        hands=row['native_hands']
        mesh=row['hands_mesh_clearance']
        tcp_gap=min(np.linalg.norm(np.array(h['tcp'])-plug[:3]) for h in hands)
        clear=tcp_gap>.070 and mesh is not None and mesh['minimum_gap_m']>=.005
        sample=dict(time=t,clip_travel=travel,source_insertion=ins(source),target_insertion=ins(target),
            target_lateral_error=np.linalg.norm(plug[1:3]-target[1:3]),target_angle=np.arccos(np.clip(r[0,0],-1.,1.)),
            speed=0 if last is None else 20*np.linalg.norm(plug[:3]-last),height=plug[2],
            max_neighbour_withdrawal=neighbour,neighbour_median_depth=float(np.median([plugs[j][0]+.012*quat_xyzw_to_matrix(plugs[j][3:])[0,0]-.3985 for j in USED if j!=SERVICE])),hands_clear=clear,hook_retained=retaining_face(clip,TARGET)['retained'])
        last=plug[:3].copy()
        metrics=scorer.update(sample)
        assert scorer.invalid is None
        assert abs(metrics['score']-row['metrics']['score'])<=1e-9
        if scorer.out:events.setdefault('source_cleared_s',t)
        if scorer.tug_start is not None:events.setdefault('tug_start_s',scorer.tug_start)
        if scorer.value>=.5:events.setdefault('seated_s',t)
        if scorer.hold>0:finalrows+=1;minfinalclear=min(minfinalclear,mesh['minimum_gap_m'])
        else:finalrows=0;minfinalclear=float('inf')
        if scorer.value==1:events.setdefault('full_score_s',t)
        peak=max(peak,travel);biggest_neighbor=max(biggest_neighbor,neighbour)
    assert scorer.value==1. and scorer.passed_tug and finalrows>=40
    neighbors=[j for j in USED if j!=SERVICE]
    return dict(score=scorer.value,trace_sha256=digest.hexdigest(),steps=i+1,control_hz=20,
        events=events,hands_off_hold_s=round(scorer.hold,5),tug_movement_mm=round(1000*(scorer.tug_depth0-scorer.tug_min),4),tug_attempts=scorer.tug_attempts,peak_clip_travel_mm=round(1000*peak,4),
        final_clip_travel_mm=round(1000*travel,4),target_depth_mm=round(1000*sample['target_insertion'],3),
        target_lateral_error_mm=round(1000*sample['target_lateral_error'],3),
        target_axis_error_deg=round(np.degrees(sample['target_angle']),3),
        final_gripper_clearance_mm=round(1000*minfinalclear,3),maximum_neighbour_withdrawal_mm=round(1000*biggest_neighbor,3),
        minimum_neighbour_insertion_mm=round(1000*min_neighbor_depth,3),
        final_neighbour_latches={str(j):bool(retaining_face(clips[j],j)['retained']) for j in neighbors},
        final_neighbour_lateral_error_mm={str(j):round(1000*float(np.linalg.norm(plugs[j][1:3]-np.asarray(port(j))[1:3])),3) for j in neighbors},
        policy_external_forces=False,scorer_tug_N=3.0,passed=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run');a=p.parse_args();res=audit(a.run)
    print(json.dumps(res,indent=2))
