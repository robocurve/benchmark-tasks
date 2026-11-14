"""Compile bounded joint-target feedforward from a previous physical trial.

This is ordinary iterative learning for a scripted demonstration. The reference
nozzle/piston recipe, scene, actuator limits and scoring do not change. A prior
command-minus-measured-joint offset estimates motor load; no material score is
read. The resulting frozen plan records its calibration source hashes.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def compile_feedforward(plan, trace, *, limit=.16, anticipate_tail_load=False):
    frames=plan['frames']
    if len(trace)!=len(frames)+1:
        raise ValueError('Calibration must contain the complete matching trial')
    offsets=np.zeros((len(frames),14))
    active=np.array(['scripted_tool_target' in f for f in frames])
    for i,(frame,state) in enumerate(zip(frames,trace[1:])):
        if state.get('step')!=i+1:
            raise ValueError('Non-contiguous calibration trace')
        if not np.allclose(frame['action14'],state['reference_action14'],atol=1e-12,rtol=0):
            raise ValueError('Calibration reference differs from the frozen plan')
        applied=np.asarray(state['applied_action14'],float)
        observed=np.asarray(state['observed_action14'],float)
        if applied.shape!=(14,) or observed.shape!=(14,) or not np.isfinite(np.r_[applied,observed]).all():
            raise ValueError('Invalid calibration joint data')
        # Keep the pressure hand unchanged; compensate only the barrel hand.
        offsets[i,7:13]=applied[7:13]-observed[7:13]
    filtered=np.zeros_like(offsets)
    for i in range(len(frames)):
        if not active[i]:continue
        indices=np.arange(max(0,i-2),min(len(frames),i+3))
        indices=indices[active[indices]]
        filtered[i]=np.mean(offsets[indices],axis=0)
        # Ramp at the boundaries of each physically calibrated tool phase.
        previous=0
        for j in range(i,-1,-1):
            if not active[j]:break
            previous+=1
            if previous==5:break
        following=0
        for j in range(i,len(frames)):
            if not active[j]:break
            following+=1
            if following==5:break
        filtered[i]*=min(previous,following,5)/5
    filtered=np.clip(filtered,-limit,limit)
    if anticipate_tail_load:
        # The calibration stalls early in each tail while its feedback builds
        # sufficient motor load. Use the measured moving half's median offset
        # at the start of that same fixed phase; do not repeat the learned lag.
        i=0
        while i<len(frames):
            if frames[i].get('phase')!='draw tapered shell tail':i+=1;continue
            end=i+1
            while end<len(frames) and frames[end].get('phase')=='draw tapered shell tail':end+=1
            load=np.clip(np.median(offsets[i+(end-i)//2:end],axis=0),-limit,limit)
            for j in range(i,end):
                blend=min((j-i+1)/2,1.)
                filtered[j]=(1-blend)*filtered[j]+blend*load
            i=end
    result=json.loads(json.dumps(plan))
    for frame,offset in zip(result['frames'],filtered):
        frame['joint_feedforward14']=offset.tolist()
    result['joint_feedforward']={'method':'five-sample-smoothed previous applied-minus-measured right-arm joint offsets',
        'bound_rad':limit,'changes_motor_settings':False,'uses_material_or_score':False,
        'anticipate_tail_load':anticipate_tail_load}
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True)
    p.add_argument('--trace',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--anticipate-tail-load',action='store_true')
    a=p.parse_args()
    result=compile_feedforward(json.loads(a.plan.read_text()),json.loads(a.trace.read_text()),anticipate_tail_load=a.anticipate_tail_load)
    result['joint_feedforward']['calibration_sha256']={
        'plan':hashlib.sha256(a.plan.read_bytes()).hexdigest(),
        'trace':hashlib.sha256(a.trace.read_bytes()).hexdigest()}
    a.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['joint_feedforward']))
