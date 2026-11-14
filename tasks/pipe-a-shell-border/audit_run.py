"""Independently score saved material and a complete physical trial record.

No intended path, shell counter or commanded dispenser pose earns credit.
Native photographs/video remain necessary for agreement with simple judging.
"""
import argparse
import itertools
import json
import hashlib
from pathlib import Path
import numpy as np
from native import rotation
from scoring import assess


def distance(a,b):
    a,b=np.asarray(a,float),np.asarray(b,float)
    if a.shape!=(2,3) or b.shape!=(2,3) or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('Finite measured bounds are required')
    if np.any(a[1]<a[0]) or np.any(b[1]<b[0]):raise ValueError('Inverted measured bounds')
    return float(np.linalg.norm(np.maximum(0.,np.maximum(a[0]-b[1],b[0]-a[1]))))


def tool_corners(row):
    # Conservative envelope includes the barrel and complete metal nozzle.
    local=np.asarray(list(itertools.product((-.015,.015),(-.040,.080),(-.015,.015))))
    pose=np.asarray(row['barrel_pose_xyzw'],float)
    return local@rotation(pose[3:]).T+pose[:3]


def release_evidence(trace,paths):
    result={'released_clear':False,'reason':'missing complete one-second release evidence'}
    if len(trace)<21 or set(paths)!= {'barrel','plunger'}:return result
    recent=trace[-21:]
    if recent[-1].get('time_s',0)-recent[0].get('time_s',0)<1.-1e-8:return result
    minimum_robot=minimum_border=float('inf');motion=0.;first=None
    border=np.array(((.226,-.047,.8005),(.244,.047,.8085)))
    for row in recent:
        bounds=row.get('rigid_bounds_m',{})
        if not all(p in bounds for p in paths.values()) or len(bounds)<20:return result
        corners=tool_corners(row)
        barrel=np.stack((corners.min(0),corners.max(0)))
        plunger=np.asarray(bounds[paths['plunger']],float)
        points=np.vstack((corners,plunger))
        if not np.isfinite(points).all():return result
        if first is None:first=points.copy()
        motion=max(motion,float(np.linalg.norm(points-first,axis=1).max()))
        for tool in (barrel,plunger):
            minimum_border=min(minimum_border,distance(tool,border))
            for name,robot in bounds.items():
                if name not in paths.values():minimum_robot=min(minimum_robot,distance(tool,robot))
    result.update(minimum_robot_clearance_mm=minimum_robot*1000,
                  minimum_border_clearance_mm=minimum_border*1000,
                  maximum_tool_motion_mm=motion*1000,
                  released_clear=minimum_robot>=.005 and minimum_border>=.020 and motion<=.002,
                  reason='Conservative measured tool/robot bounds and one-second retention')
    return result


def audit(directory):
    report=json.loads((directory/'preview.json').read_text())
    trace=json.loads((directory/'trace.json').read_text())
    n=report.get('completed_control_steps',0)
    complete=(report.get('status')=='complete' and n==report.get('preview_steps')
              and len(trace)==n+1 and [r.get('step') for r in trace]==list(range(n+1)))
    released=release_evidence(trace,report.get('tool_body_paths',{}))
    invalid=(report.get('other_contact',{}).get('invalid',True)
             or any(r.get('material_contact_invalid',True) for r in trace))
    with np.load(directory/'material.npz',allow_pickle=False) as state:
        result=assess(state['x'],state['y'],state['volume'],dx=float(state['dx']),
            emitted_m3=float(state['emitted_m3']),lost_m3=float(state['lost_m3']),
            airborne_m3=float(state['airborne_m3']),damage_depth_m=float(state['damage_depth_m']),
            completed=bool(complete),tools_clear=bool(released['released_clear']),other_tool_contact=bool(invalid))
    return {'complete_record':bool(complete),'release':released,'assessment':result,
            'scoring_sha256':hashlib.sha256(Path(__file__).with_name('scoring.py').read_bytes()).hexdigest(),
            'audit_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'photo_agreement':'Requires independent review of the matching native images and video'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path);parser.add_argument('--out',type=Path)
    args=parser.parse_args();data=json.dumps(audit(args.directory),indent=2)+'\n'
    if args.out:args.out.write_text(data)
    else:print(data,end='')
