"""Read-only shape, nipple and motion evidence from a native robot trace.

No target nipple angle is used as a success criterion. Shape measurements
refer to the actual front rim surface and axle, including sector rotations.
They support the released-revolution review; this module never certifies it.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np
from thread_coupling import rotation_batch
from rim_geometry import sector_mesh,maximum_radial_chord_error_m


def rim_geometry(poses,hub):
    poses=np.asarray(poses,float);hub=np.asarray(hub,float)
    rotations=rotation_batch(poses[:,3:]);front=[];back=[]
    for i,(pose,R) in enumerate(zip(poses,rotations)):
        vertices,_=sector_mesh(index=i)
        vertices=vertices@R.T+pose[:3]
        front.append(vertices[2::4]);back.append(vertices[1::4])
    front=np.vstack(front);back=np.vstack(back)
    centre=(front+back)/2
    radial=np.linalg.norm(centre[:,1:]-hub[None,1:3],axis=1)
    # The straight collider edges can dip below their circular vertices by at
    # most this known chord error. Include it in the upper runout bound.
    chord=maximum_radial_chord_error_m()
    return dict(lateral_runout_m=float(np.ptp(front[:,0])),
                radial_runout_upper_bound_m=float(np.ptp(radial)+chord),
                dish_offset_m=float(abs(np.mean(centre[:,0])-hub[0])),
                radial_chord_error_bound_m=chord)


def assess(directory):
    directory=Path(directory);report=json.loads((directory/'preview.json').read_text())
    with gzip.open(directory/'trace.jsonl.gz','rt') as stream:rows=[json.loads(line) for line in stream]
    samples=[]
    for row in rows:
        names=row['body_names'];poses=np.asarray(row['body_poses'],float)
        ids=[names.index(f'wheel/rim_{i:02d}') for i in range(32)]
        hub=poses[names.index('wheel/hub'),:3]
        centre=poses[ids[0],:3]-hub
        values=rim_geometry(poses[ids],hub)
        materials=row['materials'];tension=[m['tension_n'] for m in materials]
        values.update(step=row['step'],time_s=row['time_s'],stage=row['stage'],
            marker_angle_rad=float(np.arctan2(centre[2],centre[1])),
            spoke_tensions_n=tension,nipple_turns=[m['turns'] for m in materials],
            minimum_tension_n=min(tension),maximum_tension_n=max(tension),
            angular_momentum_kg_m2_s=row['momentum']['angular_momentum_kg_m2_s'],
            tool_servo=row.get('tool_pose_servo'))
        samples.append(values)
    angles=np.unwrap([s['marker_angle_rad'] for s in samples])
    for sample,angle in zip(samples,angles):sample['marker_unwrapped_angle_rad']=float(angle)
    final=samples[-1]
    return dict(schema='wheel-native-geometric-evidence-v1',score=None,submission_certified=False,
        status=report['status'],completed_steps=report['completed_steps'],
        rim_integrity=report.get('rim_integrity'),final_geometry={k:final[k] for k in
        ('lateral_runout_m','radial_runout_upper_bound_m','dish_offset_m','minimum_tension_n','maximum_tension_n')},
        total_marker_excursion_rad=float(np.ptp(angles)),samples=samples,
        trace_sha256=hashlib.sha256((directory/'trace.jsonl.gz').read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        review_required=['Complete released revolution with hands and tool clear','Only nipple adjustment; no direct rim bending or external bracing','No damage to wheel, spokes, tool or stand'],
        interpretation='Measured shape and material evidence only. Total marker motion can include robot-held rotation and is not a released-revolution certificate.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();result=assess(a.directory);a.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','review_required')},indent=2))
