"""Offline measurements from a native diagnostic; never a policy or task score.

The recorded source defines the material patch and fixed fixture geometry.
Actual gripper poses and actual FEM nodes are read only after a run. Relative
positions are useful for diagnosing a missed grasp but do not prove retention.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import numpy as np


def rotation_xyzw(q):
    x,y,z,w=np.asarray(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial',type=Path)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--every',type=int,default=20,help='Trace samples between full triangle audits')
    p.add_argument('--side',choices=('left','right'),default='right')
    args=p.parse_args()
    if args.every<1:p.error('--every must be positive')
    source=args.trial/'task-source'
    sys.path.insert(0,str(source.resolve()))
    from cloth_mesh import edge_strain
    from fixture_clearance import fixture_intersections
    import scene
    TABLE=scene.TABLE
    fixtures=scene.support_geometry()
    if hasattr(scene,'rack_geometry'):fixtures.update(scene.rack_geometry())
    task_text=(source/'task.py').read_text()
    if 'PEG_RESTS_ENABLED=True' in task_text:
        from peg_rests import geometry
        fixtures.update(geometry())
    fixtures['Table']=dict(kind='box',center=(.36,0.,TABLE-.03),size=(1.1,1.3,.06))
    meshes=json.loads((args.trial/'meshes.json').read_text())
    mesh=meshes['tshirt'];rest=np.asarray(mesh['rest']);faces=np.asarray(mesh['faces'])
    hem_x=-.12 if args.side=='left' else .12
    patch=np.flatnonzero((abs(rest[:,0]-hem_x)<.011)&(rest[:,1]<.021))
    if not len(patch):raise ValueError('No material nodes in declared grasp patch')
    plan_path=source/'fixed-motion.json'
    frames=json.loads(plan_path.read_text())['frames'] if plan_path.exists() else []
    endpoints={i+1 for i,f in enumerate(frames) if i==len(frames)-1 or f['stage']!=frames[i+1]['stage']}
    report=json.loads((args.trial/'run.json').read_text())
    samples=[];initial=None;last=None;truncated=False
    def complete_lines(handle):
        nonlocal truncated
        while True:
            try:line=handle.readline()
            except EOFError:
                truncated=True
                return
            if not line:return
            try:json.loads(line)
            except json.JSONDecodeError:
                truncated=True
                return
            yield line
    with gzip.open(args.trial/'trace.jsonl.gz','rt') as handle:
        for line in complete_lines(handle):
            row=json.loads(line);step=row['step'];last=row
            if step%args.every and step not in endpoints and step!=report['completed_steps']:continue
            from trace_data import cloth_array
            nodes=cloth_array(args.trial,row,'tshirt');mean=nodes[patch].mean(0)
            if initial is None:initial=mean.copy()
            robot=row['robot_links'][args.side+'_robot']
            grip=np.asarray(robot['poses_xyzw'][robot['names'].index('gripper')])
            rotation=rotation_xyzw(grip[3:]);tcp=grip[:3]+rotation@np.array((0.,0.,.134))
            strain=np.abs(edge_strain(rest,nodes,faces))
            hits=fixture_intersections(nodes,faces,fixtures,margin=-.00005)
            samples.append(dict(step=step,time_s=row['time_s'],stage='reset' if step==0 else frames[step-1]['stage'] if frames else 'ready',
                cloth_bounds_m=dict(low=nodes.min(0).tolist(),high=nodes.max(0).tolist()),
                patch_mean_m=mean.tolist(),patch_displacement_m=(mean-initial).tolist(),
                layer_means_m={layer:nodes[patch[rest[patch,2]*sign>0]].mean(0).tolist() for layer,sign in [('front',1),('back',-1)]},
                actual_tcp_m=tcp.tolist(),patch_relative_to_gripper_m=((mean-tcp)@rotation).tolist(),
                patch_tcp_distance_m=float(np.linalg.norm(mean-tcp)),
                p99_abs_edge_strain=float(np.quantile(strain,.99)),max_abs_edge_strain=float(strain.max()),
                fixture_intersecting_triangle_counts={name:len(values) for name,values in hits.items()}))
    output=dict(kind='offline_native_motion_measurements_not_score',run_status=report['status'],truncated_trace=truncated,last_complete_trace_step=None if last is None else last['step'],fixture_audit_scope=list(fixtures),
        completed_steps=report['completed_steps'],observed_hand=args.side,material_patch=dict(hem_x_m=hem_x,max_longitudinal_m=.021,half_width_m=.011,nodes=len(patch)),
        source_trace_sha256=hashlib.sha256((args.trial/'trace.jsonl.gz').read_bytes()).hexdigest(),
        samples=samples,retained_grasp=None,score=None,
        limitation='Sampled assessor measurements only. Visual contact and released hold require separate review; proximity alone is not grasp success.')
    args.out.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(dict(samples=len(samples),status=report['status'],
        max_fixture_triangle_hits=max((sum(s['fixture_intersecting_triangle_counts'].values()) for s in samples),default=0),
        max_patch_lift_m=max((s['patch_displacement_m'][2] for s in samples),default=0.)),indent=2))


if __name__=='__main__':main()
