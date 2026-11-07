"""Recompute outcome/clearance evidence from measured integrated-scene traces.

For historical runs invoke the copy preserved in that run's source/ directory.
This audit never changes simulator state or generates a success trajectory.
"""
import argparse,json
from pathlib import Path
import numpy as np
from yam_arena.task_file import ObjectState
from yam_arena.ik import tcp_pose
from yam_arena.station import BASES
from geometry import SIZES,FILM_NAMES,N,CARRIER_POS,STRIP_SIZE,TAB_SEAM,corners,rotation
from tool_geometry import TOOL_SIZE,TOOL_NAMES,ALL_TOOL_NAMES,TOOLS,TIP_SIZE,tool_objects,return_metrics,box_penetration
from scoring import Trial
from fixture_geometry import tab_parts


def states(row):
    sizes=dict(SIZES,**{o.name:np.array(o.size) for o in tool_objects()})
    sizes.update({n:TOOL_SIZE for n in ALL_TOOL_NAMES})
    result={}
    for name,s in row['objects'].items():
        obj=ObjectState(np.array(s['pos']),np.array(s['quat']),np.array(s['vel']),
                        np.array(s['ang_vel']),np.zeros(3),np.zeros(3))
        points=corners(obj,sizes[name])
        result[name]=ObjectState(obj.pos,obj.quat_xyzw,obj.lin_vel,obj.ang_vel,points.min(0),points.max(0))
    return result


def clearance(row,objects):
    distances=[]
    for arm,q in [('left',row['joint_pos'][:6]),('right',row['joint_pos'][7:13])]:
        tcp,_=tcp_pose(q,BASES[arm])
        distances.extend(float(np.linalg.norm(tcp-np.clip(tcp,s.low,s.high))) for s in objects.values())
    return min(distances)


def audit(folder,epoch):
    summary=json.loads((folder/f'{epoch}-summary.json').read_text())
    rows=json.loads((folder/f'{epoch}-trace.json').read_text());trial=Trial();mismatches=[];measurements=[]
    for row in rows:
        o=states(row);gap=clearance(row,o);trial.observe(o,hands_clear=gap>=.08)
        seam=0.
        for i in range(N):
            a,b=o[FILM_NAMES[i]],o[FILM_NAMES[i+1]]
            anchor=TAB_SEAM if i==0 else np.array([0,-STRIP_SIZE[1]/2,0])
            seam=max(seam,float(np.linalg.norm(a.pos+rotation(a.quat_xyzw)@anchor
                           -b.pos-rotation(b.quat_xyzw)@[0,STRIP_SIZE[1]/2,0])))
        trial.invalid=trial.invalid or seam>.003
        trial.peeled=int(row['info']['peeled_strips']);trial.finish_step(o,.05)
        score=trial.score(o)
        if score!=row['info']['score']:mismatches.append(dict(step=row['step'],recomputed=score,recorded=row['info']['score']))
        carrier=o['carrier'];r=rotation(carrier.quat_xyzw)
        penetration=0.
        if row['right_phase'] in ('pinch tab','lift tab from slot','peel film'):
            tab=np.array(row['root_poses']['pull_tab']);rt=rotation(tab[3:])
            for name in TOOL_NAMES:
                tool=np.array(row['root_poses'][name]);rr=rotation(tool[3:])
                for part in tab_parts():
                    penetration=max(penetration,box_penetration(tool[:3]+rr@[0,0,-.048],rr,TIP_SIZE,
                                    tab[:3]+rt@part.center,rt,part.size)*1000)
        measurements.append(dict(step=row['step'],carrier_xy_mm=float(np.linalg.norm((carrier.pos-CARRIER_POS)[:2])*1000),
            carrier_z_mm=float(abs(carrier.pos[2]-CARRIER_POS[2])*1000),
            carrier_tilt_deg=float(np.degrees(np.arccos(np.clip(r[2,2],-1,1)))),
            carrier_yaw_deg=float(abs(np.degrees(np.arctan2(r[1,0],r[0,0])))),
            seam_mm=seam*1000,clearance_mm=gap*1000,tip_tab_penetration_mm=penetration,
            tool_return={k:return_metrics(o,k) for k in TOOLS},recomputed_score=score,settled_seconds=trial.settled_seconds))
    result=dict(source_sha256=summary['source_sha256'],source_unchanged=summary['source_unchanged'],
                steps=len(rows),continuous_steps=[r['step'] for r in rows]==list(range(len(rows))),
                score_mismatches=mismatches,final=measurements[-1],
                maxima={key:max(m[key] for m in measurements) for key in
                        ('carrier_xy_mm','carrier_z_mm','carrier_tilt_deg','carrier_yaw_deg','seam_mm','tip_tab_penetration_mm')})
    result['passed']=bool(result['source_unchanged'] and result['continuous_steps'] and not mismatches)
    if summary['mode']=='demo':result['passed'] &= bool(summary['success'] and measurements[-1]['recomputed_score']==1
                                                       and result['maxima']['tip_tab_penetration_mm']<=.1)
    elif summary['mode'] in ('hold','press','shear','direct','decoy'):
        result['passed'] &= all(r['info']['score']==0 and r['info']['peeled_strips']==0 for r in rows)
    elif summary['mode']=='single':result['passed'] &= summary['score']==0
    (folder/f'{epoch}-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    (folder/f'{epoch}-measurements.json').write_text(json.dumps(measurements))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);p.add_argument('--epoch',type=int,default=0)
    args=p.parse_args();result=audit(args.folder,args.epoch)
    print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
