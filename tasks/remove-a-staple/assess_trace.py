"""Read-only geometric evidence for a recorded native panel-paper trial.

This does not certify a task score. It supplies conservative distance bounds,
material continuity evidence and measured robot clearance for the photo/video
review fields in scoring.py. A zero bound is inconclusive, not proof of contact.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from facet_paper import box_task_mesh
from materials import SteelSpec
from contact_guard import wire_chain_continuity


def box_distance(low_a, high_a, low_b, high_b):
    """Pairwise Euclidean lower bounds between shapes enclosed by AABBs."""
    gap=np.maximum(np.maximum(np.asarray(low_a)[:,None]-np.asarray(high_b)[None],
                              np.asarray(low_b)[None]-np.asarray(high_a)[:,None]),0.)
    return np.linalg.norm(gap,axis=-1)


def measured_wire(row,lengths):
    names=row['hardware_names'];poses=np.asarray(row['hardware_poses'],float)
    ids=[names.index(f'staple/wire_{i:03d}') for i in range(len(lengths))]
    poses=poses[ids];axis=Rotation.from_quat(poses[:,3:]).as_matrix()[:,:,0]
    half=axis*np.asarray(lengths)[:,None]/2
    return np.stack((poses[:,:3]-half,poses[:,:3]+half),axis=1)


def paper_boxes(nodes,faces,reconstruction_error_m=0.,thickness_m=.0001):
    triangles=np.asarray(nodes,float)[faces]
    # The reconstructed surface is at each panel's midplane. Expanding
    # every axis by half-thickness conservatively includes arbitrary bending.
    if (not np.isfinite(reconstruction_error_m) or reconstruction_error_m<0 or
            not np.isfinite(thickness_m) or thickness_m<=0):
        raise ValueError('Finite nonnegative paper reconstruction error required')
    # Shared nodes are averages of native panel corners. Include the recorded
    # maximum corner-to-average error so a numerical hinge residual cannot
    # overstate final staple clearance.
    radius=thickness_m/2+float(reconstruction_error_m)
    return triangles.min(1)-radius,triangles.max(1)+radius


def wire_boxes(segments):
    s=SteelSpec();radius=np.hypot(s.wire_width_m,s.wire_thickness_m)/2
    return segments.min(1)-radius,segments.max(1)+radius


def clearance_lower_bound(segments,papers,faces,reconstruction_errors=None,thickness_m=.0001):
    low,high=wire_boxes(segments)
    errors=[0.]*len(papers) if reconstruction_errors is None else reconstruction_errors
    if len(errors)!=len(papers):raise ValueError('One reconstruction bound per paper required')
    return min(float(box_distance(low,high,*paper_boxes(nodes,f,error,thickness_m)).min())
               for nodes,f,error in zip(papers,faces,errors))


class RobotClearance:
    def __init__(self,offset):
        from yam_arena.analysis import link_hulls
        self.hulls=link_hulls();self.offset=np.asarray(offset,float)

    def boxes(self,joints):
        from yam_arena import station
        from yam_arena.analysis import fk_link_poses_world
        from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN
        result=[]
        for side,start in (('left',0),('right',7)):
            q=dict(zip(ARM_JOINTS,joints[start:start+6]))
            q.update({name:joints[start+6]*FINGER_OPEN for name in FINGER_JOINTS})
            for name,T in fk_link_poses_world(q,station.BASES[side]).items():
                for hull in self.hulls.get(name,[]):
                    v=hull.vertices@T[:3,:3].T+T[:3,3]+self.offset
                    result.append((v.min(0),v.max(0)))
        result=np.asarray(result)
        return result[:,0],result[:,1]


def assess(directory):
    directory=Path(directory);report=json.loads((directory/'preview.json').read_text())
    lengths=report['wire_segment_lengths_m']
    from paper import paper_material
    thickness=paper_material(report.get('paper_profile','office-80gsm')).thickness_m
    spacing=report.get('panel_spacing_m',.012)
    faces=[box_task_mesh(i,spacing=spacing).faces for i in range(2)]
    robot=RobotClearance(report['world_coordinate_offset_m'])
    from audit_robot_plan import tool_props
    with gzip.open(directory/'trace.jsonl.gz','rt') as stream:
        rows=[json.loads(line) for line in stream]
    break_events=report.get('native_joint_break_events')
    native_topology_broken=bool(break_events and break_events.get('paper_hinge_break_count',0))
    # A diagnostic label is not a physical failure. Require either the original
    # strict endpoint bound or the direct sub-wire-thickness aperture model, alongside
    # all recorded crossing, wire-continuity and native fracture checks.
    continuity_supported=(report.get('paper_gap_envelope_m',.000025)<=.000025 or
        (report.get('geometry_aware_continuity') is True and
         0.<report.get('empty_aperture_limit_m',1.)<=.00025))
    samples=[];whole_run_valid=(not native_topology_broken and continuity_supported and report['status']!='invalid_model_domain' and
                               all(m['valid'] for m in report.get('material',[])))
    for row in rows:
        wire=measured_wire(row,lengths)
        ids=[row['hardware_names'].index(f'staple/wire_{i:03d}') for i in range(len(lengths))]
        chain=wire_chain_continuity(wire,poses_xyzw=np.asarray(row['hardware_poses'])[ids])
        material=bool(row['material']) and all(m['valid'] for m in row['material'])
        whole_run_valid=whole_run_valid and material and chain['valid']
        errors=[x['maximum_vertex_average_error_m'] for x in row['paper_continuity']]
        d=clearance_lower_bound(wire,row['paper_nodes'],faces,errors,thickness)
        sample=dict(step=row['step'],time_s=row['time_s'],material_valid=material,
            wire_continuity=chain,paper_reconstruction_bounds_m=errors,
            minimum_staple_to_paper_clearance_lower_bound_m=d)
        # Full robot-hull clearance is only needed for a potential complete
        # extraction. Earlier frames retain measured extraction diagnostics.
        if d>=.01 and whole_run_valid:
            robot_low,robot_high=robot.boxes(np.asarray(row['joint_pos'],float))
            paper_bounds=[paper_boxes(nodes,f,error,thickness) for nodes,f,error in zip(row['paper_nodes'],faces,errors)]
            paper_gap=min(float(box_distance(robot_low,robot_high,*p).min()) for p in paper_bounds)
            wire_gap=float(box_distance(robot_low,robot_high,*wire_boxes(wire)).min())
            index=row['hardware_names'].index('remover');pose=np.asarray(row['hardware_poses'][index])
            props=tool_props(pose[:3],Rotation.from_quat(pose[3:]).as_matrix())
            low=np.asarray([p['low'] for p in props]);high=np.asarray([p['high'] for p in props])
            tool_gap=float(box_distance(robot_low,robot_high,low,high).min())
            sample['robot_clearance_lower_bounds_m']=dict(papers=paper_gap,staple=wire_gap,remover=tool_gap)
            sample['all_objects_proven_clear_of_robot']=min(paper_gap,wire_gap,tool_gap)>.0001
        else:sample['all_objects_proven_clear_of_robot']=False
        samples.append(sample)
    suffix=[]
    for sample in reversed(samples):
        if not sample['all_objects_proven_clear_of_robot'] or not sample['material_valid']:break
        suffix.append(sample)
    release_s=suffix[0]['time_s']-suffix[-1]['time_s'] if suffix else 0.
    return dict(schema='staple-native-geometric-evidence-v1',score=None,submission_certified=False,
        run_status=report['status'],whole_run_material_valid=whole_run_valid,
        paper_thickness_m=thickness,panel_spacing_m=spacing,
        native_joint_break_events=break_events,
        native_topology_broken=native_topology_broken,
        final_clearance_lower_bound_m=samples[-1]['minimum_staple_to_paper_clearance_lower_bound_m'],
        final_released_interval_lower_bound_s=release_s,
        complete_geometric_outcome_proven=bool(report['status']=='complete' and whole_run_valid and release_s>=3.),
        samples=samples,trace_sha256=hashlib.sha256((directory/'trace.jsonl.gz').read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        review_still_required=['Starting layout and scene/source integrity','Both faces of both sheets and original puncture boundaries','Physical remover support and absence of unintended modifications'],
        limitations=['Distances are conservative enclosing-box lower bounds; zero means inconclusive.',
                     'Paper bounds include native corner-to-reconstructed-surface error. A reported paper-hinge break invalidates the intact-topology reconstruction; it is not a new physical tear rubric.',
                     'Release evidence is sampled at20Hz; continuous video and all-substep material guards must also be reviewed.',
                     'No score or human review field is automatically marked accepted.'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();result=assess(a.directory);a.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','review_still_required','limitations')},indent=2))
