"""Unscored native fit/retention measurements. No simulation state writes."""
import math
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from config import CLIP_Z,CLIP_TRAVEL,USED,port

FIT_MAX_PITCH_DEG=3.5
FIT_MAX_CLIP_TRAVEL=.0002


def bench_command(step,mode):
    if mode=='fit-probe':return 'untouched_fit',0.,0.
    if mode not in ('retention-probe','clip-constraint-probe'):return 'robot',0.,0.
    phase='settle' if step<40 else 'pull_engaged' if step<60 else 'press_clip' if step<80 else 'pull_pressed' if step<120 else 'release'
    load=.50 if mode=='clip-constraint-probe' else .70
    return phase,(-1. if phase in ('pull_engaged','pull_pressed') else 0.),(-load if phase in ('press_clip','pull_pressed') else 0.)


def offcenter_clip_wrench(plug,clip,press):
    """Diagnostic force at the actual ordinary tab; never used in robot mode."""
    force=quat_xyzw_to_matrix(np.asarray(plug[3:]))@np.array((0.,0.,press))
    lever=quat_xyzw_to_matrix(np.asarray(clip[3:]))@np.array((-.010,-.0015,.0005))
    return force,np.cross(lever,force)


def clip_joint_geometry(plug,clip):
    body=quat_xyzw_to_matrix(np.asarray(plug[3:]));relative=body.T@quat_xyzw_to_matrix(np.asarray(clip[3:]))
    local=body.T@(np.asarray(clip[:3])-np.asarray(plug[:3]))
    return {'relative_angle_deg':math.degrees(math.acos(float(np.clip((np.trace(relative)-1.)/2.,-1.,1.)))),
            'lateral_anchor_error_m':float(np.linalg.norm(local[:2])),
            'travel_m':float(CLIP_Z-local[2])}


def clip_constraint_summary(samples):
    """Engineering constraint check, independent of unchanged task scoring."""
    if not samples:raise ValueError('An actual constraint sample window is required')
    limits={'relative_angle_deg_strict':.5,'lateral_anchor_error_m_strict':.0001,'travel_tolerance_m':.00001}
    result={}
    for i in USED:
        values=[clip_joint_geometry(s['plug_poses'][str(i)],s['clip_poses'][str(i)]) for s in samples]
        angle=max(v['relative_angle_deg'] for v in values);lateral=max(v['lateral_anchor_error_m'] for v in values)
        low=min(v['travel_m'] for v in values);high=max(v['travel_m'] for v in values)
        valid=all(math.isfinite(v) for value in values for v in value.values())
        result[str(i)]={'max_relative_angle_deg':angle,'max_lateral_anchor_error_m':lateral,
                        'min_travel_m':low,'max_travel_m':high,
                        'pass':bool(valid and angle<.5 and lateral<.0001 and low>=-.00001 and high<=CLIP_TRAVEL+.00001)}
    return {'limits':limits,'connectors':result,'pass':all(v['pass'] for v in result.values())}


def connector_fit(plug,clip,port_index=None):
    rotation=quat_xyzw_to_matrix(np.asarray(plug[3:]))
    axis=rotation[:,0];pitch=math.degrees(math.atan2(-axis[2],math.hypot(axis[0],axis[1])))
    local=rotation.T@(np.asarray(clip[:3])-np.asarray(plug[:3]))
    if port_index is None:port_index=min(USED,key=lambda i:np.linalg.norm(np.asarray(plug[1:3])-np.asarray(port(i)[1:3])))
    return {'lateral_error_m':float(np.linalg.norm(np.asarray(plug[1:3])-np.asarray(port(port_index)[1:3]))),'pitch_deg':pitch,'yaw_deg':math.degrees(math.atan2(axis[1],axis[0])),
            'axis_angle_deg':math.degrees(math.acos(float(np.clip(axis[0],-1,1)))),
            'clip_travel_m':float(CLIP_Z-local[2])}


def fit_summary(samples):
    if not samples:raise ValueError('An actual untouched sample window is required')
    connectors={}
    for i in USED:
        values=[s['connector_fit'][str(i)] for s in samples]
        pitch=max(abs(v['pitch_deg']) for v in values);travel=max(abs(v['clip_travel_m']) for v in values)
        axis=max(v['axis_angle_deg'] for v in values);lateral=max(v['lateral_error_m'] for v in values)
        task_fit=bool(axis<5. and lateral<.001 and max(v['clip_travel_m'] for v in values)<.0003)
        engineering=bool(axis<=3.5 and travel<=FIT_MAX_CLIP_TRAVEL)
        connectors[str(i)]={'max_abs_pitch_deg':pitch,'max_axis_angle_deg':axis,'max_lateral_error_m':lateral,
                            'max_abs_clip_travel_m':travel,'last':values[-1],'task_fit_pass':task_fit,
                            'engineering_margin_pass':engineering,'pitch_clip_baseline_pass':bool(pitch<=3.5 and travel<=.0002),
                            'pass':task_fit}
    return {'sample_count':len(samples),'window_s':[samples[0]['time'],samples[-1]['time']],
            'limits':{'task':{'axis_angle_deg_strict':5.,'lateral_error_m_strict':.001,'clip_travel_m_strict':.0003},
                      'engineering':{'max_axis_angle_deg':3.5,'max_abs_clip_travel_m':.0002}},
            'connectors':connectors,'task_fit_pass':all(c['task_fit_pass'] for c in connectors.values()),
            'engineering_margin_pass':all(c['engineering_margin_pass'] for c in connectors.values()),
            'pass':all(c['task_fit_pass'] for c in connectors.values())}


def robot_snapshot(env,array):
    """Post-step joint/FK and independent native link poses, one copy per arm."""
    from yam_arena import ik
    result=[]
    for side in ('left','right'):
        asset=env._base.scene[side+'_robot'];ids,fids=env._ids[side+'_robot']
        joints=array(asset.data.joint_pos)[0];tcp,rotation=ik.tcp_pose(joints[ids],env.rig.bases[side])
        links=array(asset.data.body_link_pose_w)[0]
        result.append({'side':side,'joints':joints[ids].tolist(),'finger_joints':joints[fids].tolist(),
                       'tcp':tcp.tolist(),'tcp_rotation':rotation.tolist(),'robot_base_w':list(env.rig.bases[side]),
                       'robot_root_pose_w':array(asset.data.root_pose_w)[0].tolist(),
                       'body_names':list(asset.body_names),'body_link_pose_w':links.tolist()})
    return result
