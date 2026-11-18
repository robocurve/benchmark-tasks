"""Privileged feasibility controller; imported only for an explicit demo plan.

It sends ordinary joint targets. It never writes a rigid-body transform, creates
an attachment, changes material state or alters scoring. Dream never installs it.
"""
import numpy as np
from scipy.spatial.transform import Rotation

def apply(row,observation,poses):
    from yam_arena import ik,station
    from inspect_robots_yam.packing import STATE_KEY
    spec=row['feedback'];off=0 if spec['arm']=='left' else 7
    measured=np.asarray(observation.state[STATE_KEY],dtype=float);command=np.asarray(row['action'],dtype=float).copy()
    obj=np.asarray(poses[spec['object']]);base=np.asarray(poses[spec['base']]);br=Rotation.from_quat(base[3:]);orr=Rotation.from_quat(obj[3:])
    tcp,rr=ik.tcp_pose(measured[off:off+6],base_pos=station.BASES[spec['arm']])
    relative_rotation=rr.T@orr.as_matrix();relative_position=rr.T@(obj[:3]-tcp)
    target_rot=br*Rotation.from_quat(spec['rotation'])
    target_tip=base[:3]+br.apply(spec['tip_in_base'])
    object_target=target_tip-target_rot.apply(spec['tip_in_object'])
    gripper=target_rot.as_matrix()@relative_rotation.T
    target=object_target-gripper@relative_position
    q,pe,re=ik.solve(target,Rotation.from_matrix(gripper).as_quat(),q_init=measured[off:off+6],base_pos=station.BASES[spec['arm']],iters=180)
    if not np.isfinite(q).all() or pe>.004 or re>.04:raise RuntimeError(f'Demo feedback target unreachable: {row["label"]}, position_error={pe:.6f}m, rotation_error={re:.6f}rad; preserve failed run')
    command[off:off+6]=np.clip(q,measured[off:off+6]-.06,measured[off:off+6]+.06)
    return command

_FORCE_BIAS=0.
_FORCE_ENGAGED=False
_BLADE_OFFSET=None

def apply_force(row,observation,poses,forces,filters):
    """Demo-only6N normal-load regulator, with bounded ordinary joint commands."""
    global _FORCE_BIAS,_FORCE_ENGAGED,_BLADE_OFFSET
    from yam_arena import ik,station
    from inspect_robots_yam.packing import STATE_KEY
    spec=row['force_control'];command=np.asarray(row['action'],float).copy();measured=np.asarray(observation.state[STATE_KEY],float)
    normal=Rotation.from_quat(poses[0,3:]).as_matrix()[:,2]
    f=float(np.mean([abs(v[1,filters.index('/World/envs/env_0/pcb_stencil')]@normal) for v in forces]))
    target_force=spec['target_N'];_FORCE_ENGAGED|=f>.05
    if target_force==0:_FORCE_BIAS=min(0.,_FORCE_BIAS+.0003)
    elif _FORCE_ENGAGED:
        _FORCE_BIAS=np.clip(_FORCE_BIAS-.00015*(target_force-f)*.05,-.012,0.)
        if f>15:_FORCE_BIAS=min(0.,_FORCE_BIAS+.0005)
    if 'edge_in_stencil' in spec:
        current_tcp,current_rot=ik.tcp_pose(measured[7:13],base_pos=station.BASES['right'])
        tool=np.asarray(poses[1]);stencil=np.asarray(poses[0]);sr=Rotation.from_quat(stencil[3:]);tr=Rotation.from_quat(tool[3:])
        if _BLADE_OFFSET is None:_BLADE_OFFSET=current_rot.T@(tool[:3]-current_tcp)
        relative_rotation=current_rot.T@tr.as_matrix()
        body_rotation=sr*Rotation.from_euler('x',45,degrees=True)
        rot=body_rotation.as_matrix()@relative_rotation.T
        edge=stencil[:3]+sr.apply(spec['edge_in_stencil'])
        centre=edge-body_rotation.apply([0,-.00025,-.0325])
        tcp=centre-rot@_BLADE_OFFSET+normal*_FORCE_BIAS
    else:
        tcp,rot=ik.tcp_pose(command[7:13],base_pos=station.BASES['right']);tcp=tcp+normal*_FORCE_BIAS
    q,pe,re=ik.solve(tcp,Rotation.from_matrix(rot).as_quat(),q_init=measured[7:13],base_pos=station.BASES['right'],iters=200)
    if pe>.002 or re>.02:raise RuntimeError('Demo load-regulation target unreachable')
    command[7:13]=np.clip(q,measured[7:13]-.06,measured[7:13]+.06)
    return command
