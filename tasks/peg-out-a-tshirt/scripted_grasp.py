"""Explicit state-assisted scripted feasibility demo, never the Dream policy.

Read the naturally settled hem once, then compile ordinary bounded robot
commands around it. This function never writes a simulator node, object pose,
attachment, material, collision setting or force. Actual physical grip remains
necessary; this is a known-state scripted demonstration, not an image-policy
capability claim.
"""
import copy
import hashlib
import numpy as np
from fixed_motion import ROTATIONS,compile_grasp


def calibrate(reference_plan,rest,nodes):
    rest=np.asarray(rest,float);nodes=np.asarray(nodes,float)
    if rest.shape!=nodes.shape or rest.ndim!=2 or rest.shape[1]!=3 or not np.isfinite(nodes).all():
        raise ValueError('Finite matching material and native positions are required')
    targets={};patches={}
    for side,hem_x,sign in (('left',-.12,1),('right',.12,-1)):
        ids=np.flatnonzero((abs(rest[:,0]-hem_x)<.011)&(rest[:,1]<.021))
        if len(ids)<6:raise ValueError('Missing declared hem material patch')
        point=nodes[ids].mean(0)
        if not (.17<point[0]<.26 and abs(point[1]-sign*.115)<.04 and .78<point[2]<.90):
            raise ValueError('Hem lies outside the audited reset pickup region')
        # A20mm bite behind the TCP stays within the physical finger pad.
        # Both textile layers are between the ordinary fingers' closing plane.
        targets[side]=(point+ROTATIONS[side][:,2]*.020).tolist()
        patches[side]=dict(material_node_ids=ids.tolist(),observed_center_m=point.tolist(),target_tcp_m=targets[side],desired_patch_local_m=[0.,0.,-.020])
    prefix=compile_grasp(grasp_targets=targets)['frames'][:200]
    pinch=float(reference_plan.get('scripted_cloth_pinch_opening',0.))
    if not 0.<=pinch<=.03:raise ValueError('Cloth pinch target outside the declared gentle-pinch diagnostic range')
    if pinch:
        # A finite ordinary finger target avoids crushing the two thin layers.
        # The mesh, friction, contacts and motor limits are unchanged.
        for frame in prefix[100:]:
            for index in (6,13):
                opening=frame['action14'][index]
                frame['action14'][index]=pinch+(opening/.6)*(.6-pinch)
    result=copy.deepcopy(reference_plan)
    if len(result['frames'])<200:raise ValueError('Reference sequence has no complete grasp prefix')
    result['frames'][:200]=prefix
    result['feedback']='Explicit state-assisted scripted demo: measured reset cloth patches select ordinary grasp targets; remainder uses ordinary joint tracking. Dream receives no such state.'
    result['state_assisted_reset_grasp']=patches
    result['native_reset_nodes_sha256']=hashlib.sha256(np.ascontiguousarray(nodes).tobytes()).hexdigest()
    return result,dict(patches=patches,input_native_nodes_sha256=result['native_reset_nodes_sha256'],state_writes=False)


def held_patch_geometry(rest, nodes, gripper_pose, side='left'):
    """Reject a plainly lost grasp; proximity alone never proves retention."""
    from interpret_motion import rotation_xyzw
    rest = np.asarray(rest, float)
    nodes = np.asarray(nodes, float)
    pose = np.asarray(gripper_pose, float)
    hem_x = -.12 if side == 'left' else .12
    ids = np.flatnonzero((abs(rest[:, 0] - hem_x) < .011) & (rest[:, 1] < .021))
    if not len(ids) or nodes.shape != rest.shape or pose.shape != (7,):
        raise ValueError('Missing actual cloth or gripper observations')
    rotation = rotation_xyzw(pose[3:])
    tcp = pose[:3] + rotation @ np.array((0., 0., .134))
    local = (nodes[ids].mean(0) - tcp) @ rotation
    nearby = abs(local[0]) < .015 and abs(local[1]) < .05 and -.06 < local[2] < .03
    return dict(patch_relative_to_tcp_m=local.tolist(), plainly_lost=not bool(nearby),
        scope='Early failure guard only; no grasp or task-success claim')
