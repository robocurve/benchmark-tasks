"""Output-only native actor capture and optional camera suppression.

No object pose, joint target, physics parameter or scoring state is written.
The capture is source material for a later offline view, not camera evidence.
"""
import hashlib
import math
from pathlib import Path
import numpy as np


def configure_embodiment(instance,args,kwargs,original_init,review,*,no_video):
    """Keep the existing physical embodiment; optionally omit camera sensors."""
    kwargs=dict(kwargs);kwargs['cam_width']=320;kwargs['cam_height']=180
    kwargs['enable_cameras']=not no_video
    original_init(instance,*args,**kwargs)
    if not no_video:review.add_cameras(instance)


def suppress_state_only_rendering(base,*,no_video):
    """Call after construction, before reset; native stepping stays unchanged."""
    if not no_video:return
    names=('top_cam','left_cam','right_cam','review_top','review_left','review_right')
    cameras=[name for name in names if name in base.scene.sensors]
    if cameras:raise RuntimeError('state-only mode unexpectedly created cameras: '+str(cameras))
    base.sim.render=lambda *args,**kwargs:None


def array(value):
    value=value.torch if hasattr(value,'torch') else value
    if hasattr(value,'detach'):value=value.detach().cpu().numpy()
    elif hasattr(value,'numpy'):value=value.numpy()
    return np.asarray(value)


def native_body_owners(scene,link_paths,panel_names,*,env_path='/World/envs/env_0'):
    """Actual native body indices and imported USD paths, including parents."""
    panels=list(panel_names)
    if len(panels)!=12 or len(set(panels))!=12 or 'cutter' in panels:
        raise ValueError('capture requires the twelve unique native carton panels')
    owners={}
    for side,prim in (('left','LeftArm'),('right','RightArm')):
        entity=side+'_robot';names=list(scene[entity].body_names)
        if len(names)!=9 or len(set(names))!=9:raise ValueError('expected nine native robot bodies for '+entity)
        for index,name in enumerate(names):
            if name not in link_paths:raise ValueError('missing imported USD link path for '+name)
            path=env_path+'/'+prim+link_paths[name]
            if path in owners:raise ValueError('duplicate native body owner '+path)
            owners[path]={'kind':'robot','entity':entity,'body_name':name,'body_index':index,'source':'ArticulationData.body_link_pose_w'}
    for name in [*panels,'cutter']:
        owners[env_path+'/'+name]={'kind':'object','entity':name,'source':'arena_world.get_pose_w'}
    if len(owners)!=31:raise ValueError('capture requires 31 native body owners')
    for path,owner in owners.items():
        parents=[candidate for candidate in owners if path.startswith(candidate+'/')]
        owner['parent_body_owner']=max(parents,key=len) if parents else None
    return owners


def native_body_world_poses(scene,owners,object_poses):
    """Read actor frames; reuse this row's measured native object root poses.

    object_poses must come from arena_world.get_pose_w in the current post-step
    row. Robot body_link_pose_w is copied once per arm. There is no FK fallback.
    """
    robot_poses={};result={}
    for path,owner in owners.items():
        entity=owner['entity']
        if owner['kind']=='robot':
            if entity not in robot_poses:
                robot_poses[entity]=array(scene[entity].data.body_link_pose_w)
                if robot_poses[entity].shape!=(1,9,7):raise ValueError('invalid body_link_pose_w shape for '+entity)
            if scene[entity].body_names[owner['body_index']]!=owner['body_name']:
                raise ValueError('native robot body ordering changed')
            pose=np.asarray(robot_poses[entity][0,owner['body_index']],dtype=float)
        else:
            if entity not in object_poses:raise ValueError('missing actual native object pose '+entity)
            pose=np.asarray(object_poses[entity],dtype=float)
        if pose.shape!=(7,) or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1.)>.001:
            raise ValueError('invalid actual native actor pose for '+path)
        result[path]=pose.tolist()
    if len(result)!=31:raise ValueError('incomplete native pose capture')
    return result


def export_scene(stage,path):
    """Flatten to a separate layer; never edit the live simulation stage."""
    path=Path(path);layer=stage.Flatten()
    if not layer.Export(str(path)):raise RuntimeError('post-reset scene export failed')
    dependencies=getattr(layer,'GetExternalAssetDependencies',None)
    return {'scene_file':path.name,'scene_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'external_asset_dependencies':sorted(str(value) for value in dependencies()) if dependencies else None}


def virtual_review_views(camera_cfg,target=(.32,0.,.82)):
    """Same view calls as review.py; optics read from the actual spawn config."""
    x,y,z=target
    eyes=((x-.40,y-.32,z+.43),(x-.30,y+.32,z+.25),(x-.30,y-.30,z+.23))
    optics={}
    for name in ('focal_length','horizontal_aperture','vertical_aperture','horizontal_aperture_offset','vertical_aperture_offset'):
        value=getattr(camera_cfg,name,None)
        if value is not None and not math.isfinite(value):raise ValueError('invalid virtual review optics '+name)
        optics[name+'_mm']=None if value is None else float(value)
    if optics['focal_length_mm']!=18. or optics['horizontal_aperture_mm'] is None:
        raise ValueError('review views require actual 18 mm focal/aperture configuration')
    optics['clipping_range_m']=list(camera_cfg.clipping_range)
    return {'kind':'virtual_offline_view_configuration','camera_observations':False,
            'source':'review.py fixed eye/target calls and actual PinholeCameraCfg; not rendered observations',
            'view_method':'Camera.set_world_poses_from_view(eye,target)',
            'width':480,'height':270,'optics':optics,
            'vertical_aperture_note':'None retains the native config aspect-ratio default; no aperture value is invented',
            'views':{name:{'eye_world_m':list(eye),'target_world_m':list(target)}
                     for name,eye in zip(('review_top','review_left','review_right'),eyes)}}


def capture_manifest(scene_export,owners,source_hashes,views,*,no_video):
    return {**scene_export,'owners':owners,'source_sha256':dict(source_hashes),
            'pose_order':['x','y','z','qx','qy','qz','qw'],'quaternion_order':'xyzw','pose_space':'simulation_world',
            'native_pose_source':'actual ArticulationData.body_link_pose_w and arena_world.get_pose_w; no FK animation',
            'capture_hz':20.,'first_time_s':.05,'row_timing':'post-step; existing first row .05 s; no invented initial row',
            'scene_export_timing':'after existing env reset, before first control step',
            'interpolated':False,'appearance':'flattened composed post-reset USD; static native materials/geometry',
            'virtual_review_views':views,'state_only':bool(no_video),'media_eligible':False,
            'scope':'output-only capture; not a completed task demo or native camera observation'}
