"""Two rendering-only oblique evidence views; neither is a policy input."""
import os
import numpy as np

EVIDENCE_VIEWS = {
    'observer_cam': {'position':(1.05,.35,.96), 'target':(.42,0,.79)},
    'observer_opposite_cam': {'position':(1.05,-.35,.96), 'target':(.42,0,.79)},
}


def camera_metadata():
    return {'views':EVIDENCE_VIEWS, 'focal_length_mm':18., 'horizontal_aperture_mm':20.955,
            'width':640, 'height':352, 'policy_inputs':False, 'adds_bodies_colliders_or_lights':False}


def add_camera(scene_config):
    if os.environ.get('FURNITURE_SLIDERS_OBSERVER')!='1':return
    from isaaclab.sensors import CameraCfg
    from isaaclab.sim import PinholeCameraCfg
    from scipy.spatial.transform import Rotation
    from yam_arena.embodiment import RENDERER
    from copy import deepcopy
    width,height=map(int,os.environ['FURNITURE_SLIDERS_OBSERVER_SIZE'].split('x'))
    for name,view in EVIDENCE_VIEWS.items():
        position=np.array(view['position']);target=np.array(view['target'])
        forward=target-position;forward/=np.linalg.norm(forward)
        left=np.cross((0,0,1),forward);left/=np.linalg.norm(left)
        up=np.cross(forward,left)
        quaternion=tuple(Rotation.from_matrix(np.column_stack((forward,left,up))).as_quat())
        setattr(scene_config,name,CameraCfg(
            prim_path='{ENV_REGEX_NS}/'+name,width=width,height=height,data_types=['rgb'],
            update_period=0.,spawn=PinholeCameraCfg(focal_length=18.,horizontal_aperture=20.955,
                                                  clipping_range=(.02,20.)),
            offset=CameraCfg.OffsetCfg(pos=tuple(position),rot=quaternion,convention='world'),
            renderer_cfg=deepcopy(RENDERER)))


def recording_images(embodiment,observation):
    images=dict(observation.images)
    if os.environ.get('FURNITURE_SLIDERS_OBSERVER')=='1':
        for name in EVIDENCE_VIEWS:
            value=embodiment._base.scene[name].data.output['rgb']
            value=value.torch if hasattr(value,'torch') else value
            images[name]=np.ascontiguousarray(value[0,...,:3].cpu().numpy(),dtype=np.uint8)
    return images
