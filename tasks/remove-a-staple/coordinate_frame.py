"""A fixed pre-construction change of world origin for thin paper precision.

Robot joints, geometry, gravity, materials and relative camera poses do not
change. Custom paper/hardware spawners use the same offset when authoring and
recording their reset poses. This module never edits a live simulation state.
"""
import copy
import numpy as np

OFFSET_M=np.zeros(3)
CONFIG_AUDIT=[]
_CONFIGURED_SCENES=[]


def configure(recenter=False):
    global OFFSET_M
    OFFSET_M=np.array((-.32,0.,-.7675)) if recenter else np.zeros(3)
    CONFIG_AUDIT.clear()
    _CONFIGURED_SCENES.clear()


def to_native(points):
    return np.asarray(points,float)+OFFSET_M


def to_station(points):
    return np.asarray(points,float)-OFFSET_M


def shift_scene_config(cfg):
    """Shift root assets and fixed cameras; preserve relative wrist offsets."""
    if any(cfg.scene is scene for scene in _CONFIGURED_SCENES):return cfg
    shifted=set()
    for name,asset in vars(cfg.scene).items():
        path=getattr(asset,'prim_path',None)
        if not isinstance(path,str):continue
        if name in ('left_cam','right_cam'):
            CONFIG_AUDIT.append(dict(name=name,prim_path=path,kind='unchanged_relative_wrist_camera'))
            continue
        initial=getattr(asset,'init_state',None)
        camera=getattr(asset,'offset',None)
        if initial is not None and hasattr(initial,'pos'):
            old=np.asarray(initial.pos,float)
            asset.init_state=copy.deepcopy(initial)
            asset.init_state.pos=tuple(to_native(old))
            kind='root_asset'
        elif camera is not None and hasattr(camera,'pos'):
            if name not in ('top_cam','overview_cam','judging_top','judging_side'):
                raise ValueError('Unclassified camera coordinate frame: '+name)
            old=np.asarray(camera.pos,float)
            asset.offset=copy.deepcopy(camera)
            asset.offset.pos=tuple(to_native(old))
            kind='fixed_world_camera'
        else:continue
        shifted.add(name)
        CONFIG_AUDIT.append(dict(name=name,prim_path=path,kind=kind,
                                 original_position_m=old.tolist(),native_position_m=to_native(old).tolist()))
    missing={'left_robot','right_robot','top_cam','table'}-shifted
    if missing:raise ValueError('Required world-root configurations were not shifted: '+','.join(sorted(missing)))
    _CONFIGURED_SCENES.append(cfg.scene)
    return cfg
