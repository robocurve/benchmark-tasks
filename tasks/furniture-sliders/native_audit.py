"""Read the live robot properties and fingerprint the installed native assets."""
import hashlib
from pathlib import Path
import numpy as np


def capture(base):
    from yam_arena import PACKAGE_DIR,ASSET_DIR,ik
    from yam_arena.constants import SHOULDER,WRIST,GRIPPER,ARM_JOINTS
    from partkit import _np
    files=list(ASSET_DIR.rglob('*'))+[PACKAGE_DIR/n for n in (
        'robot.py','actions.py','station.py','constants.py','ik.py','inspect_embodiment.py')]
    fingerprints={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files
                  if p.is_file() and p.suffix.lower() in ('.py','.urdf','.usd','.usda','.usdc','.stl','.obj')}
    out={'source_sha256':fingerprints,'arms':{}}
    for side in ('left','right'):
        robot=base.scene[side+'_robot'];names=list(robot.joint_names)
        data={key:_np(getattr(robot.data,key))[0].tolist() for key in (
            'joint_stiffness','joint_damping','joint_pos_limits','joint_vel_limits','joint_effort_limits')}
        props=[SHOULDER if n in ARM_JOINTS[:3] else WRIST if n in ARM_JOINTS else GRIPPER for n in names]
        for key,constant in [('joint_stiffness','stiffness'),('joint_damping','damping'),
                             ('joint_vel_limits','velocity'),('joint_effort_limits','effort')]:
            if not np.allclose(data[key],[p[constant] for p in props],atol=1e-5,rtol=1e-6):
                raise RuntimeError(f'Live {side} {key} differs from stock native configuration')
        ids=[names.index(n) for n in ARM_JOINTS]
        limits=np.asarray(data['joint_pos_limits'])[ids]
        if not np.allclose(limits,np.column_stack((ik.LOWER,ik.UPPER)),atol=1e-5):
            raise RuntimeError('Live joint limits differ from stock URDF')
        index=list(robot.body_names).index('base')
        pose=_np(robot.data.body_link_pose_w)[0,index]
        if not np.allclose(pose[:3],[0,.305 if side=='left' else -.305,.75],atol=1e-5):
            raise RuntimeError('Stock arm base location changed')
        out['arms'][side]={'joint_names':names,'base_pose':pose.tolist(),**data}
    out['verified_stock_properties']=True
    from config import F,P
    from adapter import PAD_NAMES
    from assets import furniture_inertia
    out['task_bodies']={}
    for name in ('furniture',*PAD_NAMES):
        data=base.scene[name].data
        row={key:_np(getattr(data,key))[0].tolist() for key in ('body_mass','body_com_pose_b','body_inertia')}
        expected=F['mass_kg'] if name=='furniture' else P['mass_kg']
        if not np.allclose(row['body_mass'],expected,atol=1e-6):
            raise RuntimeError(f'Live {name} mass differs from the declared estimate')
        if name=='furniture' and not np.allclose(np.asarray(row['body_com_pose_b'])[0,:3],furniture_inertia()[0],atol=1e-6):
            raise RuntimeError('Live furniture centre of mass differs from the declared model')
        out['task_bodies'][name]=row
    out['gravity']=list(base.cfg.sim.gravity)
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics
    stage=get_current_stage()
    floor=stage.GetPrimAtPath('/World/envs/env_0/floor_panel/body')
    colliders=[p for p in floor.GetChildren() if p.HasAPI(UsdPhysics.CollisionAPI)]
    if len(colliders)!=1 or colliders[0].GetName()!='mesh_0' or floor.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError('Floor must have one static mesh collider and noncolliding markings')
    approximation=UsdPhysics.MeshCollisionAPI(colliders[0]).GetApproximationAttr().Get()
    if approximation!='none':raise RuntimeError('Floor mesh must preserve the mounting cutouts')
    out['floor_collision']={'colliders':[str(p.GetPath()) for p in colliders],
                            'static':True,'approximation':approximation}
    return out
