"""Record actual simulator mass properties and the supplied station geometry."""
from dataclasses import asdict
import numpy as np

def array(x):
    if hasattr(x,'torch'):x=x.torch
    if hasattr(x,'detach'):x=x.detach().cpu()
    if hasattr(x,'numpy'):x=x.numpy()
    return np.asarray(x).tolist()

def capture(emb, observation, names):
    from pxr import UsdGeom,Usd,UsdPhysics,UsdShade
    from yam_arena import station
    assert emb.rig == station.rig('calibrated')
    stage=emb._base.sim.stage
    result={'revision':emb.info.environment_revision,'rig':asdict(emb.rig),
            'dt':float(emb._base.cfg.sim.dt),'decimation':int(emb._base.cfg.decimation),
            'images':{n:list(im.shape) for n,im in observation.images.items()},'assets':{}}
    for name in names:
        asset=emb._base.scene[name];view=asset.root_physx_view
        result['assets'][name]={'body_names':list(asset.body_names),
          'mass_kg':array(view.get_masses()),'inertia_kg_m2':array(view.get_inertias()),
          'com_pose':array(view.get_coms())}
    bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_])
    table=stage.GetPrimAtPath('/World/envs/env_0/table')
    box=bounds.ComputeWorldBound(table).ComputeAlignedRange()
    result['table_world_bounds']={'min':list(box.GetMin()),'max':list(box.GetMax())}
    result['joints']=[];result['physics_materials']=[]
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if '/env_0/' not in path:continue
        if prim.IsA(UsdPhysics.RevoluteJoint) and '/cart/' in path:
            result['joints'].append({'path':path,'attributes':{a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith(('drive:','physics:'))}})
        if prim.HasAPI(UsdPhysics.MaterialAPI):
            result['physics_materials'].append({'path':path,'attributes':{a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith(('physics:','physxMaterial:'))}})
    return result


def write_reset(emb, observation, names):
    import os,json
    from pathlib import Path
    dest=os.environ.get('TASK_TELEMETRY_DIR')
    if not dest:return
    out=Path(dest);out.mkdir(parents=True,exist_ok=True)
    (out/'runtime-audit.json').write_text(json.dumps(capture(emb,observation,names),indent=2))


def write_episode(emb):
    import os,json
    from pathlib import Path
    dest=os.environ.get('TASK_TELEMETRY_DIR')
    if not dest or not hasattr(emb,'trace'):return
    out=Path(dest);out.mkdir(parents=True,exist_ok=True)
    (out/'physics.json').write_text(json.dumps(emb.trace,default=array))
    (out/'native-result.json').write_text(json.dumps(dict(score=emb.score,steps=len(emb.trace),revision=emb.info.environment_revision,events=getattr(emb,'events',[])),indent=2,default=array))
