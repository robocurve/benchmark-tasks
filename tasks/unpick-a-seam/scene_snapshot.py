"""Read-only native authoring diagnostics, separate from task scoring.

Resolve composed USD attributes, collider bounds and physics bindings after
the loader has finished authoring. Reset snapshots read native tensors; they
never alter state or declare an otherwise invalid trial valid.
"""
import math
import numpy as np


def plain(value):
    if value is None or isinstance(value,(str,bool,int)):
        return value
    if isinstance(value,float):
        return value if math.isfinite(value) else str(value)
    try:
        return [plain(v) for v in value]
    except TypeError:
        return str(value)


def attributes(prim,prefixes):
    return {attr.GetName():plain(attr.Get()) for attr in prim.GetAttributes()
            if any(attr.GetName().startswith(prefix) for prefix in prefixes)}


def authored_scene(stage,roots):
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade
    import carb
    settings=carb.settings.get_settings()
    result={'settings':{key:plain(settings.get(key)) for key in
        ('/physics/suppressReadback','/physics/cudaDevice',
         '/physics/disableContactProcessing','/physics/updateToUsd')},
        'physics_scenes':{},'fixtures':{},'fixture_joints':{},'surfaces':{}}
    bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_],False)
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            result['physics_scenes'][str(prim.GetPath())]=attributes(prim,('physics:','physxScene:'))
        if prim.IsA(UsdPhysics.Joint) and '/clamp_' in str(prim.GetPath()):
            result['fixture_joints'][str(prim.GetPath())]=attributes(prim,('physics:','drive:','physxJoint:'))
    for name,path in roots.items():
        if not name.startswith('clamp_'):continue
        colliders=[]
        for prim in Usd.PrimRange(stage.GetPrimAtPath(path)):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
            box=bounds.ComputeWorldBound(prim).ComputeAlignedRange()
            material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
            colliders.append({'path':str(prim.GetPath()),'type':prim.GetTypeName(),
                'low_m':plain(box.GetMin()),'high_m':plain(box.GetMax()),
                'contact':attributes(prim,('physics:','physxCollision:')),
                'parent_body':attributes(stage.GetPrimAtPath(path),('physics:','physxRigidBody:')),
                'physics_material_path':str(material.GetPath()) if material else None,
                'physics_material':attributes(material.GetPrim(),('physics:','physxMaterial:')) if material else None})
        result['fixtures'][name]=colliders
    for prim in stage.Traverse():
        if 'OmniPhysicsSurfaceDeformableSimAPI' not in prim.GetAppliedSchemas():continue
        material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
        result['surfaces'][str(prim.GetPath())]={
            'contact':attributes(prim,('physics:','physxCollision:','physxDeformableBody:')),
            'physics_material_path':str(material.GetPath()) if material else None,
            'physics_material':attributes(material.GetPrim(),('omniphysics:','physxDeformableMaterial:')) if material else None,
            'world_transform':plain(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))}
    return result


def native_state(views,cloth,thread_view,order):
    def array(value):
        return getattr(value,'torch',value).detach().cpu().numpy().copy()
    data={}
    for name,view,mesh in zip(('left','right'),views,cloth):
        data[name]=array(view.get_simulation_nodal_positions())[0,:len(mesh.rest)]
        data[name+'_velocity']=array(view.get_simulation_nodal_velocities())[0,:len(mesh.rest)]
        data[name+'_rest']=mesh.rest
    data['thread']=array(thread_view.get_transforms())[order]
    data['thread_velocity']=array(thread_view.get_velocities())[order]
    return data


def state_summary(data,expected):
    return {name:{'maximum_position_error_m':float(np.linalg.norm(data[name]-points,axis=1).max()),
                  'maximum_speed_m_s':float(np.linalg.norm(data[name+'_velocity'],axis=1).max())}
            for name,points in zip(('left','right'),expected)}
