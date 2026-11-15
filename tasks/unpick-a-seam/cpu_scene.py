"""Preconstruction CPU sweep-CCD authoring for the explicit panel candidate."""
def configure_native(stage,paths):
    from runtime_config import CONFIG
    if not CONFIG.cpu_ccd:return
    from pxr import UsdPhysics,PhysxSchema
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            api=PhysxSchema.PhysxSceneAPI.Apply(prim)
            api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP')
            api.CreateEnableCCDAttr(True);api.CreateSolverTypeAttr(CONFIG.solver)
            api.CreateEnableExternalForcesEveryIterationAttr(False)
    for path in paths:
        prim=stage.GetPrimAtPath(path)
        if not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError('Missing dynamic body for CPU CCD: '+str(path))
        api=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        api.CreateEnableCCDAttr(True);api.CreateEnableSpeculativeCCDAttr(False)
