"""CPU USD tests for applied solver settings/global clamping diagnostics."""
from types import SimpleNamespace
from pxr import Usd,UsdPhysics,Sdf
from pen_capping.solver_audit import collect_solver_settings


def settings(body_position=128,scene_max=255,config_max=255):
    stage=Usd.Stage.CreateInMemory();scene=UsdPhysics.Scene.Define(stage,'/World/physicsScene').GetPrim()
    for name,value in [('minPositionIterationCount',1),('maxPositionIterationCount',scene_max),('minVelocityIterationCount',0),('maxVelocityIterationCount',255)]:
        scene.CreateAttribute('physxScene:'+name,Sdf.ValueTypeNames.Int).Set(value)
    root=stage.DefinePrim('/World/envs/env_0/cap_0','Xform');UsdPhysics.RigidBodyAPI.Apply(root)
    root.CreateAttribute('physxRigidBody:solverPositionIterationCount',Sdf.ValueTypeNames.Int).Set(body_position)
    root.CreateAttribute('physxRigidBody:solverVelocityIterationCount',Sdf.ValueTypeNames.Int).Set(16)
    cfg=SimpleNamespace(min_position_iteration_count=1,max_position_iteration_count=config_max,
                        min_velocity_iteration_count=0,max_velocity_iteration_count=255,solver_type=1)
    return collect_solver_settings(stage,cfg,['cap_0'],128,16)


def test_applied_counts_and_global_bounds_are_reported():
    audit=settings()
    assert not audit['issues'] and audit['bodies']['cap_0']==dict(position=128,velocity=16)
    assert audit['scene_solver_settings'][0]['attributes']['physxScene:maxPositionIterationCount']==255
    assert audit['simulation_config']['solver_type']==1


def test_actual_scene_clamp_detected_even_if_config_allows_requested_count():
    audit=settings(scene_max=64)
    assert any('would clamp requested 128' in row for row in audit['issues'])


def test_unapplied_body_setting_is_not_claimed_successful():
    audit=settings(body_position=32)
    assert any('applied body counts' in row for row in audit['issues'])
