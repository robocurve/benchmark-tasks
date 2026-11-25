"""Build the portable scene prototype; no policy, demo or paid calls."""
from pathlib import Path
import argparse
import json

NAME = 'balloon-catheter-removal'
HERE = Path(__file__).resolve().parent
INSTRUCTION = json.loads((HERE / 'task.json').read_text())['instruction']
HZ = 20
MAX_STEPS = 3600
OBJECT_NAMES = ('catheter','valve','barrel','plunger')

def make_scorer():
    from scoring import TrialScorer
    return TrialScorer()

def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.scene import task_room,env_cfg_callback
    objects=[o for o in task_room('station',embodiment.rig.top_cam) if o.name not in ('table','floor')]
    objects.append(Object(name='surgical_room',usd_path=str(HERE/'assets/surgical-drape/surgical_room.usda'),
        object_type=ObjectType.BASE,initial_pose=Pose(position_xyz=(0,0,0))))
    # Preserve the complete station and its authored cross-body joints. Native
    # measured-state hooks track its four bodies without respawning duplicates.
    objects.append(Object(name='catheter_station',usd_path=str(HERE/'station-layout.usda'),
        object_type=ObjectType.BASE,initial_pose=Pose(position_xyz=(0,.10,0))))
    objects.append(Object(name='instrument_rests',usd_path=str(HERE/'rest-layout.usda'),
        object_type=ObjectType.BASE,initial_pose=Pose(position_xyz=(0,0,0))))
    from layout import RECOVERY_TRAY_POSITION,RECOVERY_RESTS_POSITION
    objects.append(Object(name='recovery_tray',usd_path=str(HERE/'assets/instrument-recovery-tray-380/tray.usda'),
        object_type=ObjectType.BASE,initial_pose=Pose(position_xyz=RECOVERY_TRAY_POSITION)))
    objects.append(Object(name='recovery_rests',usd_path=str(HERE/'assets/catheter-recovery-v-rests-v4/rests.usda'),
        object_type=ObjectType.BASE,initial_pose=Pose(position_xyz=RECOVERY_RESTS_POSITION)))
    for side in ('left','right'):
        getattr(embodiment.scene_config,side+'_robot').spawn.activate_contact_sensors=True

    def configure(cfg):
        from isaaclab.sensors import ContactSensorCfg
        cfg=env_cfg_callback(control_hz)(cfg)
        for side,arm in [('left','LeftArm'),('right','RightArm')]:
            for finger in ('tip_left','tip_right'):
                sensor=ContactSensorCfg(
                    prim_path='{ENV_REGEX_NS}/'+arm+embodiment.link_paths[finger],
                    filter_prim_paths_expr=['{ENV_REGEX_NS}/catheter_station/Valve','{ENV_REGEX_NS}/catheter_station/Syringe/Barrel'],
                    history_length=0)
                setattr(cfg.scene,side+'_'+finger+'_device',sensor)
        return cfg
    cfg=IsaacLabArenaEnvironment(name='yam_balloon_catheter',scene=Scene(objects),embodiment=embodiment,
        env_cfg_callback=configure)
    env=ArenaEnvBuilder(cfg,ArenaEnvBuilderCfg(num_envs=1,solve_relations=False,device=device)).make_registered()
    import omni.usd
    for p in omni.usd.get_context().get_stage().Traverse():
        if '/surgical_room/' in str(p.GetPath()) and p.GetName() in ('Contents','Tools'):p.SetActive(False)
    return env

def make_dynamics(env):
    from dynamics import Dynamics
    base=env.unwrapped
    if not hasattr(base,'_catheter_dynamics'):base._catheter_dynamics=Dynamics(env)
    return base._catheter_dynamics

def read_object_states(env):
    return make_dynamics(env).read_object_states()

def demo():
    # The measured-pose controller records its explicit Cartesian waypoints.
    return {}

def make_demo_controller(env, plans):
    from controller import Controller
    return Controller(env,plans)


def build_scene(output=None):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Sdf, Gf
    from layout import build_layouts
    build_layouts()
    output = Path(output or HERE / 'scene.usdc').resolve()
    source = HERE / 'assets/surgical-drape/surgical_scene.usdc'
    base = Usd.Stage.Open(str(source))
    def calibration(stage):
        return {str(p.GetPath()): {a.GetName(): str(a.Get()) for a in p.GetAttributes()}
                for p in stage.Traverse() if p.IsA(UsdGeom.Camera) or p.HasAPI(UsdLux.LightAPI)}
    original = calibration(base)
    s = Usd.Stage.CreateNew(str(output))
    import os
    # Copy the flattened source's scene roots before opening in Kit. The source
    # capture also saved live render graphs; importing/deleting those after Kit
    # activation leaves stale SyntheticData graph handles.
    flat = base.Flatten()
    for prim in base.GetPseudoRoot().GetChildren():
        if prim.GetName() not in ('Render','Replicator'):
            Sdf.CopySpec(flat,prim.GetPath(),s.GetRootLayer(),prim.GetPath())
    UsdGeom.SetStageMetersPerUnit(s,1.)
    UsdGeom.SetStageUpAxis(s,'Z')
    s.SetDefaultPrim(s.GetPrimAtPath('/World'))
    for path in ('/World/SurgicalScene/Contents', '/World/SurgicalScene/Tools'):
        s.OverridePrim(path).SetActive(False)
    # The preview asset originally HID its stock table/floor. Disable their
    # physics too, since this scene retains the replacement surgical table/floor.
    disabled = []
    for p in s.Traverse():
        if '/World/SurgicalScene/' in str(p.GetPath()):
            continue
        if p.HasAPI(UsdPhysics.CollisionAPI) and any(x.lower() in ('table','floor') for x in str(p.GetPath()).split('/')):
            UsdPhysics.CollisionAPI(p).CreateCollisionEnabledAttr(False)
            disabled.append(str(p.GetPath()))
    prop = UsdGeom.Xform.Define(s, '/World/MedicalStation')
    asset = HERE / 'station-layout.usda'
    prop.GetPrim().GetReferences().AddReference(os.path.relpath(asset,output.parent), '/MedicalStation')
    rests=UsdGeom.Xform.Define(s,'/World/InstrumentRests')
    rests.GetPrim().GetReferences().AddReference(os.path.relpath(HERE/'rest-layout.usda',output.parent),'/InstrumentRests')
    from layout import RECOVERY_TRAY_POSITION,RECOVERY_RESTS_POSITION
    recovery=UsdGeom.Xform.Define(s,'/World/RecoveryTray')
    recovery.GetPrim().GetReferences().AddReference(os.path.relpath(HERE/'assets/instrument-recovery-tray-380/tray.usda',output.parent),'/RecoveryTray')
    recovery.AddTranslateOp().Set(Gf.Vec3d(*RECOVERY_TRAY_POSITION))
    receiving=UsdGeom.Xform.Define(s,'/World/RecoveryRests')
    receiving.GetPrim().GetReferences().AddReference(os.path.relpath(HERE/'assets/catheter-recovery-v-rests-v4/rests.usda',output.parent),'/RecoveryRests')
    receiving.AddTranslateOp().Set(Gf.Vec3d(*RECOVERY_RESTS_POSITION))
    s.GetRootLayer().Save()
    assert calibration(s) == original, 'Base camera/light calibration changed'
    if output.name == 'scene.usdc':
        # Keep the editable entry point small while storing robot meshes in USD's
        # binary format, rather than duplicating their large arrays as ASCII.
        wrapper = Sdf.Layer.CreateNew(str(output.with_suffix('.usda')))
        wrapper.subLayerPaths = [output.name]
        wrapper.defaultPrim = 'World'
        wrapper.Save()
    evidence = {'scene':output.name,'camera_light_attributes_unchanged':True,
                'source':'assets/surgical-drape/surgical_scene.usdc',
                'disabled_original_colliders':disabled,'excluded_source_runtime_graphs':['/Render','/Replicator'],
                'active_prims':sum(1 for _ in s.Traverse()),
                'state':'bimanual scene; measured connection and pressure-volume surrogate in scene_runtime.py'}
    output.with_suffix('.json').write_text(json.dumps(evidence,indent=2)+'\n')
    return evidence


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path)
    print(json.dumps(build_scene(p.parse_args().output),indent=2))
