"""Local adapter for the supplied Arena runtime; call after Isaac Sim has started.

This builds the authored USD scene. It does not invoke Dream or implement a policy.
"""
from pathlib import Path
import json
import math

ROOT=Path(__file__).resolve().parent
INSTRUCTION='Use the scoop to put clean ice into the bowl.'
SCENE='station'
CONTROL_HZ=20
POLICY_CAMERAS=('top_cam','left_cam','right_cam')
POLICY_SIZE=(320,176)


def object_specs():
    return json.loads((ROOT/'layout.json').read_text())['objects']


def reset_ice_collection(env,env_ids):
    collection=env.scene['ice']
    poses=collection.data.default_body_pose.torch[env_ids].clone()
    poses[:,:,:3]+=env.scene.env_origins[env_ids,None,:]
    collection.write_body_link_pose_to_sim_index(body_poses=poses,env_ids=env_ids)
    collection.write_body_com_velocity_to_sim_index(body_velocities=collection.data.default_body_vel.torch[env_ids],env_ids=env_ids)


def pose_reader(env):
    scene=env.unwrapped.scene
    collection=scene.rigid_object_collections.get('ice')
    indices={} if collection is None else {name:i for i,name in enumerate(collection.body_names)}
    def pose(name):
        if name in indices:
            return collection.data.body_link_pose_w.torch[0,indices[name]].cpu().numpy().copy()
        return scene[name].data.root_link_pose_w.torch[0].cpu().numpy().copy()
    return pose


def build_environment(*,enable_cameras=False,control_hz=20,extra_objects=(),batch_ice=True):
    import isaaclab.sim as sim_utils
    from isaaclab.app.settings_manager import get_settings_manager
    get_settings_manager().set_bool("/physics/disableContactProcessing",False)
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.utils.pose import Pose
    from yam_arena import station
    from yam_arena.embodiment import BimanualYamEmbodiment
    from yam_arena.scene import task_room,env_cfg_callback
    from isaaclab.assets import RigidObjectCollectionCfg
    from isaaclab.managers import EventTermCfg

    embodiment=BimanualYamEmbodiment(enable_cameras=enable_cameras,contact_sensors=False,rig=station.rig())
    objects=task_room(SCENE,embodiment.rig.top_cam)
    specs=object_specs()
    for name,row in specs.items():
        a=math.radians(row.get('yaw',0))/2
        obj=Object(name=name,spawner_cfg=sim_utils.UsdFileCfg(usd_path=str(ROOT/'assets'/row['asset']),activate_contact_sensors=row['dynamic']),
            object_type=ObjectType.RIGID if row['dynamic'] else ObjectType.BASE,
            initial_pose=Pose(position_xyz=tuple(row['position']),rotation_xyzw=tuple(row.get('rotation_xyzw',(0,0,math.sin(a),math.cos(a))))))
        if batch_ice and 'ice_source' in row:obj.disable_reset_pose()
        objects.append(obj)
    objects.extend(extra_objects)
    configure_base=env_cfg_callback(control_hz)
    def configure(env_cfg):
        env_cfg=configure_base(env_cfg)
        if batch_ice:
            cubes={name:getattr(env_cfg.scene,name) for name,row in specs.items() if 'ice_source' in row}
            for name in cubes:setattr(env_cfg.scene,name,None)
            env_cfg.scene.ice=RigidObjectCollectionCfg(rigid_objects=cubes)
            env_cfg.events.reset_ice=EventTermCfg(func=reset_ice_collection,mode='reset')
        return env_cfg
    cfg=IsaacLabArenaEnvironment(name='local_clean_ice_fish',scene=Scene(objects),
        embodiment=embodiment,env_cfg_callback=configure)
    return ArenaEnvBuilder(cfg,ArenaEnvBuilderCfg(num_envs=1,solve_relations=False,device='cuda:0')).make_registered()
