"""Read-only post-reset measurements and native contact sensors. No object motion API."""
import os
import numpy as np
from config import HZ, S, F, P, PAD_H, FOOT_CENTERS, NUMERICS
from partkit import on_reset, on_step, _np, _part_state, _SPECS
from scoring import Scorer, rotation

monitor=Scorer()
last_record={}
_last_tick=None
PAD_NAMES=[f'slider_{i}' for i in range(int(os.environ.get('FURNITURE_SLIDERS_PAD_COUNT','4')))]
HISTORY=NUMERICS['physics_hz']//HZ


def add_sensors(embodiment):
    from isaaclab.sensors import ContactSensorCfg
    from isaaclab_arena.utils.configclass import make_configclass
    from observer import add_camera
    add_camera(embodiment.scene_config)
    robot_paths=['{ENV_REGEX_NS}/'+prim+embodiment.link_paths[n]
                 for prim in ('LeftArm','RightArm')
                 for n in ('base','link1','link2','link3','link4','link5','gripper','tip_left','tip_right')]
    for name in PAD_NAMES:
        setattr(embodiment.scene_config, 'contact_'+name, ContactSensorCfg(
            prim_path='{ENV_REGEX_NS}/'+name+'/body',
            filter_prim_paths_expr=['{ENV_REGEX_NS}/furniture/body']+
                ['{ENV_REGEX_NS}/floor_panel/body/mesh_0']+robot_paths,
            update_period=0.,history_length=HISTORY))
    setattr(embodiment.scene_config,'furniture_robot_contacts',ContactSensorCfg(
        prim_path='{ENV_REGEX_NS}/furniture/body',filter_prim_paths_expr=robot_paths,
        update_period=0.,history_length=HISTORY))
    for side,prim in [('left','LeftArm'),('right','RightArm')]:
        for tip in ('tip_left','tip_right'):
            setattr(embodiment.scene_config,side+'_'+tip,ContactSensorCfg(
                prim_path='{ENV_REGEX_NS}/'+prim+embodiment.link_paths[tip],
                filter_prim_paths_expr=['{ENV_REGEX_NS}/furniture/body']+
                ['{ENV_REGEX_NS}/'+n+'/body' for n in PAD_NAMES],history_length=HISTORY))
        setattr(embodiment.scene_config,side+'_all_contacts',ContactSensorCfg(
            prim_path='{ENV_REGEX_NS}/'+prim+'/.*',history_length=HISTORY))
    fields=[(n,type(v),v) for n,v in vars(embodiment.scene_config).items() if not n.startswith('_')]
    embodiment.scene_config=make_configclass('FurnitureSlidersSceneCfg',fields)()


def arm_state(base,side):
    from yam_arena.constants import ARM_JOINTS
    robot=base.scene[side+'_robot']
    names=list(robot.joint_names)
    ids=[names.index(n) for n in ARM_JOINTS]
    return _np(robot.data.joint_pos)[0][ids]


def measure(environment,step):
    from yam_arena.analysis import world_hulls,pose7_to_matrix,box_signed_distance
    base=environment.unwrapped
    scene=base.scene
    furniture=_part_state(environment,_SPECS['furniture'])
    fp=np.r_[furniture.pos,furniture.quat_xyzw]
    pads={n:_part_state(environment,_SPECS[n]) for n in PAD_NAMES}
    poses={n:np.r_[p.pos,p.quat_xyzw].tolist() for n,p in pads.items()}
    contacts={n:np.linalg.norm(_np(scene['contact_'+n].data.normal_force_matrix_w_history)[0,:,0],axis=-1).mean(0).tolist() for n in PAD_NAMES}
    detail={'furniture_velocity':np.r_[furniture.lin_vel,furniture.ang_vel].tolist(),
            'pad_contacts_N':contacts,'arms':{}}
    pad_touched={n:bool(np.any(np.asarray(contacts[n][2:])>.1)) for n in PAD_NAMES}
    robot_contact=bool(np.linalg.norm(_np(scene['furniture_robot_contacts'].data.normal_force_matrix_w_history)[0,:,0],
                                      axis=-1).mean(0).max()>.1)
    clearances=[]
    for side in ('left','right'):
        robot=scene[side+'_robot']
        bodies=_np(robot.data.body_link_pose_w)[0]
        body_map={n:pose7_to_matrix(p) for n,p in zip(robot.body_names,bodies)}
        hulls=world_hulls(body_map,links=['gripper','tip_left','tip_right'])
        vertices=np.concatenate([h.vertices for _,h in hulls])
        minimum=float('inf')
        for box in _SPECS['furniture'].boxes:
            minimum=min(minimum,float(box_signed_distance(vertices,furniture.to_world(box[0]),
                                                          furniture.rotation(),box[1]).min()))
        for n,p in pads.items():
            local=(vertices-p.pos)@p.rotation()
            # Bounding cylinder is conservative for hand release, including foam/backing.
            q=np.column_stack([np.linalg.norm(local[:,:2],axis=1)-P['diameter_m']/2,
                               abs(local[:,2]-PAD_H/2)-PAD_H/2])
            dist=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(q.max(axis=1),0)
            minimum=min(minimum,float(dist.min()))
        fingers={};finger_peaks={}
        for tip in ('tip_left','tip_right'):
            history=np.linalg.norm(_np(scene[side+'_'+tip].data.normal_force_matrix_w_history)[0,:,0],axis=-1)
            force=history.mean(0)
            fingers[tip]=force.tolist()
            finger_peaks[tip]=history.max(0).tolist()
            if np.any(force>.1): minimum=min(minimum,0.)
            for i,n in enumerate(PAD_NAMES):
                pad_touched[n]=pad_touched[n] or bool(force[i+1]>.1)
        clearances.append(minimum)
        all_force=_np(scene[side+'_all_contacts'].data.net_normal_forces_w_history)[0].mean(0)
        detail['arms'][side]={'q':arm_state(base,side).tolist(),'finger_contacts_N':fingers,
                              'finger_peak_contacts_N':finger_peaks,
                              'joint_names':list(robot.joint_names),
                              'joint_positions':_np(robot.data.joint_pos)[0].tolist(),
                              'joint_velocities':_np(robot.data.joint_vel)[0].tolist(),
                              'net_contacts_N':{n:f.tolist() for n,f in zip(scene[side+'_all_contacts'].body_names,all_force)},
                              'clearance_m':minimum}
        for key in ('applied_torque','gravity_compensation_forces'):
            if hasattr(robot.data,key):
                detail['arms'][side][key]=_np(getattr(robot.data,key))[0].tolist()
    measurement={'step':step,'furniture':fp.tolist(),'pads':poses,'hand_clearance_m':min(clearances),
                 'pad_loads_N':{n:f[0] for n,f in contacts.items()},'pad_held':pad_touched,'robot_contact':robot_contact}
    return measurement,detail


@on_reset
def reset(environment):
    global _last_tick,last_record
    monitor.reset()
    _last_tick=environment.unwrapped.common_step_counter
    m,d=measure(environment,0)
    last_record={'measurement':m,'diagnostics':d,'scoring':monitor.update(m)}


@on_step
def update(environment):
    global _last_tick,last_record
    tick=environment.unwrapped.common_step_counter
    if tick==_last_tick: return
    _last_tick=tick
    m,d=measure(environment,monitor.previous['step']+1)
    last_record={'measurement':m,'diagnostics':d,'scoring':monitor.update(m)}
