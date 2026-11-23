"""Initialize the station exactly once per episode, before the YAM settling steps."""
import json
import math
from functools import wraps
from pathlib import Path

from fixture import BODIES, KEY, WORLD, SHIELD_UP
from logic import reset_failures

START_JOINTS = {body[3]: (SHIELD_UP if body[3]=="shield_angle" else 0.)
                for body in BODIES if body[3]}


def install(env, task):
    """Anchor fixture initialization between Gym reset and YAM's normal READY settling."""
    if task is None or not hasattr(task.module, "initialize_fixture"):
        return env
    original_reset = env.reset

    @wraps(original_reset)
    def reset(*args, **kwargs):
        task.module.close_episode()
        result = original_reset(*args, **kwargs)
        task.module.initialize_fixture(env)
        return result

    env.reset = reset
    return env


def ordered_positions(names):
    if len(names)!=len(START_JOINTS) or set(names)!=set(START_JOINTS):
        raise ValueError(f"Unexpected lathe joints at reset: {names}")
    return [START_JOINTS[name] for name in names]


def initialize(base, torch):
    """Write actual state and neutral drive targets. Gravity loads the pad during settle."""
    rig = base.scene["guard"]
    insert = base.scene["insert"]
    device = base.device
    count = int(base.num_envs)
    q = torch.tensor([ordered_positions(rig.joint_names)],device=device,dtype=torch.float32).repeat(count,1)
    zero = torch.zeros_like(q)
    ids = torch.arange(count,device=device,dtype=torch.long)
    if hasattr(rig,"write_joint_position_to_sim_index"):
        rig.write_joint_position_to_sim_index(position=q,env_ids=ids)
        rig.write_joint_velocity_to_sim_index(velocity=zero,env_ids=ids)
        command = rig.actuators.target_command
        command.set_position_index(value=zero,env_ids=ids)
        command.set_velocity_index(value=zero,env_ids=ids)
    else:
        rig.write_joint_state_to_sim(q,zero,env_ids=ids)
        rig.set_joint_position_target(zero,env_ids=ids)
        rig.set_joint_velocity_target(zero,env_ids=ids)

    # Retain the SDK's native quaternion layout. Only replace translation and velocities.
    if hasattr(insert,"write_root_pose_to_sim_index"):
        default = insert.data.default_root_pose
        pose = getattr(default,"torch",default).clone()
    else:
        pose = insert.data.default_root_state[:,:7].clone()
    origins = base.scene.env_origins
    pose[:,:3] = getattr(origins,"torch",origins) + pose.new_tensor(tuple(a+b for a,b in zip(WORLD,KEY)))
    velocity = pose.new_zeros((count,6))
    if hasattr(insert,"write_root_pose_to_sim_index"):
        insert.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids)
        insert.write_root_velocity_to_sim_index(root_velocity=velocity,env_ids=ids)
    else:
        insert.write_root_pose_to_sim(pose,env_ids=ids)
        insert.write_root_velocity_to_sim(velocity,env_ids=ids)


def readback(base):
    from scenekit import _np
    rig = base.scene["guard"]
    q = dict(zip(rig.joint_names,map(float,_np(rig.data.joint_pos)[0])))
    speed = float(_np(rig.data.joint_vel)[0,rig.joint_names.index("shaft")])
    tab = _np(base.scene["insert"].data.root_link_pose_w)[0,:3]-_np(base.scene.env_origins)[0]-WORLD
    return dict(angle=q["shield_angle"],cam=q["cam_angle"],pad=q["pad_z"],seat=q["seat_z"],
                speed=speed,button=q["button_z"],tab_x=float(tab[0]),tab_y=float(tab[1]),tab_z=float(tab[2]))


def settled_failures(state):
    why = reset_failures(state["angle"],state["cam"],state["pad"],state["speed"],state["seat"])
    if not all(math.isfinite(v) for v in state.values()):
        why.append("nonfinite fixture measurement")
    if abs(state["button"])>=.005:
        why.append("start button depressed at reset")
    if (abs(state["tab_x"]-KEY[0])>.012 or abs(state["tab_y"]-KEY[1])>.012
            or not .071<=state["tab_z"]<=.096):
        why.append("insert outside service-pad starting position")
    return why


def record(trace_path, report):
    if trace_path:
        destination = Path(trace_path).with_name("reset_diagnostics.json")
        # Native runs are single-writer. Readers only see complete snapshots.
        tmp=destination.with_suffix(".tmp")
        tmp.write_text(json.dumps(report,indent=2)+"\n",encoding="utf8")
        tmp.replace(destination)
