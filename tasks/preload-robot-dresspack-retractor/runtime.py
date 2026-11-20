"""Small helpers for the lab's PhysX station task adapters."""
from __future__ import annotations

import numpy as np


def bodies(environment, names):
    from yam_arena.scene import object_states

    return object_states(environment, names)


def contacts(environment, names):
    import touch

    return touch.touching(environment, names)


def reset_contacts():
    import touch

    touch.reset()


def asset(environment, name):
    return environment.unwrapped.scene[name]


def scalar_array(value):
    value = getattr(value, "torch", value)
    return value.detach().cpu().numpy() if hasattr(value, "detach") else np.asarray(value)


def link_pose(environment, name, link):
    """Read a named link in station coordinates, XYZW quaternion."""
    unit = asset(environment, name)
    idx = unit.body_names.index(link)
    origin = scalar_array(environment.unwrapped.scene.env_origins)[0]
    at = scalar_array(unit.data.body_pos_w)[0, idx] - origin
    q = scalar_array(unit.data.body_quat_w)[0, idx]
    return at, rotation(q)


def rotation(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def velocity(asset):
    return asset.data.root_vel_w


def tick(environment):
    return float(environment.unwrapped.step_dt)


def install_writer(unit, step):
    before = unit.write_data_to_sim

    def with_forces():
        step()
        before()

    unit.write_data_to_sim = with_forces


def wrench(unit, force, torque=None, ids=None):
    kwargs = {"forces": force, "is_global": True}
    if torque is not None:
        kwargs["torques"] = torque
    if ids is not None:
        kwargs["body_ids"] = ids
    unit.permanent_wrench_composer.set_forces_and_torques_index(**kwargs)


def to_tensor(vec, like):
    import torch

    return torch.as_tensor(vec, dtype=like.dtype, device=like.device)


def paint_prim(environment, object_name, subpath, rgb):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdGeom

    unit = asset(environment, object_name)
    path = unit.root_view.prim_paths[0]
    stage = get_current_stage()
    prim = stage.GetPrimAtPath(path + "/" + subpath)
    if not prim:
        prim = stage.GetPrimAtPath(path.rsplit("/", 1)[0] + "/" + subpath)
    if prim:
        UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*rgb)])


def register(objects, reset, update):
    from scenekit import on_reset, on_step

    on_reset(reset)
    on_step(update)
    return objects
