"""Small helpers for the lab's PhysX station task adapters."""
from __future__ import annotations

import numpy as np


def asset(environment, name):
    return environment.unwrapped.scene[name]


def scalar_array(value):
    value = getattr(value, "torch", value)
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return np.asarray(value.numpy() if hasattr(value, "numpy") else value)


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


def register(objects, reset, update):
    from scenekit import on_reset, on_step

    on_reset(reset)
    on_step(update)
    return objects
