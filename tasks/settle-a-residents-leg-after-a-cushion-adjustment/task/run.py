"""A trial returns raw observations and actuator commands at 20 Hz."""

import json
from pathlib import Path

import mujoco as mj
import numpy as np

from .measure import Reader
from .observation import Cameras, action_vector
from .witness import Witness
from .policy import Push
from .station import HZ, STEPS, SUBSTEPS, make_model, reset


def voluntary_torque(d, t, delay):
    # A brief voluntary lift followed by complete relaxation. No robot state enters this function.
    return lift_moment(
        float(d.joint("knee").qpos[0]), float(d.joint("knee").qvel[0]), t, delay
    )


def lift_moment(angle, rate, t, delay):
    t -= delay
    if not 0.30 < t < 6.2:
        return 0.0
    blend = min(1.0, (t - 0.30) / 1.0, (6.2 - t) / 1.15)
    target = -0.51 * blend
    # No downward drive as the resident relaxes.
    return float(
        np.clip(blend * (-7.3 + 42 * (target - angle) - 4.2 * rate), -14.0, 0.0)
    )


def trial(
    mode="reference",
    delay=0.0,
    resident_y=0.0,
    stiffness=850.0,
    leg_mass=3.0,
    cushion_y=0.020,
    video=None,
    action_provider=None,
):
    m = make_model(
        cushion_y=cushion_y,
        stiffness=stiffness,
        leg_mass=leg_mass,
        resident_y=resident_y,
    )
    d = reset(m)
    reader = Reader(m)
    policy = Push(m, mode)
    rows = []
    obs = reader.sample(m, d, 0.0)
    knee = m.joint("knee").dofadr[0]
    cameras = Cameras(m) if action_provider is not None else None
    instruction = json.loads(
        Path(__file__).with_name("task.template.json").read_text()
    )["instruction"]
    try:
        return episode(
            m,
            d,
            reader,
            policy,
            rows,
            obs,
            knee,
            delay,
            video,
            cameras,
            instruction,
            action_provider,
        )
    finally:
        if cameras is not None:
            cameras.close()


def episode(
    m,
    d,
    reader,
    policy,
    rows,
    obs,
    knee,
    delay,
    video,
    cameras,
    instruction,
    action_provider,
):
    witness = Witness(m)
    witness.sample(m, d, obs["heel_n"], 0)
    for tick in range(STEPS):
        if action_provider is None:
            q = policy.command(tick, obs)
        else:
            q = action_vector(m, action_provider(cameras.observe(d, instruction)))
        d.ctrl[:] = q
        direct = 0.0
        squeeze = 0.0
        for sub in range(SUBSTEPS):
            t = (tick + sub / SUBSTEPS) / HZ
            torque = voluntary_torque(d, t, delay)
            d.qfrc_applied[knee] = torque
            mj.mj_step(m, d)
            # Average robot-to-person normal load over the full action interval.
            load, pinch = reader.robot_person(m, d)
            direct += load
            squeeze += pinch
        obs = reader.sample(m, d, torque)
        obs["witness"] = witness.sample(m, d, obs["heel_n"], torque)
        obs["robot_person_mean_n"] = direct / SUBSTEPS
        obs["opposed_grip_mean_n"] = squeeze / SUBSTEPS
        rows.append(dict(tick=tick, action=q.tolist(), **obs))
        if video:
            video(m, d, rows[-1])
    return rows
