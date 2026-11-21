"""Dual-YAM station and native joint-position controller.

The controller is adapted from the previous RoboCurve toilet fixtures in this workspace.
"""

import math
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from dream_examples.yam_mujoco import yam_env

HZ = 20
DT = 0.001
SIDES = ("left", "right")
FINGER_OPEN = 0.0475
MAX_STEPS = 1200


def element(parent, tag, **attrs):
    vals = {}
    for key, value in attrs.items():
        if isinstance(value, (list, tuple, np.ndarray)):
            value = yam_env.numbers(value)
        vals[key.rstrip("_")] = str(value)
    return ET.SubElement(parent, tag, vals)


def geom(parent, name, kind, pos, size, rgba, **attrs):
    return element(
        parent, "geom", name=name, type=kind, pos=pos, size=size, rgba=rgba, **attrs
    )


def station_xml():
    tree = ET.fromstring(yam_env.build_xml())
    tree.find("option").set("timestep", str(DT))
    tree.find("option").set("jacobian", "dense")
    tree.find("visual/global").set("offwidth", "960")
    tree.find("visual/global").set("offheight", "540")
    world = tree.find("worldbody")
    turn = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    shift = np.array([0.041, 0.0, -0.597])
    q_turn = np.array([2**-0.5, 0.0, 0.0, 2**-0.5])
    for item in list(world):
        name = item.get("name")
        if name not in ("top", "left_base", "right_base") and item.tag != "light":
            world.remove(item)
            continue
        if item.tag == "light":
            continue
        p = np.fromstring(item.get("pos"), sep=" ")
        q = np.fromstring(item.get("quat"), sep=" ")
        rotated = np.empty(4)
        mj.mju_mulQuat(rotated, q_turn, q)
        pos = turn @ p + shift
        if name in ("left_base", "right_base"):
            pos[1] *= 0.59
        item.set("pos", yam_env.numbers(pos))
        item.set("quat", yam_env.numbers(rotated))
    for side in (-1, 1):
        geom(
            world,
            f"pedestal_{side}",
            "box",
            (-0.199, side * 0.17995, 0.077),
            (0.075, 0.065, 0.078),
            (0.19, 0.24, 0.28, 1),
        )
    fixture = element(tree.find("default"), "default", class_="fixture")
    element(fixture, "geom", friction="1.1 .02 .003", solref=".006 1", condim="4")
    element(fixture, "joint", damping=".1", armature=".00001")
    scene = element(world, "body", name="workbench_fixture", childclass="fixture")
    return tree, scene


class Station:
    def __init__(self, fixture, render=None, model=None):
        self.fixture = fixture
        self.model = fixture.compile() if model is None else model
        self.data = mj.MjData(self.model)
        self.ik_data = mj.MjData(self.model)
        self.targets = np.array([[0.0, 1.0, 1.0, -0.6, 0.0, 0.0, 1.0]] * 2)
        self.yaw_targets = {}
        self.yaw_choices = {}
        self.joints = {
            side: [self.model.joint(f"{side}_joint{i}") for i in range(1, 7)]
            for side in SIDES
        }
        self.fingers = {
            side: [f"{side}_finger_{sign}" for sign in (-1, 1)] for side in SIDES
        }
        self.sites = {side: self.model.site(f"{side}_tcp").id for side in SIDES}
        self.steps = 0
        self.trace = []
        self.commands = []
        self.phase = "Starting state"
        self.render = render
        self.set_targets(self.targets)
        for i, side in enumerate(SIDES):
            for joint, pos in zip(self.joints[side], self.targets[i, :6]):
                self.data.qpos[joint.qposadr[0]] = pos
            for name in self.fingers[side]:
                self.data.joint(name).qpos[0] = FINGER_OPEN
        fixture.reset(self)
        mj.mj_forward(self.model, self.data)
        fixture.sample(self)
        if render:
            render.attach(self)
            render.draw()

    def joint(self, name):
        return float(self.data.joint(name).qpos[0])

    def speed(self, name):
        return float(self.data.joint(name).qvel[0])

    def position(self, name):
        return self.data.body(name).xpos.copy()

    def action(self, targets):
        if self.steps >= MAX_STEPS:
            raise RuntimeError("Trial action limit reached")
        self.set_targets(targets)
        for _ in range(round(1 / (HZ * DT))):
            self.fixture.before_step(self)
            mj.mj_step(self.model, self.data)
        mj.mj_forward(self.model, self.data)
        self.fixture.sample(self)
        self.steps += 1
        self.targets[:] = targets
        self.commands.append(np.asarray(targets).reshape(-1).tolist())
        if self.steps % 2 == 0:
            self.trace.append(self.reading())
            if self.render:
                self.render.draw()

    def reading(self):
        return {
            "step": self.steps,
            "seconds": self.steps / HZ,
            "front_from_reset": self.joint("front_angle") / math.tau,
            "rear_from_reset": self.joint("rear_angle") / math.tau,
            "phase": self.phase,
            **self.fixture.measure(self),
        }

    def wait(self, seconds, phase=None):
        if phase:
            self.phase = phase
        for _ in range(round(seconds * HZ)):
            self.action(self.targets)

    def set_targets(self, targets):
        targets = np.asarray(targets, dtype=float)
        if targets.shape != (2, 7) or not np.isfinite(targets).all():
            raise ValueError(
                "Expected two sets of six YAM joint targets and gripper opening"
            )
        for i, side in enumerate(SIDES):
            for joint, angle in zip(self.joints[side], targets[i, :6]):
                self.data.actuator(joint.name).ctrl[0] = np.clip(angle, *joint.range)
            for name in self.fingers[side]:
                self.data.actuator(name).ctrl[0] = FINGER_OPEN * np.clip(
                    targets[i, -1], 0, 1
                )

    def solve(self, side, goal, yaw=0.0, initial=None, rotation=None):
        data, model = self.ik_data, self.model
        data.qpos[:] = self.data.qpos
        pos_indices = [j.qposadr[0] for j in self.joints[side]]
        vel_indices = [j.dofadr[0] for j in self.joints[side]]
        site = self.sites[side]
        limits = np.array([j.range for j in self.joints[side]])
        if initial is not None:
            data.qpos[pos_indices] = initial
        if rotation is None:
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c, s, 0], [s, -c, 0], [0, 0, -1]])
        jac_pos, jac_rot = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        relative = np.empty(4)
        rot_error = np.empty(3)
        for _ in range(100):
            mj.mj_kinematics(model, data)
            mj.mj_comPos(model, data)
            position_error = goal - data.site_xpos[site]
            mj.mju_mat2Quat(
                relative, (rotation @ data.site_xmat[site].reshape(3, 3).T).ravel()
            )
            if relative[0] < 0:
                relative *= -1
            mj.mju_quat2Vel(rot_error, relative, 1)
            if (
                np.linalg.norm(position_error) < 0.0006
                and np.linalg.norm(rot_error) < 0.012
            ):
                break
            mj.mj_jacSite(model, data, jac_pos, jac_rot, site)
            jac = np.vstack((jac_pos[:, vel_indices], 0.27 * jac_rot[:, vel_indices]))
            error = np.r_[position_error, 0.27 * rot_error]
            delta = jac.T @ np.linalg.solve(jac @ jac.T + 0.002 * np.eye(6), error)
            current = data.qpos[pos_indices]
            data.qpos[pos_indices] = np.clip(
                current + np.clip(delta, -0.18, 0.18), *limits.T
            )
        return (
            data.qpos[pos_indices].copy(),
            float(np.linalg.norm(position_error)),
            float(np.linalg.norm(rot_error)),
        )

    def move_arm(self, side, goal, seconds, phase, opening=None, yaw=0.0):
        index = SIDES.index(side)
        start_pose = self.data.site_xpos[self.sites[side]].copy()
        start = self.targets.copy()
        target = start.copy()
        count = max(1, round(seconds * HZ))
        old = self.yaw_choices.get(side)
        if old is None or old[0] != yaw:
            options = [
                (yaw, self.solve(side, np.asarray(goal), yaw)),
                (yaw + math.pi, self.solve(side, np.asarray(goal), yaw + math.pi)),
            ]
            final_yaw, _ = min(
                options,
                key=lambda item: item[1][1]
                + 0.028 * item[1][2]
                + 0.002 * np.linalg.norm(item[1][0] - target[index, :6]),
            )
            self.yaw_choices[side] = (yaw, final_yaw)
        yaw = self.yaw_choices[side][1]
        start_yaw = self.yaw_targets.get(side, yaw)
        turn = math.remainder(yaw - start_yaw, math.tau)
        self.phase = phase
        for i in range(count):
            t = (i + 1) / count
            weight = t * t * (3 - 2 * t)
            pos = start_pose + weight * (np.asarray(goal) - start_pose)
            q, error, rot = self.solve(
                side, pos, start_yaw + weight * turn, target[index, :6]
            )
            if i + 1 == count and (error > 0.014 or rot > 0.48):
                raise RuntimeError(
                    f"{side} {phase} IK error {error:.4f} m / {rot:.3f} rad"
                )
            target[index, :6] = q
            if opening is not None:
                target[index, -1] = start[index, -1] + weight * (
                    opening - start[index, -1]
                )
            self.action(target)
        self.yaw_targets[side] = yaw
