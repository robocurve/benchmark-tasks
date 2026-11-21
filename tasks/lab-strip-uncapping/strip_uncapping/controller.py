"""Reference policy using the same gripper and joint commands as learned policies."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config, plan

REGISTERED = (
    "register the prongs",
    "insert both",
    "settle the prongs",
    "pry upward",
    "pry beyond",
    "lift the linked",
)


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        self.feedback = None
        self.info = PolicyInfo(
            name=config.KEY + "_scripted",
            action_space=action_box(
                np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0],
                np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0],
            ),
        )
        self.reset(None)

    def bind(self, info):
        pass

    def reset(self, scene):
        self.moves = plan.waypoints(self.mode)
        self.q = {n: np.asarray(READY).copy() for n in self.moves}
        self.starts = {
            n: (*ik.tcp_pose(self.q[n], station.BASES[n]), 1.0) for n in self.q
        }
        self.previous = np.r_[READY, 1.0, READY, 1.0]
        self.integral = np.zeros(14)
        self.tool_trim = np.zeros(3)
        travel = max(
            (sum(w["duration"] for w in v) for v in self.moves.values()), default=0
        )
        self.duration = config.MAX_STEPS / 20 if self.mode == "hold" else travel + 1.0
        self.worst_ik_m = self.worst_ik_rad = 0.0
        self.phase = "ready"

    def register(self, p, r, measured):
        tool, rack, _ = self.feedback()
        tool_axes = quat_xyzw_to_matrix(tool[3:])
        rack_axes = quat_xyzw_to_matrix(rack[3:])
        if rack_axes[2, 2] < 0.993 or abs(rack[2] - config.RACK[2]) > 0.006:
            self.tool_trim *= 0.9
            return p, r
        hand, hand_axes = ik.tcp_pose(measured[7:13], station.BASES["right"])
        desired_axes = r @ plan.TOP.T
        nominal_origin = p - desired_axes @ plan.TOOL_GRIP_OFFSET
        origin = rack[:3] + rack_axes @ (nominal_origin - config.RACK)
        desired_axes = rack_axes @ desired_axes
        grasp = tool_axes.T @ (hand - tool[:3])
        error = np.clip(origin - tool[:3], -0.006, 0.006)
        self.tool_trim = np.clip(self.tool_trim + 0.08 * error, -0.004, 0.004)
        correct = origin + desired_axes @ grasp + self.tool_trim + 0.25 * error
        return p + np.clip(
            correct - p, -0.016, 0.016
        ), desired_axes @ tool_axes.T @ hand_axes

    def action_at(self, t, joint_pos=None):
        controls = []
        phases = []
        for side in self.moves:
            p, r, grip = self.starts[side]
            age = t
            label = ""
            for entry in self.moves[side]:
                target_p = np.asarray(entry["position"])
                target_r = np.asarray(entry["rotation"])
                target_g = entry["grip"]
                span = entry["duration"]
                if age < span:
                    blend = ik.smoothstep(age / span)
                    p = p + blend * (target_p - p)
                    r = ik.slerp_matrix(r, target_r, blend)
                    grip += blend * (target_g - grip)
                    label = entry["label"]
                    break
                age -= span
                p, r, grip = target_p, target_r, target_g
            if (
                side == "right"
                and self.feedback
                and joint_pos is not None
                and label.startswith(REGISTERED)
            ):
                p, r = self.register(p, r, joint_pos)
            q, pe, re = ik.solve(
                p,
                ik.matrix_to_quat_xyzw(r),
                self.q[side],
                base_pos=station.BASES[side],
                iters=60,
            )
            if pe > 0.002 or re > 0.035:
                raise RuntimeError(
                    f"{side} pose failed in {label!r}: {pe:.5f} m, {re:.5f} rad"
                )
            self.q[side] = q
            controls.extend([*q, grip])
            self.worst_ik_m = max(self.worst_ik_m, pe)
            self.worst_ik_rad = max(self.worst_ik_rad, re)
            if label and label not in phases:
                phases.append(label)
        target = np.asarray(controls)
        if joint_pos is not None:
            error = target - joint_pos
            error[[6, 13]] = 0.0
            self.integral = np.clip(self.integral + 0.044 * error, -0.15, 0.15)
            target += self.integral
        target = np.clip(
            target, self.info.action_space.low, self.info.action_space.high
        )
        self.previous = self.previous + np.clip(target - self.previous, -0.04, 0.04)
        self.phase = " / ".join(phases) or "settle"
        return self.previous

    def act(self, observation):
        t = float(observation.state_time) + 0.05
        return ActionChunk(
            actions=[
                Action(
                    data=self.action_at(t, observation.state["joint_pos"]),
                    meta={"request_stop": True} if t > self.duration else {},
                )
            ],
            control_hz=20.0,
        )


def policy(**kwargs):
    return ScriptedPolicy(**kwargs)
