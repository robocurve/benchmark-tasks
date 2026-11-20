"""Native reference using elapsed time and ordinary joint positions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY

from . import config, plan
from .scoring import leading_corners, link_frame


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        self.feedback = None
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(
            name=config.KEY + "_scripted", action_space=action_box(low, high)
        )
        self.reset(None)

    def bind(self, info):
        pass

    def reset(self, scene):
        self.moves = plan.waypoints(self.mode)
        self.q = {n: np.asarray(READY).copy() for n in ("left", "right")}
        self.starts = {
            n: (*ik.tcp_pose(self.q[n], station.BASES[n]), 1.0) for n in self.q
        }
        self.previous = np.r_[READY, 1.0, READY, 1.0]
        self.integral = np.zeros(14)
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.phase = "ready"
        self.entry_start = None
        self.wrist_translation = np.zeros(3)
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def register(self, position, orientation, measured, page, leading_goal, blend):
        hand, hand_rotation = ik.tcp_pose(measured[7:13], station.BASES["right"])
        _, paper_rotation = link_frame(page["links"]["Rear"])
        front = leading_corners(page).mean(axis=0)
        match = config.SLEEVE_ROTATION @ paper_rotation.T @ hand_rotation
        cosine = np.clip((np.trace(orientation.T @ match) - 1.) / 2., -1., 1.)
        angle = np.arccos(cosine)
        fraction = min(1., np.deg2rad(14.) / max(float(angle), 1e-6))
        match = ik.slerp_matrix(orientation, match, fraction * .91 * blend)
        registered = leading_goal - match @ hand_rotation.T @ (front - hand)
        correction = np.clip(registered - position, [-.035, -.047, -.07],
                             [.05, .047, .076]) * blend
        self.wrist_translation += .24 * (correction - self.wrist_translation)
        return position + self.wrist_translation, match

    def action_at(self, t, joint_pos=None):
        out = []
        labels = []
        for name in ("left", "right"):
            pos, rot, grip = self.starts[name]
            age = t
            label = ""
            lead_goal = None
            lead_previous = None
            weight = 0.
            for w in self.moves.get(name, []):
                goal = np.asarray(w["position"])
                r = np.asarray(w["rotation"])
                g = w.get("grip", grip)
                if age < w["duration"]:
                    s = ik.smoothstep(age / w["duration"])
                    pos = pos + s * (goal - pos)
                    rot = ik.slerp_matrix(rot, r, s)
                    grip += s * (g - grip)
                    label = w["label"]
                    labels.append(label)
                    if "leading_target" in w:
                        target = np.asarray(w["leading_target"])
                        if lead_previous is None and self.feedback is not None:
                            if self.entry_start is None:
                                self.entry_start = leading_corners(
                                    self.feedback()["objects"]["page"]).mean(axis=0)
                            lead_previous = self.entry_start
                        if lead_previous is not None:
                            lead_goal = (1 - s) * lead_previous + s * target
                            weight = s if label == plan.GUIDE_PHASES[0] else 1.
                    break
                age -= w["duration"]
                pos, rot, grip = goal, r, g
                if "leading_target" in w:
                    lead_previous = np.asarray(w["leading_target"])
            applied = (name == "right" and self.feedback is not None
                       and joint_pos is not None and lead_goal is not None)
            nominal = (pos, rot)
            if applied:
                page = self.feedback()["objects"]["page"]
                pos, rot = self.register(pos, rot, np.asarray(joint_pos), page, lead_goal, weight)
            base = station.BASES[name]
            seed = self.q[name]
            target = ik.matrix_to_quat_xyzw(rot)
            q, pe, re = ik.solve(pos, target, seed, base_pos=base, iters=60)
            if applied and (pe > .001 or re > .024):
                retry, rp, rr = ik.solve(pos, target, np.asarray(joint_pos)[7:13],
                                         base_pos=base, iters=85)
                if rp + .04 * rr < pe + .04 * re:
                    q, pe, re = retry, rp, rr
            if applied and (pe > .003 or re > .06):
                q, pe, re = ik.solve(nominal[0], ik.matrix_to_quat_xyzw(nominal[1]),
                                     seed, base_pos=base, iters=75)
            if pe > .005 or re > .09:
                raise RuntimeError(
                    f"Unreachable {name} reference target: {pe:.6f} m, {re:.6f} rad"
                )
            self.q[name] = q
            out.extend([*q, grip])
            self.worst_ik_m = max(self.worst_ik_m, pe)
            self.worst_ik_rad = max(self.worst_ik_rad, re)
        target = np.asarray(out)
        if joint_pos is not None:
            error = target - np.asarray(joint_pos)
            error[[6, 13]] = 0.0
            self.integral = np.clip(self.integral + 0.6 * 0.05 * error, -0.15, 0.15)
            target = target + self.integral
        target = np.clip(
            target, self.info.action_space.low, self.info.action_space.high
        )
        # Also pass Inspect's default delta guard after this taught command ramp.
        result = self.previous + np.clip(target - self.previous, -0.04, 0.04)
        self.previous = result
        self.phase = " / ".join(labels) or "settle"
        return result

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
