"""Native reference using elapsed time and ordinary joint positions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config, plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
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
        self.guide = None
        self.stock_released = False
        self.q = {n: np.asarray(config.STOWED[n]).copy() for n in ("left", "right")}
        self.starts = {
            n: (*ik.tcp_pose(self.q[n], config.BASES[n]), 1.0) for n in self.q
        }
        self.previous = np.r_[config.STOWED["left"], 1.0, config.STOWED["right"], 1.0]
        self.integral = np.zeros(14)
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def action_at(self, t, joint_pos=None, feedback=None):
        if (
            feedback is not None
            and self.guide is None
            and t >= 6.23
            and self.mode not in ("hold", "crank-only")
        ):
            stock = np.asarray(feedback["stock_links"])
            tcp, hand_frame = ik.tcp_pose(
                np.asarray(joint_pos[:6]), config.BASES["left"]
            )
            stock_frame = quat_xyzw_to_matrix(stock[1, 3:])
            desired = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
            hand_to_stock = hand_frame.T @ stock_frame
            guide_rot = desired @ hand_to_stock.T
            shift = guide_rot @ hand_frame.T @ (stock[:3, :3].mean(axis=0) - tcp)
            self.guide = guide_rot, shift
            self.moves = plan.waypoints(self.mode, self.guide)
        out = []
        labels = []
        for name in ("left", "right"):
            pos, rot, grip = self.starts[name]
            age = t
            label = ""
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
                    break
                age -= w["duration"]
                pos, rot, grip = goal, r, g
            if name == "right" and feedback is not None:
                pos, grip = self.track_crank(t, label, pos, grip, feedback)
            if name == "left" and self.guide is not None and feedback is not None:
                pos, rot, grip = self.feed_until_caught(t, pos, rot, grip, feedback)
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=config.BASES[name],
                iters=60,
            )
            if pe > 0.002 or re > 0.03:
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

    def feed_until_caught(self, time, pos, rot, grip, feedback):
        if self.stock_released or time < 14.85:
            return pos, rot, grip
        tip_height = feedback["stock_links"][-1][2]
        if tip_height < config.NIP[2] - 0.011 or time >= 24.0:
            self.stock_released = True
            return pos, rot, grip
        guide_rot, shift = self.guide
        advance = min((time - 14.85) * 0.0075, 0.025)
        # Keep the genuine jaw grip and feed until the stock is seen below the
        # nip. Releasing just above this point leaves it balanced on one roll.
        return config.NIP + [0.0, 0.0, 0.061 - advance] - shift, guide_rot, 0.024

    @staticmethod
    def track_crank(t, label, pos, grip, feedback):
        """Follow the appliance's moving knob in the same way as a visual servo."""
        sleeve = np.asarray(feedback["crank_sleeve"])
        shaft = np.asarray(feedback["rollers"][0])
        if 2.4 <= t < 4.6:
            progress = np.clip((t - 2.4) / 1.30, 0.0, 1.0)
            gap = 0.078 * (1.0 - ik.smoothstep(progress))
            return sleeve - gap * plan.CRANK[:, 2], 0.72 if t < 3.72 else 0.18
        if 4.6 <= t < 9.5:
            return sleeve, 0.114
        if "crank while" in label or "draw the outfeed" in label:
            radial = sleeve - shaft
            tangent = np.cross([0.0, 1.0, 0.0], radial)
            tangent /= np.linalg.norm(tangent)
            # A small tangential lead loads the physical bearing through the
            # gripper. No action sets the shaft position or its velocity.
            lead = 0.009 if t < 29.0 else 0.019
            return sleeve + lead * tangent, 0.082 if t < 29.0 else 0.045
        return pos, grip

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
