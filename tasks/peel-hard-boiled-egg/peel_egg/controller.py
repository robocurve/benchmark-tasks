"""Native reference using elapsed time and ordinary joint positions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY
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
        self.q = {n: np.asarray(READY).copy() for n in ("left", "right")}
        self.starts = {
            n: (*ik.tcp_pose(self.q[n], config.BASES[n]), 1.0) for n in self.q
        }
        self.previous = np.r_[READY, 1.0, READY, 1.0]
        self.integral = np.zeros(14)
        self.serve_after_peel = self.mode == "demo"
        self.regrasp_attempted = False
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def action_at(self, t, joint_pos=None, scene_state=None):
        if self.serve_after_peel and t >= self.duration - 2:
            self._start_plate_reference(joint_pos, scene_state)
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
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=config.BASES[name],
                iters=60,
            )
            if pe > 0.006 or re > 0.06:
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
            self.integral = np.clip(self.integral + 1.75 * 0.05 * error, -0.24, 0.24)
            target = target + self.integral + 0.40 * error
        target = np.clip(
            target, self.info.action_space.low, self.info.action_space.high
        )
        # Also pass Inspect's default delta guard after this taught command ramp.
        result = self.previous + np.clip(target - self.previous, -0.04, 0.04)
        self.previous = result
        self.phase = " / ".join(labels) or "settle"
        return result

    def _start_plate_reference(self, joint_pos, scene_state):
        self.serve_after_peel = False
        actual = {}
        for i, (name, moves) in enumerate(self.moves.items()):
            last = moves[-1]
            if joint_pos is None:
                pos, rot = np.asarray(last["position"]), np.asarray(last["rotation"])
            else:
                pos, rot = ik.tcp_pose(np.asarray(joint_pos)[i * 7:i * 7 + 6], config.BASES[name])
            actual[name] = (pos, rot, last["grip"])
        if scene_state is None:
            holder, egg = "left", config.WORK
            remainder = plan.plate_reference(holder, actual, egg)
        else:
            egg = np.asarray(scene_state["objects"]["core"]["pose"][:3])
            holder = min(actual, key=lambda side: np.linalg.norm(actual[side][0] - egg))
            opening = float(joint_pos[6 if holder == "left" else 13])
            pos, rot, grip = actual[holder]
            band = np.asarray(scene_state["objects"]["band"]["pose"])
            still_bonded = np.linalg.norm(band[:3] - egg) < .018
            if still_bonded and holder == "left" and not self.regrasp_attempted:
                # The first crack can leave the band rolled away from the
                # taught pinch. Take one fresh grip aligned to its visible rim.
                self.regrasp_attempted = True
                self.serve_after_peel = True
                remainder = plan.regrasp_band(actual, band[:3], quat_xyzw_to_matrix(band[3:]))
            else:
                # Use 3 to 4 mm of nominal preload on the now-exposed white.
                actual[holder] = pos, rot, max(grip, opening - .04)
                remainder = plan.plate_reference(holder, actual, egg)
        for name in self.moves:
            self.moves[name].extend(remainder[name])
        self.duration = max(sum(w["duration"] for w in moves) for moves in self.moves.values()) + 2

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
