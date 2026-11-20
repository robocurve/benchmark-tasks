"""Native reference using elapsed time, joint positions, and read-only native object poses."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY

from . import config, plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(
            name=config.KEY + "_scripted", action_space=action_box(low, high)
        )
        self.feedback = None
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
        self.aim_offset = np.zeros(3)
        self.press_trim = 0.0
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def action_at(self, t, joint_pos=None):
        out = []
        labels = []
        native_pose = None if self.feedback is None else self.feedback()
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
            if native_pose is not None and self.mode != "hold" and name == "right":
                sensor, tool, stroke = native_pose
                if label in ("straddle both grips", "close on housing and probe"):
                    tool_r = ik.quat_xyzw_to_matrix(tool[3:])
                    error = tool[:3] + config.TOOL_GRIP * tool_r[:, 2] - (
                        config.TOOL_START + [0, 0, config.TOOL_GRIP]
                    )
                    self.aim_offset += 0.16 * (np.clip(error, -0.025, 0.025) - self.aim_offset)
                    pos += self.aim_offset
                elif label.startswith(("aim ", "indent ", "capture ", "lift ")):
                    index = int(label.split()[-1])
                    nominal = config.SENSOR_WORK + plan.SENSOR_R @ np.r_[
                        config.REGIONS[index], config.PAD_Z + config.BALL_R - config.BALL_Z
                    ]
                    sensor_r = ik.quat_xyzw_to_matrix(sensor[3:])
                    target = sensor[:3] + sensor_r @ np.r_[
                        config.REGIONS[index], config.PAD_Z + config.BALL_R
                    ]
                    tool_r = ik.quat_xyzw_to_matrix(tool[3:])
                    expected_to_ball = np.array([0., 0., config.BALL_Z - config.TOOL_GRIP])
                    actual_q = self.q[name] if joint_pos is None else np.asarray(joint_pos)[7:13]
                    tcp = ik.tcp_pose(actual_q, station.BASES[name])[0]
                    actual_to_ball = tool[:3] + tool_r[:, 2] * config.BALL_Z - tcp
                    error = target - (nominal + [0, 0, config.BALL_Z])
                    error += np.clip(expected_to_ball - actual_to_ball, -0.020, 0.020)
                    self.aim_offset += 0.12 * (np.clip(error, -0.035, 0.035) - self.aim_offset)
                    pos += self.aim_offset
                    if label.startswith("capture ") and self.mode == "demo":
                        self.press_trim = float(np.clip(
                            self.press_trim + 0.34 * (-stroke - 0.00126), -0.010, 0.006
                        ))
                    elif label.startswith("lift "):
                        self.press_trim *= 0.88
                    pos[2] += self.press_trim
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=station.BASES[name],
                iters=60,
            )
            if pe > 0.004 or re > 0.055:
                raise RuntimeError(
                    f"Unreachable {name} target during {label!r}: {pe:.6f} m, {re:.6f} rad"
                )
            self.q[name] = q
            out.extend([*q, grip])
            self.worst_ik_m = max(self.worst_ik_m, pe)
            self.worst_ik_rad = max(self.worst_ik_rad, re)
        target = np.asarray(out)
        if joint_pos is not None:
            error = target - np.asarray(joint_pos)
            error[[6, 13]] = 0.0
            self.integral = np.clip(self.integral + 0.65 * 0.05 * error, -0.18, 0.18)
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
