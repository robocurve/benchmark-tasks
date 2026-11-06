"""Robot-only scripted reference and causal controls; no apparatus state writes."""

from __future__ import annotations

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
from yam_arena.urdf import axis_angle_matrix

from .config import CAM_PIVOT, MOUNT_POSITION, PAD_POSITION, PAD_TOP, PAYLOAD_POSITION


def demo_plan(mode="demo"):
    """Absolute Cartesian poses in the calibrated station's world frame."""
    if mode not in {"demo", "hold", "direct", "open_only", "drop"}:
        raise ValueError("mode must be demo, hold, direct, open_only or drop")
    down = grip_rotation()
    plan = []

    def add(phase, seconds, arm=None, position=None, rotation=None, grip=None):
        stage = dict(phase=phase, seconds=seconds)
        if arm:
            stage[arm] = dict(position=np.asarray(position).tolist(),
                              rotation_xyzw=ik.matrix_to_quat_xyzw(down if rotation is None else rotation).tolist(),
                              grip=grip)
        plan.append(stage)

    if mode == "hold":
        return [dict(phase="hold reset", seconds=10)]
    pivot = MOUNT_POSITION+CAM_PIVOT
    lever = np.array([-0.020, -0.018, 0.008])
    if mode != "direct":
        start = pivot+lever
        add("approach cam", 2.0, "right", [start[0], start[1], 0.88], grip=0.35)
        add("lower to lever", 2.0, "right", start, grip=0.35)
        add("grasp lever", 1.0, "right", start, grip=0)
        for angle in np.linspace(0.15, 0.92, 7):
            rot = axis_angle_matrix(np.array([0, 0, 1]), angle)
            add("rotate cam", 0.5, "right", pivot+rot@lever, rot@down, 0)
        rot = axis_angle_matrix(np.array([0, 0, 1]), 0.92)
        finish = pivot+rot@lever
        add("release lever", 0.8, "right", finish, rot@down, 0.25)
        add("clear cam", 1.5, "right", finish+np.array([0, 0, 0.11]), rot@down, 0.25)
        add("park cam hand", 2.0, "right", [0.20, -0.24, 0.89], down, 1)
        if mode == "open_only":
            add("hold cam released", 3.0)
            return plan
    grasp = PAYLOAD_POSITION+np.array([0, 0, 0.033])
    add("approach camera", 2.0, "left", [grasp[0], grasp[1], 0.89], grip=1)
    add("lower to camera", 2.0, "left", grasp, grip=1)
    add("grasp camera", 1.0, "left", grasp, grip=0)
    if mode == "direct":
        add("gentle direct lift", 3.0, "left", grasp+np.array([0, 0, 0.005]), grip=0)
        add("hold direct lift", 2.0)
        return plan
    # Pivot about the plate's back underside, then clear the retaining edge.
    rear = PAYLOAD_POSITION+np.array([0, 0.019, -0.001])
    tilt = axis_angle_matrix(np.array([1, 0, 0]), -0.35)
    tilted = rear+tilt@(grasp-rear)
    add("tilt plate out of retaining lip", 2.0, "left", tilted, tilt@down, 0)
    pulled = tilted+np.array([0, -0.045, 0.006])
    add("clear rear lip", 2.5, "left", pulled, tilt@down, 0)
    lifted = pulled+np.array([0, 0, 0.065])
    add("lift free camera", 1.5, "left", lifted, tilt@down, 0)
    add("level camera", 1.5, "left", lifted, down, 0)
    hover = np.array([PAD_POSITION[0], PAD_POSITION[1], 0.89])
    add("carry to pad", 2.0, "left", hover, down, 0)
    place = np.array([PAD_POSITION[0], PAD_POSITION[1], PAD_TOP+0.034])
    if mode == "drop":
        place[2] += 0.050
    add("lower onto pad", 2.0, "left", place, down, 0)
    add("release camera", 1.0, "left", place, down, 1)
    add("withdraw from camera", 1.5, "left", hover, down, 1)
    add("hold final placement", 3.0)
    return plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        self.plan = demo_plan(mode)
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(name="camera_release_scripted", action_space=action_box(low, high))
        self.dt = 0.05
        self.phase = "reset"
        self.worst_position_error = 0.0
        self.worst_rotation_error = 0.0

    def bind(self, embodiment_info):
        self.dt = 1.0/float(embodiment_info.control_hz)

    def reset(self, scene):
        self.q = np.r_[READY, 1.0, READY, 1.0].astype(float)
        self.worst_position_error = self.worst_rotation_error = 0.0
        initial = {}
        for arm, offset in (("left", 0), ("right", 7)):
            pos, rot = ik.tcp_pose(self.q[offset:offset+6], station.BASES[arm])
            initial[arm] = (pos, rot, 1.0)
        self.stages = []
        time = 0.0
        for stage in self.plan:
            start = {arm: (p.copy(), r.copy(), g) for arm, (p, r, g) in initial.items()}
            for arm in ("left", "right"):
                if arm in stage:
                    goal = stage[arm]
                    from yam_arena.urdf import quat_xyzw_to_matrix
                    initial[arm] = (np.array(goal["position"]),
                                    quat_xyzw_to_matrix(np.array(goal["rotation_xyzw"])), goal["grip"])
            self.stages.append((time, time+stage["seconds"], stage["phase"], start, initial.copy()))
            time += stage["seconds"]
        self.duration = time

    def targets(self, time):
        for begin, end, phase, starts, goals in self.stages:
            if time <= end:
                f = ik.smoothstep(np.clip((time-begin)/(end-begin), 0, 1))
                result = {}
                for arm in ("left", "right"):
                    p0, r0, g0 = starts[arm]
                    p1, r1, g1 = goals[arm]
                    result[arm] = (p0+f*(p1-p0), ik.slerp_matrix(r0, r1, f), g0+f*(g1-g0))
                return phase, result
        return "finished", self.stages[-1][4]

    def action_at(self, time):
        self.phase, targets = self.targets(time)
        for arm, offset in (("left", 0), ("right", 7)):
            pos, rot, grip = targets[arm]
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q[offset:offset+6],
                                 base_pos=station.BASES[arm], iters=100)
            self.worst_position_error = max(self.worst_position_error, pe)
            self.worst_rotation_error = max(self.worst_rotation_error, re)
            if pe > 0.003 or re > 0.04:
                raise RuntimeError(f"Unreachable {arm} target during {self.phase}: {pe:.5f} m, {re:.5f} rad")
            self.q[offset:offset+6], self.q[offset+6] = q, grip
        return self.q.copy()

    def act(self, observation):
        time = float(observation.state_time)+self.dt
        action = self.action_at(time)
        meta = {"request_stop": True} if time > self.duration+self.dt else {}
        return ActionChunk(actions=[Action(data=action, meta=meta)], control_hz=1/self.dt)


def scripted_policy(**kwargs):
    return ScriptedPolicy(**kwargs)
