"""Scripted robot targets only; apparatus state is neither read nor overwritten."""
from __future__ import annotations
import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
from yam_arena.urdf import axis_angle_matrix
from .config import FRAME_POSITION, OLD_POSITION, NEW_POSITION, TRAY_POSITION, STROKE, ROD_GRIP_OFFSET


def demo_plan(mode="demo"):
    if mode not in {"demo", "hold", "engaged_pull", "release"}:
        raise ValueError("Unknown policy mode")
    down = grip_rotation()
    turn_angle = -np.pi/3
    turn_matrix = axis_angle_matrix(np.array([0, 1, 0]), turn_angle)
    rod_turn = turn_matrix@down
    # Grip the end of the T-handle, keeping the front finger off the shaft.
    rod_grasp = np.array([-.026, 0, .003])
    turned_offset = turn_matrix@rod_grasp
    across = grip_rotation(-np.pi/2)
    plan = []
    def add(phase, seconds, arm=None, position=None, rotation=None, grip=None):
        stage = dict(phase=phase, seconds=seconds)
        if arm:
            stage[arm] = dict(position=np.asarray(position).tolist(),
                rotation_xyzw=ik.matrix_to_quat_xyzw(down if rotation is None else rotation).tolist(), grip=grip)
        plan.append(stage)
    if mode == "hold":
        return [dict(phase="hold reset", seconds=5.)]
    # The passive cradle supports the tool. The left arm stays clear until
    # collecting the old refill; there is no scripted hold on the base.
    grip = FRAME_POSITION+ROD_GRIP_OFFSET+rod_grasp
    add("approach rod handle", 2., "right", grip+[0, 0, .035], down, .5)
    add("lower to rod handle", 2., "right", grip, down, .5)
    add("grasp rod handle", 1., "right", grip, down, .10)
    if mode == "engaged_pull":
        add("bounded pull with ratchet engaged", 2., "right", grip+[0, -.006, 0], down, .10)
        add("hold engaged pull", 2.)
        return plan
    for angle in np.linspace(turn_angle/6, turn_angle, 6):
        turn = axis_angle_matrix(np.array([0, 1, 0]), angle)
        add("rotate rod clear of pawl", .6, "right", FRAME_POSITION+ROD_GRIP_OFFSET+turn@rod_grasp, turn@down, .10)
    pulled = FRAME_POSITION+ROD_GRIP_OFFSET+turned_offset+[0, -STROKE, 0]
    add("retract released rod", 3., "right", pulled, rod_turn, .10)
    add("hold retraction", 1.)
    if mode == "release":
        return plan
    # Right keeps the rod retracted while left exchanges the refills.
    old = OLD_POSITION+[0, 0, .005]
    above = old+[0, 0, .040]
    clear = above+[-.09, 0, 0]
    add("rise outside frame", 2., "left", clear, across, 1)
    add("approach used cartridge", 1.5, "left", above, across, 1)
    add("lower to used cartridge", 1.5, "left", old, across, 1)
    add("grasp used cartridge", .8, "left", old, across, .45)
    add("lift used cartridge", 1.5, "left", above, across, .45)
    add("clear frame sideways", 1.5, "left", clear, across, .45)
    tray = TRAY_POSITION+[0, 0, .029+.010]
    add("carry around front of frame", 2., "left", [clear[0], TRAY_POSITION[1], .90], across, .45)
    add("carry used cartridge to tray", 2., "left", tray+[0, 0, .10], across, .45)
    add("lower used cartridge into tray", 1.5, "left", tray, across, .45)
    add("release used cartridge", .8, "left", tray, across, 1)
    add("withdraw from tray", 1., "left", tray+[0, 0, .10], across, 1)
    new = NEW_POSITION+[0, 0, .005]
    add("approach replacement", 2., "left", new+[0, 0, .10], across, 1)
    add("lower to replacement", 1.5, "left", new, across, 1)
    add("grasp replacement", .8, "left", new, across, .45)
    add("lift replacement", 1.5, "left", new+[0, 0, .14], across, .45)
    # Leave 3 mm loading clearance to the front plate; the scorer still measures
    # the original seating target and tolerances. This is a robot target change.
    replacement_seat = old+[0, -.003, 0]
    replacement_above = above+[0, -.003, 0]
    add("carry replacement clear of frame", 2., "left", clear, across, .45)
    add("align replacement above frame", 1.5, "left", replacement_above, across, .45)
    add("seat replacement", 2., "left", replacement_seat, across, .45)
    add("release replacement", .8, "left", replacement_seat, across, 1)
    add("withdraw above replacement", 1., "left", replacement_above, across, 1)
    add("clear frame after seating", 1.5, "left", clear, across, 1)
    add("park refill hand", 1.5, "left", [.16, .30, .90], across, 1)
    ready = FRAME_POSITION+ROD_GRIP_OFFSET+turned_offset+[0, -.185, 0]
    add("bring piston to replacement", 1.5, "right", ready, rod_turn, .10)
    final_angle = 0.
    for angle in np.linspace(turn_angle*5/6, final_angle, 6):
        turn = axis_angle_matrix(np.array([0, 1, 0]), angle)
        add("re-engage ratchet", .6, "right", FRAME_POSITION+ROD_GRIP_OFFSET+[0, -.185, 0]+turn@rod_grasp, turn@down, .10)
    finish = axis_angle_matrix(np.array([0, 1, 0]), final_angle)
    end = FRAME_POSITION+ROD_GRIP_OFFSET+[0, -.185, 0]+finish@rod_grasp
    add("hold ratchet re-engaged", 1.)
    add("release rod", .8, "right", end, finish@down, .5)
    add("lift clear of rod", 1., "right", end+[0, 0, .03], finish@down, 1)
    add("withdraw rod hand", 1.5, "right", end+[-.10, 0, .05], finish@down, 1)
    add("hold final state", 2.)
    return plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        self.plan = demo_plan(mode)
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(name="sealant_reload_scripted", action_space=action_box(low, high))
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
