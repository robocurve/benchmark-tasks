"""Demonstration using only standard joint-position and jaw commands."""
from dataclasses import replace
import numpy as np
from inspect_robots import Action, ActionChunk
from inspect_robots.policy import PolicyBase
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.station import BASES, TABLE_TOP

from .kinematics import orientation, solve_pose
from .scene import gripper_command, load_fixture


class ReferencePolicy(PolicyBase):
    def __init__(self):
        self.info = replace(YamScriptedPolicy().info, name="tabletop_scripted")

    def bind(self, embodiment_info):
        pass

    def reset(self, scene):
        self.waypoints = load_fixture()["trajectory"]
        self.index = 0
        self.tick = 0
        self.start = self.goal = None

    def act(self, observation):
        from . import adapter
        measured = np.asarray(observation.state["joint_pos"], dtype=float)
        if self.start is None and self.index < len(self.waypoints):
            wp = self.waypoints[self.index]
            previous_command = self.goal
            self.start = measured.copy()
            self.goal = measured.copy()
            adapter.active.demo_command = {"waypoint":self.index+1,"target":{}}
            for arm, index in [("left", 0), ("right", 7)]:
                target = wp.get(arm)
                if target is None:
                    continue
                origin, rotation = adapter.active.frames[target["relative_to"]][:2]
                tip = origin + rotation @ np.asarray(target["tip"], dtype=float)
                if "held_actor" in target:
                    # Translate the actually held object toward its next waypoint.
                    # The reference commands only joints and jaw opening.
                    held_origin = adapter.active.frames[target["held_actor"]][0]
                    wrist, wrist_rot = adapter.active.frames[arm+"_tcp"][:2]
                    pad_tip = wrist + float(PAD_TIP_PAST_TCP) * wrist_rot[:, 2]
                    tip = tip + pad_tip - held_origin
                rot = orientation(target["forward"], target["closing"])
                tcp = tip - float(PAD_TIP_PAST_TCP) * rot[:, 2]
                bases = np.asarray(BASES[arm]) - [0., 0., float(TABLE_TOP)]
                joints, position_error, angle_error = solve_pose(tcp, rot, measured[index:index+6], bases)
                adapter.active.demo_command["target"][arm] = {
                    "tip_m":np.round(tip,4).tolist(),"jaw_m":target["opening_m"],"ik_error_m":round(float(position_error),5),
                    "ik_error_deg":round(float(np.degrees(angle_error)),2)}
                self.goal[index:index+6] = joints
                self.goal[index+6] = gripper_command(target["opening_m"],FINGER_OPEN)
                # A held object keeps measured jaws wider than the commanded gap.
                # Reusing measured width here would release pinch force during each roll segment.
                if previous_command is not None and self.goal[index+6] == previous_command[index+6]:
                    self.start[index+6] = previous_command[index+6]
        if self.index < len(self.waypoints):
            self.tick += 1
            duration = self.waypoints[self.index]["ticks"]
            alpha = min(1., self.tick / duration)
            eased = alpha*alpha*(3.-2.*alpha)
            values = self.start + eased * (self.goal - self.start)
            if self.tick >= duration:
                self.index += 1
                self.tick = 0
                self.start = None
        else:
            values = measured if self.goal is None else self.goal
        return ActionChunk(actions=[Action(data=values, meta={})], control_hz=20)


def make_scripted():
    return ReferencePolicy()
