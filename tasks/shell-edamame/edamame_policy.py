"""Play stock YAM joint commands from waypoints with a side-entry wrist option."""
from dataclasses import dataclass, replace
import json
import os
from pathlib import Path

import numpy as np

from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena import ik
from yam_arena.demo import ArmPlan, FINGER_OPEN, Waypoint, grip_rotation
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.urdf import axis_angle_matrix


@dataclass
class EdamameWaypoint(Waypoint):
    roll: float = 0.
    pitch: float = 0.


class EdamamePlan(ArmPlan):
    def target(self, time):
        pos, rotation, finger = self.start_pos, self.start_rot, self.finger_start
        for item in self.waypoints:
            end = np.asarray(item.pos)
            roll = axis_angle_matrix(np.array([1., 0., 0.]), item.roll)
            pitch = axis_angle_matrix(np.array([0., 1., 0.]), item.pitch)
            end_rotation = pitch @ roll @ grip_rotation(item.yaw)
            if time < item.duration:
                blend = ik.smoothstep(time / item.duration)
                return (pos + blend * (end - pos), ik.slerp_matrix(rotation, end_rotation, blend),
                        finger + blend * (item.finger - finger), item.label)
            time -= item.duration
            pos, rotation, finger = end, end_rotation, item.finger
        return pos, rotation, finger, "done"


class EdamameScripted(YamScriptedPolicy):
    def reset(self, scene):
        super().reset(scene)
        self._plans = {arm: EdamamePlan(plan.waypoints, self._bases[arm])
                       for arm, plan in self._plans.items()}


class EdamameOracle(EdamameScripted):
    """Use measured shell pose and contact to verify opening and bean transfer."""

    def __init__(self, late_pour=False, **kwargs):
        super().__init__(**kwargs)
        self.late_pour = late_pour

    def reset(self, scene):
        super().reset(scene)
        self.phase = "carry to bowl"
        self.ticks = 0
        self.retry = 0
        self.lost = 0
        self.left_roll = -.65
        self.right_roll = 0.

    def _enter(self, phase):
        self.phase = phase
        self.ticks = 0
        print(f"[edamame-oracle] {phase}", flush=True)

    def _drive(self, side, goal, roll, pitch, finger, step_limit=.0017):
        np.clip(goal, [.16, -.30, .755], [.58, .30, .92], out=goal)
        delta = goal - self.targets[side]
        self.targets[side] += delta * min(1., step_limit / max(np.linalg.norm(delta), 1e-9))
        rotation = axis_angle_matrix(np.array([0., 1., 0.]), pitch) @ axis_angle_matrix(
            np.array([1., 0., 0.]), roll) @ grip_rotation()
        plan = self._plans[side]
        plan.q, _, _ = ik.solve(self.targets[side], ik.matrix_to_quat_xyzw(rotation), plan.q,
                                base_pos=plan.base_pos, iters=42)
        return np.r_[plan.q, np.clip(finger / FINGER_OPEN, 0., 1.)]

    def act(self, observation):
        import task
        from edamame_geometry import BIN_POS, BOWL_POS, TABLE
        from runtime import scalar_array
        from yam_arena.urdf import quat_xyzw_to_matrix

        t = float(observation.state_time)
        monitor = task._monitor
        if t < (28.3 if self.late_pour else 13.75) or monitor is None:
            return super().act(observation)
        if self.phase == "carry to bowl":
            self.targets = {side: ik.tcp_pose(plan.q, plan.base_pos)[0]
                            for side, plan in self._plans.items()}
            self.right_hold = self.targets["right"].copy()
            self._enter("pour" if self.late_pour else "approach")

        pose = scalar_array(monitor.pod.data.body_link_pose_w)[0, monitor.pod.body_names.index("pod_upper_left")]
        angle = float(scalar_array(monitor.pod.data.joint_pos)[0, monitor.hinge_id])
        beans = np.asarray([task._world_body(bean) for bean in monitor.beans])
        self.ticks += 1
        if not np.isfinite(pose).all() or not np.isfinite(beans).all() or task._trial.invalid:
            if self.phase != "stop":
                self._enter("stop")

        target = pose[:3].copy()
        if self.phase != "stop":
            # Pose of the upper wall near the free seam, in its own frame.
            target += quat_xyzw_to_matrix(pose[3:]) @ np.array([-.020 + .006*self.retry, .004, .005])
        pads = {path.rsplit("/", 1)[-1] for name, paths in monitor.contacts.items()
                if name.startswith("pod/pod_upper") for path in paths if "/LeftArm/" in path}
        held = {"tip_left", "tip_right"} <= pads
        left_goal = self.targets["left"].copy()
        right_goal = self.right_hold.copy()
        l_grip, r_grip = FINGER_OPEN, 0.
        left_park = np.array([.25, .20, TABLE + .14])
        if self.phase == "approach":
            left_goal = target + [0., .038, .036]
            if np.linalg.norm(left_goal - self.targets["left"]) < .003:
                self._enter("align")
        elif self.phase == "align":
            left_goal = target
            l_grip = -.011
            if np.linalg.norm(left_goal - self.targets["left"]) < .003 or self.ticks > 64:
                self._enter("pinch")
        elif self.phase == "pinch":
            left_goal, l_grip = target, 0.
            if held:
                self.lost = 0
                self._enter("open")
            elif self.ticks >= 24:
                self.retry += 1
                self._enter("approach" if self.retry < 3 else "stop")
        elif self.phase == "open":
            l_grip = 0.
            if angle > .92 and not scalar_array(monitor.seal).any():
                self._enter("pour")
            elif held:
                self.lost = 0
                left_goal += [-.00015, .00065, .00080]
            else:
                self.lost += 1
            if self.lost >= 5 or self.ticks >= 66:
                self.retry += 1
                self._enter("approach" if self.retry < 3 else "stop")
        elif self.phase == "pour":
            left_goal = left_park
            self.right_roll = min(1.30, self.right_roll + .012)
            correction = np.asarray(BOWL_POS[:2]) - task._world_body(monitor.pod)[:2]
            right_goal[:2] = self.targets["right"][:2] + np.clip(correction, -.001, .001)
            right_goal[2] = TABLE + .102
            if all(task._inside_bowl(bean) for bean in beans) and task._trial.contained_count == 0:
                self._enter("carry shell")
            elif self.ticks > 166:
                self._enter("stop")
        elif self.phase == "carry shell":
            left_goal = left_park
            right_goal = np.array([BIN_POS[0], BIN_POS[1], TABLE + .154])
            if np.linalg.norm(right_goal - self.targets["right"]) < .004:
                self._enter("release shell")
        elif self.phase == "release shell":
            left_goal = left_park
            right_goal = np.array([BIN_POS[0], BIN_POS[1], TABLE + .107])
            if np.linalg.norm(right_goal - self.targets["right"]) < .004:
                r_grip = FINGER_OPEN
                if self.ticks > 55:
                    self._enter("retreat")
        elif self.phase in {"retreat", "stop"}:
            left_goal = left_park
            right_goal = np.array([.25, -.23, TABLE + .14])
            r_grip = FINGER_OPEN

        left = self._drive("left", left_goal, self.left_roll, -.64, l_grip)
        right = self._drive("right", right_goal, self.right_roll, -.55, r_grip)
        stop = self.phase in {"retreat", "stop"} and self.ticks >= 45
        return ActionChunk(actions=[Action(data=pack(left, right), meta={"request_stop": True} if stop else {})],
                           control_hz=1. / self._dt)


class PlateTransferOracle(EdamameOracle):
    """Open over the plate, pour retained beans, then recover the loose bean."""

    def act(self, observation):
        import task
        from edamame_geometry import BIN_POS, BOWL_POS, TABLE

        t = float(observation.state_time)
        monitor = task._monitor
        if t < 19.75 or monitor is None:
            return EdamameScripted.act(self, observation)
        beans = np.asarray([task._world_body(bean) for bean in monitor.beans])
        shell = task._world_body(monitor.pod)
        if self.phase == "carry to bowl":
            self.targets = {side: ik.tcp_pose(plan.q, plan.base_pos)[0] for side, plan in self._plans.items()}
            self.right_hold = self.targets["right"].copy()
            self.loose = int(np.argmin(beans[:, 2]))
            self.pitch = .25
            self.pick_z = None
            self._enter("clear left" if task._trial.opened else "stop")

        self.ticks += 1
        if task._trial.invalid or not np.isfinite(beans).all() or not np.isfinite(shell).all():
            if self.phase != "stop":
                self._enter("stop")

        left_goal = np.array([.25, .20, TABLE + .14])
        right_goal = self.targets["right"].copy()
        grip = 0.
        loose = beans[self.loose]
        if self.phase == "clear left":
            right_goal = self.right_hold.copy()
            if np.linalg.norm(left_goal - self.targets["left"]) < .006:
                self._enter("above serving bowl")
        elif self.phase == "above serving bowl":
            delta = np.asarray(BOWL_POS[:2]) - shell[:2]
            right_goal[:2] += np.clip(delta, -.0011, .0011)
            right_goal[2] = TABLE + .094
            self.pitch = max(-.55, self.pitch - .009)
            if np.linalg.norm(delta) < .024:
                self._enter("empty shell")
            elif self.ticks > 160:
                self._enter("stop")
        elif self.phase == "empty shell":
            right_goal[:2] += np.clip(np.asarray(BOWL_POS[:2]) - shell[:2], -.001, .001)
            right_goal[2] = TABLE + .101
            self.right_roll = min(1.2, self.right_roll + .009)
            if task._trial.beans_in_bowl >= 2 and task._trial.contained_count == 0:
                self._enter("above scraps bin")
            elif self.ticks > 160:
                self._enter("stop")
        elif self.phase == "above scraps bin":
            right_goal = np.array([BIN_POS[0], BIN_POS[1], TABLE + .154])
            if np.linalg.norm(right_goal - self.targets["right"]) < .005:
                self._enter("release shell")
        elif self.phase == "release shell":
            right_goal = np.array([BIN_POS[0], BIN_POS[1], TABLE + .106])
            if np.linalg.norm(right_goal - self.targets["right"]) < .004:
                grip = FINGER_OPEN
                if self.ticks > 54:
                    self._enter("above loose bean" if task._trial.shell_in_bin else "stop")
        elif self.phase in {"above loose bean", "lower over bean", "pinch bean", "lift bean",
                            "bring bean to bowl", "drop bean"}:
            self.pitch, self.right_roll = 0., 0.
            pads = {path.rsplit("/", 1)[-1] for path in monitor.contacts.get(f"bean_{self.loose}", [])
                    if "/RightArm/" in path}
            if self.phase == "above loose bean":
                grip = FINGER_OPEN
                right_goal = loose + [0., 0., .064]
                if np.linalg.norm(right_goal - self.targets["right"]) < .004:
                    self._enter("lower over bean")
            elif self.phase == "lower over bean":
                grip = -.009
                right_goal = loose + [0., 0., .006]
                if np.linalg.norm(right_goal - self.targets["right"]) < .003:
                    self._enter("pinch bean")
            elif self.phase == "pinch bean":
                if {"tip_left", "tip_right"} <= pads:
                    self.pick_z = float(loose[2])
                    self._enter("lift bean")
                elif self.ticks >= 24:
                    self._enter("stop")
            elif self.phase == "lift bean":
                right_goal[2] = self.pick_z + .063
                if loose[2] > self.pick_z + .028:
                    self._enter("bring bean to bowl")
                elif self.ticks >= 50:
                    self._enter("stop")
            else:
                right_goal = np.array([BOWL_POS[0] - .022, BOWL_POS[1], TABLE + .076])
                if np.linalg.norm(right_goal - self.targets["right"]) < .004:
                    if self.phase == "bring bean to bowl":
                        self._enter("drop bean")
                    else:
                        grip = FINGER_OPEN
                        if self.ticks > 22:
                            self._enter("stop")
        elif self.phase == "stop":
            right_goal = np.array([.25, -.23, TABLE + .14])
            grip = FINGER_OPEN

        left = self._drive("left", left_goal, -.25, -.36, FINGER_OPEN)
        right = self._drive("right", right_goal, self.right_roll, self.pitch, grip)
        stop = self.phase == "stop" and self.ticks > 55
        return ActionChunk(actions=[Action(data=pack(left, right), meta={"request_stop": True} if stop else {})],
                           control_hz=1. / self._dt)


class CompletionOracle(EdamameOracle):
    """Keep the verified opening and pour, then verify shell release and recover rim beans."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.release_above = os.getenv("EDAMAME_RELEASE_ABOVE") == "1"
        self.grip_force_limit = float(os.getenv("EDAMAME_GRIP_FORCE_LIMIT_N", "0"))
        if not np.isfinite(self.grip_force_limit) or self.grip_force_limit < 0:
            raise ValueError("EDAMAME_GRIP_FORCE_LIMIT_N must be finite and nonnegative.")

    def reset(self, scene):
        super().reset(scene)
        self.phase = "opening and pour"
        self.retries = 0
        self.target_bean = None

    @staticmethod
    def _shell_pads(monitor, side="right"):
        loads = {"tip_left": 0., "tip_right": 0.}
        for body, contacts in monitor.contact_forces.items():
            if body.startswith("pod/"):
                for path, values in contacts.items():
                    pad = path.rsplit("/", 1)[-1]
                    if f"/{side.title()}Arm/" in path and pad in loads:
                        loads[pad] += values[0]
        return loads

    def _shell_grip(self, monitor, side, finger):
        from runtime import scalar_array
        from yam_arena.constants import FINGER_JOINTS

        if not self.grip_force_limit or finger < -.0145:
            return finger
        robot = monitor.scene[f"{side}_robot"]
        ids = robot.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        measured = float(scalar_array(robot.data.joint_pos)[0, ids].mean())
        load = max(self._shell_pads(monitor, side).values())
        if finger < measured - .001:
            return finger
        if load <= .05:
            return min(finger, measured + .0008)
        correction = np.clip((self.grip_force_limit - load) / 9000., -.0010, .00032)
        return float(np.clip(min(finger, measured + correction), FINGER_OPEN, 0.))

    def _opening_action(self, observation, monitor):
        chunk = EdamameScripted.act(self, observation)
        if self.grip_force_limit and float(observation.state_time) > 5.8:
            command = np.asarray(chunk.actions[0].data).copy()
            for side, index in (("left", 6), ("right", 13)):
                finger = self._shell_grip(monitor, side, command[index] * FINGER_OPEN)
                command[index] = finger / FINGER_OPEN
            return replace(chunk, actions=[replace(chunk.actions[0], data=command)])
        return chunk

    def _trace(self, monitor, trial, time):
        from runtime import scalar_array

        shell = monitor.pod.data
        beans = [bean.data for bean in monitor.beans]
        record = {
            "t": round(time, 3),
            "phase": self.phase,
            "shell_link_pose": scalar_array(shell.body_link_pose_w)[0].round(6).tolist(),
            "shell_link_velocity": scalar_array(shell.body_link_vel_w)[0].round(6).tolist(),
            "shell_joint_position": scalar_array(shell.joint_pos)[0].round(6).tolist(),
            "seam_intact": scalar_array(monitor.seal).tolist(),
            "bean_pose": [scalar_array(bean.root_link_pose_w)[0].round(6).tolist() for bean in beans],
            "bean_velocity": [scalar_array(bean.root_link_vel_w)[0].round(6).tolist() for bean in beans],
            "arm_joints": {side: scalar_array(monitor.scene[f"{side}_robot"].data.joint_pos)[0].round(6).tolist()
                           for side in ("left", "right")},
            "contacts": monitor.contact_forces,
            "shell_in_bin": trial.shell_in_bin,
            "beans_in_bowl": trial.beans_in_bowl,
            "score": trial.score_value,
        }
        print("[edamame-expert] " + json.dumps(record, separators=(",", ":")), flush=True)

    def _command(self, goal, rotation, finger):
        plan = self._plans["right"]
        delta = np.asarray(goal) - self.right_target
        self.right_target += delta * min(1., .0025 / max(np.linalg.norm(delta), 1e-9))
        plan.q, _, _ = ik.solve(self.right_target, ik.matrix_to_quat_xyzw(rotation), plan.q,
                                base_pos=plan.base_pos, iters=25)
        right = np.r_[plan.q, np.clip(finger / FINGER_OPEN, 0., 1.)]
        stop = self.phase in {"done", "failed", "end opening probe"} and self.ticks > 30
        return ActionChunk(actions=[Action(data=pack(self.left_command, right),
                                          meta={"request_stop": True} if stop else {})],
                           control_hz=1. / self._dt)

    def act(self, observation):
        import task
        from edamame_geometry import BIN_POS, BOWL_POS, TABLE

        time = float(observation.state_time)
        monitor, trial = task._monitor, task._trial
        if monitor is None:
            return EdamameScripted.act(self, observation)
        self._trace(monitor, trial, time)
        if self.phase == "opening and pour":
            if time < 31.0:
                return self._opening_action(observation, monitor)
            plan = self._plans["right"]
            self.right_target, self.carry_rotation = ik.tcp_pose(plan.q, plan.base_pos)
            self.left_command = self._plans["left"].action7(time + self._dt)[0]
            self.transport_z = float(self.right_target[2])
            self.release_target = self.right_target.copy()
            self._enter("lift shell past caddy rim" if trial.opened and trial.contained_count == 0 else "failed")

        self.ticks += 1
        if trial.invalid and self.phase != "failed":
            self._enter("failed")
        actual = np.asarray(trial.tools.get("right", self.right_target))
        shell = np.asarray([task._world_body(monitor.pod, name) for name in
                           ("pod_lower", "pod_upper", "pod_upper_left", "pod_upper_right")])
        tcp_up = np.array([.43, -.17, TABLE + .157])
        target = self.right_target.copy()
        rotation = self.carry_rotation
        finger = FINGER_OPEN

        if self.phase == "turn held shell for caddy":
            target = self.release_target.copy()
            travel = axis_angle_matrix(np.array([0., 1., 0.]), -.52) @ grip_rotation()
            self.carry_rotation = ik.slerp_matrix(self.carry_rotation, travel, .10)
            rotation = self.carry_rotation
            finger = 0.
            if self.ticks >= 42:
                if max(self._shell_pads(monitor).values()) > .04:
                    self._enter("lift shell past caddy rim")
                else:
                    self._enter("failed")
        elif self.phase == "lift shell past caddy rim":
            target = self.release_target + [0., 0., .033]
            finger = 0.
            if self.grip_force_limit:
                # A shell regrasp starts lower than the opening grip. Raise it
                # clear of the caddy before starting any horizontal transport.
                target[2] = max(target[2], BIN_POS[2] + .182)
            above_rim = shell[:, 2].min() > BIN_POS[2] + .151
            if above_rim or (not self.grip_force_limit and self.ticks > 35):
                self.transport_z = float(self.right_target[2])
                self._enter("center shell over bin")
            elif self.ticks > 105:
                self._enter("failed")
        elif self.phase == "center shell over bin":
            correction = np.asarray(BIN_POS[:2]) - shell[:, :2].mean(axis=0)
            target[:2] = actual[:2] + correction
            target[2] = max(self.transport_z, BIN_POS[2] + .167)
            finger = 0.
            if np.linalg.norm(correction) < .014 and np.linalg.norm(actual - self.right_target) < .012:
                self.release_target = self.right_target.copy()
                self._enter("open both jaws above bin" if self.release_above else
                            "lower shell through the center of the bin")
            elif self.ticks >= 110:
                self._enter("failed")
        elif self.phase == "open both jaws above bin":
            target = self.release_target.copy()
            # Open clear of the rim so both pads can travel through the full
            # stock aperture without pulling the shell sideways against a wall.
            finger = max(FINGER_OPEN, -.0015 * self.ticks)
            if self.ticks >= 15 and trial.shell_in_bin and max(self._shell_pads(monitor).values()) < .04:
                self._enter("lift clear of bin")
            elif self.ticks > 45:
                self._enter("failed")
        elif self.phase == "lower shell through the center of the bin":
            target = self.release_target.copy()
            target[2] = BIN_POS[2] + .090
            finger = 0.
            if trial.shell_in_bin and actual[2] < BIN_POS[2] + .107:
                self.release_target = self.right_target.copy()
                self.release_shift = np.zeros(3)
                self._enter("release shell while unloading the loaded pad")
            elif self.ticks > 75:
                self._enter("failed")
        elif self.phase == "release shell while unloading the loaded pad":
            loads = self._shell_pads(monitor)
            # A 48 mm opening clears the pod and keeps both pads inside the
            # stock 165 mm bin during the axial retreat.
            finger = max(-.024, -.001 - .00065 * self.ticks)
            if max(loads.values()) > .06 and finger > -.024:
                # Opening the leading pad can hook a folded shell. Keep that
                # pad approximately stationary while the opposite pad clears.
                side = (loads["tip_left"] - loads["tip_right"]) / sum(loads.values())
                self.release_shift += .00065 * side * self.carry_rotation[:, 0]
                self.release_shift = np.clip(self.release_shift, -.014, .014)
            target = self.release_target + self.release_shift
            if self.ticks >= 9 and trial.shell_in_bin and max(loads.values()) < .04:
                self.release_target = target.copy()
                self._enter("lift clear of bin")
            elif self.ticks >= 52:
                self.release_target = target.copy()
                self._enter("withdraw wrist along finger length")
        elif self.phase == "withdraw wrist along finger length":
            target = self.release_target - self.carry_rotation[:, 2] * min(.038, .00070 * self.ticks)
            finger = -.024
            if trial.shell_in_bin and max(self._shell_pads(monitor).values()) < .04:
                self.release_target = target.copy()
                self._enter("lift clear of bin")
            elif self.ticks > 64:
                center = np.asarray(BIN_POS[:2]) - shell[:, :2].mean(axis=0)
                self.brace_goal = np.r_[actual[:2] + center, BIN_POS[2] + .041]
                self._enter("support shell on bin floor")
        elif self.phase == "support shell on bin floor":
            target, finger = self.brace_goal.copy(), -.024
            if trial.shell_in_bin and max(self._shell_pads(monitor).values()) < .04:
                self.release_target = target.copy()
                self._enter("lift clear of bin")
            elif self.ticks > 35 and np.linalg.norm(actual - target) < .013:
                self.brace_goal = actual.copy()
                self._enter("turn pad away from grounded shell")
            elif self.ticks > 90:
                self._enter("failed")
        elif self.phase == "turn pad away from grounded shell":
            turn = min(.62, self.ticks * .018)
            rotation = axis_angle_matrix(np.array([0., 0., 1.]), turn) @ self.carry_rotation
            target = self.brace_goal + [-min(.019, self.ticks * .00045), 0., 0.]
            finger = -.024
            if trial.shell_in_bin and max(self._shell_pads(monitor).values()) < .04:
                self.release_target = target.copy()
                self.carry_rotation = rotation
                self._enter("lift clear of bin")
            elif self.ticks > 68:
                self._enter("failed")
        elif self.phase == "lift clear of bin":
            target = self.release_target + [0., 0., .043]
            target[2] = max(target[2], BIN_POS[2] + .160)
            finger = -.024 if not self.release_above and actual[2] < BIN_POS[2] + .149 else FINGER_OPEN
            if actual[2] >= BIN_POS[2] + .149:
                self._enter("find displaced bean")
            elif self.ticks > 55:
                self._enter("failed")
        elif self.phase == "find displaced bean":
            target = tcp_up
            outside = [i for i, bean in enumerate(monitor.beans) if not task._inside_bowl(task._world_body(bean))]
            if not outside:
                self._enter("verify released result")
            elif self.ticks > 14:
                self.target_bean = outside[0]
                bean = task._world_body(monitor.beans[self.target_bean])
                self.push_center = bean.copy()
                outward = bean[:2] - np.asarray(BOWL_POS[:2])
                self.radial = outward / max(np.linalg.norm(outward), 1e-9)
                yaw = (np.arctan2(*self.radial[::-1]) + np.pi / 2) % np.pi - np.pi / 2
                self.bean_rotation = grip_rotation(yaw)
                self.push_start = bean + np.r_[self.radial * .010, .012]
                rim_height = BOWL_POS[2] + .031
                self.push_rim = abs(bean[2] - rim_height) < .013 and abs(np.linalg.norm(outward) - .078) < .016
                self._enter("approach rim bean" if self.push_rim else "approach loose bean")
        elif self.phase in {"verify released result", "done", "failed"}:
            target = np.array([.25, -.23, TABLE + .17])
            if self.phase == "verify released result" and trial.score_value >= 1.:
                self._enter("done")
            elif self.phase == "verify released result" and self.ticks > 130:
                self._enter("failed")
        else:
            bean = task._world_body(monitor.beans[self.target_bean])
            safe_z = BOWL_POS[2] + .119
            rotation = self.bean_rotation
            contact = {p.rsplit("/", 1)[-1]: data[0]
                       for p, data in monitor.contact_forces[f"bean_{self.target_bean}"].items()
                       if "/RightArm/" in p}
            if self.phase == "approach rim bean":
                target = self.push_start.copy()
                target[2] = safe_z
                finger = 0.
                if np.linalg.norm(actual - target) < .014:
                    self._enter("lower pusher to rim")
            elif self.phase == "lower pusher to rim":
                target = self.push_start
                finger = 0.
                if np.linalg.norm(actual - target) < .007 or self.ticks >= 56:
                    self._enter("push rim bean")
            elif self.phase == "push rim bean":
                travel = min(.039, .00085 * self.ticks)
                target = self.push_start - np.r_[travel * self.radial, 0.]
                finger = 0.
                if task._inside_bowl(bean):
                    self._enter("find displaced bean")
                elif self.ticks > 65:
                    self.retries += 1
                    self._enter("find displaced bean" if self.retries < 3 else "failed")
            elif self.phase == "approach loose bean":
                target = np.r_[bean[:2], safe_z]
                if np.linalg.norm(actual - target) < .013:
                    self._enter("lower jaws around bean")
            elif self.phase == "lower jaws around bean":
                target = bean + [0., 0., .012]
                finger = -.009
                if np.linalg.norm(actual - target) < .006 or self.ticks >= 52:
                    self._enter("pinch loose bean")
            elif self.phase == "pinch loose bean":
                finger = -.0025
                if min(contact.get("tip_left", 0.), contact.get("tip_right", 0.)) > .008:
                    self._enter("lift loose bean")
                elif self.ticks >= 20:
                    self.retries += 1
                    self._enter("find displaced bean" if self.retries < 3 else "failed")
            elif self.phase == "lift loose bean":
                target[2] = safe_z
                finger = -.0025
                if bean[2] > BOWL_POS[2] + .063:
                    self._enter("carry bean over bowl")
                elif self.ticks >= 50:
                    self._enter("failed")
            elif self.phase == "carry bean over bowl":
                target = np.r_[np.asarray(BOWL_POS[:2]) + (actual[:2] - bean[:2]), safe_z]
                finger = -.0025
                if np.linalg.norm(bean[:2] - np.asarray(BOWL_POS[:2])) < .015:
                    self._enter("release recovered bean")
                elif self.ticks > 100:
                    self._enter("failed")
            elif self.phase == "release recovered bean" and self.ticks >= 24:
                self._enter("find displaced bean")
            if self.ticks > 145:
                self._enter("failed")
        if self.phase in {"turn held shell for caddy", "lift shell past caddy rim", "center shell over bin",
                          "lower shell through the center of the bin"}:
            finger = self._shell_grip(monitor, "right", finger)
        return self._command(target, rotation, finger)


class EdamameReplay(YamScriptedPolicy):
    """Replay a validated Inspect Robots action log through the normal joint interface."""

    def __init__(self, action_log, **kwargs):
        super().__init__(**kwargs)
        with Path(action_log).open() as source:
            header = json.loads(source.readline())
            if header.get("kind") != "header" or header.get("action_dim") != 14:
                raise ValueError("Expected a 14-dimensional YAM action log.")
            self.actions = []
            for index, line in enumerate(source):
                frame = json.loads(line)
                action = np.asarray(frame["action"], dtype=float)
                if frame["t"] != index or action.shape != (14,) or not np.isfinite(action).all():
                    raise ValueError(f"Invalid YAM action at step {index}.")
                self.actions.append(action)
            if not self.actions:
                raise ValueError("The action log contains no steps.")

    def reset(self, scene):
        self.next_action = 0

    def act(self, observation):
        action = self.actions[min(self.next_action, len(self.actions) - 1)]
        self.next_action += 1
        meta = {"request_stop": True} if self.next_action >= len(self.actions) else {}
        return ActionChunk(actions=[Action(data=action, meta=meta)], control_hz=1. / self._dt)


class AdaptiveSeamOracle(CompletionOracle):
    """Approach the measured upper lip, then pull tangentially about the physical hinge."""

    def reset(self, scene):
        super().reset(scene)
        self.phase = "carry closed pod"
        self.seam_attempt = 0

    def _begin_seam_control(self):
        self.right_target, self.carry_rotation = ik.tcp_pose(
            self._plans["right"].q, self._plans["right"].base_pos,
        )
        self.right_hold = self.right_target.copy()
        self.left_target, _ = ik.tcp_pose(self._plans["left"].q, self._plans["left"].base_pos)
        self.left_command = np.r_[self._plans["left"].q, 1.]
        self.left_rotation = (
            axis_angle_matrix(np.array([0., 1., 0.]), -.68)
            @ axis_angle_matrix(np.array([1., 0., 0.]), -.60)
            @ grip_rotation()
        )
        self._enter("above live upper lip")

    def _left_action(self, target, finger):
        delta = np.asarray(target) - self.left_target
        self.left_target += delta * min(1., .0023 / max(np.linalg.norm(delta), 1e-9))
        plan = self._plans["left"]
        plan.q, _, _ = ik.solve(self.left_target, ik.matrix_to_quat_xyzw(self.left_rotation),
                                plan.q, base_pos=plan.base_pos, iters=29)
        return np.r_[plan.q, np.clip(finger / FINGER_OPEN, 0., 1.)]

    def act(self, observation):
        import task
        from edamame_geometry import BOWL_POS, TABLE, Y_SCALE, Z_SCALE, bow, radius
        from runtime import scalar_array
        from yam_arena.urdf import quat_xyzw_to_matrix

        time = float(observation.state_time)
        monitor, trial = task._monitor, task._trial
        if monitor is None:
            return EdamameScripted.act(self, observation)
        if self.phase == "carry closed pod":
            if time < 13.45:
                self._trace(monitor, trial, time)
                return self._opening_action(observation, monitor)
            self._begin_seam_control()
        active = {"above live upper lip", "align jaws at upper lip", "pinch upper lip",
                  "pull along hinge tangent", "clear left after opening", "pour from open shell", "end opening probe"}
        if self.phase not in active:
            return super().act(observation)
        self._trace(monitor, trial, time)
        self.ticks += 1
        if trial.invalid:
            self._enter("failed")
            return super().act(observation)

        pod = monitor.pod
        poses = scalar_array(pod.data.body_link_pose_w)[0]
        lower = poses[pod.body_names.index("pod_lower")]
        upper = poses[pod.body_names.index("pod_upper_left")]
        frame = quat_xyzw_to_matrix(upper[3:])
        lower_frame = quat_xyzw_to_matrix(lower[3:])
        x = (-.021, -.017, -.025)[self.seam_attempt]
        angle = .34
        rind = radius(x) + .00055
        local_edge = np.array([x, Y_SCALE * (bow(x) + rind * np.cos(angle)), Z_SCALE * rind * np.sin(angle)])
        lip = upper[:3] + frame @ local_edge
        approach = self.left_rotation[:, 2]
        normal = frame @ np.array([0., np.cos(angle) / Y_SCALE, np.sin(angle) / Z_SCALE])
        jaw_axis = normal - approach * np.dot(normal, approach)
        jaw_axis /= max(np.linalg.norm(jaw_axis), 1e-9)
        if np.dot(jaw_axis, self.left_rotation[:, 0]) < 0.:
            jaw_axis = -jaw_axis
        if self.phase in {"above live upper lip", "align jaws at upper lip", "pinch upper lip",
                          "pull along hinge tangent"}:
            aligned = np.column_stack([jaw_axis, np.cross(approach, jaw_axis), approach])
            self.left_rotation = ik.slerp_matrix(self.left_rotation, aligned, .18)
        pinch = lip - approach * .0117
        hinge = lower[:3] + lower_frame @ np.array([x, -.0091 * Y_SCALE, 0.])
        opening_direction = np.cross(lower_frame[:, 0], lip - hinge)
        opening_direction /= max(np.linalg.norm(opening_direction), 1e-9)
        left_actual = np.asarray(trial.tools.get("left", self.left_target))
        pads = {path.rsplit("/", 1)[-1] for body, contacts in monitor.contact_forces.items()
                if body.startswith("pod/pod_upper") for path, values in contacts.items()
                if "/LeftArm/" in path and values[0] >= .025}
        holding_lip = {"tip_left", "tip_right"} <= pads
        park = np.array([.35, .133, TABLE + .138])
        left_goal, lf = park, FINGER_OPEN
        right_goal, rf = self.right_hold.copy(), 0.

        if self.phase == "above live upper lip":
            left_goal = pinch + [-.009, .048, .040]
            lf = -.011
            if np.linalg.norm(left_actual - left_goal) < .010:
                self._enter("align jaws at upper lip")
            elif self.ticks > 180:
                self._enter("end opening probe")
        elif self.phase == "align jaws at upper lip":
            left_goal, lf = pinch, -.008
            if np.linalg.norm(left_actual - pinch) < .004:
                self._enter("pinch upper lip")
            elif self.ticks > 65:
                self._enter("end opening probe")
        elif self.phase == "pinch upper lip":
            left_goal, lf = pinch, 0.
            if holding_lip:
                self.no_contact = 0
                self._enter("pull along hinge tangent")
            elif self.ticks > 26:
                self._retry_seam()
        elif self.phase == "pull along hinge tangent":
            left_goal = pinch + .0035 * opening_direction
            lf = 0.
            self.no_contact = 0 if holding_lip else self.no_contact + 1
            if trial.opened and trial.hinge_deg >= 55.:
                self._enter("clear left after opening")
            elif self.no_contact > 8 or self.ticks > 115:
                self._retry_seam()
        elif self.phase == "clear left after opening":
            if np.linalg.norm(left_actual - park) < .017:
                self._enter("pour from open shell")
            elif self.ticks > 100:
                self._enter("end opening probe")
        elif self.phase == "pour from open shell":
            beans = np.asarray([task._world_body(bean) for bean in monitor.beans])
            right_goal[:2] = self.right_target[:2] + np.clip(
                np.asarray(BOWL_POS[:2]) - beans.mean(axis=0)[:2], -.0014, .0014,
            )
            right_goal[2] = TABLE + .110
            axis = lower_frame[:, 0]
            positive = axis_angle_matrix(axis, .010)
            negative = axis_angle_matrix(axis, -.010)
            normal = lower_frame[:, 2]
            if normal[2] > -.75:
                change = positive if (positive @ normal)[2] < (negative @ normal)[2] else negative
                self.carry_rotation = change @ self.carry_rotation
            if trial.contained_count == 0:
                self.transport_z = float(self.right_target[2])
                self.release_target = self.right_target.copy()
                self._enter("lift shell past caddy rim")
            elif self.ticks > 140:
                self._enter("end opening probe")
        self.left_command = self._left_action(left_goal, self._shell_grip(monitor, "left", lf))
        return self._command(right_goal, self.carry_rotation, self._shell_grip(monitor, "right", rf))

    def _retry_seam(self):
        if self.seam_attempt + 1 < 3:
            self.seam_attempt += 1
            self._enter("above live upper lip")
        else:
            self._enter("end opening probe")


class PlateBeanRecovery(CompletionOracle):
    """Recover individual beans without bringing both wrists to the pod again."""

    def reset(self, scene):
        super().reset(scene)
        self.phase = "plate opening"
        self.left_roll = 0.
        self.left_pitch = -.30
        self.parked_shell = None
        self.disposal = False
        self.bean_index = None
        self.plate_pitch = .25
        self.shell_pick_attempt = 0

    def _retry_shell_pick(self):
        self.shell_pick_attempt += 1
        self._enter("hover over empty shell" if self.shell_pick_attempt < 3 else "stop")

    def _begin_disposal(self, actual):
        plan = self._plans["right"]
        _, self.carry_rotation = ik.tcp_pose(plan.q, plan.base_pos)
        self.right_target = actual.copy()
        self.release_target = self.right_target.copy()
        self.transport_z = float(self.right_target[2])
        self._enter("turn held shell for caddy")
        self.disposal = True

    def act(self, observation):
        import task
        from edamame_geometry import BOWL_POS, PLATE_POS, TABLE
        from runtime import scalar_array
        from yam_arena.urdf import quat_xyzw_to_matrix

        time = float(observation.state_time)
        monitor, trial = task._monitor, task._trial
        if self.disposal:
            return super().act(observation)
        if monitor is None:
            return EdamameScripted.act(self, observation)
        if time < 19.75:
            return self._opening_action(observation, monitor)
        self._trace(monitor, trial, time)
        if self.phase == "plate opening":
            self.targets = {side: ik.tcp_pose(plan.q, plan.base_pos)[0]
                            for side, plan in self._plans.items()}
            if trial.contained_count < len(monitor.beans):
                self.bean_index = int(np.argmin([task._world_body(bean)[2] for bean in monitor.beans]))
            self._enter("move held shell clear of plate" if trial.opened else "stop")
        self.ticks += 1
        if trial.invalid:
            self.phase = "stop"
        bean = task._world_body(monitor.beans[self.bean_index]) if self.bean_index is not None else None
        left_actual = np.asarray(trial.tools.get("left", self.targets["left"]))
        right_actual = np.asarray(trial.tools.get("right", self.targets["right"]))
        right_goal = (np.array([.345, -.240, TABLE + .117]) if self.parked_shell is None
                      else self.parked_shell)
        left_goal = np.array([.25, .19, TABLE + .132])
        grip = FINGER_OPEN
        right_roll = 0.
        right_pitch = .25
        right_finger = 0.
        right_step = .0017
        left_step = .0017
        if self.phase == "move held shell clear of plate":
            positions = np.asarray([task._world_body(item) for item in monitor.beans])
            clearance = float(np.linalg.norm(positions[:, :2] - right_actual[:2], axis=1).min())
            if task.JOINED_SHELL and trial.contained_count == 0 and right_actual[1] < -.225:
                # The pour can leave beans on the plate's near edge. Keep the
                # same arm clearance by parking farther toward the right base.
                right_goal[1] = -.270
            if clearance >= .145 and trial.contained_count == 0:
                if task.JOINED_SHELL:
                    self.bean_index = int(np.argmin(np.linalg.norm(positions[:, :2] - left_actual[:2], axis=1)))
                elif self.bean_index is None:
                    self.bean_index = int(np.argmin(positions[:, 2]))
                self.parked_shell = self.targets["right"].copy()
                self._enter("hover over escaped bean")
            elif task.JOINED_SHELL and trial.contained_count and self.ticks >= 36:
                self._enter("center retained beans over plate")
        elif self.phase in {"center retained beans over plate", "tip retained beans onto plate"}:
            lower = scalar_array(monitor.pod.data.root_link_pose_w)[0]
            # The right wrist reaches the near half of the plate with a
            # forward pitch. Positive pitch binds at the elbow and wrist limits.
            spill_target = np.asarray(PLATE_POS) + [0., -.028, .055]
            correction = spill_target - lower[:3]
            right_goal = right_actual + np.clip(correction, -.013, .013)
            if trial.contained_count == 0:
                self._enter("move held shell clear of plate")
            elif self.phase == "center retained beans over plate":
                self.plate_pitch = max(-.52, self.plate_pitch - .024)
                if np.linalg.norm(correction) < .024 and self.plate_pitch <= -.5:
                    normal = quat_xyzw_to_matrix(lower[3:])[:, 2]
                    self.pour_roll_sign = 1. if normal[1] < 0. else -1.
                    self._enter("tip retained beans onto plate")
            else:
                self.plate_pitch = max(-1.28, self.plate_pitch - .016)
                right_roll = self.pour_roll_sign * min(.70, .010 * self.ticks)
                if self.ticks > 160:
                    # Return upright over the plate. The open lip sweeps
                    # down as the wrist rolls back, freeing retained beans.
                    self._enter("move held shell clear of plate")
            right_pitch = self.plate_pitch
        elif self.phase == "hover over escaped bean":
            pickup_pitch = -.30
            if task.JOINED_SHELL:
                if bean[1] < -.08:
                    pickup_pitch = -.48
                self.left_pitch += float(np.clip(pickup_pitch - self.left_pitch, -.012, .012))
                left_step = .0026
            else:
                self.left_pitch = min(pickup_pitch, self.left_pitch + .012)
            left_goal = bean + [0., 0., .056]
            pitch_ready = not task.JOINED_SHELL or abs(self.left_pitch - pickup_pitch) < .02
            if np.linalg.norm(left_actual - left_goal) < .005 and pitch_ready:
                self._enter("lower to escaped bean")
        elif self.phase == "lower to escaped bean":
            # The distal pads extend along the pitched approach axis.
            pickup_offset = [-.0035, 0., .0112]
            if task.JOINED_SHELL:
                pickup_offset = .0117 * np.array([np.sin(self.left_pitch), 0., np.cos(self.left_pitch)])
            left_goal = bean + pickup_offset
            grip = -.009
            if np.linalg.norm(left_actual - left_goal) < .004:
                self.pick = left_goal.copy()
                self.bean_floor = float(bean[2])
                self._enter("pinch escaped bean")
        elif self.phase == "pinch escaped bean":
            left_goal, grip = self.pick, -.0018
            pads = {path.rsplit("/", 1)[-1] for path, sample in
                    monitor.contact_forces[f"bean_{self.bean_index}"].items()
                    if "/LeftArm/" in path and sample[0] > .006}
            if {"tip_left", "tip_right"} <= pads:
                self._enter("verify bean lift")
            elif self.ticks > 30:
                self._enter("stop")
        elif self.phase == "verify bean lift":
            left_goal, grip = self.pick + [0., 0., .052], -.0018
            if bean[2] > self.bean_floor + .032:
                self._enter("carry recovered bean")
        elif self.phase == "carry recovered bean":
            self.left_pitch = max(-.60, self.left_pitch - .012)
            left_goal = np.array([BOWL_POS[0] - .047, BOWL_POS[1] + .022, TABLE + .076])
            grip = -.0018
            if task.JOINED_SHELL:
                left_step = .0026
            if np.linalg.norm(left_actual - left_goal) < .006:
                self._enter("release recovered bean")
        elif self.phase == "release recovered bean":
            left_goal = np.array([BOWL_POS[0] - .047, BOWL_POS[1] + .022, TABLE + .076])
            if task._inside_bowl(bean):
                remaining = [i for i, item in enumerate(monitor.beans)
                             if not task._inside_bowl(task._world_body(item))]
                if remaining:
                    self.bean_index = remaining[0]
                    self._enter("hover over escaped bean")
                else:
                    self._enter("clear left for shell disposal")
            elif self.ticks > 40:
                self._enter("stop")
        elif self.phase == "clear left for shell disposal":
            if np.linalg.norm(left_actual - left_goal) < .018:
                if max(self._shell_pads(monitor).values()) > .04:
                    self._begin_disposal(right_actual)
                else:
                    self._enter("hover over empty shell")
        elif self.phase in {"hover over empty shell", "lower to empty shell", "pinch empty shell", "lift empty shell"}:
            pose = scalar_array(monitor.pod.data.root_link_pose_w)[0]
            frame = quat_xyzw_to_matrix(pose[3:])
            lengthwise = (-.016, .016, 0.)[self.shell_pick_attempt]
            shell_edge = pose[:3] + frame @ np.array([lengthwise, 0., 0.])
            right_pitch = -.52
            approach = (axis_angle_matrix(np.array([0., 1., 0.]), right_pitch) @ grip_rotation())[:, 2]
            right_goal = shell_edge - .011 * approach
            pads = self._shell_pads(monitor)
            if self.phase == "hover over empty shell":
                right_goal[2] += .056
                right_finger = -.017
                if np.linalg.norm(right_actual - right_goal) < .006:
                    self._enter("lower to empty shell")
            elif self.phase == "lower to empty shell":
                right_finger = -.014
                if np.linalg.norm(right_actual - right_goal) < .003:
                    self.shell_pick = right_goal.copy()
                    self.shell_contact_ticks = 0
                    self._enter("pinch empty shell")
            elif self.phase == "pinch empty shell":
                right_goal = self.shell_pick.copy()
                self.shell_contact_ticks = self.shell_contact_ticks + 1 if min(pads.values()) > .15 else 0
                if self.shell_contact_ticks >= 4:
                    self.shell_floor = float(pose[2])
                    self.shell_lost_ticks = 0
                    self._enter("lift empty shell")
                elif self.ticks >= 42:
                    self._retry_shell_pick()
            else:
                right_goal = self.shell_pick + [0., 0., .048]
                rise = float(pose[2]) - self.shell_floor
                right_step = .00075 if rise < .010 else right_step
                self.shell_lost_ticks = self.shell_lost_ticks + 1 if min(pads.values()) < .025 else 0
                if rise > .027 and min(pads.values()) > .03:
                    self._begin_disposal(right_actual)
                elif self.shell_lost_ticks >= 6 or self.ticks >= 75:
                    self._retry_shell_pick()
        if self.phase != "stop" and self.ticks > 160:
            self._enter("stop")
        left = self._drive("left", left_goal, 0., self.left_pitch, grip, step_limit=left_step)
        right = self._drive("right", right_goal, right_roll, right_pitch,
                            self._shell_grip(monitor, "right", right_finger), step_limit=right_step)
        self.left_command = left
        meta = {"request_stop": True} if self.phase == "stop" and self.ticks > 35 else {}
        return ActionChunk(actions=[Action(data=pack(left, right), meta=meta)], control_hz=1. / self._dt)


def edamame_scripted(oracle=False, plate_transfer=False, completion=False, action_log=None, **kwargs):
    if action_log:
        return EdamameReplay(action_log, **kwargs)
    if completion:
        controller = AdaptiveSeamOracle if os.getenv("EDAMAME_ADAPTIVE_SEAM") == "1" else CompletionOracle
        return controller(**kwargs)
    if plate_transfer:
        controller = PlateBeanRecovery if os.getenv("EDAMAME_PLATE_RECOVERY") == "1" else PlateTransferOracle
        return controller(**kwargs)
    return (EdamameOracle if oracle else EdamameScripted)(**kwargs)
