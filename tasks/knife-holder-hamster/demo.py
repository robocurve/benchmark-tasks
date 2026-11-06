"""Physical scripted policy: clear the toy, then store the knife with a reachable grip.

Only robot joint/gripper targets are issued. Object state is read for planning;
there are no attachment joints, object pose writes, or object teleports.
"""

import json
from pathlib import Path
import time

import numpy as np

from yam_arena import ik, station
from yam_arena.constants import READY

from demo_runtime import rotation


FRONT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
KNIFE_VERTICAL = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)


class Demo:
    def __init__(self, env, reader, *, recorder=None, output=None, parameters=None):
        import torch

        self.torch, self.env, self.reader = torch, env, reader
        self.recorder, self.output = recorder, Path(output) if output else None
        self.parameters = parameters or {}
        self.knife_arm = self.parameters.get("knife_arm", "left")
        self.hamster_arm = self.parameters.get("hamster_arm", "right")
        self.command = {s: np.r_[READY, 1.0] for s in ("left", "right")}
        self.steps, self.rows, self.phases = 0, [], []
        self.worst_position_error, self.worst_rotation_error = 0.0, 0.0
        self.started = time.monotonic()

    def tick(self, phase):
        action = np.r_[self.command["left"], self.command["right"]]
        self.env.step(self.torch.as_tensor(action[None], dtype=self.torch.float32, device=self.reader.base.device))
        self.steps += 1
        score = self.reader.score()
        if self.recorder:
            self.recorder.capture()
        if self.steps % 5 == 0:
            self.rows.append({"step": self.steps, "phase": phase, "knife_pose": self.reader.body("knife").pose.tolist(),
                              "hamster_pose": self.reader.body("hamster").pose.tolist(),
                              "tcp": {s: self.reader.arm(s)[2].tolist() for s in ("left", "right")},
                              "gripper_openings": {s: self.reader.arm(s)[1] for s in ("left", "right")}, **score})
            if self.output and self.steps % 20 == 0:
                (self.output/"progress.json").write_text(json.dumps(self.rows[-1], indent=2)+"\n")
        return score

    def move(self, side, goal, rotation_target, seconds, grip, label):
        q, grip0, start, rotation0 = self.reader.arm(side)
        goal = np.array(goal, float)
        self.phases.append({"label": label, "start_step": self.steps, "tcp_target": goal.tolist(), "arm": side})
        print("DEMO_PHASE", label, "hamster", self.reader.body("hamster").pose[:3].tolist(),
              "knife", self.reader.body("knife").pose[:3].tolist(), flush=True)
        for k in range(1, round(seconds * 20) + 1):
            t = ik.smoothstep(k / round(seconds * 20))
            target = start + t * (goal - start)
            r = ik.slerp_matrix(rotation0, rotation_target, t)
            q, pe, re = ik.solve(target, ik.matrix_to_quat_xyzw(r), q,
                                 base_pos=station.BASES[side], iters=100)
            self.worst_position_error = max(self.worst_position_error, pe)
            self.worst_rotation_error = max(self.worst_rotation_error, float(np.degrees(re)))
            if pe > 0.012 or np.degrees(re) > 12:
                raise RuntimeError(f"Unreachable demo waypoint {label}: {pe*1000:.1f} mm, {np.degrees(re):.1f} deg")
            self.command[side] = np.r_[q, grip0 + t * (grip - grip0)]
            self.tick(label)

    def hold(self, side, seconds, grip, label):
        q = self.command[side][:6].copy()
        grip0 = self.reader.arm(side)[1]
        self.phases.append({"label": label, "start_step": self.steps, "arm": side})
        print("DEMO_PHASE", label, flush=True)
        count = round(seconds*20)
        for k in range(1, count+1):
            t = ik.smoothstep(k/count)
            self.command[side] = np.r_[q, grip0+t*(grip-grip0)]
            self.tick(label)

    def ready(self, side, seconds=2.0):
        q, g, _, _ = self.reader.arm(side)
        for k in range(1, round(seconds*20)+1):
            t = ik.smoothstep(k / round(seconds*20))
            self.command[side] = np.r_[q + t*(np.array(READY)-q), g+t*(1-g)]
            self.tick(f"{side}_return_ready")

    def carry_knife(self, tip_goal, rotation_goal, seconds, label):
        """Servo the held blade from measured poses using only robot actions."""
        side = self.knife_arm
        knife = self.reader.body("knife")
        start, start_r = knife.pose[:3].copy(), rotation(knife.pose[3:])
        self.phases.append({"label": label, "start_step": self.steps, "knife_tip_target": list(tip_goal), "arm": side})
        print("DEMO_PHASE", label, "target_tip", list(tip_goal), flush=True)
        count = round(seconds*20)
        for k in range(1, count+1):
            t = ik.smoothstep(k/count)
            desired_tip = start+t*(np.array(tip_goal)-start)
            desired_r = ik.slerp_matrix(start_r, rotation_goal, t)
            knife = self.reader.body("knife")
            kr = rotation(knife.pose[3:])
            measured_q, _, tcp, er = self.reader.arm(side)
            if np.linalg.norm(tcp-(knife.pose[:3]+kr@np.array([0, 0, 0.215]))) > 0.055:
                raise RuntimeError(f"Knife left the physical gripper during {label}")
            # Correct both measured position and angle. Even a flat-sided grip
            # has a little compliance under gravity and contact with the slot.
            correction = desired_r @ kr.T
            target = desired_tip + correction @ (tcp-knife.pose[:3])
            target_r = correction @ er
            q, pe, re = ik.solve(target, ik.matrix_to_quat_xyzw(target_r), measured_q,
                                 base_pos=station.BASES[side], iters=100)
            self.worst_position_error = max(self.worst_position_error, pe)
            self.worst_rotation_error = max(self.worst_rotation_error, float(np.degrees(re)))
            if pe > 0.012 or np.degrees(re) > 12:
                raise RuntimeError(f"Held-knife target unreachable in {label}: {pe*1000:.1f} mm, {np.degrees(re):.1f} deg")
            # Increment the commanded joints from measured Cartesian error.
            # Absolute IK alone leaves gravity sag with the supplied compliant
            # position actuators. This outer loop removes that steady error.
            delta = np.clip(0.12*(q-measured_q), -0.020, 0.020)
            target_q = np.clip(self.command[side][:6]+delta, ik.LOWER, ik.UPPER)
            self.command[side] = np.r_[target_q, 0.0]
            self.tick(label)

    def run(self):
        layout = self.reader.layout
        top = layout["table_top_z"]
        grasp = np.array(layout["hamster_grasp_center_world"])
        # Same front approach and physical side grip as the measured extraction.
        self.move(self.hamster_arm, [0.23, 0, top+0.125], FRONT, 2.0, 0.64, "hamster_hover")
        self.move(self.hamster_arm, [0.27, 0, grasp[2]], FRONT, 2.0, 0.64, "hamster_align")
        self.move(self.hamster_arm, grasp, FRONT, 2.5, 0.64, "hamster_enter_bay")
        self.hold(self.hamster_arm, 1.0, 0.0, "hamster_close")
        self.move(self.hamster_arm, grasp+[0, 0, 0.020], FRONT, 1.25, 0.0, "hamster_lift")
        self.move(self.hamster_arm, [0.235, 0, grasp[2]+0.025], FRONT, 2.25, 0.0, "hamster_withdraw")
        if self.reader.body("hamster").pose[0] > 0.29:
            raise RuntimeError("Hamster was not physically withdrawn; refusing to continue")
        if "hamster_place_tcp" in self.parameters:
            place = np.array(self.parameters["hamster_place_tcp"])
            self.move(self.hamster_arm, [place[0], place[1], top+0.105], FRONT, 2.5, 0.0, "hamster_carry")
        else:
            self.hold(self.hamster_arm, 0.5, 0.0, "hamster_hold_outside")
            _, _, place, _ = self.reader.arm(self.hamster_arm)
            place = place.copy()
            place[2] += top+0.001-self.reader.body("hamster").low[2]
        self.move(self.hamster_arm, place, FRONT, 1.5, 0.0, "hamster_lower")
        self.hold(self.hamster_arm, 1.0, 1.0, "hamster_release")
        self.move(self.hamster_arm, place+[-0.065, 0, 0.075], FRONT, 1.5, 1.0, "hamster_retreat")
        self.ready(self.hamster_arm)

        knife = self.reader.body("knife")
        initial_r = rotation(knife.pose[3:])
        handle_local = np.array([0.0, 0.0, 0.215])
        grasp_r = initial_r @ (KNIFE_VERTICAL.T @ FRONT)
        handle = knife.pose[:3] + initial_r @ handle_local
        # TCP is 13 mm short of the fingertip: the slight bias keeps fingers off
        # the tabletop while still overlapping the top half of the handle.
        pickup_bias = float(self.parameters.get("knife_pickup_bias_m", 0.001))
        pickup = handle - grasp_r[:, 2]*pickup_bias
        arm = self.knife_arm
        self.move(arm, pickup-grasp_r[:, 2]*0.070, grasp_r, 2.5, 0.70, "knife_hover")
        self.move(arm, pickup, grasp_r, 2.0, 0.70, "knife_descend")
        self.hold(arm, 0.4, 0.70, "knife_align_handle")
        self.hold(arm, 1.25, 0.0, "knife_close")
        self.move(arm, pickup+[0, 0, 0.070], grasp_r, 2.0, 0.0, "knife_lift")
        held = self.reader.body("knife")
        measured_handle = held.pose[:3]+rotation(held.pose[3:])@handle_local
        if np.linalg.norm(self.reader.arm(arm)[2]-measured_handle) > 0.035:
            raise RuntimeError("Knife handle did not follow the physical fingers")
        tip_target = np.array(layout["target_knife_tip_world"])
        clear_tip = tip_target.copy();clear_tip[2] = self.reader.scorer.holder_high[2]+0.025
        turn = clear_tip.copy();turn[0] = layout["holder_pose"][0]-0.12
        self.carry_knife(turn, KNIFE_VERTICAL, 5.0, "knife_turn_upright")
        self.carry_knife(clear_tip, KNIFE_VERTICAL, 4.0, "knife_above_slot")
        self.carry_knife(clear_tip, KNIFE_VERTICAL, 4.0, "knife_align_slot")
        self.carry_knife(tip_target, KNIFE_VERTICAL, 12.0, "knife_insert")
        self.carry_knife(tip_target, KNIFE_VERTICAL, 4.0, "knife_seat")
        self.hold(arm, 1.0, 1.0, "knife_release")
        _, _, tcp, er = self.reader.arm(arm)
        self.move(arm, tcp+[-0.11, 0, 0.050], er, 2.0, 1.0, "knife_retreat")
        self.ready(arm)
        for _ in range(40):
            result = self.tick("final_hold")
        return result

    def report(self):
        return {"control_steps": self.steps, "trajectory": self.rows, "phases": self.phases,
                "simulated_seconds": self.steps/20, "wall_seconds": time.monotonic()-self.started,
                "worst_ik_position_error_m": self.worst_position_error,
                "worst_ik_rotation_error_deg": self.worst_rotation_error,
                "knife_arm": self.knife_arm, "hamster_arm": self.hamster_arm,
                "object_motion": "Physical contacts only; robot joint/gripper actions; no attachments or object pose writes"}
