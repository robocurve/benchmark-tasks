"""Synthetic expert: robot joint/gripper commands, measured feedback and real contact."""
import json
import time
from pathlib import Path
import numpy as np
from yam_arena import ik, station
from yam_arena.constants import READY


def downward(closing_direction):
    x = np.asarray(closing_direction, float)
    x[2] = 0
    x /= np.linalg.norm(x)
    z = np.array([0., 0., -1.])
    return np.column_stack([x, np.cross(z, x), z])


class Demo:
    def __init__(self, env, reader, *, recorder=None, output=None, parameters=None):
        self.env, self.reader, self.recorder = env, reader, recorder
        self.output = Path(output) if output else None
        self.parameters = parameters or {}
        self.command = {s: np.r_[READY, 1.] for s in ["left", "right"]}
        self.steps, self.rows, self.phases = 0, [], []
        self.started = time.monotonic()
        self.worst_position_error = self.worst_rotation_error = 0.
        self.held_wrist = {}

    def tick(self, label):
        import torch
        if self.steps >= 2400:
            raise RuntimeError("Scripted trial exceeded its 2400-step allowance")
        action = np.r_[self.command["left"], self.command["right"]]
        self.env.step(torch.as_tensor(action[None], device=self.reader.base.device, dtype=torch.float32))
        self.steps += 1
        score = self.reader.score()
        if self.recorder:
            self.recorder.capture()
        row = dict(step=self.steps, time_s=self.steps / 20, phase=label, action=action.tolist(),
                   measured=self.reader.controls(),
                   joint_pos=np.r_[self.reader.arm("left")[0], self.reader.arm("left")[1],
                                   self.reader.arm("right")[0], self.reader.arm("right")[1]].tolist(),
                   tcp={s: self.reader.arm(s)[2].tolist() for s in ["left", "right"]},
                   grippers={s: self.reader.arm(s)[1] for s in ["left", "right"]}, **score)
        self.rows.append(row)
        if self.output and self.steps % 20 == 0:
            (self.output / "progress.json").write_text(json.dumps(row, indent=2) + "\n")
        return score

    def solve_tick(self, side, target, rotation, grip, label, seed, servo=False):
        q, pe, re = ik.solve(target, ik.matrix_to_quat_xyzw(rotation), seed,
                             base_pos=station.BASES[side], iters=120)
        self.worst_position_error = max(self.worst_position_error, pe)
        self.worst_rotation_error = max(self.worst_rotation_error, float(np.degrees(re)))
        if pe > .012 or re > np.radians(12):
            raise RuntimeError(f"Unreachable {label}: {pe*1000:.1f} mm / {np.degrees(re):.1f} degrees")
        if servo:
            measured = self.reader.arm(side)[0]
            gain = float(self.parameters.get("servo_gain", .20))
            q = np.clip(self.command[side][:6] + np.clip(gain * (q - measured), -.025, .025), ik.LOWER, ik.UPPER)
        self.command[side] = np.r_[q, grip]
        self.tick(label)
        return q

    def move(self, side, goal, rotation, seconds, grip, label, verify=True):
        q, g, start, r0 = self.reader.arm(side)
        goal = np.asarray(goal, float)
        self.phases.append(dict(label=label, start_step=self.steps, arm=side, target=goal.tolist()))
        print("DEMO_PHASE", label, "target", goal.tolist(), flush=True)
        count = round(seconds * 20)
        for k in range(1, count + 1):
            t = ik.smoothstep(k / count)
            q = self.solve_tick(side, start + t * (goal - start), ik.slerp_matrix(r0, rotation, t),
                                g + t * (grip - g), label, q)
        error = np.linalg.norm(self.reader.arm(side)[2] - goal)
        if verify and error > .025:
            raise RuntimeError(f"Blocked {label}: TCP tracking error {error*1000:.1f} mm")

    def hold(self, seconds, label):
        for _ in range(round(seconds * 20)):
            result = self.tick(label)
        return result

    def grip(self, side, value, seconds, label):
        start = self.reader.arm(side)[1]
        count = round(seconds * 20)
        for k in range(1, count + 1):
            self.command[side][-1] = start + ik.smoothstep(k / count) * (value - start)
            self.tick(label)

    def ready(self, side, seconds=2.):
        q, g, _, _ = self.reader.arm(side)
        count = round(seconds * 20)
        for k in range(1, count + 1):
            t = ik.smoothstep(k / count)
            self.command[side] = np.r_[q + t * (np.array(READY) - q), g + t * (1 - g)]
            self.tick(side + "_return_ready")

    def carry(self, side, name, goal, seconds, label, *, upright=False, yaw_degrees=0.,
              stop_on_contact=False, stop_when_supported=False):
        body = self.reader.body(name)
        start, r0 = body["pose"][:3].copy(), body["rotation"].copy()
        if name not in self.held_wrist:
            self.held_wrist[name] = r0.T @ self.reader.arm(side)[3]
        goal = np.asarray(goal, float)
        grip = float(self.command[side][-1])
        self.phases.append(dict(label=label, start_step=self.steps, arm=side,
                                object=name, root_target=goal.tolist()))
        print("DEMO_PHASE", label, "object", name, "target", goal.tolist(), flush=True)
        count = round(seconds * 20)
        for k in range(1, count + 1):
            if stop_when_supported and self.reader.controls()["pot_on_trivet"]:
                break
            t = ik.smoothstep(k / count)
            desired = start + t * (goal - start)
            angle = np.radians(yaw_degrees)
            rz = np.array([[np.cos(angle), -np.sin(angle), 0],
                           [np.sin(angle), np.cos(angle), 0], [0, 0, 1.]])
            desired_r = ik.slerp_matrix(r0, rz, t) if upright else r0
            body = self.reader.body(name)
            q, _, tcp, wrist = self.reader.arm(side)
            distance = np.linalg.norm(tcp - body["grasp"])
            if distance > .055:
                raise RuntimeError(f"{name} left the physical grip in {label}; distance {distance*1000:.1f} mm")
            correction = desired_r @ body["rotation"].T
            target = desired + correction @ (tcp - body["pose"][:3])
            rotation = ik.slerp_matrix(np.eye(3), correction, .5) @ desired_r @ self.held_wrist[name]
            self.solve_tick(side, target, rotation, grip, label, q, servo=True)
            if stop_on_contact and self.reader.scorer.contact_ever:
                break

    def move_egg(self):
        side = "right"
        egg = self.reader.body("egg")
        axis = egg["rotation"][:, 0]
        direction = np.array([axis[1], -axis[0], 0.])
        pitch = np.radians(float(self.parameters.get("egg_pitch_deg", 60.)))
        forward = np.array([axis[0], axis[1], 0.])
        forward /= np.linalg.norm(forward)
        x = direction / np.linalg.norm(direction)
        z = np.r_[np.cos(pitch) * forward[:2], -np.sin(pitch)]
        rotation = np.column_stack([x, np.cross(z, x), z])
        if self.parameters.get("egg_grip_flip"):
            rotation = downward(-direction)
        grasp = egg["grasp"] + [0, 0, float(self.parameters.get("egg_grasp_raise_m", .002))]
        self.move(side, grasp + [0, 0, .100], rotation, 2.5, .80, "egg_hover")
        self.move(side, grasp, rotation, 2., .80, "egg_descend")
        self.grip(side, float(self.parameters.get("egg_grip", .28)), 1.2, "egg_close")
        self.hold(.5, "egg_grip_settle")
        initial = self.reader.body("egg")["pose"][:3].copy()
        target = initial.copy()
        target[2] = .875
        self.carry(side, "egg", target, 2.8, "egg_lift")
        if self.reader.body("egg")["low"][2] < .84:
            raise RuntimeError("Egg did not physically lift; refusing to move the pot")
        cup = np.array(self.reader.specs["egg_cup"].position)
        target[:2] = cup[:2]
        self.carry(side, "egg", target, 3., "egg_over_rest")
        egg = self.reader.body("egg")
        target = egg["pose"][:3].copy()
        target[:2] = cup[:2]
        target[2] += cup[2] + .015 - egg["low"][2]
        self.carry(side, "egg", target, 3., "egg_lower_into_rest")
        self.hold(.5, "egg_supported")
        self.grip(side, .68, 1., "egg_release")
        _, _, tcp, wrist = self.reader.arm(side)
        self.move(side, tcp + [0, 0, .14], wrist, 2.2, .68, "egg_retreat")
        self.ready(side)
        self.hold(1., "egg_safe_hold")
        state = self.reader.controls()
        if not state["egg_clear"] or not state["egg_supported"]:
            raise RuntimeError("Egg is not safely supported away from the trivet")

    def pickup_pot(self):
        side = "left"
        pitch = np.radians(float(self.parameters.get("pot_pitch_deg", 35.)))
        rotation = np.array([[np.sin(pitch), 0, np.cos(pitch)],
                             [0, -1, 0], [np.cos(pitch), 0, -np.sin(pitch)]])
        grasp = self.reader.body("pot")["grasp"]
        self.move(side, grasp + [-.065, 0, .13], rotation, 2.5, .95, "pot_hover")
        self.move(side, grasp - rotation[:, 2]*.085, rotation, 2., .95, "pot_align")
        self.move(side, grasp, rotation, 2., .95, "pot_enter")
        self.grip(side, 0., 1.2, "pot_close")
        self.hold(.5, "pot_grip_settle")
        target = self.reader.body("pot")["pose"][:3].copy()
        target[2] = .86
        self.carry(side, "pot", target, 3., "pot_lift", upright=True)
        if self.reader.body("pot")["low"][2] < .79:
            raise RuntimeError("Pot did not physically lift by its handle")

    def place_pot(self, *, unsafe=False):
        side = "left"
        target = np.array(self.reader.specs["trivet"].position)
        target[2] = .86
        yaw = float(self.parameters.get("pot_final_yaw_deg", -35.))
        self.carry(side, "pot", target, 3.5, "pot_over_trivet", upright=True, yaw_degrees=yaw)
        target[2] = self.reader.layout["table_top_z"] + .018 + (.015 if unsafe else .001)
        self.carry(side, "pot", target, 3., "unsafe_pot_contact" if unsafe else "pot_seat",
                   upright=True, yaw_degrees=yaw, stop_on_contact=unsafe,
                   stop_when_supported=not unsafe)
        # Remove accumulated loaded-arm setpoint bias once the table supports
        # the pot. Otherwise unloading the gripper springs the hand upwards.
        self.command[side][:6] = self.reader.arm(side)[0]
        self.hold(.35, "pot_seated_hold")
        if unsafe:
            target = self.reader.body("pot")["pose"][:3].copy() + [0, 0, .10]
            self.carry(side, "pot", target, 2., "unsafe_recovery_stays_zero", upright=True, yaw_degrees=yaw)
            return self.hold(1., "unsafe_final")
        self.grip(side, 1., .7, "pot_release")
        _, _, tcp, wrist = self.reader.arm(side)
        back = -self.reader.body("pot")["rotation"][:, 0].copy()
        back[2] = 0
        back /= np.linalg.norm(back)
        self.move(side, tcp + back*.085, wrist, 2., 1., "pot_slide_clear", verify=False)
        _, _, tcp, wrist = self.reader.arm(side)
        self.move(side, tcp + [0, 0, .10], wrist, 2., 1., "pot_retreat")
        self.ready(side)
        return self.hold(1.5, "final_safe_hold")

    def run(self):
        mode = self.parameters.get("mode", "demo")
        if mode != "unsafe_contact":
            if mode != "pot_lift_only":
                self.move_egg()
                if mode == "egg_only":
                    return self.hold(1., "egg_only_final")
        self.pickup_pot()
        if mode == "pot_lift_only":
            return self.hold(2., "pot_lift_validation")
        return self.place_pot(unsafe=mode == "unsafe_contact")

    def report(self):
        if self.output and self.rows:
            with (self.output / "trajectory.jsonl").open("w") as out:
                for row in self.rows:
                    out.write(json.dumps(row) + "\n")
            np.savez_compressed(self.output / "trajectory.npz",
                                actions=np.array([r["action"] for r in self.rows]),
                                egg_poses=np.array([r["measured"]["egg_pose"] for r in self.rows]),
                                pot_poses=np.array([r["measured"]["pot_pose"] for r in self.rows]),
                                control_hz=20)
        return dict(control_steps=self.steps, simulated_seconds=self.steps / 20,
                    wall_seconds=time.monotonic() - self.started,
                    parameters=self.parameters, phases=self.phases,
                    worst_ik_position_error_m=self.worst_position_error,
                    worst_ik_rotation_error_deg=self.worst_rotation_error,
                    object_motion="Robot contacts/friction only; no object attachments or pose writes during trial")


class Replay(Demo):
    """Validate an exported joint trajectory in a fresh physical environment."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        import hashlib
        self.reference = Path(self.parameters["replay_file"])
        self.reference_hash = hashlib.sha256(self.reference.read_bytes()).hexdigest()
        self.actions = np.load(self.reference)["actions"]
        if (self.actions.ndim != 2 or self.actions.shape[1] != 14
                or len(self.actions) > 2380 or not np.isfinite(self.actions).all()):
            raise ValueError("Replay requires finite 14-D commands within the run limit")

    def run(self):
        self.phases.append(dict(label="frozen_joint_trajectory_replay", start_step=0,
                                source_sha256=self.reference_hash))
        for row in self.actions:
            self.command["left"] = row[:7].copy()
            self.command["right"] = row[7:].copy()
            self.tick("joint_trajectory_replay")
        return self.hold(1., "replay_final_verification")

    def report(self):
        return dict(super().report(), replay_source_sha256=self.reference_hash,
                    replay_actions=len(self.actions), ik_solved_during_replay=False)
