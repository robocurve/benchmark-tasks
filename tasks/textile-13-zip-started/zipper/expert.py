"""Scripted two-arm expert for the open-start padded jacket."""

import argparse
import json
import math
import traceback
from contextlib import ExitStack
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .jacket_fabric import FABRIC_NAME, THICKNESS
from .mechanism import CONTROL_HZ, END_TOLERANCE, LENGTH, PIN_SEATED_TOLERANCE, INITIAL_PIN
from .scoring import zip_complete, zip_score


ARM_ACTION = {"left": slice(0, 7), "right": slice(7, 14)}
PIN_GRASP_YAW = -math.pi / 4


class ZipExpert:
    def __init__(self, robot, video, max_steps):
        from yam_arena.constants import READY

        self.robot = robot
        self.ready = np.r_[READY, 1.0]
        self.action = np.tile(self.ready, 2)
        self.video = video
        self.history = [robot.reading()]
        self.max_steps = max_steps

    def step(self):
        if len(self.history) > self.max_steps:
            raise RuntimeError("Expert reached the action limit")
        command = self.robot._torch.as_tensor(
            self.action, dtype=self.robot._torch.float32, device=self.robot._base.device,
        )
        self.robot._env.step(command.unsqueeze(0))
        self.history.append(self.robot.reading())
        if self.video is not None:
            self.video.send(self.frame())

    def frame(self):
        import cv2

        def rgb(name):
            buffer = self.robot._base.scene[name].data.output["rgb"]
            buffer = buffer.torch if hasattr(buffer, "torch") else buffer
            return buffer[0, ..., :3].cpu().numpy()

        wrists = [cv2.resize(rgb(name), (320, 180)) for name in ("left_cam", "right_cam")]
        return np.ascontiguousarray(np.vstack((rgb("top_cam"), np.hstack(wrists))))

    def move(self, side, waypoints, *, precise=False):
        from yam_arena.constants import FINGER_OPEN
        from yam_arena.demo import ArmPlan
        from yam_arena import ik
        from yam_arena.station import BASES

        target = ARM_ACTION[side]
        current = self.action[target]
        plan = ArmPlan(waypoints, BASES[side], q_start=current[:6], finger_start=current[6] * FINGER_OPEN)
        for i in range(1, math.ceil(plan.duration * CONTROL_HZ) + 1):
            if precise:
                pos, rot, finger, _ = plan.target(i / CONTROL_HZ)
                q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), plan.q,
                                     base_pos=BASES[side], iters=80, tol=0.000005, damping=0.03)
                plan.q = q
                plan.worst_ik_mm = max(plan.worst_ik_mm, pe * 1000)
                plan.worst_ik_deg = max(plan.worst_ik_deg, float(np.degrees(re)))
                self.action[target] = np.r_[q, np.clip(finger / FINGER_OPEN, 0, 1)]
            else:
                self.action[target] = plan.action7(i / CONTROL_HZ)[0]
            self.step()
        print(f"{side} {waypoints[-1].label}: IK {plan.worst_ik_mm:.1f} mm, {plan.worst_ik_deg:.1f} deg; "
              f"zipper {self.history[-1]}", flush=True)

    def hand_pose(self, side):
        from yam_arena.ik import tcp_pose
        from yam_arena.station import BASES

        name = f"{side}_robot"
        joint_ids, finger_ids = self.robot._ids[name]
        measured = self.robot._base.scene[name].data.joint_pos.torch[0].cpu().numpy()
        tip, _ = tcp_pose(measured[joint_ids], BASES[side])
        return tip, -sum(measured[finger_ids])

    def target_pose(self, side):
        from yam_arena.ik import tcp_pose
        from yam_arena.station import BASES

        return tcp_pose(self.action[ARM_ACTION[side]][:6], BASES[side])[0]

    def grasp_tab(self, yaw):
        from yam_arena.demo import Waypoint

        x, y, center_z = self.robot.handle_positions()[1]
        z = center_z + 0.004
        opened, pinched = -0.018, -0.004
        self.move("right", [
            Waypoint(1.5, (x, y, z + 0.075), opened, "approach", yaw),
            Waypoint(1.3, (x, y, z), opened, "descend", yaw),
            Waypoint(0.8, (x, y, z), pinched, "grasp", yaw),
        ])
        tip, jaw_gap = self.hand_pose("right")
        delta = (tip - self.robot.handle_positions()[1]) * 1000
        print(f"right measured TCP minus handle (mm): {np.round(delta, 1)}; jaw gap {jaw_gap * 1000:.1f} mm",
              flush=True)

    def brace_cloth(self, xy=(0.252, 0.133), depth=0.004):
        from yam_arena.constants import PAD_TIP_PAST_TCP
        from yam_arena.demo import Waypoint

        nodes = self.robot._base.scene[FABRIC_NAME].data.nodal_pos_w.torch[0].cpu().numpy()
        x, y = xy
        distance = np.linalg.norm(nodes[:, :2] - (x, y), axis=1)
        nearest = np.argpartition(distance, 5)[:5]
        z = float(np.median(nodes[nearest, 2])) + THICKNESS / 2 + PAD_TIP_PAST_TCP - depth
        self.move("left", [
            Waypoint(1.5, (x, y, z + 0.065), -0.004, "approach cloth brace"),
            Waypoint(1.3, (x, y, z), -0.004, "brace cloth panel"),
        ])
        return (x, y, z)

    def release(self, side, point, *, yaw=0.0):
        from yam_arena.demo import Waypoint
        from yam_arena.ik import smoothstep

        x, y, z = point
        self.move(side, [
            Waypoint(0.65, (x, y, z), -0.018, "open", yaw),
            Waypoint(1.1, (x, y, z + 0.075), -0.022, "retreat", yaw),
        ])
        target = ARM_ACTION[side]
        start = self.action[target].copy()
        for i in range(1, 31):
            self.action[target] = start + smoothstep(i / 30) * (self.ready - start)
            self.step()

    def insert_pin(self):
        from yam_arena.demo import Waypoint
        from yam_arena.urdf import quat_xyzw_to_matrix

        center = self.robot.handle_positions()[2]
        grip_offset = np.array((0, 0, 0.012))
        point = center + grip_offset
        yaw = PIN_GRASP_YAW
        opened, pinched = -0.025, -0.001
        self.move("right", [Waypoint(1.3, tuple(point + (0, 0, 0.060)), opened,
                                       "approach loose insertion pin", yaw)])
        for _ in range(3):
            point = self.robot.handle_positions()[2] + grip_offset
            self.move("right", [Waypoint(0.65, tuple(point), opened, "straddle reinforced tape end", yaw)])
            hand, _ = self.hand_pose("right")
            if np.linalg.norm(hand - self.robot.handle_positions()[2] - grip_offset) < 0.0015:
                break
        point = self.robot.handle_positions()[2] + grip_offset
        self.move("right", [Waypoint(0.8, tuple(point), pinched, "grasp tape behind pin", yaw)])
        hand, gap = self.hand_pose("right")
        print(f"Initial pin grip mm {1000 * (hand-self.robot.handle_positions()[2])}, gap {1000*gap:.1f}",flush=True)
        # Retract the pin clear of the box mouth before bringing the axes into alignment.
        for _ in range(4):
            state = self.robot.reading()
            if state.pin_x_m >= 0.018:
                break
            rotation = quat_xyzw_to_matrix(self.robot.jacket.data.root_link_pose_w.torch[0, 3:].cpu().numpy())
            hand = self.target_pose("right")
            clear = hand + rotation[:, 0] * (0.025 - state.pin_x_m)
            self.move("right", [Waypoint(1.0, tuple(clear), pinched, "clear pin tip from box mouth", yaw)])
        hand, gap = self.hand_pose("right")
        print(f"Pin grip tracking mm {1000 * (hand-self.robot.handle_positions()[2])}, gap {1000*gap:.1f}",flush=True)
        # Use short feedback-controlled increments. Align before advancing and
        # retract clear of the mouth when contact knocks the pin off axis.
        axial_goal = 0.018
        for cycle in range(240):
            state = self.robot.reading()
            if state.engaged:
                break
            transverse = max(abs(state.pin_y_m), abs(state.pin_z_m))
            if state.pin_x_m > 0.011 and transverse < 0.0008:
                axial_goal = max(0.0105, min(axial_goal, state.pin_x_m) - 0.0015)
            elif transverse < PIN_SEATED_TOLERANCE:
                axial_goal = max(0.0, min(axial_goal, state.pin_x_m) - 0.0005)
            elif state.pin_x_m < 0.0105 and transverse > 0.00065:
                axial_goal = min(0.012, max(axial_goal, state.pin_x_m + 0.001))
            error = np.array([axial_goal - state.pin_x_m, -state.pin_y_m, -state.pin_z_m])
            # Joint-limit/contact velocity spikes are unsuitable for the slow
            # insertion servo. Estimate motion over several control samples.
            lag = min(4, len(self.history) - 1)
            past = self.history[-1 - lag]
            offset = np.array((state.pin_x_m, state.pin_y_m, state.pin_z_m))
            previous = np.array((past.pin_x_m, past.pin_y_m, past.pin_z_m))
            velocity = (offset - previous) / (lag / CONTROL_HZ) if lag else np.zeros(3)
            correction = np.clip(0.25 * error - 0.12 * velocity,
                                 (-0.001, -0.0015, -0.0005), (0.001, 0.0015, 0.0005))
            rotation = quat_xyzw_to_matrix(self.robot.jacket.data.root_link_pose_w.torch[0, 3:].cpu().numpy())
            target = self.target_pose("right") + rotation @ correction
            hand, gap = self.hand_pose("right")
            if np.linalg.norm(target - hand) > 0.035:
                raise RuntimeError(f"Insertion tracking exceeded 35 mm: target-minus-hand mm "
                                   f"{1000*(target-hand)}, root pose "
                                   f"{self.robot.jacket.data.root_link_pose_w.torch[0].cpu().numpy()}")
            self.move("right", [Waypoint(0.2, tuple(target), pinched, "servo insertion", yaw)], precise=True)
            if cycle % 10 == 0:
                hand, gap = self.hand_pose("right")
                print(f"Pin servo cycle {cycle}: grip offset mm "
                      f"{1000 * (hand-self.robot.handle_positions()[2])}, gap {1000*gap:.1f}; "
                      f"command tracking mm {1000*(target-hand)}; correction mm {1000*correction}", flush=True)
        hand, _ = self.hand_pose("right")
        if not self.robot.reading().engaged:
            raise RuntimeError("The robot did not insert the pin into the retaining box")
        return hand

    def run(self):
        from yam_arena.demo import Waypoint
        from yam_arena.urdf import quat_xyzw_to_matrix

        brace = self.brace_cloth()
        pin_hand = self.insert_pin()
        self.release("right", pin_hand, yaw=PIN_GRASP_YAW)
        pull_yaw = math.pi / 2
        for attempt in range(3):
            self.grasp_tab(pull_yaw)
            tab = self.robot.handle_positions()[1] + (0, 0, 0.004)
            rotation = self.robot.jacket.data.root_link_pose_w.torch[0, 3:].cpu().numpy()
            direction = quat_xyzw_to_matrix(rotation)[:, 0]
            travel = max(0.0, LENGTH - self.history[-1].slider_m)
            finish = tab + direction * (travel + 0.006)
            self.move("right", [
                Waypoint(max(1.2, travel / 0.036), tuple(finish), -0.004, "zip to collar", pull_yaw),
                Waypoint(0.45, tuple(finish), -0.004, "settle at top", pull_yaw),
            ])
            hand, _ = self.hand_pose("right")
            self.release("right", hand, yaw=pull_yaw)
            if self.history[-1].slider_m >= LENGTH - END_TOLERANCE:
                break
        self.release("left", brace)
        for _ in range(30):
            self.step()


def record(robot, scene, output, max_steps):
    import imageio_ffmpeg
    from PIL import Image

    robot.reset(scene, seed=0)
    initial = robot.reading()
    if initial.engaged or initial.pin_y_m > -0.012 or abs(initial.slider_m) > 0.004:
        raise RuntimeError(f"The pin must start outside the retainer: {initial}")
    video = imageio_ffmpeg.write_frames(
        str(output / "scripted-demo.mp4"), (640, 540), fps=CONTROL_HZ, codec="libx264",
        pix_fmt_out="yuv420p", macro_block_size=2, output_params=["-movflags", "+faststart", "-preset", "fast"],
    )
    failure = None
    try:
        video.send(None)
        expert = ZipExpert(robot, video, max_steps)
        Image.fromarray(expert.frame()[:360]).save(output / "setup.png")
        for _ in range(10):
            expert.step()
        expert.run()
    except Exception:
        failure = traceback.format_exc()
        print(failure, flush=True)
    finally:
        if "expert" in locals():
            final = expert.history[-1]
            result = {"score": zip_score(expert.history), "goal_met": zip_complete(final),
                      "initial": asdict(initial), "final": asdict(final),
                      "steps": len(expert.history) - 1, "control_hz": CONTROL_HZ}
            if failure is not None:
                result["error"] = failure
            (output / "scripted-score.json").write_text(json.dumps(result, indent=2) + "\n")
            (output / "states.json").write_text(json.dumps([asdict(r) for r in expert.history]) + "\n")
            print(json.dumps(result, indent=2), flush=True)
        video.close()
    return 0 if failure is None and result["goal_met"] else 1
