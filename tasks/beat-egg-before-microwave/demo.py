"""Scripted Cartesian motion for the native YAM embodiment."""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import grip_rotation
from yam_arena.station import BASES
from yam_arena.urdf import axis_angle_matrix

TASK_PATH = Path(__file__).with_name("task.py").resolve()
HZ = 20


@dataclass(frozen=True)
class Move:
    seconds: float
    xyz: tuple
    rotation: np.ndarray
    grip: float
    name: str


def _tilted(degrees):
    return axis_angle_matrix(np.array((0.0, 1.0, 0.0)), -math.radians(degrees)) @ grip_rotation()


def plans(task, variant):
    if variant == "hold":
        return {"left": [], "right": []}

    top = task.TOP
    bx, by, _ = task.BOWL_START
    fx, fy, _ = task.FORK_START
    left, right = [], []
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    down = grip_rotation()
    stir = axis_angle_matrix(np.array((0.0, 0.0, 1.0)), math.radians(-32)) @ _tilted(54)
    brace = axis_angle_matrix(np.array((1.0, 0.0, 0.0)), math.radians(-38)) @ down
    door = _tilted(35)
    transfer = _tilted(40)
    button = _tilted(72)
    parked_left = (0.235, 0.245, top + 0.152)
    parked_right = (0.225, -0.365, top + 0.145)
    door_angle = -math.radians(84)
    fork_pick_z = top + 0.013
    fork_offset = stir @ down.T @ (np.array(task.FORK_TIP) + (0, 0, task.FORK_START[2] - fork_pick_z))

    def move(arm, seconds, xyz, grip, name, rotation=down):
        arm.append(Move(round(seconds * HZ) / HZ, tuple(xyz), rotation, grip, name))

    def sync():
        lengths = [sum(move.seconds for move in arm) for arm in (left, right)]
        if abs(lengths[0] - lengths[1]) < 1e-6:
            return
        arm = left if lengths[0] < lengths[1] else right
        fallback = parked_left if arm is left else parked_right
        last = arm[-1] if arm else Move(0.0, fallback, down, opened, "park")
        move(arm, abs(lengths[0] - lengths[1]), last.xyz, last.grip, "hold", last.rotation)

    def tip(x, y, z, duration, name):
        move(right, duration, np.array((bx + x, by + y, top + z)) - fork_offset, closed, name, stir)

    move(left, 2.0, parked_left, opened, "left park")
    move(right, 2.0, parked_right, opened, "right park")

    left_rim = (bx, by + task.BOWL_RADIUS - 0.002, top + task.BOWL_HEIGHT + 0.007)
    move(left, 1.0, (left_rim[0], left_rim[1], top + 0.125), opened, "above bowl rim", brace)
    move(left, 1.0, left_rim, opened, "approach bowl rim", brace)
    move(left, 0.75, left_rim, closed, "steady bowl", brace)
    move(right, 1.1, (fx, fy, top + 0.10), opened, "above fork")
    move(right, 1.1, (fx, fy, fork_pick_z), opened, "approach fork handle")
    move(right, 0.75, (fx, fy, fork_pick_z), closed, "grasp fork")
    move(right, 1.25, (fx, fy, top + 0.130), closed, "lift fork")
    tip(0.0, 0.0, 0.049, 1.5, "angle fork above yolk")
    tip(0.0, 0.0, 0.009, 1.0, "break yolk")
    if variant != "puncture-only":
        tip(0.0, -0.032, 0.010, 0.8, "sweep into white")
        turns = {"short-mix": 6, "mix-only": 7}.get(variant, 10)
        for index in range(1, 12 * turns + 1):
            angle = 2 * math.pi * index / 12 - math.pi / 2
            tip(0.032 * math.cos(angle), 0.032 * math.sin(angle), 0.007, 0.23, "beat egg")
    tip(0.0, -0.020, 0.070, 1.1, "withdraw fork")
    move(right, 1.3, (fx, fy, top + 0.166), closed, "move fork away from bowl", stir)
    move(right, 0.9, (fx, fy, top + 0.166), closed, "level fork above counter")
    move(right, 1.2, (fx, fy, top + 0.014), closed, "set fork down")
    move(right, 0.7, (fx, fy, top + 0.014), opened, "release fork")
    move(right, 0.9, (fx, fy, top + 0.105), opened, "clear the fork")
    move(right, 0.9, parked_right, opened, "right park")
    sync()
    move(left, 0.6, left_rim, opened, "release bowl", brace)
    move(left, 1.0, parked_left, opened, "clear the bowl")
    sync()

    if variant in ("mix-only", "short-mix", "puncture-only"):
        return {"left": left, "right": right}

    handle = task.door_handle(0.0)
    move(left, 1.3, handle + (-0.045, 0.0, 0.0), opened, "in front of door handle", door)
    move(left, 1.0, handle, opened, "straddle door handle", door)
    move(left, 0.7, handle, closed, "grasp door handle", door)
    for angle in np.linspace(0.0, door_angle, 10)[1:]:
        move(left, 0.45, task.door_handle(angle), closed, "open door", door)
    handle = task.door_handle(door_angle)
    move(left, 0.6, handle, opened, "release opened door", door)
    move(left, 1.0, handle + (0.0, 0.09, 0.026), opened, "left clear of cavity", door)
    sync()

    rim_y = by - task.BOWL_RADIUS + 0.003
    rim_z = top + task.BOWL_HEIGHT + 0.005
    target_x = task.MICROWAVE_ORIGIN[0] + task.PLATE_CENTER[0] - 0.018
    target_y = task.MICROWAVE_ORIGIN[1] + task.PLATE_CENTER[1] - (task.BOWL_RADIUS - 0.003)
    target_z = rim_z + task.PLATE_TOP
    carry_z = target_z + 0.036
    move(right, 1.3, (bx - 0.050, rim_y, rim_z + 0.070), opened, "above bowl", transfer)
    move(right, 1.1, (bx, rim_y, rim_z), opened, "straddle bowl rim", transfer)
    move(right, 0.7, (bx, rim_y, rim_z), closed, "grip bowl rim", transfer)
    move(right, 1.0, (bx, rim_y, carry_z), closed, "lift bowl level", transfer)
    move(right, 1.8, (bx, target_y, carry_z), closed, "align bowl in front of cavity", transfer)
    move(right, 1.8, (target_x, target_y, carry_z), closed, "carry through open door", transfer)
    move(right, 0.8, (target_x, target_y, target_z), closed, "set bowl on plate", transfer)
    move(right, 0.7, (target_x, target_y, target_z), opened, "release bowl", transfer)
    move(right, 1.5, (bx - 0.03, rim_y, target_z + 0.009), opened, "withdraw from cavity", transfer)
    plate_y = task.MICROWAVE_ORIGIN[1] + task.PLATE_CENTER[1]
    push_z = top + task.PLATE_TOP + 0.020
    move(right, 1.5, (0.329, plate_y, push_z + 0.025), closed, "approach front of bowl", button)
    move(right, 0.8, (0.343, plate_y, push_z), closed, "align with bowl wall", button)
    move(right, 1.5, (0.438, plate_y, push_z), closed, "slide bowl fully onto plate", button)
    move(right, 1.0, (0.329, plate_y, push_z), closed, "withdraw after seating bowl", button)
    move(right, 1.2, parked_right, opened, "right clear of door")
    sync()

    move(left, 1.0, handle, opened, "return to handle", door)
    move(left, 0.6, handle, closed, "grasp door again", door)
    for angle in np.linspace(door_angle, 0.0, 10)[1:]:
        move(left, 0.45, task.door_handle(angle), closed, "close door", door)
    handle = task.door_handle(0.0)
    move(left, 0.7, handle, opened, "release closed door", door)
    move(left, 0.8, handle + (-0.065, 0.0, 0.0), opened, "back straight away from the handle", door)
    move(left, 1.2, parked_left, opened, "clear the controls")
    sync()

    sx, sy, sz = np.add(task.MICROWAVE_ORIGIN, task.BUTTON)
    move(right, 1.4, (sx - 0.047, sy, sz), closed, "align with Start", button)
    move(right, 1.0, (sx - 0.010, sy, sz), closed, "press Start", button)
    move(right, 0.6, (sx - 0.010, sy, sz), closed, "hold Start", button)
    move(right, 1.0, (sx - 0.047, sy, sz), closed, "release Start", button)
    return {"left": left, "right": right}


class ArmMotion:
    def __init__(self, side, moves):
        self.moves = iter(moves)
        self.base = BASES[side]
        self.q = np.array(READY)
        self.xyz, self.rotation = ik.tcp_pose(self.q, self.base)
        self.grip = FINGER_OPEN
        self.elapsed = 0.0
        self.move = next(self.moves, None)
        self.position_error = (0.0, "")
        self.angle_error = (0.0, "")

    def command(self, dt):
        if self.move is None:
            return np.r_[self.q, self.grip / FINGER_OPEN]
        target = self.move
        self.elapsed += dt
        u = ik.smoothstep(min(1.0, self.elapsed / target.seconds))
        xyz = self.xyz + u * (np.asarray(target.xyz) - self.xyz)
        rotation = ik.slerp_matrix(self.rotation, target.rotation, u)
        grip = self.grip + u * (target.grip - self.grip)
        self.q, pe, re = ik.solve(xyz, ik.matrix_to_quat_xyzw(rotation), self.q, base_pos=self.base, iters=60)
        self.position_error = max(self.position_error, (float(pe), target.name))
        self.angle_error = max(self.angle_error, (math.degrees(re), target.name))
        if self.elapsed + 1e-9 >= target.seconds:
            self.xyz, self.rotation, self.grip = np.asarray(target.xyz), target.rotation, target.grip
            self.elapsed = 0.0
            self.move = next(self.moves, None)
        return np.r_[self.q, np.clip(grip / FINGER_OPEN, 0.0, 1.0)]


def _motion(task, variant):
    return [ArmMotion(side, moves) for side, moves in plans(task, variant).items()]


def _kinematics(variant):
    from yam_arena.task_file import load

    motion = _motion(load(TASK_PATH).module, variant)
    while any(arm.move is not None for arm in motion):
        for arm in motion:
            arm.command(1 / HZ)
    for name, arm in zip(("left", "right"), motion):
        err, position_phase = arm.position_error
        angle, angle_phase = arm.angle_error
        print(f"{name}: max position error {err * 1000:.1f} mm ({position_phase}); "
              f"max orientation error {angle:.1f} degrees ({angle_phase})")
    return all(arm.position_error[0] <= 0.010 and arm.angle_error[0] <= 3.0 for arm in motion)


def save_mixing_reference(close_up, concentrations, output):
    """Pair the camera image with a fixed-scale view of the simulated pigment."""
    from PIL import Image
    from egg_state import CELL, CELLS

    indices = np.rint(CELLS / CELL).astype(int)
    extent = int(indices.max())
    pixels = np.full((2 * extent + 1, 2 * extent + 1, 3), 236, dtype=np.uint8)
    fraction = np.clip(concentrations / 0.35, 0, 1)[:, None]
    purple, yellow = np.array([57, 18, 108]), np.array([255, 218, 62])
    pixels[extent - indices[:, 1], indices[:, 0] + extent] = purple + fraction * (yellow - purple)
    diagram = Image.fromarray(pixels).resize((close_up.height, close_up.height), Image.Resampling.NEAREST)
    pair = Image.new("RGB", (close_up.width + diagram.width, close_up.height))
    pair.paste(close_up)
    pair.paste(diagram, (close_up.width, 0))
    pair.save(output)


def run(args):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    env = YamArenaEmbodiment(task=str(TASK_PATH), object_jitter=0, control_hz=HZ,
                             cam_width=1280, cam_height=720, **({"device": args.device} if args.device else {}))
    task = env.task.module
    motion = _motion(task, args.variant)
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    writer = None
    try:
        obs = env.reset(Scene(id="beat-egg-before-microwave", instruction=task.INSTRUCTION), seed=0)
        if not args.no_video:
            from imageio_ffmpeg import write_frames
            from PIL import Image

            def bowl_view(top):
                # Fixed crop of the preparation area in the overhead camera.
                return top.crop((660, 332, 900, 467)).resize((640, 360), Image.Resampling.LANCZOS)

            def frame(observation):
                top = Image.fromarray(observation.images["top_cam"])
                wrist = [Image.fromarray(pixels) for name, pixels in observation.images.items() if name != "top_cam"]
                panels = (top.resize((640, 360)), bowl_view(top), *(view.resize((640, 360)) for view in wrist))
                return np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))

            Image.fromarray(obs.images["top_cam"]).save(output / "setup.png")
            height, width = frame(obs).shape[:2]
            writer = write_frames(str(output / "demo.mp4"), (width, height), fps=HZ, codec="libx264",
                                  macro_block_size=2, pix_fmt_out="yuv420p", output_params=["-movflags", "+faststart"])
            writer.send(None)
            writer.send(np.ascontiguousarray(frame(obs)))
        with (output / "trace.jsonl").open("w", encoding="utf-8") as trace:
            hold = 0
            previous_left_phase = None
            for step in range(1, args.max_steps + 1):
                phases = [arm.move.name if arm.move else "done" for arm in motion]
                commands = [arm.command(1 / HZ) for arm in motion]
                result = env.step(Action(data=pack(*commands)))
                status = task.progress({})
                row = {"step": step, "score": result.info.get("score", 0.0), "phases": phases, **status}
                trace.write(json.dumps(row) + "\n")
                if writer:
                    writer.send(np.ascontiguousarray(frame(result.observation)))
                    if previous_left_phase == "clear the bowl" and phases[0] != previous_left_phase:
                        top = Image.fromarray(result.observation.images["top_cam"])
                        close_up = bowl_view(top)
                        close_up.save(output / "prepared-egg.png")
                        save_mixing_reference(close_up, task._trial.egg.yolk, output / "mixing-reference.png")
                    previous_left_phase = phases[0]
                if result.terminated:
                    break
                if all(arm.move is None for arm in motion):
                    hold += 1
                    if hold >= 30:
                        break
        (output / "result.json").write_text(json.dumps(row, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(row))
        expected = {"full": 1.0, "hold": 0.0, "puncture-only": 0.0, "short-mix": 0.0, "mix-only": 0.5}[args.variant]
        if row["score"] != expected:
            return False
        if args.variant in ("puncture-only", "short-mix"):
            return row["membrane_open"] and not row["mixed"] and row["fork_outside"] and not row["unsafe_or_spilled"]
        return True
    finally:
        if writer:
            writer.close()
        env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=("full", "hold", "puncture-only", "short-mix", "mix-only"), default="full")
    parser.add_argument("--output", type=Path, default=Path("/tmp/beat-egg-demo"))
    parser.add_argument("--max-steps", type=int, default=1800)
    parser.add_argument("--device", help="Isaac Lab device, for example cpu or cuda:0")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--kinematics", action="store_true", help="check geometric IK without running the simulator")
    args = parser.parse_args()
    if args.max_steps < 1:
        parser.error("--max-steps must be positive")
    return 0 if (_kinematics(args.variant) if args.kinematics else run(args)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
