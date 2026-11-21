"""Execute this task with Inspect Robots, YAM and the task_score grader."""

import argparse
import json
import os
import sys
from dataclasses import dataclass
from math import atan2
from pathlib import Path

import numpy as np
from yam_arena import ik, inspect_embodiment
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import grip_rotation

import task


@dataclass
class Hand:
    position: np.ndarray
    rotation: np.ndarray
    open_fraction: float
    yaw: float
    upright: bool


def bind(path, directory, setup, case):
    cls = inspect_embodiment.YamArenaEmbodiment
    original_reset, original_step = cls.reset, cls._task_step

    def current(instance):
        return instance.task is not None and instance.task.path == path

    def reset(self, *args, **kwargs):
        observation = original_reset(self, *args, **kwargs)
        if not current(self):
            return observation
        self._fixture_state = task.State(case)
        self._fixture_view = task.View(self._fixture_state)
        self._fixture_passes = 0
        self._fixture_trace = (directory / "mechanism.jsonl").open("w", encoding="utf-8")
        for _ in range(8):
            self._base.sim.render()
        for name in ("top_cam", "left_cam", "right_cam"):
            self._base.scene[name].update(self._base.step_dt, force_recompute=True)
        if setup:
            import cv2

            pixels = self._base.scene["top_cam"].data.output["rgb"]
            pixels = pixels.torch if hasattr(pixels, "torch") else pixels
            cv2.imwrite(str(setup), cv2.cvtColor(pixels[0, ..., :3].cpu().numpy(), cv2.COLOR_RGB2BGR))
        return self._observation()

    def step(self):
        if not current(self):
            return original_step(self)
        from inspect_robots.types import StepResult

        hands = {}
        down = grip_rotation()
        for arm in ("left", "right"):
            name = arm + "_robot"
            joint_ids, finger_ids = self._ids[name]
            joints = self._base.scene[name].data.joint_pos.torch[0]
            xyz, rotation = ik.tcp_pose(joints[joint_ids].cpu().numpy(), self.rig.bases[arm])
            spin = rotation @ down.T
            hands[arm] = Hand(
                xyz, spin, float(joints[finger_ids].mean().item() / FINGER_OPEN),
                atan2(spin[1, 0], spin[0, 0]), bool(rotation[:, 2] @ down[:, 2] > 0.96),
            )
        state = self._fixture_state
        info = state.advance(hands, self._base.step_dt)
        self._fixture_view.update(state)
        self._fixture_passes = self._fixture_passes + 1 if info["score"] == 1.0 else 0
        success = self._fixture_passes >= 5
        info["success"] = success
        info["hands"] = {key: [*map(float, hand.position), round(hand.open_fraction, 4),
                                round(hand.yaw, 4), hand.upright] for key, hand in hands.items()}
        self._fixture_trace.write(json.dumps({"t": round(self._t, 4), **info}, allow_nan=False) + "\n")
        self._fixture_trace.flush()
        return StepResult(observation=self._observation(), terminated=success,
                          termination_reason="success" if success else None, info=info)

    cls.reset, cls._task_step = reset, step


def main():
    from inspect_robots.cli import main as inspect_main

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("policy", choices=("yam_scripted", "dream"))
    parser.add_argument("--log-dir", required=True, type=Path)
    parser.add_argument("--setup", type=Path)
    parser.add_argument("--case", default="nominal", choices=task.CASES)
    args, rest = parser.parse_known_args()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    path = Path(task.__file__).resolve()
    os.environ["ROBOCURVE_FIXTURE_CASE"] = args.case
    task.configure_demo(args.case)
    bind(path, args.log_dir, args.setup, args.case)
    sys.argv[1:] = [
        "run", "--embodiment", "yam_arena", "--policy", args.policy,
        "-E", f"task={path}", "-E", "object_jitter=0", "-E", "control_hz=20",
        "-E", "cam_width=320", "-E", "cam_height=176",
        "--instruction", task.INSTRUCTION, "--scorer", "task_score",
        "--max-steps", str(task.MAX_STEPS), "--store-frames", "--log-dir", str(args.log_dir), *rest,
    ]
    inspect_main()


if __name__ == "__main__":
    main()
