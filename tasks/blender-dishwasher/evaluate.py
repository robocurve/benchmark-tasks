"""Inspect Robots evaluation of the local task. Dream is opt-in and needs a key.

No-API check:
  /isaac-sim/python.sh evaluate.py --policy hold_ready --max-steps 5 --out /outputs/eval-check
After API-key setup, explicitly select --policy dream for model evaluation.
"""

import argparse
from pathlib import Path
import os

import numpy as np
from inspect_robots import Action, ActionChunk, PolicyBase, PolicyInfo, Scene, Task, eval
from yam_arena.scoring import task_score

import task
from benchmark import BlenderEmbodiment


class HoldReadyPolicy(PolicyBase):
    def __init__(self, info):
        self.info = PolicyInfo(name="hold_ready", action_space=info.action_space,
                               observation_space=info.observation_space, control_hz=20)

    def act(self, observation):
        return ActionChunk([Action(np.array(observation.state["joint_pos"], copy=True))], control_hz=20)


class JointReplayPolicy(PolicyBase):
    """Replay an exported synthetic trajectory; no inference calls."""
    def __init__(self, info, path):
        self.info = PolicyInfo(name="joint_trajectory_replay", action_space=info.action_space,
                               observation_space=info.observation_space, control_hz=20)
        self.actions = np.load(path)["actions"]
        if (self.actions.ndim != 2 or self.actions.shape[1] != 14 or not len(self.actions)
                or len(self.actions)>2380 or not np.isfinite(self.actions).all()):
            raise ValueError("Replay requires a finite 14-D trajectory within the run limit")
        self.cursor = 0

    def act(self, observation):
        rows = self.actions[self.cursor:self.cursor+48]
        self.cursor += len(rows)
        if not len(rows):
            rows = self.actions[-1:]
        return ActionChunk([Action(row.copy()) for row in rows], control_hz=20)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=("hold_ready", "dream", "replay"), default="hold_ready")
    parser.add_argument("--replay-file", type=Path)
    parser.add_argument("--max-steps", type=int)
    parser.add_argument("--epochs", type=int, choices=[1], default=1, help="Run each trial in a fresh process to clear PhysX contact history")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.policy=="replay" and (args.replay_file is None or not args.replay_file.is_file()):
        parser.error("--policy replay requires --replay-file")
    args.out.mkdir(parents=True, exist_ok=True)
    os.environ["YAM_STATE_LOG"] = str(args.out/"measured_trajectory.jsonl")
    if args.policy == "dream" and not (
        os.environ.get("DREAM_API_KEY") or (Path.home()/".dreamlabs/credentials").is_file()
    ):
        parser.error("Save your Dream API key in ~/.dreamlabs/credentials before selecting --policy dream")
    if args.policy=='replay':os.environ['YAM_PHYSICS_SUBSTEP_LOG']=str(args.out/'physics_substeps.jsonl')
    embodiment = BlenderEmbodiment()
    policy = (HoldReadyPolicy(embodiment.info) if args.policy == "hold_ready" else
              JointReplayPolicy(embodiment.info,args.replay_file) if args.policy=="replay" else "dream")
    steps = args.max_steps or (5 if args.policy == "hold_ready" else 2400)
    benchmark = Task(name="blender-dishwasher", scenes=[Scene(id="fixed-layout", instruction=task.INSTRUCTION)],
                     scorer=task_score(), max_steps=steps, epochs=args.epochs,
                     metadata={"fixed_layout": True, "object_jitter_m": 0.0})
    exit_code = 1
    try:
        logs = eval(benchmark, policy, embodiment, log_dir=str(args.out), store_frames=True, seed=0)
        for log in logs:
            print("EVALUATION_RESULT", log.status, log.results, flush=True)
        if any(log.status != "success" for log in logs):
            raise RuntimeError("Evaluation did not complete; inspect the saved log before reporting a score")
        exit_code = 0
    finally:
        embodiment.shutdown(exit_code=exit_code)


if __name__ == "__main__":
    main()
