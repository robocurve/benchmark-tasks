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
from benchmark import BottleWarmingEmbodiment


class HoldReadyPolicy(PolicyBase):
    def __init__(self, info):
        self.info = PolicyInfo(name="hold_ready", action_space=info.action_space,
                               observation_space=info.observation_space, control_hz=20)

    def act(self, observation):
        return ActionChunk([Action(np.array(observation.state["joint_pos"], copy=True))], control_hz=20)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=("hold_ready", "dream"), default="hold_ready")
    parser.add_argument("--max-steps", type=int)
    parser.add_argument("--epochs", type=int, choices=[1], default=1, help="Run each trial in a fresh process to clear PhysX contact history")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.policy == "dream" and not (
        os.environ.get("DREAM_API_KEY") or (Path.home()/".dreamlabs/credentials").is_file()
    ):
        parser.error("Save your Dream API key in ~/.dreamlabs/credentials before selecting --policy dream")
    embodiment = BottleWarmingEmbodiment()
    policy = HoldReadyPolicy(embodiment.info) if args.policy == "hold_ready" else "dream"
    steps = args.max_steps or (5 if args.policy == "hold_ready" else 2400)
    benchmark = Task(name="baby-bottle-warming", scenes=[Scene(id="fixed-layout", instruction=task.INSTRUCTION)],
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
