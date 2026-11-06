"""Inspect Robots evaluation of the local task. Dream is opt-in and needs a key.

No-API check:
  /isaac-sim/python.sh evaluate.py --policy hold_ready --max-steps 5 --out /outputs/eval-check
After API-key setup, explicitly select --policy dream for model evaluation.
"""

import argparse
import json
from pathlib import Path
import os

import numpy as np
from inspect_robots import Action, ActionChunk, PolicyBase, PolicyInfo, Scene, Task, eval
from yam_arena.scoring import task_score
from yam_arena.constants import READY

import task
from benchmark import CleanSpoonEmbodiment


class HoldReadyPolicy(PolicyBase):
    def __init__(self, info):
        self.info = PolicyInfo(name="hold_ready", action_space=info.action_space,
                               observation_space=info.observation_space, control_hz=20)

    def act(self, observation):
        return ActionChunk([Action(np.r_[READY,1.0,READY,1.0])], control_hz=20)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=("hold_ready", "dream"), default="hold_ready")
    parser.add_argument("--max-steps", type=int)
    parser.add_argument("--epochs", type=int, default=1)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.policy == "dream" and not (
        os.environ.get("DREAM_API_KEY") or (Path.home()/".dreamlabs/credentials").is_file()
    ):
        parser.error("Save your Dream API key in ~/.dreamlabs/credentials before selecting --policy dream")
    embodiment = CleanSpoonEmbodiment()
    policy = HoldReadyPolicy(embodiment.info) if args.policy == "hold_ready" else "dream"
    steps = args.max_steps or (5 if args.policy == "hold_ready" else 2400)
    benchmark = Task(name="clean-spoon-meatball", scenes=[Scene(id="fixed-layout", instruction=task.INSTRUCTION)],
                     scorer=task_score(), max_steps=steps, epochs=args.epochs,
                     metadata={"fixed_layout": True, "object_jitter_m": 0.0})
    try:
        logs = eval(benchmark, policy, embodiment, log_dir=str(args.out), store_frames=True, seed=0)
        for log in logs:
            print("EVALUATION_RESULT", log.status, log.results, flush=True)
        if any(log.status != "success" for log in logs):
            raise RuntimeError("Evaluation did not complete; inspect the saved log before reporting a score")
        if args.policy == 'hold_ready':
            audit = embodiment.audit
            assert audit['resets'] == args.epochs
            assert audit['steps'] == steps * args.epochs
            assert audit['last_score']['score'] == 0.0
            assert audit['last_score']['contact_reports_healthy']
            args.out.mkdir(parents=True, exist_ok=True)
            (args.out/'evaluation_check.json').write_text(json.dumps({
                'passed':True, 'policy':'hold_ready', 'dream_api_calls':0,
                'environment_revision':embodiment.info.environment_revision,
                **audit},indent=2)+'\n')
    finally:
        embodiment.close()
        embodiment.shutdown()


if __name__ == "__main__":
    main()
