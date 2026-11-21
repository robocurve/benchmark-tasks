"""Run Dream against the same calibrated bottle physics and scoring."""

import argparse
import json
from pathlib import Path

from dream_episode import run_episode
from run import ACTION_LIMIT, Trial
from scene import CAMERAS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--dry-run", action="store_true", help="Exercise the policy interface offline")
    args = parser.parse_args()
    task = json.loads(Path(__file__).with_name("task.json").read_text())
    trial = Trial()
    trial.phase = "policy"

    def action_step(action):
        trial.step(action.reshape(2, 7))

    result = run_episode(
        trial.m, trial.d, action_step, trial.report, instruction=task["instruction"],
        task_id="push-button-removable-straw", action_limit=8 if args.dry_run else ACTION_LIMIT,
        out=args.out, cameras=CAMERAS, dry_run=args.dry_run,
    )
    trial.save_trace(args.out / "trace.jsonl")
    print(json.dumps(result, indent=2))
    if result["status"] != "complete":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
