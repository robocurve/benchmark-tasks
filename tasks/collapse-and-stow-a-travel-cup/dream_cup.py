"""Run the Dream policy against the same fresh scene and scorer used by the demo."""

import argparse
import json
from pathlib import Path

from dream_episode import run_episode

from run import ACTION_LIMIT, OPEN, RATE, SIDE_SLICE, Trial
from scene import CAMERAS
from scoring import grade


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--dry-run", action="store_true", help="Check camera, action and scoring paths offline")
    args = parser.parse_args()
    task = json.loads(Path(__file__).with_name("task.json").read_text())
    trial = Trial()

    def step(action):
        action[[6, 13]] *= OPEN
        trial.commands = {side: action[cols].copy() for side, cols in SIDE_SLICE.items()}
        trial._tick("policy")

    def score():
        result = grade(trial.trace)
        return {"score": result["task_score"], "grade": result, "steps": len(trial.actions),
                "time_s": len(trial.actions)/RATE, "actuator_efforts": trial.effort_summary()}

    result = run_episode(trial.model, trial.data, step, score, instruction=task["instruction"],
                         task_id="collapse-and-stow-a-travel-cup", action_limit=8 if args.dry_run else ACTION_LIMIT,
                         out=args.out, cameras=CAMERAS, dry_run=args.dry_run)
    with (args.out / "trace.jsonl").open("w") as trace:
        for row in trial.trace:
            trace.write(json.dumps(row) + "\n")
    print(json.dumps(result, indent=2))
    if result["status"] != "complete":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
