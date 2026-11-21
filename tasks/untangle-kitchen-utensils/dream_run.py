"""Run one scored Dream-1 trial with the stock Inspect Robots evaluator and policy."""

import argparse
from pathlib import Path

from task import INSTRUCTION, LIMIT, KitchenUtensils
from inspect_robots.cli import main
from inspect_robots.registry import embodiment

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    embodiment("utensils_yam")(KitchenUtensils)
    raise SystemExit(
        main(
            [
                "run",
                "--embodiment",
                "utensils_yam",
                "--policy",
                "dream",
                "--instruction",
                INSTRUCTION,
                "--scorer",
                "task_score",
                "--max-steps",
                str(LIMIT),
                "--seed",
                "0",
                "--store-frames",
                "--no-prompt",
                "--log-dir",
                str(args.out),
            ]
        )
    )
