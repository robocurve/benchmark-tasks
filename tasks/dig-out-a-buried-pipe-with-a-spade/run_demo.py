r"""Run the bimanual scripted demo and export its videos.

Example, from the benchmark-tasks repository root:

    python tasks/dig-out-a-buried-pipe-with-a-spade/run_demo.py \
        --seed 0 \
        --max-steps 2500 \
        --out "$HOME/yam-eval/pipe-dig-$(date +%Y%m%d-%H%M%S)"

This records a fixed-layout trial and exports three camera MP4s into the
output directory's videos/ folder. Use a new output directory for each trial.
Check summary.json: evaluation_scores must report task_score == 1.0 and
first_failure must be null before calling the recording a successful demo.
A completed run or exported video alone does not establish task success.

Joint feedback compensates motor compliance using observed joint positions.
It only sends ordinary joint targets: no object attachment, pose writes, extra
forces, changed motor limits, or access to the scorer's private soil state.
"""

import argparse
import hashlib
import json
import os
import platform
import shutil
from datetime import datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import numpy as np
from inspect_robots.cli import main as inspect_main
from inspect_robots.registry import policy
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import STATE_KEY
from retime import retime
from yam_arena.demo import ArmPlan
from yam_arena.scripted import YamScriptedPolicy


@policy("bimanual_dig_demo")
class DigDemo(YamScriptedPolicy):
    def reset(self, scene):
        super().reset(scene)
        waypoints = retime(self._plans, self._dt)
        self._plans = {
            arm: ArmPlan(w, self._bases[arm]) for arm, w in waypoints.items()
        }
        self._end = max(p.duration for p in self._plans.values())
        self.integral = np.zeros(14)
        self.previous = None

    def act(self, observation):
        chunk = super().act(observation)
        nominal = np.asarray(chunk.actions[0].data)
        actual = np.asarray(observation.state[STATE_KEY])
        error = nominal - actual
        error[[6, 13]] = 0.0
        self.integral = np.clip(self.integral + 1.5 * self._dt * error, -0.20, 0.20)
        target = nominal + 0.3 * error + self.integral
        previous = actual if self.previous is None else self.previous
        delta = target - previous
        # Scale the whole bimanual command together instead of independently
        # clipping joints, which can make the hands pull against each other.
        target = previous + delta * min(
            1.0, 0.045 / max(float(np.abs(delta).max()), 1e-12)
        )
        self.previous = target.copy()
        return ActionChunk(
            actions=[Action(data=target, meta=chunk.actions[0].meta)],
            control_hz=1 / self._dt,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=Path.home()
        / "yam-eval"
        / datetime.now().strftime("pipe-dig-%Y%m%d-%H%M%S"),
    )
    parser.add_argument("--max-steps", type=int, default=2500)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out / "source-sha256.json").exists():
        parser.error("Use a new output directory to preserve the previous trial.")
    source = out / "source"
    source.mkdir(exist_ok=True)
    hashes = {}
    for file in Path(__file__).parent.iterdir():
        if file.suffix in (".py", ".npz", ".md", ".json") and file.is_file():
            shutil.copy2(file, source / file.name)
            hashes[file.name] = hashlib.sha256(file.read_bytes()).hexdigest()
    (out / "source-sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
    versions = {"python": platform.python_version()}
    for package in (
        "inspect-robots",
        "inspect-robots-yam",
        "isaacsim",
        "isaaclab",
        "isaaclab-arena",
        "yam-arena",
        "numpy",
        "torch",
    ):
        try:
            versions[package] = version(package)
        except PackageNotFoundError:
            versions[package] = "not installed as a distribution"
    (out / "environment.json").write_text(json.dumps(versions, indent=2) + "\n")
    instruction = json.loads(
        Path(__file__).with_name("task.json").read_text()
    )["instruction"]
    os.environ["DIG_TRACE"] = str(out / "trace.jsonl")
    os.environ["SOIL_SAVE"] = str(out / "latest-soil.npz")
    result = inspect_main(
        [
            "run",
            "--embodiment",
            "yam_arena",
            "--policy",
            "bimanual_dig_demo",
            "-E",
            f"task={Path(__file__).with_name('task.py')}",
            "-E",
            "object_jitter=0",
            "-E",
            "control_hz=20",
            "-E",
            "cam_width=640",
            "-E",
            "cam_height=360",
            "--instruction",
            instruction,
            "--scorer",
            "task_score",
            "--max-steps",
            str(args.max_steps),
            "--seed",
            str(args.seed),
            "--no-prompt",
            "--no-rerun",
            "--store-frames",
            "--log-dir",
            str(out),
        ]
    )
    logs = list(out.glob("adhoc_*.json"))
    if not args.no_video:
        for log in logs:
            inspect_main(["video", str(log), "--out", str(out / "videos")])
    for log in logs:
        data = json.loads(log.read_text())
        if data.get("error"):
            raise RuntimeError(data["error"])
    scores = {
        log.name: json.loads(log.read_text()).get("results", {}).get("metrics", {})
        for log in logs
    }
    trace = out / "trace.jsonl"
    if trace.exists():
        rows = [json.loads(line) for line in trace.read_text().splitlines()]
        if rows:
            summary = {
                "final": rows[-1],
                "seed": args.seed,
                "evaluation_scores": scores,
                "max_score": max(r["metrics"]["score"] for r in rows),
                "first_failure": next(
                    (r["step"] for r in rows if r["metrics"]["failure"]), None
                ),
            }
            (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Run and videos: {out}")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
