#!/usr/bin/env python3
"""Run one trial of this task: the scripted demonstration (yam_scripted) or Dream-1, scored by task_score."""

import argparse
from datetime import datetime
from hashlib import sha256
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from inspect_robots.cli import main  # noqa: E402
from inspect_robots.registry import embodiment  # noqa: E402

from runtime import CoatRackEmbodiment  # noqa: E402


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["demo", "dream"])
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--headless", choices=["true", "false"], default="true")
    parser.add_argument("--max-steps", type=int, default=3200)
    args = parser.parse_args()
    root = Path(os.environ.get("YAM_LOG_ROOT", str(Path.home() / "yam-eval")))
    output = args.log_dir or root / "coat-rack" / f"{args.mode}-{datetime.now():%Y%m%d-%H%M%S}"
    output.mkdir(parents=True, exist_ok=True)
    manifest = {p.name: sha256(p.read_bytes()).hexdigest() for p in sorted(HERE.glob("*.py"))}
    (output / "source_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    @embodiment("coat_rack")
    def make(**kwargs):
        instance = CoatRackEmbodiment(**kwargs)
        instance.diagnostics_path = output / "measurements.jsonl"
        return instance

    instruction = json.loads((HERE / "task.json").read_text())["instruction"]
    argv = [
        "run", "--embodiment", "coat_rack",
        "--policy", "yam_scripted" if args.mode == "demo" else "dream",
        "-E", f"task={HERE / 'task.py'}", "-E", "object_jitter=0", "-E", "control_hz=20",
        "-E", "cam_width=320", "-E", "cam_height=176", "-E", f"headless={args.headless}",
        "--instruction", instruction, "--scorer", "task_score", "--max-steps", str(args.max_steps),
        "--seed", "0", "--no-prompt", "--store-frames", "--no-rerun", "--no-rerun-save",
        "--log-dir", str(output),
    ]  # fmt: skip
    return main(argv)


if __name__ == "__main__":
    sys.exit(run())
