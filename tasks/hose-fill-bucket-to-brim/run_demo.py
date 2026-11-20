r"""Run one trial of the hose task (the scripted demo by default) and export its videos.

From the benchmark-tasks repository root:

    python tasks/hose-fill-bucket-to-brim/run_demo.py --out "$HOME/yam-eval/hose-$(date +%Y%m%d-%H%M%S)"

It records the three cameras into ``<out>/videos/``, the water's state every 0.1 s into ``<out>/trace.jsonl`` and the
outcome into ``<out>/summary.json``. A finished run or a video alone does not show success: ``task_score`` in
``summary.json`` must be 1.0. ``--policy dream`` (or any other Inspect Robots policy) runs that policy instead.
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

from inspect_robots.cli import main as inspect_main

HERE = Path(__file__).resolve().parent
INSTRUCTION = json.loads((HERE / "task.json").read_text())["instruction"] if (HERE / "task.json").exists() else (
    "Fill the bucket to the brim with the hose, then stop the water.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path,
                        default=Path.home() / "yam-eval" / datetime.now().strftime("hose-%Y%m%d-%H%M%S"))
    parser.add_argument("--policy", default="yam_scripted")
    parser.add_argument("--max-steps", type=int, default=1600)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--cam-height", type=int, default=360)
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out / "source-sha256.json").exists():
        parser.error("use a new output directory for each trial")
    (out / "source").mkdir(exist_ok=True)
    hashes = {}
    for f in sorted(HERE.iterdir()):
        if f.suffix in (".py", ".md", ".json") and f.is_file():
            shutil.copy2(f, out / "source" / f.name)
            hashes[f.name] = hashlib.sha256(f.read_bytes()).hexdigest()
    (out / "source-sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
    versions = {"python": platform.python_version()}
    for pkg in ("inspect-robots", "inspect-robots-yam", "isaacsim", "isaaclab", "isaaclab-arena", "yam-arena",
                "numpy", "torch"):
        try:
            versions[pkg] = version(pkg)
        except PackageNotFoundError:
            versions[pkg] = "not installed as a distribution"
    (out / "environment.json").write_text(json.dumps(versions, indent=2) + "\n")
    os.environ["HOSE_TRACE"] = str(out / "trace.jsonl")
    inspect_main([
        "run", "--embodiment", "yam_arena", "--policy", args.policy,
        "-E", f"task={HERE / 'task.py'}", "-E", "object_jitter=0", "-E", "control_hz=20",
        "-E", f"cam_width={args.cam_width}", "-E", f"cam_height={args.cam_height}",
        "--instruction", INSTRUCTION, "--scorer", "task_score", "--max-steps", str(args.max_steps),
        "--seed", str(args.seed), "--no-prompt", "--no-rerun", "--store-frames", "--log-dir", str(out),
    ])
    logs = sorted(out.glob("adhoc_*.json"))
    if not args.no_video:
        for log in logs:
            inspect_main(["video", str(log), "--out", str(out / "videos")])
    metrics = {}
    for log in logs:
        data = json.loads(log.read_text())
        if data.get("error"):
            print("run error:", data["error"])
        metrics[log.name] = data.get("results", {}).get("metrics", {})
    rows = [json.loads(line) for line in (out / "trace.jsonl").read_text().splitlines()] if (
        out / "trace.jsonl").exists() else []
    summary = {"seed": args.seed, "policy": args.policy, "evaluation": metrics,
               "final": rows[-1] if rows else None, "max_score": max((r["score"] for r in rows), default=None)}
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    print(f"run and videos: {out}")


if __name__ == "__main__":
    main()
