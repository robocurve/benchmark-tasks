"""Run one trial of the pulley heavy-load hoisting task and export its videos and trace."""

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
from datetime import datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from inspect_robots.cli import main as inspect_main

HERE = Path(__file__).resolve().parent
DEFAULT_INSTRUCTION = (
    "Hoist the heavy weight above the building's roof level by pulling the rope over the pulley with both arms "
    "together, switching hands along the rope as needed."
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=Path.home() / "yam-eval" / datetime.now().strftime("pulley-%Y%m%d-%H%M%S"),
    )
    parser.add_argument("--policy", default="yam_scripted")
    parser.add_argument("--max-steps", type=int, default=1200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--cam-height", type=int, default=360)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--no-review-cams", action="store_true", help="skip the extra side and wide review videos")
    parser.add_argument("--no-frames", action="store_true", help="don't store camera frames (fast development runs)")
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
            versions[pkg] = "not installed"
    (out / "environment.json").write_text(json.dumps(versions, indent=2) + "\n")

    instruction = (
        json.loads((HERE / "task.json").read_text())["instruction"]
        if (HERE / "task.json").exists()
        else DEFAULT_INSTRUCTION
    )
    os.environ["HOIST_TRACE"] = str(out / "trace.jsonl")
    if not args.no_review_cams:
        os.environ["HOIST_REVIEW_CAMS"] = str(out / "videos" / "review")
    inspect_main([
        "run", "--embodiment", "yam_arena", "--policy", args.policy,
        "-E", f"task={HERE / 'task.py'}", "-E", "object_jitter=0", "-E", "control_hz=20",
        "-E", f"cam_width={args.cam_width}", "-E", f"cam_height={args.cam_height}",
        "--instruction", instruction, "--scorer", "task_score", "--max-steps", str(args.max_steps),
        "--seed", str(args.seed), "--no-prompt", "--no-rerun", *([] if args.no_frames else ["--store-frames"]), "--log-dir", str(out),
    ])

    logs = sorted(out.glob("adhoc_*.json"))
    if not (args.no_video or args.no_frames):
        for log in logs:
            inspect_main(["video", str(log), "--out", str(out / "videos")])

    review = sys.modules.get("review_cams")  # the task file imported it; finish its videos
    if review is not None:
        review.close()
    for mp4 in sorted((out / "videos" / "review").glob("*.mp4")):
        fixed = mp4.with_suffix(".tmp.mp4")
        subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", str(mp4), "-c", "copy", "-movflags", "+faststart",
                        str(fixed)], check=True)
        fixed.replace(mp4)

    metrics = {}
    for log in logs:
        data = json.loads(log.read_text())
        if data.get("error"):
            print("run error:", data["error"])
        metrics[log.name] = data.get("results", {}).get("metrics", {})
    rows = (
        [json.loads(line) for line in (out / "trace.jsonl").read_text().splitlines()]
        if (out / "trace.jsonl").exists()
        else []
    )
    summary = {
        "seed": args.seed,
        "policy": args.policy,
        "evaluation": metrics,
        "final": rows[-1] if rows else None,
        "max_lift_cm": max((r["lift_cm"] for r in rows), default=None),
        "max_score": max((r["score"] for r in rows), default=None),
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    print(f"run and videos: {out}")


if __name__ == "__main__":
    main()
