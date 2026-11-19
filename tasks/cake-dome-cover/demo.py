"""Run the task file's scripted demo (its demo() waypoints, played by the yam_scripted policy) and keep its media.

    python demo.py task.py OUT_DIR [--no-record] [--max-steps N]

Writes OUT_DIR/result.json (task_score, success, steps), and unless --no-record, demo_top_cam.mp4 (and the side
cameras) and setup.png (the first top-camera frame). Negative variants are chosen with DEMO_VARIANT=<name>, which
task.py's demo() reads. The demo plan reads nothing from the simulator: see demo() in task.py.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import pathlib
import shutil
import subprocess
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--max-steps", type=int, default=None)
    a = ap.parse_args()
    here = pathlib.Path(a.task).resolve().parent
    spec = json.loads((here / "task.json").read_text())
    steps = a.max_steps or int(spec["run_limit"].split()[0].replace(",", ""))
    out = pathlib.Path(a.out).resolve()
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    cmd = ["inspect-robots", "run", "--embodiment", "yam_arena", "--policy", "yam_scripted",
           "-E", f"task={pathlib.Path(a.task).resolve()}", "-E", "object_jitter=0", "-E", "control_hz=20",
           "-E", "cam_width=640", "-E", "cam_height=352",
           "--instruction", spec["instruction"], "--scorer", "task_score", "--max-steps", str(steps),
           "--log-dir", str(out / "log")]
    if not a.no_record:
        cmd.append("--store-frames")
    print(" ".join(cmd), flush=True)
    rc = subprocess.run(cmd, cwd=here).returncode
    logs = sorted(glob.glob(str(out / "log" / "*.json")))
    if rc != 0 or not logs:
        print(f"demo.py: inspect-robots exited {rc}, logs {logs}", file=sys.stderr)
        return rc or 1
    log = json.loads(pathlib.Path(logs[-1]).read_text())
    ep = log["samples"][0]["epochs"][0]
    score = float(log["results"]["metrics"]["task_score"])
    result = {"task_score": score, "success": score == 1.0, "steps": int(log["stats"]["total_steps"]),
              "termination": ep.get("termination_reason"), "variant": os.environ.get("DEMO_VARIANT", "")}
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    if not a.no_record:
        subprocess.run(["inspect-robots", "video", logs[-1]], check=False)
        for cam in ("top", "left", "right"):
            hits = sorted(glob.glob(str(out / "log" / "frames" / "*" / f"*_{cam}_cam.mp4")))
            if hits:
                shutil.copy(hits[-1], out / f"demo_{cam}_cam.mp4")
        top = out / "demo_top_cam.mp4"
        if top.exists():
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(top), "-frames:v", "1",
                            str(out / "setup.png")], check=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
