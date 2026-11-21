r"""Run the scripted bimanual demo and export its videos.

From the benchmark-tasks repository root:

    python tasks/hang-a-shirt-on-a-partly-full-airer/run_demo.py --out ~/yam-eval/airer-demo

It records one fixed-layout trial (three cameras), exports MP4s into <out>/videos/ and writes trace.jsonl and
summary.json. The demo succeeded only if summary.json reports a final score of 1.0.

The policy plays the planned joint targets (shirtplan.plan()) and corrects for the arms' sag from the observed joint
positions, as ordinary joint targets: no object is moved, attached or pushed by software. (The only hold on the
fabric is the gripper's own pinch, pinch.py, which is the same for every policy.)
"""

import argparse
import hashlib
import json
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import shirtplan  # noqa: E402
from inspect_robots.cli import main as inspect_main  # noqa: E402
from inspect_robots.registry import policy  # noqa: E402
from inspect_robots.types import Action, ActionChunk  # noqa: E402
from inspect_robots_yam.packing import STATE_KEY, pack  # noqa: E402
from yam_arena.scripted import YamScriptedPolicy  # noqa: E402

MAX_DELTA = 0.045  # rad per control step, under the embodiment's 0.05 rad command guard


@policy("shirt_hang_demo")
class ShirtDemo(YamScriptedPolicy):
    def reset(self, scene):
        self.steps = shirtplan.plan().steps
        self.k = 0
        self.integral = np.zeros(14)
        self.previous = None

    def act(self, observation):
        row = self.steps[min(self.k, len(self.steps) - 1)]
        nominal = pack(*[np.r_[row[arm][0], row[arm][1]] for arm in ("left", "right")])
        actual = np.asarray(observation.state[STATE_KEY])
        error = nominal - actual
        error[[6, 13]] = 0.0
        self.integral = np.clip(self.integral + 1.5 * self._dt * error, -0.15, 0.15)
        target = nominal + 0.3 * error + self.integral
        target[[6, 13]] = nominal[[6, 13]]
        previous = actual if self.previous is None else self.previous
        delta = target - previous
        delta[[6, 13]] = 0.0
        scale = min(1.0, MAX_DELTA / max(float(np.abs(delta).max()), 1e-12))
        target = previous + delta * scale
        target[[6, 13]] = nominal[[6, 13]]
        if scale >= 0.999:  # a command the arms could not make in one step holds the plan back a step
            self.k += 1
        self.previous = target.copy()
        meta = {"request_stop": True} if self.k >= len(self.steps) + int(self.hold_s / self._dt) else {}
        if self.k >= len(self.steps):
            self.k += 0 if meta else 1
        return ActionChunk(actions=[Action(data=target, meta=meta)], control_hz=1 / self._dt)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path,
                        default=Path.home() / "yam-eval" / datetime.now().strftime("airer-demo-%Y%m%d-%H%M%S"))
    parser.add_argument("--max-steps", type=int, default=900)
    parser.add_argument("--hold-s", type=float, default=8.0, help="seconds the demo holds still after its plan")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--no-frames", action="store_true", help="tuning runs: store no camera frames")
    parser.add_argument("--cam", default="640x360")
    args = parser.parse_args()
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out / "source-sha256.json").exists():
        parser.error("Use a new output directory to keep the previous trial.")
    (out / "source").mkdir(exist_ok=True)
    hashes = {}
    for file in HERE.iterdir():
        if file.suffix in (".py", ".md", ".json") and file.is_file():
            shutil.copy2(file, out / "source" / file.name)
            hashes[file.name] = hashlib.sha256(file.read_bytes()).hexdigest()
    (out / "source-sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
    instruction = json.loads((HERE / "task.json").read_text())["instruction"] if (HERE / "task.json").exists() else "x"
    os.environ["SHIRT_TRACE"] = str(out / "trace.jsonl")
    w, h = args.cam.split("x")
    result = inspect_main(["run", "--embodiment", "yam_arena", "--policy", "shirt_hang_demo",
                           "-P", f"hold_s={args.hold_s}",
                           "-E", f"task={HERE / 'task.py'}", "-E", "object_jitter=0",
                           "-E", "control_hz=20", "-E", f"cam_width={w}", "-E", f"cam_height={h}",
                           "--instruction", instruction, "--scorer", "task_score",
                           "--max-steps", str(args.max_steps), "--no-prompt", "--no-rerun",
                           *([] if args.no_frames else ["--store-frames"]), "--log-dir", str(out)])
    logs = list(out.glob("adhoc_*.json"))
    if not args.no_video and not args.no_frames:
        for log in logs:
            inspect_main(["video", str(log), "--out", str(out / "videos")])
    trace = out / "trace.jsonl"
    if trace.exists():
        rows = [json.loads(line) for line in trace.read_text().splitlines()]
        if rows:
            summary = {"final": rows[-1],
                       "evaluation_scores": {log.name: json.loads(log.read_text()).get("results", {}).get("metrics", {})
                                             for log in logs}}
            (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            print(json.dumps(rows[-1].get("metrics", {}), indent=1))
    print(f"Run and videos: {out}")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
