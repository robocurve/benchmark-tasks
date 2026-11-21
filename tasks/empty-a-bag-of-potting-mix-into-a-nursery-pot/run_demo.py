r"""Run the scripted bimanual demo and export its videos.

From the benchmark-tasks repository root:

    python tasks/empty-a-bag-of-potting-mix-into-a-nursery-pot/run_demo.py \
        --out "$HOME/yam-eval/potting-mix-$(date +%Y%m%d-%H%M%S)"

It records one fixed-layout trial (three cameras), exports MP4s into <out>/videos/, and writes trace.jsonl and
summary.json. The demo succeeded only if summary.json reports a final score of 1.0.

The policy plays the planned joint targets (bagplan.plan()) and corrects for the arms' sag under load from the
observed joint positions, as ordinary joint targets: no object attachment, pose writes or extra forces.
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

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bagplan  # noqa: E402
from inspect_robots.cli import main as inspect_main  # noqa: E402
from inspect_robots.registry import policy  # noqa: E402
from inspect_robots.types import Action, ActionChunk  # noqa: E402
from inspect_robots_yam.packing import STATE_KEY, pack  # noqa: E402
from yam_arena.scripted import YamScriptedPolicy  # noqa: E402

MAX_DELTA = 0.045  # rad per control step, under the embodiment's 0.05 rad command guard


@policy("bimanual_bag_demo")
class BagDemo(YamScriptedPolicy):
    def reset(self, scene):
        self.steps = bagplan.plan().steps
        self._end = len(self.steps) * self._dt
        self.k = 0
        self.integral = np.zeros(14)
        self.previous = None

    def act(self, observation):
        row = self.steps[min(self.k, len(self.steps) - 1)]
        nominal = pack(*[np.r_[row[arm][0], 1.0 - row[arm][1]] for arm in ("left", "right")])
        actual = np.asarray(observation.state[STATE_KEY])
        error = nominal - actual
        error[[6, 13]] = 0.0
        self.integral = np.clip(self.integral + 1.5 * self._dt * error, -0.20, 0.20)
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
                        default=Path.home() / "yam-eval" / datetime.now().strftime("potting-mix-%Y%m%d-%H%M%S"))
    parser.add_argument("--max-steps", type=int, default=1200)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--cam", default="640x360")
    args = parser.parse_args()
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out / "source-sha256.json").exists():
        parser.error("Use a new output directory to keep the previous trial.")
    source = out / "source"
    source.mkdir(exist_ok=True)
    hashes = {}
    for file in Path(__file__).parent.iterdir():
        if file.suffix in (".py", ".npz", ".md", ".json") and file.is_file():
            shutil.copy2(file, source / file.name)
            hashes[file.name] = hashlib.sha256(file.read_bytes()).hexdigest()
    (out / "source-sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
    instruction = json.loads(Path(__file__).with_name("task.json").read_text())["instruction"]
    os.environ["BAG_TRACE"] = str(out / "trace.jsonl")
    w, h = args.cam.split("x")
    result = inspect_main(["run", "--embodiment", "yam_arena", "--policy", "bimanual_bag_demo",
                           "-E", f"task={Path(__file__).with_name('task.py')}", "-E", "object_jitter=0",
                           "-E", "control_hz=20", "-E", f"cam_width={w}", "-E", f"cam_height={h}",
                           "--instruction", instruction, "--scorer", "task_score",
                           "--max-steps", str(args.max_steps), "--seed", str(args.seed), "--no-prompt", "--no-rerun",
                           "--store-frames", "--log-dir", str(out)])
    logs = list(out.glob("adhoc_*.json"))
    if not args.no_video:
        for log in logs:
            inspect_main(["video", str(log), "--out", str(out / "videos")])
    trace = out / "trace.jsonl"
    if trace.exists():
        rows = [json.loads(line) for line in trace.read_text().splitlines()]
        if rows:
            summary = {"final": rows[-1], "seed": args.seed,
                       "evaluation_scores": {log.name: json.loads(log.read_text()).get("results", {}).get("metrics", {})
                                             for log in logs}}
            (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            print(json.dumps(rows[-1]["metrics"], indent=1))
    print(f"Run and videos: {out}")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
