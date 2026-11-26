r"""Run the scripted bimanual demo and export its videos.

From the benchmark-tasks repository root:

    python tasks/take-one-tortilla-out-of-a-resealable-bag/run_demo.py --out "$HOME/yam-eval/tortilla-demo-1"

It records one fixed-layout trial (three cameras), exports MP4s into <out>/videos/, and writes trace.jsonl and
summary.json. The demo succeeded only if summary.json reports a final score of 1.0.

The policy plays plan.Demo's joint targets, planned phase by phase from where the lip and the top tortilla are when
the phase starts (read from the simulation, as a scripted demo may), and corrects for the arms' sag under load from
the observed joint positions. It sends ordinary joint targets: no object attachment, pose writes or extra forces.
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

import plan  # noqa: E402
from inspect_robots.cli import main as inspect_main  # noqa: E402
from inspect_robots.registry import policy  # noqa: E402
from inspect_robots.types import Action, ActionChunk  # noqa: E402
from inspect_robots_yam.packing import STATE_KEY, pack  # noqa: E402
from yam_arena.scripted import YamScriptedPolicy  # noqa: E402

MAX_DELTA = 0.045  # rad per control step, under the embodiment's 0.05 rad command guard


def sense():
    """Where things are now, from the simulation (clothkit is the module the task file loaded)."""
    import clothkit
    import layout as L

    def lip(y):
        top = clothkit.points("film_top")
        nu = L.film("top")[3]["nu"]
        row0 = top[:nu]  # the lip's edge
        return row0[int(np.argmin(np.abs(row0[:, 1] - y)))].copy()

    def bottom_lip(y):
        bot = clothkit.points("film_bottom")
        nu = L.film("bottom")[3]["nu"]
        row0 = bot[:nu]
        return row0[int(np.argmin(np.abs(row0[:, 1] - y)))].copy()

    def top_tortilla():
        names = [f"tortilla{k}" for k in range(L.N_TORTILLAS)]
        return max(names, key=lambda n: clothkit.points(n)[:, 2].mean())

    def near_edge():
        p = clothkit.points(top_tortilla())
        return p[int(np.argmin(p[:, 0]))].copy()

    def front_edge(y):
        names = [f"tortilla{k}" for k in range(L.N_TORTILLAS)]
        over = [n for n in names if clothkit.points(n)[:, 0].min() < L.BOARD_NEAR_X - 0.02] or names
        # the top one of those hanging past the board's edge (the one slid out)
        n = max(over, key=lambda n: clothkit.points(n)[clothkit.points(n)[:, 0] < L.BOARD_NEAR_X, 2].mean())
        p = clothkit.points(n)
        band = np.abs(p[:, 1] - y) < 0.012
        q = p[band]
        return q[int(np.argmin(q[:, 0]))].copy()

    def uncovered(want):
        t = clothkit.points(top_tortilla())
        film = clothkit.points("film_top")
        front = t[(t[:, 0] < t[:, 0].min() + 0.050) & (t[:, 0] > t[:, 0].min() + 0.018) & (t[:, 1] > 0.0)]
        best = None
        for p in front:
            d = np.hypot(film[:, 0] - p[0], film[:, 1] - p[1])
            covered = np.any((d < 0.014) & (film[:, 2] > p[2] - 0.002) & (film[:, 2] < p[2] + 0.030))
            if not covered:
                cost = np.hypot(p[0] - want[0], p[1] - want[1])
                if best is None or cost < best[0]:
                    best = (cost, p)
        if best is None:
            return want
        return np.array([best[1][0], best[1][1], want[2]])

    return {"lip": lip, "bottom_lip": bottom_lip, "uncovered": uncovered, "near_edge": near_edge, "front_edge": front_edge}


@policy("tortilla_bag_demo")
class TortillaDemo(YamScriptedPolicy):
    def reset(self, scene):
        self.demo = plan.Demo(sense=sense)
        self.integral = np.zeros(14)
        self.previous = None
        self.row = None
        self.done = 0

    def act(self, observation):
        if self.row is None or self.advance:
            nxt = self.demo.next()
            if nxt is None:
                self.done += 1
            else:
                self.row = nxt
        row = self.row
        nominal = pack(*[np.r_[row[arm][0], 1.0 - row[arm][1]] for arm in ("left", "right")])
        actual = np.asarray(observation.state[STATE_KEY])
        error = nominal - actual
        error[[6, 13]] = 0.0
        self.integral = np.clip(self.integral + 4.0 * self._dt * error, -0.08, 0.08)  # enough for sag, little windup in contact
        target = nominal + 0.3 * error + self.integral
        target[[6, 13]] = nominal[[6, 13]]
        previous = actual if self.previous is None else self.previous
        delta = target - previous
        delta[[6, 13]] = 0.0
        scale = min(1.0, MAX_DELTA / max(float(np.abs(delta).max()), 1e-12))
        target = previous + delta * scale
        target[[6, 13]] = nominal[[6, 13]]
        self.advance = scale >= 0.999  # a command the arms could not make in one step holds the plan back a step
        self.previous = target.copy()
        os.environ["TORT_PHASE"] = row["right"][2]
        os.environ["TORT_CMD"] = json.dumps({a: self.demo.tip[a].round(4).tolist() for a in ("left", "right")})
        meta = {"request_stop": True} if self.done > int(1.0 / self._dt) else {}
        return ActionChunk(actions=[Action(data=target, meta=meta)], control_hz=1 / self._dt)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path,
                        default=Path.home() / "yam-eval" / datetime.now().strftime("tortilla-%Y%m%d-%H%M%S"))
    parser.add_argument("--max-steps", type=int, default=1400)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--no-frames", action="store_true")
    parser.add_argument("--cam", default="640x360")
    args = parser.parse_args()
    out = args.out.expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out / "source-sha256.json").exists():
        parser.error("Use a new output directory to keep the previous trial.")
    source = out / "source"
    source.mkdir(exist_ok=True)
    hashes = {}
    for file in HERE.iterdir():
        if file.suffix in (".py", ".npz", ".md", ".json") and file.is_file():
            shutil.copy2(file, source / file.name)
            hashes[file.name] = hashlib.sha256(file.read_bytes()).hexdigest()
    (out / "source-sha256.json").write_text(json.dumps(hashes, indent=2) + "\n")
    task_json = HERE / "task.json"
    instruction = (json.loads(task_json.read_text())["instruction"] if task_json.exists()
                   else "Take one tortilla out of the bag and put it on the plate.")
    os.environ["TORT_TRACE"] = str(out / "trace.jsonl")
    w, h = args.cam.split("x")
    cmd = ["run", "--embodiment", "yam_arena", "--policy", "tortilla_bag_demo",
           "-E", f"task={HERE / 'task.py'}", "-E", "object_jitter=0",
           "-E", "control_hz=20", "-E", f"cam_width={w}", "-E", f"cam_height={h}",
           "--instruction", instruction, "--scorer", "task_score",
           "--max-steps", str(args.max_steps), "--seed", str(args.seed), "--no-prompt", "--no-rerun",
           "--log-dir", str(out)]
    if not args.no_frames:
        cmd.append("--store-frames")
    result = inspect_main(cmd)
    logs = list(out.glob("adhoc_*.json"))
    if not args.no_video and not args.no_frames:
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
