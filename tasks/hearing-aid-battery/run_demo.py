"""Run the scripted reference demo through Inspect Robots' own CLI, as any policy runs.

    python run_demo.py OUT_DIR [--width 640 --height 360] [--mode full] [--no-frames] [--stills N]

The demo is closed loop (demo.py): each control step it reads the applicator's pose and the plaster's tabs and ends, which
the cameras see, the fixed positions of the forearm, the wound, the cup and the pot, and its own arms' joints. It is registered as the policy
``hearing_aid_demo`` and run with the same embodiment, task file, scorer and step limit as any other policy
(``-E object_jitter=0``, 20 Hz), through the same guards: its commands stay within the bounds and the per-step change
that the guards allow (``demokit.Ctx.command``), so they reach the arms unchanged. A learned policy gets only the cameras and joints. ``--no-frames`` keeps the log
without camera frames; ``--stills N`` writes the three cameras every N control steps as PNG files.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
STILLS = [0, None]
sys.path.insert(0, str(HERE))


def register(mode: str) -> str:
    from inspect_robots.registry import register as reg
    from inspect_robots.types import Action, ActionChunk
    from yam_arena.scripted import YamScriptedPolicy

    import demokit
    import bodykit
    import numpy as np

    class _Emb:
        """What demokit's Ctx reads from an embodiment: the Isaac Lab env and the task's object names."""

        def __init__(self, names):
            self.task = type("T", (), {"names": names})()

        @property
        def _env(self):
            return bodykit.env()

    class HearingAidDemo(YamScriptedPolicy):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.info = replace(self.info, name="hearing_aid_demo")

        def reset(self, scene):
            os.environ["TASK_DEMO"] = mode
            module = self._task.module
            names = tuple(o.name for o in module.OBJECTS)
            self.ctx = None
            self.module, self.names = module, names
            self.done = False

        def act(self, observation):
            if self.ctx is None:
                self.ctx = demokit.Ctx(_Emb(self.names), observation, mode)
                self.script = self.module.demo_script(self.ctx)
            self.ctx.obs = observation
            self.ctx.t = float(observation.state_time)
            self.k = getattr(self, "k", -1) + 1
            if STILLS[0] and self.k % STILLS[0] == 0:
                for cam, image in observation.images.items():
                    demokit._save_png(np.asarray(image), STILLS[1] / f"{cam}_{self.k:05d}.png")
            if not self.done:
                try:
                    next(self.script)
                except StopIteration:
                    self.done = True
            meta = {"request_stop": True} if self.done else {}
            return ActionChunk(actions=[Action(data=self.ctx.command(), meta=meta)], control_hz=20.0)

    reg("policy", "hearing_aid_demo")(HearingAidDemo)
    return "hearing_aid_demo"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", type=Path)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--mode", default="full")
    ap.add_argument("--no-frames", action="store_true")
    ap.add_argument("--stills", type=int, default=0)
    a = ap.parse_args()
    task_json = json.loads((HERE / "task.json").read_text())
    steps = int(re.search(r"[\d,]+", task_json["run_limit"]).group().replace(",", ""))
    a.out.mkdir(parents=True, exist_ok=True)
    os.environ["TASK_DEMO"] = a.mode
    STILLS[0], STILLS[1] = a.stills, a.out
    policy = register(a.mode)
    from inspect_robots.cli import main as cli

    args = ["run", "--embodiment", "yam_arena", "--policy", policy, "-E", f"task={HERE / 'task.py'}", "-E", "object_jitter=0",
            "-E", "control_hz=20", "-E", f"cam_width={a.width}", "-E", f"cam_height={a.height}",
            "--instruction", task_json["instruction"], "--scorer", "task_score", "--max-steps", str(steps),
            "--log-dir", str(a.out)] + ([] if a.no_frames else ["--store-frames"])
    result = cli(args)
    return 0 if result in (None, 0) else int(result)


if __name__ == "__main__":
    raise SystemExit(main())
