"""Run the recessed-trim task through the OFFICIAL Inspect Robots CLI (`inspect-robots run`, default guardrails on) with the scripted
reference demo as a registered closed-loop policy.

Inspect Robots resolves policies only by registered name, so this wrapper registers `recessed_trim_scripted` in-process and
then calls the unmodified `inspect_robots.cli.main` with the given arguments: the trial runs through the standard
rollout loop (CLI bounds clamp + per-step delta limit guardrails, the yam_arena embodiment, the task file's
`score()` through `--scorer task_score`, and the EvalLog that `inspect-robots video` renders).

The policy is demo.py's script (same generator, same constants). It acts only through the 14-D joint/gripper action
it returns; it reads the observation's joint state, the
trim, leg and housing poses (trimkit.bench_state(), read-only), exactly as demo.py does. After the script ends it holds its last action for
`hold_s` seconds, then asks the rollout to stop.

Usage (in the Isaac container, cwd = <task folder>/isaac):
  python official_run.py [--stage demo] [--hold-s 2] -- run --embodiment yam_arena --policy recessed_trim_scripted \
      -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "<task.json instruction>" --scorer task_score --max-steps <run_limit> --store-frames \
      --log-dir <log dir>
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))

from inspect_robots import registry  # noqa: E402
from inspect_robots.policy import PolicyBase, PolicyInfo  # noqa: E402
from inspect_robots.types import Action, ActionChunk  # noqa: E402
from inspect_robots_yam.config import action_box  # noqa: E402
from inspect_robots_yam.packing import pack, STATE_KEY  # noqa: E402
from yam_arena import ik  # noqa: E402

OPTS = {"stage": "demo", "hold_s": 2.0, "log": None}


def _demo_module():
    """isaac/demo.py by path."""
    import importlib.util

    if "recessed_trim_isaac_demo" not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            "recessed_trim_isaac_demo", HERE / "demo.py"
        )
        mod = importlib.util.module_from_spec(spec)
        sys.modules["recessed_trim_isaac_demo"] = mod
        spec.loader.exec_module(mod)
    return sys.modules["recessed_trim_isaac_demo"]


class ScriptedPolicy(PolicyBase):
    def __init__(self, stage=None, hold_s=None):
        low, high = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
        self.info = PolicyInfo(
            name="recessed_trim_scripted",
            action_space=action_box(pack(low, low), pack(high, high)),
        )
        self.stage = stage or OPTS["stage"]
        self.hz = 20.0
        self.hold = int(
            round(float(hold_s if hold_s is not None else OPTS["hold_s"]) * self.hz)
        )
        self.reset(None)

    def reset(self, scene):
        demo = _demo_module()
        self.demo = demo
        self.c = demo.Ctx(None)
        self.gen = demo.STAGES[self.stage](self.c)
        self.k, self.ended, self.stop = 0, None, None
        self.actions = []

    def act(self, observation):
        jp = np.asarray(observation.state[STATE_KEY], float).ravel()
        if (
            self.k > 0
        ):  # the first observation is the reset one (arms at READY, as the Ctx assumes)
            self.c.left.observe(jp[:7])
            self.c.right.observe(jp[7:14])
        self.c.step = self.k
        if self.ended is None:
            try:
                next(self.gen)
            except StopIteration:
                self.ended = self.k
            except self.demo.Stop as e:
                self.ended, self.stop = self.k, str(e)
                print("STOP", self.stop, flush=True)
        self.k += 1
        a = np.asarray(self.c.action().data, float)
        self.actions.append(np.round(a, 6).tolist())
        if OPTS["log"]:  # every commanded action, one line per step (the executed-vs-commanded check reads it)
            Path(OPTS["log"]).mkdir(parents=True, exist_ok=True)
            with open(Path(OPTS["log"], "policy_actions.jsonl"), "a") as f:
                f.write(json.dumps(dict(t=self.k - 1, action=self.actions[-1])) + "\n")
        meta = (
            {"request_stop": True}
            if self.ended is not None and self.k - self.ended >= self.hold
            else {}
        )
        if OPTS["log"] and (meta or self.k % 20 == 0):
            self.dump()
        return ActionChunk(
            actions=[Action(data=a.copy(), meta=meta)], control_hz=self.hz
        )

    def dump(self):
        import trimkit

        Path(OPTS["log"]).mkdir(parents=True, exist_ok=True)
        Path(OPTS["log"], "policy_log.json").write_text(
            json.dumps(
                dict(
                    stage=self.stage,
                    steps=self.k,
                    stop=self.stop,
                    looks=self.c.log,
                    scorer=trimkit.STATE,
                    actions=self.actions,
                ),
                indent=1,
                default=float,
            )
        )


@registry.policy("recessed_trim_scripted")
def recessed_trim_scripted(**kwargs):
    return ScriptedPolicy(**kwargs)


def main():
    argv = sys.argv[1:]
    if "--" not in argv:
        raise SystemExit(__doc__)
    i = argv.index("--")
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage", default="demo")
    ap.add_argument("--hold-s", type=float, default=2.0)
    a = ap.parse_args(argv[:i])
    cli_args = argv[i + 1 :]
    OPTS.update(stage=a.stage, hold_s=a.hold_s)
    if "--log-dir" in cli_args:
        OPTS["log"] = cli_args[cli_args.index("--log-dir") + 1]
    from inspect_robots.cli import main as cli_main

    code = cli_main(cli_args)
    sys.stdout.flush()
    os._exit(int(code or 0))  # Kit shutdown can hang; the EvalLog is already written


if __name__ == "__main__":
    main()
