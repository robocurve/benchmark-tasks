"""Run the task through the official Inspect Robots CLI (`inspect-robots run`, default guardrails on) with an open-loop
replay of the scripted demo's recorded 14-D joint/gripper actions (`actions.json`). Registers the policy
`insole_replay` in-process and calls the unmodified `inspect_robots.cli.main`. After the last action it holds it for
`hold_s` seconds, then asks the rollout to stop.

  python official_replay.py --actions DEMO/actions.json [--hold-s 1] -- run --embodiment yam_arena \\
      --policy insole_replay -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
      --instruction "<task.INSTRUCTION>" --scorer task_score --max-steps 2400 --store-frames --log-dir OUT
"""
import argparse
import json
import os
import sys

import numpy as np
from inspect_robots import registry
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from inspect_robots_yam.packing import pack
from yam_arena import ik

ACTIONS = {}


class ReplayPolicy(PolicyBase):
    def __init__(self, actions=None, hold_s=1.0):
        low, high = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
        self.info = PolicyInfo(name="insole_replay", action_space=action_box(pack(low, low), pack(high, high)))
        rec = json.loads(open(actions or ACTIONS["path"]).read())
        self.actions = [np.asarray(a, float) for a in rec["actions"]]
        self.hz = float(rec["control_hz"])
        self.hold = int(round(float(hold_s) * self.hz))
        self.k = 0

    def reset(self, scene):
        self.k = 0

    def act(self, observation):
        a = self.actions[min(self.k, len(self.actions) - 1)]
        self.k += 1
        meta = {"request_stop": True} if self.k >= len(self.actions) + self.hold else {}
        return ActionChunk(actions=[Action(data=a.copy(), meta=meta)], control_hz=self.hz)


@registry.policy("insole_replay")
def insole_replay(**kwargs):
    return ReplayPolicy(**kwargs)


def main():
    argv = sys.argv[1:]
    i = argv.index("--")
    ap = argparse.ArgumentParser()
    ap.add_argument("--actions", required=True)
    ap.add_argument("--hold-s", type=float, default=1.0)
    a = ap.parse_args(argv[:i])
    ACTIONS["path"] = a.actions
    sys.argv = sys.argv[:1]  # Kit reads sys.argv too; a literal -- there hides the appended kit arguments from it
    from inspect_robots.cli import main as cli_main
    code = cli_main(argv[i + 1:] + ["-P", f"hold_s={a.hold_s}"])
    sys.stdout.flush()
    os._exit(int(code or 0))  # Kit shutdown can hang; the EvalLog is already written


if __name__ == "__main__":
    main()
