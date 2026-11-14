"""Check the calibrated robot program; native execution still proves feasibility."""

import argparse
import json
import numpy as np
from chain_link.controller import ScriptedPolicy

parser = argparse.ArgumentParser()
parser.add_argument("--mode", default="demo")
args = parser.parse_args()
policy = ScriptedPolicy(mode=args.mode)
commands = np.array(
    [policy.action_at(i / 20) for i in range(1, len(policy.actions) + 1)]
)
print(json.dumps(dict(
    scope="offline robot command validation only",
    mode=args.mode,
    actions=len(commands),
    duration_s=policy.duration,
    maximum_step_change=np.abs(np.diff(commands, axis=0)).max(axis=0).tolist(),
    program_sha256=policy.program_sha256,
    runtime_inputs=["elapsed time"],
)))
