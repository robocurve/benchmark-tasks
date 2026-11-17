"""hhdebug: with HH_DEBUG=<file>, append each tenth control step's gripper TCP and fingertips and the named loose
bodies' poses (position, yaw, tilt) to a JSON-lines file. Offline helper for developing the scripted plans; does
nothing when HH_DEBUG is unset."""
import json
import math
import os

import hhkit

_PATH = os.environ.get("HH_DEBUG")


def log(trial, objects, names):
    if not _PATH or trial.step % 10:
        return
    rec = {"step": trial.step, "grippers": [{"tcp": [round(float(v), 4) for v in g["tcp"]],
                                             "tips": [[round(float(v), 4) for v in t] for t in g["tips"]]}
                                            for g in trial.grippers()]}
    for n in names:
        s = objects[n]
        rec[n] = [round(float(v), 4) for v in s.pos] + [round(hhkit.yaw_deg(s), 1), round(hhkit.tilt_deg(s), 1)]
    rec["mem"] = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in trial.mem.items()
                  if isinstance(v, (int, float, str, bool))}
    with open(_PATH, "a") as f:
        f.write(json.dumps(rec) + "\n")
