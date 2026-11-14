"""Run Inspect Robots with an additional per-step mechanism audit (no action changes).

Usage: RATCHET_TRACE=/new/trace.jsonl python diagnostics/record.py run ...
The normal Inspect Robots JSON and native camera frames remain authoritative.
"""
import json
import os
from pathlib import Path
import sys

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ratchet_task.embodiment import RatchetEmbodiment, FEEDBACK
from inspect_robots.cli import main


def serial(v):
    if isinstance(v, np.ndarray):return v.tolist()
    if isinstance(v, np.generic):return v.item()
    if isinstance(v, dict):return {k:serial(x) for k,x in v.items()}
    if isinstance(v, (tuple,list)):return [serial(x) for x in v]
    return v


if __name__ == '__main__':
    path=Path(os.environ['RATCHET_TRACE'])
    path.parent.mkdir(parents=True, exist_ok=True)
    original=RatchetEmbodiment._task_step
    with path.open('x') as trace:
        def recorded(self):
            result=original(self)
            trace.write(json.dumps(serial(dict(time=self._t, feedback=FEEDBACK,
                history=vars(self.history), revision=self.info.environment_revision)))+'\n')
            trace.flush()
            return result
        RatchetEmbodiment._task_step=recorded
        sys.exit(main())
