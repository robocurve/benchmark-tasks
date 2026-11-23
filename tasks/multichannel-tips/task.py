"""On the station table: a 96-tip box of universal 200 uL yellow tips, loose on the bench on its tape marks, lid open
flat on the -y side, columns 1-3 already used; beside it, an 8-channel pipette (20-200 uL, conical friction-fit nozzles,
9 mm pitch) standing upright in its stand.

The task: load the eight tips of column 4 onto the pipette, every tip fully sealed, lift it so the tips are clear of
the box and hold it there, without a gripper touching a tip and with the box kept on its marks.
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # lets this file import helpers beside it

import bodykit  # noqa: E402
import steptime  # noqa: E402
import tipkit  # noqa: E402
from tip_rules import Tracker  # noqa: E402

bodykit.install()

SCENE = "station"
SUCCESS_HOLD_S = 2.0  # the pipette held with all eight tips seated and level, for 2 s
OBJECTS = tipkit.bodies()

MODEL = tipkit.Model()
TRACKER = Tracker(dt=0.05)
if MODEL.trace_path:
    import atexit

    atexit.register(MODEL.save)


@bodykit.on_reset
def _reset() -> None:
    MODEL.reset()
    TRACKER.reset()


@bodykit.on_step
def _step(env) -> None:
    MODEL.attach(env)
    TRACKER.update(MODEL.step())
    if os.environ.get("TASK_CLOSEUP"):  # the demo video's close-up of the nozzles (closeup.py); not a policy camera
        import closeup

        closeup.start(env.unwrapped.scene.env_prim_paths[0])
        closeup.step()


def score(objects) -> float:
    """1.0 for all eight column-4 tips seated and level with the pipette held for 2 s; 0.5 for seven; 0.25 for six;
    0 otherwise, and 0 after any broken rule (tip_rules.py)."""
    return TRACKER.score()


def success(objects) -> bool:
    steptime.tick()  # the harness calls success() once per control step
    return TRACKER.success_now()


# -- the scripted demo (closed loop; reads the poses of the pipette and the tip box, which every camera sees) -----------

def demo_script(ctx):
    import tipdemo

    yield from tipdemo.script(ctx)
