"""On the station table: a forearm suture-practice trainer lying across the table, palm down, with a 14 x 7 mm abrasion
on the top of the mid-forearm and a dotted skin-marker outline 5 mm out from its edge. To the robot's right, a sterile
cotton-tipped applicator in its opened peel pouch on a small tray, and a 30 mL medicine cup holding a dollop of
antiseptic cream. Behind the forearm, a 2 x 4 in fabric plaster on its opened wrapper on a riser, its two release
liners' tabs standing out past its ends. In front, a waste pot.

The task: cream the wound and its margin with the applicator and bin it, then put the plaster on: pad centred over the
creamed wound, both liners off and binned, both wings pressed down, without touching the wound, the pad's face or the
adhesive, and without cream under the adhesive.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # lets this file import helpers beside it

import bodykit  # noqa: E402
import props  # noqa: E402
import steptime  # noqa: E402

bodykit.install()

SCENE = "station"
SUCCESS_HOLD_S = 0.05  # the 1 s hold of the full state is checked by the scorer itself (rules.HOLD_STEPS)
OBJECTS = props.bodies()

_MODEL = {}


def model():
    if "m" not in _MODEL:
        import model as M

        _MODEL["m"] = M.Model()
    return _MODEL["m"]


@bodykit.on_reset
def _reset() -> None:
    model().reset()


@bodykit.on_step
def _step(env) -> None:
    m = model()
    m.attach(env)
    m.step()


def score(objects) -> float:
    """1.0: cream on, applicator binned, plaster centred, both liners binned, both wings tacked, hands clear, held 1 s;
    0.75 / 0.5 / 0.25 as rules.py says; 0 after any broken rule."""
    return model().tracker.score()


def success(objects) -> bool:
    steptime.tick()  # the harness calls success() once per control step
    return model().tracker.success_now()


def demo_script(ctx):
    """The scripted reference demo (demo.py): it reads what the cameras show (the applicator, the liner tabs, the strip's
    ends), the fixed scene and its own joints."""
    import demo
    import firstaid as F
    import scene as S

    m = model()
    yield from demo.script(ctx, lambda: F.view(m.w, S.TABLE))
