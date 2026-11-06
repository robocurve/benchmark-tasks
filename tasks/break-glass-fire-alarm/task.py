"""On the station table, a red fire alarm box stands on a short wall section, its front sloped towards the arms, with a fire burning along the back of the table. The box has a glass window and, behind the glass, a yellow
handle that slides towards the robot, over a printed PULL and arrow. Smashing the glass breaks it; pulling the handle to the bottom sets off the flashing strobes.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import alarm_usd as u  # noqa: E402
import alarmkit as k  # noqa: E402  (hooks yam_arena so the parts below are simulated)
from alarm_score import PULLED  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
INSTRUCTION = "Smash the glass and pull the alarm."
OBJECTS = k.parts()
SUCCESS_HOLD_S = 5.0  # the run goes on this long after the pull, so the flashing strobes can be seen
SCORE_OVER = "end"


def _report(objects) -> dict:
    return objects["alarm_wall"].report


def success(objects) -> bool:
    """The glass broken and the handle pulled."""
    return bool(_report(objects)["success_now"])


def score(objects) -> float:
    """0.3 for the glass broken; 1.0 once the handle is pulled."""
    return _report(objects)["score"]


def distance(objects) -> float:
    """How far the handle still has to go, metres."""
    return max(0.0, PULLED - _report(objects)["handle_mm"] / 1000.0)


# -- the scripted demo: open loop, straight-down grips ---------------------------------------
# One arm does the job; the left arm stays at y +0.40, so the arms are never near each other.

YAW = math.pi / 2  # the right arm closes its fingers along x
TIP = 0.013  # fingertips past the TCP
CLOSED, GRIP_OPEN, GRIP_SHUT = 0.0, -0.016, -0.001  # the bar is 13 mm across: closing 5.5 mm short of it holds it firmly
STRIKE_ALONG = -0.03  # up the slope from its middle: in the window, below the handle's bar
STRIKE_PAST = 0.012  # the target past the glass's face
GRIP_ABOVE_BAR = 0.008  # the TCP above the bar's axis, which keeps the fingertips off the box's floor
PULL_PAST = 0.006  # the pull's target past the stop: the arm lags and the grip gives a little, and the alarm counts only at the bottom


def _at(p, up: float = 0.0) -> tuple[float, float, float]:
    return float(p[0]), float(p[1]), float(k.TOP + p[2] + up)


def demo() -> dict[str, list[Waypoint]]:
    """`ALARM_DEMO=safe` (default): smash the glass and pull the handle. `glass`: smash the glass and stop. `slow`: push through the glass slowly and stop. `touch`: rest on the glass and stop. `still`: both arms stay away."""
    mode = os.environ.get("ALARM_DEMO", "safe")
    if mode not in ("safe", "glass", "slow", "touch", "still"):
        raise ValueError(f"ALARM_DEMO must be safe, glass, slow, touch or still, got {mode!r}")
    left = [Waypoint(1.5, (0.22, 0.40, k.TOP + 0.18), FINGER_OPEN, "away", -math.pi / 2)]
    w: list[Waypoint] = []

    def go(duration, pos, finger, label):
        w.append(Waypoint(duration, pos, finger, label, YAW))

    away = (0.22, -0.40, k.TOP + 0.18)
    if mode == "still":
        go(1.5, away, FINGER_OPEN, "away")
        return {"left": left, "right": w}
    face = u.on_table(STRIKE_ALONG, 0.0, u.RIM + u.PANE_THICK)
    tcp_on_face = face + (0.0, 0.0, TIP)
    above = _at(tcp_on_face, 0.05)
    bar = u.on_table(u.BAR_X, 0.0, u.BAR_Z)
    bar_tcp = bar + (0.0, 0.0, GRIP_ABOVE_BAR)
    pulled = bar_tcp - (u.TRAVEL + PULL_PAST) * u.UP_SLOPE
    go(1.8, above, CLOSED, "above the glass")
    go(0.7, above, CLOSED, "steady over the glass")
    if mode in ("slow", "touch"):
        go(6.0, _at(tcp_on_face, -0.008 if mode == "slow" else 0.0), CLOSED, "push slowly" if mode == "slow" else "rest on the glass")
        go(1.0, above, CLOSED, "back up")
        go(1.5, away, FINGER_OPEN, "away")
        return {"left": left, "right": w}
    go(0.28, _at(tcp_on_face, -STRIKE_PAST), CLOSED, "strike")
    go(0.2, _at(tcp_on_face, -STRIKE_PAST), CLOSED, "through")
    go(0.8, above, CLOSED, "back up")
    if mode == "glass":
        go(1.5, away, FINGER_OPEN, "away")
        return {"left": left, "right": w}
    over_bar = _at(bar_tcp, 0.05)
    go(1.0, over_bar, GRIP_OPEN, "over the handle")
    go(0.7, over_bar, GRIP_OPEN, "steady over the handle")
    go(1.4, _at(bar_tcp), GRIP_OPEN, "down to the handle")
    go(0.6, _at(bar_tcp), GRIP_SHUT, "close on the handle")
    go(0.3, _at(bar_tcp), GRIP_SHUT, "hold the handle")
    go(1.5, _at(pulled), GRIP_SHUT, "pull")
    go(0.4, _at(pulled), GRIP_SHUT, "hold it pulled")
    go(0.5, _at(pulled), GRIP_OPEN, "let go")
    go(1.0, _at(pulled, 0.05), GRIP_OPEN, "up and clear")
    go(1.6, away, GRIP_OPEN, "away")
    return {"left": left, "right": w}
