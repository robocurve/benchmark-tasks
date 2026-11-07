"""On the station table: in the middle, a black wedge-shaped bedside clock. Its display, three white-on-black number
drums seen through a window in the sloping front (as on a flip clock), shows 1:25; two grey buttons stand on its flat
top, printed HOUR (on the left) and MIN (on the right). Behind it, an analog clock on a wooden easel stand leans back
toward the camera.

Each press of HOUR adds one hour (12 goes to 1). Each press of MIN adds one minute (59 goes to 00, and the hour stays
as it is). Holding a button down does not repeat.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, on_reset, on_step  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 4.0     # the time must stay right for 4 s: pressing on past it does not succeed on the way

INSTRUCTION = "Set the bedside clock to the time on the analog clock."

TOP = g.TOP


def _drum_angles(hour_presses: int, min_presses: int) -> dict:
    """Each number drum's angle after these presses, unwrapped: it only ever turns forward, a step per number."""
    counts = g.drum_counts(hour_presses, min_presses)
    return {f"drum_{name}": -(counts[name] - g.drum_start(name)) * g.drum_step(name) for name in g.DRUMS}


# -- objects --------------------------------------------------------------------------------------------------------
CLOCK = UsdBody("bedside_clock", usd="assets/bedside_clock.usda", pos=(0.0, 0.0, TOP), size=(0.15, 0.22, 0.11),
                articulation=True,
                joint_pos=(("hour_btn", 0.0), ("min_btn", 0.0), ("drum_hours", 0.0), ("drum_tens", 0.0),
                           ("drum_ones", 0.0)),
                drives=(("hour_btn", 200.0, 2.0), ("min_btn", 200.0, 2.0), ("drum_.*", 0.0, 0.0)))
ANALOG = UsdBody("analog_clock", usd="assets/analog_clock.usda", pos=(0.0, 0.0, TOP), size=(0.1, 0.24, 0.3))
OBJECTS = [CLOCK, ANALOG]

# -- monitor: count presses, run the display ------------------------------------------------------------------------
BUTTONS = {"hour": "hour_btn", "min": "min_btn"}
_STATE: dict = {}


DRUM_TURN_STEPS = 2     # control steps a drum takes to roll on to the next number: no slower than presses can come


def _turn_drums(environment, reset: bool = False) -> None:
    """Roll the number drums toward the time the presses so far set. The monitor writes their angles itself, a few
    control steps per number, and folds them into one turn: the revolute joints report wrapped angles."""
    from scenekit import write_joint_positions

    target = _drum_angles(_STATE["presses"]["hour"], _STATE["presses"]["min"])
    shown = _STATE["drums"]
    for joint, angle in target.items():
        if reset:
            shown[joint] = angle
            continue
        limit = g.drum_step(joint[len("drum_"):]) / DRUM_TURN_STEPS
        shown[joint] += max(-limit, min(limit, angle - shown[joint]))
    write_joint_positions(environment, "bedside_clock", {j: math.remainder(a, 2 * math.pi) for j, a in shown.items()})


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(hour=g.START[0], minute=g.START[1], presses={"hour": 0, "min": 0}, armed={"hour": True, "min": True},
                  depth={"hour": 0.0, "min": 0.0},
                  drums={f"drum_{name}": 0.0 for name in g.DRUMS})
    if environment is not None:
        _turn_drums(environment, reset=True)


@before_reset
def _restart_clock(environment) -> None:
    """Put the buttons up and the drums back to 1:25 before a reset (the reset moves the clock only by its root, not
    its joints)."""
    from scenekit import write_joint_positions

    write_joint_positions(environment, "bedside_clock", {"hour_btn": 0.0, "min_btn": 0.0, "drum_hours": 0.0,
                                                         "drum_tens": 0.0, "drum_ones": 0.0})


_reset_state()


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    joints = object_states(environment, ["bedside_clock"])["bedside_clock"].joints
    for name, joint in BUTTONS.items():
        depth = -float(joints.get(joint, 0.0))
        _STATE["depth"][name] = depth
        if _STATE["armed"][name] and depth > g.PRESSED:
            _STATE["armed"][name] = False
            _STATE["presses"][name] += 1
            if name == "hour":
                _STATE["hour"] = _STATE["hour"] % 12 + 1
            else:
                _STATE["minute"] = (_STATE["minute"] + 1) % 60
        elif not _STATE["armed"][name] and depth < g.RELEASED:
            _STATE["armed"][name] = True
    _turn_drums(environment)


def _hours_off() -> int:
    """How many hours the bedside clock is from the analog clock's hour, either way round."""
    d = (_STATE["hour"] - g.TARGET[0]) % 12
    return min(d, 12 - d)


def _minute_ok() -> bool:
    d = (_STATE["minute"] - g.TARGET[1]) % 60
    return min(d, 60 - d) <= 1


def success(objects) -> bool:
    """The bedside clock shows the analog clock's time (4:40, within a minute); held 4 s."""
    return bool(_hours_off() == 0 and _minute_ok())


def score(objects) -> float:
    """Partial credit (success scores 1.0): 1.0 for the right time; 0.5 for the right minutes with the hour one off
    (the hour hand misread); else 0. The hour alone earns nothing: three blind HOUR presses would get it."""
    if not _minute_ok():
        return 0.0
    return {0: 1.0, 1: 0.5}.get(_hours_off(), 0.0)
