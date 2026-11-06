"""On the bench, in the middle: a machine's reset button on a vertical steel control panel, behind a
clear polycarbonate guard 160 x 123 x 100 mm in a dark steel frame, closed on its sides and roof. The red button is lit, a 25 mm
plunger standing 15 mm proud of the panel, 90 mm behind the guard, and it springs back out when
nothing holds it in. The guard has one 34 mm square service port, level with the button and on its
axis. The gripper does not fit through the port.

A rack nearer the arms holds three bars, each lying along the bench away from the arms and level with
the port: a 320 x 12 x 12 mm aluminium rod in the middle, a 120 x 25 x 4 mm wooden rule beyond it on
the left, and a 320 x 45 x 45 mm oak baton on the right. Only the rod is both long enough to reach
the button and thin enough to pass the port.

Score (task_score): 1.0 once the button has been held at least 5 mm in for a second by a bar lying
through the port; otherwise the best the trial reached -- 0.5 if a bar's end got at least 50 mm inside
the guard, 0.25 if the rod was taken off its rack, 0 otherwise.

Run it:  inspect-robots run --embodiment yam_arena -E task=press_button_through_hole.py \\
           --instruction "Press the red button." --scorer task_score ...
"""

import importlib.util
import os
import pathlib
import sys

# The embodiment imports this file by path, so its folder is not on sys.path and a plain ``import
# cage`` would fail.
for _name, _file in (("simkit_pbth", "simkit.py"), ("cage_pbth", "cage.py")):
    if _name not in sys.modules:
        _spec = importlib.util.spec_from_file_location(_name, pathlib.Path(__file__).with_name(_file))
        _module = importlib.util.module_from_spec(_spec)
        sys.modules[_name] = _module
        if _name == "cage_pbth":
            sys.modules["simkit"] = sys.modules["simkit_pbth"]  # cage.py does `import simkit`
        _spec.loader.exec_module(_module)

simkit = sys.modules["simkit_pbth"]
cage = sys.modules["cage_pbth"]

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP

simkit.install()
simkit.patch_demo()  # 3x3 rotations in Waypoint.yaw, and ArmPlan gliding along a spline

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "best"  # partial credit is the furthest the trial got, not where it happened to end

OBJECTS = [
    *cage.guard_objects(TOP),
    *cage.frame_objects(TOP),
    cage.button_object(TOP),
    *cage.rack_objects(TOP),
    *cage.bar_objects(TOP),
]

BARS = {"rod": cage.ROD_LEN, "short_rule": cage.RULE_LEN, "thick_baton": cage.BATON_LEN}

# -- thresholds -------------------------------------------------------------------------

_TRACE = {"on": bool(os.environ.get("PBTH_TRACE")), "n": 0, "last": 0.0}

PRESS_M = cage.PRESSED_M  # 5 mm of button travel counts as pressed
REACH_M = 0.050  # a bar's end this far inside the guard earns partial credit
MAX_TILT_DEG = 12.0  # how far off horizontal the rod may be while it presses


def _pressed(objects) -> bool:
    return cage.button_travel(objects["button"]) >= PRESS_M


def _rod_in_port(objects) -> bool:
    """The rod lies through the port, far end deep inside, near end still outside.

    The guarded volume is 105 mm deep and the rod is 320 mm long, so a rod that reaches the button
    must be sticking out of the port. That is the geometry of a bar being held and pushed, which is
    what stops one being thrown in and abandoned.
    """
    far, near = cage.bar_ends(objects["rod"], cage.ROD_LEN)
    return bool(
        cage.inside_guard(far, TOP)
        and far[0] >= cage.CAGE_X0 + REACH_M
        and near[0] < cage.CAGE_X0
        and cage.bar_tilt_deg(objects["rod"]) <= MAX_TILT_DEG
    )


def success(objects) -> bool:
    """Full score: the button at least 5 mm in, with the rod lying through the port.

    ``SUCCESS_HOLD_S`` (1 s) means this must stay true for a second together, which is what makes the
    press a *held* press. That hold is the stability requirement -- the bodies are not additionally
    required to be at rest, because the rod is gripped by a moving arm throughout and a strict
    velocity test on it would be a test of the arm's tremor rather than of the press.

    Full score is reached only here. ``score()`` stops at 0.5, so a press that is let go after a few
    control steps cannot latch 1.0 through ``SCORE_OVER = "best"``.
    """
    return bool(_pressed(objects) and _rod_in_port(objects))


def distance(objects) -> float:
    """Metres from the rod's far end to the point it must reach to press the button."""
    far, _ = cage.bar_ends(objects["rod"], cage.ROD_LEN)
    goal = (cage.BUTTON_FACE_X - PRESS_M, cage.CAGE_YC, TOP + cage.PLINTH_H + cage.PORT_ZC_REL)
    return float(sum((far[i] - goal[i]) ** 2 for i in range(3)) ** 0.5)


def _trace(objects) -> None:
    """With PBTH_TRACE=1, print the rod and button every 10th control step, to diagnose a run."""
    _TRACE["n"] += 1
    if _TRACE["n"] % 10:
        return
    far, near = cage.bar_ends(objects["rod"], cage.ROD_LEN)
    print(
        f"[pbth {_TRACE['n']:4d}] rod far={far.round(4)} near_x={near[0]:.4f}"
        f" tilt={cage.bar_tilt_deg(objects['rod']):5.1f}deg"
        f" press={cage.button_travel(objects['button']) * 1000:6.2f}mm"
        f" in_port={_rod_in_port(objects)} score={_TRACE['last']}",
        flush=True,
    )


def score(objects) -> float:
    """Partial credit, every step (``SCORE_OVER = "best"`` keeps the furthest the trial got).

    0.5   the *rod's* end at least 50 mm inside the guard. Only the rod counts: the rule is thin
          enough to pass the port too, so crediting any bar would hand half marks to a policy that
          picked the wrong tool, and choosing the tool is the task. This is the ceiling for
          ``score()`` -- 1.0 comes only from ``success()`` holding for a second, so a momentary
          press cannot latch full marks through ``SCORE_OVER = "best"``.
    0.25  the rod taken off its rack (its centre 30 mm from where it started): the right tool
          was chosen.
    0.0   otherwise.
    """
    value = 0.0
    if any(
        cage.inside_guard(end, TOP) and end[0] >= cage.CAGE_X0 + REACH_M
        for end in cage.bar_ends(objects["rod"], cage.ROD_LEN)
    ):
        value = 0.5
    elif cage.taken_from_rack(objects["rod"], TOP):
        value = 0.25
    if _TRACE["on"]:
        _TRACE["last"] = value
        _trace(objects)
    return value


# -- demo -------------------------------------------------------------------------------

# The rack puts the rod's axis on the port's axis, so the whole job happens at one height: the
# only vertical motion is a 10 mm lift to clear the rack posts.
ROD_Z = cage.bar_rest_z(TOP, cage.ROD_SIDE)  # the shaft's axis, on the port's axis
GRIP_Z = ROD_Z + PAD_TIP_PAST_TCP  # TCP with the pads straddling the rod
LIFT = 0.010  # just enough to clear the rack posts
GRIP_FINGER = cage.finger_for(cage.rod_grip_width() - cage.GRIP_SQUEEZE)  # closed onto the handle
GRIP_X = cage.rod_grip_x()  # the rod is held 210 mm back from its far end, not at its middle
STAGE_X = cage.CAGE_X0 - cage.ROD_AHEAD - 0.010
PRESS_X = cage.BUTTON_FACE_X + 0.012 - cage.ROD_AHEAD
OVERSHOOT = 0.014  # commanded past the spring's resistance. At 6 mm the press settled at only
# 5.3 mm against a 5 mm threshold, which is too thin a margin for a non-deterministic solver.


def demo():
    """Left arm only, in three moves. Reach to the rod and close on it; then one continuous sweep --
    lift off the rack, draw back clear of the guard, curve across to the port's axis, and push the rod
    in until the button is well past 5 mm; then hold.

    ``simkit.patch_arm_plan`` makes the middle move one smooth spline through its via points rather
    than a chain of straight segments, so the arm accelerates once, flows round the corner and
    decelerates onto the button.
    """
    return {
        "left": [
            Waypoint(2.2, (GRIP_X, cage.ROD_Y, GRIP_Z), FINGER_OPEN, "reach to the rod"),
            Waypoint(0.9, (GRIP_X, cage.ROD_Y, GRIP_Z), GRIP_FINGER, "close on it"),
            # One move: same finger target throughout, so it is a single smooth sweep.
            Waypoint(1.0, (GRIP_X, cage.ROD_Y, GRIP_Z + LIFT), GRIP_FINGER, "lift off the rack"),
            Waypoint(1.2, (STAGE_X, cage.ROD_Y, GRIP_Z + LIFT), GRIP_FINGER, "draw back"),
            # The swing is deliberately unhurried: the bar is held 210 mm from its far end, so a
            # brisk lateral move yaws it in the jaws and the tip misses the button.
            Waypoint(2.2, (STAGE_X - 0.005, 0.110, GRIP_Z + LIFT), GRIP_FINGER, "swing across"),
            Waypoint(2.2, (STAGE_X - 0.004, 0.040, GRIP_Z + 0.002), GRIP_FINGER, "swing across"),
            Waypoint(1.8, (STAGE_X, 0.002, GRIP_Z), GRIP_FINGER, "settle on the port's axis"),
            Waypoint(1.6, (STAGE_X, 0.000, GRIP_Z), GRIP_FINGER, "let the bar stop swaying"),
            Waypoint(1.2, (STAGE_X + 0.030, 0.000, GRIP_Z), GRIP_FINGER, "enter the port"),
            Waypoint(3.0, (PRESS_X + OVERSHOOT, 0.000, GRIP_Z), GRIP_FINGER, "push the button"),
            Waypoint(2.5, (PRESS_X + OVERSHOOT, 0.000, GRIP_Z), GRIP_FINGER, "hold it in"),
        ]
    }
