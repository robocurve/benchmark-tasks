"""On the station table, between the arms, a ground-fault outlet stands on a grey pedestal: a light
faceplate carrying a dark rocker resting level on a fulcrum ridge, one end toward each arm. This
outlet is beside the fixture area a faucet would occupy: within six feet of a water source, exactly
where a ground-fault interrupter is required.

A ground-fault outlet is verified by its sequence: press the TEST end down (the trip — the RESET end
rises), then press the RESET end down to restore it. Pressing RESET without a test proves nothing.

Run it:  inspect-robots run --embodiment yam_arena -E task=gfci_test_reset.py \
           --instruction "test the GFCI outlet: press TEST, then press RESET" --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import TIPS_ON_TABLE, Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

PEDESTAL = Box("pedestal", pos=(0.30, 0.0, TOP + 0.06), size=(0.16, 0.10, 0.12), color=(0.45, 0.47, 0.50))
FACE = Box("faceplate", pos=(0.30, 0.0, TOP + 0.125), size=(0.16, 0.14, 0.01), color=(0.88, 0.88, 0.88))
RIDGE = Box("fulcrum", pos=(0.30, 0.0, TOP + 0.135), size=(0.06, 0.012, 0.01), color=(0.60, 0.62, 0.65))
_FACE_TOP = TOP + 0.13                      # faceplate top; the fulcrum sits on it
_ROCKER_REST_Z = _FACE_TOP + 0.01 + 0.007   # on the fulcrum (1 cm tall) + half the rocker's thickness

ROCKER_HALF = 0.045                         # rocker half-length along y (m); TEST end at +y, RESET at -y
# The rocker is 5.2 cm across so a closed parallel jaw (4.8 cm) presses ON its top face rather than
# straddling it and hitting nothing.
ROCKER = Box("rocker", pos=(0.30, 0.0, _ROCKER_REST_Z), size=(0.052, 2 * ROCKER_HALF, 0.014),
             mass=0.08, color=(0.20, 0.20, 0.20))
WALL_L = Box("wall_left", pos=(0.265, 0.0, TOP + 0.145), size=(0.008, 0.10, 0.028), color=(0.75, 0.75, 0.75))
WALL_R = Box("wall_right", pos=(0.335, 0.0, TOP + 0.145), size=(0.008, 0.10, 0.028), color=(0.75, 0.75, 0.75))

OBJECTS = [PEDESTAL, FACE, RIDGE, ROCKER, WALL_L, WALL_R]

TILT_DIP = 0.003                            # end displacement that counts as pressed (m)
TILT_SIN = TILT_DIP / ROCKER_HALF
LEVEL_SIN = 0.002 / ROCKER_HALF             # within ~2.5 deg of level counts as level

# Sequence state machine (module state; re-arms after a completed sequence returns to level).
_state = {"test_seen": False, "reset_after_test": False}


def _tilt_sin(state):
    """Signed tilt: negative = TEST (+y) end down, positive = RESET (-y) end down. It is the z
    component of the rocker's local y axis, R[2][1] = 2(yz + wx): pressing the +y end down rotates
    the box by a negative angle about +x, so this reads negative."""
    x, y, z, w = state.quat_xyzw
    return 2.0 * (y * z + w * x)


def _update(objects):
    s = _tilt_sin(objects["rocker"])
    if _state["reset_after_test"] and abs(s) < LEVEL_SIN:
        _state["test_seen"] = False                 # a completed sequence, back at level: re-arm
        _state["reset_after_test"] = False
        return
    if s < -TILT_SIN:
        _state["test_seen"] = True                  # the TEST end was pressed down (the trip)
    if _state["test_seen"] and s > TILT_SIN:
        _state["reset_after_test"] = True           # then the RESET end was pressed down


def success(objects):
    """The RESET end is being held down after a TEST press: the outlet was tested and restored. No
    resting() check: the scripted plans end ~10 steps after the press settles, so the hold budget is
    tight; a flick still has to keep the tilt past the threshold for SUCCESS_HOLD_S."""
    _update(objects)
    return bool(_state["reset_after_test"]
                and _tilt_sin(objects["rocker"]) > TILT_SIN)


def distance(objects):
    """0.0 when the sequence is complete; otherwise the RESET end's remaining press depth (m)."""
    _update(objects)
    s = _tilt_sin(objects["rocker"])
    if _state["reset_after_test"]:
        return 0.0
    return max(0.0, TILT_DIP - max(0.0, s) * ROCKER_HALF)


def score(objects):
    """0.5 for the trip (TEST pressed first); 1.0 for the full ordered test-and-reset sequence."""
    _update(objects)
    if _state["reset_after_test"]:
        return 1.0
    return 0.5 if _state["test_seen"] else 0.0


SCORE_OVER = "best"     # the sequence completes transiently (the rocker then rests level); keep the best

SUCCESS_HOLD_S = 0.4    # the scripted plans end ~15 steps after the press crosses the threshold


def _press_waypoints(py, start_delay=0.0):
    """One arm approaches its rocker end horizontally at press height (high hovers over the table's
    centre are outside the arm's workspace), presses ~8 mm past level and holds."""
    contact_z = TIPS_ON_TABLE + (_ROCKER_REST_Z + 0.007 - 0.75)   # pad tips at the level end's top
    press_z = contact_z - 0.012                                   # deep press: the pad slips a little
    wps = []
    if start_delay:
        wps.append(Waypoint(start_delay, (0.30, py + 0.12, contact_z), -0.023, "wait for the other arm"))
    wps += [
        Waypoint(1.8, (0.30, py + 0.12, contact_z), -0.023, "approach at press height"),
        Waypoint(1.2, (0.30, py, contact_z), -0.023, "over the end"),
        Waypoint(0.8, (0.30, py, press_z), -0.023, "press"),
        Waypoint(1.4, (0.30, py, press_z), -0.023, "hold the press"),
        Waypoint(1.0, (0.30, py + 0.12, contact_z), -0.023, "release and back off"),
    ]
    return wps


def demo():
    """The left arm presses the TEST end down (the trip); the right arm then presses the RESET end to
    restore the outlet. Each arm works from its own side of the rocker."""
    left_dur = 1.8 + 1.2 + 0.8 + 1.4 + 1.0
    return {
        "left": _press_waypoints(+0.038),
        "right": _press_waypoints(-0.038, start_delay=left_dur + 1.0),
    }
