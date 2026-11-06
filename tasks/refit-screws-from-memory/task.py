"""On the station table, a robot arm's worn timing pulley, 16 cm across, sits on an aluminium bench stand in front of the arms, held by four knurled-head screws, with a new pulley on a mat to the
right and an empty mat to the left. Behind the stand is an open aluminium rack with four slots, where a screw hangs by its head.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import refit_usd as u  # noqa: E402
import refitkit as k  # noqa: E402  (hooks yam_arena so the parts below are simulated)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Replace this worn timing pulley with the new one: lift out its four screws and hang them in the rack, put the worn pulley on its mat, set the new pulley on the two pins, "
               "then drop each screw back into the hole it came from, and move both arms away.")
OBJECTS = k.parts()
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

TOP = k.TOP


def _report(objects) -> dict:
    return objects["refit_stand"].report


def success(objects) -> bool:
    """The worn pulley on its mat, the new pulley seated, every hole holding a screw of the right length, the grippers 5 cm clear and everything still. yam_arena then requires this to hold for 1 s."""
    return bool(_report(objects)["success_now"])


def score(objects) -> float:
    """0.1 for the worn pulley on its mat, 0.1 for the new pulley seated, 0.2 for each right hole; a total of 1.0 needs the 1 s hold and scores 0.9 without it. A hole counts only if its screw has
    hung in the rack. A long screw bottoming out in a shallow hole loses that hole for the rest of the run."""
    return _report(objects)["score"]


def distance(objects) -> float:
    """Metres from the new pulley to the stand."""
    x, y, _ = _report(objects)["new"]
    return math.hypot(x - k.STAND_XY[0], y - k.STAND_XY[1])


# -- the scripted demo: open loop, straight-down grips ---------------------------------------

YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # both arms close their fingers along x
HIGH = 0.158  # TCP above the table: a long screw's tip clears the heads of screws in the pulley and in the rack
PULLEY_TOP = u.STAND[2] + u.PULLEY_THICK
HEAD_TO_TCP = 0.014  # with the fingertips 1 mm above what the head stands on
SCREW_OPEN, SCREW_SHUT = -0.016, -0.001  # the 14 mm head: closing 6 mm short of it gives the fingers about 30 N each and keeps it centred
HUB_OPEN, HUB_SHUT = -0.030, -0.011  # the 32 mm hub
HUB_BELOW_TOP = 0.012  # the fingertips this far below the pulley's top
TCP_ABOVE_TIPS = 0.013
LIFTED_HUB = 0.10
# Where the loaded right arm actually puts the new pulley, against the command: it comes up this far short, in metres, and the hub turns this far in the grip, in degrees.
PULLEY_TRIM = (0.0033, 0.0047)
PULLEY_TURN = 1.85
AWAY = {"left": (0.20, 0.30), "right": (0.20, -0.30)}
# Each screw's hole and rack slot.
JOBS = {"left": (("far_left", 0), ("near_left", 1)), "right": (("far_right", 3), ("near_right", 2))}


class Arm:
    """A list of waypoints that knows its own duration and last pose. Long moves along x go by an outer lane, because the two wrists meet if they pass with less than 16 cm between them."""

    def __init__(self, side: str):
        self.side, self.w, self.xy = side, [], None
        self.lane = 0.14 if side == "left" else -0.14

    @property
    def t(self) -> float:
        return sum(x.duration for x in self.w)

    def go(self, d: float, xy, h: float, finger: float, label: str, turn_deg: float = 0.0) -> None:
        self.w.append(Waypoint(d, (xy[0], xy[1], TOP + h), finger, label, YAW[self.side] + math.radians(turn_deg)))
        self.xy = (float(xy[0]), float(xy[1]))

    def travel(self, xy, finger: float, label: str) -> None:
        """At carrying height, to above `xy`, by the outer lane."""
        if self.xy is not None and abs(self.xy[0] - xy[0]) < 0.02:
            self.go(1.0, xy, HIGH, finger, label)
            return
        if self.xy is not None:
            self.go(0.7, (self.xy[0], self.lane), HIGH, finger, label)
        self.go(1.0, (xy[0], self.lane), HIGH, finger, label)
        self.go(0.7, xy, HIGH, finger, label)

    def wait_until(self, t: float) -> None:
        if t > self.t:
            last = self.w[-1]
            self.w.append(Waypoint(t - self.t, last.pos, last.finger, "wait", last.yaw))

    def move_screw(self, a, a_seat: float, b, b_seat: float, length: float, what: str) -> None:
        """From `a` to `b`, each with the height its head stands on. It lowers fast until the tip is just above the hole, steadies, and enters slowly: the arm lags 4 mm sideways at speed."""
        self.travel(a, SCREW_OPEN, f"above {what}")
        self.go(0.3, a, HIGH, SCREW_OPEN, f"steady over {what}")
        self.go(1.6, a, a_seat + HEAD_TO_TCP, SCREW_OPEN, f"down to {what}")
        self.go(0.5, a, a_seat + HEAD_TO_TCP, SCREW_SHUT, f"close on {what}")
        self.go(0.3, a, a_seat + HEAD_TO_TCP, SCREW_SHUT, f"hold {what}")
        self.go(1.8, a, HIGH, SCREW_SHUT, f"lift {what}")
        self.travel(b, SCREW_SHUT, f"carry {what}")
        self.go(0.5, b, HIGH, SCREW_SHUT, f"steady {what}")
        tip_above = b_seat + length + 0.006 + HEAD_TO_TCP
        self.go(1.4, b, tip_above, SCREW_SHUT, f"lower {what}")
        self.go(0.7, b, tip_above, SCREW_SHUT, f"steady {what} over the hole")
        self.go(1.5, b, b_seat + 0.003 + HEAD_TO_TCP, SCREW_SHUT, f"enter {what}")
        self.go(0.4, b, b_seat + 0.003 + HEAD_TO_TCP, SCREW_OPEN, f"let go of {what}")
        self.go(0.3, b, b_seat + 0.003 + HEAD_TO_TCP, SCREW_OPEN, f"let {what} settle")
        self.go(0.9, b, HIGH, SCREW_OPEN, f"up from {what}")

    def move_pulley(self, a, a_bottom: float, b, b_bottom: float, what: str, arrive_after: float = 0.0, turn_deg: float = 0.0) -> None:
        """By the hub, from `a` to `b`, each with the height its underside rests on."""
        grip_a = a_bottom + u.PULLEY_THICK - HUB_BELOW_TOP + TCP_ABOVE_TIPS
        grip_b = b_bottom + u.PULLEY_THICK - HUB_BELOW_TOP + TCP_ABOVE_TIPS
        lift = LIFTED_HUB
        self.go(1.8, a, HIGH, HUB_OPEN, f"above {what}")
        self.go(0.7, a, HIGH, HUB_OPEN, f"steady over {what}")
        self.go(1.6, a, grip_a, HUB_OPEN, f"down to {what}")
        self.go(0.6, a, grip_a, HUB_SHUT, f"close on {what}")
        self.go(1.2, a, lift, HUB_SHUT, f"lift {what}")
        self.go(2.6, b, lift, HUB_SHUT, f"carry {what}", turn_deg)
        self.wait_until(arrive_after)
        self.go(0.5, b, lift, HUB_SHUT, f"steady {what}", turn_deg)
        self.go(1.8, b, grip_b + 0.002, HUB_SHUT, f"lower {what}", turn_deg)
        self.go(0.5, b, grip_b + 0.002, HUB_OPEN, f"let go of {what}", turn_deg)
        self.go(0.9, b, HIGH, HUB_OPEN, f"up from {what}")


def _slot(i: int) -> tuple[float, float]:
    return k.RACK_XY[0], k.RACK_XY[1] + u.SLOTS_Y[i]


TURN = 7.5  # seconds between one arm starting a screw and the other: one is always at the rack while the other is at the holes


def demo() -> dict[str, list[Waypoint]]:
    """`REFIT_DEMO=safe` (default): each arm takes its screws to the rack, the left arm moves the worn pulley to its mat, the right arm sets the new pulley on the pins, and the screws go back to
    the holes they came from. `swapped`: the left arm returns its two screws to each other's holes. `still`: both arms stay away."""
    mode = os.environ.get("REFIT_DEMO", "safe")
    if mode not in ("safe", "swapped", "still"):
        raise ValueError(f"REFIT_DEMO must be safe, swapped or still, got {mode!r}")
    left, right = Arm("left"), Arm("right")
    if mode == "still":
        left.go(1.0, AWAY["left"], HIGH, FINGER_OPEN, "away")
        right.go(1.0, AWAY["right"], HIGH, FINGER_OPEN, "away")
        return {"left": left.w, "right": right.w}
    rack_top = u.RACK_TOP_H + k.REST_GAP
    pulley_top = PULLEY_TOP + 2 * k.REST_GAP

    def length(arm: Arm, job: int) -> float:
        return k.SCREW_LENGTH[k.SCREW_NAME[JOBS[arm.side][job][0]]]

    def out(arm: Arm, job: int) -> None:
        hole, slot = JOBS[arm.side][job]
        arm.move_screw(tuple(k.hole_world(hole)), pulley_top, _slot(slot), rack_top, length(arm, job), f"screw from {hole}")

    def back(arm: Arm, job: int, hole_job: int) -> None:
        slot = JOBS[arm.side][job][1]
        hole = JOBS[arm.side][hole_job][0]
        arm.move_screw(_slot(slot), rack_top, tuple(k.hole_world(hole)), pulley_top, length(arm, job), f"screw to {hole}")

    out(left, 0)
    right.go(1.5, AWAY["right"], HIGH, FINGER_OPEN, "away")
    right.wait_until(TURN)
    out(right, 0)
    left.wait_until(2 * TURN)
    out(left, 1)
    left.go(1.5, AWAY["left"], HIGH, FINGER_OPEN, "away")
    right.wait_until(3 * TURN)
    out(right, 1)
    right.go(1.5, AWAY["right"], HIGH, FINGER_OPEN, "away")
    # The pulleys, once every screw is out of the worn one. The new one arrives after the worn one has left.
    left.wait_until(3 * TURN + 6.9)
    left.move_pulley(k.STAND_XY, u.STAND[2], k.WORN_MAT_XY, u.MAT_H, "worn pulley")
    left.go(1.2, AWAY["left"], HIGH, FINGER_OPEN, "away")
    right.move_pulley(k.NEW_MAT_XY, u.MAT_H, (k.STAND_XY[0] + PULLEY_TRIM[0], k.STAND_XY[1] + PULLEY_TRIM[1]), u.STAND[2], "new pulley", arrive_after=left.t, turn_deg=-PULLEY_TURN)
    right.go(1.2, AWAY["right"], HIGH, FINGER_OPEN, "away")
    # The screws back, taking turns again.
    swap = mode == "swapped"
    start = right.t
    left.wait_until(start)
    back(left, 0, 1 if swap else 0)
    # Arms this close have their wrists meet, so each waits away while the other works at a far hole.
    left.go(1.5, AWAY["left"], HIGH, FINGER_OPEN, "away")
    right.wait_until(start + TURN)
    back(right, 0, 0)
    right.go(1.5, AWAY["right"], HIGH, FINGER_OPEN, "away")
    left.wait_until(right.t)
    last_turn = left.t
    back(left, 1, 0 if swap else 1)
    left.go(1.5, AWAY["left"], HIGH, FINGER_OPEN, "away")
    right.wait_until(left.t if swap else last_turn + TURN)
    back(right, 1, 1)
    right.go(1.5, AWAY["right"], HIGH, FINGER_OPEN, "away")
    return {"left": left.w, "right": right.w}
