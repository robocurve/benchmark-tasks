"""Scoring for the gladhand coupling task. Pure Python: no simulator needed, so it can be tested on its own.

The simulator (or a person with a photo, a feeler gauge and a spring scale) fills in an :class:`EndState`;
:func:`score` turns it into the trial's score. Every threshold in the task's Scoring text is a constant here.

The hose's gladhand is **locked** to a trailer gladhand when all three hold:
- its connector plate is no more than ``GAP_TOL_MM`` further from that gladhand's plate than when fully locked;
- it has turned to within ``ANGLE_TOL_DEG`` of the lock stop;
- it holds a 60 N pull along its hose for 1 s (in simulation: the locking lug is fully under the flange).

Points: ``OFF_DUMMY_POINTS`` if the hose's gladhand is off its dummy coupler, or ``LOCKED_POINTS`` in place of that
if it is locked to the red trailer gladhand. Plus ``RELEASE_POINTS`` if it is locked there and both grippers have
been clear of every gladhand and the hose for the last ``CLEAR_S`` seconds.

The score is 0 if the hose's gladhand ends locked to the blue trailer gladhand, if the gladhand or hose ever
touched the floor or left the table, or if the hose was ever pulled off its fitting or post.
"""

from __future__ import annotations

from dataclasses import dataclass, field

RIGHT = "red"  # the trailer gladhand the red hose belongs on
TRAILER_COLOURS = ("red", "blue")

GAP_TOL_MM = 2.0  # plate gap beyond the fully locked gap
ANGLE_TOL_DEG = 10.0  # short of the lock stop
OFF_DUMMY_MM = 50.0  # seal centre further than this from the dummy coupler's seal centre
CLEAR_S = 1.0  # both grippers clear of every gladhand and the hose for this long

OFF_DUMMY_POINTS = 0.2
LOCKED_POINTS = 0.8
RELEASE_POINTS = 0.2


@dataclass(frozen=True)
class Coupling:
    """The hose gladhand measured against one trailer gladhand it is pressed onto."""

    gap_excess_mm: float  # plate gap minus the fully locked gap (0 when fully locked)
    angle_from_stop_deg: float  # how far short of the lock stop it is turned (0 at the stop)
    holds_pull: bool  # real trial: stays on under 60 N along the hose for 1 s; simulation: lug fully under the flange


@dataclass(frozen=True)
class Line:
    """The hose's gladhand at the end of the trial."""

    dummy_distance_mm: float  # seal centre to the dummy coupler's seal centre
    couplings: dict[str, Coupling] = field(default_factory=dict)  # by trailer gladhand colour: "red", "blue"


@dataclass(frozen=True)
class EndState:
    line: Line
    clear_s: float = 0.0  # how long both grippers have been clear of every gladhand and the hose, up to the end
    floor_touched: bool = False  # the gladhand or hose touched the floor or left the table, at any time
    hose_detached: bool = False  # the hose was pulled off its fitting or out of its post, at any time


def is_locked(c: Coupling | None) -> bool:
    return bool(
        c is not None
        and c.gap_excess_mm <= GAP_TOL_MM
        and abs(c.angle_from_stop_deg) <= ANGLE_TOL_DEG
        and c.holds_pull
    )


def is_off_dummy(line: Line) -> bool:
    return line.dummy_distance_mm > OFF_DUMMY_MM


def locked_to(line: Line) -> str | None:
    """The trailer gladhand colour the hose gladhand is locked to, if any."""
    for colour in TRAILER_COLOURS:
        if is_locked(line.couplings.get(colour)):
            return colour
    return None


def score(state: EndState) -> float:
    if state.floor_touched or state.hose_detached:
        return 0.0
    target = locked_to(state.line)
    if target is not None and target != RIGHT:
        return 0.0  # on the wrong line
    if target == RIGHT:
        return round(LOCKED_POINTS + (RELEASE_POINTS if state.clear_s >= CLEAR_S else 0.0), 3)
    return OFF_DUMMY_POINTS if is_off_dummy(state.line) else 0.0
