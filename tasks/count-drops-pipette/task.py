"""On the station table, a glass pipette with a red rubber bulb rests in its holder, the ring of a lab stand, a clear glass test tube stands in an open steel rack, and a cork stands near the right arm.
The pipette and the tube hold the same blue chemical. The left arm carries the pipette by its bulb, counts out three drops into the tube and stands it back. The right arm corks the tube, lifts it,
shakes it from side to side four times, like a bell, and stands it back.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import dropkit  # noqa: E402  (hooks yam_arena so the parts below are simulated)
from dropkit import CORK_SEATS_AT, CORK_XY, HOLDER_XY, N_DROPS, N_SWINGS, RACK_PLATE, RING_TOP, START_Z, TIP_BELOW, TOP, TUBE_H, TUBE_XY  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
INSTRUCTION = (
    "Add exactly 3 drops of the chemical in the pipette to the test tube. The chemical is already in the pipette. Stand the pipette back in its holder. "
    "Then put the cork in the tube, lift the tube out of the rack, shake it from side to side with your wrist, like a bell, exactly 4 times, stand it back in the rack and let go."
)
OBJECTS = dropkit.parts()
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

OPEN = -0.015  # each finger 1.5 cm from the middle, round the 1.4 cm bulb
HOLD = -0.0055  # a light grip: about 3 N a finger, the bulb about 12 mm wide
SQUEEZE = -0.0015  # the pads reach their stops, the bulb 7 mm wide
RIM = TOP + RACK_PLATE + TUBE_H
CARRY = TOP + 0.2  # the tip over a centimetre above the tube's rim
OVER_RIM = RIM + 0.005 + TIP_BELOW  # the tip 5 mm above the tube's rim
PICK = START_Z
ABOVE_RING = TOP + RING_TOP + 0.006 + TIP_BELOW  # the tip just above the ring
PARKED_LEFT = (0.26, 0.31)
PARKED_RIGHT = (0.26, -0.31)
HOVER = TOP + 0.15

OPEN_WIDE = -0.018  # round the cork's 23 mm top or the tube, with room
GRIP_CORK = -0.008  # short of its 11.5 mm half width: about 17 N a finger
GRIP_TUBE = -0.007  # short of its 10 mm half width: about 15 N a finger
CORK_PINCH_OVER_BOTTOM = 0.024  # TCP over the cork's bottom: the fingertips stay above the rim as it seats
CORK_LIFT = TOP + 0.145
TUBE_PINCH = TOP + 0.075  # low enough to leave the cork alone; the fingertips above the rack's plate
SWINGING = TOP + 0.15
TUBE_BACK_OFFSET = (0.002, 0.003)  # the right arm stands this far off its target with the tube, from a logged run
LEAN_DEG = 30.0
LEAN_AXIS_RIGHT = (0.56, 0.83, 0.0)  # along the right arm from its base: a tilt about this takes only the wrist's own joint, and either way is in reach at this height


def _report(objects) -> dict:
    return objects["pipette"].report


def success(objects) -> bool:
    """Exactly three drops in the tube, the pipette back in its holder, the cork in, four shakes, the tube back in its rack, the fingers clear, all still."""
    return bool(_report(objects)["ready"])


def score(objects) -> float:
    """See the scoring text in task.json: dropkit.task_score applies it."""
    return _report(objects)["score"]


def _wp(duration, xy, z, finger, label) -> Waypoint:
    return Waypoint(duration, (float(xy[0]), float(xy[1]), float(z)), finger, label, 0.0)


def _lean(duration, xy, z, finger, label, degrees) -> Waypoint:
    return Waypoint(duration, (float(xy[0]), float(xy[1]), float(z)), finger, label, dropkit.lean(LEAN_AXIS_RIGHT, degrees))


def demo() -> dict[str, list[Waypoint]]:
    """`DROP_COUNT` squeezes and `SWING_COUNT` shakes from side to side, 3 and 4 by default. Open loop, from the scene's starting layout; heights are of the TCP. The plan holds both counts as numbers.
    `DROP_MODE` runs the same moves with a fault: `hard` grips the bulb hard in the holder, `spill` also squeezes once on the way to the tube, `dropped` lets go on the way,
    `early_lift` lifts the tube before corking it, `no_tube` stops after the cork."""
    drops = int(os.environ.get("DROP_COUNT", N_DROPS))
    swings = int(os.environ.get("SWING_COUNT", N_SWINGS))
    mode = os.environ.get("DROP_MODE", "")
    if mode not in ("", "hard", "spill", "dropped", "early_lift", "no_tube"):
        raise ValueError(f"DROP_MODE must be hard, spill, dropped, early_lift or no_tube, got {mode!r}")
    midway = (0.23, 0.08)
    left = [
        _wp(2.0, HOLDER_XY, PICK + 0.05, OPEN, "above the bulb"),
        _wp(1.4, HOLDER_XY, PICK, OPEN, "down to the bulb"),
        _wp(0.8, HOLDER_XY, PICK, SQUEEZE if mode == "hard" else HOLD, "pinch"),
        _wp(0.8, HOLDER_XY, PICK, HOLD, "ease to a light pinch"),
        _wp(3.0, HOLDER_XY, CARRY, HOLD, "lift it out"),
    ]
    if mode == "dropped":
        left += [_wp(1.4, midway, CARRY, HOLD, "on the way"), _wp(0.5, midway, CARRY, OPEN, "let it fall"), _wp(1.5, PARKED_LEFT, HOVER, OPEN, "clear"), _wp(3.0, PARKED_LEFT, HOVER, OPEN, "wait")]
        return {"left": left, "right": [_wp(sum(w.duration for w in left), PARKED_RIGHT, HOVER, OPEN, "stay clear")]}
    if mode == "spill":
        left += [_wp(1.4, midway, CARRY, HOLD, "on the way"), _wp(0.6, midway, CARRY, SQUEEZE, "squeeze away from the tube"), _wp(0.6, midway, CARRY, HOLD, "release")]
    left += [
        _wp(2.0, TUBE_XY, CARRY, HOLD, "over the tube"),
        _wp(1.2, TUBE_XY, OVER_RIM, HOLD, "down to the rim"),
        _wp(0.8, TUBE_XY, OVER_RIM, HOLD, "settle"),
    ]
    for i in range(drops):
        left += [
            _wp(0.6, TUBE_XY, OVER_RIM, SQUEEZE, f"squeeze {i}"),
            _wp(0.2, TUBE_XY, OVER_RIM, SQUEEZE, f"squeezed {i}"),
            _wp(0.6, TUBE_XY, OVER_RIM, HOLD, f"release {i}"),
            _wp(0.3, TUBE_XY, OVER_RIM, HOLD, f"released {i}"),
        ]
    left += [
        _wp(1.2, TUBE_XY, CARRY, HOLD, "up from the rim"),
        _wp(2.0, HOLDER_XY, CARRY, HOLD, "back over the ring"),
        _wp(1.6, HOLDER_XY, ABOVE_RING, HOLD, "down to the ring"),
        _wp(1.0, HOLDER_XY, ABOVE_RING, HOLD, "settle over the ring"),
        _wp(3.0, HOLDER_XY, PICK + 0.001, HOLD, "down into the ring"),
        _wp(0.5, HOLDER_XY, PICK + 0.001, HOLD, "settle in the ring"),
        _wp(0.8, HOLDER_XY, PICK + 0.001, OPEN, "let go"),
        _wp(1.4, HOLDER_XY, PICK + 0.10, OPEN, "rise"),
        _wp(1.2, PARKED_LEFT, HOVER, OPEN, "clear"),
    ]
    wait = sum(w.duration for w in left)
    cork_pick = TOP + CORK_PINCH_OVER_BOTTOM
    cork_seat = RIM - CORK_SEATS_AT + CORK_PINCH_OVER_BOTTOM
    right = [_wp(wait, PARKED_RIGHT, HOVER, OPEN_WIDE, "wait for the left arm")]
    tube_up = [
        _wp(1.0, TUBE_XY, TOP + 0.16, OPEN_WIDE, "above the tube"),
        _wp(1.6, TUBE_XY, TUBE_PINCH, OPEN_WIDE, "down round the tube"),
        _wp(0.6, TUBE_XY, TUBE_PINCH, GRIP_TUBE, "pinch the tube"),
        _wp(0.3, TUBE_XY, TUBE_PINCH, GRIP_TUBE, "hold the tube"),
        _wp(2.4, TUBE_XY, SWINGING, GRIP_TUBE, "lift the tube"),
        _wp(0.4, TUBE_XY, SWINGING, GRIP_TUBE, "settle"),
    ]
    if mode != "early_lift":
        right += [
            _wp(1.8, CORK_XY, TOP + 0.09, OPEN_WIDE, "above the cork"),
            _wp(1.2, CORK_XY, cork_pick, OPEN_WIDE, "down to the cork"),
            _wp(0.6, CORK_XY, cork_pick, GRIP_CORK, "pinch the cork"),
            _wp(0.3, CORK_XY, cork_pick, GRIP_CORK, "hold the cork"),
            _wp(1.2, CORK_XY, CORK_LIFT, GRIP_CORK, "lift the cork"),
            _wp(2.4, TUBE_XY, CORK_LIFT, GRIP_CORK, "over the tube"),
            _wp(0.8, TUBE_XY, CORK_LIFT, GRIP_CORK, "settle over the tube"),
            _wp(1.6, TUBE_XY, cork_seat + 0.003, GRIP_CORK, "lower it into the bore"),
            _wp(0.4, TUBE_XY, cork_seat + 0.003, GRIP_CORK, "settle"),
            _wp(0.6, TUBE_XY, cork_seat + 0.003, OPEN_WIDE, "let the cork go"),
        ]
    if mode == "no_tube":
        right += [_wp(1.2, TUBE_XY, TOP + 0.16, OPEN_WIDE, "rise"), _wp(1.5, PARKED_RIGHT, HOVER, OPEN_WIDE, "clear")]
        return {"left": left, "right": right}
    right += tube_up
    tube_back = (TUBE_XY[0] + TUBE_BACK_OFFSET[0], TUBE_XY[1] + TUBE_BACK_OFFSET[1])
    for i in range(swings):
        right += [_lean(0.8 if i == 0 else 1.2, TUBE_XY, SWINGING, GRIP_TUBE, f"swing {i} to one side", LEAN_DEG), _lean(1.2, TUBE_XY, SWINGING, GRIP_TUBE, f"swing {i} to the other side", -LEAN_DEG)]
    right += [
        _lean(0.8, TUBE_XY, SWINGING, GRIP_TUBE, "back to upright", 0.0),
        _wp(1.0, tube_back, SWINGING, GRIP_TUBE, "settle"),
        _wp(3.6, tube_back, TUBE_PINCH + 0.001, GRIP_TUBE, "lower it into the rack"),
        _wp(0.6, tube_back, TUBE_PINCH + 0.001, GRIP_TUBE, "settle in the rack"),
        _wp(0.6, tube_back, TUBE_PINCH + 0.001, OPEN_WIDE, "let the tube go"),
        _wp(1.6, TUBE_XY, TOP + 0.16, OPEN_WIDE, "rise"),
        _wp(1.5, PARKED_RIGHT, HOVER, OPEN_WIDE, "clear"),
    ]
    return {"left": left, "right": right}
