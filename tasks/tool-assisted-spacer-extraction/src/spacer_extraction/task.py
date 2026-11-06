"""Robot assembly fixture: remove the spacer using the long pusher.

A blue aluminium spacer sits inside a fixed grey covered channel near
(0.350,0.100). Its service path runs along +x. An orange rigid pusher starts
in its parking outline at (0.270,0.340). The green recovery nest is at
(0.350,0.260). Dimensions and positions are in metres, x forward and y left.
Use the pusher to move the separate spacer through the service path, recover
the spacer to the nest, return the pusher, and open both grippers clear.
"""

from __future__ import annotations

from typing import Any, Mapping

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

from spacer_extraction.tracker import SpacerScoreSnapshot, SpacerTrialTracker

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the per-step tracker requires its own continuous 1.0 s released dwell
SCORE_OVER = "end"  # score is determined from the final state, never stale best-so-far credit

CHANNEL_START_X = 0.330
CHANNEL_END_X = 0.370
CHANNEL_CENTER_X = (CHANNEL_START_X + CHANNEL_END_X) / 2
CHANNEL_CENTER_Y = 0.100
SPACER_START = (0.351, CHANNEL_CENTER_Y, TOP + 0.012)
# Park the rod outside the nest. The robot grips the rear 20 mm of the rod;
# this keeps its wrist and camera clear of the channel roof during pushing.
PUSHER_START = (0.270, 0.340, TOP + 0.008)
PUSHER_GRASP_X = PUSHER_START[0] - 0.110
NEST_CENTER = (0.350, 0.260, TOP + 0.012)

CHANNEL_WALLS = [
    Box(
        "channel_left_wall",
        pos=(CHANNEL_CENTER_X, CHANNEL_CENTER_Y - 0.018, TOP + 0.015),
        size=(0.040, 0.008, 0.030),
        color=(0.38, 0.42, 0.47),
    ),
    Box(
        "channel_right_wall",
        pos=(CHANNEL_CENTER_X, CHANNEL_CENTER_Y + 0.018, TOP + 0.015),
        size=(0.040, 0.008, 0.030),
        color=(0.38, 0.42, 0.47),
    ),
]

# Roof rails cap the 28 mm channel with a 4 mm center viewing slit. The spacer
# clears the channel at TOP+30 mm; the pusher's 16 mm square section slides
# beneath the roof without being attached to either part.
CHANNEL_ROOF = [
    Box(
        "channel_left_roof",
        pos=(CHANNEL_CENTER_X, CHANNEL_CENTER_Y - 0.008, TOP + 0.034),
        size=(0.040, 0.012, 0.008),
        color=(0.66, 0.70, 0.74),
    ),
    Box(
        "channel_right_roof",
        pos=(CHANNEL_CENTER_X, CHANNEL_CENTER_Y + 0.008, TOP + 0.034),
        size=(0.040, 0.012, 0.008),
        color=(0.66, 0.70, 0.74),
    ),
]

# The nest is open from above and leaves 8 mm/7 mm clearance around the spacer.
NEST_RAILS = [
    Box("nest_xminus_rail", pos=(NEST_CENTER[0] - 0.024, NEST_CENTER[1], TOP + 0.004), size=(0.004, 0.044, 0.008), color=(0.22, 0.52, 0.34)),
    Box("nest_xplus_rail", pos=(NEST_CENTER[0] + 0.024, NEST_CENTER[1], TOP + 0.004), size=(0.004, 0.044, 0.008), color=(0.22, 0.52, 0.34)),
    Box("nest_yminus_rail", pos=(NEST_CENTER[0], NEST_CENTER[1] - 0.022, TOP + 0.004), size=(0.044, 0.004, 0.008), color=(0.22, 0.52, 0.34)),
    Box("nest_yplus_rail", pos=(NEST_CENTER[0], NEST_CENTER[1] + 0.022, TOP + 0.004), size=(0.044, 0.004, 0.008), color=(0.22, 0.52, 0.34)),
]

# Low, narrow printed guide marks sit flush with the table surface, outside
# the pusher footprint and channel. They make its initial parking pose and
# 40 mm extraction distance inspectable without adding a mechanical stop.
PUSHER_PARK_MARKS = [
    Box("pusher_park_mark_yminus", pos=(PUSHER_START[0], PUSHER_START[1] - 0.018, TOP + 0.00025), size=(0.264, 0.001, 0.0005), color=(0.25, 0.28, 0.32)),
    Box("pusher_park_mark_yplus", pos=(PUSHER_START[0], PUSHER_START[1] + 0.018, TOP + 0.00025), size=(0.264, 0.001, 0.0005), color=(0.25, 0.28, 0.32)),
]

# A graduated reference beside the channel shows its 40 mm length. Markings
# are low profile and outside the outer wall edge at y=0.078 m.
EXTRACTION_MARKS = [
    Box("extraction_mark_start", pos=(CHANNEL_START_X, 0.064, TOP + 0.00025), size=(0.001, 0.008, 0.0005), color=(0.85, 0.35, 0.12)),
    Box("extraction_mark_end", pos=(CHANNEL_END_X, 0.064, TOP + 0.00025), size=(0.001, 0.008, 0.0005), color=(0.85, 0.35, 0.12)),
]

pusher = Box(
    "pusher",
    pos=PUSHER_START,
    size=(0.260, 0.016, 0.016),
    mass=0.060,
    color=(0.94, 0.48, 0.16),
)
spacer = Box(
    "spacer",
    pos=SPACER_START,
    size=(0.028, 0.024, 0.024),
    # Solid 28 x 24 x 24 mm aluminium: 16.128 cm^3 * 2.70 g/cm^3 = 43.55 g.
    mass=0.04355,
    color=(0.15, 0.42, 0.82),
)

OBJECTS = [*CHANNEL_WALLS, *CHANNEL_ROOF, *NEST_RAILS, *PUSHER_PARK_MARKS, *EXTRACTION_MARKS, pusher, spacer]

_TRACKER = SpacerTrialTracker()


def reset_tracker() -> None:
    """Reset per-episode event history; called by the task-local runtime adapter."""
    _TRACKER.reset()


def evaluate(
    objects: Mapping[str, Any], *, time_s: float | None = None, released: bool = False
) -> SpacerScoreSnapshot:
    """Update and return the episode snapshot (one call per control frame)."""
    return _TRACKER.update(objects, time_s=time_s, released=released)


def score(objects: Mapping[str, Any]) -> float:
    """Read the wrapper's last score snapshot; unwrapped stock runtime safely scores 0."""
    return _TRACKER.last.score


def success(objects: Mapping[str, Any]) -> bool:
    """End only after verified ejection, recovery, parked tool, release and one-second dwell."""
    return _TRACKER.last.success_ready


def distance(objects: Mapping[str, Any]) -> float:
    """Approximate remaining displacement to the next geometric subgoal, in metres."""
    state = _TRACKER.last
    if not state.mediated_push:
        tool = objects["pusher"]
        spacer_state = objects["spacer"]
        face_gap = spacer_state.pos[0] - 0.014 - (tool.pos[0] + 0.130)
        return float(max(face_gap, 0.0) + max(0.040 - state.verified_push_m, 0.0))
    if not state.ejected:
        return float(max(CHANNEL_END_X + 0.014 - objects["spacer"].pos[0], 0.0))
    if not state.recovered:
        import numpy as np

        return float(np.linalg.norm(objects["spacer"].pos - NEST_CENTER))
    if not state.tool_parked:
        import numpy as np

        return float(np.linalg.norm(objects["pusher"].pos - PUSHER_START))
    return max(1.0 - state.stable_s, 0.0)


def _tcp_z(object_center_z: float) -> float:
    """YAM's scripted gripper TCP sits PAD_TIP_PAST_TCP above the contacted object centre."""
    return object_center_z + PAD_TIP_PAST_TCP


def demo() -> dict[str, list[Waypoint]]:
    """Left arm: grasp and push with the long tool, park it, then recover the spacer."""
    hover_z = TOP + 0.120
    push_contact_z = _tcp_z(TOP + 0.008)
    spacer_grasp_z = _tcp_z(TOP + 0.012)
    # Measured compliance/yaw under this off-centre load makes the bar tip sit
    # about 25 mm to the left of the commanded TCP line. This fixed open-loop
    # compensation was selected from the actual failed trial's raw trace.
    push_grip_y = CHANNEL_CENTER_Y - 0.030
    park_grip_x = PUSHER_GRASP_X + 0.020
    # Compensate the measured ~6 mm rearward deflection while lowering the
    # held spacer; the object target remains the centre of the fixed nest.
    nest_grip_x = NEST_CENTER[0] + 0.007
    return {
        "left": [
            Waypoint(1.2, (PUSHER_GRASP_X, PUSHER_START[1], hover_z), FINGER_OPEN, "approach rear pusher grip"),
            Waypoint(1.2, (PUSHER_GRASP_X, PUSHER_START[1], push_contact_z), FINGER_OPEN, "lower onto rear pusher grip"),
            Waypoint(0.7, (PUSHER_GRASP_X, PUSHER_START[1], push_contact_z), FINGER_CLOSED, "grasp pusher near rear end"),
            Waypoint(1.0, (PUSHER_GRASP_X, PUSHER_START[1], hover_z), FINGER_CLOSED, "lift pusher"),
            Waypoint(1.4, (0.080, push_grip_y, hover_z), FINGER_CLOSED, "align grip before channel"),
            Waypoint(1.0, (0.080, push_grip_y, push_contact_z), FINGER_CLOSED, "lower pusher into guide"),
            Waypoint(4.0, (0.180, push_grip_y, push_contact_z), FINGER_CLOSED, "push spacer through channel"),
            Waypoint(0.5, (0.180, push_grip_y, push_contact_z), FINGER_CLOSED, "complete ejection"),
            Waypoint(1.6, (0.080, push_grip_y, push_contact_z), FINGER_CLOSED, "withdraw pusher"),
            Waypoint(1.0, (0.080, push_grip_y, hover_z), FINGER_CLOSED, "lift pusher clear"),
            Waypoint(1.4, (park_grip_x, PUSHER_START[1], hover_z), FINGER_CLOSED, "return pusher to park"),
            Waypoint(1.0, (park_grip_x, PUSHER_START[1], push_contact_z), FINGER_CLOSED, "lower pusher to park"),
            Waypoint(0.7, (park_grip_x, PUSHER_START[1], push_contact_z), FINGER_OPEN, "release pusher"),
            Waypoint(0.8, (park_grip_x, PUSHER_START[1], hover_z), FINGER_OPEN, "clear parked pusher"),
            Waypoint(1.2, (0.406, CHANNEL_CENTER_Y, hover_z), FINGER_OPEN, "approach ejected spacer"),
            Waypoint(1.0, (0.406, CHANNEL_CENTER_Y, spacer_grasp_z), FINGER_OPEN, "lower onto spacer"),
            Waypoint(0.7, (0.406, CHANNEL_CENTER_Y, spacer_grasp_z), FINGER_CLOSED, "grasp ejected spacer"),
            Waypoint(1.0, (0.406, CHANNEL_CENTER_Y, hover_z), FINGER_CLOSED, "lift spacer"),
            Waypoint(1.5, (nest_grip_x, NEST_CENTER[1], hover_z), FINGER_CLOSED, "carry spacer to nest"),
            Waypoint(1.5, (nest_grip_x, NEST_CENTER[1], spacer_grasp_z), FINGER_CLOSED, "lower spacer into nest"),
            Waypoint(0.5, (nest_grip_x, NEST_CENTER[1], spacer_grasp_z), FINGER_CLOSED, "settle spacer in nest"),
            Waypoint(0.7, (nest_grip_x, NEST_CENTER[1], spacer_grasp_z), FINGER_OPEN, "release spacer"),
            Waypoint(0.8, (nest_grip_x, NEST_CENTER[1], hover_z), FINGER_OPEN, "clear recovery nest"),
        ]
    }
