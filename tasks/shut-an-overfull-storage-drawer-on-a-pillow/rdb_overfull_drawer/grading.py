"""Score the trial from where the drawer, the pillow and the grippers are at the end. NumPy only.

Every rule can be judged with a ruler, a photo from the front and the trial video:

- open: the gap between the frame's front face and the outer face of the drawer front, in mm;
- pillow in the drawer: the whole pillow lies inside the drawer's walls, on its floor;
- shut: the drawer front is within 5 mm of the frame's front face, the pillow is in the drawer and no part
  of the pillow shows outside the frame;
- hands clear: no part of the robot touches the unit or the pillow and each gripper's centre point (midway
  between its fingertips) is at least 5 cm from the frame, the drawer and its pull.

Score at the end of the trial: 0 if the pillow is not in the drawer, or the unit has been shifted more than
10 mm on the table; otherwise 1.0 if the drawer has been shut with hands clear for 2 s, 0.75 if it is shut,
0.5 if it is open 90 mm or less, 0.25 if it is open 190 mm or less, else 0.
"""

from dataclasses import dataclass

import numpy as np

from . import layout

try:  # the Inspect Robots scorer wrapper is only needed inside a trial
    from inspect_robots.scorer import Score
except ImportError:  # pragma: no cover
    Score = None

CONTROL_HZ = 20
HOLD_S = 2
HOLD_TICKS = CONTROL_HZ * HOLD_S
SHUT_M = .005                # drawer front within this of the frame's front face
MOSTLY_M = .090              # every bulging part of the pillow is under the frame's top by here (the last one catches at 112 mm)
STARTED_M = .190             # the first bulging part is under the frame's top by here (it catches at 214 mm)
CLEAR_M = .050               # gripper centre point to the unit
TOUCH_N = .05                # contact force that counts as touching
SHOWS_M = .003               # a pad this far above the underside of the frame's top is outside the frame
UNIT_SHIFT_M = .010          # the unit pushed or pulled this far on the table: 0
PLAN_SLACK_M = .012          # pillow's middle may be this far off the drawer's middle
LIFTED_M = .020              # pillow's underside this far above the drawer's floor: not lying in the drawer
TILT_COS = float(np.cos(np.radians(20.)))


def pillow_in_drawer(base_position, base_up, open_m):
    """``base_position``: middle of the pillow's underside, tabletop frame. ``base_up``: its up axis."""
    position = np.asarray(base_position, dtype=float)
    centre_x = layout.drawer_x(open_m) + layout.FRONT_T + layout.DRAWER_INNER[0] / 2
    return bool(abs(position[0] - centre_x) <= PLAN_SLACK_M and abs(position[1]) <= PLAN_SLACK_M
                and -.005 <= position[2] - layout.FLOOR_Z <= LIFTED_M and float(base_up[2]) >= TILT_COS)


def clearance(tcp, open_m):
    """Smallest distance from a gripper centre point to the box round the frame, the drawer and its pull."""
    low, high = layout.unit_box(open_m)
    tcp = np.atleast_2d(np.asarray(tcp, dtype=float))
    return float(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.), axis=1).min())


@dataclass
class DrawerGrade:
    """Running grade of one trial. Call :meth:`update` once per control step; the last value is the score."""
    hold: int = 0

    def update(self, open_m, base_position, base_up, pad_tops, tcp, touching, unit_shift_m=0.):
        """``open_m``: drawer opening. ``base_position``, ``base_up``: the pillow's underside middle and up
        axis. ``pad_tops``: height of the top of each of the 16 pads above the table. ``tcp``: both gripper
        centre points. ``touching``: whether any part of the robot touches the drawer or the pillow.
        ``unit_shift_m``: how far the frame has moved on the table (0 in the simulator, where it is fixed)."""
        values = [np.asarray(v, dtype=float) for v in (open_m, base_position, base_up, pad_tops, tcp, unit_shift_m)]
        if not all(np.isfinite(v).all() for v in values):
            self.hold = 0
            return self._grade(float("nan"), False, False, False, False, 0.)
        open_m = float(open_m)
        inside = pillow_in_drawer(base_position, base_up, open_m)
        hidden = bool(np.max(pad_tops) <= layout.RAIL_Z + SHOWS_M)
        shut = bool(open_m <= SHUT_M and inside and hidden)
        gap = clearance(tcp, open_m)
        hands_clear = bool(gap >= CLEAR_M and not touching)
        shifted = bool(unit_shift_m > UNIT_SHIFT_M)
        self.hold = self.hold + 1 if shut and hands_clear and not shifted else 0
        if not inside or shifted:
            score = 0.
        elif shut:
            score = 1. if self.hold >= HOLD_TICKS else .75
        elif open_m <= MOSTLY_M:
            score = .5
        elif open_m <= STARTED_M:
            score = .25
        else:
            score = 0.
        return self._grade(open_m, inside, hidden, shut, hands_clear, score, gap, shifted)

    def _grade(self, open_m, inside, hidden, shut, hands_clear, score, gap=float("nan"), shifted=False):
        return {"score": score, "open_mm": round(1000. * open_m, 1), "pillow_in_drawer": inside,
                "pillow_hidden": hidden, "shut": shut, "hands_clear": hands_clear,
                "gripper_clearance_mm": round(1000. * gap, 1), "unit_shifted": shifted, "hold_ticks": self.hold}


@dataclass(frozen=True)
class OverfullDrawerScorer:
    name: str = "overfull_drawer_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["drawer_grade"]
        if not grade["pillow_in_drawer"]:
            text = "0: the pillow is not lying in the drawer"
        elif grade["shut"]:
            text = "drawer shut on the pillow" + ("" if grade["score"] == 1. else ", but the hands are not clear of it for 2 s")
        else:
            text = f'drawer left open {grade["open_mm"]:.0f} mm'
        return Score(value=grade["score"], explanation=text)


def make_scorer():
    return OverfullDrawerScorer()
