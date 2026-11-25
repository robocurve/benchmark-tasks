"""Score the trial from where the bag's base board, its four walls and two handles, the carton, the pack and the
grippers are. NumPy only.

Every rule can be judged from the trial video, photos of the end and a ruler (README, "Scoring a real trial").

Definitions

- An item *rests on the base*: its lowest point is within 3 mm of the base board, it stands on one of its faces
  (that face within 10 degrees of the board) and, seen from above, it lies wholly within the board's outline. An
  item lying on a folded-in wall is more than 3 mm up and does not count.
- The bag *stands upright*: the base board is on the table (its lowest point within 5 mm of it), level within 10
  degrees and with its middle within 50 mm of its marks, and the top edge of each of the four walls is at least
  144 mm above the table (80 % of the 180 mm bag).
- The *lift* is done once, for 2 s without a break, the base board's lowest point is at least 50 mm above the table,
  both handles are pulled up (each grip at least 30 mm above the bag's rim) and both items remain in the bag.

Score, from the state at the end of the trial:

- 1.0: both items rest on the base, the bag stands upright, the lift is done, and for the last 2 s no part of the
  robot touches the bag or an item and both grippers' centre points are at least 50 mm from the bag.
- 0.75: both items rest on the base and the bag stands upright, but the lift is not done or the robot has not let go.
- 0.5: the pack rests on the base and the bag stands upright (the carton is anywhere).
- 0.25: only the carton rests on the base and the bag stands upright.
- 0 otherwise. Also 0 for the rest of the trial once an item has left the table, an item has come out of the bag
  while the bag was off the table, or a corner seam of the bag has been pulled more than 30 mm apart (torn).

Nothing is scored for the order in which the items go in or for how they are held.
"""

from dataclasses import dataclass, field

import numpy as np

from . import bag_model as m

try:  # the Inspect Robots scorer wrapper is only needed inside a trial
    from inspect_robots.scorer import Score
except ImportError:  # pragma: no cover
    Score = None

CONTROL_HZ = 20
HOLD_S = 2
HOLD_TICKS = CONTROL_HZ * HOLD_S
REST_GAP_M = .003                # an item's lowest point above the base board
REST_TILT_DEG = 10.              # the face an item stands on, to the base board
OUTLINE_SLACK_M = .001           # an item's corners may be this far outside the board's outline
ON_TABLE_M = .005                # base board's lowest point above the table when the bag stands on it
LEVEL_DEG = 10.                  # base board to level
MARK_M = .050                    # base board's middle from its marks
RIM_M = .8 * m.RIM               # each wall's top edge above the table: 144 mm
LIFT_M = .050                    # base board's lowest point above the table during the lift
GRIP_UP_M = .030                 # a handle's grip above the rim while the bag hangs from it
OFF_TABLE_M = .020               # bag counts as off the table (for spilling) above this

CLEAR_M = .050                   # gripper centre point to the bag at the end
TOUCH_N = .05                    # contact force that counts as touching
FELL_M = -.020                   # an item's middle below the table top: it has left the table
START_XY = m.BAG_XY
SIZES = {"carton": m.CARTON, "pack": m.PACK}


def tilt_deg(rot):
    return float(np.degrees(np.arccos(np.clip(np.asarray(rot, dtype=float)[2, 2], -1., 1.))))


def corners(size):
    half = np.asarray(size, dtype=float) / 2
    return np.array([(sx * half[0], sy * half[1], sz * half[2]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def item_state(name, pos, rot, base_pos, base_rot):
    """How an item sits on the base board. Poses in the tabletop frame."""
    base_rot, rot = np.asarray(base_rot, dtype=float), np.asarray(rot, dtype=float)
    local = (corners(SIZES[name]) @ rot.T + np.asarray(pos, dtype=float) - np.asarray(base_pos, dtype=float)) @ base_rot
    gap = float(local[:, 2].min() - m.BASE_T)
    over = float(max(np.abs(local[:, 0]).max() - m.BASE_X / 2, np.abs(local[:, 1]).max() - m.BASE_Y / 2))
    centre = base_rot.T @ (np.asarray(pos, dtype=float) - np.asarray(base_pos, dtype=float))
    lean = float(np.degrees(np.arccos(np.clip(np.abs(base_rot.T @ rot)[2].max(), 0., 1.))))     # nearest face to the board
    inside = bool(abs(centre[0]) <= m.BASE_X / 2 and abs(centre[1]) <= m.BASE_Y / 2 and -m.BASE_T <= gap < m.RIM)
    rests = bool(abs(gap) <= REST_GAP_M and lean <= REST_TILT_DEG and over <= OUTLINE_SLACK_M)
    return {"rests": rests, "inside": inside, "above_base_mm": round(1000. * gap, 2), "lean_deg": round(lean, 1),
            "outside_outline_mm": round(1000. * over, 1)}


def base_lowest(base_pos, base_rot):
    """Height of the base board's lowest corner above the table."""
    base_rot = np.asarray(base_rot, dtype=float)
    hx, hy = m.BASE_X / 2, m.BASE_Y / 2
    return float(min((np.asarray(base_pos, dtype=float) + base_rot @ (sx * hx, sy * hy, 0.))[2] for sx in (-1, 1) for sy in (-1, 1)))


def clearance(tcp, base_pos, base_rot, grips):
    """Smallest distance from a gripper centre point to the bag: its outline up to the rim, and each handle's grip."""
    origin, rot = np.asarray(base_pos, dtype=float), np.asarray(base_rot, dtype=float)
    high = np.array((m.BASE_X / 2, m.BASE_Y / 2, m.RIM))
    low = np.array((-m.BASE_X / 2, -m.BASE_Y / 2, 0.))
    best = np.inf
    for point in np.atleast_2d(np.asarray(tcp, dtype=float)):
        local = rot.T @ (point - origin)
        best = min(best, float(np.linalg.norm(np.maximum(np.maximum(low - local, local - high), 0.))))
        for grip, along in grips:
            offset = point - np.asarray(grip, dtype=float)
            reach = float(np.clip(offset @ np.asarray(along, dtype=float), -m.HANDLE_SPAN / 2, m.HANDLE_SPAN / 2))
            best = min(best, float(np.linalg.norm(offset - reach * np.asarray(along, dtype=float))))
    return best


@dataclass
class BagGrade:
    """Running grade of one trial. Call :meth:`update` once per control step; the last value is the score."""
    hold: int = 0
    lift_ticks: int = 0
    lifted: bool = False
    fell: bool = False
    spilled: bool = False
    torn: bool = False
    peak_seam_n: float = 0.
    was_inside: dict = field(default_factory=lambda: {"carton": False, "pack": False})

    def update(self, *, base, uppers, handles, items, tcp, touching):
        """``base``: (position, rotation) of the base board (origin on its underside, in the middle). ``uppers``:
        wall -> (position, rotation) of that wall's upper panel. ``handles``: wall -> (position, rotation) of the
        handle (origin on the rim). ``items``: name -> (position, rotation). ``tcp``: both gripper centre points.
        ``touching``: whether any part of the robot touches the bag or an item. Everything in the tabletop frame."""
        values = [np.asarray(v, dtype=float) for pair in (base, *uppers.values(), *handles.values(), *items.values()) for v in pair]
        values.append(np.asarray(tcp, dtype=float))
        if not all(np.isfinite(v).all() for v in values):
            self.hold = self.lift_ticks = 0
            return self._grade(0., dict(error="nonfinite state"))
        base_pos, base_rot = (np.asarray(v, dtype=float) for v in base)
        rims = {wall: m.rim_height(wall, *uppers[wall]) for wall in m.WALLS}
        seams = m.seam_lengths({m.UPPER[wall]: uppers[wall] for wall in m.WALLS})
        pull = max(m.seam_pull(v) for v in seams.values())
        self.peak_seam_n = max(self.peak_seam_n, pull)
        self.torn = self.torn or pull > m.SEAM_TORN_N
        state = {name: item_state(name, *items[name], base_pos, base_rot) for name in m.ITEMS}
        low = base_lowest(base_pos, base_rot)
        tilt = tilt_deg(base_rot)
        shift = float(np.hypot(base_pos[0] - START_XY[0], base_pos[1] - START_XY[1]))
        upright = bool(low <= ON_TABLE_M and tilt <= LEVEL_DEG and shift <= MARK_M and min(rims.values()) >= RIM_M)
        # the lift: bag hanging from both handles, both items retained inside, 2 s without a break
        grips = {wall: m.grip_centre(*handles[wall]) for wall in m.HANDLES}
        rim_top = base_pos + base_rot @ (0., 0., m.RIM)
        grips_up = {wall: float(base_rot[:, 2] @ (grips[wall] - rim_top)) for wall in m.HANDLES}
        hanging = bool(low >= LIFT_M and min(grips_up.values()) >= GRIP_UP_M and all(state[n]["inside"] for n in m.ITEMS))
        self.lift_ticks = self.lift_ticks + 1 if hanging else 0
        self.lifted = self.lifted or self.lift_ticks >= HOLD_TICKS
        # an item that was in the bag and is not any more while the bag is off the table has come out of it
        if low >= OFF_TABLE_M:
            for name in m.ITEMS:
                self.spilled = self.spilled or (self.was_inside[name] and not state[name]["inside"])
        for name in m.ITEMS:
            self.was_inside[name] = state[name]["inside"]
            self.fell = self.fell or float(np.asarray(items[name][0], dtype=float)[2]) < FELL_M
        gap = clearance(tcp, base_pos, base_rot, [(grips[wall], np.asarray(handles[wall][1], dtype=float)[:, 0]) for wall in m.HANDLES])
        hands_clear = bool(gap >= CLEAR_M and not touching)
        carton, pack = state["carton"]["rests"], state["pack"]["rests"]
        done = bool(carton and pack and upright and self.lifted and hands_clear)
        self.hold = self.hold + 1 if done else 0
        if self.fell or self.spilled or self.torn:
            score = 0.
        elif not upright:
            score = 0.
        elif carton and pack:
            score = 1. if self.hold >= HOLD_TICKS else .75
        elif pack:
            score = .5
        elif carton:
            score = .25
        else:
            score = 0.
        return self._grade(score, dict(
            carton=state["carton"], pack=state["pack"], upright=upright, hanging=hanging, hands_clear=hands_clear,
            rims_mm={wall: round(1000. * v, 1) for wall, v in rims.items()}, base_low_mm=round(1000. * low, 1),
            base_tilt_deg=round(tilt, 2), base_shift_mm=round(1000. * shift, 1),
            grips_above_rim_mm={wall: round(1000. * v, 1) for wall, v in grips_up.items()},
            gripper_clearance_mm=round(1000. * gap, 1), seam_pull_n=round(pull, 2)))

    def _grade(self, score, detail):
        return {"score": score, "hold_ticks": self.hold, "lift_ticks": self.lift_ticks, "lifted": self.lifted,
                "fell": self.fell, "spilled": self.spilled, "torn": self.torn,
                "peak_seam_pull_n": round(self.peak_seam_n, 2), **detail}


@dataclass(frozen=True)
class BagScorer:
    name: str = "bag_load_score"

    def __call__(self, record, target):
        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["bag_load_grade"]
        if grade["fell"]:
            text = "0: an item left the table"
        elif grade["spilled"]:
            text = "0: an item came out of the bag while it was lifted"
        elif grade["torn"]:
            text = "0: a corner seam of the bag was pulled past its limit"
        elif "error" in grade:
            text = "0: " + grade["error"]
        else:
            inside = [name for name in m.ITEMS if grade[name]["rests"]]
            text = (f"{', '.join(inside) or 'nothing'} on the bag's base; bag {'upright' if grade['upright'] else 'not upright'}; "
                    f"lift {'done' if grade['lifted'] else 'not done'}; robot {'clear' if grade['hands_clear'] else 'not clear'}")
        return Score(value=grade["score"], explanation=text)


def make_scorer():
    return BagScorer()
