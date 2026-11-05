"""On the station table, in front of the two arms: a running field base station with one battery bay, a charger rack
with two spare packs on the right and a fire-safe return bin on the left. The pack in the bay is failing (red tile in
front of the bay). It has to be replaced by the charged spare (green tile in front of its cradle), not by the one
still charging (amber tile), and the failing pack goes into the return bin. The station runs without a pack only on
its hold-up capacitor, for at most 4 s, so the bay may never stay empty longer than that. Lithium packs are never set
down loose: the failing pack may only be in the bay, the return bin or a gripper (a damaged pack never goes back on a
charger); the charged spare only in its cradle, the bay or a gripper; the charging pack stays on its charger. With
one gripper the bay stays empty far longer than 4 s (the failing pack has to go all the way to the return bin before
that gripper can fetch the spare from the rack), so the swap is make-before-break: one gripper holds the spare ready
beside the bay while the other pulls the failing pack out, and the spare goes in the moment the bay is clear.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- station: a fixed dark grey housing, 10 cm front to back x 24 cm side to side x 10 cm tall, centred at (0.37, 0).
- bay: a pocket in front of the housing, centred at (0.25, 0): fixed dark grey walls 1 cm thick and 3 cm tall around
  a 9 x 8 cm floor (the table). A red tile (tape, 1 mm) on the table in front of it.
- packs: identical dark blue boxes, 7 cm front to back x 6 cm side to side x 8 cm tall, 400 g each. The failing pack
  stands in the bay.
- charger rack: two black cradles (the same pocket as the bay) centred at (0.22, -0.30) and (0.11, -0.30), a pack
  standing in each; a green tile in front of the first (charged), an amber tile in front of the second (charging).
- return bin: fixed orange walls 1 cm thick and 4 cm tall around a 12 x 12 cm floor (the table), centred at
  (0.18, 0.28).

A pack is in the bay when its centre is within 1.5 cm of the bay's centre (seen from above) and its bottom within
1 cm of the table. Full score: the charged spare stands in the bay and the failing pack stands in the return bin, the
charging pack still on its charger, everything still for 2 s; and at no time was the bay empty for more than 4 s, was
a pack set down anywhere it may not be (on the table, the station, another pack or the wrong pocket), did a pack tip
over (lean more than 25 degrees), or did the charging pack move more than 1 cm.
"""

import os

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 2.0
CONTROL_DT = 0.05  # success() is called once per control step (20 Hz)

DARK = (0.20, 0.21, 0.23)
BLACK = (0.04, 0.04, 0.05)
PACK_BLUE = (0.13, 0.22, 0.42)
RED, GREEN, AMBER = (0.82, 0.12, 0.10), (0.15, 0.66, 0.25), (0.98, 0.62, 0.05)
ORANGE = (0.95, 0.42, 0.08)

PACK = (0.07, 0.06, 0.08)  # x, y, z
PACK_KG = 0.4
POCKET_IN = (0.09, 0.08)  # inside of the bay and of each cradle, x, y (1 cm clearance round a pack)
POCKET_T, POCKET_H = 0.01, 0.03
BAY_XY = (0.25, 0.0)
CHARGED_XY, CHARGING_XY = (0.22, -0.30), (0.11, -0.30)
BIN_XY, BIN_IN, BIN_T, BIN_H = (0.18, 0.28), 0.12, 0.01, 0.04
HOUSING_XY, HOUSING = (0.37, 0.0), (0.10, 0.24, 0.10)


def _pocket(name, cx, cy, color):
    ox = POCKET_IN[0] / 2 + POCKET_T / 2
    oy = POCKET_IN[1] / 2 + POCKET_T / 2
    z = TOP + POCKET_H / 2
    long_y = POCKET_IN[1] + 2 * POCKET_T
    return [
        Box(f"{name}_front", pos=(cx - ox, cy, z), size=(POCKET_T, long_y, POCKET_H), color=color),
        Box(f"{name}_back", pos=(cx + ox, cy, z), size=(POCKET_T, long_y, POCKET_H), color=color),
        Box(f"{name}_left", pos=(cx, cy + oy, z), size=(POCKET_IN[0], POCKET_T, POCKET_H), color=color),
        Box(f"{name}_right", pos=(cx, cy - oy, z), size=(POCKET_IN[0], POCKET_T, POCKET_H), color=color),
    ]


def _bin():
    cx, cy = BIN_XY
    off = BIN_IN / 2 + BIN_T / 2
    outer = BIN_IN + 2 * BIN_T
    z = TOP + BIN_H / 2
    return [
        Box("bin_front", pos=(cx - off, cy, z), size=(BIN_T, outer, BIN_H), color=ORANGE),
        Box("bin_back", pos=(cx + off, cy, z), size=(BIN_T, outer, BIN_H), color=ORANGE),
        Box("bin_left", pos=(cx, cy + off, z), size=(BIN_IN, BIN_T, BIN_H), color=ORANGE),
        Box("bin_right", pos=(cx, cy - off, z), size=(BIN_IN, BIN_T, BIN_H), color=ORANGE),
    ]


def _tile(name, xy, color):
    x = xy[0] - (POCKET_IN[0] / 2 + POCKET_T + 0.012)  # just in front of the pocket
    return Box(name, pos=(x, xy[1], TOP + 0.0005), size=(0.012, 0.05, 0.001), color=color)


PACK_Z = TOP + 0.001 + PACK[2] / 2
OBJECTS = (
    [Box("station", pos=(*HOUSING_XY, TOP + HOUSING[2] / 2), size=HOUSING, color=DARK)]
    + _pocket("bay", *BAY_XY, DARK)
    + _pocket("cradle_charged", *CHARGED_XY, BLACK)
    + _pocket("cradle_charging", *CHARGING_XY, BLACK)
    + _bin()
    + [
        _tile("tile_failing", BAY_XY, RED),
        _tile("tile_charged", CHARGED_XY, GREEN),
        _tile("tile_charging", CHARGING_XY, AMBER),
        Box("pack_failing", pos=(*BAY_XY, PACK_Z), size=PACK, mass=PACK_KG, color=PACK_BLUE),
        Box("pack_charged", pos=(*CHARGED_XY, PACK_Z), size=PACK, mass=PACK_KG, color=PACK_BLUE),
        Box("pack_charging", pos=(*CHARGING_XY, PACK_Z), size=PACK, mass=PACK_KG, color=PACK_BLUE),
    ]
)
PACKS = ("pack_failing", "pack_charged", "pack_charging")

# Thresholds, as a reviewer would check them from the video and with a ruler.
IN_BAY_XY, IN_BAY_Z = 0.015, 0.01
IN_POCKET_Z = 0.015  # a pack stands in a cradle or the bin: centre inside it and bottom within 1.5 cm of the table
GAP_MAX_S = 4.0  # "at any time": the bay empty for longer than this scores 0
SET_DOWN = 0.004  # "at any time": a pack's bottom this close to a surface it may not stand on scores 0 ...
TIP_MAX = np.radians(25.0)  # ... and so does a pack leaning more than this ...
CHARGING_MOVED = 0.01  # ... or the charging pack moved more than this

_DEBUG = os.environ.get("HS_DEBUG") == "1"
_state = {"empty_steps": 0, "worst_gap": 0.0, "gap": False, "set_down": "", "tipped": "", "charging": False,
          "step": 0, "max_tilt": {}}


def _tilt(quat_xyzw) -> float:
    x, y, _, _ = quat_xyzw
    return float(np.arccos(np.clip(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0)))


def _bottom(pack) -> float:
    return float(pack.low[2] - TOP)


def _within(pack, xy, half_x, half_y) -> bool:
    return bool(abs(pack.pos[0] - xy[0]) <= half_x and abs(pack.pos[1] - xy[1]) <= half_y)


def _in_bay(pack) -> bool:
    return _within(pack, BAY_XY, IN_BAY_XY, IN_BAY_XY) and _bottom(pack) <= IN_BAY_Z


def _in_pocket(pack, xy, half_x, half_y) -> bool:
    return _within(pack, xy, half_x, half_y) and _bottom(pack) <= IN_POCKET_Z


def _in_cradle(pack, xy) -> bool:
    return _in_pocket(pack, xy, POCKET_IN[0] / 2, POCKET_IN[1] / 2)


def _in_bin(pack) -> bool:
    return _in_pocket(pack, BIN_XY, BIN_IN / 2, BIN_IN / 2)


ALLOWED = {  # where each pack may stand
    "pack_failing": lambda p: _in_bay(p) or _in_bin(p),
    "pack_charged": lambda p: _in_bay(p) or _in_cradle(p, CHARGED_XY),
    "pack_charging": lambda p: _in_cradle(p, CHARGING_XY),
}


def _where_set_down(name, objects) -> str:
    """A surface this pack stands on (or is within a few mm of) where it may not be set down, or ''."""
    pack = objects[name]
    if ALLOWED[name](pack):
        return ""
    bottom = _bottom(pack)
    if bottom < SET_DOWN:
        return "table"
    hx, hy = HOUSING[0] / 2 + PACK[0] / 2, HOUSING[1] / 2 + PACK[1] / 2
    if _within(pack, HOUSING_XY, hx, hy) and abs(bottom - HOUSING[2]) < SET_DOWN:
        return "station"
    for other in PACKS:
        if other != name:
            o = objects[other]
            if (abs(pack.pos[0] - o.pos[0]) < PACK[0] and abs(pack.pos[1] - o.pos[1]) < PACK[1]
                    and abs(pack.low[2] - o.high[2]) < SET_DOWN):
                return "another pack"
    return ""


def _update(objects) -> None:
    """Once per control step (success() is called every step): the bay's empty time and the 'at any time' rules."""
    _state["step"] += 1
    if any(_in_bay(objects[p]) for p in PACKS):
        _state["empty_steps"] = 0
    else:
        _state["empty_steps"] += 1
        gap = _state["empty_steps"] * CONTROL_DT
        _state["worst_gap"] = max(_state["worst_gap"], gap)
        if gap > GAP_MAX_S:
            _state["gap"] = True
    for name in PACKS:
        where = _where_set_down(name, objects)
        if where and not _state["set_down"]:
            _state["set_down"] = f"{name} on the {where}"
        tilt = _tilt(objects[name].quat_xyzw)
        _state["max_tilt"][name] = max(_state["max_tilt"].get(name, 0.0), tilt)
        if tilt > TIP_MAX and not _state["tipped"]:
            _state["tipped"] = f"{name} at step {_state['step']}"
    moved = np.hypot(objects["pack_charging"].pos[0] - CHARGING_XY[0], objects["pack_charging"].pos[1] - CHARGING_XY[1])
    if moved > CHARGING_MOVED:
        _state["charging"] = True


def _failed() -> bool:
    return bool(_state["gap"] or _state["set_down"] or _state["tipped"] or _state["charging"])


def success(objects):
    _update(objects)
    spare, failing = objects["pack_charged"], objects["pack_failing"]
    done = _in_bay(spare) and _in_bin(failing)
    if _DEBUG and _state["step"] % 10 == 1:
        print(f"DBG step={_state['step']} spare=({spare.pos[0]:.3f},{spare.pos[1]:.3f},b{_bottom(spare):.3f}) "
              f"failing=({failing.pos[0]:.3f},{failing.pos[1]:.3f},b{_bottom(failing):.3f}) "
              f"spare_in_bay={_in_bay(spare)} failing_in_bin={_in_bin(failing)} "
              f"empty={_state['empty_steps'] * CONTROL_DT:.2f}s worst={_state['worst_gap']:.2f}s "
              f"gap={_state['gap']} set_down='{_state['set_down']}' tipped='{_state['tipped']}' "
              f"charging_moved={_state['charging']} tilt_deg="
              f"{ {k[5:]: round(float(np.degrees(_tilt(objects[k].quat_xyzw))), 1) for k in PACKS} }", flush=True)
    still = all(objects[p].resting() for p in PACKS)
    return done and still and not _failed()


def distance(objects):
    spare, failing = objects["pack_charged"], objects["pack_failing"]
    d = float(np.hypot(spare.pos[0] - BAY_XY[0], spare.pos[1] - BAY_XY[1])) + max(0.0, _bottom(spare))
    d += float(np.hypot(failing.pos[0] - BIN_XY[0], failing.pos[1] - BIN_XY[1])) + max(0.0, _bottom(failing))
    return d


def score(objects):
    """Partial credit (success scores 1.0): 0.5 when the charged spare stands in the bay but the failing pack is not
    in the return bin; 0 if the bay was ever empty for more than 4 s, a pack was set down where it may not be, a pack
    tipped over, or the charging pack moved more than 1 cm, and otherwise."""
    if _failed():
        return 0.0
    return 0.5 if _in_bay(objects["pack_charged"]) else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# Fingers close front to back (gripper turned a quarter turn) on the top 2.5 cm of a pack. The right gripper lifts
# the charged spare out of its cradle and holds it to the right of the bay, its bottom clear of the walls, 18 cm away
# (the two wrists need that much room); the left gripper grips the failing pack. Then, on one timeline: the left
# gripper lifts the failing pack clear of the walls, both grippers move left together (the failing pack out over the
# table, the spare over the bay) and the right gripper lowers the spare in. Then the left gripper carries the failing
# pack to the return bin.
TIP = 0.013
GRIP_ABOVE_BOTTOM = PACK[2] - 0.025 + TIP  # grip point (pad tips 2.5 cm below the top) above a pack's bottom
CLEAR = POCKET_H + 0.015  # pack bottom while swapping: 1.5 cm above the bay walls
HIGH = BIN_H + 0.025  # pack bottom while carrying to the bin and from the rack
LIFT_S = float(os.environ.get("HS_LIFT_S", "0.5"))
SLIDE_S = float(os.environ.get("HS_SLIDE_S", "0.9"))
SEAT_S = float(os.environ.get("HS_SEAT_S", "0.6"))
SETTLE_S = float(os.environ.get("HS_SETTLE_S", "0.3"))
SEAT_DX = float(os.environ.get("HS_SEAT_DX", "-0.009"))
SET_GAP = float(os.environ.get("HS_SET_GAP", "0.003"))
WAIT_Y = float(os.environ.get("HS_WAIT_Y", "-0.18"))  # spare waits here (bay at y 0)
OUT_Y = float(os.environ.get("HS_OUT_Y", "0.20"))  # failing pack moves out to here
YAW = {"left": -np.pi / 2, "right": np.pi / 2}


def _z(bottom):
    """Grip-point height for a pack whose bottom is `bottom` above the table."""
    return TOP + bottom + GRIP_ABOVE_BOTTOM


def _dur(plan):
    return sum(w.duration for w in plan)


def demo():
    yl, yr = YAW["left"], YAW["right"]
    bx, by = BAY_XY
    sx = bx + SEAT_DX  # a gripped pack sits about 9 mm off the grip point along the fingers' closing direction
    z_grip = _z(0.001)
    z_clear, z_high, z_seat = _z(CLEAR), _z(HIGH), _z(SET_GAP)
    hover = _z(HIGH + 0.02)
    right = [
        Waypoint(2.0, (*CHARGED_XY, hover), FINGER_OPEN, "above the charged spare", yr),
        Waypoint(1.5, (*CHARGED_XY, z_grip), FINGER_OPEN, "down", yr),
        Waypoint(1.0, (*CHARGED_XY, z_grip), FINGER_CLOSED, "grasp the spare", yr),
        Waypoint(1.2, (*CHARGED_XY, z_high), FINGER_CLOSED, "lift out of the cradle", yr),
        Waypoint(1.5, (bx, WAIT_Y, z_clear), FINGER_CLOSED, "ready beside the bay", yr),
    ]
    left = [
        Waypoint(2.0, (bx, by, hover), FINGER_OPEN, "above the failing pack", yl),
        Waypoint(1.5, (bx, by, z_grip), FINGER_OPEN, "down", yl),
        Waypoint(1.0, (bx, by, z_grip), FINGER_CLOSED, "grasp the failing pack", yl),
    ]
    t_ready = max(_dur(right), _dur(left)) + 1.0  # both settle
    right.append(Waypoint(t_ready - _dur(right), (bx, WAIT_Y, z_clear), FINGER_CLOSED, "hold ready", yr))
    left.append(Waypoint(t_ready - _dur(left), (bx, by, z_grip), FINGER_CLOSED, "hold", yl))
    # the swap: the bay is empty from just after the lift starts until the spare is down
    left += [
        Waypoint(LIFT_S, (bx, by, z_clear), FINGER_CLOSED, "lift clear of the walls", yl),
        Waypoint(SLIDE_S, (bx, OUT_Y, z_clear), FINGER_CLOSED, "out to the left", yl),
    ]
    right += [
        Waypoint(LIFT_S, (bx, WAIT_Y, z_clear), FINGER_CLOSED, "hold ready", yr),
        Waypoint(SLIDE_S, (sx, by, z_clear), FINGER_CLOSED, "spare over the bay", yr),
        Waypoint(SETTLE_S, (sx, by, z_clear), FINGER_CLOSED, "steady over the bay", yr),
        Waypoint(SEAT_S, (sx, by, z_seat), FINGER_CLOSED, "lower it in", yr),
        Waypoint(0.6, (sx, by, z_seat), FINGER_OPEN, "let go", yr),
        Waypoint(1.0, (sx, by, hover), FINGER_OPEN, "rise", yr),
        Waypoint(1.5, (0.16, -0.20, hover), FINGER_OPEN, "park", yr),
    ]
    # the failing pack into the return bin
    left += [
        Waypoint(0.8, (bx, OUT_Y, z_high), FINGER_CLOSED, "up", yl),
        Waypoint(1.2, (*BIN_XY, z_high), FINGER_CLOSED, "over the return bin", yl),
        Waypoint(1.2, (*BIN_XY, _z(SET_GAP)), FINGER_CLOSED, "lower into the bin", yl),
        Waypoint(0.6, (*BIN_XY, _z(SET_GAP)), FINGER_OPEN, "let go", yl),
        Waypoint(1.0, (*BIN_XY, hover), FINGER_OPEN, "rise", yl),
    ]
    end = max(_dur(right), _dur(left))
    for plan, y in ((left, yl), (right, yr)):
        if _dur(plan) < end:
            plan.append(Waypoint(end - _dur(plan), plan[-1].pos, plan[-1].finger, "wait", y))
    return {"left": left, "right": right}
