"""On the heat-treat bench, in front of the two arms: a furnace tray has just been pulled out of the solution furnace
with three hot fittings on it, and the clock starts now. The two bare aluminium fittings have to be under water in the
quench tank within 6 s: the time from furnace to water (the quench delay) is what keeps the alloy's strength, and thin
parts lose it within seconds in air. The glowing steel fitting is being normalised: it has to cool in still air on the
cooling rack and must never go in the water (it would crack). Hot parts are never set down on the bench: only on the
tray, in the tank or on the rack. One gripper cannot get both aluminium fittings into the water in time (the second
one would go in about 10 s after the start); the two grippers have to take one each, at the same time.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- furnace tray: a fixed dark steel plate, 11 cm front to back x 36 cm side to side x 1 cm thick, centred at (0.17, 0).
- fittings: blocks 4 cm front to back x 7 cm side to side x 2.5 cm tall, lying on the tray: aluminium (bare silver,
  190 g) centred at (0.17, 0.12) and (0.17, -0.12), steel (glowing orange, 550 g) centred at (0.17, 0).
- quench tank: fixed dark walls 1 cm thick and 6 cm tall around a 14 x 36 cm floor (blue: the water), centred at
  (0.36, 0). The water line is 4.5 cm above the floor.
- cooling rack: three fixed grey slats, 10 cm front to back x 1.2 cm wide x 1.5 cm tall, 2.5 cm apart, centred at
  (0.20, -0.30).

A fitting is under water when its centre is inside the tank and its top is below the water line. Full score: both
aluminium fittings lie in the tank under water and the steel fitting lies on the cooling rack, all still for 2 s; and
each aluminium fitting was under water within 6 s of the start and stayed there, the steel fitting never went into the
tank, and no fitting was ever set down on the bench (the table outside the tank) or on another fitting.
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

STEEL_PLATE = (0.30, 0.31, 0.33)
ALUMINIUM = (0.80, 0.81, 0.83)
GLOWING = (0.96, 0.36, 0.06)
TANK_WALL = (0.16, 0.19, 0.23)
WATER = (0.15, 0.42, 0.80)
RACK_GREY = (0.58, 0.59, 0.61)

PART = (0.04, 0.07, 0.025)  # x, y, z
ALU_KG, STEEL_KG = 0.19, 0.55
TRAY_XY, TRAY = (0.17, 0.0), (0.11, 0.36, 0.01)
START_XY = {"aluminium_left": (0.17, 0.12), "steel": (0.17, 0.0), "aluminium_right": (0.17, -0.12)}
ALUMINIUM_PARTS = ("aluminium_left", "aluminium_right")
PARTS = ("aluminium_left", "steel", "aluminium_right")
TANK_XY, TANK_IN, TANK_T, TANK_H = (0.36, 0.0), (0.14, 0.36), 0.01, 0.06
WATER_T = 0.002
WATER_LINE = 0.045  # above the tank floor (the table)
RACK_XY, SLAT, SLAT_GAP = (0.20, -0.30), (0.10, 0.012, 0.015), 0.025


def _walls(name, cx, cy, in_x, in_y, t, h, color):
    ox, oy = in_x / 2 + t / 2, in_y / 2 + t / 2
    z = TOP + h / 2
    return [
        Box(f"{name}_front", pos=(cx - ox, cy, z), size=(t, in_y + 2 * t, h), color=color),
        Box(f"{name}_back", pos=(cx + ox, cy, z), size=(t, in_y + 2 * t, h), color=color),
        Box(f"{name}_left", pos=(cx, cy + oy, z), size=(in_x, t, h), color=color),
        Box(f"{name}_right", pos=(cx, cy - oy, z), size=(in_x, t, h), color=color),
    ]


PART_Z = TOP + TRAY[2] + 0.001 + PART[2] / 2
OBJECTS = (
    [Box("furnace_tray", pos=(*TRAY_XY, TOP + TRAY[2] / 2), size=TRAY, color=STEEL_PLATE)]
    + _walls("tank", *TANK_XY, *TANK_IN, TANK_T, TANK_H, TANK_WALL)
    + [Box("water", pos=(*TANK_XY, TOP + WATER_T / 2), size=(*TANK_IN, WATER_T), color=WATER)]
    + [Box(f"rack_slat_{i}", pos=(RACK_XY[0], RACK_XY[1] + k * SLAT_GAP, TOP + SLAT[2] / 2), size=SLAT,
           color=RACK_GREY) for i, k in enumerate((-1, 0, 1))]
    + [
        Box("aluminium_left", pos=(*START_XY["aluminium_left"], PART_Z), size=PART, mass=ALU_KG, color=ALUMINIUM),
        Box("steel", pos=(*START_XY["steel"], PART_Z), size=PART, mass=STEEL_KG, color=GLOWING),
        Box("aluminium_right", pos=(*START_XY["aluminium_right"], PART_Z), size=PART, mass=ALU_KG, color=ALUMINIUM),
    ]
)

# Thresholds, as a reviewer would check them from the video and with a ruler.
QUENCH_BY_S = 6.0  # "at any time": an aluminium fitting not under water by then scores 0
IN_TANK_MARGIN = 0.005  # a fitting's centre at least this far inside the tank's walls
SET_DOWN = 0.004  # a fitting's bottom this close to the bench (outside the tank) or to another fitting: set down
ON_RACK_Z = 0.01  # resting on the slats: bottom within this of their top

_DEBUG = os.environ.get("QA_DEBUG") == "1"
_state = {"step": 0, "quenched": {}, "late": "", "pulled_out": "", "steel_in_tank": False, "set_down": ""}


def _bottom(part) -> float:
    return float(part.low[2] - TOP)


def _top(part) -> float:
    return float(part.high[2] - TOP)


def _inside_tank(part) -> bool:
    return bool(abs(part.pos[0] - TANK_XY[0]) <= TANK_IN[0] / 2 - IN_TANK_MARGIN
                and abs(part.pos[1] - TANK_XY[1]) <= TANK_IN[1] / 2 - IN_TANK_MARGIN)


def _under_water(part) -> bool:
    return _inside_tank(part) and _top(part) <= WATER_LINE


def _into_tank(part) -> bool:
    """Gone into the tank: centre inside its walls and bottom below their top."""
    return _inside_tank(part) and _bottom(part) < TANK_H


def _on_rack(part) -> bool:
    return bool(abs(part.pos[0] - RACK_XY[0]) <= SLAT[0] / 2
                and abs(part.pos[1] - RACK_XY[1]) <= SLAT_GAP + SLAT[1] / 2
                and abs(_bottom(part) - SLAT[2]) <= ON_RACK_Z)


def _where_set_down(name, objects) -> str:
    part = objects[name]
    if _bottom(part) < SET_DOWN and not _inside_tank(part):
        return "bench"
    for other in PARTS:
        if other != name:
            o = objects[other]
            if (abs(part.pos[0] - o.pos[0]) < PART[0] and abs(part.pos[1] - o.pos[1]) < PART[1]
                    and abs(part.low[2] - o.high[2]) < SET_DOWN):
                return other
    return ""


def _update(objects) -> None:
    """Once per control step (success() is called every step): the quench clock and the 'at any time' rules."""
    _state["step"] += 1
    t = _state["step"] * CONTROL_DT
    for name in ALUMINIUM_PARTS:
        if _under_water(objects[name]):
            _state["quenched"].setdefault(name, t)
        elif name in _state["quenched"] and not _state["pulled_out"]:
            _state["pulled_out"] = f"{name} at {t:.2f} s"
        if name not in _state["quenched"] and t > QUENCH_BY_S and not _state["late"]:
            _state["late"] = name
    if _into_tank(objects["steel"]):
        _state["steel_in_tank"] = True
    for name in PARTS:
        where = _where_set_down(name, objects)
        if where and not _state["set_down"]:
            _state["set_down"] = f"{name} on the {where} at {t:.2f} s"


def _failed() -> bool:
    return bool(_state["late"] or _state["pulled_out"] or _state["steel_in_tank"] or _state["set_down"])


def success(objects):
    _update(objects)
    done = all(_under_water(objects[n]) for n in ALUMINIUM_PARTS) and _on_rack(objects["steel"])
    if _DEBUG and _state["step"] % 10 == 1:
        pos = {n: f"({objects[n].pos[0]:.3f},{objects[n].pos[1]:.3f},b{_bottom(objects[n]):.3f})" for n in PARTS}
        print(f"DBG step={_state['step']} t={_state['step'] * CONTROL_DT:.2f} {pos} quenched={_state['quenched']} "
              f"late='{_state['late']}' pulled='{_state['pulled_out']}' steel_in_tank={_state['steel_in_tank']} "
              f"set_down='{_state['set_down']}' steel_on_rack={_on_rack(objects['steel'])}", flush=True)
    still = all(objects[n].resting() for n in PARTS)
    return done and still and not _failed()


def distance(objects):
    d = 0.0
    for name in ALUMINIUM_PARTS:
        p = objects[name]
        d += float(np.hypot(p.pos[0] - TANK_XY[0], p.pos[1] - START_XY[name][1])) + max(0.0, _top(p) - WATER_LINE)
    s = objects["steel"]
    d += float(np.hypot(s.pos[0] - RACK_XY[0], s.pos[1] - RACK_XY[1]))
    return d


def score(objects):
    """Partial credit (success scores 1.0): 0.5 when both aluminium fittings were quenched in time and lie under water
    but the steel fitting is not on the cooling rack; 0 if an aluminium fitting was not under water within 6 s or was
    taken out again, the steel fitting went into the tank, or a fitting was set down on the bench or another fitting,
    and otherwise."""
    if _failed():
        return 0.0
    return 0.5 if all(_under_water(objects[n]) for n in ALUMINIUM_PARTS) else 0.0


# -- scripted demo -------------------------------------------------------------------------------------------------
# Fingers close front to back (gripper turned a quarter turn, the two arms in opposite directions) on a fitting's
# 4 cm depth, pad tips 5 mm above its bottom. Both grippers take an aluminium fitting at the same time, 24 cm apart,
# lift it over the tank wall and lower it into the water straight ahead; then the right gripper carries the steel
# fitting to the cooling rack while the left one waits out of its way.
TIP = 0.013
GRIP_ABOVE_BOTTOM = 0.005 + TIP  # grip point above a fitting's bottom
CARRY_BOTTOM = TANK_H + 0.025  # a fitting's bottom while carrying over the tank wall
REACH_S = float(os.environ.get("QA_REACH_S", "1.4"))
DOWN_S = float(os.environ.get("QA_DOWN_S", "0.6"))
CLOSE_S = float(os.environ.get("QA_CLOSE_S", "0.5"))
LIFT_S = float(os.environ.get("QA_LIFT_S", "0.5"))
CARRY_S = float(os.environ.get("QA_CARRY_S", "1.2"))
PLUNGE_S = float(os.environ.get("QA_PLUNGE_S", "0.6"))
RELEASE = -0.03  # fingers opened just enough to let go of a fitting, clear of the tank walls
YAW = {"left": -np.pi / 2, "right": np.pi / 2}


def _z(bottom):
    """Grip-point height for a fitting whose bottom is `bottom` above the table."""
    return TOP + bottom + GRIP_ABOVE_BOTTOM


def _dur(plan):
    return sum(w.duration for w in plan)


def _quench(arm, name):
    yaw = YAW[arm]
    sx, sy = START_XY[name]
    on_tray = TRAY[2] + 0.001
    tx, ty = TANK_XY[0], sy
    return [
        Waypoint(REACH_S, (sx, sy, _z(on_tray + 0.04)), FINGER_OPEN, "above the fitting", yaw),
        Waypoint(DOWN_S, (sx, sy, _z(on_tray)), FINGER_OPEN, "down", yaw),
        Waypoint(CLOSE_S, (sx, sy, _z(on_tray)), FINGER_CLOSED, "grip", yaw),
        Waypoint(LIFT_S, (sx, sy, _z(CARRY_BOTTOM)), FINGER_CLOSED, "lift", yaw),
        Waypoint(CARRY_S, (tx, ty, _z(CARRY_BOTTOM)), FINGER_CLOSED, "over the tank", yaw),
        Waypoint(PLUNGE_S, (tx, ty, _z(WATER_T + 0.002)), FINGER_CLOSED, "into the water", yaw),
        Waypoint(0.5, (tx, ty, _z(WATER_T + 0.002)), RELEASE, "let go", yaw),
        Waypoint(0.8, (tx, ty, _z(CARRY_BOTTOM)), RELEASE, "out of the tank", yaw),
    ]


def demo():
    left = _quench("left", "aluminium_left")
    right = _quench("right", "aluminium_right")
    yl, yr = YAW["left"], YAW["right"]
    left.append(Waypoint(1.2, (0.30, 0.26, _z(CARRY_BOTTOM)), RELEASE, "out of the way", yl))
    sx, sy = START_XY["steel"]
    on_tray = TRAY[2] + 0.001
    rx, ry = RACK_XY
    on_rack = SLAT[2] + 0.003
    right += [
        Waypoint(1.4, (sx, sy, _z(on_tray + 0.06)), FINGER_OPEN, "above the steel fitting", yr),
        Waypoint(0.8, (sx, sy, _z(on_tray)), FINGER_OPEN, "down", yr),
        Waypoint(0.6, (sx, sy, _z(on_tray)), FINGER_CLOSED, "grip", yr),
        Waypoint(0.6, (sx, sy, _z(on_tray + 0.05)), FINGER_CLOSED, "lift", yr),
        Waypoint(1.4, (rx, ry, _z(on_rack + 0.04)), FINGER_CLOSED, "over the cooling rack", yr),
        Waypoint(0.8, (rx, ry, _z(on_rack)), FINGER_CLOSED, "onto the slats", yr),
        Waypoint(0.5, (rx, ry, _z(on_rack)), FINGER_OPEN, "let go", yr),
        Waypoint(0.8, (rx, ry, _z(on_rack + 0.06)), FINGER_OPEN, "rise", yr),
    ]
    end = max(_dur(left), _dur(right))
    for plan, y in ((left, yl), (right, yr)):
        if _dur(plan) < end:
            plan.append(Waypoint(end - _dur(plan), plan[-1].pos, plan[-1].finger, "wait", y))
    return {"left": left, "right": right}
