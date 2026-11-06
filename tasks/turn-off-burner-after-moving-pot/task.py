"""On the station table: a black hob centred between the arms with its burner on (a red ring) and a coffee pot
standing on it, a plate in front of the right arm, and the hob's slide switch in front of the left arm, pushed to
its red ON end. The instruction asks for the pot to be moved to the plate and the stove left safe: a robot that
moves the pot but leaves the empty burner on has not finished the job. The hob, switch rails and plate are fixed;
the pot and the switch's knob are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "move the coffee pot off the hot burner onto the plate, and leave the stove safe" \\
           --scorer task_score ...
"""

import numpy as np

from yam_arena.demo import FINGER_CLOSED, Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DREAM_YAW = np.pi / 2  # the station's own objects are turned this way

# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
# -- the hob: a thin black glass top with a red burner ring (fixed) -----------------------------------------------
BURNER_XY = (0.30, 0.0)
BURNER_R = 0.09
HOB = Box("hob", pos=(*BURNER_XY, TOP + 0.0005), size=(0.30, 0.30, 0.001), color=(0.05, 0.05, 0.06))
BURNER = Cylinder("burner", pos=(*BURNER_XY, TOP + 0.0015), radius=BURNER_R, height=0.001, color=(0.85, 0.1, 0.05))

# -- the hob's slide switch: a knob between two rails, red pad at ON, white pad at OFF ----------------------------
SWITCH_X = 0.15
ON_Y, OFF_Y = 0.22, 0.12  # the knob's centre at each end of its travel
RAIL_H = 0.02
KNOB_SIZE = (0.034, 0.03, 0.045)  # taller than the rails, so it can be pushed from the side
_rail_y, _rail_len = (ON_Y + OFF_Y) / 2, ON_Y - OFF_Y + KNOB_SIZE[1] + 0.02
_stop_gap = (ON_Y - OFF_Y) / 2 + KNOB_SIZE[1] / 2 + 0.005  # stops sit just past each end of travel
SWITCH = [
    Box("rail_front", pos=(SWITCH_X - 0.022, _rail_y, TOP + RAIL_H / 2), size=(0.008, _rail_len, RAIL_H),
        color=(0.3, 0.3, 0.32)),
    Box("rail_back", pos=(SWITCH_X + 0.022, _rail_y, TOP + RAIL_H / 2), size=(0.008, _rail_len, RAIL_H),
        color=(0.3, 0.3, 0.32)),
    Box("stop_on", pos=(SWITCH_X, _rail_y + _stop_gap + 0.005, TOP + RAIL_H / 2), size=(0.052, 0.01, RAIL_H),
        color=(0.3, 0.3, 0.32)),
    Box("stop_off", pos=(SWITCH_X, _rail_y - _stop_gap - 0.005, TOP + RAIL_H / 2), size=(0.052, 0.01, RAIL_H),
        color=(0.3, 0.3, 0.32)),
    Box("pad_on", pos=(SWITCH_X + 0.045, ON_Y, TOP + 0.0005), size=(0.03, 0.03, 0.001), color=(0.85, 0.1, 0.05)),
    Box("pad_off", pos=(SWITCH_X + 0.045, OFF_Y, TOP + 0.0005), size=(0.03, 0.03, 0.001), color=(0.95, 0.95, 0.95)),
]
KNOB = Box("knob", pos=(SWITCH_X, ON_Y, TOP + KNOB_SIZE[2] / 2), size=KNOB_SIZE, mass=0.05, color=(0.1, 0.1, 0.1))

# -- the pot (loose) and the plate (fixed): library models ---------------------------------------------------------
POT = Asset("pot", "coffee_pot_hot3d_robolab", xy=BURNER_XY, extent=0.12, mass=0.4, color=(0.75, 0.75, 0.78),
            yaw=DREAM_YAW, convex=True)
PLATE = Asset("plate", "plate_small_vomp_robolab", xy=(0.30, -0.26), extent=0.146, color=(0.880, 0.876, 0.841),
              yaw=DREAM_YAW)

OBJECTS = [HOB, BURNER, *SWITCH, KNOB, POT, PLATE]

# -- stillness ----------------------------------------------------------------------------------------------------
_STEP = {"objects": None, "prev": {}, "cur": {}}


def still(objects, name) -> bool:
    """At rest: yam_arena's resting() (under 1 cm/s and 0.5 rad/s), or not moved at all since the previous control
    step (under 0.5 mm, and under 0.01 in each quaternion component). The second test is needed because PhysX can
    put a light, settled body to sleep while it still reports the small velocity it had just before, so resting()
    would never become true for it. The hooks run once per control step on the same ``objects``, so a new
    ``objects`` dict marks a new step."""
    if objects is not _STEP["objects"]:
        _STEP["objects"], _STEP["prev"] = objects, _STEP["cur"]
        _STEP["cur"] = {n: np.r_[np.asarray(o.pos, dtype=float), np.asarray(o.quat_xyzw, dtype=float)]
                        for n, o in objects.items()}
    now, last = _STEP["cur"][name], _STEP["prev"].get(name)
    unmoved = (last is not None and float(np.linalg.norm(now[:3] - last[:3])) < 0.0005
               and float(np.abs(now[3:] - last[3:]).max()) < 0.01)
    return bool(objects[name].resting() or unmoved)


# -- scoring ------------------------------------------------------------------------------------------------------
SWITCH_TOL = 0.015  # off: the knob's centre within 1.5 cm of the OFF end of its travel
ON_PLATE_R = 0.05  # the pot's centre within 5 cm of the plate's
OFF_BURNER_R = 0.12  # the pot's centre at least 12 cm from the burner's, clear of the 9 cm ring
ON_TABLE_DZ = 0.02  # the pot's base within 2 cm above the table or the plate


def _flat(a, xy) -> float:
    return float(np.hypot(*(np.asarray(a.pos[:2]) - np.asarray(xy))))


def switch_off(objects) -> bool:
    return objects["knob"].pos[1] <= OFF_Y + SWITCH_TOL


def pot_on_plate(objects) -> bool:
    pot, plate = objects["pot"], objects["plate"]
    return (_flat(pot, plate.pos[:2]) <= ON_PLATE_R and pot.low[2] <= plate.high[2] + ON_TABLE_DZ
            and still(objects, "pot"))


def pot_off_burner(objects) -> bool:
    """The pot stands on the table, clear of the burner."""
    pot = objects["pot"]
    return (_flat(pot, BURNER_XY) >= OFF_BURNER_R and TOP - 0.005 <= pot.low[2] <= TOP + ON_TABLE_DZ
            and still(objects, "pot"))


def pot_on_burner(objects) -> bool:
    """The pot still stands on the burner ring."""
    pot = objects["pot"]
    return _flat(pot, BURNER_XY) <= BURNER_R and TOP - 0.005 <= pot.low[2] <= TOP + ON_TABLE_DZ and still(objects, "pot")


def outcome(objects) -> str:
    """'done' (pot on the plate, switch off), 'off_pot_moved' (switch off, pot off the burner but not on the plate),
    'left_on' (pot on the plate, burner left on), 'off_only' (switch off, pot still on the burner), or 'other'
    (including a pot that fell, tipped or ended off the table)."""
    off = switch_off(objects)
    if pot_on_plate(objects):
        return "done" if off else "left_on"
    if off and pot_off_burner(objects):
        return "off_pot_moved"
    if off and pot_on_burner(objects):
        return "off_only"
    return "other"


def success(objects):
    """The pot rests on the plate and the burner's switch is at OFF."""
    return outcome(objects) == "done"


def distance(objects):
    """Metres (horizontal) from the pot's centre to the plate's."""
    return _flat(objects["pot"], objects["plate"].pos[:2])


def score(objects):
    """Partial credit (success scores 1.0): 0.5 with the switch off and the pot set down off the burner but not on
    the plate; 0.25 with the pot on the plate but the burner left on, or the switch off with the pot still on the
    burner; 0 for anything else."""
    return {"done": 1.0, "off_pot_moved": 0.5, "left_on": 0.25, "off_only": 0.25}.get(outcome(objects), 0.0)


# -- scripted demo ------------------------------------------------------------------------------------------------
PUSH_Z = TOP + 0.035  # fingertips above the rails and stops (2 cm), against the knob's upper half
POT_GRIP_Z = TOP + 0.06  # its body, about halfway up (the model is 12.5 cm tall at extent 0.12; measured in Isaac)
POT_GRIP_YAW = np.pi / 2  # fingers close along x, across the 7 cm body; the handle points along y


def demo():
    """The right arm lifts the pot off the burner and sets it on the plate. The left arm closes its fingers, comes
    down just beyond the knob's ON side, and pushes the knob along its rails to OFF, then lifts clear."""
    travel_z = TOP + 0.16  # straight-down grips reach only ~20 cm above the table
    pot = pick_place_waypoints((*BURNER_XY, POT_GRIP_Z), (*PLATE.xy, POT_GRIP_Z + 0.0114 + 0.01), travel_z=travel_z,
                               yaw=POT_GRIP_YAW)
    start_y = ON_Y + KNOB_SIZE[1] / 2 + 0.025
    end_y = OFF_Y + KNOB_SIZE[1] / 2 + 0.01
    switch = [
        Waypoint(1.5, (SWITCH_X, start_y, travel_z), FINGER_CLOSED, "above the switch", 0.0),
        Waypoint(1.5, (SWITCH_X, start_y, PUSH_Z), FINGER_CLOSED, "down beside the knob", 0.0),
        Waypoint(2.5, (SWITCH_X, end_y, PUSH_Z), FINGER_CLOSED, "push the knob to OFF", 0.0),
        Waypoint(1.5, (SWITCH_X, end_y, travel_z), FINGER_CLOSED, "lift clear", 0.0),
    ]
    return {"right": pot, "left": switch}
