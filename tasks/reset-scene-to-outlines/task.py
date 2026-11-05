"""On the station table: four objects (a bowl, a wooden cube, a soup can and a cracker box) scattered in front of
the arms, and four flat outlines taped in a row across the far side, one per object, each the colour of its
object. The robot must put every object back on its outline, the cracker box turned to match its outline, so the
next data-collection episode can start. Robots that reset their own workspace let data collection run without a
person. The outlines are fixed; the objects are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "put everything back on its outline so the next run can start" \\
           --scorer task_score ...
"""


import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DREAM_YAW = np.pi / 2  # the station's own objects are turned this way

# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
ROW_X = 0.36
BOWL_C, CUBE_C, CAN_C, BOX_C = (0.65, 0.65, 0.68), (0.85, 0.55, 0.25), (0.75, 0.2, 0.15), (0.95, 0.75, 0.15)

# -- the outlines (fixed, 1 mm thick) ------------------------------------------------------------------------------
HOME = {"bowl": (ROW_X, 0.20), "cube": (ROW_X, 0.07), "can": (ROW_X, -0.06), "crackers": (ROW_X, -0.20)}
CRACKERS_HOME_YAW = DREAM_YAW  # its long side front to back
CRACKERS_FOOT = (0.11, 0.06)  # the outline: front to back, left to right
OUTLINES = [
    Cylinder("outline_bowl", pos=(*HOME["bowl"], TOP + 0.0005), radius=0.085, height=0.001, color=BOWL_C),
    Box("outline_cube", pos=(*HOME["cube"], TOP + 0.0005), size=(0.06, 0.06, 0.001), color=CUBE_C),
    Cylinder("outline_can", pos=(*HOME["can"], TOP + 0.0005), radius=0.04, height=0.001, color=CAN_C),
    Box("outline_crackers", pos=(*HOME["crackers"], TOP + 0.0005), size=(*CRACKERS_FOOT, 0.001), color=BOX_C),
]

# -- the objects (loose), scattered -------------------------------------------------------------------------------
START = {"bowl": (0.20, 0.30), "cube": (0.10, 0.12), "can": (0.12, -0.10), "crackers": (0.20, -0.30)}
CUBE_SIZE = 0.05
CAN_R, CAN_H = 0.033, 0.10
BOWL = Asset("bowl", "bowl_ycb_robolab", xy=START["bowl"], extent=0.16, mass=0.10, color=BOWL_C, yaw=DREAM_YAW,
             convex=False)  # the model's own convex decomposition keeps it hollow, so its rim can be gripped
CUBE = Box("cube", pos=(*START["cube"], TOP + CUBE_SIZE / 2), size=(CUBE_SIZE,) * 3, mass=0.1, color=CUBE_C)
CAN = Cylinder("can", pos=(*START["can"], TOP + CAN_H / 2), radius=CAN_R, height=CAN_H, mass=0.35, color=CAN_C)
CRACKERS = Asset("crackers", "cheez_it_ycb_robolab", xy=START["crackers"], extent=0.10, mass=0.12, yaw=0.0,
                 convex=True)  # 5.3 x 10 x 13 cm (measured), turned a quarter turn from its outline

OBJECTS = [*OUTLINES, BOWL, CUBE, CAN, CRACKERS]
ITEMS = ["bowl", "cube", "can", "crackers"]

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
ON_OUTLINE = 0.02  # centre within 2 cm of the outline's centre
FLAT_DZ = 0.01  # underside within 1 cm of the table
MIN_HEIGHT = {"bowl": 0.0, "cube": 0.04, "can": 0.09, "crackers": 0.12}  # standing up, not lying on its side
CRACKERS_MAX_SIDEWAYS = 0.08  # its left-to-right size: 5.3 cm square-on, 8 cm at about 18 degrees off


def home(objects, name) -> bool:
    """Resting upright on its outline (and, for the cracker box, turned to match it)."""
    item = objects[name]
    near = float(np.hypot(*(np.asarray(item.pos[:2]) - np.asarray(HOME[name])))) <= ON_OUTLINE
    size = np.asarray(item.high) - np.asarray(item.low)
    ok = near and item.low[2] <= TOP + FLAT_DZ and size[2] >= MIN_HEIGHT[name] and still(objects, name)
    if name == "crackers":
        ok = ok and size[1] <= CRACKERS_MAX_SIDEWAYS
    return bool(ok)


def outcome(objects) -> str:
    """'reset' when all four are home, otherwise 'n_of_4' (how many are home)."""
    n = sum(home(objects, name) for name in ITEMS)
    return "reset" if n == 4 else f"{n}_of_4"


def success(objects):
    """All four objects rest upright on their outlines, the cracker box turned to match its outline."""
    return outcome(objects) == "reset"


def distance(objects):
    """Metres (horizontal), summed over the four objects, from each centre to its outline's."""
    return float(sum(np.hypot(*(np.asarray(objects[n].pos[:2]) - np.asarray(HOME[n]))) for n in ITEMS))


def score(objects):
    """0.25 for each object home (success, all four, scores 1.0)."""
    return 0.25 * sum(home(objects, name) for name in ITEMS)


# -- scripted demo ------------------------------------------------------------------------------------------------
# Straight-down grips reach only ~20 cm above the table, so every carry stays low. Waypoint.yaw turns the grip
# about the vertical (0 closes the fingers along world y).
BOWL_RIM = 0.075  # the 16 cm bowl is wider than the open gripper, so it is gripped by its rim, 7.5 cm off centre
GRIP_Z = {"bowl": TOP + 0.045, "cube": TOP + CUBE_SIZE / 2, "can": TOP + 0.06, "crackers": TOP + 0.066}
GRIP_OFFSET = {"bowl": (0.0, -BOWL_RIM), "cube": (0.0, 0.0), "can": (0.0, 0.0), "crackers": (0.0, 0.0)}
TRAVEL_Z = {"bowl": TOP + 0.12, "cube": TOP + 0.12, "can": TOP + 0.15, "crackers": TOP + 0.14}
YAW = {"crackers": (np.pi / 2, 0.0)}  # across its 5.3 cm side at the start, and again once turned to its outline


def _move(name):
    dx, dy = GRIP_OFFSET[name]
    pick = (START[name][0] + dx, START[name][1] + dy, GRIP_Z[name])
    place = (HOME[name][0] + dx, HOME[name][1] + dy, GRIP_Z[name] + 0.01)
    yaw, place_yaw = YAW.get(name, (0.0, None))
    return pick_place_waypoints(pick, place, travel_z=TRAVEL_Z[name], yaw=yaw, place_yaw=place_yaw)


def demo():
    """The left arm puts the bowl (gripped by its rim) and then the cube home; the right arm puts the soup can home,
    then the cracker box, turning it a quarter turn on the way to match its outline."""
    return {"left": _move("bowl") + _move("cube"), "right": _move("can") + _move("crackers")}
