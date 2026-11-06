"""On the station table, in the right arm's half: three phones (15 x 7.5 x 0.8 cm, 180 g) lie screen up in a row
along y at x = 0.39, at y = -0.07, -0.19 and -0.31, each on a 2.5 cm foam block narrower than the phone, their top
ends away from the robot. Each has an orange power button on its right edge (the -y side), 3 cm from its centre
toward its top end. The button stands 3.5 mm proud and is sprung, with 2.5 mm of travel. All three screens are dark.
In front of the phones, nearer the robot, stand a green tray at (0.20, -0.09) and a grey tray at (0.20, -0.25), each
18 x 12 x 3 cm, their long sides along x.

The task: hold each phone's power button for 3 s, then put the phones whose screens light up in the green tray and
the others in the grey tray. Only the middle phone (phone_2) works: once its button has been held in 1.5 mm or more
for 2.5 s without a break, its screen turns white and stays white. A phone only counts if it was tested that way
before it entered a tray.

The button is a box on a sprung prismatic joint (``bonds.py``), and the screens are meshes modelled in Blender
(``looks.py``, ``looks/make_looks.py``), shown or hidden by a step hook.

Run it:  inspect-robots run --embodiment yam_arena -E task=phone_power_test_sort.py --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bonds import Sliding, on_reset, on_step  # noqa: E402
from looks import dress, show  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the phones, on their foam blocks --------------------------------------------------------------------------------
PHONE_SIZE = (0.15, 0.075, 0.008)  # long (x), wide (y), thick
PHONE_MASS = 0.17  # kg, 180 g with its button
PHONE_X = 0.39
PHONE_YS = (-0.07, -0.19, -0.31)
BLOCK_SIZE = (0.11, 0.045, 0.025)  # narrower than the phone: the button and the phone's edges hang free
PHONE_Z = TOP + BLOCK_SIZE[2] + PHONE_SIZE[2] / 2
NAMES = ("phone_1", "phone_2", "phone_3")
BUTTONS = ("button_1", "button_2", "button_3")
WORKING = 1  # index of the phone that switches on: phone_2, the middle one

BLOCKS = [
    Box(f"block_{i + 1}", pos=(PHONE_X, y, TOP + BLOCK_SIZE[2] / 2), size=BLOCK_SIZE, color=(0.10, 0.11, 0.12))
    for i, y in enumerate(PHONE_YS)
]
PHONES = [
    Box(name, pos=(PHONE_X, y, PHONE_Z), size=PHONE_SIZE, mass=PHONE_MASS, color=(0.05, 0.05, 0.05))
    for name, y in zip(NAMES, PHONE_YS)
]

# -- the power buttons: sprung, on the right edge (-y), pressed toward +y ------------------------------------------
BUTTON_SIZE = (0.03, 0.0035, 0.005)  # along the edge, proud of it, tall
BUTTON_DX = 0.03  # m from the phone's centre toward its top end (+x)
BUTTON_TRAVEL = 0.0025  # m: it goes in 2.5 mm and stays 1 mm proud
BUTTON_RATE = 1200.0  # N/m: 3 N at full travel
PRESSED = 0.0015  # m in: the button is held
HOLD_S = 2.5  # s held without a break: the phone has been tested
BUTTON_REST_Y = -(PHONE_SIZE[1] / 2 + BUTTON_SIZE[1] / 2)  # the button's centre from the phone's, in its frame
BUTTON_ORANGE = (1.0, 0.38, 0.0)
BUTTON_OBJECTS = [
    Sliding(
        name,
        pos=(PHONE_X + BUTTON_DX, y + BUTTON_REST_Y, PHONE_Z),
        size=BUTTON_SIZE,
        mass=0.01,
        color=BUTTON_ORANGE,
        to=phone,
        axis="Y",
        lower=0.0,
        upper=BUTTON_TRAVEL,
        rate=BUTTON_RATE,
        damping=5.0,
        collide=False,  # it sinks into the phone's box
    )
    for name, phone, y in zip(BUTTONS, NAMES, PHONE_YS)
]

# -- the trays: 18 x 12 x 3 cm, 6 mm walls, 4 mm floor, fixed, in front of the phones -------------------------------
TRAY_OUTER = (0.18, 0.12)
TRAY_H = 0.03
TRAY_WALL = 0.006
TRAY_FLOOR_T = 0.004
TRAY_FLOOR_TOP = TOP + TRAY_FLOOR_T
TRAY_X = 0.20
TRAYS = {"green": (TRAY_X, -0.09), "grey": (TRAY_X, -0.25)}
TRAY_COLORS = {"green": (0.20, 0.62, 0.28), "grey": (0.55, 0.56, 0.57)}


def _tray(name: str) -> list[Box]:
    tx, ty = TRAYS[name]
    color = TRAY_COLORS[name]
    hx, hy = TRAY_OUTER[0] / 2 - TRAY_WALL / 2, TRAY_OUTER[1] / 2 - TRAY_WALL / 2
    wall_h = TRAY_H - TRAY_FLOOR_T
    wz = TRAY_FLOOR_TOP + wall_h / 2
    inner_x = TRAY_OUTER[0] - 2 * TRAY_WALL
    return [
        Box(f"tray_{name}", pos=(tx, ty, TOP + TRAY_FLOOR_T / 2), size=(*TRAY_OUTER, TRAY_FLOOR_T), color=color),
        Box(f"tray_{name}_near", pos=(tx - hx, ty, wz), size=(TRAY_WALL, TRAY_OUTER[1], wall_h), color=color),
        Box(f"tray_{name}_far", pos=(tx + hx, ty, wz), size=(TRAY_WALL, TRAY_OUTER[1], wall_h), color=color),
        Box(f"tray_{name}_right", pos=(tx, ty - hy, wz), size=(inner_x, TRAY_WALL, wall_h), color=color),
        Box(f"tray_{name}_left", pos=(tx, ty + hy, wz), size=(inner_x, TRAY_WALL, wall_h), color=color),
    ]


TRAY_BOXES = _tray("green") + _tray("grey")

OBJECTS = [*BLOCKS, *TRAY_BOXES, *PHONES, *BUTTON_OBJECTS]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the boxes' colliders unchanged. Each
# phone's mesh has a dark screen and a white one over it, hidden until the phone switches on. Each tray's mesh, on its
# floor box, also draws its walls.
LOOKS = {name: f"{name}.usdc" for name in NAMES} | {"tray_green": "tray_green.usdc", "tray_grey": "tray_grey.usdc"}
SCREEN_ON = "screen_on"
dress(LOOKS, hide=[b.name for b in TRAY_BOXES if b.name not in LOOKS], hidden=[SCREEN_ON])

# -- scoring ---------------------------------------------------------------------------------------------------------
IN_TRAY_XY = (TRAY_OUTER[0] / 2 - TRAY_WALL, TRAY_OUTER[1] / 2 - TRAY_WALL)  # (0.084, 0.054): over the inner floor
IN_TRAY_Z = 0.04  # m above the tray floor: lower than this, the phone lies in the tray (or on another phone)
ENTER_XY = (TRAY_OUTER[0] / 2, TRAY_OUTER[1] / 2)  # over the tray's outline
ENTER_Z = TRAY_H + 0.02  # m above the table: lower than this, over a tray, a phone has entered it
OFF_TABLE_Z = -0.02  # m above the table: a phone whose centre is lower has left the table

# What the episode remembers, per phone: how long its button has been held without a break, whether it has been
# tested (held 2.5 s), and whether it entered a tray before it was tested (it can no longer count).
STATE = {"held": [0, 0, 0], "tested": [False] * 3, "early": [False] * 3, "dt": 0.05}


@on_reset
def reset_memory(env=None) -> None:
    STATE["held"] = [0, 0, 0]
    STATE["tested"] = [False] * 3
    STATE["early"] = [False] * 3
    for name in NAMES:
        show(name, SCREEN_ON, False)


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) for s in objects.values())


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def button_depth(objects, i: int) -> float:
    """How far phone i's button is pressed in (m), in the phone's own frame, from where it rests."""
    phone, button = objects[NAMES[i]], objects[BUTTONS[i]]
    local = _rotation(phone.quat_xyzw).T @ (np.asarray(button.pos) - np.asarray(phone.pos))
    return float(local[1] - BUTTON_REST_Y)


def pressed(objects, i: int) -> bool:
    return button_depth(objects, i) >= PRESSED


def entered(objects, i: int, tray: str) -> bool:
    """Phone i's centre is over the tray's 18 x 12 cm outline and less than 5 cm above the table."""
    c = objects[NAMES[i]].pos
    tx, ty = TRAYS[tray]
    return bool(abs(c[0] - tx) < ENTER_XY[0] and abs(c[1] - ty) < ENTER_XY[1] and c[2] < TOP + ENTER_Z)


def in_tray(objects, i: int, tray: str) -> bool:
    """Phone i lies in the tray: its centre over the inner floor (within 8.4 x 5.4 cm of the tray's centre) and
    under 4 cm above the floor."""
    c = objects[NAMES[i]].pos
    tx, ty = TRAYS[tray]
    return bool(abs(c[0] - tx) < IN_TRAY_XY[0] and abs(c[1] - ty) < IN_TRAY_XY[1] and c[2] < TRAY_FLOOR_TOP + IN_TRAY_Z)


def right_tray(i: int) -> str:
    return "green" if i == WORKING else "grey"


def lit(i: int) -> bool:
    """Phone i's screen is on: it is the working phone and it has been tested."""
    return bool(i == WORKING and STATE["tested"][i])


def update(objects, dt: float) -> None:
    """One control step of the memory: the hold timers, the tests and early entries into a tray."""
    for i in range(3):
        STATE["held"][i] = STATE["held"][i] + 1 if pressed(objects, i) else 0
        if STATE["held"][i] * dt >= HOLD_S - 1e-6:
            STATE["tested"][i] = True
        if not STATE["tested"][i] and any(entered(objects, i, t) for t in TRAYS):
            STATE["early"][i] = True


@on_step
def test_phones(env) -> None:
    """Advance the hold timers, and turn the working phone's screen white once it has been tested."""
    from yam_arena.scene import object_states

    dt = float(getattr(getattr(env, "unwrapped", env), "step_dt", STATE["dt"]))
    STATE["dt"] = dt
    objects = object_states(env, [*NAMES, *BUTTONS])
    if _finite(objects):
        update(objects, dt)
    for i, name in enumerate(NAMES):
        if lit(i):
            show(name, SCREEN_ON, True)


def _violation(objects) -> bool:
    if not _finite(objects):
        return True
    return any(objects[n].pos[2] < TOP + OFF_TABLE_Z for n in NAMES)


def counted(objects, i: int) -> bool:
    """Phone i counts: tested before it entered any tray, lying still in its right tray."""
    phone = objects[NAMES[i]]
    return bool(STATE["tested"][i] and not STATE["early"][i] and in_tray(objects, i, right_tray(i)) and phone.resting())


def success(objects) -> bool:
    """All three phones count."""
    return not _violation(objects) and all(counted(objects, i) for i in range(3))


def score(objects) -> float:
    """1/3 for each phone that counts: its button was held 1.5 mm in for 2.5 s without a break before it first
    entered a tray, and it lies still in the right tray (green if its screen lit up, grey if not). 0 for a phone
    never tested or tested only after it entered a tray. 0 overall if any phone has left the table or the state is
    not finite."""
    if _violation(objects):
        return 0.0
    return sum(counted(objects, i) for i in range(3)) / 3


# -- scripted demo ---------------------------------------------------------------------------------------------------
# The right arm alone, every grip straight down with the fingers closing along y, across the phone. It closes on each
# phone where the button is, so one pad squeezes the button in (the pads push 50 N, the button's spring 3 N) and the
# other holds the phone's left edge: the hand holds the phone and its button at once. It keeps squeezing on the foam
# block for 3 s, lifts and carries the phone, still squeezed, to its tray, and lets it drop the last 12 mm. The
# right arm works in its own half: every waypoint has y < -0.03. Numbers are TCP positions, which sit 13 mm above
# the pad tips.
TEST_ORDER = (2, 1, 0)  # phone_3, then phone_2, then phone_1 (on top of phone_3 in the grey tray)
DEMO_LIT = WORKING  # the phone the demo puts in the green tray
GRIP_DX = BUTTON_DX  # m from the phone's centre along x: on the button
GRIP_DY = -BUTTON_SIZE[1] / 2  # the middle of the phone and its button, across: -1.75 mm
GRIP_Z = PHONE_Z  # the pads span the phone's edge, their tips 9 mm below it, beside the block
HOLD_DEMO_S = 3.0
SQUEEZE_SHIFT = -0.011  # m along y: where the squeezed phone sits from the commanded TCP (measured in the sim:
# the pads push it 10 to 12 mm toward its button side as they close), so the demo aims the phone, not the TCP
HOVER_Z = TOP + 0.10
DROP = 0.012  # m: the phone's bottom above where it lands when let go (the pad tips 3 mm above it)
FINGER_RELEASE = -0.043  # pads 86 mm apart: clear of the phone, inside the tray's walls


def demo():
    w = Waypoint
    plan = []
    landed = {"green": 0, "grey": 0}
    for i in TEST_ORDER:
        x, y = PHONE_X + GRIP_DX, PHONE_YS[i] + GRIP_DY
        tray = "green" if i == DEMO_LIT else "grey"
        tx, ty = TRAYS[tray]
        px, py = tx + GRIP_DX, ty - SQUEEZE_SHIFT  # the TCP that puts the phone's centre over the tray's
        place_z = TRAY_FLOOR_TOP + landed[tray] * PHONE_SIZE[2] + DROP + PHONE_SIZE[2] / 2
        landed[tray] += 1
        n = NAMES[i]
        plan += [
            w(1.5, (x, y, HOVER_Z), FINGER_OPEN, f"over {n}"),
            w(1.0, (x, y, GRIP_Z), FINGER_OPEN, f"round {n}"),
            w(0.6, (x, y, GRIP_Z), FINGER_CLOSED, f"squeeze {n}"),
            w(HOLD_DEMO_S, (x, y, GRIP_Z), FINGER_CLOSED, f"hold {n}"),
            w(1.0, (x, y, HOVER_Z), FINGER_CLOSED, f"lift {n}"),
            w(1.5, (px, py, HOVER_Z), FINGER_CLOSED, f"carry {n} to {tray}"),
            w(1.0, (px, py, place_z), FINGER_CLOSED, f"lower {n}"),
            w(0.5, (px, py, place_z), FINGER_RELEASE, f"release {n}"),
            w(0.8, (px, py, HOVER_Z), FINGER_RELEASE, f"rise from {n}"),
        ]
    return {"right": plan}
