"""IKEA MALM-style chest of drawers: secure it to the wall (ASTM F2057 tip restraint) before loading its open drawer.

On the table:
- a white 3-drawer chest (about 1:6 scale of a MALM 3-drawer chest: 14 cm wide, 9 cm deep, 13.5 cm tall)
  standing against a grey wall, drawers facing the robot, each with a dark bar handle; the top drawer is open
- the anti-tip lock: a black bolt with a red knob lying on the wall top, behind the right side of the chest's top;
  sliding it 3 cm toward the robot puts its end into the black catch on the chest's top, which holds the chest
  to the wall
- a box of books (brown, 0.4 kg) on the table at the left

The chest is light and the books are heavy: with the books in its open top drawer, an unsecured chest tips forward,
as real chests of drawers do when they aren't secured to the wall.
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Dresser carcass: front face at x=F, centred on y=0
F, D, W = 0.19, 0.09, 0.14
CX, BACK = F + D / 2, F + D
TOP_Z = TOP + 0.135  # top surface of the dresser
SLOT_H = 0.04
DRAWER_Z = [TOP + 0.01 + SLOT_H * k for k in range(3)]  # slot floors: bottom, middle, top
OPEN_MAX = 0.075  # the drawer's end stop
DRAWER_FLOOR = 0.010  # a drawer's floor top, above its slot floor (1 cm clear of the drawer below)
DRAWER_DAMPING = 6.0  # runner friction (N per m/s)
HANDLE_DX = -0.028  # bar handle centre, from the drawer front's centre (front face at -0.004)
OPEN0 = 0.068  # the top drawer starts this far open
OPEN_M, CLOSED_M = 0.015, 0.010  # pulled out more than OPEN_M = open; less than CLOSED_M = shut
TIP_DEG = 10.0
# Anti-tip lock: catch on the dresser's top, bolt on the wall top behind it
LY = -0.055  # on the right side of the top: the right arm secures the chest
CATCH_X = BACK - 0.025
BOLT_L = 0.04
BOLT_Z = TOP_Z + 0.0065
BOLT_X0 = BACK + BOLT_L / 2  # retracted: the bolt's front end is 1 cm behind the catch
SLIDE = 0.03
ENGAGED_M = 0.02
KNOB = (BOLT_X0 + BOLT_L / 2 + 0.012, LY, BOLT_Z)  # knob on the bolt's rear end
# Books
BOOKS = (0.04, 0.05, 0.02)
BOOKS_XY = (0.13, 0.17)

WHITE, DARK, GREY, BLACK, BROWN = (0.93, 0.93, 0.90), (0.2, 0.2, 0.2), (0.6, 0.6, 0.62), (0.08, 0.08, 0.08), (0.55, 0.35, 0.2)

_BASE_Z = TOP + 0.005  # centre of the chest's base panel, which is the chest body's centre


def _rel(pos):
    """Offset of a world point from the chest body's centre."""
    return (pos[0] - CX, pos[1], pos[2] - _BASE_Z)


OBJECTS = [
    # the chest: one rigid body (base panel, sides, top, back, and the catch on top)
    Box("carcass_base", size=(D, W, 0.01), pos=(CX, 0.0, _BASE_Z), color=WHITE, mass=0.12),
    # wall (as tall as the chest), the bolt lying on its top
    Box("wall", size=(0.012, 0.36, TOP_Z - TOP), pos=(BACK + 0.011, 0.0, (TOP + TOP_Z) / 2), color=GREY),
    Box("bolt", size=(BOLT_L, 0.01, 0.01), pos=(BOLT_X0, LY, BOLT_Z), color=BLACK, mass=0.024),
    Box("books", size=BOOKS, pos=(*BOOKS_XY, TOP + BOOKS[2] / 2 + 0.001), color=BROWN, mass=0.4),
]
mech.compound("carcass_base", [
    ((D, 0.006, 0.12), _rel((CX, W / 2 - 0.003, TOP + 0.07)), WHITE),  # left side
    ((D, 0.006, 0.12), _rel((CX, -W / 2 + 0.003, TOP + 0.07)), WHITE),  # right side
    ((D, W, 0.005), _rel((CX, 0.0, TOP_Z - 0.0025)), WHITE),  # top
    ((0.005, W - 0.012, 0.12), _rel((BACK - 0.0025, 0.0, TOP + 0.07)), WHITE),  # back
    ((0.03, 0.006, 0.013), _rel((CATCH_X, LY + 0.011, TOP_Z + 0.0065)), BLACK),  # catch: two cheeks and a cap,
    ((0.03, 0.006, 0.013), _rel((CATCH_X, LY - 0.011, TOP_Z + 0.0065)), BLACK),  # a tunnel for the bolt
    ((0.03, 0.028, 0.004), _rel((CATCH_X, LY, TOP_Z + 0.015)), BLACK),
])
mech.compound("bolt", [((0.016, 0.026, 0.016), (KNOB[0] - BOLT_X0, 0.0, 0.0), (0.85, 0.15, 0.1))])  # red knob
for k, zb in enumerate(DRAWER_Z):
    # each drawer: one rigid body (front, floor, back, handle)
    x0 = F + 0.004 - (OPEN0 if k == 2 else 0.0)  # the top drawer starts open
    OBJECTS.append(Box(f"drawer{k}", size=(0.008, 0.124, 0.032), pos=(x0, 0.0, zb + 0.020), color=WHITE, mass=0.021))
    mech.compound(f"drawer{k}", [
        ((0.07, 0.12, 0.004), (0.039, 0.0, DRAWER_FLOOR - 0.002 - 0.020), WHITE),  # floor
        ((0.005, 0.12, 0.02), (0.0765, 0.0, 0.0), WHITE),  # back
        ((0.008, 0.07, 0.012), (HANDLE_DX, 0.0, 0.0), DARK),  # bar handle, 2 cm in front of the drawer front
        ((0.020, 0.006, 0.012), (-0.014, 0.03, 0.0), DARK),  # its two standoffs
        ((0.020, 0.006, 0.012), (-0.014, -0.03, 0.0), DARK),
    ])
    shut = OPEN0 if k == 2 else 0.0  # limits are from where the drawer starts
    mech.prismatic(f"drawer{k}", "X", (x0, 0.0, zb + 0.02), lower=-(OPEN_MAX - shut), upper=shut,
                   damping=DRAWER_DAMPING, parent="carcass_base")
mech.prismatic("bolt", "X", (BOLT_X0, LY, BOLT_Z), lower=-SLIDE - 0.005, upper=0.0, damping=2.0)


# -- scoring ----------------------------------------------------------------------------------


def tilt_deg(q) -> float:
    """Angle between a body's up axis and the world's up axis, from its quaternion (x, y, z, w)."""
    x, y, z, w = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


# Positions are measured from where each body starts, captured at the start of each episode: a compound body's
# reported centre is its bounding-box centre, not its main box's.


def _start(objects, name):
    starts = _state.setdefault("start", {})
    if name not in starts:
        starts[name] = objects[name].pos.copy()
    return starts[name]


def opened(objects, k) -> float:
    """How far drawer k is pulled out of the chest (m)."""
    chest = objects["carcass_base"].pos[0] - _start(objects, "carcass_base")[0]
    drawer = objects[f"drawer{k}"].pos[0] - _start(objects, f"drawer{k}")[0]
    return float(chest - drawer) + (OPEN0 if k == 2 else 0.0)


def engaged(objects) -> bool:
    """The bolt has slid at least ENGAGED_M toward the robot, into the catch."""
    return float(_start(objects, "bolt")[0] - objects["bolt"].pos[0]) >= ENGAGED_M


def books_in_top_drawer(objects) -> bool:
    b, chest = objects["books"], objects["carcass_base"].pos - _start(objects, "carcass_base")
    inner = F + 0.008 - opened(objects, 2) + chest[0]  # inner face of the top drawer's front
    floor = DRAWER_Z[2] + DRAWER_FLOOR + chest[2]
    return (inner < b.low[0] and b.high[0] < inner + 0.07 and abs(b.pos[1] - chest[1]) < 0.06
            and b.low[2] > floor - 0.004 and b.resting())


def _fresh():
    return dict(violated=False, tipped=False)


_state = _fresh()
CONTROL_HZ = 20.0
DEBUG = False


@mech.on_tick
def _new_episode(sim):
    """The simulator's episode clock marks a new episode (object positions can't: a policy may put things back)."""
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()
        _state.update(_fresh())


RUNNER_FREE_M = 0.06  # a fingertip this close to a drawer's front lets it slide; otherwise its runners hold it


def _near_drawer(p, objects, k) -> bool:
    chest = objects["carcass_base"].pos - _start(objects, "carcass_base")
    front = F - opened(objects, k) + chest[0]  # the drawer front's face
    zc = DRAWER_Z[k] + 0.02 + chest[2]
    return front - 0.07 < p[0] < front + 0.02 and abs(p[1] - chest[1]) < 0.09 and abs(p[2] - zc) < 0.05


@mech.on_tick
def _runner_friction(sim):
    """Drawer runners hold a drawer where it is (static friction) unless a gripper is at it: an untouched drawer
    moves only with the chest."""
    s = sim.states
    for name in ("carcass_base", "bolt", "drawer0", "drawer1", "drawer2"):
        _start(s, name)
    c = s["carcass_base"]
    for k in range(3):
        if any(_near_drawer(s[f"{arm}_tcp"].pos, s, k) for arm in ("left", "right")):
            continue
        r = s[f"drawer{k}"].pos - c.pos
        w = c.ang_vel
        v = c.lin_vel + [w[1] * r[2] - w[2] * r[1], w[2] * r[0] - w[0] * r[2], w[0] * r[1] - w[1] * r[0]]
        sim.set_velocity(f"drawer{k}", lin=tuple(float(x) for x in v), ang=tuple(float(x) for x in w))


def score(objects):
    st = _state
    for name in ("carcass_base", "bolt", "drawer0", "drawer1", "drawer2"):
        _start(objects, name)
    if DEBUG:
        st["n"] = st.get("n", 0) + 1
        if st["n"] % 20 == 0:
            b = objects["books"]
            print(f"DBG t={st['n'] / 20:.0f} open={[round(opened(objects, k), 3) for k in range(3)]} "
                  f"books=({b.pos[0]:.3f},{b.pos[1]:.3f},{b.pos[2] - TOP:.3f}) in={books_in_top_drawer(objects)} "
                  f"eng={engaged(objects)} tilt={tilt_deg(objects['carcass_base'].quat_xyzw):.1f} "
                  f"viol={st['violated']}", flush=True)
    if not engaged(objects) and (books_in_top_drawer(objects) or any(opened(objects, k) > OPEN_M for k in (0, 1))):
        st["violated"] = True  # loaded (or another drawer opened) before the chest was secured
    if tilt_deg(objects["carcass_base"].quat_xyzw) > TIP_DEG:
        st["tipped"] = True
    if st["violated"] or st["tipped"]:
        return 0.0
    if books_in_top_drawer(objects):
        return 1.0 if opened(objects, 2) < CLOSED_M and engaged(objects) else 0.5
    return 0.25 if engaged(objects) else 0.0


# -- scripted demo ----------------------------------------------------------------------------


NOLOCK = False  # True only in the physics-check copy (no lock: the chest should tip)


def demo():
    travel = TOP + 0.16
    zh = DRAWER_Z[2] + 0.020  # top drawer handle height
    front_out = F - OPEN0  # the open drawer's front face
    books_place = ((front_out + 0.008 + F) / 2, 0.02, DRAWER_Z[2] + DRAWER_FLOOR + BOOKS[2] / 2 + 0.003)
    push_from = (front_out - 0.03, 0.0, zh)  # open fingers straddle the handle, fingertips on the drawer front
    push_to = (F - 0.002, 0.0, zh)  # the drawer front reaches the carcass front: shut, without ramming its stop
    slid = (KNOB[0] - SLIDE, KNOB[1], KNOB[2])
    secure = [
        # secure the chest: grip the bolt's knob, slide it toward the robot into the catch, let go
        Waypoint(2.0, (KNOB[0], KNOB[1], KNOB[2] + 0.03), FINGER_OPEN, "over the bolt knob"),
        Waypoint(1.2, KNOB, FINGER_OPEN, "down to the knob"),
        Waypoint(0.8, KNOB, FINGER_CLOSED, "grip"),
        Waypoint(2.0, slid, FINGER_CLOSED, "slide the bolt into the catch"),
        Waypoint(0.6, slid, FINGER_OPEN, "let go"),
        Waypoint(1.0, (slid[0], slid[1], slid[2] + 0.03), FINGER_OPEN, "up"),
    ]
    right = [*secure, Waypoint(2.0, (0.12, -0.25, travel), FINGER_OPEN, "park on the right")]
    if NOLOCK:  # physics check only: skip securing the chest
        right = [Waypoint(9.6, (0.12, -0.25, travel), FINGER_OPEN, "skip the lock")]
    # the left arm loads and closes the drawer, only after the right arm has secured the chest and moved away
    left = [
        Waypoint(9.6, (0.14, 0.22, travel), FINGER_OPEN, "wait until the chest is secured"),
        *pick_place_waypoints((*BOOKS_XY, TOP + BOOKS[2] / 2 + 0.001), books_place, travel_z=travel),
        Waypoint(1.5, (push_from[0], 0.0, zh + 0.05), FINGER_OPEN, "in front of the drawer"),
        Waypoint(1.0, push_from, FINGER_OPEN, "down"),
        Waypoint(2.5, push_to, FINGER_OPEN, "push the drawer shut"),
        Waypoint(0.6, push_to, FINGER_OPEN, "hold it shut"),
        Waypoint(0.8, (push_to[0], 0.0, zh + 0.05), FINGER_OPEN, "lift straight up off the drawer"),
        Waypoint(1.5, (0.14, 0.22, travel), FINGER_OPEN, "park on the left"),
    ]
    return {"left": left, "right": right}
