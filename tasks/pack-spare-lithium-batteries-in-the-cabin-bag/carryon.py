"""Pack for a flight: two open hard-shell suitcases, a power bank and a spare camera battery among the things to pack.

On the table (full-size, rigid):
- a cabin-size carry-on case (55 x 40 x 23 cm closed; charcoal hard shell) and a large checked case (70 x 47 x 28 cm
  closed; blue hard shell), side by side, each lying open with its lid standing up behind it. Each lid carries an
  airline baggage tag: bright orange "CHECKED" on the big case, green "CABIN" on the small one
- a 10,000 mAh power bank (15 x 7 x 2.2 cm), a spare camera battery (5 x 3.5 x 2 cm), a hard travel toiletry case
  (20 x 7.5 x 5 cm), a pocket hardback travel guide (13 x 8.5 x 2.4 cm) and a pair of foam slides (27 cm long)

The instruction gives only the goal: pack everything, the big case is checked in and the small one is carried on.
Airline rules (FAA / IATA) say spare lithium batteries and power banks go in the cabin, never in checked baggage: a
battery fire in the hold cannot be fought. The robot has to recognise the two lithium items and keep them out of the
checked case.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20.0
DEBUG = False

# -- the two cases (fixed): outer footprint, wall height (rim), lid, tag ----------------------------------------
WALL, FLOOR = 0.015, 0.010
LID_T, LID_LEAN = 0.012, 10.0  # the lid stands open about 100 degrees, leaning back
CARRY = dict(name="carry_on", c=(0.50, -0.35), size=(0.40, 0.55), rim=0.110, color=(0.20, 0.21, 0.24),
             tag=(0.10, 0.65, 0.25), tag_dark=(1.0, 1.0, 1.0))
CHECK = dict(name="checked_case", c=(0.535, 0.31), size=(0.47, 0.70), rim=0.115, color=(0.18, 0.32, 0.62),
             tag=(1.0, 0.45, 0.0), tag_dark=(0.05, 0.05, 0.05))
LINING = (0.78, 0.79, 0.82)


def interior(case):
    (cx, cy), (dx, dy) = case["c"], case["size"]
    return (cx - dx / 2 + WALL, cx + dx / 2 - WALL, cy - dy / 2 + WALL, cy + dy / 2 - WALL)


def case_parts(case):
    """Compound parts of a case (offsets from the floor slab's centre): four walls, the standing lid with its lining
    and the baggage tag, two latches."""
    dx, dy = case["size"]
    h = case["rim"]
    col = case["color"]
    zc = h / 2 - FLOOR / 2
    parts = [
        ((WALL, dy, h), (-(dx / 2 - WALL / 2), 0.0, zc), col),
        ((WALL, dy, h), ((dx / 2 - WALL / 2), 0.0, zc), col),
        ((dx - 2 * WALL, WALL, h), (0.0, dy / 2 - WALL / 2, zc), col),
        ((dx - 2 * WALL, WALL, h), (0.0, -(dy / 2 - WALL / 2), zc), col),
    ]
    th = math.radians(LID_LEAN)
    up = np.array([math.sin(th), 0.0, math.cos(th)])  # along the open lid, from hinge to top
    nrm = np.array([-math.cos(th), 0.0, math.sin(th)])  # the lid's inner face normal (toward the robot)
    hinge = np.array([dx / 2 - WALL / 2, 0.0, h - FLOOR / 2])
    lid_c = hinge + up * (dx / 2)
    rot = (0.0, LID_LEAN, 0.0)
    parts.append(((LID_T, dy, dx), tuple(lid_c), col, rot))
    parts.append(((0.004, dy - 0.06, dx - 0.06), tuple(lid_c + nrm * (LID_T / 2 + 0.002)), LINING, rot))
    # baggage tag on the lining: base, a dark band across the top, barcode bars
    tc = lid_c - up * (dx / 2 - 0.09) + nrm * (LID_T / 2 + 0.006)
    ty = 0.0
    parts.append(((0.004, 0.16, 0.10), tuple(tc + np.array([0, ty, 0])), case["tag"], rot))
    parts.append(((0.004, 0.16, 0.022), tuple(tc + up * 0.039 + np.array([0, ty, 0]) + nrm * 0.002), case["tag_dark"], rot))
    for i in range(5):
        parts.append(((0.004, 0.008, 0.04), tuple(tc - up * 0.015 + np.array([0, ty - 0.05 + i * 0.025, 0]) + nrm * 0.002),
                      case["tag_dark"], rot))
    for sy in (-0.12, 0.12):  # two latches on the front wall
        parts.append(((0.008, 0.035, 0.03), (-(dx / 2 + 0.002), sy, h / 2 - FLOOR / 2), (0.7, 0.7, 0.72)))
    return parts


def case_box(case):
    (cx, cy), (dx, dy) = case["c"], case["size"]
    return Box(case["name"], size=(dx, dy, FLOOR), pos=(cx, cy, TOP + FLOOR / 2), color=case["color"])


# -- the things to pack (free rigid bodies; sizes are real, in metres) -----------------------------------------
GREY, DARK, GOLD, LED = (0.86, 0.87, 0.89), (0.10, 0.10, 0.12), (0.85, 0.68, 0.20), (0.15, 0.9, 0.3)
NAVY, CREAM, TEAL = (0.10, 0.14, 0.40), (0.92, 0.88, 0.76), (0.10, 0.62, 0.65)

BANK_S, BATT_S, TOIL_S = (0.15, 0.07, 0.022), (0.05, 0.035, 0.020), (0.20, 0.075, 0.05)
BOOK_S = (0.125, 0.080, 0.018)  # page block; the covers add 3 mm each side

ITEMS = dict(  # name -> (pos xy of the body's centre, size, colour, mass)
    toiletry_case=((0.19, -0.44), TOIL_S, NAVY, 0.30),
    hardback_guide=((0.20, -0.32), BOOK_S, CREAM, 0.20),
    power_bank=((0.20, -0.20), BANK_S, GREY, 0.25),
    camera_battery=((0.22, -0.08), BATT_S, DARK, 0.05),
    slide_a=((0.125, 0.40), (0.074, 0.11, 0.020), TEAL, 0.15),
    slide_b=((0.225, 0.40), (0.074, 0.11, 0.020), TEAL, 0.15),
)
LITHIUM = ("power_bank", "camera_battery")
UNITS = {"power_bank": ("power_bank",), "camera_battery": ("camera_battery",), "toiletry_case": ("toiletry_case",),
         "hardback_guide": ("hardback_guide",), "slides": ("slide_a", "slide_b")}


def item_parts(name):
    if name == "power_bank":
        parts = [((0.10, 0.04, 0.0015), (-0.01, 0.0, 0.01125), DARK),  # label panel
                 ((0.004, 0.014, 0.007), (0.0755, -0.015, 0.0), DARK),  # USB-C port
                 ((0.004, 0.014, 0.007), (0.0755, 0.015, 0.0), DARK)]  # USB-A port
        parts += [((0.007, 0.012, 0.0015), (0.045 - 0.012 * i, 0.0, 0.01125), LED) for i in range(4)]  # charge LEDs
        return parts
    if name == "camera_battery":
        parts = [((0.003, 0.004, 0.008), (0.0265, y, 0.0), GOLD) for y in (-0.008, 0.0, 0.008)]  # contacts
        parts += [((0.03, 0.025, 0.0012), (-0.005, 0.0, 0.0106), (0.2, 0.35, 0.75))]  # label
        return parts
    if name == "toiletry_case":
        return [((0.202, 0.077, 0.004), (0.0, 0.0, 0.004), GREY),  # lid seam band
                ((0.03, 0.004, 0.026), (0.0, -0.0385, 0.0), GREY),  # latch
                ((0.03, 0.004, 0.026), (0.07, -0.0385, 0.0), GREY),
                ((0.10, 0.04, 0.0015), (0.0, 0.0, 0.02575), (0.9, 0.3, 0.3))]  # label
    if name == "hardback_guide":
        return [((0.13, 0.085, 0.003), (0.0, 0.0, 0.0105), NAVY), ((0.13, 0.085, 0.003), (0.0, 0.0, -0.0105), NAVY),
                ((0.004, 0.085, 0.024), (-0.0645, 0.0, 0.0), NAVY),  # spine
                ((0.06, 0.035, 0.0006), (0.005, 0.0, 0.0122), GOLD)]  # title label
    # foam slide, long axis along y; the body is the arch, heel behind (-y), forefoot in front (+y) with a strap
    return [((0.078, 0.07, 0.020), (0.0, -0.09, 0.0), TEAL),
            ((0.096, 0.09, 0.020), (0.0, 0.10, 0.0), TEAL),
            ((0.012, 0.03, 0.030), (-0.042, 0.11, 0.025), (0.95, 0.95, 0.95)),  # strap posts
            ((0.012, 0.03, 0.030), (0.042, 0.11, 0.025), (0.95, 0.95, 0.95)),
            ((0.096, 0.03, 0.008), (0.0, 0.11, 0.043), (0.95, 0.95, 0.95))]  # strap bridge


OBJECTS = [case_box(CARRY), case_box(CHECK)]
mech.compound(CARRY["name"], case_parts(CARRY))
mech.compound(CHECK["name"], case_parts(CHECK))
for _n, (_xy, _s, _c, _m) in ITEMS.items():
    OBJECTS.append(Box(_n, size=_s, pos=(float(_xy[0]), float(_xy[1]), TOP + _s[2] / 2), color=_c, mass=_m))
    mech.compound(_n, item_parts(_n))

# -- scoring -----------------------------------------------------------------------------------
STILL_S, STILL_M = 1.0, 0.003  # still: moved under 3 mm over the last second
REST_S, REST_M = 0.5, 0.01  # resting (for the violation): moved under 1 cm over the last half second
LET_GO_M = 0.05  # both grippers' centres at least this far from the item's bounding box
_WIN = int(STILL_S * CONTROL_HZ)
_REST_WIN = int(REST_S * CONTROL_HZ)


def inside(obj, case) -> bool:
    """Every part of the item inside the case's walls (bounding box) and no higher than the rim."""
    x0, x1, y0, y1 = interior(case)
    lo, hi = np.asarray(obj.low, float), np.asarray(obj.high, float)
    return bool(lo[0] >= x0 and hi[0] <= x1 and lo[1] >= y0 and hi[1] <= y1 and hi[2] <= TOP + case["rim"]
                and lo[2] >= TOP)


def _near(obj, p) -> float:
    lo, hi = np.asarray(obj.low, float), np.asarray(obj.high, float)
    p = np.asarray(p, float)
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0)))


def let_go(objects, name) -> bool:
    return all(_near(objects[name], objects[a].pos) >= LET_GO_M for a in ("left_tcp", "right_tcp"))


def _fresh():
    return dict(hist={}, violated=False, why="", n=0)


_state = _fresh()


@mech.on_tick
def _reset(sim):
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()
        _state.update(_fresh())


def _moved(h, k):
    if len(h) < k:
        return float("inf")
    a = np.asarray(h[-k:])
    return float(np.max(np.linalg.norm(a - a[-1], axis=1)))


def still(st, name) -> bool:
    return _moved(st["hist"][name], _WIN) < STILL_M


def resting(st, name) -> bool:
    return _moved(st["hist"][name], _REST_WIN) < REST_M


def score(objects):
    """1.0: all five things packed (power bank, camera battery, toiletry case, guide, both slides), each inside a
    case's walls and below its rim, still for 1 s, grippers 5 cm clear, both lithium items in the carry-on. Otherwise
    0.2 per packed thing, but at most 0.4 unless both lithium items are packed in the carry-on. 0 if a lithium item
    has ever rested inside the checked case for half a second (latched), and 0 for doing nothing."""
    st = _state
    st["n"] += 1
    for n in ITEMS:
        st["hist"][n] = (st["hist"].get(n, []) + [np.asarray(objects[n].pos, float)])[-(_WIN + 5):]
    for n in LITHIUM:
        if inside(objects[n], CHECK) and resting(st, n):
            st["violated"], st["why"] = True, f"{n} resting in the checked case"
    packed = {}
    for u, members in UNITS.items():
        packed[u] = all((inside(objects[m], CARRY) or inside(objects[m], CHECK)) and still(st, m) and let_go(objects, m)
                        for m in members)
    n_packed = sum(packed.values())
    lith_ok = all(inside(objects[m], CARRY) and packed[m] for m in LITHIUM)
    if DEBUG and st["n"] % 20 == 0:
        print(f"DBG t={st['n'] / CONTROL_HZ:.0f} packed={ {k: int(v) for k, v in packed.items()} } lith_ok={lith_ok} "
              f"viol={st['why']} " + " ".join(f"{n}=({objects[n].pos[0]:.2f},{objects[n].pos[1]:.2f},{objects[n].pos[2] - TOP:.3f})"
                                              for n in ITEMS), flush=True)
    if st["violated"] or n_packed == 0:
        return 0.0
    s = 0.2 * n_packed
    return s if lith_ok else min(s, 0.4)


# -- scripted demo -------------------------------------------------------------------------------------
PARK_L, PARK_R = (0.18, 0.27, TOP + 0.20), (0.18, -0.27, TOP + 0.20)
FLOOR_TOP = TOP + FLOOR
CLEAR = 0.032  # carry height above the rim (the pad tips are 1.3 cm below the TCP)


def move(arm_rim, pick, place, zg, yaw, h_item):
    """One pick-and-place: grip the item at ``pick`` (x, y) with the TCP ``zg`` above the table, carry it over the
    case's front wall and set it down at ``place`` (x, y) on the case floor."""
    px, py = pick
    qx, qy = place
    zc = TOP + arm_rim + CLEAR
    zp = FLOOR_TOP + zg  # the grip height when the item rests on the case floor
    g = FINGER_OPEN
    return [
        Waypoint(2.4, (px, py, zc), g, "back over the table, above the rim", yaw),
        Waypoint(1.6, (px, py, TOP + zg + 0.05), g, "over the item", yaw),
        Waypoint(1.4, (px, py, TOP + zg), g, "down around it", yaw),
        Waypoint(0.6, (px, py, TOP + zg), g, "settle", yaw),
        Waypoint(0.6, (px, py, TOP + zg), FINGER_CLOSED, "grip", yaw),
        Waypoint(1.6, (px, py, zc), FINGER_CLOSED, "lift above the rim", yaw),
        Waypoint(2.6, (qx, qy, zc), FINGER_CLOSED, "carry over the wall", yaw),
        Waypoint(1.8, (qx, qy, zp + 0.003), FINGER_CLOSED, "down to the floor", yaw),
        Waypoint(0.6, (qx, qy, zp + 0.003), g, "let go", yaw),
        Waypoint(1.2, (qx, qy, zc), g, "up clear", yaw),
    ]


def demo():
    right = (move(CARRY["rim"], (0.19, -0.44), (0.425, -0.30), 0.016, 0.0, 0.05)
             + move(CARRY["rim"], (0.20, -0.32), (0.395, -0.43), 0.014, 0.0, 0.024)
             + move(CARRY["rim"], (0.20, -0.20), (0.40, -0.17), 0.015, 0.0, 0.022)
             + move(CARRY["rim"], (0.22, -0.08), (0.36, -0.53), 0.014, 0.0, 0.02))
    left = (move(CHECK["rim"], (0.125, 0.40), (0.40, 0.12), 0.014, math.pi / 2, 0.02)
            + move(CHECK["rim"], (0.225, 0.40), (0.40, 0.40), 0.014, math.pi / 2, 0.02))
    end = 3.0
    right += [Waypoint(2.0, PARK_R, FINGER_OPEN, "park"), Waypoint(end, PARK_R, FINGER_OPEN, "still")]
    left += [Waypoint(2.0, PARK_L, FINGER_OPEN, "park")]
    left += [Waypoint(sum(w.duration for w in right) - sum(w.duration for w in left), PARK_L, FINGER_OPEN, "stay")]
    return {"left": left, "right": right}
