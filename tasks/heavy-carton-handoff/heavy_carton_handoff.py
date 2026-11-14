"""In a stockroom: three identical small cartons wait on the bench. Two hold packing material; one is packed with lead
fishing weights (2 kg), more than the arm can raise to the 14 cm shelf. Which one is heavy changes from trial to trial,
so the robot only finds out by test-lifting each. It shelves the light cartons and slides the heavy one forward onto
the handoff mat at the colleague's end of the bench, where they can lift it up themselves.
"""

import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import trialkit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BENCH = TOP + 0.003
CL, CW, CH = 0.09, 0.065, 0.05  # carton length (front to back), width and height
START_X, SLOTS = 0.20, (0.10, 0.0, -0.10)
SHELF = (np.array([0.365, 0.31]), np.array([0.13, 0.24]), 0.14)  # centre, footprint, height
MAT = (np.array([0.34, 0.0]), np.array([0.16, 0.34]))  # handoff mat in front of all three slots
CARDBOARD = (0.72, 0.55, 0.36)
MARGIN = 0.01  # how far a carton may overhang the shelf top or the mat

HEAVY = "carton_heavy"
MASSES = {"carton_light_a": 0.3, "carton_light_b": 0.3, HEAVY: float(os.environ.get("HEAVY_KG", 2.0))}
LIGHT = [n for n in MASSES if n != HEAVY]
START_SLOT = 1  # where the heavy carton is declared; each reset moves it to a slot drawn from the trial's seed

CARTONS = [Box(name, pos=(START_X, SLOTS[k], BENCH + CH / 2), size=(CL, CW, CH), mass=m, color=CARDBOARD)
           for (name, m), k in zip(MASSES.items(), (0, 2, START_SLOT))]
DRESSING = [
    Box("bench", pos=(0.45, 0.0, TOP + 0.0015), size=(0.90, 1.0, 0.003), color=(0.62, 0.62, 0.60)),
    Box("shelf", pos=(*map(float, SHELF[0]), BENCH + SHELF[2] / 2), size=(*map(float, SHELF[1]), SHELF[2]),
        color=(0.35, 0.40, 0.48)),
    Box("handoff_mat", pos=(*map(float, MAT[0]), BENCH + 0.0001), size=(*map(float, MAT[1]), 0.0002),
        color=(0.95, 0.75, 0.15)),
    Box("work_gloves", pos=(0.52, -0.06, BENCH + 0.006), size=(0.10, 0.06, 0.012), yaw=0.3, color=(0.20, 0.35, 0.60)),
    Cylinder("coffee_flask", pos=(0.58, -0.18, BENCH + 0.09), radius=0.035, height=0.18, color=(0.15, 0.15, 0.17)),
    Box("clipboard", pos=(0.53, 0.14, BENCH + 0.004), size=(0.12, 0.17, 0.008), yaw=-0.2, color=(0.55, 0.40, 0.25)),
    Cylinder("tape_roll", pos=(0.62, 0.34, BENCH + 0.025), radius=0.045, height=0.05, color=(0.75, 0.65, 0.45)),
]
OBJECTS = [*DRESSING, *CARTONS]

TEST_LIFT, DROP_SPEED = 0.01, 0.6  # m above the bench; m/s downward
_S = {"lifted": False, "dropped": False, "slot": START_SLOT}


@trialkit.on_reset
def _new_trial(environment, rng):
    """Put the heavy carton in a slot drawn from the trial's seed (or HEAVY_SLOT) and the light ones in the rest."""
    forced = os.environ.get("HEAVY_SLOT")
    slot = int(forced) if forced is not None else int(rng.integers(len(SLOTS)))
    others = [k for k in range(len(SLOTS)) if k != slot]
    for name, k in zip((HEAVY, *LIGHT), (slot, *others)):
        trialkit.place(environment, name, (START_X, SLOTS[k], BENCH + CH / 2))
    _S.update(lifted=False, dropped=False, slot=slot)


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _within(obj, area):
    centre, size = area[0], area[1]
    lo, hi = centre - size / 2 - MARGIN, centre + size / 2 + MARGIN
    return bool(np.all(obj.low[:2] >= lo) and np.all(obj.high[:2] <= hi))


def _on_shelf(obj):
    return _within(obj, SHELF) and abs(obj.low[2] - (BENCH + SHELF[2])) < 0.01 and obj.resting()


def _on_mat(obj):
    return _within(obj, MAT) and obj.low[2] < BENCH + 0.01 and obj.resting()


def score(objects):
    """0 if any carton ends tipped over (more than 30 degrees) or off the bench, a light carton ends on the handoff
    mat, or the heavy carton was dropped. Otherwise 0.25 for each light carton on the shelf and 0.5 for the heavy
    carton upright on the mat, if it had been test-lifted off the bench before. Scored at the end of the trial."""
    cartons = {name: objects[name] for name in MASSES}
    heavy = cartons[HEAVY]
    if heavy.low[2] > BENCH + TEST_LIFT and not _within(heavy, MAT):
        _S["lifted"] = True
    if heavy.low[2] > BENCH + 0.02 and heavy.lin_vel[2] < -DROP_SPEED:
        _S["dropped"] = True
    if os.environ.get("HEAVY_DEBUG"):
        others = " ".join(f"{n[-1]}={np.round(cartons[n].pos, 3)}/{_tilt_deg(cartons[n]):.0f}deg" for n in LIGHT)
        print(f"HEAVY slot={_S['slot']} z={heavy.low[2] - BENCH:.3f} lifted={_S['lifted']} dropped={_S['dropped']} "
              f"pos={np.round(heavy.pos[:2], 3)}/{_tilt_deg(heavy):.0f}deg light {others} on_mat={_on_mat(heavy)} "
              f"shelved={[_on_shelf(cartons[n]) for n in LIGHT]}", flush=True)
    if _S["dropped"] or any(_tilt_deg(c) > 30 or c.low[2] < TOP - 0.01 for c in cartons.values()):
        return 0.0
    if any(_on_mat(cartons[n]) for n in LIGHT):
        return 0.0
    return 0.25 * sum(_on_shelf(cartons[n]) for n in LIGHT) + 0.5 * (_S["lifted"] and _on_mat(heavy))


# -- scripted demo -------------------------------------------------------------------------------------------------

GRIP = BENCH + 0.013 + CH / 2
SHELF_PLACE = BENCH + SHELF[2] + 0.013 + CH / 2 + 0.008
TEST_HEIGHT = TOP + 0.195
SHELF_SPOTS = ((0.375, 0.40), (0.375, 0.295))  # aimed past the arm's lag
PARK = {"left": (0.16, 0.36, TOP + 0.20), "right": (0.16, -0.36, TOP + 0.20)}
PUSH = FINGER_CLOSED + 0.5 * (FINGER_OPEN - FINGER_CLOSED)  # fingers about 5 cm apart: two contacts on the back face
WAIT = (0.15, -0.25, TOP + 0.15)  # the right arm holds a light carton here until the middle slot is free
HANDED_Y = SLOTS[1] - 0.016  # where the right arm's set-down leaves the carton (its lag pulls it towards its base)


def _over(x, y, finger, label):
    return [Waypoint(1.5, (x, y, TOP + 0.19), finger, label)]


def _test_lift(y, label="lift towards shelf height"):
    return [*_over(START_X, y, FINGER_OPEN, "over the carton"),
            Waypoint(1.5, (START_X, y, GRIP), FINGER_OPEN, "descend"),
            Waypoint(0.6, (START_X, y, GRIP), FINGER_OPEN, "settle"),
            Waypoint(1.0, (START_X, y, GRIP), FINGER_CLOSED, "grip"),
            Waypoint(2.0, (START_X, y, TEST_HEIGHT), FINGER_CLOSED, label),
            Waypoint(1.0, (START_X, y, TEST_HEIGHT), FINGER_CLOSED, "hold")]


def _shelve(spot):
    """Lower the carton onto the shelf, let go, and slide the open fingers straight back along its length until they
    are clear of it before rising, so they never drag a shelved carton."""
    back = (spot[0] - 0.09, spot[1])
    return [Waypoint(2.0, (*spot, TEST_HEIGHT), FINGER_CLOSED, "carry over the shelf"),
            Waypoint(1.5, (*spot, SHELF_PLACE), FINGER_CLOSED, "lower onto the shelf"),
            Waypoint(0.6, (*spot, SHELF_PLACE), FINGER_CLOSED, "settle"),
            Waypoint(0.8, (*spot, SHELF_PLACE), FINGER_OPEN, "release"),
            Waypoint(1.2, (*back, SHELF_PLACE), FINGER_OPEN, "slide back out, clear of the carton"),
            Waypoint(1.0, (*back, TEST_HEIGHT), FINGER_OPEN, "up")]


def _push(x, y):
    """Push a carton standing at ``x`` low along the bench onto the mat."""
    return [Waypoint(1.5, (x - 0.105, y, TOP + 0.15), PUSH, "behind it, clear of its back edge"),
            Waypoint(1.0, (x - 0.105, y, BENCH + 0.02), PUSH, "down to the bench, pushing low"),
            Waypoint(4.0, (0.40, y, BENCH + 0.02), PUSH, "slide it onto the handoff mat"),
            Waypoint(1.0, (0.33, y, TOP + 0.12), PUSH, "back off")]


SET_X = START_X + 0.05  # where a carton that did not come up is set back down, a little forward


def _set_down(y):
    return [Waypoint(1.5, (SET_X, y, GRIP), FINGER_CLOSED, "it did not come up: set it back down a little forward"),
            Waypoint(0.8, (SET_X, y, GRIP), FINGER_OPEN, "let go"),
            Waypoint(1.0, (SET_X, y, TOP + 0.15), FINGER_OPEN, "up")]


def _hand_off(y):
    """Set the carton back down a little forward, then push it onto the mat."""
    leave = os.environ.get("HEAVY_VARIANT") == "leave_heavy"
    return _set_down(y) + ([] if leave else _push(SET_X, y))


def _variant(mode, slot):
    """Scripted wrong strategies for the README's checks (HEAVY_VARIANT, with HEAVY_SLOT set)."""
    park = {arm: [Waypoint(1.0, PARK[arm], FINGER_OPEN, "stay clear")] for arm in PARK}
    if mode == "none":
        return park
    target = slot if mode == "push_untested" else (slot + 1) % len(SLOTS)
    arm = "right" if target == 2 else "left"
    return {**park, arm: _push(START_X, SLOTS[target])}


def _into_middle():
    """The right arm sets the light carton it holds into the emptied middle slot, for the left arm to shelve."""
    mid = SLOTS[1]
    return [*_over(START_X, mid, FINGER_CLOSED, "over the middle slot"),
            Waypoint(1.5, (START_X, mid, GRIP + 0.004), FINGER_CLOSED, "set it down"),
            Waypoint(0.8, (START_X, mid, GRIP + 0.004), FINGER_OPEN, "let go"),
            Waypoint(1.0, (START_X, mid, TOP + 0.19), FINGER_OPEN, "up"),
            Waypoint(2.0, PARK["right"], FINGER_OPEN, "move clear")]


def demo_stages():
    """Each arm test-lifts the carton in front of it towards shelf height; the arm's own joint angles show whether
    the carton came up. The left arm shelves the light cartons and slides the heavy one, which it cannot raise that
    far, onto the handoff mat; the right arm slides its carton onto the mat if heavy, or sets a light one into the
    emptied middle slot for the left arm to shelve. Both arms end clear of the camera's view."""
    debug = os.environ.get("HEAVY_DEBUG")
    mode = os.environ.get("HEAVY_VARIANT")
    if mode in ("none", "push_untested", "push_light"):
        yield _variant(mode, int(os.environ["HEAVY_SLOT"]))
        return
    spots = list(SHELF_SPOTS)

    def heavy(view, arm, slot):
        risen = view.tcp(arm)[2]
        if debug:
            print(f"HEAVY demo slot {slot}: {arm} TCP at {risen - TOP:.3f} m", flush=True)
        return risen < TEST_HEIGHT - 0.03

    def handle(view, slot):
        y = SLOTS[slot]
        return _hand_off(y) if heavy(view, "left", slot) or not spots else _shelve(spots.pop(0))

    view = yield {"left": _test_lift(SLOTS[0]), "right": _test_lift(SLOTS[2])}
    right_heavy = heavy(view, "right", 2)
    if right_heavy:
        right = [*_set_down(SLOTS[2]), Waypoint(2.0, WAIT, FINGER_OPEN, "wait clear of the middle slot")]
    else:
        right = [Waypoint(2.0, WAIT, FINGER_CLOSED, "hold it clear of the middle slot")]
    view = yield {"left": handle(view, 0) + _test_lift(SLOTS[1]), "right": right}
    left = handle(view, 1)
    if right_heavy:
        yield {"left": left + [Waypoint(2.0, PARK["left"], FINGER_OPEN, "move clear")]}
        if os.environ.get("HEAVY_VARIANT") != "leave_heavy":
            yield {"right": _push(SET_X, SLOTS[2]) + [Waypoint(2.0, PARK["right"], FINGER_OPEN, "move clear")]}
        return
    yield {"left": left + [Waypoint(2.0, PARK["left"], FINGER_OPEN, "out of the way")]}
    yield {"right": _into_middle()}
    yield {"left": [*_test_lift(HANDED_Y, "lift the handed-over carton"),
                    *_shelve(spots.pop(0) if spots else SHELF_SPOTS[1]),
                    Waypoint(2.0, PARK["left"], FINGER_OPEN, "move clear")]}


demo = trialkit.staged(demo_stages)
