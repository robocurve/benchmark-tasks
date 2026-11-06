"""Offline unit tests for ``press_button_through_hole.py``: every row of the scoring table, each
condition at both sides of its threshold, and the geometry helpers. No GPU and no Isaac Sim.

Run:  python test_press_button_through_hole.py     (or: pytest test_press_button_through_hole.py)
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from yam_arena.task_file import ObjectState  # noqa: E402

_spec = importlib.util.spec_from_file_location("pbth_task", HERE / "press_button_through_hole.py")
task = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(task)
cage = task.cage
TOP = task.TOP


# -- builders ---------------------------------------------------------------------------


def _quat_y(angle: float) -> np.ndarray:
    """Rotation about world y as XYZW: tilts a bar lying along x out of horizontal."""
    return np.array([0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2)])


def state(pos, size, *, quat=(0.0, 0.0, 0.0, 1.0), still=True) -> ObjectState:
    pos = np.asarray(pos, dtype=float)
    half = np.asarray(size, dtype=float) / 2
    vel = np.zeros(3) if still else np.array([0.5, 0.0, 0.0])
    return ObjectState(
        pos=pos,
        quat_xyzw=np.asarray(quat, dtype=float),
        lin_vel=vel,
        ang_vel=np.zeros(3) if still else np.array([2.0, 0.0, 0.0]),
        low=pos - half,
        high=pos + half,
    )


def button(travel: float, *, still=True) -> ObjectState:
    return state(
        (cage.BUTTON_CX + travel, cage.CAGE_YC, cage.port_axis_z(TOP)),
        (cage.BUTTON_LEN, cage.BUTTON_D, cage.BUTTON_D),
        still=still,
    )


def rod(centre_x, *, y=cage.CAGE_YC, z=None, tilt_deg=0.0, still=True) -> ObjectState:
    z = cage.port_axis_z(TOP) if z is None else z
    return state(
        (centre_x, y, z),
        (cage.ROD_LEN, cage.ROD_SIDE, cage.ROD_SIDE),
        quat=_quat_y(math.radians(tilt_deg)),
        still=still,
    )


def rod_at_rest() -> ObjectState:
    return state(
        (cage.ROD_CX, cage.ROD_Y, cage.bar_rest_z(TOP, cage.ROD_SIDE)),
        (cage.ROD_LEN, cage.ROD_SIDE, cage.ROD_SIDE),
    )


def rule_at_rest() -> ObjectState:
    return state(
        (cage.RULE_CX, cage.RULE_Y, cage.bar_rest_z(TOP, cage.RULE_T)),
        (cage.RULE_LEN, cage.RULE_W, cage.RULE_T),
    )


def baton_at_rest() -> ObjectState:
    return state(
        (cage.BATON_CX, cage.BATON_Y, cage.bar_rest_z(TOP, cage.BATON_SIDE)),
        (cage.BATON_LEN, cage.BATON_SIDE, cage.BATON_SIDE),
    )


def scene(*, press=0.0, rod_state=None, rule=None, baton=None, plunger_still=True) -> dict:
    return {
        "button": button(press, still=plunger_still),
        "rod": rod_state if rod_state is not None else rod_at_rest(),
        "short_rule": rule if rule is not None else rule_at_rest(),
        "thick_baton": baton if baton is not None else baton_at_rest(),
    }


# Rod centre that puts its far end 12 mm past the button face, as the demo commands.
PRESSING_ROD_X = cage.BUTTON_FACE_X + 0.012 - cage.ROD_LEN / 2
# A rod lying through the port with its far end just past the 50 mm credit line.
REACHED_ROD_X = cage.CAGE_X0 + task.REACH_M + 0.005 - cage.ROD_LEN / 2


# -- tests ------------------------------------------------------------------------------

CASES: list[tuple[str, callable]] = []


def case(name):
    def wrap(fn):
        CASES.append((name, fn))
        return fn

    return wrap


@case("start of trial scores 0")
def _():
    s = scene()
    assert task.score(s) == 0.0, task.score(s)
    assert task.success(s) is False


@case("taking the rod off its rack scores 0.25")
def _():
    lifted = state(
        (cage.ROD_CX, cage.ROD_Y, cage.bar_rest_z(TOP, cage.ROD_SIDE) + 0.030),
        (cage.ROD_LEN, cage.ROD_SIDE, cage.ROD_SIDE),
    )
    assert task.score(scene(rod_state=lifted)) == 0.25


@case("nudging the rod 5 mm is not yet taking it off the rack")
def _():
    low = state(
        (cage.ROD_CX, cage.ROD_Y, cage.bar_rest_z(TOP, cage.ROD_SIDE) + 0.005),
        (cage.ROD_LEN, cage.ROD_SIDE, cage.ROD_SIDE),
    )
    assert task.score(scene(rod_state=low)) == 0.0


@case("rod 55 mm into the enclosure scores 0.5")
def _():
    s = scene(rod_state=rod(REACHED_ROD_X))
    assert task.score(s) == 0.5, task.score(s)
    assert task.success(s) is False


@case("rod only 40 mm in does not reach 0.5 (still 0.25 for the lift)")
def _():
    shallow = rod(cage.CAGE_X0 + 0.040 - cage.ROD_LEN / 2)
    assert task.score(scene(rod_state=shallow)) == 0.25


@case("a distractor through the port scores 0, not 0.5: it is the wrong tool")
def _():
    # The rule is thin enough to pass the port, so the 0.5 tier has to name the rod specifically,
    # or picking the wrong tool would earn half marks.
    deep_rule = state(
        (cage.CAGE_X0 + task.REACH_M + 0.005 - cage.RULE_LEN / 2, cage.CAGE_YC, cage.port_axis_z(TOP)),
        (cage.RULE_LEN, cage.RULE_W, cage.RULE_T),
    )
    assert task.score(scene(rule=deep_rule)) == 0.0


@case("pressed by the rod through the port succeeds; score() itself stops at 0.5")
def _():
    s = scene(press=0.008, rod_state=rod(PRESSING_ROD_X))
    assert task.success(s) is True
    # 1.0 comes only from success() holding for a second, never from score() latching.
    assert task.score(s) == 0.5, task.score(s)


@case("4 mm of travel is not pressed")
def _():
    s = scene(press=0.004, rod_state=rod(PRESSING_ROD_X))
    assert task.success(s) is False
    assert task.score(s) == 0.5, task.score(s)


@case("exactly 5 mm of travel is pressed")
def _():
    s = scene(press=cage.PRESSED_M, rod_state=rod(PRESSING_ROD_X))
    assert task.success(s) is True
    assert task.score(s) == 0.5


@case("a pressed button with the rod back on its rack does not succeed")
def _():
    s = scene(press=0.008, rod_state=rod_at_rest())
    assert task.success(s) is False
    assert task.score(s) == 0.0


@case("a rod tilted 20 degrees does not succeed")
def _():
    s = scene(press=0.008, rod_state=rod(PRESSING_ROD_X, tilt_deg=20.0))
    assert task.success(s) is False


@case("a rod tilted 8 degrees still succeeds")
def _():
    s = scene(press=0.008, rod_state=rod(PRESSING_ROD_X, tilt_deg=8.0))
    assert task.success(s) is True


@case("score() never returns 1.0, so a momentary press cannot latch full marks")
def _():
    for press in (0.0, cage.PRESSED_M, 0.012):
        for rx in (cage.ROD_CX, REACHED_ROD_X, PRESSING_ROD_X):
            assert task.score(scene(press=press, rod_state=rod(rx))) <= 0.5


@case("a rod pressed from inside with no tail out of the port does not succeed")
def _():
    # Physically impossible (the rod is longer than the guard is deep), but the guard must hold.
    short_inside = state(
        (cage.BUTTON_FACE_X - 0.020, cage.CAGE_YC, cage.port_axis_z(TOP)),
        (0.030, cage.ROD_SIDE, cage.ROD_SIDE),
    )
    s = scene(press=0.008, rod_state=short_inside)
    assert task.success(s) is False


@case("a rod beside the enclosure, not through the port, scores only the lift")
def _():
    beside = rod(0.360 - cage.ROD_LEN / 2, y=cage.CAGE_Y1 + 0.060, z=cage.port_axis_z(TOP))
    assert task.score(scene(rod_state=beside)) == 0.25


@case("score never leaves 0..1")
def _():
    for press in (-0.01, 0.0, 0.004, 0.02):
        for rx in (cage.ROD_CX, REACHED_ROD_X, PRESSING_ROD_X):
            v = task.score(scene(press=press, rod_state=rod(rx)))
            assert v in (0.0, 0.25, 0.5), (press, rx, v)


@case("distance falls to ~0 as the rod reaches the button")
def _():
    far = task.distance(scene())
    near = task.distance(scene(press=0.008, rod_state=rod(PRESSING_ROD_X)))
    assert far > near, (far, near)
    assert near < 0.02, near


@case("geometry: the port is smaller than the baton and bigger than the rod")
def _():
    assert cage.ROD_SIDE < cage.PORT < cage.BATON_SIDE
    assert cage.BUTTON_DEPTH >= 0.080


@case("geometry: the rod cannot fit entirely inside the enclosure")
def _():
    assert cage.ROD_LEN > (cage.IN_X1 - cage.IN_X0) + 0.100


@case("geometry: the button's travel is bigger than the press threshold")
def _():
    assert cage.SPRING_TRAVEL > cage.PRESSED_M


@case("geometry: the button has a bore to recess into")
def _():
    assert cage.BORE > cage.BUTTON_D, (cage.BORE, cage.BUTTON_D)
    assert cage.SPRING_TRAVEL < cage.BUTTON_LEN, "the button would vanish into the panel"


@case("geometry: the guard is closed on every side but the port")
def _():
    names = {o.name for o in task.OBJECTS}
    for side in ("guard_side_left", "guard_side_right", "guard_roof", "control_panel_low"):
        assert side in names, side
    # The four screen pieces must tile the front exactly around the port.
    assert cage.PORT_Z0_REL > 0 and cage.PORT_Z1_REL < cage.IN_TOP_REL
    assert cage.IN_Y0 < cage.PORT_Y0 < cage.PORT_Y1 < cage.IN_Y1


@case("every object has a unique, valid name and the fixed ones have no mass")
def _():
    names = [o.name for o in task.OBJECTS]
    assert len(names) == len(set(names)), "duplicate object names"
    assert all(n.isidentifier() for n in names)
    free = {o.name for o in task.OBJECTS if o.free}
    assert free == {"button", "rod", "short_rule", "thick_baton"}, free
    for o in task.OBJECTS:
        o._check()


@case("only the button is spring-returned")
def _():
    sprung = {o.name for o in task.OBJECTS if getattr(o, "spring", None) is not None}
    assert sprung == {"button"}, sprung


@case("the rack puts the rod on the port's axis")
def _():
    assert abs(cage.bar_rest_z(TOP, cage.ROD_SIDE) - (cage.port_axis_z(TOP))) < 1e-9


@case("rack posts sit clear of the grip and of the press")
def _():
    for x in cage.ROD_POSTS:
        assert abs(x - cage.ROD_CX) > 0.045, x
    assert max(cage.ROD_POSTS) > task.PRESS_X + task.OVERSHOOT or cage.ROD_Y != cage.CAGE_YC


def main() -> int:
    failed = 0
    for name, fn in CASES:
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            print(f"  FAIL  {name}: {exc}")
        except Exception as exc:  # noqa: BLE001
            failed += 1
            print(f"  ERROR {name}: {type(exc).__name__}: {exc}")
        else:
            print(f"  PASS  {name}")
    print(f"\n{len(CASES) - failed}/{len(CASES)} tests passed")
    return 1 if failed else 0


# Let pytest collect the same cases.
def test_all():
    for _name, fn in CASES:
        fn()


if __name__ == "__main__":
    raise SystemExit(main())
