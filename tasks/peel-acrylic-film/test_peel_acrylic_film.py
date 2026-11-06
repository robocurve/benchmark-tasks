"""Offline checks (no GPU): the task file loads, the glue, hinges and tab are specified as the trap needs, the scorer
covers each outcome and remembers peeled strips and a torn tab, and the demo holds the sheet before it peels, keeps
the arms apart and stays within reach. Run in the yam_arena environment:  python -m pytest -q test_peel_acrylic_film.py
"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "peel_acrylic_film.py")
T = TASK.module
import bonds  # noqa: E402  (the task file put its folder on the path)

FLAT = (0.0, 0.0, 0.0, 1.0)


def _state(pos, quat=FLAT, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.01, high=pos + 0.01)


def _start():
    out = {o.name: _state(o.pos) for o in [T.SHEET, *T.STRIP, T.TAB]}
    return out


def _scene(**changes):
    out = _start()
    for name, value in changes.items():
        out[name] = value if isinstance(value, ObjectState) else _state(value)
    return out


def _peeled(*idx, film_at=None):
    """Strips ``idx`` lifted 3 cm off the sheet (or every piece at ``film_at``), the tab with strip 1."""
    out = _start()
    for i in idx:
        p = np.asarray(T.STRIP[i - 1].pos) + (0.0, 0.0, 0.03) if film_at is None else np.asarray(film_at)
        out[f"strip_{i}"] = _state(p)
        if i == 1:
            out["tab"] = _state(np.asarray(T.TAB.pos) - T.STRIP[0].pos + p)
    return out


IN_BASKET = (*T.BASKET_XY, T.TOP + 0.01)


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"sheet", "strip_1", "strip_2", "strip_3", "strip_4", "tab"}


def _joints():
    return [j for js in bonds._JOINTS.values() for j in js]


def test_glue_hinges_and_tab_are_specified():
    joints = _joints()
    glue = [j for j in joints if j.name.startswith("glue")]
    assert sorted(j.to for j in glue) == [f"strip_{i}" for i in range(1, 5)]
    for j in glue:
        i = int(j.to[-1])
        # authored under the sheet, so the strip is the joint's first body and PhysX measures the twist on the strip
        assert j.kind == "fixed" and j.body == "sheet" and j.twist == T.GLUE_TWIST and j.hold == T.GLUE_HOLD
        assert j.at[0] == pytest.approx(T.strip_near_edge(i))  # at the edge away from the pull
        assert j.at[2] == pytest.approx(T.SHEET_TOP)
    hinges = [j for j in joints if j.name == "hinge"]
    assert sorted((j.body, j.to) for j in hinges) == [(f"strip_{i}", f"strip_{i + 1}") for i in range(1, 4)]
    assert all(j.kind == "ball" and math.isinf(j.hold) and not j.collide for j in hinges)
    (fold,) = [j for j in joints if j.name == "fold"]
    assert fold.body == "tab" and fold.to == "strip_1" and fold.kind == "ball" and fold.hold == T.TAB_HOLD
    # Pulled along the sheet, the tab tears before the glue gives. How hard a peel pulls is measured in the simulator
    # (README): the loose sheet tips up before the first strip lets go, and a pressed sheet stays flat.
    assert T.TAB_HOLD < T.GLUE_HOLD
    assert T.GLUE_TWIST > 0


def test_layout():
    assert T.STRIP[0].pos[0] + T.STRIP_W / 2 == pytest.approx(T.SHEET_FAR)
    assert T.STRIP[-1].pos[0] - T.STRIP_W / 2 == pytest.approx(T.SHEET_NEAR)
    for s in T.STRIP:
        assert s.pos[2] - T.STRIP_T / 2 == pytest.approx(T.SHEET_TOP)
        assert s.pos[1] - T.FILM_LEN / 2 == pytest.approx(T.SHEET_XY[1] - T.SHEET_SIZE[1] / 2)
    assert T.TAB.pos[2] - T.TAB_SIZE[2] / 2 == pytest.approx(T.STRIP_TOP)
    assert T.TAB.pos[0] + T.TAB_SIZE[0] / 2 == pytest.approx(T.SHEET_FAR)
    # the pressing hand is on the bare acrylic, 3 cm or more from the film
    assert T.PRESS_XY[1] - (T.FILM_Y + T.FILM_LEN / 2) >= 0.03
    # the basket stands off the mat, on the robot's right
    assert T.BASKET_XY[1] + T.BASKET_OUTER[1] / 2 < T.MAT_XY[1] - T.MAT_SIZE[1] / 2
    # hanging from the hand, the film's lowest edge clears the basket's rim by less than its last strip: dragged over
    # the rim, that strip folds over it
    hinge_z = T.CARRY_Z - T.PAD_TIP_PAST_TCP - (T.GRIP_TIP_Z - T.STRIP_TOP)
    assert hinge_z - T.STRIPS * T.STRIP_W > T.BASKET_RIM - T.STRIP_W


def test_start_scores_zero():
    assert T.score(_start()) == 0.0
    assert not T.success(_start())


def test_one_strip_zero():
    assert T.score(_peeled(1)) == 0.0
    assert T.STATE["peeled"] == {1}


def test_two_strips_quarter():
    assert T.score(_peeled(1, 2)) == 0.25
    assert T.score(_peeled(1, 2, 3)) == 0.25


def test_all_strips_half():
    assert T.score(_peeled(1, 2, 3, 4)) == 0.5


def test_peeled_strips_are_remembered():
    T.score(_peeled(1, 2, 3, 4))
    assert T.score(_start()) == 0.5  # laid back on the sheet, still unstuck


def test_small_shift_is_not_peeled():
    moved = _scene(strip_1=np.asarray(T.STRIP[0].pos) + (0.0, 0.0, 0.006))
    T.score(moved)
    assert T.STATE["peeled"] == set()


def test_film_in_basket_succeeds():
    objects = _peeled(1, 2, 3, 4, film_at=IN_BASKET)
    assert T.score(objects) == 1.0 and T.success(objects)


def test_film_in_basket_but_moving_is_not_success():
    objects = _peeled(1, 2, 3, 4, film_at=IN_BASKET)
    objects["tab"] = _state(objects["tab"].pos, vel=(0.1, 0.0, 0.0))
    assert T.score(objects) == 1.0 and not T.success(objects)


def test_one_piece_outside_the_basket_half():
    objects = _peeled(1, 2, 3, 4, film_at=IN_BASKET)
    objects["strip_4"] = _state((T.BASKET_XY[0], T.BASKET_XY[1] + 0.09, T.TOP + 0.02))  # hanging over the rim
    assert T.score(objects) == 0.5


def test_film_held_above_the_basket_half():
    objects = _peeled(1, 2, 3, 4, film_at=(*T.BASKET_XY, T.BASKET_RIM + 0.03))
    assert T.score(objects) == 0.5


def test_torn_tab_zero_for_the_rest_of_the_trial():
    torn = _scene(tab=np.asarray(T.TAB.pos) + (-0.03, 0.0, 0.02))
    assert T.score(torn) == 0.0 and T.STATE["torn"]
    objects = _peeled(1, 2, 3, 4, film_at=IN_BASKET)
    assert T.score(objects) == 0.0 and not T.success(objects)


def test_tab_turned_on_its_fold_is_not_torn():
    """Turned flat on its fold (a quarter turn about y), the tab's centre stays 1 cm from the fold."""
    fold = np.array([T.TAB_HINGE_X, T.FILM_Y, T.STRIP_TOP])
    flat = _scene(tab=fold + (-0.01, 0.0, 0.0))
    T.score(flat)
    assert not T.STATE["torn"]


def test_sheet_dragged_zero():
    moved = _peeled(1, 2, 3, 4, film_at=IN_BASKET)
    moved["sheet"] = _state(np.asarray(T.SHEET.pos) + (0.015, 0.0, 0.0))
    assert T.score(moved) == 1.0
    moved["sheet"] = _state(np.asarray(T.SHEET.pos) + (0.025, 0.0, 0.0))
    assert T.score(moved) == 0.0
    moved["sheet"] = _state(T.SHEET.pos)  # put back: still 0, it was dragged
    assert T.score(moved) == 0.0 and T.STATE["lifted"]


def test_sheet_lifted_with_the_film_zero():
    """The film peeled from a sheet tilted up 20 degrees about its near edge: the hand that should hold it didn't."""
    a = math.radians(20)
    q = (0.0, -math.sin(a / 2), 0.0, math.cos(a / 2))
    sheet = np.asarray(T.SHEET.pos) + (0.0, 0.0, 0.025)
    objects = _scene(sheet=_state(sheet, quat=q))
    assert T.score(objects) == 0.0
    done = _peeled(1, 2, 3, 4, film_at=IN_BASKET)  # the film then peeled and binned, the sheet fallen back flat
    assert T.score(done) == 0.0 and not T.success(done)


def test_strips_measured_on_the_sheet():
    """A sheet moved 1.5 cm with its film still stuck: no strip counts as peeled, and the sheet still counts as in
    place."""
    shift = np.array([0.015, 0.0, 0.0])
    objects = {o.name: _state(np.asarray(o.pos) + shift) for o in [T.SHEET, *T.STRIP, T.TAB]}
    assert T.score(objects) == 0.0 and T.STATE["peeled"] == set()


def test_sheet_off_table_zero():
    assert T.score(_scene(sheet=(0.3, 0.0, T.TOP - 0.5))) == 0.0


def test_non_finite_zero():
    assert T.score(_scene(strip_2=(math.nan, 0.0, 0.0))) == 0.0


# -- the demo --------------------------------------------------------------------------------------------------------


def _at(steps, label):
    t = 0.0
    for w in steps:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_holds_before_it_peels_and_until_the_film_is_off():
    plan = T.demo()
    pressed, press = _at(plan["left"], "press")
    held, _ = _at(plan["left"], "hold the sheet")
    pinched, _ = _at(plan["right"], "pinch")
    peeled, _ = _at(plan["right"], "peel up and back")
    assert pressed < pinched and peeled < held
    assert press.finger == T.FINGER_CLOSED
    tips = press.pos[2] - T.PAD_TIP_PAST_TCP
    assert T.SHEET_TOP - 0.04 < tips < T.SHEET_TOP - 0.005  # the arm leans on the sheet


def test_demo_pinches_the_tab_above_the_film():
    _, w = _at(T.demo()["right"], "pinch")
    tips = w.pos[2] - T.PAD_TIP_PAST_TCP
    assert T.STRIP_TOP + 0.003 < tips < T.STRIP_TOP + T.TAB_SIZE[2] - 0.01
    assert w.pos[:2] == pytest.approx(T.TAB_XY)


def test_demo_peels_in_an_arc_back_over_the_sheet():
    """The peel stroke goes up at least as far as it comes back: the film between the hand and the peel line leans
    up steeply, so the tab never takes the 4 N that pulling along the sheet would."""
    plan = T.demo()["right"]
    _, start = _at(plan, "hold")
    _, end = _at(plan, "peel up and back")
    back = start.pos[0] - end.pos[0]
    up = end.pos[2] - start.pos[2]
    assert back > 0.1 and up > back


def test_demo_keeps_the_arms_apart():
    plans = {a: ArmPlan(T.demo()[a], BASES[a]) for a in ("left", "right")}
    duration = max(p.duration for p in plans.values())
    closest = min(np.linalg.norm(plans["left"].target(k / 20)[0] - plans["right"].target(k / 20)[0])
                  for k in range(int(duration * 20) + 1))
    assert closest > 0.14
    assert min(w.pos[1] for w in T.demo()["left"]) > -0.01
    assert max(w.pos[1] for w in T.demo()["right"]) < -0.07


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


# -- the meshes --------------------------------------------------------------------------------------------------------


def test_every_dressed_box_exists_and_its_mesh_is_there():
    names = {o.name for o in T.OBJECTS}
    for name, file in T.LOOKS.items():
        assert name in names
        assert (HERE / "looks" / file).is_file()


@pytest.mark.parametrize("name", ["sheet", "strip_1", "strip_2", "strip_3", "strip_4", "tab"])
def test_scored_meshes_fit_in_their_boxes(name):
    """Each scored piece's mesh stays inside its collider and fills it, so what the cameras see is what collides and
    the scorer's centres are the boxes' centres."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray({o.name: o.size for o in T.OBJECTS}[name]) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)
    assert np.all(np.asarray(box.GetMax()) - np.asarray(box.GetMin()) >= 2 * half - 0.001)
