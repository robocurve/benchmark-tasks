"""calc_cover without the simulator: the start (keys down in the cover, the open display end past
the notebook's edge), the cover's fit on the calculator in both positions (no shape of one inside the other, the play
the rails need, the bare strip above the walls, no way on from above), the rails' rules step by step and the score's
levels from states set directly."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

TASK = Path(__file__).resolve().parent / "task.py"  # the task file in this test's own folder (it imports the folder's mechkit)


@pytest.fixture(autouse=True)
def clean_registry():
    """Each test loads the task afresh with the folder's own mechkit, and leaves no registry or module behind."""
    before = sys.modules.pop("mechkit", None)
    yield
    kit = sys.modules.pop("mechkit", None)
    if kit is not None:
        kit.MECH.clear()
    if before is not None:
        sys.modules["mechkit"] = before


def load():
    task = task_file.load(TASK)
    return task, task.module, task.module.mk


def boxes_in(kit, name: str, pose) -> list[tuple[np.ndarray, np.ndarray]]:
    """(low, high) of every solid shape of body ``name`` at ``pose`` (position, rotation), in the frame the pose is
    given in; every shape here is a box turned by quarter turns, so the bounds are exact."""
    position, rotation = pose
    out = []
    for s in kit.MECH.bodies[name].shapes:
        if not s.solid:
            continue
        signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
        local = np.array(s.pos) + (kit.quat_to_matrix(s.quat) @ (signs * s.half_extents()).T).T
        world = local @ rotation.T + position
        out.append((world.min(axis=0), world.max(axis=0)))
    return out


def gap(a, b) -> float:
    """Clearance between two boxes (m): negative if they overlap (then the depth of the shallowest overlap)."""
    (alo, ahi), (blo, bhi) = a, b
    sep = np.maximum(blo - ahi, alo - bhi)
    return float(sep.max())


def least_gap(kit, m, which: str) -> float:
    """The least clearance between the calculator's and the cover's solid shapes, the cover home on ``which`` side,
    in the calculator's frame."""
    calc = boxes_in(kit, "calc", (np.zeros(3), np.eye(3)))
    cover = boxes_in(kit, "cover", (np.zeros(3), np.eye(3) if which == "front" else m.FLIP))
    return min(gap(a, b) for a in calc for b in cover)


def test_the_start_is_keys_down_in_the_cover_with_the_open_end_past_the_notebook():
    task, m, kit = load()
    assert set(task.names) == {"notebook", "pencil", "calc", "cover", "carrier"}
    assert task.free_objects == ("calc", "cover", "carrier")
    (cp, cr), (vp, vr) = kit.built_pose("calc"), kit.built_pose("cover")
    assert np.allclose(cp, vp) and np.allclose(cr, vr)  # home on the front
    assert cr[:, 2] @ [0, 0, 1] == pytest.approx(-1.0)  # keys down
    assert cr[:, 1] @ [0, 1, 0] == pytest.approx(-1.0)  # the display end (its +y) to the world's -y
    cover = boxes_in(kit, "cover", (vp, vr))
    low = np.min([b[0] for b in cover], axis=0)
    high = np.max([b[1] for b in cover], axis=0)
    assert low[2] - m.BOOK_TOP == pytest.approx(0.001)  # the plate 1 mm over the notebook
    assert m.BOOK_RIGHT - low[1] == pytest.approx(m.OVERHANG)  # the open end 60 mm past the notebook's right edge
    calc = boxes_in(kit, "calc", (cp, cr))
    assert min(b[0][1] for b in calc) == pytest.approx(low[1])  # the open end flush with the display end
    end_wall = cover[-1]
    assert end_wall[0][1] > m.BOOK_RIGHT and end_wall[1][1] < m.BOOK_XY[1] + m.BOOK[1] / 2  # the end wall over the notebook
    rails = kit.MECH.joints["rails"]
    assert (rails.kind, rails.child, rails.parent, rails.lower, rails.upper) == ("slide", "carrier", "calc", 0.0, pytest.approx(m.RAILS + m.REACH))
    assert rails.axis == pytest.approx((0.0, 1.0, 0.0))  # the cover leaves towards the ON-key end, the world's +y at the start
    assert [(kit.MECH.joints[w].kind, kit.MECH.joints[w].made) for w in ("on_front", "on_again")] == [("weld", True), ("weld", False)]
    top = max(b[1][2] for b in calc)  # the calculator's back, up
    assert m.COVER_W == pytest.approx(0.0844) and top - low[2] == pytest.approx(0.022)  # the pair stands 22 mm
    assert high[2] - low[2] == pytest.approx(0.0131)  # the cover alone: 13.1 mm


@pytest.mark.parametrize("which", ["front", "back"])
def test_home_on_either_side_no_shape_of_the_cover_is_inside_the_calculator_and_the_play_is_as_stated(which):
    task, m, kit = load()
    assert least_gap(kit, m, which) == pytest.approx(m.END_GAP)  # the nearest: the end wall, 0.3 mm off the calculator's end
    flip = np.eye(3) if which == "front" else m.FLIP
    calc = boxes_in(kit, "calc", (np.zeros(3), np.eye(3)))
    shapes = boxes_in(kit, "cover", (np.zeros(3), flip))
    lips = [b for b in shapes if b[1][2] - b[0][2] == pytest.approx(m.LIP_T)]
    assert len(lips) == 2
    for lo, hi in lips:  # each lip in its groove: 0.6 mm over and under it, 0.6 mm short of the groove's floor
        assert -lo[2] - m.GROOVE_H / 2 == pytest.approx(-0.0006) and hi[2] - m.GROOVE_H / 2 == pytest.approx(-0.0006)
        tip = min(abs(lo[0]), abs(hi[0]))
        assert tip - (m.CALC[0] / 2 - m.GROOVE_D) == pytest.approx(0.0006)
    walls = [b for b in shapes if b[1][0] - b[0][0] == pytest.approx(m.WALL_T) and b[1][1] - b[0][1] > 0.1]
    assert sorted(round(min(abs(lo[0]), abs(hi[0])) - m.CALC[0] / 2, 6) for lo, hi in walls) == [0.0005, 0.0005]
    assert calc  # (the calculator's own boxes were found)


def test_above_the_walls_the_calculators_sides_are_bare_for_8_9_mm_and_no_cover_goes_on_from_above():
    task, m, kit = load()
    strip = m.CALC[2] / 2 - (m.LIP_T / 2)  # from the walls' top (the lips' far face) to the calculator's back
    assert strip == pytest.approx(0.0089)
    cover = boxes_in(kit, "cover", (np.zeros(3), np.eye(3)))
    reach = max(b[1][2] for b in cover if b[1][0] - b[0][0] == pytest.approx(m.WALL_T))  # the walls' far end, front position
    assert reach == pytest.approx(m.PLATE_LOW) and -min(b[0][2] for b in cover) == pytest.approx(m.LIP_T / 2)
    tip = m.WALL_IN - m.LIP_IN  # the lips reach inside the calculator's outline: lowered from above they land on it
    assert tip < m.CALC[0] / 2 - 0.0005


def test_the_rails_resistance_holds_within_its_stretch_and_follows_beyond_it():
    task, m, kit = load()
    assert m.cap(0.0) == m.cap(m.DETENT_FROM) == m.DETENT_FORCE and m.cap(m.DETENT_FROM + 1e-6) == m.RAIL_FORCE
    assert m.follow(0.05, 0.0005) == 0.0  # within CLICK_IN: drawn home
    lag = m.RAIL_FORCE / m.STIFFNESS
    assert m.follow(0.05, 0.05 + lag) == 0.05 and m.follow(0.05, 0.05 - lag) == 0.05
    assert m.follow(0.05, 0.06) == pytest.approx(0.06 - lag) and m.follow(0.05, 0.04) == pytest.approx(0.04 + lag)
    near = m.DETENT_FORCE / m.STIFFNESS  # in the click's bump the stretch is the click's
    assert m.follow(0.002, 0.002 + near) == 0.002 and m.follow(0.0025, 0.002) == 0.0025


def calc_turned(m, kit, keys_up: bool) -> tuple[np.ndarray, np.ndarray]:
    """The calculator where it starts, keys up or down (both with the display end to -y)."""
    position, rotation = kit.built_pose("calc")
    return position, (rotation if not keys_up else rotation @ m.FLIP)


def cover_at(m, calc_pose, which: str, travel: float, across=0.0, up=0.0, tilt=0.0):
    """The cover's pose (world) ``travel`` off home on the ``which`` side of the calculator at ``calc_pose``, turned
    ``tilt`` about the calculator's z through its leading end's middle, then moved so that this end lies ``across`` and
    ``up`` off the cover's rail line (in the cover's frame): the three as the rule reads them."""
    position, rotation = calc_pose
    lead = m.LEAD
    turn = m.mk.axis_angle(np.array([0.0, 0.0, 1.0]), tilt) @ (np.eye(3) if which == "front" else m.FLIP)  # in the calculator's frame
    on_line = np.array([0.0, -travel, 0.0])
    p = lead + m.mk.axis_angle(np.array([0.0, 0.0, 1.0]), tilt) @ (on_line - lead) - turn @ np.array([across, 0.0, up])
    return position + rotation @ p, rotation @ turn


def test_an_offer_tells_front_from_back_and_holds_the_play_and_tilt():
    task, m, kit = load()
    calc = calc_turned(m, kit, keys_up=True)
    for which in ("front", "back"):
        kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, which, 0.100)})
        o = m.offer()
        assert o["which"] == which and o["travel"] == pytest.approx(0.100) and o["in_line"]
    for kw, ok in (({"across": 0.00045}, True), ({"across": -0.00055}, False), ({"up": 0.00055}, True), ({"up": -0.00065}, False),
                   ({"tilt": math.radians(0.8)}, True), ({"tilt": math.radians(1.2)}, False)):  # fmt: skip
        kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, "back", 0.100, **kw)})
        assert m.offer()["in_line"] is ok, kw


def test_the_rails_correction_is_bounded_by_the_real_play():
    """Setting the calculator on the cover's rail line corrects the simulator's state, so it may
    happen only within the rails' real play: the largest correction is the walls' play across, the lips' play up or down and
    one degree; an offer beyond any of these is not taken (it would jam on the bench)."""
    task, m, kit = load()
    assert m.LATERAL == pytest.approx(m.WALL_GAP) == pytest.approx(0.0005)
    assert m.ACROSS == pytest.approx((m.GROOVE_H - m.LIP_T) / 2) == pytest.approx(0.0006)
    assert m.LINE_TILT == pytest.approx(math.radians(1.0))
    calc = calc_turned(m, kit, keys_up=True)
    kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, "back", 0.120, across=0.00049, up=-0.00059, tilt=math.radians(0.99))})
    o = m.offer()  # the largest correction taken: just inside the play every way at once
    assert o["in_line"] and abs(o["across"]) <= m.LATERAL and abs(o["up"]) <= m.ACROSS and o["tilt"] <= m.LINE_TILT
    for kw in ({"across": 0.00051}, {"up": 0.00061}, {"tilt": math.radians(1.01)}):
        kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, "back", 0.120, **kw)})
        assert not m.offer()["in_line"], kw
    kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, "back", m.RAILS - m.RAILS_IN + 0.0005)})
    assert not m.offer()["in_line"]  # not yet 6 mm onto the rails


def test_the_play_is_read_at_the_calculators_leading_end():
    """The walls and the lips act on an entering calculator at its leading (ON-key) end: the offsets are read there.
    Turned 0.9 degree about that end, 8 mm into the rails, it is taken though the cover's origin, 70 mm further in,
    lies 1.1 mm off its line; with the cover's origin on the line but the leading end 0.6 mm off, it is not."""
    task, m, kit = load()
    calc = calc_turned(m, kit, keys_up=True)
    travel = m.RAILS - 0.008
    kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, "back", travel, tilt=math.radians(0.9))})
    o = m.offer()
    assert o["in_line"] and abs(o["across"]) < 1e-9 and o["tilt"] == pytest.approx(math.radians(0.9))
    cp, cr = calc
    vp, vr = kit.MECH.poses["cover"]
    origin_off = (vr.T @ (cp + cr @ np.array([0.0, -travel, 0.0]) - vp))  # the point of the calculator's axis at the cover's origin, in the cover's frame
    assert abs(origin_off[0]) > 0.001  # read at the cover's origin, the same entry would be more than 1 mm across
    position, rotation = calc
    turn = m.mk.axis_angle(np.array([0.0, 0.0, 1.0]), math.asin(0.0006 / (travel - m.CALC[1] / 2)))  # about the cover's own origin
    kit.set_state(poses={"calc": calc, "cover": (position + rotation @ np.array([0.0, -travel, 0.0]), rotation @ turn @ m.FLIP)})
    o = m.offer()  # the cover's origin on the calculator's axis (the old reading: 0 mm), the leading end 0.6 mm off its line
    assert not o["in_line"] and abs(o["across"]) == pytest.approx(0.0006, abs=1e-9) and o["tilt"] < m.LINE_TILT


def test_held_in_line_beyond_the_closed_end_it_is_not_taken():
    """The take has a lower bound on travel: a calculator held in line on the rails' axis beyond the cover's CLOSED end
    (out in the air, 25 cm away, or 2 mm past the end wall) is not taken for 10 steps (it would be set home through the
    end wall); the same held 8 mm into the open end is taken."""
    task, m, kit = load()
    m.start(0)
    kit.set_state(joints={"rails": m.RAILS + m.RELEASE_PAST}, time=1.0)
    m.step()
    assert m.STATE["on"] is None  # the cover off the front
    calc = calc_turned(m, kit, keys_up=True)
    for k, travel in enumerate([-0.25] * 10 + [-0.002] * 10):
        kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, "back", travel)}, time=1.05 + 0.05 * k)
        assert m.offer()["travel"] == pytest.approx(travel) and not m.offer()["in_line"]
        m.step()
        assert m.STATE["on"] is None and m.STATE["corrections"] == []
    for k in range(2):
        kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, "back", m.RAILS - 0.008)}, time=2.1 + 0.05 * k)
        m.step()
    assert m.STATE["on"] == "back"


def test_off_the_front_past_the_rails_end_then_taken_onto_the_back_in_line_and_home():
    task, m, kit = load()
    m.start(0)
    assert m.STATE["on"] == "front" and not kit.shown("carrier", 0)
    kit.set_state(joints={"rails": 0.050}, time=1.0)
    m.step()
    assert m.STATE["on"] == "front" and m.STATE["rest"] == pytest.approx(0.050 - m.RAIL_FORCE / m.STIFFNESS)
    kit.set_state(joints={"rails": m.RAILS + m.RELEASE_PAST - 1e-6}, time=1.05)
    m.step()
    assert m.STATE["on"] == "front"
    kit.set_state(joints={"rails": m.RAILS + m.RELEASE_PAST}, time=1.1)
    m.step()
    assert m.STATE["on"] is None and m.STATE["left_front"] and kit.is_cut("on_front") and m.STATE["events"] == [(1.1, "off the front")]
    assert m.score(None) == 0.25
    calc = calc_turned(m, kit, keys_up=True)
    inside = {"across": 0.0003, "up": -0.0002, "tilt": math.radians(0.5)}  # within the play: taken, and the correction recorded
    for k, (which, kw) in enumerate((("back", {"across": 0.002}), ("back", {"across": 0.002}), ("back", inside))):
        kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, which, 0.140, **kw)}, time=1.15 + 0.05 * k)
        m.step()
        assert m.STATE["on"] is None and m.STATE["corrections"] == []
    kit.set_state(poses={"calc": calc, "cover": cover_at(m, calc, "back", 0.140, **inside)}, time=1.30)
    m.step()
    assert m.STATE["on"] == "back" and not kit.is_cut("on_again") and m.STATE["events"][-1] == (1.3, "on the back")
    assert m.STATE["corrections"] == [{"t": 1.3, "across (mm)": 0.3, "up (mm)": -0.2, "tilt (deg)": 0.5}]
    assert m.report()["corrections at the takes"] == m.STATE["corrections"]
    kit.set_state(joints={"rails": 0.0005}, poses={"calc": calc, "cover": cover_at(m, calc, "back", 0.0005)}, time=2.0,
                  grippers={"left": ((0.1, 0.3, 0.9), 1.0), "right": ((0.1, -0.3, 0.9), 1.0)})  # fmt: skip
    m.step()
    assert m.STATE["rest"] == 0.0 and m.score(None) == 1.0 and m.success(None)


def test_the_score_levels_from_states_set_directly():
    task, m, kit = load()
    far = {"left": ((0.1, 0.3, 0.9), 1.0), "right": ((0.1, -0.3, 0.9), 1.0)}
    m.start(0)
    kit.set_state(time=0.5, grippers=far)
    m.step()
    assert m.score(None) == 0.0  # the start: keys down in the cover, the cover over the keys
    calc_up = calc_turned(m, kit, keys_up=True)
    m.STATE.update(on="back", left_front=True)
    for travel, want in ((0.002, 1.0), (0.004, 0.5), (m.RAILS - m.IN_BACK - 0.0005, 0.5), (m.RAILS - m.IN_BACK + 0.001, 0.25)):
        kit.set_state(joints={"rails": travel}, poses={"calc": calc_up, "cover": cover_at(m, calc_up, "back", travel)}, grippers=far)
        assert m.score(None) == want, travel
    kit.set_state(joints={"rails": 0.0}, poses={"calc": calc_up, "cover": cover_at(m, calc_up, "back", 0.0)},
                  grippers={"left": far["left"], "right": (tuple(calc_up[0] + [0.0, 0.0, 0.02]), 0.2)})  # fmt: skip
    assert m.score(None) == 0.5  # home, but a gripper still on it
    kit.set_state(joints={"rails": 0.0}, poses={"calc": calc_up, "cover": cover_at(m, calc_up, "back", 0.0)}, velocities={"calc": ((0.05, 0, 0), (0, 0, 0))}, grippers=far)
    assert m.score(None) == 0.5  # home, still moving
    calc_down = calc_turned(m, kit, keys_up=False)
    kit.set_state(joints={"rails": 0.0}, poses={"calc": calc_down, "cover": cover_at(m, calc_down, "back", 0.0)}, grippers=far)
    assert m.score(None) == 0.25  # on the back but the pair lies keys down
    m.STATE.update(on="front")
    assert m.score(None) == 0.0  # keys down in the cover, as at the start
    m.STATE.update(on="back", fell=True)
    assert m.score(None) == 0.0
    m.STATE.update(on=None, fell=False, left_front=False)
    assert m.score(None) == 0.0
