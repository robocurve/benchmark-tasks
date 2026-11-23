"""Scoring and demo checks for copy-id-card that run without the simulator: python -m pytest -q test_copy_id_card.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import copy_id_card as M
from mechanism import MechanismState

HERE = Path(__file__).resolve().parent
Z3 = np.zeros(3)


def pads(x, y, z) -> M.Pads:
    return M.Pads(back=np.array([x, y, z + 0.04]), tips=np.array([x, y, z]))


AWAY = {"left_gripper": pads(*M.PARK_LEFT), "right_gripper": pads(*M.PARK_RIGHT)}


def quat(axis, deg) -> np.ndarray:
    h = math.radians(deg) / 2
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[a * math.sin(h), math.cos(h)]


def qmul(a, b) -> np.ndarray:
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


FACE_DOWN = quat((1, 0, 0), 180.0)
GLASS_MID_Z = M.TOP + M.GLASS_Z + M.CARD[2] / 2


def card(x=None, y=None, z=GLASS_MID_Z, q=FACE_DOWN, moving=False) -> ObjectState:
    x = M.ALIGNED_CENTRE[0] if x is None else x
    y = M.ALIGNED_CENTRE[1] if y is None else y
    pos = np.array([x, y, z])
    r = M._rot(q)
    half = np.abs(r) @ (np.asarray(M.CARD) / 2)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(q, float), lin_vel=np.array([0.1, 0, 0]) if moving else Z3,
                       ang_vel=Z3, low=pos - half, high=pos + half)


def printer(lid_deg=0.0, black=0.0, color=0.0) -> MechanismState:
    return MechanismState(pos=np.array([0, 0, M.TOP]), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=Z3, ang_vel=Z3,
                          low=Z3, high=Z3, joints={"lid": math.radians(lid_deg), "black_copy": black,
                                                   "color_copy": color})


def objects(c=None, lid_deg=0.0, record=None, grippers=None) -> dict:
    out = {"card": c if c is not None else card(), "printer": printer(lid_deg), **(grippers or AWAY)}
    out["copy_record"] = record if record is not None else M.CopyRecord()
    return out


GOOD = M.CopyRecord(presses=1, good=True, max_lid=math.radians(65))


def test_the_task_file_loads():
    task = load(HERE / "copy_id_card.py")
    assert [o.name for o in task.objects] == ["printer", "card"]
    assert task.scene == "station"


# -- the card's place ---------------------------------------------------------------------------------------------------
def test_a_card_square_in_the_corner_face_down_is_aligned():
    c = card()
    assert M.face_down(c) and M.on_glass(c) and M.aligned(c)
    assert M.squareness_deg(c) < 1e-6


def test_alignment_limits():
    ok = card(M.ALIGNED_CENTRE[0] + 0.0039, M.ALIGNED_CENTRE[1] + 0.0039)
    off_x = card(M.ALIGNED_CENTRE[0] + 0.0045)
    off_y = card(y=M.ALIGNED_CENTRE[1] + 0.0045)
    assert M.aligned(ok) and not M.aligned(off_x) and not M.aligned(off_y)


def test_a_turned_card_is_not_square():
    centre = (M.ALIGNED_CENTRE[0] + 0.004, M.ALIGNED_CENTRE[1] + 0.004)
    turned = card(*centre, q=qmul(quat((0, 0, 1), 4.0), FACE_DOWN))
    assert M.squareness_deg(turned) == pytest.approx(4.0, abs=0.01)
    assert not M.aligned(turned)


def test_a_card_turned_a_quarter_still_counts_as_square():
    # long side front to back: its own front-right corner at the corner
    centre = (M.GX0 + M.CARD[1] / 2, M.GY0 + M.CARD[0] / 2)
    c = card(*centre, q=qmul(quat((0, 0, 1), 90.0), FACE_DOWN))
    assert M.squareness_deg(c) < 1e-6 and M.aligned(c)


def test_a_card_on_the_frame_or_raised_is_not_on_the_glass():
    assert not M.on_glass(card(M.GX0 + M.CARD[0] / 2 - 0.002))  # its front edge over the frame
    assert not M.on_glass(card(z=GLASS_MID_Z + 0.004))  # held above it
    assert not M.on_glass(card(z=M.CARD_START[2], q=(0, 0, 0, 1.0)))


def test_faces():
    assert M.face_up(card(q=(0, 0, 0, 1.0))) and not M.face_down(card(q=(0, 0, 0, 1.0)))
    tilted = card(q=qmul(quat((1, 0, 0), 15.0), FACE_DOWN))
    assert not M.face_down(tilted) and not M.face_up(tilted)


# -- the grade ----------------------------------------------------------------------------------------------------------
def test_full_score_needs_a_good_copy_and_hands_off():
    assert M.grade(objects(record=GOOD)) == 1.0
    assert M.success(objects(record=GOOD))
    assert M.score(objects(record=GOOD)) == M.COMPLETE_SCORE
    on_card = {"left_gripper": pads(*M.PARK_LEFT), "right_gripper": pads(*M.ALIGNED_CENTRE, GLASS_MID_Z + 0.01)}
    assert M.grade(objects(record=GOOD, grippers=on_card)) == 0.75
    assert M.grade(objects(c=card(moving=True), record=GOOD)) == 0.75


def test_placed_and_shut_without_a_copy_is_three_quarters():
    assert M.grade(objects()) == 0.75


def test_a_good_copy_then_the_card_moved_is_not_full():
    assert M.grade(objects(c=card(M.ALIGNED_CENTRE[0] + 0.02), record=GOOD)) == 0.5


def test_face_down_but_not_aligned_or_lid_open_is_half():
    assert M.grade(objects(c=card(M.ALIGNED_CENTRE[0] + 0.02))) == 0.5
    assert M.grade(objects(lid_deg=30.0)) == 0.5


def test_face_up_on_the_glass_after_opening_the_lid_is_a_quarter():
    up = card(q=(0, 0, 0, 1.0))
    assert M.grade(objects(c=up, record=M.CopyRecord(max_lid=math.radians(65)))) == 0.25
    assert M.grade(objects(c=up, record=M.CopyRecord(max_lid=math.radians(40)))) == 0.0


def test_a_wasted_copy_scores_nothing():
    wasted = M.CopyRecord(presses=1, good=False, max_lid=math.radians(65))
    assert M.grade(objects(record=wasted)) == 0.0
    assert M.grade(objects(c=card(q=(0, 0, 0, 1.0)), record=wasted)) == 0.0


def test_the_card_still_on_the_lid_or_on_the_table_is_nothing():
    assert M.grade(objects(c=card(*M.CARD_START[:2], z=M.CARD_START[2], q=(0, 0, 0, 1.0)))) == 0.0
    assert M.grade(objects(c=card(0.15, -0.3, z=M.TOP + M.CARD[2] / 2))) == 0.0


def test_a_color_copy_scores_nothing():
    """The instruction asks for a black-and-white copy: Color Copy is a wrong copy even with the card placed perfectly,
    and nothing after it earns credit."""
    color = M.CopyRecord(presses=1, good=False, max_lid=math.radians(65), color=1)
    assert color.wrong_copy() and not color.asked_for_copy()
    assert M.grade(objects(record=color)) == 0.0
    then_black = M.CopyRecord(presses=2, good=False, max_lid=math.radians(65), color=1)
    assert M.grade(objects(record=then_black)) == 0.0


def test_a_second_copy_scores_nothing():
    """One copy is asked for: a good Black Copy followed by any second press is an extra copy and scores 0."""
    twice = M.CopyRecord(presses=2, good=True, max_lid=math.radians(65))
    assert twice.wrong_copy() and not twice.asked_for_copy()
    assert M.grade(objects(record=twice)) == 0.0
    assert not M.success(objects(record=twice))


def test_the_press_record():
    M._clear_record()
    shut = objects()
    open_lid = objects(lid_deg=65.0)
    black = {**shut, "printer": printer(0.0, black=-0.002)}
    color = {**shut, "printer": printer(0.0, color=-0.002)}
    assert M._record(open_lid).max_lid == pytest.approx(math.radians(65))
    r = M._record(black)
    assert r.presses == 1 and r.good and r.color == 0 and r.asked_for_copy()
    assert M._record(black).presses == 1  # held down: still one press
    M._record(shut)
    r = M._record(black)  # the same button again: a second copy
    assert r.presses == 2 and r.wrong_copy()
    M._clear_record()
    assert M._record(shut) == M.CopyRecord()
    r = M._record(color)  # Color Copy with the card placed perfectly
    assert r.presses == 1 and r.color == 1 and not r.good and r.wrong_copy()
    M._record(shut)
    r = M._record(black)  # Black Copy afterwards doesn't undo it
    assert r.presses == 2 and not r.good and r.wrong_copy()
    M._clear_record()
    bad = {**objects(c=card(q=(0, 0, 0, 1.0))), "printer": printer(0.0, black=-0.002)}
    r = M._record(bad)
    assert r.presses == 1 and not r.good
    M._clear_record()


@pytest.mark.parametrize("variant,presses", [("", 1), ("color", 1), ("twice", 2)])
def test_the_probes_press_the_buttons_they_say(monkeypatch, variant, presses):
    monkeypatch.setenv("COPY_DEMO", variant)
    left = M.demo()["left"]
    assert sum(w.label.startswith("press it") for w in left) == presses
    press = next(w for w in left if w.label == "press it")
    target = M.COLOR_XY if variant == "color" else M.BLACK_XY
    assert press.pos[:2] == pytest.approx(target)


# -- the printer and the demo -------------------------------------------------------------------------------------------
def test_the_printer_is_real_size_and_the_glass_is_the_scan_area():
    assert (M.BODY_D, M.BODY_W) == (0.304, 0.425)
    assert (M.GLASS_X, M.GLASS_Y) == (0.216, 0.297)
    assert M.GX0 >= M.BODY_X0 and M.GX1 <= M.BODY_X1 and M.GY0 >= -M.BODY_W / 2 and M.GY1 <= M.LID_Y1
    assert M.PRINTER.size[2] == pytest.approx(M.LID_TOP, abs=1e-6)


def test_the_card_starts_on_the_lid_overhanging_its_front():
    assert M.CARD_START[0] - M.CARD[0] / 2 == pytest.approx(M.LID_X0 - M.CARD_OVERHANG)
    assert M.LID_Y0 < M.CARD_START[1] - M.CARD[1] / 2 and M.CARD_START[1] + M.CARD[1] / 2 < M.LID_Y1


def test_the_card_is_turned_over_off_the_lids_side():
    assert M.FLIP_AT[1] + M.CARD[1] / 2 < M.LID_Y0  # clear of the lid while it swings up


def test_the_pinch_stays_short_of_the_lid():
    assert M.PINCH_TCP[0] + M.PAD_TIP_PAST_TCP < M.LID_X0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(monkeypatch, arm):
    monkeypatch.delenv("COPY_DEMO", raising=False)
    p = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, p.duration, 0.05):
        p.action7(t)
    # WHY 1.5 mm, not 0.5: the right hand's IK misses by up to 1.2 mm for a moment while sliding in beside the card,
    # in free air, as its wrist roll passes its limit; every contact step is within 0.1 mm (offline check, 2026-11-20).
    assert p.worst_ik_mm < 1.5 and p.worst_ik_deg < 0.5, (p.worst_ik_mm, p.worst_ik_deg)


def test_the_demo_fits_the_run_limit():
    plans = M.demo()
    assert max(sum(w.duration for w in plans[a]) for a in ("left", "right")) * 20 < 1800
