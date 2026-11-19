"""Scoring tests with no simulator (run in the simulator's Python environment): python -m pytest -q test_score.py"""
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

import task as t
from plasma_cut import mech as M

DT = 0.05
PARK = {"left_tcp": (0.12, 0.34, t.PARK[2]), "right_tcp": (0.12, -0.34, t.PARK[2])}
HEAD = M.TORCH_REST["heading"]


def quat(yaw=0.0, tilt=0.0):
    """Yaw about z, then a tilt about the world x axis (the torch leaning along the line's normal... about the travel axis)."""
    cz, sz, cx, sx = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
    # q = q_tilt(x) * q_yaw(z)
    return np.array((sx * cz, sx * sz, cx * sz, cx * cz))


def state(pos, yaw=0.0, v=0.0, tilt=0.0):
    p = np.array(pos, float)
    return NS(pos=p, quat_xyzw=quat(yaw, tilt), lin_vel=np.array((v, 0.0, 0.0)), ang_vel=np.zeros(3), low=p, high=p)


def torch_at(rim, tilt=0.0):
    q = quat(HEAD, tilt)
    g = np.asarray(rim, float) - t._rot(q) @ M.RIM
    return NS(pos=g, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=g, high=g)


OFF0 = (M.OFFCUT_C[0], M.OFFCUT_C[1], t.TOP + M.OFFCUT_C[2])
OFF_DOWN = (M.OFFCUT_C[0], M.OFFCUT_C[1], t.TOP + M.TRAY["base"] + M.PLATE_T / 2)
GUIDE_USE = (M.GUIDE_USE["x"], M.GUIDE_USE["y"], t.TOP + M.GUIDE_USE["z"])
GUIDE_REST = (M.GUIDE_REST["x"], M.GUIDE_REST["y"], t.TOP + M.GUIDE_REST_Z)
TORCH_REST = (M.TORCH_REST["x"], M.TORCH_REST["y"], t.TOP + M.TORCH_REST_Z)


def rest_world(**over):
    w = {a: state(p) for a, p in PARK.items()}
    w["torch"] = state(TORCH_REST, HEAD)
    w["guide"] = state(GUIDE_REST)
    w["offcut"] = state(OFF0)
    w.update(over)
    return w


def cutting(x, lat=0.0, rim_h=0.0, tilt=0.0, held=True, guide=GUIDE_USE, guide_held=True, offcut=OFF0):
    """The torch in the right hand with its shield rim's centre at x along the line, lat off it, rim_h above the plate."""
    w = {a: state(p) for a, p in PARK.items()}
    w["torch"] = torch_at((x, M.LINE_Y + lat, t.PLATE_Z + rim_h), tilt)
    if held:
        w["right_tcp"] = state(w["torch"].pos + (0, 0, t.GRIP_DZ))
    w["guide"] = state(guide)
    if guide_held:
        w["left_tcp"] = state(np.asarray(guide) + (0, 0, t.GRIP_DZ))
    w["offcut"] = state(offcut)
    return w


@pytest.fixture(autouse=True)
def fresh():
    t.reset_latch()


def run(speed=t.V_RATED, x_from=M.X1 + 0.01, x_to=M.X0 - 0.006, **kw):
    """Drag the torch from beyond the far edge to beyond the near edge, step by step through note_step."""
    x = x_from
    w = cutting(x, **kw)
    while x > x_to:
        x -= speed * DT
        w = cutting(x, **kw)
        t.note_step(w, DT)
    return w


def finish(w=None, n=30, **over):
    w = rest_world(offcut=state(OFF_DOWN), **over)
    for _ in range(n):
        t.note_step(w, DT)
    return w


def test_nothing_scores_zero():
    w = rest_world()
    for _ in range(100):
        t.note_step(w, DT)
    assert t.score(w) == 0.0 and not t.severed()


def test_full_cut_then_everything_back():
    w = run()
    assert t.sound_segments().all() and len(t.severed()) == t.N
    assert t.score(w) == 0.8  # the offcut hasn't dropped (here it is held where it was)
    w = cutting(M.X0 - 0.006, offcut=OFF_DOWN)
    t.note_step(w, DT)
    assert t.score(w) == 0.9  # dropped, tools still in hand
    w = finish()
    assert t.score(w) == 1.0


def test_rated_window_edges():
    run(speed=0.020)
    assert t.cut_through().all() and not t.overheated().any()
    t.reset_latch()
    run(speed=0.007)
    assert t.sound_segments().all()


def test_half_cut_is_partial_and_offcut_stays():
    w = run(x_to=(M.X0 + M.X1) / 2)
    assert len(t.severed()) == 20 and t.score(w) == 0.4
    assert t.score(finish_no_drop()) == 0.4


def finish_no_drop():
    w = rest_world()
    for _ in range(30):
        t.note_step(w, DT)
    return w


def test_too_fast_does_not_cut():
    w = run(speed=0.030)
    assert not t.severed() and t.score(w) == 0.0 and set(t.faults()) == {"uncut"}


def test_too_slow_cuts_with_dross_and_scores_zero():
    w = run(speed=0.004)
    assert len(t.severed()) == t.N and t.overheated().all()
    assert t.score(finish()) == 0.0


def test_hovering_in_one_place_overheats_one_segment():
    w = cutting(M.X0 + 0.1026)
    for _ in range(40):
        t.note_step(w, DT)
    assert t.overheated().sum() == 1 and t.score(w) == 0.0


def test_freehand_on_the_line_is_unguided():
    w = run(guide=GUIDE_REST, guide_held=False)
    assert len(t.severed()) == t.N and set(t.faults()) == {"unguided"}
    assert t.score(finish()) == 0.0


def test_straightedge_not_on_the_plate_is_unguided():
    up = (GUIDE_USE[0], GUIDE_USE[1], GUIDE_USE[2] + 0.003)
    run(guide=up)
    assert not t.sound_segments().any()


def test_straightedge_set_3_mm_off_cuts_off_line():
    g = (GUIDE_USE[0], GUIDE_USE[1] + 0.003, GUIDE_USE[2])
    w = run(lat=0.003, guide=g)
    assert len(t.severed()) == t.N and set(t.faults()) == {"off-line"} and t.score(w) == 0.0


def test_within_2_mm_of_the_line_is_sound():
    g = (GUIDE_USE[0], GUIDE_USE[1] - 0.0018, GUIDE_USE[2])
    run(lat=-0.0018, guide=g)
    assert t.sound_segments().all()


def test_no_arc_far_from_the_line_or_off_the_plate():
    run(lat=-0.012)
    assert not t._trial["dwell"].any()
    w = cutting(M.X1 + 0.004)
    for _ in range(40):
        t.note_step(w, DT)
    assert not t._trial["dwell"].any()


def test_no_arc_when_lifted_or_not_held():
    run(rim_h=0.003)
    assert not t._trial["dwell"].any()
    run(held=False)
    assert not t._trial["dwell"].any()


def test_rim_up_to_2_mm_above_the_plate_still_cuts():
    g = (GUIDE_USE[0], GUIDE_USE[1], GUIDE_USE[2])
    run(rim_h=0.0018, guide=g)
    assert t.sound_segments().all()


def test_tilted_torch_bevels_or_does_not_fire():
    run(tilt=math.radians(15))
    assert t.cut_through().any() and not t.sound_segments().any() and "bevel" in t.faults()
    t.reset_latch()
    run(tilt=math.radians(35))
    assert not t._trial["dwell"].any()


def test_other_hand_near_the_arc_scores_zero():
    x = M.X0 + 0.1
    w = cutting(x)
    w["left_tcp"] = state((x, M.LINE_Y + 0.03, t.PLATE_Z + 0.01))
    t.note_step(w, DT)
    run()
    assert t._trial["unsafe"] and t.score(finish()) == 0.0


def test_straightedge_hand_at_its_handle_is_safe():
    run()
    assert not t._trial["unsafe"]


def test_tool_off_the_table_scores_zero():
    run()
    w = finish(torch=state((0.2, -0.8, t.TOP - 0.3), HEAD))
    assert t.score(w) == 0.0


def test_end_state_each_missing_item_caps_at_0_9():
    run()
    assert t.score(finish(torch=state((TORCH_REST[0], TORCH_REST[1] + 0.03, TORCH_REST[2]), HEAD))) == 0.9
    assert t.score(finish(guide=state(GUIDE_USE))) == 0.9
    assert t.score(finish(left_tcp=state(np.add(GUIDE_REST, (0, 0, 0.04))))) == 0.9
    w = finish(torch=state(TORCH_REST, HEAD, v=0.05))
    assert t.score(w) == 0.9
    w = finish(right_tcp=state(np.add(OFF_DOWN, (0, 0, 0.03))))
    assert t.score(w) == 0.9  # a hand on the hot offcut


def test_offcut_must_be_down_in_the_tray():
    run()
    w = rest_world()
    for _ in range(30):
        t.note_step(w, DT)
    assert t.score(w) == 0.8
    w = rest_world(offcut=state((0.6, -0.5, t.TOP + 0.002)))
    assert t.score(w) == 0.8


def test_two_fast_passes_add_up():
    run(speed=0.030)
    assert not t.severed()
    run(speed=0.030)
    assert len(t.severed()) == t.N


def test_shown_names():
    run(x_to=(M.X0 + M.X1) / 2)
    on = t.shown()
    assert "kerf_39" in on and "kerf_0" not in on and "fx_arc" in on


def test_demo_fits_and_reaches():
    wps = t.demo()
    steps = max(sum(w.duration for w in ws) for ws in wps.values()) * 20
    _, _, bad = t.ik_errors(wps)
    assert steps < 2400 and not bad
    assert np.linalg.norm(M.TORCH_COM[:2]) < 0.001


def test_offcut_snapped_off_is_not_a_cut():
    """The cut stops one segment short and the offcut is then broken off: it lies in the tray, but that segment isn't cut."""
    run(x_to=M.X0 + 0.0058)
    assert len(t.severed()) == t.N - 1
    w = finish()
    assert t.offcut_dropped(w) and t.score(w) == round(0.8 * 39 / 40, 3)


def test_offcut_pushed_off_with_no_cut_scores_zero():
    assert t.score(finish()) == 0.0
