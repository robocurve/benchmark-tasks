"""Offline checks for task.py: geometry, the button's lock, scoring on hand-built states, and the demo's reach.

    PYTHONPATH=<path to yam-isaac-lab-arena> python -m pytest test_task.py -q

No Isaac Sim needed.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as T  # noqa: E402
from bodykit import state_from_pose  # noqa: E402

IDENT = (0.0, 0.0, 0.0, 1.0)


# -- geometry and the lock ---------------------------------------------------------------------------------------------

def test_let_out_button_locks_the_leg():
    # its straight shank (below the bevel) passes right through the main tube's wall
    assert T.BUTTON_Z - T.CHAMFER > T.R_OUT + 0.0005


def test_button_must_be_pressed_nearly_flush_to_free_the_leg():
    assert 0.0028 <= T.UNLOCK_PRESS <= 0.0032  # about 3 of its 4 mm
    foot = T.BUTTON_Z - T.PROUD - T.CHAMFER  # the bevel's foot with the button's top flush with the tube
    assert foot < T.R_INNER - 0.0008  # pressed flush, the whole wall faces the 45-degree bevel


def test_bevel_leaves_a_flat_top():
    assert 0.0008 <= T.TOP_R <= 0.0012


def test_button_travel_reaches_inside_the_main_tube():
    inside = T.BUTTON_Z - T.R_INNER  # press needed for its top to slide under the wall
    assert inside < T.BUTTON_TRAVEL - 0.0005


def test_button_fits_its_hole_and_the_leg_fits_the_tube():
    assert 0.0002 <= (T.HOLE_W - 2 * T.BUTTON_R) / 2 <= 0.0005
    assert 0.0002 <= T.R_INNER - T.LEG_R <= 0.0006


def test_spring_force():
    rest = T.BUTTON_K * T.BUTTON_PRE
    inside = T.BUTTON_K * (T.BUTTON_PRE + T.BUTTON_Z - T.R_INNER)
    assert 2.5 <= rest <= 4.0  # a light thumb press frees it
    assert 4.5 <= inside <= 7.0


def test_handle_height_matches_a_real_adult_forearm_crutch():
    # e.g. Drive 10403: handle height 28 to 37 in (0.71 to 0.94 m)
    assert 0.70 <= T.length_at(T.N_HOLES) <= 0.75
    assert 0.88 <= T.length_at(1) <= 0.94
    assert T.length_at(1) - T.length_at(T.N_HOLES) == pytest.approx(8 * T.PITCH)


def test_leg_stays_in_the_hollow_part_of_the_tube():
    assert T.hole_y(T.N_HOLES) + 0.006 + T.LEG_TOP < T.OCT_TO
    assert T.hole_y(T.N_HOLES) + T.HOLE_W / 2 < T.OCT_TO


def test_handgrip_and_cuff_clear_the_table():
    assert T.AXIS_UP - (0.010 + T.GRIP_L) >= 0.008
    lowest_cuff = T.CUFF_DOWN + T.CUFF_R * math.sin(math.radians(30.0)) + 0.003
    assert T.AXIS_UP - lowest_cuff >= 0.03
    assert T.AXIS_UP - T.TIP_R >= 0.1


def test_everything_on_the_table():
    from yam_arena import station

    cx, cy = station.TABLE_CENTER[:2]
    hx, hy = station.TABLE_SIZE[0] / 2, station.TABLE_SIZE[1] / 2
    for name, (x, _) in T.CRUTCHES.items():
        longest_tip = T.C_Y - (T.LEG_LEN - T.LEG_TOP - T.hole_y(1)) - T.TIP_PAST
        assert longest_tip > cy - hy + 0.02
        assert T.C_Y + T.CUFF_Y1 < cy + hy - 0.02
        assert cx - hx + 0.05 < x < cx + hx - 0.05


def test_crutches_far_enough_apart():
    xs = sorted(x for x, _ in T.CRUTCHES.values())
    gap = xs[1] - xs[0]
    assert gap > 2 * (T.CUFF_R + 0.003) + 0.01
    assert gap - T.R_OUT - 0.094 / 2 > 0.03  # the open fingers round one tube stay clear of the other


def test_centre_of_mass_between_the_cradles():
    leg_mid = {n: T.hole_y(k) + (T.LEG_TOP - T.LEG_LEN / 2) for n, (_, k) in T.CRUTCHES.items()}
    for name in T.CRUTCHES:
        y = (T.CRUTCH_MASS * 0.30 + T.LEG_MASS * leg_mid[name]) / (T.CRUTCH_MASS + T.LEG_MASS)
        assert T.CRADLE_YS[0] + 0.05 < y < T.CRADLE_YS[1] - 0.05


def test_starts_and_target():
    assert T.CRUTCHES["near"][1] < T.TARGET < T.CRUTCHES["far"][1]
    assert all(1 <= k <= T.N_HOLES for _, k in T.CRUTCHES.values())


def test_objects_and_joints():
    from bodykit import _SLIDERS

    names = [o.name for o in T.OBJECTS]
    assert len(names) == len(set(names))
    assert names.index("crutch_near") < names.index("leg_near") < names.index("button_near")
    sl = {s.name: s for s in _SLIDERS}
    for name, (_, k) in T.CRUTCHES.items():
        lo, hi = sl[f"slide_{name}"].limits
        assert lo == pytest.approx(T.hole_y(1) - T.hole_y(k) - 0.006)
        assert hi == pytest.approx(T.hole_y(T.N_HOLES) - T.hole_y(k) + 0.006)
        assert sl[f"spring_{name}"].limits == (-T.BUTTON_TRAVEL, 0.0)


# -- scoring -----------------------------------------------------------------------------------------------------------

def _objects(button_k=None, out=None, roll=0.0, crutch_dz=None, crutch_dy=None):
    """Hand-built states: each crutch where it started (turned ``roll`` degrees about its axis), its button at hole
    position ``button_k[name]`` standing ``out[name]`` m out of the tube."""
    objs = {}
    for name, (x, k0) in T.CRUTCHES.items():
        o = T.crutch_origin(name) + [0.0, (crutch_dy or {}).get(name, 0.0), (crutch_dz or {}).get(name, 0.0)]
        a = math.radians(roll) / 2
        q = (0.0, math.sin(a), 0.0, math.cos(a))  # about y, the crutch's axis
        rot = T.rotation(q)
        k = (button_k or {}).get(name, k0)
        h = (out or {}).get(name, T.PROUD)
        b_local = np.array([0.0, T.hole_y(1) + (k - 1) * T.PITCH, T.R_OUT + h])
        l_local = np.array([0.0, b_local[1], 0.0])
        objs[f"crutch_{name}"] = state_from_pose(T.BODIES[name][0], o, q)
        objs[f"leg_{name}"] = state_from_pose(T.BODIES[name][1], o + rot @ l_local, q)
        objs[f"button_{name}"] = state_from_pose(T.BODIES[name][2], o + rot @ b_local, q)
    return objs


@pytest.fixture(autouse=True)
def _clear():
    T._new_episode()
    T._EP["tcps"] = [np.array([0.15, 0.30, T.TOP + 0.20]), np.array([0.15, -0.30, T.TOP + 0.20])]
    yield


SET = {"near": T.TARGET, "far": T.TARGET}


def test_nothing_done_scores_zero():
    assert T.score(_objects()) == 0.0


def test_both_set_and_let_go_scores_one():
    assert T.score(_objects(button_k=SET)) == 1.0
    assert T.success(_objects(button_k=SET))


def test_one_set_scores_04():
    assert T.score(_objects(button_k={"near": T.TARGET})) == pytest.approx(0.4)
    assert T.score(_objects(button_k={"far": T.TARGET})) == pytest.approx(0.4)


def test_button_caught_on_the_hole_edge_is_not_set():
    assert T.score(_objects(button_k=SET, out={"near": 0.002})) == pytest.approx(0.4)


def test_button_pressed_in_under_the_wall_is_not_set():
    # the leg slid 6 mm short: the button is under the wall, its top level with the tube's inside
    k = T.TARGET - 0.006 / T.PITCH
    st = _objects(button_k={"near": k, "far": T.TARGET}, out={"near": -T.WALL})
    assert T.score(st) == pytest.approx(0.4)


def test_neighbouring_hole_is_not_set():
    assert T.score(_objects(button_k={"near": T.TARGET + 1, "far": T.TARGET - 1})) == 0.0


def test_in_hole_tolerance():
    assert T.score(_objects(button_k={n: T.TARGET + 0.0012 / T.PITCH for n in T.CRUTCHES})) == 1.0
    assert T.score(_objects(button_k={n: T.TARGET + 0.0020 / T.PITCH for n in T.CRUTCHES})) == 0.0


def test_still_holding_a_leg_scores_08():
    o = T.crutch_origin("far")
    T._EP["tcps"][1] = o + np.array([0.0, -0.10, 0.004])  # pinching the far leg
    assert T.score(_objects(button_k=SET)) == pytest.approx(0.8)


def test_pressing_a_button_counts_as_holding():
    o = T.crutch_origin("near")
    T._EP["tcps"][0] = o + np.array([0.0, T.hole_y(T.TARGET), T.R_OUT + 0.013])
    assert T.score(_objects(button_k=SET)) == pytest.approx(0.8)


def test_crutch_rolled_off_its_keel_is_not_on_its_stand():
    assert T.score(_objects(button_k=SET, roll=30.0)) == pytest.approx(0.8)


def test_dropped_scores_zero():
    T._EP["dropped"] = True
    assert T.score(_objects(button_k=SET)) == 0.0


def test_lifted_off_its_stand():
    assert T.score(_objects(button_k=SET, crutch_dz={"near": 0.02})) == pytest.approx(0.8)


def test_set_reading_survives_a_roll():
    # the reading is in the crutch's own frame: a crutch turned 10 degrees (still on its stand) still reads correctly
    assert T.score(_objects(button_k=SET, roll=10.0)) == 1.0


# -- the demo ------------------------------------------------------------------------------------------------------------

VARIANTS = ["full", "nothing", "near_only", "probe_far", "probe_near", "no_press", "half_way", "keep_holding"]


@pytest.mark.parametrize("variant", VARIANTS)
def test_demo_builds(variant, monkeypatch):
    monkeypatch.setenv("DEMO_VARIANT", variant)
    d = T.demo()
    la, lr = T._length(d["left"]), T._length(d["right"])
    assert la == pytest.approx(lr)
    assert la < 2400 / 20 - 10  # well within the run limit


def _times(wps):
    out, t = [], 0.0
    for w in wps:
        out.append((t, t + w.duration, w))
        t += w.duration
    return out


def test_leg_only_slides_while_the_left_holds_still():
    for name in T.CRUTCHES:
        left, right = T.plan_crutch(name)
        moves = [(a, b) for (a, b, w), prev in zip(_times(right)[1:], right) if w.pos != prev.pos
                 and w.finger == T.FINGER_CLOSED and prev.finger == T.FINGER_CLOSED]
        assert len(moves) == abs(T.TARGET - T.CRUTCHES[name][1]) + 1
        for a, b in moves:
            for t in np.arange(a + 0.01, b, 0.05):
                assert np.allclose(_pos_at(left, t), _pos_at(left, b - 0.001), atol=1e-9), (name, t)


def test_left_never_presses_the_red_hole():
    for name in T.CRUTCHES:
        left, _ = T.plan_crutch(name)
        red = T.press_point(name, T.TARGET)
        for w in left:
            if "press on" in w.label:
                assert abs(w.pos[1] - red[1]) >= T.PITCH / 2 - 1e-9


def test_left_presses_every_hole_the_button_passes():
    for name, (_, k0) in T.CRUTCHES.items():
        left, _ = T.plan_crutch(name)
        pressed = sorted(round(1 + (w.pos[1] - T.crutch_origin(name)[1] - T.hole_y(1)) / T.PITCH)
                         for w in left if "press on hole" in w.label)
        assert pressed == list(range(min(k0, T.TARGET) + 1, max(k0, T.TARGET)))


def test_slide_ends_at_the_red_hole():
    for name, (_, k0) in T.CRUTCHES.items():
        left, right = T.plan_crutch(name)
        last = next(w for w in reversed(right) if w.finger == T.FINGER_CLOSED)
        want = T.hole_y(T.TARGET) - T.hole_y(k0)
        assert last.pos[1] - T.grip_point(name, 0.0)[1] == pytest.approx(want + math.copysign(T.OVERSHOOT, want))
        assert T.OVERSHOOT < T.PITCH / 2 - T.ALIGN  # the lock catches the button before the next hole


def test_no_press_never_reaches_the_button(monkeypatch):
    monkeypatch.setenv("DEMO_VARIANT", "no_press")
    d = T.demo()
    for w in d["left"]:
        for name in T.CRUTCHES:
            o = T.crutch_origin(name)
            if abs(w.pos[0] - o[0]) < 0.03:
                assert w.pos[2] - T.TIP >= o[2] + T.BUTTON_Z + 0.004, w.label


def test_grip_stays_clear_of_collar_and_cradle():
    for name, (_, k) in T.CRUTCHES.items():
        slide = T.hole_y(T.TARGET) - T.hole_y(k)
        for s in (0.0, slide):
            y = T.GRIP_LEG_Y[name] + s
            assert y + 0.0137 + 0.005 < T.COLLAR_Y[0]  # the finger's 27 mm width, 5 mm clear of the collar
            leg_bottom = T.hole_y(k) + s + T.LEG_TOP - T.LEG_LEN  # the rubber tip covers its last 1 cm
            assert y - 0.0137 > leg_bottom + 0.010 + 0.010  # on the tube, 1 cm clear of the rubber tip


def _pos_at(wps, t: float) -> np.ndarray:
    """Where a waypoint list has the TCP at time t, moving straight between waypoints (each reached at its end)."""
    prev, start = np.array(wps[0].pos), 0.0
    for w in wps:
        end = start + w.duration
        if t <= end:
            s = (t - start) / w.duration if w.duration > 0 else 1.0
            return prev + (np.array(w.pos) - prev) * s
        prev, start = np.array(w.pos), end
    return prev


def test_hands_stay_apart(monkeypatch):
    monkeypatch.setenv("DEMO_VARIANT", "full")
    d = T.demo()
    total = T._length(d["left"])
    gaps = [np.linalg.norm(_pos_at(d["left"], t)[:2] - _pos_at(d["right"], t)[:2])
            for t in np.arange(2.0, total, 0.05)]  # from 2 s: both arms have left their start
    assert min(gaps) > 0.12


def _reaches(arm: str, waypoints) -> list[str]:
    import itertools

    from yam_arena import ik, station
    from yam_arena.demo import grip_rotation

    seeds = [np.array(s) for s in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3, 1.8), (0.4, 1.0, 1.6), (-0.8, 0.0, 0.8),
                                                    (-1.4, 0.0, 1.4), (0.0,))]
    bad, q, seen = [], None, {}
    for w in waypoints:
        key = (tuple(np.round(w.pos, 4)), round(w.yaw, 4))
        if key in seen:
            continue
        target = ik.matrix_to_quat_xyzw(grip_rotation(w.yaw))
        best = None
        for seed in ([q] if q is not None else []) + seeds:
            qs, pe, re = ik.solve(np.array(w.pos), target, q_init=seed, base_pos=station.BASES[arm], iters=150)
            if best is None or pe + re * 0.05 < best[1] + best[2] * 0.05:
                best = (qs, pe, re)
            if pe < 0.001 and re < 0.01:
                break
        q, pe, re = best
        seen[key] = True
        if pe >= 0.002 or re >= 0.02:
            bad.append(f"{arm} {w.label} at {np.round(w.pos, 3).tolist()}: {pe * 1000:.1f} mm, {math.degrees(re):.1f} deg")
    return bad


@pytest.mark.skipif(os.environ.get("SKIP_IK") == "1", reason="SKIP_IK=1")
def test_demo_reaches_every_pose(monkeypatch):
    monkeypatch.setenv("DEMO_VARIANT", "full")
    d = T.demo()
    bad = _reaches("left", d["left"]) + _reaches("right", d["right"])
    assert not bad, "\n".join(bad)


def test_slid_along_its_stand():
    assert T.score(_objects(button_k=SET, crutch_dy={"far": 0.015})) == 1.0
    assert T.score(_objects(button_k=SET, crutch_dy={"far": 0.030})) == pytest.approx(0.8)


# -- the lock -----------------------------------------------------------------------------------------------------------

H5 = None


def test_held_stays_held_however_far_it_is_pulled():
    assert T.lock_step(5, T.hole_y(5) + 0.004, T.PROUD) == (5, 5, False)


def test_pressing_frees_a_held_leg():
    assert T.lock_step(5, T.hole_y(5), 0.0005) == (None, 5, False)


def test_free_between_holes_the_wall_holds_the_button_in():
    assert T.lock_step(None, T.hole_y(5) + 0.003, -0.0012) == (None, 5, True)
    assert T.lock_step(None, T.hole_y(5) + T.PITCH / 2, -0.0012)[2]


def test_free_over_a_hole_it_springs_out_and_catches():
    assert T.lock_step(None, T.hole_y(6) - 0.002, -0.0012) == (None, 6, False)  # free to rise
    assert T.lock_step(None, T.hole_y(6) - 0.002, 0.0015) == (6, 6, False)  # out past 1 mm: held


def test_pressed_button_passes_over_a_hole():
    assert T.lock_step(None, T.hole_y(6), 0.0) == (None, 6, False)


def test_catch_window_is_the_flat_top_in_the_hole():
    assert T.ALIGN == pytest.approx(0.0025)


def test_unlock_needs_three_of_four_millimetres():
    assert T.OUT_LOCKS == pytest.approx(T.PROUD - T.UNLOCK_PRESS)
    assert 0.0008 <= T.OUT_LOCKS <= 0.0012


def test_nearest_hole():
    assert T.nearest_hole(T.hole_y(5) + 0.012) == 5
    assert T.nearest_hole(T.hole_y(5) + 0.013) == 6
    assert T.nearest_hole(-1.0) == 1 and T.nearest_hole(1.0) == T.N_HOLES


def test_button_inside_sits_level_with_the_tubes_inside():
    assert T.BUTTON_Z + T.INSIDE == pytest.approx(T.R_INNER)
    assert -T.BUTTON_TRAVEL < T.INSIDE < 0.0


def test_held_leg_resists_a_strong_pull():
    assert 20.0 / T.LOCK_K <= 0.0012  # a 20 N pull moves it about 1 mm
    assert T.LOCK_FORCE >= 200.0


def test_first_stretch_passes_under_the_wall():
    assert T.ALIGN < T.PAST < T.PITCH - T.ALIGN - 0.005  # under the wall, short of the next hole


def test_a_hole_flown_over_between_steps_catches():
    # free, walled, the leg jumps 3 cm in one step across hole 6: the button springs out at hole 6 and the leg is held
    assert T.skipped_hole(T.hole_y(6) - 0.012, T.hole_y(6) + 0.018) == 6
    assert T.lock_step(None, T.hole_y(6) + 0.018, -0.0012, prev_y=T.hole_y(6) - 0.012) == (6, 6, False)


def test_a_slow_slide_never_skips_a_hole():
    # 1.3 mm per step: every hole is seen inside its window, so the crossing rule never fires
    y = T.hole_y(7) - 0.010
    while y < T.hole_y(7) + 0.010:
        assert T.skipped_hole(y, y + 0.0013) is None
        y += 0.0013
    assert T.skipped_hole(None, T.hole_y(7)) is None


def test_a_crossed_hole_under_a_fingertip_frees_again():
    held, k, inside = T.lock_step(None, T.hole_y(6) + 0.018, -0.0012, prev_y=T.hole_y(6) - 0.012)
    assert held == 6
    # the drive centred the leg on hole 6 and the pad kept the button at 0.5 mm out: free again next step
    assert T.lock_step(held, T.hole_y(6), 0.0005) == (None, 6, False)
