"""Offline unit, shortcut, reachability and top-camera visibility tests for ims-cassette-load-and-latch."""

from __future__ import annotations

import inspect
import json
import math
import pathlib
import re

import numpy as np
import pytest

from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

PKG = pathlib.Path(__file__).resolve().parent
TASK = load(PKG / "ims_cassette_load_and_latch.py")
M = TASK.module
PARK = ((0.20, +0.26, M.TOP + 0.20), (0.16, -0.32, M.TOP + 0.20))
NEAR = ((0.20, +0.26, M.TOP + 0.20), (M.CXY[0] - 0.04, M.CXY[1], M.Z0 + 0.04))
Q_UP = np.array([0.0, 0.0, 0.0, 1.0])


def _ps(pos, joints=None, quat=Q_UP):
    pos = np.asarray(pos, float)
    return M.labkit.PartState(pos, np.asarray(quat, float), np.zeros(3), np.zeros(3), pos, pos, joints or {})


def _inst_at(x, z=None, y=0.0, quat=Q_UP):
    z = M.SEAT_Z if z is None else z
    return _ps((M.CXY[0] + x, M.CXY[1] + y, M.Z0 + z), quat=quat)


def _state(slots=None, lid_deg=0.0, clip_deg=-90.0):
    """slots: {name: slot index} for seated instruments; the others lie on the tray."""
    slots = {"inst1": 0, "inst2": 1, "inst3": 2, "inst4": 3, "inst5": 4} if slots is None else slots
    out = {"cassette": _ps((*M.CXY, M.Z0), {"lid": math.radians(lid_deg), "clip": math.radians(clip_deg)})}
    for n in M.NAMES:
        if n in slots:
            out[n] = _inst_at(M.SLOT_X[slots[n]])
        else:
            out[n] = _ps((M.CXY[0] + M.SPARE_X[0 if n == "inst2" else 1], M.CXY[1], M.TOP + M.HOLD + M.HANDLE_R))
    return out


START = {"inst1": 0, "inst3": 2, "inst5": 4}


@pytest.fixture(autouse=True)
def _clean_trial():
    M._reset(None)
    yield
    M._reset(None)


# 1. loads
def test_task_file_loads_with_station_and_hooks():
    assert TASK.scene == "station"
    assert {o.name for o in TASK.objects} == {"cassette", "keeper", "tray", "inst1", "inst2", "inst3", "inst4", "inst5"}
    assert TASK.score is not None and TASK.success is not None and TASK.demo is not None
    assert M.SCORE_OVER == "end" and M.SUCCESS_HOLD_S == 1.0


# 2. start 0, solved 1
def test_start_state_scores_zero_and_solved_scores_one():
    assert TASK.score(_state(START, lid_deg=120.0, clip_deg=125.0)) == 0.0
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_state()) == 1.0
    assert TASK.success(_state())


# 3. every documented tier reachable, nothing else
def test_documented_partial_score_tiers():
    text = json.loads((PKG / "task.json").read_text())["scoring"]
    documented = sorted(float(v) for v in re.search(r"Possible: ([\d., ]+)\.", text).group(1).split(","))
    seen = set()
    for tcps in (None, PARK, NEAR):
        for slots in (START, {**START, "inst2": 1}, {**START, "inst4": 3}, {**START, "inst2": 1, "inst4": 3},
                      {**START, "inst2": 3, "inst4": 1}):
            for lid in (0.0, 0.5, 7.0, 105.0):
                for clip in (125.0, 0.0, -45.0, -87.0, -89.0, -92.0):
                    M._set_test_trial(tcps=tcps)
                    seen.add(round(TASK.score(_state(slots, lid, clip)), 6))
    assert sorted(seen) == documented == [0.0, 0.2, 0.4, 0.7, 1.0]


def test_tiers_named_in_scoring():
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_state({**START, "inst2": 1}, 105.0, 0.0)) == pytest.approx(0.2)
    assert TASK.score(_state({**START, "inst2": 1, "inst4": 3}, 105.0, 0.0)) == pytest.approx(0.4)
    assert TASK.score(_state(None, 0.0, 0.0)) == pytest.approx(0.7)  # lid flat, clip open
    assert TASK.score(_state(None, 0.0, -90.0)) == pytest.approx(1.0)


def test_lid_and_latch_count_only_with_all_five_seated():
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_state({**START, "inst2": 1}, 0.0, -90.0)) == pytest.approx(0.2)


@pytest.mark.parametrize("lid_deg, want", [(0.0, 1.0), (0.6, 1.0), (0.8, 0.0), (6.8, 0.0)])
def test_lid_flat_means_front_gap_1_mm(lid_deg, want):
    M._set_test_trial(tcps=PARK)
    got = TASK.score(_state(None, lid_deg, -90.0))
    assert got == pytest.approx(1.0 if want else 0.4)


@pytest.mark.parametrize("clip_deg, want", [(-92.0, 1.0), (-89.5, 1.0), (-88.5, 0.7), (-87.0, 0.7), (0.0, 0.7),
                                            (125.0, 0.7)])
def test_clip_hooked_under_the_keeper_lip(clip_deg, want):
    # -88.5 deg: the tab overlaps the lip by only ~0.35 mm (< 0.5 mm), so it does not count
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_state(None, 0.0, clip_deg)) == pytest.approx(want)


@pytest.mark.parametrize("dx_mm, dz_mm, want", [(0.0, 0.0, True), (2.9, 0.0, True), (3.2, 0.0, False),
                                                (0.0, 0.9, True), (0.0, 1.2, False), (6.35, 7.25, False)])
def test_seated_tolerances(dx_mm, dz_mm, want):
    ps = _inst_at(M.SLOT_X[1] + dx_mm / 1000.0, M.SEAT_Z + dz_mm / 1000.0)
    assert (M._slot_of(ps) == 1) is want


def test_handle_across_the_rails_is_not_seated():
    q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))  # turned 90 deg about the vertical
    assert M._slot_of(_inst_at(0.0, M.TOOTH_Z + M.HANDLE_R, y=M.RAIL_Y, quat=q)) is None


def test_two_instruments_in_one_slot_count_once():
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_state({**START, "inst2": 1, "inst4": 1}, 0.0, -90.0)) == pytest.approx(0.2)


# 4. fail closed
def test_fail_closed_when_tcps_missing():
    M._set_test_trial(tcps=None)
    assert TASK.score(_state()) == pytest.approx(0.7)
    assert not M._hands_clear(None)


# 5. shortcuts (state level; mj_check SHORTCUTS rolls them out in MuJoCo)
def test_shortcuts_score_low():
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_state(START, 105.0, 0.0)) == 0.0  # nothing
    assert TASK.score(_state(START, 0.0, -90.0)) == 0.0  # lid shut and locked with two instruments still out
    assert TASK.score(_state(START, 6.8, -90.0)) == 0.0  # lid down on a misplaced instrument
    M._set_test_trial(tcps=NEAR)
    assert TASK.score(_state()) == pytest.approx(0.7)  # done, gripper still on the cassette


# 6. rehearsal uses real contact
def test_no_welds_or_state_writes():
    src = (PKG / "ims_cassette_load_and_latch.py").read_text()
    for banned in ("weld", "attach", "set_joint_position(", "write_joint_state", "write_root"):
        assert banned not in src, banned


# 7. demo open loop
def test_demo_is_pure_and_open_loop():
    src = inspect.getsource(M.demo)
    for banned in ("_part_state", "labkit.env", "_trial", "score(", "success(", "objects"):
        assert banned not in src, banned
    assert M.demo() == M.demo()


# 8. reach + joint limits
@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reachability_and_joint_limits(arm):
    tj = json.loads((PKG / "task.json").read_text())
    limit = int(re.search(r"([\d,]+)", tj["run_limit"]).group(1).replace(",", ""))
    plan = ArmPlan(TASK.demo()[arm], station.BASES[arm])
    steps = round(plan.duration * CONTROL_HZ) + 5
    actions = np.array([plan.action7((k + 1) / CONTROL_HZ)[0] for k in range(steps)])
    assert plan.worst_ik_mm < 0.5
    assert plan.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER)
    assert np.all(actions[:, :6] <= ik.UPPER)
    assert plan.duration * CONTROL_HZ < limit


def test_demo_tcp_stays_under_reach_ceiling():
    for arm in ("left", "right"):
        for w in TASK.demo()[arm]:
            assert w.pos[2] - M.TOP <= 0.215, (arm, w.label)


def test_parked_arms_are_clear():
    assert M._hands_clear(PARK)
    assert not M._hands_clear(NEAR)
    for x in M.SPARE_X:  # a gripper at an instrument on the tray is on the work area
        assert not M._hands_clear((PARK[0], (M.CXY[0] + x, M.CXY[1], M.TOP + 0.03)))


# 9. scored region visible and clear of the parked arms
def _project(lens, cam_pos, cam_rot, p_world):
    p = cam_rot.T @ (np.asarray(p_world, float) - np.asarray(cam_pos, float))
    return (lens.cx + lens.fx * p[0] / (-p[2]), lens.cy - lens.fy * p[1] / (-p[2]))


def test_scored_regions_visible_and_unoccluded_at_demo_end():
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.top_camera("calibrated")
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))
    lens_low, lens_hi = cam.lens.resized(320, 176), cam.lens.resized(1280, 704)
    cx, cy = M.CXY
    regions = {"cassette": ((cx, cy, M.Z0 + M.BASE_H + M.LID_H), M.CAS_Y),
               "clip": ((cx - M.CAS_X / 2 - M.CLIP_L / 2, cy, M.Z0 + M.BASE_H), M.CLIP_W)}
    final_tcp = {arm: ArmPlan(TASK.demo()[arm], station.BASES[arm]).target(1e9)[0] for arm in ("left", "right")}
    for name, (pt, size) in regions.items():
        u, v = _project(lens_low, cam.pos, rot, pt)
        assert 0.0 <= u < lens_low.width and 0.0 <= v < lens_low.height, (name, u, v)
        u_hi, v_hi = _project(lens_hi, cam.pos, rot, pt)
        for arm, tcp in final_tcp.items():
            tu, tv = _project(lens_hi, cam.pos, rot, tcp)
            reach_px = 0.06 * lens_hi.fx / float(np.linalg.norm(tcp - np.asarray(cam.pos)))
            assert np.hypot(u_hi - tu, v_hi - tv) > reach_px, (name, arm)


def test_cassette_at_least_80_px_at_1280():
    cam = station.top_camera("calibrated")
    lens_hi = cam.lens.resized(1280, 704)
    pt = (*M.CXY, M.Z0 + M.BASE_H)
    assert M.CAS_Y * lens_hi.fx / float(np.linalg.norm(np.asarray(pt) - np.asarray(cam.pos))) >= 80.0


def test_subaction_end_log_records_end_values(capsys):
    M._set_test_trial(tcps=PARK)
    M._log_subaction_end(_state(None, 0.0, -90.0), step=600, run_end=True)
    recs = [json.loads(ln.split("SUBACTION_END", 1)[1]) for ln in capsys.readouterr().out.splitlines()]
    assert [r["name"] for r in recs] == [n for n, _ in M.SUBACTIONS]
    assert [r["end_value"] for r in recs] == [2, 4, 0.0, -90.0]


def test_clip_rests_over_centre_and_hooks_under_the_lip():
    # open: total angle lid + clip stays in (90, 270) deg for every lid angle, so gravity holds it on its stop
    for lid in np.linspace(0.0, M.LID_OPEN, 25):
        assert 90.0 < math.degrees(lid + M.CLIP_UP) < 270.0
    # locked at -90 deg: the hook tab's top is under the lip's underside, and its inner edge reaches under the lip
    xp, zp = M.CLIP_PIVOT
    tab_top = zp - (M.CLIP_L - M.TAB[0]) - 0.0
    lip_bottom = M.BASE_H - 0.004 - M.LIP[2] / 2
    assert tab_top < lip_bottom
    tab_inner = xp + M.CLIP_T / 2 + M.TAB[1]
    assert tab_inner > -M.CAS_X / 2 - M.LIP[0]
    # the scorer's hook geometry agrees: hooked at -90 deg on a flat lid, gap under the lip ~0.9 mm
    cas = lambda lid, clip: _ps((*M.CXY, M.Z0), {"lid": math.radians(lid), "clip": math.radians(clip)})
    x0, x1, z = M._tab_top(cas(0.0, -90.0))
    assert z == pytest.approx(tab_top, abs=1e-6) and x1 == pytest.approx(tab_inner, abs=1e-6)
    assert M._hooked(cas(0.0, -90.0)) and not M._hooked(cas(0.0, 125.0)) and not M._hooked(cas(1.5, -90.0))


def test_latch_lip_is_a_separate_colliding_body():
    # the clip is a link of the cassette articulation (no self-collision): the lip it hooks under must be another body
    keeper = next(o for o in TASK.objects if o.name == "keeper")
    cas = next(o for o in TASK.objects if o.name == "cassette")
    assert keeper.mass is None and len(keeper.boxes) == 1
    assert not any(tuple(b[1]) == M.LIP for b in M._base_boxes())
    assert {j.name for j in cas.joints} >= {"lid", "clip"}


def test_only_the_pre_seated_instruments_follow_the_cassette_under_start_jitter():
    assert M.SEATED_IN == {"inst1": "cassette", "inst3": "cassette", "inst5": "cassette"}  # explorer + mirror: own jitter


def test_every_instrument_is_a_free_body():
    for n in M.NAMES:
        assert M.INSTS[n].mass == pytest.approx(M.INST_M), n


@pytest.mark.parametrize("dy_mm, want", [(0.0, True), (7.5, True), (-7.5, True), (8.5, False), (-12.0, False)])
def test_seated_means_both_handle_ends_on_their_rails(dy_mm, want):
    assert (M._slot_of(_inst_at(M.SLOT_X[1], y=dy_mm / 1000.0)) == 1) is want
    assert M.HANDLE_L / 2 - M.SEAT_TOL_Y >= M.RAIL_Y - M.RAIL_T / 2 + 0.001  # >= 1 mm onto each rail


def test_jaw_narrows_the_mouth_below_the_handle():
    assert M.NOTCH_W - M.JAW_IN < 2 * M.HANDLE_R < M.NOTCH_W
    assert M.JAW_Z - M.JAW_R > M.SEAT_Z + M.HANDLE_R  # the jaw closes back over a seated handle
    assert M.REST_Z > M.SEAT_Z + 0.005  # a handle resting on the mouth stands clearly higher


def test_dimensions_are_real_size():
    assert (M.CAS_X, M.CAS_Y, M.BASE_H + M.LID_H) == (0.076, 0.203, 0.032)  # catalog: 76 x 203 x 32 mm
    assert M.HANDLE_L + 2 * M.TIP_L == pytest.approx(0.165)
    assert M.N_SLOT * M.PITCH < M.CAS_X - 2 * M.WALL  # five notches fit across the 76 mm width
    assert M.SEAT_Z + M.HANDLE_R < M.PAD_Z < M.TOOTH_Z  # pads clear a seated handle, not one on the tooth tops
