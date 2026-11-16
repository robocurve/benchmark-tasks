"""Offline unit, shortcut, reachability and top-camera visibility tests for mortise-latchbolt-handing-flip."""

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
TASK = load(PKG / "mortise_latchbolt_handing_flip.py")
M = TASK.module
PARK = ((0.20, +0.26, M.TOP + 0.20), (0.16, -0.32, M.TOP + 0.20))
NEAR = ((0.20, +0.26, M.TOP + 0.20), (M.CASE_XY[0], M.CASE_XY[1], M.TOP + M.FACE_TOP + 0.03))


def _ps(joints):
    pos = np.array([*M.CASE_XY, M.TOP])
    return M.labkit.PartState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), np.zeros(3), pos, pos, joints)


def _lb(bolt=0.0, turn_deg=0.0, jamb=0.0):
    """Latchbolt state + jamb state (jamb 0 = closed at its stop, -TRAVEL = open)."""
    return {"latchbolt": _ps({"bolt": bolt, "turn": math.radians(turn_deg)}), "jamb": _ps({"close": jamb})}


@pytest.fixture(autouse=True)
def _clean_trial():
    M._reset(None)
    yield
    M._reset(None)


def _solved_trial(**over):
    kw = dict(tcps=PARK)
    kw.update(over)
    M._set_test_trial(**kw)


# 1. loads
def test_task_file_loads_with_station_and_hooks():
    assert TASK.scene == "station"
    assert {o.name for o in TASK.objects} == {"lockcase", "latchbolt", "jamb"}
    assert TASK.score is not None and TASK.success is not None and TASK.demo is not None
    assert M.SCORE_OVER == "end" and M.SUCCESS_HOLD_S == 1.0


# 2. start 0, solved 1
def test_start_state_scores_zero_and_solved_scores_one():
    assert TASK.score(_lb(jamb=-M.TRAVEL)) == 0.0
    _solved_trial()
    assert TASK.score(_lb(0.0, 180.0)) == 1.0
    assert TASK.success(_lb(0.0, 180.0))


# 3. every documented tier reachable, nothing else
def test_documented_partial_score_tiers():
    text = json.loads((PKG / "task.json").read_text())["scoring"]
    documented = sorted(float(v) for v in re.search(r"Possible: ([\d., ]+)\.", text).group(1).split(","))
    seen = set()
    for tcps in (None, PARK, NEAR):
        for bolt in (-0.016, 0.0, 0.0009, 0.0015, M.HEAD_IN, M.PULL_LIMIT):
            for turn in (0.0, 90.0, 171.0, 180.0, 189.0, 195.0, -180.0):
                for jamb in (-M.TRAVEL, -0.042, -0.004, -0.001, 0.0):
                    M._set_test_trial(tcps=tcps)
                    seen.add(TASK.score(_lb(bolt, turn, jamb)))
    assert sorted(seen) == documented == [0.0, 0.4, 0.7, 1.0]
    for tier, (bolt, turn, kw) in {
        0.0: (M.HEAD_IN, 90.0, dict(tcps=PARK)),
        0.4: (0.0, 180.0, dict(tcps=PARK)),
        0.7: (0.0, 180.0, dict(tcps=NEAR)),
    }.items():
        M._reset(None)
        M._set_test_trial(**kw)
        assert TASK.score(_lb(bolt, turn, -M.TRAVEL if tier == 0.4 else 0.0)) == tier


@pytest.mark.parametrize("turn, want", [(170.5, 1.0), (189.5, 1.0), (169.0, 0.0), (191.0, 0.0), (-180.0, 1.0)])
def test_turn_tolerance_is_180_plus_minus_10(turn, want):
    _solved_trial()
    assert TASK.score(_lb(0.0, turn)) == want


@pytest.mark.parametrize("bolt_mm, want", [(0.9, 1.0), (-0.9, 1.0), (1.2, 0.4), (-1.2, 0.4)])
def test_seat_tolerance_is_1_mm_of_the_throw(bolt_mm, want):
    _solved_trial()
    assert TASK.score(_lb(bolt_mm / 1000.0, 180.0)) == want


@pytest.mark.parametrize("jamb_mm, want", [(0.0, 1.0), (-1.1, 1.0), (-2.9, 1.0), (-3.1, 0.4), (-42.0, 0.4), (-90.0, 0.4)])
def test_jamb_must_be_within_3_mm_of_its_stop(jamb_mm, want):
    _solved_trial()
    assert TASK.score(_lb(0.0, 180.0, jamb_mm / 1000.0)) == want


# 4. fail closed
def test_fail_closed_when_tcps_missing():
    _solved_trial(tcps=None)
    assert TASK.score(_lb(0.0, 180.0)) == 0.7
    assert not M._hands_clear(None)


# 5. shortcuts (state level; mj_check SHORTCUTS rolls them out in MuJoCo)
def test_shortcuts_score_low():
    _solved_trial()
    assert TASK.score(_lb(0.0, 0.0)) == 0.0  # nothing / tug-and-release: handing unchanged
    assert TASK.score(_lb(M.PULL_LIMIT, 0.0)) == 0.0  # held pulled out, not turned
    assert TASK.score(_lb(M.HEAD_IN, 90.0)) == 0.0  # half-turn parked on the faceplate
    assert TASK.score(_lb(0.0, 5.0)) == 0.0  # undo: turned back after success
    assert TASK.score(_lb(0.0, 0.0, 0.0)) == 0.0  # jamb closed over an unturned latch (held in, then let out)
    assert TASK.score(_lb(0.0, 180.0, -M.TRAVEL)) == 0.4  # turned, jamb left open
    assert TASK.score(_lb(0.0, 180.0, -0.042)) == 0.4  # turned, jamb not closed
    assert TASK.score(_lb(-0.016, 180.0, 0.0)) == 0.4  # held pushed in under a closed jamb at the end
    _solved_trial(tcps=NEAR)
    assert TASK.score(_lb(0.0, 180.0)) == 0.7  # done, gripper still on the lock


# 6. rehearsal uses real contact
def test_rehearsal_uses_real_contact_no_welds():
    src = (PKG / "mortise_latchbolt_handing_flip.py").read_text()
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


def test_parked_arms_are_clear_and_pushing_hand_is_not():
    assert M._hands_clear(PARK)
    ky = M.CASE_XY[1] + M.PUSH_Y
    z = M.TOP + M.JAMB_Z0 + M.JAMB_H / 2 + M.PAD_TIP_PAST_TCP
    back = M.CASE_XY[0] + M.JAMB_X0 - 0.012
    for x, ky in ((back - M.TRAVEL, ky), (back, ky)):  # pushing hand at the open and at the closed end face
        assert not M._hands_clear((PARK[0], (x, ky, z)))
        assert not M._hands_clear(((x, ky, z), PARK[1]))


# 9. scored region visible and clear of the parked arms
def _project(lens, cam_pos, cam_rot, p_world):
    p = cam_rot.T @ (np.asarray(p_world, float) - np.asarray(cam_pos, float))
    return (lens.cx + lens.fx * p[0] / (-p[2]), lens.cy - lens.fy * p[1] / (-p[2]))


def test_scored_regions_visible_and_unoccluded_at_demo_end():
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.top_camera("calibrated")
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))
    lens_low, lens_hi = cam.lens.resized(320, 176), cam.lens.resized(1280, 704)
    cx, cy = M.CASE_XY
    regions = {
        "latchbolt_head": ((cx, cy, M.TOP + M.FACE_TOP + M.THROW), M.HEAD_L),
        "armored_front": ((cx, cy, M.TOP + M.FACE_TOP), M.FRONT_LEN),
        "jamb_closed": ((cx, cy, M.TOP + M.JAMB_Z0 + M.JAMB_H), M.LIP_X1 - M.JAMB_X0),
    }
    final_tcp = {arm: ArmPlan(TASK.demo()[arm], station.BASES[arm]).target(1e9)[0] for arm in ("left", "right")}
    for name, (pt, size) in regions.items():
        u, v = _project(lens_low, cam.pos, rot, pt)
        assert 0.0 <= u < lens_low.width and 0.0 <= v < lens_low.height, (name, u, v)
        px_hi = size * lens_hi.fx / float(np.linalg.norm(np.asarray(pt) - np.asarray(cam.pos)))
        assert px_hi >= 20.0, (name, px_hi)
        u_hi, v_hi = _project(lens_hi, cam.pos, rot, pt)
        for arm, tcp in final_tcp.items():
            tu, tv = _project(lens_hi, cam.pos, rot, tcp)
            reach_px = 0.06 * lens_hi.fx / float(np.linalg.norm(tcp - np.asarray(cam.pos)))
            assert np.hypot(u_hi - tu, v_hi - tv) > reach_px, (name, arm)


def test_main_object_at_least_80_px_at_1280():
    cam = station.top_camera("calibrated")
    lens_hi = cam.lens.resized(1280, 704)
    pt = (*M.CASE_XY, M.TOP + M.FACE_TOP)
    assert M.FRONT_LEN * lens_hi.fx / float(np.linalg.norm(np.asarray(pt) - np.asarray(cam.pos))) >= 80.0


def test_subaction_end_log_records_end_values(capsys):
    _solved_trial()
    M._trial.update(q_max=0.0125, jamb_q=-0.0011)
    M._log_subaction_end(_lb(0.0, 180.0)["latchbolt"], step=349, run_end=True)
    recs = [json.loads(ln.split("SUBACTION_END", 1)[1]) for ln in capsys.readouterr().out.splitlines()]
    assert [r["name"] for r in recs] == [n for n, _ in M.SUBACTIONS]
    assert all(r["step"] == 349 and r["run_end"] is True for r in recs)
    assert [r["end_value"] for r in recs] == [0.0125, 180.0, 0.0, -0.0011]


def test_dimensions_are_real_size():
    assert (M.CASE_DEPTH, M.CASE_LEN, M.CASE_T) == (0.113, 0.154, 0.025)  # 4-7/16 x 6-1/16 x 1 in
    assert M.THROW == 0.019  # 3/4 in throw
    assert M.HOLE_T - M.HEAD_T == pytest.approx(0.0015) and M.HOLE_L - M.HEAD_L == pytest.approx(0.0015)
    assert M.PULL_LIMIT - M.HEAD_IN >= 0.006  # the tail clears the opening by 6.5 mm before the head turns


def test_strike_geometry_catches_only_the_reversed_bevel():
    # the strike lip runs 3 mm above the faceplate; the bevel starts below it, the square back face reaches it
    assert M.BEVEL_LOW < M.GAP < M.THROW
    # opening wider than the head along X, latch flat face 1 mm from the opening's lip-side edge
    assert M.OPEN_X0 < -M.HEAD_T / 2 and 0.0 < M.OPEN_X1 - M.HEAD_T / 2 <= 0.0015
    # the latch fits the strike pocket at full throw
    assert M.THROW < M.JAMB_H - 0.003 - M.GAP + M.GAP
    # jamb fully open leaves the head and both jaws free: lip edge >= 45 mm behind the head
    assert M.LIP_X1 - M.TRAVEL <= -M.HEAD_T / 2 - 0.045
