"""Offline unit, shortcut, reachability and top-camera visibility tests for gondola-divider-planogram-respace."""

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
TASK = load(PKG / "gondola_divider_planogram_respace.py")
M = TASK.module
PARK = ((0.16, +0.30, M.TOP + 0.20), (0.16, -0.30, M.TOP + 0.20))
NEAR = ((0.16, +0.30, M.TOP + 0.20), (M.ANCHOR[0], -0.076, M.TOP + 0.12))
QUAT0 = np.array([0.0, 0.0, 0.0, 1.0])


def _div(name, dk=0, lift=0.0, dx=0.0, dy=0.0, roll_deg=0.0, yaw_deg=0.0):
    """A divider state moved dk holes outward from its start hole (plus offsets), as its joint chain reports it."""
    k0 = M.K_START if name == "divider_l" else -M.K_START
    side = 1 if k0 > 0 else -1
    pos = np.array([0.0, k0 * M.PITCH, M.TOP + M.SHELF_Z])
    joints = {"tx": dx, "ty": side * dk * M.PITCH + dy, "tz": lift, "yaw": math.radians(yaw_deg),
              "roll": math.radians(roll_deg), "pitch": 0.0}
    return M.labkit.PartState(pos, QUAT0, np.zeros(3), np.zeros(3), pos, pos, joints)


def _glove(side, tilt_deg=0.0, dy=0.0):
    pos = np.array([M.GLOVE_X, side * M.GLOVE_Y + dy, M.TOP + M.SHELF_Z + M.GLOVE[2] / 2])
    a = math.radians(tilt_deg) / 2
    q = np.array([math.sin(a), 0.0, 0.0, math.cos(a)])
    return M.labkit.PartState(pos, q, np.zeros(3), np.zeros(3), pos - 0.05, pos + 0.05, {})


def _objs(l=None, r=None, gl=None, gr=None):
    return {"divider_l": l or _div("divider_l"), "divider_r": r or _div("divider_r"),
            "glove_l": gl or _glove(+1), "glove_r": gr or _glove(-1)}


SOLVED = dict(l=_div("divider_l", 1), r=_div("divider_r", 1))


@pytest.fixture(autouse=True)
def _clean_trial():
    M._reset(None)
    yield
    M._reset(None)


# 1. loads
def test_task_file_loads_with_station_and_hooks():
    assert TASK.scene == "station"
    assert {o.name for o in TASK.objects} == {"shelf", "divider_l", "divider_r", "glove_l", "glove_r"}
    assert TASK.score is not None and TASK.success is not None and TASK.demo is not None
    assert M.SCORE_OVER == "end" and M.SUCCESS_HOLD_S == 1.0


# 2. start 0, solved 1
def test_start_state_scores_zero_and_solved_scores_one():
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_objs()) == 0.0
    assert TASK.score(_objs(**SOLVED)) == 1.0
    assert TASK.success(_objs(**SOLVED))


def test_seated_start_geometry_foot_in_hole_and_hook_over_wire():
    st = _div("divider_l")
    assert M.foot_ok(st, M.K_START) and M.hook_ok(st, M.K_START)
    assert not M.foot_ok(st, M.K_TARGET) and not M.hook_ok(st, M.K_TARGET)


# 3. every documented tier reachable, nothing else
def test_documented_partial_score_tiers():
    text = json.loads((PKG / "task.json").read_text())["scoring"]
    documented = sorted(float(v) for v in re.search(r"Possible: ([\d., ]+)\.", text).group(1).split(","))
    variants = lambda n: [_div(n), _div(n, 1), _div(n, 1, lift=0.0095), _div(n, 1, lift=0.012), _div(n, 2)]
    seen = set()
    for tcps in (None, PARK, NEAR):
        for l in variants("divider_l"):
            for r in variants("divider_r"):
                for gl in (_glove(+1), _glove(+1, 80.0)):
                    M._set_test_trial(tcps=tcps)
                    seen.add(TASK.score(_objs(l, r, gl)))
    assert sorted(seen) == documented == [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]


def test_foot_and_hook_scored_separately():
    M._set_test_trial(tcps=PARK)
    # hook over the wire at +3 but the foot resting on the deck beside the hole (lifted 10 mm, i.e. on the rim)
    assert M.hook_ok(_div("divider_l", 1, lift=0.0095), M.K_TARGET)
    assert not M.foot_ok(_div("divider_l", 1, lift=0.0095), M.K_TARGET)
    assert TASK.score(_objs(l=_div("divider_l", 1, lift=0.0095), r=_div("divider_r", 1))) == 0.6
    # foot in the hole, hook pulled back off the wire (frame slid 4 mm toward the rear is impossible with the foot in
    # the hole, so test the hook term alone: hook leg behind the wire)
    assert not M.hook_ok(_div("divider_l", 1, dx=0.012), M.K_TARGET)


@pytest.mark.parametrize("dy_mm, foot", [(1.1, True), (-1.1, True), (3.0, True), (3.4, False), (-3.4, False)])
def test_foot_tolerance_is_the_hole_radius(dy_mm, foot):
    assert M.foot_ok(_div("divider_l", 1, dy=dy_mm / 1000), M.K_TARGET) is foot


def test_wrong_hole_scores_zero_for_that_divider():
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_objs(l=_div("divider_l", 2), r=_div("divider_r", 1))) == 0.4  # left two holes out: 0
    assert TASK.score(_objs(l=_div("divider_l", -1), r=_div("divider_r", 1))) == 0.4  # left moved inward


def test_knocked_glove_box_zeroes_its_lane():
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_objs(gl=_glove(+1, 85.0), **SOLVED)) == 0.4
    assert TASK.score(_objs(gr=_glove(-1, 0.0, dy=0.12), **SOLVED)) == 0.4  # pushed into the middle lane
    assert TASK.score(_objs(gl=_glove(+1, 9.0), **SOLVED)) == 1.0


def test_fallen_divider_scores_zero():
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_objs(l=_div("divider_l", 1, roll_deg=80.0), r=_div("divider_r", 1))) == 0.4


# 4. fail closed
def test_fail_closed_when_tcps_missing():
    M._set_test_trial(tcps=None)
    assert TASK.score(_objs(**SOLVED)) == 0.8
    assert not M._hands_clear(None)


# 5. shortcuts (state level; mj_check SHORTCUTS rolls them out in MuJoCo)
def test_shortcuts_score_low():
    M._set_test_trial(tcps=PARK)
    assert TASK.score(_objs()) == 0.0  # nothing
    assert TASK.score(_objs(l=_div("divider_l", 1, lift=0.025), r=_div("divider_r", 1, lift=0.025))) == 0.0  # held up
    assert TASK.score(_objs(l=_div("divider_l", 1), r=_div("divider_r"))) == 0.4  # half done
    assert TASK.score(_objs(l=_div("divider_l", 0), r=_div("divider_r", 1))) == 0.4  # undo one after success
    M._set_test_trial(tcps=NEAR)
    assert TASK.score(_objs(**SOLVED)) == 0.8  # done, a gripper still over the dividers


# 6. real contact only
def test_task_uses_real_contact_no_welds():
    src = (PKG / "gondola_divider_planogram_respace.py").read_text()
    for banned in ("weld", "attach", "set_joint_position(", "write_joint_state", "write_root", "set_joint_position_target"):
        assert banned not in src, banned


def test_divider_chain_is_unpowered():
    for d in (M.DIVIDER_L, M.DIVIDER_R):
        assert [j.name for j in d.joints] == ["tx", "ty", "tz", "yaw", "roll", "pitch"]
        for j in d.joints:
            assert j.stiffness == 0.0 and j.friction == 0.0 and j.initial == 0.0  # damping only: no spring, no hold


# 7. demo open loop
def test_demo_is_pure_and_open_loop():
    src = inspect.getsource(M.demo) + inspect.getsource(M._move)
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


def test_demo_tcp_stays_under_reach_ceiling_and_yaw_zero():
    for arm in ("left", "right"):
        for w in TASK.demo()[arm]:
            assert w.pos[2] - M.TOP <= 0.215, w.label
            assert w.yaw == 0.0, w.label


def test_demo_ends_with_both_arms_parked_for_1_5_s():
    plan = TASK.demo()
    for arm in ("left", "right"):
        tail = 0.0
        for w in reversed(plan[arm]):
            if math.dist(w.pos, plan[arm][-1].pos) > 1e-9:
                break
            tail += w.duration
        assert tail >= 1.5, (arm, tail)


# 9. scored region visible and clear of the parked arms (occlusion test in the top camera)
def _project(lens, cam_pos, cam_rot, p_world):
    p = cam_rot.T @ (np.asarray(p_world, float) - np.asarray(cam_pos, float))
    return (lens.cx + lens.fx * p[0] / (-p[2]), lens.cy - lens.fy * p[1] / (-p[2]))


def _regions():
    deck = M.TOP + M.SHELF_Z
    out = {}
    for name, k in (("left", M.K_TARGET), ("right", -M.K_TARGET)):
        y = k * M.PITCH
        out[f"{name}_foot_hole"] = ((M.X_HOLE, y, deck), 0.00635)
        out[f"{name}_hook"] = ((M.X_HOOK, y, deck + M.RAIL_Z), 0.012)
        out[f"{name}_divider_mid"] = ((M.ANCHOR[0], y, deck + M.RAIL_Z), M.X_HOLE - M.X_HOOK)
    for s, n in ((1, "glove_l"), (-1, "glove_r")):
        out[n] = ((M.GLOVE_X, s * M.GLOVE_Y, deck + M.GLOVE[2]), M.GLOVE[0])
    return out


def test_scored_regions_visible_and_unoccluded_at_demo_end():
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.top_camera("calibrated")
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))
    lens_low, lens_hi = cam.lens.resized(320, 176), cam.lens.resized(1280, 704)
    final_tcp = {arm: ArmPlan(TASK.demo()[arm], station.BASES[arm]).target(1e9)[0] for arm in ("left", "right")}
    for name, (pt, _size) in _regions().items():
        u, v = _project(lens_low, cam.pos, rot, pt)
        assert 0.0 <= u < lens_low.width and 0.0 <= v < lens_low.height, (name, u, v)
        u_hi, v_hi = _project(lens_hi, cam.pos, rot, pt)
        for arm, tcp in final_tcp.items():
            tu, tv = _project(lens_hi, cam.pos, rot, tcp)
            reach_px = 0.08 * lens_hi.fx / float(np.linalg.norm(tcp - np.asarray(cam.pos)))
            assert np.hypot(u_hi - tu, v_hi - tv) > reach_px, (name, arm)


def test_main_object_at_least_80_px_at_1280():
    cam = station.top_camera("calibrated")
    lens_hi = cam.lens.resized(1280, 704)
    pt, size = _regions()["left_divider_mid"]
    assert size * lens_hi.fx / float(np.linalg.norm(np.asarray(pt) - np.asarray(cam.pos))) >= 80.0


def test_subaction_end_log_records_end_values(capsys):
    M._log_subaction_end(_objs(**SOLVED), step=330, run_end=True)
    recs = [json.loads(ln.split("SUBACTION_END", 1)[1]) for ln in capsys.readouterr().out.splitlines()]
    assert [r["name"] for r in recs] == [n for n, _ in M.SUBACTIONS]
    assert all(r["step"] == 330 and r["run_end"] is True for r in recs)
    foot_l = recs[0]["end_value"]
    assert foot_l[1] == pytest.approx(M.K_TARGET * M.PITCH, abs=1e-5)
    assert foot_l[2] == pytest.approx(M.TOP + M.SHELF_Z - M.FOOT, abs=1e-5)


def test_dimensions_are_real_size():
    assert M.PITCH == 0.0254 and M.HOLE_HALF * 2 == pytest.approx(0.00635)  # 1/4 in holes on 1 in centres
    assert M.FENCE_H == 0.0762  # 3 in fencing
    assert M.X_HOLE - M.X_FENCE == pytest.approx(0.260)  # divider 2 3/4 in shorter than the 13 in shelf
    assert M.X_BACK - M.X_FRONT == pytest.approx(0.330) and 2 * M.SHELF_HALF_LEN == pytest.approx(0.914)
    # lanes: 8-4-8 at the start, 7-6-7 on the planogram
    assert (M.K_OUTER - M.K_START, 2 * M.K_START) == (8, 4)
    assert (M.K_OUTER - M.K_TARGET, 2 * M.K_TARGET) == (7, 6)
