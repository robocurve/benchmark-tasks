"""Offline unit, shortcut, reachability, and top-camera visibility tests for clin-drain."""

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
TASK = load(PKG / "clin_drain.py")
M = TASK.module
_SPECS = {o.name: o for o in TASK.objects}


def _quat_from_tilt_x(deg: float) -> np.ndarray:
    a = math.radians(deg) * 0.5
    return np.array([math.sin(a), 0.0, 0.0, math.cos(a)], dtype=float)


def _part(name: str, pos, quat=(0.0, 0.0, 0.0, 1.0), lin_vel=(0.0, 0.0, 0.0), joints=None):
    spec = _SPECS[name]
    lo, hi = M.labkit._extent(spec.boxes, spec.cylinders)
    probe = M.labkit.PartState(
        np.asarray(pos, dtype=float),
        np.asarray(quat, dtype=float),
        np.asarray(lin_vel, dtype=float),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        dict(joints or {}),
    )
    pts = np.array([probe.to_world((x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    return M.labkit.PartState(
        probe.pos,
        probe.quat_xyzw,
        probe.lin_vel,
        probe.ang_vel,
        pts.min(0),
        pts.max(0),
        dict(joints or {}),
    )


def _start_objects():
    dx, dy = M.DRAIN_XY
    hx, hy = M.HOLDER_LOCAL
    return {
        "drain": _part("drain", (dx, dy, M.MAT_TOP), joints={"bellows_mid": 0.0, "top_plate": 0.0}),
        "plug": _part("plug", (dx + hx, dy + hy, M.PLUG_SPAWN_Z)),
    }


def _compressed_objects(q_each: float | None = None):
    dx, dy = M.DRAIN_XY
    hx, hy = M.HOLDER_LOCAL
    qe = -M.STROKE_EACH if q_each is None else q_each
    return {
        "drain": _part("drain", (dx, dy, M.MAT_TOP), joints={"bellows_mid": qe, "top_plate": qe}),
        "plug": _part("plug", (dx + hx, dy + hy, M.PLUG_SPAWN_Z)),
    }


def _solved_objects():
    dx, dy = M.DRAIN_XY
    sx, sy = M.SPOUT_LOCAL
    qe = -M.STROKE_EACH
    h_comp = M.H_EXPANDED + 2.0 * qe
    rim_z = M.MAT_TOP + h_comp + M.SPOUT_H
    return {
        "drain": _part("drain", (dx, dy, M.MAT_TOP), joints={"bellows_mid": qe, "top_plate": qe}),
        "plug": _part("plug", (dx + sx, dy + sy, rim_z - M.PLUG_STEM_H)),
    }


@pytest.fixture(autouse=True)
def _clean_trial():
    M._reset(None)
    yield
    M._reset(None)


def test_task_file_loads_with_station_and_hooks():
    assert TASK.scene == "station"
    assert TASK.score_over == "end"
    assert TASK.success_hold_s == pytest.approx(M.SUCCESS_HOLD_S)
    assert len(TASK.objects) == 3
    assert len(TASK.free_objects) == 2


def test_start_state_scores_zero_and_solved_scores_one():
    start = _start_objects()
    assert TASK.score(start) == pytest.approx(0.0)
    assert TASK.success(start) is False

    M._set_test_trial(
        compressed_ever=True,
        sealed=True,
        sealed_while_expanded=False,
        clear_s=M.CLEAR_REQ_S,
        tcps=((0.20, +0.26, M.MAT_TOP + 0.14), (0.20, -0.26, M.MAT_TOP + 0.14)),
    )
    solved = _solved_objects()
    assert TASK.score(solved) == pytest.approx(1.0)
    assert TASK.success(solved) is True


def test_documented_partial_score_tiers():
    # 0.3: top and bottom brought <= 30% expanded height, plug still unseated
    M._set_test_trial(compressed_ever=True)
    assert TASK.score(_compressed_objects()) == pytest.approx(0.3)
    assert TASK.success(_compressed_objects()) is False

    # 0.3 persists if compressed once on mat then expanded without sealing
    assert TASK.score(_start_objects()) == pytest.approx(0.3)

    # 0.7: plug seated while <= 30% compressed, but gripper still holding / < 5s clear
    solved = _solved_objects()
    M._set_test_trial(
        compressed_ever=True,
        sealed=True,
        clear_s=1.0,
        tcps=((0.34, 0.05, M.MAT_TOP + 0.05), (0.20, -0.26, M.MAT_TOP + 0.14)),
    )
    assert TASK.score(solved) == pytest.approx(0.7)
    assert TASK.success(solved) is False

    # 0.7 when hands are clear but clear_s has not reached CLEAR_REQ_S yet
    M._set_test_trial(
        compressed_ever=True,
        sealed=True,
        clear_s=M.CLEAR_REQ_S - 0.5,
        tcps=((0.20, +0.26, M.MAT_TOP + 0.14), (0.20, -0.26, M.MAT_TOP + 0.14)),
    )
    assert TASK.score(solved) == pytest.approx(0.7)
    assert TASK.success(solved) is False


def test_fail_closed_when_tcps_missing():
    M._set_test_trial(compressed_ever=True, sealed=True, clear_s=M.CLEAR_REQ_S)
    # tcps left as None -> _hands_clear returns False -> cannot achieve 1.0
    assert TASK.score(_solved_objects()) == pytest.approx(0.7)
    assert TASK.success(_solved_objects()) is False


def test_shortcut_and_failure_modes_score_zero_or_blocked():
    # 1. Plugging spout while expanded -> 0.0 (even if later compressed)
    dx, dy = M.DRAIN_XY
    sx, sy = M.SPOUT_LOCAL
    M._set_test_trial(
        compressed_ever=True,
        sealed=True,
        sealed_while_expanded=True,
        clear_s=M.CLEAR_REQ_S + 1.0,
        tcps=((0.20, +0.26, M.MAT_TOP + 0.14), (0.20, -0.26, M.MAT_TOP + 0.14)),
    )
    assert TASK.score(_solved_objects()) == pytest.approx(0.0)
    assert TASK.success(_solved_objects()) is False

    # 2. Reservoir tipped > DRAIN_TILT_MAX_DEG -> 0.0
    M._reset(None)
    M._set_test_trial(compressed_ever=True, sealed=True, clear_s=M.CLEAR_REQ_S)
    tipped = {
        "drain": _part("drain", (dx, dy, M.MAT_TOP + 0.02), quat=_quat_from_tilt_x(45.0),
                       joints={"bellows_mid": -M.STROKE_EACH, "top_plate": -M.STROKE_EACH}),
        "plug": _solved_objects()["plug"],
    }
    assert TASK.score(tipped) == pytest.approx(0.0)

    # 3. Reservoir pushed off mat onto floor -> 0.0
    off_mat = {
        "drain": _part("drain", (dx + 0.35, dy, M.MAT_TOP), joints={"bellows_mid": -M.STROKE_EACH, "top_plate": -M.STROKE_EACH}),
        "plug": _solved_objects()["plug"],
    }
    assert TASK.score(off_mat) == pytest.approx(0.0)

    # 4. Reservoir lifted off mat & dropped -> 0.0
    M._reset(None)
    M._set_test_trial(
        compressed_ever=False,
        lifted_off_mat=True,
        sealed=True,
        clear_s=M.CLEAR_REQ_S,
        tcps=((0.20, +0.26, M.MAT_TOP + 0.14), (0.20, -0.26, M.MAT_TOP + 0.14)),
    )
    assert TASK.score(_solved_objects()) == pytest.approx(0.0)

    # 5. Plug knocked off bench -> 0.0
    M._reset(None)
    M._set_test_trial(compressed_ever=True)
    plug_off = {
        "drain": _compressed_objects()["drain"],
        "plug": _part("plug", (dx + 0.40, dy, M.TOP - 0.10)),
    }
    assert TASK.score(plug_off) == pytest.approx(0.0)

    # 6. Plug inverted on spout or unseated after release -> cannot succeed (0.3)
    M._reset(None)
    M._set_test_trial(
        compressed_ever=True,
        sealed=False,
        clear_s=M.CLEAR_REQ_S,
        tcps=((0.20, +0.26, M.MAT_TOP + 0.14), (0.20, -0.26, M.MAT_TOP + 0.14)),
    )
    inverted = {
        "drain": _compressed_objects()["drain"],
        "plug": _part("plug", (dx + sx, dy + sy, M.MAT_TOP + M.H_COMPRESSED + M.SPOUT_H), quat=_quat_from_tilt_x(180.0)),
    }
    assert TASK.score(inverted) == pytest.approx(0.3)
    assert TASK.success(inverted) is False


def test_rehearsal_uses_real_contact_no_welds():
    src = (PKG / "rehearsal.py").read_text()
    assert "<weld" not in src.lower()
    assert "<equality" not in src.lower()


def test_demo_is_pure_and_open_loop():
    src = inspect.getsource(M.demo)
    for banned in ("_part_state", "labkit.env", "_trial", "score(", "success(", "objects"):
        assert banned not in src, banned
    assert M.demo() == M.demo()


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reachability_and_joint_limits(arm):
    tj = json.loads((PKG / "task.json").read_text())
    m = re.search(r"([\d,]+)", tj["run_limit"])
    limit = int(m.group(1).replace(",", ""))
    plan = ArmPlan(TASK.demo()[arm], station.BASES[arm])
    steps = round(plan.duration * CONTROL_HZ) + 5
    actions = np.array([plan.action7((k + 1) / CONTROL_HZ)[0] for k in range(steps)])
    assert plan.worst_ik_mm < 0.5
    assert plan.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER)
    assert np.all(actions[:, :6] <= ik.UPPER)
    assert plan.duration * CONTROL_HZ < limit


def _project(cam_lens, cam_pos, cam_rot, p_world):
    p = cam_rot.T @ (np.asarray(p_world, float) - np.asarray(cam_pos, float))
    return (cam_lens.cx + cam_lens.fx * p[0] / (-p[2]), cam_lens.cy - cam_lens.fy * p[1] / (-p[2]))


def test_scored_regions_visible_and_unoccluded_at_demo_end():
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.top_camera("calibrated")
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))
    lens_low = cam.lens.resized(320, 176)
    lens_hi = cam.lens.resized(1280, 704)

    dx, dy = M.DRAIN_XY
    sx, sy = M.SPOUT_LOCAL
    regions = {
        "drain_reservoir": ((dx, dy, M.MAT_TOP + M.H_COMPRESSED * 0.5), 2.0 * M.DRAIN_RADIUS),
        "spout_and_plug": ((dx + sx, dy + sy, M.MAT_TOP + M.H_COMPRESSED + M.SPOUT_H), 2.0 * M.PLUG_FLANGE_R),
        "non_slip_mat": ((M.MAT_POS[0], M.MAT_POS[1], M.MAT_TOP), min(M.MAT_SIZE[0], M.MAT_SIZE[1])),
    }
    final_tcp = {arm: ArmPlan(TASK.demo()[arm], station.BASES[arm]).target(1e9)[0] for arm in ("left", "right")}

    for name, (pt, dia) in regions.items():
        u, v = _project(lens_low, cam.pos, rot, pt)
        assert 0.0 <= u < lens_low.width and 0.0 <= v < lens_low.height, (name, u, v)
        px_hi = dia * lens_hi.fx / float(np.linalg.norm(np.asarray(pt) - np.asarray(cam.pos)))
        assert px_hi >= 20.0, (name, px_hi)

        u_hi, v_hi = _project(lens_hi, cam.pos, rot, pt)
        for arm, tcp in final_tcp.items():
            tu, tv = _project(lens_hi, cam.pos, rot, tcp)
            reach_px = 0.06 * lens_hi.fx / float(np.linalg.norm(tcp - np.asarray(cam.pos)))
            assert np.hypot(u_hi - tu, v_hi - tv) > reach_px, (name, arm)


def _seated_at_height(h: float):
    """Plug seated flush on the spout of a reservoir held at total height h (m)."""
    dx, dy = M.DRAIN_XY
    sx, sy = M.SPOUT_LOCAL
    qe = -(M.H_EXPANDED - h) / 2.0
    return {
        "drain": _part("drain", (dx, dy, M.MAT_TOP), joints={"bellows_mid": qe, "top_plate": qe}),
        "plug": _part("plug", (dx + sx, dy + sy, M.MAT_TOP + h + M.SPOUT_H - M.PLUG_STEM_H)),
    }


@pytest.mark.parametrize("h_mm, want", [(20.0, 1.0), (22.5, 1.0), (23.5, 0.7), (25.0, 0.7), (30.0, 0.7)])
def test_full_score_needs_reservoir_still_flat_after_release(h_mm, want):
    # "stays compressed flat after you let go": 1.0 only while the reservoir is still <= 22.8 mm (30%)
    M._set_test_trial(
        compressed_ever=True,
        sealed=True,
        clear_s=M.CLEAR_REQ_S,
        tcps=((0.20, +0.26, M.MAT_TOP + 0.14), (0.20, -0.26, M.MAT_TOP + 0.14)),
    )
    assert TASK.score(_seated_at_height(h_mm / 1000.0)) == pytest.approx(want)


def test_subaction_end_log_records_joint_and_plug_end_values(capsys):
    # the official run log keeps each sub-action's end value (observe-only print, read by the end-state check)
    parts = _seated_at_height(0.020)
    M._log_subaction_end(parts["drain"], parts["plug"], step=290, run_end=True)
    recs = [json.loads(ln.split("SUBACTION_END", 1)[1]) for ln in capsys.readouterr().out.splitlines() if "SUBACTION_END" in ln]
    by_key = {r["joint_or_body"]: r for r in recs}
    assert set(by_key) == {"drain.bellows_mid", "drain.top_plate", "plug"} and all(r["name"] for r in recs)
    assert all(r["step"] == 290 and r["run_end"] is True for r in recs)
    assert by_key["drain.bellows_mid"]["end_value"] == pytest.approx(-0.028, abs=1e-5)
    assert by_key["drain.top_plate"]["end_value"] == pytest.approx(-0.028, abs=1e-5)
    assert by_key["plug"]["end_value"] == pytest.approx(list(parts["plug"].pos), abs=1e-5)
