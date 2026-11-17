"""Offline tests for qc_met10: scoring tiers, shortcuts, constants vs task.json, reach, demo purity, top-camera
occlusion of every scored region at the final arm poses."""

from __future__ import annotations

import ast
import inspect
import json
import math
import pathlib
import re
import sys

import numpy as np
import pytest
from yam_arena import ik, station
from yam_arena.constants import CONTROL_HZ
from yam_arena.demo import ArmPlan
from yam_arena.urdf import quat_xyzw_to_matrix

PKG = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(PKG))

import qc_met10 as task  # noqa: E402
from labkit import PartState  # noqa: E402

CLEAR_TCPS = (np.array([0.180, 0.270, task.TOP + 0.200]), np.array([0.180, -0.270, task.TOP + 0.200]))
SEATED = (task.STAND_XY[0], task.STAND_XY[1], task.AXIS_WORLD[2])


def _quat(yaw_deg=0.0, pitch_deg=0.0):
    """Rotation about z by yaw, then about x by pitch (tilts the long y axis up), as xyzw."""
    y, p = math.radians(yaw_deg) / 2.0, math.radians(pitch_deg) / 2.0
    qz = np.array([0.0, 0.0, math.sin(y), math.cos(y)])
    qx = np.array([math.sin(p), 0.0, 0.0, math.cos(p)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def _jaw(s, gap):
    o = np.array([*task.STAND_XY, task.TOP])
    return PartState(pos=o, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=o - 0.1, high=o + 0.1, joints={f"gap_{s}": gap})


def _objs(pos=SEATED, yaw=0.0, pitch=0.0, lever_l=0.0, lever_r=0.0, tcps=CLEAR_TCPS, xh=task.XH_BACK):
    task._set_test_trial(tcps=tcps)
    p = np.array(pos, float)
    dog = PartState(pos=p, quat_xyzw=_quat(yaw, pitch), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                    low=p - 0.09, high=p + 0.09, joints={})
    s = np.array([*task.STAND_XY, task.TOP])
    stand = PartState(pos=s, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                      low=s - 0.2, high=s + 0.2, joints={"lever_l": lever_l, "lever_r": lever_r, "crosshead": xh})
    return {"dogbone": dog, "test_stand": stand, "upper_jaw_l": _jaw("l", task.SPEC_T),
            "upper_jaw_r": _jaw("r", task.SPEC_T)}


def test_task_json_limits_and_constants():
    tj = json.loads((PKG / "task.json").read_text())
    for k, n in {"title": 120, "instruction": 1000, "context": 1000, "setup": 4000, "fidelity": 500,
                 "scores": 300, "run_limit": 200}.items():
        assert len(tj[k]) <= n
    assert len(", ".join(tj["keywords"])) <= 200
    assert tj["category"] == "Other"
    sc = tj["scoring"]
    # the scoring text quotes the same named thresholds score() uses (no stale constants)
    assert "17 mm ruler mark" in sc and task.DEPTH_MIN == pytest.approx(0.017)
    assert "inside the slot" in sc and task.SEAT_Z_TOL == pytest.approx(0.003)
    assert "overhangs its jaw by at most 4 mm" in sc and task.SEAT_X_TOL == pytest.approx(0.004)
    assert "2 mm gap" in sc and task.STOP_GAP == pytest.approx(0.002)
    assert "within 2 mm of its forward stop" in sc and task.LEVER_FWD_MAX == pytest.approx(0.002)
    assert "within 2 deg" in sc and task.SKEW_MAX_DEG == 2.0
    assert "115 +- 5 mm" in sc and (task.GAUGE_L, task.GAUGE_TOL) == (0.115, 0.005)
    assert "5 cm" in sc and task.HAND_CLEAR_M == pytest.approx(0.05)
    assert "held 3 s" in sc and task.SUCCESS_HOLD_S == 3.0
    assert "Possible scores " + ", ".join(f"{v:g}" for v in _possible_scores()) + "." in sc
    assert "jaw gap at most 4.7 mm" in sc and task.JAW_ON_TAB_MAX == pytest.approx(0.0047)
    for name in ("tension", "gauge reads", "RIGHT", "preload"):
        assert name not in sc
    credits = (task.SEAT_CREDIT, task.LEVER_CREDIT, task.GAP_CREDIT) * 2 + (task.STRAIGHT_CREDIT, task.GAUGE_CREDIT,
                                                                          task.CLEAR_CREDIT)
    assert sum(credits) == pytest.approx(1.0)
    for c in (task.SEAT_CREDIT, task.LEVER_CREDIT, task.GAP_CREDIT, task.STRAIGHT_CREDIT, task.GAUGE_CREDIT,
              task.CLEAR_CREDIT):
        assert f"+{c:.2f}" in sc
    assert "115.0 mm" in tj["setup"] and 2 * task.GRIP_FACE == pytest.approx(0.115)
    assert "backed off 35 mm" in tj["setup"] and task.XH_BACK == pytest.approx(0.035)
    assert "500 mm/min" in tj["setup"] and task.JOG_SPEED == pytest.approx(0.5 / 60.0)
    assert "6.4 mm" in tj["setup"] and task.JAW_OPEN == pytest.approx(0.0064)
    assert "6.2 mm" in tj["setup"] and task.GAP_LIFTED == pytest.approx(0.0062)
    assert "19.1 mm" in tj["setup"] and task.JAW_W == pytest.approx(0.0191)


def _possible_scores():
    """Every score the table can give, by enumerating its row outcomes the way components() gates them (oracle for
    the "Possible scores" list in task.json and the README)."""
    import itertools
    grip = {0.0}
    for seat, lever, gap in itertools.product((0, 1), repeat=3):
        if seat:
            grip.add(round(task.SEAT_CREDIT + lever * task.LEVER_CREDIT + gap * task.GAP_CREDIT, 6))
    out = set()
    for a, b in itertools.product(grip, grip):
        out.add(round(a + b, 6))
        if a >= task.SEAT_CREDIT + task.LEVER_CREDIT - 1e-9 and b >= task.SEAT_CREDIT + task.LEVER_CREDIT - 1e-9:
            both_gaps_fail = a + b < 2 * (task.SEAT_CREDIT + task.LEVER_CREDIT) + task.GAP_CREDIT - 1e-9
            for rows in itertools.product((0, 1), repeat=3):
                if both_gaps_fail and rows[1]:  # geometry: see test_both_gaps_failing_excludes_the_gauge_row
                    continue
                extra = sum(r * c for r, c in zip(rows, (task.STRAIGHT_CREDIT, task.GAUGE_CREDIT, task.CLEAR_CREDIT)))
                out.add(round(a + b + extra, 6))
    return sorted(out)


def test_both_gaps_failing_excludes_the_gauge_row():
    """Both tab ends within STOP_GAP of their slot ends = more than 2 x 31 mm of the 165 mm specimen inside the grips,
    so the jaw faces are under 103 mm apart: the 115 +- 5 mm row cannot hold (why 0.9 is not a possible score)."""
    max_gauge = 2 * task.SPEC_HALF - 2 * (task.STOP_Y - task.GRIP_FACE - task.STOP_GAP)
    assert max_gauge < task.GAUGE_L - task.GAUGE_TOL
    hits = set()
    for dy in np.arange(-0.015, 0.0151, 0.0005):
        for xh in np.arange(0.0, task.XH_MAX + 1e-9, 0.0005):
            c = task.components(_objs(pos=(SEATED[0], dy, SEATED[2]), xh=xh))
            hits.add(round(c["score"], 6))
            assert not ({"lever_a", "lever_b", "gauge"} <= set(c["passed"]) and not {"gap_a", "gap_b"} & set(c["passed"]))
    assert 0.9 not in hits and {0.95, 1.0} <= hits


def test_intended_solution_scores_one():
    objs = _objs()
    assert task.score(objs) == pytest.approx(1.0)
    assert task.success(objs)
    assert task.distance(objs) == pytest.approx(0.0, abs=1e-6)
    assert task.components(objs)["passed"] == ["seat_a", "lever_a", "gap_a", "seat_b", "lever_b", "gap_b",
                                               "straight", "gauge", "clear"]


def test_hold_is_success_only():
    """The 3 s hold is not scored: score() is the end state's sum (1.0 reachable at once), success() is the full end
    state and the runtime ends the episode only after it has held SUCCESS_HOLD_S."""
    import inspect as _inspect
    from yam_arena import task_file
    assert task.SUCCESS_HOLD_S == 3.0 and task.SCORE_OVER == "end"
    task._reset(None)
    assert task._trial["hold_s"] == 0.0 and task.score(_objs()) == pytest.approx(1.0)
    assert "SUCCESS_HOLD_S" in _inspect.getsource(task_file)  # the runtime reads the task's hold
    for name in ("HOLD_CAP", "HOLD_CREDIT", "PRELOAD_N", "TUG_MIN_N", "tension_n"):
        assert not hasattr(task, name)


@pytest.mark.parametrize("kw,expect", [
    (dict(lever_r=task.LEVER_OPEN), 0.40),                          # grip B still held open: B seated, gap only
    (dict(lever_l=task.LEVER_OPEN), 0.40),                          # grip A held open: B's rows count on their own
    (dict(lever_l=task.LEVER_OPEN, lever_r=task.LEVER_OPEN), 0.30),  # slid in, both grips left open
    (dict(lever_l=0.003), 0.40),                                    # lever A 3 mm short of forward (jaw held up)
    (dict(yaw=1.5), 1.0),                                           # 1.5 deg: inside the protractor tolerance
    (dict(yaw=2.5), 0.85),                                          # 2.5 deg skew: clamped but not straight
    (dict(yaw=4.0), 0.0),                                           # 4 deg: tab ends 5.8 mm off the 19.1 mm jaws
    (dict(pos=(SEATED[0], 0.0045, SEATED[2])), 1.0),                # 4.5 mm off centre: both ends >= 2 mm off
    (dict(pos=(SEATED[0], 0.0075, SEATED[2])), 0.95),               # +y tab end on its slot end
    (dict(pos=(SEATED[0], -0.0075, SEATED[2])), 0.95),              # -y tab end on its slot end
    (dict(xh=task.XH_BACK - 0.007), 0.80),                          # crosshead 7 mm short: gauge length 122 mm
    (dict(xh=task.XH_BACK + 0.0055), 0.80),                         # crosshead 5.5 mm past: gauge 109.5 mm
    (dict(tcps=(np.array(task.PADDLE_WORLD[+1]), CLEAR_TCPS[1])), 0.85),  # gripper still on a lever
    (dict(tcps=(np.array(task.KEY_WORLD["right"]), CLEAR_TCPS[1])), 0.85),  # gripper still on a jog key
])
def test_partial_credit_tiers(kw, expect):
    objs = _objs(**kw)
    assert task.score(objs) == pytest.approx(expect)
    assert task.success(objs) == (expect == 1.0)
    assert round(task.score(objs), 6) in _possible_scores()


def test_minus_y_tab_earns_its_rows_without_the_plus_y_tab():
    """No chaining across the grips: the -y tab clamped in grip B with the +y tab off its jaw keeps B's 0.25."""
    objs = _objs(pos=(SEATED[0], -0.012, SEATED[2]), xh=task.XH_BACK - 0.012)  # +y end 7 mm short of 17 mm
    c = task.components(objs)
    assert c["passed"] == ["seat_b", "lever_b", "gap_b"] and task.score(objs) == pytest.approx(0.25)


def test_end_for_end_mount_scores_full():
    """The dogbone is symmetric: mounted turned 180 deg end-for-end it earns the same rows as in its tray orientation."""
    assert task.score(_objs(yaw=180.0)) == pytest.approx(task.score(_objs())) == pytest.approx(1.0)
    assert task.score(_objs(yaw=180.0, lever_r=task.LEVER_OPEN)) == pytest.approx(task.score(_objs(lever_r=task.LEVER_OPEN)))


@pytest.mark.parametrize("pos", [
    (0.180, 0.0, task.TOP + task.REST_H + task.SPEC_T / 2),         # never moved
    (SEATED[0], SEATED[1], SEATED[2] + task.JAW_TOP - task.FLOOR_Z),  # lying on top of the closed jaws
    (SEATED[0], 0.035, SEATED[2]),                                  # one end only (other tab short of its jaw)
    (SEATED[0], SEATED[1], SEATED[2] + 0.005),                      # held 5 mm up (riding a jaw edge)
    (SEATED[0] - 0.012, SEATED[1], SEATED[2]),                      # hanging off the front of the jaws
    (0.30, 0.0, task.TOP + 0.003),                                  # knocked off onto the mat
])
def test_shortcuts_score_zero(pos):
    assert task.score(_objs(pos=pos)) == 0.0


@pytest.mark.parametrize("xh", [
    0.0,     # crosshead never jogged: grip B still 10 mm short of the -y tab end
    0.020,   # grip B let go before the tab was in: its closed jaw stops the crosshead with the tab 10 mm in
])
def test_first_grip_only_earns_grip_a_rows(xh):
    a = task.SEAT_CREDIT + task.LEVER_CREDIT + task.GAP_CREDIT
    assert task.score(_objs(xh=xh)) == pytest.approx(a)
    assert task.score(_objs(xh=xh, lever_l=task.LEVER_OPEN)) == pytest.approx(task.SEAT_CREDIT + task.GAP_CREDIT)


def test_jogging_an_empty_stand_scores_zero():
    """Crosshead jogged to the 115 mm gauge length and arms clear, but the specimen still on the tray: 0."""
    tray = (0.180, 0.0, task.TOP + task.REST_H + task.SPEC_T / 2)
    assert task.score(_objs(pos=tray)) == 0.0


def test_missing_crosshead_reads_backed_off():
    objs = _objs()
    objs["test_stand"].joints.pop("crosshead")
    assert task.score(objs) == pytest.approx(task.SEAT_CREDIT + task.LEVER_CREDIT + task.GAP_CREDIT)


def test_undo_after_success_loses_the_credit():
    assert task.score(_objs()) == pytest.approx(1.0)
    objs = _objs(lever_l=task.LEVER_OPEN)  # lever A opened again afterwards
    assert task.score(objs) == pytest.approx(0.40) and not task.success(objs)


def test_missing_lever_joint_reads_held_open():
    objs = _objs()
    objs["test_stand"].joints.pop("lever_r")
    assert task.score(objs) == pytest.approx(0.40)


def test_real_grip_opening_not_enlarged():
    """The jaw gap never exceeds the maker's 0-6.4 mm opening; the 19 mm tab only goes in through the open side."""
    assert task.GAP_LIFTED <= task.JAW_OPEN <= 0.0064 + 1e-9
    assert task.GAP_LIFTED - task.SPEC_T >= 0.0025  # >= 2.5 mm free height for the 3.2 mm tab while it slides in
    for jaw in (task.JAW_L.joints[0], task.JAW_R.joints[1]):
        assert (jaw.kind, jaw.axis, jaw.upper, jaw.stiffness > 0) == ("slide", "Z", task.JAW_OPEN, True)
    assert all(j.stiffness > 0 for j in task.STAND.joints if j.name.startswith("lever"))  # spring-return levers
    assert task.HOUSING_D == pytest.approx(0.0381) and task.HOUSING_A == pytest.approx(0.0495)
    assert task.HOUSING_B == pytest.approx(0.0622)


def test_declared_clamp_and_lever_forces():
    """Declared springs (README "Mechanism"): the jaw spring bites the 3.2 mm tab with 1.9 N; held open, the lever and
    jaw springs stay finger-sized (< 15 N)."""
    assert task.JAW_K * task.SPEC_T == pytest.approx(1.92)
    assert task.LEVER_K * task.LEVER_OPEN < 15.0 and task.JAW_K * task.GAP_LIFTED < 15.0


def test_right_key_is_a_plain_momentary_jog(monkeypatch):
    """No automated stand behaviour: the on_step hook only moves the crosshead target at JOG_SPEED while a key is
    held (LEFT +y, RIGHT -y; released, it stays), clamped to the travel; no load holding or force readout."""
    src = inspect.getsource(task._watch)
    assert "tension" not in src and "load_hold" not in src
    sent, keys = [], {}
    objs = _objs()
    objs["control_panel"] = PartState(pos=np.zeros(3), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3),
                                      ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), joints=keys)
    monkeypatch.setattr(task.labkit, "_SPECS", {n: n for n in objs})
    monkeypatch.setattr(task.labkit, "_part_state", lambda env, spec: objs[spec])
    monkeypatch.setattr(task, "_arm_tcps", lambda env: None)
    monkeypatch.setattr(task, "_drive_crosshead", lambda env, q: sent.append(q))
    env = type("Env", (), {"unwrapped": type("U", (), {"step_dt": 0.05})()})()
    task._reset(None)
    step = task.JOG_SPEED * 0.05
    for held, n, want in (({"key_left": -0.002}, 10, 10 * step), ({}, 5, 10 * step),
                          ({"key_right": -0.002}, 4, 6 * step), ({"key_right": -0.002}, 40, 0.0),
                          ({"key_left": -0.002}, 200, task.XH_MAX), ({"key_left": -0.001}, 3, task.XH_MAX)):
        keys.clear()
        keys.update(held)
        for _ in range(n):
            task._watch(env)
        assert sent[-1] == pytest.approx(want), held
    task._reset(None)
    keys.clear()
    keys["key_left"] = -0.001  # pressed less than KEY_TRIP (1.2 mm): no motion
    task._watch(env)
    assert sent[-1] == 0.0


def test_subaction_end_log_records_end_values(capsys):
    task._reset(None)
    objs = _objs()
    task._log_subaction_end(objs, step=470, run_end=True)
    recs = [json.loads(ln.split("SUBACTION_END", 1)[1]) for ln in capsys.readouterr().out.splitlines()
            if "SUBACTION_END" in ln]
    by_key = {r["joint_or_body"]: r for r in recs}
    assert set(by_key) == {k for _, k in task.SUBACTIONS} and all(r["name"] for r in recs)
    assert all(r["step"] == 470 and r["run_end"] is True for r in recs)
    assert by_key["dogbone.tab_+y"]["end_value"]["end"][1] == pytest.approx(task.SPEC_HALF, abs=1e-6)
    assert by_key["test_stand.lever_l+upper_jaw_l.gap_l"]["end_value"]["gap_l"] == pytest.approx(task.SPEC_T)
    assert by_key["test_stand.crosshead+dogbone.tab_-y"]["end_value"]["crosshead"] == pytest.approx(task.XH_BACK)
    assert by_key["dogbone+gauge"]["end_value"]["yaw_deg"] == pytest.approx(0.0, abs=1e-6)
    assert by_key["dogbone+gauge"]["end_value"]["gauge_m"] == pytest.approx(task.GAUGE_L, abs=1e-6)
    assert by_key["dogbone+gauge"]["end_value"]["stop_gap_m"] == pytest.approx([0.008, 0.008], abs=1e-6)


def test_missing_tcps_fail_closed():
    objs = _objs(tcps=None)
    assert task.score(objs) == pytest.approx(0.85)


def _plans():
    return {arm: ArmPlan(wps, station.BASES[arm]) for arm, wps in task.demo().items()}


def test_demo_reach_limits_and_run_limit():
    tj = json.loads((PKG / "task.json").read_text())
    limit = int(tj["run_limit"].split(" ")[0].replace(",", ""))
    for arm, p in _plans().items():
        steps = round(p.duration * CONTROL_HZ) + 5
        acts = np.array([p.action7((k + 1) / CONTROL_HZ)[0] for k in range(steps)])
        assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, arm
        assert np.all(acts[:, :6] >= ik.LOWER) and np.all(acts[:, :6] <= ik.UPPER), arm
        assert p.duration * 20.0 < limit


def test_demo_is_pure_and_smooth():
    tree = ast.parse(inspect.getsource(task.demo))
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    assert not names & {"_trial", "env", "objects", "labkit"}
    for arm, wps in task.demo().items():
        for a, b in zip(wps, wps[1:]):
            same = np.allclose(a.pos, b.pos) and a.finger == b.finger and a.yaw == b.yaw
            assert not same, f"{arm}: idle hover at '{b.label}'"
            if a.finger != b.finger:
                assert b.duration >= 0.6  # finger stroke under the runtime's per-step rate limit


def _project(pts):
    cam = station.top_camera("calibrated")
    lens = cam.lens.resized(320, 176)
    rot = quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw, float))
    pc = (np.asarray(pts, float) - np.asarray(cam.pos, float)) @ rot
    z = -pc[:, 2]
    return np.c_[lens.cx + lens.fx * pc[:, 0] / z, lens.cy - lens.fy * pc[:, 1] / z], z


def test_top_camera_scored_regions_visible_and_unoccluded_at_end():
    """Project every scored region into the top camera (320x176) and check the final arm poses occlude none."""
    x, y, z = SEATED
    regions = {
        "tab_l_in_grip": np.array([x, y + task.GRIP_FACE + 0.012, z]),
        "tab_r_in_grip": np.array([x, y - task.GRIP_FACE - 0.012, z]),
        "neck": np.array(SEATED),
        "lever_l": np.array(task.PADDLE_WORLD[+1]),
        "lever_r": np.array(task.PADDLE_WORLD[-1]),
    }
    pts = np.stack(list(regions.values()))
    uv, depth = _project(pts)
    assert np.all(depth > 0.3)
    assert np.all((uv[:, 0] >= 5) & (uv[:, 0] <= 315) & (uv[:, 1] >= 5) & (uv[:, 1] <= 171))
    ends, _ = _project(np.array([[x, task.SPEC_HALF, z], [x, -task.SPEC_HALF, z]]))
    assert np.linalg.norm(ends[0] - ends[1]) >= 20.0  # the 165 mm dogbone spans >= 20 px at 320x176
    for arm, p in _plans().items():
        q = p.action7(p.duration)[0][:6]
        tcp, rot = ik.tcp_pose(q, station.BASES[arm])
        arm_pts = np.stack([tcp - rot[:, 2] * d for d in np.linspace(0.0, 0.25, 11)])
        auv, _ = _project(arm_pts)
        for i, name in enumerate(regions):
            assert float(np.min(np.linalg.norm(pts[i] - arm_pts, axis=1))) >= task.HAND_CLEAR_M, (arm, name)
            px = float(np.min(np.linalg.norm(auv - uv[i], axis=1)))
            assert px >= 12.0, f"{arm} final pose occludes {name} in the top camera ({px:.1f} px)"


def test_visual_meshes_exist_and_are_z_up_metres():
    """Every part and jaw draws an authored USD mesh that ships inside the package (rsync'd to the GPU host)."""
    here = pathlib.Path(__file__).resolve().parent
    jaw_parts = (task.JAW_L, task.JAW_R)  # base = a hidden 10 mm block inside the housing; the jaw link is drawn
    plain = {"carriage_r", "key_left", "key_right"}  # a 4 mm pin hidden inside the jaw stem; the two jog keys
    vis = [p.visual for p in task.OBJECTS if p not in jaw_parts] + [j.visual for p in task.OBJECTS for j in p.joints
                                                                    if j.name not in plain]
    assert all(vis) and len(vis) == 11
    for v in vis:
        raw = (here / v).read_bytes()
        if v.endswith(".usdc"):  # binary crate: header magic, z-up metres, texture paths as stored strings
            assert raw[:8] == b"PXR-USDC" and b"upAxis" in raw and b"metersPerUnit" in raw, v
            for tex in set(re.findall(rb"textures/[A-Za-z0-9_.-]+\.png", raw)):
                assert (here / v).parent.joinpath(tex.decode()).is_file(), tex
            continue
        text = raw.decode()
        assert 'upAxis = "Z"' in text and "metersPerUnit = 1" in text and 'defaultPrim = "' in text
        for tex in set(re.findall(r"@(textures/[^@]+)@", text)):
            assert (here / v).parent.joinpath(tex).is_file(), tex
        for ref in set(re.findall(r"@(\./props/[^@]+)@", text)):  # SimReady props: flattened usdc + JPEG textures
            usd = (here / v).parent / ref
            assert usd.is_file() and any((usd.parent / "Textures").iterdir()) and (usd.parent / "LICENSE.txt").is_file(), ref


def test_bench_props_stay_off_the_robot_path():
    """Every visual-only prop footprint lies behind x = 0.48 m (the demo's arms never come below 0.30 m above the
    bench there) and on the table."""
    here = pathlib.Path(__file__).resolve().parent
    info = json.loads((here / "assets/qc_met10/props/props.json").read_text())
    for pkg, places in info["placements"].items():
        lo, hi = np.array(info["props"][pkg]["min"]), np.array(info["props"][pkg]["max"])
        half = (hi - lo)[:2] / 2
        for x, y, yaw in places:
            c, s = abs(math.cos(math.radians(yaw))), abs(math.sin(math.radians(yaw)))
            hx, hy = c * half[0] + s * half[1], s * half[0] + c * half[1]
            assert x - hx >= 0.48 and x + hx <= 0.90 and abs(y) + hy <= 0.64, (pkg, x, y)


def test_seated_tab_with_the_jaw_held_open_is_not_clamped():
    """Negative control (jaw closed on the tab): both tabs seated, both levers forward, but grip a's jaw is held open: no lever row for it."""
    o = _objs(); o["upper_jaw_l"] = _jaw("l", task.JAW_OPEN)
    s = task.score(o)
    assert s == pytest.approx(0.4) and not task.success(o)
    assert task.components(o)["passed"] == ["seat_a", "gap_a", "seat_b", "lever_b", "gap_b"]


def test_a_tab_slid_out_of_its_grip_is_not_seated():
    """Negative control (slip): the specimen displaced 12 mm toward -y loses the +y seat rows (depth under 17 mm) and every both-clamped row."""
    o = _objs(pos=(SEATED[0], SEATED[1] - 0.012, SEATED[2]))
    assert task.score(o) < 0.6 and not task.success(o)
