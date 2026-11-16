"""Offline tests for estop-twist-release: latch / release / contactor logic, score tiers, shortcuts, demo reach and
top-camera visibility (SCORING-PREFLIGHT 1-9)."""

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
TASK = load(PKG / "estop_twist_release.py")
M = TASK.module
DT = 0.05  # the official runs use -E control_hz=20
FAR = (np.array([0.32, 0.305, 1.00]), np.array([0.15, -0.33, 0.95]))  # parked TCPs (left READY, right park)
NEAR = (np.array([0.32, 0.305, 1.00]), np.array([*M.RESET_XY, M.TOP + 0.12]))  # right gripper still over RESET


def _state(name, joints):
    return M.labkit.PartState(np.zeros(3), np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), np.zeros(3),
                              np.zeros(3), dict(joints))


def _objects(press=0.0, twist=0.0, reset=0.0, lamp=0.0, platen=0.0):
    return {"estop_station": _state("estop_station", {"head_press": press, "head_twist": twist}),
            "control_station": _state("control_station", {"reset": reset, "lamp": lamp}),
            "grinder_polisher": _state("grinder_polisher", {"platen": platen})}


def run(seq, tcps=FAR):
    """Play (press_m, twist_rad, reset_m, n_steps) segments through the step logic; return the end score."""
    M._trial.clear()
    M._trial.update(M._fresh())
    objs = _objects()
    angle = 0.0
    for press, twist, reset, n in seq:
        for _ in range(n):
            # Unit-only ideal output observations. Separate negative-controls use real MuJoCo physics.
            running = M._trial["running"]
            angle += 2 * math.pi * M.PLATEN_RPM / 60 * DT if running else 0.0
            objs = _objects(press, twist, reset, 0.0 if running else -M.LAMP_OFF_DROP_M, angle)
            M._trial["steps"] += 1
            M._observe_outputs(objs["control_station"], objs["grinder_polisher"], DT)
            M.update_machine(objs["estop_station"], objs["control_station"], DT)
    M._trial["tcps"] = tcps
    return M.score(objs)


REL = -math.radians(M.RELEASE_DEG + 2)
LATCH = -M.LATCH_HOLD_M
IDLE = (0.0, 0.0, 0.0, 10)
PRESS = (-M.STROKE_MAX_M, 0.0, 0.0, 4)
HELD = (LATCH, 0.0, 0.0, 6)
TWIST = (LATCH, REL, 0.0, 4)
OUT = (0.0, 0.0, 0.0, 30)  # head out, machine stopped for 1.5 s
RESET = (0.0, 0.0, -M.RESET_STROKE_M, 4)
SOLVED = [IDLE, PRESS, HELD, TWIST, OUT, RESET, IDLE]


# 1
def test_task_file_loads_with_station_and_hooks():
    assert TASK.scene == "station"
    assert TASK.score_over == "end"
    assert TASK.success_hold_s == pytest.approx(M.SUCCESS_HOLD_S)
    assert [o.name for o in TASK.objects] == ["estop_station", "control_station", "grinder_polisher"]
    assert not TASK.free_objects
    assert M._watch in M.labkit._STEP and M._reset in M.labkit._RESET


# 2
def test_start_scores_zero_and_solved_scores_one():
    assert run([IDLE]) == 0.0
    assert run(SOLVED) == 1.0
    assert M._trial["running"] and not M._trial["latched"]


# 3
def test_documented_tiers_reachable_and_nothing_else():
    text = json.loads((PKG / "task.json").read_text())["scoring"]
    documented = sorted(float(x) for x in re.search(r"Possible: ([0-9., ]+)\.", text).group(1).split(", "))
    assert documented == sorted(M.SCORE_TIERS)
    hits = {
        run([IDLE]): "nothing",
        run([IDLE, PRESS, HELD]): "latched only",
        run([IDLE, PRESS, HELD, TWIST, (0.0, 0.0, 0.0, 5)]): "released, not yet verified",
        run([IDLE, PRESS, HELD, TWIST, OUT]): "released + verified",
        run([IDLE, PRESS, HELD, TWIST, (0.0, 0.0, 0.0, 5), RESET, IDLE]): "reset without no-restart check",
        run(SOLVED, tcps=NEAR): "complete, arm still over RESET",
        run(SOLVED): "complete",
    }
    assert set(hits) == set(M.SCORE_TIERS)
    assert run([IDLE, PRESS, HELD, TWIST, (0.0, 0.0, 0.0, 5)]) == 0.55
    rng = np.random.default_rng(0)
    seen = set()
    for _ in range(400):  # random joint histories only ever give documented tiers
        seq = [(float(rng.choice([0.0, -0.001, LATCH, -M.STROKE_MAX_M])), float(rng.choice([0.0, -0.2, REL])),
                float(rng.choice([0.0, -0.001, -M.RESET_STROKE_M])), int(rng.integers(1, 25))) for _ in range(8)]
        s = run(seq, tcps=FAR if rng.random() < 0.5 else NEAR)
        assert s in M.SCORE_TIERS, (s, seq)
        seen.add(s)
    # Full success is proved by the explicit SOLVED case above, not a requirement
    # that this particular finite random sample happens to complete the sequence.
    assert {0.0, 0.35} <= seen


# 4
def test_fail_closed_on_missing_objects_or_tcps():
    run(SOLVED)
    assert M.score({"estop_station": _objects()["estop_station"]}) == 0.0
    assert M.score({}) == 0.0
    assert run(SOLVED, tcps=None) == 0.9


# 5
@pytest.mark.parametrize("name, seq, want", [
    ("do nothing", [IDLE, (0.0, 0.0, 0.0, 200)], 0.0),
    ("half press, no latch", [IDLE, (-0.003, 0.0, 0.0, 20), IDLE], 0.0),
    ("pull the latched head up instead of twisting", [IDLE, PRESS, HELD, (0.0, 0.0, 0.0, 40)], 0.35),
    ("twist without a latch first", [IDLE, (0.0, REL, 0.0, 10), OUT, RESET, IDLE], 0.0),
    ("reset while latched, then release + reset", [IDLE, PRESS, HELD, (LATCH, 0.0, -M.RESET_STROKE_M, 4), TWIST,
                                                    OUT, RESET, IDLE], 0.65),
    ("press RESET first, then the test", [IDLE, RESET, IDLE] + SOLVED, 1.0),
    ("undo after success: latch again", SOLVED + [PRESS, HELD], 0.35),
    ("no-restart hold broken by a dip, then RESET", [IDLE, PRESS, HELD, TWIST, (0.0, 0.0, 0.0, 12), (-0.002, 0.0, 0.0, 2),
                                                     (0.0, 0.0, 0.0, 12), RESET, IDLE], 0.8),
    ("hold the twisted head down at the end", [IDLE, PRESS, HELD, TWIST, (-0.002, REL, 0.0, 30)], 0.35),
])
def test_shortcuts_and_order(name, seq, want):
    assert run(seq) == pytest.approx(want), name


# 6
def test_rehearsal_uses_real_contact_no_welds():
    src = (PKG / "estop_twist_release.py").read_text()
    assert "mechkit" not in src and "weld" not in src.lower().replace("no welds", "")
    assert all(j.kind in ("slide", "hinge") for o in TASK.objects for j in o.joints)


# 7
def test_demo_is_open_loop_and_inside_run_limit():
    src = inspect.getsource(M.demo)
    for banned in ("_trial", "_part_state", "env", "joint_pos"):
        assert banned not in src, banned
    limit = int(re.match(r"([0-9,]+)", json.loads((PKG / "task.json").read_text())["run_limit"]).group(1).replace(",", ""))
    plan = ArmPlan(TASK.demo()["right"], station.BASES["right"])
    assert (plan.duration + M.SUCCESS_HOLD_S + 1.0) * 20.0 < limit  # -E control_hz=20


# 8
def test_demo_reach_and_joint_limits():
    plan = ArmPlan(TASK.demo()["right"], station.BASES["right"])
    steps = round(plan.duration * CONTROL_HZ) + 5
    actions = np.array([plan.action7((k + 1) / CONTROL_HZ)[0] for k in range(steps)])
    assert plan.worst_ik_mm < 0.5
    assert plan.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)


# 9
def test_scored_parts_visible_and_unoccluded_at_demo_end():
    from yam_arena.urdf import quat_xyzw_to_matrix

    cam = station.top_camera("calibrated")
    rot = quat_xyzw_to_matrix(np.array(cam.rot_xyzw))
    lens = cam.lens.resized(1280, 704)

    def project(p):
        q = rot.T @ (np.asarray(p, float) - np.asarray(cam.pos, float))
        return np.array([lens.cx + lens.fx * q[0] / (-q[2]), lens.cy - lens.fy * q[1] / (-q[2])]), -q[2]

    panel = M.TOP + 0.035 + M.PANEL_Z
    regions = {  # (point, real diameter m, minimum px at 1280 x 704)
        "e-stop head": ((*M.ESTOP_XY, panel + M.HEAD_TOP_ABOVE_PANEL), 2 * M.HEAD_R, 30.0),
        "e-stop + control stations": ((M.STATION_POS[0], -0.050, panel), 0.200, 80.0),
        "RESET": ((*M.RESET_XY, panel + M.RESET_TOP_ABOVE_PANEL), 2 * M.RESET_CAP_R, 15.0),
        "RUN lamp": ((M.STATION_POS[0], M.STATION_POS[1] + M.LAMP_LOCAL[1], panel + 0.013), 0.021, 15.0),
        "platen": ((M.MACHINE_POS[0], M.MACHINE_POS[1], M.MACHINE_POS[2] + M.PLATEN_LOCAL[2]), 0.200, 80.0),
    }
    park = ArmPlan(TASK.demo()["right"], station.BASES["right"]).target(1e9)[0]
    for name, (pt, dia, min_px) in regions.items():
        uv, depth = project(pt)
        assert 0 <= uv[0] < 1280 and 0 <= uv[1] < 704, name
        assert dia * lens.fx / depth >= min_px, (name, dia * lens.fx / depth)
        tuv, tdepth = project(park)
        assert np.linalg.norm(uv - tuv) > 0.10 * lens.fx / tdepth, name


def test_subaction_end_log(capsys):
    run(SOLVED)
    M._log_subaction_end(_objects()["estop_station"], _objects()["control_station"], M._components(_objects()), True)
    out = capsys.readouterr().out
    recs = [json.loads(l[len("SUBACTION_END "):]) for l in out.splitlines() if l.startswith("SUBACTION_END {")]
    assert [r["name"] for r in recs] == [n for n, _ in M.SUBACTIONS]
    assert "run_end=True" in out and "score=1.00" in out


def test_restart_inhibited_until_head_physically_returns():
    run([IDLE, PRESS, HELD, TWIST, (-0.002, REL, -M.RESET_STROKE_M, 4)])
    assert not M._trial["running"]
    assert not M._trial["reset_after_release"]


def test_missing_machine_or_nonfinite_joint_fails_closed():
    run(SOLVED)
    objects = _objects()
    assert M.score({k:v for k,v in objects.items() if k != "grinder_polisher"}) == 0.0
    objects["grinder_polisher"].joints["platen"] = float("nan")
    assert M.score(objects) == 0.0


def test_stuck_lamp_does_not_verify_machine_stop():
    run([IDLE, PRESS, HELD])
    M._trial["stop_verified"] = False
    M._trial["no_restart_ok"] = False
    M._trial["stopped_out_steps"] = 0
    for _ in range(40):
        objects = _objects(LATCH, 0, 0, lamp=0.0, platen=1.0)
        M._observe_outputs(objects["control_station"], objects["grinder_polisher"], DT)
        M.update_machine(objects["estop_station"], objects["control_station"], DT)
    assert not M._trial["stop_verified"]
    assert not M._trial["no_restart_ok"]
    assert M.score(objects) == 0.0


def test_motor_driver_never_writes_joint_position_to_sim():
    assert "write_joint_position_to_sim_index" not in inspect.getsource(M._watch)


def test_twist_reset_alone_does_not_start():
    assert run([IDLE, PRESS, HELD, TWIST, OUT, IDLE]) == 0.65
    assert not M._trial["running"]
    assert not M._trial["reset_after_release"]
    assert M._trial["output_stopped"]


def test_start_held_across_twist_release_does_not_restart():
    held = -M.RESET_STROKE_M
    assert run([IDLE, PRESS, HELD, (LATCH, 0, held, 5),
                (LATCH, REL, held, 5), (0, 0, held, 30)]) == 0.65
    assert not M._trial["running"]
    assert not M._trial["reset_after_release"]
    assert M._trial["output_stopped"]


def test_start_pressed_before_head_returns_needs_a_new_edge():
    held = -M.RESET_STROKE_M
    run([IDLE, PRESS, HELD, TWIST, (-0.002, REL, held, 5), (0, 0, held, 30)])
    assert not M._trial["running"]
    objects = _objects(0, 0, 0, lamp=-M.LAMP_OFF_DROP_M)
    M.update_machine(objects["estop_station"], objects["control_station"], DT)
    objects["control_station"].joints["reset"] = held
    M.update_machine(objects["estop_station"], objects["control_station"], DT)
    assert M._trial["running"]  # fresh deliberate START, not head return, caused restart
