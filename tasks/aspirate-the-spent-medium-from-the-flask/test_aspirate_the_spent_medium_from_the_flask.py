"""The kit's 7 tests for aspirate_the_spent_medium_from_the_flask (written by sync_kit.py --with-tests; edit freely below the line).

    PYTHONPATH=<yam-isaac-lab-arena> PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q

The task file provides ``TEXT_NUMBERS`` [(snippet as task.json says it, the code's value in SI)], ``CHEATS``
{demo mode: the most it may score} and ``HOLD`` {"mech.joint": the most it may move with no command, mm or deg}.
Tests 4-6 read physcheck runs (``.planning/kit/kit_check.sh <task>``) and run holdcheck; they skip where our pipeline
isn't (a public copy of the task).
"""

import pathlib
import sys

import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from ourkit import testkit as T  # noqa: E402
from yam_arena import task_file  # noqa: E402

TASK = task_file.load(T.task_file_of(HERE))
M = TASK.module
SPEC = T.spec(HERE)
README = (HERE / "README.md").read_text() if (HERE / "README.md").exists() else ""
PIPE = T.pipeline_dir(HERE)
needs_pipeline = pytest.mark.skipif(PIPE is None, reason="no .planning/pipeline (public copy)")


def test_1_text_matches_code():
    assert getattr(M, "TEXT_NUMBERS", None), "list TEXT_NUMBERS in the task file"
    assert T.check_text_numbers(SPEC, M.TEXT_NUMBERS) == []


def test_2_blind_text():
    assert T.check_blind(SPEC, README) == []


def test_3_json_fields_and_run_limit():
    assert T.check_json(SPEC, M) == []


@needs_pipeline
@pytest.mark.parametrize("mode", sorted(getattr(M, "CHEATS", {"none": 0.0})))
def test_4_cheats_score_low(mode):
    cap = getattr(M, "CHEATS", {"none": 0.0})[mode]
    r, problem = T.physcheck_result(HERE, mode)
    assert r is not None and not problem, problem
    assert r["final_score"] <= cap + 1e-9, f"{mode} scored {r['final_score']} (cap {cap})"


@needs_pipeline
def test_5_demo_scores_1_in_physcheck():
    r, problem = T.physcheck_result(HERE, "demo")
    assert r is not None and not problem, problem
    assert r["final_score"] == 1.0 and r["success"], r["final_score"]
    assert r["steps"] <= T.run_limit_steps(SPEC)
    assert not r["arm_arm"], f"arms touched: {r['arm_arm']}"


@needs_pipeline
def test_6_state_holds_without_command():
    h = T.run_hold(HERE, seconds=5.0)
    caps = getattr(M, "HOLD", {})
    for joint, row in h["joints"].items():
        assert row["drift"] <= caps.get(joint, 0.5), f"{joint} moved {row['drift']} {row['unit']} with no command"
    assert all(v <= 2.0 for v in h["body_mm"].values()), h["body_mm"]
    assert (h["score"] or 0.0) == 0.0


def test_7_text_limits():
    assert T.check_limits(SPEC) == []


# == task tests (from the spec: one per rung, the latched zeros, the undo, the volume rule's cheats) ==================
import math  # noqa: E402

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402


def _poses(theta_deg, phi_deg):
    return M.flask_pose(math.radians(theta_deg), math.radians(phi_deg))


def _tip_world(R, o, tip_local, target=None):
    """The open end's corners with the tool on the demo's line, its tip centre at ``tip_local`` (flask frame)."""
    d_loc, _ = M.line_of(M.TARGET if target is None else target)
    d = R @ d_loc
    Rt = M.tool_rot(d)
    org = o + R @ tip_local - M.LEVER * d
    return org + M.TIP_PTS @ Rt.T, org + M.PIP_PTS @ Rt.T


def _drain(theta, phi, tip_local, seconds=12.0):
    R, o = _poses(theta, phi)
    tips, _ = _tip_world(R, o, tip_local)
    V = M.V0_ML * 1e-6
    for _ in range(int(seconds / M.DT)):
        V, h, _d = M.medium_step(V, R, o, tips)
    return V * 1e6


def _resting(theta, phi, start, along=True, target=None):
    """The tip moved from ``start`` along the line until its face meets the back wall (``along``), then down until its
    lowest corner meets the floor: where the walls stop a pressed tip."""
    R, o = _poses(theta, phi)
    d_loc, _ = M.line_of(M.TARGET if target is None else target)
    tip = np.array(start, float)
    for _ in range(400):
        tips, _ = _tip_world(R, o, tip, target)
        q = M.in_flask(tips, R, o)
        if along and q[:, 0].min() > M.IN_LO[0] + 0.0001:
            tip = tip + 0.0001 * d_loc
        elif q[:, 2].min() > M.IN_LO[2] + 0.0001:
            tip = tip + np.array([0.0, 0.0, -0.0001])
        else:
            break
    return tip


def test_rule_demo_rest_draws_to_full_credit():
    tip = _resting(35, 15, M.TARGET)
    ml = _drain(35, 15, tip)
    assert ml <= M.V_FULL, ml
    R, o = _poses(35, 15)
    assert not M.cell_touch(_tip_world(R, o, tip)[1], R, o)


def test_rule_tip_short_of_the_corner_misses_full_credit():
    # resting on the floor but 12 mm short of the back wall: more than 0.5 mL stays, and the floor there is cells
    tip = _resting(35, 15, M.TARGET + np.array([0.008, 0.0, 0.0]), along=False)
    R, o = _poses(35, 15)
    assert M.cell_touch(_tip_world(R, o, tip)[1], R, o)


def test_rule_flat_cheat_leaves_the_film():
    # tip resting at the back wall on the floor of a level flask: the 0.25 mm film and what lies under the rim
    # (about 1.9-2.5 mL) stay: 0.5
    tip = _resting(0, 0, np.array([M.IN_LO[0] + 0.006, M.IN_HI[1] - 0.014, M.IN_LO[2] + 0.005]))
    ml = _drain(0, 0, tip, seconds=20)
    assert 1.7 <= ml <= M.V_050, ml
    assert M.grade(ml, False, False, False, True) == 0.5


def test_rule_hover_cheat_leaves_more_than_3_ml():
    ml = _drain(35, 15, M.HOVER)
    assert M.V_050 < ml <= M.V_025, ml
    assert M.grade(ml, False, False, False, True) == 0.25


def test_rule_no_tip_no_draw():
    R, o = _poses(35, 15)
    far = np.array([[1.0, 1.0, 2.0]] * 4)
    V, _, d = M.medium_step(M.V0_ML * 1e-6, R, o, far)
    assert not d and abs(V * 1e6 - M.V0_ML) < 1e-9


def test_pour_is_a_spill():
    R, o = M.flask_pose(math.radians(-40.0), 0.0)  # neck down
    fv = M.film(M.floor_slope(R))
    h = M.surface(R, o, M.V0_ML * 1e-6 - fv)
    assert h > M.lip_z(R, o)
    R, o = _poses(35, 15)
    h = M.surface(R, o, M.V0_ML * 1e-6 - M.film(M.floor_slope(R)))
    assert h < M.lip_z(R, o)
    R, o = _poses(0, 0)
    h = M.surface(R, o, M.V0_ML * 1e-6 - M.film(0.0))
    assert h < M.lip_z(R, o)


def test_cell_touch_only_beyond_the_strip():
    R, o = _poses(35, 15)
    mid = _resting(35, 15, M.MIDDLE, along=False, target=M.MIDDLE)
    assert M.cell_touch(_tip_world(R, o, mid, M.MIDDLE)[1], R, o)
    assert not M.cell_touch(_tip_world(R, o, M.TARGET)[1], R, o)


@pytest.mark.parametrize("ml,touched,put,want", [
    (15.0, False, True, 0.0), (10.5, False, True, 0.0), (10.0, False, False, 0.25), (3.0, False, False, 0.5),
    (2.0, False, True, 0.5), (1.0, False, True, 0.75), (1.0, False, False, 0.5), (0.5, False, False, 0.75),
    (0.5, False, True, 1.0), (0.2, True, True, 0.25), (12.0, True, True, 0.0)])
def test_ladder_rungs(ml, touched, put, want):
    assert M.grade(ml, touched, False, False, put) == want


def test_latched_zeros_stay_zero():
    assert M.grade(0.2, False, True, False, True) == 0.0
    assert M.grade(0.2, False, False, True, True) == 0.0


def _state(spec, R, p):
    return k.mech_state(spec, p, k.scene.quat_xyzw(R))


def _scene(theta=0.0, asp_pos=None):
    Rf, of = M.flask_pose(math.radians(theta), 0.0)
    fl = _state(M.FLASK, Rf, of + Rf @ M.G_LOC)
    asp = _state(M.ASPIRATOR, M.STAND_ROT, M.STAND_G + 0.0006 * M.UP if asp_pos is None else asp_pos)
    return {"flask": fl, "aspirator": asp}


def test_undo_loses_the_put_back():
    st = _scene()
    M._reset(st)
    R, o = M.flask_frame(st["flask"])
    assert M.put_back(R, o, st["aspirator"]) == (True, True)
    lifted = _scene(asp_pos=M.STAND_G + np.array([0.0, -0.03, 0.08]))
    assert M.put_back(R, o, lifted["aspirator"])[1] is False
    tilted = _scene(theta=10.0)
    R, o = M.flask_frame(tilted["flask"])
    assert M.put_back(R, o, tilted["aspirator"])[0] is False


def test_drop_latches():
    st = _scene()
    M._reset(st)
    for _ in range(3):
        M._watch(st)
    assert not M.DROPPED
    M._watch(_scene(asp_pos=M.STAND_G + np.array([0.0, 0.0, -0.030])))
    assert M.DROPPED
    for _ in range(5):
        M._watch(st)
    assert M.DROPPED and M.score(st) == 0.0
    M._reset(st)


def test_untouched_start_scores_zero():
    st = _scene()
    M._reset(st)
    for _ in range(50):
        M._watch(st)
    assert M.RUN["ml"] == M.V0_ML and M.score(st) == 0.0
