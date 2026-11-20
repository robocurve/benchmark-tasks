"""The kit's 7 tests for salt_a_rocks_glass_rim_outer_edge_only (written by sync_kit.py --with-tests; edit freely below the line).

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


# -- named layouts for the set-down rungs (scorer only, no simulator) -------------------------------------------------
import numpy as np  # noqa: E402

UPRIGHT = (1.0, 0.0, 0.0, 0.0)  # xyzw: 180 degrees about x, rim up (the glass frame's +z runs rim to base)


def _states(xy=None, z=None, quat=UPRIGHT):
    bar = M.k.mech_state(M.BAR, (*M.MAT_XY, M.TOP + M.GAP))
    xy = M.MARK_XY if xy is None else xy
    z = M.TOP + M.GAP + M.MAT_T + M.H if z is None else z  # upright: base on the mat
    return {"bar": bar, "glass": M.k.mech_state(M.GLASS, (*xy, z), quat)}


def _salted(n=20, over=True):
    M._reset(None)
    M.RUN["wet"][: n + 1] = True
    M.RUN["salted"][:n] = True
    M.RUN["over_n"] = n if over else -1
    M.LIFTED.update(True, 0.0)


def _run(states, seconds=3.2):
    for _ in range(int(seconds * 20)):
        M._watch(states)
    return M.grade(states)


def test_8_stood_on_the_spot_let_go_scores_1():
    _salted()
    assert _run(_states()) == 1.0


def test_9_still_held_on_the_spot_caps_at_075(monkeypatch):
    _salted()
    monkeypatch.setattr(M.k, "hands_clear", lambda *a, **kw: False)
    assert _run(_states()) == 0.75


def test_10_off_the_spot_or_tipped_caps_at_075():
    _salted()
    assert _run(_states(xy=(M.MARK_XY[0] + 0.04, M.MARK_XY[1]))) == 0.75
    _salted()
    tip = np.radians(12.0)  # 12 degrees off upright, about y, still resting on the base's low edge
    quat = (np.cos(tip / 2), 0.0, np.sin(tip / 2), 0.0)  # xyzw of Rx(180) then a 12-degree turn about y
    assert _run(_states(quat=quat)) == 0.75


def test_11_no_hold_over_the_plate_after_the_last_sector_caps_at_075():
    _salted(over=False)
    assert _run(_states()) == 0.75
    _salted()
    M.RUN["over_n"] = 19  # a sector took salt after the hold
    assert _run(_states()) == 0.75


def test_12_dropped_caps_at_075_even_landing_upright():
    _salted()
    s = _states()
    M._watch(_states(z=s["glass"].pos[2] + 0.030))
    M._watch(s)  # fell 30 mm in one step, no gripper near (none outside a simulator)
    assert M.DROPPED
    assert _run(s) == 0.75


def test_13_rim_on_the_mat_after_wetting_is_zero():
    _salted()
    down = _states(xy=M.GLASS_XY, z=M.TOP + M.GAP + M.MAT_T, quat=(0.0, 0.0, 0.0, 1.0))  # set back down on its rim
    M._watch(down)
    assert M.TOUCHED and M.grade(down) == 0.0
    assert _run(_states()) == 0.0  # standing it up afterwards does not undo it


def test_14_too_many_salted_caps_at_075():
    _salted(n=26)
    assert _run(_states()) == 0.75
