"""The kit's 7 tests for polish_a_disk_electrode_in_a_figure_8_face_flat (written by sync_kit.py --with-tests; edit freely below the line).

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
