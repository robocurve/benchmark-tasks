"""In Isaac Sim, through ``inspect-robots run --policy yam_scripted``: every plan in ``task.PLANS`` scores what
``task.EXPECTED`` says (the demo 1.0, each check its tier). Each plan runs in its own process (``tests/run_plan.py``).
Needs an RTX GPU and the Isaac Lab-Arena environment of the simulation walkthrough; skipped without it
(``-m "not isaac"`` skips it too).
"""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

import task as T

pytestmark = pytest.mark.isaac
HERE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    if importlib.util.find_spec("isaaclab_arena") is None:
        pytest.skip("needs the Isaac Lab-Arena environment")
    out = tmp_path_factory.mktemp("plans")
    results = {}
    for plan in T.PLANS:
        args = [sys.executable, str(HERE / "tests" / "run_plan.py"), plan, str(out / plan), "--video"]
        subprocess.run(args, check=True, cwd=HERE, timeout=3600)
        results[plan] = json.loads((out / plan / "summary.json").read_text())
    return out, results


@pytest.mark.parametrize("plan", T.PLANS)
def test_plan_scores_as_expected(runs, plan):
    summary = runs[1][plan]
    assert summary["status"] == "success" and summary["task_score"] == T.EXPECTED[plan]


def test_demo_finishes_within_the_run_limit(runs):
    demo = runs[1]["demo"]
    assert demo["termination"] == "policy_stop" and demo["steps"] < T.MAX_STEPS


def test_demo_video_is_the_top_camera_one_frame_per_step(runs):
    import imageio_ffmpeg

    out, results = runs
    path = str(out / "demo" / "demo.mp4")
    reader = imageio_ffmpeg.read_frames(path)
    meta = next(reader)
    reader.close()
    frames, _ = imageio_ffmpeg.count_frames_and_secs(path)
    assert tuple(meta["size"]) == (640, 360) and meta["codec"] == "h264"
    assert frames == results["demo"]["steps"] + 1


@pytest.mark.parametrize("plan", T.PLANS)
def test_each_check_failed_for_the_reason_it_claims(runs, plan):
    """``task.WHY[plan](final)`` checks the last scored quantities say what the plan was meant to do."""
    assert T.WHY[plan](runs[1][plan]["final"]), runs[1][plan]["final"]
