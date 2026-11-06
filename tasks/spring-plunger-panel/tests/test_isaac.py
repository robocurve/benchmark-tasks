"""In Isaac Sim: the scripted demo reaches full score, the three controls score 0, and the videos are right.

Runs ``demo.py`` once for all four policies (Isaac Sim starts once), and ``tests/eval_path.py`` once: the demo on the
Dream-1 command path. Needs an RTX GPU and the Isaac Lab-Arena environment of the simulation walkthrough; skipped
without it. Select or skip it with ``-m isaac`` / ``-m "not isaac"``.
"""

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.isaac
HERE = Path(__file__).resolve().parents[1]
TOP_CAM = (640, 360)  # the station top camera's render size


def _needs_isaac():
    if importlib.util.find_spec("isaaclab_arena") is None:
        pytest.skip("needs the Isaac Lab-Arena environment")


def _video(path: Path) -> dict:
    """The size, codec and frame count of an MP4."""
    import imageio_ffmpeg

    reader = imageio_ffmpeg.read_frames(str(path))
    meta = next(reader)
    reader.close()
    frames, _ = imageio_ffmpeg.count_frames_and_secs(str(path))
    return {"size": tuple(meta["size"]), "codec": meta["codec"], "frames": frames}


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    _needs_isaac()
    out = tmp_path_factory.mktemp("isaac")
    subprocess.run([sys.executable, str(HERE / "demo.py"), "--policy", "demo", "hold", "pull_only", "early_release",
                    "--out", str(out)], check=True, cwd=HERE, timeout=3600)
    return out, {r["policy"]: r for r in json.loads((out / "result.json").read_text())}


@pytest.fixture(scope="module")
def results(run):
    return run[1]


def test_demo_reaches_full_score_without_forcing(results):
    demo = results["demo"]
    assert demo["success"] and demo["task_score"] == 1.0
    assert not demo["forced"] and demo["worst_pin_overlap_mm"] < 1.0
    assert demo["steps"] <= 600 and demo["max_plunger_travel_mm"] >= 12.0


def test_demo_video_is_the_top_camera_one_frame_per_step(run):
    out, results = run
    video = _video(out / "demo.mp4")
    assert video["size"] == TOP_CAM and video["codec"] == "h264"
    assert video["frames"] == results["demo"]["steps"] + 1  # the start, then every control step

    import imageio.v2 as imageio

    setup = imageio.imread(out / "setup.png")
    assert setup.shape[:2] == TOP_CAM[::-1]


def _check_control(results, policy):
    control = results[policy]
    assert control["task_score"] == 0.0 and not control["success"]
    assert control["steps"] == 600  # a whole trial
    assert control["max_angle_deg"] < 2.0


def test_hold_scores_0(results):
    _check_control(results, "hold")


def test_pulling_the_latched_panel_scores_0(results):
    _check_control(results, "pull_only")
    assert results["pull_only"]["max_plunger_travel_mm"] < 0.5  # the plunger was never touched


def test_letting_go_of_the_knob_early_scores_0(results):
    _check_control(results, "early_release")
    assert results["early_release"]["max_plunger_travel_mm"] >= 12.0  # the pin was fully back before the release


def test_dream_command_path_scores_the_demo_and_records_the_top_camera(tmp_path):
    """The demo through ``inspect_robots.eval`` as ``inspect-robots run --policy dream`` runs it (default guardrails,
    cameras at 320 x 176 for the model), with the README's ``TOP_CAM_VIDEO`` recording."""
    _needs_isaac()
    video_path = tmp_path / "dream.mp4"
    env = dict(os.environ, TOP_CAM_VIDEO=str(video_path))
    subprocess.run([sys.executable, str(HERE / "tests" / "eval_path.py"), str(tmp_path / "log")], check=True, cwd=HERE,
                   env=env, timeout=3600)
    summary = json.loads((tmp_path / "log" / "summary.json").read_text())
    assert summary["status"] == "success" and summary["task_score"] == 1.0
    assert not summary["episode"]["forced"]
    video = _video(video_path)
    assert video["size"] == TOP_CAM and video["codec"] == "h264"
    assert video["frames"] == summary["steps"] + 1
    assert (tmp_path / "dream.png").exists()  # the first frame
