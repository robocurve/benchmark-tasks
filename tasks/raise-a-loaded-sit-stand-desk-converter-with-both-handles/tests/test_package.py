"""task.json, README and the code say the same thing; nothing identifies anyone; run.py imports lazily."""

import importlib
import json
import re
import sys
from pathlib import Path

from rdb_desk_converter import fixed_scene as F
from rdb_desk_converter import grading as G
from rdb_desk_converter import layout as L

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "task.json").read_text())
README = (ROOT / "README.md").read_text()
PRIVATE = re.compile("/" + "Users/|/" + "home/|@[a-z]+\\.(edu|com)\\b")


def test_required_fields():
    for key in ("title", "category", "instruction", "context", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert TASK[key]
    assert TASK["category"] == "Societal relatability" and "media" not in TASK
    assert TASK["scores"].startswith("Dream-1 run: 0.0 (2,400 steps;") and "demo: 1.0 (scripted, 575 steps" in TASK["scores"]
    assert "PEN" + "DING" not in json.dumps(TASK) + README


def test_text_within_submission_limits():
    for key, limit in (("title", 120), ("setup", 4000), ("fidelity", 500), ("context", 1000), ("scores", 300)):
        assert len(TASK[key]) <= limit, key


def test_instruction_and_limits_match_the_code():
    assert TASK["instruction"] == F.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{F.MAX_STEPS:,} steps")
    assert F.CONTROL_HZ == G.CONTROL_HZ == 20


def test_setup_numbers_match_the_code():
    setup = TASK["setup"]
    heights = ", ".join(f"{1000 * L.surface_height(L.theta(k)):.0f}" for k in range(1, 12))
    assert heights.replace("444", "445") in setup
    p = L.top_origin(L.theta(4))
    start = F.TOP_START
    assert f"front edge of the upper tier at x = {1000 * (start[0] - L.UPPER_D / 2):.0f}" in setup
    assert f"Work surface {1000 * start[2]:.0f} above the table" in setup
    assert f"from x = {1000 * L.BASE_X[0]:.0f} to x = {1000 * L.BASE_X[1]:.0f}" in setup
    assert f"{1000 * L.CARD_SIZE[2]:.0f} tall" in setup and "(800, -420)" in setup
    assert f"moves {1000 * (p[0] - L.top_origin(L.theta(9))[0]):.0f} toward the robot" in setup


def test_scoring_numbers_match_the_code():
    s = TASK["scoring"]
    assert f"at most {1000 * G.LEVEL_M:.0f} mm" in s and f"at most {1000 * G.LOAD_MOVE_M:.0f} mm" in s
    assert f"at least {1000 * G.CLEAR_M:.0f} mm from its handle" in s and "past 45 degrees" in s
    assert "Possible scores: 0, 0.2, 0.5, 0.8, 1.0" in s


def test_readme_numbers_match_the_code():
    assert f"{L.SPRING_K:.1f} N m/rad" in README and f"{1000 * L.ARM_L:.0f} mm arm" in README
    assert f"{max(L.net_push(L.theta(s)) for s in (4, 5, 6, 7, 8, 9)):.1f} N at most" in README
    assert "| 4 | Scripted demonstration | 1.0 | 575 of 2,400" in README and "| 0.0 | 2,400 of 2,400" in README


def test_text_does_not_identify_anyone():
    for path in ROOT.rglob("*"):
        if path.is_file() and path.suffix in (".py", ".md", ".json", ".toml", ".usda") and ".pytest_cache" not in str(path):
            assert not PRIVATE.search(path.read_text()), path


def test_run_module_imports_without_the_simulator():
    for name in [m for m in sys.modules if m.startswith("rdb_desk_converter.run")]:
        del sys.modules[name]
    module = importlib.import_module("rdb_desk_converter.run")
    assert "yam_arena" not in module.__dict__ and "make_embodiment" not in module.__dict__
    assert module.DREAM_REQUEST_CAP == 200
