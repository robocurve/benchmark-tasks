"""task.json, README and the code say the same thing."""

import json
import re
from pathlib import Path

import numpy as np

from rdb_etch_triangle import fixed_scene as fixture
from rdb_etch_triangle import grading, layout

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def test_required_fields_and_media_when_uploaded():
    for key in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert TASK[key]
    assert TASK["category"] == "Societal relatability"
    if "media" in TASK:
        assert re.fullmatch(r"M-[A-Z0-9]+", TASK["media"])
    assert TASK["scores"] == "Dream-1 run: 0.0; demo: 1.0"
    assert "PENDING" not in json.dumps(TASK) and "PENDING" not in README


def test_readme_reports_the_measured_isaac_trials():
    record = json.loads((ROOT / "validation.json").read_text())
    assert record["scripted"] == {"score": 1.0, "steps": 1980, "wall_seconds": 364.24}
    assert record["dream_1"]["score"] == 0.0 and record["dream_1"]["steps"] == 3000
    assert "364.24 s" in README and "410.93 s" in README


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == fixture.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{fixture.MAX_STEPS:,} steps")
    assert f"({fixture.MAX_STEPS // fixture.CONTROL_HZ} s at 20 Hz)" in TASK["run_limit"]
    assert fixture.CONTROL_HZ == grading.CONTROL_HZ == 20
    assert f"{fixture.MAX_STEPS:,} steps" in README


def test_setup_numbers_match_the_code():
    setup = TASK["setup"]
    assert f"{1000 * layout.BODY[0]:.0f} x {1000 * layout.BODY[1]:.0f} x {1000 * layout.BODY_H:.1f}" in setup
    assert f"{1000 * layout.RIDGE_H:.1f} at the raised lip" in setup
    assert f"corners rounded {1000 * layout.CORNER_R:.1f}" in setup and "about 454 g" in setup
    assert f"centre at ({1000 * fixture.TOY_XY[0]:.0f}, {1000 * fixture.TOY_XY[1]:.0f})" in setup
    assert f"Screen {1000 * layout.SCREEN[0]:.0f} x {1000 * layout.SCREEN[1]:.0f}" in setup
    assert f"reach {1000 * layout.TRAVEL[0]:.0f} x {1000 * layout.TRAVEL[1]:.0f}" in setup
    assert f"{1000 * layout.KNOB_D:.1f} across and {1000 * layout.KNOB_H:.1f} tall, tops {1000 * layout.KNOB_TOP:.1f}" in setup
    left = 1000 * fixture.knob_station("left_knob")
    assert f"centres at ({left[0]:.0f}, +{left[1]:.0f})" in setup and f"({left[0]:.0f}, -{left[1]:.0f})" in setup
    assert f"One knob turn moves the line {1000 * layout.MM_PER_TURN:.1f}" in setup
    assert f"about {np.degrees(layout.BACKLASH):.0f} degrees" in setup
    sx, sy = (1000 * v for v in layout.START_ON_SCREEN)
    assert f"{sx:.0f} from its left edge and {sy:.0f} from its bottom" in setup
    dot = 1000 * fixture.to_station(layout.START)
    assert f"at ({dot[0]:.0f}, +{dot[1]:.0f})" in setup
    b = 1000 * fixture.to_station(layout.screen_to_toy(layout.CORNERS[1]))
    c = 1000 * fixture.to_station(layout.screen_to_toy(layout.CORNERS[2]))
    assert f"({b[0]:.0f}, {b[1]:.0f})" in setup and f"({c[0]:.0f}, {c[1]:.0f})" in setup
    assert f"x = {dot[0]:.1f} + Y`, `y = {dot[1]:.1f} - X`" in README


def test_scoring_numbers_match_the_code():
    scoring = TASK["scoring"]
    assert f"within {grading.NEAR_MM:.0f} mm of the side" in scoring
    for side, key, total in (("bottom side AB", "bottom", 80), ("upright side BC", "upright", 60)):
        assert f"{side} covered over at least {grading.NEEDED_MM[key]} of its {total} mm" in scoring
    assert grading.NEEDED_MM["slant"] == 100 and "slanted side CA covered over all of its 100 mm" in scoring
    assert f"longer than {grading.STEP_MM:.0f} mm along either axis within {grading.STEP_DEG:.0f} degrees of horizontal or vertical" in scoring
    assert f"Between the {grading.CORNER_CLEAR_MM:.0f} and {100 - grading.CORNER_CLEAR_MM:.0f} mm ticks" in scoring
    assert f"caps the score at {grading.STRAY_CAP}" in scoring
    assert f"{1000 * grading.LIFTED_M:.0f} mm or more off the table" in scoring
    assert f"{1000 * grading.MOVED_M:.0f} mm or more from its marks" in scoring
    assert f"turned {np.degrees(grading.TURNED_RAD):.0f} degrees" in scoring
    assert grading.LET_GO_TICKS == grading.CONTROL_HZ and "during the last second" in scoring
    assert "0, 0.25, 0.5, 1.0" in scoring and grading.TIERS == (.25, .5, 1.)


def test_fidelity_numbers_match_the_code():
    fidelity = TASK["fidelity"]
    assert f"capped at {layout.KNOB_FRICTION:.2f} N m" in fidelity
    assert f"{np.degrees(layout.BACKLASH):.0f} degree dead band" in fidelity
    assert f"{1000 * layout.MM_PER_TURN:.1f} mm per turn" in fidelity
    assert f"{1000 * layout.LINE_WIDTH:.0f} mm wide" in fidelity


def test_text_does_not_identify_anyone():
    for text in (json.dumps(TASK), README):
        assert not re.search(r"/Us" r"ers/|/ho" r"me/|@[a-z]+\.(edu|com)\b", text)
