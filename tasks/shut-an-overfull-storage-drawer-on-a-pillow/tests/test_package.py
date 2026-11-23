"""task.json, README and the code say the same thing."""

import json
import re
from pathlib import Path

from rdb_overfull_drawer import fixed_scene as fixture
from rdb_overfull_drawer import grading, layout as L

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def mm(value):
    return f"{1000 * value:.0f}"


def test_required_fields_and_media_when_uploaded():
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "reset_notes", "fidelity", "scores"):
        assert TASK[key]
    assert TASK["category"] == "Societal relatability"
    if "media" in TASK:
        assert re.fullmatch(r"M-[A-Z0-9]+", TASK["media"])
    assert TASK["scores"].startswith("Dream-1 run: 0.0; demo: 1.0")
    assert "PENDING" not in TASK["scores"]


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == fixture.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{fixture.MAX_STEPS:,} steps")
    assert fixture.CONTROL_HZ == grading.CONTROL_HZ == 20
    assert f"{fixture.MAX_STEPS:,} steps" in README


def test_setup_numbers_match_the_code():
    setup = TASK["setup"]
    assert "{} front to back x {} wide x {} high".format(*map(mm, L.FRAME_SIZE)) in setup
    assert "{} x {} x {} deep inside".format(*map(mm, L.DRAWER_INNER)) in setup
    assert f"front face at x = {mm(L.FRAME_FRONT_X)}" in setup
    assert f"pulled out {mm(L.OPEN_START)}, its front at x = {mm(L.drawer_x(L.OPEN_START))}" in setup
    assert f"a stop at {mm(L.OPEN_LIMIT)}" in setup and f"about {L.RUNNER_FRICTION_N:.0f} N to move it" in setup
    assert f"sides is {mm(L.RIM_Z)} above the table" in setup and f"catches on, {mm(L.RAIL_Z)}" in setup
    assert "{} x {} x {} thick".format(*map(mm, L.PILLOW_PUBLISHED)) in setup and f"{mm(L.PILLOW_MASS)} g" in setup
    assert f"top is {mm(L.FLOOR_Z + max(L.REST_TOP))} above the table" in setup
    assert f"{mm(max(L.ROW_PROUD))} above the opening" in setup
    assert f"the middle {mm(4 * L.ROW_PITCH)} of its length" in setup
    # the book test: the four middle rows pressed from their rest height to the rim
    force = 4 * L.press_force(2, L.FLOOR_Z + .150 - L.RIM_Z)
    assert abs(force / 9.81 - 1.5) < .3 and "1.5 kg book" in setup


def test_scoring_and_context_numbers_match_the_code():
    scoring = TASK["scoring"]
    assert f"open {mm(grading.SHUT_M)} mm or less" in scoring and f"at least {mm(grading.CLEAR_M)} mm from" in scoring
    assert f"open {mm(grading.MOSTLY_M)} mm or less" in scoring and f"open {mm(grading.STARTED_M)} mm or less" in scoring
    assert f"clear for {grading.HOLD_S} s" in scoring and f"more than {mm(grading.UNIT_SHIFT_M)} mm" in scoring
    catches = ", ".join(mm(L.blocking_open(r)) for r in (3, 2))
    assert f"{catches} and {mm(L.blocking_open(1))} mm" in scoring
    context = TASK["context"]
    assert len(context.split()) <= 50 and "pillow" in context


def test_fidelity_states_the_model():
    fidelity = TASK["fidelity"]
    assert f"{len(L.PADS)} rigid pads" in fidelity and f"{L.PAD_STIFFNESS:.0f} N/m" in fidelity
    assert f"{mm(L.PAD_SIZE[0])} x {mm(L.PAD_SIZE[1])} mm" in fidelity and "not cloth" in fidelity
    assert "drawn as one fabric surface" in fidelity and "no collider" in fidelity


def test_text_does_not_identify_anyone():
    for text in (json.dumps(TASK), README):
        assert not re.search(r"/Users/|/home/|@[a-z]+\.(edu|com)\b", text)
