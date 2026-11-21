"""task.json, README and the code say the same thing."""

import json
import re
from pathlib import Path

from rdb_tablets_cap import fixed_scene as fixture
from rdb_tablets_cap import grading
from rdb_tablets_cap import layout as L

ROOT = Path(__file__).resolve().parents[1]
TASK = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text() + "\n" + (ROOT / "TECHNICAL.md").read_text()


def test_required_fields_and_no_media():
    for key in ("title", "category", "instruction", "context", "setup", "scoring", "run_limit", "reset_notes", "fidelity", "scores"):
        assert TASK[key]
    assert "media" not in TASK and TASK["category"] == "Societal relatability"
    assert len(TASK["fidelity"].split()) <= 95


def test_instruction_and_run_limit_match_the_code():
    assert TASK["instruction"] == fixture.INSTRUCTION
    assert TASK["run_limit"].startswith(f"{fixture.MAX_STEPS:,} steps")
    assert fixture.CONTROL_HZ == grading.CONTROL_HZ == 20
    assert f"{fixture.MAX_STEPS:,} actions" in README


def test_setup_numbers_match_the_code():
    setup = TASK["setup"]
    assert f"({fixture.BOTTLE_XY[0]:.3f}, {fixture.BOTTLE_XY[1]:.3f})" in setup
    assert f"({fixture.CAP_XY[0]:.3f}, {fixture.CAP_XY[1]:.3f})" in setup
    assert f"({fixture.CUP_XY[0]:.3f}, {fixture.CUP_XY[1]:.3f})" in setup
    assert f"{2000 * L.BOTTLE_R:.1f} mm across, {1000 * L.BOTTLE_H:.1f} mm tall, {1000 * L.BOTTLE_MASS:.1f} g" in setup
    assert f"bore about {2000 * L.NECK_R_IN:.0f} mm" in setup
    assert f"{L.N_TABLETS} round brown" in setup and f"{2000 * L.TABLET_R:.0f} mm across and about {1000 * L.TABLET_T:.0f} mm thick" in setup
    assert f"about {1000 * L.TABLET_MASS:.2f} g each" in setup
    assert f"{2000 * L.CAP_R:.1f} mm across, {1000 * L.CAP_H:.1f} mm tall, about {1000 * L.CAP_MASS:.0f} g" in setup
    assert f"inside about {2000 * L.CAP_R_IN:.0f} mm across and {1000 * L.CAP_DEPTH:.0f} mm deep" in setup
    assert f"{2000 * L.CUP_R_TOP:.1f} mm across the top, {2000 * L.CUP_R_BOTTOM:.1f} mm across the bottom, {1000 * L.CUP_H:.1f} mm tall" in setup


def test_scoring_numbers_match_the_code():
    scoring = TASK["scoring"]
    tiers = grading.TIERS
    assert tiers == {"dose_given_and_tidy": 1., "dose_given": .7, "one_off": .4, "dose_in_cap": .2}
    for value in ("1.0:", "0.7:", "0.4:", "0.2:"):
        assert value in scoring
    assert f"the other {L.N_TABLETS - L.DOSE} in the bottle" in scoring
    assert f"lain in the cap for {grading.LIE_TICKS / grading.CONTROL_HZ:g} s" in scoring
    assert f"for {grading.TOUCH_TICKS / grading.CONTROL_HZ:g} s or more in total" in scoring
    assert f"at least {1000 * grading.CLEAR_M:.0f} mm from" in scoring
    assert f"under {1000 * grading.STILL_M:.0f} mm of movement) for {grading.STILL_TICKS // grading.CONTROL_HZ} s" in scoring
    assert f"capped at {grading.CAP_NOT_DONE}" in scoring and "up to three" in scoring and grading.MAX_STRAYS == 3
    assert "within 10 degrees" in scoring


def test_fidelity_matches_the_model():
    fidelity = TASK["fidelity"]
    assert f"{L.SIDES} flat staves" in fidelity and f"{1000 * L.BOTTLE_WALL:.0f} mm thick in the bottle" in fidelity
    assert f"{1000 * L.CUP_MODEL_WALL:.1f} mm in the cup" in fidelity and f"friction {L.FRICTION}" in fidelity
    assert f"inside {2000 * (L.BOTTLE_WALL - .001):.0f} mm narrower" in fidelity


def test_text_does_not_identify_anyone():
    for text in (json.dumps(TASK), README):
        assert not re.search(r"/Users/|/home/|@[a-z]+\.(edu|com)\b", text)


def test_reported_scores_match_the_evidence_for_the_closed_start_revision():
    evidence = json.loads((ROOT / "validation.json").read_text())
    if evidence.get("status") == "awaiting_closed_start_native_evidence":
        assert "pending" in TASK["scores"]
        assert "recorded_hashes" not in evidence
    else:
        import hashlib
        from rdb_tablets_cap import fixed_scene, grading, layout

        for key, source in (("scene", fixed_scene.__file__), ("asset", fixed_scene.USD),
                            ("layout", layout.__file__), ("grader", grading.__file__)):
            assert evidence["recorded_hashes"][f"{key}_sha256"] == hashlib.sha256(Path(source).read_bytes()).hexdigest()
        for field, name in (("Dream-1 run", "dream"), ("demo", "scripted")):
            reported = re.search(re.escape(field) + r":\s*([0-9.]+)", TASK["scores"])
            assert reported and float(reported[1]) == evidence[name]["score"]


def test_held_cap_rule_matches_the_code():
    scoring = TASK["scoring"]
    assert "not pinched by both fingertips of one gripper" in scoring and "the bottle or the cup" in scoring
    assert f"for {grading.UNHELD_TICKS / grading.CONTROL_HZ:g} s a tablet lies in the cap" in scoring
    assert f"at least {grading.HELD_N:g} N each" in README and f"{grading.SUPPORT_N:g} N or more" in README


def test_two_handed_pour_is_asked_for_and_scored():
    assert "hold the bottle's cap in one hand and, with the other, tip two tablets" in TASK["instruction"]
    assert f"lowest point less than {1000 * grading.RESTING_M:.0f} mm above the table" in TASK["scoring"]
    assert "one holds the cap level under the bottle's lip while the other tips the bottle" in TASK["context"]
    assert "Tipping the cap into the cup at the end is done with one hand." in TASK["context"]
    assert "with one hand, the left, while the right arm waits" in README
