import json
from pathlib import Path


HERE = Path(__file__).parent
SPEC = json.loads((HERE / "task.json").read_text())
SOURCE = (HERE / "clip_a_condenser_onto_a_still_head.py").read_text()
README = (HERE / "README.md").read_text()


def test_submission_fields_and_category():
    required = {"title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores", "media"}
    assert required <= SPEC.keys()
    assert SPEC["category"] == "Other"
    assert SPEC["run_limit"] == "2,400 steps (2 minutes at 20 Hz)"


def test_matched_24_40_geometry_is_explicit():
    assert "CONE_LARGE, CONE_SMALL, CONE_LEN = 0.0120, 0.0100, 0.040" in SOURCE
    assert "JOINT_CLEAR = 0.00025" in SOURCE
    assert "24.0 mm large and 20.0 mm small" in SPEC["setup"]
    assert "0.50 mm simulated diametral clearance" in SPEC["setup"]


def test_clip_is_the_matching_size_and_captures_both_lips():
    assert "size-24 POM Keck clip" in SPEC["setup"]
    assert "capture both joint lips" in SPEC["setup"]
    assert "CLIP_IN = (0.006, 0.006, 0.004)" in SOURCE


def test_support_carries_vertical_load():
    assert "fixed cork-lined V support" in SPEC["setup"]
    assert "carries gravity" in SPEC["setup"]
    assert "support_point = cp + cr @ np.array([0.0, -0.205, 0.0])" in SOURCE


def test_full_score_is_current_not_historical():
    assert 'if s["retained"] and s["home"]' in SOURCE
    assert 's.get("seated") and s.get("clip_now")' in SOURCE
    assert "currently seated and clipped" in SPEC["scoring"]


def test_release_and_retention_are_numeric():
    assert "RELEASE_CLEARANCE = 0.06" in SOURCE
    assert "RETENTION_HOLD_S = 2.0" in SOURCE
    assert "1.47 N weight for 2 continuous seconds" in SPEC["scoring"]
    assert "at least 6 cm" in SPEC["scoring"]


def test_drop_and_breakage_path_zeroes_score():
    assert "downward speed over 0.7 m/s or tilt over 35 degrees" in SPEC["scoring"]
    assert "total speed over 1.2 m/s" in SPEC["scoring"]
    assert 'if s["dropped"] or s["damaged"]:\n        return 0.0' in SOURCE


def test_one_consistent_clip_tolerance_everywhere():
    for text in (SPEC["scoring"], README):
        assert "6 mm" in text and "4 mm" in text and "20" in text
    assert "8 mm across" not in README
    assert "10 mm along" not in README


def test_instruction_requires_release():
    instruction = SPEC["instruction"]
    assert "24/40 female joint" in instruction
    assert "size-24 Keck clip" in instruction
    assert "Release both objects" in instruction


def test_readme_has_reproducible_run_and_reset():
    assert "--max-steps 2400" in README
    assert "cam_width=1280" in README and "cam_height=720" in README
    assert "## Reset" in README
