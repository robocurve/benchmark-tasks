"""Cross-check the physical instructions, the board card and the live settings."""

import json
from pathlib import Path

from rdb_filter_clamp import fixed_scene as F
from rdb_filter_clamp import grading as G

ROOT = Path(__file__).resolve().parents[1]
CARD = json.loads((ROOT / "task.json").read_text())


def test_rubric_matches_code():
    assert CARD["instruction"] == F.INSTRUCTION
    assert f"{1000 * G.LIFT_HEIGHT:.0f} mm" in CARD["scoring"]
    assert f"{G.HOLD_TICKS / G.CONTROL_HZ:g} s" in CARD["scoring"]
    assert f"{G.RELEASE_TICKS / G.CONTROL_HZ:g} s" in CARD["scoring"]
    assert f"{F.MAX_STEPS:,} steps" in CARD["run_limit"]
    assert F.CONTROL_HZ == G.CONTROL_HZ == 20


def test_scene_contains_stationary_height_reference_and_free_lower_body():
    src = F.USD.read_text()
    assert 'def Mesh "lift_mark"' in src
    assert "PhysicsFixedJoint" not in src and "physics:kinematicEnabled = 1" not in src
    assert "PhysicsRigidBodyAPI" in src
