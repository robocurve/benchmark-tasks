import json
from pathlib import Path


ROOT = Path(__file__).parent
TASK = json.loads((ROOT / "task.json").read_text())
SOURCE = (ROOT / "task.py").read_text()


def test_instruction_requires_both_caps_and_aseptic_handling():
    instruction = TASK["instruction"].lower()
    assert "spike-port cap" in instruction
    assert "giving-set spike cap" in instruction
    assert "without touching" in instruction


def test_setup_defines_aseptic_presentation_and_orientation():
    setup = TASK["setup"].lower()
    assert "molded cradle" in setup
    assert "nonsterile" in setup
    assert "12 degrees of vertical" in setup
    assert "fingertip" in setup
    assert "outlet end" in setup


def test_primary_scorer_latches_contamination():
    assert 'if s["spilled"] or s["contaminated"]' in SOURCE
    assert '"asepsis": AsepsisMonitor()' in SOURCE
    assert 'spike_exposed=_EP["spike_cap_off"]' in SOURCE


def test_spike_cap_is_physical_and_required_before_capture():
    assert 'Body.build("spike_cap"' in SOURCE
    assert 'hinge("spike_cap_seal"' in SOURCE
    assert '_EP["cap_off"] and _EP["spike_cap_off"]' in SOURCE
