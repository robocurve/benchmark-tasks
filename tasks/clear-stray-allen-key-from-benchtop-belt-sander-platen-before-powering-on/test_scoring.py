"""Unit tests for belt sander scoring and hazard defect latch."""

from __future__ import annotations

import numpy as np
import pytest

from clear_stray_allen_key_from_benchtop_belt_sander_platen_before_powering_on import (
    ALLEN_KEY_INIT_POS,
    SWITCH_POS,
    TRAY_POS,
    failed,
    score,
    success,
)
import mechkit as mk


class MockState:
    def __init__(self, pos):
        self.pos = np.array(pos, dtype=float)
        self.quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)


def test_initial_state_zero_score():
    mk.MEMORY.clear()
    objects = {
        "belt_sander": MockState([0.24, 0.0, 0.748 + 0.05]),
        "power_switch": MockState(SWITCH_POS),
        "allen_key": MockState(ALLEN_KEY_INIT_POS),
        "magnetic_tray": MockState(TRAY_POS),
    }
    assert score(objects) == 0.0
    assert not success(objects)
    assert not failed(objects)


def test_hazard_latch_tripped():
    mk.MEMORY.clear()
    mk.MEMORY["init_key_pos"] = ALLEN_KEY_INIT_POS.copy()
    mk.MEMORY["init_switch_pos"] = SWITCH_POS.copy()

    # Switch toggled ON while key is still on platen
    mk.MEMORY["switch_on"] = True
    mk.MEMORY["hazard_tripped"] = True

    objects = {
        "belt_sander": MockState([0.24, 0.0, 0.748 + 0.05]),
        "power_switch": MockState(SWITCH_POS + np.array([0, 0, 0.015])),
        "allen_key": MockState(ALLEN_KEY_INIT_POS),
        "magnetic_tray": MockState(TRAY_POS),
    }
    assert score(objects) == 0.0
    assert failed(objects)


def test_full_success_score():
    mk.MEMORY.clear()
    mk.MEMORY["key_lifted"] = True
    mk.MEMORY["key_in_tray"] = True
    mk.MEMORY["switch_on"] = True
    mk.MEMORY["hazard_tripped"] = False
    mk.MEMORY["hold_timer"] = 25

    objects = {
        "belt_sander": MockState([0.24, 0.0, 0.748 + 0.05]),
        "power_switch": MockState(SWITCH_POS + np.array([0, 0, 0.015])),
        "allen_key": MockState(TRAY_POS + np.array([0, 0, 0.015])),
        "magnetic_tray": MockState(TRAY_POS),
    }
    assert score(objects) == 1.0
    assert success(objects)
    assert not failed(objects)


if __name__ == "__main__":
    test_initial_state_zero_score()
    test_hazard_latch_tripped()
    test_full_success_score()
    print("All unit tests passed successfully!")
