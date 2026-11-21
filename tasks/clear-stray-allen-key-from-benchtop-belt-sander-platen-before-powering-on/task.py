"""Yam arena task file redirect for clear-stray-allen-key-from-benchtop-belt-sander-platen-before-powering-on."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from clear_stray_allen_key_from_benchtop_belt_sander_platen_before_powering_on import (
    INSTRUCTION,
    OBJECTS,
    SCENE,
    SCORE_OVER,
    SUCCESS_HOLD_S,
    demo,
    score,
    success,
)

__all__ = [
    "INSTRUCTION",
    "OBJECTS",
    "SCENE",
    "SCORE_OVER",
    "SUCCESS_HOLD_S",
    "demo",
    "score",
    "success",
]
