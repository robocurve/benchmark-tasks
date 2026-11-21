"""Yam arena task file redirect for reassemble-commercial-citrus-juicer."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from reassemble_commercial_citrus_juicer import (
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
