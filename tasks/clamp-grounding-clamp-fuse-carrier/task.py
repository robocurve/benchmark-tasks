"""Yam arena task file redirect for clamp-grounding-clamp-fuse-carrier."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from clamp_grounding_clamp_fuse_carrier import (
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
