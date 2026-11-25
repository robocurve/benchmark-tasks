"""Require the released full score and frame audit before the capped model run."""
import os
from pathlib import Path

from verify_last_seat import verify

HERE = Path(__file__).resolve().parent
FULL = HERE / f"evidence-{os.environ.get('DISH_REPAIR_REVISION', '8')}-full"
CONTROLS = 360 if os.environ.get("DISH_REPAIR_REVISION") == "13" else 340


def verify_demo():
    result = verify(FULL, CONTROLS, full=True)
    if not all((FULL / "media" / f).is_file() for f in ("demo.mp4", "setup.png")):
        raise ValueError("The demonstration and first overhead frame must be present")
    return result
