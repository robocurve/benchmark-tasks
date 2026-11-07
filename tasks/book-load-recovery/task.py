"""Park the left gripper pointing down over the green pad and leave both books flat on the blue mat without dropping them."""

from pathlib import Path
from resist_isaac.recovery import RecoveryPilot, export_assets

CONDITIONS = ("recoverable",)

def make_task(condition):
    if condition not in CONDITIONS:
        raise ValueError(f"Unsupported condition: {condition}")
    return RecoveryPilot(Path(__file__).parent)

def create_assets(directory, condition="recoverable"):
    if condition not in CONDITIONS:
        raise ValueError(f"Unsupported condition: {condition}")
    return export_assets(Path(__file__).parent, directory)
