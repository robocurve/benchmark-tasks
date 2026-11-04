"""Move the left hand to the green marker and leave the loaded tray resting flat on the blue mat without spilling its contents."""

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
