"""Put the battery pack on the blue shelf and hold the left gripper 15 cm above the green marker, without dropping the pack."""
from pathlib import Path
from resist_isaac.clearance_recovery import ClearanceRecovery, export_assets

CONDITIONS=("recoverable",)

def make_task(condition):
    if condition not in CONDITIONS: raise ValueError(condition)
    return ClearanceRecovery(Path(__file__).parent)

def create_assets(directory,condition="recoverable"):
    if condition not in CONDITIONS: raise ValueError(condition)
    return export_assets(Path(__file__).parent,directory)
