"""Register the matched docking policies for the same receiving fixture."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT / "task" if (ROOT / "task/entry.py").is_file() else ROOT
sys.path.insert(0, str(TASK))
from cover_motion import demo as plan

PAIR = ("dock_while_occupied", "wait_then_dock")
