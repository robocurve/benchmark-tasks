"""Entry point for the Arena task-file loader."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from task import *  # noqa: F403
