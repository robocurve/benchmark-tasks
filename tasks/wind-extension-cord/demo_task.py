"""Inspect Robots entry point for the state-assisted scripted demo."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from controller import install_policy
from task import INSTRUCTION, OBJECTS, SCENE, SCORE_OVER, SUCCESS_HOLD_S, demo, score, success

install_policy()
