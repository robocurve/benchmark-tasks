"""Prepare a raised cart for attended work after the technician withdraws."""
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
sys.path.insert(0,str(HERE.parent))
from yam_arena.task_file import Box
from native.bridge import install
from native.entry_layout import CART_ROOT

SCENE="station"
OBJECTS=[Box("cart_fixture",pos=CART_ROOT,size=(0.001,0.001,0.001))]
SCORE_OVER="end"
SUCCESS_HOLD_S=2.0
monitor=None

def score(objects):
    if monitor is None:raise RuntimeError("The task requires the native cart monitor")
    return float(monitor.qualification_score)

def success(objects):return score(objects)==1.0

from native.cart_motion import demo
install(HERE/'entry.py')
