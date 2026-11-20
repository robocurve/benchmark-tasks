"""Passive reset diagnostic only. This is not the full task reference."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from task import *
def demo():
 from yam_arena.constants import READY
 import numpy as np
 for _ in range(60):yield np.r_[READY,1.,READY,1.]
