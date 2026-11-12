"""Insert the sleeve fully into the housing, with its radial oil hole aligned to the housing oil passage. Release it and leave it seated and still for at least one second. Gravity seating is allowed. This operation ends before a separate locking operation."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from prototype.layout import OBJECTS
SCENE='station';SCORE_OVER='end';SUCCESS_HOLD_S=0
def success(objects):return objects['__evaluation__']['complete']
def score(objects):return objects['__evaluation__']['score']
def distance(objects):return objects['__evaluation__']['distance_m']
def demo():return {}
