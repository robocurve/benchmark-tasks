"""Move the orange-handled collar along the curved guide from the blue support to the green support, Leave the guide passing through the collar tunnel, and centre the collar upright on the green support, parallel to the support, release it, and leave it still for at least one second. Keep the collar clear of the guide and its anchors; keep both robots clear of the fixture and each other. Finger contact with the collar is allowed. Keep the fixture in place."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from guide_task.layout import OBJECTS
SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=0
def success(objects): return objects['__evaluation__']['complete'] and objects.get('__stop_ready__',True)
def score(objects): return objects['__evaluation__']['score']
def distance(objects): return objects['__evaluation__']['distance_m']
def demo(): return {}
