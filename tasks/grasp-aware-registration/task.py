"""Place the registration plate flat on the white board so that each illuminated cyan cross is centred in its matching pink-cornered window. Release the plate and leave it still for at least one second. Keep the boards and screen in place."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from prototype.layout import OBJECTS
SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=0
def success(objects):return objects['__evaluation__']['complete']
def score(objects):return objects['__evaluation__']['score']
def distance(objects):return objects['__evaluation__']['distance_m']
def demo():return {}
