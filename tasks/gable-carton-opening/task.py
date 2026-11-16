"""Form the pouring spout of the already-unsealed carton, leaving it upright and released."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from carton_probe.geometry import objects
from carton_probe import scoring
OBJECTS=objects()
SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=0.05  # The native scorer itself checks a complete three-second hold.
def score(*args,**kwargs): return scoring.score()
def success(*args,**kwargs): return scoring.AUTO_TERMINATE and scoring.score()>=1.
def distance(*args,**kwargs): return 1.
def demo(*args,**kwargs): return {}
