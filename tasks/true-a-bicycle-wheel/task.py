"""Scene adapter; completed trials are judged with scoring.py and JUDGING.md."""
from arena_wheel import WheelAssembly
SCENE = 'station'
SCORE_OVER = 'end'
INSTRUCTION = 'Adjust the spoke nipples to leave the bicycle wheel running straight and freely rotating.'
EXTERNAL_JUDGING = True
OBJECTS = [WheelAssembly('wheel_station', pos=(.35, 0., 1.005), size=(.20, .45, .45))]

def score(objects):
    return 0.

def success(objects):
    return False
