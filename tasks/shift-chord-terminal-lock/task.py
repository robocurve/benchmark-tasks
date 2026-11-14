"""Fixed eight-key keypad, code card and monitor; each cap is a prismatic plunger."""
import pathlib
import sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import geometry as g
from parts import part,Slider
import tracekit
import scoring
import demo_plan

SCENE='station'
TOP=g.TABLE_TOP
SUCCESS_HOLD_S=1.
SCORE_OVER='end'
keys=[]
for key,xy in g.KEYS.items():
    centre=(xy[0]-g.DECK_XY[0],xy[1]-g.DECK_XY[1],g.CAP_CENTRE_Z)
    keys.append(Slider('digit7' if key=='7' else key,rest=centre,boxes=[(centre,g.CAP_SIZE,g.colour(key))],axis='Z',travel=g.TRAVEL,rate=g.MOD_SPRING if key in g.MODIFIERS else g.KEY_SPRING,mass=g.MOD_MASS if key in g.MODIFIERS else g.KEY_MASS,damping=g.DAMPING))
DECK=part(g.DECK_NAME,(*g.DECK_XY,TOP),boxes=[((0,0,g.DECK_SIZE[2]/2),g.DECK_SIZE,g.BODY)],sliders=keys,mass=g.FIXTURE_MASS,friction=g.FIXTURE_FRICTION)
SCREEN=part(g.SCREEN_NAME,(*g.SCREEN_XY,TOP),boxes=g.SCREEN_BOXES,mass=g.FIXTURE_MASS,friction=g.FIXTURE_FRICTION)
CARD=part(g.CARD_NAME,(*g.CARD_XY,TOP),boxes=[((0,0,g.CARD_SIZE[2]/2),g.CARD_SIZE,g.CREAM)],mass=g.FIXTURE_MASS,friction=g.FIXTURE_FRICTION)
OBJECTS=[DECK,SCREEN,CARD]
score=scoring.score
success=scoring.success
distance=scoring.distance
demo=demo_plan.plans
