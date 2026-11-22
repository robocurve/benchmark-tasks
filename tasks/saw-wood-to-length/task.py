"""Clamp a wooden strip in a bench vise and saw it to a specified length.

Own task geometry; ordinary full-size hand tool and bench vise. The cutting law
is a disclosed reduced-order approximation; this file does not imply readiness.
"""
from __future__ import annotations
import pathlib
import sys
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
from partkit import part, on_reset, on_before_reset
from yam_arena.task_file import TABLE_TOP

SCENE='station'
SCORE_OVER='end'
INSTRUCTION='Clamp the wooden strip in the vise, then saw it so the retained piece is 150 mm long, within 1 mm. Leave the saw clear of the wood when finished.'
TOP=TABLE_TOP[SCENE]
WOOD_ORIGIN=(.20,-.03,TOP+.080)
WOOD_SIZE=(.300,.020,.020)
SAW_ORIGIN=(.38,.30,TOP+.090)
GRIP_LOCAL=(0,-.04,0)
STEEL=(.46,.49,.53)
BLUE=(.12,.28,.43)
WOOD_COLOR=(.69,.45,.24)

# A cast bench vise with an 80 mm jaw. The moving jaw and lead screw are free
# rigid mechanisms installed by runtime; they are never moved by the demo.
VISE_ORIGIN=(.245,-.03,TOP)
frame=[((0,0,.012),(.115,.150,.024),BLUE),
       ((0,0,.043),(.090,.100,.038),BLUE),
       ((0,.025,.078),(.080,.030,.036),BLUE),
       ((0,.012,.082),(.080,.004,.025),STEEL),
       ((0,-.062,.052),(.070,.018,.045),BLUE),
       ((0,0,.0775),(.080,.018,.005),STEEL)]
VISE=part('vise',VISE_ORIGIN,frame,mass=None,friction=.65)
JAW=part('jaw',(.245,-.03-.0275,TOP+.082),
         [((0,0,0),(.080,.030,.025),STEEL)],mass=.35,friction=.65,
         visuals=[((0,-.01,-.020),(.065,.050,.020),BLUE)])
# Handwheel rotates about Y. A 6 mm/revolution lead closes a 22.5 mm opening
# to 19.5 mm in half a turn; wood contact determines actual jaw position/force.
KNOB=part('screw',(.245,-.140,TOP+.062),
    [((0,0,0),(.034,.026,.034),(.18,.19,.20))],mass=.08,friction=.9,
    cylinders=[((0,.035,0),.006,.070,STEEL,'Y')],
    visuals=[((.010,-.0135,.010),(.004,.001,.009),(.94,.94,.91))])

STRIP=part('wood',WOOD_ORIGIN,[((.150,0,.010),WOOD_SIZE,WOOD_COLOR)],mass=.060,friction=.6,
           visuals=[((.150,0,.0202),(.001,.020,.0004),(.13,.11,.10))])
# A low support under the waste end keeps the initially unclamped stock level.
REST=part('stock_rest',(.475,-.03,TOP),
    [((0,0,.039),(.035,.060,.078),BLUE),((0,0,.079),(.035,.030,.002),STEEL)],mass=None,friction=.4)

# 200 mm blade, 110 mm grip. Tooth-strip proxy is 1 mm wide. No guide/weld.
SAW=part('saw',SAW_ORIGIN,
    [((0,0,0),(.026,.110,.028),(.80,.31,.08)),
     ((0,-.155,-.014),(.0006,.200,.040),(.67,.70,.73)),
     ((0,-.155,-.0345),(.001,.200,.001),(.44,.46,.49)),
     ((0,-.050,0),(.030,.010,.032),(.15,.16,.18))],mass=.15,friction=.45,
    visuals=[((.0004,-.065-i*.008,-.012),(.0001,.001,.030),(.37,.39,.41)) for i in range(22)])
# Two open slots keep the stored saw upright and permit a straight upward lift.
rack=[]
rack.append(((0,0,.037),(.040,.060,.074),WOOD_COLOR))
for y in [-.10,-.22]:
    rack += [((0,y,.0255),(.025,.025,.051),WOOD_COLOR)]
    for sign in [-1,1]:rack += [((sign*.004,y,.063),(.005,.025,.025),WOOD_COLOR)]
RACK=part('saw_rack',(SAW_ORIGIN[0],SAW_ORIGIN[1],TOP),rack,mass=None,friction=.3)
OBJECTS=[VISE,JAW,KNOB,STRIP,REST,SAW,RACK]

import runtime
JAW.meta['spawn_hook']=runtime.spawn_mechanism
KNOB.meta['spawn_hook']=runtime.spawn_mechanism
SAW.meta['spawn_hook']=runtime.spawn_contact
STRIP.meta['spawn_hook']=runtime.spawn_contact
runtime.install()

@on_before_reset
def clear_runtime():
    runtime.clear()

@on_reset
def initialize(environment):
    runtime.reset(environment)

def score(states):return runtime.score()
def success(states):return score(states)>=1.
def distance(states):return runtime.distance()
