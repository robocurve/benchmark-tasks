"""Fit each pen with a cap of the same color. Put the two red pens on the left
and the two blue pens on the right.

Four thick pens and four separate matching hollow caps start in a fixed
scattered layout. A cap retains only after measured aligned insertion and
seating; the retention joint preserves the reached pose. Pens rest flat in
the two printed destination areas.
"""
import math
from yam_arena.task_file import Box
from yam_arena.station import TABLE_TOP as TOP
from pen_capping.scoring import History
SCENE='station'
SCORE_OVER='end'
COLORS=('red','blue','red','blue')
PENS=[(.16,-.16,TOP+.010,math.radians(-100)),(.29,-.15,TOP+.010,math.radians(-80)),(.42,-.16,TOP+.010,math.radians(-65)),(.21,-.34,TOP+.010,math.radians(-110))]
CAPS=[(.18,.15,TOP+.0163,math.radians(-120)),(.38,.32,TOP+.0163,math.radians(-105)),(.40,.10,TOP+.0163,math.radians(-145)),(.24,.05,TOP+.0163,math.radians(-125))]
DESTINATIONS={'red':(.30,.23,.24,.20),'blue':(.30,-.23,.24,.20)}
OBJECTS=[Box(name='area_'+color,pos=(x,y,TOP+.0003),size=(sx,sy,.0001),color=(.9,.8,.8)) for color,(x,y,sx,sy) in DESTINATIONS.items()]
for i,(color,p,c) in enumerate(zip(COLORS,PENS,CAPS)):
 rgb=(.75,.045,.035) if color=='red' else (.035,.16,.80)
 OBJECTS += [Box(name=f'pen_{i}',pos=p[:3],yaw=p[3],size=(.1652,.0194,.0194),mass=.035,color=rgb),Box(name=f'cap_{i}',pos=c[:3],yaw=c[3],size=(.035,.0321,.0321),mass=.008,color=rgb)]
HISTORY=History(COLORS)
def score(objects):return HISTORY.score()
def demo():
 from pen_capping.policy import make_plan
 return make_plan(TOP,PENS,CAPS,COLORS)
