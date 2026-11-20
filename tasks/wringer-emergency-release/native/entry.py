"""A stationary wringer trainer holds the blue sleeve of an orange surrogate arm.
The yellow crosshead has a lifting tab on each side. Free the sleeve and leave the
arm on the near side with both hands clear. The drive is already stopped.
The horizontal sleeve carriage has passive velocity damping on its guide.
"""
import os
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
if str(HERE) not in sys.path:sys.path.insert(0,str(HERE))
from yam_arena.task_file import TABLE_TOP, Box
from wringer_bridge import install
from reference_guard import install as install_reference_guard
SCENE="station"
X=.35
OBJECTS=[Box("wringer_trainer",pos=(X,0,TABLE_TOP[SCENE]),size=(.001,.001,.001))]
# Keep stepping after the first full score to film the unloaded fixture settling.
SUCCESS_HOLD_S=2.55
SCORE_OVER="end"
monitor=None
TIMING=(1.6,1.3,.8,1.6,1.,2.5,1.,1.1,1.2,1.)
DRAW_START_S=sum(TIMING[:5])
DRAW_END_S=sum(TIMING[:6])
LIFT_START_S=sum(TIMING[:3])
LIFT_END_S=sum(TIMING[:7])
def score(objects):
    if monitor is None:raise RuntimeError("Wringer substep observer is missing")
    return monitor.trial.score
def success(objects):return score(objects)==1.
def demo():
    from yam_arena.demo import Waypoint
    from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP
    mode=os.environ.get("WRINGER_CONTROL","reference")
    if mode not in ("reference","alternate","unsafe","idle","release_only"):raise ValueError("Invalid scripted control")
    top=TABLE_TOP[SCENE]
    # Waypoints address the TCP; the grasp targets are the centres of actual collision tabs.
    # Seat the pads lower beneath the release lip, then give the loaded arm more lift travel.
    ztab=top+.115+PAD_TIP_PAST_TCP
    zarm=top+.0805+PAD_TIP_PAST_TCP
    tab=(X,.104);grip=(X-.131,-.035)
    arm_target=grip[0]-(.091 if mode=="alternate" else .102)
    def at(dt,xy,z,f,label):return Waypoint(dt,(*xy,z),f,label)
    hold=mode in ("reference","alternate","release_only")
    pull=mode in ("reference","alternate","unsafe")
    lhome=(X-.03,.15);rhome=(X-.13,-.13);hover=top+.16+PAD_TIP_PAST_TCP
    # Both trajectories share a time base. Open and retreat after withdrawing the arm.
    timing=TIMING
    dz=.035 if mode=="alternate" else .038
    left=[(tab,hover,O),(tab,ztab,O),(tab,ztab,C),(tab,ztab+dz,C),(tab,ztab+dz,C),(tab,ztab+dz,C),
          (tab,ztab+dz,C),(tab,ztab+dz,O),(lhome,hover,O),(lhome,hover,O)] if hold else [(lhome,hover,O)]*10
    dest=(arm_target,grip[1])
    right=[(grip,hover,O),(grip,zarm,O),(grip,zarm,O),(grip,zarm,O),(grip,zarm,C),(dest,zarm,C),
           (dest,zarm,O),(rhome,hover,O),(rhome,hover,O),(rhome,hover,O)] if pull else [(rhome,hover,O)]*10
    if mode in ('reference','alternate'):
        # Clear the freed dummy vertically before the open fingers travel back across it.
        right[7]=(dest,hover,O)
    if mode=='alternate':
        def mirror(route):
            return [((x,-y),z,f) for (x,y),z,f in route]
        left,right=mirror(right),mirror(left)
    return {k:[at(t,xy,z,f,str(i)) for i,(t,(xy,z,f)) in enumerate(zip(timing,r))] for k,r in (('left',left),('right',right))}
install(HERE/'entry.py')
install_reference_guard(HERE/'entry.py')
