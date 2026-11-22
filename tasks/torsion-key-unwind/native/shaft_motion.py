import math
import os
from yam_arena.task_file import TABLE_TOP
SCENE='station'
X=.335
def demo(*, trim_deg=0.0):
    from yam_arena.demo import Waypoint
    from yam_arena.constants import FINGER_OPEN as O,FINGER_CLOSED as C,PAD_TIP_PAST_TCP
    mode=os.environ.get("SHAFT_CONTROL","reference")
    if mode not in ("reference","alternate","idle","loaded_pull","unload_only"):raise ValueError(mode)
    home={"left":(-.08,.26,.16,O),"right":(-.08,-.26,.16,O)}
    plans={k:[] for k in home}
    def phase(hand,t,pose,label,yaw=0.0):
        for arm in plans:
            x,y,z,f=pose if arm==hand else home[arm]
            plans[arm].append(Waypoint(t,(X+x,y,TABLE_TOP[SCENE]+z+PAD_TIP_PAST_TCP),f,label,yaw if arm==hand else 0.0))
    if mode in ("reference","alternate","unload_only"):
        # Primary grasp carries the input lever along an arc. Alternate pushes its flank.
        push=mode=="alternate";f=C if push else -.008
        def lever(deg,gap=.020):
            ang=math.radians(deg)
            x=-.067*math.cos(ang)-.041*math.sin(ang)
            y=-.067*math.sin(ang)+.041*math.cos(ang)
            return (x-gap*y/.0785,y+gap*x/.0785) if push else (x,y)
        x,y=lever(0,.030)
        approach=C if push else -.018
        phase("left",1.4,(x,y,.145,approach),"above blue handle")
        phase("left",1.,(x,y,.062,approach),"align blue handle")
        x,y=lever(0)
        phase("left",.8,(x,y,.062,f),"engage blue handle")
        if not push:
            phase("left",.6,(x,y,.062,f),"settle opposed hub grasp")
        for ang in (-6,-12,-18,-24,-30,-34 if push else -36):
            x,y=lever(ang)
            yaw=0.0 if push else math.radians(ang)
            phase("left",.52 if push else .70,(x,y,.062,f),"unwind input hub",yaw)
        if not push:
            phase("left",1.0,(x,y,.062,f),"settle hub at rest mark",yaw)
        if push:
            phase("left",.35,(x,y,.062,f),"settle flank push")
            x,y=lever(-34.0-trim_deg)
            phase("left",.90,(x,y,.062,f),"trim flank push")
            phase("left",.40,(x,y,.062,f),"settle corrected hub")
            x,y=lever(-34.0-trim_deg,.032)
        phase("left",.65,(x,y,.062,C if push else -.018),"release blue handle",yaw)
        phase("left",.8,(x,y,.14,C if push else -.018),"withdraw vertically",yaw)
        phase("left",.8,home["left"],"clear fixture")
    if mode not in ("idle","unload_only"):
        for t,pose,label in [(1.4,(-.083,-.063,.16,O),"above service key"),
                             (1.,(-.083,-.063,.094,O),"straddle handle"),
                             (.9,(-.083,-.063,.094,-.008),"grasp key"),
                             (.60,(-.083,-.063,.094,-.008),"settle key grasp"),
                             (1.8,(-.128,-.063,.094,-.008),"withdraw along rail"),
                             (.75,(-.128,-.063,.094,O),"release parked key"),
                             (.85,(-.128,-.063,.16,O),"lift clear")]:
            phase("right",t,pose,label)
    phase("right",1.8,home["right"],"hands clear")
    return plans
