"""Scripted bimanual demo; all poses command robot joints through IK."""
from fixture_geometry import TABLE_Z as Z, FILM_Z


def demo():
    from yam_arena.demo import Waypoint as W
    x,y=.36,-.24
    h=Z+.083
    # Measured grip: the tips sit about 80 mm below the TCP.
    # Centre the approach over the measured three-reset acquisition variation.
    # The 5.6 mm notch keeps clearance for the finite calibrated pose range.
    tx,ty=.2807,-.052
    grasp=FILM_Z-.007+.080
    peel=FILM_Z+.018+.080
    pinch=-.004
    right=[W(3,(x,y,h+.045),-.020,'approach stand'),
           W(2,(x,y,h),-.020,'descend'),
           W(2,(x,y,h),-.0067,'grasp tool'),
           W(3,(x,y,h+.04),-.0067,'lift tool'),
           W(3,(tx,ty,grasp+.025),-.0067,'approach slot'),
           W(2,(tx,ty,grasp+.025),-.0067,'settle above slot'),
           W(4,(tx,ty,grasp),-.0067,'insert tips'),
           W(2,(tx,ty,grasp),pinch,'pinch tab'),
           W(3,(tx,ty,peel),pinch,'lift tab from slot'),
           W(15,(.28,-.31,peel+.035),pinch,'peel film'),
           W(2,(.28,-.31,peel+.06),pinch,'lift clear'),
           W(3,(.17,-.35,peel+.06),pinch,'carry film'),
           W(3,(.17,-.40,Z+.088),pinch,'lay film on table'),
           W(2,(.17,-.40,Z+.088),-.0067,'release film'),
           W(3,(.17,-.40,Z+.123),-.0067,'lift empty tool'),
           W(3,(x,y,h+.04),-.0067,'return above stand'),
           W(3,(x,y,h),-.0067,'lower into stand'),
           W(2,(x,y,h),-.020,'release tool'),
           W(3,(x,y,h+.110),-.020,'withdraw above tool'),
           W(3,(.20,-.40,Z+.193),-.020,'clear stand'),
           W(2,(.20,-.40,Z+.13),-.020,'settle')]
    left=[W(5,(.30,.115,Z+.080),0.,'approach carrier'),
          W(3,(.30,.115,Z+.014),0.,'press carrier'),
          W(41,(.30,.115,Z+.014),0.,'hold carrier'),
          W(3,(.30,.115,Z+.13),0.,'withdraw'),
          W(2,(.30,.115,Z+.13),0.,'settle')]
    return dict(left=left,right=right)
