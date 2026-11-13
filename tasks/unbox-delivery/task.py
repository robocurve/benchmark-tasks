"""Unbox a taped delivery parcel using a guarded cutter.

A plain brown cardboard parcel is held in a packing fixture at world (0.27, 0).
Its plain top flaps meet at y=0, with no handles or grasp tabs. A 48 mm tape
strip spans the 22 cm top seam and wraps 40 mm down both end walls. Cut the
long seam and the two taped end folds to free the flaps.
The orange guarded cutter is on the left; the blue 4 cm item is inside.
Cut the whole seam, return the cutter to its orange pad, physically open both
flaps, and put the item on the green pad on the right. Use run.py: it installs
hinges and the tape fracture approximation and records measured object poses.
"""
from yam_arena.task_file import Box
from config import *
SCENE = 'station'
SUCCESS_HOLD_S = 0.
SCORE_OVER = 'end'
BROWN = (.58, .36, .16)
OBJECTS = [
    Box('bottom', pos=(*CENTER, TOP + WALL/2), size=(LENGTH, WIDTH, WALL), color=BROWN),
    Box('front', pos=(CENTER[0]-LENGTH/2+WALL/2, 0., TOP+HEIGHT/2), size=(WALL, WIDTH, HEIGHT), color=BROWN),
    Box('back', pos=(CENTER[0]+LENGTH/2-WALL/2, 0., TOP+HEIGHT/2), size=(WALL, WIDTH, HEIGHT), color=BROWN),
    Box('left_wall', pos=(CENTER[0], WIDTH/2-WALL/2, TOP+HEIGHT/2), size=(LENGTH, WALL, HEIGHT), color=BROWN),
    Box('right_wall', pos=(CENTER[0], -WIDTH/2+WALL/2, TOP+HEIGHT/2), size=(LENGTH, WALL, HEIGHT), color=BROWN),
    Box('left_flap', pos=(CENTER[0], FLAP_CENTER_Y, LID_Z), size=(FLAP_LENGTH, FLAP_WIDTH, WALL), mass=.018, color=BROWN),
    Box('right_flap', pos=(CENTER[0], -FLAP_CENTER_Y, LID_Z), size=(FLAP_LENGTH, FLAP_WIDTH, WALL), mass=.018, color=BROWN),
    Box('front_tape',pos=(SEAM[0]-.0002,0.,LID_Z-TAPE_DROP/2),size=(.0003,TAPE_WIDTH,TAPE_DROP),color=(.34,.20,.08)),
    Box('back_tape',pos=(SEAM[1]+.0002,0.,LID_Z-TAPE_DROP/2),size=(.0003,TAPE_WIDTH,TAPE_DROP),color=(.34,.20,.08)),
    Box('cutter', pos=CUTTER_START, size=(.14,.025,.025), mass=.055, color=(1.,.28,.03)),
    Box('item', pos=(*CENTER,TOP+WALL+.02), size=(.04,.04,.04), mass=.06, color=(.08,.35,.9)),
    Box('cutter_pad',pos=(.13,.19,TOP+.001),size=(.17,.07,.002),color=(.9,.35,.08)),
    Box('cutter_cradle',pos=(.13,.19,TOP+.00575),size=(.08,.04,.0075),color=(.75,.25,.04)),
    Box('destination',pos=(*DESTINATION,TOP+.001),size=(.10,.10,.002),color=(.1,.6,.22)),
]
_tracker = None
_diagnostic = False

def success(objects):
    if _tracker is None:
        raise RuntimeError('Use run.py to install the hinged box and tape mechanics')
    return not _diagnostic and _tracker.value == 1.

def score(objects):
    return _tracker.value if _tracker and not _diagnostic else 0.

def demo():
    """Cutting prefix; run.py's feedback policy opens the plain flap edges."""
    from yam_arena.demo import Waypoint as W
    from yam_arena.constants import FINGER_OPEN as O
    from math import pi
    grip=-.011
    hover=TOP+.17
    cut_hover=LID_Z+.036
    cut_height=LID_Z+.029
    seam_y=-.002  # Compensate the measured held-tool lateral offset at seam entry.
    x,y,z=CUTTER_START
    left=[W(2,(x,y,hover),O),W(2,(x,y,z+.006),O),W(1,(x,y,z+.006),grip),
          W(2,(x,y,hover),grip),W(2,(SEAM[0]-.070,seam_y,LID_Z+.06),grip),
          W(2,(SEAM[0]-.070,seam_y,cut_height),grip),
          W(6,(SEAM[1]-.045,seam_y,cut_height),grip),
          W(1,(SEAM[1]-.045,seam_y,cut_hover),grip)]
    # Rotate the cutter so its blade runs along each transverse end-fold cut.
    for xx in END_CUT_X:
        left += [W(2,(xx,.100,cut_hover),grip,yaw=-pi/2),
                 W(1.5,(xx,.100,cut_height),grip,yaw=-pi/2),
                 W(2,(xx,.025,cut_height),grip,yaw=-pi/2),
                 W(1,(xx,.025,cut_hover),grip,yaw=-pi/2)]
    left += [W(2,(x,y,hover),grip),W(2,(x,y,z+.006),grip),
             W(1,(x,y,z+.006),O),W(2,(x,y,hover),O)]
    return {'left':left}
