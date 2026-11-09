"""Candidate passive registration plate. Original station coordinates, metres."""
import numpy as np
from yam_arena.task_file import Box, TABLE_TOP

TOP = TABLE_TOP['station']
BOARD_TOP = TOP + .020
HOME = (.25, -.20, BOARD_TOP + .009)
GOAL = (.21, -.015, BOARD_TOP + .009)
BOARD_CENTER = (.22, .00)
SCENE_REVISION = 'screen-offset-backlit-v1'
WINDOWS = [(0., -.033), (0., .033)]
GRASPS = {'central': (0., 0., .019), 'end': (.109, 0., .019)}
TCP_HEIGHT = .007
ORANGE = (.95, .32, .025)
GREY = (.60, .64, .69)
PINK = (.95, .04, .45)

# The orange central bridge and end tab are part of the same passive rigid body.
# Window openings: 30 x 10 mm; the plate is 18 mm thick. No mechanical locator.
ROOT_SIZE = (.160, .056, .018)
CHILDREN = [
    ('central_handle', GRASPS['central'], (.034, .028, .020), ORANGE, True),
    ('rim_neg', (0., -.0445, 0.), (.160, .013, .018), GREY, True),
    ('rim_pos', (0., .0445, 0.), (.160, .013, .018), GREY, True),
    ('tab_bridge', (.085, 0., .001), (.014, .020, .016), ORANGE, True),
    ('end_tab', (.109, 0., .001), (.034, .028, .020), ORANGE, True),
    ('end_handle', GRASPS['end'], (.034, .028, .020), ORANGE, True),
]
for i, (_, y) in enumerate(WINDOWS):
    for side in (-1, 1):
        CHILDREN.append((f'side_{i}_{side}', (side*.0475, y, 0.), (.065, .010, .018), GREY, True))
    # Four high-contrast corner references around each actual opening.
    for x in (-.018, .018):
        for dy in (-.008, .008):
            CHILDREN.append((f'corner_{i}_{x}_{dy}', (x, y+dy, .0093), (.006, .006, .0006), PINK, False))
CHILDREN=[(name.replace('-','n').replace('.','p'),pos,size,color,collision)
          for name,pos,size,color,collision in CHILDREN]

FIXED = [
    Box('start_board', pos=(HOME[0], HOME[1], TOP+.010), size=(.27,.17,.020), color=(.16,.30,.50)),
    Box('goal_board', pos=(*BOARD_CENTER, TOP+.010), size=(.27,.20,.020), color=(.88,.88,.83)),
    Box('screen', pos=(.16,-.005,.898), size=(.018,.260,.090), color=(.40,.45,.51)),
    Box('screen_post_left', pos=(.145,.117,.801), size=(.030,.016,.107), color=(.40,.45,.51)),
    Box('screen_post_right', pos=(.145,-.108,.801), size=(.030,.012,.107), color=(.40,.45,.51)),
]
OBJECTS = [*FIXED, Box('plate',pos=HOME,size=ROOT_SIZE,mass=.12,color=ORANGE)]
PROTECTED = ()  # Contact logging is diagnostic; the candidate score is registration.
INSTRUCTION = ('Place the registration plate flat on the white board so that each illuminated cyan cross '
               'is centred in its matching pink-cornered window. Release the plate and leave it '
               'still for at least one second. Keep the boards and screen in place.')

def crosses():
    return [np.array([GOAL[0]+x, GOAL[1]+y, BOARD_TOP+.0005]) for x,y in WINDOWS]

def boxes_at(pose):
    """Private geometry diagnostic only; no sensor or policy channel."""
    p=np.asarray(pose)
    return [('bridge',p,ROOT_SIZE)]+[(n,p+np.asarray(x),s) for n,x,s,_,collision in CHILDREN if collision]
