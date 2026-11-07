"""Fixed prototype dimensions in world metres; evaluation/setup data, not policy observations."""

from yam_arena.task_file import Box, TABLE_TOP

TOP = TABLE_TOP['station']
HOME = (0.205, 0.0, TOP + 0.040)
TRAVEL = 0.090
TARGET = 0.065
SLIDER_SIZE = (0.024, 0.050, 0.024)
MASS = 0.080  # complete carriage, tab and printed index; kg
ORANGE = (0.95, 0.34, 0.035)
GREY = (0.48, 0.50, 0.53)
WHITE = (0.92, 0.92, 0.90)
GREEN = (0.03, 0.85, 0.10)


def fixed(name, pos, size, color=GREY):
    return Box(name, pos=pos, size=size, color=color)


# Explicit boxes leave the +y inspection opening and -y operating slot empty.
# The roof and x end walls block the elevated fixed camera's rays to the marks.
FIXED = [
    fixed('fixture_base', (0.25, 0, TOP + .006), (.250, .200, .012)),
    fixed('rail_near', (.25, -.031, TOP + .020), (.150, .008, .008)),
    fixed('rail_far', (.25, .031, TOP + .020), (.150, .008, .008)),
    fixed('stop_home', (.185, 0, TOP + .040), (.008, .050, .024)),
    fixed('stop_end', (.315, 0, TOP + .040), (.008, .050, .024)),
    fixed('housing_x_near', (.139, 0, TOP + .066), (.008, .180, .108)),
    fixed('housing_x_far', (.361, 0, TOP + .066), (.008, .180, .108)),
    fixed('housing_roof', (.25, 0, TOP + .124), (.230, .180, .008)),
    fixed('housing_front_sill', (.25, -.086, TOP + .018), (.214, .008, .012)),
    fixed('housing_front_header', (.25, -.086, TOP + .104), (.214, .008, .032)),
    fixed('scale_backing', (.25, .026, TOP + .072), (.150, .003, .018), WHITE),
    fixed('target_mark', (HOME[0] + TARGET, .0281, TOP + .072), (.012, .001, .018), GREEN),
]
SLIDER = Box('slider', pos=HOME, size=SLIDER_SIZE, mass=MASS, color=ORANGE)
OBJECTS = [*FIXED, SLIDER]

# Child cubes: offsets relative to carriage origin, fixed to the same rigid body.
CHILDREN = [
    ('push_tab', (0, -.087, .005), (.028, .130, .034), ORANGE, True),
    ('position_index', (0, .0256, 0), (.008, .001, .022), (.015, .015, .015), False),
]
PROTECTED = tuple(o.name for o in FIXED)
ALIGNMENT_POINT = (HOME[0] + TARGET, .029, TOP + .060)


def bounds(spec):
    import numpy as np
    return np.asarray(spec.pos) - np.asarray(spec.size) / 2, np.asarray(spec.pos) + np.asarray(spec.size) / 2
