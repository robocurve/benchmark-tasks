"""Clutched return springs and measured release conditions for the tabletop trainer."""

SIDES = ("left", "right")
STIFFNESS = 55.0       # N/m, equilibrium at the retracted stop
DAMPING = 18.0         # N s/m, engaged return
END_STROKE = .078      # within 2 mm of the 80 mm withdrawal stop
HOME = .003
# Axis-aligned local bounds of the YAM collision meshes, rounded out to 1 mm.
LINK_BOUNDS = {
    "gripper": ((-.052, -.033, -.011), (.051, .033, .064)),
    "tip_left": ((-.011, -.069, -.091), (.058, .004, .004)),
    "tip_right": ((-.058, -.004, -.091), (.011, .069, .004)),
}

def outside_corridors(lo, hi):
    # Entire stroke of both keys, expanded by >= 4 mm about their extreme surfaces.
    for ylow, yhigh in ((.063,.126),(-.126,-.063)):
        if all(hi[i] >= a and lo[i] <= b for i,(a,b) in enumerate(
            ((-.018,.058),(ylow,yhigh),(.024,.183)))):
            return False
    return True

def grippers_clear(poses):
    """Transform conservative body boxes using actual link poses in the fixture frame."""
    import numpy as np
    for name, origin, rotation in poses:
        low, high = map(np.asarray,LINK_BOUNDS[name])
        center = origin + rotation @ ((low+high)/2)
        radius = np.abs(rotation) @ ((high-low)/2)
        if not np.all(np.isfinite(center+radius)):
            raise ValueError("Missing gripper pose")
        if not outside_corridors(center-radius,center+radius):
            return False
    return True

class ReturnClutch:
    def __init__(self):
        self.active = False
        self.samples = 0

    def update(self, q, digit, bearing, touching, clear):
        eligible = (q["withdraw"] >= END_STROKE and max(digit) < .15
                    and min(q["advance_"+s] for s in SIDES) > .008 and min(bearing) > .30
                    and not touching and clear)
        self.samples = self.samples + 1 if eligible else 0
        engage = not self.active and self.samples >= 2
        self.active |= engage
        return engage
