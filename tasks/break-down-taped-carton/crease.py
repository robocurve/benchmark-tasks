"""Physical hinge angles and explicit USD angular-drive units (CPU only)."""
from math import pi
BOTTOM_CENTER=137.45
BOTTOM_LOW=0.
BOTTOM_HIGH=274.9
BOTTOM_OFFSET=.006


def physical_bottom_angle(raw):
    """Outward-only hinge branch; raw -90 denotes the outer 270-degree fold."""
    return (float(raw)-BOTTOM_CENTER+180.)%360.-180.+BOTTOM_CENTER


def joint_parameters(hinge,sealed=False):
    center=hinge.get('frame_center',0.)
    low=hinge['low']-center;high=hinge['high']-center
    if sealed and hinge['sealed']:low=high=-center
    factor=pi/180. if hinge.get('drive_units')=='radians' else 1.
    return dict(center=center,low=low,high=high,target=hinge['target']-center,
                stiffness=hinge['spring']*factor,damping=hinge.get('damping',.001)*factor,
                collision_enabled=hinge['b'].endswith('_top'))
