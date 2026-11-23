"""Read the valve, bar hinge angles, contact gaps and withdrawal from native poses."""
import math
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from native.fixture import HEIGHT, STOW

BODIES = ("frame","deck","crate","left_support","right_support","valve_lever","ram_plunger",
          "scissorRB","scissorLB","worker","left_grip","right_grip","valve_knob")


def observations(p):
    if p.keys() != set(BODIES):
        raise ValueError('Missing cart body')
    transforms = {}
    for name, pose in p.items():
        a = np.asarray(pose, dtype=float)
        if a.shape != (7,) or not np.all(np.isfinite(a)) or abs(np.linalg.norm(a[3:])-1) > .002:
            raise ValueError(f'Invalid {name} pose')
        transforms[name] = (a[:3], quat_xyzw_to_matrix(a[3:]))
    def point(name, local):
        origin, rotation = transforms[name]
        return origin + rotation @ local
    def deck_local(xyz):
        origin, rotation = transforms['deck']
        return rotation.T @ (xyz-origin)
    def angle(name):
        v=transforms[name][1][:,2]
        return math.degrees(math.atan2(v[0],v[2]))
    joints, gaps = {}, {}
    for side, leg in [('left','scissorRB'),('right','scissorLB')]:
        bar=side+'_support'
        joints[side] = angle(bar) - angle(leg) + STOW
        corners = [deck_local(point(bar,(x,y,HEIGHT))) for x in (-.0235,.0235) for y in (-.026,.026)]
        under = [q[2] for q in corners if abs(q[0]) <= .4675 and abs(q[1]) <= .25]
        gaps[side] = float(-.019-max(under)) if under else 1.
    hand = min(deck_local(point('worker',(x,.166,z)))[1] for x in (-.183,-.066) for z in (.575,.646))
    return dict(prop_angle_deg=joints, bar_joint_deg=joints, relief_angle_deg=-angle('valve_lever'),
                deck_world_z_m=float(p['deck'][2]), worker_gap_m=float(hand-.250),
                bearing_gaps_m=gaps, ram_gap_m=float(-.019-deck_local(point('ram_plunger',(0,0,.206)))[2]),
                ram_relative_z_m=float(p['ram_plunger'][2]-p['frame'][2]-.6),
                load_shift_m=float(np.linalg.norm(deck_local(transforms['crate'][0])-(.06,0,.125))))
