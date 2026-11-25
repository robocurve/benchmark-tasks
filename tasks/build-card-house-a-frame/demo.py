"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback.

Each hand pinches its card's outer side edge with the last 15 mm of its pads, below mid-height (fingers pointing in across the
card and 45 degrees down, closing through the card's thickness), lifts it out of its stand, swings it to a 15 degree lean over the felt
with the two top edges 16 mm apart and 6 mm up, brings the top edges together in the air (a foot dragging on the felt
would pivot its card in the grip), lowers both cards onto the felt at once (the cards 24 mm apart sideways, so neither hand's inner finger is in front of
the other card), then opens to 8 mm, waits half a second and slowly draws
its fingers out sideways past the card's outer edge.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
import spec

OPEN = .030
SQUEEZE = 0.
RELEASED = .008
EDGE_IN = .006                       # the pinch point (13 mm short of the pad tips) this far in from the card's outer side edge
GRASP_Z = -.012                      # below the card's middle, in its own frame
# Times (s).
IN_T, GRASP_T, CLOSE_T, LIFT_T, SWING_T, MEET_T, SET_T, OPEN_T, OUT_T, UP_T = (
    2.0, 3.5, 4.5, 5.5, 9.0, 10.5, 11.5, 12.2, 15.2, 16.2)
END = 21.0
SETTLE = .5                          # still, open, before drawing the fingers out
SIDE = {'right': 'near', 'left': 'far'}

# Tool axes in the card's frame: closing through the card (its x), fingers pointing in across its width and
# down at APPROACH_DEG, so each wrist stays above and outside its own card.
APPROACH_DEG = 45.
_c, _s = math.cos(math.radians(APPROACH_DEG)), math.sin(math.radians(APPROACH_DEG))
TOOL = {'near': np.column_stack([(1., 0., 0.), (0., -_s, -_c), (0., _c, -_s)]),
        'far': np.column_stack([(1., 0., 0.), (0., -_s, _c), (0., -_c, -_s)])}


def smooth(u):
    u = min(max(u, 0.), 1.)
    return u ** 3 * (10 - 15 * u + 6 * u * u)


def grasp_local(card):
    s = -1. if card == 'near' else 1.
    return np.array([0., s * (spec.CARD_W / 2 - EDGE_IN), GRASP_Z])


def card_pose(card, t):
    """Planned card centre and lean (deg, about +y) at time t, while it is held."""
    start, _ = spec.start_pose(card)
    start = np.array(start)
    tilt0 = spec.START_TILT_DEG
    lean = spec.LEAN_DEG if card == 'near' else -spec.LEAN_DEG
    s = 1. if card == 'near' else -1.
    lifted = start + [0., 0., .025]
    pre, _ = spec.lean_pose(card, ridge_x=spec.RIDGE_X - s * .008, lift=.006)
    meet, _ = spec.lean_pose(card, lift=.006)
    down, _ = spec.lean_pose(card, lift=-.0005)
    pre, meet, down = map(np.array, (pre, meet, down))
    if t < CLOSE_T:
        return start, tilt0
    if t < LIFT_T:
        return start + smooth(t - CLOSE_T) * (lifted - start), tilt0
    if t < SWING_T:
        u = smooth((t - LIFT_T) / (SWING_T - LIFT_T))
        return lifted + u * (pre - lifted), tilt0 + u * (lean - tilt0)
    if t < MEET_T:
        return pre + smooth((t - SWING_T) / (MEET_T - SWING_T)) * (meet - pre), lean
    return meet + smooth((t - MEET_T) / (SET_T - MEET_T)) * (down - meet), lean


def hand_path(arm, t):
    """Commanded pinch point (m), tool rotation matrix and aperture (m) for an arm at time t (s)."""
    card = SIDE[arm]
    centre, lean = card_pose(card, min(t, SET_T))
    rot = Rotation.from_euler('y', lean, degrees=True).as_matrix()
    tool = rot @ TOOL[card]
    approach = tool[:, 2]
    grasp = centre + rot @ grasp_local(card)
    if t < IN_T:
        start = grasp - .045 * approach + [0., 0., .040]
        pre = grasp - .045 * approach
        return start + smooth(t / IN_T) * (pre - start), tool, OPEN
    if t < GRASP_T:
        pre = grasp - .045 * approach
        return pre + smooth((t - IN_T) / (GRASP_T - IN_T)) * (grasp - pre), tool, OPEN
    if t < CLOSE_T:
        return grasp, tool, OPEN + smooth(t - GRASP_T) * (SQUEEZE - OPEN)
    if t < SET_T:
        return grasp, tool, SQUEEZE
    if t < OPEN_T:
        return grasp, tool, SQUEEZE + smooth((t - SET_T) / (OPEN_T - SET_T)) * (RELEASED - SQUEEZE)
    # Out sideways past the card's outer edge, level and in the card's plane, slowly: the wrists are soft.
    out = grasp + [0., -.040 if card == 'near' else .040, 0.]
    if t < OUT_T:
        return grasp + smooth((t - OPEN_T - SETTLE) / (OUT_T - OPEN_T - SETTLE)) * (out - grasp), tool, RELEASED
    return out + smooth((t - OUT_T) / (UP_T - OUT_T)) * np.array([0., 0., .050]), tool, RELEASED


def plan(ik, constants, station):
    steps = int(END * spec.CONTROL_HZ)
    seeds = {}
    errors = []
    result = []
    for k in range(steps + 1):
        t = k / spec.CONTROL_HZ
        command = {}
        for arm in ('left', 'right'):
            p, tool, gap = hand_path(arm, t)
            quat = ik.matrix_to_quat_xyzw(tool)
            seed = seeds.get(arm, constants.READY)
            q, ep, er = ik.solve(p, quat, seed, station.BASES[arm], iters=400 if k == 0 else 80, damping=.015)
            seeds[arm] = q
            errors.append({'time': t, 'side': arm, 'position_m': float(ep), 'rotation_rad': float(er),
                           'target': p.tolist()})
            f = spec.finger_joint_position(gap)
            command[arm] = [*q, f, f]
        result.append((k, command))
    bad = [e for e in errors if e['position_m'] > .002 or e['rotation_rad'] > .03]
    if bad:
        raise RuntimeError('Unreachable nominal waypoint: ' + str(bad[:8]))
    return result, errors


class Controller:
    def __init__(self, ik, constants, station):
        self.plan, self.errors = plan(ik, constants, station)
        self.observations = []

    def action(self, step, rgb):
        if set(rgb) != {'overhead_cam', 'left_wrist_cam', 'right_wrist_cam'}:
            raise ValueError('Three RGB views required')
        if step % 20 == 0:
            self.observations.append({'step': step})
        k = min(step + 1, len(self.plan) - 1)
        return {s: np.asarray(v) for s, v in self.plan[k][1].items()}
