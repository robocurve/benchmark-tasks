"""Fixed-layout RGB-only rehearsal: seat cartridge (right), close lid (left), clear."""
import math
import numpy as np
import spec


def plan(ik, constants, station):
    # Keep the vertical insert that reached 0.25 on demo-11, then close the lid with the left arm.
    cart = np.array(spec.CART_START) + np.array([0., 0., -0.006])
    high = cart + np.array([0., 0., .10])
    seat = np.array([spec.PITCHER_START[0], spec.PITCHER_START[1],
                     spec.STAND_TOP + spec.PITCHER_H / 2 + spec.WELL_Z_LOCAL + spec.CART_H / 2])
    # Lid tip while open toward +X; left arm presses it down toward closed (smaller angle).
    hinge = np.array([spec.PITCHER_START[0], spec.PITCHER_START[1],
                      spec.PITCHER_START[2] + spec.LID_HINGE_LOCAL[2]])
    L = spec.LID_SIZE[0] / 2 - .01
    th_open = spec.LID_OPEN * 0.70
    lid_tip = hinge + np.array([L * math.cos(th_open), .04, max(L * math.sin(th_open) - .03, .02)])
    # Push the tip down/forward so the hinge rotates toward closed.
    lid_push = hinge + np.array([L * 0.85, .04, -0.01])
    ready_l = np.array([.26, .22, .76])
    ready_r = np.array([.26, -.22, .76])
    clear_r = np.array([.20, -.30, .78])
    clear_l = np.array([.22, .28, .78])
    rows = [
        (0, ready_l, ready_r, .075, .075),
        (4, ready_l, high, .075, .075),
        (8, ready_l, cart + [0, 0, .05], .075, .060),
        (12, ready_l, cart + [0, 0, .02], .075, .030),
        (16, ready_l, cart, .075, .000),
        (28, ready_l, cart, .075, .000),
        (32, ready_l, cart + [0, 0, .08], .075, .000),
        # Hover above the well, then push straight down and release in place.
        (38, ready_l, seat + [0, -.02, .12], .075, .000),
        (44, ready_l, seat + [0, 0, .08], .075, .000),
        (50, ready_l, seat + [0, 0, .04], .075, .000),
        (56, ready_l, seat + [0, 0, .00], .075, .000),
        (62, ready_l, seat + [0, 0, -.025], .075, .000),
        (68, ready_l, seat + [0, 0, -.025], .075, .000),
        (72, ready_l, seat + [0, 0, -.025], .075, .045),
        # Right arm clears so the left can close the lid without knocking the cartridge.
        (76, ready_l, clear_r, .075, .075),
        (80, lid_tip + [0, .02, .04], clear_r, .055, .075),
        (84, lid_tip, clear_r, .000, .075),
        (88, lid_push, clear_r, .000, .075),
        (92, lid_push + [0.02, 0, -0.02], clear_r, .000, .075),
        (96, clear_l, clear_r, .075, .075),
        (100, ready_l, ready_r, .075, .075),
    ]
    q = {s: np.array(constants.READY) for s in station.BASES}
    result, errors = [], []
    rng = np.random.default_rng(23)
    for sec, lp, rp, lg, rg in rows:
        poses = {}
        for side, p, gap in [('left', lp, lg), ('right', rp, rg)]:
            p = np.asarray(p, dtype=float)
            R = np.diag([-1., 1., -1.]) if side == 'left' else np.diag([1., -1., -1.])
            candidates = []
            for seed in [q[side], constants.READY, *[rng.uniform(ik.LOWER, ik.UPPER) for _ in range(24)]]:
                trial = ik.solve(p, ik.matrix_to_quat_xyzw(R), seed, station.BASES[side],
                                 iters=400, damping=.012)
                candidates.append(trial)
                if trial[1] < .0005 and trial[2] < .005:
                    break
            q[side], ep, er = min(candidates, key=lambda v: v[1] + .2 * v[2])
            errors.append({'time': sec, 'side': side, 'position_m': float(ep), 'rotation_rad': float(er),
                           'target': p.tolist()})
            f = spec.finger_joint_position(gap)
            poses[side] = [*q[side], f, f]
        result.append((int(sec * 20), poses))
    bad = [e for e in errors if e['position_m'] > .018 or e['rotation_rad'] > .10]
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
        for (a, p), (b, q) in zip(self.plan[:-1], self.plan[1:]):
            if step <= b:
                u = np.clip((step - a) / max(1, b - a), 0, 1)
                u = u * u * (3 - 2 * u)
                return {s: (1 - u) * np.array(p[s]) + u * np.array(q[s]) for s in p}
        return {s: np.asarray(v) for s, v in self.plan[-1][1].items()}
