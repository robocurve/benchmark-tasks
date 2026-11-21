"""Fixed-layout RGB-only rehearsal: pinch snap body, hook D-ring, clear."""
import numpy as np
import spec


def plan(ik, constants, station):
    snap = np.array(spec.SNAP_START)
    pinch = snap + np.array([0., 0., -0.008])
    ring = np.array([spec.COLLAR_START[0], spec.COLLAR_START[1],
                     spec.COLLAR_START[2] + spec.DRING_LOCAL[2]])
    ready_l = np.array([.26, .22, .76])
    ready_r = np.array([.26, -.28, .76])
    clear_l = np.array([.22, .30, .80])
    clear_r = np.array([.20, -.32, .82])
    high = pinch + np.array([0., 0., .10])
    carry_z = max(pinch[2] + .08, ring[2] + .06)
    rows = [
        (0, ready_l, ready_r, .075, .075),
        (3, ready_l, high, .075, .075),
        (6, ready_l, pinch + [0, 0, .05], .075, .065),
        (9, ready_l, pinch + [0, 0, .02], .075, .035),
        (12, ready_l, pinch, .075, .000),
        (22, ready_l, pinch, .075, .000),
        (26, ready_l, np.array([pinch[0], pinch[1], carry_z]), .075, .000),
        (34, ring + [0, .12, .05], np.array([ring[0], -0.12, carry_z]), .050, .000),
        (42, ring + [0, .10, .03], np.array([ring[0], -0.04, carry_z]), .040, .000),
        (48, ring + [0, .08, .03], np.array([ring[0], -0.01, carry_z]), .040, .000),
        (54, ring + [0, .08, .03], ring + [0, -0.01, .02], .040, .000),
        (60, ring + [0, .08, .03], ring + [0, 0.00, .00], .040, .000),
        (70, ring + [0, .08, .03], ring + [0, 0.00, .00], .040, .000),
        (74, ring + [0, .12, .06], ring + [0, -.08, .04], .075, .055),
        (78, clear_l, clear_r, .075, .075),
        (84, ready_l, ready_r, .075, .075),
    ]


    q = {s: np.array(constants.READY) for s in station.BASES}
    result, errors = [], []
    rng = np.random.default_rng(53)
    for sec, lp, rp, lg, rg in rows:
        poses = {}
        for side, p, gap in [('left', lp, lg), ('right', rp, rg)]:
            p = np.asarray(p, dtype=float)
            R = np.diag([-1., 1., -1.]) if side == 'left' else np.diag([1., -1., -1.])
            candidates = []
            for seed in [q[side], constants.READY, *[rng.uniform(ik.LOWER, ik.UPPER) for _ in range(28)]]:
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
