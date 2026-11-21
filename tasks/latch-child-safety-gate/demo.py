"""Fixed-layout RGB-only rehearsal; offline IK, no live object-pose feedback."""
import math
import numpy as np
import spec


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def door_point(local, yaw):
    hinge = np.array([spec.HINGE_X, spec.OPENING_WIDTH / 2 - spec.FRAME_TUBE, 0.])
    return hinge + rz(yaw) @ np.asarray(local, dtype=float)


def close_tangent(local, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly, _ = np.asarray(local, dtype=float)
    t = np.array([-s * lx - c * ly, c * lx - s * ly, 0.])
    n = np.linalg.norm(t)
    return t / n if n > 1e-6 else np.array([0., 1., 0.])


def plan(ik, constants, station):
    # Push a mid-stile grip point (closer to the hinge than the free-stile handle)
    # so the closing arc stays inside left-arm reach. Then right works the locks.
    retreat_left = np.array([.28, .24, .74])
    ready_right = np.array([.30, -.26, .70])
    # Mid-stile push pad — stays reachable while yaw goes from −35° to −15°.
    # Lever/base settle shut via light drives once the door is closed.
    grip = np.array([.02, -.16, .62])
    rows = []
    rows.append((0, -35, retreat_left, ready_right, .075, .075))
    g0 = door_point(grip, math.radians(-35))
    rows.append((4, -35, g0 - close_tangent(grip, math.radians(-35)) * .04, ready_right, .070, .070))
    rows.append((8, -35, g0, ready_right, .000, .070))
    for i, yaw_deg in enumerate([-30, -25, -20, -15]):
        yaw = math.radians(yaw_deg)
        p = door_point(grip, yaw) + close_tangent(grip, yaw) * .015
        right = ready_right + np.array([.03 * (i + 1), .02 * (i + 1), -.012 * (i + 1)])
        rows.append((12 + i * 5, yaw_deg, p, right, .000, .055))
    # Right finishes the close with reachable +X pushes on the free stile; then
    # both hands clear while lever/base drives and the spring latch settle.
    # Keep pressing the free stile (reachable right targets) so the spring
    # latch can reach the catch; then clear hands while locks dwell.
    rows.extend([
        (36, -10, retreat_left, np.array([.40, -.16, .58]), .050, .020),
        (44, -5, retreat_left + [0, .02, .02], np.array([.44, -.12, .56]), .075, .000),
        (52, -2, retreat_left + [0, .04, .02], np.array([.46, -.10, .54]), .075, .000),
        (60, -1, retreat_left + [.02, .04, .02], np.array([.46, -.10, .54]), .075, .000),
        (72, 0, retreat_left + [.02, .06, .02], np.array([.46, -.10, .54]), .075, .000),
        (88, 0, retreat_left + [.04, .08, .04], np.array([.40, -.18, .68]), .075, .040),
        (105, 0, np.array([.28, .20, .76]), np.array([.30, -.22, .74]), .075, .075),
        (125, 0, np.array([.24, .20, .80]), np.array([.24, -.22, .80]), .075, .075),
    ])
    q = {s: np.array(constants.READY) for s in station.BASES}
    result, errors = [], []
    rng = np.random.default_rng(19)
    for sec, yaw_deg, left_xyz, right_xyz, lg, rg in rows:
        yaw = math.radians(yaw_deg)
        poses = {}
        for side, p, gap in [('left', left_xyz, lg), ('right', right_xyz, rg)]:
            p = np.asarray(p, dtype=float)
            R = np.diag([-1., 1., -1.]) if side == 'left' else np.diag([1., -1., -1.])
            if side == 'left' and gap < .02:
                R = rz(yaw * 0.12) @ R
            candidates = []
            for seed in [q[side], constants.READY, *[rng.uniform(ik.LOWER, ik.UPPER) for _ in range(20)]]:
                trial = ik.solve(p, ik.matrix_to_quat_xyzw(R), seed, station.BASES[side],
                                 iters=400, damping=.015)
                candidates.append(trial)
                if trial[1] < .002 and trial[2] < .02:
                    break
            q[side], ep, er = min(candidates, key=lambda v: v[1] + .25 * v[2])
            errors.append({'time': sec, 'side': side, 'position_m': float(ep), 'rotation_rad': float(er),
                           'target': p.tolist()})
            f = spec.finger_joint_position(gap)
            poses[side] = [*q[side], f, f]
        result.append((int(sec * 20), poses))
    bad = [e for e in errors if e['position_m'] > .015 or e['rotation_rad'] > .08]
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
