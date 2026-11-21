"""Fixed-layout RGB-only rehearsal: remove fork, park it, press start, clear hands."""
import numpy as np
import spec


def plan(ik, constants, station):
    fork = np.array(spec.FORK_START)
    safe = np.array(spec.FORK_SAFE)
    button = np.array(spec.BUTTON_POS)
    ready_l = np.array([.26, .22, .72])
    ready_r = np.array([.26, -.22, .72])
    # Aim TCP slightly below cube centre so pads clamp the body.
    pinch = fork + np.array([0., 0., -0.008])
    high = pinch + np.array([0., 0., .10])
    # Straight-up retreat after release so we do not nudge the parked fork.
    above_park = safe + np.array([0., 0., .14])
    clear_r = np.array([.18, -.18, .82])
    # Open-pad push along +X into the soft start button.
    btn_pre = np.array([button[0] - .055, button[1], button[2]])
    btn_press = np.array([button[0] + .020, button[1], button[2]])
    rows = [
        (0, ready_l, ready_r, .075, .075),
        (4, ready_l, high, .075, .075),
        (8, ready_l, pinch + [0, 0, .05], .075, .070),
        (12, ready_l, pinch + [0, 0, .02], .075, .035),
        (16, ready_l, pinch, .075, .000),
        (28, ready_l, pinch, .075, .000),
        (32, ready_l, pinch + [0, 0, .06], .075, .000),
        (38, ready_l, safe + [0, 0, .12], .075, .000),
        (44, ready_l, safe + [0, 0, .025], .075, .000),
        (48, ready_l, safe + [0, 0, .012], .075, .000),
        # Settle, open in place, then lift straight up before any lateral move.
        (54, ready_l, safe + [0, 0, .012], .075, .000),
        (58, ready_l, safe + [0, 0, .012], .075, .060),
        (62, ready_l, above_park, .075, .075),
        (66, ready_l, clear_r, .075, .075),
        # Open-pad press to latch, then clear promptly so dwell can finish.
        (68, ready_l, btn_pre, .075, .075),
        (72, ready_l, btn_press, .075, .075),
        (75, ready_l, btn_press, .075, .075),
        (77, ready_l, clear_r, .075, .075),
        (79, ready_l, ready_r, .075, .075),
    ]
    R_left = np.diag([-1., 1., -1.])
    R_right = np.diag([1., -1., -1.])
    q = {s: np.array(constants.READY) for s in station.BASES}
    result, errors = [], []
    rng = np.random.default_rng(41)
    for sec, lp, rp, lg, rg in rows:
        poses = {}
        for side, p, gap in [("left", lp, lg), ("right", rp, rg)]:
            p = np.asarray(p, dtype=float)
            R = R_left if side == "left" else R_right
            candidates = []
            for seed in [q[side], constants.READY, *[rng.uniform(ik.LOWER, ik.UPPER) for _ in range(24)]]:
                trial = ik.solve(p, ik.matrix_to_quat_xyzw(R), seed, station.BASES[side],
                                 iters=400, damping=.012)
                candidates.append(trial)
                if trial[1] < .0005 and trial[2] < .005:
                    break
            q[side], ep, er = min(candidates, key=lambda v: v[1] + .2 * v[2])
            errors.append({"time": sec, "side": side, "position_m": float(ep), "rotation_rad": float(er),
                           "target": p.tolist()})
            f = spec.finger_joint_position(gap)
            poses[side] = [*q[side], f, f]
        result.append((int(sec * 20), poses))
    bad = [e for e in errors if e["position_m"] > .012 or e["rotation_rad"] > .06]
    if bad:
        raise RuntimeError("Unreachable nominal waypoint: " + str(bad[:8]))
    return result, errors


class Controller:
    def __init__(self, ik, constants, station):
        self.plan, self.errors = plan(ik, constants, station)
        self.observations = []

    def action(self, step, rgb):
        if set(rgb) != {"overhead_cam", "left_wrist_cam", "right_wrist_cam"}:
            raise ValueError("Three RGB views required")
        if step % 20 == 0:
            self.observations.append({"step": step})
        for (a, p), (b, q) in zip(self.plan[:-1], self.plan[1:]):
            if step <= b:
                u = np.clip((step - a) / max(1, b - a), 0, 1)
                u = u * u * (3 - 2 * u)
                return {s: (1 - u) * np.array(p[s]) + u * np.array(q[s]) for s in p}
        return {s: np.asarray(v) for s, v in self.plan[-1][1].items()}
