"""Dev only: summarise a PAN_LOG trace (.npz) from a simulator run: the grip (pan against gripper), the wrist and shoulder
torques, how the dice flew, whether any went through the pan, and how many ended turned.  python dev/trace.py run.npz"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pan_toss import pan as P  # noqa: E402


def rot(q):
    x, y, z, w = q
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)), (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def main(path, verbose=True):
    d = np.load(path)
    t, pan, dice, q, tau, grip = d["t"], d["pan"], d["dice"], d["q"], d["tau"], d["gripper"]
    n = len(t)
    rel, tilt, local, nz = [], [], np.zeros((n, dice.shape[1], 3)), np.zeros((n, dice.shape[1]))
    for k in range(n):
        rp, rg = rot(pan[k, 3:]), rot(grip[k, 3:7])
        rel.append(rg.T @ (pan[k, :3] - grip[k, :3]))  # the pan's origin in the gripper link's frame
        tilt.append(math.degrees(math.atan2(-rp[2, 0], rp[2, 2])))  # nose-down pitch about the pan's y
        for i in range(dice.shape[1]):
            local[k, i] = rp.T @ (dice[k, i, :3] - pan[k, :3])
            nz[k, i] = -rot(dice[k, i, 3:])[2, 2]
    rel = np.array(rel)
    tilt = np.array(tilt)
    held = np.where(pan[:, 2] > pan[0, 2] + 0.02)[0]
    out = {"steps": n, "final_turned": int(d["turned"][-1]), "final_inside": int(d["inside"][-1]), "final_lost": int(d["lost"][-1]),
           "score": float(d["score"][-1]), "peak_robot_food_N": float(d["peak"][-1])}
    if len(held):
        a, b = held[0], held[-1]
        r0 = rot(grip[a, 3:7]).T @ rot(pan[a, 3:])
        ang = [math.degrees(math.acos(max(-1.0, min(1.0, (np.trace((rot(grip[k, 3:7]).T @ rot(pan[k, 3:])).T @ r0) - 1) / 2)))) for k in range(a, b + 1)]
        slip = np.linalg.norm(rel[a:b + 1, :3] - rel[a, :3], axis=1)
        out.update(held_s=(float(t[a]), float(t[b])), slip_mm_max=float(1000 * slip.max()), slip_mm_end=float(1000 * slip[-1]), slip_deg_max=float(max(ang)),
                   slip_deg_end=float(ang[-1]), tau_max=np.round(np.abs(tau[a:b + 1, :6]).max(0), 2).tolist(), lift_max_mm=float(1000 * (pan[:, 2].max() - pan[0, 2])),
                   pan_pitch_range=(float(tilt[a:b + 1].min()), float(tilt[a:b + 1].max())),
                   dice_peak_above_floor_mm=float(1000 * local[a:b + 1, :, 2].max()), dice_lowest_mm=float(1000 * local[:, :, 2].min()),
                   pan_speed_max=float(np.linalg.norm(np.diff(pan[:, :3], axis=0), axis=1).max() / (t[1] - t[0])),
                   pitch_rate_max_deg_s=float(np.abs(np.diff(tilt)).max() / (t[1] - t[0])))
    out["pan_end_offset_mm"] = np.round(1000 * (pan[-1, :3] - pan[0, :3]), 1).tolist()
    out["brown_nz_end"] = np.round(nz[-1], 2).tolist()
    if verbose:
        for k, v in out.items():
            print(f"{k}: {v}")
        print("turned by time:", [(round(float(t[k]), 1), int(d["turned"][k]), int(d["lost"][k])) for k in range(0, n, 20)])
    return out


def per_toss(path, toss_t, length):
    """One line per toss window: how the pan moved, how high the dice went above the cooking surface, how many changed the
    face they rest on, and the running counts."""
    d = np.load(path)
    t, pan, dice, grip = d["t"], d["pan"], d["dice"], d["gripper"]
    faces = np.array(((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)), float)

    def down(k):
        return [int(np.argmin((rot(dice[k, i, 3:]) @ faces.T)[2])) for i in range(dice.shape[1])]

    for j, t0 in enumerate(toss_t):
        a, b = int(np.searchsorted(t, t0 + 1e-6)), min(len(t) - 1, int(np.searchsorted(t, t0 + length - 1e-6)))
        pitch, z, zw, rel = [], [], [], []
        for k in range(a, b + 1):
            rp = rot(pan[k, 3:])
            pitch.append(math.degrees(math.atan2(-rp[2, 0], rp[2, 2])))
            z.append(max((rp.T @ (dice[k, i, :3] - pan[k, :3]))[2] for i in range(dice.shape[1])))
            zw.append(dice[k, :, 2].max())
            rel.append(rot(grip[k, 3:7]).T @ (pan[k, :3] - grip[k, :3]))
        pitch, rel = np.array(pitch), np.array(rel)
        v = np.linalg.norm(np.diff(pan[a:b + 1, :3], axis=0), axis=1) / 0.05
        changed = sum(1 for x, y in zip(down(a), down(b)) if x != y)
        print(f"toss {j + 1}: pitch {pitch.min():6.1f}..{pitch.max():5.1f} deg, rate {np.abs(np.diff(pitch)).max() / 0.05:5.0f} deg/s, pan speed {v.max():4.2f} m/s, "
              f"dice up to {1000 * max(z):4.0f} mm over the floor ({1000 * (max(zw) - dice[a, :, 2].max()):4.0f} mm rise), face changed {changed:2d}/12, "
              f"turned {int(d['turned'][b])}, lost {int(d['lost'][b])}, slip {1000 * np.linalg.norm(rel - rel[0], axis=1).max():4.1f} mm, tau {np.round(np.abs(d['tau'][a:b + 1, 1:4]).max(0), 1).tolist()}")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[2].endswith(".json"):
        import json

        meta = json.load(open(sys.argv[2]))
        tt = meta["toss_t"]
        print(meta)
        per_toss(sys.argv[1], tt, (tt[1] - tt[0]) if len(tt) > 1 else 2.4)
        sys.exit(0)
    for p in sys.argv[1:]:
        print("==", p)
        main(p)
