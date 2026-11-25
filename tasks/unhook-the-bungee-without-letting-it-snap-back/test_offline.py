"""Offline check of the scorer on kinematic replays of the scripted plans (no simulator): the demo scores 1.0, the
two let-go controls 0.0. The replay has no cord, so a hook let go while the cord is still stretched is made to fly
back along it (at the speed a 135 N/m line on a 25 g hook gives, about 7 m/s for 9.4 cm of stretch) until it lands
near its partner, as in the simulator; a hook let go with the cord slack drops onto the lid."""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

M = load(str(pathlib.Path(__file__).resolve().parent / "task.py")).module
GEOMS = {n: R.Geom("box", np.array(M.FERRULE) / 2) for n in M.HOOKS}


def _fly(frames):
    """After the frame a hook is let go with the cord stretched, move it towards the other hook at the spring's
    speed (and report that velocity)."""
    other = {M.HOOKS[0]: M.HOOKS[1], M.HOOKS[1]: M.HOOKS[0]}
    flying = {}
    for k in range(1, len(frames)):
        objs, grips, _pts = frames[k]
        h = {n: objs[n] for n in M.HOOKS}
        for n in M.HOOKS:
            if n in flying:
                continue
            M.TRIAL.grippers_override = frames[k - 1][1]
            held_before = M.gripping(frames[k - 1][0][n])
            M.TRIAL.grippers_override = grips
            if held_before and not M.gripping(h[n]) and M.span(h) > M.SLACK_M:
                flying[n] = (k, (M.span(h) - M.REST_M) * math.sqrt(M.K_N_PER_M / M.HOOK_KG))
        for n, (k0, v) in flying.items():
            o, p0 = frames[k][0][other[n]].pos, frames[k0 - 1][0][n].pos
            d = o - p0
            dist = float(np.linalg.norm(d))
            u = d / dist
            t = (k - k0 + 1) / R.HZ
            moving = v * t < dist - 0.03
            objs[n] = R.state(p0 + u * min(v * t, dist - 0.03), objs[n].quat_xyzw, GEOMS[n].half,
                              u * v if moving else np.zeros(3))
    return frames


def run(name):
    M.TRIAL.clear()
    frames = _fly(R.replay(M, M.PLANS[name](), links=True, extra_s=2.0, geoms=GEOMS))
    M.TRIAL.clear()
    return R.run(M, M.TRIAL, frames), M.TRIAL.mem.get("why", ""), {k: round(v, 3) if isinstance(v, float) else v
                                                                   for k, v in M.TRIAL.mem.items()}


if __name__ == "__main__":
    for n in M.PLANS:
        print(n, *run(n))
