"""Offline reach check (no simulator): run the demo's own grasp and fin planning on several seeds' layouts, with the
nominal grasp, and print the worst IK miss (mm + degrees) of the brace and of each beater's pick-to-search path."""

import sys

import numpy as np

sys.path.insert(0, ".")
import hand_mixer_beaters as T  # noqa: E402
import mixdemo as D  # noqa: E402

worst = 0.0
for seed in [int(a) for a in sys.argv[1:]] or [1, 2, 3, 4, 5]:
    lay = T.layout(seed)
    P = {n: (np.asarray(v[0], float), T.bodykit.rotation(v[1])) for n, v in lay.items() if not n.startswith("_")}
    P["_spin"] = {k: lay[f"_spin_{k}"] for k in T.SOCKETS}
    behind, press, srot = D._strut(T, P)
    out = [f"seed {seed}: brace {D._chain_err(None, [(behind + np.array([0, 0, 0.03]), srot), (behind, srot), (press, srot)], 'left'):.1f}"]
    for kind, sock in D.ORDER:
        pb, rb = P[f"beater_{kind}"]
        pb = pb + np.array([0.0, 0.0, T.REST_Z - 0.006])  # the lying stem's axis height at the grip (about 10 mm)
        P[f"beater_{kind}"] = (pb, rb)
        g = pb + rb @ np.array([T.GRIP_X, 0.0, 0.0])
        rg = D._plan_grasp(T, P, kind, sock, g)
        # then the insertion choice from the lifted pose, as at the demo's stage 1/5
        at = g + np.array([0.0, 0.0, D.TCP_ABOVE_STEM])
        from yam_arena import ik, station
        from yam_arena.constants import READY

        q, _, _ = ik.solve(at + np.array([0, 0, 0.10]), ik.matrix_to_quat_xyzw(rg), np.array(READY, float),
                           base_pos=station.BASES["right"], iters=200)
        r_t, p_t = rg.T @ rb, rg.T @ (pb - at)
        pm, rm = P["mixer"]
        f = pm + rm @ np.array([T.FACE_X, T.SOCK_Y[sock], T.Z_S])
        errs = [D._chain_err(q, D._insert_poses(T, f, rm[:, 0], rm @ T._rx(np.radians(P["_spin"][sock] + 90 * k)),
                                                r_t, p_t)) for k in range(4)]
        out.append(f"{kind}->{sock} insert {min(errs):.1f}")
        worst = max(worst, min(errs))
    print(" | ".join(out), flush=True)
print(f"worst {worst:.1f}")
