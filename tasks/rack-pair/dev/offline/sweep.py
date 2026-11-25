"""Sweep station spacing X and rack offset d; IK feasibility and post clearance per phase for all four arms.

Run: python3 sweep.py [X d]   (no args: sweep). Writes sweep.txt / plan_<X>_<d>.txt when given a layout.
"""
import sys, math, json
import numpy as np
import geometry as G
import fastik as F
from reach import Layout, grip_rot, push_rot, post_boxes, seg_box_dist, server_center, grasp_point
from yam_arena import station
from yam_arena.constants import READY

YAWS = [0, 30, -30, 50, -50]
ROLLS = [math.pi, 0.0]
PITCHES = [0, 25]
SEEDS = [READY, [0.6, 1.3, 1.1, -0.2, 0.3, 0], [-0.6, 1.3, 1.1, -0.2, -0.3, 0], [0.0, 1.8, 1.8, 0.2, 0, 0]]
TOL_MM, TOL_DEG, LINK_R = 2.0, 2.0, 0.040

def solve_world(L, agent, side, p_w, R_w, q_init):
    Rl = L.R[agent].T
    p_l = Rl @ (np.asarray(p_w) - L.origin[agent]); R_l = Rl @ R_w
    base = np.array(station.BASES[side])
    best = None
    for s in ([q_init] if q_init is not None else []) + SEEDS:
        q, pe, re = F.solve(p_l, R_l, base, s)
        sc = pe + 0.05 * re
        if best is None or sc < best[3]: best = (q, pe, re, sc)
        if pe < 1e-3 and re < 1e-2: break
    return best[:3]

def chain_world(L, agent, side, q):
    base = np.array(station.BASES[side])
    frames, axes, pts, p, R = F.fk(q, base)
    P = [base] + pts + [frames[-1][:3, 3], p]
    return np.array([L.origin[agent] + L.R[agent] @ x for x in P])

def post_clearance(L, agent, side, q):
    P = chain_world(L, agent, side, q); d = 1e9
    for lo, hi in post_boxes(L):
        for i in range(len(P) - 1):
            d = min(d, seg_box_dist(P[i], P[i + 1], lo, hi, n=25))
    return d - LINK_R

def rx(a):
    c, s_ = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])

def rzm(a):
    c, s_ = math.cos(a), math.sin(a); return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])

def grip_rots(agent, push=False):
    out = []
    if push:
        for roll in ROLLS:
            out.append((f'push+y r{roll:.1f}', push_rot(agent) @ rzm(roll)))
        return out
    for yaw in YAWS:
        for roll in ROLLS:
            for pitch in PITCHES:
                out.append((f'y{yaw}p{pitch}r{roll:.1f}', grip_rot(agent, yaw) @ rx(math.radians(pitch)) @ rzm(roll)))
    return out

def attempt(L, agent, side, p, rots, q_init):
    res = []
    for tag, R in rots:
        q, pe, re = solve_world(L, agent, side, p, R, q_init)
        ok_ik = pe * 1000 < TOL_MM and math.degrees(re) < TOL_DEG
        clr = post_clearance(L, agent, side, q) if ok_ik else -9
        res.append((ok_ik and clr > 0.0, clr, tag, q, pe, re))
        if ok_ik and clr > 0.03: break
    res.sort(key=lambda r: (not r[0], -r[1]))
    return res[0]

def plan_rows(L):
    z0 = G.TABLE_TOP + G.STAND_H; zr = G.Z_RACKED_BOTTOM + 0.002
    yc0 = L.y_f - 0.12 - G.D / 2
    rows = []
    for agent in ('a', 'b'):
        sx = -1 if agent == 'a' else 1
        for role in ('front', 'rear'):
            side = L.arm_for(agent, role); q_prev = None
            steps = [('grasp on stand', yc0, z0), ('lifted', yc0, zr), ('stab start', server_center(L, e=0.005), zr),
                     ('both rails engaged 0.06', server_center(L, e=0.06), zr)]
            if role == 'front':
                steps += [(f'front support e={e:.2f}', server_center(L, e=e), zr) for e in (0.10, 0.15)]
            for name, yc, zb in steps:
                p = grasp_point(L, agent, role, yc, zb)
                rots = grip_rots(agent)
                ok, clr, tag, q, pe, re = attempt(L, agent, side, p, rots, q_prev)
                q_prev = q
                rows.append((agent, side, role, name, p, ok, tag, pe, re, clr, q))
            if role == 'rear':
                for e, yg in ((0.15, L.y_f - 0.10), (0.19, L.y_f - 0.06)):
                    p = np.array([sx * (G.W / 2 - G.EDGE_INSET), yg, zr + G.H / 2])
                    ok, clr, tag, q, pe, re = attempt(L, agent, side, p, grip_rots(agent), q_prev)
                    q_prev = q
                    rows.append((agent, side, role, f'regrip side e={e:.2f}', p, ok, tag, pe, re, clr, q))
                for e in (0.19, 0.28, G.D):
                    ff = L.y_f - G.D + e
                    p = np.array([sx * 0.19, ff - 0.013, zr + G.H / 2])
                    ok, clr, tag, q, pe, re = attempt(L, agent, side, p, grip_rots(agent, push=True), q_prev)
                    q_prev = q
                    rows.append((agent, side, role, f'push front face e={e:.3f}', p, ok, tag, pe, re, clr, q))
    return rows

def fmt(L, rows):
    out = []
    for agent, side, role, name, p, ok, tag, pe, re, clr, q in rows:
        base = L.base_world(agent, side)
        out.append(f"{agent}-{side:5s} {role:5s} {name:26s} tcp=({p[0]:+.3f},{p[1]:+.3f},{p[2]:.3f}) reach_h={np.hypot(*(p-base)[:2]):.3f} "
                   f"{'OK ' if ok else 'BAD'} yaw={tag} ik={pe*1000:.2f}mm/{math.degrees(re):.2f}deg post_clear={clr:+.3f} q={np.round(q,2).tolist()}")
    return out

if __name__ == '__main__':
    if len(sys.argv) == 3:
        X, d = float(sys.argv[1]), float(sys.argv[2]); L = Layout(X, d); rows = plan_rows(L)
        txt = '\n'.join(fmt(L, rows)); print(txt)
        open(f'plan_{X:.2f}_{d:.2f}.txt', 'w').write(txt + '\n')
    else:
        lines = []
        for X in (0.40, 0.45, 0.50, 0.55):
            for d in (0.34, 0.38, 0.42, 0.46):
                L = Layout(X, d); rows = plan_rows(L)
                bad = [f"{r[0]}-{r[1]} {r[3]}" for r in rows if not r[5]]
                minclr = min(r[9] for r in rows)
                line = f"X={X:.2f} d={d:.2f} ok={len(rows)-len(bad)}/{len(rows)} min_post_clear={minclr:+.3f} bad: {'; '.join(bad)}"
                print(line, flush=True); lines.append(line)
        open('sweep.txt', 'w').write('\n'.join(lines) + '\n')
