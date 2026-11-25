"""Four-arm plan for the iStarUSA D-118V2-ITX on TC-RAIL-20 rails (short body, chassis members overhanging its rear).

e = how far the chassis members' rear tips are past the front posts' front faces (m). Loaded phases end at e = 0.06,
after which the demo ramps its arms' pre-load down and the rails carry the chassis; later phases are pushes.
Per waypoint: IK error, post clearance, same-station arm-arm clearance, horizontal reach, static joint torque (own links
plus the arm's load share) against the motors' continuous (DM4340 9 N m, DM4310 3 N m) and peak (27, 7 N m) ratings.

Run: python plan2.py [X d]   (X = station half-spacing, d = rack front ahead of the stations' midline)
"""
import math
import sys

import numpy as np

import fastik as F
from plan import gravity_torque, push_rot_theta
from reach import Layout, grip_rot
from sweep import solve_world, post_clearance, chain_world, rx, rzm
from yam_arena import station

TABLE_TOP = 0.7475
W, H, D = 0.4275, 0.0435, 0.2824          # D-118V2-ITX body width (drawing), body height, depth (datasheet)
RAIL_FROM_FRONT, RAIL_LEN = 0.030, 0.470  # chassis member: starts 30 mm behind the front face, 470 long (estimate)
OVERHANG = RAIL_FROM_FRONT + RAIL_LEN - D  # 0.2176 behind the body's rear face
MASS = 6.4
WEIGHT = MASS * 9.81
U, RACK_BASE, SERVER_U = 0.04445, 0.052, 3
Z_RACKED = TABLE_TOP + RACK_BASE + (SERVER_U - 1) * U   # chassis underside when racked
Z_STAND = TABLE_TOP + 0.060
GRASP_IN = 0.045                           # grasp centres 45 mm in from the body's front and rear faces
EDGE_INSET = 0.015
E_START = -0.10                            # rail tips 100 mm short of the posts' faces on the stand
CONT = np.array([9.0, 9.0, 9.0, 3.0, 3.0, 3.0])
PEAK = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])
LINK_R = 0.02


def front_face(L, e):
    return L.y_f + e - (RAIL_FROM_FRONT + RAIL_LEN)


def grasp(L, agent, role, e, z_bottom):
    sx = -1 if agent == 'a' else 1
    ff = front_face(L, e)
    y = ff + GRASP_IN if role == 'front' else ff + D - GRASP_IN
    return np.array([sx * (W / 2 - EDGE_INSET), y, z_bottom + H / 2])


def grip_candidates(agent):
    out = []
    for yaw in (0, 20, -20, 40, -40, 60, -60):
        for roll in (math.pi, 0.0):
            for pitch in (0, 25, -25):
                out.append((f'y{yaw}p{pitch}r{roll:.1f}', grip_rot(agent, yaw) @ rx(math.radians(pitch)) @ rzm(roll)))
    return out


def arm_gap(L, agent, qa, qb):
    """Closest distance between the two arms of one station (link chains as 40 mm capsules)."""
    sa, sb = ('right', 'left') if agent == 'a' else ('left', 'right')
    Pa, Pb = chain_world(L, agent, sa, qa), chain_world(L, agent, sb, qb)
    d = 1e9
    for i in range(2, len(Pa) - 1):
        for j in range(2, len(Pb) - 1):
            for t in np.linspace(0, 1, 9):
                p = Pa[i] + t * (Pa[i + 1] - Pa[i])
                for u in np.linspace(0, 1, 9):
                    q = Pb[j] + u * (Pb[j + 1] - Pb[j])
                    d = min(d, np.linalg.norm(p - q))
    return d - 2 * LINK_R


def solve_best(L, agent, side, cands, q_prev, load_n):
    best = None
    for tag, p, R in cands:
        q, pe, re = solve_world(L, agent, side, p, R, q_prev)
        if pe * 1000 >= 2 or math.degrees(re) >= 2:
            continue
        clr = post_clearance(L, agent, side, q)
        tau = np.abs(gravity_torque(q, np.array(station.BASES[side]), load_n))
        cont = float(np.max(tau / CONT))
        key = (clr > 0.03, -cont)
        if best is None or key > best[0]:
            best = (key, (q, clr, tag, p, pe, re, tau))
    return None if best is None else best[1]


def run(X, d):
    L = Layout(X, d)
    arm_for = {('a', 'front'): 'right', ('a', 'rear'): 'left', ('b', 'front'): 'left', ('b', 'rear'): 'right'}
    phases = [('grasp on stand', E_START, Z_STAND, 'grip', True), ('lifted', E_START, Z_RACKED + 0.002, 'grip', True),
              ('stab start', 0.0, Z_RACKED + 0.002, 'grip', True), ('engaged 0.06', 0.06, Z_RACKED + 0.002, 'grip', True),
              ('rails carry, hold to 0.15', 0.15, Z_RACKED + 0.002, 'grip', False)]
    rows = []
    for agent in ('a', 'b'):
        sx = -1 if agent == 'a' else 1
        q_prev = {r: None for r in ('front', 'rear')}
        for name, e, z, kind, loaded in phases:
            qs = {}
            for role in ('front', 'rear'):
                if name.startswith('rails carry') and role == 'rear':
                    continue
                side = arm_for[(agent, role)]
                p = grasp(L, agent, role, e, z)
                load = WEIGHT / 4 if loaded else 0.15 * WEIGHT / 4
                r = solve_best(L, agent, side, [(t, p, R) for t, R in grip_candidates(agent)], q_prev[role], load)
                rows.append((agent, side, role, name, p, r, load))
                if r:
                    q_prev[role] = r[0]
                    qs[role] = r[0]
            if len(qs) == 2:
                rows.append((agent, 'both', 'gap', name, None, arm_gap(L, agent, qs['front'], qs['rear']), 0))
        # pushes on the front face (unloaded): front arm to e = 0.30, rear arm from 0.30 to home (0.50)
        for role, es in (('front', (0.15, 0.22, 0.30, 0.36, 0.40, 0.43)), ('rear', (0.43, 0.47, 0.50))):
            side = arm_for[(agent, role)]
            for e in es:
                ff = front_face(L, e)
                cands = []
                for theta in (70, 55, 40, 85):
                    for roll in (math.pi, 0.0):
                        for xc in (0.13, 0.17):
                            a = np.array([math.cos(math.radians(theta)) * (-sx), math.sin(math.radians(theta)), 0])
                            contact = np.array([sx * xc, ff, Z_RACKED + H / 2])
                            cands.append((f'push th{theta} x{xc}', contact - 0.013 * a, push_rot_theta(agent, theta, roll)))
                r = solve_best(L, agent, side, cands, q_prev[role], 0.0)
                rows.append((agent, side, role, f'push front face e={e:.2f}', cands[0][1], r, 0.0))
                if r:
                    q_prev[role] = r[0]
    return L, rows


def report(L, rows):
    lines, ok, minclr, maxreach, maxcont, maxpeak, mingap = [], True, 9, 0, 0, 0, 9
    for agent, side, role, name, p, r, load in rows:
        if role == 'gap':
            lines.append(f'{agent} same-station arm gap at {name}: {r:+.3f} m')
            mingap = min(mingap, r)
            continue
        if r is None:
            lines.append(f'{agent}-{side:5s} {role:5s} {name:28s} NO SOLUTION')
            ok = False
            continue
        q, clr, tag, pp, pe, re, tau = r
        base = L.base_world(agent, side)
        reach = float(np.hypot(*(pp - base)[:2]))
        cont, peak = tau / CONT, tau / PEAK
        lines.append(f'{agent}-{side:5s} {role:5s} {name:28s} tcp=({pp[0]:+.3f},{pp[1]:+.3f},{pp[2]:.3f}) reach={reach:.3f} '
                     f'{tag:18s} ik={pe*1000:.2f}mm post_clear={clr:+.3f} load={load/9.81:.2f}kg '
                     f'tau={np.round(tau,1).tolist()} N m  /cont={np.round(cont,2).tolist()} /peak max={peak.max():.2f}')
        minclr, maxreach = min(minclr, clr), max(maxreach, reach)
        maxpeak = max(maxpeak, float(peak.max()))
        if load > 0.5 * 9.81:
            maxcont = max(maxcont, float(cont.max()))
    s = (f'SUMMARY X={L.X} d={L.d} all_ok={ok} min_post_clear={minclr:+.3f} min_same_station_gap={mingap:+.3f} '
         f'max_reach={maxreach:.3f} loaded_max_tau/continuous={maxcont:.2f} max_tau/peak={maxpeak:.2f} '
         f'overhang={OVERHANG:.3f} z_racked={Z_RACKED-TABLE_TOP:.3f}')
    lines.append(s)
    return lines, s


if __name__ == '__main__':
    if len(sys.argv) >= 3:
        if len(sys.argv) == 4:
            E_START = float(sys.argv[3])
        L, rows = run(float(sys.argv[1]), float(sys.argv[2]))
        lines, s = report(L, rows)
        txt = '\n'.join(lines)
        print(txt)
        open(f'plan2_{L.X:.2f}_{L.d:.2f}_{-E_START:.2f}.txt', 'w').write(txt + '\n')
    else:
        out = []
        for X in (0.40, 0.45):
            for d in (0.38, 0.45, 0.52):
                L, rows = run(X, d)
                _, s = report(L, rows)
                print(s, flush=True)
                out.append(s)
        open('plan2_sweep.txt', 'w').write('\n'.join(out) + '\n')
