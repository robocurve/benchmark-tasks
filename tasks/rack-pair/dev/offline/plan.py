"""Full four-arm plan for one layout (X = station half-spacing, d = rack front face ahead of the stations' midline).

Checks per waypoint: IK error, post clearance (links as 40 mm capsules vs post boxes), same-station arm-arm distance,
horizontal reach, static joint torques (own links + carried load share) against the sim effort limits.
Run: python3 plan.py            -> sweep summary (plan_sweep.txt)
     python3 plan.py X d        -> per-waypoint table (plan_X_d.txt)
"""
import sys, math, xml.etree.ElementTree as ET
import numpy as np
import geometry as G
import fastik as F
from reach import Layout, grip_rot, post_boxes, server_center, grasp_point
from sweep import solve_world, post_clearance, chain_world, rzm, rx, YAWS, ROLLS, PITCHES
from yam_arena import station, YAM_URDF
from yam_arena.constants import SHOULDER, WRIST

EFFORT = np.array([SHOULDER['effort']] * 3 + [WRIST['effort']] * 3)
MASS = G.M_SERVER + 2 * G.M_SLIDING_RAIL
WEIGHT = MASS * 9.81

# link inertials for gravity torques
_INERT = {}
for link in ET.parse(YAM_URDF).getroot().findall('link'):
    i = link.find('inertial')
    if i is None: continue
    o = i.find('origin')
    _INERT[link.get('name')] = (float(i.find('mass').get('value')),
                                np.array([float(v) for v in o.get('xyz').split()]) if o is not None else np.zeros(3))
LINK_ORDER = ['link1', 'link2', 'link3', 'link4', 'link5', 'gripper']

def gravity_torque(q, base, load_n):
    frames, axes, pts, p, R = F.fk(q, base)
    tau = np.zeros(6)
    g = np.array([0, 0, -9.81])
    masses = []
    for k, name in enumerate(LINK_ORDER):
        m, c = _INERT[name]
        com = frames[k][:3, :3] @ c + frames[k][:3, 3]
        masses.append((m * g, com))
    extra = 2 * _INERT['tip_left'][0]
    masses.append((extra * g, p))
    masses.append((np.array([0, 0, -load_n]), p))
    for i in range(6):
        for f, x in masses:
            # only links distal to joint i contribute: link k (frame k) is distal to joint i if k >= i
            pass
    # simple: torque_i = sum over distal bodies of axis_i . ((x - pt_i) x f)
    for i in range(6):
        t = 0.
        for k, (f, x) in enumerate(masses):
            body_index = k if k < 6 else 5
            if body_index >= i:
                t += axes[i] @ np.cross(x - pts[i], f)
        tau[i] = t
    return tau

def push_rot_theta(agent, theta, roll):
    inward = 1 if agent == 'a' else -1
    z_t = np.array([inward * math.cos(math.radians(theta)), math.sin(math.radians(theta)), 0.])
    y_t = np.array([0, 0, 1.]); x_t = np.cross(y_t, z_t)
    return np.column_stack([x_t, y_t, z_t]) @ rzm(roll)

def grip_rots(agent):
    return [(f'y{yaw}p{pitch}r{roll:.1f}', grip_rot(agent, yaw) @ rx(math.radians(pitch)) @ rzm(roll))
            for yaw in YAWS for roll in ROLLS for pitch in PITCHES]

def best(L, agent, side, cands, q_prev):
    """cands: list of (tag, p, R). Returns best feasible by clearance."""
    out = None
    for tag, p, R in cands:
        q, pe, re = solve_world(L, agent, side, p, R, q_prev)
        if pe * 1000 < 2 and math.degrees(re) < 2:
            clr = post_clearance(L, agent, side, q)
            if out is None or clr > out[1]:
                out = (q, clr, tag, p, pe, re)
            if clr > 0.06: break
    return out

def load_share(phase_e, role, both_supported):
    """Vertical load on each arm (N). Carry: four arms share equally (level, symmetric).
    After the rear arms let go with rails engaged by e: front arms + rails; rails act at the mouth (y_f)."""
    if both_supported:
        return WEIGHT / 4
    if role == 'rear':
        return 0.0
    # front grasp at front face + 0.05; CoM at front face + D/2; rails reaction at the mouth (front face + D - e)
    a = G.D / 2 - G.FRONT_GRASP_FROM_FRONT          # grasp to CoM
    b = (G.D - phase_e) - G.FRONT_GRASP_FROM_FRONT   # grasp to mouth
    frac = max(0.0, 1 - a / b)
    return WEIGHT * frac / 2

def run(X, d, verbose=False):
    L = Layout(X, d)
    z0 = G.TABLE_TOP + G.STAND_H; zr = G.Z_RACKED_BOTTOM + 0.002
    yc0 = L.y_f - 0.12 - G.D / 2
    rows = []
    for agent in ('a', 'b'):
        sx = -1 if agent == 'a' else 1
        plans = {}
        for role in ('front', 'rear'):
            side = L.arm_for(agent, role); base_l = np.array(station.BASES[side]); q_prev = None
            wps = [('grasp on stand', yc0, z0, None, True), ('lifted level', yc0, zr, None, True),
                   ('stab start', server_center(L, e=0.005), zr, 0.005, True),
                   ('both rails engaged 0.06', server_center(L, e=0.06), zr, 0.06, True)]
            if role == 'front':
                wps += [(f'hold front, push to e={e:.2f}', server_center(L, e=e), zr, e, False) for e in (0.10, 0.15)]
            for name, yc, zb, e, both in wps:
                p = grasp_point(L, agent, role, yc, zb)
                r = best(L, agent, side, [(t, p, R) for t, R in grip_rots(agent)], q_prev)
                rows.append((agent, side, role, name, p, r, load_share(e or 0, role, both))); q_prev = r[0] if r else q_prev
            push_range = (0.15, 0.22) if role == 'front' else (0.22, 0.28, 0.33, G.D)
            for e in push_range:
                ff = L.y_f - G.D + e
                cands = []
                for theta in (70, 55, 40, 85):
                    for roll in (math.pi, 0.):
                        for xc in (0.13, 0.17):
                            a = np.array([math.cos(math.radians(theta)) * (-sx), math.sin(math.radians(theta)), 0])
                            contact = np.array([sx * xc, ff, zr + G.H / 2])
                            cands.append((f'push th{theta} x{xc}', contact - 0.013 * a, push_rot_theta(agent, theta, roll)))
                r = best(L, agent, side, cands, q_prev)
                rows.append((agent, side, role, f'push front face e={e:.3f}', cands[0][1], r, 0.0)); q_prev = r[0] if r else q_prev
    return L, rows

def summarize(L, rows):
    ok = all(r[5] is not None and r[5][1] > 0 for r in rows)
    minclr = min((r[5][1] if r[5] else -9) for r in rows)
    reach = max(np.hypot(*(r[4] - L.base_world(r[0], r[1]))[:2]) for r in rows)
    tau_ratio = 0
    for agent, side, role, name, p, res, load in rows:
        if res is None: continue
        q = res[0]; base_l = np.array(station.BASES[side])
        t = gravity_torque(q, base_l, load)
        tau_ratio = max(tau_ratio, np.max(np.abs(t) / EFFORT))
    return ok, minclr, reach, tau_ratio

def table(L, rows):
    lines = []
    for agent, side, role, name, p, res, load in rows:
        base = L.base_world(agent, side)
        if res is None:
            lines.append(f'{agent}-{side:5s} {role:5s} {name:28s} NO SOLUTION'); continue
        q, clr, tag, pp, pe, re = res
        t = gravity_torque(q, np.array(station.BASES[side]), load)
        lines.append(f"{agent}-{side:5s} {role:5s} {name:28s} tcp=({pp[0]:+.3f},{pp[1]:+.3f},{pp[2]:.3f}) reach_h={np.hypot(*(pp-base)[:2]):.3f} "
                     f"{tag:18s} ik={pe*1000:.2f}mm/{math.degrees(re):.2f}deg post_clear={clr:+.3f} load={load/9.81:.2f}kg "
                     f"tau/limit={np.round(np.abs(t)/EFFORT,2).tolist()}")
    return lines

if __name__ == '__main__':
    if len(sys.argv) == 3:
        X, d = float(sys.argv[1]), float(sys.argv[2]); L, rows = run(X, d)
        lines = table(L, rows); ok, minclr, reach, tr = summarize(L, rows)
        lines.append(f'SUMMARY X={X} d={d} all_ok={ok} min_post_clear={minclr:+.3f} max_reach_h={reach:.3f} max_tau/limit={tr:.2f} lift_mass={MASS:.2f}kg')
        txt = '\n'.join(lines); print(txt); open(f'plan_{X:.2f}_{d:.2f}.txt', 'w').write(txt + '\n')
    else:
        out = []
        for X in (0.45, 0.50, 0.55):
            for d in (0.36, 0.40, 0.44):
                L, rows = run(X, d); ok, minclr, reach, tr = summarize(L, rows)
                s = f'X={X:.2f} d={d:.2f} all_ok={ok} min_post_clear={minclr:+.3f} max_reach_h={reach:.3f} max_tau/limit={tr:.2f}'
                print(s, flush=True); out.append(s)
        open('plan_sweep.txt', 'w').write('\n'.join(out) + '\n')
