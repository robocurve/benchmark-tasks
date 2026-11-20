"""Portable finite-sample geometry screen for the refined turnover prefix.

Run from this folder with the YAM package available:
    python check_refined_turnover.py --full --out turnover-screen.json

This constructs hypothetical carton configurations for CPU planning only. It
never instantiates an environment, writes actor state, applies force, or claims
native contact/turnover. Samples do NOT certify a continuous collision-free
trajectory. In particular contact angles 66..89 and 146..180 are not screened,
rigid authored flap poses are assumed, and separate acquisition/withdrawal
components do not prove a physically assembled controller sequence.
"""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np
from yam_arena import ik, YAM_URDF
from yam_arena.constants import READY
from yam_arena.urdf import quat_xyzw_to_matrix
from check_turnover import guard_pair, parked_cutter
from config import CENTER, TOP
from model import HEIGHT, WIDTH, BOARD
from scene import layout
from turnover import push_target, rx, solve_target
from turnover_safety import Geometry, BASES
from turnover_targets import wall_target, tip_vertices

SIDES = ('left', 'right')
FINGERS = [[-.012, -.012], [-.012, -.012]]
PRESS = .0002
POSITION_TOL = .0003
ROTATION_TOL = .003
TARGET_MIN = -.002


def finite_joint_bounds(q):
    """Scalar finite/bound checks; array/tuple comparisons cannot bypass them."""
    q = np.asarray(q, dtype=float)
    valid = q.shape == (6,) and bool(np.isfinite(q).all())
    if not valid:
        return False, None
    margins = [min(float(q[i])-float(ik.LOWER[i]),
                   float(ik.UPPER[i])-float(q[i])) for i in range(6)]
    return all(m >= -1e-10 for m in margins), math.degrees(min(margins))


def solve(p, rotation, seed, side, starts):
    p, rotation = np.asarray(p), np.asarray(rotation)
    if not np.isfinite(p).all() or not np.isfinite(rotation).all():
        raise ValueError('nonfinite hypothetical Cartesian target')
    q, pe, re = solve_target(p, rotation, seed, BASES[side], starts=starts)
    bounded, margin = finite_joint_bounds(q)
    valid = bounded and math.isfinite(pe) and math.isfinite(re)
    return np.asarray(q), dict(side=side, q=np.asarray(q).tolist(),
        position_error_m=float(pe), rotation_error_rad=float(re),
        joint_margin_deg=margin, finite_and_within_joint_bounds=bounded,
        ik_pass=bool(valid and pe < POSITION_TOL and re < ROTATION_TOL))


class Screen:
    def __init__(self, starts):
        self.geometry = Geometry()
        self.vertices = tip_vertices(self.geometry)
        self.starts = starts
        self.raw = {p['name']: (np.asarray(p['pos']),
            quat_xyzw_to_matrix([*p['quat'][1:], p['quat'][0]]), p['size'])
            for p in layout()[0]}
        corners = np.asarray(list(itertools.product((-1, 1), repeat=3)))
        self.carton_vertices = np.concatenate([p+(corners*np.asarray(size)/2)@R.T
                                              for p, R, size in self.raw.values()])

    def carton(self, angle):
        R = rx(-math.radians(angle))
        a = math.radians(angle)
        # Same piecewise ideal table-pivot hypothesis as the reviewed search.
        # This is not an observed support trajectory or a physics constraint.
        pivot = WIDTH/2+BOARD
        if angle <= 90:
            y = CENTER[1]+pivot-math.cos(a)*pivot+math.sin(a)*(-BOARD/2)
        else:
            y = (CENTER[1]+pivot+HEIGHT-math.cos(a)*(WIDTH/2+1.5*BOARD)
                 -math.sin(a)*HEIGHT)
        z = TOP+.0002-float((self.carton_vertices@R.T)[:, 2].min())
        origin = np.array([CENTER[0], y, z])
        obstacles = {n: (origin+R@p, R@Q, size)
                     for n, (p, Q, size) in self.raw.items()}
        obstacles.update(parked_cutter())
        return origin, R, obstacles

    def guard(self, qs, obstacles, allowed):
        bounds = [finite_joint_bounds(q) for q in qs]
        if not all(b[0] for b in bounds):
            return False, [], [b[1] for b in bounds]
        guards = guard_pair(self.geometry, qs, FINGERS, obstacles, allowed)
        okay = all(bool(g['safe']) and math.isfinite(g['target_gap_m'])
                   and g['target_gap_m'] >= TARGET_MIN for g in guards)
        return okay, guards, [b[1] for b in bounds]

    def contacts(self, angles):
        rows = []
        seeds = {s: np.array(READY) for s in SIDES}
        for angle in angles:
            origin, R, obs = self.carton(angle)
            qs, solutions = [], []
            for side in SIDES:
                if side == 'left' and angle > 10:
                    q = np.array(READY)
                    bounded, margin = finite_joint_bounds(q)
                    solution = dict(side=side, q=q.tolist(),
                        position_error_m=0., rotation_error_rad=0.,
                        joint_margin_deg=margin, finite_and_within_joint_bounds=bounded,
                        ik_pass=bounded, role='parked_READY')
                else:
                    if side == 'right' and angle >= 90:
                        p, Q = push_target(origin, R, 'bottom', .0128425,
                                           PRESS, roll_deg=90.)
                    else:
                        p, Q = wall_target(origin, R, side, HEIGHT, WIDTH,
                                           self.vertices, PRESS)
                    q, solution = solve(p, Q, seeds[side], side, self.starts)
                    seeds[side] = q.copy()
                qs.append(q); solutions.append(solution)
            allowed = [('north',) if angle <= 10 else (),
                ('south', 'south_top') if angle < 90 else
                ('north_bottom', 'south_bottom', 'east_bottom', 'west_bottom')]
            safe, guards, margins = self.guard(qs, obs, allowed)
            rows.append(dict(stage='contact', angle_deg=angle,
                safe=bool(safe and all(s['ik_pass'] for s in solutions)),
                solutions=solutions, guards=guards, joint_margins_deg=margins))
        return rows

    def acquisition(self, fractions):
        origin = np.array([CENTER[0], CENTER[1], TOP+BOARD/2+.0002])
        obs = {n: (origin+p, Q, size) for n, (p, Q, size) in self.raw.items()}
        obs.update(parked_cutter())
        begin = [np.array(READY), np.array(READY)]
        rows = []
        for stage in ('hover', 'contact'):
            goals, solutions = [], []
            for i, side in enumerate(SIDES):
                p, Q = wall_target(origin, np.eye(3), side, HEIGHT, WIDTH,
                                   self.vertices, PRESS)
                normal = np.array([0., -1. if side == 'left' else 1., 0.])
                if stage == 'hover':
                    p = p-normal*.05
                q, solution = solve(p, Q, begin[i], side, self.starts)
                goals.append(q); solutions.append(solution)
            allowed = [('north',), ('south', 'south_top')] if stage == 'contact' else [(), ()]
            for fraction in fractions:
                qs = [a+fraction*(b-a) for a, b in zip(begin, goals)]
                safe, guards, margins = self.guard(qs, obs, allowed)
                rows.append(dict(stage='acquisition_'+stage, fraction=float(fraction),
                    safe=bool(safe and all(s['ik_pass'] for s in solutions)),
                    endpoint_solutions=solutions, joint_margins_deg=margins,
                    guards=guards))
            begin = [q.copy() for q in goals]
        return rows

    def withdrawal(self, fractions):
        rows = []
        for angle, side in ((25, 'left'), (55, 'right')):
            origin, R, obs = self.carton(angle)
            qs, solutions = [], []
            for s in SIDES:
                if s == 'left' and angle > 25:
                    qs.append(np.array(READY)); continue
                p, Q = wall_target(origin, R, s, HEIGHT, WIDTH,
                                   self.vertices, PRESS)
                q, solution = solve(p, Q, np.array(READY), s, self.starts)
                qs.append(q); solutions.append(solution)
            index = SIDES.index(side)
            p, Q = wall_target(origin, R, side, HEIGHT, WIDTH,
                               self.vertices, PRESS)
            p = p+np.array([0., .05 if side == 'left' else -.05, .05])
            end, solution = solve(p, Q, qs[index], side, self.starts)
            solutions.append(solution)
            for stage, goal in (('withdraw', end), ('ready', np.array(READY))):
                begin = qs[index].copy()
                for fraction in fractions:
                    qs[index] = begin+(goal-begin)*fraction
                    allowed = ([('north',), ('south', 'south_top')]
                               if side == 'left' else [(), ('south', 'south_top')])
                    safe, guards, margins = self.guard(qs, obs, allowed)
                    rows.append(dict(stage='withdrawal_'+stage, angle_deg=angle,
                        side=side, fraction=float(fraction),
                        safe=bool(safe and all(s['ik_pass'] for s in solutions)),
                        endpoint_solutions=solutions, joint_margins_deg=margins,
                        guards=guards))
        return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--full', action='store_true', help='Default:122+82+164 configurations')
    mode.add_argument('--quick', action='store_true', help='Sparse9+18+36 preview, not full screen')
    parser.add_argument('--starts', type=int, default=10)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.starts <= 20:
        parser.error('--starts must be1..20')
    if not math.isclose(HEIGHT, .18) or not np.allclose(CENTER[:2], [.25, -.10]):
        parser.error('This reviewed candidate requires180mm height and CENTERxy=(.25,-.10)')
    screen = Screen(args.starts)
    angles = ([0, 10, 25, 45, 65, 90, 110, 130, 145] if args.quick else
              list(range(66))+list(range(90, 146)))
    fractions = np.linspace(0., 1., 9 if args.quick else 41)
    groups = {}
    for name, call in (('contact', lambda: screen.contacts(angles)),
                       ('acquisition', lambda: screen.acquisition(fractions)),
                       ('withdrawal', lambda: screen.withdrawal(fractions))):
        groups[name] = call()
        print(name, sum(r['safe'] for r in groups[name]), '/', len(groups[name]), flush=True)
    source = Path(__file__).resolve().parent
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(source.glob('*.py'))}
    result = dict(kind='finite_CPU_geometry_screen_only', native_validated=False,
        complete_turnover_demonstrated=False, full_sampling=not args.quick,
        setup=dict(height_m=HEIGHT, center_xy_m=list(CENTER[:2]), table_z_m=TOP,
                   fingers_m=FINGERS, support_press_m=PRESS, ik_starts=args.starts,
                   target_overlap_min_m=TARGET_MIN),
        limitations=['No physics, contact forces, slip, flap settling or table friction verified.',
            'No continuous clearance certificate; only recorded finite configurations.',
            'Contact angles66..89 and146..180 absent; complete turnover not screened.',
            'Left contact retained only through10deg in contact screen, withdrawn separately at25deg.',
            'Separate components do not establish an assembled controller trajectory.'],
        source_sha256=hashes, yam_urdf_sha256=hashlib.sha256(Path(YAM_URDF).read_bytes()).hexdigest(),
        counts={name:len(rows) for name,rows in groups.items()},
        passed={name:sum(r['safe'] for r in rows) for name,rows in groups.items()},
        all_sampled_pass=all(r['safe'] for rows in groups.values() for r in rows),
        groups=groups)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    raise SystemExit(0 if result['all_sampled_pass'] else 1)

if __name__ == '__main__':
    main()
