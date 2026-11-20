"""Tiered, latched, geometry-only score for the drawstring bin-bag task.

Shared by the scripted demo, the Dream-1 run and offline replay. It reads only
recorded geometry (film mesh, waste-block poses, strap-loop link positions, hem
markers, gripper poses and finger openings). Every rule mirrors a check a person
can make with a ruler, a photo or video and a count.

Grasp evidence (gripper frame: approach axis +z, fingers closing along x):
  loop held      the fingers are closed to <= 15 mm and a link of that strap
                 loop lies between the finger pads;
  film gripped   a film node lies between the finger pads while the fingers are
                 closed to <= 30 mm (latched: voids any tier not yet earned);
  film contact   a film node lies within 5 mm of either finger's outline;
  pulled         the string is drawn out of the hem cut-outs by >= 100 mm in
                 total (strap-end-to-cut-out distances, summed over the four
                 strand ends, minus their start values) and is taut (routed
                 string length no shorter than at the start minus 10 mm).

Tiers (highest satisfied applies; failures latch to 0 for the rest of the run):
  0.00  never cinched, any waste block escapes the bag, or invalid/incomplete
        geometry (film edge stretch > 1.5, gaps, NaN).
  0.25  cinched: opening <= 80 mm with all three blocks inside, while both
        loops are held (one per gripper) and pulled, and no film has been gripped.
  0.50  carried: after cinching, the whole bag and its contents are clear of
        the bin (>= 20 mm above its rim) with the opening <= 80 mm, while both
        loops are held and the string taut, neither gripper touches the film
        and no film has been gripped: the load hangs from the loops.
  0.75  set down: at the end the bag rests upright on the tabletop, beside and
        clear of the bin, opening <= 80 mm, contents inside; still held or
        moving.
  1.00  released: the 0.75 state holds for the final two seconds with both
        grippers >= 60 mm from film and drawstring loops and bag bounds moving
        <= 10 mm.
"""
from collections import deque
import numpy as np
from evaluate_removal import contents_retained
from drawstring_physics import SlidingDrawstring

TABLE_Z = .7475
TABLE_LO = np.array([-.19, -.65])
TABLE_HI = np.array([.91, .65])
# A 70 x 100 x 45 mm block's smallest face has an 83 mm diagonal, so it cannot
# pass a mouth opening of 80 mm or less: that is the closure threshold.
CINCH_M = .080          # opening that counts as cinched
CLOSED_M = .080         # opening that must be kept while carried / set down
CLEAR_M = .020          # clearance above rim, and margin to table edges / bin
UPRIGHT_MOUTH_M = .15   # gathered mouth at least this high above the tabletop
SUPPORT_M = .025        # bag bottom within this of the tabletop
RELEASE_M = .060        # grippers away from film and loops
STILL_M = .010          # bag-bounds motion over the final hold
HOLD_STEPS = 40         # two seconds at 20 Hz
# Grasp geometry in the gripper link frame (adapter: TCP 134 mm along +z, pads
# end 13 mm beyond it, fingers close along x).
TCP_Z = .134
PAD_TIP = .013          # pad tips beyond the TCP
PAD_BACK = .030         # pad length behind the TCP counted as "between the pads"
PAD_HALF_W = .012       # half the pad width (gripper y)
PAD_MARGIN = .003       # contact margin beyond the pad faces
FINGER_T = .010         # finger thickness behind each pad face
FINGER_BASE = .070      # finger root along +z
CONTACT_M = .005        # film-contact margin around the finger outline
HELD_OPEN_MAX = .015    # fingers closed on the strap
GRIP_OPEN_MAX = .030    # fingers closed enough to grip film
DRAW_M = .100           # string drawn out of the cut-outs (sum of four strand ends)
TAUT_M = .010           # routed length may be at most this short of its start value
EXIT_INDEX = 5          # hem markers 5 and 15 are the cut-outs


def _local(points, pose):
    pose = np.asarray(pose, float)
    return (np.asarray(points, float).reshape(-1, 3) - pose[:3, 3]) @ pose[:3, :3]


def between_pads(points, pose, opening):
    """Mask of points between a gripper's finger pads."""
    x, y, z = _local(points, pose).T
    z = z - TCP_Z
    return (np.abs(x) <= opening / 2 + PAD_MARGIN) & (np.abs(y) <= PAD_HALF_W) & (z >= -PAD_BACK) & (z <= PAD_TIP + PAD_MARGIN)


def near_fingers(points, pose, opening):
    """Mask of points touching (within CONTACT_M of) a gripper's finger outline."""
    x, y, z = _local(points, pose).T
    return (np.abs(x) <= opening / 2 + FINGER_T + CONTACT_M) & (np.abs(y) <= PAD_HALF_W + CONTACT_M) \
        & (z >= FINGER_BASE) & (z <= TCP_Z + PAD_TIP + CONTACT_M)


def grasp_evidence(s, v, string=None):
    """Per-step grasp facts from recorded geometry (see module docstring)."""
    poses = np.asarray(s['gripper_poses'], float)
    opening = np.asarray(s['finger_opening'], float)
    loops = [np.asarray(x, float).reshape(-1, 3) for x in s['loop_points']]
    if poses.shape != (2, 4, 4) or opening.shape != (2,) or len(loops) != 2 or min(len(x) for x in loops) < 3:
        raise ValueError('grasp evidence missing')
    held = np.array([[bool(opening[g] <= HELD_OPEN_MAX and between_pads(loop, poses[g], opening[g]).any())
                      for g in range(2)] for loop in loops])
    both_held = bool((held[0, 0] and held[1, 1]) or (held[0, 1] and held[1, 0]))
    film_gripped = any(opening[g] <= GRIP_OPEN_MAX and between_pads(v, poses[g], opening[g]).any() for g in range(2))
    film_contact = any(near_fingers(v, poses[g], opening[g]).any() for g in range(2))
    hem = np.asarray(s['hem_marker_positions'], float)
    if hem.shape != (20, 3):
        raise ValueError('hem marker evidence missing')
    ends = np.array([loops[0][0], loops[0][-1], loops[1][0], loops[1][-1]])
    drawn = float(sum(np.linalg.norm(ends[2 * k + i] - hem[c]) for k, c in enumerate((EXIT_INDEX, EXIT_INDEX + 10)) for i in range(2)))
    positions = np.r_[hem, ends]
    if string is None:
        string = SlidingDrawstring(positions, exit_index=EXIT_INDEX)
    routed = string.length_gradient(positions)[0]
    return {'held': held, 'both_held': both_held, 'film_gripped': film_gripped, 'film_contact': film_contact,
            'drawn': drawn, 'routed': routed, 'string': string}


class DrawstringJudge:
    def __init__(self, parameters):
        import trimesh
        from realistic_meshes_v32 import dirty_mesh
        self.top = float(parameters['bin_top_z'])
        self.centre = np.asarray(parameters['bin_center_xy'], float)
        self.radius = float(parameters.get('bin_outer_radius_m', .106))
        self.maximum_gap = int(parameters.get('sample_interval_steps', 1))
        vv, ff, _ = dirty_mesh()
        mesh = trimesh.Trimesh(vertices=vv, faces=ff, process=False)
        self.shape = mesh.vertices.shape
        self.edges = mesh.edges_unique
        self.rest = np.linalg.norm(mesh.vertices[self.edges[:, 1]] - mesh.vertices[self.edges[:, 0]], axis=1)
        self.mouth_ids = np.unique(mesh.edges_unique[np.bincount(mesh.edges_unique_inverse) == 1])
        self.failure = None
        self.count = 0
        self.previous = -1
        self.first_cinch = None
        self.first_carry = None
        self.max_strain = 1.
        self.tail = deque()
        self.last = {}
        self.string = None
        self.drawn0 = None
        self.routed0 = None
        self.film_gripped_step = None

    def fail(self, why, **extra):
        self.failure = {'score': 0., 'full_task_success': False, 'tier': 'failed', 'reason': why,
                        'failure_step': self.previous + 1, **extra}
        return self.failure

    @staticmethod
    def opening(v, ids):
        neck = v[ids]
        return float(np.max(np.linalg.norm(neck[:, None, :] - neck[None, :, :], axis=2)))

    def update(self, s, vertices):
        if self.failure is not None:
            return self.failure
        try:
            step = int(s['step'])
            v = np.asarray(vertices, float)
            if step > 4000:
                return self.fail('Run limit exceeded')
            if step < self.previous or (self.previous >= 0 and step - self.previous > self.maximum_gap):
                return self.fail('Incomplete or unordered motion evidence')
            if v.shape != self.shape or not np.isfinite(v).all():
                return self.fail('Invalid bag mesh evidence')
            mouth = self.opening(v, self.mouth_ids)
            if abs(mouth - float(s['mouth_diameter'])) > .001:
                return self.fail('Mouth measurement inconsistent with recorded mesh')
            if self.count == 0:
                if step != 0:
                    return self.fail('Missing initial state')
                if mouth < .10:
                    return self.fail('Bag did not start open')
                if np.linalg.norm(v[:, :2].mean(0) - self.centre) > .03 or v[:, 2].min() > self.top - .10:
                    return self.fail('Bag did not start inside bin')
            strain = float(np.max(np.linalg.norm(v[self.edges[:, 1]] - v[self.edges[:, 0]], axis=1) / self.rest))
            self.max_strain = max(self.max_strain, strain)
            if strain > 1.5:
                return self.fail('Numerically invalid film deformation; rerun required', trial_valid=False,
                                 max_recorded_edge_stretch_ratio=self.max_strain)
            retained, corners = contents_retained(s, v)
            if not retained:
                return self.fail('Waste block outside the bag' if self.count else 'Initial three waste blocks not retained')
            points = np.concatenate([v, corners])
            g = grasp_evidence(s, v, self.string)
            if self.count == 0:
                self.string, self.drawn0, self.routed0 = g['string'], g['drawn'], g['routed']
            drawn_out = g['drawn'] - self.drawn0
            taut = g['routed'] >= self.routed0 - TAUT_M
            if g['film_gripped'] and self.film_gripped_step is None:
                self.film_gripped_step = step
            clean = self.film_gripped_step is None
            pulled = bool(g['both_held'] and taut and drawn_out >= DRAW_M)
            if mouth <= CINCH_M and self.first_cinch is None and pulled and clean:
                self.first_cinch = step
            above_rim = bool(points[:, 2].min() >= self.top + CLEAR_M)
            lifted_by_loops = bool(g['both_held'] and taut and not g['film_contact'] and clean)
            if self.first_cinch is not None and self.first_carry is None and above_rim and mouth <= CLOSED_M and lifted_by_loops:
                self.first_carry = step
            # Tabletop state.
            loops = np.concatenate([np.asarray(x, float).reshape(-1, 3) for x in s.get('loop_points', [])] or [np.zeros((0, 3))])
            tcp = np.asarray(s.get('tcp', []), float)
            if len(loops) and tcp.shape == (2, 3):
                grasp_points = np.concatenate([v, loops])
                separation = float(np.linalg.norm(tcp[:, None, :] - grasp_points[None, :, :], axis=2).min())
            else:
                separation = 0.
            lo, hi = points[:, :2].min(0), points[:, :2].max(0)
            on_table = bool(np.all(lo >= TABLE_LO + CLEAR_M) and np.all(hi <= TABLE_HI - CLEAR_M))
            beside = bool(np.any(lo >= self.centre + self.radius + CLEAR_M) or np.any(hi <= self.centre - self.radius - CLEAR_M))
            bottom = float(points[:, 2].min())
            mouth_height = float(v[self.mouth_ids, 2].mean() - TABLE_Z)
            set_down = bool(on_table and beside and abs(bottom - TABLE_Z) <= SUPPORT_M
                            and mouth_height >= UPRIGHT_MOUTH_M and mouth <= CLOSED_M)
            released = bool(set_down and separation >= RELEASE_M)
            bounds = np.concatenate([v.min(0), v.max(0)])
            self.tail.append((step, set_down, released, bounds))
            while len(self.tail) > 1 and self.tail[1][0] <= step - HOLD_STEPS:
                self.tail.popleft()
            self.last = {'string_drawn_out_m': drawn_out, 'string_taut': bool(taut), 'loops_held': bool(g['both_held']),
                         'film_contact_now': bool(g['film_contact']), 'film_gripped_step': self.film_gripped_step,
                         'final_mouth_diameter_m': mouth, 'bag_bottom_above_table_m': bottom - TABLE_Z,
                         'mouth_height_above_table_m': mouth_height, 'whole_load_on_table': on_table,
                         'beside_bin': beside, 'minimum_gripper_distance_m': separation,
                         'set_down_now': set_down, 'released_now': released}
            self.previous = step
            self.count += 1
            return self.result()
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            return self.fail('Invalid evidence: ' + str(exc))

    def result(self):
        if self.failure is not None:
            return self.failure
        if self.count < 1:
            return {'score': 0., 'full_task_success': False, 'tier': 'none', 'reason': 'Missing initial state'}
        base = {'first_cinch_step': self.first_cinch, 'first_carry_step': self.first_carry,
                'max_recorded_edge_stretch_ratio': self.max_strain, 'steps': self.previous, **self.last}
        if self.first_cinch is None:
            return dict(base, score=0., full_task_success=False, tier='none',
                        reason='Opening never reached 80 mm while both loops were held and pulled with no film gripped')
        tail = list(self.tail)
        motion = float(np.ptp(np.asarray([x[3] for x in tail]), axis=0).max()) if tail else float('inf')
        full_window = len(tail) >= 2 and tail[0][0] <= self.previous - HOLD_STEPS
        if self.first_carry is not None and full_window and all(x[2] for x in tail) and motion <= STILL_M:
            score, tier = 1., 'released'
        elif self.first_carry is not None and tail and tail[-1][1]:
            score, tier = .75, 'set_down'
        elif self.first_carry is not None:
            score, tier = .5, 'carried'
        else:
            score, tier = .25, 'cinched'
        return dict(base, score=score, full_task_success=score == 1., tier=tier, cinch_verified=True,
                    final_two_second_bag_bounds_motion_m=motion)


def evaluate(trace, snapshots, parameters):
    judge = DrawstringJudge(parameters)
    for s in trace:
        judge.update(s, snapshots[s['step']])
        if judge.failure is not None:
            break
    return judge.result()
