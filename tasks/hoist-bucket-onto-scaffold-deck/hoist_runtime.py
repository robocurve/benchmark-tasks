"""Read-only custom articulation states and the lazy scene hook."""
from dataclasses import dataclass, field
import functools
import importlib.machinery
import sys
import math
import numpy as np
from yam_arena.task_file import ObjectState
from yam_arena.ik import matrix_to_quat_xyzw
from hoist_geometry import HoistRope, rotation, PITCH, RADIUS, GROOVE_R, HALF_GAP, SPAWN_R, TOP
import hoist_physics as physics
import yam_parts as parts
import yam_joints as joints

def array(value):
    value = getattr(value, 'torch', value)
    return value.detach().cpu().numpy().astype(float) if hasattr(value, 'detach') else np.asarray(value, float)


@dataclass(frozen=True)
class RopeState(ObjectState):
    poses: dict = field(default_factory=dict)
    velocities: dict = field(default_factory=dict)
    spec: object = None

    @property
    def centres(self):
        return np.array([self.poses[f'seg_{i:02d}'][0] for i in range(self.spec.segments)])

    @property
    def axes(self):
        return np.array([rotation(self.poses[f'seg_{i:02d}'][1])[:, 0] for i in range(self.spec.segments)])


def state(s, poses, velocities):
    # Root state describes the bucket; all segment poses stay available separately.
    p, q = poses['bucket']
    lo, hi = s.bucket_part().aabb(p, rotation(q))
    vel = velocities['bucket']
    return RopeState(p, q, vel[:3], vel[3:], lo, hi, poses, velocities, s)


def missing_state(s):
    return state(s, {n: (np.full(3, np.nan), np.full(4, np.nan)) for n in s.body_names()},
                 {n: np.full(6, np.nan) for n in s.body_names()})


NOMINAL_ARC_SAMPLES = 256
NOMINAL_TABLE_HEIGHT = .006


def _tangent_angle(s, point):
    axle = np.asarray(s.axle)
    dy, dz = s.side*(point[1]-axle[1]), point[2]-axle[2]
    distance = math.hypot(dy, dz)
    if distance <= SPAWN_R:
        return None
    return math.atan2(dz, dy)+math.acos(SPAWN_R/distance)


@functools.lru_cache(maxsize=256)
def nominal_state(s, pos, matrix, slack=0.):
    """An attached, length-conserving nominal chain for offline pose layouts.

    Walk a load tangent, upper groove arc, vertical hauling fall and tabletop
    tail. Each next vertex is the first intersection with a 20 mm sphere about
    the previous one. Thus all 53 physical links have their actual length and
    share endpoints, including links straddling bends; leftover rope forms the
    tail. This describes geometry, not a prediction of contact dynamics.
    With slack > 0 the load fall bows sideways (toward the load side, in the
    wheel plane) so it is about `slack` metres longer than its straight span:
    a fall that is not carrying the pail.
    """
    pos, R = np.asarray(pos), np.asarray(matrix).reshape(3, 3)
    if not np.isfinite(np.r_[pos, matrix]).all():
        return missing_state(s)
    anchor = pos+R@s.bail_offset()
    axle = np.asarray(s.axle)
    lead = [anchor]
    if slack > 0:
        phi0 = _tangent_angle(s, anchor)
        if phi0 is None:
            return missing_state(s)
        tangent = axle+np.array((0., s.side*SPAWN_R*math.cos(phi0), SPAWN_R*math.sin(phi0)))
        span = tangent-anchor
        length = float(np.linalg.norm(span))
        bow = math.sqrt(((length+slack)/2)**2-(length/2)**2)
        normal = s.side*np.array((0., span[2], -span[1]))
        lead.append((anchor+tangent)/2+bow*normal/np.linalg.norm(normal))
    phi = _tangent_angle(s, lead[-1])
    if phi is None:
        return missing_state(s)
    angles = np.linspace(phi, math.pi, NOMINAL_ARC_SAMPLES+1)
    arc = axle+np.column_stack((np.zeros(len(angles)),
                               s.side*SPAWN_R*np.cos(angles), SPAWN_R*np.sin(angles)))
    foot = np.array((axle[0], axle[1]-s.side*SPAWN_R, TOP+NOMINAL_TABLE_HEIGHT))
    # Extend beyond the available rope; only 53 exact-length chords are taken.
    path = np.array([*lead, *arc, foot, foot+(0., -s.side*s.segments*PITCH, 0.)])
    vertices, edge = [anchor], 0
    for _ in range(s.segments):
        previous = vertices[-1]
        while edge < len(path)-1 and np.linalg.norm(path[edge+1]-previous) < PITCH:
            edge += 1
        if edge == len(path)-1:
            return missing_state(s)
        a, direction = path[edge]-previous, path[edge+1]-path[edge]
        aa, bb = direction@direction, a@direction
        u = (-bb+math.sqrt(max(0., bb*bb-aa*(a@a-PITCH**2))))/aa
        vertices.append(path[edge]+u*direction)
    vertices = np.asarray(vertices)
    poses = {'bucket': (pos, matrix_to_quat_xyzw(R))}
    for i, (a, b) in enumerate(zip(vertices[:-1], vertices[1:])):
        x = (b-a)/PITCH
        y = np.array((1., 0., 0.))-x*x[0]
        y /= np.linalg.norm(y)
        frame = np.column_stack((x, y, np.cross(x, y)))
        poses[f'seg_{i:02d}'] = ((a+b)/2, matrix_to_quat_xyzw(frame))
    return state(s, poses, {name: np.zeros(6) for name in poses})


SEAT_ENTRY_HEIGHT = -.012
SEAT_RADIUS_GATE = .055
SEAT_AXIAL_TOL = .0001
SEAT_CENTRE_GAP = .002
SEAT_SURFACE_TOL = .001


def seating(s, centres, axes):
    rel = centres-np.asarray(s.axle)
    radial = np.linalg.norm(rel[:, 1:], axis=1)
    # Includes tangent entry/exit segments; radius gate excludes remote falls.
    mask = (rel[:, 2] >= SEAT_ENTRY_HEIGHT) & (radial <= SEAT_RADIUS_GATE)
    rows = []
    for i in np.flatnonzero(mask):
        p, a = rel[i], axes[i]
        half = PITCH/2-RADIUS
        t = np.clip(-np.dot(p[1:], a[1:])/max(np.dot(a[1:], a[1:]), 1.e-30), -half, half)
        radial_surface = np.linalg.norm(p[1:]+t*a[1:])-RADIUS
        extent = RADIUS+half*abs(a[0])
        rows.append({'segment': f'seg_{i:02d}', 'radius_mm': radial[i]*1000,
                     'axial_mm': p[0]*1000, 'surface_radius_mm': radial_surface*1000,
                     'outside_flange': bool(abs(p[0])+extent > HALF_GAP+SEAT_AXIAL_TOL)})
    return {'count': len(rows), 'segments': rows,
            'min_radius_mm': min((r['radius_mm'] for r in rows), default=None),
            'min_surface_radius_mm': min((r['surface_radius_mm'] for r in rows), default=None),
            'max_abs_axial_mm': max((abs(r['axial_mm']) for r in rows), default=None),
            'outside_flanges': any(r['outside_flange'] for r in rows),
            'tunnelling': any(r['radius_mm'] < (GROOVE_R+SEAT_CENTRE_GAP)*1000 or
                             r['surface_radius_mm'] < (GROOVE_R-SEAT_SURFACE_TOL)*1000 for r in rows),
            'wrap_missing': not bool(rows)}


def read_rope(env, s):
    art = env.unwrapped.scene.articulations[s.name]
    d = art.data
    poses = array(d.body_link_pose_w)[0]
    # The belt GPU probe found a stale all-zero pose cache after reset. Refresh
    # read caches only; never write simulation state, step or invent a pose.
    if np.isfinite(poses).all() and np.any(np.linalg.norm(poses[:, 3:], axis=1) < 1.e-8):
        d._reset_pose()
        d._root_link_pose_w.timestamp = -float('inf')
        d._root_link_pose_w_ta = None
        d._body_link_pose_w_ta = None
        poses = array(d.body_link_pose_w)[0]
        if np.any(np.linalg.norm(poses[:, 3:], axis=1) < 1.e-8):
            raise ValueError(f'{s.name}: zero body quaternion after read-cache refresh')
    vel = array(d.body_link_vel_w)[0]
    bi = {n: i for i, n in enumerate(d.body_names)}
    origin = array(env.unwrapped.scene.env_origins)[0]
    return state(s, {n: (poses[bi[n], :3]-origin, poses[bi[n], 3:]) for n in s.body_names()},
                 {n: vel[bi[n]] for n in s.body_names()})


_SPECS = ()
_TOKEN = object()


def _patch(scene):
    joints._patch_scene(scene)
    if getattr(scene, '_landing_hoist_token', None) is _TOKEN:
        return
    old_object, old_states = scene.task_object, scene.object_states

    @functools.wraps(old_object)
    def task_object(spec):
        return physics.rope_object(spec) if isinstance(spec, HoistRope) else old_object(spec)

    @functools.wraps(old_states)
    def object_states(env, names):
        names = tuple(names)
        own = {s.name for s in _SPECS}
        result = old_states(env, tuple(n for n in names if n not in own))
        for s in _SPECS:
            if s.name in names:
                result[s.name] = read_rope(env, s)
        return result

    scene.task_object, scene.object_states = task_object, object_states
    scene._landing_hoist_token = _TOKEN


class Finder(importlib.machinery.PathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname != 'yam_arena.scene':
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is not None and spec.loader is not None:
            spec.loader = parts._SceneLoader(spec.loader, _patch)
        return spec


def install(specs):
    global _SPECS
    _SPECS = tuple(specs)
    if not any(isinstance(f, Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, Finder())
    if 'yam_arena.scene' in sys.modules:
        _patch(sys.modules['yam_arena.scene'])
