"""Procedural closed triangle meshes, additive to the parts and joints kits.

Metres and radians (including generator phase); hinge specs retain the joints
kit's degrees. Mesh proxies are offline approximations, never USD colliders.
No simulator imports occur until an Arena object is constructed or spawned.
"""
from __future__ import annotations

import functools
import importlib.abc
import importlib.machinery
import math
import sys
from dataclasses import dataclass, replace
from types import MappingProxyType

import numpy as np
import yam_parts as parts
import yam_joints as joints
from yam_parts import PartState  # factory geometry uses aabb.__globals__["PartState"]

TAU = 2 * math.pi


def signed_volume(mesh):
    """Signed enclosed volume of a Mesh or a (points, faces) pair."""
    p, f = (mesh.points, mesh.faces) if isinstance(mesh, Mesh) else mesh
    # Translation improves conditioning for meshes far from the origin.
    tri = (p - p.mean(axis=0))[f]
    return float(np.einsum('ij,ij->i', tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() / 6)


@dataclass(frozen=True, eq=False)
class Mesh:
    name: str
    points: np.ndarray
    faces: np.ndarray
    collision: str = 'sdf'
    sdf_resolution: int = 256
    sdf_subgrid: int = 6
    color: tuple | None = None
    proxy: tuple = ()

    def __post_init__(self):
        p = np.asarray(self.points, dtype=float)
        raw = np.asarray(self.faces)
        if p.ndim != 2 or p.shape[1:] != (3,) or len(p) < 4 or not np.isfinite(p).all():
            raise ValueError('mesh points must be finite Nx3, with at least four vertices')
        if (raw.ndim != 2 or raw.shape[1:] != (3,) or len(raw) < 4
                or not np.issubdtype(raw.dtype, np.integer)):
            raise ValueError('mesh faces must be integer Mx3 triangles')
        f = raw.astype(np.int64)
        if f.min() < 0 or f.max() >= len(p):
            raise ValueError('mesh face index outside points')
        if not isinstance(self.name, str) or not self.name.isidentifier():
            raise ValueError('mesh name must be an identifier')
        if self.collision not in ('sdf', 'exact', 'convex'):
            raise ValueError('collision must be sdf, exact or convex')
        for name in ('sdf_resolution', 'sdf_subgrid'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value <= 0:
                raise ValueError(f'{name} must be a positive integer')
        if self.color is not None:
            rgb = np.asarray(self.color, float)
            if rgb.shape != (3,) or not np.isfinite(rgb).all() or np.any((rgb < 0) | (rgb > 1)):
                raise ValueError('mesh color must be finite RGB in 0..1')
            object.__setattr__(self, 'color', tuple(rgb))
        tri = p[f]
        scale = float(np.max(np.ptp(p, axis=0)))
        area = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1)
        if np.any(area <= np.finfo(float).eps * scale**2 * 8):
            raise ValueError('mesh contains degenerate triangles')
        edges = np.concatenate((f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]))
        _, counts = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
        directed, dc = np.unique(edges, axis=0, return_counts=True)
        if not np.all(counts == 2) or not np.all(dc == 1) or not np.array_equal(
                directed, np.unique(edges[:, ::-1], axis=0)):
            raise ValueError('mesh must be closed 2-manifold with opposite edge winding')
        # Edge incidence alone misses two closed shells pinched at one vertex.
        rings = [{} for _ in p]
        for a, b, c in f:
            rings[a][b] = c
            rings[b][c] = a
            rings[c][a] = b
        for ring in rings:
            if not ring:
                raise ValueError('mesh contains an unused vertex')
            start = current = next(iter(ring))
            visited = set()
            while current not in visited:
                visited.add(current)
                current = ring[current]
            if current != start or len(visited) != len(ring):
                raise ValueError('mesh vertex is not 2-manifold')
        if signed_volume((p, f)) <= 0:
            raise ValueError('mesh must have outward orientation (positive signed volume)')
        proxy = tuple(self.proxy)
        if proxy:
            parts.Part('proxy', (0, 0, 0), proxy)._check()
        # Bytes-backed arrays cannot be made writable, even with setflags().
        object.__setattr__(self, 'points', np.frombuffer(p.tobytes(), dtype=float).reshape(p.shape))
        object.__setattr__(self, 'faces', np.frombuffer(f.tobytes(), dtype=np.int64).reshape(f.shape))
        object.__setattr__(self, 'proxy', proxy)


def _positive(**values):
    if any(not math.isfinite(v) or v <= 0 for v in values.values()):
        raise ValueError(f'positive finite values required: {", ".join(values)}')


def _samples(value, minimum=3):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < minimum:
        raise ValueError(f'samples must be an integer >= {minimum}')
    return int(value)


def _cross(a, b):
    return a[..., 0]*b[..., 1] - a[..., 1]*b[..., 0]


def _outline(outline):
    p = np.asarray(outline, float)
    if p.ndim != 2 or p.shape[1:] != (2,) or not np.isfinite(p).all():
        raise ValueError('outline must be finite Nx2')
    if len(p) > 1 and np.array_equal(p[0], p[-1]):
        p = p[:-1]
    if len(p) < 3 or np.any(np.linalg.norm(np.roll(p, -1, axis=0)-p, axis=1) == 0):
        raise ValueError('outline needs three distinct consecutive vertices')
    area = _cross(p, np.roll(p, -1, axis=0)).sum()
    if area == 0:
        raise ValueError('outline has zero area')
    return p if area > 0 else p[::-1].copy()


def triangulate_polygon(outline):
    """Ear clipping of a simple CCW polygon, including concave outlines."""
    p = np.asarray(outline, float)
    remaining, faces = list(range(len(p))), []
    eps = np.finfo(float).eps * np.max(np.ptp(p, axis=0))**2 * 8
    while len(remaining) > 3:
        for k, b in enumerate(remaining):
            a, c = remaining[k-1], remaining[(k+1) % len(remaining)]
            pa, pb, pc = p[[a, b, c]]
            if _cross(pb-pa, pc-pb) <= eps:
                continue
            others = p[[v for v in remaining if v not in (a, b, c)]]
            # A hole bridge repeats its endpoints; those copies are not
            # interior blockers for an ear incident on the same point.
            others = others[~np.any(np.all(others[:, None, :] == p[[a, b, c]], axis=2), axis=1)]
            inside = ((_cross(pb-pa, others-pa) >= -eps)
                      & (_cross(pc-pb, others-pb) >= -eps)
                      & (_cross(pa-pc, others-pc) >= -eps))
            if inside.any():
                continue
            faces.append((a, b, c))
            remaining.pop(k)
            break
        else:
            raise ValueError('outline must be a simple nondegenerate polygon')
    faces.append(tuple(remaining))
    return faces


def _extrude(outline, thickness):
    n = len(outline)
    p = np.vstack([np.column_stack((outline, np.full(n, z))) for z in (-thickness/2, thickness/2)])
    f = []
    for i in range(n):
        k = (i+1) % n
        f.extend(((i, k, n+k), (i, n+k, n+i)))
    for a, b, c in triangulate_polygon(outline):
        f.extend(((c, b, a), (n+a, n+b, n+c)))
    return p, np.asarray(f, int)


def prism(outline_xy, thickness, axis='z', *, name='prism', proxy=None, **options):
    """Extrude a simple polygon about its mid-plane; x/y use proper rotations."""
    _positive(thickness=thickness)
    if axis not in ('x', 'y', 'z'):
        raise ValueError('axis must be x, y or z')
    p, f = _extrude(_outline(outline_xy), thickness)
    p = p @ parts._cylinder_rotation(axis).T
    if proxy is None:
        proxy = (parts.box((p.min(0)+p.max(0))/2, np.ptp(p, axis=0)),)
    return Mesh(name, p, f, proxy=proxy, **options)


def thread_radius(theta, z, *, pitch, root_r, major_r, crest_width=None,
                  root_width=None, flank=None, hand='right', starts=1, phase=0):
    """Scout phase z/pitch - theta/2pi; pitch is ridge spacing, lead=starts*pitch.

    Widths and flank are axial lengths; crest + root + 2*flank = pitch.
    Internal radial clearance is added by thread(), not by this function.
    """
    _positive(pitch=pitch, root_r=root_r, major_r=major_r)
    if major_r <= root_r:
        raise ValueError('major_r must exceed root_r')
    _samples(starts, 1)
    if hand not in ('right', 'left') or not math.isfinite(phase):
        raise ValueError('hand must be right or left and phase finite (radians)')
    crest = pitch/8 if crest_width is None else crest_width
    root = pitch/8 if root_width is None else root_width
    flank = (pitch-crest-root)/2 if flank is None else flank
    _positive(crest_width=crest, root_width=root, flank=flank)
    if not math.isclose(crest+root+2*flank, pitch, rel_tol=1e-9, abs_tol=1e-14):
        raise ValueError('crest_width + root_width + 2*flank must equal pitch')
    sign = 1 if hand == 'right' else -1
    u = np.mod(np.asarray(z)/pitch - sign*starts*(np.asarray(theta)-phase)/TAU, 1)
    distance = np.minimum(u, 1-u)*pitch
    return root_r + (major_r-root_r)*np.clip((crest/2+flank-distance)/flank, 0, 1)


def thread(pitch, root_r, major_r, length, *, internal=False, outer=None,
           crest_width=None, root_width=None, flank=None, clearance=0.0005,
           hand='right', starts=1, phase=0, angular_samples=192, axial_samples=None,
           axial_step=None, grid_offset=0, center=(0, 0, 0), name='thread', proxy=None, **options):
    """Capped helical stud (z=0..length), or annular nut (z=+-length/2).

    center translates the generated mesh and proxies into the owning part frame.
    grid_offset aligns axial tessellation to a mating body's world-z grid:
    planes are k*axial_step-grid_offset before adding center.
    """
    _positive(length=length)
    n = _samples(angular_samples)
    if not math.isfinite(clearance) or clearance < 0 or not math.isfinite(grid_offset):
        raise ValueError('clearance must be nonnegative and grid_offset finite')
    lo, hi = (-length/2, length/2) if internal else (0, length)
    if axial_step is not None and axial_samples is not None:
        raise ValueError('choose axial_step or axial_samples')
    if axial_samples is not None:
        zs = np.linspace(lo, hi, _samples(axial_samples, 1)+1)
    else:
        dz = pitch/32 if axial_step is None else axial_step
        _positive(axial_step=dz)
        middle = np.arange(math.ceil((lo+grid_offset)/dz), math.floor((hi+grid_offset)/dz)+1)*dz-grid_offset
        zs = np.unique(np.clip(np.r_[lo, middle, hi], lo, hi))
        # Avoid numerical sliver rows at a nominally aligned endpoint.
        zs = zs[np.r_[True, np.diff(zs) > 1e-12*length]]
        zs[-1] = hi
    theta = np.arange(n)*TAU/n
    radius = thread_radius(theta[None, :], zs[:, None], pitch=pitch, root_r=root_r,
                           major_r=major_r, crest_width=crest_width, root_width=root_width,
                           flank=flank, hand=hand, starts=starts, phase=phase)
    if internal:
        radius = radius + clearance
    p = np.stack((radius*np.cos(theta), radius*np.sin(theta),
                  np.broadcast_to(zs[:, None], radius.shape)), axis=-1).reshape(-1, 3).tolist()
    f, rows = [], len(zs)
    for j in range(rows-1):
        for i in range(n):
            a, b = j*n+i, j*n+(i+1) % n
            f.extend(((a, b, b+n), (a, b+n, a+n)))
    if internal:
        if outer is None or len(outer) != 2 or outer[0] not in ('hex', 'cyl'):
            raise ValueError('internal thread requires outer=(hex, across_flats) or (cyl, radius)')
        _positive(outer_size=outer[1])
        min_r = outer[1]/2 if outer[0] == 'hex' else outer[1]
        if min_r <= major_r+clearance:
            raise ValueError('outer body must be outside the major radius plus clearance')
        f = [tuple(reversed(face)) for face in f]
        delta = (theta+math.pi/6) % (math.pi/3)-math.pi/6
        ro = min_r/np.cos(delta) if outer[0] == 'hex' else np.full(n, min_r)
        offset = len(p)
        for z in (lo, hi):
            p.extend(np.column_stack((ro*np.cos(theta), ro*np.sin(theta), np.full(n, z))).tolist())
        for i in range(n):
            k = (i+1) % n
            a, b, c, d = offset+i, offset+k, offset+n+i, offset+n+k
            f.extend(((a, b, d), (a, d, c), (i, k, b), (i, b, a)))
            a, b, c, d = (rows-1)*n+i, (rows-1)*n+k, offset+n+i, offset+n+k
            f.extend(((a, c, d), (a, d, b)))
        if proxy is None:
            hole = major_r+clearance
            xmax, ymax = min_r, (2*min_r/math.sqrt(3) if outer[0] == 'hex' else min_r)
            proxy = tuple(parts.box((sign*(hole+xmax)/2, 0, 0), (xmax-hole, 2*ymax, length))
                          for sign in (-1, 1))
            proxy += tuple(parts.box((0, sign*(hole+ymax)/2, 0), (2*hole, ymax-hole, length))
                           for sign in (-1, 1))
    else:
        bottom, top = len(p), len(p)+1
        p.extend(((0, 0, lo), (0, 0, hi)))
        for i in range(n):
            k = (i+1) % n
            f.extend(((bottom, k, i), (top, (rows-1)*n+i, (rows-1)*n+k)))
        if proxy is None:
            proxy = (parts.cyl((0, 0, length/2), root_r, length),)
    center = np.asarray(center, float)
    if center.shape != (3,) or not np.isfinite(center).all():
        raise ValueError('center must be finite xyz')
    proxy = tuple(replace(s, center=tuple(np.asarray(s.center)+center)) for s in proxy)
    return Mesh(name, np.asarray(p)+center, np.asarray(f, int), proxy=proxy, **options)


def gear_outline(teeth, module, pressure_angle=20, backlash=.15e-3, phase=0):
    """Scout involutes with radial dedendum extensions (no undercut/fillet).

    backlash is the circular tooth thickness removed from EACH gear.
    """
    _samples(teeth, 4)
    _positive(module=module)
    if not 0 < pressure_angle < 45 or not math.isfinite(phase) or not 0 <= backlash < math.pi*module/2:
        raise ValueError('invalid pressure angle, backlash or phase')
    pitch = module*teeth/2
    root, tip = pitch-1.25*module, pitch+module
    alpha = math.radians(pressure_angle)
    base = pitch*math.cos(alpha)
    half_pitch = (math.pi*module/2-backlash)/(2*pitch)
    # A chord between coarse involute samples cuts inside the analytical
    # flank at the pitch circle. Include that circle explicitly so the
    # authored polygon preserves the requested tooth thickness/backlash.
    radii = np.unique(np.r_[root, pitch, np.linspace(max(root, base), tip, 10)])
    angles = np.arccos(np.minimum(1, base/radii))
    half = half_pitch+math.tan(alpha)-alpha-(np.tan(angles)-angles)
    if half[-1] <= 0:
        raise ValueError('backlash/pressure angle leaves no tooth crest')
    out = []
    for k in range(teeth):
        c = phase+k*TAU/teeth
        polar = list(zip(radii, c-half))
        polar += [(tip, a) for a in np.linspace(c-half[-1], c+half[-1], 5)[1:-1]]
        polar += list(zip(radii[::-1], (c+half)[::-1]))
        polar += [(root, a) for a in np.linspace(c+half[0], c+TAU/teeth-half[0], 5)[1:-1]]
        out.extend((r*math.cos(a), r*math.sin(a)) for r, a in polar)
    return np.asarray(out)


def _bored_extrusion(outline, thickness, bore):
    """Ear-clipped annular extrusion with a circular bore."""
    n = len(outline)
    angle = np.arange(96)*TAU/96
    inner = bore*np.column_stack((np.cos(angle), np.sin(angle)))
    xy = np.vstack((outline, inner))
    total = len(xy)
    p = np.vstack([np.column_stack((xy, np.full(total, z))) for z in (-thickness/2, thickness/2)])
    # Bridge the hole to the rightmost exterior vertex. The cap polygon is
    # weakly simple; repeated bridge endpoints map to the SAME mesh indices.
    i = int(np.argmax(outline[:, 0]))
    ring = list(range(i+1)) + [n] + list(range(total-1, n-1, -1)) + [i] + list(range(i+1, n))
    caps = triangulate_polygon(xy[ring])
    f = []
    for indices in (list(range(n)), list(range(total-1, n-1, -1))):
        for a, b in zip(indices, indices[1:]+indices[:1]):
            f.extend(((a, b, total+b), (a, total+b, total+a)))
    for a, b, c in caps:
        a, b, c = ring[a], ring[b], ring[c]
        f.extend(((c, b, a), (total+a, total+b, total+c)))
    return p, np.asarray(f, int)


def spur_gear(teeth, module, thickness, pressure_angle=20, backlash=.15e-3, phase=0,
              bore=None, *, name='gear', proxy=None, **options):
    """Involute spur gear; optional bore is its radius, phase in radians."""
    _positive(thickness=thickness)
    outline = gear_outline(teeth, module, pressure_angle, backlash, phase)
    root = module*(teeth/2-1.25)
    if bore is None:
        p, f = _extrude(outline, thickness)
    else:
        _positive(bore=bore)
        if bore >= root:
            raise ValueError('bore must lie inside root radius')
        p, f = _bored_extrusion(outline, thickness, bore)
    return Mesh(name, p, f, proxy=(parts.cyl((0, 0, 0), root, thickness),) if proxy is None else proxy, **options)


def ratchet_wheel(teeth, outer_r, root_r, thickness, direction=+1, *, name='ratchet', proxy=None, **options):
    """Sawtooth outline: radial locking face and straight sloped back per tooth."""
    _samples(teeth)
    _positive(outer_r=outer_r, root_r=root_r, thickness=thickness)
    if outer_r <= root_r or direction not in (-1, 1):
        raise ValueError('outer_r must exceed root_r; direction must be +1 or -1')
    a = np.repeat(np.arange(teeth)*TAU/teeth, 2)*direction
    r = np.tile([root_r, outer_r], teeth)
    outline = _outline(np.column_stack((r*np.cos(a), r*np.sin(a))))
    p, f = _extrude(outline, thickness)
    return Mesh(name, p, f, proxy=(parts.cyl((0, 0, 0), root_r, thickness),) if proxy is None else proxy, **options)


def cam(radius_fn, thickness, samples=360, *, name='cam', proxy=None, **options):
    """Polar plate cam; its minimum sampled radius defines the base proxy."""
    _positive(thickness=thickness)
    a = np.arange(_samples(samples))*TAU/samples
    r = np.asarray([radius_fn(float(t)) for t in a], float)
    if r.shape != a.shape or not np.isfinite(r).all() or np.any(r <= 0):
        raise ValueError('radius_fn must return positive finite scalar radii')
    p, f = _extrude(np.column_stack((r*np.cos(a), r*np.sin(a))), thickness)
    return Mesh(name, p, f, proxy=(parts.cyl((0, 0, 0), float(r.min()), thickness),) if proxy is None else proxy, **options)


def revolve(profile_rz, samples=96, *, name='revolve', proxy=None, **options):
    """Revolve a closed simple r/z section; axis vertices are welded poles.

    A solid cylinder is [(0,z0),(r,z0),(r,z1),(0,z1)]. A section entirely
    outside r=0 produces a hollow collar. First/last repeated point is optional.
    """
    profile = _outline(profile_rz)
    if np.any(profile[:, 0] < 0):
        raise ValueError('profile radii must be nonnegative')
    n = _samples(samples)
    a = np.arange(n)*TAU/n
    p, rows = [], []
    for r, z in profile:
        if r == 0:
            rows.append([len(p)]*n)
            p.append((0, 0, z))
        else:
            rows.append(list(range(len(p), len(p)+n)))
            p.extend(zip(r*np.cos(a), r*np.sin(a), np.full(n, z)))
    f = []
    for row, other in zip(rows, rows[1:]+rows[:1]):
        for i in range(n):
            k = (i+1) % n
            for face in ((row[i], row[k], other[k]), (row[i], other[k], other[i])):
                if len(set(face)) == 3:
                    f.append(face)
    p, f = np.asarray(p, float), np.asarray(f, int)
    if signed_volume((p, f)) < 0:
        f = f[:, ::-1]
    if proxy is None:
        proxy = (parts.cyl((0, 0, float((p[:, 2].min()+p[:, 2].max())/2)),
                           float(profile[:, 0].max()), float(np.ptp(p[:, 2]))),)
    return Mesh(name, p, f, proxy=proxy, **options)


def _mesh_tuple(meshes, dynamic):
    meshes = tuple(meshes)
    if not all(isinstance(m, Mesh) for m in meshes):
        raise TypeError('meshes must contain Mesh objects')
    if len({m.name for m in meshes}) != len(meshes):
        raise ValueError('mesh names must be unique within a body')
    if dynamic and any(m.collision == 'exact' for m in meshes):
        raise ValueError('exact triangle collision is static bodies only; dynamic bodies require sdf or convex')
    return meshes


def _proxy(meshes):
    return tuple(s for m in meshes for s in m.proxy)


class MeshPart(parts.Part):
    """Part with real primitives in shapes and independent meshes in meshes.

    grip indexes real primitives first, then mesh proxies in mesh order. Custom
    Mesh objects may have no proxies; mesh-only bodies need offline proxies.
    """
    def __init__(self, name, pos, shapes=(), mass=None, color=parts.GREY, yaw=0,
                 friction=None, contact_offset=.002, grip=None, *, meshes=()):
        meshes = _mesh_tuple(meshes, mass is not None)
        object.__setattr__(self, 'meshes', meshes)
        super().__init__(name, pos, shapes, mass, color, yaw, friction, contact_offset, grip)
        object.__setattr__(self, 'size', self._offline_part().size)

    def _offline_part(self):
        return parts.Part(self.name, self.pos, self.shapes+_proxy(self.meshes), self.mass,
                          self.color, self.yaw, self.friction, self.contact_offset, self.grip)

    def _check(self):
        _mesh_tuple(self.meshes, self.mass is not None)
        self._offline_part()._check()
        if self.mass is not None and not math.isfinite(self.mass):
            raise ValueError('mass must be finite')

    def local_aabb(self):
        return self._offline_part().local_aabb()

    def aabb(self, origin, R):
        return self._offline_part().aabb(origin, R)

    def world_shapes(self, origin, R):
        return self._offline_part().world_shapes(origin, R)

    def footprints(self, origin, R):
        return self._offline_part().footprints(origin, R)

    def volume_centroid(self):
        """Offline proxy centroid; PhysX derives true inertia from authored meshes."""
        return self._offline_part().volume_centroid()

    def grip_shape(self):
        return self._offline_part().grip_shape()


class MeshMechanism(joints.Mechanism):
    """Mechanism with meshes keyed by base/link, all in the part frame at q=0."""
    def __init__(self, name, pos, shapes=(), links=(), mass=None, color=parts.GREY, yaw=0,
                 friction=None, contact_offset=.002, grip=None, self_collide=False,
                 lamps=(), lamp_logic=None, *, meshes=None):
        links = tuple(links)
        allowed = {'base', *(l.name for l in links)}
        meshes = {} if meshes is None else dict(meshes)
        if meshes.keys()-allowed:
            raise ValueError('mesh body must name base or a defined link')
        # Even a world-bolted articulation base carries RigidBodyAPI.
        frozen = {key: _mesh_tuple(value, True) for key, value in meshes.items()}
        object.__setattr__(self, 'meshes', MappingProxyType(frozen))
        super().__init__(name, pos, shapes, links, mass, color, yaw, friction, contact_offset,
                         grip, self_collide, lamps, lamp_logic)
        object.__setattr__(self, 'size', self._offline_part().size)

    def _all_proxies(self):
        return _proxy(self.meshes.get('base', ())) + tuple(
            s for l in self.links for s in _proxy(self.meshes.get(l.name, ())))

    def _offline_part(self):
        return parts.Part(self.name, self.pos, self.shapes+self._all_proxies(), self.mass,
                          self.color, self.yaw, self.friction, self.contact_offset, self.grip)

    def _check(self):
        for meshes in self.meshes.values():
            _mesh_tuple(meshes, True)
        # Delegate all unchanged kit validation using a temporary proxy-bearing
        # specification. Never replace the real spec's primitive-only shapes.
        base = self.base_shapes+_proxy(self.meshes.get('base', ()))
        links = tuple(replace(l, shapes=l.shapes+_proxy(self.meshes.get(l.name, ()))) for l in self.links)
        joints.Mechanism(self.name, self.pos, base, links, self.mass, self.color, self.yaw,
                         self.friction, self.contact_offset, None, self.self_collide,
                         self.lamps, self.lamp_logic)._check()
        self._offline_part()._check()

    local_aabb = MeshPart.local_aabb
    aabb = MeshPart.aabb
    world_shapes = MeshPart.world_shapes
    footprints = MeshPart.footprints
    volume_centroid = MeshPart.volume_centroid
    grip_shape = MeshPart.grip_shape

    def _world_shapes(self, origin, R, poses):
        out = list(super()._world_shapes(origin, R, poses))
        out.extend(parts.Part(self.name, self.pos, _proxy(self.meshes.get('base', ()))).world_shapes(origin, R))
        for link in self.links:
            p, rot = poses[link.name]
            shifted = tuple(replace(s, center=tuple(np.asarray(s.center)-link.joint.anchor))
                            for s in _proxy(self.meshes.get(link.name, ())))
            out.extend(parts.Part(link.name, p, shifted).world_shapes(p, rot))
        return tuple(out)

    def shape_labels(self):
        """Labels aligned with world_shapes_at: all real shapes, then proxies."""
        return ((None,)*len(self.base_shapes)
                + tuple(l.name for l in self.links for _ in l.shapes)
                + (None,)*len(_proxy(self.meshes.get('base', ())))
                + tuple(l.name for l in self.links for _ in _proxy(self.meshes.get(l.name, ()))))

    def _state(self, *args, **kwargs):
        state = super()._state(*args, **kwargs)
        return MeshMechanismState(**state.__dict__)


@dataclass(frozen=True)
class MeshMechanismState(joints.MechanismState):
    def link_box(self, link):
        spec = self._mechanism._link(link)
        shapes = spec.shapes+_proxy(self._mechanism.meshes.get(link, ()))
        shifted = tuple(replace(s, center=tuple(np.asarray(s.center)-spec.joint.anchor)) for s in shapes)
        return joints._bounds(parts.Part(link, (0, 0, 0), shifted).world_shapes(
            self.links[link][0], self.link_rotation(link)), exact=True)


def _author_mesh(stage, parent, mesh, anchor, color, contact_offset):
    import isaaclab.sim as sim_utils
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    path = f'{parent}/mesh_{mesh.name}'
    points = mesh.points-np.asarray(anchor)
    geom = UsdGeom.Mesh.Define(stage, path)
    geom.CreatePointsAttr(points.tolist())
    geom.CreateFaceVertexCountsAttr([3]*len(mesh.faces))
    geom.CreateFaceVertexIndicesAttr(mesh.faces.ravel().tolist())
    geom.CreateSubdivisionSchemeAttr().Set('none')
    geom.CreateOrientationAttr().Set('rightHanded')
    geom.CreateExtentAttr([Gf.Vec3f(*map(float, points.min(0))), Gf.Vec3f(*map(float, points.max(0)))])
    prim = geom.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    approximation = {'exact': 'none', 'convex': 'convexHull', 'sdf': 'sdf'}[mesh.collision]
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set(approximation)
    collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    collision.CreateContactOffsetAttr().Set(contact_offset)
    collision.CreateRestOffsetAttr().Set(0.)
    if mesh.collision == 'sdf':
        sdf = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
        sdf.CreateSdfResolutionAttr().Set(mesh.sdf_resolution)
        sdf.CreateSdfSubgridResolutionAttr().Set(mesh.sdf_subgrid)
    sim_utils.bind_physics_material(path, f'{parent}/physics_material', stage=stage)
    visual_path = f'{parent}/mesh_{mesh.name}_look'
    look = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color if mesh.color is None else mesh.color),
                                       roughness=.35, metallic=.8)
    look.func(visual_path, look)
    sim_utils.bind_visual_material(path, visual_path, stage=stage)


def _spawn_mesh_part(path, cfg, *args, **kwargs):
    import isaaclab.sim as sim_utils
    part = parts._SPECS[cfg.part_name]
    _mesh_tuple(part.meshes, part.mass is not None)
    prim = parts._spawn_part(path, cfg, *args, **kwargs)
    if part.meshes:
        stage = kwargs.get('stage') or sim_utils.get_current_stage()
        for mesh in part.meshes:
            _author_mesh(stage, path, mesh, (0, 0, 0), part.color, part.contact_offset)
    return prim


def _spawn_mesh_mechanism(path, cfg, *args, **kwargs):
    import isaaclab.sim as sim_utils
    mech = joints._SPECS[cfg.mech_name]
    for meshes in mech.meshes.values():
        _mesh_tuple(meshes, True)
    prim = joints._spawn_mechanism(path, cfg, *args, **kwargs)
    if any(mech.meshes.values()):
        stage = kwargs.get('stage') or sim_utils.get_current_stage()
        for body, meshes in mech.meshes.items():
            link = None if body == 'base' else mech._link(body)
            anchor = (0, 0, 0) if link is None else link.joint.anchor
            color = mech.color if link is None or link.color is None else link.color
            for mesh in meshes:
                _author_mesh(stage, f'{path}/{body}', mesh, anchor, color, mech.contact_offset)
    return prim


_PART_CFG = _MECH_CFG = _PART_SPAWN = _MECH_SPAWN = None
_PATCH_TOKEN = object()


def spawner_config_type(base, configclass, *, mechanism=False):
    """Classes live in this registered module, never in an unregistered task."""
    if mechanism:
        @configclass
        class MeshMechanismSpawnerCfg(base):
            mech_name: str = ''
        return MeshMechanismSpawnerCfg
    @configclass
    class MeshPartSpawnerCfg(base):
        part_name: str = ''
    return MeshPartSpawnerCfg


def _mesh_part_object(spec):
    global _PART_CFG, _PART_SPAWN
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    obj = parts._part_object(spec)
    if _PART_CFG is None:
        _PART_CFG = spawner_config_type(sim_utils.SpawnerCfg, configclass)
        _PART_SPAWN = clone(_spawn_mesh_part)
    cfg = _PART_CFG(func=_PART_SPAWN, part_name=spec.name)
    obj.spawner_cfg = obj.object_cfg.spawn = cfg
    parts._PART_NAMES.add(spec.name)
    joints._SPECS.pop(spec.name, None)
    return obj


def _mesh_mechanism_object(spec):
    global _MECH_CFG, _MECH_SPAWN
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    obj = joints._mechanism_object(spec)
    if _MECH_CFG is None:
        _MECH_CFG = spawner_config_type(sim_utils.SpawnerCfg, configclass, mechanism=True)
        _MECH_SPAWN = clone(_spawn_mesh_mechanism)
    cfg = _MECH_CFG(func=_MECH_SPAWN, mech_name=spec.name)
    obj.spawner_cfg = obj.object_cfg.spawn = cfg
    parts._PART_NAMES.discard(spec.name)
    return obj


def _patch_scene(scene):
    joints._patch_scene(scene)
    if getattr(scene, '_yam_mesh_patch_token', None) is _PATCH_TOKEN:
        return
    inner = getattr(scene.task_object, '_yam_mesh_original', scene.task_object)

    @functools.wraps(inner)
    def task_object(spec):
        if isinstance(spec, MeshMechanism):
            return _mesh_mechanism_object(spec)
        if isinstance(spec, MeshPart):
            return _mesh_part_object(spec)
        return inner(spec)

    task_object._yam_mesh_original = inner
    scene.task_object = task_object
    scene._yam_mesh_patch_token = _PATCH_TOKEN


class _SceneFinder(importlib.abc.MetaPathFinder):
    _yam_mesh_scene_finder = True

    def __init__(self, patch):
        self.patch = patch

    def find_spec(self, fullname, path=None, target=None):
        if fullname != 'yam_arena.scene':
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is not None and spec.loader is not None:
            spec.loader = parts._SceneLoader(spec.loader, self.patch)
        return spec


def install():
    finder = next((f for f in sys.meta_path if getattr(f, '_yam_mesh_scene_finder', False)), None)
    if finder is None:
        finder = _SceneFinder(_patch_scene)
    else:
        sys.meta_path.remove(finder)
        finder.patch = _patch_scene
    sys.meta_path.insert(0, finder)
    if 'yam_arena.scene' in sys.modules:
        _patch_scene(sys.modules['yam_arena.scene'])


install()
__all__ = ['Mesh', 'MeshPart', 'MeshMechanism', 'MeshMechanismState', 'thread', 'thread_radius',
           'spur_gear', 'ratchet_wheel', 'cam', 'prism', 'revolve', 'install']
