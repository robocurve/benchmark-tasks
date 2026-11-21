"""curtain_kit: a cloth panel hung from clip rings that slide on a round rod, for YAM Arena task files (on cloth_kit).

What it models
    * :func:`clip_ring`: one metal curtain clip ring, a rigid body of its own: a ring (a torus built from 12 convex
      pieces) round a round rod, an eyelet and a spring clip hanging below it. It has mass, so it hangs on the rod by
      gravity and slides or swings on it by contact and friction only; nothing else moves it.
    * :class:`Panel`: a cloth_kit sheet (a PhysX surface deformable, every node simulated) hung from those clips. The
      cloth nodes in each clip's bite (the top two rows of nodes within 10.5 mm of the clip's middle) are tied to that
      ring by a native PhysX vertex-to-body attachment (``OmniPhysicsVtxXformAttachment``, the same joint cloth_kit uses
      for its pad grip), authored with the cloth before the run starts and always on: a clip that never lets go. The
      cloth pulls its rings and the rings hold the cloth; there is no other coupling.
    * The panel can start gathered: between two clips that are closer on the rod than the cloth between them, the cloth
      falls in one soft wave (a sine bulge, or a U fold when the clips are very close), in front of or behind the rod
      (alternating, as gathered curtains do); corners where two gaps meet are rounded over +-8 mm. A panel longer than
      its drop starts with the rest lying flat on the table behind it (``floor_z``; keep the top edge single-valued
      along the rod, i.e. no U folds, or the lying strips would overlap). The start shape is built isometric (every
      length of the flat cloth kept, up to a few % at the rounded corners), so the cloth starts nearly unstrained;
      ``bend`` is small (a limp woven fabric) and the rest shape is flat, so the waves are not springs.

Declare (module level of a task file)::

    import cloth_kit as ck
    import curtain_kit as cu
    rings = [cu.clip_ring(f"ring_l{k}", (ROD_X, y, ROD_Z - cu.RING_DROP)) for k, y in enumerate(YS)]
    panel = cu.panel("curtain_l", rod_x=ROD_X, top_z=ROD_Z - cu.CLOTH_DROP, ring_u=(0, .1525, ...), ring_y=YS,
                     bulge=(+1, -1, ...), width=0.61, length=0.305, thickness=0.002, mass_kg=0.032, ...)
    cu.hang(panel, rings)          # ties each clip's bite to its ring (authored when the cloth is spawned)
    OBJECTS = [ROD, *rings, panel]     # rings before the cloth
    # score(): objects["ring_l0"].pos (a PartState), objects["curtain_l"].nodes; panel.uv() material coordinates

Tuned numbers (estimates, see the task README): ring 1 in (25.4 mm) inside, 4 mm wire (33.4 mm outside), eyelet 6 mm,
clip 12 x 6 x 16 mm, 9 g each with the clip; the clip's bottom 1.5 mm above the cloth's top edge; the ring hangs on a
7/16 in (11.1 mm) rod 7.4 mm below the rod's axis, so the cloth's top edge is ``CLOTH_DROP`` (47.6 mm) below the rod's
axis. Ring-on-rod friction (0.25 static, 0.20 dynamic, PhysX averages the two materials) is an estimate.

Pitfalls
    * A ring is a free object: a seeded reset with ``object_jitter`` > 0 shifts each ring on its own, sideways into the
      rod, and the clip's attachment then pulls the cloth. Run with ``-E object_jitter=0`` (the default for scripted and
      Dream runs) or a jitter well under the ring's 4.8 mm play round the rod.
    * Rings must be listed in OBJECTS (any order; the attachment names the ring's prim path, which PhysX resolves when
      the simulation starts).
    * GPU host only: the attachment is authored by wrapping Isaac Lab's mesh spawner when cloth_kit builds the cloth;
      offline (no Isaac Sim) nothing is patched.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields

import numpy as np

import cloth_kit as ck
from yam_mesh import Mesh, MeshPart
from yam_parts import box

# ----------------------------------------------------------------------------------------------- the clip ring
ROD_R = 0.0111 / 2           # 7/16 in rod (cafe tension rod)
RING_IN = 0.0254 / 2         # 1 in inside diameter
RING_WIRE = 0.004            # 33.4 mm outside diameter
RING_DROP = RING_IN - ROD_R + 0.0003            # ring centre below the rod axis when it hangs on it
EYELET = (0.003, 0.003, 0.006)                   # x, y, z
CLIP = (0.006, 0.012, 0.016)                     # x (across the cloth), y (along the rod), z
CLIP_GAP = 0.0015                                # clip bottom above the cloth's top edge
RING_TO_CLOTH = RING_IN + RING_WIRE + EYELET[2] + CLIP[2] + CLIP_GAP   # ring centre to the cloth's top edge
CLOTH_DROP = RING_DROP + RING_TO_CLOTH          # rod axis to the cloth's top edge (~47.6 mm)
RING_MASS = 0.009
RING_FRICTION = (0.25, 0.20)
BITE_U, BITE_V = 0.0105, 0.0105                  # cloth nodes tied to a clip: within BITE_U of its middle, top BITE_V
SMOOTH = 0.008                                   # start shape: corners of the top edge rounded over +-8 mm of cloth


def _torus_piece(name, a0, a1, r0, r1, half, color, samples=3):
    """One convex piece of a ring round the y axis: the square section (r0..r1) x (-half..half) swept from a0 to a1 about
    y. Ring plane x-z. Closed, outward faces."""
    section = [(r0, -half), (r1, -half), (r1, half), (r0, half)]
    angles = np.linspace(a0, a1, samples)
    pts = np.array([(r * math.cos(a), t, r * math.sin(a)) for a in angles for r, t in section])
    n = len(section)
    faces = []
    for i in range(samples - 1):
        for j in range(n):
            a, b = i * n + j, i * n + (j + 1) % n
            faces += [(a, b, b + n), (a, b + n, a + n)]
    faces += [(0, j + 1, j) for j in range(1, n - 1)]
    last = (samples - 1) * n
    faces += [(last, last + j, last + j + 1) for j in range(1, n - 1)]
    faces = np.asarray(faces, int)
    tri = (pts - pts.mean(0))[faces]
    if np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        faces = faces[:, ::-1]
    am = (a0 + a1) / 2
    rm = (r0 + r1) / 2
    chord = 2 * rm * math.sin((a1 - a0) / 2)
    proxy = box((rm * math.cos(am), 0.0, rm * math.sin(am)),
                (max(abs(chord * math.sin(am)), r1 - r0), 2 * half, max(abs(chord * math.cos(am)), r1 - r0)))
    return Mesh(name, pts, faces, collision="convex", color=color, proxy=(proxy,))


def clip_ring(name, centre, mass=RING_MASS, color=(0.08, 0.08, 0.09), friction=RING_FRICTION, pieces=12):
    """A clip ring hanging round a rod along y: ``centre`` is the ring's middle (part frame origin)."""
    r0, r1 = RING_IN, RING_IN + RING_WIRE
    meshes = [_torus_piece(f"ring{i}", i * 2 * math.pi / pieces, (i + 1) * 2 * math.pi / pieces, r0, r1, RING_WIRE / 2,
                           color) for i in range(pieces)]
    z_eye = -r1 - EYELET[2] / 2
    z_clip = -r1 - EYELET[2] - CLIP[2] / 2
    shapes = [box((0.0, 0.0, z_eye), EYELET), box((0.0, 0.0, z_clip), CLIP)]
    return MeshPart(name, tuple(float(v) for v in centre), shapes, mass=mass, color=color, friction=friction,
                    contact_offset=0.0015, meshes=tuple(meshes))


# ---------------------------------------------------------------------------------------------- the cloth panel
def _gap_curve(y0, y1, length, bulge, samples=600, fold=True):
    """(n, 2) points (dx from the rod plane, y) of the cloth's top edge between two clips at y0, y1 that hold ``length``
    of cloth between them: straight if taut, a sine bulge if slack, a U fold (two legs and a half circle of the clips'
    spacing) if gathered and ``fold``. ``bulge`` +1 falls behind the rod (+x), -1 in front, 0 behind."""
    d = abs(y1 - y0)
    e = 1.0 if y1 >= y0 else -1.0
    sig = -1.0 if bulge < 0 else 1.0
    if length < d - 1e-6:
        raise ValueError(f"clips {d * 1000:.1f} mm apart hold only {length * 1000:.1f} mm of cloth")
    if length <= d * 1.0005:
        t = np.linspace(0.0, 1.0, samples)
        return np.stack([np.zeros_like(t), y0 + e * d * t], 1)
    if fold and length >= math.pi * d / 2:
        r = d / 2
        leg = (length - math.pi * r) / 2
        s = np.linspace(0.0, length, samples)
        out = np.zeros((samples, 2))
        a = s <= leg
        out[a] = np.stack([sig * s[a], np.full(a.sum(), y0)], 1)
        b = (s > leg) & (s <= leg + math.pi * r)
        th = (s[b] - leg) / r
        out[b] = np.stack([sig * (leg + r * np.sin(th)), y0 + e * r * (1 - np.cos(th))], 1)
        c = s > leg + math.pi * r
        out[c] = np.stack([sig * (leg - (s[c] - leg - math.pi * r)), np.full(c.sum(), y1)], 1)
        return out
    t = np.linspace(0.0, 1.0, samples)

    def curve(amp):
        return np.stack([sig * amp * np.sin(math.pi * t), y0 + e * d * t], 1)

    lo, hi = 0.0, length
    for _ in range(80):
        mid = (lo + hi) / 2
        p = curve(mid)
        if np.linalg.norm(np.diff(p, axis=0), axis=1).sum() < length:
            lo = mid
        else:
            hi = mid
    return curve((lo + hi) / 2)


def heading_path(ring_u, ring_y, bulge, width, fold=True):
    """u (along the heading from the leading edge, 0..width) -> (dx, y): the cloth's top edge as it starts."""
    pts, us = [], []
    for k in range(len(ring_u) - 1):
        c = _gap_curve(ring_y[k], ring_y[k + 1], ring_u[k + 1] - ring_u[k], bulge[k], fold=fold)
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(c, axis=0), axis=1))])
        s *= (ring_u[k + 1] - ring_u[k]) / s[-1]
        pts.append(c)
        us.append(ring_u[k] + s)
    pts, us = np.concatenate(pts), np.concatenate(us)
    if ring_u[0] > 1e-9 or ring_u[-1] < width - 1e-9:
        raise ValueError("the first clip must hold the leading edge (u 0) and the last the outer edge (u = width)")
    # round the corners where one gap's shape meets the next (a mesh row across a sharp corner would start squeezed):
    # resample by arc length, average over +-SMOOTH, and spread the material evenly over the smoothed curve again
    keep = np.concatenate([[True], np.diff(us) > 1e-9])
    pts, us = pts[keep], us[keep]
    grid = np.arange(0.0, width + 1e-9, 0.0005)
    xy = np.stack([np.interp(grid, us, pts[:, 0]), np.interp(grid, us, pts[:, 1])], 1)
    k = int(round(SMOOTH / 0.0005))
    kern = np.ones(2 * k + 1) / (2 * k + 1)
    pad = np.concatenate([np.repeat(xy[:1], k, 0), xy, np.repeat(xy[-1:], k, 0)])
    xy = np.stack([np.convolve(pad[:, 0], kern, "valid"), np.convolve(pad[:, 1], kern, "valid")], 1)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    us, pts = s * width / s[-1], xy

    def at(u):
        u = np.asarray(u, float)
        return np.stack([np.interp(u, us, pts[:, 0]), np.interp(u, us, pts[:, 1])], -1)
    return at


@dataclass(frozen=True)
class Panel(ck.Cloth):
    """A cloth_kit sheet hung from clips. Material frame: u along the heading from the leading edge (0..width), v down
    from the top edge (0..length). Starts with its top edge on the path given by the clips (see :func:`heading_path`)
    at ``top_z`` and hanging straight down. ``pos`` is set by :func:`panel`."""

    rod_x: float = 0.0
    top_z: float = 0.0
    ring_u: tuple = ()
    ring_y: tuple = ()
    bulge: tuple = ()
    floor_z: float = -1.0e9     # cloth below this height (world z) lies on the floor/table instead, toward +x
    floor_bend: float = 0.006   # radius of the turn from hanging to lying

    # the offline rigid-body layout checker places an object it does not know as a small fixed block at ``xy``
    @property
    def xy(self):
        return (float(self.pos[0]), float(self.pos[1]))

    @property
    def extent(self):
        return 0.02

    def uv(self):
        """(n, 2) material coordinates (u from the leading edge along the heading, v down from the top edge)."""
        nodes = ck.sheet_mesh(self.width, self.length, self.edge)[0]
        return np.stack([nodes[:, 0] + self.width / 2, self.length / 2 - nodes[:, 1]], 1)

    def _local(self):
        nodes, faces, rings, cells = super()._local()
        uv = self.uv()
        at = heading_path(self.ring_u, self.ring_y, self.bulge, self.width, fold=self.floor_z < -1.0e8)
        xy = at(uv[:, 0])
        world = np.stack([self.rod_x + xy[:, 0], xy[:, 1], self.top_z - uv[:, 1]], 1)
        r = self.floor_bend
        hang = self.top_z - self.floor_z - r          # straight drop before the turn
        v = uv[:, 1]
        turn = (v > hang) & (v <= hang + math.pi * r / 2)
        phi = (v[turn] - hang) / r
        world[turn, 0] += r - r * np.cos(phi)
        world[turn, 2] = self.floor_z + r - r * np.sin(phi)
        flat = v > hang + math.pi * r / 2
        world[flat, 0] += r + (v[flat] - hang - math.pi * r / 2)
        world[flat, 2] = self.floor_z
        return world - np.asarray(self.pos, float), faces, rings, cells

    def hanging_rows(self) -> float:
        """Material v (down from the top edge) where the cloth reaches the floor at the start."""
        return float(self.top_z - self.floor_z)

    def clip_nodes(self, k) -> np.ndarray:
        """Indices of the nodes in clip k's bite."""
        uv = self.uv()
        return np.flatnonzero((np.abs(uv[:, 0] - self.ring_u[k]) <= BITE_U) & (uv[:, 1] <= BITE_V))


def panel(name, rod_x, top_z, ring_u, ring_y, bulge, width, length, floor_z=-1.0e9, **cloth):
    """A :class:`Panel` (``cloth`` = cloth_kit.Cloth fields: thickness, mass_kg, youngs, bend, edge, color, ...). With
    ``floor_z`` the part of the cloth longer than the drop to that height starts lying flat on it, behind (+x)."""
    base = ck.Cloth.sheet(name, pos=(rod_x, float(np.mean(ring_y)), top_z - length / 2), width=width, length=length,
                          **cloth)
    return Panel(**{f.name: getattr(base, f.name) for f in fields(base)}, rod_x=float(rod_x), top_z=float(top_z),
                 ring_u=tuple(float(v) for v in ring_u), ring_y=tuple(float(v) for v in ring_y),
                 bulge=tuple(int(v) for v in bulge), floor_z=float(floor_z))


# ------------------------------------------------------------------------------------- clips tied to the rings
_CLIPS: dict[str, list] = {}


def hang(panel_spec: Panel, rings) -> None:
    """Tie each clip's bite of ``panel_spec`` to its ring (in heading order), authored when the cloth is spawned."""
    if len(rings) != len(panel_spec.ring_u):
        raise ValueError("one ring per clip position")
    nodes = panel_spec.mesh(world=True)[0]
    plan = []
    for k, ring in enumerate(rings):
        ids = panel_spec.clip_nodes(k)
        if len(ids) < 2:
            raise ValueError(f"clip {k} of {panel_spec.name} bites only {len(ids)} nodes")
        plan.append((ring.name, ids, nodes[ids] - np.asarray(ring.pos, float)))
    _CLIPS[panel_spec.name] = plan
    _install()


def _install() -> None:
    if getattr(ck._cloth_object, "_curtain_kit", False):
        return
    inner = ck._cloth_object

    def cloth_object(spec):
        _hook_mesh_spawner()
        return inner(spec)

    cloth_object._curtain_kit = True
    ck._cloth_object = cloth_object


def _hook_mesh_spawner() -> None:
    from isaaclab.sim.spawners.meshes import meshes as mesh_spawners

    original = mesh_spawners._spawn_mesh_geom_from_mesh
    if getattr(original, "_curtain_kit", False):
        return

    def spawn(prim_path, *args, **kwargs):
        out = original(prim_path, *args, **kwargs)
        name = str(prim_path).rstrip("/").rsplit("/", 1)[-1]
        if name in _CLIPS:
            from isaaclab.sim.utils import get_current_stage

            _author_clips(kwargs.get("stage") or get_current_stage(), str(prim_path), name)
        return out

    spawn._curtain_kit = True
    mesh_spawners._spawn_mesh_geom_from_mesh = spawn


def _author_clips(stage, prim_path, name) -> None:
    from pxr import Sdf, Usd, Vt

    body = prim_path
    for prim in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
        if "OmniPhysicsDeformableBodyAPI" in prim.GetAppliedSchemas():
            body = str(prim.GetPath())
            break
    parent = prim_path.rsplit("/", 1)[0]
    for ring_name, ids, local in _CLIPS[name]:
        prim = stage.DefinePrim(f"{body}_clip_{ring_name}", "OmniPhysicsVtxXformAttachment")
        prim.CreateRelationship("omniphysics:src0").SetTargets([Sdf.Path(body)])
        prim.CreateRelationship("omniphysics:src1").SetTargets([Sdf.Path(f"{parent}/{ring_name}")])
        prim.CreateAttribute("omniphysics:attachmentEnabled", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("omniphysics:vtxIndicesSrc0", Sdf.ValueTypeNames.IntArray).Set([int(i) for i in ids])
        prim.CreateAttribute("omniphysics:localPositionsSrc1", Sdf.ValueTypeNames.Point3fArray).Set(
            Vt.Vec3fArray([tuple(float(v) for v in row) for row in local]))
    print(f"[curtain_kit] {name}: {len(_CLIPS[name])} clips tied to their rings ({body})", flush=True)


__all__ = ["CLOTH_DROP", "Panel", "RING_DROP", "ROD_R", "clip_ring", "hang", "heading_path", "panel"]
