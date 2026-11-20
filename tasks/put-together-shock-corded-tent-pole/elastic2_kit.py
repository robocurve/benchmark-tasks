"""elastic2_kit: an elastic LINE between graspable rigid ends, for YAM Arena task files (on yam_parts / yam_mesh).

What it models
    A stretchy cord, strap or rubber link whose two ends are rigid parts (a hook, toggle, T-handle, tab, ring, ball,
    or a point on a body). The strand itself is NOT a body: it is a massless Hooke line recomputed before every
    240 Hz physics step, the way the accepted bungee task (PR 1872) does it:

        tension = stiffness x (length - rest) + damping x d(length)/dt     (0 when slack, never negative)

    applied to both end bodies along the line, and to the body of every guide it bends round, through
    ``asset.permanent_wrench_composer.set_forces_and_torques_index(forces, torques, is_global=True)`` (force at the
    centre of mass plus the matching torque). The strand does not collide with anything. It is DRAWN as ``n_draw``
    kinematic pieces without colliders, moved once per control step: they touch end to end at the rest length and
    part as the line stretches (the visible sign of tension); a slack line is drawn sagging.

    Guides: fixed smooth points the line runs over, in order from ``a`` to ``b`` (no friction, like a polished edge
    or an eyelet). ``Guide(body, local)`` is an eyelet (the line always passes through it);
    ``Guide(body, local, normal=n)`` is a convex corner whose solid lies on the -n side: the line bends round it only
    while the straight run between its neighbours would cut through the solid, otherwise it lifts off.

Use (module level of a task file)::

    import elastic2_kit as e2
    BODY = e2.EPart("box", pos, shapes, mass=0.3, ...)            # anything a line attaches to or bends round
    TOGGLE = e2.EPart("toggle", pos, shapes, mass=0.012, ccd=True, damping=(0.5, 2.0))
    STRAP = e2.Line("strap", (BODY, anchor_local), (TOGGLE, attach_local), rest=0.165, stiffness=110.0,
                    damping=1.5, guides=[e2.Guide("box", edge_local, normal=(0, 1, 1))], floor=deck_z)
    e2.group("set", [BODY, TOGGLE])                               # one jitter offset for bodies joined by a line
    OBJECTS = [BODY, TOGGLE, *STRAP.segments]
    # scoring / demos:  STRAP.measure(states) -> {"length", "stretch", "tension", "points"};  e2.live("strap")

Proven on the GPU (Nov 20, a test task with a T-handle on a strap, three jittered runs)
    A 12 g T-handle on a 100 N/m, 1.2 N s/m line (rest 235 mm, over one corner guide) pinched by its 10 mm waist,
    stretched 64 mm (6.4 N) against a 0.3 kg box held by the other hand, set behind a fork keeper and released:
    it stayed, 0.00 mm of movement, in 3 of 3 jittered episodes (and 3 of 3 in the run before). Friction pinches
    alone held 10.9 N at 46 % stretch. 0 force errors, no NaN in 12 episodes.

Numbers that mattered
    * Explicit force per substep is stable only for end bodies heavy enough: keep stiffness x dt^2 / m_end < 0.3 and
      damping x dt / m_end < 0.45 (dt = 1/240 s), and damping >= 2.5 x stiffness x dt so that a freely flying end
      loses energy even if the force lands one substep late (12 g end, 100 N/m: damping 1.05 .. 1.3 N s/m). This
      is why the first spike's 0.1 g beads could not use it (ratio 69); rigid ends of 5 g or more can.
    * End bodies: 32 position iterations, 1 m/s max depenetration, 1 mm contact offset, speculative CCD, body
      damping (0.5, 2.0). Pinch a rigid waist of 10 mm or more, commanded 4 mm narrower (about 10 N a pad).
    * THE ARMS GIVE. Under a sideways pull on a top-down grip each arm's tool point moves 4-5 mm per newton (the
      10 N m/rad wrist joints sit 134 mm above it); along the fingers 1.2-2.2 mm/N; a level grip pulled along its
      fingers 0.4 mm/N (a simple model, checked against the runs). At 6.5 N the two hands closed 5-6 cm on each
      other. A demo must correct from a measured position every control step (a servo taking 0.15 of
      the error per step; one-shot corrections converge by only half each time) or stretch along the fingers.
    * Let go only with the end already resting against its catch (ease the hand back until the catch carries the
      pull, then open). A careless release left the handle hanging on a finger.
    * Judge stillness by position over 1 s: a body pressed on a contact by the line reports a steady phantom speed
      (0.029 m/s) while it does not move. Friction on the catch can hold a light end off the surface below it.

What it cannot do: the strand has no contact, so it cannot be pinched, cannot wrap or rub on anything except its
declared guides, has no weight, no friction on guides and no twist. The line is a model of the taut cord only.
"""
from __future__ import annotations

import functools
import importlib.abc
import importlib.machinery
import math
import sys
from dataclasses import dataclass

import numpy as np
import yam_mesh as ymesh
import yam_parts as parts
from yam_mesh import MeshPart

RUBBER = (0.08, 0.08, 0.09)
OPTS: dict[str, dict] = {}  # body name -> solver options
LINES: dict[str, "Line"] = {}
GROUPS: dict[str, tuple[str, ...]] = {}
SPECS: dict[str, "EPart"] = {}
DT = 1.0 / 240.0


# ---------------------------------------------------------------- specs
class EPart(MeshPart):
    """A body a line attaches to or bends round (free if it has a mass). Options: iterations (pos, vel), damping
    (linear, angular) 1/s, max_depen (m/s), ccd (speculative), ghost (kinematic, no collider: a drawn piece)."""

    def __init__(self, name, pos, shapes=(), mass=None, color=parts.GREY, yaw=0.0, friction=None,
                 contact_offset=0.001, grip=None, *, meshes=(), iterations=(32, 4), damping=(0.05, 0.05),
                 max_depen=1.0, ccd=False, ghost=False):
        super().__init__(name, pos, shapes, mass, color, yaw, friction, contact_offset, grip, meshes=meshes)
        OPTS[name] = {"iterations": tuple(iterations), "damping": tuple(damping), "max_depen": float(max_depen),
                      "ccd": bool(ccd), "ghost": bool(ghost)}
        SPECS[name] = self


@dataclass(frozen=True)
class Guide:
    """A fixed smooth point on ``body`` (None: the world) the line runs over. ``normal`` None: an eyelet (always
    on the line). Otherwise a convex corner, solid on the -normal side (body frame): active only while the straight
    run between its neighbours would pass on the solid side."""

    body: str | None
    local: tuple
    normal: tuple | None = None


def group(name, members):
    """Bodies that a seeded reset shifts together by one random x/y offset (the harness's own jitter skips them)."""
    GROUPS[name] = tuple(m.name if hasattr(m, "name") else str(m) for m in members)


def _name(body):
    return None if body is None else (body.name if hasattr(body, "name") else str(body))


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat_from_matrix(m):
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)
    q = np.asarray(q, float)
    return q / np.linalg.norm(q)


def _along(direction, up=(0.0, 0.0, 1.0)):
    """Rotation whose x axis lies along ``direction`` (z as near ``up`` as it can be)."""
    x = np.asarray(direction, float)
    n = np.linalg.norm(x)
    x = x / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
    u = np.asarray(up, float) if abs(x[2]) < 0.95 else np.array([1.0, 0.0, 0.0])
    z = u - (u @ x) * x
    z /= np.linalg.norm(z)
    return np.column_stack([x, np.cross(z, x), z])


def spec_frame(spec):
    """(origin, R) of a part at its start pose."""
    c, s = math.cos(spec.yaw), math.sin(spec.yaw)
    return np.asarray(spec.pos, float), np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def frame_of(state):
    """(origin, R) of a part from a scorer's object state."""
    origin = getattr(state, "origin", None)
    return np.asarray(state.pos if origin is None else origin, float), _rot(state.quat_xyzw)


_WORLD = (np.zeros(3), np.eye(3))


class Line:
    """A massless elastic line from ``a`` to ``b`` (each ``(EPart | name | None, local point)``; None = a fixed
    world point) over ``guides``.

    rest        relaxed length along the whole path (m)
    stiffness   N per metre of stretch of the whole line (state it as an estimate unless published)
    damping     N s/m on the stretch rate while taut (the rubber's own loss; keeps a light end from ringing)
    max_force   cap on the tension (N), a guard against a bad state, not a material property
    thickness   drawn thickness (m); n_draw pieces; floor: height a slack line is drawn resting on (None: no floor)
    """

    def __init__(self, name, a, b, rest, stiffness, damping=1.0, guides=(), max_force=40.0, color=RUBBER,
                 thickness=0.004, n_draw=12, floor=None):
        self.name = name
        self.a = (_name(a[0]), tuple(float(v) for v in a[1]))
        self.b = (_name(b[0]), tuple(float(v) for v in b[1]))
        self.rest, self.k, self.c, self.max_force = float(rest), float(stiffness), float(damping), float(max_force)
        self.guides = tuple(guides)
        self.color, self.thickness, self.n_draw, self.floor = color, float(thickness), int(n_draw), floor
        self.seg_len = self.rest / self.n_draw
        self.state = {"length": None, "tension": 0.0, "stretch": 0.0, "points": None, "active": (), "peak": 0.0}
        self._last = None
        frames = {n: spec_frame(SPECS[n]) for n in self.bodies() if n in SPECS}
        start = self.measure_frames(frames)
        if start["length"] > self.rest + 1e-6:
            print(f"elastic2_kit: line {name} starts stretched {1000 * (start['length'] - self.rest):.1f} mm "
                  f"({start['tension']:.2f} N)", flush=True)
        self.start_length = start["length"]
        self.segments = tuple(
            EPart(f"{name}_d{i:02d}", tuple(p), (parts.box((0, 0, 0), (self.seg_len, self.thickness, self.thickness)),),
                  mass=0.001, color=color, ghost=True)
            for i, (p, _) in enumerate(self.draw_poses(start["points"])))
        LINES[name] = self

    # -- geometry
    def bodies(self):
        return tuple(dict.fromkeys(n for n in (self.a[0], *(g.body for g in self.guides), self.b[0]) if n))

    @staticmethod
    def _pt(frames, body, local):
        o, R = _WORLD if body is None else frames[body]
        return o + R @ np.asarray(local, float)

    def path(self, frames):
        """(points a .. b, bodies of those points, indices into ``guides`` of the active ones)."""
        pa, pb = self._pt(frames, *self.a), self._pt(frames, *self.b)
        act = list(range(len(self.guides)))
        gp = [self._pt(frames, g.body, g.local) for g in self.guides]
        gn = []
        for g in self.guides:
            if g.normal is None:
                gn.append(None)
            else:
                n = (_WORLD if g.body is None else frames[g.body])[1] @ np.asarray(g.normal, float)
                gn.append(n / np.linalg.norm(n))
        changed = True
        while changed:
            changed = False
            pts = [pa, *(gp[i] for i in act), pb]
            for j, i in enumerate(act):
                if gn[i] is None:
                    continue
                p, q = pts[j], pts[j + 2]
                d = q - p
                u = float(np.clip((gp[i] - p) @ d / max(d @ d, 1e-12), 0.0, 1.0))
                if (p + u * d - gp[i]) @ gn[i] >= 0.0:  # the straight run passes on the free side: lifts off
                    act.pop(j)
                    changed = True
                    break
        pts = [pa, *(gp[i] for i in act), pb]
        names = [self.a[0], *(self.guides[i].body for i in act), self.b[0]]
        return pts, names, tuple(act)

    def measure_frames(self, frames):
        pts, names, act = self.path(frames)
        L = float(sum(np.linalg.norm(q - p) for p, q in zip(pts, pts[1:])))
        return {"length": L, "stretch": L - self.rest, "tension": min(self.k * max(L - self.rest, 0.0), self.max_force),
                "points": pts, "names": names, "active": act}

    def measure(self, states):
        """From a scorer's object states: length, stretch (m, negative = slack), static tension (N), path points."""
        return self.measure_frames({n: frame_of(states[n]) for n in self.bodies()})

    # -- dynamics (one physics substep)
    def reset(self):
        self._last = None
        self.state.update(length=None, tension=0.0, stretch=0.0, points=None, active=(), peak=0.0)

    def step(self, frames, dt=DT):
        """Advance one substep: returns {body: [(force, point), ...]} (world) and updates ``state``."""
        m = self.measure_frames(frames)
        L = m["length"]
        rate = 0.0 if self._last is None else float(np.clip((L - self._last) / dt, -10.0, 10.0))
        self._last = L
        T = 0.0
        if L > self.rest:
            T = float(np.clip(self.k * (L - self.rest) + self.c * rate, 0.0, self.max_force))
        pts, names = m["points"], m["names"]
        out: dict = {}
        for j, (p, body) in enumerate(zip(pts, names)):
            if body is None:
                continue
            f = np.zeros(3)
            for q in ([pts[j - 1]] if j > 0 else []) + ([pts[j + 1]] if j + 1 < len(pts) else []):
                d = q - p
                n = np.linalg.norm(d)
                if n > 1e-6:
                    f += T * d / n
            out.setdefault(body, []).append((f, p))
        self.state.update(length=L, tension=T, stretch=L - self.rest, points=pts, active=m["active"],
                          peak=max(self.state["peak"], T), rate=rate)
        return out

    # -- drawing
    def draw_poses(self, pts):
        """n_draw (centre, R) of the drawn pieces along the path: evenly spread when taut (they part), sagging
        when slack."""
        pts = [np.asarray(p, float) for p in pts]
        spans = np.array([np.linalg.norm(q - p) for p, q in zip(pts, pts[1:])])
        total = max(float(spans.sum()), 1e-9)
        cum = np.concatenate([[0.0], np.cumsum(spans)])
        slack = max(self.rest - total, 0.0)

        def at(s):
            i = min(int(np.searchsorted(cum, s, side="right")) - 1, len(pts) - 2)
            u = (s - cum[i]) / max(spans[i], 1e-9)
            p = pts[i] + u * (pts[i + 1] - pts[i])
            if slack > 0:
                w = s / total
                p = p - np.array([0.0, 0.0, math.sqrt(3 * total * slack / 8) * 4 * w * (1 - w)])
                if self.floor is not None and min(pts[i][2], pts[i + 1][2]) >= self.floor - 1e-3:
                    p[2] = max(p[2], self.floor + self.thickness / 2)  # a span lying over the floor rests on it
            return p

        out = []
        for k in range(self.n_draw):
            s0, s1 = total * k / self.n_draw, total * (k + 1) / self.n_draw
            p0, p1 = at(s0), at(s1)
            out.append(((p0 + p1) / 2, _along(p1 - p0)))
        return out


def live(name):
    """The line's state at the last physics substep (simulator only): length, stretch, tension, peak, points."""
    return LINES[name].state


# ---------------------------------------------------------------- runtime (Isaac Sim only; imported lazily)
_CFG = []
_LOG = {"force_err": 0, "draw_err": 0, "substeps": 0, "applied": 0}
_RUN = {"env": None, "t": None, "step": 0, "episode": 0}


def _spawn_body(prim_path, cfg, *args, **kwargs):
    from pxr import PhysxSchema, UsdPhysics

    spec = parts._SPECS[cfg.part_name]
    prim = ymesh._spawn_mesh_part(prim_path, cfg, *args, **kwargs)
    opts = OPTS.get(spec.name, {})
    if spec.mass is not None:
        rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        pi, vi = opts.get("iterations", (32, 4))
        rigid.CreateSolverPositionIterationCountAttr().Set(int(pi))
        rigid.CreateSolverVelocityIterationCountAttr().Set(int(vi))
        rigid.CreateMaxDepenetrationVelocityAttr().Set(float(opts.get("max_depen", 1.0)))
        ld, ad = opts.get("damping", (0.05, 0.05))
        rigid.CreateLinearDampingAttr().Set(float(ld))
        rigid.CreateAngularDampingAttr().Set(float(ad))
        rigid.CreateSleepThresholdAttr().Set(0.0)
        rigid.CreateStabilizationThresholdAttr().Set(0.0)
        if opts.get("ghost"):  # a drawn piece: kinematic, no collider (as PR 1872's cord pieces)
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
            for child in prim.GetChildren():
                if child.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(child).CreateCollisionEnabledAttr(False)
                    try:
                        child.RemoveAPI(PhysxSchema.PhysxCollisionAPI)
                        child.RemoveAPI(UsdPhysics.CollisionAPI)
                    except Exception as e:  # noqa: BLE001  (collisionEnabled = False already keeps it out of contact)
                        print(f"ELASTIC2 ghost {spec.name}: collider kept but disabled ({e!r})", flush=True)
        elif opts.get("ccd"):  # speculative CCD needs no scene-wide flag
            rigid.CreateEnableSpeculativeCCDAttr().Set(True)
    return prim


def _e_object(spec):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass

    obj = ymesh._mesh_part_object(spec)
    if not _CFG:
        _CFG.append((ymesh.spawner_config_type(sim_utils.SpawnerCfg, configclass), clone(_spawn_body)))
    cfg_type, spawn = _CFG[0]
    obj.spawner_cfg = obj.object_cfg.spawn = cfg_type(func=spawn, part_name=spec.name)
    return obj


def _np(t):
    t = getattr(t, "torch", t)
    return t.detach().cpu().numpy().astype(float)


def body_frames(env, names):
    """name -> (origin, R) in env coordinates: free bodies read from the simulator, fixed ones from their spec."""
    base = env.unwrapped
    origin = _np(base.scene.env_origins)[0]
    out = {}
    for n in names:
        spec = SPECS.get(n)
        if spec is not None and spec.mass is None:
            out[n] = spec_frame(spec)
            continue
        pose = _np(base.scene[n].data.root_link_pose_w)[0]
        out[n] = (pose[:3] - origin, _rot(pose[3:7]))
    return out


def _apply(env, name, pairs):
    """Sum of (force, point) pairs (world, env frame) on a free body for the next physics step: the force at the
    centre of mass plus the matching torque (the call PR 1872 makes)."""
    import torch

    base = env.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins)[0]
    com = _np(asset.data.root_com_pos_w)[0][:3] - origin
    f, tq = np.zeros(3), np.zeros(3)
    for force, point in pairs:
        force = np.asarray(force, float)
        f += force
        tq += np.cross(np.asarray(point, float) - com, force)
    dev = asset.device
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=torch.tensor([[f.tolist()]], dtype=torch.float32, device=dev),
        torques=torch.tensor([[tq.tolist()]], dtype=torch.float32, device=dev), is_global=True)


def substep(env):
    """Before every physics step: recompute every line and apply its pulls."""
    if not LINES:
        return
    try:
        names = sorted({n for ln in LINES.values() for n in ln.bodies()})
        frames = body_frames(env, names)
        loads: dict = {n: [] for n in names if SPECS.get(n) is None or SPECS[n].mass is not None}
        for ln in LINES.values():
            for body, pairs in ln.step(frames, _RUN.get("dt", DT)).items():
                if body in loads:
                    loads[body] += pairs
        for body, pairs in loads.items():
            _apply(env, body, pairs or [(np.zeros(3), frames[body][0])])
        _LOG["substeps"] += 1
        _LOG["applied"] += len(loads)
    except Exception as e:  # noqa: BLE001
        _LOG["force_err"] += 1
        if _LOG["force_err"] <= 3:
            import traceback

            print(f"ELASTIC2 force error: {e!r}\n{traceback.format_exc()}", flush=True)


def _wrap_scene(env):
    sc = env.unwrapped.scene
    if getattr(sc, "_elastic2_wrapped", False):
        return
    orig = sc.write_data_to_sim
    _RUN["dt"] = float(getattr(env.unwrapped, "physics_dt", DT) or DT)

    def write_data_to_sim(*a, **k):
        sys.modules[__name__].substep(env)
        return orig(*a, **k)

    sc.write_data_to_sim = write_data_to_sim
    sc._elastic2_wrapped = True
    print(f"ELASTIC2 substep hook on, dt={_RUN['dt']:.5f} s, lines {sorted(LINES)}", flush=True)


def draw(env):
    """Move every line's drawn pieces (once per control step)."""
    try:
        import torch

        base = env.unwrapped
        origin = base.scene.env_origins.torch[0] if hasattr(base.scene.env_origins, "torch") else base.scene.env_origins[0]
        for ln in LINES.values():
            pts = ln.state["points"]
            if pts is None:
                pts = ln.measure_frames(body_frames(env, ln.bodies()))["points"]
            for seg, (p, R) in zip(ln.segments, ln.draw_poses(pts)):
                t = torch.tensor([[*p, *_quat_from_matrix(R)]], dtype=torch.float32, device=origin.device)
                t[:, :3] += origin
                base.scene[seg.name].write_root_pose_to_sim_index(root_pose=t)
    except Exception as e:  # noqa: BLE001  (drawing is cosmetic: never stop a run for it)
        _LOG["draw_err"] += 1
        if _LOG["draw_err"] <= 3:
            print(f"ELASTIC2 drawing failed: {e!r}", flush=True)


def _group_members(task_objects):
    names = {o.name for o in task_objects}
    return {g: tuple(m for m in mem if m in names) for g, mem in GROUPS.items() if any(m in names for m in mem)}


def _around_env_reset(base, env_reset, seed, jitter, groups, args, kwargs):
    result = env_reset(*args, **kwargs)
    if seed is not None and jitter > 0 and groups:
        import torch

        for gi, (gname, members) in enumerate(sorted(groups.items())):
            d = np.random.default_rng([int(seed), 4411, gi]).uniform(-jitter, jitter, size=2)
            for n in members:
                obj = base.scene[n]
                pose = obj.data.default_root_pose.torch.clone()
                pose[:, :3] += base.scene.env_origins
                pose[:, :2] += torch.tensor(d, dtype=pose.dtype, device=pose.device)
                obj.write_root_pose_to_sim_index(root_pose=pose)
                obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
            print(f"ELASTIC2_JITTER {gname} seed={seed} dx_mm={d[0] * 1000:.1f} dy_mm={d[1] * 1000:.1f}", flush=True)
    return result


def _patch_scene(scene):
    ymesh._patch_scene(scene)
    scene._elastic2_kit_impl = sys.modules[__name__]
    if getattr(scene, "_elastic2_kit_patched", False):
        return
    inner = scene.task_object

    @functools.wraps(inner)
    def task_object(spec):
        if any(c.__name__ == "EPart" for c in type(spec).__mro__):
            return scene._elastic2_kit_impl._e_object(spec)
        return inner(spec)

    scene.task_object = task_object
    scene._elastic2_kit_patched = True


class _SceneFinder(importlib.abc.MetaPathFinder):
    _elastic2_kit_finder = True

    def __init__(self, patch):
        self.patch = patch

    def find_spec(self, fullname, path=None, target=None):
        if fullname != "yam_arena.scene":
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is not None and spec.loader is not None:
            spec.loader = parts._SceneLoader(spec.loader, self.patch)
        return spec


def _install_embodiment():
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment as cls
    except Exception:  # noqa: BLE001  (offline: no simulator)
        return
    cls._elastic2_kit_impl = sys.modules[__name__]
    if getattr(cls, "_elastic2_kit_hooked", False):
        return
    ensure0, reset0, step0 = cls._ensure_sim, cls.reset, cls._task_step

    def _ensure_sim(self, goal):
        ensure0(self, goal)
        impl = cls._elastic2_kit_impl
        if self.task is not None and hasattr(self, "_free_objects"):
            skip = {m for mem in impl._group_members(self.task.objects).values() for m in mem}
            skip |= {n for n, o in impl.OPTS.items() if o.get("ghost")}
            self._free_objects = tuple(n for n in self._free_objects if n not in skip)
        if self._env is not None and impl.LINES:
            impl._wrap_scene(self._env)

    def reset(self, scene, *, seed=None):
        impl = cls._elastic2_kit_impl
        for ln in impl.LINES.values():
            ln.reset()
        groups = impl._group_members(self.task.objects) if self.task is not None else {}
        if not groups:
            out = reset0(self, scene, seed=seed)
        else:
            self._ensure_sim(None)
            base = self._env.unwrapped
            had, saved = "reset" in vars(base), vars(base).get("reset")
            env_reset = base.reset
            jitter = self.object_jitter
            base.reset = lambda *a, **kw: impl._around_env_reset(base, env_reset, seed, jitter, groups, a, kw)
            try:
                out = reset0(self, scene, seed=seed)
            finally:
                if had:
                    base.reset = saved
                else:
                    del base.reset
        impl._RUN.update(env=self._env, t=0.0, episode=impl._RUN["episode"] + 1)
        impl.draw(self._env)
        for ln in impl.LINES.values():
            st = ln.state
            if st["length"] is not None:
                print(f"ELASTIC2 episode {impl._RUN['episode']} line {ln.name}: length {st['length'] * 1000:.1f} mm "
                      f"(rest {ln.rest * 1000:.1f}), tension {st['tension']:.2f} N, peak in settle {st['peak']:.2f} N, "
                      f"guides on {st['active']}", flush=True)
            st["peak"] = 0.0
        print(f"ELASTIC2 episode {impl._RUN['episode']}: substeps {impl._LOG['substeps']}, body loads set "
              f"{impl._LOG['applied']}, force errors {impl._LOG['force_err']}, draw errors {impl._LOG['draw_err']}",
              flush=True)
        return out

    def _task_step(self, *args, **kwargs):
        impl = cls._elastic2_kit_impl
        impl._RUN.update(env=self._env, t=self._t, step=impl._RUN["step"] + 1)
        impl.draw(self._env)
        return step0(self, *args, **kwargs)

    cls._ensure_sim, cls.reset, cls._task_step = _ensure_sim, reset, _task_step
    cls._elastic2_kit_hooked = True


def install():
    finder = next((f for f in sys.meta_path if getattr(f, "_elastic2_kit_finder", False)), None)
    if finder is None:
        finder = _SceneFinder(_patch_scene)
    else:
        sys.meta_path.remove(finder)
        finder.patch = _patch_scene
    sys.meta_path.insert(0, finder)
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    _install_embodiment()


# ---------------------------------------------------------------- the hands (diagnostics, demos)
def finger_state(env):
    """Per arm: (pads' gap m, mean finger drive force N = stiffness x (target - position), capped)."""
    from yam_arena.constants import FINGER_JOINTS, GRIPPER

    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        q = _np(art.data.joint_pos)[0][ids]
        qt = _np(art.data.joint_pos_target)[0][ids]
        f = np.clip(GRIPPER["stiffness"] * (qt - q), -GRIPPER["effort"], GRIPPER["effort"])
        out[arm] = (float(-2 * np.mean(q)), float(np.mean(f)))
    return out


def tcp_frames(env, target=False):
    """Per arm: (TCP position, TCP rotation) from the measured joints (or the commanded ones), env coordinates."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        q = _np(art.data.joint_pos_target if target else art.data.joint_pos)[0][ids]
        out[arm] = ik.tcp_pose(q, station.BASES[arm])
    return out


install()
