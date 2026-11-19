"""cableshape: opt-in pre-shaped starting layouts for cablekit cables (S-curves, crossings, coils, pre-wound spools).

cablekit spawns every cable straight. This module reshapes a cable at the start of each episode, inside the
embodiment's reset, before the policy's first observation. It never changes cablekit.py: a task opts in by
importing it (symlink it next to task.py like cablekit.py / boxfont.py) and registering a reset hook.

    import cableshape
    from cableshape import on_reset, preshape, prewind, settle

    def _shape(emb):                       # runs after every reset (emb = the YamArenaEmbodiment)
        preshape(emb, CORD, [(0.20, 0.25), (0.30, 0.15), (0.20, 0.05), (0.30, -0.05)], settle_s=0)
        prewind(emb, HOSE, SPOOL, turns=2.5, settle_s=0)
        settle(emb, 1.5)                   # one shared settle for every reshaped cable
    on_reset(_shape)

How it works (PhysX, measured 2026-11-05)
-----------------------------------------
The cable is a reduced-coordinate articulation rooted at ``seg_0``; each bend joint has two DOFs, ``bend_i:1``
(rotY, positive bends the cable down) and ``bend_i:2`` (rotZ, positive turns it left / counter-clockwise from above).
PhysX's two-axis swing convention is the exponential map: the child segment's frame is
``parent * exp([0, q1, q2])``. So a target centreline is turned into joint coordinates by

1. walking the polyline in chords of exactly one segment length (segment centres + directions),
2. framing the segments with zero-twist (parallel-transport / Bishop) frames, since the joints cannot twist,
3. reading each joint's relative rotation as a rotation vector (its y, z parts are the two DOFs).

The root pose (seg_0), every joint position, zero velocities and the drive targets are then written with
``write_root_pose_to_sim_index`` / ``write_joint_state_to_sim_index`` / ``set_joint_position_target_index``.
These writes DO reach PhysX (the older AUTHORING note said they did not: that attempt bent the cable downwards
into the table, which NaNs; bending in the table plane or upwards is fine). One bare physics substep is run right
after the write: without it the next ``env.step()`` dropped the write and the cable sat straight again. Afterwards the cable settles under
gravity and friction for ``settle_s`` with the arms held at READY. No bodies or joints are added to the scene, so
nothing is left behind in the observations: with ``hold="none"`` or ``"ramp"`` the drives end exactly at the
kit's straight targets.

``hold`` (what the bend drives do after the shape is written):
- ``"none"``: drive targets straight at once. The cable springs back as far as table friction allows (ends
  straighten first). Good for floppy cords (bend_stiffness <= ~0.005).
- ``"ramp"``: drives hold the shape while the cable lands, then the targets ramp to straight over ``ramp_s``.
  Same end state physics as "none", with less dynamic recoil (smaller springback for stiffer cords).
- ``"set"``: the drive targets STAY at the shape for the whole episode: the cord has a memory/set, like a hose
  taken off a reel or a cord that was coiled for months. It straightens only as far as it is pulled. Say so in
  the task's fidelity text ("cord has coil memory").

Determinism: the same task on the same machine gives the same settled state (writes are exact; GPU PhysX is
deterministic for identical inputs). ``settled(name)`` returns the measured layout after the last reset.

Limits (see PATTERNS.md "Pre-shaped starts"):
- Joint angle per segment must stay under the kit's max_bend (1.2 rad); keep it <= 0.8 rad: centreline radius
  >= seg_len / 0.8 = 2.5 cm for the default 2 cm segments. ``preshape`` raises above 1.1 rad.
- Never write a shape that bends DOWN into the table or intersects fixed parts/other cables: deep initial
  penetration NaNs. Give 2D waypoints (z = the cable's spawn height) or lift crossings with ``over()``.
- Anchored cables (anchor_start=True) keep seg_0 where it spawned; the shape starts there along the spawn yaw.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

__all__ = ["on_reset", "preshape", "prewind", "settle", "settled", "shape_points", "helix", "over", "wind_path",
           "spawn_for", "Shaped"]

_SETTLED: dict[str, np.ndarray] = {}  # cable name -> (segments, 3) settled segment centres after the last reset
_RAMPS: list = []  # (art, q_shape, ramp_steps) waiting for settle()


@dataclass
class Shaped:
    """What preshape wrote: target segment centres (n,3), directions (n,3), joint coordinates (n-1, 2)."""

    centres: np.ndarray
    dirs: np.ndarray
    q: np.ndarray
    frames: list | None = None  # zero-twist segment frames (3x3, columns x y z)


# -- pure geometry (numpy only; usable offline, e.g. to draw the intended layout or check reach) ---------------


def _rotvec_to_mat(v: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def _mat_to_rotvec(R: np.ndarray) -> np.ndarray:
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(th))
    return w * th


def _min_rot(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Smallest rotation taking unit vector a to unit vector b."""
    ax = np.cross(a, b)
    s, c = np.linalg.norm(ax), float(np.dot(a, b))
    if s < 1e-12:
        return np.eye(3)
    return _rotvec_to_mat(ax / s * math.atan2(s, c))


def _mat_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def _quat_xyzw_to_mat(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _as_path(spec, waypoints) -> np.ndarray:
    p = np.asarray(waypoints, dtype=float)
    if p.ndim != 2 or p.shape[1] not in (2, 3) or len(p) < 2:
        raise ValueError("waypoints: need >= 2 points, each (x, y) or (x, y, z)")
    if p.shape[1] == 2:
        p = np.c_[p, np.full(len(p), float(spec.start[2]))]
    return p


def _chords(path: np.ndarray, n: int, L: float, first: np.ndarray | None = None) -> np.ndarray:
    """n+1 nodes along ``path`` with consecutive nodes exactly L apart (straight extension past the end)."""
    seg = np.diff(path, axis=0)
    lens = np.linalg.norm(seg, axis=1)
    keep = lens > 1e-9
    path = np.vstack([path[:1], path[1:][keep]])
    dense = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        k = max(1, int(math.ceil(np.linalg.norm(b - a) / (L / 40))))
        dense += [a + (b - a) * t for t in np.linspace(0, 1, k + 1)[1:]]
    dense = np.array(dense)
    tail = dense[-1] - dense[-2]
    tail /= np.linalg.norm(tail)
    nodes = [dense[0] if first is None else first]
    j = 0
    while len(nodes) < n + 1:
        c = nodes[-1]
        while j < len(dense) and np.linalg.norm(dense[j] - c) < L:
            j += 1
        if j >= len(dense):  # extend straight along the last direction
            nodes.append(c + L * tail if np.linalg.norm(dense[-1] - c) < 1e-9 else _sphere_line(c, dense[-1], tail, L))
            continue
        a, b = dense[j - 1], dense[j]  # |a-c| < L <= |b-c|: solve |a + t(b-a) - c| = L
        nodes.append(_sphere_seg(c, a, b, L))
    return np.array(nodes)


def _sphere_seg(c, a, b, L):
    d = b - a
    f = a - c
    A, B, C = d @ d, 2 * f @ d, f @ f - L * L
    t = (-B + math.sqrt(max(B * B - 4 * A * C, 0.0))) / (2 * A)
    return a + min(max(t, 0.0), 1.0) * d


def _sphere_line(c, p, u, L):
    f = p - c
    B, C = 2 * f @ u, f @ f - L * L
    t = (-B + math.sqrt(max(B * B - 4 * C, 0.0))) / 2
    return p + t * u


def shape_points(spec, waypoints, *, anchored: bool | None = None) -> Shaped:
    """Segment centres, directions and joint coordinates for laying cable ``spec`` along ``waypoints``.

    ``waypoints``: (x, y) or (x, y, z) centreline points (world frame, z = cable centre height; 2D uses the
    cable's spawn height), starting at the cable's start end. The cable is laid from the first waypoint (for an
    anchored cable: from its fixed start, along its spawn yaw for one segment, then towards the waypoints);
    past the last waypoint it continues straight. Pure numpy.
    """
    n, L = spec.segments, spec.seg_len
    path = _as_path(spec, waypoints)
    anchored = spec.anchor_start if anchored is None else anchored
    if anchored:
        s0 = np.asarray(spec.start, dtype=float)
        path = np.vstack([s0, s0 + L * spec.direction, path])
        path = np.vstack([path[:2], path[2:][np.linalg.norm(path[2:] - path[1], axis=1) > 1e-6]])
    nodes = _chords(path, n, L)
    dirs = np.diff(nodes, axis=0) / L
    centres = (nodes[:-1] + nodes[1:]) / 2
    # zero-twist frames: x along the segment, seg_0 with a horizontal y axis
    x0 = dirs[0]
    y0 = np.cross([0.0, 0.0, 1.0], x0)
    y0 = y0 / np.linalg.norm(y0) if np.linalg.norm(y0) > 1e-6 else np.array([0.0, 1.0, 0.0])
    F = [np.c_[x0, y0, np.cross(x0, y0)]]
    q = np.zeros((n - 1, 2))
    for i in range(n - 1):
        Fn = _min_rot(dirs[i], dirs[i + 1]) @ F[-1]
        rv = _mat_to_rotvec(F[-1].T @ Fn)  # x part ~0 by construction
        q[i] = rv[1], rv[2]
        F.append(Fn)
    return Shaped(centres=centres, dirs=dirs, q=q, frames=F)


def helix(centre_xy, radius: float, turns: float, *, z_top: float, z_bottom: float, start_angle: float = 0.0,
          sense: int = 1, tail: float = 0.3, tail_dir: str = "tangent", step: float = 0.004) -> np.ndarray:
    """Waypoints winding ``turns`` turns round a vertical axis at ``centre_xy`` (centreline ``radius``), from
    z_top down to z_bottom (the inner end starts at the top, the last turn ends on the bottom), then a straight
    ``tail`` (m) along the tangent at z_bottom. ``sense`` +1 = counter-clockwise from above."""
    cx, cy = centre_xy
    tot = 2 * math.pi * turns
    m = max(8, int(radius * tot / step))
    ph = np.linspace(0, tot, m + 1)
    a = start_angle + sense * ph
    z = z_top + (z_bottom - z_top) * ph / tot
    pts = np.c_[cx + radius * np.cos(a), cy + radius * np.sin(a), z]
    t = np.array([-math.sin(a[-1]), math.cos(a[-1]), 0.0]) * sense
    if tail > 0:
        pts = np.vstack([pts, pts[-1] + tail * t])
    return pts


def over(path, under, *, r_self: float, r_under: float, z_under: float, gap: float = 0.0015,
         width: float = 0.05, where=None, step: float = 0.002) -> np.ndarray:
    """``path`` (N,2|3 centreline) raised where it passes over the centreline ``under`` (M,2|3): the cable centre
    goes to z_under + r_under + r_self + gap within ~2 cm of the crossing and blends back down (cosine) over ``width`` either
    side. 2D paths start at z_under. ``where(xy) -> bool`` limits which crossings are raised (e.g.
    ``lambda p: p[1] > 0``); pair two calls with opposite ``where`` for an over-under weave. Returns dense (N',3)
    points every ``step`` m. Gravity drapes the raised part after the write."""
    p = np.asarray(path, dtype=float)
    if p.shape[1] == 2:
        p = np.c_[p, np.full(len(p), z_under)]
    u = np.asarray(under, dtype=float)[:, :2]

    def dense(q):
        out = []
        for a, b in zip(q[:-1], q[1:]):
            k = max(1, int(math.ceil(np.linalg.norm(b[:2] - a[:2]) / step)))
            out += [a + (b - a) * t for t in np.linspace(0, 1, k, endpoint=False)]
        return np.array(out + [q[-1]])

    dp, du = dense(p), dense(u)
    top = z_under + r_under + r_self + gap
    out = dp.copy()
    for i, q in enumerate(dp[:, :2]):
        dist = np.linalg.norm(du - q, axis=1)
        j = int(dist.argmin())
        if where is not None and not where(du[j]):
            continue
        d = dist[j] - (r_under + r_self) - 0.012  # flat top 12 mm beyond touching (2 cm segments are straight)
        w = 1.0 if d <= 0 else max(0.0, 1.0 - d / width)
        w = 0.5 - 0.5 * math.cos(math.pi * w)
        out[i, 2] = max(dp[i, 2], dp[i, 2] + w * (top - dp[i, 2]))
    return out


# -- Isaac side -------------------------------------------------------------------------------------------------


def _ctx(emb):
    """(gym env, unwrapped env, ready action) from the YamArenaEmbodiment."""
    env = getattr(emb, "_env", emb)
    base = env.unwrapped
    ready = getattr(emb, "_ready", None)
    if ready is None:
        import torch
        from yam_arena.constants import READY

        ready = torch.tensor([[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device=base.device)
    return env, base, ready


def _t(x):
    return getattr(x, "torch", x)


def _dof_index(art) -> tuple[list[int], list[int]]:
    names = list(art.joint_names)
    i1 = [names.index(f"bend_{i}:1") for i in range(1, len(names) // 2 + 1)]
    i2 = [names.index(f"bend_{i}:2") for i in range(1, len(names) // 2 + 1)]
    return i1, i2


def preshape(emb, spec, waypoints, *, settle_s: float = 1.5, hold: str = "ramp", hold_s: float = 0.5,
             ramp_s: float = 1.0, lift: float = 0.0015, max_angle: float = 1.1) -> Shaped:
    """Lay cable ``spec`` (a cablekit Cable) along ``waypoints`` now, then settle ``settle_s`` seconds.

    Call it from an :func:`on_reset` hook. ``emb`` is the embodiment passed to the hook. ``lift`` (m) raises the
    written shape so no segment starts inside the table (the cable lands within a few steps). ``hold``: "none",
    "ramp" (hold the shape ``hold_s`` while it lands, then ramp the drive targets to straight over ``ramp_s``;
    both happen inside :func:`settle`) or "set" (keep the shape as the drives' rest shape all episode). Use
    ``settle_s=0`` to shape several cables and call :func:`settle` once. Returns what was written.
    """
    import torch

    if hold not in ("none", "ramp", "set"):
        raise ValueError("hold must be 'none', 'ramp' or 'set'")
    sh = shape_points(spec, waypoints)
    if np.abs(sh.q).max() > max_angle:
        i = int(np.abs(sh.q).max(axis=1).argmax())
        raise ValueError(f"{spec.name}: joint {i + 1} would bend {np.abs(sh.q).max():.2f} rad (> {max_angle}); "
                         f"centreline radius must be >= ~{spec.seg_len / 0.8:.3f} m")
    env, base, _ = _ctx(emb)
    art = base.scene[spec.name]
    dev = base.device
    origin = _t(base.scene.env_origins)[0]
    i1, i2 = _dof_index(art)
    jp = _t(art.data.joint_pos)
    q = torch.zeros_like(jp)
    q[0, i1] = torch.tensor(sh.q[:, 0], dtype=q.dtype, device=dev)
    q[0, i2] = torch.tensor(sh.q[:, 1], dtype=q.dtype, device=dev)
    if not spec.anchor_start:
        pose = _t(art.data.root_pose_w).clone()
        c0 = sh.centres[0] + np.array([0.0, 0.0, lift])
        pose[0, :3] = origin + torch.tensor(c0, dtype=pose.dtype, device=dev)
        pose[0, 3:7] = torch.tensor(_mat_to_quat_xyzw(sh.frames[0]), dtype=pose.dtype, device=dev)
        art.write_root_pose_to_sim_index(root_pose=pose)
        art.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=pose.dtype, device=dev))
    art.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q))
    keep = hold in ("ramp", "set")
    art.set_joint_position_target_index(target=q if keep else torch.zeros_like(q))
    # One bare physics substep right away. Without it the first env.step() after the write lost the whole write
    # (cable back at its straight spawn layout; measured 2026-11-05); with it the write sticks (readback exact).
    base.scene.write_data_to_sim()
    base.sim.step(render=False)
    base.scene.update(dt=base.physics_dt)
    if _DEBUG:
        jq = _t(art.data.joint_pos)[0, i2[:4]].cpu().numpy()
        print(f"[cableshape] {spec.name} wrote q2[:4]={np.round(sh.q[:4, 1], 3)} after 1 substep {np.round(jq, 3)}",
              flush=True)
    if hold == "ramp":
        _RAMPS.append((art, q.clone(), hold_s, ramp_s))
    _NAMES.append(spec)
    if settle_s > 0:
        settle(emb, settle_s)
    return sh


_NAMES: list = []
_DEBUG = bool(__import__("os").environ.get("CABLESHAPE_DEBUG"))


def wind_path(spec, spool, turns: float, *, pitch: float | None = None, clearance: float = 0.0015,
              start_angle: float | None = None, sense: int | None = None, z_bottom: float | None = None,
              tail: list | None = None) -> np.ndarray:
    """Waypoints for :func:`prewind` (pure numpy): ``turns`` turns round the vertical cylinder ``spool`` (a
    yam_arena Cylinder or (x, y, radius)), inner end on top, last turn at ``z_bottom``, then a straight tail along
    the tangent (or the ``tail`` waypoints)."""
    cx, cy, rs = (spool.pos[0], spool.pos[1], spool.radius) if hasattr(spool, "radius") else spool
    r = spec.radius
    pitch = 2 * r + 0.004 if pitch is None else pitch
    zb = float(spec.start[2]) if z_bottom is None else z_bottom
    # straight 2 cm chords cut inside the helix by their sagitta, and the octagonal collider's corners reach
    # r / cos(pi/8): keep both clear of the spool (a written shape that penetrates it NaNs at once)
    R0 = rs + r / math.cos(math.pi / 8) + clearance
    R = R0 + spec.seg_len ** 2 / (8 * R0)
    if start_angle is None:
        start_angle = math.atan2(spec.start[1] - cy, spec.start[0] - cx)
    if sense is None:  # wind the way the spawn yaw points round the spool
        rad = np.array([spec.start[0] - cx, spec.start[1] - cy, 0.0])
        sense = 1 if np.cross(rad, spec.direction)[2] >= 0 else -1
    rest = max(spec.length - R * 2 * math.pi * turns, 0.0) + 0.05
    pts = helix((cx, cy), R, turns, z_top=zb + pitch * turns, z_bottom=zb, start_angle=start_angle, sense=sense,
                tail=0.0 if tail is not None else rest)
    if tail is not None:
        pts = np.vstack([pts, _as_path(spec, tail)])
    return pts


def prewind(emb, spec, spool, turns: float, *, pitch: float | None = None, clearance: float = 0.0015,
            start_angle: float | None = None, sense: int | None = None, z_bottom: float | None = None,
            tail: list | None = None, settle_s: float = 1.5, hold: str = "set", **kw) -> Shaped:
    """Wind cable ``spec`` ``turns`` times round the vertical cylinder ``spool`` now (see :func:`wind_path`), then
    settle. ``pitch`` defaults to 2*radius + 4 mm (touching turns grind), ``clearance`` is the gap to the spool,
    ``z_bottom`` defaults to the cable's spawn height (last turn lying on the table), ``start_angle`` (rad round
    the spool) to the direction of ``spec.start``, ``sense`` (+1 = counter-clockwise from above) to the way the
    spawn yaw points. ``hold`` defaults to "set" (the cord keeps its coil: real cords taken off a reel do); with
    "none"/"ramp" a coil springs open against the spool as far as friction allows. Other keywords go to
    :func:`preshape`. A free cable may spawn anywhere (it is moved); see :func:`spawn_for` to attach the inner end
    to a hub with a joint.
    """
    pts = wind_path(spec, spool, turns, pitch=pitch, clearance=clearance, start_angle=start_angle, sense=sense,
                    z_bottom=z_bottom, tail=tail)
    return preshape(emb, spec, pts, settle_s=settle_s, hold=hold, **kw)


def spawn_for(spec, waypoints, lift: float = 0.0015) -> dict:
    """Spawn arguments that put seg_0 exactly where :func:`preshape` will write it: returns
    {"start": (x, y, z), "yaw": rad, "seg0_pos": (3,), "seg0_quat_xyzw": (4,)}. Rebuild the cable with
    ``cable(name, start=d["start"], yaw=d["yaw"], ...)`` when its inner end must be joined to another body at spawn
    (e.g. a FixedJoint to a turning reel hub in a spawn wrap, using seg0_pos/seg0_quat for the joint frame). The
    spawn can't pitch, so a sloped first segment is off by its slope (a few hundredths of a rad) until the write."""
    sh = shape_points(spec, waypoints)
    d = sh.dirs[0]
    c0 = sh.centres[0] + np.array([0.0, 0.0, lift])
    yaw = math.atan2(d[1], d[0])
    start = c0 - spec.seg_len / 2 * np.array([math.cos(yaw), math.sin(yaw), 0.0])
    return {"start": tuple(float(v) for v in start), "yaw": yaw, "seg0_pos": c0,
            "seg0_quat_xyzw": _mat_to_quat_xyzw(sh.frames[0])}


def settle(emb, seconds: float) -> None:
    """Step the sim ``seconds`` with both arms at READY, running pending "ramp" releases, then record each
    reshaped cable's settled points (see :func:`settled`) and check them for NaN."""
    import torch

    env, base, ready = _ctx(emb)
    dt = base.step_dt
    steps = max(1, round(seconds / dt), *(round((hs + rs) / dt) + 1 for _, _, hs, rs in _RAMPS))
    for k in range(steps):
        t = k * dt
        for art, q, hs, rs in _RAMPS:
            if t >= hs:
                a = max(0.0, 1.0 - (t - hs) / max(rs, 1e-6))
                art.set_joint_position_target_index(target=q * a)
        env.step(ready)
    for art, q, _, _ in _RAMPS:
        art.set_joint_position_target_index(target=torch.zeros_like(q))
    _RAMPS.clear()
    from cablekit import _cable_state

    for spec in _NAMES:
        pts = _cable_state(env, spec).points
        if not np.isfinite(pts).all():
            raise RuntimeError(f"cableshape: {spec.name} went NaN while settling")
        _SETTLED[spec.name] = pts.copy()
    _NAMES.clear()


def settled(name: str) -> np.ndarray | None:
    """Segment centres (n,3, env frame) of cable ``name`` right after the last reset's shaping, or None."""
    return _SETTLED.get(name)


def on_reset(fn, *, renders: int = 4) -> None:
    """Run ``fn(emb)`` after every embodiment reset (after the arms settle at READY), then re-render so the first
    observation shows the shaped layout. Idempotent per function; several tasks/hooks can register."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # offline import
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_cableshape_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            base_reset(self, *a, **k)
            for f in reset._cableshape_hooks:
                f(self)
            for _ in range(renders):
                self._base.sim.render()
            self._t = 0.0
            return self._observation()

        reset._cableshape_hooks = []
        cls.reset = reset
    if fn not in cls.reset._cableshape_hooks:
        cls.reset._cableshape_hooks.append(fn)
