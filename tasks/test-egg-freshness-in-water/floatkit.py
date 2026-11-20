"""floatkit: water forces on small rigid bodies in an open vessel, for a yam_arena task file.

There is no fluid in the simulator. This kit adds the water's forces to chosen rigid bodies before every physics step
(240 Hz), through the same runtime hook as our insertkit (an external force and torque on the body, env 0). PhysX still
does every contact (the pan's floor and wall, the spoon, the plate, the other pieces). The water has no collider, so a
slotted spoon drains for free and a piece lifted in it simply leaves the water.

    import floatkit as fk
    PAN = fk.Vessel("pan", center=(x, y), radius=0.12, floor_z=z0, liquid_m3=2.7e-3)
    fk.setup(PAN, [fk.Body("g1", size=(0.025, 0.020, 0.015), mass=0.0081, cook=fk.Cook(10.0, 950.0)), ...],
             tools=[fk.Tool("spoon", points=bowl_points_in_the_spoon_frame, radius=0.03)],
             drift=fk.Drift(rms=0.010, seed=0))
    fk.state()             # Water: t (s since the trial began), surface (m), cooked {name: bool}, t_cooked {name: s}
    fk.surface_z()         # the water's surface height (m, world)
    fk.submerged(name, pos, quat)   # submerged share of a body's volume (0..1)

The model, per body (a cloud of sample points on a 2.5 mm grid through its shape, 1.25 mm for a pillow, each carrying an
equal share of its volume):
- Surface: a flat plane at floor + (liquid + displaced) / area, so what is put in raises the level (exact for a
  straight-walled pan). Recomputed every physics step from the previous step's displaced volume.
- Buoyancy: rho g times the submerged volume, straight up, at the submerged points' centroid. A point is submerged
  when it is below the surface, blended linearly over one grid step so the force has no steps. The righting moment and
  any capsize come out of where that centroid moves; there is no tuned spring.
- Cooking (optional, per body): a body whose density drops at a set time (a gnocchi that sinks raw and rises when
  cooked through) gets an extra upward force (rho_raw - rho_cooked) V g at its center of mass, ramped in over `ramp` s.
  Its rigid body keeps its raw mass.
- Drag: per submerged point, f = -0.5 rho Cd a |u| u, u the point's velocity relative to the water, a the body's
  mean projected area shared over its points; plus a linear term on the vertical part only, standing in for the energy
  a real floater loses to the waves it makes when it bobs (waves are not otherwise modeled; it settles a bobbing piece
  in under a second, about 0.2 of critical damping). Rotation is damped by the same point forces. The drag is taken
  implicitly over each physics step (implicit_drag), so it can slow a body to the water's speed but never overshoot:
  held explicitly over a 1/240 s step it reversed and multiplied a 4.6 g piece's speed past 16 m/s (b19: a kicked
  piece left the pan at 140-980 m/s); at real speeds the two agree within 1%.
- Water velocity: (a) a simmer drift, a smooth seeded stream-function field inside the pan (no flow through the
  wall), `rms` m/s at the surface falling to 30% at the floor, changing slowly (on each body, taken with its gradient
  at the body's center); (b) each tool point under the surface
  pushes the water ahead of it (V from how the point moved since the previous physics step), u = V (a/r)^3 cos(angle off its direction of motion) beyond r = a (the front half of a
  sphere's potential flow), so a spoon swept fast pushes a floater ahead of it, one moved slowly barely disturbs it,
  and a bowl slid flat under a floater does not drag it along.
Not modeled: waves and slosh (the surface stays flat), added mass, surface tension, heat.

Pure NumPy at import; the simulator hook installs itself only when Isaac Lab is imported.
"""
from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np

RHO_W = 1000.0  # kg/m3, water (simmering water is about 960; the 4% is inside the model's error, said in fidelity)
G = 9.81
GRID = 0.0025  # m: sample spacing through a body


@dataclass(frozen=True)
class Vessel:
    """A round vessel: inside radius at the floor, inside floor height (world z), liquid volume, and how far its
    inside wall flares out per metre of height (`flare`, 0 for a straight-walled pan; a mixing bowl's sloping side)."""

    name: str
    center: tuple
    radius: float
    floor_z: float
    liquid_m3: float
    flare: float = 0.0

    @property
    def area(self) -> float:
        return math.pi * self.radius ** 2

    def radius_at(self, z):
        """The inside radius (m) at height z (world; the floor's radius below the floor)."""
        return self.radius + self.flare * np.clip(np.asarray(z, float) - self.floor_z, 0.0, None)

    def volume_below(self, z: float) -> float:
        """Inside volume (m3) from the floor up to height z (a frustum)."""
        h = max(float(z) - self.floor_z, 0.0)
        r0, r1 = self.radius, self.radius + self.flare * h
        return math.pi * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1)


@dataclass(frozen=True)
class Cook:
    """The body's density drops from its raw value to `rho` (kg/m3) at `at` s, ramped over `ramp` s."""

    at: float
    rho: float
    ramp: float = 2.0


@dataclass(frozen=True)
class Body:
    """A rigid body the water acts on: a box `size` (m, its own frame, centered on its origin), its mass (kg),
    a drag coefficient and an optional cooking schedule."""

    name: str
    size: tuple
    mass: float
    cd: float = 1.0
    cook: Cook | None = None
    shape: str = "box"  # "box", "sphere" (size = diameter x3), "cylinder" (size = (d, d, h), axis z) or "pillow"
    # (size = (length, width, thickness): a flat rectangle rounded all round by half the thickness, as a gnocchi), or
    # "cloud": `cloud` holds the sample points (body frame, each an equal share of the outside volume `vol`); size is
    # then the bounding box (length, breadth, breadth) and the mean projected area is an ellipsoid's
    cloud: tuple = ()
    vol: float = 0.0

    @property
    def volume(self) -> float:
        if self.shape == "box":
            return float(np.prod(self.size))
        if self.shape == "pillow":
            a, b, r = pillow_dims(self.size)
            return float(8 * a * b * r + 2 * math.pi * r * r * (a + b) + 4 / 3 * math.pi * r ** 3)
        if self.shape == "cloud":
            return float(self.vol)
        return float(len(points_for(self)) * GRID ** 3)


@dataclass(frozen=True)
class Tool:
    """A tool whose submerged points push the water: points in its own frame, and the radius of water each drags."""

    name: str
    points: tuple
    radius: float = 0.03
    radii: tuple = ()  # optional per-point radii (a thin handle drags less water than the bowl); else `radius`


@dataclass(frozen=True)
class Drift:
    """The simmer's slow surface drift: rms speed at the surface (m/s), a seed for its pattern, how fast it changes."""

    rms: float = 0.010
    seed: int = 0
    modes: int = 6
    omega: float = 0.08  # rad/s: the pattern turns over in about a minute and a quarter


@dataclass
class Water:
    """The water's record for one trial."""

    t: float = 0.0
    surface: float = 0.0
    displaced: dict = field(default_factory=dict)
    cooked: dict = field(default_factory=dict)
    t_cooked: dict = field(default_factory=dict)
    last_force: dict = field(default_factory=dict)


FLOAT_DEBUG = bool(os.environ.get("FLOAT_DEBUG"))  # telemetry every 0.25 s (debug runs only)
# FLOAT_DEBUG_WINDOW="t0,t1" (s): every physics step in that window, each body's and tool's full state and the water's
# force on it (a contact event between two 0.25 s lines is otherwise invisible); also, at the first step, the PhysX
# attributes as authored on the stage for every body and tool (max depenetration, solver iterations, offsets)
try:
    DEBUG_WINDOW = tuple(float(v) for v in os.environ.get("FLOAT_DEBUG_WINDOW", "").split(",")) or None
except ValueError:
    DEBUG_WINDOW = None
if DEBUG_WINDOW is not None and len(DEBUG_WINDOW) != 2:
    DEBUG_WINDOW = None
_CFG: dict = {}
_W = Water()


# -- geometry ------------------------------------------------------------------------------------------------------


_PTS: dict = {}


def pillow_dims(size) -> tuple:
    """(half-length and half-width of a pillow's flat core, its rounding radius) from its overall size."""
    r = size[2] / 2
    return max(size[0] / 2 - r, 0.0), max(size[1] / 2 - r, 0.0), r


def _pillow_points(size) -> np.ndarray:
    """A pillow's points on a lattice of GRID / 2, symmetric about its center (a small body needs the finer lattice
    for its floating depth)."""
    a, b, r = pillow_dims(size)
    g = GRID / 2
    axes = []
    for h in np.asarray(size, float) / 2:
        n = max(1, int(round(2 * h / g)))
        axes.append((np.arange(n) - (n - 1) / 2) * g)
    P = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    d = np.stack([np.clip(np.abs(P[:, 0]) - a, 0, None), np.clip(np.abs(P[:, 1]) - b, 0, None), P[:, 2]], -1)
    return P[np.linalg.norm(d, axis=1) <= r]


def points_for(body: Body) -> np.ndarray:
    """Sample points (body frame) filling the body's shape, each standing for an equal share of its volume (a GRID
    lattice; a pillow's is finer)."""
    key = (body.shape, tuple(body.size), body.name if body.shape == "cloud" else "")
    if key in _PTS:
        return _PTS[key]
    if body.shape == "cloud":
        _PTS[key] = np.asarray(body.cloud, float)
        return _PTS[key]
    if body.shape == "pillow":
        _PTS[key] = _pillow_points(body.size)
        return _PTS[key]
    h = np.asarray(body.size, float) / 2
    axes = [np.arange(-hk + GRID / 2, hk - 1e-12, GRID) for hk in h]
    axes = [a if len(a) else np.zeros(1) for a in axes]
    P = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    if body.shape == "sphere":
        P = P[np.linalg.norm(P, axis=1) <= h[0]]
    elif body.shape == "cylinder":
        P = P[np.hypot(P[:, 0], P[:, 1]) <= h[0]]
    _PTS[key] = P
    return P


def quat_R(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _wet(z: np.ndarray, surface: float) -> np.ndarray:
    """Submerged share of each point (0..1), blended over one grid step around the surface."""
    return np.clip((surface - z) / GRID + 0.5, 0.0, 1.0)


def _inside(p: np.ndarray) -> np.ndarray:
    """1.0 for world points inside the vessel's wall (and above its floor), else 0.0: there is no water outside the
    pan, so a piece on the plate or the counter below the water's level gets no buoyancy, drift or drag."""
    v = _CFG.get("vessel")
    if v is None:
        return np.ones(len(p))
    r = np.hypot(p[:, 0] - v.center[0], p[:, 1] - v.center[1])
    return ((r < v.radius_at(p[:, 2])) & (p[:, 2] > v.floor_z - GRID)).astype(float)


def mean_area(body: Body) -> float:
    """The body's mean projected area (m2): the average of its three face areas for a box (a fair mean over
    orientations for a squat body), pi r^2 for a sphere."""
    a, b, c = body.size
    if body.shape == "pillow":
        p, q, r = pillow_dims(body.size)
        top = 4 * p * q + 4 * r * (p + q) + math.pi * r * r
        return (top + 4 * p * r + math.pi * r * r + 4 * q * r + math.pi * r * r) / 3
    if body.shape == "sphere":
        return math.pi * a * a / 4
    if body.shape == "cloud":  # an ellipsoid's three principal silhouettes, averaged
        return math.pi / 4 * (a * b + b * c + a * c) / 3
    if body.shape == "cylinder":
        return (math.pi * a * a / 4 + a * c + a * c) / 3
    return (a * b + b * c + a * c) / 3


# -- the water's velocity ------------------------------------------------------------------------------------------


def _modes(d: Drift):
    rng = np.random.default_rng(1000 + int(d.seed))
    k = rng.uniform(8.0, 30.0, d.modes)  # 1/m: patterns 20-80 cm long, a few cells across the pan
    ang = rng.uniform(0, 2 * math.pi, d.modes)
    ph = rng.uniform(0, 2 * math.pi, d.modes)
    om = d.omega * rng.uniform(0.5, 1.5, d.modes) * rng.choice((-1.0, 1.0), d.modes)
    amp = rng.normal(0.0, 1.0, d.modes)
    return k, ang, ph, om, amp


def _psi(xy: np.ndarray, t: float) -> np.ndarray:
    """Stream function (unscaled) at points xy (n, 2) relative to the pan's center, zero with zero slope at the wall."""
    v, d = _CFG["vessel"], _CFG["drift"]
    k, ang, ph, om, amp = _CFG["modes"]
    r2 = (xy ** 2).sum(-1) / v.radius ** 2
    env = np.clip(1.0 - r2, 0.0, None) ** 2
    arg = (xy[:, :1] * (k * np.cos(ang)) + xy[:, 1:2] * (k * np.sin(ang))) + ph + om * t
    return env * (np.sin(arg) * amp).sum(-1)


def drift_velocity(p: np.ndarray, t: float) -> np.ndarray:
    """The simmer drift (m/s, world) at world points p (n, 3): horizontal, along the stream function's contours."""
    if not _CFG or _CFG.get("drift") is None or _CFG["drift"].rms <= 0:
        return np.zeros_like(p)
    v = _CFG["vessel"]
    xy = p[:, :2] - np.asarray(v.center, float)
    h = 1e-3
    dpx = (_psi(xy + [h, 0], t) - _psi(xy - [h, 0], t)) / (2 * h)
    dpy = (_psi(xy + [0, h], t) - _psi(xy - [0, h], t)) / (2 * h)
    depth = max(_W.surface - v.floor_z, 1e-3)
    f = 0.3 + 0.7 * np.clip((p[:, 2] - v.floor_z) / depth, 0.0, 1.0)
    u = np.zeros_like(p)
    u[:, 0], u[:, 1] = dpy, -dpx
    return u * (_CFG["drift_scale"] * f)[:, None]


def _drift_scale(d: Drift, v: Vessel) -> float:
    """Scale so the drift's rms speed over the pan's surface (inside 80% of its radius) is d.rms."""
    rng = np.random.default_rng(7)
    r = v.radius * 0.8 * np.sqrt(rng.uniform(0, 1, 600))
    a = rng.uniform(0, 2 * math.pi, 600)
    xy = np.stack([r * np.cos(a), r * np.sin(a)], -1)
    h = 1e-3
    sp = []
    for t in (0.0, 20.0, 40.0, 60.0):
        dpx = (_psi(xy + [h, 0], t) - _psi(xy - [h, 0], t)) / (2 * h)
        dpy = (_psi(xy + [0, h], t) - _psi(xy - [0, h], t)) / (2 * h)
        sp.append(dpx ** 2 + dpy ** 2)
    rms = math.sqrt(float(np.mean(sp)))
    return d.rms / rms if rms > 0 else 0.0


def drift_velocity_near(p: np.ndarray, center, t: float) -> np.ndarray:
    """The simmer drift at points p (n, 3) close to `center` (one body's points): the horizontal field and its
    gradient taken at the center, the depth factor at each point. The field's patterns are 20-80 cm long, so over a
    piece 2-3 cm across this stays close to drift_velocity (median error 6% of the local speed) at a fraction of the
    cost of evaluating the field at every point."""
    if not _CFG or _CFG.get("drift") is None or _CFG["drift"].rms <= 0:
        return np.zeros_like(p)
    v = _CFG["vessel"]
    c = np.asarray(center, float)
    h = 2e-3
    q = np.array([[c[0], c[1]], [c[0] + h, c[1]], [c[0] - h, c[1]], [c[0], c[1] + h], [c[0], c[1] - h]])
    xy = q - np.asarray(v.center, float)
    e = 1e-3
    dpx = (_psi(xy + [e, 0], t) - _psi(xy - [e, 0], t)) / (2 * e)
    dpy = (_psi(xy + [0, e], t) - _psi(xy - [0, e], t)) / (2 * e)
    uh = np.stack([dpy, -dpx], -1) * _CFG["drift_scale"]  # (5, 2): at the center, +x, -x, +y, -y
    J = np.stack([(uh[1] - uh[2]) / (2 * h), (uh[3] - uh[4]) / (2 * h)], -1)  # d(u)/d(x, y)
    d = np.asarray(p, float)[:, :2] - c[:2]
    depth = max(_W.surface - v.floor_z, 1e-3)
    f = 0.3 + 0.7 * np.clip((p[:, 2] - v.floor_z) / depth, 0.0, 1.0)
    # zero outside the wall, as the stream function's envelope makes it at the wall
    r = np.hypot(p[:, 0] - v.center[0], p[:, 1] - v.center[1])
    u = np.zeros_like(p)
    u[:, :2] = (uh[0][None, :] + d @ J.T) * (f * (r < v.radius))[:, None]
    return u


TOOL_REACH = 6.0  # a tool's push is left out beyond this many push radii (there it is under 0.5% of its speed)


def tool_velocity(p: np.ndarray, tools: dict) -> np.ndarray:
    """Water velocity at world points p dragged by the tools' submerged points: V (a/r)^3 for r > a, V inside a."""
    u = np.zeros_like(p)
    for name, (tp, tv) in (tools or {}).items():
        spec = _CFG["tools"].get(name)
        if spec is None or len(tp) == 0:
            continue
        under = (tp[:, 2] < _W.surface) & (_inside(tp) > 0)
        if not under.any():
            continue
        a_all = np.asarray(spec.radii, float) if len(spec.radii) == len(spec.points) else np.full(len(spec.points), spec.radius)
        tp, tv = tp[under], np.asarray(tv, float)[under]
        # no tool point pushes water faster than MAX_TOOL_SPEED (the runtime already drops faster jumps as resets;
        # this also bounds a caller that passes solver velocities): the push field is bounded by it everywhere
        s = np.linalg.norm(tv, axis=1)
        tv = tv * np.minimum(1.0, MAX_TOOL_SPEED / np.maximum(s, 1e-12))[:, None]
        a = a_all[under] if len(a_all) == len(under) else np.full(len(tp), spec.radius)
        # a body far from every submerged tool point is not pushed (checked on its bounding sphere first)
        mid = p.mean(0)
        reach = float(np.max(np.linalg.norm(p - mid, axis=1))) + TOOL_REACH * float(a.max())
        if float(np.min(np.linalg.norm(tp - mid, axis=1))) > reach or not np.any(tv):
            continue
        dv = p[:, None, :] - tp[None, :, :]
        d = np.linalg.norm(dv, axis=-1)  # (n, m)
        sp = np.linalg.norm(tv, axis=-1)
        ahead = np.clip((dv * tv[None, :, :]).sum(-1) / np.maximum(d * sp[None, :], 1e-9), 0.0, 1.0)
        # each tool point pushes the water ahead of it (the front half of a sphere's potential flow); water beside or
        # behind it is left alone, so a bowl slid flat under a floater does not drag it along
        wgt = np.minimum(1.0, (a[None, :] / np.maximum(d, 1e-6)) ** 3) * ahead
        # the strongest pull over the tool's points (not the sum: a tool is one body, not m separate spheres)
        j = np.argmax(wgt, axis=1)
        u += wgt[np.arange(len(p)), j][:, None] * tv[j]
    return u


# -- forces --------------------------------------------------------------------------------------------------------


def density_force(body: Body, t: float) -> float:
    """The extra upward force (N) of a body whose density has dropped (cooked), ramped in."""
    c = body.cook
    if c is None or t < c.at:
        return 0.0
    rho_raw = body.mass / body.volume
    s = min(1.0, (t - c.at) / max(c.ramp, 1e-6))
    return s * (rho_raw - c.rho) * body.volume * G


def _inertia(body: Body) -> np.ndarray:
    """The body's inertia tensor (body frame, about its center) from its sample points at uniform density."""
    key = ("I", body.shape, tuple(body.size), float(body.mass), body.name if body.shape == "cloud" else "")
    if key not in _PTS:
        P = points_for(body) - points_for(body).mean(0)
        _PTS[key] = body.mass / len(P) * (np.eye(3) * (P ** 2).sum() - P.T @ P)
    return _PTS[key]


def implicit_drag(body: Body, R: np.ndarray, r: np.ndarray, kq: np.ndarray, kz: np.ndarray, F: np.ndarray,
                  T: np.ndarray, dt: float, rest=None) -> tuple:
    """The drag wrench (F, T) to apply over a physics step of dt so that the drag can never overshoot.

    Each point's drag is f_i = -kq_i u_i - kz_i u_iz e_z (kq_i = 0.5 rho Cd a |u_i|, the quadratic drag's secant
    coefficient, kz_i the heave damping), u_i = v + w x r_i - water velocity. Applied explicitly (the force from the
    velocity at the start of the step, held over the step), a light piece overshoots once k |u| dt > 1 and runs away
    past 2 (a 4.6 g gnocchi at 240 Hz: 8 and 16 m/s): every step reverses its velocity and multiplies it, and it leaves
    the pan at hundreds of m/s. Here the body's velocity change over the step is solved implicitly,
    (M + dt D) dnu = dt (W + rest) with D = sum J_i^T K_i J_i (6x6, positive semi-definite, from the same coefficients),
    M its mass and inertia and `rest` the step's other known wrench (weight, buoyancy, the cooking lift), and the drag
    applied is W - D dnu. For a body coasting straight that is the exact solution of quadratic drag over the step,
    v / (1 + k |v| dt); in every case the drag takes the velocity relative to the water toward zero without crossing
    it, translation and rotation together; and a body at its terminal speed (drag + rest = 0) gets exactly the
    explicit drag, so terminal speeds and floating equilibria are unchanged. Contacts are not in the solve."""
    # u_i = J_i (v, w) = v - [r_i]x w, so sum kq_i J_i^T J_i = [[K I, -[s]x], [[s]x, sum kq_i (|r_i|^2 I - r_i r_i^T)]]
    # with K = sum kq_i, s = sum kq_i r_i; the heave term adds sum kz_i jz_i jz_i^T, jz_i = (0, 0, 1, r_iy, -r_ix, 0)
    s = kq @ r
    S = np.array([[0.0, -s[2], s[1]], [s[2], 0.0, -s[0]], [-s[1], s[0], 0.0]])
    D = np.zeros((6, 6))
    D[:3, :3] = np.eye(3) * kq.sum()
    D[:3, 3:], D[3:, :3] = -S, S
    D[3:, 3:] = np.eye(3) * (kq @ (r * r).sum(1)) - (r * kq[:, None]).T @ r
    jz = np.zeros((len(r), 6))
    jz[:, 2], jz[:, 3], jz[:, 4] = 1.0, r[:, 1], -r[:, 0]
    D += (jz * kz[:, None]).T @ jz
    M = np.zeros((6, 6))
    M[:3, :3] = np.eye(3) * body.mass
    M[3:, 3:] = R @ _inertia(body) @ R.T
    W = np.r_[F, T]
    W = W - D @ np.linalg.solve(M + dt * D, dt * (W if rest is None else W + rest))
    return W[:3], W[3:]


def wrench(body: Body, pos, quat, lin_vel, ang_vel, t: float, surface: float, tools: dict | None = None,
           com=None, dt: float = 1 / 240) -> tuple:
    """(force, torque about the center of mass, submerged volume) of the water on one body, world frame, to apply
    over the next physics step of `dt` s (the drag is taken implicitly over the step; dt=0: the plain explicit drag)."""
    P = points_for(body)
    R = quat_R(quat)
    pos = np.asarray(pos, float)
    com = pos if com is None else np.asarray(com, float)
    pw = pos + P @ R.T
    wet = _wet(pw[:, 2], surface) * _inside(pw)
    dv = body.volume / len(P)  # each point's share (GRID^3 on a GRID lattice)
    vol = float(wet.sum() * dv)
    F = np.zeros(3)
    T = np.zeros(3)
    if vol <= 0.0:
        return F, T, 0.0
    # buoyancy at the submerged centroid
    c = (pw * wet[:, None]).sum(0) / wet.sum()
    fb = np.array([0.0, 0.0, RHO_W * G * vol])
    F += fb
    T += np.cross(c - com, fb)
    # cooking: an extra lift at the center of mass, scaled by how much of the body is under (a floating piece's
    # lift is limited by what is submerged, as real buoyancy is)
    fc = density_force(body, t) * min(1.0, vol / body.volume)
    F[2] += fc
    # drag
    vpt = np.asarray(lin_vel, float) + np.cross(np.asarray(ang_vel, float), pw - com)
    uw = drift_velocity_near(pw, pos, t) + tool_velocity(pw, tools or {})
    rel = vpt - uw
    a_i = mean_area(body) / len(P)
    c_lin = _CFG.get("c_lin", 10.0) * body.mass / len(P)
    speed = np.linalg.norm(rel, axis=1)
    kq = 0.5 * RHO_W * body.cd * a_i * speed * wet
    fd = -kq[:, None] * rel
    share = vol / body.volume
    at_surface = min(1.0, min(share, 1.0 - share) / 0.05) if 0.0 < share < 1.0 else 0.0
    kz = at_surface * c_lin * wet
    fd[:, 2] -= kz * rel[:, 2]  # waves damp a floater's bobbing; nothing under water
    Fd, Td = fd.sum(0), np.cross(pw - com, fd).sum(0)
    if dt > 0:
        m = wet > 0
        rest = np.r_[F - [0.0, 0.0, body.mass * G], T]  # what the water lifts, and the body's weight (PhysX's)
        Fd, Td = implicit_drag(body, R, (pw - com)[m], kq[m], kz[m], Fd, Td, dt, rest)
    F += Fd
    T += Td
    return F, T, vol


def submerged(name: str, pos, quat) -> float:
    """Share of a body's volume below the surface now (0..1)."""
    b = _CFG["bodies"][name]
    pw = np.asarray(pos, float) + points_for(b) @ quat_R(quat).T
    return float((_wet(pw[:, 2], _W.surface) * _inside(pw)).mean())


def surface_for(displaced_m3: float) -> float:
    """The surface height (world z) with the liquid and what is displaced in it (bisection on the vessel's volume)."""
    v = _CFG["vessel"]
    vol = v.liquid_m3 + displaced_m3
    if v.flare == 0.0:
        return v.floor_z + vol / v.area
    lo, hi = v.floor_z, v.floor_z + vol / v.area
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if v.volume_below(mid) < vol:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# -- the trial -----------------------------------------------------------------------------------------------------


def setup(vessel: Vessel, bodies, tools=(), drift: Drift | None = None, c_lin: float = 10.0, monitors=()) -> None:
    """Declare the pan, the bodies the water acts on, the tools that stir it and the simmer drift.
    c_lin: the heave damping per unit mass (1/s) standing in for wave-making (about 0.2 of critical damping in heave
    for an 8 g piece)."""
    _CFG.clear()
    _CFG.update(vessel=vessel, bodies={b.name: b for b in bodies}, tools={t.name: t for t in tools}, drift=drift,
                c_lin=float(c_lin), monitors=tuple(monitors))  # monitors: fn(base, origin, dt) after each step's wrench
    if drift is not None:
        _CFG["modes"] = _modes(drift)
        _CFG["drift_scale"] = _drift_scale(drift, vessel)
    reset()


def set_drift(drift: Drift | None) -> None:
    """Change the simmer drift (e.g. per trial seed)."""
    _CFG["drift"] = drift
    if drift is not None:
        _CFG["modes"] = _modes(drift)
        _CFG["drift_scale"] = _drift_scale(drift, _CFG["vessel"])


def set_cook(name: str, cook: Cook | None) -> None:
    """Change one body's cooking time (e.g. per trial seed)."""
    b = _CFG["bodies"][name]
    _CFG["bodies"][name] = Body(b.name, b.size, b.mass, b.cd, cook, b.shape)


def reset() -> None:
    global _W
    _W = Water()
    if _CFG:
        _W.surface = surface_for(0.0)
        _W.cooked = {n: False for n in _CFG["bodies"]}


def state() -> Water:
    return _W


def surface_z() -> float:
    return _W.surface


def step(states: dict, dt: float, tools: dict | None = None) -> dict:
    """Advance the water by one physics step: `states` {body name: (pos, quat, lin_vel, ang_vel[, com])}; `tools`
    {tool name: (world points (m, 3), their velocities (m, 3))}. Returns {name: (force, torque)} to apply."""
    out = {}
    disp = 0.0
    tools = {n: tv for n, tv in (tools or {}).items()
             if np.all(np.isfinite(tv[0])) and np.all(np.isfinite(tv[1]))}  # a tool read as NaN pushes nothing
    for name, s in states.items():
        b = _CFG["bodies"][name]
        if all(np.all(np.isfinite(np.asarray(x, float))) for x in s):
            F, T, vol = wrench(b, s[0], s[1], s[2], s[3], _W.t, _W.surface, tools, s[4] if len(s) > 4 else None,
                               dt=dt)
        else:
            F, T, vol = np.zeros(3), np.zeros(3), 0.0
        if not (np.all(np.isfinite(F)) and np.all(np.isfinite(T))):  # never hand PhysX a NaN: it poisons the scene
            F, T, vol = np.zeros(3), np.zeros(3), 0.0
        out[name] = (F, T)
        disp += vol
        _W.displaced[name] = vol
        if b.cook is not None and _W.t >= b.cook.at and not _W.cooked.get(name):
            _W.cooked[name] = True
            _W.t_cooked[name] = _W.t
        _W.last_force[name] = F
    _W.surface = surface_for(disp)
    _W.t += dt
    return out


def tool_points_world(name: str, pos, quat, lin_vel, ang_vel) -> tuple:
    """A tool's sample points and their velocities in the world."""
    spec = _CFG["tools"][name]
    P = np.asarray(spec.points, float)
    R = quat_R(quat)
    pw = np.asarray(pos, float) + P @ R.T
    v = np.asarray(lin_vel, float) + np.cross(np.asarray(ang_vel, float), pw - np.asarray(pos, float))
    return pw, v


MAX_TOOL_SPEED = 2.0  # m/s: a tool point that moves faster than this in one physics step was moved by a reset
SPIKE_V, SPIKE_W = 1.0, 100.0  # m/s, rad/s: FLOAT_DEBUG reports a body faster than this (nothing real in a pan is)


def tool_points_moved(name: str, pos, quat, prev, dt: float) -> tuple:
    """A tool's sample points in the world and their velocities from how far each point moved since the previous
    physics step (`prev`: the points then, or None at the start). A tool held in a grip reports solver velocities that
    are not its motion (a spoon steady in the hand reported 200-400 deg/s of spin and several cm/s of slide), so the
    water is pushed by how the tool actually moves. A jump faster than MAX_TOOL_SPEED (a reset) counts as no motion."""
    spec = _CFG["tools"][name]
    P = np.asarray(spec.points, float)
    pw = np.asarray(pos, float) + P @ quat_R(quat).T
    if prev is None or np.shape(prev) != pw.shape:
        return pw, np.zeros_like(pw)
    v = (pw - np.asarray(prev, float)) / dt
    if float(np.max(np.linalg.norm(v, axis=1))) > MAX_TOOL_SPEED:
        v = np.zeros_like(pw)
    return pw, v


# -- simulator hook (Isaac Lab; only when it is imported) ----------------------------------------------------------


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


class _Runtime:
    """Reads the bodies and tools (env 0) and applies the water's wrench on each body before each physics step."""

    def __init__(self, base):
        self.base = base
        self.origin = _arr(base.scene.env_origins)[0]
        self.dt = float(getattr(base, "physics_dt", 1 / 240))
        self.assets = {n: base.scene[n] for n in _CFG["bodies"]}
        self.tools = {}
        for n in _CFG["tools"]:
            try:
                self.tools[n] = base.scene[n]
            except KeyError:
                print(f"[floatkit] WARNING: no tool {n} in the scene", flush=True)
        self.mode = None
        self.prev = {}  # each tool's points at the previous physics step
        self.n = 0
        self.spiked = {}  # body name -> the trial time its last spike was reported

    def read(self, asset):
        d = asset.data
        pos = _arr(d.root_pos_w)[0] - self.origin
        quat = _arr(d.root_quat_w)[0]  # (x, y, z, w), as yamkit and insertkit read it on this Isaac Lab
        return pos, quat, _arr(d.root_lin_vel_w)[0], _arr(d.root_ang_vel_w)[0]

    def apply(self, asset, force, torque, quat) -> None:
        import torch

        f = torch.tensor(np.asarray(force, np.float32).reshape(1, 1, 3), device=asset.device)
        tq = torch.tensor(np.asarray(torque, np.float32).reshape(1, 1, 3), device=asset.device)
        tries = []
        comp = getattr(asset, "permanent_wrench_composer", None)
        if comp is not None:
            tries.append(("wrench_composer", lambda: comp.set_forces_and_torques(
                forces=f, torques=tq, body_ids=[0], env_ids=[0], is_global=True)))
        if hasattr(asset, "set_external_force_and_torque"):
            tries.append(("external_global", lambda: asset.set_external_force_and_torque(
                f, tq, body_ids=[0], env_ids=[0], is_global=True)))
            R = quat_R(quat)
            fl = torch.tensor((R.T @ np.asarray(force)).astype(np.float32).reshape(1, 1, 3), device=asset.device)
            tl = torch.tensor((R.T @ np.asarray(torque)).astype(np.float32).reshape(1, 1, 3), device=asset.device)
            tries.append(("external", lambda: asset.set_external_force_and_torque(fl, tl, body_ids=[0],
                                                                                env_ids=[0])))
        for name, fn in tries:
            if self.mode not in (None, name):
                continue
            try:
                fn()
                if self.mode is None:
                    self.mode = name
                    if FLOAT_DEBUG:
                        print(f"[floatkit] applying the water's wrench through {name}", flush=True)
                return
            except Exception as error:  # try the next API
                if self.mode == name:
                    raise
                print(f"[floatkit] {name} failed: {type(error).__name__}: {error}", flush=True)
        raise RuntimeError("floatkit: no way to apply an external wrench on this Isaac Lab")

    def dump_physx(self) -> None:
        """Print the PhysX attributes authored on the stage for each body and tool (env 0) and their colliders."""
        try:
            import isaaclab.sim as sim_utils
            from pxr import Usd

            names = ("physxRigidBody:maxDepenetrationVelocity", "physxRigidBody:maxAngularVelocity",
                     "physxRigidBody:maxLinearVelocity", "physxRigidBody:solverPositionIterationCount",
                     "physxRigidBody:solverVelocityIterationCount", "physxRigidBody:enableCCD",
                     "physxRigidBody:disableGravity", "physics:mass", "physxCollision:contactOffset",
                     "physxCollision:restOffset")
            for n, a in {**self.assets, **self.tools}.items():
                root = sim_utils.find_matching_prims(a.cfg.prim_path)[0]
                for prim in Usd.PrimRange(root):
                    got = [f"{k.split(':')[1]}={prim.GetAttribute(k).Get()}" for k in names
                           if prim.GetAttribute(k) and prim.GetAttribute(k).HasAuthoredValue()]
                    if got:
                        print(f"[floatkit-physx] {prim.GetPath()} " + " ".join(got), flush=True)
        except Exception as error:  # debug only
            print(f"[floatkit-physx] could not read the stage: {type(error).__name__}: {error}", flush=True)

    def physics_step(self) -> None:
        if DEBUG_WINDOW is not None and self.n == 0:
            self.dump_physx()
        states = {n: self.read(a) for n, a in self.assets.items()}
        tools = {}
        for n, a in self.tools.items():
            pos, quat, _, _ = self.read(a)
            tools[n] = tool_points_moved(n, pos, quat, self.prev.get(n), self.dt)
            self.prev[n] = tools[n][0]
        out = step(states, self.dt, tools)
        for n, (F, T) in out.items():
            self.apply(self.assets[n], F, T, states[n][1])
        for fn in _CFG.get("monitors", ()):
            fn(self.base, self.origin, self.dt)
        self.n += 1
        if FLOAT_DEBUG and self.n % 60 == 0:  # every 0.25 s: each body's position, and each tool's first point
            msg = " ".join(f"{n}:{p[0][0]:.4f},{p[0][1]:.4f},{p[0][2]:.4f}" for n, p in states.items())
            msg += "".join(f" {n}:{v[0][0][0]:.4f},{v[0][0][1]:.4f},{v[0][0][2]:.4f}" for n, v in tools.items())
            print(f"[floatkit] t={_W.t:.2f} surface={_W.surface:.4f} {msg}", flush=True)
        if FLOAT_DEBUG:  # a body suddenly fast or spinning (a solver kick): its state, the water's force, the tools
            for n, s_ in states.items():
                v, w = float(np.linalg.norm(s_[2])), float(np.linalg.norm(s_[3]))
                if (v > SPIKE_V or w > SPIKE_W) and _W.t - self.spiked.get(n, -9.0) > 0.5:
                    self.spiked[n] = _W.t
                    tl = " ".join(f"{k}:p={','.join(f'{x:.4f}' for x in self.read(a)[0])}"
                                  f",|v|={np.linalg.norm(self.read(a)[2]):.3f},|w|={np.linalg.norm(self.read(a)[3]):.1f}"
                                  for k, a in self.tools.items())
                    print(f"[floatkit-spike] t={_W.t:.5f} {n} p={','.join(f'{x:.4f}' for x in s_[0])} |v|={v:.3f} "
                          f"|w|={w:.1f} |F|={np.linalg.norm(out[n][0]):.4f} N {tl}", flush=True)
        if DEBUG_WINDOW is not None and DEBUG_WINDOW[0] <= _W.t <= DEBUG_WINDOW[1]:
            f3 = lambda v: ",".join(f"{x:.4f}" for x in v)  # noqa: E731
            for n, s_ in states.items():
                F, T = out[n]
                print(f"[floatkit-step] t={_W.t:.5f} {n} p={f3(s_[0])} q={f3(s_[1])} v={f3(s_[2])} w={f3(s_[3])} "
                      f"F={','.join(f'{x:.5f}' for x in F)} vol={_W.displaced.get(n, 0.0) * 1e6:.3f}", flush=True)
            for n, a in self.tools.items():
                p_, q_, v_, w_ = self.read(a)
                print(f"[floatkit-step] t={_W.t:.5f} tool {n} p={f3(p_)} q={f3(q_)} v={f3(v_)} w={f3(w_)}", flush=True)


_RT: dict = {}


def _patch_scene_cls(module) -> None:
    cls = getattr(module, "InteractiveScene", None)
    if cls is None or getattr(cls, "_floatkit", False):
        return
    init = cls.__init__

    def __init__(self, *a, **k):
        init(self, *a, **k)
        write = self.write_data_to_sim

        def write_data_to_sim(*aa, **kk):
            base = _RT.get("base")
            if base is not None and _CFG:
                if "rt" not in _RT:
                    _RT["rt"] = _Runtime(base)
                _RT["rt"].physics_step()
            return write(*aa, **kk)

        self.write_data_to_sim = write_data_to_sim

    cls.__init__, cls._floatkit = __init__, True


def _patch_embodiment(module) -> None:
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_floatkit", False):
        return
    ensure_sim, reset_ = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None and "base" not in _RT:
            _RT["base"] = self._env.unwrapped
        return out

    def _reset(self, *a, **k):
        reset()
        obs = reset_(self, *a, **k)
        reset()  # the settle steps are not part of the trial: the clock starts at the first control step
        return obs

    cls._ensure_sim, cls.reset, cls._floatkit = _ensure_sim, _reset, True


_TARGETS = {"isaaclab.scene.interactive_scene": _patch_scene_cls,
            "yam_arena.inspect_embodiment": _patch_embodiment}


class _Finder(importlib.abc.MetaPathFinder):
    """Patches each target module right after it is first imported."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _TARGETS:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module = spec.loader.exec_module

        def _exec(module):
            exec_module(module)
            _TARGETS[fullname](module)

        spec.loader.exec_module = _exec
        return spec


def install() -> None:
    for name, patch in _TARGETS.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
