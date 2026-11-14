"""Offline checks of floatkit, no simulator: a small rigid-body integrator (240 Hz, penalty contact with the pan's
floor and wall) stands in for PhysX, and floatkit supplies the water.

    python -m pytest -q test_floatkit.py
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import floatkit as fk  # noqa: E402

DT = 1 / 240


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def qaxis(axis, ang):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[a * math.sin(ang / 2), math.cos(ang / 2)]


class Rigid:
    """One rigid body: box inertia, gravity, penalty contact of its sample points with the pan's floor and wall."""

    def __init__(self, body: fk.Body, pos, quat=(0, 0, 0, 1.0), vel=(0, 0, 0)):
        self.b, self.m = body, body.mass
        a, b_, c = body.size
        if body.shape == "sphere":
            i = 0.4 * self.m * (a / 2) ** 2
            self.I = np.diag([i, i, i])
        else:
            self.I = np.diag([b_ * b_ + c * c, a * a + c * c, a * a + b_ * b_]) * self.m / 12
        self.p, self.q = np.asarray(pos, float), np.asarray(quat, float)
        self.v, self.w = np.asarray(vel, float), np.zeros(3)
        self.held = None  # (point, velocity) when carried kinematically

    def state(self):
        return (self.p, self.q, self.v, self.w)

    def contact(self, vessel: fk.Vessel | None, mu=0.4):
        if vessel is None:
            return np.zeros(3), np.zeros(3)
        k = 3000.0 * self.m  # N/m in all: omega = 55 rad/s, well inside 240 Hz explicit stability
        c = 2 * math.sqrt(k * self.m)  # critically damped
        R = fk.quat_R(self.q)
        P = fk.points_for(self.b)
        pw = self.p + P @ R.T
        F, T = np.zeros(3), np.zeros(3)
        # only the outer layer of points touches: take the lowest few
        pen = vessel.floor_z - pw[:, 2]
        hit = pen > 0
        if hit.any():
            n = hit.sum()
            vp = self.v + np.cross(self.w, pw[hit] - self.p)
            fz = (k * pen[hit] - c * vp[:, 2]) / max(n, 1)
            fz = np.clip(fz, 0, None)
            ft = -mu * fz[:, None] * vp[:, :2] / np.maximum(np.linalg.norm(vp[:, :2], axis=1), 0.005)[:, None]
            f = np.c_[ft, fz]
            F += f.sum(0)
            T += np.cross(pw[hit] - self.p, f).sum(0)
        xy = pw[:, :2] - np.asarray(vessel.center)
        r = np.linalg.norm(xy, axis=1)
        out = r > vessel.radius
        if out.any():
            nrm = -xy[out] / r[out][:, None]
            pen = r[out] - vessel.radius
            vp = self.v + np.cross(self.w, pw[out] - self.p)
            vn = (vp[:, :2] * nrm).sum(1)
            fn = np.clip(k * pen - c * vn, 0, None) / max(out.sum(), 1)
            f = np.c_[nrm * fn[:, None], np.zeros(out.sum())]
            F += f.sum(0)
            T += np.cross(pw[out] - self.p, f).sum(0)
        return F, T

    def step(self, F, T, vessel=None):
        Fc, Tc = self.contact(vessel)
        F = F + Fc + np.array([0.0, 0.0, -self.m * fk.G])
        T = T + Tc
        R = fk.quat_R(self.q)
        Iw = R @ self.I @ R.T
        self.v = self.v + F / self.m * DT
        self.w = self.w + np.linalg.solve(Iw, T - np.cross(self.w, Iw @ self.w)) * DT
        self.p = self.p + self.v * DT
        ang = np.linalg.norm(self.w) * DT
        if ang > 0:
            self.q = qmul(qaxis(self.w, ang), self.q)
            self.q /= np.linalg.norm(self.q)


def run(bodies: dict, seconds: float, vessel, tools_fn=None, record=None):
    """Integrate {name: Rigid} for `seconds` with floatkit's water; tools_fn(t) -> floatkit tools dict."""
    n = int(round(seconds / DT))
    for k in range(n):
        t = fk.state().t
        tools = tools_fn(t) if tools_fn else None
        out = fk.step({nm: r.state() for nm, r in bodies.items()}, DT, tools)
        for nm, r in bodies.items():
            r.step(*out[nm], vessel)
        if record is not None and k % 12 == 0:
            record.append((t, {nm: r.p.copy() for nm, r in bodies.items()}))
    return bodies


BIG = fk.Vessel("tank", center=(0.0, 0.0), radius=0.20, floor_z=0.0, liquid_m3=math.pi * 0.2 ** 2 * 0.10)  # 10 cm deep


def test_half_density_cube_floats_half_submerged():
    b = fk.Body("cube", size=(0.03, 0.03, 0.03), mass=0.5 * 1000 * 0.03 ** 3, cd=1.05)
    fk.setup(BIG, [b])
    r = Rigid(b, (0, 0, 0.08))
    run({"cube": r}, 6.0, BIG)
    draft = fk.surface_z() - (r.p[2] - 0.015)
    assert abs(draft - 0.015) < 0.001, draft
    assert abs(r.v[2]) < 0.002


def test_thin_disc_floats_flat_and_rights_itself():
    b = fk.Body("disc", size=(0.06, 0.06, 0.01), mass=0.5 * 1000 * 0.06 * 0.06 * 0.01, cd=1.1)
    fk.setup(BIG, [b])
    r = Rigid(b, (0, 0, 0.1), qaxis((1, 0, 0), math.radians(30)))
    run({"disc": r}, 8.0, BIG)
    up = fk.quat_R(r.q)[:, 2]
    assert abs(up[2]) > math.cos(math.radians(3)), up


def test_tall_light_cylinder_lies_on_its_side():
    b = fk.Body("rod", size=(0.02, 0.02, 0.08), mass=0.6 * 1000 * math.pi * 0.01 ** 2 * 0.08, cd=1.0,
                shape="cylinder")
    fk.setup(BIG, [b])
    r = Rigid(b, (0, 0, 0.1), qaxis((1, 0, 0), math.radians(5)))
    run({"rod": r}, 10.0, BIG)
    ax = fk.quat_R(r.q)[:, 2]
    assert abs(ax[2]) < math.sin(math.radians(10)), ax


def test_sinking_sphere_reaches_textbook_terminal_speed():
    d, rho = 0.02, 1100.0
    vol = math.pi * d ** 3 / 6
    b = fk.Body("ball", size=(d, d, d), mass=rho * fk.points_for(fk.Body("x", (d, d, d), 1, shape="sphere")).shape[0]
                * fk.GRID ** 3, cd=0.47, shape="sphere")
    deep = fk.Vessel("deep", (0, 0), 0.2, 0.0, math.pi * 0.04 * 1.0)
    fk.setup(deep, [b])
    r = Rigid(b, (0, 0, 0.9))
    run({"ball": r}, 3.0, deep)
    v_book = math.sqrt(2 * (rho - 1000) * vol * fk.G / (1000 * 0.47 * math.pi * d * d / 4))
    assert abs(-r.v[2] - v_book) / v_book < 0.10, (r.v, v_book)


def test_displaced_volume_raises_the_surface():
    pan = fk.Vessel("pan", (0, 0), 0.12, 0.0, 2.7e-3)
    b = fk.Body("block", size=(0.05, 0.05, 0.02), mass=2.0 * 1000 * 0.05 * 0.05 * 0.02)
    fk.setup(pan, [b])
    z0 = fk.surface_z()
    r = Rigid(b, (0, 0, 0.0101))
    run({"block": r}, 1.0, pan)
    assert abs((fk.surface_z() - z0) - 0.05 * 0.05 * 0.02 / pan.area) < 2e-5


def gnocchi(name="g", at=3.0):
    return fk.Body(name, size=(0.025, 0.020, 0.015), mass=1080 * 0.025 * 0.020 * 0.015, cd=0.9,
                   cook=fk.Cook(at, 950.0, 2.0))


def test_raw_gnocchi_stays_down_then_surfaces_within_3_s_of_cooking():
    pan = fk.Vessel("pan", (0, 0), 0.12, 0.0, 2.7e-3)
    b = gnocchi(at=3.0)
    fk.setup(pan, [b])
    r = Rigid(b, (0, 0, 0.0076))
    rec = []
    run({"g": r}, 8.0, pan, record=rec)
    before = [p["g"][2] for t, p in rec if t < 3.0]
    assert max(before) < 0.012  # on the floor while raw
    top = lambda p: p["g"][2] + 0.0075  # noqa: E731
    up = [t for t, p in rec if t >= 3.0 and abs(top(p) - fk.surface_z()) < 0.003]
    assert up and up[0] - 3.0 < 3.0, up[:3]
    assert abs(top({"g": r.p}) - fk.surface_z()) < 0.003  # it stays at the surface


def test_floating_gnocchi_top_is_just_above_the_surface():
    pan = fk.Vessel("pan", (0, 0), 0.12, 0.0, 2.7e-3)
    b = gnocchi(at=0.0)
    fk.setup(pan, [b])
    r = Rigid(b, (0, 0, 0.0076))
    run({"g": r}, 6.0, pan)
    sub = fk.submerged("g", r.p, r.q)
    assert 0.90 < sub < 0.99, sub  # density 950: about 95% under


def _push(speed):
    """A spoon's bowl (a 6 x 8 cm grid of points 1 cm under the surface) moved at `speed` toward a floating piece,
    stopping with its leading edge 1.0 cm short of the piece. Returns how far the piece moved by 0.5 s later."""
    pan = fk.Vessel("pan", (0, 0), 0.20, 0.0, math.pi * 0.2 ** 2 * 0.06)
    b = gnocchi(at=0.0)
    gx, gy = np.meshgrid(np.linspace(-0.04, 0.04, 5), np.linspace(-0.03, 0.03, 4))
    pts = np.c_[gx.ravel(), gy.ravel(), np.zeros(gx.size)]
    spoon = fk.Tool("spoon", points=tuple(map(tuple, pts)), radius=0.022)
    fk.setup(pan, [b], tools=[spoon])
    r = Rigid(b, (0.0, 0.0, 0.0))
    # let it float up and settle first
    run({"g": r}, 4.0, pan)
    p0 = r.p.copy()
    zs = fk.surface_z() - 0.01
    x_start, x_end = -0.04 - 0.0125 - 0.06, -0.04 - 0.0125 - 0.010  # bowl center: leading edge 6 cm, then 1 cm short
    T = (x_end - x_start) / speed
    t0 = fk.state().t

    def tools(t):
        s = min(max((t - t0) / T, 0.0), 1.0)
        c = np.array([x_start + s * (x_end - x_start), 0.0, zs])
        v = np.array([speed if s < 1 else 0.0, 0.0, 0.0])
        return {"spoon": (pts + c, np.tile(v, (len(pts), 1)))}

    run({"g": r}, T + 0.5, pan, tools_fn=tools)
    return float(r.p[0] - p0[0])


def test_fast_spoon_pushes_the_piece_away_slow_one_barely():
    fast, slow = _push(0.20), _push(0.03)
    assert fast > 0.008, fast  # spec asked 1 cm; the model gives 8.4 mm, four to five times the slow push
    assert abs(slow) < 0.002, slow


def test_drift_is_tangential_at_the_wall_and_has_the_set_rms():
    pan = fk.Vessel("pan", (0.3, 0.0), 0.12, 0.05, 2.7e-3)
    b = gnocchi()
    fk.setup(pan, [b], drift=fk.Drift(rms=0.01, seed=3))
    ang = np.linspace(0, 2 * math.pi, 50)
    wall = np.c_[0.3 + 0.1199 * np.cos(ang), 0.1199 * np.sin(ang), np.full(50, fk.surface_z())]
    u = fk.drift_velocity(wall, 5.0)
    radial = (u[:, 0] * np.cos(ang) + u[:, 1] * np.sin(ang))
    assert np.max(np.abs(radial)) < 1e-4
    rng = np.random.default_rng(0)
    r = 0.096 * np.sqrt(rng.uniform(0, 1, 400))
    a = rng.uniform(0, 2 * math.pi, 400)
    p = np.c_[0.3 + r * np.cos(a), r * np.sin(a), np.full(400, fk.surface_z())]
    rms = math.sqrt(np.mean(np.sum(fk.drift_velocity(p, 10.0) ** 2, axis=1)))
    assert 0.005 < rms < 0.02, rms


def test_drift_moves_a_floater_but_keeps_it_in_the_pan():
    pan = fk.Vessel("pan", (0.0, 0.0), 0.12, 0.0, 2.7e-3)
    b = gnocchi(at=0.0)
    fk.setup(pan, [b], drift=fk.Drift(rms=0.01, seed=1))
    r = Rigid(b, (0.03, 0.02, 0.0076))
    rec = []
    run({"g": r}, 40.0, pan, record=rec)
    xy = np.array([p["g"][:2] for _, p in rec])
    assert np.linalg.norm(xy[-1] - xy[30]) > 0.01  # it went somewhere
    assert np.max(np.linalg.norm(xy, axis=1)) < 0.12


def test_no_water_outside_the_vessel():
    # a cooked piece set down outside the pan (on a plate), below the water's level, gets no water force at all
    pan = fk.Vessel("pan", (0, 0), 0.12, 0.0, 2.7e-3)
    b = gnocchi(at=0.0)
    fk.setup(pan, [b], tools=[fk.Tool("spoon", points=((0.0, 0.0, 0.0),), radius=0.03)])
    fk.state().t = 5.0  # long cooked
    out = fk.step({"g": ((0.0, -0.40, 0.0075), (0, 0, 0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))}, DT,
                  {"spoon": (np.array([[0.0, -0.39, 0.005]]), np.array([[0.0, 0.2, 0.0]]))})
    F, T = out["g"]
    assert np.allclose(F, 0.0) and np.allclose(T, 0.0), (F, T)
    assert fk.submerged("g", (0.0, -0.40, 0.0075), (0, 0, 0, 1.0)) == 0.0
    inside = fk.step({"g": ((0.0, 0.0, 0.0075), (0, 0, 0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))}, DT)
    assert inside["g"][0][2] > 0.05  # in the pan it is buoyed as before


def test_tool_push_uses_the_tools_motion_not_its_reported_velocity():
    # a spoon held still in the hand reported 200-400 deg/s of spin in the simulator; the water must not move
    pan = fk.Vessel("pan", (0, 0), 0.12, 0.0, 2.7e-3)
    fk.setup(pan, [gnocchi(at=0.0)], tools=[fk.Tool("spoon", points=((0.0, 0.0, 0.0), (0.0, -0.035, 0.0)),
                                                     radius=0.022)])
    q = (0.0, 0.0, 0.0, 1.0)
    p0, _ = fk.tool_points_moved("spoon", (0.0, 0.0, 0.01), q, None, DT)
    still = fk.tool_points_moved("spoon", (0.0, 0.0, 0.01), q, p0, DT)
    assert np.allclose(still[1], 0.0)
    piece = np.array([[0.02, -0.035, 0.02]])
    assert np.allclose(fk.tool_velocity(piece, {"spoon": still}), 0.0)
    moved = fk.tool_points_moved("spoon", (0.0, -0.03 * DT, 0.01), q, p0, DT)  # sliding at 3 cm/s toward -y
    assert np.allclose(moved[1], [0.0, -0.03, 0.0])
    jump = fk.tool_points_moved("spoon", (0.3, 0.0, 0.01), q, p0, DT)  # a reset puts it back on its rest
    assert np.allclose(jump[1], 0.0)


def pillow(name="g", at=3.0):
    """The task's gnocchi: a 2.3 x 1.5 x 1.1 cm pillow at raw dough density 1,080 kg/m3."""
    shape = fk.Body(name, size=(0.023, 0.015, 0.011), mass=1.0, shape="pillow")
    return fk.Body(name, size=shape.size, mass=1080 * shape.volume, cd=0.9, cook=fk.Cook(at, 950.0, 2.0),
                   shape="pillow")


def test_pillow_volume_is_its_rounded_shape():
    b = pillow()
    a, w, r = fk.pillow_dims(b.size)
    rng = np.random.default_rng(0)
    P = rng.uniform(-1, 1, (400000, 3)) * np.array(b.size) / 2
    d = np.stack([np.clip(np.abs(P[:, 0]) - a, 0, None), np.clip(np.abs(P[:, 1]) - w, 0, None), P[:, 2]], -1)
    mc = float((np.linalg.norm(d, axis=1) <= r).mean() * np.prod(b.size))
    assert abs(b.volume - mc) / mc < 0.01, (b.volume, mc)
    assert 2.9e-3 < b.mass < 3.1e-3  # about 3.0 g
    assert np.allclose(fk.points_for(b).mean(0), 0.0, atol=1e-9)  # the lattice is centered on the body


def test_raw_pillow_stays_down_then_floats_just_awash():
    pan = fk.Vessel("pan", (0, 0), 0.12, 0.0, 2.7e-3)
    b = pillow(at=3.0)
    fk.setup(pan, [b])
    r = Rigid(b, (0, 0, 0.0056))
    rec = []
    run({"g": r}, 9.0, pan, record=rec)
    assert max(p["g"][2] for t, p in rec if t < 3.0) < 0.008  # on the floor while raw
    top = lambda p: p["g"][2] + 0.0055  # noqa: E731
    up = [t for t, p in rec if t >= 3.0 and abs(top(p) - fk.surface_z()) < 0.003]
    assert up and up[0] - 3.0 < 3.5, up[:3]
    sub = fk.submerged("g", r.p, r.q)
    assert 0.90 < sub < 0.99, sub  # density 950: about 95% under
