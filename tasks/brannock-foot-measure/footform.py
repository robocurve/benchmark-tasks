"""Right-foot forms for the Brannock task: one parametric mesh per form, used both by the simulator and as the STL
to 3D-print for real trials. Own design (no third-party geometry), released with the task under CC BY 4.0.

Three numbers per form come from the Brannock tables (sizing.py): heel-to-toe length L, heel-to-ball length A
(heel to the medial ball joint) and ball width W. The rest of the shape is this file's own design, scaled from L
and W: a rounded heel, a medial arch lifted off the sole, ball bulges on both sides, five toes and an ankle post
60 mm across for grasping.

Frame (the device frame of sizing.py when the heel is seated): u from the back of the heel toward the toes,
v toward the medial (big-toe) side, z up from the sole. Millimetres.

The body is a loft of convex cross-sections, so consecutive pairs of sections bound convex pieces: the collision
shapes are the convex hulls of those pairs, of each toe and of the post, i.e. pieces of the very same mesh.
"""

from __future__ import annotations

import math

import numpy as np

POST_R = 30.0  # ankle post radius (60 mm across)
M_BALL = 52.0  # medial ball bulge from the heel's axis, mm (all forms; the lateral bulge takes the rest of W)
POST_TOP = 130.0  # height of the post's top above the sole
N_THETA = 40


def _pchip(xs, ys):
    from scipy.interpolate import PchipInterpolator

    return PchipInterpolator(np.asarray(xs, float), np.asarray(ys, float), extrapolate=True)


class Form:
    def __init__(self, L: float, A: float, W: float, name: str = "form"):
        self.L, self.A, self.W, self.name = L, A, W, name
        R = 0.33 * W  # heel half-width
        self.R = R
        self.m_ball = M_BALL  # medial ball bulge, from the heel axis (the same on every form)
        self.l_ball = W - self.m_ball  # lateral ball bulge
        self.u_lat = A - 0.10 * L  # lateral (fifth) ball joint lies behind the medial one
        self.v_tip = 0.45 * self.m_ball  # big-toe tip
        self.u_end = L - 14.0  # the loft ends at the big toe's base; the toes are separate rounded pieces
        k = L / 262.5
        self.med = _pchip([R, 0.30 * L, 0.45 * L, A - 40, A - 18, A, A + 14, self.u_end],
                          [R, 0.97 * R, 0.90 * R, self.m_ball - 11.0, self.m_ball - 6.0, self.m_ball,
                           self.m_ball - 3.0, self.v_tip + 12.0])
        self.lat = _pchip([R, 0.36 * L, self.u_lat, 0.80 * L],
                          [R, 1.02 * R, self.l_ball, 0.82 * self.l_ball])
        self.height = _pchip([0, 6, 18, 0.20 * L, 0.38 * L, 0.55 * L, A, 0.86 * L, L],
                             [26 * k, 46 * k, 60 * k, 68 * k, 70 * k, 56 * k, 38 * k, 25 * k, 16 * k])
        self.zmid = _pchip([0, 0.2 * L, A, L], [16 * k, 20 * k, 14 * k, 9 * k])
        self.lift = _pchip([0, 0.22 * L, 0.40 * L, 0.55 * L, 0.66 * L, L], [0, 0, 13 * k, 9 * k, 0, 0])

    # -- outline -------------------------------------------------------------------------------------------------
    def v_bounds(self, u: float) -> tuple[float, float]:
        """(lateral, medial) edge of the cross-section at u (v, mm)."""
        R = self.R
        if u <= R:
            h = math.sqrt(max(R * R - (R - u) ** 2, 0.0))
            return -h, h
        vm = float(self.med(u))
        if u <= 0.80 * self.L:
            vl = -float(self.lat(u))
        else:  # the toe line, slanting from the little toe's base to the big toe's
            t = (u - 0.80 * self.L) / (self.u_end - 0.80 * self.L)
            vl = (1 - t) * -float(self.lat(0.80 * self.L)) + t * (self.v_tip - 12.0)
        return vl, vm

    def section(self, u: float, n: int = N_THETA) -> np.ndarray:
        """Convex cross-section at u: (n, 3) points (u, v, z)."""
        vl, vm = self.v_bounds(u)
        a, vc = (vm - vl) / 2, (vm + vl) / 2
        H, zc = float(self.height(u)), float(self.zmid(u))
        lift = float(self.lift(u))
        pts = []
        for th in np.linspace(0, 2 * math.pi, n, endpoint=False):
            c, s = math.cos(th), math.sin(th)
            v = vc + a * math.copysign(abs(c) ** (2 / 2.6), c)
            if s >= 0:
                z = zc + (H - zc) * abs(s) ** (2 / 2.6)
            else:
                z = zc - zc * abs(s) ** (2 / 4.0)
                floor = lift * min(max((v - vl) / max(vm - vl, 1e-6), 0.0), 1.0)  # arch: medial sole lifted
                z = max(z, min(floor, zc - 1.0))
            pts.append((u, v, z))
        return np.array(pts)

    def stations(self) -> np.ndarray:
        L = self.L
        a = np.linspace(0.4, 0.06 * L, 8)
        b = np.arange(0.06 * L, self.u_end, 6.0)
        return np.unique(np.r_[a, b, self.A - 6, self.A, self.A + 6, self.u_lat, self.u_end].round(3))

    # -- pieces --------------------------------------------------------------------------------------------------
    def toes(self) -> list[tuple[np.ndarray, np.ndarray]]:
        """(centre, semi-axes) of five toe ellipsoids, big toe first; the big toe's tip is at u = L."""
        out = [(np.array([self.L - 13.0, self.v_tip, 11.0]), np.array([13.0, 12.5, 10.0]))]
        base_l = -float(self.lat(0.80 * self.L))
        for i, (ax, ay, az) in enumerate([(10.5, 8.5, 8.5), (9.5, 8.0, 8.0), (8.5, 7.5, 7.5), (8.0, 7.5, 7.0)]):
            t = 0.78 - 0.21 * i  # along the toe line, toward the little toe
            v = t * (self.v_tip - 12.0) + (1 - t) * base_l - 3.0
            u_line = 0.80 * self.L + t * (self.u_end - 0.80 * self.L)
            out.append((np.array([u_line + 0.45 * ax, v, az + 1.0]), np.array([ax, ay, az])))
        return out

    def post_center(self) -> tuple[float, float]:
        return 0.21 * self.L, 0.0

    def meshes(self):
        """trimesh shells: loft body, five toes, ankle post (overlapping closed shells)."""
        import trimesh

        secs = [self.section(u) for u in self.stations()]
        n = len(secs[0])
        V = np.concatenate(secs)
        F = []
        for i in range(len(secs) - 1):
            for j in range(n):
                a, b = i * n + j, i * n + (j + 1) % n
                c, d = a + n, b + n
                F += [(a, c, b), (b, c, d)]
        c0 = len(V)
        V = np.vstack([V, secs[0].mean(0), secs[-1].mean(0)])
        last = (len(secs) - 1) * n
        for j in range(n):
            F.append((c0, j, (j + 1) % n))
            F.append((c0 + 1, last + (j + 1) % n, last + j))
        body = trimesh.Trimesh(V, np.array(F), process=True)
        if body.volume < 0:
            body.invert()
        shells = [body]
        for c, ax in self.toes():
            e = trimesh.creation.icosphere(subdivisions=3)
            e.apply_scale(ax)
            e.apply_translation(c)
            shells.append(e)
        pu, pv = self.post_center()
        cyl = trimesh.creation.cylinder(radius=POST_R, height=POST_TOP - 15.0 - 25.0, sections=48)
        cyl.apply_translation((pu, pv, 25.0 + (POST_TOP - 40.0) / 2))
        cap = trimesh.creation.icosphere(subdivisions=3)
        cap.apply_scale((POST_R, POST_R, 15.0))
        cap.apply_translation((pu, pv, POST_TOP - 15.0))
        shells += [trimesh.util.concatenate([cyl, cap]).convex_hull]
        return shells

    def mesh(self):
        import trimesh

        return trimesh.util.concatenate(self.meshes())

    def hulls(self) -> list[np.ndarray]:
        """Convex collision pieces (vertex arrays): consecutive section pairs, each toe, the post."""
        import trimesh

        secs = [self.section(u, 24) for u in self.stations()]
        out = [np.vstack([secs[i], secs[i + 1]]) for i in range(len(secs) - 1)]
        for sh in self.meshes()[1:]:
            out.append(trimesh.convex.convex_hull(sh).vertices)
        return out

    # -- printing ------------------------------------------------------------------------------------------------
    def union_volume_cm3(self, step: float = 1.5) -> float:
        """Volume of the union of the convex pieces (voxel count at `step` mm, half-space tests)."""
        from scipy.spatial import ConvexHull

        eqs = [ConvexHull(h).equations for h in self.hulls()]
        allv = np.vstack(self.hulls())
        lo, hi = allv.min(0), allv.max(0)
        g = np.stack(np.meshgrid(*[np.arange(lo[i] + step / 2, hi[i], step) for i in range(3)], indexing="ij"), -1)
        p = g.reshape(-1, 3)
        inside = np.zeros(len(p), bool)
        for e in eqs:
            inside |= np.all(p @ e[:, :3].T + e[:, 3] <= 1e-9, axis=1)
        return float(inside.sum() * step ** 3 / 1000.0)


# Print settings for the real forms; the simulated mass is the same estimate.
PLA_G_CM3 = 1.24  # PLA density
PRINT = dict(material="PLA", scale="100 %", infill="15 % gyroid", walls="3 perimeters of 0.4 mm (1.2 mm shell)",
             top_bottom="5 layers at 0.2 mm (1.0 mm)", layer="0.2 mm", orientation="sole down, ankle post up")


def print_mass_g(form: Form) -> float:
    """Expected printed mass: a 1.1 mm average shell of solid PLA over the surface plus 15 % infill inside."""
    vol = form.union_volume_cm3()
    area_cm2 = sum(s.area for s in form.meshes()) / 100.0 * 0.8  # overlapping shells: about a fifth is buried
    shell = min(area_cm2 * 0.11, vol)
    return PLA_G_CM3 * (shell + 0.15 * (vol - shell))


def ball_points(form: Form) -> dict[str, np.ndarray]:
    """The medial and lateral ball apexes on the outline (u, v) at the widest level, and the heel's back point."""
    us = np.linspace(form.A - 30, form.A + 20, 501)
    vm = np.array([form.v_bounds(u)[1] for u in us])
    ul = np.linspace(form.u_lat - 30, form.u_lat + 30, 601)
    vl = np.array([form.v_bounds(u)[0] for u in ul])
    return {"medial": np.array([us[vm.argmax()], vm.max()]), "lateral": np.array([ul[vl.argmin()], vl.min()]),
            "heel": np.array([0.0, 0.0])}
