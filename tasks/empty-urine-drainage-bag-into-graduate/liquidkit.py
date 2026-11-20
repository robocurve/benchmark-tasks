"""liquidkit: a lumped, documented liquid model for YAM bimanual tasks (no particles, no beads).

The simulator moves rigid vessels (cups, bowls, ladles, spoons, trays). This module keeps, for each vessel, a volume
of liquid and works out every control step, from the vessel's world pose alone:

  * the free surface: a plane whose normal follows the effective gravity (g minus the liquid's acceleration) through
    the first sloshing mode, a damped oscillator at that mode's natural frequency (the equivalent-pendulum model of
    NASA SP-106 and its update by Dodge), with the viscous damping of a smooth upright tank (Mikishev-Dorozhkin, Dodge
    ch. 2);
  * the level: the plane height at which the vessel's real inside shape holds the current volume (exact slice
    integration: circular segments for round vessels, a closed form for rectangular cells);
  * the outflow over the rim or a spout: a sharp-crested weir, integrated element by element along the crest, so a
    level crest gives the rectangular weir, a sloped crest the V-notch, a tilted round rim the circular weir;
    and through a hole: Torricelli with a discharge coefficient;
  * the stream: it leaves the crest at the critical-flow speed plus the lip's own velocity and falls ballistically;
    it counts as caught only for the part of its width that lands inside a receiving vessel's opening (the real rim,
    no margin); the rest is spilled where the trajectory meets the table (logged as puddles with positions);
  * ladling: a submerged ladle or spoon fills over its own rim at the weir rate until its level meets the pot's;
  * the film a dipped tool carries out and its drips (Landau-Levich film at withdrawal, then Jeffreys gravity
    drainage), released as drops (Tate's law) from the tool's lowest point; a drop lands in a vessel or is a drip mark.

Scoring and anti-gaming helpers (Monitor, zones, tiers) and the drawing layer (surface meshes, stream tubes, drip
marks, all created at env build so frame 0 shows the start state) are at the end. The model part is numpy only and
runs offline. Units: SI (m, m^3, s, kg). A vessel's local frame: z up its axis, origin at the
middle of its inside floor. Quaternions are (x, y, z, w) unless a name says wxyz.

Limits (state these in a task's fidelity text): one planar sloshing mode (no breaking waves or splashes); no surface
tension at the rim (a brim-full vessel spills at zero head, slightly earlier than real water, which can stand a few mm
proud); vessel shapes are convex stacks of round or rectangular sections; a stream is blocked by nothing but the
receiving vessels unless the task passes the arms' hulls to World.step (then a hand in its way stops it, see
stream_blocked); the liquid's weight is not added to the rigid body.

SOURCES: see the SOURCES dict below (URL and the exact figure for every physical constant).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# --------------------------------------------------------------------------------------------------------------------
# Physical constants, each with its source (the exact figure as printed there).
G = 9.80665                 # m/s^2, standard gravity
RHO_WATER = 998.21          # kg/m^3, water at 20 C, 1 atm
NU_WATER = 0.0010016 / 998.21  # m^2/s: dynamic viscosity 0.0010016 Pa s over the density (1.0034e-6)
SIGMA_WATER = 0.072736      # N/m, water at 20 C
WEIR_CD = 0.602             # sharp-crested weir, low head over a tall crest: C_D = 0.602 + 0.075 H/P
ORIFICE_CD = 0.61           # sharp-edged orifice, full contraction
XI_11 = 1.841               # first root of dJ1/dr = 0: the first antisymmetric sloshing mode of an upright cylinder
LL_COEF = 0.94              # Landau-Levich film: h = 0.94 l_c Ca^(2/3)
ML = 1e-6                   # m^3 per mL
US_FL_OZ = 2.957353e-5      # m^3 per US fluid ounce

SOURCES = {
    "G": ("https://physics.nist.gov/cgi-bin/cuu/Value?gn", "standard acceleration of gravity: 9.806 65 m s-2"),
    "RHO_WATER, NU_WATER": (
        "https://webbook.nist.gov/cgi/fluid.cgi?T=20&PLow=0.101325&PHigh=0.101325&PInc=1&Digits=5&ID=C7732185"
        "&Action=Load&Type=IsoTherm&TUnit=C&PUnit=MPa&DUnit=kg%2Fm3&HUnit=kJ%2Fkg&WUnit=m%2Fs&VisUnit=Pa*s"
        "&STUnit=N%2Fm&RefState=DEF",
        "NIST WebBook, water at 20 C, 0.101325 MPa: Density (kg/m3) 998.21; Viscosity (Pa*s) 0.0010016"),
    "SIGMA_WATER": (
        "https://webbook.nist.gov/cgi/fluid.cgi?TLow=20&THigh=25&TInc=5&Digits=5&ID=C7732185&Action=Load"
        "&Type=SatP&TUnit=C&PUnit=MPa&DUnit=kg%2Fm3&HUnit=kJ%2Fkg&WUnit=m%2Fs&VisUnit=Pa*s&STUnit=N%2Fm&RefState=DEF",
        "NIST WebBook saturation curve, 20 C: Surf. Tension (N/m) 0.072736"),
    "WEIR_CD": ("https://www.johndfenton.com/Papers/Calculating-flow-over-rectangular-sharp-edged-weirs.pdf",
                "Fenton (2015), after eq. (24): 'C_D = 0.602 + 0.075 H/P' (weir across the channel, Kindsvater-"
                "Carter); weir law 'Q = C_d (2/3) sqrt(2g) b H^(3/2)', V-notch 'Q = C_d (8/15) sqrt(2g) tan(theta/2)"
                " H^(5/2)': https://eng.libretexts.org/Bookshelves/Mechanical_Engineering/Applied_Fluid_Mechanics_"
                "Lab_Manual_(Ahmari_and_Kabir)/01%3A_Lab_Manual/01.9%3A_Experiment_%239%3A_Flow_Over_Weirs"),
    "ORIFICE_CD": ("https://www.usbr.gov/tsc/techreferences/mands/wmm/chap02_08.html",
                   "USBR Water Measurement Manual 2.8: 'the contraction coefficient is about 0.61'"),
    "XI_11, slosh frequencies": (
        "https://www2.swri.org/www2/fluid-slosh/dodge-sp-106-update.pdf",
        "Dodge, The New Dynamic Behavior of Liquids in Moving Containers (update of NASA SP-106): eq. (1.40) "
        "omega^2 = (g eps/a) tanh(eps h/a), eq. (1.41) 'xi1 = 1.841'; rectangular eq. (1.19a), n = 1: "
        "omega^2 = (pi g/a) tanh(pi h/a). Original: https://ntrs.nasa.gov/citations/19670006555"),
    "slosh damping": (
        "https://www2.swri.org/www2/fluid-slosh/dodge-sp-106-update.pdf",
        "Dodge ch. 2, Mikishev and Dorozhkin [1961], eq. (2.6a): gamma = 0.79 sqrt(Re1) [1 + (0.318/sinh(1.84h/R))"
        " (1 + (1 - h/R)/cosh(1.84h/R))], Re1 = nu/(g^1/2 R^3/2); valid while the wall amplitude <= about 0.1 R"),
    "LL_COEF": ("https://arxiv.org/html/1903.09350v1", "eq. (2): 'h = 0.94 l_c Ca^{2/3}', l_c = sqrt(gamma/(rho g)),"
                " Ca = eta U/gamma"),
    "Jeffreys drainage": ("https://par.nsf.gov/servlets/purl/10268476",
                          "'h_J = (mu x / rho g t)^{1/2} ... known as Jeffreys' solution' (Jeffreys 1930, Proc. Camb."
                          " Phil. Soc. 26, 204)"),
    "Tate's law": ("https://en.wikipedia.org/wiki/Stalagmometric_method", "'mg = 2 pi r sigma': the drop falls when "
                   "its weight equals the circumference times the surface tension"),
    "capillary length": ("https://physicstoday.aip.org/quick-study/your-wetting-day",
                         "'For the water-air interface, l_c = 2.7 mm.'"),
    "puddle thickness": ("https://arxiv.org/html/2402.10185", "eq. (59): h_inf = 2 sqrt(sigma/(rho_L g)) "
                         "sin(theta_s/2)"),
    "critical depth": ("https://www.codecogs.com/library/engineering/fluid_mechanics/weirs/discharge.php",
                       "broad-crested weir: discharge is maximum at h = (2/3) H, v = sqrt(2 g (H - h)) "
                       "(= sqrt(g h) there)"),
    "US_FL_OZ": ("https://www.nist.gov/pml/special-publication-811/nist-guide-si-appendix-b-conversion-factors/"
                 "nist-guide-si-appendix-b8", "ounce (U.S. fluid) (fl oz) to milliliter: 2.957 353 E+01"),
}

N_SLICES = 240              # z-slices for the volume integrals (midpoint rule)
RIM_K = 96                  # crest elements around a rim
SURF_K = 48                 # boundary points of a drawn free surface (fixed mesh topology)


def capillary_length(sigma: float = SIGMA_WATER, rho: float = RHO_WATER) -> float:
    """kappa^-1 = sqrt(gamma / (rho g)): 2.7 mm for water."""
    return math.sqrt(sigma / (rho * G))


# --------------------------------------------------------------------------------------------------------------------
# rotations

def rot(q) -> np.ndarray:
    """3x3 rotation matrix from a quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat(R) -> tuple:
    """Rotation matrix to a quaternion (x, y, z, w)."""
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


def axis_quat(axis, deg: float) -> tuple:
    """Quaternion (x, y, z, w) turning `deg` degrees about `axis`."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    h = math.radians(deg) / 2
    return (*(a * math.sin(h)), math.cos(h))


def tilt_deg(q) -> float:
    """Angle (deg) between the vessel's axis and straight up."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rot(q)[2, 2])))))


# --------------------------------------------------------------------------------------------------------------------
# geometry of a vessel's inside

@dataclass
class Shape:
    """The inside of a vessel: a stack of sections from the floor (z = 0) to the rim (z = height), round
    (`profile` = [(z, r), ...]) or rectangular (`profile` = [(z, half_x, half_y), ...]), linear between the rows.
    The sections must widen (or stay) upward so the inside is convex (cups, bowls, ladles, tray cells, pans)."""

    profile: list
    round: bool = True

    def __post_init__(self):
        p = np.asarray(self.profile, float)
        if p.ndim != 2 or len(p) < 2 or p[0, 0] != 0.0 or np.any(np.diff(p[:, 0]) <= 0):
            raise ValueError("profile rows must start at z = 0 and rise")
        self._p = p
        self.height = float(p[-1, 0])
        dz = self.height / N_SLICES
        self._z = (np.arange(N_SLICES) + 0.5) * dz
        self._dz = dz
        self._r = self.section(self._z)                       # (N,) radius or (N, 2) half sizes

    def section(self, z):
        z = np.clip(np.asarray(z, float), 0.0, self.height)
        if self.round:
            return np.interp(z, self._p[:, 0], self._p[:, 1])
        return np.stack([np.interp(z, self._p[:, 0], self._p[:, 1]), np.interp(z, self._p[:, 0], self._p[:, 2])], -1)

    # -- volumes ------------------------------------------------------------------------------------------------------
    def _area_below(self, a: float, b: float, c: np.ndarray) -> np.ndarray:
        """Area of each slice's section where a x + b y <= c (c per slice)."""
        s = math.hypot(a, b)
        if self.round:
            r = self._r
            full = math.pi * r * r
            if s < 1e-12:
                return np.where(c >= 0.0, full, 0.0)
            t = np.clip(c / s, -r, r)
            return r * r * (math.pi - np.arccos(np.clip(t / np.maximum(r, 1e-12), -1, 1))) + t * np.sqrt(
                np.maximum(r * r - t * t, 0.0))
        hx, hy = self._r[:, 0], self._r[:, 1]
        full = 4 * hx * hy
        al, be = abs(a) * hx, abs(b) * hy
        if s < 1e-12:
            return np.where(c >= 0.0, full, 0.0)
        R2 = lambda t: np.maximum(t, 0.0) ** 2 / 2  # noqa: E731
        big, small = np.maximum(al, be), np.minimum(al, be)
        thin = small < 1e-9 * np.maximum(big, 1e-12)
        with np.errstate(divide="ignore", invalid="ignore"):
            f2 = (R2(c + al + be) - R2(c + al - be) - R2(c - al + be) + R2(c - al - be)) / (4 * al * be)
            f1 = np.clip((c + big) / (2 * big), 0.0, 1.0)
        f = np.where(thin, f1, f2)
        return full * np.clip(np.nan_to_num(f), 0.0, 1.0)

    def _area_integral(self, a: float, b: float, c: np.ndarray) -> np.ndarray:
        """Antiderivative in c of _area_below (per slice, its section held constant): G(c) with dG/dc = A(c)."""
        s = math.hypot(a, b)
        if self.round:
            r = self._r
            if s < 1e-12:
                return math.pi * r * r * np.maximum(c, 0.0)
            t = c / s
            tc = np.clip(t, -r, r)
            rr = np.maximum(r, 1e-12)
            root = np.sqrt(np.maximum(r * r - tc * tc, 0.0))
            F = math.pi * r * r * tc - r * r * (tc * np.arccos(np.clip(tc / rr, -1, 1)) - root) - root ** 3 / 3
            F = F + math.pi * r * r * np.maximum(t - r, 0.0)       # past the far side: the full section
            return s * F
        hx, hy = self._r[:, 0], self._r[:, 1]
        full = 4 * hx * hy
        al, be = abs(a) * hx, abs(b) * hy
        if s < 1e-12:
            return full * np.maximum(c, 0.0)
        R3 = lambda t: np.maximum(t, 0.0) ** 3 / 6  # noqa: E731
        R2 = lambda t: np.maximum(t, 0.0) ** 2 / 2  # noqa: E731
        big, small = np.maximum(al, be), np.minimum(al, be)
        thin = small < 1e-6 * np.maximum(big, 1e-12)
        with np.errstate(divide="ignore", invalid="ignore"):
            g2 = (R3(c + al + be) - R3(c + al - be) - R3(c - al + be) + R3(c - al - be)) / (4 * al * be)
            g1 = (R2(c + big) - R2(c - big)) / (2 * big)
        return full * np.nan_to_num(np.where(thin, g1, g2))

    def volume_below(self, n_local, d: float) -> float:
        """Inside volume where n . x <= d (n a unit vector in the vessel's frame). Each slice is integrated exactly
        through its height (its section held at the slice's middle), so the level moves smoothly with the volume."""
        n = np.asarray(n_local, float)
        if abs(n[2]) * self._dz < 1e-9:
            c = d - n[2] * self._z
            return float(self._area_below(n[0], n[1], c).sum() * self._dz)
        c0 = d - n[2] * (self._z - self._dz / 2)
        c1 = d - n[2] * (self._z + self._dz / 2)
        G0 = self._area_integral(n[0], n[1], c0)
        G1 = self._area_integral(n[0], n[1], c1)
        return float(((G0 - G1) / n[2]).sum())

    @property
    def capacity(self) -> float:
        return self.volume_below((0.0, 0.0, 1.0), self.height + 1.0)

    def depth_for(self, volume: float) -> float:
        """Upright fill depth holding `volume` (exact for the piecewise-linear profile)."""
        return self.plane_for((0.0, 0.0, 1.0), volume)

    def corners(self, k: int = 64) -> np.ndarray:
        """Points on every section ring at the profile's rows (bounds the inside)."""
        out = []
        for row in self._p:
            out.append(self.ring(row[0], k))
        return np.concatenate(out)

    def ring(self, z: float, k: int = RIM_K) -> np.ndarray:
        """k points around the section at height z (closed loop, counter-clockwise seen from above)."""
        a = np.linspace(0.0, 2 * math.pi, k, endpoint=False)
        if self.round:
            r = float(self.section(z))
            return np.stack([r * np.cos(a), r * np.sin(a), np.full(k, z)], 1)
        hx, hy = (float(v) for v in self.section(z))
        u = np.stack([np.cos(a), np.sin(a)], 1)
        u = u / np.max(np.abs(u) / np.array([hx, hy]), axis=1, keepdims=True)  # square the circle onto the rectangle
        return np.stack([u[:, 0], u[:, 1], np.full(k, z)], 1)

    def plane_for(self, n_local, volume: float) -> float:
        """The plane height d (n . x = d) that holds `volume` below it."""
        n = np.asarray(n_local, float)
        h = self.corners() @ n
        lo, hi = float(h.min()), float(h.max())
        if volume <= 0:
            return lo
        if volume >= self.capacity:
            return hi
        for _ in range(48):
            mid = (lo + hi) / 2
            if self.volume_below(n, mid) < volume:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    def inside(self, local_pts, tol: float = 0.0) -> np.ndarray:
        """Which local points are inside the hollow (walls shrunk by tol)."""
        p = np.atleast_2d(np.asarray(local_pts, float))
        z = p[:, 2]
        ok = (z >= -1e-9) & (z <= self.height + 1e-9)
        s = self.section(z)
        if self.round:
            return ok & (np.hypot(p[:, 0], p[:, 1]) <= s - tol)
        return ok & (np.abs(p[:, 0]) <= s[:, 0] - tol) & (np.abs(p[:, 1]) <= s[:, 1] - tol)

    def section_polygon(self, n_local, d: float, k: int = 64):
        """The free surface's outline in the vessel frame: the plane n . x = d cut through the (convex) inside, as an
        (m, 3) loop ordered around its centroid, or None if the plane misses the inside."""
        n = np.asarray(n_local, float)
        rows = [self.ring(z, k) for z in self._p[:, 0]]
        pts = []
        for ring in (rows[0], rows[-1]):                           # floor and rim edges
            h = ring @ n - d
            h2 = np.roll(h, -1)
            nxt = np.roll(ring, -1, axis=0)
            cross = (h <= 0) != (h2 <= 0)
            t = h[cross] / (h[cross] - h2[cross])
            pts.append(ring[cross] + t[:, None] * (nxt[cross] - ring[cross]))
        for lo_r, hi_r in zip(rows[:-1], rows[1:]):                # wall generators
            h1, h2 = lo_r @ n - d, hi_r @ n - d
            cross = (h1 <= 0) != (h2 <= 0)
            t = h1[cross] / (h1[cross] - h2[cross])
            pts.append(lo_r[cross] + t[:, None] * (hi_r[cross] - lo_r[cross]))
        pts = np.concatenate(pts) if pts else np.zeros((0, 3))
        if len(pts) < 3:
            return None
        c = pts.mean(0)
        e1 = np.cross(n, [1.0, 0.0, 0.0] if abs(n[0]) < 0.9 else [0.0, 1.0, 0.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ang = np.arctan2((pts - c) @ e2, (pts - c) @ e1)
        return pts[np.argsort(ang)]


def cylinder(radius: float, height: float) -> Shape:
    return Shape([(0.0, radius), (height, radius)])


def frustum(r_bottom: float, r_top: float, height: float) -> Shape:
    """A straight-sided round bowl, cup, ladle bowl or graduate."""
    return Shape([(0.0, r_bottom), (height, r_top)])


def box(lx: float, ly: float, height: float) -> Shape:
    """A rectangular cell or pan (inside lengths)."""
    return Shape([(0.0, lx / 2, ly / 2), (height, lx / 2, ly / 2)], round=False)


def tapered_box(lx0: float, ly0: float, lx1: float, ly1: float, height: float) -> Shape:
    """A rectangular cell whose walls flare from (lx0, ly0) at the floor to (lx1, ly1) at the rim (ice-tray cell)."""
    return Shape([(0.0, lx0 / 2, ly0 / 2), (height, lx1 / 2, ly1 / 2)], round=False)


# --------------------------------------------------------------------------------------------------------------------
# flow laws

def weir_flow(heads, lengths, cd: float = WEIR_CD) -> float:
    """Sharp-crested weir integrated along the crest: Q = Cd (2/3) sqrt(2g) sum(h_i^1.5 L_i), with h_i the head over
    each crest element and L_i its horizontal length. A level crest of length b gives Cd (2/3) sqrt(2g) b H^1.5; a
    V-notch of half-angle phi gives Cd (8/15) sqrt(2g) tan(phi) H^2.5 (both exact limits of this sum)."""
    h = np.maximum(np.asarray(heads, float), 0.0)
    return float(cd * (2.0 / 3.0) * math.sqrt(2 * G) * np.sum(h ** 1.5 * np.asarray(lengths, float)))


def orifice_flow(area: float, head: float, cd: float = ORIFICE_CD) -> float:
    """Torricelli with a discharge coefficient: Q = Cd A sqrt(2 g h)."""
    return cd * area * math.sqrt(2 * G * head) if head > 0 else 0.0


def slosh_omega(shape: Shape, depth: float, along=None) -> float:
    """First sloshing mode's angular frequency (rad/s) of the upright vessel at fill depth `depth`. Round:
    omega^2 = (g xi11 / R) tanh(xi11 h / R), R the radius at the surface. Rectangular: omega^2 = (g pi / L) tanh(pi h / L),
    L the length along `along` ("x" or "y"; default the longer side)."""
    depth = max(depth, 1e-4)
    s = shape.section(depth)
    if shape.round:
        k = XI_11 / max(float(s), 1e-4)
    else:
        hx, hy = float(s[0]), float(s[1])
        L = 2 * (hx if along == "x" else hy if along == "y" else max(hx, hy))
        k = math.pi / max(L, 1e-4)
    return math.sqrt(G * k * math.tanh(k * depth))


def slosh_damping(radius: float, depth: float, nu: float = NU_WATER) -> float:
    """Damping ratio of the first mode in a smooth upright cylinder (Mikishev-Dorozhkin; Dodge eq. 2.6a)."""
    radius, depth = max(radius, 1e-4), max(depth, 1e-4)
    re = math.sqrt(G * radius ** 3) / nu
    x = 1.84 * depth / radius
    return 0.79 / math.sqrt(re) * (1 + 0.318 / math.sinh(x) * (1 + (1 - depth / radius) / math.cosh(x)))


def ballistic(p0, v0, plane_point, plane_normal):
    """When and where the trajectory p0 + v0 t - g t^2/2 z crosses the plane going down through it (from the side the
    normal points to). Returns (t, point) or None."""
    p0, v0, m = np.asarray(p0, float), np.asarray(v0, float), np.asarray(plane_normal, float)
    s0 = float((p0 - np.asarray(plane_point, float)) @ m)
    if s0 < 0:
        return None
    # s(t) = s0 + (v0.m) t - (g m_z / 2) t^2
    a, b, c = -0.5 * G * m[2], float(v0 @ m), s0
    if abs(a) < 1e-12:
        if b >= 0:
            return None
        t = -c / b
    else:
        disc = b * b - 4 * a * c
        if disc < 0:
            return None
        r = sorted([(-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)])
        pos = [t for t in r if t >= 0]
        if not pos:
            return None
        t = pos[0]
    return t, p0 + v0 * t - 0.5 * G * t * t * np.array([0.0, 0.0, 1.0])


def math_jet_radius(rate: float, speed: float) -> float:
    """Radius of a round jet carrying `rate` (m^3/s) at `speed` (continuity)."""
    return math.sqrt(max(rate, 0.0) / (math.pi * max(speed, 0.05)))


def jet_fraction(dist_to_edge: float, half_width: float) -> float:
    """Share of a jet of half-width w whose center lands `dist_to_edge` inside an opening's edge (negative outside)."""
    if half_width <= 1e-9:
        return 1.0 if dist_to_edge >= 0 else 0.0
    return min(max((dist_to_edge + half_width) / (2 * half_width), 0.0), 1.0)


def puddle_radius(volume: float, contact_deg: float = 70.0) -> float:
    """Radius of a spilled volume resting on the table: a spherical cap below the capillary length, a gravity puddle
    of thickness e = 2 l_c sin(theta/2) above it (de Gennes et al. 2004 ch. 2). For drawing drip marks."""
    if volume <= 0:
        return 0.0
    th = math.radians(contact_deg)
    cap = (2 - 3 * math.cos(th) + math.cos(th) ** 3) / (3 * math.sin(th) ** 3)
    r_cap = (volume / (math.pi * cap)) ** (1 / 3)
    lc = capillary_length()
    e = 2 * lc * math.sin(th / 2)
    if r_cap * (1 - math.cos(th)) / math.sin(th) <= e:
        return r_cap
    return math.sqrt(volume / (math.pi * e))


def drop_volume(radius: float | None = None, sigma: float = SIGMA_WATER, rho: float = RHO_WATER) -> float:
    """Tate's law: V = 2 pi r gamma / (rho g); r defaults to the capillary length (a drop forming under a broad
    wetted surface). Real drops leave a residue, so this is the largest drop: it sets how the dripped volume is
    split into marks, never how much drips."""
    r = capillary_length(sigma, rho) if radius is None else radius
    return 2 * math.pi * r * sigma / (rho * G)


# --------------------------------------------------------------------------------------------------------------------
# a vessel

@dataclass
class Vessel:
    """A rigid vessel with liquid in it. `spout`: optional (local point, width): a notch whose crest sits at that point
    (it pours there first). `hole`: optional (local point, area): an orifice (a drain or a nozzle). `receiver`: a
    stream can land in it. `teleport` (m per control step): a pose change bigger than this is a reset, not motion
    (5 m/s at 20 Hz, beyond the arm's top speed); anything smaller is real motion and sloshes the liquid."""

    name: str
    shape: Shape
    volume: float = 0.0
    spout: tuple | None = None
    hole: tuple | None = None
    receiver: bool = True
    nu: float = NU_WATER
    teleport: float = 0.25

    def __post_init__(self):
        rim = self.shape.ring(self.shape.height, RIM_K)
        if self.spout is not None:
            sp, width = np.asarray(self.spout[0], float), float(self.spout[1])
            tang = np.cross([0.0, 0.0, 1.0], sp * [1, 1, 0])
            tang = tang / max(np.linalg.norm(tang), 1e-9)
            k = 9
            seg = sp + np.outer(np.linspace(-width / 2, width / 2, k), tang)
            j = int(np.argmin(np.linalg.norm(rim[:, :2] - sp[:2], axis=1)))
            rim = np.concatenate([rim[:j], seg, rim[j + 1:]])
        self._crest = rim                                       # closed crest loop (local)
        mids = (rim + np.roll(rim, -1, axis=0)) / 2
        out = mids * [1, 1, 0]
        self._out_dir = out / np.maximum(np.linalg.norm(out, axis=1, keepdims=True), 1e-9)
        self._n = np.array([0.0, 0.0, 1.0])                    # free-surface normal (world)
        self._ndot = np.zeros(3)
        self._p = None                                          # last liquid-center position (world)
        self._v = None
        self._pose = None                                       # last (pos, R)
        self.accel = np.zeros(3)
        self.out = None                                         # last step's outflow (dict) or None
        self.jumped = False

    @property
    def capacity(self) -> float:
        return self.shape.capacity

    # -- geometry at a pose ------------------------------------------------------------------------------------------
    def frame(self, pos, quat_):
        return np.asarray(pos, float), rot(quat_)

    def _plane(self, pos, quat_, normal=None, volume=None):
        """(world normal, world plane offset d_w, local normal, local d) of the free surface holding `volume`."""
        p, R = self.frame(pos, quat_)
        n = self._n if normal is None else np.asarray(normal, float)
        nl = R.T @ n
        dl = self.shape.plane_for(nl, self.volume if volume is None else volume)
        return n, dl + float(n @ p), nl, dl

    def held_at(self, pos, quat_, normal=None) -> float:
        """Most the vessel holds at this pose: the inside below its lowest crest point (along the surface normal)."""
        p, R = self.frame(pos, quat_)
        n = self._n if normal is None else np.asarray(normal, float)
        nl = R.T @ n
        return self.shape.volume_below(nl, float((self._crest @ nl).min()))

    def level(self, pos, quat_) -> float:
        """World height (along the surface normal) of the free surface."""
        return self._plane(pos, quat_)[1]

    def depth(self) -> float:
        """Upright depth of the current volume (for the slosh frequency)."""
        return self.shape.depth_for(self.volume)

    def surface(self, pos, quat_, inset: float = 0.001):
        """Drawing hook: the free surface as (center, normal, (SURF_K, 3) boundary points), world frame, the outline
        pulled `inset` inside the walls; None if empty."""
        if self.volume <= 1e-12:
            return None
        p, R = self.frame(pos, quat_)
        n, dw, nl, dl = self._plane(pos, quat_)
        poly = self.shape.section_polygon(nl, dl)
        if poly is None:
            return None
        poly = poly @ R.T + p
        c = poly.mean(0)
        e1 = np.cross(n, [1.0, 0.0, 0.0] if abs(n[0]) < 0.9 else [0.0, 1.0, 0.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        rel = poly - c
        u = np.stack([rel @ e1, rel @ e2], 1)                  # the outline in the plane, ordered around c
        a, b = u, np.roll(u, -1, axis=0)
        want = np.linspace(-math.pi, math.pi, SURF_K, endpoint=False)
        d = np.stack([np.cos(want), np.sin(want)], 1)          # rays from c; exact hit on the outline's edges
        e = b - a
        den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (a[None, :, 0] * e[None, :, 1] - a[None, :, 1] * e[None, :, 0]) / den
            s_ = (a[None, :, 0] * d[:, None, 1] - a[None, :, 1] * d[:, None, 0]) / den
        ok = (np.abs(den) > 1e-15) & (t >= 0) & (s_ >= -1e-9) & (s_ <= 1 + 1e-9)
        r = np.where(ok, t, np.inf).min(1)
        r = np.where(np.isfinite(r), r, 0.0)
        r = np.maximum(r - inset, 0.0)
        pts = c + np.outer(r * np.cos(want), e1) + np.outer(r * np.sin(want), e2)
        return c, n.copy(), pts

    # -- motion and slosh ---------------------------------------------------------------------------------------------
    def _center_local(self) -> np.ndarray:
        return np.array([0.0, 0.0, max(self.depth(), 1e-3) / 2])

    def _move(self, pos, quat_, dt: float) -> None:
        """Track the liquid's acceleration and advance the slosh oscillator one control step."""
        p, R = self.frame(pos, quat_)
        c = p + R @ self._center_local()
        self.jumped = False
        if self._p is not None and float(np.linalg.norm(c - self._p)) > self.teleport:
            self._p = None                                     # a reset or respawn: restart the motion state
            self.jumped = True
        if self._p is None:
            self._p, self._v, self.accel = c, np.zeros(3), np.zeros(3)
            self._pose = (p, R)
            return
        v = (c - self._p) / dt
        a = (v - self._v) / dt
        self._p, self._v, self.accel = c, v, a
        self._pose = (p, R)
        # target surface normal: along (a + g up); clamp the vertical part (free fall: see Monitor.airborne)
        up = G + a[2]
        up = max(up, 0.2 * G)
        target = np.array([a[0], a[1], up])
        target /= np.linalg.norm(target)
        depth = self.depth()
        if self.shape.round:
            rs = float(self.shape.section(depth))
            w = np.array([slosh_omega(self.shape, depth)] * 2)
        else:
            rs = float(max(self.shape.section(depth)))
            w = np.array([slosh_omega(self.shape, depth, "x"), slosh_omega(self.shape, depth, "y")])
        zeta = slosh_damping(rs, depth, self.nu)
        # oscillate the surface's horizontal tilt components in the vessel's own horizontal axes
        ax = R[:, 0] * [1, 1, 0]
        ax = ax / np.linalg.norm(ax) if np.linalg.norm(ax) > 1e-6 else np.array([1.0, 0.0, 0.0])
        ay = np.cross([0.0, 0.0, 1.0], ax)
        E = np.stack([ax, ay])                                 # 2x3
        th, thd = E @ (self._n / self._n[2]), E @ self._ndot
        th_t = E @ (target / target[2])
        m = max(1, int(math.ceil(float(w.max()) * dt / 0.05)))
        h = dt / m
        for _ in range(m):
            thd = thd + h * (w * w * (th_t - th) - 2 * zeta * w * thd)
            th = th + h * thd
        n = np.array([*(E.T @ th)[:2], 1.0])
        self._n = n / np.linalg.norm(n)
        self._ndot = E.T @ thd

    # -- outflow ------------------------------------------------------------------------------------------------------
    def crest_world(self, pos, quat_):
        p, R = self.frame(pos, quat_)
        return self._crest @ R.T + p

    def step(self, pos, quat_, dt: float) -> float:
        """Advance one control step at this pose; returns the volume that left (m^3). `self.out` describes the
        stream: point, velocity, flow rate, half-width."""
        prev = self._pose
        self._move(pos, quat_, dt)
        self.out = None
        if self.volume <= 0:
            return 0.0
        p, R = self.frame(pos, quat_)
        n, dw, nl, dl = self._plane(pos, quat_)
        crest = self._crest @ R.T + p
        heads_y = crest @ n
        nxt = np.roll(crest, -1, axis=0)
        seg = nxt - crest
        mid = (crest + nxt) / 2
        hy = (heads_y + np.roll(heads_y, -1)) / 2
        L = np.linalg.norm(seg - np.outer(seg @ n, n), axis=1)
        head = dw - hy
        left = 0.0
        q = weir_flow(head, L)
        excess = self.volume - self.held_at(pos, quat_)
        if q > 0 and excess > 0:
            left = min(excess, q * dt)
        out = None
        if left > 0:
            wts = np.maximum(head, 0.0) ** 1.5 * L
            wts = wts / wts.sum()
            pt = wts @ mid
            hbar = float(wts @ np.maximum(head, 0.0))
            odir = (self._out_dir @ R.T) * [1, 1, 0]
            odir = wts @ odir
            odir = odir / max(np.linalg.norm(odir), 1e-9)
            v_out = math.sqrt(G * 2.0 / 3.0 * hbar) * odir   # critical-flow speed at the crest
            v_lip = np.zeros(3)
            if prev is not None and not self.jumped:
                lp_local = R.T @ (pt - p)
                v_lip = (pt - (prev[1] @ lp_local + prev[0])) / dt
            wet = mid[head > 0]
            lat = np.cross([0.0, 0.0, 1.0], odir)
            half = float(np.ptp(wet @ lat)) / 2 if len(wet) > 1 else 0.0
            out = {"point": pt, "vel": v_out + v_lip, "rate": left / dt, "half_width": half, "kind": "rim"}
        if self.hole is not None and self.volume - left > 0:
            hp = p + R @ np.asarray(self.hole[0], float)
            hh = dw - float(hp @ n)
            qh = min(orifice_flow(float(self.hole[1]), hh) * dt, self.volume - left)
            if qh > 0:
                left += qh
                if out is None:
                    vdir = -(R @ np.array([0.0, 0.0, 1.0]))
                    r = math.sqrt(float(self.hole[1]) / math.pi)
                    out = {"point": hp, "vel": math.sqrt(2 * G * hh) * vdir, "rate": qh / dt, "half_width": r,
                           "kind": "hole"}
        self.volume -= left
        self.out = out
        return left


# --------------------------------------------------------------------------------------------------------------------
# films and drips on dipped tools

@dataclass
class Film:
    """The liquid film a tool (ladle, spoon) carries out of a liquid on its outside, and its drips.

    On withdrawal the wetted outside (perimeter `perimeter` x the depth it was dipped) carries a Landau-Levich film,
    h0 = 0.94 l_c Ca^(2/3) with Ca = mu U / gamma at the withdrawal speed U; then it drains under gravity (Jeffreys:
    h = sqrt(nu x / (g t)), started at the t0 that matches h0's mean). What drains collects at the tool's lowest
    point and falls as drops of Tate's volume."""

    perimeter: float
    nu: float = NU_WATER
    sigma: float = SIGMA_WATER
    rho: float = RHO_WATER
    wet_depth: float = 0.0
    t: float = 0.0
    t0: float = 0.0
    on: float = 0.0          # m^3 on the tool's outside now
    pendant: float = 0.0     # m^3 hanging at the lowest point
    drop: float = field(default_factory=drop_volume)

    def film_volume(self, t: float) -> float:
        L = self.wet_depth
        if L <= 0:
            return 0.0
        return self.perimeter * (2.0 / 3.0) * L ** 1.5 * math.sqrt(self.nu / (G * max(t, 1e-9)))

    def withdraw(self, depth: float, speed: float) -> float:
        """The tool leaves the liquid, wetted to `depth` (m) at `speed` (m/s). Returns the film volume taken out."""
        if depth <= 0:
            return 0.0
        mu = self.nu * self.rho
        ca = mu * max(speed, 1e-4) / self.sigma
        h0 = LL_COEF * capillary_length(self.sigma, self.rho) * ca ** (2.0 / 3.0)
        self.wet_depth = depth
        # Jeffreys' mean thickness over the wetted depth, (2/3) sqrt(nu L / (g t)), equals h0 at t0
        self.t0 = (4.0 / 9.0) * self.nu * depth / (G * h0 * h0)
        self.t = self.t0
        taken = self.perimeter * depth * h0
        self.on += taken
        return taken

    def step(self, dt: float) -> int:
        """Drain for dt; returns how many drops fell this step."""
        if self.on <= 0:
            return 0
        v0, v1 = self.film_volume(self.t), self.film_volume(self.t + dt)
        self.t += dt
        drained = min(max(v0 - v1, 0.0), self.on)
        self.on -= drained
        self.pendant += drained
        n = int(self.pendant // self.drop)
        self.pendant -= n * self.drop
        return n


# --------------------------------------------------------------------------------------------------------------------
# a scene of vessels

@dataclass
class Puddle:
    x: float
    y: float
    volume: float
    drips: int = 0

    @property
    def radius(self) -> float:
        return puddle_radius(self.volume)


@dataclass
class World:
    """Vessels, the streams between them and the liquid that ends up on the table.

    Each step: `step(poses, dt)` with poses[name] = (pos, quat xyzw) for every vessel (the vessel frame: inside floor
    center, z up its axis). `exchange` lists (tool, pot) pairs where a submerged tool fills over its rim (ladling).
    `films` maps a tool name to its Film."""

    vessels: dict
    table_z: float = 0.0
    exchange: tuple = ()
    films: dict = field(default_factory=dict)
    spilled: float = 0.0
    dripped: float = 0.0
    puddles: list = field(default_factory=list)
    log: list = field(default_factory=list)       # (step, source, dest or "table", m^3, (x, y))
    moved: dict = field(default_factory=dict)     # (source, dest) -> m^3 that went there
    streams: dict = field(default_factory=dict)   # source -> drawable stream (points, half-width, caught) this step
    drops: list = field(default_factory=list)     # (x, y, z) where drops fell this step (for drawing)
    hand_hits: list = field(default_factory=list)  # (step, source, robot link, m^3) streams that ran into a hand
    n: int = 0
    _sub: dict = field(default_factory=dict)      # tool -> submerged depth last step

    def total(self) -> float:
        """Every drop of liquid in the scene: constant over a trial."""
        return float(sum(v.volume for v in self.vessels.values()) + self.spilled + self.dripped
                     + sum(f.on + f.pendant for f in self.films.values()))

    def _receivers(self, poses, exclude):
        for name, v in self.vessels.items():
            if name != exclude and v.receiver:
                yield name, v, poses[name]

    def _catch(self, src, pt, vel, rate, poses):
        """Where a stream from `pt` with velocity `vel` carrying `rate` (m^3/s) goes: [(dest, share, landing point)],
        dest None = the table. Its width where it meets a rim is a round jet carrying that flow at the speed it has
        there (continuity: r = sqrt(Q / (pi v))); the share inside the opening goes in."""
        for name, v, (rp, rq) in self._receivers(poses, src):
            p, R = v.frame(rp, rq)
            if v.shape.inside(((np.asarray(pt) - p) @ R)[None])[0]:
                return [(name, 1.0, np.asarray(pt))]
        best = None
        for name, v, (rp, rq) in self._receivers(poses, src):
            p, R = v.frame(rp, rq)
            top = p + R @ np.array([0.0, 0.0, v.shape.height])
            hit = ballistic(pt, vel, top, R[:, 2])
            if hit is None:
                continue
            t, lp = hit
            if best is None or t < best[0]:
                best = (t, name, v, p, R, lp)
        half = 0.0
        if best is not None and rate > 0:
            vl = np.asarray(vel, float) - G * best[0] * np.array([0.0, 0.0, 1.0])
            half = math_jet_radius(rate, float(np.linalg.norm(vl)))
        parts = []
        share = 1.0
        if best is not None:
            _t, name, v, p, R, lp = best
            loc = R.T @ (lp - p)
            s = v.shape.section(v.shape.height)
            if v.shape.round:
                f = jet_fraction(float(s) - math.hypot(loc[0], loc[1]), half)
            else:
                f = jet_fraction(float(s[0]) - abs(loc[0]), half) * jet_fraction(float(s[1]) - abs(loc[1]), half)
            if f > 0:
                parts.append((name, f, lp))
                share -= f
        if share > 1e-12:
            hit = ballistic(pt, vel, (0.0, 0.0, self.table_z), (0.0, 0.0, 1.0))
            lp = hit[1] if hit is not None else np.array([pt[0], pt[1], self.table_z])
            parts.append((None, share, lp))
        return parts

    def _to_table(self, src, vol, xy, drip=False):
        if drip:
            self.dripped += vol
        else:
            self.spilled += vol
        x, y = float(xy[0]), float(xy[1])
        for pd in self.puddles:
            if math.hypot(pd.x - x, pd.y - y) <= pd.radius:
                pd.volume += vol
                pd.drips += int(drip)
                break
        else:
            self.puddles.append(Puddle(x, y, vol, int(drip)))
        self.log.append((self.n, src, "table", vol, (x, y)))
        self.moved[(src, "table")] = self.moved.get((src, "table"), 0.0) + vol

    def _into(self, src, dest, vol):
        rv = self.vessels[dest]
        room = max(rv.capacity - rv.volume, 0.0)
        got = min(vol, room)
        rv.volume += got
        self.moved[(src, dest)] = self.moved.get((src, dest), 0.0) + got
        self.log.append((self.n, src, dest, got, None))
        return vol - got

    def _exchange(self, tool, pot, poses, dt):
        """A tool whose rim is below the pot's surface (and inside the pot) fills over its rim at the weir rate until
        its level meets the pot's. Returns how deep the tool's floor is below the pot surface (for the film)."""
        tv, pv = self.vessels[tool], self.vessels[pot]
        tp, tq = poses[tool]
        pp, pq = poses[pot]
        if pv.volume <= 0:
            return 0.0
        P, PR = pv.frame(pp, pq)
        n, dw, _nl, _dl = pv._plane(pp, pq)
        T, TR = tv.frame(tp, tq)
        crest = tv._crest @ TR.T + T
        floor = T
        if not pv.shape.inside(((floor - P) @ PR)[None])[0]:
            return 0.0
        sub = dw - float(floor @ n)
        if sub <= 0:
            return 0.0
        # the tool holds, at most, its inside below the pot's surface plane and below its own lowest crest point
        nl = TR.T @ n
        d_pot = dw - float(n @ T)
        d_rim = float((tv._crest @ nl).min())
        target = tv.shape.volume_below(nl, min(d_pot, d_rim))
        if tv.volume >= target:
            return sub
        d_tool = tv.shape.plane_for(nl, tv.volume) + float(n @ T)
        y = crest @ n
        nxt = np.roll(crest, -1, axis=0)
        seg = nxt - crest
        L = np.linalg.norm(seg - np.outer(seg @ n, n), axis=1)
        yh = (y + np.roll(y, -1)) / 2
        head = dw - np.maximum(yh, d_tool)
        q = weir_flow(head, L) * dt
        mv = min(q, target - tv.volume, pv.volume)
        tv.volume += mv
        pv.volume -= mv
        self.moved[(pot, tool)] = self.moved.get((pot, tool), 0.0) + mv
        return sub

    def step(self, poses: dict, dt: float, hulls=None) -> None:
        """Advance every vessel one control step. `hulls` (optional, from arm_hulls): a stream that runs into a robot
        hull before it lands stops there; that part runs off the hand onto the table below the hit (logged against
        the source as spill, and in `hand_hits`)."""
        self.n += 1
        self.streams = {}
        self.drops = []
        subs = {}
        for tool, pot in self.exchange:
            subs[tool] = (self._exchange(tool, pot, poses, dt), pot)
        for name, v in self.vessels.items():
            pos, q = poses[name]
            out = v.step(pos, q, dt)
            if out <= 0 or v.out is None:
                continue
            parts = self._catch(name, v.out["point"], v.out["vel"], v.out["rate"], poses)
            if hulls:
                main_end = max(parts, key=lambda x: x[1])[2]
                pts = stream_points({"start": np.asarray(v.out["point"]), "vel": np.asarray(v.out["vel"]),
                                     "end": np.asarray(main_end)})
                r = math_jet_radius(v.out["rate"], max(float(np.linalg.norm(v.out["vel"])), 0.3))
                hit = stream_blocked(pts, hulls, r)
                if hit is not None:
                    hp = pts[hit[0]]
                    self.hand_hits.append((self.n, name, hit[1], out))
                    parts = [(None, 1.0, np.array([hp[0], hp[1], self.table_z]))]
            for dest, share, lp in parts:
                vol = out * share
                if dest is None:
                    self._to_table(name, vol, lp)
                else:
                    rest = self._into(name, dest, vol)
                    if rest > 0:  # the receiver overflowed: it spills over its own rim, on its outside
                        self._to_table(dest, rest, poses[dest][0])
            main = max(parts, key=lambda x: x[1])
            self.streams[name] = {"start": np.asarray(v.out["point"]), "vel": np.asarray(v.out["vel"]),
                                  "end": np.asarray(main[2]), "rate": v.out["rate"],
                                  "half_width": v.out["half_width"], "caught": main[0]}
        for tool, film in self.films.items():
            sub, pot = subs.get(tool, (0.0, None))
            prev = self._sub.get(tool, 0.0)
            tv = self.vessels[tool]
            if sub > 0:                                        # how deep it went in during this dip
                self._sub[tool + "/max"] = max(self._sub.get(tool + "/max", 0.0), sub)
            if prev > 0 and sub <= 0 and pot is not None:      # the tool just left the liquid
                speed = float(np.linalg.norm(tv._v)) if tv._v is not None else 0.0
                taken = film.withdraw(self._sub.pop(tool + "/max", prev), speed)
                have = self.vessels[pot].volume
                if taken > have:                               # the film comes out of the pot: never more than it has
                    film.on -= taken - have
                    taken = have
                self.vessels[pot].volume -= taken
            self._sub[tool] = sub
            if sub > 0:
                continue
            drops = film.step(dt)
            if drops:
                pos, q = poses[tool]
                p, R = tv.frame(pos, q)
                hull = tv.shape.corners(32) @ R.T + p
                low = hull[int(np.argmin(hull[:, 2]))]
                vel = tv._v if tv._v is not None else np.zeros(3)
                for _ in range(drops):
                    vol = film.drop
                    for dest, share, lp in self._catch(tool, low, vel, 0.0, poses):  # a drop: no width
                        if dest is None:
                            self._to_table(tool, vol * share, lp, drip=True)
                        else:
                            rest = self._into(tool, dest, vol * share)
                            if rest > 0:
                                self._to_table(dest, rest, poses[dest][0])
                    self.drops.append(tuple(low))


# --------------------------------------------------------------------------------------------------------------------
# scoring and anti-gaming helpers

def in_zone(xy, zone) -> bool:
    """zone: ("circle", cx, cy, r) or ("rect", x0, y0, x1, y1) on the table."""
    x, y = float(xy[0]), float(xy[1])
    if zone[0] == "circle":
        return math.hypot(x - zone[1], y - zone[2]) <= zone[3]
    _, x0, y0, x1, y1 = zone
    return min(x0, x1) <= x <= max(x0, x1) and min(y0, y1) <= y <= max(y0, y1)


def table_liquid(world: World, zone=None, inside: bool = True, drips_only: bool = False) -> float:
    """m^3 on the table (spilled and dripped), optionally only inside (or outside) a zone."""
    tot = 0.0
    for step, src, dest, vol, xy in world.log:
        if dest != "table":
            continue
        if zone is not None and in_zone(xy, zone) != inside:
            continue
        tot += vol
    if drips_only:
        tot = sum(pd.volume for pd in world.puddles if pd.drips and (zone is None or in_zone((pd.x, pd.y), zone) == inside))
    return tot


def drip_count(world: World, zone=None, inside: bool = False) -> int:
    """How many drip marks lie outside (or inside) a zone."""
    return sum(pd.drips for pd in world.puddles if zone is None or in_zone((pd.x, pd.y), zone) == inside)


def tier(value: float, table, higher_is_better: bool = True) -> float:
    """Monotone tiered score: table = [(threshold, score), ...]; the best score whose threshold is met."""
    best = 0.0
    for thr, sc in table:
        ok = value >= thr if higher_is_better else value <= thr
        if ok:
            best = max(best, sc)
    return best


@dataclass
class Monitor:
    """Watches one vessel for the moves a policy could use to game a liquid score, from poses alone:
    tilt (max, time outside a band), hard impacts, free flight (a thrown or dropped vessel: vertical acceleration
    near -g for several steps), and pose jumps."""

    band_deg: float = 10.0
    impact_dv: float = 0.5         # m/s of speed lost or gained in one control step: a knock, a slam or a landing
                                   # (a careful 0.3 m/s move stopped over half a second changes by 0.03 m/s per step)
    airborne_steps: int = 3
    max_tilt: float = 0.0
    outside_band_s: float = 0.0
    impacts: int = 0
    airborne: bool = False
    jumps: int = 0
    _fall: int = 0

    def step(self, vessel: Vessel, quat_, dt: float) -> None:
        t = tilt_deg(quat_)
        self.max_tilt = max(self.max_tilt, t)
        if t > self.band_deg:
            self.outside_band_s += dt
        a = vessel.accel
        if vessel.jumped:
            self.jumps += 1
            self._fall = 0
            return
        if float(np.linalg.norm(a)) * dt > self.impact_dv:
            self.impacts += 1
        self._fall = self._fall + 1 if a[2] < -0.7 * G else 0
        if self._fall >= self.airborne_steps:
            self.airborne = True


def start_satisfies(rule, world: World) -> bool:
    """True if `rule(world)` already holds in the start state (a scoring bug: the task would pay for nothing)."""
    return bool(rule(world))


# --------------------------------------------------------------------------------------------------------------------
# drawing (Isaac Lab; imported lazily, never needed offline). Create at env build, draw the start state on reset.

def stream_points(s: dict, k: int = 24) -> np.ndarray:
    """Center line of a drawn stream: the ballistic arc from its start to where it ends."""
    p0, v0, end = s["start"], s["vel"], s["end"]
    hit = ballistic(p0, v0, end, (0.0, 0.0, 1.0))
    T = hit[0] if hit is not None else 0.0
    ts = np.linspace(0.0, max(T, 1e-3), k)
    return p0[None] + np.outer(ts, v0) - 0.5 * G * np.outer(ts * ts, [0.0, 0.0, 1.0])


def stream_radius(s: dict, minimum: float = 0.0015) -> float:
    """Drawn radius of a stream: a round jet carrying its flow at its speed halfway down (never thinner than
    `minimum`, so it shows on a 320 px camera)."""
    v0 = np.asarray(s["vel"], float)
    drop = max(float(s["start"][2] - s["end"][2]), 0.0)
    v = math.sqrt(float(v0 @ v0) + G * drop)
    return max(math_jet_radius(s["rate"], v), minimum)


def tube(pts, r: float, rings: int = 32, sides: int = 10) -> np.ndarray:
    """(rings*sides, 3) vertices of a tube of radius r along the polyline pts (fixed topology)."""
    pts = np.asarray(pts, float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    u = np.linspace(0.0, max(s[-1], 1e-6), rings)
    c = np.stack([np.interp(u, s, pts[:, k]) for k in range(3)], 1)
    t = np.gradient(c, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
    ang = np.linspace(0.0, 2 * math.pi, sides, endpoint=False)
    out = np.zeros((rings, sides, 3))
    for k in range(rings):
        ref = np.array([0.0, 0.0, 1.0]) if abs(t[k][2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        a = np.cross(t[k], ref)
        a /= max(np.linalg.norm(a), 1e-9)
        b = np.cross(t[k], a)
        out[k] = c[k] + r * (np.outer(np.cos(ang), a) + np.outer(np.sin(ang), b))
    return out.reshape(-1, 3)


def fan(c, ring) -> np.ndarray:
    """Vertices of a surface fan: the center then the ring (fixed topology for SURF_K)."""
    return np.concatenate([np.asarray(c, float)[None], np.asarray(ring, float)])


class Drawer:
    """Draws a World in Isaac Sim: one surface mesh per vessel, one stream tube per vessel, drip marks as discs.
    Everything is created by `create()` (call it at env build) and moved by `draw()`; nothing is created lazily, so
    frame 0 already shows the start state when `draw()` runs in a reset hook."""

    MAX_MARKS = 32

    def __init__(self, names, rgb=(0.66, 0.83, 0.96), opacity: float = 0.6, mark_rgb=None, root="/World/Visuals/liquid"):
        self.names = list(names)
        self.rgb, self.opacity, self.root = rgb, opacity, root
        self.mark_rgb = mark_rgb or rgb
        self.meshes = {}
        self.marks = None
        self.ok = False

    def create(self) -> bool:
        try:
            import carb
            import isaaclab.sim as sim_utils
            import omni.usd
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
            from pxr import Gf, UsdGeom, UsdShade, Vt
        except Exception:  # noqa: BLE001  (offline)
            return False
        try:
            carb.settings.get_settings().set_bool("/rtx/raytracing/fractionalCutoutOpacity", True)
        except Exception:  # noqa: BLE001
            pass
        lin = tuple((c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4) for c in self.rgb)
        stage = omni.usd.get_context().get_stage()
        mats = {}
        for key, op in (("surface", self.opacity), ("stream", 1.0)):
            path = f"{self.root}/mat_{key}"
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=lin, roughness=0.05, opacity=op)
            cfg.func(path, cfg)
            mats[key] = UsdShade.Material(stage.GetPrimAtPath(path))
        self._api = (Gf, Vt)

        def mesh(path, counts, idx, nverts, mat):
            m = UsdGeom.Mesh.Define(stage, path)
            m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
            m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
            m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            m.CreateDoubleSidedAttr(True)
            m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, -1.0)] * nverts))
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
            return m

        k = SURF_K
        fan_idx = []
        for j in range(k):
            fan_idx += [0, 1 + j, 1 + (j + 1) % k]
        rings, sides = 32, 10
        tube_idx = []
        for a in range(rings - 1):
            for j in range(sides):
                j2 = (j + 1) % sides
                tube_idx += [a * sides + j, a * sides + j2, (a + 1) * sides + j2, (a + 1) * sides + j]
        for nm in self.names:
            self.meshes[nm] = (mesh(f"{self.root}/{nm}_surface", [3] * k, fan_idx, k + 1, mats["surface"]),
                               mesh(f"{self.root}/{nm}_stream", [4] * ((rings - 1) * sides), tube_idx, rings * sides,
                                    mats["stream"]))
        mlin = tuple((c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4) for c in self.mark_rgb)
        self.marks = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path=f"{self.root}/marks",
            markers={"mark": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z",
                                                   visual_material=sim_utils.PreviewSurfaceCfg(
                                                       diffuse_color=mlin, roughness=0.05, opacity=1.0))}))
        self.ok = True
        return True

    def _set(self, m, pts):
        Gf, Vt = self._api
        m.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(np.asarray(pts, np.float32)))

    def draw(self, world: World, poses: dict, origin, extra_streams=None) -> None:
        if not self.ok:
            return
        origin = np.asarray(origin, float)
        hidden = origin + np.array([0.0, 0.0, -1.0])
        for nm, (surf, strm) in self.meshes.items():
            v = world.vessels[nm]
            s = v.surface(*poses[nm]) if nm in poses else None
            self._set(surf, fan(s[0], s[2]) + origin if s is not None else np.tile(hidden, (SURF_K + 1, 1)))
            st = (extra_streams or {}).get(nm) or world.streams.get(nm)
            if st is not None:
                self._set(strm, tube(stream_points(st), stream_radius(st)) + origin)
            else:
                self._set(strm, np.tile(hidden, (32 * 10, 1)))
        pds = sorted(world.puddles, key=lambda p: -p.volume)[: self.MAX_MARKS]
        if pds:
            tr = np.array([[p.x, p.y, world.table_z + 0.0006] for p in pds]) + origin
            sc = np.array([[max(p.radius, 0.003), max(p.radius, 0.003), 0.001] for p in pds])
        else:
            tr, sc = hidden[None], np.array([[1e-4, 1e-4, 1e-4]])
        self.marks.visualize(translations=tr, orientations=np.tile([1.0, 0.0, 0.0, 0.0], (len(tr), 1)), scales=sc,
                             marker_indices=np.zeros(len(tr), int))


# --------------------------------------------------------------------------------------------------------------------
# the simulator hook (yam_arena + yamkit). Offline code never calls this.

def compose(pos, quat_, offset=(0.0, 0.0, 0.0), turn=(0.0, 0.0, 0.0, 1.0)):
    """World pose of a vessel frame fixed to a body: the body's pose (pos, quat xyzw) times (offset, turn)."""
    R = rot(quat_)
    return np.asarray(pos, float) + R @ np.asarray(offset, float), quat(R @ rot(turn))


class Sim:
    """Runs a World inside a yam_arena task, once per control step, and draws it.

    `frames`: vessel name -> (object key, offset, turn): the vessel's frame (inside floor center, z up its axis) on a
    task object ("cup") or a yamkit mechanism link ("stand/cup"). `make_world()` builds a fresh World (called at
    every new trial). `on_step(sim, objects)` (optional) runs after the world steps (monitors, tallies).
    After `install()`, scorers read `sim.world` (call `sim.update(objects)` first in success()/score(); it is a no-op
    when the step already ran). The start state is drawn from a reset hook, so frame 0 shows the liquid.
    LIQUIDKIT_FRAME0_TEST=1 also draws a short test stream under every vessel and a test drip mark at reset (a
    check only: shows every visual renders in frame 0); they vanish at the first control step."""

    def __init__(self, frames: dict, make_world, rgb=(0.66, 0.83, 0.96), opacity: float = 0.6, on_step=None,
                 debug_every: int = 0, tag: str = "liquidkit", mark_rgb=None, hands_block: bool = True):
        self.frames, self.make_world, self.on_step = frames, make_world, on_step
        self.hands_block = hands_block   # streams stop on the arms' live collision hulls (yam_arena.analysis)
        self.world = make_world()
        self.drawer = Drawer(list(frames), rgb=rgb, opacity=opacity, mark_rgb=mark_rgb)
        self.debug_every, self.tag = debug_every, tag
        self._episode, self._key = None, None
        self.failed = False

    def poses(self, objects) -> dict:
        out = {}
        for name, (key, off, turn) in self.frames.items():
            s = objects[key]
            out[name] = compose(s.pos, s.quat_xyzw, off, turn)
        return out

    def update(self, objects, env=None) -> None:
        import yamkit as kit
        if self._episode != kit.episode():
            self._episode = kit.episode()
            self.world = self.make_world()
            self._key = None
        n = kit._EPISODE.get("last")
        if n is not None and n == self._key:
            return
        self._key = n
        hulls = None
        if self.hands_block and env is not None:
            try:
                hulls = arm_hulls(env.unwrapped)
            except Exception as e:  # noqa: BLE001  (never break a run; report once)
                if not getattr(self, "_hull_warned", False):
                    import sys
                    sys.stderr.write(f"{self.tag}: arm hulls unavailable ({e!r})\n")
                    self._hull_warned = True
        self.world.step(self.poses(objects), 1.0 / 20, hulls=hulls)
        if self.on_step is not None:
            self.on_step(self, objects)
        if self.debug_every and n is not None and n % self.debug_every == 0:
            w = self.world
            vols = ", ".join(f"{k} {v.volume / ML:.1f}" for k, v in w.vessels.items())
            print(f"{self.tag} step {n}: mL {vols}; spilled {w.spilled / ML:.2f}, dripped {w.dripped / ML:.2f}, "
                  f"streams {sorted(w.streams)}, total {w.total() / ML:.2f}", flush=True)

    # -- drawing -------------------------------------------------------------------------------------------------------
    def _draw(self, objects, origin, extra=None, world=None) -> None:
        if self.failed or not self.drawer.ok:
            return
        try:
            self.drawer.draw(world or self.world, self.poses(objects), origin, extra)
        except Exception as e:  # noqa: BLE001  (visuals never break a run)
            import sys
            sys.stderr.write(f"{self.tag}: visuals off ({e!r})\n")
            self.failed = True

    def _start_objects(self, base) -> dict:
        """Poses straight from the sim at reset (before the first control step), as states for poses()."""
        from types import SimpleNamespace

        np_ = lambda t: getattr(t, "torch", t).detach().cpu().numpy().astype(np.float64)  # noqa: E731
        origin = np_(base.scene.env_origins)[0]
        out = {}
        for key, _off, _turn in self.frames.values():
            if "/" in key:
                mech, link = key.split("/", 1)
                art = base.scene[mech]
                i = list(art.body_names).index(link)
                out[key] = SimpleNamespace(pos=np_(art.data.body_pos_w)[0][i] - origin,
                                           quat_xyzw=np_(art.data.body_quat_w)[0][i])
            else:
                p = np_(base.arena_world.get_pose_w(key))[0]
                out[key] = SimpleNamespace(pos=p[:3] - origin, quat_xyzw=p[3:])
        return out, origin

    def _prime(self, env) -> None:
        import os
        import sys

        base = env.unwrapped
        if getattr(base, f"_{self.tag}_vis", False):
            return
        setattr(base, f"_{self.tag}_vis", True)
        try:
            self.drawer.create()
        except Exception as e:  # noqa: BLE001
            sys.stderr.write(f"{self.tag}: no visuals ({e!r})\n")
            return
        inner = base.reset
        sim = self

        def reset(*a, **k):
            out = inner(*a, **k)
            try:
                objs, origin = sim._start_objects(base)
                world = sim.make_world()
                extra = None
                if os.environ.get("LIQUIDKIT_FRAME0_TEST"):
                    extra = {}
                    for nm, (p, q) in sim.poses(objs).items():
                        top = np.asarray(p) + rot(q) @ np.array([0.0, 0.0, world.vessels[nm].shape.height])
                        extra[nm] = {"start": top + [0.0, 0.10, 0.02], "vel": np.zeros(3),
                                     "end": top + [0.0, 0.10, -0.03], "rate": 5e-6, "half_width": 0.0}
                        world.puddles.append(Puddle(float(top[0]), float(top[1]) + 0.10, 5e-6, 1))
                sim._draw(objs, origin, extra, world)
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"{sim.tag}: start drawing off ({e!r})\n")
            return out

        base.reset = reset

    def _patch(self, scene) -> None:
        import sys

        flag = f"_{self.tag}"
        if getattr(scene, flag, False):
            return
        setattr(scene, flag, True)
        inner = scene.object_states
        sim = self
        keys = {k for k, _o, _t in self.frames.values()}

        def object_states(env, names):
            out = inner(env, names)
            if keys <= set(out):
                sim.update(out, env)
                sim._draw(out, env.unwrapped.scene.env_origins[0].detach().cpu().numpy())
            return out

        scene.object_states = object_states
        make = getattr(scene, "make_env", None)
        if make is not None:
            def make_env(*a, **k):
                env = make(*a, **k)
                try:
                    sim._prime(env)
                except Exception as e:  # noqa: BLE001
                    sys.stderr.write(f"{sim.tag}: visuals not primed ({e!r})\n")
                return env

            scene.make_env = make_env

    def install(self) -> "Sim":
        """Hook yam_arena.scene after yamkit's own patch (import yamkit first)."""
        import sys

        import yamkit as kit
        sim = self

        class _Hook(kit._Hook):
            def exec_module(self, module):
                self._inner.exec_module(module)
                kit._patch(module)
                sim._patch(module)

        if "yam_arena.scene" in sys.modules:
            kit._patch(sys.modules["yam_arena.scene"])
            self._patch(sys.modules["yam_arena.scene"])
        else:
            sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
            sys.meta_path.insert(0, _Hook())
        return self


# --------------------------------------------------------------------------------------------------------------------
# the robot's own collision hulls (loaded at run time from the competition environment; nothing robot-derived ships)

ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")


def arm_hulls(base, robots=("left_robot", "right_robot"), links=ARM_LINKS) -> list:
    """World hulls [(link, Hull)] of the arms at their live poses, env origin removed (yam_arena.analysis)."""
    from yam_arena.analysis import world_hulls

    np_ = lambda t: getattr(t, "torch", t).detach().cpu().numpy().astype(np.float64)  # noqa: E731
    origin = np_(base.scene.env_origins)[0]
    out = []
    for robot in robots:
        try:
            art = base.scene[robot]
        except KeyError:
            continue
        names = list(art.body_names)
        pos, qs = np_(art.data.body_pos_w)[0], np_(art.data.body_quat_w)[0]
        poses = {}
        for ln in links:
            if ln in names:
                i = names.index(ln)
                T = np.eye(4)
                T[:3, :3] = rot(qs[i])
                T[:3, 3] = pos[i] - origin
                poses[ln] = T
        out += [(f"{robot}/{n}", h) for n, h in world_hulls(poses, list(poses))]
    return out


def hull_box_gap(hulls, center, R, size) -> float:
    """Smallest signed distance (m) from any hull vertex to an oriented box (negative = inside it)."""
    from yam_arena.analysis import box_signed_distance

    if not hulls:
        return float("inf")
    pts = np.concatenate([h.vertices for _n, h in hulls])
    return float(box_signed_distance(pts, np.asarray(center, float), np.asarray(R, float), np.asarray(size, float)).min())


def _inside_hull(points, hull, margin: float = 0.0) -> np.ndarray:
    """Which points are inside a convex hull (planes n . x <= d), grown by margin."""
    P = np.atleast_2d(points)
    n, d = hull.planes[:, :3], hull.planes[:, 3]
    return np.all(P @ n.T <= d + margin, axis=1)


def stream_blocked(points, hulls, radius: float = 0.0):
    """The first point along a stream's center line that runs into a robot hull (grown by the jet's radius), as
    (index, link name), or None. A task can then end the stream there (the liquid runs off the hand onto the table)."""
    P = np.asarray(points, float)
    first = None
    for name, h in hulls:
        inside = _inside_hull(P, h, radius)
        if inside.any():
            k = int(np.argmax(inside))
            if first is None or k < first[0]:
                first = (k, name)
    return first
