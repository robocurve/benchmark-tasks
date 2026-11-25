"""squeezekit: a soft squeeze bottle that dispenses a thick sauce by how hard its walls are pressed, for a yam_arena
task file (works with yamkit and mechkit).

The simulator has no fluids, so the sauce is a flow model driven by physics outputs, not particles:

- The bottle is a yamkit `Mechanism` with mass (a loose tool). Its wall panels (two or four) are prismatic links on
  springs (`Wall`, a mechkit-style law: stiffness `k` back to rest, plus the bottle's inside pressure pushing them
  out). A hand squeezes the bottle only by pushing those panels in; a grip on the rigid core or ends squeezes nothing.
  The panels' inward deflection d (m) is read from the joints every control step: it is what the fingers did to the
  bottle, not a gripper command.
- Squeezing displaces `area` mL per cm of each panel's deflection. The air inside is compressed isothermally:
  gauge pressure p = P_ATM * (air / V_air - 1), with V_air = capacity - sauce left - displaced volume, and `air` the
  amount of air inside (mL at atmospheric pressure).
- The sauce is a yield-stress (Bingham-like) paste. With the tip pointing within `down_deg` of straight down and the
  sauce settled at the tip (`settle` s tip-down), it flows at Q = flow * (p + head - p_yield) mL/s when that is
  positive, else not at all; head is the sauce column above the tip (rho * g * h * cos(tilt)). So a bottle standing
  tip-down does not drip, a light holding squeeze does not flow, and a firm squeeze does. Each mL that leaves lets
  the air expand, so a constant squeeze gives a flow that dies away (time constant about 1.5 s with the defaults):
  an even line needs a squeeze that keeps closing at the rate the sauce leaves. Releasing makes p negative: air is
  drawn back in through the tip (`air_in` mL/s per kPa) and the flow stops.
- With the tip up (or not yet settled), squeezing only pushes air out of the tip: p stays 0 and nothing flows.
- The stream falls plumb from the tip. It lands on the highest `Face` (a fixed horizontal rectangle, e.g. a roll's
  cut face) under the tip, in `cell` squares; else in a `Catch` (e.g. a holder's drip well); else it is `off`
  (the paper, the board, the table). A cell holds at most `rest` cm of sauce (a thick sauce's bead height); more
  slumps into its four neighbors, and slump that crosses a face's edge runs off it (`runoff`, also off-target).
  So a nozzle that dwells builds a puddle that spreads, and a nozzle carried on while still squeezed dribbles.
- Volume is conserved: dispensed = on the faces + caught + off + runoff.

Units: mL, cm for heights inside the model, kPa, s; positions in m (world frame, as the scorer gets them).

Usage in task.py:

    import squeezekit as sq
    B = sq.Bottle("bottle", tip=(0, 0, -0.112))
    ROLL = sq.Face("roll", center=(0.28, -0.03, TOP + 0.03), size=(0.072, 0.150))
    WELL = sq.Catch("holder", center=(0.22, -0.20), radius=0.02, z=TOP + 0.04)
    sauce = sq.Sauce(B, [ROLL], [WELL])          # one trial
    sauce.step(objects["bottle"].pos, objects["bottle"].quat_xyzw, [d_left, d_right])   # once per control step
    sauce.per_cm("roll", x0, x1, y0, y1)        # mL per 1 cm slice along y inside a band

Panel links get `law=sq.Wall("bottle", sign)`; the law reads the bottle's live pressure through `register`.
Constants a task states come with the measurement that sets them.
"""
from __future__ import annotations

import math
import sys
import warnings
from dataclasses import dataclass

import numpy as np

P_ATM = 101.325  # kPa
RHO_G = 1.05 * 9.81 / 100.0  # kPa per cm of a 1.05 g/mL sauce column


def rotm(q) -> np.ndarray:
    """Rotation matrix from a quaternion (x, y, z, w)."""
    x, y, z, w = (float(c) for c in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat(r: np.ndarray) -> np.ndarray:
    """Quaternion (x, y, z, w) from a rotation matrix."""
    r = np.asarray(r, float)
    t = float(np.trace(r))
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
        q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
    q = np.array(q)
    return q if q[3] >= 0 else -q


# -- configuration -----------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Bottle:
    """A squeeze bottle `name` (a yamkit Mechanism). `tip` is the tip opening in the bottle's frame (m) and `axis` the
    unit vector from the body toward the tip in that frame. `panels` are (link name, sign) pairs: a panel's inward
    deflection is sign * its joint position. `capacity` (mL, brim full), `fill` (mL of sauce at the start), `area`
    (mL displaced per cm of one panel's deflection), `cross` (cm^2, the body's inside cross-section) and `neck` (cm of
    tip, cap and shoulder below the body, holding `neck_ml`) set the air and the sauce column. `p_yield` (kPa) is
    the pressure the sauce must overcome to move in the tip, `flow` (mL/s per kPa past it) the tip's conductance,
    `air_in` (mL/s per kPa of vacuum) how fast air is drawn back in, `down_deg` how far from straight down the tip may
    point and still feed sauce, `settle` (s) how long sauce takes to run down to the tip after the bottle is turned
    tip-down (a bottle that starts tip-down starts settled when `start_settled`)."""

    name: str
    tip: tuple
    axis: tuple = (0.0, 0.0, -1.0)
    panels: tuple = (("panel_l", -1.0), ("panel_r", 1.0))
    capacity: float = 125.0
    fill: float = 60.0
    area: float = 5.0
    cross: float = 10.2
    neck: float = 4.6
    neck_ml: float = 5.0
    p_yield: float = 3.5
    flow: float = 0.4
    air_in: float = 5.0
    down_deg: float = 60.0
    settle: float = 1.0
    start_settled: bool = True


@dataclass(frozen=True)
class Face:
    """A fixed horizontal rectangle where sauce can land and stay (world frame): `center` is the middle of its top
    surface (m), `size` its (x, y) extent (m), `cell` the square cell (m) and `rest` the most sauce a cell holds (cm
    high) before it slumps into its neighbors."""

    name: str
    center: tuple
    size: tuple
    cell: float = 0.002
    rest: float = 0.30


@dataclass(frozen=True)
class Catch:
    """A fixed round catch (a drip well): sauce falling within `radius` of `center` (x, y) from above `z` lands in it."""

    name: str
    center: tuple
    radius: float
    z: float


@dataclass(frozen=True)
class Wall:
    """mechkit law for a wall panel of bottle `bottle`: a spring of `k` N/m back to rest (the LDPE wall), plus the
    bottle's inside pressure on `area_m2` pushing it outward. `sign` as in Bottle.panels."""

    bottle: str
    sign: float
    k: float = 600.0
    area_m2: float = 5e-4
    damp: float = 2.0

    def drive(self, key, q, qd, J, mem):
        p = pressure(self.bottle)
        mem["deflection"] = self.sign * q
        return 0.0, self.k, -self.sign * p * 1000.0 * self.area_m2, self.damp


_LIVE: dict = {}


def register(sauce: "Sauce") -> None:
    """Make `sauce` the live state of its bottle (the Wall laws read its pressure)."""
    _LIVE[sauce.b.name] = sauce


def pressure(bottle: str) -> float:
    """The live gauge pressure (kPa) inside `bottle` (0 before any trial is registered)."""
    s = _LIVE.get(bottle)
    return 0.0 if s is None else max(0.0, s.p)


# -- a loose articulated bottle ------------------------------------------------------------------------------------------

_FREE: dict = {}


def free_body(mech, com=None):
    """Mark a loose (massed) yamkit Mechanism so that its base body carries the articulation root, with its center of
    mass pinned at `com` (base frame, m) if given. Without this PhysX roots a floating articulation at its most
    central body and the joint drives misbehave (seen in the simulator with loose tools). Returns `mech`."""
    import yamkit as kit

    _FREE[mech.name] = com
    if not getattr(kit, "_squeezekit_root", False):
        inner = kit._mech_usda

        def _mech_usda(m):
            text = inner(m)
            return _root_on_base(text, _FREE[m.name]) if m.name in _FREE and m.mass is not None else text

        kit._mech_usda = _mech_usda
        kit._squeezekit_root = True
    return mech


def _root_on_base(text: str, com) -> str:
    """Move yamkit's articulation root schemas (and the root's physxArticulation settings) from the Root Xform onto the
    base body, and pin the base's center of mass."""
    root = 'def Xform "Root" (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)\n{\n'
    base = ('    def Xform "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
            '"PhysxRigidBodyAPI"]\n    )\n    {\n')
    if root not in text or base not in text:
        raise RuntimeError("squeezekit: unexpected yamkit USD layout (articulation root)")
    lines = text.replace(root, 'def Xform "Root"\n{\n', 1).splitlines(True)
    settings = [ln for ln in lines if ln.startswith("    ") and not ln.startswith("     ") and "physxArticulation:" in ln]
    text = "".join(ln for ln in lines if ln not in settings)
    text = text.replace(base, base.replace('"PhysxRigidBodyAPI"]', '"PhysxRigidBodyAPI", "PhysicsArticulationRootAPI", '
                                           '"PhysxArticulationAPI"]') + "".join("    " + ln for ln in settings), 1)
    if com is not None:
        i = text.index('    def Xform "base" (')
        eol = text.index("\n", text.index("float physics:mass", i)) + 1
        text = text[:eol] + "        point3f physics:centerOfMass = (%.6g, %.6g, %.6g)\n" % tuple(com) + text[eol:]
    return text


# -- the model ---------------------------------------------------------------------------------------------------------


class _Grid:
    def __init__(self, f: Face):
        self.f = f
        self.nx = int(round(f.size[0] / f.cell))
        self.ny = int(round(f.size[1] / f.cell))
        self.v = np.zeros((self.nx, self.ny))  # mL per cell
        self.cap = f.rest * (f.cell * 100.0) ** 2  # mL a cell holds before it slumps
        self.x0 = f.center[0] - f.size[0] / 2
        self.y0 = f.center[1] - f.size[1] / 2

    def inside(self, x, y) -> bool:
        return 0.0 <= x - self.x0 < self.f.size[0] and 0.0 <= y - self.y0 < self.f.size[1]

    def add(self, x, y, ml) -> None:
        i = min(self.nx - 1, max(0, int((x - self.x0) / self.f.cell)))
        j = min(self.ny - 1, max(0, int((y - self.y0) / self.f.cell)))
        self.v[i, j] += ml

    def relax(self, iters: int = 300) -> float:
        """Slump every cell over `cap` into its four neighbors; returns the mL that ran off the edges."""
        off = 0.0
        for _ in range(iters):
            ex = np.clip(self.v - self.cap, 0.0, None)
            if float(ex.max()) < 1e-7:
                break
            self.v -= ex
            q = ex / 4.0
            self.v[1:, :] += q[:-1, :]
            self.v[:-1, :] += q[1:, :]
            self.v[:, 1:] += q[:, :-1]
            self.v[:, :-1] += q[:, 1:]
            off += float(q[0, :].sum() + q[-1, :].sum() + q[:, 0].sum() + q[:, -1].sum())
        return off

    def centers(self):
        xs = self.x0 + (np.arange(self.nx) + 0.5) * self.f.cell
        ys = self.y0 + (np.arange(self.ny) + 0.5) * self.f.cell
        return xs, ys


class Sauce:
    """One trial's sauce: in the bottle, its air, and where it landed. Advance it with step() once per control step."""

    def __init__(self, bottle: Bottle, faces=(), catches=(), live: bool = True):
        self.b = bottle
        self.grids = {f.name: _Grid(f) for f in faces}
        self.catches = tuple(catches)
        self.sauce = float(bottle.fill)  # mL left in the bottle
        self.air = float(bottle.capacity - bottle.fill)  # mL of air at atmospheric pressure
        self.p = 0.0  # gauge pressure, kPa
        self.t_down = bottle.settle if bottle.start_settled else 0.0
        self.dispensed = 0.0
        self.caught = {c.name: 0.0 for c in self.catches}
        self.off = 0.0  # landed off every face and catch
        self.runoff = 0.0  # slumped off a face's edge
        self.q_now = 0.0  # mL/s at the last step
        self.squeeze_ml = 0.0
        self.last_tip = None
        self.flow_steps = 0
        if live:
            register(self)

    # -- state ------------------------------------------------------------------------------------------------------
    def displaced(self, deflections) -> float:
        """mL displaced by the panels at these inward deflections (m); a panel pulled outward displaces nothing."""
        return sum(self.b.area * max(0.0, float(d)) * 100.0 for d in deflections)

    def v_air(self, squeeze_ml: float) -> float:
        return max(1e-3, self.b.capacity - self.sauce - squeeze_ml)

    def head(self, cos_tilt: float) -> float:
        """kPa from the sauce column above the tip, the bottle tip-down at this tilt."""
        h = self.b.neck * min(1.0, self.sauce / self.b.neck_ml) + max(0.0, self.sauce - self.b.neck_ml) / self.b.cross
        return RHO_G * h * max(0.0, cos_tilt)

    def tip_world(self, pos, quat_xyzw) -> np.ndarray:
        return np.asarray(pos, float) + rotm(quat_xyzw) @ np.asarray(self.b.tip, float)

    def cos_down(self, quat_xyzw) -> float:
        """Cosine of the angle between the tip's direction and straight down."""
        return float(-(rotm(quat_xyzw) @ np.asarray(self.b.axis, float))[2])

    # -- one control step --------------------------------------------------------------------------------------------
    def step(self, pos, quat_xyzw, deflections, dt: float = 0.05, sub: int = 10) -> float:
        """Advance by dt with the bottle at pos/quat and its panels at these inward deflections (m). Returns the mL
        dispensed in this step."""
        tip1 = self.tip_world(pos, quat_xyzw)
        tip0 = tip1 if self.last_tip is None else self.last_tip
        self.last_tip = tip1
        c = self.cos_down(quat_xyzw)
        down = c >= math.cos(math.radians(self.b.down_deg))
        self.t_down = self.t_down + dt if down else 0.0
        settled = down and self.t_down >= self.b.settle - 1e-9
        sq1 = self.displaced(deflections)
        sq0 = self.squeeze_ml
        self.squeeze_ml = sq1
        h = dt / sub
        out = 0.0
        landed = {}
        for k in range(1, sub + 1):
            sq = sq0 + (sq1 - sq0) * k / sub
            va = self.v_air(sq)
            if not settled:  # air at the tip: a squeeze only vents air; a release draws it back
                self.air = va
                self.p = 0.0
                continue
            self.p = P_ATM * (self.air / va - 1.0)
            drive = self.p + self.head(c) - self.b.p_yield
            if drive > 0.0 and self.sauce > 0.0:
                ml = min(self.sauce, self.b.flow * drive * h)
                self.sauce -= ml
                out += ml
                tip = tip0 + (tip1 - tip0) * k / sub
                key = (round(float(tip[0]), 4), round(float(tip[1]), 4), float(tip[2]))
                landed[key] = landed.get(key, 0.0) + ml
            elif self.p < 0.0:  # vacuum: air comes back in through the tip
                self.air = min(va, self.air + self.b.air_in * (-self.p) * h)
            self.p = P_ATM * (self.air / self.v_air(sq) - 1.0)
        for (x, y, z), ml in landed.items():
            self._land(x, y, z, ml)
        for g in self.grids.values():
            self.runoff += g.relax()
        self.dispensed += out
        self.q_now = out / dt
        self.flow_steps += out > 0.0
        return out

    def _land(self, x, y, z, ml) -> None:
        best = None
        for g in self.grids.values():
            if g.inside(x, y) and g.f.center[2] <= z + 0.002 and (best is None or g.f.center[2] > best.f.center[2]):
                best = g
        if best is not None:
            best.add(x, y, ml)
            return
        for c in self.catches:
            if math.hypot(x - c.center[0], y - c.center[1]) <= c.radius and z >= c.z - 0.03:
                self.caught[c.name] += ml
                return
        self.off += ml

    # -- readouts -----------------------------------------------------------------------------------------------------
    def on(self, face: str) -> float:
        """mL lying on a face."""
        return float(self.grids[face].v.sum())

    def off_target(self) -> float:
        """mL that landed off every face and catch or ran off a face's edge."""
        return self.off + self.runoff

    def in_rect(self, face: str, x0, x1, y0, y1) -> float:
        """mL on `face` in cells whose centers lie in [x0, x1) x [y0, y1) (world m)."""
        g = self.grids[face]
        xs, ys = g.centers()
        mi = (xs >= x0 - 1e-9) & (xs < x1 - 1e-9)
        mj = (ys >= y0 - 1e-9) & (ys < y1 - 1e-9)
        return float(g.v[np.ix_(mi, mj)].sum())

    def per_cm(self, face: str, x0, x1, y0, y1, step: float = 0.01) -> np.ndarray:
        """mL in each `step`-long slice along y from y0 to y1, inside x0..x1 (a band along y): mL per cm."""
        n = int(round((y1 - y0) / step))
        return np.array([self.in_rect(face, x0, x1, y0 + i * step, y0 + (i + 1) * step) for i in range(n)])

    def covered(self, face: str, x0, x1, y0, y1, min_h: float = 0.05) -> float:
        """cm^2 of `face` covered by sauce at least `min_h` cm deep, in cells whose centers lie in [x0, x1) x [y0, y1)
        (world m): what a photo through a grid shows."""
        g = self.grids[face]
        xs, ys = g.centers()
        mi = (xs >= x0 - 1e-9) & (xs < x1 - 1e-9)
        mj = (ys >= y0 - 1e-9) & (ys < y1 - 1e-9)
        h = g.v[np.ix_(mi, mj)] / (g.f.cell * 100.0) ** 2
        return float(np.count_nonzero(h >= min_h)) * (g.f.cell * 100.0) ** 2

    def max_run(self, face: str, min_h: float = 0.05) -> float:
        """The widest unbroken stretch of sauce across `face` along x (cm), over all its rows: a line's width, or a
        puddle's or two merged lines' width."""
        g = self.grids[face]
        cov = g.v / (g.f.cell * 100.0) ** 2 >= min_h
        best = 0
        run = np.zeros(g.ny, int)
        for i in range(g.nx):
            run = np.where(cov[i], run + 1, 0)
            best = max(best, int(run.max()))
        return best * g.f.cell * 100.0

    def max_height(self, face: str) -> float:
        """The highest cell on a face, cm."""
        g = self.grids[face]
        return float(g.v.max()) / (g.f.cell * 100.0) ** 2

    def balance(self) -> float:
        """dispensed - everything accounted for (0 up to rounding)."""
        return self.dispensed - (sum(g.v.sum() for g in self.grids.values()) + sum(self.caught.values())
                                 + self.off + self.runoff)


# -- what the cameras see --------------------------------------------------------------------------------------------


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


class Tiles:
    """The sauce on each face drawn as boxes standing on it (height = the cell's sauce), and the falling stream as a
    thin column, with Isaac Lab visualization markers. Call draw(sauce, origin) after each step (origin: the env's
    origin in world). Off outside the simulator; turns itself off with one warning on any error."""

    def __init__(self, color=(0.86, 0.42, 0.16), min_ml: float = 2e-4, max_tiles: int = 4000,
                 path: str = "/Visuals/squeezekit"):
        self.color, self.min_ml, self.max = color, min_ml, max_tiles
        self.path, self.markers, self.off, self.shown = path, None, False, None

    def draw(self, sauce: Sauce, origin=(0.0, 0.0, 0.0), tip=None) -> None:
        if self.off:
            return
        tr, sc = [], []
        for g in sauce.grids.values():
            xs, ys = g.centers()
            ii, jj = np.nonzero(g.v > self.min_ml)
            for i, j in zip(ii[: self.max], jj[: self.max]):
                hgt = float(g.v[i, j]) / (g.f.cell * 100.0) ** 2 / 100.0  # m
                tr.append((xs[i], ys[j], g.f.center[2] + hgt / 2))
                sc.append((g.f.cell, g.f.cell, max(hgt, 2e-4)))
        if tip is not None and sauce.q_now > 0.0:
            t = np.asarray(tip, float)
            z0 = max((g.f.center[2] for g in sauce.grids.values() if g.inside(t[0], t[1])), default=0.0)
            if t[2] > z0:
                tr.append((t[0], t[1], (t[2] + z0) / 2))
                sc.append((0.0025, 0.0025, t[2] - z0))
        sig = (len(tr), round(float(np.sum(sc)) if sc else 0.0, 7))
        if sig == self.shown:
            return
        try:
            if self.markers is None:
                import isaaclab.sim as sim_utils
                from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

                proto = sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(_linear(x) for x in self.color), roughness=0.5))
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path=self.path,
                                                                            markers={"sauce": proto}))
            self.shown = sig
            if not tr:
                self.markers.set_visibility(False)
                return
            self.markers.set_visibility(True)
            self.markers.visualize(translations=(np.array(tr, np.float32) + np.asarray(origin, np.float32)),
                                   orientations=np.tile(np.array([0, 0, 0, 1], np.float32), (len(tr), 1)),
                                   scales=np.array(sc, np.float32), marker_indices=np.zeros(len(tr), int))
        except ImportError:
            self.off = True
        except Exception as e:  # noqa: BLE001 (cosmetic only)
            self.off = True
            warnings.warn(f"squeezekit tiles off ({type(e).__name__}: {e}); scoring is unaffected", RuntimeWarning)
            print(f"squeezekit: tiles off ({e!r})", file=sys.stderr, flush=True)
