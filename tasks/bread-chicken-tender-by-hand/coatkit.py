"""coatkit: a dry coating (seasoned flour, breadcrumbs) pressed onto a wet piece of food in a bed of breading, and
wiped off where fingers grip the piece afterwards, for a yam_arena task file. Plain Python and NumPy at import;
Isaac Lab is imported only by the simulator hooks. Offline checks: `python3 test_coatkit.py`.

The simulator has no flour, so the coat is a scored model on the piece's surface. Its parts:

- `Slab`: the piece (a chicken tender) as a box `size` = (length, width, thickness) in its own frame (x along the
  length, z through the thickness), centered on the body's origin. Its six faces are cut into cells about `cell`
  on a side; each cell has a center, an outward normal and an area. Each cell is bare or coated (`coated`, bool).
- `Bed`: the breading in a pan, world-fixed: a rectangle of the pan's inside (`center` x, y; `size` x, y) on a floor at
  `floor_z`, filled `depth` deep. The flour has no collider: the piece lies on the pan's floor with its lower part in
  the flour, as a pressed tender sits in a real pan of breading. A cell is in the flour when its center is over the
  rectangle (shrunk by `margin`) and no higher than `floor_z + depth`.
- `Law`: the numbers below. One `Coat` (the state of one trial) takes contact forces every physics step.

Physical model (each number is a `Law` field; the task states its reason):

1. Pressed, not dropped. Breading sticks to a wet piece only where it is pressed on; loose breading falls off when the
   piece is lifted or tapped. A press is read from contact forces, not from poses: the pan pushes up on the piece with
   at least `press_n` (2.0 N, about four times a 50 g tender's weight) while a gripper pushes down on it, for
   `press_steps` physics steps in a row (6 steps, 25 ms at 240 Hz), so a piece dropped into the pan (an impact of a
   step or two) or lying under its own weight sets nothing.
2. Where it is pressed. A press sets the in-flour cells within `press_half` (3 cm) along the piece's length of the
   point the gripper pushes (its tool point, projected onto the piece's long axis). A long piece is pressed at more
   than one spot, as a cook presses along a tender.
3. Wiped where it is gripped. A gripper grips the piece when both of its finger links push on it with at least
   `wipe_n` (0.5 N) each, from opposite sides (the two forces point against each other). A fist pushing down on the
   top (both fingers pushing the same way) is a press, not a grip. While a gripper grips, every coated cell under its
   finger pads' inner faces goes bare: a pinch that carries the piece's weight by friction shears the coat off under
   the pads, which is why cooks handle a breaded piece as little as possible. A gripped cell cannot be coated in that
   same step. The pads' shape is guardkit's fit of the YAM's finger collision meshes (pad and tip blocks).
4. Coated cells stay coated until wiped; a wiped spot can be coated again by pressing it into the flour again (a cook
   patches a bare spot the same way).

State the scorer reads:
    C = Coat(slab, bed, law)       one trial
    C.physics(pos, quat, bed_fz, pressers, grips)   one physics step; `bed_fz` is the upward force (N) of the
                                    breading pan on the piece; `pressers` the tool points of grippers pushing it
                                    down; `grips` = [Grip(tcp, rot, opening_m)] for grippers squeezing it
    C.coverage()                   coated area / total area (0..1)
    C.face_coverage()              per face {"top": .., "bottom": .., "+y": .., "-y": .., "+x": .., "-x": ..}
    C.presses, C.grips_seen        how many presses and grips were registered (for logs)

Simplifications (say them in the task's fidelity): the coat is a per-cell flag, not a layer of particles; the flour
has no collider and does not move or run out; pressure is read as one force over the whole piece (where it acts comes
from the presser's position); the piece is a box, rigid or cut into rigid parts on hinges (`Coat`'s `cuts`); unpressed
breading counts as bare.
"""
from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys
import warnings
from dataclasses import dataclass

import numpy as np

FACES = ("+x", "-x", "+y", "-y", "top", "bottom")


def rotm(q) -> np.ndarray:
    """xyzw quaternion to a rotation matrix."""
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat(R) -> np.ndarray:
    """Rotation matrix to an xyzw quaternion."""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (R[j, i] + R[i, j]) / s
        q[k] = (R[k, i] + R[i, k]) / s
    q = np.asarray(q, float)
    return q / np.linalg.norm(q)


@dataclass(frozen=True)
class Slab:
    """The piece: a box (length x, width y, thickness z, m) centered on its body's origin, cut into cells."""

    size: tuple
    cell: float = 0.0025

    def cells(self):
        """(centers (n, 3), normals (n, 3), areas (n,), face index (n,)) in the piece's frame."""
        L = np.asarray(self.size, float) / 2
        cs, ns, ar, fi = [], [], [], []
        for f, name in enumerate(FACES):
            axis = {"+x": 0, "-x": 0, "+y": 1, "-y": 1, "top": 2, "bottom": 2}[name]
            sign = -1.0 if name in ("-x", "-y", "bottom") else 1.0
            u, v = [k for k in range(3) if k != axis]
            nu = max(1, round(2 * L[u] / self.cell))
            nv = max(1, round(2 * L[v] / self.cell))
            gu = (np.arange(nu) + 0.5) / nu * 2 * L[u] - L[u]
            gv = (np.arange(nv) + 0.5) / nv * 2 * L[v] - L[v]
            for a in gu:
                for b in gv:
                    p = np.zeros(3)
                    p[axis], p[u], p[v] = sign * L[axis], a, b
                    n = np.zeros(3)
                    n[axis] = sign
                    cs.append(p)
                    ns.append(n)
                    ar.append(4 * L[u] * L[v] / (nu * nv))
                    fi.append(f)
        return np.array(cs), np.array(ns), np.array(ar), np.array(fi)


@dataclass(frozen=True)
class Bed:
    """The breading: the inside of a pan (world x, y center and size, m) on a floor at `floor_z`, `depth` deep."""

    center: tuple
    size: tuple
    floor_z: float
    depth: float
    margin: float = 0.003

    def holds(self, pts) -> np.ndarray:
        p = np.atleast_2d(pts)
        h = np.asarray(self.size, float) / 2 - self.margin
        c = np.asarray(self.center, float)
        return ((np.abs(p[:, 0] - c[0]) <= h[0]) & (np.abs(p[:, 1] - c[1]) <= h[1])
                & (p[:, 2] <= self.floor_z + self.depth) & (p[:, 2] >= self.floor_z - 0.01))


@dataclass(frozen=True)
class Law:
    """Numbers of the coating model (N, m, physics steps)."""

    press_n: float = 2.0  # upward force of the pan on the piece while a gripper pushes it down ...
    press_steps: int = 6  # ... for this many physics steps in a row (25 ms at 240 Hz): a press, not an impact
    presser_n: float = 0.3  # a gripper pushing down on the piece with at least this much is pressing it
    press_half: float = 0.030  # a press sets in-flour cells within this distance along the length of the presser
    wipe_n: float = 0.5  # each finger link pushing on the piece with this much, from opposite sides, is a grip
    pad_reach: float = 0.003  # a cell this close to a pad's inner face (and over the pad) is under the pad


@dataclass(frozen=True)
class Grip:
    """A gripper squeezing the piece: its tool point (world), rotation (columns: closing axis, finger width, approach)
    and each finger's inner-face distance from the tool point (m: half the opening between the pads)."""

    tcp: tuple
    rot: np.ndarray
    half_open: float


# The YAM finger's inner contact faces in the gripper's tool frame (guardkit's fit of the collision meshes): for each
# finger, the pad block (4.0 cm wide, z -2.1 to -0.3 cm) and the narrower tip block (2.64 cm wide, z -0.3 to +1.28 cm;
# z is the approach, toward the fingertips). Each is (half width along y, z from, z to).
PAD_FACES = ((0.020, -0.021, -0.003), (0.0132, -0.003, 0.0128))


def under_pads(points, normals, grip: Grip, reach: float) -> np.ndarray:
    """Cells (world points and outward normals) under either finger's pad or tip inner face."""
    q = (np.atleast_2d(points) - np.asarray(grip.tcp, float)) @ np.asarray(grip.rot, float)  # gripper frame
    nq = np.atleast_2d(normals) @ np.asarray(grip.rot, float)
    hit = np.zeros(len(q), bool)
    for s in (-1.0, 1.0):
        face_x = s * grip.half_open
        facing = nq[:, 0] * s > 0.5  # the cell's normal points out toward this finger
        near = np.abs(q[:, 0] - face_x) <= reach + 0.004  # the pads squeeze a few mm into a soft grip
        for hw, z0, z1 in PAD_FACES:
            hit |= facing & near & (np.abs(q[:, 1]) <= hw) & (q[:, 2] >= z0) & (q[:, 2] <= z1)
    return hit


class Coat:
    """One trial's coat on one piece. A soft piece is cut across its length at `cuts` (x in the piece's frame) into
    parts on hinges, lowest x first; `origins` are the parts' own frame origins in the piece's frame (each part's frame
    is the piece's, moved there). Then `pos` and `q` are one per part, and each cell moves with its part."""

    def __init__(self, slab: Slab, bed: Bed, law: Law = Law(), cuts=(), origins=None):
        self.slab, self.bed, self.law = slab, bed, law
        self.c, self.n, self.area, self.face = slab.cells()
        self.cuts = tuple(float(x) for x in sorted(cuts))
        self.part = np.searchsorted(np.asarray(self.cuts, float), self.c[:, 0]) if self.cuts else \
            np.zeros(len(self.c), int)
        k = len(self.cuts) + 1
        self.origins = np.zeros((k, 3)) if origins is None else np.asarray(origins, float).reshape(k, 3)
        self.coated = np.zeros(len(self.c), bool)
        self.press_run = 0
        self.presses = 0  # physics steps that set breading
        self.grips_seen = 0  # physics steps with a grip on the piece
        self.wiped_total = 0.0  # m^2 of coat wiped off in all
        self.steps = 0

    def to_world(self, pts, nrm, pos, q):
        """Points and normals given in the piece's (straight) frame, each moved with its cell's part: (points, normals,
        rotation per cell (n, 3, 3))."""
        P = np.asarray(pos, float).reshape(-1, 3)
        Q = np.asarray(q, float).reshape(-1, 4)
        Rs = np.array([rotm(x) for x in Q])
        if len(P) == 1:  # one pose: the piece lying straight
            Rc = np.broadcast_to(Rs[0], (len(pts), 3, 3))
            w = np.asarray(pts, float) @ Rs[0].T + P[0]
        else:
            Rc = Rs[self.part]
            w = np.einsum("nij,nj->ni", Rc, np.asarray(pts, float) - self.origins[self.part]) + P[self.part]
        return w, np.einsum("nij,nj->ni", Rc, np.asarray(nrm, float)), Rc

    def world(self, pos, q):
        """The cells' world centers and normals, and the rotation (the piece's, or each cell's part's)."""
        if len(self.cuts) == 0:
            R = rotm(q)
            return self.c @ R.T + np.asarray(pos, float), self.n @ R.T, R
        return self.to_world(self.c, self.n, pos, q)

    def in_flour(self, pos, q) -> np.ndarray:
        p, _, _ = self.world(pos, q)
        return self.bed.holds(p)

    def physics(self, pos, q, bed_fz: float, pressers=(), grips=()) -> None:
        """One physics step. `bed_fz`: the breading pan's upward force on the piece (N). `pressers`: tool points of the
        grippers pushing it down (world). `grips`: Grip records of grippers squeezing it."""
        self.steps += 1
        p, n, R = self.world(pos, q)
        gripped = np.zeros(len(p), bool)
        for g in grips:
            gripped |= under_pads(p, n, g, self.law.pad_reach)
        if grips:
            self.grips_seen += 1
            lost = self.coated & gripped
            self.wiped_total += float(self.area[lost].sum())
            self.coated &= ~gripped
        if bed_fz >= self.law.press_n and len(pressers):
            self.press_run += 1
        else:
            self.press_run = 0
        if self.press_run >= self.law.press_steps:
            flour = self.bed.holds(p)
            ax = R[:, 0] if R.ndim == 2 else R[:, :, 0]  # the long axis (of the piece, or of each cell's part)
            near = np.zeros(len(p), bool)
            for t in pressers:
                u = ((p - np.asarray(t, float)) * ax).sum(axis=1)  # along the length, from the presser
                near |= np.abs(u) <= self.law.press_half
            new = flour & near & ~gripped & ~self.coated
            if new.any() or (flour & near).any():
                self.presses += 1
            self.coated |= flour & near & ~gripped

    def coverage(self) -> float:
        return float(self.area[self.coated].sum() / self.area.sum())

    def face_coverage(self) -> dict:
        return {name: float(self.area[(self.face == f) & self.coated].sum() / self.area[self.face == f].sum())
                for f, name in enumerate(FACES)}

    def total_area(self) -> float:
        return float(self.area.sum())


# -- one coat per task, shared by the simulator hook and the scorer --------------------------------------------------
_SPEC: dict = {}
_STATE: dict = {}


def setup(body: str, slab: Slab, bed: Bed, law: Law = Law(), pan: str = "", links=("gripper", "tip_left", "tip_right"),
          parts=(), cuts=()):
    """Declare the piece's body name, its slab, the bed, and the breading pan's body name (its contact force on the
    piece is the bed force). `links` are each robot's links whose contact forces are read (the palm and both
    fingers). A soft piece (an articulation) names its rigid bodies in `parts`, lowest x first, each as (body name,
    its frame origin in the piece's frame), with the `cuts` between them (see `Coat`)."""
    _SPEC.update(body=body, slab=slab, bed=bed, law=law, pan=pan, links=tuple(links), parts=tuple(parts),
                 cuts=tuple(cuts))
    reset()


def coat_for(spec: dict) -> Coat:
    return Coat(spec["slab"], spec["bed"], spec["law"], cuts=spec.get("cuts", ()),
                origins=[o for _, o in spec["parts"]] if spec.get("parts") else None)


def reset() -> None:
    if _SPEC:
        _STATE["coat"] = coat_for(_SPEC)


def state() -> Coat:
    if "coat" not in _STATE:
        reset()
    return _STATE["coat"]


def classify(link_forces: dict, gripper_poses: dict, law: Law):
    """From contact forces on the piece (world, N; force ON the piece) keyed (robot, link), pick the pressers and the
    grips. gripper_poses: robot -> (tcp, rot, half_open). A gripper presses when its links together push the piece
    down by at least `presser_n`; it grips when both fingers push with at least `wipe_n` from opposite sides."""
    pressers, grips = [], []
    for robot, (tcp, rot, half) in gripper_poses.items():
        fs = {lk: np.asarray(f, float) for (r, lk), f in link_forces.items() if r == robot}
        if not fs:
            continue
        total = sum(fs.values(), np.zeros(3))
        if total[2] <= -law.presser_n:
            pressers.append(np.asarray(tcp, float))
        fl, fr = fs.get("tip_left"), fs.get("tip_right")
        if fl is not None and fr is not None and np.linalg.norm(fl) >= law.wipe_n and np.linalg.norm(fr) >= law.wipe_n \
                and float(fl @ fr) < 0:
            grips.append(Grip(tuple(tcp), np.asarray(rot, float), float(half)))
    return pressers, grips


# -- the coat the cameras see ------------------------------------------------------------------------------------------
# Breading crumbs: pale seasoned flour, toasted crumb and paprika flecks (sRGB), and how often each is drawn.
COAT_SHADES = ((0.94, 0.87, 0.72), (0.87, 0.73, 0.50), (0.78, 0.52, 0.30))
COAT_SHARE = (0.55, 0.33, 0.12)


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def quats(Rs) -> np.ndarray:
    """Rotation matrices (n, 3, 3) to xyzw quaternions (n, 4)."""
    R = np.asarray(Rs, float).reshape(-1, 3, 3)
    m00, m11, m22 = R[:, 0, 0], R[:, 1, 1], R[:, 2, 2]
    w = np.sqrt(np.maximum(0.0, 1 + m00 + m11 + m22)) / 2
    x = np.sqrt(np.maximum(0.0, 1 + m00 - m11 - m22)) / 2
    y = np.sqrt(np.maximum(0.0, 1 - m00 + m11 - m22)) / 2
    z = np.sqrt(np.maximum(0.0, 1 - m00 - m11 + m22)) / 2
    x = np.copysign(x, R[:, 2, 1] - R[:, 1, 2])
    y = np.copysign(y, R[:, 0, 2] - R[:, 2, 0])
    z = np.copysign(z, R[:, 1, 0] - R[:, 0, 1])
    q = np.stack([x, y, z, w], axis=1)
    for i in np.nonzero(w < 0.2)[0]:  # near a half turn the signs above are unreliable
        q[i] = quat(R[i])
    return q / np.linalg.norm(q, axis=1, keepdims=True)


class Tiles:
    """Coated cells drawn as thin crumb-colored flakes on the piece (Isaac Lab VisualizationMarkers), so the coat shows
    where it is. Each cell is one flake, laid flat on the drawn surface: `surface(points, normals)` maps the cells'
    centers and outward normals (the piece's frame) onto the visible mesh (default: the box faces themselves). The
    flakes vary in shade (`COAT_SHADES`), size and turn, so a coated face reads as breading rather than a grid. Turns
    itself off with one warning on any error; scoring never depends on it."""

    def __init__(self, lift: float = 0.0005, thick: float = 0.0006, path: str = "/Visuals/coatkit", surface=None,
                 grow: float = 1.25, seed: int = 11):
        self.lift, self.thick, self.path, self.surface, self.grow, self.seed = lift, thick, path, surface, grow, seed
        self.markers, self.off, self.shown = None, False, None
        self.local = None  # (points, normals, tile rotations, shade index, size scale) in the piece's frame

    def _layout(self, coat: Coat):
        c, n = coat.c, coat.n
        if self.surface is not None:
            c, n = self.surface(c, n)
        n = np.asarray(n, float) / np.linalg.norm(n, axis=1, keepdims=True)
        rng = np.random.default_rng(self.seed)
        k = len(c)
        rots = np.zeros((k, 3, 3))
        for i in range(k):  # tile frame: z along the normal, turned at random about it
            a = np.array([1.0, 0.0, 0.0]) if abs(n[i, 0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            u = np.cross(n[i], a)
            u /= np.linalg.norm(u)
            v = np.cross(n[i], u)
            t = rng.uniform(0, math.pi)
            u, v = math.cos(t) * u + math.sin(t) * v, -math.sin(t) * u + math.cos(t) * v
            rots[i] = np.stack([u, v, n[i]], axis=1)
        shade = rng.choice(len(COAT_SHADES), size=k, p=np.asarray(COAT_SHARE) / sum(COAT_SHARE))
        size = rng.uniform(0.85, 1.15, size=k) * self.grow
        self.local = (np.asarray(c, float), n, rots, shade, size)

    def draw(self, coat: Coat, pos, q, origin=(0.0, 0.0, 0.0)) -> None:
        if self.off:
            return
        try:
            sig = (hash(coat.coated.tobytes()), tuple(np.round(np.asarray(pos, float), 4).ravel().tolist()),
                   tuple(np.round(np.asarray(q, float), 3).ravel().tolist()))
            if sig == self.shown:
                return
            if self.markers is None:
                import isaaclab.sim as sim_utils
                from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

                protos = {f"crumb{i}": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(_linear(x) for x in rgb), roughness=0.95)) for i, rgb in enumerate(COAT_SHADES)}
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path=self.path, markers=protos))
            if self.local is None:
                self._layout(coat)
            self.shown = sig
            idx = np.nonzero(coat.coated)[0]
            if len(idx) == 0:
                self.markers.set_visibility(False)
                return
            pts, nrm, rots, shade, size = self.local
            side = math.sqrt(float(np.median(coat.area)))
            w, _, Rc = coat.to_world(pts + nrm * (self.lift + self.thick / 2), nrm, pos, q)
            tr = w[idx] + np.asarray(origin, float)
            q4 = quats(np.einsum("nij,njk->nik", Rc[idx], rots[idx]))[:, [3, 0, 1, 2]]  # wxyz for the markers
            sc = np.stack([side * size[idx], side * size[idx], np.full(len(idx), self.thick)], axis=1)
            self.markers.set_visibility(True)
            self.markers.visualize(translations=tr.astype(np.float32), orientations=q4.astype(np.float32),
                                   scales=sc.astype(np.float32), marker_indices=shade[idx].astype(np.int32))
        except ImportError:
            self.off = True  # no simulator: offline tests
        except Exception as e:  # noqa: BLE001 (cosmetic only)
            self.off = True
            warnings.warn(f"coatkit tiles off ({type(e).__name__}: {e}); scoring is unaffected", RuntimeWarning)
            print(f"coatkit: tiles off ({e!r})", file=sys.stderr, flush=True)


# -- simulator hooks ---------------------------------------------------------------------------------------------------
def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _find(stage, name: str, under: str = "", rigid: bool = False) -> str | None:
    """Path of the prim named `name` (under a path containing `under`), preferring env_0 and, with `rigid`, a rigid
    body (an arm link's own prim, not a same-named mesh or collider below it)."""
    from pxr import UsdPhysics

    best, best_rank = None, -1
    for prim in stage.Traverse():
        if prim.GetName() != name:
            continue
        path = str(prim.GetPath())
        if under and f"/{under}" not in path:
            continue
        rank = 2 * ("env_0" in path) + (bool(rigid) and prim.HasAPI(UsdPhysics.RigidBodyAPI))
        if rank > best_rank:
            best, best_rank = path, rank
    return best


class _Runtime:
    """Before each physics step (240 Hz): reads PhysX's contact forces on the piece from the pan and from each robot's
    palm and finger links (a rigid contact view with those filters, as cutkit reads the blade's), the piece's pose,
    and the grippers' poses and openings, and steps the coat."""

    ROBOTS = ("left_robot", "right_robot")
    PRIMS = {"left_robot": "LeftArm", "right_robot": "RightArm"}  # the arms' prims (scene keys differ)

    def __init__(self, base):
        from isaaclab.sim.utils.stage import get_current_stage

        self.base = base
        self.asset = base.scene[_SPEC["body"]]
        self.origin = _arr(base.scene.env_origins)[0]
        self.dt = float(getattr(base, "physics_dt", 1 / 240))
        self.debug = bool(os.environ.get("COAT_DEBUG"))
        stage = get_current_stage()
        self.parts = _SPEC.get("parts") or ()
        if self.parts:  # an articulation: one rigid body per part
            pieces = [_find(stage, name, _SPEC["body"], rigid=True) for name, _ in self.parts]
            bodies = list(self.asset.body_names)
            self.body_ids = [bodies.index(name) for name, _ in self.parts]
        else:
            pieces = [_find(stage, _SPEC["body"])]
        piece = pieces
        pan = _find(stage, _SPEC["pan"])
        self.keys, filters = [("pan", "pan")], [pan]
        for robot in self.ROBOTS:
            for lk in _SPEC["links"]:
                path = _find(stage, lk, self.PRIMS[robot], rigid=True)
                if path is not None:
                    self.keys.append((robot, lk))
                    filters.append(path)
        print(f"[coatkit] piece {piece}; filters {filters}", flush=True)
        if len(self.keys) < 1 + 2 * len(_SPEC["links"]):
            print("[coatkit] WARNING: not every gripper link was found; grips there are never seen and the coat is "
                  "never wiped (the score would be too high). Fix the link names or _find's path match.", flush=True)
        try:
            from isaaclab_physx.physics import PhysxManager

            view = PhysxManager.get_physics_sim_view()
        except Exception:  # noqa: BLE001 (older Isaac Lab)
            import omni.physics.tensors as tensors

            view = tensors.create_simulation_view("torch")
        self.contact = view.create_rigid_contact_view(pieces, filter_patterns=[filters] * len(pieces),
                                                      max_contact_data_count=256 * len(pieces))
        self.n = 0

    def contact_rows(self):
        return self.parts or (None,)

    def gripper_poses(self) -> dict:
        from yam_arena.constants import FINGER_OPEN, TCP_OFFSET

        out = {}
        for robot in self.ROBOTS:
            try:
                art = self.base.scene[robot]
            except KeyError:
                continue
            i = art.body_names.index("gripper")
            p = _arr(art.data.body_pos_w)[0, i] - self.origin
            R = rotm(_arr(art.data.body_quat_w)[0, i])
            tcp = p + R @ np.asarray(TCP_OFFSET, float)
            j = art.joint_names.index("joint7")
            half = abs(float(_arr(art.data.joint_pos)[0, j]))  # each finger's travel from closed (m)
            half = min(half, abs(FINGER_OPEN))
            out[robot] = (tcp, R, half)
        return out

    def physics_step(self) -> None:
        d = self.asset.data
        if self.parts:
            pos = _arr(d.body_pos_w)[0, self.body_ids] - self.origin
            q = _arr(d.body_quat_w)[0, self.body_ids]  # (x, y, z, w), as yamkit reads it
        else:
            pos = _arr(d.root_pos_w)[0] - self.origin
            q = _arr(d.root_quat_w)[0]  # (x, y, z, w) in this Isaac Lab (as yamkit and insertkit read it)
        fm = _arr(self.contact.get_contact_force_matrix(dt=self.dt)).reshape(len(self.contact_rows()), -1, 3)
        fm = fm.sum(axis=0)  # force ON the piece (all its parts) from each filter body
        forces = {k: fm[i] for i, k in enumerate(self.keys)}
        bed_fz = float(forces[("pan", "pan")][2])
        links = {k: f for k, f in forces.items() if k[0] != "pan"}
        pressers, grips = classify(links, self.gripper_poses(), _SPEC["law"])
        c = state()
        c.physics(pos, q, bed_fz, pressers, grips)
        self.n += 1
        if self.debug and self.n % 24 == 0:
            print(f"[coatkit] bed {bed_fz:.2f} N pressers {len(pressers)} grips {len(grips)} "
                  f"coverage {c.coverage() * 100:.1f}% presses {c.presses}", flush=True)


_RT: dict = {}


def author_report(stage) -> bool:
    """Contact reporting on the piece's rigid body (threshold 0), as cutkit does on its blade, so the contact view
    reports every touch. True when authored."""
    from pxr import PhysxSchema

    if _SPEC.get("parts"):
        paths = [_find(stage, name, _SPEC["body"], rigid=True) for name, _ in _SPEC["parts"]]
    else:
        paths = [_find(stage, _SPEC["body"])]
    if any(p is None for p in paths):
        return False
    for path in paths:
        api = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path))
        api.CreateThresholdAttr(0.0)
        print(f"[coatkit] contact report on {path}", flush=True)
    return True


def _patch_scene_cls(module) -> None:
    cls = getattr(module, "InteractiveScene", None)
    if cls is None or getattr(cls, "_coatkit", False):
        return
    init = cls.__init__

    def __init__(self, *a, **k):
        init(self, *a, **k)
        try:
            if _SPEC and author_report(self.stage):
                _RT["report"] = True
        except Exception as error:  # noqa: BLE001
            print(f"[coatkit] contact report at scene build failed: {error}", flush=True)
        write = self.write_data_to_sim

        def write_data_to_sim(*aa, **kk):
            base = _RT.get("base")
            if base is not None and _SPEC:
                try:
                    if "rt" not in _RT:
                        _RT["rt"] = _Runtime(base)
                    _RT["rt"].physics_step()
                except Exception as error:  # noqa: BLE001 (report once, never stop the sim)
                    if not _RT.get("warned"):
                        _RT["warned"] = True
                        print(f"[coatkit] physics hook failed: {type(error).__name__}: {error}", flush=True)
            return write(*aa, **kk)

        self.write_data_to_sim = write_data_to_sim

    cls.__init__, cls._coatkit = __init__, True


def _patch_embodiment(module) -> None:
    cls = getattr(module, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_coatkit", False):
        return
    ensure_sim, reset_ = cls._ensure_sim, cls.reset

    def _ensure_sim(self, *a, **k):
        out = ensure_sim(self, *a, **k)
        if self._env is not None and "base" not in _RT:
            _RT["base"] = self._env.unwrapped
            if not _RT.get("report"):
                import omni.usd

                author_report(omni.usd.get_context().get_stage())
                print("[coatkit] WARNING: contact report authored after the scene was built (check COAT_DEBUG shows "
                      "forces)", flush=True)
        return out

    def _reset(self, *a, **k):
        reset()
        obs = reset_(self, *a, **k)
        reset()  # the settle steps are not part of the trial
        return obs

    cls._ensure_sim, cls.reset, cls._coatkit = _ensure_sim, _reset, True


_TARGETS = {"isaaclab.scene.interactive_scene": _patch_scene_cls, "yam_arena.inspect_embodiment": _patch_embodiment}


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
