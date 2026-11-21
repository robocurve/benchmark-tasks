"""tipkit: the scene's parts (tip box with its 96-tip rack, eight loose column-4 tips, the 8-channel pipette, its stand,
tape marks) and the tip-fit model that runs inside the physics step.

How a tip goes on (tiplaw.py has the numbers):
- Each column-4 tip is a rigid body standing in its rack hole on its collar. The other columns' tips are part of the
  box (they never move unless a nozzle takes one, which breaks a rule).
- Every physics step, each nozzle is matched with the tip hole under it. A nozzle enters a tip if, as its end reaches the
  tip's mouth, it is within 0.7 mm and 4 deg of the tip's axis; further off but within half a pitch, it jams on the rim.
  An entered nozzle keeps its tip until it rises out of the mouth. While entered, the cone friction fit
  (``tiplaw.fit_force``) pushes the pipette up at the nozzle's end and the box down at the tip's hole: the tip stands on
  the rack plate, so its load goes through the rack into the box (a 0.35 g rigid tip under several newtons is not
  stable at 240 Hz, so the load is not applied to the tip body itself). Below their roots the nozzles have no collider.
- A tip comes off the rack only on the nozzle: once the nozzle has risen 9 mm above where it would touch the tip (to
  about the tip's mouth), a tip pushed to full seat (load F_SEAT) is moved onto its nozzle (a snap of about 9 mm) and
  locked to the pipette with a fixed joint at full seat; a tip pushed to 0.4 F_SEAT or more is locked 1.5 mm short; a tip
  pushed less stays in its hole. Nothing attaches by nearness. Over 80 N in all, the rack insert counts as pushed out.
"""

from __future__ import annotations

import json
import math
import os

import numpy as np

import bodykit
from bodykit import Body, BoxShape, CylShape, MeshShape
import tiplaw as L

TOP = 0.7475  # the station table's top
DEBUG = os.environ.get("TIP_DEBUG") == "1"  # development only: print contacts of the pipette and the box
MM = 1e-3

# -- the tip box (frame: the middle of its underside) -------------------------------------------------------------------
BOX_XY = (0.2875, 0.0)
BOX_L, BOX_W = 0.122, 0.082  # along x (the columns), along y (the rows)
WALL_T = 0.002
PLATE_TOP = 0.050  # the rack plate's top above the bench
PLATE_T = 0.002
RIM_H = 0.052  # the box's top rim and lip
LIP_W = 0.005
TIP_TOP = 0.058  # the tips' mouths above the bench
HOLE = 0.007  # the rack holes, square for the collider
BOX_MASS = 0.147  # with 72 tips, about 0.15 kg
COL4_X = 0.265
COL_X = [COL4_X + (c - 4) * L.PITCH * MM for c in range(1, 13)]  # world x of columns 1..12
ROW_Y = (L.ROW_Y * MM).tolist()  # world y of rows A..H
RIM_Z = TOP + RIM_H

# -- the pipette (frame: the middle of the manifold's underside; nozzles point down -z, the row along y) ----------------
MANIFOLD = (0.040, 0.085, 0.070)
HANDLE_R, HANDLE_LEN = 0.016, 0.120
PIPETTE_MASS = 0.20
GRIP_Z = MANIFOLD[2] + 0.019  # the TCP of a sideways grasp across the handle, its pads just above the manifold's shoulder

# -- the stand the pipette starts in -------------------------------------------------------------------------------------
STAND_XY = (0.31, -0.19)
RAIL_TOP = 0.075
TIP_MASS = 0.00035

TIP_COLOR = (0.96, 0.80, 0.18)
BOX_COLOR = (0.20, 0.40, 0.74)
PLATE_COLOR = (0.90, 0.91, 0.93)


def _lathe(profile, segments: int, offset=(0.0, 0.0, 0.0), cap_top: float | None = None):
    """Points and faces of a surface of revolution about z: ``profile`` is (radius, z) from top to bottom (m)."""
    ox, oy, oz = offset
    pts, faces = [], []
    n = len(profile)
    for r, z in profile:
        for k in range(segments):
            a = 2 * math.pi * k / segments
            pts.append((ox + r * math.cos(a), oy + r * math.sin(a), oz + z))
    for i in range(n - 1):
        for k in range(segments):
            a, b = i * segments + k, i * segments + (k + 1) % segments
            c, d = a + segments, b + segments
            faces += [(a, c, b), (b, c, d)]
    if cap_top is not None:  # a flat ring at the top, from the profile's top radius in to ``cap_top``
        base = len(pts)
        r0, z0 = profile[0]
        for k in range(segments):
            a = 2 * math.pi * k / segments
            pts.append((ox + cap_top * math.cos(a), oy + cap_top * math.sin(a), oz + z0))
        for k in range(segments):
            a, b = k, (k + 1) % segments
            faces += [(a, b, base + a), (b, base + b, base + a)]
    return pts, faces


def _merge(parts):
    pts, faces = [], []
    for p, f in parts:
        o = len(pts)
        pts += p
        faces += [(a + o, b + o, c + o) for a, b, c in f]
    return tuple(pts), tuple(faces)


def _disc(r: float, centre, segments: int = 12):
    cx, cy, cz = centre
    pts = [(cx, cy, cz)] + [(cx + r * math.cos(2 * math.pi * k / segments), cy + r * math.sin(2 * math.pi * k / segments), cz)
                            for k in range(segments)]
    faces = [(0, 1 + k, 1 + (k + 1) % segments) for k in range(segments)]
    return pts, faces


# a universal 200 uL tip, mouth at z 0: wide top 8 mm, then tapering to the 0.6 mm end, 51 mm long
TIP_PROFILE = [(0.0035, 0.0), (0.0035, -0.0075), (0.0031, -0.0085), (0.0029, -0.020), (0.0016, -0.040), (0.0005, -0.051)]


def tip_mesh(offset=(0.0, 0.0, 0.0)):
    outer = _lathe(TIP_PROFILE, 12, offset, cap_top=0.00265)
    hole = _disc(0.00265, (offset[0], offset[1], offset[2] - 0.0012))  # the dark inside, seen through the mouth
    return outer, hole


def _box_shapes() -> list:
    shapes = []
    hx, hy = BOX_L / 2, BOX_W / 2
    shapes.append(BoxShape((0, 0, 0.001), (BOX_L, BOX_W, 0.002), BOX_COLOR))
    for sx in (-1, 1):
        shapes.append(BoxShape((sx * (hx - WALL_T / 2), 0, PLATE_TOP / 2), (WALL_T, BOX_W, PLATE_TOP), BOX_COLOR))
    for sy in (-1, 1):
        shapes.append(BoxShape((0, sy * (hy - WALL_T / 2), PLATE_TOP / 2), (BOX_L, WALL_T, PLATE_TOP), BOX_COLOR))
        shapes.append(BoxShape((0, sy * (hy + LIP_W / 2 - WALL_T), RIM_H - 0.002), (BOX_L, LIP_W, 0.004), BOX_COLOR))
    for sx in (-1, 1):  # the short sides' rim
        shapes.append(BoxShape((sx * (hx - WALL_T / 2), 0, RIM_H - 0.001), (WALL_T, BOX_W, 0.002), BOX_COLOR))
    # the rack plate: one visual plate, colliders with a hole for each column-4 tip
    zc = PLATE_TOP - PLATE_T / 2
    shapes.append(BoxShape((0, 0, zc), (BOX_L - 2 * WALL_T, BOX_W - 2 * WALL_T, PLATE_T), PLATE_COLOR, collide=False))
    x4 = COL4_X - BOX_XY[0]
    ix0, ix1 = -(hx - WALL_T), hx - WALL_T
    iy0, iy1 = -(hy - WALL_T), hy - WALL_T
    h = HOLE / 2
    for a, b in ((ix0, x4 - h), (x4 + h, ix1)):
        shapes.append(BoxShape(((a + b) / 2, 0, zc), (b - a, iy1 - iy0, PLATE_T), None))
    edges = [iy1] + [v for y in ROW_Y for v in (y + h, y - h)] + [iy0]
    for k in range(0, len(edges), 2):
        top, bot = edges[k], edges[k + 1]
        shapes.append(BoxShape((x4, (top + bot) / 2, zc), (HOLE, top - bot, PLATE_T), None))
    # the other columns' tips (5-12), the empty holes of columns 1-3, and the dark holes under column 4
    tips, holes = [], []
    for c in range(1, 13):
        xb = COL_X[c - 1] - BOX_XY[0]
        for y in ROW_Y:
            if c >= 5:
                outer, hole = tip_mesh((xb, y, TIP_TOP))
                tips.append(outer)
                holes.append(hole)
            else:
                holes.append(_disc(0.0032, (xb, y, PLATE_TOP + 0.0002)))
    p, f = _merge(tips)
    shapes.append(MeshShape((0, 0, 0.03), p, f, TIP_COLOR, normals=None, roughness=0.35))
    p, f = _merge(holes)
    shapes.append(MeshShape((0, 0, 0.03), p, f, (0.08, 0.09, 0.10)))
    # the hinged lid, open flat on the -y side
    shapes.append(BoxShape((0, -(hy + 0.002 + 0.041), 0.0015), (BOX_L, 0.082, 0.003), (0.80, 0.84, 0.88), collide=False))
    shapes.append(BoxShape((0, -(hy + 0.002 + 0.041), 0.004), (BOX_L - 0.01, 0.072, 0.002), (0.86, 0.89, 0.92), collide=False))
    return shapes


def _pipette_shapes() -> list:
    mx, my, mz = MANIFOLD
    grey, dark, light = (0.36, 0.38, 0.42), (0.10, 0.11, 0.12), (0.86, 0.87, 0.89)
    s = [BoxShape((0, 0, mz / 2), MANIFOLD, grey),
         BoxShape((0, 0, 0.004), (mx + 0.001, my - 0.004, 0.008), light, collide=False),  # the ejector sleeve's edge
         CylShape((0, 0, mz + HANDLE_LEN / 2), HANDLE_R, HANDLE_LEN, light),
         BoxShape((HANDLE_R - 0.0008, 0, mz + 0.060), (0.002, 0.014, 0.026), dark, collide=False),  # volume window
         BoxShape((HANDLE_R + 0.010, 0, mz + 0.106), (0.022, 0.016, 0.010), light),  # finger hook
         CylShape((0, 0, mz + HANDLE_LEN + 0.010), 0.007, 0.020, light),
         CylShape((0, 0, mz + HANDLE_LEN + 0.0235), 0.011, 0.007, (0.22, 0.42, 0.78)),
         BoxShape((-0.012, 0, mz + HANDLE_LEN - 0.008), (0.008, 0.012, 0.016), (0.22, 0.42, 0.78), collide=False)]  # ejector button
    cones = []
    for y in ROW_Y:
        s.append(CylShape((0, y, -0.0015), 0.0015, 0.003, None))  # the nozzle's root only: below, the fit model acts
        cones.append(_lathe([(0.0028, 0.0), (0.0021, -0.012)], 10, (0, y, 0.0), cap_top=None))
        cones.append(_disc(0.0021, (0, y, -0.012), 10))
    p, f = _merge(cones)
    # flip the end discs to face down: winding does not matter (double-sided)
    s.append(MeshShape((0, 0, -0.006), p, f, (0.70, 0.71, 0.73)))
    return s


def _stand_shapes() -> list:
    c = (0.84, 0.85, 0.86)
    s = [BoxShape((0, 0, 0.003), (0.070, 0.110, 0.006), c)]
    for sy in (-1, 1):
        s.append(BoxShape((0, sy * 0.040, 0.0065 + (RAIL_TOP - 0.008 - 0.006) / 2), (0.012, 0.008, RAIL_TOP - 0.008 - 0.006), c))
        s.append(BoxShape((0, sy * 0.040, RAIL_TOP - 0.004), (0.050, 0.008, 0.008), c))
    return s


def _mark_shapes() -> list:
    red, leg, w, t = (0.80, 0.16, 0.12), 0.020, 0.006, 0.0004
    s = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = BOX_XY[0] + sx * (BOX_L / 2 + 0.001 + w / 2), BOX_XY[1] + sy * (BOX_W / 2 + LIP_W - WALL_T + 0.001 + w / 2)
            if sy < 0:  # the lid lies over the -y side's marks: put them past its ends
                cy = BOX_XY[1] - (BOX_W / 2 + 0.001 + w / 2)
            s.append(BoxShape((cx - sx * (leg / 2 - w / 2), cy, t / 2), (leg, w, t), red, collide=False))
            s.append(BoxShape((cx, cy - sy * (leg / 2 - w / 2), t / 2), (w, leg, t), red, collide=False))
    return s


def tip_shapes() -> list:
    outer, hole = tip_mesh()
    p, f = _merge([outer, hole])
    return [BoxShape((0, 0, -0.004), (0.0086, 0.0086, 0.008), None),
            BoxShape((0, 0, -0.024), (0.003, 0.003, 0.032), None),
            MeshShape((0, 0, -0.025), p, f, TIP_COLOR, roughness=0.35)]


TIP_NAMES = [f"tip_{'ABCDEFGH'[i]}4" for i in range(8)]


def bodies() -> list:
    marks = Body(name="tape_marks", pos=(0.0, 0.0, TOP), size=(BOX_L + 0.03, BOX_W + 0.03, 0.001), mass=0.01,
                 shapes=tuple(_mark_shapes()), kinematic=True)
    stand = Body.build("pipette_stand", (*STAND_XY, TOP), 0.3, _stand_shapes(), kinematic=True, friction=(0.5, 0.4))
    box = Body.build("tip_box", (*BOX_XY, TOP + 0.0003), BOX_MASS, _box_shapes(), friction=(0.4, 0.35), com=(0.0, 0.0, 0.022),
                     solver_iterations=32, contact_offset=0.001, contact_report=True)
    tips = []
    for i, name in enumerate(TIP_NAMES):
        tips.append(Body.build(name, (COL4_X, ROW_Y[i], TOP + TIP_TOP + 0.0002), TIP_MASS, tip_shapes(), friction=(0.4, 0.3),
                               solver_iterations=32, contact_offset=0.0005, contact_report=True, inertia=1e-7,
                               ignore=tuple(TIP_NAMES[:i]), damping=(0.0, 0.05)))
    pipette = Body.build("pipette", (*STAND_XY, TOP + RAIL_TOP + 0.0004), PIPETTE_MASS, _pipette_shapes(), friction=(0.9, 0.8),
                         com=(0.0, 0.0, 0.075), solver_iterations=32, contact_report=True, ignore=tuple(TIP_NAMES),
                         contact_offset=0.0015)
    return [marks, stand, box, *tips, pipette]


# -- the lock joints: two fixed joints per column-4 tip, to its own nozzle, switched off until a lift --------------------

LOCK_S = {"full": L.S_FULL, "short": L.S_SHORT}


def _lock_local(i: int, kind: str) -> np.ndarray:
    """The tip's origin (its mouth) in the pipette's frame when locked on nozzle ``i``."""
    return np.array([0.0, ROW_Y[i], -L.NOZZLE_LEN * MM + LOCK_S[kind] * MM])


def _author_locks(stage, prim_path: str, spec) -> None:
    if spec.name not in TIP_NAMES:
        return
    from pxr import Gf, UsdPhysics

    i = TIP_NAMES.index(spec.name)
    parent = prim_path.rsplit("/", 1)[0]
    for kind in LOCK_S:
        joint = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/lock_{kind}")
        joint.CreateBody0Rel().SetTargets([f"{parent}/pipette/body"])
        joint.CreateBody1Rel().SetTargets([f"{prim_path}/body"])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*_lock_local(i, kind).tolist()))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateJointEnabledAttr(False)
        joint.CreateExcludeFromArticulationAttr(True)
        Model.LOCK_PATHS[(i, kind)] = f"{prim_path}/lock_{kind}"


bodykit.EXTRA_AUTHORS.append(_author_locks)


# -- small maths ------------------------------------------------------------------------------------------------------------

def quat_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _arr(x) -> np.ndarray:
    if hasattr(x, "torch"):
        x = x.torch
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return np.asarray(x, dtype=np.float64)


# -- the model ------------------------------------------------------------------------------------------------------------

class Model:
    """The tip fits, the locks and the rule latches of one trial. ``attach(env)`` once per reset; it subscribes to the
    physics step."""

    LOCK_PATHS: dict = {}

    def __init__(self):
        self.env = None
        self.view = None
        self.sub = None
        self.trace_path = os.environ.get("TIP_TRACE")
        self.trace: list = []
        self.trace_n = 0
        self.reset_state()
        bodykit.PRE_RESET.append(self.unlock_all)

    # -- state
    def reset_state(self) -> None:
        self.d = {}  # slot (col, row) -> depth reached, mm
        self.loaded_by = {}  # slot -> nozzle that pushed it
        self.mode = [None] * 8  # per nozzle: None, ("in", slot) or ("jam", slot)
        self.prev_s = [None] * 8
        self.prev_slot = [None] * 8
        self.meas = np.zeros(8)  # collar-to-rack load, smoothed (N)
        self.meas_peak = np.zeros(8)
        self.collar = np.zeros(8)
        self.dbg_last = None
        self.locked = [None] * 8  # None, "full", "short"
        self.flags: set = set()
        self.total = 0.0
        self.total_peak = 0.0
        self.rack = 0.0
        self.t = 0.0
        self.events: list = []
        self.held_once = False
        self.prev_pip_z = None
        self.peaks_log: list = []

    def reset(self) -> None:
        if self.trace_path and self.trace:
            self.save()
            self.trace_n += 1
        self.reset_state()
        self.trace = []
        for path in self.LOCK_PATHS.values():
            self._set_joint(path, False)

    def unlock_all(self) -> None:
        """Switch every lock off before the scene is put back, so that no tip drags the pipette or the box."""
        for path in self.LOCK_PATHS.values():
            self._set_joint(path, False)
        self.locked = [None] * 8

    def _set_joint(self, path: str, on: bool) -> None:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(path)
        if prim.IsValid():
            UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(bool(on))

    def attach(self, env) -> None:
        self.env = env
        if self.view is not None:
            return
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        base = env.unwrapped
        stage = get_current_stage()
        self.origin = _arr(base.scene.env_origins)[0]
        names = ["pipette", "tip_box", *TIP_NAMES]
        paths = [base.scene[n].root_view.prim_paths[0] for n in names]
        fingers = []
        for side in ("left", "right"):
            lp = list(base.scene[f"{side}_robot"].root_view.link_paths[0])
            for nm in ("tip_left", "tip_right"):
                p = [q for q in lp if q.endswith("/" + nm) and stage.GetPrimAtPath(q).HasAPI(UsdPhysics.RigidBodyAPI)]
                fingers.append(p[0] if p else [q for q in lp if q.endswith("/" + nm)][0])
        sim_view = base.scene["pipette"]._physics_sim_view
        self.sim_view = sim_view
        self.view = sim_view.create_rigid_body_view(paths)
        got = list(self.view.prim_paths)
        self.idx = [got.index(p) for p in paths]  # our order -> the view's
        self.n = len(paths)
        self.tip_contacts = sim_view.create_rigid_contact_view(paths[2:], filter_patterns=[[paths[1], *fingers]] * 8)
        self.pip_contacts = sim_view.create_rigid_contact_view([paths[0]], filter_patterns=[fingers])
        self.dbg_contacts = sim_view.create_rigid_contact_view([paths[0]], filter_patterns=[[paths[1], *paths[2:]]])
        links = []
        for side in ("left", "right"):
            for q in base.scene[f"{side}_robot"].root_view.link_paths[0]:
                if stage.GetPrimAtPath(q).HasAPI(UsdPhysics.RigidBodyAPI):
                    links.append(q)
        self.link_names = [q.rsplit("/", 1)[-1] + ("L" if "left" in q else "R") for q in links]
        self.box_links = sim_view.create_rigid_contact_view([paths[1]], filter_patterns=[links])
        sample = self.view.get_transforms()
        self._torch = hasattr(sample, "device") and not hasattr(sample, "numpy")
        self.device = str(sample.device)  # a torch or a warp device; torch takes the string
        import omni.physx

        self.sub = omni.physx.get_physx_interface().subscribe_physics_on_step_events(self._physics_step, pre_step=True, order=50)

    # -- reading the scene
    def _poses(self):
        tr = _arr(self.view.get_transforms())
        out = []
        for k in range(self.n):
            row = tr[self.idx[k]]
            out.append((row[:3] - self.origin, quat_matrix(row[3:7])))
        return out

    def _slot_of(self, e, box) -> tuple | None:
        bp, br = box
        local = br.T @ (e - bp)
        c = int(round((local[0] + BOX_XY[0] - COL_X[0]) / (L.PITCH * MM))) + 1
        r = int(round((ROW_Y[0] - local[1]) / (L.PITCH * MM)))
        if not (1 <= c <= 12 and 0 <= r <= 7) or c <= 3:
            return None
        if c == 4 and self.locked[r] is not None:
            return None  # its tip is on a nozzle: the hole is empty
        return (c, r)

    def _mouth(self, slot, poses):
        c, r = slot
        bp, br = poses[1]
        local = np.array([COL_X[c - 1] - BOX_XY[0], ROW_Y[r], TIP_TOP + (L.SEAT_H[r] * MM if c == 4 else 0.0)])
        return bp + br @ local, br[:, 2]

    # -- the physics step: the fits' forces
    def _physics_step(self, dt) -> None:
        if self.env is None or self.view is None:
            return
        try:
            self._fits(float(dt))
        except Exception as exc:  # noqa: BLE001  (views not ready during a reset)
            if "invalid" not in str(exc).lower():
                print(f"[tipkit] physics step: {exc!r}", flush=True)

    def _fits(self, dt: float) -> None:
        self.t += dt
        # a gripper touching a column-4 tip breaks a rule: checked every physics step, wherever the pipette is
        cm = _arr(self.tip_contacts.get_contact_force_matrix(dt))  # (8, 5, 3): each tip against the box and 4 finger pads
        self._check_touch(cm)
        poses = self._poses()
        pp, pr = poses[0]
        ap = pr[:, 2]
        ends = [pp + pr @ np.array([0.0, y, -L.NOZZLE_LEN * MM]) for y in ROW_Y]
        lowest = min(e[2] for e in ends)
        active = any(m is not None for m in self.mode)
        if lowest > TOP + TIP_TOP + 0.015 and not active:
            self.prev_s = [None] * 8
            self.total = 0.0
            self.meas *= max(0.0, 1.0 - dt / 0.05)
            return
        forces = np.zeros((self.n, 3))
        points = np.zeros((self.n, 3))
        torques = np.zeros((self.n, 3))
        total = 0.0
        static_load = 0.0
        load = np.zeros(8)
        for j in range(8):
            e = ends[j]
            slot = self.mode[j][1] if self.mode[j] is not None else self._slot_of(e, poses[1])  # a nozzle in a tip keeps it
            if slot is None:
                self.mode[j], self.prev_s[j], self.prev_slot[j] = None, None, None
                continue
            m, at = self._mouth(slot, poses)
            rel = e - m
            s = -float(rel @ at) / MM  # the nozzle end below the mouth, mm
            latv = rel - at * float(rel @ at)
            lat = float(np.linalg.norm(latv)) / MM
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(ap @ at)))))
            prev = self.prev_s[j] if self.prev_slot[j] == slot else None
            mode = self.mode[j] if (self.mode[j] is not None and self.mode[j][1] == slot) else None
            if mode is None and s >= 0.0 and prev is not None and prev < 0.0:  # reaching the mouth
                if lat <= L.CAPTURE_MM and ang <= L.CAPTURE_DEG:
                    mode = ("in", slot)
                elif lat <= L.RIM_MM:
                    mode = ("jam", slot)
                if mode is not None:
                    self.events.append((round(self.t, 3), mode[0], f"nozzle {j} tip {'ABCDEFGH'[slot[1]]}{slot[0]}: {lat:.2f} mm off, {ang:.1f} deg"))
                    print(f"[tipkit] t {self.t:.3f} s: nozzle {j} {mode[0]} tip {'ABCDEFGH'[slot[1]]}{slot[0]}: {lat:.2f} mm off axis, {ang:.1f} deg", flush=True)
            if mode is not None and (s < -2.0 or lat > (L.SLIP_MM if mode[0] == "in" else L.RIM_MM)):
                mode = None
            self.mode[j], self.prev_slot[j] = mode, slot
            vs = 0.0 if prev is None else max(-200.0, min(200.0, (s - prev) / dt))
            self.prev_s[j] = s
            if mode is None:
                continue
            if mode[0] == "in":
                c, r = slot
                u = s - L.S_CONTACT + (L.SEAT_H[r] if c == 4 else 0.0)
                f, dn = L.fit_force(u, self.d.get(slot, 0.0))
                if dn > self.d.get(slot, 0.0) + 1e-9:
                    self.d[slot] = dn
                    self.loaded_by[slot] = j
                    if (c != 4 or j != r) and dn >= L.HOLD_FRACTION * L.U_FULL:
                        self._flag("wrong_tip", f"nozzle {j} pushed tip {'ABCDEFGH'[r]}{c} on")
            else:
                f = L.K_RIM * max(s, 0.0)
            if f > 0.0:
                f = max(0.0, f + L.DAMP * vs)
            fl = np.zeros(3)
            if mode[0] == "in" and s > 0.0 and lat > 1e-6:
                fl = -latv / (lat * MM) * min(L.LAT_MAX_N, L.K_LAT * lat)
            fvec = at * f + fl
            total += f
            forces[0] += fvec
            torques[0] += np.cross(e - pp, fvec)  # force at the nozzle end, as a force plus a torque at the origin
            # the tip stands on the rack plate: its load goes into the box at the tip's hole
            if slot[0] == 4:
                load[slot[1]] += f
            else:
                static_load += f
            bp = poses[1][0]
            forces[1] -= fvec
            torques[1] += np.cross(m - bp, -fvec)
        # apply: forces at the bodies' origins plus torques (global frame)
        for k in range(self.n):
            if k >= 2 and np.any(forces[k]):
                torques[k] += np.cross(points[k] - poses[k][0], forces[k])
        bpos = poses[1][0]
        if DEBUG and self.dbg_last is not None and np.linalg.norm(bpos - self.dbg_last) > 0.001:
            print(f"[tipkit] t {self.t:.4f} s: box jumped {np.round((bpos - self.dbg_last) * 1000, 2).tolist()} mm; "
                  f"modes {self.mode}; F {np.round(forces[:2], 2).tolist()} T {np.round(torques[:2], 3).tolist()}", flush=True)
        self.dbg_last = bpos
        self._apply(forces, torques, poses)
        self.total = total
        self.total_peak = max(self.total_peak, total)
        # the column-4 tips' loads on the rack
        a = min(1.0, dt / 0.05)
        self.meas = self.meas + a * (load - self.meas)
        self.collar = self.collar + a * (np.linalg.norm(cm[:, 0, :], axis=1) - self.collar)  # a locked tip pressed on the rack
        self.meas_peak = np.maximum(self.meas_peak, self.meas)
        self.rack = self.rack + a * (float(self.meas.sum()) + static_load - self.rack)
        if self.rack > L.RACK_LIMIT_N:
            self._flag("rack_out", f"rack load {self.rack:.0f} N")

    def _check_touch(self, cm) -> None:
        """``cm``: the column-4 tips' contact forces (8, 5, 3), column 0 the box, 1-4 the grippers' finger pads."""
        if np.any(np.linalg.norm(np.asarray(cm)[:, 1:, :], axis=2) > 0.05):
            self._flag("tip_touched", "a gripper touched a column-4 tip")

    def _apply(self, forces, torques, poses) -> None:
        import torch

        fz = np.zeros((self.n, 3), np.float32)
        tz = np.zeros((self.n, 3), np.float32)
        pz = np.zeros((self.n, 3), np.float32)
        for k in range(self.n):
            fz[self.idx[k]] = forces[k]
            tz[self.idx[k]] = torques[k]
            pz[self.idx[k]] = poses[k][0] + self.origin  # forces act at the body origins, with the torques about them
        if not (np.any(fz) or np.any(tz)):
            return
        idx = torch.arange(self.n, dtype=torch.int32, device=self.device)
        f = torch.tensor(fz, device=self.device)
        t = torch.tensor(tz, device=self.device)
        p = torch.tensor(pz, device=self.device)
        if not self._torch:
            import warp as wp

            f, t, p, idx = wp.from_torch(f), wp.from_torch(t), wp.from_torch(p), wp.from_torch(idx)
        self.view.apply_forces_and_torques_at_position(f, t, p, idx, True)

    def _flag(self, key: str, why: str) -> None:
        if key not in self.flags:
            self.flags.add(key)
            self.events.append((round(self.t, 2), key, why))
            print(f"[tipkit] t {self.t:.2f} s: {key}: {why}", flush=True)

    # -- the control step: locks, and the scoring snapshot
    def step(self):
        """Locks tips that lift away with their nozzle; returns the scoring snapshot (tip_rules.Snapshot)."""
        from tip_rules import Snapshot

        poses = self._poses()
        pp, pr = poses[0]
        for i in range(8):
            slot = (4, i)
            d = self.d.get(slot, 0.0)
            tp, tr = poses[2 + i]
            if self.locked[i] is None:
                if d < L.HOLD_FRACTION * L.U_FULL or self.loaded_by.get(slot) != i:
                    continue
                e = pp + pr @ np.array([0.0, ROW_Y[i], -L.NOZZLE_LEN * MM])
                m, at = self._mouth(slot, poses)
                rel = e - m
                s = -float(rel @ at) / MM
                lat = float(np.linalg.norm(rel - at * float(rel @ at))) / MM
                u = s - L.S_CONTACT + L.SEAT_H[i]
                if u < d - L.LOCK_RISE_MM and lat <= 10.0:  # on its nozzle, which has risen to about its mouth
                    full = d >= L.U_FULL - 0.02  # it reached full seat, which took F_SEAT
                    self._lock(i, "full" if full else "short", poses)
            elif self.locked[i] == "short" and self.collar[i] >= L.F_SEAT:
                self._lock(i, "full", poses)  # pressed on again until it reached full seat
        return self.snapshot(poses)

    def _lock(self, i: int, kind: str, poses) -> None:
        pp, pr = poses[0]
        if self.locked[i] is not None:
            self._set_joint(self.LOCK_PATHS[(i, self.locked[i])], False)
        want = pp + pr @ _lock_local(i, kind)
        from scipy.spatial.transform import Rotation

        q = Rotation.from_matrix(pr).as_quat()  # xyzw
        tr = self.view.get_transforms()
        arr = _arr(tr).astype(np.float32)
        arr[self.idx[2 + i], :3] = want + self.origin
        arr[self.idx[2 + i], 3:7] = q
        import torch

        data = torch.tensor(arr, device=self.device)
        idx = torch.tensor([self.idx[2 + i]], dtype=torch.int32, device=self.device)
        vel = torch.zeros((self.n, 6), dtype=torch.float32, device=self.device)
        if not self._torch:
            import warp as wp

            data, idx, vel = wp.from_torch(data), wp.from_torch(idx), wp.from_torch(vel)
        self.view.set_transforms(data, idx)
        self.view.set_velocities(vel, idx)
        self._set_joint(self.LOCK_PATHS[(i, kind)], True)
        self.locked[i] = kind
        d = self.d.get((4, i), 0.0)
        self.events.append((round(self.t, 2), f"lock_{kind}", f"tip {'ABCDEFGH'[i]}4, depth {d:.2f} mm, peak load {self.meas_peak[i]:.2f} N"))
        print(f"[tipkit] t {self.t:.2f} s: tip {'ABCDEFGH'[i]}4 locked {kind} (depth {d:.2f} mm, peak collar load "
              f"{self.meas_peak[i]:.2f} N, peak total {self.total_peak:.1f} N)", flush=True)

    def snapshot(self, poses=None):
        from tip_rules import Snapshot

        poses = poses or self._poses()
        pp, pr = poses[0]
        ap = pr[:, 2]
        tips = []
        for i in range(8):
            tp, tr = poses[2 + i]
            end = tp - tr[:, 2] * (L.TIP_LEN * MM)
            below = float((pp - end) @ ap) / MM
            want = pp + pr @ _lock_local(i, "full")
            on = self.locked[i] is not None and float(np.linalg.norm((tp - want) - ap * float((tp - want) @ ap))) < 0.002
            tips.append((on, below, float(end[2])))
        pc = _arr(self.pip_contacts.get_contact_force_matrix(1.0 / 240.0))[0]  # (4, 3): left arm's two tips, right's two
        touch = np.linalg.norm(pc, axis=1) > 0.05
        in_grip = bool((touch[0] and touch[1]) or (touch[2] and touch[3]))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(ap[2])))))
        bp, br = poses[1]
        fails = set(self.flags)
        # the box against its marks
        for sx in (-1, 1):
            for sy in (-1, 1):
                local = np.array([sx * BOX_L / 2, sy * BOX_W / 2, 0.0])
                now = bp + br @ local
                mark = np.array([BOX_XY[0], BOX_XY[1], TOP]) + local
                if np.linalg.norm(now[:2] - mark[:2]) > 0.005:
                    fails.add("box_moved")
                if now[2] - TOP > 0.005:
                    fails.add("corner_lifted")
        if in_grip:
            self.held_once = True
        z = float(pp[2])
        if self.prev_pip_z is not None and self.held_once and not in_grip and (z - self.prev_pip_z) / 0.05 < -0.3:
            fails.add("dropped")
        self.prev_pip_z = z
        low = [pp + pr @ np.array([0.0, y, -L.NOZZLE_LEN * MM]) for y in ROW_Y] + [
            poses[2 + i][0] - poses[2 + i][1][:, 2] * (L.TIP_LEN * MM) for i in range(8) if self.locked[i]]
        if min(p[2] for p in low) < TOP + 0.001:
            fails.add("table")
        for k in fails - self.flags:
            self._flag(k, "seen at a control step")
        if DEBUG:  # development only: what the pipette and the box are touching
            dbg = np.linalg.norm(_arr(self.dbg_contacts.get_contact_force_matrix(1.0 / 240.0))[0], axis=1)
            bl = np.linalg.norm(_arr(self.box_links.get_contact_force_matrix(1.0 / 240.0))[0], axis=1)
            if bl.max() > 0.5:
                print(f"[tipkit] t {self.t:.2f} s: box touched by " + ", ".join(f"{n} {v:.1f} N" for n, v in zip(self.link_names, bl) if v > 0.5), flush=True)
            if dbg.max() > 2.0:
                print(f"[tipkit] t {self.t:.2f} s: pipette contact box {dbg[0]:.1f} N, tips {np.round(dbg[1:], 1).tolist()}", flush=True)
        if self.trace_path:
            self.trace.append({"t": round(self.t, 3), "total": round(self.total, 2), "rack": round(self.rack, 2),
                               "meas": np.round(self.meas, 2).tolist(), "d": {f"{k[0]}{'ABCDEFGH'[k[1]]}": round(v, 2) for k, v in self.d.items()},
                               "locked": self.locked, "pip": np.round(pp, 4).tolist(), "tilt": round(tilt, 2),
                               "grip": in_grip, "box": np.round(bp, 4).tolist(), "fails": sorted(fails)})
            if len(self.trace) % 100 == 0:
                self.save()
        return Snapshot(tips=tips, in_gripper=in_grip, tilt_deg=tilt, rim_z=float(bp[2]) + RIM_H, fails=fails)

    def summary(self) -> dict:
        return {"total_peak_N": round(self.total_peak, 2), "meas_peak_N": np.round(self.meas_peak, 2).tolist(),
                "depth_mm": {f"{'ABCDEFGH'[k[1]]}{k[0]}": round(v, 2) for k, v in self.d.items()},
                "locked": self.locked, "flags": sorted(self.flags), "events": self.events}

    def save(self) -> None:
        with open(f"{self.trace_path}.{self.trace_n}.json", "w") as f:
            json.dump({"summary": self.summary(), "trace": self.trace}, f)
