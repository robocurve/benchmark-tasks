"""Bedside tray table (the station table). A wrapped sterile pack lies in front of the robot: a stainless instrument
tray (15 x 20 x 4 cm, 0.35 kg) envelope-wrapped in one blue Halyard H300 sterilization wrap sheet (45 x 45 cm, SMS
polypropylene). The sheet lies as a diamond under the tray: its near corner was folded over the tray first, then the
left corner, then the right, then the far corner last, so the far flap lies on top. Every flap's tip is folded back
into a small tab near the middle of the pack.

Sterile technique (nursing fundamentals): the outside of the pack and the outer 2.5 cm border of the wrap are not
sterile; the inside face of the wrap and the tray are. Open the far flap first, away from you; then each side flap
with the hand on that side; then the near flap toward you, last. Touch the wrap only on its outside or its border,
never the tray, and never let a hand pass over the opened field.

The wrap is a reduced position-based sheet (wrap.py), stepped every control step from what the robot does: TCP
poses, jaw openings, the gripper collision boxes and the tray pose, all measured in the sim. A jaw closing with a
border point of the top ply between its pads pinches that ply.

Score (see score()): 0 if a gripper touches the sterile field (the inner part, more than 2.5 cm in from the wrap's
edge, of a flap that has been lifted or lies open, or the tray itself), any part of either robot (gripper, finger pads,
arm links) passes vertically over the field (an opened, let-go flap's inner part, or any part of the tray top no longer
covered by the wrap), a side flap is pinched by the hand of the other side, a flap opens out of order (far first; near
last), or the tray is pushed more than 3 cm. Otherwise 0.2 per flap now lying open (tip on the table at least 5 cm
outside the tray, no more than 3% of the flap over the tray more than 1 cm in from its edges);
1.0 once all four lie open, the tray top is uncovered, both jaws are open, nothing is held and both hands are at least
10 cm from the tray and the wrap, still (under 2 cm/s) for at least SUCCESS_HOLD_S.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import math
import os
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import wrap as wr  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.5

INSTRUCTION = "Open the sterile pack by its border: open the far flap first, each side flap with its same-side hand, then the near flap, without touching or passing over the exposed sterile field; release the wrap and move both hands clear."

CENTER = np.array([0.30, 0.0])                 # tray centre (the wrap start state is built for this pose)
TRAY_MASS = 0.35
STEEL = (0.72, 0.73, 0.75)
WRAP_COL = (0.30, 0.52, 0.80)                  # H300 blue

TRAY = Box("instrument_tray", pos=(float(CENTER[0]), float(CENTER[1]), float(TOP + wr.TRAY[2] / 2)),
           size=tuple(float(v) for v in wr.TRAY), color=STEEL, mass=TRAY_MASS)
OBJECTS = [TRAY]

TWIN_EXTRA = [
    {"name": "sterilization_wrap", "kind": "body", "real": "Halyard H300 sequential sterilization wrap 18 x 18 in",
     "size_mm": [450, 450, 1], "colour": "blue", "pose": [0.30, 0.0, TOP]},
]

# -- rules (m) ------------------------------------------------------------------------------------------------------
OPEN_OUT = 0.05        # an open flap's tip lies at least this far outside the tray
OPEN_TIP_Z = 0.03      # ... and this low (above the table)
OPEN_OVER = 0.03       # ... and no more than this fraction of the flap lies over the tray
OVER_INSET = 0.01      # ('over the tray': more than this in from its edges)
TAB_LEN = {"near": 0.06, "left": 0.035, "right": 0.03, "far": 0.045}   # folded-back tab of each flap
ARM_R = 0.03          # arm links: centreline points, this radius added
COVER_R = 0.02        # a tray-top point with no wrap point within this (horizontally) above it is exposed
OVER_R = 0.012         # a jaw point within this (horizontal) of a field point, above it, is over the field
TRAY_TOUCH = 0.003     # a gripper part this close to the tray touches it ...
CLOTH_BETWEEN = 0.020  # ... unless a ply of the wrap lies within this of that point (cloth between)
PUSH_MAX = 0.03
CLEAR = 0.10
STILL = 0.02
ORDER_NEED = {"far": (), "left": ("far",), "right": ("far",), "near": ("far", "left", "right")}
SIDE_ARM = {"left": 0, "right": 1}             # arm index: 0 left, 1 right

# -- gripper collision boxes for the wrap (from the URDF's convex pieces, grouped per finger part) ------------------


def _gripper_boxes_local():
    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    text = pathlib.Path(YAM_URDF).read_text()
    root = pathlib.Path(YAM_URDF).parent

    def stl(p):
        b = p.read_bytes()
        if b[:5] == b"solid" and b"facet" in b[:300]:
            return np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", b), float)
        n = int.from_bytes(b[80:84], "little")
        a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        return a["v"].reshape(-1, 3).astype(float)

    out = {}
    for link, split in (("gripper", None), ("tip_left", -0.044), ("tip_right", 0.044)):
        block = re.search(r'<link name="%s".*?</link>' % link, text, re.S).group(0)
        groups: dict[int, list] = {}
        for c in re.findall(r"<collision>.*?</collision>", block, re.S):
            xyz = np.array(re.search(r'xyz="([^"]+)"', c).group(1).split(), float)
            rpy = np.array(re.search(r'rpy="([^"]+)"', c).group(1).split(), float)
            v = stl(root / re.search(r'filename="([^"]+)"', c).group(1)) @ rpy_matrix(rpy).T + xyz
            key = 0 if split is None else int((v[:, 1].mean() < split) if split < 0 else (v[:, 1].mean() > split))
            groups.setdefault(key, []).append(v)
        out[link] = [(np.min(np.concatenate(g), 0), np.max(np.concatenate(g), 0)) for g in groups.values()]
    return out


_BOXES_LOCAL = None
_ENV: dict = {}
_GRID = np.array([[i, j, k] for i in (0, 0.5, 1) for j in (0, 0.5, 1) for k in (0, 0.5, 1)])


def _robots(env):
    base = env.unwrapped
    return [base.scene.articulations[f"{s}_robot"] for s in ("left", "right")]


def _arm_state(env):
    """Measured TCP positions, gripper rotations, jaw openings, wrap collision boxes and jaw points of both arms."""
    global _BOXES_LOCAL
    from yam_arena.runtime import link_poses, tcp_world

    if _BOXES_LOCAL is None:
        _BOXES_LOCAL = _gripper_boxes_local()
    tcp, rot, ap, cmd, boxes, jaw, allpts, armpts = [], [], [], [], [], [], [], []
    for i, r in enumerate(_robots(env)):
        lp = link_poses(r)
        tcp.append(tcp_world(r))
        rot.append(lp["gripper"][:3, :3])
        q = r.data.joint_pos.torch[0].cpu().numpy()
        names = list(r.joint_names)
        ap.append(abs(float(q[names.index("joint7")])) + abs(float(q[names.index("joint8")])))
        target_data = r.data.joint_pos_target
        target = getattr(target_data, "torch", target_data)[0].cpu().numpy()
        cmd.append(abs(float(target[names.index("joint7")])) + abs(float(target[names.index("joint8")])))
        pts = []
        for link, bl in _BOXES_LOCAL.items():
            for lo, hi in bl:
                boxes.append((lp[link], lo, hi, i))
                loc = lo + _GRID * (hi - lo)
                w = loc @ lp[link][:3, :3].T + lp[link][:3, 3]
                pts.append(w)
        half = 0.5 * ap[-1] * rot[-1][:, 0]
        jaw.append(np.array([tcp[-1], tcp[-1] + half, tcp[-1] - half]))   # TCP and the two finger pads
        allpts.append(np.concatenate(pts))
        # the arm itself: points every 2 cm along the chain of link origins, base to gripper
        org = [lp[n][:3, 3] for n in r.body_names if n not in ("tip_left", "tip_right")]
        seg = [a + (b - a) * t for a, b in zip(org[:-1], org[1:])
               for t in np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / 0.02) + 1))]
        armpts.append(np.array(seg))
    return np.array(tcp), np.array(rot), np.array(ap), np.array(cmd), boxes, jaw, allpts, armpts


def _tray_pose(env):
    from yam_arena.scene import object_states

    s = object_states(env, [TRAY.name])[TRAY.name]
    qx, qy, qz, qw = s.quat_xyzw
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return np.array([s.pos[0], s.pos[1], TOP]), yaw


# -- episode state ---------------------------------------------------------------------------------------------------
class Tracker:
    def __init__(self):
        self.w = None
        self.mesh = None
        self.reset()

    def reset(self, tray=None):
        self.w = wr.Wrap(TOP)
        x0 = np.load(HERE / "wrap_start.npz")["x"]
        if tray is None:
            tray = (np.array([CENTER[0], CENTER[1], TOP]), 0.0)
        c, yaw = tray
        cs, sn = math.cos(yaw), math.sin(yaw)
        d = x0[:, :2] - CENTER
        x = x0.copy()
        x[:, 0] = c[0] + cs * d[:, 0] - sn * d[:, 1]
        x[:, 1] = c[1] + sn * d[:, 0] + cs * d[:, 1]
        self.w.place(x, c, yaw)
        self.tray0 = np.array(c[:2])
        self.steps = 0
        self.opened_at = {}             # flap -> step it first lay open
        self.field_flaps = set()        # flaps that have lain open and been released remain sterile
        self.pinned_by = {f: set() for f in wr.FLAPS}
        self.why = ""                   # first contamination / rule break
        self._said = set()
        self.tcp = np.zeros((2, 3)) + [0, 0, 5]
        self.aperture = np.full(2, 0.09)
        self.jaw = [np.zeros((1, 3)) + [0, 0, 5]] * 2
        self.allpts = [np.zeros((1, 3)) + [0, 0, 5]] * 2
        self.prev_jaw = None
        self.speed = 0.0
        self.still_t = 0.0
        self._grid = None

    def fail(self, why):
        if not self.why:
            self.why = why
        if why not in self._said:
            self._said.add(why)
            print(f"[wrap] step {self.steps}: {why}", flush=True)

    # -- geometry -----------------------------------------------------------------------------------------------
    def open_now(self) -> dict:
        w = self.w
        u = w.to_tray(w.x)
        hx, hy = wr.TRAY[0] / 2, wr.TRAY[1] / 2
        over = (np.abs(u[:, 0]) < hx - OVER_INSET) & (np.abs(u[:, 1]) < hy - OVER_INSET)
        out = {}
        for k, f in enumerate(wr.FLAPS):
            n2 = wr.DIRS[f]
            h = hx if f in ("near", "far") else hy
            tip = u[w.corner[f]]
            m = w.flap == k
            out[f] = bool(tip[:2] @ n2 >= h + OPEN_OUT and tip[2] <= OPEN_TIP_Z and over[m].mean() <= OPEN_OVER)
        return out

    def field_points(self, is_open):
        """World points of the opened sterile field: inner parts of the opened flaps lying open."""
        w = self.w
        sel = np.zeros(w.n, bool)
        for k, f in enumerate(wr.FLAPS):
            if is_open.get(f):
                sel |= (w.flap == k)
        sel &= ~w.border
        return w.x[sel]

    def tray_uncovered(self):
        u = self.w.to_tray(self.w.x)
        on = (np.abs(u[:, 0]) < wr.TRAY[0] / 2 - 0.005) & (np.abs(u[:, 1]) < wr.TRAY[1] / 2 - 0.005) & \
             (u[:, 2] > wr.TRAY[2] - 0.01)
        return not on.any()

    def tray_exposed(self):
        """Points of the tray top (9 x 9 grid) with no ply of the wrap anywhere above them."""
        w = self.w
        if self._grid is None:
            g = np.linspace(-0.5, 0.5, 9)
            self._grid = np.array([[x * wr.TRAY[0], y * wr.TRAY[1]] for x in g for y in g])
        pos, yaw = w.tray
        c, s_ = np.cos(yaw), np.sin(yaw)
        top = np.c_[pos[0] + c * self._grid[:, 0] - s_ * self._grid[:, 1],
                    pos[1] + s_ * self._grid[:, 0] + c * self._grid[:, 1],
                    np.full(len(self._grid), pos[2] + wr.TRAY[2] / 2)]
        hi = w.x[w.x[:, 2] > top[0, 2] - 0.005]
        if not len(hi):
            return top
        d = np.linalg.norm(top[:, None, :2] - hi[None, :, :2], axis=2).min(axis=1)
        return top[d > COVER_R]

    # -- per step -------------------------------------------------------------------------------------------------
    def step(self, env):
        w = self.w
        tcp, rot, ap, cmd, boxes, jaw, allpts, armpts = _arm_state(env)
        tray = _tray_pose(env)
        self.tcp, self.aperture, self.jaw, self.allpts = tcp, ap, jaw, allpts
        touched_before = w.touched.copy()
        # only gripper parts near the cloth can touch it this step
        lo, hi = w.x.min(axis=0) - 0.15, w.x.max(axis=0) + 0.15
        boxes = [b for b in boxes if np.all((b[0][:3, 3] > lo) & (b[0][:3, 3] < hi))]
        w.step(tcp, rot, ap, boxes=boxes, tray=tray, jaw_command=cmd)
        self.steps += 1
        jp = np.concatenate(jaw)
        if self.prev_jaw is not None:
            self.speed = float(np.max(np.linalg.norm(jp - self.prev_jaw, axis=1))) * 20.0
        self.prev_jaw = jp
        # who pinched which flap
        for arm, (nodes, _off) in w.pins.items():
            for k in np.unique(w.flap[nodes]):
                if k >= 0:
                    self.pinned_by[wr.FLAPS[k]].add(arm)
        for f, arm in SIDE_ARM.items():
            if self.pinned_by[f] - {arm}:
                self.fail(f"{f} flap taken by the other side's hand (reaching across)")
        # flaps opening, in order
        is_open = self.open_now()
        for f, o in is_open.items():
            if o and f not in self.opened_at:
                self.opened_at[f] = self.steps
                miss = [g for g in ORDER_NEED[f] if g not in self.opened_at]
                if miss:
                    self.fail(f"{f} flap opened before {', '.join(miss)}")
                print(f"[wrap] step {self.steps}: {f} flap open", flush=True)
        opened = {f: (f in self.opened_at) and is_open[f] for f in wr.FLAPS}
        # a flap becomes part of the field once it lies open and no hand holds it any more
        held = {wr.FLAPS[k] for nodes, _o in w.pins.values() for k in np.unique(w.flap[nodes]) if k >= 0}
        self.field_flaps.update(f for f in self.opened_at if f not in held)
        field = {f: f in self.field_flaps for f in wr.FLAPS}
        # touching the sterile field: new gripper contacts on an opened flap's inner part, or on the tray
        new = w.touched & ~touched_before
        if new.any():
            for k, f in enumerate(wr.FLAPS):
                # a flap's inner face is sterile once it is lifted (pinched) or lies open; while a hand holds it,
                # its tab (the tip folded back, the handle) is handled too
                held_ok = w.border | (np.linalg.norm(w.ab - w.ab[w.corner[f]], axis=1) < TAB_LEN[f] + 0.01)
                inner = (w.flap == k) & ~(w.border if field[f] else held_ok)
                if (field[f] or self.pinned_by[f]) and (new & inner).any():
                    if os.environ.get("WRAP_LOG"):
                        ids = np.flatnonzero(new & inner)
                        for node in ids:
                            dxy = np.linalg.norm(w.x[:, :2] - w.x[node, :2], axis=1)
                            above = np.flatnonzero((dxy < 0.012) & (w.x[:, 2] > w.x[node, 2] + 0.0001))
                            print(f"[touch] {f} node {node} position {w.x[node].round(4).tolist()} "
                                  f"above {above.tolist()} positions {w.x[above].round(4).tolist()}", flush=True)
                    self.fail(f"gripper touched the inside of the {f} flap")
        for arm in range(2):
            d, _n = w._tray_sdf(allpts[arm])
            near = allpts[arm][d < TRAY_TOUCH]
            if len(near):
                # bare contact: no ply of the wrap between the gripper and the tray there
                gap = np.min(np.linalg.norm(near[:, None] - w.x[None], axis=2), axis=1)
                if (gap > CLOTH_BETWEEN).any():
                    self.fail(f"{'left' if arm == 0 else 'right'} gripper touched the tray")
        # passing over the sterile field: any part of either robot (gripper geometry, finger pads, arm links)
        # vertically over an opened flap's inner part, or over any part of the tray top no longer covered by the wrap
        fp = np.concatenate([self.field_points(field), self.tray_exposed()])
        for arm in range(2):
            for pts, r in ((np.concatenate([jaw[arm], allpts[arm]]), OVER_R), (armpts[arm], OVER_R + ARM_R)):
                if not len(fp):
                    continue
                dxy = np.linalg.norm(pts[:, None, :2] - fp[None, :, :2], axis=2)
                hit = (dxy < r) & (pts[:, None, 2] > fp[None, :, 2])
                if hit.any():
                    if os.environ.get("WRAP_LOG") and not self.why:
                        i, j = np.argwhere(hit)[0]
                        print(f"[over] {'arm' if r > OVER_R else 'hand'} point {pts[i].round(3)} "
                              f"field point {fp[j].round(3)}", flush=True)
                    self.fail(f"{'left' if arm == 0 else 'right'} {'arm' if r > OVER_R else 'hand'} passed "
                              f"over the opened field")
        if np.linalg.norm(tray[0][:2] - self.tray0) > PUSH_MAX:
            self.fail("tray pushed off its spot")
        self.still_t = self.still_t + 0.05 if self.speed < STILL else 0.0
        self.render()
        if os.environ.get("WRAP_LOG") and self.steps % 20 == 0:
            print(f"[wrap] step {self.steps} pins { {k: len(v[0]) for k, v in w.pins.items()} } ap {ap.round(3)} "
                  f"open {[f for f in wr.FLAPS if opened[f]]} strain {w.peak_strain:.2f} why '{self.why}' "
                  f"tcpL {tcp[0].round(3)} tcpR {tcp[1].round(3)}", flush=True)

    def render(self):
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
        import omni.usd

        w = self.w
        if self.mesh is None or not self.mesh.GetPrim().IsValid():
            stage = omni.usd.get_context().get_stage()
            m = UsdGeom.Mesh.Define(stage, "/World/SterileWrap")
            m.CreateDoubleSidedAttr(True)
            m.CreateSubdivisionSchemeAttr("none")
            m.CreateFaceVertexCountsAttr([3] * len(w.faces))
            m.CreateFaceVertexIndicesAttr(w.faces.ravel().tolist())
            mat = UsdShade.Material.Define(stage, "/World/SterileWrapMat")
            sh = UsdShade.Shader.Define(stage, "/World/SterileWrapMat/Shader")
            sh.CreateIdAttr("UsdPreviewSurface")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*WRAP_COL))
            sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.85)
            mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
            self.mesh = m
        self.mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(w.x.astype(np.float32)))


TRACKER = Tracker()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_sterile_wrap_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            obs = orig(self, *a, **k)
            if getattr(getattr(self, "task", None), "module", None) is None or \
                    getattr(self.task.module, "INSTRUCTION", None) != INSTRUCTION:
                return obs
            _ENV["env"] = self._env
            TRACKER.reset(_tray_pose(self._env))
            TRACKER.render()
            for _ in range(4):
                self._base.sim.render()
            for _ in range(2):
                self._env.step(self._ready)
                self._base.sim.render()
            return self._observation()

        reset._sterile_wrap_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_sterile_wrap_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            if getattr(self.task.module, "INSTRUCTION", None) == INSTRUCTION:
                _ENV["env"] = self._env
                TRACKER.step(self._env)
            return base(self, *a, **k)

        _task_step._sterile_wrap_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- scoring ---------------------------------------------------------------------------------------------------------
def parts() -> dict:
    t = TRACKER
    w = t.w
    if w is None or w.bad or not np.isfinite(w.x).all():
        return {"valid": False}
    is_open = t.open_now()
    opened = {f: (f in t.opened_at) and is_open[f] for f in wr.FLAPS}
    pts = np.concatenate(t.allpts)
    d_tray, _ = w._tray_sdf(pts)
    d_wrap = float(np.min(np.linalg.norm(w.x[None, ::2] - pts[::4, None], axis=2)))
    clear = min(float(d_tray.min()), d_wrap)
    released = bool(not w.pins and (t.aperture > wr.PINCH_OPEN).all())
    return {"valid": True, "why": t.why, "opened": opened, "n_open": sum(opened.values()),
            "uncovered": t.tray_uncovered(), "released": released, "clear": clear, "still": t.still_t}


def score(objects=None) -> float:
    p = parts()
    if not p["valid"] or p["why"]:
        return 0.0
    if p["n_open"] == 4 and p["uncovered"] and p["released"] and p["clear"] >= CLEAR and p["still"] >= SUCCESS_HOLD_S:
        return 1.0
    return round(0.2 * p["n_open"], 3)


def success(objects=None) -> bool:
    return score(objects) >= 1.0


def progress(objects=None) -> dict:
    p = parts()
    if not p["valid"]:
        return {"valid": 0}
    return {"n_open": p["n_open"], "uncovered": int(p["uncovered"]), "released": int(p["released"]),
            "clear": round(p["clear"], 3), "why": p["why"]}


VARIATION = {}
