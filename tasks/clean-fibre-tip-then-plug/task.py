"""On the station table: a loose yellow fibre patch cord (6 mm jacket, 40 cm) lies straight across the table from left
to right, 22 cm in front of the robot. Its right end carries a blue SC/UPC connector (rigid body 55 x 14 x 12 mm)
lying flat and pointing right; its left end a green one. On the right stand, side by side and facing left, an
upright fibre cleaning card (a white lint-free pad 150 x 40 mm on a grey holder block) and a fibre adapter (a grey
sleeve with a 19 x 17 mm socket, 26 mm deep, on a dark panel block); their front faces lie in one plane. The robot
must wipe the blue connector's FRONT END FACE across the pad in one continuous stroke of at least 4 cm, then push
the connector at least 15 mm into the adapter socket without the end face touching anything else in between, and
let go. Inserting it without a valid wipe first scores 0.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var FIBRE_DEMO): correct (default) | nothing | direct | drag | hover | short | hold.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Before connecting the yellow fibre, clean its blue connector: wipe the connector's front end face "
               "across the white cleaning pad in one stroke of at least 4 cm, keep that end face from touching "
               "anything else, then push the connector fully into the grey adapter socket and let go.")

YELLOW = (0.95, 0.80, 0.08)
SC_BLUE = (0.10, 0.35, 0.85)
WHITE = (0.96, 0.96, 0.94)
GREY = (0.55, 0.57, 0.60)
DARK = (0.22, 0.23, 0.25)
SLEEVE = (0.40, 0.41, 0.43)

# -- fibre patch cord (free): blue SC/UPC connector on the end to connect, green SC/APC connector on the other end ---
PL, PW, PH = 0.055, 0.014, 0.012  # connector: length along the cord, width, height
R = 0.003  # 6 mm ruggedised jacket
CORD_LEN = 0.40
CORD_X = 0.22
START_Y = 0.25
SPAWN_Z = TOP + PH / 2 + 0.0005  # cord and connector centre height at spawn (connectors rest on the table)
SC_GREEN = (0.10, 0.62, 0.25)
CORD = cable("fibre", start=(CORD_X, START_Y, SPAWN_Z), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
             anchor_start=False, start_plug=Plug(size=(PL, PW, PH), mass=0.010, color=SC_GREEN),
             end_plug=Plug(size=(PL, PW, PH), mass=0.010, color=SC_BLUE), color=YELLOW,
             bend_stiffness=0.01, bend_damping=0.001)

# The cleaning card and the adapter stand side by side on the right; both front faces lie in the plane y = FACE_Y and
# face left (+y), towards the connector, which starts pointing right (-y).
FACE_Y = -0.235

# -- cleaning card: white pad (front face at y = FACE_Y) on a grey holder block ----------------------------------------
PAD_X0, PAD_X1 = 0.33, 0.48
PAD_Z0, PAD_Z1 = TOP + 0.005, TOP + 0.045
PAD_T = 0.003
HOLDER_D = 0.03
PAD = Box("cleaning_pad", pos=((PAD_X0 + PAD_X1) / 2, FACE_Y - PAD_T / 2, (PAD_Z0 + PAD_Z1) / 2),
          size=(PAD_X1 - PAD_X0, PAD_T, PAD_Z1 - PAD_Z0), color=WHITE)
HOLDER = Box("pad_holder", pos=((PAD_X0 + PAD_X1) / 2, FACE_Y - PAD_T - HOLDER_D / 2, (TOP + PAD_Z1) / 2),
             size=(PAD_X1 - PAD_X0, HOLDER_D, PAD_Z1 - TOP), color=GREY)

# -- fibre adapter: a sleeve sticking out of a panel block; socket opening faces left (+y) ---------------------------
PORT_X = 0.28  # socket centre (x)
PORT_Z = TOP + 0.024  # socket centre height
CAV_W, CAV_H, CAV_D = PW + 0.005, PH + 0.005, 0.026  # socket opening (x, z) and depth (y): 19 x 17 x 26 mm
WALL = 0.004
SL_LEN = 0.040  # sleeve length along y
SL_W, SL_Z0, SL_Z1 = CAV_W + 2 * WALL, TOP, PORT_Z + CAV_H / 2 + WALL
_cz0, _cz1 = PORT_Z - CAV_H / 2, PORT_Z + CAV_H / 2
_sy = FACE_Y - SL_LEN / 2
ADAPTER = [
    Box("adapter_floor", pos=(PORT_X, _sy, (SL_Z0 + _cz0) / 2), size=(SL_W, SL_LEN, _cz0 - SL_Z0), color=SLEEVE),
    Box("adapter_roof", pos=(PORT_X, _sy, (_cz1 + SL_Z1) / 2), size=(SL_W, SL_LEN, SL_Z1 - _cz1), color=SLEEVE),
    Box("adapter_wall_f", pos=(PORT_X + (CAV_W + WALL) / 2, _sy, PORT_Z), size=(WALL, SL_LEN, CAV_H), color=SLEEVE),
    Box("adapter_wall_n", pos=(PORT_X - (CAV_W + WALL) / 2, _sy, PORT_Z), size=(WALL, SL_LEN, CAV_H), color=SLEEVE),
    Box("adapter_back", pos=(PORT_X, FACE_Y - CAV_D - (SL_LEN - CAV_D) / 2, PORT_Z),
        size=(CAV_W, SL_LEN - CAV_D, CAV_H), color=SLEEVE),
    Box("adapter_panel", pos=(PORT_X, FACE_Y - SL_LEN - 0.015, TOP + 0.03), size=(0.08, 0.03, 0.06), color=DARK),
]
OBJECTS = [CORD, PAD, HOLDER, *ADAPTER]

# Surfaces the end face must not touch after its wipe (axis-aligned boxes: (lo, hi)); the table top is separate. The
# white pad itself is clean, and the inside of the socket is where the connector belongs.
def _aabb(b: Box) -> tuple[np.ndarray, np.ndarray]:
    c, s = np.asarray(b.pos, float), np.asarray(b.size, float) / 2
    return c - s, c + s


_DIRTY_BOXES = [_aabb(b) for b in (HOLDER, *ADAPTER)]

# -- per-substep tracking of the connector's end face -----------------------------------------------------------------
TOUCH = 0.001  # m: an end-face point this close to a surface (or inside it) touches it
WIPE_GAP = 0.0015  # m: end-face centre at most this far in front of the pad surface ...
WIPE_PEN = 0.003  # ... and at most this far into it (contact)
WIPE_TILT = math.radians(20.0)  # connector axis within 20 deg of the pad normal (end face flat on the pad)
WIPE_LEN = 0.04  # m: stroke length (straight-line travel of the end-face centre over the pad, in one contact)
WIPE_GAP_S = 0.1  # s: contact gaps shorter than this don't split a stroke
DIRTY_DEPTH = 0.002  # m: end face this far into the socket counts as "inserted"
_FACE_GRID = [(s, t) for s in np.linspace(-1, 1, 5) for t in np.linspace(-1, 1, 5)]

_TR: dict = {"env": None, "art": None, "plug_i": None, "seg_i": None, "err": None, "polls": 0}


def _reset_tracker() -> None:
    _TR.update(armed=True, clean=False, dirty_insert=False, stroke=None, stroke_len=0.0, gap=0, wipes=0,
               touches=0, best_stroke=0.0, last_touch=None, ever_clean=False)


_reset_tracker()
_TR["armed"] = False


def _quat_rot(q, wxyz: bool) -> np.ndarray:
    if wxyz:
        w, x, y, z = q
    else:
        x, y, z, w = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _plug_frame():
    """Live connector pose from PhysX: (centre, axis out of the fibre through the end face, width dir, height dir)."""
    env = _TR["env"]
    if env is None:
        return None
    if _TR["art"] is None:
        base = env.unwrapped
        art = base.scene[CORD.name]
        names = list(art.body_names)
        _TR.update(art=art, plug_i=names.index("end_plug"), seg_i=names.index(f"seg_{CORD.segments - 1}"),
                   origin=(getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0]).detach().cpu()
                   .numpy().astype(np.float64))
    tf = _TR["art"].root_physx_view.get_link_transforms()
    tf = getattr(tf, "torch", tf)
    if not hasattr(tf, "detach") and not isinstance(tf, np.ndarray):
        import warp as wp

        tf = wp.to_torch(tf)
    arr = tf.detach().cpu().numpy() if hasattr(tf, "detach") else np.asarray(tf)
    arr = arr.reshape(-1, arr.shape[-1])  # (links, 7) for the single env
    o = _TR["origin"]
    c = arr[_TR["plug_i"], :3].astype(np.float64) - o
    seg = arr[_TR["seg_i"], :3].astype(np.float64) - o
    ax = c - seg
    ax /= np.linalg.norm(ax) + 1e-12
    q = arr[_TR["plug_i"], 3:7].astype(np.float64)
    rots = [_quat_rot(q, False), _quat_rot(q, True)]
    rot = max(rots, key=lambda m: float(m[:, 0] @ ax))  # convention-proof: the connector's long axis = fibre axis
    return c, rot[:, 0], rot[:, 1], rot[:, 2]


def _face_points(c, a, w, h) -> np.ndarray:
    tip = c + a * PL / 2
    return np.array([tip + w * s * PW / 2 + h * t * PH / 2 for s, t in _FACE_GRID])


def _in_socket_opening(p: np.ndarray, margin: float = 0.0) -> bool:
    return (abs(p[0] - PORT_X) < CAV_W / 2 + margin and abs(p[2] - PORT_Z) < CAV_H / 2 + margin
            and p[1] < FACE_Y + 0.0005)


def _touches_dirty(pts: np.ndarray) -> str | None:
    """Name of the first non-clean surface an end-face point touches, or None."""
    if (pts[:, 2] < TOP + TOUCH).any():
        return "table"
    for p in pts:
        if _in_socket_opening(p) or (_in_socket_opening(p, margin=0.002) and p[1] < FACE_Y - 0.002):
            continue  # inside the socket: touching its inner walls is how a connector seats
        for (lo, hi), name in zip(_DIRTY_BOXES, ("pad_holder", *[b.name for b in ADAPTER])):
            d = np.maximum(np.maximum(lo - p, p - hi), 0.0)
            if float(np.linalg.norm(d)) <= TOUCH:
                return name
    return None


def _on_pad(c, a, pts) -> bool:
    tip = c + a * PL / 2
    gap = tip[1] - FACE_Y
    return bool(-WIPE_PEN <= gap <= WIPE_GAP and -a[1] >= math.cos(WIPE_TILT)
                and (pts[:, 0] >= PAD_X0).all() and (pts[:, 0] <= PAD_X1).all()
                and (pts[:, 2] >= PAD_Z0).all() and (pts[:, 2] <= PAD_Z1).all())


def _track(dt: float) -> None:
    if not _TR.get("armed"):
        return
    try:
        fr = _plug_frame()
    except Exception as e:  # pragma: no cover
        if _TR["err"] is None:
            _TR["err"] = repr(e)
            print(f"[fibre] tracker error: {e!r}", flush=True)
        return
    if fr is None:
        return
    _TR["polls"] += 1
    c, a, w, h = fr
    if not np.isfinite(c).all():
        return
    pts = _face_points(c, a, w, h)
    tip = c + a * PL / 2
    hit = _touches_dirty(pts)
    if hit is not None:
        _TR["last_touch"] = hit
        if _TR["clean"]:
            _TR["touches"] += 1
            print(f"[fibre] end face touched {hit} after its wipe: no longer clean", flush=True)
        _TR["clean"] = False
    if _on_pad(c, a, pts):
        yz = tip[[0, 2]].copy()
        if _TR["stroke"] is None:
            _TR["stroke"], _TR["stroke_len"] = yz, 0.0
        _TR["stroke_len"] = max(_TR["stroke_len"], float(np.linalg.norm(yz - _TR["stroke"])))
        _TR["best_stroke"] = max(_TR["best_stroke"], _TR["stroke_len"])
        _TR["gap"] = 0
        if _TR["stroke_len"] >= WIPE_LEN and hit is None and not _TR["clean"]:
            _TR["clean"], _TR["ever_clean"] = True, True
            _TR["wipes"] += 1
            print(f"[fibre] valid wipe ({_TR['stroke_len'] * 1000:.0f} mm)", flush=True)
    else:
        _TR["gap"] += 1
        if _TR["gap"] * dt > WIPE_GAP_S:
            _TR["stroke"], _TR["stroke_len"] = None, 0.0
    if _in_socket_opening(tip) and tip[1] < FACE_Y - DIRTY_DEPTH and not _TR["clean"] and not _TR["dirty_insert"]:
        _TR["dirty_insert"] = True
        print("[fibre] connector entered the socket without a valid wipe", flush=True)


_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"


def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr.step, "_fibre_hooked", False):
        return
    base_step = mgr.step.__func__

    def step(cls) -> None:
        base_step(cls)
        try:
            dt = float(cls.get_physics_dt())
        except Exception:  # noqa: BLE001
            dt = 1.0 / 240.0
        _track(dt)

    step._fibre_hooked = True
    mgr.step = classmethod(step)


class _MgrFinder(importlib.abc.MetaPathFinder):
    """Patches Isaac Lab's PhysxManager when it is first imported (it needs Kit, so not at task import)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


def _install_hooks() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_fibre_hooked", False):
        base_reset = cls.reset

        def reset(self, *a, **k):
            _TR["armed"] = False
            out = base_reset(self, *a, **k)
            _TR.update(env=self._env, art=None)
            _reset_tracker()  # new episode: no wipe yet
            return out

        reset._fibre_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_fibre_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            if _TR["env"] is not self._env:
                _TR.update(env=self._env, art=None)
            return base(self, *a, **k)

        _task_step._fibre_hooked = True
        cls._task_step = _task_step


_install_hooks()

# -- scoring -------------------------------------------------------------------------------------------------------
SEATED_MIN = 0.015  # m: end face at least 15 mm inside the socket
RELEASE_DIST = 0.04  # m: both grippers' TCP at least this far from the connector centre = let go


def _plug(objects) -> tuple[np.ndarray, np.ndarray]:
    st = objects[CORD.name]
    p = np.asarray(st.end_plug, dtype=float)
    a = p - np.asarray(st.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def depth(objects) -> float:
    """How far (m) the connector's end face is inside the socket; -1 if it is not lined up in the socket."""
    p, a = _plug(objects)
    if not np.isfinite(p).all():
        return -1.0
    if abs(p[0] - PORT_X) > 0.006 or abs(p[2] - PORT_Z) > 0.006 or -a[1] < 0.9:
        return -1.0
    return float(FACE_Y - (p[1] - PL / 2))


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP; [] if unknown (offline)."""
    env = _TR["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    out = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = (getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0] - origin).detach().cpu().numpy()
        g = pos[names.index("gripper")].astype(np.float64)
        mid = (pos[names.index("tip_left")] + pos[names.index("tip_right")]).astype(np.float64) / 2
        ax = (mid - g) / (np.linalg.norm(mid - g) + 1e-12)
        out.append(g + ax * 0.134)
    return out


def released(objects) -> bool:
    p, _ = _plug(objects)
    return all(float(np.linalg.norm(p - g)) >= RELEASE_DIST for g in grippers())


def clean() -> bool:
    """A valid wipe happened and the end face has touched nothing but the pad / socket inside since."""
    return bool(_TR.get("clean"))


def score(objects) -> float:
    """0 if: the connector entered the socket (end face > 2 mm in) before a valid wipe; the end face touched anything
    other than the pad or the socket inside after its last valid wipe (or no valid wipe at all); a gripper TCP is
    still within 4 cm of the connector at the end. Otherwise 1.0 if the end face is >= 15 mm inside the socket, lined
    up with it; else 0.5 (clean, wiped connector, not seated)."""
    if _TR.get("dirty_insert") or not clean() or not released(objects):
        return 0.0
    return 1.0 if depth(objects) >= SEATED_MIN else 0.5


def success(objects) -> bool:
    return bool(score(objects) == 1.0 and np.linalg.norm(objects[CORD.name].lin_vel) < 0.02)


def distance(objects) -> float:
    """1 m until a clean wipe exists; then how far (m) the end face still is from 15 mm deep."""
    if _TR.get("dirty_insert") or not clean():
        return 1.0
    d = depth(objects)
    return float(max(0.0, SEATED_MIN - d) if d > -1 else 0.1)


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------
# The right arm pinches the connector body 10 mm from its fibre end, sets it down a little further forward and picks
# it up again with the wrist turned the other way (the first grasp is the only one the wrist can reach where the
# connector starts; the second puts the wrist camera on the side the connector points to, so it sees the pad and the
# adapter). The left arm then lifts the fibre 25 cm behind the connector and holds it up beside the work area, so
# the trailing cord neither drags across the pad and socket nor tugs the connector round in the right gripper. The
# right arm presses the end face flat on the pad and wipes it 7.5 cm forward in one stroke, backs off, lines the
# connector up with the socket (closed loop on the measured connector pose) and pushes it in 22 mm. Both arms let go.

FINGER_PINCH = -0.020  # pads ~44 mm apart
GRASP_OPEN = -0.014  # pads ~28 mm apart round the fibre
GRASP_BACK = 0.010  # pinch this far from the connector's fibre end
HOLD_DZ = PH / 2 + 0.003  # TCP above the connector centre while pinching
WORK_Z = PORT_Z  # connector centre height while wiping / inserting
SEAT_TO = 0.022
HI = TOP + 0.07
YAW = -math.pi / 2  # right gripper yaw while working: fingers close along x, wrist camera on the -y side (faces
# the pad / adapter the connector points at)
PICK_YAW = math.pi / 2  # first pick only: YAW is out of the wrist's range at the connector's start
REGRASP_AT = (0.31, -0.18)  # connector centre when it is set down for the regrasp (between adapter and pad)
LYAW = -math.pi / 2  # left gripper yaw (fibre runs along y)


def _dbg(ctx, tag=""):
    st = ctx.states()
    p, a = _plug(st)
    print(f"[t={ctx.t:5.2f}] {tag} plug={np.round(p, 4).tolist()} ax={np.round(a, 3).tolist()} "
          f"depth={depth(st) * 1000:.1f}mm clean={clean()} dirty_ins={_TR.get('dirty_insert')} "
          f"stroke={_TR.get('stroke_len', 0) * 1000:.0f}mm best={_TR.get('best_stroke', 0) * 1000:.0f}mm "
          f"wipes={_TR.get('wipes')} touches={_TR.get('touches')} last_touch={_TR.get('last_touch')} "
          f"polls={_TR.get('polls')} rel={released(st)} score={score(st)} "
          f"tcp={[np.round(g, 3).tolist() for g in grippers()]}", flush=True)


def _tip(st) -> np.ndarray:
    p, a = _plug(st)
    return p + a * PL / 2


def _right_pick(R_, ctx):
    """Pick the connector (wrist turned PICK_YAW, the only way to reach it where it starts), set it down at REGRASP_AT,
    turn the wrist to YAW and pick it up again, so the wrist camera looks where the connector points. The end face
    has not been wiped yet, so setting the connector down costs nothing."""
    from yam_arena.constants import FINGER_CLOSED

    p, a = _plug(ctx.states())
    g = p - a * (PL / 2 - GRASP_BACK)
    yield from R_.move((g[0], g[1], HI), yaw=PICK_YAW, finger=FINGER_PINCH, duration=1.5)
    yield from R_.move((g[0], g[1], p[2] + HOLD_DZ), yaw=PICK_YAW, duration=1.0)
    yield from R_.grip(FINGER_CLOSED, 0.6)
    yield from R_.move((g[0], g[1], TOP + 0.05), yaw=PICK_YAW, duration=0.8)
    d = np.array([REGRASP_AT[0], REGRASP_AT[1], 0.0]) - np.array([p[0], p[1], 0.0])
    yield from R_.move((g[0] + d[0], g[1] + d[1], TOP + 0.05), yaw=PICK_YAW, duration=1.2)
    yield from R_.move((g[0] + d[0], g[1] + d[1], TOP + PH / 2 + HOLD_DZ + 0.002), yaw=PICK_YAW, duration=0.8)
    yield from R_.grip(FINGER_PINCH, 0.5)
    yield from R_.move((g[0] + d[0], g[1] + d[1], TOP + 0.08), yaw=PICK_YAW, duration=0.6)
    yield from R_.move((g[0] + d[0], g[1] + d[1], TOP + 0.08), yaw=YAW, duration=1.2)
    p, a = _plug(ctx.states())  # regrasp where and how it actually lies (the cord may have turned it a little)
    g = p - a * (PL / 2 - GRASP_BACK)
    gy = float(np.clip(math.atan2(a[1], a[0]), YAW - 0.05, YAW + 0.6))  # grip across the connector, as it lies
    print(f"[demo] regrasp: connector heading {math.degrees(math.atan2(a[1], a[0])):.1f} deg, grip yaw "
          f"{math.degrees(gy):.1f} deg", flush=True)
    yield from R_.move((g[0], g[1], TOP + 0.08), yaw=gy, duration=0.8)
    yield from R_.move((g[0], g[1], p[2] + HOLD_DZ), yaw=gy, duration=1.0)
    yield from R_.grip(FINGER_CLOSED, 0.6)
    yield from R_.move((g[0], g[1], WORK_Z + HOLD_DZ + 0.01), yaw=gy, duration=0.8)
    yield from R_.move((g[0], g[1], WORK_Z + HOLD_DZ + 0.01), yaw=YAW, duration=0.8)  # straighten: point it at -y


def _left_support(L, st):
    from yam_arena.constants import FINGER_CLOSED

    pts = st[CORD.name].points
    i = int(np.argmin(np.abs(pts[:, 1] - 0.10)))  # 25 cm behind the blue connector
    q = pts[i]
    yield from L.move((q[0], q[1], HI), yaw=LYAW, finger=GRASP_OPEN, duration=1.5)
    yield from L.move((q[0], q[1], q[2] + 0.010), yaw=LYAW, duration=1.0)
    yield from L.grip(FINGER_CLOSED, 0.5)
    yield from L.move((q[0], q[1], TOP + 0.05), yaw=LYAW, duration=0.8)
    yield from L.move((0.32, 0.0, TOP + 0.05), yaw=LYAW, duration=1.2)


def _servo_tip(ctx, R_, target_tip, iters=3, tol=0.0007, duration=0.5, axes=(0, 1, 2)):
    """Move the right TCP so the measured end-face centre reaches ``target_tip`` (the arm sags under load and the
    connector may sit a little off in the fingers). Returns the commanded TCP."""
    off = _tip(ctx.states()) - R_.tcp()  # end face relative to the TCP command
    cmd = np.asarray(target_tip, float) - off
    yield from R_.move(cmd, yaw=YAW, duration=duration)
    for _ in range(iters):
        yield from R_.wait(0.25)
        err = np.asarray(target_tip, float) - _tip(ctx.states())
        e = np.zeros(3)
        e[list(axes)] = err[list(axes)]
        if np.abs(e).max() < tol:
            break
        cmd = cmd + np.clip(e, -0.01, 0.01)
        yield from R_.move(cmd, yaw=YAW, duration=0.4)
    return cmd


def _wipe(ctx, R_, x0=0.36, length=0.075, standoff=0.0, press=0.001):
    """End face flat on the pad at x0, then one stroke of ``length`` along +x pressing ``press`` into the pad
    (``standoff`` > 0 keeps it that far off the pad instead: a hover, test only)."""
    yield from _servo_tip(ctx, R_, (x0, FACE_Y + 0.015, WORK_Z), duration=1.5)
    _dbg(ctx, "pre-wipe")
    y = FACE_Y + standoff - (press if standoff <= 0 else 0.0)
    cmd = yield from _servo_tip(ctx, R_, (x0, y, WORK_Z), axes=(0, 2), duration=0.8)
    _dbg(ctx, "on pad")
    end = cmd + np.array([length, 0.0, 0.0])
    yield from R_.move(end, yaw=YAW, duration=2.5)
    _dbg(ctx, "wiped")
    back = end + np.array([0.0, 0.02, 0.0])
    yield from R_.move(back, yaw=YAW, duration=0.6)
    return back


def _insert(ctx, R_):
    yield from _servo_tip(ctx, R_, (PORT_X, FACE_Y + 0.02, PORT_Z), duration=2.0)
    cmd = yield from _servo_tip(ctx, R_, (PORT_X, FACE_Y + 0.006, PORT_Z), iters=4, tol=0.0005, duration=0.5)
    _dbg(ctx, "lined up")
    cmd = cmd + np.array([0.0, -(SEAT_TO + 0.006), 0.0])
    yield from R_.move(cmd, yaw=YAW, duration=2.0)
    for _ in range(2):  # top up if the connector slipped back in the fingers
        d = depth(ctx.states())
        if d >= SEAT_TO - 0.002:
            break
        cmd = cmd + np.array([0.0, -(SEAT_TO - max(d, 0.0)), 0.0])
        yield from R_.move(cmd, yaw=YAW, duration=1.0)
    yield from R_.wait(0.3)
    _dbg(ctx, "seated")
    return cmd


def _right_release(R_, cmd):
    yield from R_.grip(FINGER_PINCH, 0.6)
    yield from R_.move(cmd + np.array([0.0, 0.01, 0.06]), yaw=YAW, duration=1.0)
    yield from R_.move((0.30, -0.17, TOP + 0.14), yaw=YAW, duration=1.2)


def _left_release(L):
    yield from L.move((0.32, 0.0, TOP + 0.015), yaw=LYAW, duration=1.0)
    yield from L.grip(GRASP_OPEN, 0.5)
    yield from L.move((0.22, 0.15, TOP + 0.14), yaw=LYAW, duration=1.2)


def demo_script(ctx):
    """FIBRE_DEMO (test only): correct (default) | nothing | direct (insert without wiping) | drag (wipe, then
    slide the end face 3 cm along the table, then insert) | hover (the stroke 4 mm off the pad) | short (a 2.5 cm
    stroke) | hold (everything right but never let go)."""
    mode = os.environ.get("FIBRE_DEMO", "correct")
    L, R_ = ctx.left, ctx.right
    _dbg(ctx, "start")
    if mode == "nothing":
        for _ in range(5):
            yield from L.wait(1.0)
        _dbg(ctx, "end")
        return
    st = ctx.states()
    yield from _right_pick(R_, ctx)  # the left arm waits: the cord lying slack keeps the set-down connector straight
    yield from _left_support(L, ctx.states())
    _dbg(ctx, "picked")
    if mode != "direct":
        kw = {"hover": dict(standoff=0.004), "short": dict(length=0.025)}.get(mode, {})
        cmd = yield from _wipe(ctx, R_, **kw)
        if mode == "drag":
            tip = _tip(ctx.states())
            down = cmd + np.array([-0.06, 0.02, TOP + PH / 2 - (tip[2])])
            yield from R_.move(down + np.array([0, 0, 0.01]), yaw=YAW, duration=1.0)
            yield from R_.move(down, yaw=YAW, duration=0.6)
            yield from R_.move(down + np.array([-0.03, 0, 0]), yaw=YAW, duration=1.0)
            _dbg(ctx, "dragged on table")
            yield from R_.move(down + np.array([-0.03, 0, 0.02]), yaw=YAW, duration=0.6)
    cmd = yield from _insert(ctx, R_)
    if mode == "hold":
        yield from L.wait(3.0)
        _dbg(ctx, "end")
        return
    yield from ctx.both(_right_release(R_, cmd), _left_release(L))
    yield from L.wait(1.0)
    _dbg(ctx, "end")
