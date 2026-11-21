"""On the station table: six bare aluminium blocks, each 60 mm long, 20 mm tall and about 40 mm wide, lie in two
rows of three, one row in front of each arm. Between the arms, further out, a fixed steel go/no-go gap gauge: a base
plate with two gaps across it, each between a pair of anvils; the anvils of the GO gap (to the robot's left) are
painted green, those of the NO-GO gap (to the right) red; each anvil has a small lead-in chamfer on its inner top
edge. Nearer the robot, two open trays: green (accept) on the left, red (reject) on the right. The blocks differ in
width by a few tenths of a millimetre to about 5 mm; their letters are marked on their undersides.

Run it (from this folder; GAUGE_SORT_LAYOUT picks which block starts where):
  GAUGE_SORT_LAYOUT=0 inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 \\
    -E control_hz=20 --instruction "..." --scorer task_score --max-steps 9600 ...
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import json
import math
import os
import sys
import warnings

import numpy as np

from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, PAD_TIP_PAST_TCP, PHYSICS_DT, TCP_OFFSET
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.urdf import quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20  # run with -E control_hz=20
RUN_LIMIT_STEPS = 9600  # 8 min; the demo plan plus its 2 s hold is about half of this
G = 9.81

# -- blocks -----------------------------------------------------------------------------------------------------------
BLOCK_LEN, BLOCK_H = 0.060, 0.020
DENSITY = 2700.0  # aluminium, kg/m^3
NOMINAL, TOLERANCE = 0.040, 0.0013  # specified width 40.0 +- 1.3 mm
WIDTHS = {"a": 0.0393, "b": 0.0395, "c": 0.0397, "d": 0.0371, "e": 0.0368, "f": 0.0419}
ALUMINIUM = (0.78, 0.79, 0.80)

# -- gauge: base plate, then per gap two anvils 10 mm long (x) so the fingers on the block's ends clear them --------
GAUGE_X = 0.32
GAPS = {"go": 0.0413, "nogo": 0.0387}  # distance between the anvils' inner faces: the two limits of the tolerance
GAP_Y = {"go": 0.045, "nogo": -0.045}
# Tightest margins the simulated demo handles reliably (probe runs): dropping in with 0.75 mm a side worked 12 of 12
# times and 0.6 mm 2 of 6; staying out with 0.3 mm of overlap worked 6 of 6 (the block settles between the chamfers).
DROP_CLEAR = 0.0008  # a block that must drop into a gap clears it by at least this a side, when centred
STAY_OVERLAP = 0.0003  # a block that must stay out overlaps each anvil's inner face by at least this, when centred
BASE_SIZE = (0.050, 0.170, 0.010)
ANVIL_SIZE = (0.010, 0.012, 0.015)
BASE_TOP = TOP + BASE_SIZE[2]
ANVIL_TOP = BASE_TOP + ANVIL_SIZE[2]
CHAMFER = 0.002  # 45 degree lead-in on each anvil's inner top edge: a block set slightly off centre slides into the gap
# steel gauge (static, dynamic friction); averaged with the blocks' it is below tan 45 deg, so blocks slide on a bevel
GAUGE_FRICTION = (0.4, 0.3)
STEEL, GREEN, RED = (0.55, 0.56, 0.58), (0.15, 0.55, 0.22), (0.75, 0.13, 0.12)

# -- trays: inner 120 x 160 mm, 4 mm floor, 30 mm walls 6 mm thick -----------------------------------------------------
TRAY_X = 0.17
TRAY_Y = {"green": 0.09, "red": -0.09}
TRAY_INNER = (0.120, 0.160)
TRAY_FLOOR, TRAY_WALL, TRAY_WALL_H = 0.004, 0.006, 0.030
TRAY_FLOOR_TOP = TOP + TRAY_FLOOR

# -- start places: one row of three in front of each arm --------------------------------------------------------------
START_XY = [(0.18, 0.23), (0.28, 0.23), (0.38, 0.23), (0.18, -0.23), (0.28, -0.23), (0.38, -0.23)]

# -- scoring constants ------------------------------------------------------------------------------------------------
CHECK_STEPS = 6  # a check must hold this many control steps in a row (0.3 s at 20 Hz)
SUPPORT_FRAC = 0.8  # vertical force from the gauge on the block, as a fraction of its weight (calibrate from logs)
ROBOT_TOUCH_N = 0.02  # most contact force any robot link may put on a block during a check (calibrate from logs)
RELEASE_RADIUS = 0.050  # a gripper whose TCP is this close to the block must be open...
RELEASE_OPEN = BLOCK_LEN + 0.005  # ...with its pads further apart than this
X_TOL = 0.012  # block centre to gap centre along the gap (keeps the anvils under the block's middle)
YAW_TOL = math.radians(10)  # block's long axis to world x
LEVEL_TOL = math.radians(3)
Z_TOL = 0.0015  # bottom to the face it rests on
FIT_TOL = 0.0005
OUT_TILT = math.radians(30)  # most a block resting on the anvils may be tipped (it tips if let go off centre)
START_TOL = 0.010  # backup wipe only (the embodiment-reset hook is the main one); trials use -E object_jitter=0
GAUGE_FZ_SIGN = 1.0  # flip if the contact view reports forces on the filter instead of on the sensor
LOG_PATH = os.environ.get("GAUGE_SORT_LOG")  # JSONL of every check evaluated near the gauge
PROBE = os.environ.get("GAUGE_SORT_PROBE")  # "wedge": demo() only wedges block F in GO, for force calibration


def block_mass(width: float) -> float:
    return DENSITY * BLOCK_LEN * width * BLOCK_H


def in_tolerance(width: float) -> bool:
    return abs(width - NOMINAL) <= TOLERANCE


def drops_in(width: float, gap: str) -> bool:
    return width < GAPS[gap]


def check_geometry() -> None:
    for letter, w in WIDTHS.items():
        for gap, g in GAPS.items():
            margin = DROP_CLEAR if drops_in(w, gap) else STAY_OVERLAP
            assert abs(w - g) / 2 >= margin - 1e-9, (letter, gap)
        assert w + 0.020 < 0.094 and BLOCK_LEN + 0.010 < 0.094  # fits the 94 mm jaws
    assert {k for k, w in WIDTHS.items() if in_tolerance(w)} == {"a", "b", "c"}
    # go/no-go agrees with the tolerance: in tolerance <=> drops into GO and stays out of NO-GO
    assert all(in_tolerance(w) == (drops_in(w, "go") and not drops_in(w, "nogo")) for w in WIDTHS.values())
    # the block's ends (where the fingers hold it) overhang the base plate and the anvils
    assert BLOCK_LEN / 2 - BASE_SIZE[0] / 2 >= 0.005 - 1e-9 and BASE_SIZE[0] > ANVIL_SIZE[0]


check_geometry()


# -- layout -----------------------------------------------------------------------------------------------------------


def layout(seed: int) -> list[str]:
    """Which block (letter) starts at each of the six start places."""
    return [sorted(WIDTHS)[i] for i in np.random.default_rng([seed, LAYOUT_SALT]).permutation(len(WIDTHS))]


LAYOUT_SALT = 2  # with it, layouts 0 to 3 all differ in both rows and every row mixes passing and failing blocks
LAYOUT_SEED = int(os.environ.get("GAUGE_SORT_LAYOUT", "0"))
LAYOUT = layout(LAYOUT_SEED)
BLOCKS = {f"block_{k}": WIDTHS[k] for k in sorted(WIDTHS)}
START = {f"block_{k}": START_XY[i] for i, k in enumerate(LAYOUT)}


def _box(name, lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Box(name, pos=tuple(float(v) for v in (lo + hi) / 2), size=tuple(float(v) for v in hi - lo), color=color)


def _gauge() -> list[Box]:
    bx, (sx, sy, _) = GAUGE_X, BASE_SIZE
    parts = [_box("gauge_base", (bx - sx / 2, -sy / 2, TOP), (bx + sx / 2, sy / 2, BASE_TOP), STEEL)]
    ax, ay, _ = ANVIL_SIZE
    c, below = CHAMFER, ANVIL_TOP - CHAMFER
    for gap, color in (("go", GREEN), ("nogo", RED)):
        inner = GAPS[gap] / 2
        for side, s in (("l", 1.0), ("r", -1.0)):  # s points from the gap into the anvil
            y0 = GAP_Y[gap] + s * inner  # the anvil's inner face
            name = f"{gap}_anvil_{side}"
            lo, hi = sorted((y0, y0 + s * ay))
            parts.append(_box(name, (bx - ax / 2, lo, BASE_TOP), (bx + ax / 2, hi, below), color))
            lo, hi = sorted((y0 + s * c, y0 + s * ay))
            parts.append(_box(f"{name}_top", (bx - ax / 2, lo, below), (bx + ax / 2, hi, ANVIL_TOP), color))
            # a square prism turned 45 degrees about x whose upper face is the bevel from (inner face, top - c) to
            # (inner face + c, top); it lies wholly inside the anvil's outline
            side_len = c * math.sqrt(2)
            parts.append(Box(f"{name}_bevel", pos=(bx, y0 + s * c, below), size=(ax, side_len, side_len), color=color))
    return parts


def tray_inner(tray: str) -> tuple[np.ndarray, np.ndarray]:
    half = np.array(TRAY_INNER) / 2
    c = np.array([TRAY_X, TRAY_Y[tray]])
    return c - half, c + half


def _tray(tray: str, color) -> list[Box]:
    (x0, y0), (x1, y1) = tray_inner(tray)
    t, top = TRAY_WALL, TRAY_FLOOR_TOP + TRAY_WALL_H
    return [
        _box(f"{tray}_floor", (x0 - t, y0 - t, TOP), (x1 + t, y1 + t, TRAY_FLOOR_TOP), color),
        _box(f"{tray}_wall_near", (x0 - t, y0 - t, TRAY_FLOOR_TOP), (x0, y1 + t, top), color),
        _box(f"{tray}_wall_far", (x1, y0 - t, TRAY_FLOOR_TOP), (x1 + t, y1 + t, top), color),
        _box(f"{tray}_wall_right", (x0, y0 - t, TRAY_FLOOR_TOP), (x1, y0, top), color),
        _box(f"{tray}_wall_left", (x0, y1, TRAY_FLOOR_TOP), (x1, y1 + t, top), color),
    ]


OBJECTS = [
    *(
        Box(n, pos=(*START[n], TOP + BLOCK_H / 2), size=(BLOCK_LEN, w, BLOCK_H), mass=block_mass(w), color=ALUMINIUM)
        for n, w in BLOCKS.items()
    ),
    *_gauge(),
    *_tray("green", GREEN),
    *_tray("red", RED),
]
GAUGE_PARTS = {
    gap: ("gauge_base", *(f"{gap}_anvil_{s}{p}" for s in "lr" for p in ("", "_top", "_bevel"))) for gap in GAPS
}
GAUGE_NAMES = {o.name for o in OBJECTS if o.name.startswith(("gauge_", "go_", "nogo_"))}
BEVEL_XYZW = (math.sin(math.pi / 8), 0.0, 0.0, math.cos(math.pi / 8))  # 45 degrees about x


# -- pose tests (they say which reading a block shows; never whether it is supported) ----------------------------------


def _axes(st) -> np.ndarray:
    return quat_xyzw_to_matrix(np.asarray(st.quat_xyzw, float))


def gauge_pose(st, gap: str, width: float) -> str | None:
    """'in' (seated: level on the base between the anvils), 'out' (not seated: resting on the anvils, level or tipped
    against one, centre over the gap) or None. A tipped block may reach down to the base; only the width rule (in
    the scorer) decides whether 'out' is a reading the block can give."""
    r = _axes(st)
    ex, ez = r[:, 0], r[:, 2]
    dx, dy = st.pos[0] - GAUGE_X, st.pos[1] - GAP_Y[gap]
    half = GAPS[gap] / 2
    if abs(dx) > X_TOL or abs(ex[0]) < math.cos(YAW_TOL):
        return None
    tilt = math.acos(min(1.0, abs(ez[2])))
    low = st.low[2]
    if tilt <= LEVEL_TOL and abs(low - BASE_TOP) <= Z_TOL and abs(dy) + width / 2 <= half + FIT_TOL:
        return "in"
    if tilt <= OUT_TILT and abs(dy) <= half and low <= ANVIL_TOP + Z_TOL:
        return "out"
    return None


def tray_outer(tray: str) -> tuple[np.ndarray, np.ndarray]:
    lo, hi = tray_inner(tray)
    return lo - TRAY_WALL, hi + TRAY_WALL


def _overlaps(st, lo, hi) -> bool:
    return bool(np.all(st.low[:2] < hi[:2]) and np.all(st.high[:2] > lo[:2]))


def _box_distance(p, st) -> float:
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(st.low - p, p - st.high))))


def released(st, grippers) -> bool:
    """Every gripper whose TCP is near the block is open wider than the block is long; no reading -> not released."""
    return grippers is not None and all(
        opening > RELEASE_OPEN for tcp, opening in grippers if _box_distance(tcp, st) <= RELEASE_RADIUS
    )


def tray_contents(objects, grippers) -> dict[str, str]:
    """Blocks in a tray: footprint inside its walls, still, released, and resting on its floor or on a block that is
    itself in that tray (found bottom-up, so a stack on a held block never counts)."""
    free = [n for n in BLOCKS if objects[n].resting() and released(objects[n], grippers)]
    placed, grown = {}, True
    while grown:
        grown = False
        for n in free:
            st = objects[n]
            for tray in TRAY_Y if n not in placed else ():
                lo, hi = tray_inner(tray)
                inside = bool(np.all(st.low[:2] >= lo) and np.all(st.high[:2] <= hi))
                on = abs(st.low[2] - TRAY_FLOOR_TOP) <= Z_TOL or any(
                    t == tray and abs(st.low[2] - objects[o].high[2]) <= Z_TOL
                    and _overlaps(st, objects[o].low, objects[o].high)
                    for o, t in placed.items()
                )
                if inside and on:
                    placed[n], grown = tray, True
                    break
    return placed


def escapes(objects) -> list[str]:
    """Out-of-tolerance blocks anywhere over the green tray's outer footprint: in it, on its rim, held above it."""
    lo, hi = tray_outer("green")
    return [n for n, w in BLOCKS.items() if not in_tolerance(w) and _overlaps(objects[n], lo, hi)]


def at_start(name: str, st) -> bool:
    return bool(np.hypot(*(st.pos[:2] - START[name])) <= START_TOL and abs(st.low[2] - TOP) <= Z_TOL and st.resting())


# -- readers: module-level so tests can replace them; the real ones read the running simulation ----------------------
# read_grippers() -> [(tcp_xyz, pad_gap_m) for the left and the right arm] or None
# read_contacts() -> {block: ({gauge_part: vertical force on the block (N)}, robot force magnitude (N))} or None
# Anything else (wrong shape, missing arm or block, NaN) is treated as no reading for that step.

_SIM: dict = {"env": None, "view": None, "failed": False, "t": 0.0, "hz_ok": True}
_WARNED: set = set()
_PARTS = sorted({p for parts in GAUGE_PARTS.values() for p in parts})


def _warn_once(key: str, msg: str) -> None:
    if key not in _WARNED:
        _WARNED.add(key)
        warnings.warn(f"gauge_sort: {msg}; no check counts while this lasts", RuntimeWarning, stacklevel=3)


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _sim_env(key: str):
    if _SIM["env"] is None:
        _warn_once(key, "no running simulation")
    elif not _SIM["hz_ok"]:
        _warn_once(key, f"control rate is not {CONTROL_HZ} Hz (run with -E control_hz={CONTROL_HZ})")
    else:
        return _SIM["env"]
    return None


def sim_grippers():
    env = _sim_env("read_grippers")
    if env is None:
        return None
    try:
        base = env.unwrapped
        origin = _np(base.scene.env_origins[0])
        out = []
        for arm in ("left_robot", "right_robot"):
            robot = base.scene[arm]
            pose = _np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
            q, names = _np(robot.data.joint_pos)[0], list(robot.joint_names)
            tcp = pose[:3] + quat_xyzw_to_matrix(pose[3:7]) @ np.array(TCP_OFFSET) - origin
            out.append((tcp, sum(abs(q[names.index(j)]) for j in FINGER_JOINTS)))
        return out
    except Exception as e:  # fail closed
        _warn_once("read_grippers", f"cannot read the grippers ({e!r})")
        return None


def _contact_view(env):
    """One rigid contact view: the six blocks as sensors, every gauge collider and every robot link as filters."""
    import carb
    import omni.physics.tensors as tensors
    from pxr import Usd, UsdPhysics

    carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    base = env.unwrapped
    stage = base.sim.stage
    root = base.scene.env_prim_paths[0]

    def leaves(path, api):
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise LookupError(f"no prim at {path}")
        found = [str(p.GetPath()) for p in Usd.PrimRange(prim) if p.HasAPI(api)]
        if not found:
            raise LookupError(f"no {api.__name__} under {path}")
        return found

    sensors = []
    for name in BLOCKS:
        bodies = leaves(f"{root}/{name}", UsdPhysics.RigidBodyAPI)
        assert len(bodies) == 1, bodies
        sensors += bodies
    filters, cols = [], {"gauge": {}, "robot": []}
    for part in _PARTS:
        found = leaves(f"{root}/{part}", UsdPhysics.CollisionAPI)  # static: filter its collider leaves, not the Xform
        cols["gauge"][part] = list(range(len(filters), len(filters) + len(found)))
        filters += found
    for arm in ("LeftArm", "RightArm"):  # yam_arena/embodiment.py SIDES
        found = leaves(f"{root}/{arm}", UsdPhysics.RigidBodyAPI)
        cols["robot"] += list(range(len(filters), len(filters) + len(found)))
        filters += found
    sim_view = tensors.create_simulation_view("torch")
    sim_view.set_subspace_roots("/")
    view = sim_view.create_rigid_contact_view(
        sensors, filter_patterns=[filters] * len(sensors), max_contact_data_count=64 * len(sensors) * len(filters)
    )
    # map rows and columns by the paths PhysX reports, when the view exposes them; else by the order we passed
    rep_s, rep_f = getattr(view, "sensor_paths", None), getattr(view, "filter_paths", None)
    _log({"event": "contact_view", "sensors": sensors, "filters": filters, "reported_sensor_paths": rep_s,
          "reported_filter_paths": rep_f, "sensor_count": getattr(view, "sensor_count", None),
          "filter_count": getattr(view, "filter_count", None)})
    rep_s = [str(p) for p in rep_s] if rep_s else sensors
    rep_f = rep_f or [filters] * len(rep_s)
    if rep_f and isinstance(rep_f[0], str):
        rep_f = [rep_f] * len(rep_s)
    rows = [rep_s.index(p) for p in sensors]
    perm = [[[str(q) for q in rep_f[r]].index(p) for p in filters] for r in rows]
    return sim_view, view, cols, rows, perm


def sim_contacts():
    env = _sim_env("read_contacts")
    if env is None or _SIM["failed"]:
        return None
    try:
        if _SIM["view"] is None:
            _SIM["view"] = _contact_view(env)
        _, view, cols, rows, perm = _SIM["view"]
        normal = _np(view.get_contact_force_matrix(PHYSICS_DT))  # (sensor, filter, 3): normal force on each block, N
        friction = _friction_matrix(view, normal.shape)
        out, split = {}, {}
        for i, name in enumerate(BLOCKS):
            n, f = normal[rows[i]][perm[i]], friction[rows[i]][perm[i]]  # in our filter order
            total = n + f
            out[name] = (
                {part: GAUGE_FZ_SIGN * float(total[idx, 2].sum()) for part, idx in cols["gauge"].items()},
                float(np.linalg.norm(total[cols["robot"]], axis=-1).sum()),  # per link, so opposing pushes add up
            )
            gauge = [j for idx in cols["gauge"].values() for j in idx]
            split[name] = (float(n[gauge, 2].sum()), float(f[gauge, 2].sum()))
        _SIM["split"] = split  # gauge normal vs friction share, for the log
        return out
    except Exception as e:  # fail closed, and do not retry a view that could not be built
        _SIM["failed"] = True
        _warn_once("read_contacts", f"cannot read contact forces ({e!r})")
        return None


def _friction_matrix(view, shape) -> np.ndarray:
    """Friction force per (sensor, filter) pair, summed over its contact points. A tipped block resting on an anvil
    edge is carried partly by friction, which the force matrix (normal forces only) leaves out, and a sliding robot
    contact pushes partly through friction. Any error here propagates, so sim_contacts fails closed."""
    forces, _, counts, starts = view.get_friction_data(PHYSICS_DT)
    forces, counts, starts = _np(forces).reshape(-1, 3), _np(counts).astype(int), _np(starts).astype(int)
    out = np.zeros(shape)
    for s, f in zip(*np.nonzero(counts)):
        out[s, f] = forces[starts[s, f]:starts[s, f] + counts[s, f]].sum(0)
    return out


read_grippers = sim_grippers
read_contacts = sim_contacts


def _checked_grippers(v):
    out = [(np.asarray(tcp, dtype=float).reshape(3), float(opening)) for tcp, opening in v]
    if len(out) != 2 or not all(np.all(np.isfinite(t)) and math.isfinite(o) and o >= 0 for t, o in out):
        raise ValueError("expected (tcp xyz, opening) for both arms, all finite")
    return out


def _checked_contacts(v):
    out = {}
    for name in BLOCKS:
        gauge, robot = v[name]
        g, r = {p: float(gauge[p]) for p in _PARTS}, float(robot)
        if not (all(math.isfinite(x) for x in g.values()) and math.isfinite(r) and r >= 0):
            raise ValueError(f"non-finite or negative force for {name}")
        out[name] = (g, r)
    return out


def _read(name: str):
    """The reader's value checked for shape and finiteness, or None (one warning per reader, never an exception)."""
    try:
        value = globals()[name]()  # looked up per call so tests can swap them
        if value is not None:
            value = {"read_grippers": _checked_grippers, "read_contacts": _checked_contacts}[name](value)
    except Exception as e:
        _warn_once(name, f"{name} raised or gave a malformed reading ({e!r})")
        return None
    if value is None:
        _warn_once(name, f"{name} gave no reading")
    return value


def _log(row: dict) -> None:
    if LOG_PATH:
        with open(LOG_PATH, "a") as fh:
            fh.write(json.dumps(row, default=lambda v: np.asarray(v).tolist()) + "\n")


# -- scorer -----------------------------------------------------------------------------------------------------------


class _Checks:
    def __init__(self):
        self.go = self.nogo = None  # first valid reading at each gap: "in" (drops in) or "out" (stays out)
        self.streak = (None, None, 0)  # (gap, reading, consecutive valid steps)

    def verdict(self) -> str | None:
        if self.go == "out" or self.nogo == "in":
            return "red"
        if self.go == "in" and self.nogo == "out":
            return "green"
        return None


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.checks = {n: _Checks() for n in BLOCKS}
        self.step, self._last, self._score = 0, None, 0.0

    def __call__(self, objects) -> float:
        if objects is self._last:  # success() and score() see the same states each step
            return self._score
        if all(at_start(n, objects[n]) for n in BLOCKS):  # second line; the embodiment-reset hook is the first
            self.reset()
        self._last, self.step = objects, self.step + 1
        grippers = _read("read_grippers")
        self._update_checks(objects, grippers)
        self._score = self._tally(objects, grippers)
        return self._score

    def _update_checks(self, objects, grippers) -> None:
        candidates = {}
        for name, width in BLOCKS.items():
            for gap in GAPS:
                reading = gauge_pose(objects[name], gap, width)
                if reading is not None:
                    candidates[name] = (gap, reading)
        contacts = _read("read_contacts") if candidates else None
        for name, c in self.checks.items():
            gap, reading = candidates.get(name, (None, None))
            ok = gap is not None and self._valid(name, objects[name], gap, reading, grippers, contacts)
            if not ok:
                c.streak = (None, None, 0)
                continue
            n = c.streak[2] + 1 if c.streak[:2] == (gap, reading) else 1
            c.streak = (gap, reading, n)
            if n >= CHECK_STEPS:
                if gap == "go" and c.go is None:
                    c.go = reading
                elif gap == "nogo" and c.nogo is None and c.go == "in":  # NO-GO counts only after GO "drops in"
                    c.nogo = reading

    def _valid(self, name, st, gap, reading, grippers, contacts) -> bool:
        width = BLOCKS[name]
        allowed = reading == ("in" if drops_in(width, gap) else "out")
        free = released(st, grippers)
        supported = False
        row = {"module": id(globals()), "step": self.step, "block": name, "gap": gap, "pose": reading,
               "allowed": allowed, "pos": st.pos,
               "quat_xyzw": st.quat_xyzw, "low": st.low, "high": st.high, "grippers": grippers}
        if contacts is not None:
            gauge, robot = contacts[name]
            fz = sum(gauge[p] for p in GAUGE_PARTS[gap])
            supported = fz >= SUPPORT_FRAC * block_mass(width) * G and robot <= ROBOT_TOUCH_N
            row.update(gauge_fz=gauge, robot_n=robot, weight_n=block_mass(width) * G,
                       gauge_normal_friction_z=_SIM.get("split", {}).get(name))
        valid = allowed and free and supported
        row.update(released=free, supported=supported, valid=valid)
        _log(row)
        return valid

    def _tally(self, objects, grippers) -> float:
        placed = tray_contents(objects, grippers)
        credited = sum(1 for n, c in self.checks.items() if c.verdict() is not None and placed.get(n) == c.verdict())
        score = credited / len(BLOCKS)
        return min(score, 0.25) if escapes(objects) else score


_SCORER = Scorer()


def score(objects) -> float:
    return _SCORER(objects)


def success(objects) -> bool:
    return _SCORER(objects) >= 1.0


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Open loop from the published layout; it knows each block's width from the layout seed (allowed for a demo) only to
# choose the release height in the gap and the tray. Both arms work in turn; the idle one waits at its park pose.

ARM_OF_ROW = {1: "left", -1: "right"}
GRIP_YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along world x, on the block's two ends
PARK = {"left": (0.22, 0.30), "right": (0.22, -0.30)}
TRAVEL_Z = TOP + 0.10  # TCP height for every carry; the held block's bottom is 77 mm above the table
GRASP_DZ = BLOCK_H / 2 + PAD_TIP_PAST_TCP  # TCP above the block's bottom: pad tips at the block's mid-height
HOVER_GAP = 0.004  # held block's bottom above the anvil tops before it is lowered in or set down
RELEASE_GAP = 0.002  # held block's bottom above the tray floor when it is let go
PRESS = 0.001  # on the gauge the block is set down, pressed this far into the face, not dropped: a block with
# little overlap on an anvil tips over if dropped onto it
BACKOFF_DZ = 0.05  # TCP rise after letting go, before the check dwell
DWELL_S = 0.8  # hands-off time per check (16 steps at 20 Hz; a check needs 6)
# The arms ring for a second or two after a long move (about +-3 mm, 1.2 s period), so carries are slow and the arm
# waits at travel height and again just above the anvils before setting a block down.
MOVE_S, DESCEND_S, FINE_S, LIFT_S, HOLD_S, ALIGN_S = 2.5, 1.0, 1.0, 1.0, 1.0, 0.8
ALIGN_BACK = 0.003  # beyond the joints' friction deadband (about 1.3 mm at the TCP)
HELD_YAW_TRIM = math.radians(-1.0)  # a held block sits 1 to 3 deg off the grip's yaw (measured in the probes)
# The rollout's delta-limit guard moves the gripper command at most 0.05 per step (a full stroke takes 1 s): grip moves
# are paced to that, and the arm waits LET_GO_S after letting go so it never drags a block with half-open fingers.
GRIP_S, LET_GO_S = 0.8, 0.4
# Fingers closed to 4 mm inside the block's ends rather than fully: the finger drives (5000 N/m, 50 N effort limit)
# then centre the block with about 20 N each; a fully closed command saturates both and lets the block slide.
GRIP_SQUEEZE = 0.004
HOLD = -(BLOCK_LEN / 2 - GRIP_SQUEEZE)
# Each arm lets a block go a little off its target across the slot: the base joint's friction (0.3 N m against an
# 80 N m/rad drive) leaves it lagging behind its last sideways move. Every gauge visit therefore ends with the same
# short sideways move toward the middle (APPROACH_DY), and the target is offset against that lag (RELEASE_DY, the
# block offsets seen in the development runs).
APPROACH_DY, APPROACH_S = 0.015, 0.8
SIDE = {"left": 1.0, "right": -1.0}
RELEASE_DY = {"left": -0.0012, "right": 0.0013}
TRAY_SLOTS_Y = (0.04, 0.0, -0.04)  # offsets from the tray centre, across the tray
WEDGE_DY, WEDGE_DWELL_S = 0.0035, 3.0  # wedge probe: F let go this far off the GO centre (its overlap is 1.5 mm)


def _wp(arm, d, xy, z, finger, label, yaw_trim=0.0):
    return Waypoint(d, (float(xy[0]), float(xy[1]), float(z)), finger, label, GRIP_YAW[arm] + yaw_trim)


class _Live:
    """A waypoint position worked out from the simulator the first time the scripted plan reaches it (ArmPlan reads a
    waypoint's position from the moment it becomes the current target), then kept. The demo reads only block poses."""

    def __init__(self, fn):
        self.fn, self.value = fn, None

    def get(self) -> np.ndarray:
        if self.value is None:
            self.value = np.asarray(self.fn(), dtype=float)
        return self.value

    def __array__(self, dtype=None, copy=None):
        return self.get().astype(dtype or float)

    def __iter__(self):
        return iter(self.get())

    def __getitem__(self, i):
        return self.get()[i]

    def __len__(self):
        return 3


def sim_block(name):
    env = _SIM["env"]
    if env is None:
        return None
    try:
        base = env.unwrapped
        pose = _np(base.scene[name].data.root_link_pose_w)[0]
        return np.r_[pose[:3] - _np(base.scene.env_origins[0]), pose[3:7]]  # centre, then quat_xyzw (for the log)
    except Exception as e:
        _warn_once("read_block", f"demo cannot read block poses ({e!r})")
        return None


read_block = sim_block  # name -> block centre (x, y, z[, quat]) or None; the demo then keeps its planned position


def _aligned(prev, name, want_y):
    """The previous target moved across the slot by how far the held block is from where it should be."""
    block = globals()["read_block"](name)
    _log({"event": "align", "block": name, "seen": block, "want_y": want_y, "target": prev})
    return prev if block is None else (prev[0], prev[1] - (block[1] - want_y), prev[2])


def _seen(target, name, want_y):
    _log({"event": "aligned", "block": name, "seen": globals()["read_block"](name), "want_y": want_y})
    return target


def _over_block(name, xy, floor):
    """Where to take a block again: at its seen centre (it may have shifted or tipped on the gauge)."""
    block = globals()["read_block"](name)
    _log({"event": "regrasp", "block": name, "seen": block})
    if block is None:
        return (*xy, floor + GRASP_DZ)
    return (block[0], block[1], block[2] - BLOCK_H / 2 + GRASP_DZ)


def _gauge_visit(arm, name, gap, dy=0.0) -> list[Waypoint]:
    want_y, width = GAP_Y[gap] + dy, BLOCKS[name]
    xy = (GAUGE_X, want_y + RELEASE_DY[arm])
    side = (xy[0], xy[1] + SIDE[arm] * APPROACH_DY)
    drop = drops_in(width, gap)
    floor = BASE_TOP if drop else ANVIL_TOP
    over = ANVIL_TOP + HOVER_GAP + GRASP_DZ
    # A block that will drop in is let go just above the anvils and falls through the chamfered entry on its own (held,
    # it would pivot about the fingers and jam); one that will stay out is set down on the anvils, not dropped.
    at = (ANVIL_TOP + RELEASE_GAP if drop else ANVIL_TOP - PRESS) + GRASP_DZ
    # Moving the target by about 1 mm does not move the arm (it is inside the joints' friction deadband), so the
    # correction backs out toward the arm's own side and comes in again: the last move, and so the lag, is the same as
    # on the approach on which the error was seen.
    aim = _Live(lambda: _aligned((*xy, over), name, want_y))
    back = _Live(lambda: (aim[0], aim[1] + SIDE[arm] * ALIGN_BACK, aim[2]))
    set_on = _Live(lambda: _seen((*aim.get()[:2], at), name, want_y))
    up = _Live(lambda: (*aim.get()[:2], at + BACKOFF_DZ))
    again = _Live(lambda: _over_block(name, xy, floor))
    yaw, held = GRIP_YAW[arm], GRIP_YAW[arm] + HELD_YAW_TRIM
    return [
        _wp(arm, MOVE_S, side, TRAVEL_Z, HOLD, f"carry {name} to {gap}", HELD_YAW_TRIM),
        _wp(arm, APPROACH_S, xy, TRAVEL_Z, HOLD, "approach", HELD_YAW_TRIM),
        _wp(arm, DESCEND_S, xy, over, HOLD, f"over {gap}", HELD_YAW_TRIM),
        _wp(arm, HOLD_S, xy, over, HOLD, "settle", HELD_YAW_TRIM),
        Waypoint(ALIGN_S, back, HOLD, "align: back out", held),
        Waypoint(ALIGN_S, aim, HOLD, "align: in", held),
        Waypoint(FINE_S, set_on, HOLD, f"set on {gap}", held),
        Waypoint(GRIP_S, set_on, FINGER_OPEN, f"release {name} on {gap}", held),
        Waypoint(LET_GO_S, set_on, FINGER_OPEN, "let go", held),
        Waypoint(GRIP_S, up, FINGER_OPEN, "back off", yaw),
        Waypoint(DWELL_S, up, FINGER_OPEN, f"dwell {gap}", yaw),
        Waypoint(GRIP_S, again, FINGER_OPEN, "regrasp", yaw),
        Waypoint(GRIP_S, again, HOLD, "close", yaw),
        _wp(arm, LIFT_S, xy, TRAVEL_Z, HOLD, "lift"),
    ]


def _fetch(arm, name) -> list[Waypoint]:
    start = START[name]
    return [
        _wp(arm, MOVE_S, start, TRAVEL_Z, FINGER_OPEN, f"to {name}"),
        _wp(arm, DESCEND_S, start, TOP + GRASP_DZ, FINGER_OPEN, "descend"),
        _wp(arm, GRIP_S, start, TOP + GRASP_DZ, HOLD, f"grasp {name}"),
        _wp(arm, LIFT_S, start, TRAVEL_Z, HOLD, "lift"),
    ]


def block_cycle(arm, name, tray, slot) -> list[Waypoint]:
    to = (TRAY_X, TRAY_Y[tray] + TRAY_SLOTS_Y[slot])
    path = _fetch(arm, name) + _gauge_visit(arm, name, "go")
    if drops_in(BLOCKS[name], "go"):
        path += _gauge_visit(arm, name, "nogo")
    return path + [
        _wp(arm, MOVE_S, to, TRAVEL_Z, HOLD, f"carry {name} to {tray}"),
        _wp(arm, DESCEND_S, to, TRAY_FLOOR_TOP + RELEASE_GAP + GRASP_DZ, HOLD, "lower"),
        _wp(arm, GRIP_S, to, TRAY_FLOOR_TOP + RELEASE_GAP + GRASP_DZ, FINGER_OPEN, f"release {name} in {tray}"),
        _wp(arm, LET_GO_S, to, TRAY_FLOOR_TOP + RELEASE_GAP + GRASP_DZ, FINGER_OPEN, "let go"),
        _wp(arm, LIFT_S, to, TRAVEL_Z, FINGER_OPEN, "retreat"),
        _wp(arm, MOVE_S, PARK[arm], TRAVEL_Z, FINGER_OPEN, "park"),
    ]


def wedge_probe() -> dict[str, list[Waypoint]]:
    """Calibration only: let the oversize block F go off centre over GO so it tips and wedges, then leave it."""
    arm = ARM_OF_ROW[int(np.sign(START["block_f"][1]))]
    visit = _gauge_visit(arm, "block_f", "go", dy=WEDGE_DY)
    visit = visit[: [w.label for w in visit].index("back off") + 1]
    hold = visit[-1]
    return {arm: [_wp(arm, MOVE_S, PARK[arm], TRAVEL_Z, FINGER_OPEN, "park"), *_fetch(arm, "block_f"), *visit,
                  Waypoint(WEDGE_DWELL_S, hold.pos, FINGER_OPEN, "dwell wedged", hold.yaw)]}


def demo() -> dict[str, list[Waypoint]]:
    if PROBE == "wedge":
        return wedge_probe()
    plans = {arm: [_wp(arm, MOVE_S, PARK[arm], TRAVEL_Z, FINGER_OPEN, "park")] for arm in ("left", "right")}
    rows = {arm: sorted((n for n in BLOCKS if ARM_OF_ROW[int(np.sign(START[n][1]))] == arm), key=lambda n: START[n][0])
            for arm in plans}
    used = {"green": 0, "red": 0}
    for i in range(3):
        for arm, idle in (("left", "right"), ("right", "left")):
            name = rows[arm][i]
            tray = "green" if in_tolerance(BLOCKS[name]) else "red"
            cycle = block_cycle(arm, name, tray, used[tray])
            used[tray] += 1
            plans[arm] += cycle
            plans[idle].append(_wp(idle, sum(w.duration for w in cycle), PARK[idle], TRAVEL_Z, FINGER_OPEN, "wait"))
    return plans


# -- hooks into yam_arena (no SDK edits): blocks spawn with a contact report; the scorer gets the running env ---------


def _gauge_object(scene, spec):
    """A fixed gauge part with the gauge's steel friction; a bevel prism is turned 45 degrees about x."""
    import isaaclab.sim as sim_utils
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg

    cfg = sim_utils.CuboidCfg(
        size=tuple(spec.size),
        collision_props=PhysxCollisionPropertiesCfg(),
        physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=GAUGE_FRICTION[0],
                                                        dynamic_friction=GAUGE_FRICTION[1]),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.4),
    )
    rot = BEVEL_XYZW if spec.name.endswith("_bevel") else scene._yaw_xyzw(spec.yaw)
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos), rotation_xyzw=tuple(rot)))


def _block_object(scene, spec):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils import clone
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

    @clone
    def spawn_block(prim_path, cfg, *args, **kwargs):  # as yam_arena.scene._spawn_dream_bottle, plus a contact report
        import carb
        from pxr import PhysxSchema

        prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        return prim

    cfg = sim_utils.CuboidCfg(
        func=spawn_block,
        size=tuple(spec.size),
        rigid_props=PhysxRigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=spec.mass),
        collision_props=scene._contact_props(),
        physics_material=scene._grippy(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=scene.linear_rgb(spec.color), roughness=0.4),
    )
    return Object(
        name=spec.name,
        object_type=ObjectType.RIGID,
        spawner_cfg=cfg,
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=scene._yaw_xyzw(spec.yaw)),
    )


def _on_step(emb) -> None:
    try:
        if _SIM["env"] is not emb._env:
            _SIM.update(env=emb._env, view=None, failed=False)
        _SIM["hz_ok"] = float(emb.info.control_hz) == CONTROL_HZ
        if emb._t < _SIM["t"]:  # episode clock went backwards: a new trial even if no reset was seen
            _SCORER.reset()
        _SIM["t"] = emb._t
    except Exception as e:  # never raise into the SDK step; no reading this step
        _SIM.update(env=None, view=None)
        _warn_once("embodiment", f"cannot read the embodiment ({e!r})")


def _on_reset(emb) -> None:
    _SCORER.reset()
    _SIM["t"] = 0.0
    try:
        if _SIM["env"] is not emb._env:
            _SIM.update(env=emb._env, view=None, failed=False)
    except Exception as e:
        _SIM.update(env=None, view=None)
        _warn_once("embodiment", f"cannot read the embodiment ({e!r})")
    _log({"event": "reset", "module": id(globals()), "layout": LAYOUT_SEED})


# The patched embodiment calls the copy of this file it loaded itself (YamArenaEmbodiment.task.module, set by
# task_file.load in its __init__), so a second load before or after it cannot steal or drop the simulation.
# Without that attribute (and for the scene, which spawns the same blocks either way) the newest copy is used.


def _module_for(emb, route) -> dict:
    mod = getattr(getattr(getattr(emb, "task", None), "module", None), "__dict__", None)
    return mod if mod is not None and "_on_step" in mod else route["module"]


def _patch_scene(scene, route) -> None:
    if getattr(scene.task_object, "_gauge_sort", False):
        return
    orig = scene.task_object

    def task_object(spec):
        mod = route["module"]
        if spec.name in mod["BLOCKS"]:
            return mod["_block_object"](scene, spec)
        return mod["_gauge_object"](scene, spec) if spec.name in mod["GAUGE_NAMES"] else orig(spec)

    task_object._gauge_sort = True
    scene.task_object = task_object


def _patch_embodiment(ie, route) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_gauge_sort", False):
        return
    step, reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _module_for(self, route)["_on_step"](self)
        return step(self, *a, **k)

    def _reset(self, *a, **k):
        obs = reset(self, *a, **k)
        _module_for(self, route)["_on_reset"](self)
        return obs

    _task_step._gauge_sort = True
    cls._task_step, cls.reset = _task_step, _reset


class _Finder(importlib.abc.MetaPathFinder):
    """yam_arena.scene can only be imported once Isaac Sim runs, after this file is loaded: patch it on import."""

    def __init__(self, patches):
        self.patches, self.gauge_sort_route = patches, {}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.patches:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        exec_module, patch = spec.loader.exec_module, self.patches[fullname]

        def patched(module):
            exec_module(module)
            patch(module, self.gauge_sort_route)

        spec.loader.exec_module = patched
        return spec


_FINDER = next((f for f in sys.meta_path if isinstance(getattr(f, "gauge_sort_route", None), dict)), None)
if _FINDER is None:
    _FINDER = _Finder({"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment})
    sys.meta_path.insert(0, _FINDER)
_FINDER.gauge_sort_route["module"] = globals()
for _name, _patch in _FINDER.patches.items():
    if _name in sys.modules:
        _patch(sys.modules[_name], _FINDER.gauge_sort_route)
_log({"event": "load", "module": id(globals()), "layout": LAYOUT_SEED})
