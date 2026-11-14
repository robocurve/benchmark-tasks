"""On the station table, at full size: a patch of worsted pool cloth stands for the foot end of a pool table's bed,
with the foot spot marked on it. On the cloth sits a wooden 8-ball triangle rack (apex away from the robot, back bar
toward it) holding the fifteen 2 1/4 in balls in a legal 8-ball order, loose: they stand about 2 mm apart and the
apex sits 15 mm past the foot spot. The job: rack them tight with the head ball on the foot spot, then take the rack
away without disturbing them.

Why it is hard: the rack's cavity is larger than the pack (269.9 mm inside from the head ball to the rear centre,
against 255.1 mm for fifteen touching balls), so lifting it straight off a loose pack leaves the gaps in place. The
pack has to be squeezed into the rack's apex, then the frame lifted off a group of rolling balls without dragging
any of them (a few mm of sideways motion while the walls still touch the balls rolls them apart) and set down on the
table away from the pack.

Physics: the balls are rigid spheres (0.165 kg) whose rolling resistance on the cloth is modelled by angular damping
(PhysX has no rolling friction); contact offsets of 0.1 mm keep the 0.5 mm gap measure meaningful. The rack is a free
wooden body (0.227 kg) made of three bars.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/rack-pool-balls-tight/rack_pool_balls_tight.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402  (multi-box free part for the rack, per-episode memory, installed in memory)

from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DEBUG = bool(os.environ.get("RK_DEBUG"))
M = "rk"

# -- dimensions (m) ----------------------------------------------------------------------------------------------------
IN = 0.0254
D = 2.25 * IN  # WPA ball, 57.15 mm
R = D / 2
ROW = D * math.sqrt(3) / 2  # row pitch of a tight pack, 49.49 mm
BALL_KG = 0.165  # WPA 5.5-6 oz (156-170 g)
HEAD_TO_REAR = 10.625 * IN  # rack inside, 'from head ball to rear center' (wood 8-ball rack listing)
GAP_B = HEAD_TO_REAR - (4 * ROW + D)  # 14.8 mm: the back bar this far behind a tight pack's back row
WALL_T, WALL_H = 0.015, 1.25 * IN  # bar thickness (UNVERIFIED), height 1.25 in
RACK_KG = 0.227  # 0.5 lb
CLOTH_T = 0.003
CLOTH_TOP = TOP + CLOTH_T
X_SPOT, Y_SPOT = 0.46, 0.0  # the foot spot
TABLE_X0 = 0.36 - 1.1 / 2  # the station table's edge behind the arm bases
START_PAST = float(os.environ.get("RK_START_PAST", "0.015"))  # the loose head ball this far past the spot (+x)
G_LOOSE = 0.002  # the balls stand this far apart at the start
VX0 = X_SPOT + START_PAST + 2 * R  # the rack's inner apex vertex at the start (the head ball touches both sides)
L_IN = 2 * R + 4 * ROW + R + GAP_B  # inner vertex to the back bar's inner face

# legal 8-ball order by rows from the apex (1 at the head, 8 in the middle, a solid and a stripe in the back corners)
ORDER = [[1], [9, 2], [10, 8, 3], [11, 7, 14, 4], [5, 13, 15, 6, 12]]
HUES = {1: (0.95, 0.80, 0.10), 2: (0.10, 0.25, 0.80), 3: (0.85, 0.10, 0.10), 4: (0.45, 0.10, 0.55),
        5: (0.95, 0.45, 0.05), 6: (0.05, 0.50, 0.20), 7: (0.50, 0.12, 0.08), 8: (0.04, 0.04, 0.04)}


def ball_color(n):
    if n <= 8:
        return HUES[n]
    return tuple(0.55 * c + 0.45 for c in HUES[n - 8])  # stripes: rendered as a lighter solid of the same hue


def lattice(x_head, y0, spacing):
    """Ball centres (x, y) by rows from the head ball, rows toward -x, at the given centre spacing."""
    return [[(x_head - i * spacing * math.sqrt(3) / 2, y0 + (j - i / 2) * spacing) for j in range(i + 1)] for i in range(5)]


NAMES = [f"ball_{n}" for row in ORDER for n in row]  # row by row from the head
HEAD = "ball_1"
PAIRS = []
_ix = {}
for _i in range(5):
    for _j in range(_i + 1):
        _ix[(_i, _j)] = len(_ix)
for (_i, _j), _a in _ix.items():
    if (_i, _j + 1) in _ix:
        PAIRS.append((_a, _ix[(_i, _j + 1)]))
    if _i < 4:
        PAIRS += [(_a, _ix[(_i + 1, _j)]), (_a, _ix[(_i + 1, _j + 1)])]
assert len(PAIRS) == 30


# -- the rack (a free body of three bars) --------------------------------------------------------------------------------
def rack_boxes():
    """Rack frame: origin at the inner apex vertex, x toward the apex, z at mid-height of the bars. The side bars' inner
    faces are the two 60-deg V lines; the back bar's inner face is L_IN behind the vertex."""
    c30, s30 = math.cos(math.radians(30)), math.sin(math.radians(30))
    side = L_IN / c30 + 2 * WALL_T
    out = []
    for s in (1, -1):
        u = np.array([-c30, s * s30])
        n_in = np.array([-s30, -s * c30])
        c = u * (side / 2 - WALL_T) - n_in * WALL_T / 2
        out.append(mk.box((side, WALL_T, WALL_H), at=(c[0], c[1], 0.0), yaw=math.atan2(u[1], u[0])))
    half = L_IN * math.tan(math.radians(30)) + WALL_T / c30
    out.append(mk.box((WALL_T, 2 * half, WALL_H), at=(-L_IN - WALL_T / 2, 0.0, 0.0)))
    return out


RACK_BOXES = rack_boxes()
WOOD = (0.55, 0.33, 0.16)
kit = mk.Kit()
kit.part("rack", (VX0, Y_SPOT, CLOTH_TOP + 0.0003 + WALL_H / 2), RACK_BOXES, mass=RACK_KG, color=WOOD)

START = [p for row in lattice(X_SPOT + START_PAST, Y_SPOT, D + G_LOOSE) for p in row]
OBJECTS = [
    *kit.objects(),
    Box("cloth", pos=(0.43, 0.0, TOP + CLOTH_T / 2), size=(0.56, 0.62, CLOTH_T), color=(0.05, 0.38, 0.20)),
    Cylinder("foot_spot", pos=(X_SPOT, Y_SPOT, CLOTH_TOP + 0.0001), radius=0.0127, height=0.0002, color=(0.95, 0.95, 0.92)),
    *[Sphere(name, pos=(x, y, CLOTH_TOP + R + 0.0001), radius=R, mass=BALL_KG, color=ball_color(int(name[5:])))
      for name, (x, y) in zip(NAMES, START)],
]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- materials and contact (scene setup, in memory) ----------------------------------------------------------------------
MU = {"ball": 0.06, "cloth": 0.34, "wood": 0.54}  # average combine: ball-ball 0.06, ball-cloth 0.20, ball-wood 0.30
OFFS = 0.0001  # contact offset (m) on balls, rack and cloth; rest offset 0
ANG_DAMP = 10.0  # 1/s on the balls: rolling resistance stand-in


def _descendants(prim):
    out = []
    for c in prim.GetAllChildren():
        out.append(c)
        out.extend(_descendants(c))
    return out


def _scene_setup(stage) -> None:
    from pxr import PhysxSchema, UsdPhysics

    env = mk._SIM["paths"]["rack"].rsplit("/", 1)[0]
    groups = {"wood": [f"{env}/rack"], "cloth": [f"{env}/cloth"], "ball": [f"{env}/{b}" for b in NAMES]}
    n_mat = n_col = 0
    for kind, roots in groups.items():
        for path in roots:
            root = stage.GetPrimAtPath(path)
            if not root.IsValid():
                print(f"[rk] missing prim {path}", flush=True)
                continue
            for p in [root, *_descendants(root)]:
                if p.HasAPI(UsdPhysics.MaterialAPI):
                    m = UsdPhysics.MaterialAPI(p)
                    m.GetStaticFrictionAttr().Set(MU[kind])
                    m.GetDynamicFrictionAttr().Set(MU[kind])
                    (m.GetRestitutionAttr() or m.CreateRestitutionAttr()).Set(0.0)
                    n_mat += 1
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    c = PhysxSchema.PhysxCollisionAPI.Apply(p)
                    c.CreateContactOffsetAttr().Set(OFFS)
                    c.CreateRestOffsetAttr().Set(0.0)
                    n_col += 1
            if kind == "ball":
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
                rb.CreateAngularDampingAttr().Set(ANG_DAMP)
                rb.CreateLinearDampingAttr().Set(0.0)
                rb.CreateSolverPositionIterationCountAttr().Set(32)
                rb.CreateSolverVelocityIterationCountAttr().Set(4)
                rb.CreateSleepThresholdAttr().Set(0.0)
            if kind == "wood":
                rb = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
                rb.CreateSolverPositionIterationCountAttr().Set(32)
                rb.CreateSolverVelocityIterationCountAttr().Set(4)
    spot = stage.GetPrimAtPath(f"{env}/foot_spot")
    for p in [spot, *_descendants(spot)]:  # the spot is a printed mark: no collider
        if p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)
    print(f"[rk] scene: {n_mat} materials, {n_col} colliders at contact offset {OFFS * 1000:.1f} mm / rest 0; mu {MU}; "
          f"ball angular damping {ANG_DAMP}; start: head ball {START_PAST * 1000:.0f} mm past the spot, gaps "
          f"{G_LOOSE * 1000:.0f} mm", flush=True)


def _patch_scene(mod) -> None:
    from isaaclab.scene import interactive_scene as isc

    cls = isc.InteractiveScene
    if getattr(cls, "_rk_setup", False):
        return
    init0 = cls.__init__

    def __init__(self, *a, **k):
        init0(self, *a, **k)
        try:
            _scene_setup(self.stage)
        except Exception as e:  # noqa: BLE001
            import traceback

            traceback.print_exc()
            print(f"[rk] scene setup failed: {e!r}", flush=True)

    cls.__init__ = __init__
    cls._rk_setup = True


def _install_scene_patch() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
        return
    for f in sys.meta_path:  # after mechkit's own patch of yam_arena.scene
        if isinstance(f, mk._AfterImport) and f.name == "yam_arena.scene":
            p0 = f.patch

            def both(mod, p0=p0):
                p0(mod)
                _patch_scene(mod)

            f.patch = both
            return


_install_scene_patch()

# -- readouts ------------------------------------------------------------------------------------------------------------
GATE_RISE = 0.002  # m: the rack counts as rising once its lowest point is this far above the cloth
GATE_REST = 0.0005  # m: back down (lowest point within this of the cloth, still, not away): the gate re-arms
GATE_MOVE = 0.003  # m: any ball's centre moving more than this horizontally while the rack rises: latched 0
CLEAR = 0.050  # m: rack clear: every part this far horizontally (seen from above) from every ball's outline...
DOWN_Z = 0.005  # m: ...and set down: its lowest point within this of the table top (on the table or the cloth), still
GAP_OK = 0.0005  # m: a pair counts as tight at <= 0.5 mm (feeler gauge)
SPOT_25, SPOT_100 = 0.010, 0.005  # m: head-ball centre to the spot's centre
HAND_CLEAR = 0.030  # m: every robot point this far from every ball surface
STILL = 0.005  # m/s
# Orientation (for 0.75 and 1.0). The table's long axis is x; the head of the table (head string) is +x, the way the
# apex points in the setup. APEX_TOL: the direction from the pack's centroid (132 mm behind the head ball) to the head
# ball within +-5 deg of +x, i.e. the rear row's centre within 17 mm (under a third of a ball) of the long string; this
# is what rejects a pack facing the wrong way (180 deg), which the rear-row line alone cannot tell apart. REAR_TOL: the
# line fitted through the five rear-row centres within +-3 deg of square to the long axis, i.e. its end balls within
# 6.0 mm (a tenth of a ball) of square. A tight pack is rigid, so this also bounds its rotation to +-3 deg. Overhead
# grid photo, centres to ~1 mm: angle error about 0.6 deg (apex) and 0.3 deg (rear-row fit), well inside both.
APEX_TOL, REAR_TOL = 5.0, 3.0  # deg
REAR = slice(10, 15)  # NAMES indices of the rear row (5, 13, 15, 6, 12)


def centres(objects) -> np.ndarray:
    return np.array([np.asarray(objects[b].pos, float) for b in NAMES])


def pair_gaps(c) -> np.ndarray:
    return np.array([np.linalg.norm(c[a] - c[b]) - D for a, b in PAIRS])


def _hull(p) -> np.ndarray:
    """Convex hull (counter-clockwise) of 2D points."""
    p = sorted(map(tuple, np.round(p, 9)))
    lower, upper = [], []
    for seq, out in ((p, lower), (p[::-1], upper)):
        for q in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (q[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (q[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(q)
    return np.array(lower[:-1] + upper[:-1])


def _dist_to_poly(q, poly) -> np.ndarray:
    """Distance (m) from each 2D point in q to the convex polygon poly (CCW); 0 inside."""
    a, b = poly, np.roll(poly, -1, axis=0)
    e = b - a
    rel = q[:, None, :] - a[None, :, :]
    inside = np.all(e[None, :, 0] * rel[:, :, 1] - e[None, :, 1] * rel[:, :, 0] >= 0, axis=1)
    s = np.clip((rel * e[None]).sum(-1) / (e * e).sum(-1)[None], 0.0, 1.0)
    d = np.linalg.norm(rel - s[..., None] * e[None], axis=2).min(axis=1)
    return np.where(inside, 0.0, d)


def rack_clearance(objects, c) -> float:
    """Least horizontal distance (m), seen from above, from any ball's outline to the rack: each bar's footprint (its
    oriented box projected on the table) against each ball's circle."""
    t = kit._pose_T(objects, "rack")
    best = 9.0
    for b in RACK_BOXES:
        rw = t[:3, :3] @ mk.quat_to_mat(b.quat)
        cw = t[:3, :3] @ np.asarray(b.at) + t[:3, 3]
        h = np.asarray(b.size) / 2
        corners = np.array([cw + rw @ (h * np.array([i, j, k])) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
        best = min(best, float((_dist_to_poly(c[:, :2], _hull(corners[:, :2])) - R).min()))
    return best


def rack_down(objects) -> bool:
    """The rack set down: its lowest point within 5 mm of the table top and moving under 5 mm/s."""
    st = objects["rack"]
    return bool(float(np.asarray(st.low)[2]) - TOP <= DOWN_Z and float(np.linalg.norm(np.asarray(st.lin_vel))) < STILL)


def rack_away(objects, c) -> bool:
    return rack_down(objects) and rack_clearance(objects, c) >= CLEAR


def rack_low(objects) -> float:
    return float(np.asarray(objects["rack"].low)[2] - CLOTH_TOP)


# robot points: TCP, both pad tips and both pad middles of each gripper (pad points from the URDF collision meshes, in
# each finger link's frame, m)
PAD_LOCAL = {"tip_left": [(0.0237, -0.04673, -0.09016), (0.01703, -0.04519, -0.06016)],
             "tip_right": [(-0.0237, 0.04674, -0.09016), (-0.01526, 0.04518, -0.06016)]}
_TEST_POINTS = None


def _quat_xyzw_from(q4, wxyz) -> np.ndarray:
    return np.array([q4[1], q4[2], q4[3], q4[0]]) if wxyz else np.asarray(q4)


def robot_points() -> list:
    """World points on both grippers (TCP, pad tips, pad middles); [] offline."""
    if _TEST_POINTS is not None:
        return _TEST_POINTS
    env = mk._EP["env"]
    if env is None:
        return []
    base = env.unwrapped
    origin = mk._arr(base.scene.env_origins)[0] if hasattr(mk, "_arr") else _arr(base.scene.env_origins)[0]
    out = list(mk.tcps())
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        pos = _arr(art.data.body_pos_w)[0] - origin
        quat = _arr(art.data.body_quat_w)[0]
        for link, pts in PAD_LOCAL.items():
            k = names.index(link)
            r = mk.quat_to_mat(_quat_xyzw_from(quat[k], _RT.get("wxyz", True)))
            out += [pos[k] + r @ np.asarray(p) for p in pts]
    return out


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def hand_clearance(c) -> float:
    pts = robot_points()
    if not pts:
        return 9.0
    p = np.asarray(pts)
    return float((np.linalg.norm(p[:, None, :] - c[None, :, :], axis=2) - R).min())


_RT: dict = {}


def _mem() -> dict:
    m = mk.MEMORY.get(M)
    if m is None:
        m = mk.MEMORY[M] = {"n": 0, "ref": None, "gate": None, "max_move": 0.0, "cleared": None, "tiers": "", "label": ""}
    return m


def _check_quat_convention() -> None:
    """Pick the body-quaternion order (wxyz or xyzw) for which each pad tip sits within 6 cm of its own TCP."""
    if "wxyz" in _RT:
        return
    env = mk._EP["env"]
    if env is None:
        return
    errs = {}
    for wxyz in (True, False):
        _RT["wxyz"] = wxyz
        pts = robot_points()
        tcps = pts[:2]
        tips = [pts[2], pts[4], pts[6], pts[8]]
        errs[wxyz] = max(min(np.linalg.norm(t - c) for c in tcps) for t in tips)
    _RT["wxyz"] = errs[True] <= errs[False]
    print(f"[rk] body quaternions read as {'wxyz' if _RT['wxyz'] else 'xyzw'} (pad tip to TCP: wxyz "
          f"{errs[True] * 1000:.1f} mm, xyzw {errs[False] * 1000:.1f} mm)", flush=True)


def _rack_still(objects) -> bool:
    return float(np.linalg.norm(np.asarray(objects["rack"].lin_vel))) < STILL


def _gate_step(m, c, low, away, still):
    """The lift gate, one control step. Armed: the reference is taken when the rack's lowest point first rises 2 mm
    above the cloth; from then until the rack is set down away from the pack, any ball centre moving more than 3 mm
    horizontally from the reference latches 0. If the rack comes back down on the cloth (lowest point within 0.5 mm,
    still) before it is away, the reference is dropped and the next rise takes a new one. Returns the event, if any."""
    if m["gate"] is not None or m["cleared"] is not None:
        return None
    if m["ref"] is None:
        if low > GATE_RISE:
            m["ref"] = c.copy()
            return "rising"
        return None
    mv = float(np.linalg.norm((c - m["ref"])[:, :2], axis=1).max())
    m["max_move"] = max(m["max_move"], mv)
    if mv > GATE_MOVE:
        m["gate"] = m["n"]
        return "latched"
    if away:
        m["cleared"] = m["n"]
        return "cleared"
    if low <= GATE_REST and still:
        m["ref"] = None
        m["max_move"] = 0.0
        return "rearmed"
    return None


def _on_step(objects) -> None:
    """Every control step: the lift gate (`_gate_step`), rack clearance, and a trace."""
    if mk._EP.get("env") is None:
        return
    m = _mem()
    m["n"] += 1
    if m["n"] == 2:
        _check_quat_convention()
    c = centres(objects)
    low = rack_low(objects)
    clear = rack_clearance(objects, c)
    ev = _gate_step(m, c, low, rack_away(objects, c), _rack_still(objects))
    if ev == "rising":
        g = pair_gaps(c)
        print(f"[rk] rack rising (lowest point {low * 1000:.1f} mm up) at control step {m['n']}: pairs <= 0.5 mm "
              f"{int((g <= GAP_OK).sum())}/30, widest gap {g.max() * 1000:.2f} mm, head ball "
              f"{_head_off(c) * 1000:.1f} mm from the spot", flush=True)
    elif ev == "latched":
        print(f"[rk] LATCH lift-disturbed: a ball moved {m['max_move'] * 1000:.2f} mm sideways while the rack was rising "
              f"(limit {GATE_MOVE * 1000:.0f} mm), control step {m['n']}, rack lowest point {low * 1000:.1f} mm", flush=True)
    elif ev == "rearmed":
        print(f"[rk] rack back down on the cloth at control step {m['n']}: lift gate re-armed", flush=True)
    elif ev == "cleared":
        apex, rear = orientation(c)
        g = pair_gaps(c)
        print(f"[rk] rack set down {clear * 1000:.0f} mm (horizontally) from the nearest ball at control step {m['n']}: "
              f"pairs <= 0.5 mm {int((g <= GAP_OK).sum())}/30, widest gap {g.max() * 1000:.2f} mm, head ball "
              f"{_head_off(c) * 1000:.1f} mm from the spot; most any ball moved "
              f"while it rose: {m['max_move'] * 1000:.2f} mm; orientation: apex {apex:.2f} deg off +x (limit "
              f"{APEX_TOL:.0f}), rear row {rear:.2f} deg off square (limit {REAR_TOL:.0f})", flush=True)
    if m["label"] != m.get("phase"):
        m["phase"] = m["label"]
        g = pair_gaps(c)
        apex, rear = orientation(c)
        print(f"[rk] k={m['n']} phase '{m['label']}': pairs <= 0.5 mm {int((g <= GAP_OK).sum())}/30, widest gap "
              f"{g.max() * 1000:.2f} mm, head ball {_head_off(c) * 1000:.1f} mm; apex {apex:.2f} deg, rear row "
              f"{rear:.2f} deg; rack low {low * 1000:+.1f} mm, at x {kit.pose(objects, 'rack')[0][0] * 1000:.1f} mm, "
              f"{clear * 1000:+.0f} mm from the balls", flush=True)
    tiers = _tiers(objects, c)
    if tiers != m["tiers"]:
        apex, rear = orientation(c)
        print(f"[rk] k={m['n']} tiers {tiers} (score {score(objects):.2f}); apex {apex:.2f} deg, rear row {rear:.2f} deg",
              flush=True)
        m["tiers"] = tiers
    if DEBUG and m["n"] % 10 == 0:
        g = pair_gaps(c)
        print(f"[rk] k={m['n']} rack {np.round(kit.pose(objects, 'rack')[0][:2] * 1000, 1).tolist()} low {low * 1000:+.1f} mm "
              f"clear {clear * 1000:+.1f}; pairs ok {int((g <= GAP_OK).sum())}/30 widest {g.max() * 1000:.2f} mm; head "
              f"{_head_off(c) * 1000:.1f} mm; hands {hand_clearance(c) * 1000:+.1f} mm; moved {m['max_move'] * 1000:.2f}",
              flush=True)
        pts = robot_points()
        if pts:
            bx = kit.point(objects, "rack", (-L_IN - WALL_T / 2, 0.0, 0.0))
            rel = lambda p: np.round((np.asarray(p) - bx) * 1000, 1).tolist()
            print(f"[rk]    vs bar centre (mm): TCP L {rel(pts[0])} R {rel(pts[1])}; L pad tips {rel(pts[2])} {rel(pts[4])}; "
                  f"R pad tips {rel(pts[6])} {rel(pts[8])}; corr L {np.round(_cl()['corr'][1] * 1000, 1).tolist()} R "
                  f"{np.round(_cl()['corr'][-1] * 1000, 1).tolist()} | {m['label']}", flush=True)


kit.on_step(_on_step)


def _head_off(c) -> float:
    return float(math.dist(c[0][:2], (X_SPOT, Y_SPOT)))


def orientation(c) -> tuple:
    """(apex error, rear-row error) in degrees. Apex: the angle between the vector from the 15 ball centres' centroid
    to the head ball and +x (toward the head of the table), in [0, 180]. Rear row: the angle between the least-squares
    line through the five rear-row centres and the y axis (square to the long axis), in [0, 90]."""
    xy = c[:, :2]
    v = xy[0] - xy.mean(axis=0)
    apex = abs(math.degrees(math.atan2(v[1], v[0])))
    r = xy[REAR] - xy[REAR].mean(axis=0)
    d = np.linalg.svd(r)[2][0]  # principal direction of the rear-row centres
    rear = math.degrees(math.atan2(abs(d[0]), abs(d[1])))
    return apex, rear


def _oriented(c) -> bool:
    apex, rear = orientation(c)
    return apex <= APEX_TOL and rear <= REAR_TOL


def _still(objects) -> bool:
    return all(float(np.linalg.norm(np.asarray(objects[b].lin_vel, float))) < STILL for b in NAMES)


def _tiers(objects, c) -> str:
    g = pair_gaps(c)
    return "".join([
        "S" if _head_off(c) <= SPOT_25 else "-",
        "C" if rack_away(objects, c) else "-",
        "T" if (g <= GAP_OK).sum() >= 24 else "-",
        "A" if (g <= GAP_OK).all() else "-",
        "O" if _oriented(c) else "-",
        "5" if _head_off(c) <= SPOT_100 else "-",
        "H" if hand_clearance(c) >= HAND_CLEAR else "-",
        "G" if _mem()["gate"] is not None else "-",
    ])


def score(objects) -> float:
    """0 if any ball moved more than 3 mm sideways while the rack was rising (latched). Otherwise: 0.25 the head ball's
    centre within 10 mm of the foot spot; 0.5 that and the rack taken away (set down on the table, still, every part of
    it at least 50 mm horizontally from every ball); 0.75 that, at least 24 of the 30 touching pairs within 0.5 mm and the pack facing the head of the table (apex
    within 5 deg of +x from the centroid, rear row within 3 deg of square to the long axis); 1.0 that, all 30 within
    0.5 mm, the head ball within 5 mm of the spot, every ball still and both grippers at least 30 mm from every ball."""
    m = _mem()
    if m["gate"] is not None:
        return 0.0
    c = centres(objects)
    if _head_off(c) > SPOT_25:
        return 0.0
    if not rack_away(objects, c):
        return 0.25
    g = pair_gaps(c)
    if (g <= GAP_OK).sum() < 24 or not _oriented(c):
        return 0.5
    if (g <= GAP_OK).all() and _head_off(c) <= SPOT_100 and _still(objects) and hand_clearance(c) >= HAND_CLEAR:
        return 1.0
    return 0.75


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    c = centres(objects)
    g = pair_gaps(c)
    return float(_head_off(c) + np.clip(g, 0, None).sum() + max(0.0, CLEAR - rack_clearance(objects, c))
                 + max(0.0, float(np.asarray(objects["rack"].low)[2]) - TOP))


# -- the scripted demo -----------------------------------------------------------------------------------------------------
# Both hands pinch the back bar in the cusps between back-row balls 1-2 and 4-5, fingers closing across the bar, and lift
# the rear bar about 1 mm (the apex stays on the cloth); pull it back toward the robot (its apex squeezes the
# pack tight and walks the head ball onto the spot), the last 3.5 mm slowly; let the pack settle; ease the rack 1.5 mm
# forward so its bars stand off the balls; lift it straight up, slowly past the balls' equators; carry it toward the
# robot, set it down on the table well behind the pack, let go and raise the hands.
SP0 = D + G_LOOSE
Y_HAND = 1.5 * SP0
X_BAR0 = VX0 - L_IN - WALL_T / 2
TIP_Z = 0.018  # pad tips this far above the cloth (the bar is 31.75 mm tall)
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402

Z_PINCH = CLOTH_TOP + TIP_Z + PAD_TIP_PAST_TCP
YAW = {1: -math.pi / 2, -1: math.pi / 2}  # per arm (left +y, right -y): fingers close along x, across the bar
W_OPEN, W_HOLD = 0.021, WALL_T - 0.004  # 3 mm clear of each face going down
RAISE = float(os.environ.get("RK_RAISE", "0.001"))  # hands 1 mm up while pulling (rear bar up, apex on the cloth)
PULL = float(os.environ.get("RK_PULL", "0.016"))  # total pull back
PULL_SLOW = 0.0035  # the last part of the pull, at about 1 mm/s
SETTLE = 3.0  # s
EASE = float(os.environ.get("RK_EASE", "0.0015"))  # forward ease-off before the lift: the side bars stand off 0.75 mm
LIFT_SLOW, LIFT = 0.035, 0.100  # the first 35 mm at 5 mm/s (bars past the balls' equators), then at 2 cm/s
# set-down spot: the rack's apex (20.5 mm past its inner vertex) 65 mm horizontally behind the back row's outline
X_APEX_SET = X_SPOT - 4 * ROW - R - CLEAR - 0.015
X_BAR_SET = X_APEX_SET - 0.0205 - L_IN - WALL_T / 2
DRIFT = float(os.environ.get("RK_DRIFT", "0.0"))  # controls only: sideways drift over the first 20 mm of the lift


def fw(w):
    return -w / 2


def _go(plan, dur, pos, w, label, sg):
    plan.append(Waypoint(float(dur), tuple(float(v) for v in pos), fw(w), label, YAW[sg]))


def _arm(sg, pull=PULL):
    y = sg * Y_HAND
    x0 = X_BAR0
    zr = Z_PINCH + RAISE
    p = []
    _go(p, 3.0, (x0, y, Z_PINCH + 0.10), W_OPEN, "over the back bar", sg)
    _go(p, 2.0, (x0, y, Z_PINCH), W_OPEN, "down astride the back bar", sg)
    _go(p, 1.0, (x0, y, Z_PINCH), W_HOLD, "pinch the back bar", sg)
    _go(p, 0.5, (x0, y, zr), W_HOLD, "lift the rear bar 1 mm", sg)
    if pull > 0:
        fast = pull - PULL_SLOW
        _go(p, fast / 0.003, (x0 - fast, y, zr), W_HOLD, "[tighten] pull the rack back: its apex squeezes the pack", sg)
        _go(p, PULL_SLOW / 0.001, (x0 - pull, y, zr), W_HOLD, "[tighten] the last 3.5 mm slowly", sg)
    _go(p, SETTLE, (x0 - pull, y, zr), W_HOLD, "let the pack settle", sg)
    xl = x0 - pull + EASE
    _go(p, 2.5, (xl, y, zr), W_HOLD, "ease the rack 1.5 mm forward, bars off the balls", sg)
    if DRIFT:
        _go(p, 1.0, (xl, y + DRIFT, Z_PINCH + 0.020), W_HOLD, "[ctl-a] lift with sideways drift", sg)
        _go(p, (LIFT_SLOW - 0.020) / 0.005, (xl, y + DRIFT, Z_PINCH + LIFT_SLOW), W_HOLD, "[lift] lift slowly past the equators", sg)
    else:
        _go(p, LIFT_SLOW / 0.005, (xl, y, Z_PINCH + LIFT_SLOW), W_HOLD, "[lift] lift slowly past the equators", sg)
    _go(p, (LIFT - LIFT_SLOW) / 0.02, (xl + 0, p[-1].pos[1], Z_PINCH + LIFT), W_HOLD, "[lift] lift it clear", sg)
    xs = X_BAR_SET
    _go(p, 6.0, (xs, y, Z_PINCH + LIFT), W_HOLD, "[carry] carry the rack toward the robot", sg)
    _go(p, 2.5, (xs, y, Z_PINCH - CLOTH_T + 0.0015), W_HOLD, "[set down] set it on the table", sg)
    _go(p, 1.0, (xs, y, Z_PINCH - CLOTH_T + 0.0015), W_OPEN, "let go", sg)
    _go(p, 2.0, (xs, y, Z_PINCH + 0.10), W_OPEN, "hands up", sg)
    return p


# The demo may read sim state (the Guide; v21): each hand's target is corrected live by an integrator on the measured
# TCP error in x and z (the arms sit ~5 mm toward the robot when low over the table), from the descent onward.
CL_GAIN, CL_MAX = 0.3, 0.012
CL_KEY = "rk_cl"


def _cl() -> dict:
    return mk.MEMORY.setdefault(CL_KEY, {"corr": {1: np.zeros(3), -1: np.zeros(3)}, "target": {}, "on": {1: False, -1: False}})


def _patch_arm_plan() -> None:
    from yam_arena import demo as _demo

    cls = _demo.ArmPlan
    if getattr(cls, "_rk_cl", False):
        return
    target0 = cls.target

    def target(self, t):
        pos, rot, finger, label = target0(self, t)
        if mk._EP.get("env") is None:
            return pos, rot, finger, label
        sg = 1 if self.base_pos[1] > 0 else -1
        cl = _cl()
        cl["target"][sg] = np.asarray(pos, float).copy()
        cl["on"][sg] = label != "over the back bar"
        if sg == 1:
            _mem()["label"] = label
        return pos + cl["corr"][sg], rot, finger, label

    cls.target = target
    cls._rk_cl = True


_patch_arm_plan()


def _cl_step(_objects) -> None:
    if mk._EP.get("env") is None:
        return
    cl = _cl()
    t = mk.tcps()
    for sg, tcp in zip((1, -1), t):
        if sg in cl["target"] and cl["on"][sg]:
            err = cl["target"][sg] - tcp
            err[1] = 0.0  # x and z only (along the bar a few mm do not matter)
            cl["corr"][sg] = np.clip(cl["corr"][sg] + CL_GAIN * err, -CL_MAX, CL_MAX)


kit.on_step(_cl_step)


def demo():
    """Both hands: pinch the back bar between back-row balls and lift it about 1 mm; pull it back 16 mm (12.5 mm at
    3 mm/s, the last 3.5 mm at 1 mm/s) to tighten; let the pack settle 3 s; ease the rack 1.5 mm forward; lift it
    straight up (35 mm at 5 mm/s, then to 100 mm at 2 cm/s); carry it toward the robot, set it on the table with its
    apex 65 mm behind the pack, let go and raise the hands."""
    return {"left": _arm(1), "right": _arm(-1)}


def controls_no_tighten():
    """Control (b): the same pinch and lift with no pull back (run with RK_START_PAST=0 so the head ball starts on the
    spot): the pack keeps its gaps."""
    return {"left": _arm(1, pull=0.0), "right": _arm(-1, pull=0.0)}


# -- offline self-test --------------------------------------------------------------------------------------------------
def _states(ball_xy, rack_shift=(0.0, 0.0, 0.0), moving=False, rack_moving=False):
    import dataclasses

    from yam_arena.task_file import ObjectState

    pos0, q0 = kit.pose(kit.fake_states(), "rack")
    st = kit.fake_states(moved={"rack": (np.asarray(pos0) + np.asarray(rack_shift), q0)})
    if rack_moving:
        st["rack"] = dataclasses.replace(st["rack"], lin_vel=np.array([-0.02, 0.0, 0.0]))
    z = np.zeros(3)
    for name, (x, y) in zip(NAMES, ball_xy):
        p = np.array([x, y, CLOTH_TOP + R])
        st[name] = ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]),
                               lin_vel=np.array([0.02, 0, 0]) if moving else z, ang_vel=z, low=p - R, high=p + R)
    return st


def self_test():
    global _TEST_POINTS
    tight = [p for row in lattice(X_SPOT, Y_SPOT, D) for p in row]
    tight_off7 = [p for row in lattice(X_SPOT + 0.007, Y_SPOT, D) for p in row]
    loose_on = [p for row in lattice(X_SPOT, Y_SPOT, D + G_LOOSE) for p in row]
    some = [list(p) for p in tight]
    for k in range(10, 15):  # back row pulled 1 mm back: 5 pairs of the back row + 4 diagonals... count below
        some[k][0] -= 0.001
    up = (0.0, 0.0, 0.10)  # lifted 100 mm straight up: above the pack, not taken away
    away = (X_BAR_SET - X_BAR0, 0.0, -(CLOTH_T + 0.0003))  # set down on the table where the demo puts it
    near = (X_BAR_SET - X_BAR0 + 0.040, 0.0, -(CLOTH_T + 0.0003))  # set down 40 mm short of that (25 mm from the pack)

    def turned(deg):  # the tight pack rotated about the head ball, which stays on the spot
        a = math.radians(deg)
        return [(X_SPOT + (x - X_SPOT) * math.cos(a) - (y - Y_SPOT) * math.sin(a),
                 Y_SPOT + (x - X_SPOT) * math.sin(a) + (y - Y_SPOT) * math.cos(a)) for x, y in tight]
    cases = {
        "start (loose, 15 mm past, rack on)": (_states(START), {}, [], 0.0),
        "tight on the spot, rack still on": (_states(tight, (-(START_PAST), 0, 0)), {}, [], 0.25),
        "tight on the spot, rack lifted 100 mm straight up (not taken away)": (_states(tight, up), {}, [], 0.25),
        "tight on the spot, rack carried away but still 60 mm up": (_states(tight, (away[0], 0, 0.06)), {}, [], 0.25),
        "tight on the spot, rack set down 25 mm behind the pack": (_states(tight, near), {}, [], 0.25),
        "tight on the spot, rack set down away but still sliding": (_states(tight, away, rack_moving=True), {}, [], 0.25),
        "loose on the spot, rack set down away": (_states(loose_on, away), {}, [], 0.5),
        "tight on the spot, rack set down away, still, hands away": (_states(tight, away), {}, [], 1.0),
        "tight, head 7 mm off the spot, rack away": (_states(tight_off7, away), {}, [], 0.75),
        "tight, rack away, balls rolling": (_states(tight, away, moving=True), {}, [], 0.75),
        "tight, rack away, a pad touching the head ball": (_states(tight, away), {}, [np.array([X_SPOT + R + 0.005, 0, CLOTH_TOP + R])], 0.75),
        "tight but the lift disturbed it (latched)": (_states(tight, away), {"gate": 5}, [], 0.0),
        "back row 1 mm off, rack away": (_states(some, away), {}, [], None),
        "orientation: correct pack, rotated 2 deg (inside both limits)": (_states(turned(2.0), away), {}, [], 1.0),
        "orientation: rotated 180 deg, head ball on the spot (apex faces the foot)": (_states(turned(180.0), away), {}, [], 0.5),
        "orientation: rotated 10 deg, head ball on the spot": (_states(turned(10.0), away), {}, [], 0.5),
        "orientation: rotated 4 deg (apex inside 5, rear row outside 3)": (_states(turned(4.0), away), {}, [], 0.5),
    }
    try:
        for name, (st, flags, pts, want) in cases.items():
            mk.new_episode()
            _mem().update(flags)
            _TEST_POINTS = pts
            got = score(st)
            if want is None:
                g = pair_gaps(centres(st))
                want = 0.75 if (g <= GAP_OK).sum() >= 24 else 0.5
                print(f"   ({name}: {int((g <= GAP_OK).sum())}/30 pairs tight -> {want})", file=sys.stderr)
            assert abs(got - want) < 1e-9 and success(st) == (want >= 1.0), (name, got, want)
        g = pair_gaps(np.array([[x, y, 0] for x, y in tight]))
        assert np.abs(g).max() < 1e-9
        # lift gate sequences: (lowest point m, ball shift m, away, still) per step -> latched?
        c0 = np.array([[x, y, CLOTH_TOP + R] for x, y in tight])
        seqs = {
            "rise, a ball moves 4 mm, rack set down away": ([(0.003, 0, 0, 0), (0.02, 0.004, 0, 0), (-0.003, 0.004, 1, 1)], True),
            "rise, balls still, rack set down away": ([(0.003, 0, 0, 0), (0.05, 0.001, 0, 0), (-0.003, 0.001, 1, 1)], False),
            "rise, back flat on the cloth, a ball moves 4 mm, rise again": (
                [(0.003, 0, 0, 0), (0.0, 0, 0, 1), (0.0, 0.004, 0, 1), (0.003, 0.004, 0, 0), (0.05, 0.0045, 0, 0)], False),
            "rise, back down but still moving (not re-armed), a ball moves 4 mm": (
                [(0.003, 0, 0, 0), (0.0, 0, 0, 0), (0.0, 0.004, 0, 0)], True),
            "set down away, then a ball moves 4 mm (gate over)": ([(0.003, 0, 0, 0), (-0.003, 0, 1, 1), (-0.003, 0.004, 1, 1)], False),
        }
        for name, (steps, latched) in seqs.items():
            m = {"n": 0, "ref": None, "gate": None, "max_move": 0.0, "cleared": None}
            for low_s, dx, away_s, still_s in steps:
                m["n"] += 1
                cc = c0.copy()
                cc[7, 0] += dx  # ball 7 (fourth row)
                _gate_step(m, cc, low_s, bool(away_s), bool(still_s))
            assert (m["gate"] is not None) == latched, (name, m)
        print(f"lift gate: {len(seqs)} sequences pass", file=sys.stderr)
        for deg, want in ((0.0, (0.0, 0.0)), (180.0, (180.0, 0.0)), (10.0, (10.0, 10.0)), (-4.0, (4.0, 4.0))):
            o = orientation(np.array([[x, y, 0] for x, y in turned(deg)]))
            assert abs(o[0] - want[0]) < 1e-6 and abs(o[1] - want[1]) < 1e-6, (deg, o)
            assert np.abs(pair_gaps(np.array([[x, y, 0] for x, y in turned(deg)]))).max() < 1e-9  # still tight
            print(f"   (pack turned {deg:+.0f} deg about the head ball: apex {o[0]:.2f} deg, rear row {o[1]:.2f} deg, "
                  f"oriented {_oriented(np.array([[x, y, 0] for x, y in turned(deg)]))})", file=sys.stderr)
        assert len(PAIRS) == 30
        st = _states(START)
        print(f"start: rack clearance {rack_clearance(st, centres(st)) * 1000:.2f} mm (balls touching the V), widest gap "
              f"{pair_gaps(centres(st)).max() * 1000:.2f} mm, head ball {_head_off(centres(st)) * 1000:.1f} mm past the spot; "
              f"back bar centre x {X_BAR0:.4f}", file=sys.stderr)
        assert rack_clearance(st, centres(st)) > -1e-6
        for name, sh in (("away", away), ("near", near)):
            sa = _states(tight, sh)
            print(f"rack set down {name}: {rack_clearance(sa, centres(sa)) * 1000:.1f} mm horizontally from the pack, lowest "
                  f"point {(float(sa['rack'].low[2]) - TOP) * 1000:.1f} mm above the table, nearest rack x "
                  f"{float(sa['rack'].low[0]):.3f}..{float(sa['rack'].high[0]):.3f}", file=sys.stderr)
        assert float(_states(tight, away)["rack"].low[0]) > TABLE_X0 + 0.005  # on the table
    finally:
        mk.new_episode()
        _TEST_POINTS = None
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan

    print(f"scoring: {self_test()} cases pass")
    hz = 20.0
    for kind, fn in (("demo", demo), ("control b (not the demo)", controls_no_tighten)):
        for arm, waypoints in fn().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (kind, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (kind, arm)
            print(f"{kind} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
