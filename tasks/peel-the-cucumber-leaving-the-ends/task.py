"""On the station table: a wooden chopping board, and lying loose on it a whole slicing cucumber with dark-green skin,
21 cm long and 4.2 cm thick, with rounded ends. On the table to the robot's right of
the board lies a stainless Y-peeler with a black handle: its short blade spans the two arms of the Y and swivels on
pins at its ends. A strip of skin comes off wherever the blade's sharp lip is drawn along the cucumber in light
contact, edge first, and shows the pale flesh beneath.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402  (before lots, so parts' reset hooks run before the lots' moves)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

WOOD, SKIN, FLESH, STEEL, RUBBER = ((0.70, 0.52, 0.32), (0.11, 0.28, 0.09), (0.80, 0.88, 0.58),
                                    (0.80, 0.81, 0.83), (0.12, 0.12, 0.13))

# -- the board (fixed) --------------------------------------------------------------------------------------------
BOARD_SIZE = (0.35, 0.25, 0.02)
BOARD_POS = (0.33, 0.0, TOP + BOARD_SIZE[2] / 2)
BOARD_TOP = TOP + BOARD_SIZE[2]
BOARD = parts.Part("board", pos=BOARD_POS, shapes=[parts.box(BOARD_SIZE, color=WOOD)], friction=(0.5, 0.4))

# -- the cucumber: its frame has x along the body, origin at the middle; phi is measured round the body from its own
# +y toward its own +z. The body is 42 mm across for its middle 17 cm, with rounded ends. Its collider is a 24-sided
# prism (corners 21.2 mm out, flats 21.0 mm), so it lies still where it is put. -------------------------------------
HALF_LEN = 0.106
PRISM_R, PRISM_SIDES, PRISM_HALF = 0.0212, 24, 0.086
SEGMENTS = ((0.000, PRISM_HALF, 0.0210),)
TIP_R, TIP_C = 0.0205, 0.0855  # rounded ends: a 20.5 mm sphere centred 85.5 mm out, so the tips are 106 mm out
R_MAX = SEGMENTS[0][2]
SCORED = 0.060  # the middle 12 cm (|s| <= 60 mm) is scored skin
END = 0.080  # the last 2.6 cm at each end (|s| >= 80 mm) must keep its skin
N_STRIP, N_BAND = 8, 6
PEEL_FRACTION = 0.8  # a patch is peeled when at least 80 % of its skin is cut away
CUT_W = 0.016  # one stroke takes a strip 16 mm wide, as a 5 cm Y-peeler blade does on a 4 cm cucumber
F_MIN, F_MAX = 1.0, 10.0  # N on the blade's edge: 1 to 10 N peels, more than 10 N gouges
HOLD_STEPS = 20  # 1 s at 20 Hz: let go and still this long before full credit
V_MIN = 0.01  # m/s along the body, edge first
EPS = 1e-6


def radius(s: float) -> float:
    """Skin radius (m) at s (m) along the body; 0 past the tips."""
    a = abs(s)
    for lo, hi, r in SEGMENTS:
        if lo - EPS <= a <= hi + EPS:
            return r
    d = a - TIP_C
    return math.sqrt(max(TIP_R * TIP_R - d * d, 0.0)) if d < TIP_R else 0.0  # (the tip spheres)


def _cucumber_shapes():
    out = [parts.prism(PRISM_R, 2 * PRISM_HALF, PRISM_SIDES, color=FLESH)]
    out += [parts.sphere(TIP_R, at=(sgn * TIP_C, 0, 0), color=FLESH) for sgn in (1, -1)]
    return out


# Skin: visual meshes 0.5 mm outside the flesh, one per patch (scored and margin bands) and one per end.
SKIN_OUT = 0.0005
BANDS = [(-SCORED + k * 2 * SCORED / N_BAND, -SCORED + (k + 1) * 2 * SCORED / N_BAND) for k in range(N_BAND)]
MARGINS = [(-END, -SCORED), (SCORED, END)]


def _skin_r(s: float) -> float:
    return max(radius(s - 1e-4), radius(s + 1e-4), PRISM_R if abs(s) <= PRISM_HALF + EPS else 0.0) + SKIN_OUT


def _patch_mesh(s0, s1, p0, p1, ns=8, nphi=6):
    pts, tris = [], []
    for i in range(ns + 1):
        s = s0 + (s1 - s0) * i / ns
        r = _skin_r(s)
        for j in range(nphi + 1):
            ph = p0 + (p1 - p0) * j / nphi
            pts.append((s, r * math.cos(ph), r * math.sin(ph)))
    for i in range(ns):
        for j in range(nphi):
            a, b = i * (nphi + 1) + j, (i + 1) * (nphi + 1) + j
            tris += [(a, b, b + 1), (a, b + 1, a + 1)]
    return tuple(pts), tuple(tris)


def _end_mesh(sgn):
    """The skin of one end: the body from 75 to 86 mm out and the rounded tip."""
    pts, tris, rows = [], [], []
    nphi = 24
    ring = [(END, _skin_r(END)), (PRISM_HALF, _skin_r(PRISM_HALF)), (TIP_C, TIP_R + SKIN_OUT)]
    for k in range(1, 9):
        a = math.pi / 2 * k / 8
        ring.append((TIP_C + (TIP_R + SKIN_OUT) * math.sin(a), (TIP_R + SKIN_OUT) * math.cos(a)))
    for s, r in ring:
        row = []
        for j in range(nphi):
            ph = 2 * math.pi * j / nphi
            row.append(len(pts))
            pts.append((sgn * s, r * math.cos(ph), r * math.sin(ph)))
        rows.append(row)
    for a_row, b_row in zip(rows, rows[1:]):
        for j in range(nphi):
            j2 = (j + 1) % nphi
            tris += [(a_row[j], b_row[j], b_row[j2]), (a_row[j], b_row[j2], a_row[j2])]
    return tuple(pts), tuple(tris)


def _skin_visuals():
    out = []
    dphi = 2 * math.pi / N_STRIP
    for b, (s0, s1) in enumerate(BANDS):
        for k in range(N_STRIP):
            out.append(parts.Visual(f"p{b}_{k}", *_patch_mesh(s0, s1, k * dphi, (k + 1) * dphi), color=SKIN))
    for m, (s0, s1) in enumerate(MARGINS):
        for k in range(N_STRIP):
            out.append(parts.Visual(f"m{m}_{k}", *_patch_mesh(s0, s1, k * dphi, (k + 1) * dphi, ns=4), color=SKIN))
    out += [parts.Visual(f"end{m}", *_end_mesh(sgn), color=SKIN) for m, sgn in enumerate((-1, 1))]
    return tuple(out)


def _q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def cucumber_quat(yaw: float, roll: float):
    """Lying along heading `yaw`, turned `roll` about its own length."""
    return _q_mul((0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)), (math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2)))


CUC_Z = BOARD_TOP + PRISM_R * math.cos(math.pi / PRISM_SIDES)  # resting on a flat
LAYOUT = {  # lot: (centre x, y, heading, roll)
    "A": (0.315, 0.075, 0.0, 0.30),
    "B": (0.285, 0.090, 0.0, 2.40),
    "C": (0.300, 0.060, -0.20, 4.40),
}
CUCUMBER = parts.Part(
    "cucumber", pos=(LAYOUT["A"][0], LAYOUT["A"][1], CUC_Z), quat=cucumber_quat(LAYOUT["A"][2], LAYOUT["A"][3]),
    shapes=_cucumber_shapes(), mass=0.300, friction=(0.6, 0.5), report=("base",), visuals=_skin_visuals(),
    angular_damping=5.0,  # 1/s: a real cucumber is not perfectly round and does not roll on by itself
)

# -- the Y-peeler. Frame: origin on the blade's swivel axis, x toward the handle, y along the blade, z out of the
# peeler's flat. The sharp lip is the blade's handle-side lip and faces the handle: it cuts when the peeler is drawn
# handle first. --------------------------------------------------------------------------------------------------
HANDLE = (0.105, 0.024, 0.020)
NECK = 0.045  # the Y's arms meet the handle this far behind the blade
HANDLE_X = NECK + HANDLE[0] / 2
EDGE_AT = (0.00325, 0.0, -0.0005)


def _arm_box(y_sign):
    a, b = np.array([0.006, y_sign * 0.0285]), np.array([NECK, y_sign * 0.009])
    mid, d = (a + b) / 2, b - a
    ang = math.atan2(d[1], d[0])
    return parts.box((float(np.linalg.norm(d)) + 0.002, 0.004, 0.005), at=(float(mid[0]), float(mid[1]), 0.0),
                     color=STEEL, quat=(0.0, 0.0, math.sin(ang / 2), math.cos(ang / 2)))


PEELER_POS = (0.330, -0.170, TOP + HANDLE[2] / 2)  # lying flat on the table, handle toward +x
BLADE = parts.Hinge(
    "blade",
    shapes=[parts.box((0.003, 0.045, 0.001), at=(-0.0035, 0, 0), color=STEEL),  # guard lip
            parts.box((0.010, 0.003, 0.001), at=(0, 0.0245, 0), color=STEEL),  # end tabs on the pins
            parts.box((0.010, 0.003, 0.001), at=(0, -0.0245, 0), color=STEEL)],
    pivot=(0.0, 0.0, 0.0), axis=(0.0, 1.0, 0.0), lower=-0.60, upper=0.60,
    mass=0.005, spring=0.005, damping=0.001, max_effort=1.0,
)
EDGE = parts.Fixed("edge", shapes=[parts.box((0.0025, 0.045, 0.001), at=EDGE_AT, color=STEEL)], at=EDGE_AT,
                   mass=0.005, parent="blade")
PEELER = parts.Part(
    "peeler", pos=PEELER_POS,
    shapes=[parts.box(HANDLE, at=(HANDLE_X, 0, 0), color=RUBBER),
            parts.box((0.012, 0.004, 0.006), at=(0, 0.0285, 0), color=STEEL),
            parts.box((0.012, 0.004, 0.006), at=(0, -0.0285, 0), color=STEEL),
            _arm_box(1), _arm_box(-1)],
    joints=[BLADE, EDGE], mass=0.040, friction=(1.2, 1.0), contact_offset=0.002, report=("edge", "blade", "base"),
    link_friction=(("blade", (0.35, 0.3)), ("edge", (0.35, 0.3))),  # rubber handle, polished steel blade
)

OBJECTS = [CUCUMBER, PEELER, BOARD]
LOTS = lots.declare(
    OBJECTS,
    *(lots.Lot(k, place={"cucumber": (x, y, CUC_Z)}, quat={"cucumber": cucumber_quat(h, r)})
      for k, (x, y, h, r) in LAYOUT.items()),
)

# -- run-time state, kept on the shared parts module (offline tests that import this file again see the same one) --
N_S = 212  # 1 mm cells along the body, tip to tip
N_PHI = 48  # 7.5 degree cells round it
S_C = (np.arange(N_S) + 0.5) / 1000 - HALF_LEN
PHI_C = (np.arange(N_PHI) + 0.5) * 2 * math.pi / N_PHI
_S = parts.__dict__.setdefault("peel_state", {})


def reset_state() -> None:
    _S.clear()
    _S.update(cut=np.zeros((N_S, N_PHI), bool), gouge=np.zeros((N_S, N_PHI), bool), prev=None, known=False,
              broken=False, blade_hit_hand=False, dropped=False, clear=False, still_steps=0, hidden=set(), views=None,
              t=0.0)


if "cut" not in _S:
    reset_state()


def _cells_s(s0: float, s1: float):
    lo, hi = min(s0, s1), max(s0, s1)
    return (S_C >= lo - 0.0005) & (S_C <= hi + 0.0005)


def _cells_phi(p0: float, p1: float, half: float):
    """Cells within `half` (rad) of the arc from p0 to p1 (the short way round)."""
    d = (p1 - p0 + math.pi) % (2 * math.pi) - math.pi
    lo, hi = p0 + min(0.0, d) - half, p0 + max(0.0, d) + half
    rel = (PHI_C - lo) % (2 * math.pi)
    return rel <= (hi - lo) + 1e-9


def record_contact(s: float, phi: float, force: float, facing: float, across: float, t: float) -> None:
    """One control step's edge contact (cucumber frame): s along the body, phi round it, the edge's normal force (N),
    `facing` = the edge's facing direction along the body (cos of the angle, signed), `across` = |cos| between the
    blade's length and the body. A cut needs this step and the one before both in the force window, the edge first."""
    prev = _S["prev"]
    r = max(radius(s), 0.015)
    half = (CUT_W / 2) / r
    if force > F_MAX + EPS:
        _S["gouge"][np.ix_(_cells_s(s - 0.002, s + 0.002), _cells_phi(phi, phi, half))] = True
    ok = F_MIN - EPS <= force <= F_MAX + EPS and abs(facing) >= math.cos(math.radians(45)) - EPS and across <= 0.71
    if ok and prev is not None:
        ps, pphi, pt = prev
        dt = max(t - pt, 1e-3)
        ds = s - ps
        dphi = abs((phi - pphi + math.pi) % (2 * math.pi) - math.pi)
        if (ds * facing > 0 and abs(ds) / dt >= V_MIN - EPS and abs(ds) <= 0.03 and dphi <= math.radians(15)):
            _S["cut"][np.ix_(_cells_s(ps, s), _cells_phi(pphi, phi, half))] = True
    _S["prev"] = (s, phi, t) if ok else None


def no_contact() -> None:
    _S["prev"] = None


def _frac(mask_s, k):
    dphi = 2 * math.pi / N_STRIP
    mp = (PHI_C >= k * dphi) & (PHI_C < (k + 1) * dphi)
    return _S["cut"][np.ix_(mask_s, mp)].mean(), _S["gouge"][np.ix_(mask_s, mp)].any()


def patches():
    """(peeled, gouged) counts over the 48 scored patches."""
    peeled = gouged = 0
    for s0, s1 in BANDS:
        ms = (S_C > s0) & (S_C < s1)
        for k in range(N_STRIP):
            f, g = _frac(ms, k)
            gouged += bool(g)
            peeled += bool(f >= PEEL_FRACTION - EPS and not g)
    return peeled, gouged


def ends_cut() -> bool:
    return bool(_S["cut"][np.abs(S_C) >= END].any() or _S["gouge"][np.abs(S_C) >= END].any())


# -- simulator side: contact reports, the release check and the skin's look ---------------------------------------
CLEAR = 0.005
HAND_LINKS = ("tip_left", "tip_right", "gripper")


def _arr(x) -> np.ndarray:
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy())
    return np.asarray(x)


def _make_views(sim):
    from isaaclab.sim.utils import get_current_stage
    from isaaclab_physx.physics import PhysxManager
    from pxr import UsdPhysics

    sv = PhysxManager.get_physics_sim_view()
    peeler, cuc = parts._PATHS["peeler"], parts._PATHS["cucumber"]
    env = peeler.rsplit("/", 1)[0]
    hands = []
    for prim in get_current_stage().GetPrimAtPath(env).GetAllChildren():
        if "Arm" not in prim.GetName():
            continue
        for p in [prim, *list(_descend(prim))]:
            if p.GetName() in HAND_LINKS and p.HasAPI(UsdPhysics.RigidBodyAPI):
                hands.append(p.GetPath().pathString)
    edge = sv.create_rigid_contact_view(f"{peeler}/edge", filter_patterns=[cuc], max_contact_data_count=64)
    guard = [sv.create_rigid_contact_view(f"{peeler}/{n}", filter_patterns=hands, max_contact_data_count=64)
             for n in ("edge", "blade")] if hands else []
    print(f"[peel] contact views: edge->cucumber, blade/edge->{len(hands)} hand links", file=sys.stderr, flush=True)
    if not hands:  # the blade-touches-a-gripper rule cannot be judged: no credit for this trial
        _S["broken"] = True
    return {"edge": edge, "guard": guard, "n_hands": len(hands)}


def _descend(prim):
    for c in prim.GetAllChildren():
        yield c
        yield from _descend(c)


def _hand_hulls(base):
    from yam_arena import analysis

    origin = parts._np(base.scene.env_origins[0])
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in HAND_LINKS:
                m = np.eye(4)
                m[:3, :3] = parts._quat_to_matrix(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    return hulls


def _boxes(spec, st):
    poses = {"base": (np.asarray(st.pos), parts._quat_to_matrix(st.quat_xyzw))}
    for name, (p, q) in getattr(st, "links", {}).items():
        poses[name] = (np.asarray(p), parts._quat_to_matrix(q))
    return spec.world_boxes(poses)


@parts.on_reset
def _on_reset(sim) -> None:
    views = _S.get("views")
    hidden = set(_S.get("hidden", ()))
    reset_state()
    _S["views"] = views
    for v in hidden:
        try:
            sim.set_visible("cucumber", v, True)
        except Exception as exc:  # noqa: BLE001
            print(f"[peel] could not show {v}: {exc}", file=sys.stderr)


@parts.on_step
def _on_step(sim) -> None:
    from yam_arena import analysis

    try:
        base = sim.env.unwrapped
        if _S.get("views") is None:
            _S["views"] = _make_views(sim)
        v = _S["views"]
        dt = base.physics_dt
        states = sim.states()
        cu, pe = states["cucumber"], states["peeler"]
        t = float(sim.t)
        # the edge on the cucumber: normal force and where it touches, as the simulator reports them
        fm = _arr(v["edge"].get_contact_force_matrix(dt=dt)).reshape(-1, 3)
        force = float(np.linalg.norm(fm.sum(axis=0)))
        if force > 0.2:
            f, pts, _, _, cnt, start = (_arr(x) for x in v["edge"].get_contact_data(dt=dt))
            n, i0 = int(cnt.reshape(-1)[0]), int(start.reshape(-1)[0])
            w = np.abs(f.reshape(-1)[i0:i0 + n])
            p = pts.reshape(-1, 3)[i0:i0 + n] - parts._np(base.scene.env_origins[0])
            p = (w[:, None] * p).sum(0) / w.sum() if w.sum() > 0 else p.mean(0)
            R = parts._quat_to_matrix(cu.quat_xyzw)
            loc = R.T @ (p - np.asarray(cu.pos))
            s, phi = float(loc[0]), float(math.atan2(loc[2], loc[1]) % (2 * math.pi))
            Re = parts._quat_to_matrix(pe.links["edge"][1])
            facing = float(Re[:, 0] @ R[:, 0])
            across = abs(float(Re[:, 1] @ R[:, 0]))
            record_contact(s, phi, force, facing, across, t)
        else:
            no_contact()
        for g in v["guard"]:
            if float(np.linalg.norm(_arr(g.get_contact_force_matrix(dt=dt)).reshape(-1, 3), axis=1).max()) > 0.5:
                _S["blade_hit_hand"] = True
        if lowest_z(cu) < BOARD_TOP - 0.003:
            _S["dropped"] = True
        hulls = _hand_hulls(base)
        boxes = _boxes(CUCUMBER, cu) + _boxes(PEELER, pe)
        _S["clear"] = bool(hulls) and all(analysis.hulls_to_box(hulls, c, r, sz)[0] > CLEAR for _, c, r, sz in boxes)
        _S["known"] = True
        still = _S["clear"] and cu.resting() and pe.resting() and abs(lowest_z(cu) - BOARD_TOP) <= 0.003 + EPS
        _S["still_steps"] = _S["still_steps"] + 1 if still else 0
        _update_skin(sim)
    except Exception as exc:  # unreadable contacts or arms: no credit for this trial
        _S["broken"] = True
        print(f"[peel] step check failed: {exc!r}", file=sys.stderr)


def _update_skin(sim) -> None:
    want = set()
    for b, (s0, s1) in enumerate(BANDS):
        ms = (S_C > s0) & (S_C < s1)
        for k in range(N_STRIP):
            f, _ = _frac(ms, k)
            if f >= PEEL_FRACTION - EPS:
                want.add(f"p{b}_{k}")
    for m, (s0, s1) in enumerate(MARGINS):
        ms = (S_C > s0) & (S_C < s1)
        for k in range(N_STRIP):
            if _frac(ms, k)[0] >= PEEL_FRACTION - EPS:
                want.add(f"m{m}_{k}")
    for m, sgn in enumerate((-1, 1)):
        if _S["cut"][sgn * S_C >= END].any():
            want.add(f"end{m}")
    for name in want - _S["hidden"]:
        sim.set_visible("cucumber", name, False)
        _S["hidden"].add(name)


# -- scoring -------------------------------------------------------------------------------------------------------


def lowest_z(cu) -> float:
    """The cucumber's lowest point (m), from its pose and true round shape (a turned body's bounding box is bigger)."""
    a = parts._quat_to_matrix(cu.quat_xyzw)[:, 0]
    side = math.sqrt(max(0.0, 1.0 - a[2] ** 2))
    return float(min(cu.pos[2] + a[2] * s - radius(s) * side for s in np.linspace(-0.0995, 0.0995, 41)))



def _left_alone(objects) -> bool:
    cu, pe = objects["cucumber"], objects["peeler"]
    return bool(_S["clear"] and cu.resting() and pe.resting() and abs(lowest_z(cu) - BOARD_TOP) <= 0.003 + EPS)


def _ruled_out() -> bool:
    return bool(not _S["known"] or _S["broken"] or _S["blade_hit_hand"] or _S["dropped"] or ends_cut())


def score(objects):
    """Peeled patches of the middle skin, counted once both hands have let go and everything is still."""
    if _ruled_out() or not _left_alone(objects):
        return 0.0
    peeled, gouged = patches()
    if peeled >= 44 and gouged == 0 and _S["still_steps"] >= HOLD_STEPS:
        return 1.0
    if peeled >= 32 and gouged <= 2:
        return 0.5
    if peeled >= 12:
        return 0.25
    return 0.0


def success(objects):
    return score(objects) == 1.0


# -- demo: right arm peels, left arm turns the cucumber between strokes. Open loop from the lot's visible start pose.
LEAN = 36.0  # degrees: the handle rises toward the hand
GRIP_AT = np.array([0.065, 0.0, 0.005])  # where the TCP holds the peeler, peeler frame
GRIP_W = HANDLE[1] / 2 - 0.007  # fingers closed 7 mm past the handle's faces
PRESS = 0.024  # TCP commanded below first contact (arm softness sets force)
ROLL = math.radians(30)  # each turn of the cucumber (two of its 15-degree flats); turns vary, so strokes overlap
PUSH = 2 * R_MAX * ROLL / 0.75  # the fingertips slip: on the GPU a push turns it about 3/4 of pure rolling
STROKES = 12
S_START, S_END = -0.063, 0.070  # blade centre along the body
UP = np.array([0.0, 0.0, 1.0])


def _rz(a):
    return parts._rot((0, 0, 1), a)


def _tcp_for_hinge(hinge, heading, lean_deg):
    r = _rz(heading) @ parts._rot((0, 1, 0), -math.radians(lean_deg))
    return tuple(float(v) for v in np.asarray(hinge) + r @ GRIP_AT)


def _rw(dur, pos, finger, label, heading=0.0, lean=0.0):
    if lean == 0.0:
        return tilt.Waypoint(dur, tuple(float(v) for v in pos), finger, label, yaw=heading)
    return tilt.Waypoint(dur, tuple(float(v) for v in pos), finger, label, yaw=heading, tilt=lean, heading=heading)


def demo():
    lot = lots.current()
    x0, y0, _ = lot.pos("cucumber")  # the start pose a camera sees: where the cucumber lies and which way it points
    along = parts._quat_to_matrix(lot.quat_of("cucumber"))[:, 0]
    head = math.atan2(along[1], along[0])
    a = np.array([math.cos(head), math.sin(head), 0.0])  # along the body
    n = np.array([-math.sin(head), math.cos(head), 0.0])  # the robot's-left side of it
    shift = R_MAX * ROLL  # the axis moves this far toward -n with every turn
    grip = -GRIP_W
    pp = np.asarray(PEELER_POS)
    pick = pp + GRIP_AT
    right = [
        _rw(2.0, pick + 0.08 * UP, FINGER_OPEN, "over the peeler"),
        _rw(1.5, pick, FINGER_OPEN, "down to the handle"),
        _rw(0.8, pick, grip, "grip"),
        _rw(1.2, pick + 0.10 * UP, grip, "lift"),
    ]
    left = [tilt.Waypoint(5.5, (0.22, 0.26, TOP + 0.20), FINGER_CLOSED, "wait")]
    t_r, t_l = sum(w.duration for w in right), sum(w.duration for w in left)
    for k in range(STROKES):
        c = np.array([x0, y0, CUC_Z]) - n * shift * k

        def hinge(s):
            return c + a * s + UP * (radius(s) + 0.001)

        start, end = hinge(S_START), hinge(S_END)
        seg = [
            _rw(2.0, np.add(_tcp_for_hinge(start, head, LEAN), 0.03 * UP), grip, f"over stroke {k + 1}", head, LEAN),
            _rw(1.0, np.add(_tcp_for_hinge(start, head, LEAN), -PRESS * UP), grip, "blade down", head, LEAN),
            _rw(0.4, np.add(_tcp_for_hinge(start, head, LEAN), -PRESS * UP), grip, "settle", head, LEAN),
            _rw(3.8, np.add(_tcp_for_hinge(end, head, LEAN), -PRESS * UP), grip, f"stroke {k + 1}", head, LEAN),
            _rw(0.6, np.add(_tcp_for_hinge(end, head, LEAN), 0.025 * UP), grip, "lift off", head, LEAN),
        ]
        right += seg
        t_r += sum(w.duration for w in seg)
        if k == STROKES - 1:
            break
        park = (0.22, -0.24, TOP + 0.20)  # well out of the left arm's way while it turns the cucumber
        right += [_rw(0.8, np.add(_tcp_for_hinge(end, head, LEAN), 0.035 * UP), grip, "up", head, LEAN),
                  _rw(1.5, park, grip, "aside", head, LEAN)]
        t_r += 2.3
        # left: turn the cucumber once the blade is up, pushing its top toward -n at s = -45 mm
        top = c + a * (-0.060) + UP * (R_MAX + 0.013 - 0.006)
        wait = t_r - t_l
        lseg = [
            tilt.Waypoint(max(wait, 0.1), (0.22, 0.26, TOP + 0.20), FINGER_CLOSED, "wait aside"),
            tilt.Waypoint(1.5, tuple(top + 0.04 * UP + n * 0.01), FINGER_CLOSED, "over the top", yaw=head),
            tilt.Waypoint(0.8, tuple(top + n * 0.01), FINGER_CLOSED, "fingertips on the top", yaw=head),
            tilt.Waypoint(1.4, tuple(top + n * 0.01 - n * PUSH), FINGER_CLOSED, f"turn {k + 1}", yaw=head),
            tilt.Waypoint(0.6, tuple(top - n * PUSH + 0.03 * UP), FINGER_CLOSED, "up", yaw=head),
            tilt.Waypoint(0.8, (0.22, 0.26, TOP + 0.20), FINGER_CLOSED, "aside"),
        ]
        left += lseg
        t_l += sum(w.duration for w in lseg)
        hold = t_l - t_r - 0.6  # the right arm waits aside until the left hand is nearly clear
        if hold > 0:
            right.append(_rw(hold, park, grip, "wait", head, LEAN))
            t_r += hold
    drop = pick + np.array([0.0, 0.0, 0.006])
    right += [
        _rw(1.2, np.add(_tcp_for_hinge(hinge(S_END), head, LEAN), 0.10 * UP), grip, "up", head, LEAN),
        _rw(2.5, drop + 0.08 * UP, grip, "carry back, level"),
        _rw(1.2, drop, grip, "set down"),
        _rw(0.8, drop, FINGER_OPEN, "let go"),
        _rw(1.0, drop + 0.10 * UP, FINGER_OPEN, "up"),
        _rw(1.5, (0.20, -0.26, TOP + 0.20), FINGER_OPEN, "clear"),
    ]
    t_r += sum(w.duration for w in right[-6:])
    left.append(tilt.Waypoint(max(t_r - t_l, 0.5), (0.20, 0.26, TOP + 0.20), FINGER_OPEN, "open, clear"))
    return {"left": left, "right": right}
