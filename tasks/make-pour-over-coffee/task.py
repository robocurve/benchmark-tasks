"""On the station table: a clear glass mug on a drip tray with a Hario V60 01 dripper on it (paper filter, 15 g of
ground coffee), and beside it a Hario V60 Drip Kettle Air holding 350 mL of water.

The water follows brew.py: tilt the kettle and water runs from its spout tip once the level inside is above the tip,
faster the higher it is; it lands where the spout points. In the cone the grounds first soak up 30 mL, then the
water stands above them and drains through at about 2.7 mL/s when the cone is full, far slower than the kettle pours.
Pour at the kettle's full flow without stopping and the cone overflows before a mugful has gone in.

The job: a mugful of coffee (200 to 240 mL, the line is at 220 mL) with the water never over the cone's rim and none
outside the dripper, the kettle put back down on the table.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import brew  # noqa: E402
import make_assets as A  # noqa: E402
from dynkit import Mover, UsdBody, arms, drive, now, on_reset, on_step  # noqa: E402

INSTRUCTION = "Make a mug of coffee with the pour-over."

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0

# World frame: x forward from the arms, y to the robot's left, z up (m).
RIGHT_BASE = np.array([0.0, -0.305])
TRAY = dict(c=(0.40, 0.01), size=(0.24, 0.16, 0.005))
MUG_XY = np.array([0.38, 0.0])
MUG_BASE = np.array([*MUG_XY, TOP + TRAY["size"][2]])
DRIPPER_BASE = MUG_BASE + [0.0, 0.0, A.MUG_H + 0.0005]
KETTLE_XY = np.array([0.34, -0.12])
_U = KETTLE_XY - RIGHT_BASE
KETTLE_YAW = math.atan2(_U[1], _U[0])               # spout pointing away from the right arm, handle towards it


def yaw_quat(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


KETTLE_EMPTY = 0.10                                 # kg without water
KETTLE_MASS = KETTLE_EMPTY + 0.25                   # kg: with its water, about midway through the pour

# -- scoring constants ------------------------------------------------------------------------------------------------
BREW_LO, BREW_HI = 200e-6, 240e-6                   # m3 in the mug once it has finished dripping: full credit
HALF_LO, QUARTER_LO = 150e-6, 40e-6
SPILL_TOL = 1e-6                                    # m3 landed outside the cone
SEAT_XY, SEAT_DEG = 0.012, 8.0                      # the dripper sits on the mug
MUG_TIP_DEG = 12.0
KETTLE_DOWN_DEG = 10.0
LET_GO = 0.07                                       # m from the kettle's handle to each tool point: released
FULL_HOLD_S = 2.0

KETTLE = UsdBody("kettle", pos=(*KETTLE_XY, TOP + 0.0005), size=(0.177, 0.078, 0.128), usd="assets/kettle.usda",
                 rot_xyzw=yaw_quat(KETTLE_YAW), mass=KETTLE_MASS, friction=(1.0, 0.9),
                 solver_iters=24, torsional_patch=0.015)
MUG = UsdBody("mug", pos=tuple(map(float, MUG_BASE + [0, 0, 0.0005])), size=(0.08, 0.08, 0.085),
              usd="assets/mug.usda", mass=0.30, friction=(0.9, 0.8))
DRIPPER = UsdBody("dripper", pos=tuple(map(float, DRIPPER_BASE)), size=(0.115, 0.10, 0.082),
                  usd="assets/dripper.usda", mass=0.08, friction=(0.9, 0.8))
FIXED = [Box("drip_tray", pos=(*TRAY["c"], TOP + TRAY["size"][2] / 2), size=TRAY["size"], color=(0.30, 0.32, 0.36))]

# -- the water, drawn as shapes ---------------------------------------------------------------------------------------
_STATE: dict = {}
HIDE = (0.0, 0.0, -1.0)
ID_Q = (0.0, 0.0, 0.0, 1.0)
WATER_C = (0.80, 0.88, 0.95)
BREW_C = (0.36, 0.22, 0.12)       # coffee standing in the cone
CUP_C = (0.22, 0.12, 0.06)        # coffee in the mug


def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_from_matrix(R):
    from yam_arena.ik import matrix_to_quat_xyzw

    return tuple(float(v) for v in matrix_to_quat_xyzw(R))


def _frame(name, default):
    p, q = _STATE.get(f"{name}_pos"), _STATE.get(f"{name}_quat")
    if p is None:
        return np.asarray(default, float), ID_Q
    return np.asarray(p, float), tuple(float(v) for v in q)


def tilt_deg(R) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))


# coffee standing in the cone: slices from the top of the bed to the rim
N_CONE = 10
_CONE_Z = np.linspace(A.CONE_Z0 + brew.H_BED, A.CONE_Z0 + brew.CONE_D, N_CONE + 1)


def _cone_r(z):
    return brew.R_HOLE + (brew.R_RIM - brew.R_HOLE) * (z - A.CONE_Z0) / brew.CONE_D


def _cone_slice(k):
    z0, z1 = _CONE_Z[k], _CONE_Z[k + 1]

    def pose(t):
        top = A.CONE_Z0 + brew.H_BED + _STATE.get("depth", 0.0)
        if _STATE.get("W", 0.0) < 0.3e-6 or top <= z0 + 0.0003:
            return HIDE, ID_Q
        p, q = _frame("dripper", DRIPPER_BASE)
        zc = (z0 + min(z1, top)) / 2
        return tuple(p + quat_matrix(q) @ np.array([0.0, 0.0, zc])), q
    return pose


CONE_W = []
for _k in range(N_CONE):
    _m = Mover(f"cone_water{_k}", pos=HIDE, radius=max(0.004, _cone_r((_CONE_Z[_k] + _CONE_Z[_k + 1]) / 2) - 0.0008),
               height=float(_CONE_Z[1] - _CONE_Z[0]), kind="cylinder", collide=False, color=BREW_C)
    drive(_m.name, _cone_slice(_k))
    CONE_W.append(_m)

# coffee in the mug
N_MUG = 14
_MUG_Z = np.linspace(A.MUG_BASE, A.MUG_H - 0.004, N_MUG + 1)


def _mug_slice(k):
    z0, z1 = _MUG_Z[k], _MUG_Z[k + 1]

    def pose(t):
        top = A.MUG_BASE + _STATE.get("mug", 0.0) / (math.pi * brew.MUG_R ** 2)
        if _STATE.get("mug", 0.0) < 0.5e-6 or top <= z0 + 0.0003:
            return HIDE, ID_Q
        p, q = _frame("mug", MUG_BASE)
        zc = (z0 + min(z1, top)) / 2
        return tuple(p + quat_matrix(q) @ np.array([0.0, 0.0, zc])), q
    return pose


MUG_W = []
for _k in range(N_MUG):
    _m = Mover(f"mug_coffee{_k}", pos=HIDE, radius=brew.MUG_R - 0.001, height=float(_MUG_Z[1] - _MUG_Z[0]),
               kind="cylinder", collide=False, color=CUP_C)
    drive(_m.name, _mug_slice(_k))
    MUG_W.append(_m)

# the stream from the kettle: short capsules along its path; the drip from the cone's outlet
N_STREAM, SEG = 10, 0.014


def _seg_pose(path_key, k):
    def pose(t):
        pts = _STATE.get(path_key)
        if pts is None or k >= len(pts) - 1:
            return HIDE, ID_Q
        a, b = np.asarray(pts[k]), np.asarray(pts[k + 1])
        d = b - a
        n = float(np.linalg.norm(d))
        if n < 1e-6:
            return HIDE, ID_Q
        z = d / n
        x = np.cross([0.0, 1.0, 0.0], z) if abs(z[1]) < 0.9 else np.cross([1.0, 0.0, 0.0], z)
        x /= np.linalg.norm(x)
        return tuple((a + b) / 2), quat_from_matrix(np.stack([x, np.cross(z, x), z], 1))
    return pose


STREAM = []
for _k in range(N_STREAM):
    _m = Mover(f"stream{_k}", pos=HIDE, radius=0.0022, height=SEG, kind="capsule", collide=False, color=WATER_C)
    drive(_m.name, _seg_pose("stream_pts", _k))
    STREAM.append(_m)
DRIP = []
for _k in range(3):
    _m = Mover(f"drip{_k}", pos=HIDE, radius=0.0015, height=0.010, kind="capsule", collide=False, color=CUP_C)
    drive(_m.name, _seg_pose("drip_pts", _k))
    DRIP.append(_m)


def _overflow_pose(t):
    if not _STATE.get("overflowed"):
        return HIDE, ID_Q
    p, q = _frame("dripper", DRIPPER_BASE)
    return tuple(p + quat_matrix(q) @ np.array([0.0, 0.0, A.CONE_Z0 + brew.CONE_D - 0.004])), q


def _puddle_pose(t):
    at = _STATE.get("spill_at")
    if at is None:
        return HIDE, ID_Q
    return (at[0], at[1], TOP + 0.001 + (TRAY["size"][2] if _on_tray(at) else 0.0)), ID_Q


def _on_tray(p):
    return abs(p[0] - TRAY["c"][0]) <= TRAY["size"][0] / 2 and abs(p[1] - TRAY["c"][1]) <= TRAY["size"][1] / 2


OVERFLOW = Mover("overflow", pos=HIDE, radius=brew.R_RIM + 0.004, height=0.008, kind="cylinder", collide=False,
                 color=BREW_C)
PUDDLE = Mover("puddle", pos=HIDE, radius=0.03, height=0.002, kind="cylinder", collide=False, color=(0.55, 0.66, 0.78))
drive("overflow", _overflow_pose)
drive("puddle", _puddle_pose)

OBJECTS = [KETTLE, MUG, DRIPPER, *FIXED, *CONE_W, *MUG_W, *STREAM, *DRIP, OVERFLOW, PUDDLE]


# -- the stream's path ------------------------------------------------------------------------------------------------
def stream_path(tip, d, q, cone_c, cone_R, floor_z):
    """Points along the falling stream from the spout tip (direction d, flow q m3/s) and where it ends: 'cone' when
    it crosses the rim's plane inside the rim, else 'out' (landing point). Stops at the coffee's surface or the
    floor."""
    v = max(0.12, q / (math.pi * 0.0035 ** 2)) * np.asarray(d, float)
    p = np.asarray(tip, float).copy()
    pts, dt, travelled, where = [p.copy()], 0.004, 0.0, None
    rim_z = cone_c[2]
    for _ in range(400):
        p_new = p + v * dt
        v = v + np.array([0.0, 0.0, -brew.G * dt])
        if p[2] >= rim_z > p_new[2] and where is None:
            r = float(np.hypot(*(p_new[:2] - cone_c[:2])))
            where = "cone" if r <= cone_R else "out"
            if where == "cone":
                travelled += float(np.linalg.norm(p_new - p))
                pts.append(p_new.copy())
                break
        travelled += float(np.linalg.norm(p_new - p))
        p = p_new
        if travelled >= SEG * (len(pts)):
            pts.append(p.copy())
        if p[2] <= floor_z:
            where = where or "out"
            break
    pts.append(p.copy())
    return pts[: N_STREAM + 1], (where or "out"), p


# -- the monitor ------------------------------------------------------------------------------------------------------
@on_reset
def _reset(environment=None) -> None:
    _STATE.update(t=0.0, brew=brew.Brew(), written=False, violation=None, W=0.0, depth=0.0, mug=0.0, q=0.0,
                  kettle_V=brew.V_K0, stream_pts=None, drip_pts=None, overflowed=False, spill_at=None, seated=True,
                  where=None, released=True, kettle_down=True, done_since=None, brewed=0.0, poured=0.0,
                  max_W=0.0, max_tilt=0.0, kettle_pos=None, kettle_quat=None, mug_pos=None,
                  mug_quat=None, dripper_pos=None, dripper_quat=None)
    if environment is not None:
        _write_layout(environment)


_reset()


def _write_layout(environment) -> None:
    import torch

    base = environment.unwrapped
    origin = base.scene.env_origins
    dev = origin.device
    for spec in (KETTLE, MUG, DRIPPER):
        asset = base.scene[spec.name]
        pose = torch.tensor([[*spec.pos, *spec.rot_xyzw]], dtype=torch.float32, device=dev)
        pose[:, :3] += origin
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev))
        _ = asset.data.root_link_pose_w[0].detach().cpu().numpy()      # read back: fresh buffers for the next step
    _STATE["written"] = True


def _flag(what: str) -> None:
    if _STATE["violation"] is None:
        _STATE["violation"] = (round(_STATE["t"], 2), what)


def _pose(environment, name):
    base = environment.unwrapped
    org = base.scene.env_origins[0].detach().cpu().numpy()
    pose = base.scene[name].data.root_link_pose_w[0].detach().cpu().numpy().astype(float)
    return pose[:3] - org, pose[3:7]


def handle_point(kp, R):
    return kp + R @ np.array([A.HANDLE_X, 0.0, (A.HANDLE_Z0 + A.HANDLE_Z1) / 2])


@on_step
def _monitor(environment) -> None:
    S = _STATE
    S["t"] = now()
    if not S["written"]:
        _write_layout(environment)
    dt = float(environment.unwrapped.step_dt)
    kp, kq = _pose(environment, "kettle")
    mp, mq = _pose(environment, "mug")
    dp, dq = _pose(environment, "dripper")
    S.update(kettle_pos=kp, kettle_quat=kq, mug_pos=mp, mug_quat=mq, dripper_pos=dp, dripper_quat=dq)
    Rk, Rm, Rd = quat_matrix(kq), quat_matrix(mq), quat_matrix(dq)
    S["max_tilt"] = max(S["max_tilt"], tilt_deg(Rk))

    # the dripper sits on the mug
    top = mp + Rm @ np.array([0.0, 0.0, A.MUG_H])
    S["seated"] = (float(np.hypot(*(dp[:2] - top[:2]))) < SEAT_XY and abs(dp[2] - top[2]) < 0.01
                   and tilt_deg(Rd) < SEAT_DEG)
    if tilt_deg(Rm) > MUG_TIP_DEG:
        _flag(f"mug tipped ({tilt_deg(Rm):.0f} deg)")

    b = S["brew"]
    tip = kp + Rk @ np.array(brew.TIP)
    d = Rk @ (np.array(A.SPOUT[-1]) - np.array(A.SPOUT[-2]))
    d /= np.linalg.norm(d)
    rim_c = dp + Rd @ np.array([0.0, 0.0, A.CONE_Z0 + brew.CONE_D])
    floor = TOP + (TRAY["size"][2] if _on_tray(tip) else 0.0)
    pts, where, land = stream_path(tip, d, max(b.q, 1e-7), rim_c, brew.R_RIM - 0.003, floor)
    into_cone = where == "cone" and S["seated"]
    hd = brew.head(b.V, Rk, kp)
    n = 6
    spilt0 = b.spilt
    for _ in range(n):
        b.step(hd, into_cone, dt / n)
    if b.spilt - spilt0 > 0 and S["spill_at"] is None and b.spilt > SPILL_TOL:
        S["spill_at"] = (float(land[0]), float(land[1]))
    if b.spilt > SPILL_TOL:
        _flag("water outside the dripper")
    if b.overflow:
        S["overflowed"] = True
        _flag("water over the cone's rim")
    if not S["seated"] and b.W > 0.5e-6:
        _flag("dripper off the mug with water in it")
    S["stream_pts"] = pts if b.q > 0.2e-6 else None
    out = dp + Rd @ np.array([0.0, 0.0, A.CONE_Z0 - 0.002])
    surf = mp[2] + A.MUG_BASE + b.mug / (math.pi * brew.MUG_R ** 2)
    S["drip_pts"] = ([out, out + [0, 0, -0.01], out + [0, 0, -0.02], out + [0, 0, -0.03]]
                     if b.W > 1e-6 and S["seated"] and out[2] - surf > 0.01 else None)
    if S["drip_pts"]:
        S["drip_pts"] = [p for p in S["drip_pts"] if p[2] > surf - 0.002] or None
    S.update(W=b.W, depth=b.depth, mug=b.mug, q=b.q, kettle_V=b.V, brewed=b.brewed, poured=b.poured, where=where)
    S["max_W"] = max(S["max_W"], b.W)

    # the kettle back on the table, upright and let go
    a = arms(environment)
    hp = handle_point(kp, Rk)
    S["released"] = all(float(np.linalg.norm(a[s]["tcp"] - hp)) > LET_GO for s in a)
    S["kettle_down"] = tilt_deg(Rk) < KETTLE_DOWN_DEG and kp[2] < TOP + 0.01
    done = (BREW_LO <= b.brewed <= BREW_HI and b.q < 0.05e-6 and S["kettle_down"] and S["released"])
    S["done_since"] = (S["done_since"] if S["done_since"] is not None else S["t"]) if done else None


def violation() -> bool:
    return _STATE["violation"] is not None


def score_from(S) -> float:
    if S["violation"] is not None:
        return 0.0
    v = S["brewed"]
    if BREW_LO <= v <= BREW_HI:
        if S["done_since"] is not None and S["t"] - S["done_since"] >= FULL_HOLD_S - 1e-6:
            return 1.0
        return 0.5
    if v >= HALF_LO:
        return 0.5
    if v >= QUARTER_LO:
        return 0.25
    return 0.0


def score(objects) -> float:
    return score_from(_STATE)


def success(objects) -> bool:
    """200 to 240 mL brewed, never over the rim or outside the dripper, the kettle down upright and let go; 2 s."""
    return score(objects) >= 1.0


def status() -> str:
    S = _STATE
    return (f"V {S['kettle_V'] * 1e6:.0f} q {S['q'] * 1e6:.1f} W {S['W'] * 1e6:.0f} mug {S['mug'] * 1e6:.0f} "
            f"brewed {S['brewed'] * 1e6:.0f} where {S['where']} seated {S['seated']} tilt {S['max_tilt']:.0f} "
            f"viol {S['violation']}")
