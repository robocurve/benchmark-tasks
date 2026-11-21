"""On the station table, on an absorbent pad: a 3 qt pot holding 2.4 L of soup (tinted water), an 8 oz ladle
leaning in it with its handle towards the right arm, and a soup bowl to the left of it.

The soup follows soup.py: the ladle fills when its bowl is dipped under the soup; the soup in it stays level as the
ladle turns and leans back as it accelerates (one slosh mode), and runs over the lowest point of the rim when it is
above it, falling where the lip is; the ladle's underside carries a film out of the pot that drips off its lowest
point over a few seconds. The weight of the soup in the ladle is applied to the ladle.

The job: two ladlefuls in the bowl (340 to 480 g), none on the pad, the bowl's rim or its outside (at most 3 g), and
the ladle put back in the pot.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

import make_assets as A  # noqa: E402
import soup  # noqa: E402
from dynkit import Mover, UsdBody, arms, drive, now, on_reset, on_step  # noqa: E402

INSTRUCTION = "Serve a bowl of soup: two ladlefuls."

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# World frame: x forward from the arms, y to the robot's left, z up (m).
PAD = dict(c=(0.38, -0.05), size=(0.70, 0.50, 0.004))
PAD_TOP = TOP + PAD["size"][2]
POT_XY = np.array([0.40, -0.15])
BOWL_XY = np.array([0.34, 0.07])
POT_C = np.array([*POT_XY, PAD_TOP + A.POT_BASE_T])          # centre of the pot's inside bottom
BOWL_C = np.array([*BOWL_XY, PAD_TOP + A.BOWL_FOOT])         # centre of the soup bowl's inside bottom
LADLE_YAW = math.radians(-150.0)                             # the handle's direction (from the bowl of the ladle)


def yaw_quat(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat_from_matrix(R):
    from yam_arena.ik import matrix_to_quat_xyzw

    return tuple(float(v) for v in matrix_to_quat_xyzw(R))


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def tilt_deg(R) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))


def _rest_pose():
    """The ladle at rest in the pot: its bowl's underside on the pot's floor, its handle over the rim on the yaw side.
    Leaning it back by phi lowers the handle onto the rim."""
    best = None
    h, fn = A.handle_frame()
    for phi in np.radians(np.arange(9.5, 10.51, 0.25)):          # leaning about 10 deg on the rim
        R = rot_z(LADLE_YAW) @ rot_y(phi)
        low = min(float((R @ p)[2]) for p in soup._OUT)
        oz = POT_C[2] - low + 0.0005
        u = R @ h
        for d in np.arange(-0.08, 0.08, 0.001):
            o = np.array([*POT_XY, 0.0]) + d * np.array([math.cos(LADLE_YAW), math.sin(LADLE_YAW), 0.0])
            o[2] = oz
            start = o + R @ np.array([soup.A_RIM, 0.0, 0.0])
            # where the handle's underside crosses the rim's radius
            for s in np.arange(0.0, A.HANDLE_L, 0.002):
                pt = start + s * u - (R @ fn) * (A.HANDLE_T if s < A.GRIP_S0 else A.GRIP_T) / 2
                if np.hypot(*(pt[:2] - POT_XY)) >= soup.R_POT + A.POT_WALL / 2:
                    err = abs(pt[2] - (POT_C[2] + soup.H_POT))
                    # the bowl of the ladle must be inside the pot
                    inside = all(np.hypot(*((o + R @ p)[:2] - POT_XY)) < soup.R_POT - 0.002 for p in soup._RIM)
                    if inside and (best is None or err < best[0]):
                        best = (err, o, R)
                    break
    return best[1], best[2]


LADLE_O, LADLE_R0 = _rest_pose()

POT = UsdBody("pot", pos=tuple(map(float, POT_C)), size=(0.22, 0.22, 0.14), usd="assets/pot.usda", mass=3.6,
              friction=(1.0, 0.9), solver_iters=24)
BOWL = UsdBody("bowl", pos=tuple(map(float, BOWL_C)), size=(0.17, 0.17, 0.07), usd="assets/bowl.usda", mass=0.45,
               friction=(1.0, 0.9))
LADLE = UsdBody("ladle", pos=tuple(map(float, LADLE_O)), size=(0.10, 0.10, 0.30), usd="assets/ladle.usda",
                rot_xyzw=quat_from_matrix(LADLE_R0), mass=0.18, friction=(1.0, 0.9), solver_iters=32,
                torsional_patch=0.012)
FIXED = [Box("pad", pos=(*PAD["c"], TOP + PAD["size"][2] / 2), size=PAD["size"], color=(0.42, 0.43, 0.45))]

# -- the soup, drawn as shapes ----------------------------------------------------------------------------------------
_STATE: dict = {}
HIDE = (0.0, 0.0, -1.0)
ID_Q = (0.0, 0.0, 0.0, 1.0)
SOUP_C = (0.86, 0.45, 0.12)


def _frame(name, default):
    p, q = _STATE.get(f"{name}_pos"), _STATE.get(f"{name}_quat")
    if p is None:
        return np.asarray(default, float), ID_Q
    return np.asarray(p, float), tuple(float(v) for v in q)


def _pot_soup(t):
    d = _STATE.get("pot_V", soup.V_POT0) / (math.pi * soup.R_POT ** 2)
    p, q = _frame("pot", POT_C)
    return tuple(p + quat_matrix(q) @ np.array([0.0, 0.0, d / 2])), q


POT_SOUP = Mover("pot_soup", pos=HIDE, radius=soup.R_POT - 0.0015, height=soup.V_POT0 / (math.pi * soup.R_POT ** 2),
                 kind="cylinder", collide=False, color=SOUP_C)
drive("pot_soup", _pot_soup)


# the soup's surface in the ladle: a level plane cuts the bowl's sphere in a circle, drawn as the largest of a set of
# thin discs that fits inside it
DISC_R = tuple(float(r) for r in np.arange(0.008, soup.A_RIM, 0.0015))
DISC_H = 0.002


def _ladle_soup(k):
    def pose(t):
        s = _STATE.get("ladle_disc")
        if not s or s[2] != k:
            return HIDE, ID_Q
        return s[0], s[1]
    return pose


LADLE_SOUP = []
for _k, _r in enumerate(DISC_R):
    _m = Mover(f"ladle_soup{_k}", pos=HIDE, radius=_r, height=DISC_H, kind="cylinder", collide=False, color=SOUP_C)
    drive(_m.name, _ladle_soup(_k))
    LADLE_SOUP.append(_m)

# soup in the bowl: slices from its bottom up to the level
N_BOWL = 10
_BZ = np.linspace(0.0, soup.BOWL_H - 0.002, N_BOWL + 1)


def _bowl_slice(k):
    z0, z1 = _BZ[k], _BZ[k + 1]

    def pose(t):
        top = soup.bowl_level(_STATE.get("W", 0.0))
        if _STATE.get("W", 0.0) < 1e-6 or top <= z0 + 0.0003:
            return HIDE, ID_Q
        p, q = _frame("bowl", BOWL_C)
        zc = (z0 + min(z1, top)) / 2
        return tuple(p + quat_matrix(q) @ np.array([0.0, 0.0, zc])), q
    return pose


BOWL_W = []
for _k in range(N_BOWL):
    _m = Mover(f"bowl_soup{_k}", pos=HIDE, radius=max(0.01, soup.bowl_r(_BZ[_k]) - 0.0012),
               height=float(_BZ[1] - _BZ[0]), kind="cylinder", collide=False, color=SOUP_C)
    drive(_m.name, _bowl_slice(_k))
    BOWL_W.append(_m)

# the stream over the lip and the drips off the underside: short capsules
N_STREAM, SEG = 8, 0.016


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
    _m = Mover(f"stream{_k}", pos=HIDE, radius=0.004, height=SEG, kind="capsule", collide=False, color=SOUP_C)
    drive(_m.name, _seg_pose("stream_pts", _k))
    STREAM.append(_m)
DRIP = []
for _k in range(2):
    _m = Mover(f"drip{_k}", pos=HIDE, radius=0.0018, height=0.008, kind="capsule", collide=False, color=SOUP_C)
    drive(_m.name, _seg_pose("drip_pts", _k))
    DRIP.append(_m)


def _puddle_pose(t):
    at = _STATE.get("spill_at")
    if at is None:
        return HIDE, ID_Q
    return (at[0], at[1], PAD_TOP + 0.0008), ID_Q


PUDDLE = Mover("puddle", pos=HIDE, radius=0.025, height=0.0015, kind="cylinder", collide=False, color=(0.62, 0.40, 0.22))
drive("puddle", _puddle_pose)

OBJECTS = [POT, BOWL, LADLE, *FIXED, POT_SOUP, *LADLE_SOUP, *BOWL_W, *STREAM, *DRIP, PUDDLE]

# -- scoring constants ------------------------------------------------------------------------------------------------
SERVE_LO, SERVE_HI = 340e-6, 480e-6            # m3 in the bowl (1 g per mL): two ladlefuls
ONE_LO = 150e-6                                # one ladleful
SOME = 40e-6
SPILL_OK, SPILL_HALF = 3e-6, 15e-6             # m3 on the pad, the bowl's rim or its outside
MOVED, TIPPED = 0.03, 10.0                     # m / deg: the pot or the bowl moved
DROPPED_Z = 0.07                               # m: the ladle's rim centre this low outside the pot is down
LET_GO = 0.06                                  # m from the ladle's handle to each tool point: released
FULL_HOLD_S = 2.0


# -- the monitor ------------------------------------------------------------------------------------------------------
@on_reset
def _reset(environment=None) -> None:
    _STATE.update(t=0.0, soup=soup.Soup(), written=False, violation=None, W=0.0, L=0.0, pot_V=soup.V_POT0, q=0.0,
                  spilt=0.0, rim=0.0, pours=0, stream_pts=None, drip_pts=None, spill_at=None, ladle_disc=None,
                  released=True, ladle_home=True, done_since=None, prev_c=None, prev_v=None, acc=np.zeros(3),
                  ladle_pos=None, ladle_quat=None, pot_pos=None, pot_quat=None, bowl_pos=None, bowl_quat=None,
                  max_tilt=0.0)
    if environment is not None:
        _write_layout(environment)


_reset()


def _write_layout(environment) -> None:
    import torch

    base = environment.unwrapped
    origin = base.scene.env_origins
    dev = origin.device
    for spec in (POT, BOWL, LADLE):
        asset = base.scene[spec.name]
        pose = torch.tensor([[*spec.pos, *spec.rot_xyzw]], dtype=torch.float32, device=dev)
        pose[:, :3] += origin
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev))
        _ = asset.data.root_link_pose_w[0].detach().cpu().numpy()      # read back: fresh buffers for the next step
    _set_weight(environment, 0.0, np.asarray(LADLE.pos, float))
    _STATE["written"] = True


def _set_weight(environment, mass, at) -> None:
    """The soup in the ladle pulls the ladle down at the soup's centre (world frame)."""
    import torch

    base = environment.unwrapped
    asset = base.scene["ladle"]
    org = base.scene.env_origins[0]
    dev = org.device
    f = torch.tensor([[[0.0, 0.0, -mass * soup.G]]], dtype=torch.float32, device=dev)
    p = torch.tensor([[list(map(float, at))]], dtype=torch.float32, device=dev) + org
    asset.permanent_wrench_composer.set_forces_and_torques_index(
        forces=f, positions=p, body_ids=[0], env_ids=torch.tensor([0], device=dev), is_global=True)


def _flag(what: str) -> None:
    if _STATE["violation"] is None:
        _STATE["violation"] = (round(_STATE["t"], 2), what)


def _pose(environment, name):
    base = environment.unwrapped
    org = base.scene.env_origins[0].detach().cpu().numpy()
    pose = base.scene[name].data.root_link_pose_w[0].detach().cpu().numpy().astype(float)
    return pose[:3] - org, pose[3:7]


def handle_points(p, R):
    return [p + R @ A.handle_point(s) for s in np.linspace(0.08, A.HANDLE_L, 8)]


def ladle_disc(S_, R, p):
    """The drawn surface of the soup in the ladle: (pos, quat, disc index), or None. The level plane cuts the bowl's
    sphere in a circle (all of it inside the bowl while the level is below the rim's lowest point)."""
    if S_.L < 2e-6:
        return None
    n = S_.up()
    lvl = S_.level(R, p, S_.L)[0]
    c = p + R @ soup._c
    d = lvl - float(c @ n)                       # the plane's height over the sphere's centre
    r2 = soup.R_SPH ** 2 - d * d
    if r2 <= 0:
        return None
    r = math.sqrt(r2)
    # the disc's lower face is DISC_H under the surface, where the sphere is narrower
    lo = soup.R_SPH ** 2 - (d - DISC_H) ** 2 if d - DISC_H < 0 else r2
    fit = min(r, math.sqrt(max(lo, 0.0))) - 0.0008
    k = max((i for i, rr in enumerate(DISC_R) if rr <= fit), default=None)
    if k is None:
        return None
    x = np.cross([0.0, 1.0, 0.0], n) if abs(n[1]) < 0.9 else np.cross([1.0, 0.0, 0.0], n)
    x /= np.linalg.norm(x)
    Rm = np.stack([x, np.cross(n, x), n], 1)
    return tuple(c + n * (d - DISC_H / 2 + 0.0004)), quat_from_matrix(Rm), k


@on_step
def _monitor(environment) -> None:
    S = _STATE
    S["t"] = now()
    if not S["written"]:
        _write_layout(environment)
    dt = float(environment.unwrapped.step_dt)
    lp, lq = _pose(environment, "ladle")
    pp, pq = _pose(environment, "pot")
    bp, bq = _pose(environment, "bowl")
    S.update(ladle_pos=lp, ladle_quat=lq, pot_pos=pp, pot_quat=pq, bowl_pos=bp, bowl_quat=bq)
    Rl, Rp, Rb = quat_matrix(lq), quat_matrix(pq), quat_matrix(bq)

    for name, p0, p, R in (("pot", POT_C, pp, Rp), ("bowl", BOWL_C, bp, Rb)):
        if float(np.hypot(*(p[:2] - p0[:2]))) > MOVED or tilt_deg(R) > TIPPED:
            _flag(f"the {name} was moved")

    # dropped: the ladle down on the pad or the table outside the pot
    if float(np.hypot(*(lp[:2] - pp[:2]))) > soup.R_POT + 0.01 and lp[2] < PAD_TOP + DROPPED_Z:
        _flag("the ladle was dropped")

    # the acceleration of the ladle's bowl (over control steps, smoothed a little)
    c = lp + Rl @ np.array([0.0, 0.0, -0.02])
    if S["prev_c"] is not None:
        v = (c - S["prev_c"]) / dt
        if S["prev_v"] is not None:
            S["acc"] = 0.5 * S["acc"] + 0.5 * (v - S["prev_v"]) / dt
        S["prev_v"] = v
    S["prev_c"] = c

    sp = S["soup"]
    n = 6
    stream = drip = None
    spilt0 = sp.spilt
    for _ in range(n):
        st, dr = sp.step(Rl, lp, S["acc"], pp, bp, Rb, dt / n)
        stream = st if st is not None else stream
        drip = dr if dr is not None else drip
    if sp.spilt > spilt0 and S["spill_at"] is None and sp.spilt - sp.rim > 1e-6:
        at = drip if (drip is not None and stream is None) else stream
        if at is not None:
            S["spill_at"] = (float(at[0]), float(at[1]))
    _set_weight(environment, sp.L * 1000.0, lp + Rl @ np.array([0.0, 0.0, -0.02]))

    # drawing: the stream from the lip down to the bowl's (or pot's) rim height, the drips below the low point
    if stream is not None and sp.q > 1e-6:
        pts, x = [np.asarray(stream, float)], np.asarray(stream, float).copy()
        out = x - lp
        out[2] = 0.0
        nn = float(np.linalg.norm(out))
        hh = max(0.0, sp.level(Rl, lp, sp.L)[0] - sp.level(Rl, lp, sp.L)[1])
        v = 0.6 * math.sqrt(2 * soup.G * hh) * out / nn if nn > 1e-6 else np.zeros(3)
        floor = max(bp[2] + soup.bowl_level(sp.W), PAD_TOP)
        if float(np.hypot(*(x[:2] - pp[:2]))) < soup.R_POT:
            floor = pp[2] + sp.V / (math.pi * soup.R_POT ** 2)
        for _ in range(200):
            x = x + v * 0.004
            v = v + np.array([0.0, 0.0, -soup.G * 0.004])
            if float(np.linalg.norm(x - pts[-1])) >= SEG:
                pts.append(x.copy())
            if x[2] <= floor or len(pts) > N_STREAM:
                break
        pts.append(x.copy())
        S["stream_pts"] = pts[: N_STREAM + 1]
    else:
        S["stream_pts"] = None
    S["drip_pts"] = [drip, drip + [0, 0, -0.012], drip + [0, 0, -0.03]] if drip is not None else None
    S["ladle_disc"] = ladle_disc(sp, Rl, lp)
    S["cause"] = dict(sp.cause)
    S.update(W=sp.W, L=sp.L, pot_V=sp.V, q=sp.q, spilt=sp.spilt, rim=sp.rim, pours=sp.pours_now(),
             max_tilt=sp.max_tilt)

    # the ladle back in the pot and let go
    a = arms(environment)
    S["opening_r"] = float(a["right"]["opening"])
    S["tcp_r"] = np.asarray(a["right"]["tcp"], float)
    hps = handle_points(lp, Rl)
    S["released"] = all(min(float(np.linalg.norm(a[s]["tcp"] - h)) for h in hps) > LET_GO for s in a)
    S["ladle_home"] = (float(np.hypot(*(lp[:2] - pp[:2]))) < soup.R_POT and lp[2] < pp[2] + soup.H_POT)
    done = (SERVE_LO <= sp.W <= SERVE_HI and sp.spilt <= SPILL_OK and S["pours"] == 2 and S["ladle_home"]
            and S["released"] and sp.q < 1e-7)
    S["done_since"] = (S["done_since"] if S["done_since"] is not None else S["t"]) if done else None


def violation() -> bool:
    return _STATE["violation"] is not None


def score_from(S) -> float:
    if S["violation"] is not None:
        return 0.0
    w, sp = S["W"], S["spilt"]
    if S["done_since"] is not None and S["t"] - S["done_since"] >= FULL_HOLD_S - 1e-6:
        return 1.0
    if sp <= SPILL_HALF and w >= ONE_LO:
        return 0.5
    if w >= SOME:
        return 0.25
    return 0.0


def score(objects) -> float:
    return score_from(_STATE)


def success(objects) -> bool:
    """Two ladlefuls (340 to 480 g) in the bowl, at most 3 g outside it, the ladle back in the pot and let go; 2 s."""
    return score(objects) >= 1.0


def status() -> str:
    S = _STATE
    return (f"pot {S['pot_V'] * 1e6:.0f} ladle {S['L'] * 1e6:.0f} bowl {S['W'] * 1e6:.0f} q {S['q'] * 1e6:.1f} "
            f"spilt {S['spilt'] * 1e6:.1f} rim {S['rim'] * 1e6:.1f} pours {S['pours']} tilt {S['max_tilt']:.0f} "
            f"home {S['ladle_home']} viol {S['violation']}")
