"""On the station table a free grey patient-monitor lead (6 mm PVC, 60 cm) lies straight, left to right, 40 cm in
front of the robot: a grey plug (35 x 16 x 12 mm) on its left end in front of the left arm, a red patient clip
(25 x 15 x 15 mm) on its right end in front of the right arm. A 2 cm blue band is painted on the cord 14-16 cm from
the plug. A clean white drape (76 x 18 cm, 3 mm thick) lies on the table nearer the robot, parallel to the lead.
Both grippers' finger pads are covered with disinfectant wipe (real trial; bare pads in sim).

The whole lead is wiped outward from the blue band, every part once and never wiped back over: the cord toward the
clip and then the clip body off past its tip while the plug is held; the cord toward the plug and then the plug body
off past its tip while the clip is held; then the lead is laid on the drape and let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

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

INSTRUCTION = ("Wipe the whole grey monitor lead with the wipe pads, its cord and both end pieces, wiping outward from "
               "the blue band and never back over any part already wiped, then lay the lead on the white drape and let go.")

# -- scene -------------------------------------------------------------------------------------------------------
R = 0.003                      # lead radius (6 mm lead)
D = 2 * R
LEAD_LEN = 0.60
LEAD_X = 0.40
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.012, color=(0.30, 0.31, 0.33))
CLIP = Plug(size=(0.025, 0.015, 0.015), mass=0.010, color=(0.80, 0.10, 0.08))
PLUG_Y = 0.08                  # plug centre
CORD_Y0 = PLUG_Y - PLUG.size[0] / 2   # cord start (plug face), y 0.0625; the cord runs toward -y
LEAD = cable("lead", start=(LEAD_X, CORD_Y0, TOP + CLIP.size[2] / 2 + 0.0005),
             yaw=-math.pi / 2, length=LEAD_LEN, radius=R, start_plug=PLUG, end_plug=CLIP,
             cable_mass=0.033, color=(0.62, 0.63, 0.65), bend_stiffness=0.01, bend_damping=0.001)
SEG = LEAD.seg_len             # 0.02
S_BLUE = (0.14, 0.16)          # band extent, m from the plug end of the cord
BLUE_SEG = round(S_BLUE[0] / SEG)   # 7
BLUE = (0.10, 0.30, 0.95)

DRAPE_X = (0.12, 0.30)
DRAPE_Y = (-0.56, 0.20)
DRAPE_T = 0.003
DRAPE = Box("drape", pos=(sum(DRAPE_X) / 2, sum(DRAPE_Y) / 2, TOP + DRAPE_T / 2),
            size=(DRAPE_X[1] - DRAPE_X[0], DRAPE_Y[1] - DRAPE_Y[0], DRAPE_T), color=(0.96, 0.96, 0.96))

OBJECTS = [DRAPE, LEAD]
TWIN_EXTRA = [
    {"name": "blue_band", "kind": "decal", "size_mm": [20, 6], "colour": "blue", "pose": [0.40, -0.0875, 0.003]},
    {"name": "finger_wipes", "kind": "wipe pad on each finger pad", "size_mm": [40, 40, 1.5], "colour": "white"},
]


def _install_bands() -> None:
    """cablekit has no per-segment colour: paint the blue band segment (visual only)."""
    orig = cablekit._spawn_cable
    if getattr(orig, "_wipe_bands", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != LEAD.name:
            return prim
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.sim.utils import bind_visual_material
            from isaaclab.sim.utils.stage import get_current_stage
            from yam_arena.station import linear_rgb

            look = f"{prim_path}/Looks/band_blue"
            mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(BLUE), roughness=0.5)
            mat.func(look, mat)
            bind_visual_material(f"{prim_path}/seg_{BLUE_SEG}/geom", look, stage=get_current_stage())
        except Exception as e:  # pragma: no cover - never break the scene for a colour
            print(f"[wipe task] band colour not applied: {e}", flush=True)
        return prim

    spawn._wipe_bands = True
    cablekit._spawn_cable = spawn


_install_bands()

# -- scoring -----------------------------------------------------------------------------------------------------
CLOSED_GAP = D + 0.001         # finger gap that counts as closed on the cord (pads within 0.5 mm of it)
PAD_HALF_W = 0.010             # half pad width along the lead
PAD_UP, PAD_DOWN = 0.015, 0.013  # pad extent above / below the TCP along the approach axis
BACK_TOL = 0.03                # backward tolerance for a hand closed on the cord
MIN_TRAVEL = 0.02              # a closed cord slide counts once it has slid 2 cm along the cord
REWIPE_MAX = 4                 # marks already wiped that a later slide may pass over again (stroke overlap)
HOLD_DIST = 0.025              # end-piece centre to TCP for "holds"
RELEASE_DIST = 0.03
MARK_TOL = 0.005
END_GAP = 0.01                 # cord within 1 cm of an end-piece face: closed pads cannot run there (they hit the body)
MARKS = np.round(np.arange(0.015, LEAD_LEN - 0.01, 0.01), 3)   # 0.015 .. 0.585: 58 marks, one per cm of cord
SPLIT_LO, SPLIT_HI = S_BLUE[0] - 0.005, S_BLUE[1] + 0.005   # clip-side strokes credit marks >= 0.135, plug-side <= 0.165
C_MID, C_LOW = 0.90, 0.50
ENDS = {"plug": PLUG, "clip": CLIP}
OTHER_END = {"plug": "clip", "clip": "plug"}
BODY_GAP_LO, BODY_GAP_HI = 0.002, 0.007   # pads on a body: gap from its smaller cross size - 2 mm to its larger + 7 mm
BODY_LAT = 0.005               # body centre within 5 mm of the pads' mid-plane (it is between the pads)
BODY_BACK_TOL = 0.01           # a pass on a body may not go back toward the cord by more than 1 cm
BODY_REWIPE = 0.01             # a new pass sliding 1 cm along a body already wiped = wiping back over it
BODY_IN = 0.008                # a pass must close with the pad centre at most 8 mm onto the body (pads straddle the face)

_ENV: dict = {"env": None}
_ST: dict = {}


def _reset_episode() -> None:
    _ST.clear()
    _ST.update(wiped=np.zeros(len(MARKS), bool), back=False, back_info="",
               cl={"left": None, "right": None}, ec={"left": None, "right": None},
               ends={"plug": False, "clip": False})


_reset_episode()


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_wipe_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._wipe_hooked = True
        cls._task_step = _task_step
    if getattr(cls.reset, "_task_reset_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._task_reset_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._task_reset_hooks = []
        cls.reset = reset
    if _reset_episode not in cls.reset._task_reset_hooks:
        cls.reset._task_reset_hooks.append(_reset_episode)


_install_hooks()


def _t(x):
    x = getattr(x, "torch", x)
    return x.detach().cpu().numpy()


def hands() -> dict:
    """{arm: (tcp, approach axis, closing axis, width axis, finger gap)} in env coordinates; {} offline."""
    env = _ENV["env"]
    if env is None:
        return {}
    from yam_arena.analysis import pose7_to_matrix
    from yam_arena.constants import TCP_OFFSET

    base = env.unwrapped
    origin = _t(base.scene.env_origins)[0].astype(np.float64)
    out = {}
    for arm in ("left", "right"):
        art = base.scene[f"{arm}_robot"]
        names = list(art.body_names)
        poses = _t(art.data.body_link_pose_w)[0]
        g = pose7_to_matrix(poses[names.index("gripper")])
        tcp = g[:3, :3] @ np.array(TCP_OFFSET) + g[:3, 3] - origin
        a = g[:3, 2]
        tl, tr = poses[names.index("tip_left")][:3], poses[names.index("tip_right")][:3]
        c = (tl - tr).astype(np.float64)
        c -= a * (c @ a)
        c /= np.linalg.norm(c) + 1e-12
        w = np.cross(a, c)
        jn = list(art.joint_names)
        q = _t(art.data.joint_pos)[0]
        gap = -float(q[jn.index("joint7")] + q[jn.index("joint8")])
        out[arm] = (tcp, a, c, w, gap)
    return out


def _dense_s(c, step: float = 0.002):
    """Resampled cord polyline (tips + segment centres) with arclength from the plug end."""
    poly = c.polyline()
    s_v = np.r_[0.0, (np.arange(len(poly) - 2) + 0.5) * c.seg_len, LEAD_LEN]
    pts, ss = [poly[0]], [0.0]
    for i in range(len(poly) - 1):
        k = max(1, int(round((s_v[i + 1] - s_v[i]) / step)))
        for j in range(1, k + 1):
            f = j / k
            pts.append(poly[i] + (poly[i + 1] - poly[i]) * f)
            ss.append(s_v[i] + (s_v[i + 1] - s_v[i]) * f)
    return np.array(pts), np.array(ss)


def _pinch(h, pts, ss):
    """(s_lo, s_mid, s_hi) of the cord inside this gripper's closed pinch, or None."""
    tcp, a, cax, w, gap = h
    if gap > CLOSED_GAP:
        return None
    d = pts - tcp
    da, dc, dw = d @ a, d @ cax, d @ w
    m = (da >= -PAD_UP) & (da <= PAD_DOWN) & (np.abs(dc) <= gap / 2 + 0.001) & (np.abs(dw) <= PAD_HALF_W)
    if not m.any():
        return None
    return float(ss[m].min()), float(ss[m].mean()), float(ss[m].max())


def _end_pose(c, name):
    """(centre, outward unit axis) of an end piece; the axis points from the cord toward the free tip."""
    if name == "plug":
        ctr, tip = c.start_plug, c.polyline()[0]
    else:
        ctr, tip = c.end_plug, c.polyline()[-1]
    if ctr is None or not np.isfinite(ctr).all():
        return None, None
    ax = np.asarray(ctr, float) - tip
    return np.asarray(ctr, float), ax / (np.linalg.norm(ax) + 1e-12)


def _holds(h, c, name) -> bool:
    """Gripper holds this end piece: finger gap at most its larger cross dimension + 3 mm, centre within 2.5 cm."""
    ctr, _ = _end_pose(c, name)
    p = ENDS[name]
    return ctr is not None and h[4] <= max(p.size[1], p.size[2]) + 0.003 and \
        float(np.linalg.norm(ctr - h[0])) <= HOLD_DIST


def _violate(msg: str) -> None:
    if not _ST["back"]:
        _ST["back"] = True
        _ST["back_info"] = msg


def _track_cord(arm, other, h, ho, c, pts, ss) -> None:
    p = _pinch(h, pts, ss)
    if p is None:
        _ST["cl"][arm] = None
        return
    lo, mid, hi = p
    k = _ST["cl"][arm]
    if k is None:
        k = _ST["cl"][arm] = dict(s0=mid, smin=mid, smax=mid, dir=0, dmax=0.0, a=None, b=None,
                                  snap=_ST["wiped"].copy())
    if k["dir"] == 0 and abs(mid - k["s0"]) >= 0.01:
        k["dir"] = 1 if mid > k["s0"] else -1
    if k["dir"] > 0 and mid < k["smax"] - BACK_TOL:
        _violate(f"{arm} slid back toward the plug: s={mid:.3f} after {k['smax']:.3f}")
    if k["dir"] < 0 and mid > k["smin"] + BACK_TOL:
        _violate(f"{arm} slid back toward the clip: s={mid:.3f} after {k['smin']:.3f}")
    k["smin"], k["smax"] = min(k["smin"], mid), max(k["smax"], mid)
    if k["dir"]:
        behind = "plug" if k["dir"] > 0 else "clip"
        ctr, _ = _end_pose(c, behind)
        if ctr is not None:
            # a closed stroke back toward the anchored end pushes the cord instead of sliding on it
            dist = float(np.linalg.norm(h[0] - ctr))
            if dist < k["dmax"] - BACK_TOL:
                _violate(f"{arm} hand moved back toward the {behind}: {dist:.3f} after {k['dmax']:.3f}")
            k["dmax"] = max(k["dmax"], dist)
        anchored = ho is not None and _holds(ho, c, behind)
    else:
        anchored = ho is not None and (_holds(ho, c, "plug") or _holds(ho, c, "clip"))
    if anchored:
        k["a"] = lo if k["a"] is None else min(k["a"], lo)
        k["b"] = hi if k["b"] is None else max(k["b"], hi)
    if k["dir"] and k["a"] is not None and k["smax"] - k["smin"] >= MIN_TRAVEL:
        new = (MARKS >= k["a"] - MARK_TOL - 1e-9) & (MARKS <= k["b"] + MARK_TOL + 1e-9)
        new &= (MARKS >= SPLIT_LO - 1e-9) if k["dir"] > 0 else (MARKS <= SPLIT_HI + 1e-9)
        rew = int((new & k["snap"]).sum())
        if rew > REWIPE_MAX:
            _violate(f"{arm} wiped back over {rew} cm of already-wiped cord")
        _ST["wiped"] |= new


def _on_body(h, c, name):
    """u = TCP position along the body's outward axis (from its centre, + toward the free tip) if this gripper's pads
    are closed around the body (gap fits the body, body centre between the pads and within the pads' height,
    pads overlapping the body's length); else None."""
    tcp, a, cax, _, gap = h
    p = ENDS[name]
    if not (min(p.size[1], p.size[2]) - BODY_GAP_LO <= gap <= max(p.size[1], p.size[2]) + BODY_GAP_HI):
        return None
    ctr, ax = _end_pose(c, name)
    if ctr is None:
        return None
    d = ctr - tcp
    u = float(-d @ ax)
    c2 = cax - ax * (cax @ ax)
    c2 /= np.linalg.norm(c2) + 1e-12
    if not (-PAD_UP <= d @ a <= PAD_DOWN and abs(d @ c2) <= BODY_LAT and abs(u) <= p.size[0] / 2 + PAD_HALF_W):
        return None
    return u


def _track_ends(arm, ho, h, c) -> None:
    """One closed pass per body: anchored (other hand holds the other end piece) the whole time, closing with the pads
    straddling the cord face (pad centre u <= -L/2 + 8 mm) and sliding outward until the pad centre reaches the free
    tip (u >= +L/2); never back by more than 1 cm."""
    hit = None
    for name in ENDS:
        u = _on_body(h, c, name)
        if u is not None:
            hit = (name, u)
            break
    k = _ST["ec"][arm]
    if hit is None:
        _ST["ec"][arm] = None
        return
    name, u = hit
    half = ENDS[name].size[0] / 2
    if k is None or k["name"] != name:
        k = _ST["ec"][arm] = dict(name=name, u0=u, umin=u, umax=u, ok=True, done=_ST["ends"][name])
    if k["u0"] <= -half + BODY_IN and u < k["umax"] - BODY_BACK_TOL:
        _violate(f"{arm} went back along the {name} body toward the cord: u={u:.3f} after {k['umax']:.3f}")
    if k["done"] and max(k["umax"] - k["u0"], k["u0"] - k["umin"]) >= BODY_REWIPE:
        _violate(f"{arm} wiped the {name} body again")
    k["umin"], k["umax"] = min(k["umin"], u), max(k["umax"], u)
    if not (ho is not None and _holds(ho, c, OTHER_END[name])):
        k["ok"] = False
    if k["ok"] and k["u0"] <= -half + BODY_IN and k["umax"] >= half:
        _ST["ends"][name] = True


def _track(objects) -> None:
    """Latch wiped marks, wiped end pieces and the backward rules (idempotent per step)."""
    c = objects["lead"]
    hs = hands()
    if not hs or not np.isfinite(c.points).all():
        return
    pts, ss = _dense_s(c)
    for arm, other in (("left", "right"), ("right", "left")):
        _track_cord(arm, other, hs[arm], hs.get(other), c, pts, ss)
        _track_ends(arm, hs.get(other), hs[arm], c)


def coverage(objects) -> float:
    _track(objects)
    return float(_ST["wiped"].mean())


def ends_wiped() -> int:
    return int(_ST["ends"]["plug"]) + int(_ST["ends"]["clip"])


def _on_drape(p) -> bool:
    return bool(DRAPE_X[0] <= p[0] <= DRAPE_X[1] and DRAPE_Y[0] <= p[1] <= DRAPE_Y[1] and p[2] <= TOP + 0.03)


def plug_on_drape(objects) -> bool:
    p = objects["lead"].start_plug
    return p is not None and np.isfinite(p).all() and _on_drape(p)


def lead_on_drape(objects) -> bool:
    c = objects["lead"]
    pts, _ = _dense_s(c)
    if c.start_plug is None or c.end_plug is None:
        return False
    allp = np.vstack([pts, c.start_plug[None], c.end_plug[None]])
    return bool(np.isfinite(allp).all() and all(_on_drape(q) for q in allp))


def released(objects) -> bool:
    """Every gripper at least 3 cm from every point of the cord, plug and clip; False without gripper state."""
    c = objects["lead"]
    hs = hands()
    if len(hs) != 2:
        return False
    pts, _ = _dense_s(c)
    allp = np.vstack([pts] + [q[None] for q in (c.start_plug, c.end_plug) if q is not None])
    return all(float(np.linalg.norm(allp - h[0], axis=1).min()) >= RELEASE_DIST for h in hs.values())


def score(objects) -> float:
    """0 on any backward wipe (latched), if the plug is not on the drape, or a gripper is within 3 cm of the lead at
    the end. Else by the fraction c of the 58 1 cm cord marks (1.5 to 58.5 cm) wiped: 0.3 if c >= 0.5, 0.6 if c >= 0.9,
    1.0 if every mark is wiped, the plug body and the clip body are both wiped and the whole lead (cord, plug and clip)
    lies on the drape."""
    c = coverage(objects)
    if _ST["back"] or not plug_on_drape(objects) or not released(objects):
        return 0.0
    if _ST["wiped"].all() and ends_wiped() == 2 and lead_on_drape(objects):
        return 1.0
    if c >= C_MID:
        return 0.6
    if c >= C_LOW:
        return 0.3
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["lead"].resting(max_speed=0.02)


def distance(objects) -> float:
    """1 - (wiped marks + wiped bodies) / 60 (1.0 once a backward rule fired)."""
    c = coverage(objects)
    return 1.0 if _ST["back"] else 1.0 - (c * len(MARKS) + ends_wiped()) / (len(MARKS) + 2)


def progress(objects) -> dict:
    hs = hands()
    c = objects["lead"]
    held = any(_holds(h, c, "plug") for h in hs.values())
    return {"plug_held": int(held), "coverage": round(coverage(objects), 3),
            "clip_wiped": int(_ST["ends"]["clip"]), "plug_wiped": int(_ST["ends"]["plug"]), "no_backwipe": int(not _ST["back"]),
            "plug_on_drape": int(plug_on_drape(objects)), "lead_on_drape": int(lead_on_drape(objects)),
            "released": int(released(objects))}


VARIATION = {"cable:lead": {"dx": 0.015, "dy": 0.015, "yaw": 0.05}}

# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------
# Actions come only from the fixed known layout and the robot's own measured joint state (TCP from the joint
# angles in ctx.obs.state). ctx.states() is used only by log() for the run log.
WIPE_F = -0.0025               # finger joint target: pads 5 mm apart -> light squeeze on the 6 mm cord
LIFT = TOP + 0.025
WIPE_Z = TOP + 0.0115          # TCP height while wiping the cord
OVER = 0.010                   # TCP above an end piece's centre when gripping it
WIPE_Z2 = TOP + 0.0095         # stroke 2: the left arm sits ~2 mm higher than the right
S_START = S_BLUE[0] + 0.005    # stroke 1 closes on the near half of the blue band
S2_START = S_BLUE[1] - 0.005   # stroke 2 closes on the far half
PINCH_OFF = 0.018              # measured: pinch arclength = TCP distance to the held plug centre - 1.8 cm
S1_END = 0.576                 # stop the cord stroke here: pads reach ~0.585, 1.5 cm short of the clip face
                               # (closed pads must never overlap an end piece)
S2_END = 0.024
STEP = 0.015
END_TO_END = PLUG.size[0] / 2 + LEAD_LEN + CLIP.size[0] / 2   # plug centre to clip centre on a straight lead, 0.63
CLIP_F = float(os.environ.get("WIPE_CF", -0.0105))   # bare fingers 21 mm apart on the 15 mm clip (two 3 mm wipe pads)
PLUG_F = float(os.environ.get("WIPE_PF", -0.011))    # bare fingers 22 mm apart on the 16 mm plug
OFF_TIP = 0.022                # a body pass ends with the pad centre 2.2 cm past the tip (pads fully off it)
BODY_T = float(os.environ.get("WIPE_BT", 4.0))       # body pass duration, s
CLIP_DZ = float(os.environ.get("WIPE_BZC", 0.0085))  # body pass TCP above the cord-wiping height: clip, TCP about
PLUG_DZ = float(os.environ.get("WIPE_BZP", 0.004))   # 3 mm above the body's top (the space between the fingers ends there)


def meas_tcp(ctx, side: str) -> np.ndarray:
    """TCP from the measured joint angles (robot state), not the command."""
    from yam_arena import ik

    q = np.asarray(ctx.obs.state["joint_pos"], dtype=float).reshape(-1)
    arm = ctx.left if side == "left" else ctx.right
    qa = q[0:6] if side == "left" else q[7:13]
    return ik.tcp_pose(qa, arm.base)[0]


def _cl_dbg():
    return {k: (None if v is None else (round(v['s0'], 3), v['dir'], v['a'] and round(v['a'], 3),
                                        v['b'] and round(v['b'], 3))) for k, v in _ST['cl'].items()}


def _ec_dbg():
    return {k: (None if v is None else (v['name'], round(v['u0'], 3), round(v['umax'], 3), v['ok']))
            for k, v in _ST['ec'].items()}


def body_pass(ctx, arm, f, ctr, length, v, z, log, tag, inward=False, start=-0.012):
    """Wipe an end piece's body (centre estimate ctr, outward direction v). The hand is open and lifted. Set down wide
    open at `start` from the body's face (negative: on the cord short of it; positive: pads straddling the face, so
    closing centres the body between them), close the pads to just over the body's width (f), then slide outward
    along v in one pass over the body and off past its tip. inward (test only): start past the tip and pass back
    toward the cord."""
    yaw = -math.pi / 2
    face = np.array([ctr[0], ctr[1], z]) - v * length / 2
    p0 = face + v * start
    p1 = face + v * (length + OFF_TIP)
    if inward:
        p0, p1 = p1, p0
    yield from arm.move((p0[0], p0[1], p0[2] + 0.03), yaw=yaw, duration=0.6)
    yield from arm.move(tuple(p0), yaw=yaw, duration=0.5)
    yield from arm.grip(f, 0.5)
    log(f"{tag} set")
    n = 3
    dbg = os.environ.get("WIPE_DBG")
    for i in range(1, n + 1):
        if dbg:
            for _ in arm.move(tuple(p0 + (p1 - p0) * i / n), yaw=yaw, duration=BODY_T / n):
                yield
                st = ctx.states()
                c = st["lead"]
                h = hands().get("left" if arm is ctx.left else "right")
                print(f"[dbg t={ctx.t:5.2f}] tcp={np.round(h[0], 4).tolist()} gap={h[4] * 1000:.1f} "
                      f"plug={np.round(c.start_plug, 4).tolist()} clip={np.round(c.end_plug, 4).tolist()} "
                      f"ec={_ec_dbg()}", flush=True)
        else:
            yield from arm.move(tuple(p0 + (p1 - p0) * i / n), yaw=yaw, duration=BODY_T / n)
        log(f"{tag} pass {i}/{n}")
    return p1


def demo_script(ctx):
    """Stroke 1: left holds the plug, right closes lightly on the cord at the blue band and slides toward the clip
    (closed loop on the measured hand distance) until its pads are 1.5 cm short of the clip face. Clip body: right
    opens and lifts, left pulls the lead straight, right sets down astride the clip's cord face, closes to just over
    the clip's width and slides over the clip and off its tip; then it takes the clip by its centre. Both carry the
    lead 8 cm further to the left; the left puts the plug down. Stroke 2: left closes on the cord at the blue band and
    slides toward the plug. Plug body: left opens, right pulls the lead straight, left sets down short of the plug
    face, closes to just over the plug's width and slides over the plug and off its tip. Left re-takes the plug,
    both lay the lead on the drape and let go."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("WIPE_CHEAT", "")
    # test only: none|hover|onearm|hops|scrub|back|hold|midonly|short2|noclip|noplug|clipin
    yaw = -math.pi / 2       # fingers close along x (across the lead)
    hi = TOP + 0.07
    wf = FINGER_OPEN if cheat == "hover" else WIPE_F
    cf = FINGER_OPEN if cheat == "hover" else CLIP_F
    pf = FINGER_OPEN if cheat == "hover" else PLUG_F

    def log(tag):
        st = ctx.states()
        hs = hands()
        hh = {k: (np.round(v[0], 3).tolist(), round(v[4] * 1000, 1)) for k, v in hs.items()}
        pts, ss = _dense_s(st["lead"])
        pr = {k: (None if _pinch(v, pts, ss) is None else round(_pinch(v, pts, ss)[1], 3)) for k, v in hs.items()}
        print(f"[demo t={ctx.t:5.1f}] {tag}: score={score(st):.2f} {progress(st)} pinch={pr} "
              f"back={_ST['back_info']} hands={hh} plug={np.round(st['lead'].start_plug, 3).tolist()} "
              f"clip={np.round(st['lead'].end_plug, 3).tolist()} cl={_cl_dbg()} ec={_ec_dbg()}", flush=True)

    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return
    pl = np.array([LEAD_X, PLUG_Y, TOP + PLUG.size[2] / 2])        # known layout
    b = np.array([LEAD_X, CORD_Y0 - S_START, WIPE_Z])
    take_plug = cheat != "onearm"
    # 1) left takes the plug, right closes lightly around the cord at the blue band
    if take_plug:
        yield from ctx.both(L.move((pl[0], pl[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5),
                            Rt.move((b[0], b[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5))
        yield from ctx.both(L.move((pl[0], pl[1], pl[2] + OVER), yaw=yaw, duration=1.0),
                            Rt.move(tuple(b), yaw=yaw, duration=1.0))
        yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(wf))
        yield from ctx.both(L.move((pl[0], pl[1], LIFT), yaw=yaw, duration=1.0), Rt.wait(1.0))
    else:
        yield from Rt.move((b[0], b[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5)
        yield from Rt.move(tuple(b), yaw=yaw, duration=1.0)
        yield from Rt.grip(wf)
    log("pinched")

    def s_right():
        lt = meas_tcp(ctx, "left") if take_plug else pl
        return float(np.linalg.norm((meas_tcp(ctx, "right") - lt)[:2])) - PINCH_OFF

    # 2) stroke 1: toward the clip
    pos = b.copy()
    s1_end = 0.55 if cheat == "midonly" else S1_END
    if cheat == "hops":
        hop = float(os.environ.get("WIPE_HOP", "0.10"))
        while pos[1] > CORD_Y0 - s1_end:
            yield from Rt.grip(FINGER_OPEN, 0.4)
            yield from Rt.move((pos[0], pos[1], WIPE_Z + 0.03), yaw=yaw, duration=0.5)
            pos[1] -= hop
            yield from Rt.move((pos[0], pos[1], WIPE_Z + 0.03), yaw=yaw, duration=0.6)
            yield from Rt.move(tuple(pos), yaw=yaw, duration=0.5)
            yield from Rt.grip(WIPE_F, 0.4)
            yield from Rt.move(tuple(pos), yaw=yaw, duration=0.4)
            log("hop")
    else:
        for k in range(80):
            s = s_right()
            if s >= s1_end:
                break
            fine = s >= s1_end - 0.03
            pos[1] -= min(0.005 if fine else STEP, s1_end - s + 0.002)
            yield from Rt.move(tuple(pos), yaw=yaw, duration=0.3)
            if fine:
                yield from Rt.wait(0.25)
            if k % 6 == 0:
                log(f"wipe1 k={k} s={s:.3f}")
            if cheat == "scrub" and s > 0.35:   # closed stroke back 8 cm and forward again
                yield from Rt.move(tuple(pos + np.array([0.0, 0.08, 0.0])), yaw=yaw, duration=1.0)
                yield from Rt.move(tuple(pos), yaw=yaw, duration=1.0)
                log("scrubbed back")
                cheat = "scrubbed"
            if cheat == "back" and s > 0.35:    # open, go back 8 cm along the cord, close and wipe on
                yield from Rt.grip(FINGER_OPEN, 0.4)
                pos[1] += 0.08
                yield from Rt.move((pos[0], pos[1], WIPE_Z + 0.03), yaw=yaw, duration=0.8)
                yield from Rt.move(tuple(pos), yaw=yaw, duration=0.5)
                yield from Rt.grip(WIPE_F, 0.4)
                log("went back 8 cm")
                cheat = "went_back"
    log("wiped1")
    lay_y = 0.16
    if cheat == "midonly" or not take_plug:
        # old procedure: only the band-to-55 cm stretch, then lay the lead down
        yield from ctx.both(L.wait(0.1), Rt.grip(WIPE_F, 0.2))
        dist = float(np.linalg.norm((meas_tcp(ctx, "right") - meas_tcp(ctx, "left"))[:2]))
        yield from lay(ctx, yaw, lay_y, dist, take_plug)
        return
    # 3) open; the lead recoils, so the left hand pulls it straight (plug 4 cm further left) and the clip is located
    #    from the left hand (straight lead: clip centre 63 cm from the plug centre)
    z_clip = TOP + CLIP.size[2] / 2 + OVER
    rt = meas_tcp(ctx, "right")
    yield from Rt.grip(FINGER_OPEN, 0.4)
    yield from ctx.both(Rt.move((rt[0], rt[1] + 0.03, z_clip + 0.02), yaw=yaw, duration=0.6),
                        L.move((pl[0], pl[1] + 0.04, LIFT), yaw=yaw, duration=0.6))
    yield from L.wait(0.4)

    def clip_from_left():
        lt = meas_tcp(ctx, "left")
        return np.array([lt[0], lt[1] - END_TO_END, z_clip])

    # 3b) clip body, the plug still held: one pass from the cord over the clip and off past its tip
    if cheat != "noclip":
        yield from body_pass(ctx, Rt, cf, clip_from_left(), CLIP.size[0], np.array([0.0, -1.0, 0.0]),
                             WIPE_Z + CLIP_DZ, log, "clip", inward=(cheat == "clipin"), start=0.003)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        rt = meas_tcp(ctx, "right")
        yield from Rt.move((rt[0], rt[1], z_clip + 0.02), yaw=yaw, duration=0.4)
        yield from L.wait(0.3)
    log("clip wiped")
    # 3c) take the clip by its centre as the anchor for stroke 2
    clip_c = clip_from_left()
    yield from Rt.move((clip_c[0], clip_c[1], z_clip + 0.02), yaw=yaw, duration=0.6)
    yield from Rt.move(tuple(clip_c), yaw=yaw, duration=0.4)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    u_r = 0.0
    log("clip held")
    # 4) carry the lead 8 cm further toward +y, put the plug down, left lets go
    lt, rt = meas_tcp(ctx, "left"), meas_tcp(ctx, "right")
    shift = 0.08
    yield from ctx.both(L.move((LEAD_X, lt[1] + shift, LIFT), yaw=yaw, duration=1.5),
                        Rt.move((LEAD_X, rt[1] + shift, z_clip), yaw=yaw, duration=1.5))
    yield from L.move((LEAD_X, lt[1] + shift, pl[2] + OVER), yaw=yaw, duration=0.6)
    yield from L.grip(FINGER_OPEN, 0.5)
    yield from L.move((LEAD_X, lt[1] + shift, hi), yaw=yaw, duration=0.6)
    log("plug down")
    # 5) stroke 2: close at the blue band, slide toward the plug
    rt = meas_tcp(ctx, "right")
    s_r = LEAD_LEN + CLIP.size[0] / 2 + u_r       # arclength of the right TCP (on the clip)
    v = np.array([0.0, 1.0, 0.0])

    def s_left():
        return s_r - float(np.linalg.norm((meas_tcp(ctx, "left") - meas_tcp(ctx, "right"))[:2])) + PINCH_OFF - 0.0175

    yield from Rt.move((rt[0], rt[1], LIFT), yaw=yaw, duration=0.6)
    rt = meas_tcp(ctx, "right")
    p2 = rt + v * (s_r - S2_START)
    pos = np.array([LEAD_X, p2[1], WIPE_Z2])
    yield from L.move((pos[0], pos[1], hi), yaw=yaw, duration=1.0)
    yield from L.move(tuple(pos), yaw=yaw, duration=0.8)
    yield from L.grip(wf, 0.6)
    log("pinched2")
    s2_end = 0.065 if cheat == "short2" else S2_END    # short2: stop 6.5 cm from the plug (tier check)
    for k in range(40):
        s = s_left()
        if s <= s2_end:
            break
        fine = s <= s2_end + 0.03
        pos[1] += min(0.005 if fine else STEP, s - s2_end + 0.002)
        yield from L.move(tuple(pos), yaw=yaw, duration=0.3)
        if fine:
            yield from L.wait(0.25)
        if k % 4 == 0:
            log(f"wipe2 k={k} s={s:.3f}")
    log("wiped2")
    # 6) open; the lead recoils, so the right hand pulls it straight (clip 4 cm further right) and the plug is
    #    located from the right hand on the clip
    z_plug = TOP + PLUG.size[2] / 2 + OVER
    lt, rt = meas_tcp(ctx, "left"), meas_tcp(ctx, "right")
    yield from L.grip(FINGER_OPEN, 0.4)
    yield from ctx.both(L.move((lt[0], lt[1] - 0.02, z_plug + 0.02), yaw=yaw, duration=0.6),
                        Rt.move((LEAD_X, rt[1] - 0.04, rt[2]), yaw=yaw, duration=0.6))
    yield from L.wait(0.4)

    def plug_from_right():
        rt = meas_tcp(ctx, "right")
        return np.array([rt[0], rt[1] + END_TO_END, z_plug])

    # 6b) plug body, the clip still held: one pass from the cord over the plug and off past its tip
    if cheat != "noplug":
        yield from body_pass(ctx, L, pf, plug_from_right(), PLUG.size[0], v, WIPE_Z2 + PLUG_DZ, log, "plug")
        yield from L.grip(FINGER_OPEN, 0.4)
        lt = meas_tcp(ctx, "left")
        yield from L.move((lt[0], lt[1], z_plug + 0.02), yaw=yaw, duration=0.4)
        yield from L.wait(0.3)
    log("plug wiped")

    # 7) re-take the plug by its centre, lay the lead on the drape
    plug_c = plug_from_right()
    yield from L.move((plug_c[0], plug_c[1], z_plug + 0.02), yaw=yaw, duration=0.5)
    yield from L.move(tuple(plug_c), yaw=yaw, duration=0.4)
    yield from L.grip(FINGER_CLOSED, 0.5)
    dist = float(np.linalg.norm((meas_tcp(ctx, "right") - meas_tcp(ctx, "left"))[:2]))
    yield from lay(ctx, yaw, lay_y, dist, True, hold=(cheat == "hold"))


def lay(ctx, yaw, lay_y, dist, take_plug, hold=False):
    from yam_arena.constants import FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    hi = TOP + 0.07
    lay_l = np.array([0.19, lay_y])
    lay_r = np.array([0.19, lay_y - min(dist, 0.66)])
    down = TOP + DRAPE_T + R + OVER + 0.004
    if take_plug:
        yield from ctx.both(L.move((lay_l[0], lay_l[1], LIFT), yaw=yaw, duration=2.0),
                            Rt.move((lay_r[0], lay_r[1], LIFT), yaw=yaw, duration=2.0))
        yield from ctx.both(L.move((lay_l[0], lay_l[1], down + 0.004), yaw=yaw, duration=1.0),
                            Rt.move((lay_r[0], lay_r[1], down + 0.004), yaw=yaw, duration=1.0))
    else:
        yield from Rt.move((lay_r[0], lay_r[1], LIFT), yaw=yaw, duration=2.0)
        yield from Rt.move((lay_r[0], lay_r[1], down), yaw=yaw, duration=1.0)
    st = ctx.states()
    print(f"[demo t={ctx.t:5.1f}] laid: score={score(st):.2f} {progress(st)} back={_ST['back_info']}", flush=True)
    if hold:
        yield from L.wait(4.0)
        st = ctx.states()
        print(f"[demo t={ctx.t:5.1f}] end (held): score={score(st):.2f} {progress(st)}", flush=True)
        return
    yield from ctx.both(L.grip(FINGER_OPEN), Rt.grip(FINGER_OPEN))
    yield from ctx.both(L.move((lay_l[0], lay_l[1], hi + 0.02), yaw=yaw, duration=0.8),
                        Rt.move((lay_r[0], lay_r[1], hi + 0.02), yaw=yaw, duration=0.8))
    for _ in range(4):
        yield from L.wait(0.5)
    st = ctx.states()
    print(f"[demo t={ctx.t:5.1f}] end: score={score(st):.2f} {progress(st)} back={_ST['back_info']} "
          f"unwiped={MARKS[~_ST['wiped']].tolist()}", flush=True)
