"""On the station table a free grey patient-monitor lead (6 mm PVC, 60 cm) lies straight, left to right, 40 cm in
front of the robot: a grey plug (35 x 16 x 12 mm) on its left end in front of the left arm, a red patient clip
(25 x 15 x 15 mm) on its right end in front of the right arm. A 2 cm blue band is painted on the lead 14-16 cm from
the plug, a 2 cm red band 54-56 cm from the plug. A clean white drape (76 x 18 cm, 3 mm thick) lies on the table
nearer the robot, parallel to the lead.

The lead is disinfected in one pass: one hand holds the plug, the other closes around the lead at the blue band
and slides along it toward the clip, never sliding back toward the plug, as far as the red band; then the lead is
laid on the clean drape and let go.

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

INSTRUCTION = ("Disinfect the grey monitor lead: hold it by the plug, close the other gripper around the lead at "
               "the blue band and wipe along it in one pass toward the clip end, never moving back toward the "
               "plug. Wipe all the way to the red band, then lay the whole lead on the white drape and let go.")

# -- scene -------------------------------------------------------------------------------------------------------
R = 0.003                      # lead radius (6 mm lead)
D = 2 * R
LEAD_LEN = 0.60
LEAD_X = 0.40
PLUG = Plug(size=(0.035, 0.016, 0.012), mass=0.012, color=(0.30, 0.31, 0.33))
CLIP = Plug(size=(0.025, 0.015, 0.015), mass=0.010, color=(0.80, 0.10, 0.08))
PLUG_Y = 0.08                  # plug centre
LEAD = cable("lead", start=(LEAD_X, PLUG_Y - PLUG.size[0] / 2, TOP + CLIP.size[2] / 2 + 0.0005),
             yaw=-math.pi / 2, length=LEAD_LEN, radius=R, start_plug=PLUG, end_plug=CLIP,
             cable_mass=0.033, color=(0.62, 0.63, 0.65), bend_stiffness=0.01, bend_damping=0.001)
SEG = LEAD.seg_len             # 0.02
S_BLUE = (0.14, 0.16)          # band extents, m from the plug end of the lead
S_RED = (0.54, 0.56)
BLUE_SEG, RED_SEG = round(S_BLUE[0] / SEG), round(S_RED[0] / SEG)   # 7, 27
BLUE, RED = (0.10, 0.30, 0.95), (0.90, 0.08, 0.06)

DRAPE_X = (0.12, 0.30)
DRAPE_Y = (-0.56, 0.20)
DRAPE_T = 0.003
DRAPE = Box("drape", pos=(sum(DRAPE_X) / 2, sum(DRAPE_Y) / 2, TOP + DRAPE_T / 2),
            size=(DRAPE_X[1] - DRAPE_X[0], DRAPE_Y[1] - DRAPE_Y[0], DRAPE_T), color=(0.96, 0.96, 0.96))

OBJECTS = [DRAPE, LEAD]
TWIN_EXTRA = [
    {"name": "blue_band", "kind": "decal", "size_mm": [20, 6], "colour": "blue", "pose": [0.40, -0.0875, 0.003]},
    {"name": "red_band", "kind": "decal", "size_mm": [20, 6], "colour": "red", "pose": [0.40, -0.4875, 0.003]},
]


def _install_bands() -> None:
    """cablekit has no per-segment colour: paint the blue and red band segments (visual only)."""
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

            for tag, seg, col in (("blue", BLUE_SEG, BLUE), ("red", RED_SEG, RED)):
                look = f"{prim_path}/Looks/band_{tag}"
                mat = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(col), roughness=0.5)
                mat.func(look, mat)
                bind_visual_material(f"{prim_path}/seg_{seg}/geom", look, stage=get_current_stage())
        except Exception as e:  # pragma: no cover - never break the scene for a colour
            print(f"[wipe task] band colour not applied: {e}", flush=True)
        return prim

    spawn._wipe_bands = True
    cablekit._spawn_cable = spawn


_install_bands()

# -- scoring -----------------------------------------------------------------------------------------------------
CLOSED_GAP = D + 0.001         # finger gap that counts as closed on the lead (pads within 0.5 mm of it)
PAD_HALF_W = 0.010             # half pad width along the lead
PAD_UP, PAD_DOWN = 0.015, 0.013  # pad extent above / below the TCP along the approach axis
BACK_TOL = 0.03                # backward slide tolerance along the lead
PLUG_HOLD_DIST = 0.025         # plug centre to TCP
PLUG_HOLD_GAP = PLUG.size[1] + 0.003
RELEASE_DIST = 0.03
BINS = np.round(np.arange(S_BLUE[0] + 0.01, S_RED[0] + 0.01 + 1e-9, 0.01), 3)   # 0.15 .. 0.55, 41 points
C_FULL, C_MID, C_LOW = 0.95, 0.80, 0.50

_ENV: dict = {"env": None}
_ST: dict = {}


def _reset_episode() -> None:
    _ST.clear()
    _ST.update(wiped=np.zeros(len(BINS), bool), smax={"left": -1.0, "right": -1.0}, dmax={"left": 0.0, "right": 0.0},
               prev={"left": None, "right": None}, back=False, back_info="")


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
    """Resampled lead polyline (tips + segment centres) with arclength from the plug end."""
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
    """(s_lo, s_mid, s_hi) of the lead inside this gripper's closed pinch, or None."""
    tcp, a, cax, w, gap = h
    if gap > CLOSED_GAP:
        return None
    d = pts - tcp
    da, dc, dw = d @ a, d @ cax, d @ w
    m = (da >= -PAD_UP) & (da <= PAD_DOWN) & (np.abs(dc) <= gap / 2 + 0.001) & (np.abs(dw) <= PAD_HALF_W)
    if not m.any():
        return None
    return float(ss[m].min()), float(ss[m].mean()), float(ss[m].max())


def _plug_held(h, plug) -> bool:
    tcp, _, _, _, gap = h
    return plug is not None and gap <= PLUG_HOLD_GAP and float(np.linalg.norm(plug - tcp)) <= PLUG_HOLD_DIST


def _track(objects) -> None:
    """Latch the wiped bins and the backward-slide rule (idempotent: called by score/success/distance each step)."""
    c = objects["lead"]
    hs = hands()
    if not hs or not np.isfinite(c.points).all():
        return
    pts, ss = _dense_s(c)
    for arm, other in (("left", "right"), ("right", "left")):
        p = _pinch(hs[arm], pts, ss)
        if p is None:
            _ST["prev"][arm] = None
            continue
        lo, mid, hi = p
        if mid < _ST["smax"][arm] - BACK_TOL and not _ST["back"]:
            _ST["back"] = True
            _ST["back_info"] = f"{arm} at s={mid:.3f} after s={_ST['smax'][arm]:.3f}"
        _ST["smax"][arm] = max(_ST["smax"][arm], mid)
        # the hand itself moving back: on a free lead a closed hand pushes the lead along instead of sliding on it,
        # so the along-the-lead test above misses a back-and-forth stroke
        dist = float(np.linalg.norm(hs[arm][0] - c.start_plug))
        if dist < _ST["dmax"][arm] - BACK_TOL and not _ST["back"]:
            _ST["back"] = True
            _ST["back_info"] = f"{arm} {dist:.3f} from plug after {_ST['dmax'][arm]:.3f}"
        _ST["dmax"][arm] = max(_ST["dmax"][arm], dist)
        held = _plug_held(hs[other], c.start_plug)
        if held:
            pv = _ST["prev"][arm]
            if pv is not None and mid >= pv:     # credit only the stretch the pinch slid over since the last step
                _ST["wiped"] |= (BINS >= pv - 0.005 - 1e-9) & (BINS <= mid + 0.005 + 1e-9)
            _ST["prev"][arm] = mid
        else:
            _ST["prev"][arm] = None


def coverage(objects) -> float:
    _track(objects)
    return float(_ST["wiped"].mean())


def _on_drape(p) -> bool:
    return bool(DRAPE_X[0] <= p[0] <= DRAPE_X[1] and DRAPE_Y[0] <= p[1] <= DRAPE_Y[1] and p[2] <= TOP + 0.03)


def plug_on_drape(objects) -> bool:
    p = objects["lead"].start_plug
    return p is not None and np.isfinite(p).all() and _on_drape(p)


def lead_on_drape(objects) -> bool:
    c = objects["lead"]
    pts, _ = _dense_s(c)
    allp = np.vstack([pts] + [q[None] for q in (c.start_plug, c.end_plug) if q is not None])
    return bool(np.isfinite(allp).all() and all(_on_drape(q) for q in allp))


def released(objects) -> bool:
    c = objects["lead"]
    pts, _ = _dense_s(c)
    allp = np.vstack([pts] + [q[None] for q in (c.start_plug, c.end_plug) if q is not None])
    return all(float(np.linalg.norm(allp - h[0], axis=1).min()) >= RELEASE_DIST for h in hands().values())


def score(objects) -> float:
    """0 if a gripper closed on the lead ever went > 3 cm back toward the plug (along the lead, or in straight-line
    distance to the plug), the plug is not on the drape, or a gripper is
    within 3 cm of the lead at the end. Else by wiped fraction c of the blue-to-red stretch (wiped = passed
    between the pads of a gripper closed to <= 7 mm while the other gripper held the plug): 0.3 if c >= 0.5,
    0.6 if c >= 0.8, 1.0 if c >= 0.95 and the whole lead lies on the drape."""
    c = coverage(objects)
    if _ST["back"] or not plug_on_drape(objects) or not released(objects):
        return 0.0
    if c >= C_FULL and lead_on_drape(objects):
        return 1.0
    if c >= C_MID:
        return 0.6
    if c >= C_LOW:
        return 0.3
    return 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["lead"].resting(max_speed=0.02)


def distance(objects) -> float:
    """1 - wiped fraction (1.0 once the backward rule fired)."""
    c = coverage(objects)
    return 1.0 if _ST["back"] else 1.0 - c


def progress(objects) -> dict:
    hs = hands()
    c = objects["lead"]
    held = any(_plug_held(h, c.start_plug) for h in hs.values())
    return {"plug_held": int(held), "coverage": round(coverage(objects), 3), "no_backslide": int(not _ST["back"]),
            "plug_on_drape": int(plug_on_drape(objects)), "lead_on_drape": int(lead_on_drape(objects)),
            "released": int(released(objects))}


VARIATION = {"cable:lead": {"dx": 0.015, "dy": 0.015, "yaw": 0.05}}

# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------
WIPE_F = -0.0025               # finger joint target: pads 5 mm apart -> ~2.5 N squeeze per pad on the 6 mm lead
LIFT = TOP + 0.025
WIPE_Z = TOP + 0.0115         # commanded TCP height while wiping (the arm sits ~2.5 mm higher): pad tips ~1 mm above
                              # the table, so the lead, which stays on the table, is pinched below its centre line
OVER = 0.010
S_START = S_BLUE[0] + 0.005    # close on the near half of the blue band so the slide starts before the first mark


def lead_at(st, s):
    """Live point and heading of the lead at arclength s."""
    pts, ss = _dense_s(st["lead"])
    i = int(np.clip(np.searchsorted(ss, s), 1, len(ss) - 1))
    d = pts[min(i + 3, len(pts) - 1)] - pts[max(i - 3, 0)]
    return pts[i], math.atan2(d[1], d[0])


def pinch_s(st, arm):
    hs = hands()
    if not hs:
        return None
    pts, ss = _dense_s(st["lead"])
    p = _pinch(hs[arm], pts, ss)
    return None if p is None else p[1]


def demo_script(ctx):
    """Left arm takes the plug; right arm closes lightly around the lead at the blue band; both lift it 2.5 cm. The
    right arm slides along the lead toward the clip in 1.5 cm steps, each aimed at the live lead 1.5 cm past its
    measured pinch point, until the pinch passes the red band. Both arms then carry the lead to the drape, lay it
    down and let go."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("WIPE_CHEAT", "")  # test only: none | hover | onearm | hops | scrub | back | hold
    yaw = -math.pi / 2       # fingers close along x (across the lead); wrist cams look along -y (toward the clip)
    hi = TOP + 0.07

    def log(tag):
        st = ctx.states()
        hs = hands()
        hh = {k: (np.round(v[0], 3).tolist(), round(v[4] * 1000, 1)) for k, v in hs.items()}
        print(f"[demo t={ctx.t:5.1f}] {tag}: score={score(st):.2f} {progress(st)} pinchR={pinch_s(st, 'right')} "
              f"back={_ST['back_info']} hands={hh} plug={np.round(st['lead'].start_plug, 3).tolist()} "
              f"s30={np.round(lead_at(st, 0.30)[0], 3).tolist()}", flush=True)

    log("start")
    if cheat == "none":
        yield from L.wait(3.0)
        log("end")
        return
    st = ctx.states()
    pl = st["lead"].start_plug
    b, _ = lead_at(st, S_START)
    wf = FINGER_OPEN if cheat == "hover" else WIPE_F
    take_plug = cheat != "onearm"
    # 1) left takes the plug, right closes around the lead at the blue band
    moves = [Rt.move((b[0], b[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5)]
    if take_plug:
        yield from ctx.both(L.move((pl[0], pl[1], hi), yaw=yaw, finger=FINGER_OPEN, duration=1.5), moves[0])
        yield from ctx.both(L.move((pl[0], pl[1], pl[2] + OVER), yaw=yaw, duration=1.0),
                            Rt.move((b[0], b[1], WIPE_Z), yaw=yaw, duration=1.0))
        yield from ctx.both(L.grip(FINGER_CLOSED), Rt.grip(wf))
        yield from ctx.both(L.move((pl[0], pl[1], LIFT), yaw=yaw, duration=1.0),
                            Rt.move((b[0], b[1], WIPE_Z), yaw=yaw, duration=1.0))
    else:
        yield from moves[0]
        yield from Rt.move((b[0], b[1], WIPE_Z), yaw=yaw, duration=1.0)
        yield from Rt.grip(wf)
        yield from Rt.move((b[0], b[1], WIPE_Z), yaw=yaw, duration=1.0)
    log("pinched")
    # 2) wipe toward the clip
    pos = np.array([b[0], b[1], WIPE_Z])
    s_est = S_START
    step = 0.015
    target_end = S_RED[1] + 0.003
    if cheat == "hops":
        hop = float(os.environ.get("WIPE_HOP", "0.10"))
        for k in range(math.ceil((S_RED[0] + 0.01 - S_START) / hop)):
            yield from Rt.grip(FINGER_OPEN, 0.4)
            yield from Rt.move((pos[0], pos[1], WIPE_Z + 0.03), yaw=yaw, duration=0.5)
            st = ctx.states()
            s_est += hop
            p, _ = lead_at(st, s_est)
            pos = np.array([p[0], p[1], WIPE_Z])
            yield from Rt.move((pos[0], pos[1], WIPE_Z + 0.03), yaw=yaw, duration=0.6)
            yield from Rt.move((p[0], p[1], WIPE_Z), yaw=yaw, duration=0.5)
            yield from Rt.grip(WIPE_F, 0.4)
            yield from Rt.move(tuple(pos), yaw=yaw, duration=0.4)
            log(f"hop {k}")
    else:
        lost = 0
        for k in range(60):
            st = ctx.states()
            ps = pinch_s(st, "right")
            if ps is not None:
                s_est, lost = ps, 0
            else:
                lost += 1
                if lost >= 3 and cheat != "hover":
                    log("LOST the lead")
                    break
            if s_est >= target_end or (s_est >= S_RED[0] and coverage(st) >= 1.0):
                break
            p, _ = lead_at(st, min(s_est + step, LEAD_LEN))
            pos = np.array([float(np.clip(p[0], pos[0] - 0.01, pos[0] + 0.01)), max(pos[1] - step, -0.54), WIPE_Z])
            yield from Rt.move(tuple(pos), yaw=yaw, duration=0.3)
            if k % 4 == 0:
                log(f"wipe k={k} s={s_est:.3f}")
            if cheat == "scrub" and s_est > 0.35:   # closed stroke back 8 cm and forward again
                yield from Rt.move(tuple(pos + np.array([0.0, 0.08, 0.0])), yaw=yaw, duration=1.0)
                yield from Rt.move(tuple(pos), yaw=yaw, duration=1.0)
                log("scrubbed back")
                cheat = "scrubbed"
            if cheat == "back" and s_est > 0.35:   # open, go back 8 cm along the lead, close and wipe on
                yield from Rt.grip(FINGER_OPEN, 0.4)
                p, _ = lead_at(ctx.states(), s_est - 0.08)
                yield from Rt.move((p[0], p[1], WIPE_Z + 0.03), yaw=yaw, duration=0.8)
                yield from Rt.move((p[0], p[1], WIPE_Z), yaw=yaw, duration=0.5)
                yield from Rt.grip(WIPE_F, 0.4)
                pos = np.array([p[0], p[1], WIPE_Z])
                log("went back 8 cm")
                cheat = "went_back"
    log("wiped")
    # 3) lay the lead on the drape
    lay_l = np.array([0.19, 0.13])
    lay_r = np.array([0.19, -0.43])
    down = TOP + DRAPE_T + R + OVER + 0.004
    if take_plug:
        yield from ctx.both(L.move((lay_l[0], lay_l[1], LIFT), yaw=yaw, duration=2.0),
                            Rt.move((lay_r[0], lay_r[1], LIFT), yaw=yaw, duration=2.0))
        yield from ctx.both(L.move((lay_l[0], lay_l[1], down + 0.004), yaw=yaw, duration=1.0),
                            Rt.move((lay_r[0], lay_r[1], down), yaw=yaw, duration=1.0))
    else:
        yield from Rt.move((lay_r[0], lay_r[1], LIFT), yaw=yaw, duration=2.0)
        yield from Rt.move((lay_r[0], lay_r[1], down), yaw=yaw, duration=1.0)
    log("laid")
    if cheat == "hold":
        yield from L.wait(4.0)
        log("end (held)")
        return
    yield from ctx.both(L.grip(FINGER_OPEN), Rt.grip(FINGER_OPEN))
    yield from ctx.both(L.move((lay_l[0], lay_l[1], hi + 0.02), yaw=yaw, duration=0.8),
                        Rt.move((lay_r[0], lay_r[1], hi + 0.02), yaw=yaw, duration=0.8))
    log("released")
    yield from L.wait(1.5)
    log("end")
