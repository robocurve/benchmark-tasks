"""A hospital-bed mock-up on the station table: a 3 cm dark-blue mattress pad covers the work area. On its right side
lies a skin-coloured forearm block (20 x 6 x 2.5 cm, running away from the robot). On top of the forearm, a cannula
hub (white block 30 x 14 x 10 mm, 10 g) rests in a small fixed holder (the dressing): side walls, a back wall and a
front stop whose 9 mm gap lets the tube out but not the hub. A clear 5 mm IV tube, 60 cm long, leaves the hub towards
the robot's left and lies straight across the mattress (its far end is free: the rest of the line to the pump is
not modelled). A white U-clip (two walls 20 mm long, 12 mm tall, 7 mm slot running left-right) is fixed on the
mattress at the robot's midline, 14 cm closer to the robot than the tube.

The tube must be pressed into the clip so that, between the clip and the hub, it lies on the mattress in a loose loop
5-15 cm longer than the straight hub-clip distance, the hub still in its holder, and both grippers let go.
The demo uses both arms at once: the loop's clip end is pinned right of the clip (by IK only the right arm reaches
there) while the tube beyond it, which starts where only the left arm reaches, is drawn over the slot and pressed in
(the two held points are 22.5 cm apart along the tube). A one-arm strategy has not been tested or ruled out.

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

INSTRUCTION = ("Press the patient's clear IV tube into the white clip on the mattress, leaving a loose loop on the "
               "mattress between the clip and the cannula hub on the forearm: 5 to 15 cm more tube than the straight "
               "clip-to-hub distance. Do not lift the hub out of its holder. Then let go of the tube.")

# -- mattress, forearm, hub holder ------------------------------------------------------------------------------------
MAT_X = (0.10, 0.50)
MAT_Y = (-0.38, 0.42)
MAT_H = 0.03
MT = TOP + MAT_H  # mattress top
MATTRESS = Box("mattress", pos=((MAT_X[0] + MAT_X[1]) / 2, (MAT_Y[0] + MAT_Y[1]) / 2, TOP + MAT_H / 2),
               size=(MAT_X[1] - MAT_X[0], MAT_Y[1] - MAT_Y[0], MAT_H), color=(0.10, 0.17, 0.40))

ARM_X = (0.26, 0.46)
ARM_Y = (-0.30, -0.24)
ARM_H = 0.025
AT = MT + ARM_H  # forearm top
FOREARM = Box("forearm", pos=((ARM_X[0] + ARM_X[1]) / 2, (ARM_Y[0] + ARM_Y[1]) / 2, MT + ARM_H / 2),
              size=(ARM_X[1] - ARM_X[0], ARM_Y[1] - ARM_Y[0], ARM_H), color=(0.86, 0.66, 0.52))

HUB = Plug(size=(0.030, 0.014, 0.010), mass=0.010, color=(0.95, 0.95, 0.95))
HUB_X = 0.36
TUBE_Y0 = -0.250  # tube leaves the hub here, heading +y
HUB_Y = TUBE_Y0 - HUB.size[0] / 2  # -0.265
HOLD_H = 0.014  # holder walls, above the forearm top (hub is 10 mm tall)
HOLD_T = 0.003
HOLD_GAP = 0.009  # front stop gap (tube 5 mm, hub 14 mm)
HOLD_C = (0.80, 0.84, 0.86)
_side_x = HUB.size[1] / 2 + 0.0025 + HOLD_T / 2
_hy0, _hy1 = HUB_Y - HUB.size[0] / 2 - 0.0025, TUBE_Y0 + 0.0035  # inner faces of back wall / front stop
HOLDER = [
    *[Box(f"holder_side{k}", pos=(HUB_X + s * _side_x, (_hy0 + _hy1) / 2, AT + HOLD_H / 2),
          size=(HOLD_T, _hy1 - _hy0 + 2 * HOLD_T, HOLD_H), color=HOLD_C) for k, s in ((0, -1), (1, 1))],
    Box("holder_back", pos=(HUB_X, _hy0 - HOLD_T / 2, AT + HOLD_H / 2), size=(2 * _side_x + HOLD_T, HOLD_T, HOLD_H),
        color=HOLD_C),
    *[Box(f"holder_stop{k}", pos=(HUB_X + s * (HOLD_GAP / 2 + (_side_x - HOLD_GAP / 2) / 2), _hy1 + HOLD_T / 2,
                                  AT + HOLD_H / 2),
          size=(_side_x - HOLD_GAP / 2, HOLD_T, HOLD_H), color=HOLD_C) for k, s in ((0, -1), (1, 1))],
]

# -- clear IV tube: starts at the hub (free hub), straight along +y --------------------------------------------------
R = 0.0025
TUBE_LEN = 0.60
TUBE = cable("tube", start=(HUB_X, TUBE_Y0, AT + HUB.size[2] / 2 + 0.0005), yaw=math.pi / 2, length=TUBE_LEN,
             radius=R, start_plug=HUB, cable_mass=0.03, color=(0.80, 0.90, 0.93), bend_stiffness=0.01,
             bend_damping=0.001)

# -- U-clip on the mattress (slot along y) --------------------------------------------------------------------------
CLIP_X, CLIP_Y = 0.22, 0.0
GAP, WALL_T, WALL_L, WALL_H = 0.007, 0.004, 0.020, 0.012
CLIP_C = (0.96, 0.96, 0.94)
CLIP = [Box(f"clip_wall{k}", pos=(CLIP_X + s * (GAP / 2 + WALL_T / 2), CLIP_Y, MT + WALL_H / 2),
            size=(WALL_T, WALL_L, WALL_H), color=CLIP_C) for k, s in ((0, -1), (1, 1))]

OBJECTS = [MATTRESS, FOREARM, *HOLDER, TUBE, *CLIP]

VARIATION = {"cable:tube": {"dx": 0.0, "dy": 0.0, "yaw": 0.0}}  # the hub sits in a fixed holder: no variation

# -- scoring --------------------------------------------------------------------------------------------------------
SEATED_Z = MT + WALL_H - R  # tube top not above the clip walls
MIN_SPAN = 0.012  # runs through >= 12 mm of the 20 mm slot
SLACK_LO, SLACK_HI = 0.05, 0.15  # full credit
SLACK_MIN = 0.03  # below: taut, no loop -> 0
RELEASE_DIST = 0.03
HUB_LIFT = 0.007  # hub bottom more than this above the forearm top = lifted out of the holder


def _dense(poly: np.ndarray, step: float = 0.002) -> tuple[np.ndarray, np.ndarray]:
    pts, arc = [poly[0]], [0.0]
    s = 0.0
    for a, b in zip(poly[:-1], poly[1:]):
        seg = float(np.linalg.norm(b - a))
        k = max(1, int(seg / step))
        for j in range(1, k + 1):
            pts.append(a + (b - a) * (j / k))
            arc.append(s + seg * j / k)
        s += seg
    return np.array(pts), np.array(arc)


def hub_in_holder(objects) -> bool:
    h = objects["tube"].start_plug
    return bool(abs(h[0] - HUB_X) <= 0.006 and _hy0 - 0.002 <= h[1] <= _hy1 + 0.002
                and h[2] - HUB.size[2] / 2 <= AT + HUB_LIFT)


def clip_arc(objects) -> float | None:
    """Arclength (m, from where the tube leaves the hub) of the tube section seated in the clip slot, or None."""
    pts, arc = _dense(objects["tube"].polyline())
    along = pts[:, 1] - CLIP_Y
    inside = (np.abs(pts[:, 0] - CLIP_X) <= GAP / 2) & (np.abs(along) <= WALL_L / 2) & (pts[:, 2] <= SEATED_Z)
    if not inside.any() or np.ptp(along[inside]) < MIN_SPAN:
        return None
    return float(arc[inside].mean())


def slack(objects) -> float | None:
    """Tube length from the hub to the clip minus the straight distance between those two points (m)."""
    a = clip_arc(objects)
    if a is None:
        return None
    poly = objects["tube"].polyline()
    pts, arc = _dense(poly)
    p = pts[int(np.argmin(np.abs(arc - a)))]
    return float(a - np.linalg.norm(p - poly[0]))


def loop_on_mattress(objects) -> bool:
    """Every tube point between the hub and the clip lies over the mattress, none hanging below its top."""
    a = clip_arc(objects)
    pts, arc = _dense(objects["tube"].polyline())
    sel = pts[arc <= (a if a is not None else arc[-1])]
    return bool((sel[:, 0] >= MAT_X[0]).all() and (sel[:, 0] <= MAT_X[1]).all() and (sel[:, 1] >= MAT_Y[0]).all()
                and (sel[:, 1] <= MAT_Y[1]).all() and (sel[:, 2] >= MT + R - 0.003).all())


_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_ivloop_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._ivloop_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads), [] if unknown."""
    env = _ENV["env"]
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
    c = objects["tube"]
    pts = np.vstack([_dense(c.polyline())[0], c.start_plug[None]])
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def score(objects) -> float:
    """0 unless: the tube is finite and at rest, both grippers are >= 3 cm from tube and hub, the hub is in its
    holder, the tube is seated in the clip and the tube between hub and clip lies on the mattress. Then by slack
    (tube length hub->clip minus straight distance): 5-15 cm = 1.0; 3-5 cm or more than 15 cm = 0.5; < 3 cm = 0."""
    c = objects["tube"]
    if not (np.isfinite(c.points).all() and np.isfinite(c.start_plug).all()):
        return 0.0
    if not (c.resting(max_speed=0.05) and released(objects) and hub_in_holder(objects)):
        return 0.0
    s = slack(objects)
    if s is None or not loop_on_mattress(objects) or s < SLACK_MIN:
        return 0.0
    return 1.0 if SLACK_LO <= s <= SLACK_HI else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["tube"].resting(max_speed=0.02)


def distance(objects) -> float:
    """|slack - 10 cm| (m) when the tube is seated in the clip, else 1 m."""
    s = slack(objects)
    return 1.0 if s is None else abs(s - 0.10)


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    s = slack(objects)
    return {"hub_in_holder": float(hub_in_holder(objects)), "tube_in_clip": float(s is not None),
            "loop_slack_ok": float(s is not None and SLACK_LO <= s <= SLACK_HI),
            "loop_on_mattress": float(loop_on_mattress(objects)), "released": float(released(objects))}


# -- closed-loop scripted demo (cabledemo.py) ----------------------------------------------------------------------


def _tube_at(objects, a: float) -> tuple[np.ndarray, float]:
    """Live tube point at arclength ``a`` from the hub end, and the tube heading there."""
    poly = objects["tube"].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    p = np.array([np.interp(a, s, poly[:, k]) for k in range(3)])
    i = min(max(int(np.searchsorted(s, a)), 1), len(poly) - 1)
    d = poly[i] - poly[i - 1]
    return p, math.atan2(d[1], d[0])


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _tube_dx_at_clip(objects, a_lo: float, a_hi: float) -> float:
    pts, arc = _dense(objects["tube"].polyline())
    sel = (arc >= a_lo) & (arc <= a_hi)
    pts = pts[sel] if sel.any() else pts
    i = int(np.argmin(np.abs(pts[:, 1] - CLIP_Y)))
    return float(pts[i, 0] - CLIP_X)


DEMO = dict(slack=0.10, pin_back=0.18, lead=0.045, sag=0.012, debug=True)


def demo_script(ctx):
    """Closed loop (grasp points and lateral correction from the live tube state). The tube point that must end at
    the clip (straight hub-clip distance + 10 cm of slack along the tube) starts in front of the left arm.
    1) Both hands take the tube: the right 18 cm hub-side of that point (in front of the right arm), the left 4.5 cm
    beyond it (in front of the left arm only). 2) The right hand slides its point low along the mattress (so the tube
    leaves the hub straight and turns in a wide U) and pins it 18 cm right of the clip; at the same time the left hand
    draws the section over the slot to 3.3 cm past the clip (both wrist cameras face the clip). 3) The left hand lowers
    the tube into the slot, shifting sideways from the measured tube position at the clip. 4) Both let go and lift
    away. Cheat variants (exactly 0) via IV_CHEAT."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("IV_CHEAT", "")  # test only: nothing | taut | unclipped | dislodge | hold
    over = 0.010
    hi = MT + 0.07
    carry = MT + 0.035

    def log(tag):
        if not DEMO["debug"]:
            return
        st = ctx.states()
        c = st["tube"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: clip_arc={clip_arc(st)} slack={slack(st)} hub={np.round(c.start_plug, 4)}"
              f" in_holder={hub_in_holder(st)} on_mat={loop_on_mattress(st)} rel={released(st)} "
              f"score={score(st):.2f}", flush=True)
        print("   xy:", " ".join(f"({p[0]*100:.0f},{p[1]*100:.0f},{(p[2]-MT)*1000:.0f})" for p in c.points[::2]), flush=True)

    def grasp(arm, a, tries=3, cam_away=None):
        """Grasp the tube at arclength a where it actually lies; returns the grip yaw. ``cam_away``: a world y
        direction (+1/-1) the wrist camera must point to (keeps it clear of the other hand)."""
        for attempt in range(tries):
            p, h = _tube_at(ctx.states(), a)
            gy = _grip_yaw(h)
            if cam_away is not None and math.sin(gy) * cam_away < 0:
                gy = gy + math.pi if gy < 0 else gy - math.pi
            yield from arm.move((p[0], p[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.0)
            yield from arm.move((p[0], p[1], p[2] + over), yaw=gy, duration=0.8)
            yield from arm.grip(FINGER_CLOSED, 0.5)
            yield from arm.move((p[0], p[1], carry), yaw=gy, duration=0.6)
            q, _ = _tube_at(ctx.states(), a)
            miss = float(np.linalg.norm(q - (arm.tcp() - np.array([0.0, 0.0, over]))))
            log(f"  grasp a={a:.3f} attempt {attempt} miss={miss * 1000:.0f}mm")
            if miss < 0.02:
                return gy
            yield from arm.grip(FINGER_OPEN, 0.4)
        return gy

    log("start")
    if cheat == "nothing":
        yield from L.wait(4.0)
        log("end")
        return

    if cheat == "dislodge":  # the right hand lifts the tube next to the hub: the hub comes out of its holder and is
        # set down beside the forearm (logged at x 31.9, y -33.0 cm); then the normal demo follows
        yield from grasp(Rt, 0.025, cam_away=+1)
        yield from Rt.move((HUB_X, TUBE_Y0 + 0.025, AT + 0.06), yaw=math.pi / 2, duration=0.8)
        yield from Rt.move((HUB_X - 0.06, TUBE_Y0 + 0.025, AT + 0.06), yaw=math.pi / 2, duration=0.8)
        yield from Rt.move((HUB_X - 0.06, TUBE_Y0 + 0.025, AT + 0.015), yaw=math.pi / 2, duration=0.6)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((0.25, -0.15, hi), yaw=math.pi / 2, duration=0.8)
        log("hub pulled out")

    st = ctx.states()
    p0 = st["tube"].polyline()[0]
    clip_pt = np.array([CLIP_X, CLIP_Y, MT + R])
    target = DEMO["slack"] if cheat != "taut" else 0.008
    a_clip = float(np.linalg.norm(clip_pt - p0)) + target
    a1, a2 = a_clip - DEMO["pin_back"], a_clip + DEMO["lead"]
    g1 = np.array([CLIP_X, CLIP_Y - DEMO["pin_back"]])  # right hand pins the loop end, hub side of the clip
    if cheat == "taut":  # clip it with the tube running straight from the hub to just short of the clip (no loop)
        g1 = np.array([CLIP_X + 0.02, CLIP_Y - 0.09])
        a1 = float(np.linalg.norm(np.r_[g1, MT + R] - p0)) + 0.004
        a2 = a1 + float(np.linalg.norm(g1 - clip_pt[:2])) + DEMO["lead"]
    g2 = np.array([CLIP_X, CLIP_Y + DEMO["lead"] - DEMO["sag"]])  # left hand, far side (tube sags into the slot)
    if cheat == "unclipped":  # loop formed and laid down beside the clip, never pressed in
        g1[0] -= 0.04
        g2[0] -= 0.04
    yr = math.pi / 2  # right hand (pins the loop end): wrist camera along +y, facing the clip
    yl = -math.pi / 2  # left hand (lowers the tube into the slot): wrist camera along -y, facing the clip
    low = MT + R + over

    # 1) both arms take the tube at the same time: the right at a1 (the loop's clip end, in front of the right arm),
    #    the left at a2 (22.5 cm further along, in front of the left arm only)
    yield from ctx.both(grasp(Rt, a1, cam_away=+1), grasp(L, a2, cam_away=-1))  # grasp yaw = carry yaw
    log("both grasped")
    # 2) both carry their points across: the right pins a1 on the mattress 18 cm right of the clip, the left draws the
    #    section between the hands over the slot to 3.3 cm past the clip
    #    (the right hand slides low along the mattress via a point that lets the tube leave the hub straight and
    #    turn in a wide U; lifting the near-straight tube instead levers the hub up out of its holder)
    if cheat == "taut":  # straight to its pin point (the tube to the hub stays straight)
        yield from Rt.move((Rt.tcp()[0], Rt.tcp()[1], low + 0.004), yaw=yr, duration=0.4)
    else:
        yield from Rt.move((Rt.tcp()[0], Rt.tcp()[1], low + 0.004), yaw=yr, duration=0.4)
        yield from ctx.both(Rt.move((0.27, g1[1] + 0.11, low + 0.004), yaw=yr, duration=1.2),
                            L.move((0.26, g2[1] + 0.08, carry), yaw=yl, duration=1.2))
    yield from ctx.both(Rt.move((g1[0], g1[1], low), yaw=yr, duration=1.2),
                        L.move((g2[0], g2[1], carry), yaw=yl, duration=1.2))
    yield from L.wait(0.3)
    log("right pins loop end, left holds far side")
    gx = g2[0]
    lever = (g2[1] - g1[1]) / (CLIP_Y - g1[1])  # hand shift per tube shift at the clip
    for z in (carry, MT + 0.025, MT + 0.018):
        yield from L.move((gx, g2[1], z), yaw=yl, duration=0.5)
        yield from L.wait(0.2)
        off = _tube_dx_at_clip(ctx.states(), a1 - 0.02, a2)
        if cheat != "unclipped":
            gx -= float(np.clip(off * lever, -0.01, 0.01))
        log(f"  lower z={z - MT:.3f} off={off * 1000:+.1f}mm gx={gx:.4f}")
    yield from L.move((gx, g2[1], low - 0.002), yaw=yl, duration=0.8)
    yield from L.wait(0.4)
    log("seated, holding")
    if cheat == "hold":
        yield from L.wait(4.0)
        log("end (held)")
        return
    # 4) let go and lift away (left first, then the right hand that pins the loop)
    yield from L.grip(FINGER_OPEN, 0.5)
    yield from L.move((gx, g2[1], hi), yaw=yl, duration=0.8)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((g1[0], g1[1], hi), yaw=yr, duration=0.8)
    yield from ctx.both(Rt.move((0.18, -0.20, hi), yaw=yr, duration=1.0), L.move((0.18, 0.20, hi), yaw=yl, duration=1.0))
    log("released")
    yield from L.wait(2.0)
    log("end")
