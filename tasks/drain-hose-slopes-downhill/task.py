"""An air-conditioner condensate drain hose must fall continuously from the unit to the drain.

On the station table, on the robot's left, stands a white indoor-unit block (12 x 10 x 20 cm). An 8 mm off-white
PVC condensate hose, 56 cm long, leaves its right-hand face 14 cm above the table (outlet fixed) and lies straight on
the table towards the robot's right, ending in a grey 30 x 14 x 14 mm drain nozzle. Further from the robot, two
grey support blocks of decreasing height (10.5 and 8 cm) step down along a bent path from the unit to a drain:
a dark grey tundish (open box, inner 6.5 x 6.5 cm) standing on a 2.5 cm plinth, rim 4.5 cm above the table, on the
right. The nozzle must end inside the tundish with the hose falling all the way from the outlet (no trap): carried
straight across, the hose sags to the table and has to climb back up into the raised tundish, which forms a water
trap. Laying it over the two steps gives a continuous fall. Both grippers must let go.

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

INSTRUCTION = ("Route the air conditioner's white condensate hose into the drain tundish so that it slopes downhill "
               "all the way from the unit to the drain, with no dip that would trap water: lay it over the two grey "
               "steps and put the nozzle inside the tundish. Then let go of the hose.")

WHITE = (0.93, 0.93, 0.91)
GREY = (0.55, 0.57, 0.60)
DARK = (0.25, 0.26, 0.28)
HOSE_COLOR = (0.90, 0.92, 0.95)

# -- unit and hose -------------------------------------------------------------------------------------------------
R = 0.004  # 8 mm hose
HOSE_LEN = 0.56
OUT_Z = 0.140  # outlet (hose centre) height above the table
OUT_XY = (0.14, 0.34)  # outlet position; the hose leaves towards -y
NOZZLE = Plug(size=(0.030, 0.014, 0.014), mass=0.012, color=(0.45, 0.46, 0.48))
HOSE_START = (OUT_XY[0], OUT_XY[1], TOP + OUT_Z)
UNIT_SIZE = (0.12, 0.10, 0.20)
UNIT = Box("ac_unit", pos=(OUT_XY[0], OUT_XY[1] + 0.006 + UNIT_SIZE[1] / 2, TOP + UNIT_SIZE[2] / 2), size=UNIT_SIZE,
           color=WHITE)
HOSE = cable("hose", start=HOSE_START, yaw=-math.pi / 2, length=HOSE_LEN, radius=R, anchor_start=True,
             end_plug=NOZZLE, cable_mass=0.03, color=HOSE_COLOR, bend_stiffness=0.01, bend_damping=0.001)

# -- drain: tundish (open box) on a plinth -------------------------------------------------------------------------
D_C = (0.22, -0.10)
PLINTH_H = 0.025
PLINTH = 0.11  # square
WELL_IN, WELL_WALL, WELL_H = 0.065, 0.008, 0.020
RIM = PLINTH_H + WELL_H  # 4.5 cm above the table
PLINTH_BOX = Box("drain_plinth", pos=(D_C[0], D_C[1], TOP + PLINTH_H / 2), size=(PLINTH, PLINTH, PLINTH_H),
                 color=GREY)
_wo = WELL_IN / 2 + WELL_WALL / 2
_wl = WELL_IN + 2 * WELL_WALL
_wz = TOP + PLINTH_H + WELL_H / 2
TUNDISH = [
    Box("tundish_near", pos=(D_C[0] - _wo, D_C[1], _wz), size=(WELL_WALL, _wl, WELL_H), color=DARK),
    Box("tundish_far", pos=(D_C[0] + _wo, D_C[1], _wz), size=(WELL_WALL, _wl, WELL_H), color=DARK),
    Box("tundish_left", pos=(D_C[0], D_C[1] + _wo, _wz), size=(WELL_IN, WELL_WALL, WELL_H), color=DARK),
    Box("tundish_right", pos=(D_C[0], D_C[1] - _wo, _wz), size=(WELL_IN, WELL_WALL, WELL_H), color=DARK),
]

# -- two steps along the bent path outlet -> S2 (corner) -> drain ------------------------------------------------
S2_C = (0.32, 0.14)
STEP_LEN, STEP_W = 0.10, 0.06  # along the path, across it
_a = math.atan2(S2_C[1] - OUT_XY[1], S2_C[0] - OUT_XY[0])  # leg A heading (outlet -> S2)
_b = math.atan2(D_C[1] - S2_C[1], D_C[0] - S2_C[0])  # leg B heading (S2 -> drain)
S1_C = ((OUT_XY[0] + S2_C[0]) / 2 + 0.01 * math.cos(_a), (OUT_XY[1] + S2_C[1]) / 2 + 0.01 * math.sin(_a))
S_H = (0.105, 0.080)  # step heights above the table
STEPS = [
    Box("step_high", pos=(S1_C[0], S1_C[1], TOP + S_H[0] / 2), size=(STEP_LEN, STEP_W, S_H[0]), yaw=_a, color=GREY),
    Box("step_mid", pos=(S2_C[0], S2_C[1], TOP + S_H[1] / 2), size=(0.08, 0.08, S_H[1]), yaw=(_a + _b) / 2,
        color=GREY),
]

OBJECTS = [UNIT, HOSE, PLINTH_BOX, *TUNDISH, *STEPS]

# -- grippers (score requires both to have let go) ----------------------------------------------------------------
RELEASE_DIST = 0.035  # each gripper TCP (between the finger pads) at least this far from the hose and the nozzle
_ENV: dict = {"env": None}


def _install_env_hook() -> None:
    """Keep a handle on the sim env so score() can check that the grippers have let go."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_drain_hooked", False):
        return
    base = cls._task_step

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        return base(self, *a, **k)

    _task_step._drain_hooked = True
    cls._task_step = _task_step


_install_env_hook()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads, 13 mm above the pad tips), [] if unknown."""
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


# -- scoring geometry -----------------------------------------------------------------------------------------------


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def _line(objects) -> np.ndarray:
    """Hose centre line from the outlet to the nozzle's front face."""
    c = objects["hose"]
    poly = c.polyline()
    if c.end_plug is not None:
        poly = np.vstack([poly, c.end_tip + c.end_dir * NOZZLE.size[0]])
    return poly


def released(objects) -> bool:
    pts = _dense(_line(objects))
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def in_drain(objects) -> bool:
    """Nozzle centre inside the tundish's inner walls (6.5 x 6.5 cm opening) and below its rim."""
    p = objects["hose"].end_plug
    h = WELL_IN / 2
    return bool(p is not None and abs(p[0] - D_C[0]) <= h and abs(p[1] - D_C[1]) <= h and p[2] <= TOP + RIM)


NOISE = 0.002  # height changes below 2 mm are ignored


def total_rise(objects) -> float:
    """Total height (m) the hose centre line climbs going from the outlet to the nozzle (the hose's end, nozzle body
    excluded). Climbs are counted with a 2 mm dead band: a climb counts once the line is more than 2 mm above the
    lowest point since the last counted climb, and then counts in full from that low point."""
    z = objects["hose"].polyline()[:, 2]
    low, rise = float(z[0]), 0.0
    for v in z[1:]:
        v = float(v)
        if v < low:
            low = v
        elif v - low > NOISE:
            rise += v - low
            low = v
    return rise


RISE_FULL, RISE_HALF = 0.005, 0.015


def score(objects) -> float:
    """0 unless both grippers are >= 3.5 cm from the hose and nozzle, the nozzle centre is inside the tundish
    (inner walls, below the rim) and the hose rests on both steps (some hose segment inside each step's footprint,
    its underside at most 1.1 cm above the step top). Then by the hose's total climb from outlet to nozzle: 1.0 if
    <= 5 mm, 0.5 if <= 15 mm, else 0."""
    if not in_drain(objects) or not released(objects) or not on_both_steps(objects):
        return 0.0
    r = total_rise(objects)
    return 1.0 if r <= RISE_FULL else 0.5 if r <= RISE_HALF else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["hose"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Nozzle centre to the tundish centre (m) plus the hose's total climb (m)."""
    p = objects["hose"].end_plug
    d = float(np.linalg.norm(p[:2] - np.array(D_C))) if p is not None else 1.0
    return d + total_rise(objects)


def _on_step(objects, i: int) -> bool:
    """Some hose segment rests on top of step i: its centre inside the step's footprint and 0-1.5 cm above the step
    top (the 8 mm hose's underside touching or at most 1.1 cm above it)."""
    b = STEPS[i]
    pts = objects["hose"].points
    c, s = math.cos(b.yaw), math.sin(b.yaw)
    rel = pts[:, :2] - np.array(b.pos[:2])
    u = rel @ np.array([c, s])
    v = rel @ np.array([-s, c])
    dz = pts[:, 2] - (TOP + S_H[i])
    return bool(np.any((np.abs(u) <= b.size[0] / 2) & (np.abs(v) <= b.size[1] / 2) & (dz > 0) & (dz < 0.015)))


def on_both_steps(objects) -> bool:
    return _on_step(objects, 0) and _on_step(objects, 1)


def progress(objects) -> dict:
    """Stage diagnostic for analysis (not the score)."""
    c = objects["hose"]
    g = grippers()
    r = total_rise(objects)
    return {
        "end_grasped": int(c.end_plug is not None and any(float(np.linalg.norm(x - c.end_plug)) < 0.03 for x in g)),
        "on_step_high": int(_on_step(objects, 0)),
        "on_step_low": int(_on_step(objects, 1)),
        "end_in_drain": int(in_drain(objects)),
        "rise_le_15mm": int(r <= RISE_HALF),
        "rise_le_5mm": int(r <= RISE_FULL),
        "released": int(released(objects)),
    }


# Seeded reset variation for a later runner (the demo runs on the nominal layout and follows the live hose). The
# outlet slides along the unit's face only (dy would push the welded start into the unit or away from it); the
# hose must leave the outlet square to the face (yaw 0). Every other part is fixed.
VARIATION = {"cable:hose": {"dx": 0.01, "dy": 0.0, "yaw": 0.0}}


# -- closed-loop scripted demo (cabledemo.py) --------------------------------------------------------------------


def _grip_yaw(heading: float) -> float:
    return (heading + math.pi / 2) % math.pi - math.pi / 2


def _hose_at(objects, a: float):
    """Live hose point at arclength ``a`` from the outlet, and the hose's heading there."""
    poly = objects["hose"].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    p = np.array([np.interp(a, s, poly[:, k]) for k in range(3)])
    i = min(max(int(np.searchsorted(s, a)), 1), len(poly) - 1)
    d = poly[i] - poly[i - 1]
    return p, math.atan2(d[1], d[0])


def demo_script(ctx):
    """Left arm takes the hose at the point that belongs on the low step, right arm takes the nozzle; together
    they lift the hose, the left lays its point on the low step (the hose from the outlet drops onto the high
    step) while the right carries the nozzle, camera ahead, over the tundish. The right lowers the nozzle to just
    above the rim and lets go (the hose drops onto the low step and the nozzle into the tundish), then the left lets
    go and both arms move clear.
    DRAIN_CHEAT=nothing | direct (right arm alone carries the nozzle into the tundish, the hose hanging free) |
    onestep (the left lays the hose on the high step only, picking it where it lies on the table) | onestep_taut
    (the same, but the hose is picked so it runs taut from the outlet over the high step) | hold (full demo, but the right keeps holding the nozzle
    in the tundish) -- all must score 0. DRAIN_CHEAT=settle only waits (debug)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("DRAIN_CHEAT", "")
    GRASP_OPEN = -0.014  # pads 28 mm apart round the 8 mm hose
    over = 0.010  # TCP above the hose / nozzle centre when grasping
    hi = TOP + 0.15

    def log(tag):
        st = ctx.states()
        c = st["hose"]
        print(f"[demo t={ctx.t:5.1f}] {tag}: nozzle={np.round(c.end_plug, 3)} rise={total_rise(st)*1000:.1f}mm "
              f"drain={in_drain(st)} rel={released(st)} score={score(st):.2f} prog={progress(st)}", flush=True)
        print("   z(mm):", np.round((c.polyline()[:, 2] - TOP) * 1000).astype(int).tolist(), flush=True)

    log("start")
    yield from L.wait(1.5)
    log("settled")
    if cheat in ("settle", "nothing"):
        yield from L.wait(3.0)
        log("end")
        return

    st = ctx.states()
    c = st["hose"]
    out = np.array(HOSE_START)
    # where the left arm will put its hose point: on the low step, a little before its centre
    put = np.array([S2_C[0] - 0.01 * math.cos(_a), S2_C[1] - 0.01 * math.sin(_a), TOP + S_H[1] + R])
    tgt_step = 0 if cheat.startswith("onestep") else 1
    if tgt_step == 0:
        put = np.array([S1_C[0] + 0.03 * math.cos(_a), S1_C[1] + 0.03 * math.sin(_a), TOP + S_H[0] + R])
    a_left = float(np.linalg.norm(put - out)) + float(os.environ.get("DRAIN_SLACK", "0.004"))
    if cheat == "onestep":
        a_left = 0.25  # a point lying on the table; the extra length stays on the high step
    pl, hl = _hose_at(st, a_left)
    yl = _grip_yaw(hl) if cheat != "onestep_taut" else -math.pi / 2
    pn = c.end_plug
    yn = _grip_yaw(math.atan2(c.end_dir[1], c.end_dir[0]))
    if yn < -1.2:
        yn += math.pi  # same grasp, IK-friendly for the right arm
    y_carry = math.radians(40.0)  # wrist camera ahead, towards the tundish
    print(f"[demo] left grasps a={a_left:.3f} at {np.round(pl, 3)} yaw {math.degrees(yl):.0f}; nozzle {np.round(pn, 3)}"
          f" yaw {math.degrees(yn):.0f}", flush=True)

    def right_take():
        yield from Rt.move((pn[0], pn[1], TOP + 0.06), yaw=yn, finger=FINGER_OPEN, duration=1.5)
        p = ctx.states()["hose"].end_plug
        yield from Rt.move((p[0], p[1], p[2] + over), yaw=yn, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.5)

    def left_take():
        yield from L.move((pl[0], pl[1], TOP + 0.06), yaw=yl, finger=GRASP_OPEN, duration=1.5)
        p, _ = _hose_at(ctx.states(), a_left)
        yield from L.move((p[0], p[1], p[2] + over), yaw=yl, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.5)

    if cheat == "direct":
        yield from right_take()
    else:
        yield from ctx.both(right_take(), left_take())
    log("grasped")

    above_d = (D_C[0], D_C[1], hi)
    seat = (D_C[0], D_C[1], TOP + RIM + NOZZLE.size[2] / 2 + 0.012 + over)
    ys = _a if tgt_step == 0 else (_a + _b) / 2
    ys = _grip_yaw(ys)
    if cheat == "direct":
        yield from Rt.move((pn[0], pn[1], hi), yaw=yn, duration=0.8)
        yield from Rt.move(above_d, yaw=y_carry, duration=2.0)
    else:
        yield from ctx.both(Rt.move((pn[0] + 0.02, pn[1], hi), yaw=yn, duration=1.0),
                            L.move((pl[0], pl[1], hi), yaw=yl, duration=1.0))
        yield from ctx.both(Rt.move(above_d, yaw=y_carry, duration=2.0),
                            L.move((put[0], put[1], hi), yaw=ys, duration=2.0))
        log("lifted")
        yield from L.move((put[0], put[1], put[2] + over), yaw=ys, duration=1.5)
        log("left on step")
    yield from Rt.move(seat, yaw=y_carry, duration=1.2)
    log("nozzle over tundish")
    if cheat == "hold":
        yield from L.grip(GRASP_OPEN, 0.5)
        yield from L.move((put[0], put[1], hi), yaw=ys, duration=0.8)
        yield from L.move((0.15, 0.25, TOP + 0.22), yaw=0.0, duration=1.2)
        yield from Rt.move((D_C[0], D_C[1], TOP + PLINTH_H + 0.007 + over + 0.004), yaw=y_carry, duration=1.0)
        yield from Rt.wait(3.0)
        log("end (held)")
        return
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move(above_d, yaw=y_carry, duration=0.8)
    yield from Rt.move((0.18, -0.30, TOP + 0.18), yaw=y_carry, duration=1.0)
    log("right let go")
    if cheat != "direct":
        yield from L.grip(GRASP_OPEN, 0.5)
        yield from L.move((put[0], put[1], hi), yaw=ys, duration=0.8)
        yield from L.move((0.15, 0.25, TOP + 0.22), yaw=0.0, duration=1.2)
    log("released")
    yield from L.wait(1.5)
    log("end")
