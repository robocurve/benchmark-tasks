"""On the station table: a yellow 3 mm fibre-optic patch cord (44.5 cm, single-mode yellow jacket) leaves the front
(-y) face of a grey fibre enclosure box at the left of the table and lies straight across the table towards the
right arm, its free end carrying a black SC-style connector (30 x 12 x 10 mm). Part way along, the cord passes
3 mm from a white round radius guide (a 76 mm diameter, 25 mm tall cylinder fixed on the far side of the cord);
10-13 cm further on it passes 5 mm from a dark grey sharp-edged steel bracket (30 x 30 mm, 30 mm tall), also on
the far side. Beyond the guide, 10 cm along the line that leaves it away from the robot, is a white connector
holder: an open-top U slot (40 x 18 mm inside, 10 mm deep) running away from the robot, closed at its far end.
The robot must lay the cord round the guide (turning it 90 degrees away from the robot) and seat the connector
in the holder, never bending the cord tighter than 25 mm radius at any moment, then let go. Swinging the
connector straight over to the holder low over the table catches the cord on the bracket's square corner and
pulls it taut round it (the cord is far too short for that path); the cord must be lifted over the bracket and
wound onto the guide.

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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Route the yellow fibre patch cord round the white radius guide and seat its connector in the white "
               "holder beyond the guide, then let go. Never bend the cord tighter than the guide's curve (its minimum "
               "bend radius is 25 mm) at any moment, and leave it lying against the guide.")

# -- radius guide (round spool) --------------------------------------------------------------------------------------
GUIDE_C = np.array([0.262, 0.0])
GUIDE_R = 0.038  # 76 mm diameter: 1.5x the cord's 25 mm minimum bend radius
GUIDE_H = 0.025
WHITE = (0.94, 0.94, 0.92)
GUIDE = Cylinder("radius_guide", pos=(float(GUIDE_C[0]), float(GUIDE_C[1]), TOP + GUIDE_H / 2), radius=GUIDE_R,
                 height=GUIDE_H, color=WHITE)

# -- patch cord: anchored at the enclosure, connector on the free end ----------------------------------------------
R = 0.0015  # 3.0 mm patch cord
SEG = 0.015
WRAP_RC = GUIDE_R + R  # cord centreline radius when lying against the guide
CORD_X = float(GUIDE_C[0] - WRAP_RC - 0.003)  # start: 3 mm clear of the guide
CORD_Y0 = 0.27
CORD_LEN = 0.445
PL, PW, PH = 0.030, 0.012, 0.010
PLUG = Plug(size=(PL, PW, PH), mass=0.008, color=(0.08, 0.08, 0.08))
YELLOW = (0.98, 0.82, 0.10)
CORD = cable("cord", start=(CORD_X, CORD_Y0, TOP + PH / 2 + 0.0005), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
             segment=SEG, anchor_start=True, end_plug=PLUG, color=YELLOW, cable_mass=0.012,
             bend_stiffness=0.01, bend_damping=0.001)
ENCL_SIZE = (0.08, 0.06, 0.05)
ENCLOSURE = Box("fibre_enclosure", pos=(CORD_X, CORD_Y0 + 0.006 + ENCL_SIZE[1] / 2, TOP + ENCL_SIZE[2] / 2),
                size=ENCL_SIZE, color=(0.45, 0.47, 0.50))

# -- connector holder: open-top U slot along +x on the guide's far tangent line, closed at its far end ------------
HOLD_Y = float(GUIDE_C[1] - WRAP_RC)
SLOT_X0 = float(GUIDE_C[0] + 0.10)  # open end (towards the guide)
SLOT_L, SLOT_W, SLOT_H, WALL_T = 0.040, 0.018, 0.010, 0.004
SLOT_X1 = SLOT_X0 + SLOT_L
HOLDER = [
    Box(f"holder_wall_{s}", pos=((SLOT_X0 + SLOT_X1) / 2, HOLD_Y + sg * (SLOT_W / 2 + WALL_T / 2), TOP + SLOT_H / 2),
        size=(SLOT_L, WALL_T, SLOT_H), color=WHITE)
    for s, sg in (("l", 1), ("r", -1))
] + [Box("holder_back", pos=(SLOT_X1 + WALL_T / 2, HOLD_Y, TOP + SLOT_H / 2),
         size=(WALL_T, SLOT_W + 2 * WALL_T, SLOT_H), color=WHITE)]

# -- sharp-edged steel bracket beside the cord, just past the guide: a cord swung low from its start to the holder
#    catches on it and is pulled taut round its square corners (the cord is far too short for that path) --------
BRACKET_X = (CORD_X + R + 0.005, CORD_X + R + 0.035)  # 5 mm clear of the starting cord, 30 mm wide
BRACKET_Y = (-0.130, -0.100)  # 10-13 cm past the guide
BRACKET_H = 0.030
BRACKET = Box("steel_bracket", pos=(sum(BRACKET_X) / 2, sum(BRACKET_Y) / 2, TOP + BRACKET_H / 2),
              size=(BRACKET_X[1] - BRACKET_X[0], BRACKET_Y[1] - BRACKET_Y[0], BRACKET_H), color=(0.30, 0.32, 0.36))

OBJECTS = [ENCLOSURE, GUIDE, BRACKET, CORD, *HOLDER]

# -- scoring ---------------------------------------------------------------------------------------------------------
MIN_BEND = 0.025  # cord's minimum bend radius (m)
BEND_WINDOW = 2  # radius = arc length / turning angle over 2 segment pitches (3 cm of cord), see bend_radii()
WRAP_BEARINGS = np.radians([195.0, 225.0, 255.0])  # check points on the guide quarter facing the robot and its right
WRAP_TIGHT = 0.008  # cord surface within 8 mm of the guide face at every check point: full credit
WRAP_LOOSE = 0.020  # within 20 mm: half credit
RELEASE_DIST = 0.03  # every gripper TCP at least 3 cm from the cord and the connector


def bend_radii(points: np.ndarray, seg_len: float, window: int = BEND_WINDOW) -> np.ndarray:
    """Bend radius estimates (m) along the cord from its segment centres: tangent t_i = p_(i+1) - p_i, and over every
    run of ``window`` pitches the radius is arc length / turning angle = window * seg_len / angle(t_i, t_(i+window)).
    3-D, so lifting the cord into a tight droop counts too."""
    t = np.diff(points, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True) + 1e-12
    if len(t) <= window:
        return np.array([np.inf])
    cos = np.clip(np.sum(t[:-window] * t[window:], axis=1), -1.0, 1.0)
    ang = np.arccos(cos)
    return window * seg_len / np.maximum(ang, 1e-9)


def min_bend_radius(objects) -> float:
    c = objects["cord"]
    if not np.isfinite(c.points).all():
        return 0.0
    return float(bend_radii(c.points, c.seg_len).min())


# Episode memory: the smallest bend radius seen at any control step. Reset at every episode start (embodiment reset
# is wrapped below; as a fallback, the cord back in its exact starting layout also resets it).
_STATE = {"min_r": float("inf"), "steps": 0}
_ENV: dict = {"env": None}


def _reset_track() -> None:
    _STATE["min_r"], _STATE["steps"] = float("inf"), 0


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls.reset, "_fibre_hooks", None) is None:
        base_reset = cls.reset

        def reset(self, *a, **k):
            for fn in reset._fibre_hooks:
                fn()
            return base_reset(self, *a, **k)

        reset._fibre_hooks = []
        cls.reset = reset
    cls.reset._fibre_hooks.append(_reset_track)
    if not getattr(cls._task_step, "_fibre_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._fibre_hooked = True
        cls._task_step = _task_step


_install_hooks()
_START_PTS = CORD.start_points()


def _track(objects) -> None:
    c = objects["cord"]
    if np.isfinite(c.points).all() and np.abs(c.points - _START_PTS).max() < 0.002:
        _reset_track()  # cord in its exact, untouched starting layout => a new episode (fallback for the reset hook)
    _STATE["steps"] += 1
    r = min_bend_radius(objects)
    if r < _STATE["min_r"]:
        _STATE["min_r"] = r
        if np.isfinite(c.points).all():  # where it happened (diagnostic only): segment index and step
            _STATE["min_at"] = (int(np.argmin(bend_radii(c.points, c.seg_len))), _STATE["steps"])


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP (between the finger pads); [] if unknown (offline)."""
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


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def released(objects) -> bool:
    c = objects["cord"]
    pts = np.vstack([_dense(c.polyline())] + ([c.end_plug[None]] if c.end_plug is not None else []))
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def wrap_gaps(objects) -> np.ndarray:
    """Per check bearing (on the guide quarter between the cord's two straight legs): gap (m) from the guide face to
    the nearest cord surface within +-6 deg of that bearing and below the guide top; inf where no cord is there."""
    pts = _dense(objects["cord"].polyline(), 0.002)
    d = pts[:, :2] - GUIDE_C
    bear = np.arctan2(d[:, 1], d[:, 0]) % (2 * math.pi)
    dist = np.linalg.norm(d, axis=1) - GUIDE_R - R
    low = pts[:, 2] <= TOP + GUIDE_H
    out = np.full(len(WRAP_BEARINGS), np.inf)
    for k, b in enumerate(WRAP_BEARINGS):
        m = low & (np.abs((bear - b + math.pi) % (2 * math.pi) - math.pi) <= math.radians(6.0))
        if m.any():
            out[k] = float(dist[m].min())
    return out


def plug_seated(objects) -> bool:
    """Connector lying in the holder slot: centre between the walls, fully past the open end (at most 4 mm sticking
    out), on the slot floor."""
    p = objects["cord"].end_plug
    if p is None or not np.isfinite(p).all():
        return False
    ok_x = SLOT_X0 + PL / 2 - 0.004 <= p[0] <= SLOT_X1 - PL / 2 + 0.003
    ok_y = abs(p[1] - HOLD_Y) <= (SLOT_W - PW) / 2 + 0.002
    ok_z = p[2] <= TOP + PH / 2 + 0.004
    return bool(ok_x and ok_y and ok_z)


def analyse(objects) -> dict:
    _track(objects)
    c = objects["cord"]
    if not (np.isfinite(c.points).all() and (c.end_plug is None or np.isfinite(c.end_plug).all())):
        # the cord simulation blew up (e.g. pulled far beyond its length): treat as destroyed
        _STATE["min_r"] = 0.0
        return {"min_r": 0.0, "radius_ok": False, "gaps": np.full(len(WRAP_BEARINGS), np.inf), "tight": False,
                "loose": False, "seated": False, "released": False}
    g = wrap_gaps(objects)
    return {"min_r": _STATE["min_r"], "radius_ok": _STATE["min_r"] >= MIN_BEND, "gaps": g,
            "tight": bool((g <= WRAP_TIGHT).all()), "loose": bool((g <= WRAP_LOOSE).all()),
            "seated": plug_seated(objects), "released": released(objects)}


def score(objects) -> float:
    """0 if the cord's bend radius was ever below 25 mm during the episode, or the connector is not seated in the
    holder, or a gripper is within 3 cm of the cord/connector, or the cord is not round the guide (cord surface
    within 20 mm of the guide face at all 3 check bearings). Otherwise 1.0 if it lies against the guide (within 8 mm
    at all 3), else 0.5."""
    r = analyse(objects)
    if not (r["radius_ok"] and r["seated"] and r["released"] and r["loose"]):
        return 0.0
    return 1.0 if r["tight"] else 0.5


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects["cord"].resting(max_speed=0.02)


def distance(objects) -> float:
    """Connector centre to its seated position (m); 1 m once the radius limit has been broken."""
    _track(objects)
    if _STATE["min_r"] < MIN_BEND:
        return 1.0
    p = objects["cord"].end_plug
    seat = np.array([SLOT_X0 + PL / 2 + 0.002, HOLD_Y, TOP + PH / 2])
    return float(np.linalg.norm(p - seat)) if p is not None and np.isfinite(p).all() else 1.0


def progress(objects) -> dict:
    """Stage diagnostic (not the score): radius kept so far, cord round the guide (loose 0.5 / tight 1), connector
    seated, grippers released."""
    r = analyse(objects)
    return {"radius_kept": float(r["radius_ok"]), "round_guide": 1.0 if r["tight"] else (0.5 if r["loose"] else 0.0),
            "connector_seated": float(r["seated"]), "released": float(r["released"])}


# Seeded reset variation for a later runner (the demo runs on the nominal layout): the enclosure + cord exit may shift
# 1 cm along the cord and the cord's start may shift up to 2 mm towards or away from the guide (it starts 3 mm clear,
# so it stays at least 1 mm clear of the guide and 3 mm clear of the block);
# guide and holder are one fixed fixture.
VARIATION = {"cable:cord": {"dx": 0.002, "dy": 0.01, "yaw": 0.0}}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
HOLD_PT = (CORD_X, 0.06)  # left arm presses leg A to the table here while the right arm wraps the cord
CARRY_DZ = 0.010  # TCP above the connector centre when pinching it
CARRY_Z = TOP + 0.017  # connector centre while carried (bottom 2 mm above the holder walls)
SWING_Z = TOP + 0.08  # connector centre while the cord sweeps over the bracket at the start of the swing ...
LOW_SWING_Z = TOP + 0.025  # ... and once the cord is past it


def _cord_arclength_at(objects, xy) -> float:
    """Arclength (m, from the enclosure) of the cord point nearest ``xy``."""
    poly = objects["cord"].polyline()
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    pts = _dense(poly, 0.002)
    ds = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    return float(ds[int(np.linalg.norm(pts[:, :2] - np.asarray(xy), axis=1).argmin())]) * (s[-1] / max(ds[-1], 1e-9))


def _wrap_pose(phi: float, free: float) -> tuple[np.ndarray, float]:
    """Connector centre (x, y) and heading when the cord is wrapped ``phi`` rad round the guide with ``free`` m of
    cord between the guide and the connector (the involute of the guide)."""
    c = GUIDE_C + WRAP_RC * np.array([-math.cos(phi), -math.sin(phi)])
    t = np.array([math.sin(phi), -math.cos(phi)])
    return c + (free + PL / 2) * t, math.atan2(t[1], t[0])


def demo_script(ctx):
    """Left arm presses the cord to the table on its first leg, just before the guide. Right arm pinches the
    connector, lifts it 8 cm and carries it along the involute of the guide (so the cord winds onto the guide, kept
    straight between guide and connector): high while the cord sweeps over the bracket, then down near the table.
    It ends heading away from the robot with its wrist camera facing the holder, lowers the connector into the
    holder and lets go; then the left arm lets go.
    FIBRE_CHEAT=nothing|direct|kinkfix|unclipped|hold runs a shortcut instead (each must score 0)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("FIBRE_CHEAT", "")
    hi = TOP + 0.06

    def log(tag):
        st = ctx.states()
        r = analyse(st)
        p = st["cord"].end_plug
        br = bend_radii(st["cord"].points, st["cord"].seg_len) if np.isfinite(st["cord"].points).all() else [0]
        print(f"[demo t={ctx.t:5.1f}] {tag}: minR={r['min_r'] * 1000:.1f}mm nowR={min_bend_radius(st) * 1000:.1f}mm "
              f"@seg{int(np.argmin(br))} min_at={_STATE.get('min_at')} "
              f"gaps={np.round(r['gaps'] * 1000, 1)} seated={r['seated']} rel={r['released']} "
              f"score={score(st):.2f} plug={np.round(p, 3)}", flush=True)

    log("start")
    if cheat == "nothing":
        yield from L.wait(3.0)
        log("end")
        return

    if cheat == "probe_face":
        # noise check, not a strategy: press the cord against the bracket's flat side face and hold it there
        yz = TOP + 0.003 + 0.013
        yield from Rt.move((CORD_X - 0.02, -0.115, TOP + 0.06), yaw=math.pi / 2, finger=FINGER_CLOSED, duration=1.2)
        yield from Rt.move((CORD_X - 0.02, -0.115, yz), yaw=math.pi / 2, duration=0.8)
        yield from Rt.move((BRACKET_X[0] - 2 * R - 0.003, -0.115, yz), yaw=math.pi / 2, duration=1.0)
        yield from Rt.wait(2.0)
        log("pressed against bracket face")
        yield from Rt.move((CORD_X - 0.02, -0.115, TOP + 0.06), yaw=math.pi / 2, duration=0.8)
        log("end")
        return

    # 1) left arm presses leg A (closed fingertips 1 mm into the cord); right arm pinches the connector
    press_z = TOP + 2 * R - 0.001 + 0.013
    st = ctx.states()
    p = st["cord"].end_plug
    yield from ctx.both(L.move((HOLD_PT[0], HOLD_PT[1], hi), yaw=0.0, finger=FINGER_CLOSED, duration=1.2),
                        Rt.move((p[0], p[1], hi), yaw=-math.pi / 2, finger=FINGER_OPEN, duration=1.2))
    yield from ctx.both(L.move((HOLD_PT[0], HOLD_PT[1], press_z), yaw=0.0, duration=0.8),
                        Rt.move((p[0], p[1], p[2] + CARRY_DZ), yaw=-math.pi / 2, duration=0.8))
    yield from Rt.grip(FINGER_CLOSED)
    yield from Rt.move((p[0], p[1], CARRY_Z + CARRY_DZ), yaw=-math.pi / 2, duration=0.5)
    log("gripped")

    if cheat == "kinkfix":
        # fold the cord's free end back on itself (a tight U next to the cord), then put it back and do it properly
        q = (p[0] - 0.022, p[1] + 0.07)
        yield from Rt.move((p[0] - 0.02, p[1] - 0.01, CARRY_Z + CARRY_DZ + 0.02), yaw=-math.pi / 2, duration=0.8)
        yield from Rt.move((q[0], q[1] - 0.02, CARRY_Z + CARRY_DZ + 0.02), yaw=0.0, duration=1.0)
        yield from Rt.move((q[0], q[1], CARRY_Z + CARRY_DZ), yaw=math.pi / 2, duration=1.0)
        yield from Rt.wait(0.5)
        log("folded")
        yield from Rt.move((q[0], q[1] - 0.02, CARRY_Z + CARRY_DZ + 0.02), yaw=0.0, duration=1.0)
        yield from Rt.move((p[0] - 0.02, p[1] - 0.01, CARRY_Z + CARRY_DZ + 0.02), yaw=-math.pi / 2, duration=1.0)
        yield from Rt.move((p[0], p[1], CARRY_Z + CARRY_DZ), yaw=-math.pi / 2, duration=0.8)
        log("unfolded")

    if cheat == "direct":
        # carry the connector straight to the holder, low over the table, ignoring the guide
        tgt = np.array([SLOT_X0 + PL / 2 + 0.003, HOLD_Y])
        for k in range(1, 6):
            w = p[:2] + (tgt - p[:2]) * k / 5
            yield from Rt.move((w[0], w[1], CARRY_Z + CARRY_DZ), yaw=-math.pi / 2 * (1 - k / 5), duration=0.6)
            log(f"direct {k}/5")
    else:
        # 2) wind the cord onto the guide: connector along the involute, free length from the live cord
        st = ctx.states()
        s_t1 = _cord_arclength_at(st, GUIDE_C + np.array([-WRAP_RC, 0.0]))
        free0 = CORD_LEN - s_t1 + 0.004  # a little tension
        p = ctx.states()["cord"].end_plug
        yield from Rt.move((p[0], p[1], SWING_Z + CARRY_DZ), yaw=-math.pi / 2, duration=0.8)
        for phi in np.radians(np.arange(10.0, 90.1, 10.0)):
            free = max(free0 - WRAP_RC * phi, 0.03)
            # high while the cord sweeps over the bracket (high until 20 deg), then lowered between 20 and 50 deg to near the table
            z = float(np.interp(math.degrees(phi), [20.0, 50.0], [SWING_Z, LOW_SWING_Z]))
            free = math.sqrt(max(free ** 2 - (z - TOP - R) ** 2, 0.03 ** 2))  # cord rises to the connector
            c, h = _wrap_pose(phi, free)
            yield from Rt.move((c[0], c[1], z + CARRY_DZ), yaw=h, duration=0.5)
        log("wrapped")
        # 3) slide it along the holder line into the slot (wrist camera faces the holder: yaw 0 = +x)
        tgt_x = SLOT_X0 + PL / 2 + 0.003
        if cheat == "unclipped":
            tgt_x = SLOT_X0 - PL / 2 - 0.004
        st = ctx.states()
        p = st["cord"].end_plug
        yield from Rt.move((max(p[0], tgt_x - 0.01), HOLD_Y, CARRY_Z + CARRY_DZ), yaw=0.0, duration=1.0)
        yield from Rt.move((tgt_x, HOLD_Y, CARRY_Z + CARRY_DZ), yaw=0.0, duration=0.6)
    yield from Rt.move((Rt.tcp()[0], HOLD_Y, TOP + PH / 2 + CARRY_DZ + 0.002), yaw=0.0, duration=0.6)
    log("lowered")
    if cheat == "hold":
        yield from L.grip(FINGER_OPEN)
        yield from L.move((HOLD_PT[0], HOLD_PT[1], hi), yaw=0.0, duration=0.6)
        yield from Rt.wait(3.0)
        log("end (held)")
        return
    # 4) let go: right arm opens and rises, then the left arm lifts off the cord
    yield from Rt.grip(FINGER_OPEN)
    yield from Rt.move((Rt.tcp()[0], HOLD_Y, hi), yaw=0.0, duration=0.6)
    yield from L.move((HOLD_PT[0], HOLD_PT[1], hi), yaw=0.0, duration=0.6)
    yield from ctx.both(L.move((0.15, 0.20, hi), yaw=0.0, duration=0.8),
                        Rt.move((0.20, -0.22, hi), yaw=0.0, duration=0.8))
    log("released")
    yield from L.wait(1.5)
    log("end")
