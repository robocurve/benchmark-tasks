"""Operating-room mock-up (no patient) on the station table. On the left half of the table stands a raised sterile
field: a 5 cm tall box covered with a blue drape (44 x 48 cm), whose right-hand edge runs along the table's midline
(y = 0) in front of the robot. A white sterile device cord (8 mm, 45 cm) leaves a white device holster fixed near the
field's front-right corner and lies straight across the drape, diagonally away from the robot and to the left; it
ends in a black 80 x 16 x 14 mm plug. A grey cord holder is fixed on a line running from the field edge diagonally
(-60 deg) towards a grey device console on the right half of the table: an open-top slot on the drape (14 mm wide,
30 mm walls, 8.5-12.5 cm in from the edge along the line) and a flat ledge at drape height that reaches 8.5 cm beyond the edge.
The console's jack (a 20 x 18 mm opening, 30 mm deep) faces the holder, 16 cm beyond the field edge along the line.

The left arm plays the scrubbed (sterile) team member and must stay on the field side of the edge; the right arm
plays the circulator and must never cross over the field. The left arm brings the plug in along the holder line so
the cord lies in the slot, and leaves the plug on the ledge, off the field; the right arm takes the plug from there
and pushes it fully into the console jack. The plug must never touch the table; at the end both arms let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var STERILE_CHEAT): nothing | drop | scrub_carry | noclip | hold | hold_cord | reach_over;
tier03 stops with the plug on the ledge (partial-credit check, expect 0.3).
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

INSTRUCTION = ("Plug the sterile white cord into the grey console's jack, with the cord lying in the grey holder's slot. The "
               "left arm and the blue drape are sterile, the right arm is not. Let go with both arms at the end.")

# -- colours ----------------------------------------------------------------------------------------------------------
DRAPE_BLUE = (0.16, 0.42, 0.68)
WHITE = (0.92, 0.92, 0.90)
HOLDER_GREY = (0.62, 0.64, 0.66)
CONSOLE_GREY = (0.78, 0.79, 0.80)
JACK_DARK = (0.22, 0.23, 0.25)
PLUG_BLACK = (0.07, 0.07, 0.08)

# -- sterile field: a draped raised box, right edge on y = 0 ------------------------------------------------------------
FIELD_H = 0.05
FT = TOP + FIELD_H  # drape top (sterile surface)
FIELD_X0, FIELD_X1 = 0.08, 0.52
FIELD_Y0, FIELD_Y1 = 0.0, 0.48  # y0 = the field's right edge (the boundary between the two arms' zones)
FIELD = Box("field", pos=((FIELD_X0 + FIELD_X1) / 2, (FIELD_Y0 + FIELD_Y1) / 2, TOP + FIELD_H / 2),
            size=(FIELD_X1 - FIELD_X0, FIELD_Y1 - FIELD_Y0, FIELD_H), color=DRAPE_BLUE)

# -- hand-off axis: holder slot -> ledge -> jack, all on one line through E at heading -60 deg ---------------------------
AX_YAW = math.radians(-60.0)
U = np.array([math.cos(AX_YAW), math.sin(AX_YAW)])  # along the axis, off the field towards the console
NRM = np.array([-math.sin(AX_YAW), math.cos(AX_YAW)])  # sideways (left of the axis)
E = np.array([0.30, 0.0])  # where the axis crosses the field edge


def ax(d: float, b: float = 0.0) -> np.ndarray:
    """World xy of the point d m along the hand-off axis from the field edge (positive = off the field), b sideways."""
    return E + U * d + NRM * b


def ax_coords(xy) -> tuple[float, float]:
    v = np.asarray(xy[:2], dtype=float) - E
    return float(v @ U), float(v @ NRM)


def _abox(name, d0, d1, b, w, z0, z1, color):
    """Box aligned with the hand-off axis: from d0 to d1 along it, centred b sideways, w wide, z0..z1 high."""
    c = ax((d0 + d1) / 2, b)
    return Box(name, pos=(float(c[0]), float(c[1]), (z0 + z1) / 2), size=(d1 - d0, w, z1 - z0), yaw=AX_YAW,
               color=color)


# cord holder: open-top U-slot on the drape + flat ledge sticking out beyond the edge
SLOT_D0, SLOT_D1 = -0.125, -0.085  # slot extent along the axis (on the field)
SLOT_W, SLOT_WALL, SLOT_H = 0.014, 0.004, 0.030
LEDGE_D0, LEDGE_D1, LEDGE_W, LEDGE_T = -0.025, 0.085, 0.030, 0.010
HOLDER = [
    _abox("slot_wall_l", SLOT_D0, SLOT_D1, (SLOT_W + SLOT_WALL) / 2, SLOT_WALL, FT, FT + SLOT_H, HOLDER_GREY),
    _abox("slot_wall_r", SLOT_D0, SLOT_D1, -(SLOT_W + SLOT_WALL) / 2, SLOT_WALL, FT, FT + SLOT_H, HOLDER_GREY),
    _abox("ledge", LEDGE_D0, LEDGE_D1, 0.0, LEDGE_W, FT - LEDGE_T, FT, HOLDER_GREY),
]

# console with a jack facing the holder
PL, PW, PH = 0.080, 0.016, 0.014  # plug: length along the cord, width, height
CAV_W, CAV_H, CAV_D = PW + 0.004, PH + 0.004, 0.030
JACK_D = 0.16  # jack face, m along the axis from the field edge
SLEEVE_LEN, SLEEVE_T = 0.040, 0.004
JACK_ZC = FT + PH / 2 + 0.001  # cavity centre height (= plug centre carried 1 mm above the ledge)
CAV_Z0 = JACK_ZC - CAV_H / 2
CONSOLE_LEN, CONSOLE_W, CONSOLE_TOP = 0.12, 0.16, FT + 0.025
_SW = CAV_W + 2 * SLEEVE_T
CONSOLE = [
    _abox("console_body", JACK_D + SLEEVE_LEN, JACK_D + SLEEVE_LEN + CONSOLE_LEN, 0.0, CONSOLE_W, TOP, CONSOLE_TOP,
          CONSOLE_GREY),
    _abox("jack_base", JACK_D, JACK_D + SLEEVE_LEN, 0.0, _SW, TOP, CAV_Z0, CONSOLE_GREY),
    _abox("jack_wall_l", JACK_D, JACK_D + CAV_D, (CAV_W + SLEEVE_T) / 2, SLEEVE_T, CAV_Z0, CAV_Z0 + CAV_H, JACK_DARK),
    _abox("jack_wall_r", JACK_D, JACK_D + CAV_D, -(CAV_W + SLEEVE_T) / 2, SLEEVE_T, CAV_Z0, CAV_Z0 + CAV_H, JACK_DARK),
    _abox("jack_roof", JACK_D, JACK_D + CAV_D, 0.0, _SW, CAV_Z0 + CAV_H, CAV_Z0 + CAV_H + SLEEVE_T, JACK_DARK),
    _abox("jack_back", JACK_D + CAV_D, JACK_D + SLEEVE_LEN, 0.0, _SW, CAV_Z0, CAV_Z0 + CAV_H + SLEEVE_T, JACK_DARK),
]

# sterile device cord: device end fixed in a holster on the field, plug on the free end
R = 0.004  # 8 mm cord
CORD_LEN = 0.45
A = np.array([0.14, 0.06])  # cord leaves the holster here
CORD_YAW = math.radians(50.0)
_DIR = np.array([math.cos(CORD_YAW), math.sin(CORD_YAW)])
PLUG = Plug(size=(PL, PW, PH), mass=0.018, color=PLUG_BLACK)
CORD = cable("cord", start=(float(A[0]), float(A[1]), FT + PH / 2 + 0.0005), yaw=CORD_YAW, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=PLUG, color=WHITE, cable_mass=0.06, bend_stiffness=0.005, bend_damping=0.001)
HOLSTER_SIZE = (0.050, 0.035, 0.030)
_hc = A - _DIR * (0.006 + HOLSTER_SIZE[0] / 2)
HOLSTER = Box("holster", pos=(float(_hc[0]), float(_hc[1]), FT + HOLSTER_SIZE[2] / 2), size=HOLSTER_SIZE,
              yaw=CORD_YAW, color=WHITE)

OBJECTS = [FIELD, *HOLDER, *CONSOLE, HOLSTER, CORD]
TWIN_EXTRA: list = []  # everything is in OBJECTS (the cord's device end is glued into the holster)

# -- per-step monitor: arm zones and plug-on-table (latched until reset) -----------------------------------------------
TABLE_TOUCH = 0.002  # plug's lowest point within 2 mm of the table top = touching it
TAKE_DIST = 0.03  # right gripper centre this close to the plug centre = taking it
_ENV: dict = {"env": None}
_MON: dict = {}


def _reset_monitor() -> None:
    _MON.update(left_out=False, right_in=False, plug_table=False, taken_on_field=False, right_reached=False,
                err=False, events=[], steps=0, min_left_y=9.0, max_right_y=-9.0, min_plug_low=9.0)


_reset_monitor()


def grippers() -> list[np.ndarray]:
    """World TCPs (between the finger pads, 13 mm above the pad tips) of the left and right gripper; [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = np.asarray(getattr(base.scene.env_origins, "torch", base.scene.env_origins)[0].detach().cpu().numpy(),
                        dtype=np.float64)
    out = [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]
    return out


def plug_pose(objects) -> tuple[np.ndarray, np.ndarray]:
    """(plug centre, unit axis from the cord through the plug tip)."""
    c = objects[CORD.name]
    p = np.asarray(c.end_plug, dtype=float)
    a = p - np.asarray(c.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def plug_low(objects) -> float:
    """Height of the plug's lowest point (m, world z), allowing for its tilt."""
    p, a = plug_pose(objects)
    az = abs(float(a[2]))
    return float(p[2] - az * PL / 2 - math.hypot(PW / 2, PH / 2) * math.sqrt(max(0.0, 1 - az * az)))


def _monitor(objects) -> None:
    g = grippers()
    if len(g) == 2:
        _MON["min_left_y"] = min(_MON["min_left_y"], float(g[0][1]))
        _MON["max_right_y"] = max(_MON["max_right_y"], float(g[1][1]))
        if g[0][1] < FIELD_Y0 and not _MON["left_out"]:
            _MON["left_out"] = True
            _MON["events"].append(("left gripper left the field", _MON["steps"]))
        if g[1][1] > FIELD_Y0 and not _MON["right_in"]:
            _MON["right_in"] = True
            _MON["events"].append(("right gripper over the field", _MON["steps"]))
        if not _MON["right_reached"] and np.linalg.norm(g[1] - plug_pose(objects)[0]) <= TAKE_DIST:
            _MON["right_reached"] = True  # first time the right gripper is at the plug: it must be off the field
            if plug_pose(objects)[0][1] >= FIELD_Y0:
                _MON["taken_on_field"] = True
                _MON["events"].append(("right gripper took the plug over the field", _MON["steps"]))
    low = plug_low(objects)
    _MON["min_plug_low"] = min(_MON["min_plug_low"], low)
    if low <= TOP + TABLE_TOUCH and not _MON["plug_table"]:
        _MON["plug_table"] = True
        _MON["events"].append(("plug touched the table", _MON["steps"]))
    _MON["steps"] += 1


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_sterile_hooked", False):
        return
    base_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        try:
            from yam_arena.scene import object_states

            _monitor(object_states(self._env, [CORD.name]))
        except Exception as e:  # pragma: no cover
            _MON["err"] = True  # a step the rules could not be checked on never passes
            print(f"[sterile] monitor error: {e!r}", flush=True)
        return base_step(self, *a, **k)

    def reset(self, *a, **k):
        out = base_reset(self, *a, **k)
        _reset_monitor()  # arms start at READY (left over the field, right off it), plug on the drape
        return out

    _task_step._sterile_hooked = True
    reset._sterile_hooked = True
    cls._task_step = _task_step
    cls.reset = reset


_install_hooks()

# -- scoring ----------------------------------------------------------------------------------------------------------
SEATED_MIN = 0.015  # plug tip at least 15 mm inside the jack
RELEASE_PLUG = 0.045  # let go = both gripper TCPs at least this far from the plug's centre line, end to end,
RELEASE_CORD = 0.03  # and at least this far from every point of the cord


def violations() -> list[str]:
    out = []
    if _MON["err"]:
        out.append("monitor_error")
    if _MON["left_out"]:
        out.append("left_out")
    if _MON["right_in"]:
        out.append("right_in")
    if _MON["plug_table"]:
        out.append("plug_table")
    if _MON["taken_on_field"]:
        out.append("taken_on_field")
    return out


def jack_depth(objects) -> float:
    """How far (m) the plug tip is inside the jack; -1 if the plug is not lined up with the opening (centre > 8 mm
    off the jack axis sideways or vertically, or pointing more than ~25 deg away from it)."""
    p, a = plug_pose(objects)
    if not np.isfinite(p).all():
        return -1.0
    d, b = ax_coords(p)
    if abs(b) > 0.008 or abs(p[2] - JACK_ZC) > 0.008 or float(a[:2] @ U) < 0.9:
        return -1.0
    return float(d + PL / 2 - JACK_D)


def cord_in_slot(objects) -> bool:
    """Some cord segment centre lies inside the holder slot (between its walls, below their tops)."""
    for q in objects[CORD.name].points:
        d, b = ax_coords(q)
        if SLOT_D0 - 0.002 <= d <= SLOT_D1 + 0.002 and abs(b) <= SLOT_W / 2 and FT <= q[2] <= FT + SLOT_H:
            return True
    return False


def plug_on_ledge(objects) -> bool:
    """Plug lying on the ledge with its centre beyond the field edge."""
    p, _ = plug_pose(objects)
    d, b = ax_coords(p)
    return bool(p[1] < FIELD_Y0 and LEDGE_D0 <= d <= LEDGE_D1 and abs(b) <= LEDGE_W / 2
                and p[2] <= FT + PH / 2 + 0.004)


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def released(objects) -> bool:
    """Every gripper centre at least 4.5 cm from the plug's centre line and 3 cm from the cord."""
    p, a = plug_pose(objects)
    rear, tip = p - a * PL / 2, p + a * PL / 2
    poly = np.asarray(objects[CORD.name].polyline(), dtype=float)
    for g in grippers():
        if _seg_dist(g, rear, tip) < RELEASE_PLUG:
            return False
        if min(_seg_dist(g, u, v) for u, v in zip(poly[:-1], poly[1:])) < RELEASE_CORD:
            return False
    return True


def score(objects) -> float:
    """0 if, at any step, the left gripper crossed to the right of the field edge (y < 0), the right gripper crossed
    over it (y > 0) or the plug touched the table, or if the plug centre was still over the field (y >= 0) when the
    right gripper first came within 3 cm of it; also 0 if at the end the cord is not in the holder slot or a
    gripper is within 45 mm of the plug's centre line or 3 cm of the cord. These "ever" rules are latched in _MON and
    only reset() clears them. Otherwise 1.0 if the plug tip is >= 15 mm inside the jack (lined up),
    0.3 if the plug lies on the ledge off the field, else 0."""
    if violations() or not cord_in_slot(objects) or not released(objects):
        return 0.0
    if jack_depth(objects) >= SEATED_MIN:
        return 1.0
    return 0.3 if plug_on_ledge(objects) else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[CORD.name].resting(max_speed=0.02)


def distance(objects) -> float:
    """Plug tip's remaining distance to 15 mm inside the jack (m)."""
    p, a = plug_pose(objects)
    tip = p[:2] + a[:2] * PL / 2
    goal = ax(JACK_D + SEATED_MIN)
    return float(np.linalg.norm(tip - goal))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    p, _ = plug_pose(objects)
    return {
        "cord_in_slot": int(cord_in_slot(objects)),
        "plug_off_field": int(p[1] < FIELD_Y0),
        "plug_in_jack": round(float(np.clip(jack_depth(objects) / SEATED_MIN, 0.0, 1.0)), 3),
        "released": int(released(objects)),
        "zones_kept": int(not (_MON["left_out"] or _MON["right_in"] or _MON["taken_on_field"])),
        "plug_never_on_table": int(not _MON["plug_table"]),
    }


VARIATION = {
    "cable:cord": {"dx": 0.015, "dy": 0.015, "yaw": 0.10},
}

# -- closed-loop scripted demo (cabledemo.py) -------------------------------------------------------------------------
SCRUB_GRIP_BACK = 0.010  # left arm pinches the plug this far from its cord end
CIRC_GRIP_BACK = 0.040  # right arm pinches it this far from its cord end (40 mm from the tip)
PLACE_REAR_D = -0.030  # plug's cord end placed this far along the axis (on the field side of the edge)
SEAT_DEPTH = 0.024


def _hd(v) -> float:
    return math.atan2(float(v[1]), float(v[0]))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def demo_script(ctx):
    """Left (scrub) arm pinches the plug near its cord end, turns it to point along the hand-off axis, brings it in
    from behind the holder so the cord trailing behind it drops into the slot, sets it on the ledge (plug tip off the
    field, gripper still over the field) and moves back over the field. Right (circulating) arm pinches the plug on
    the ledge 40 mm from its cord end (wrist camera towards the jack), slides it along the axis into the jack
    (closed loop on the live plug pose), lets go and backs off."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("STERILE_CHEAT", "")

    def log(tag):
        st = ctx.states()
        p, a = plug_pose(st)
        g = [np.round(x, 3).tolist() for x in grippers()]
        print(f"[demo t={ctx.t:5.1f}] {tag}: plug={np.round(p, 3).tolist()} hd={math.degrees(_hd(a)):.0f} az={a[2]:.2f} "
              f"ax={np.round(ax_coords(p), 3).tolist()} depth={jack_depth(st):.3f} slot={cord_in_slot(st)} "
              f"ledge={plug_on_ledge(st)} rel={released(st)} viol={violations()} low={plug_low(st) - TOP:.3f} "
              f"minLy={_MON['min_left_y']:.3f} maxRy={_MON['max_right_y']:.3f} grip={g} score={score(st):.2f}",
              flush=True)
        if os.environ.get("STERILE_DEBUG"):
            near = [(round(d, 3), round(b, 3), round(float(q[2] - FT), 3)) for q in st[CORD.name].points
                    for d, b in [ax_coords(q)] if -0.17 < d < -0.04 and abs(b) < 0.06]
            print(f"    cord near slot (d, b, z-FT): {near}", flush=True)

    log("start")
    if cheat == "nothing":
        yield from L.wait(5.0)
        log("end")
        return

    hi = FT + 0.06
    # 1) scrub arm takes the plug where it lies
    st = ctx.states()
    c = st[CORD.name]
    p, a = plug_pose(st)
    h0 = _hd(a)
    gy = _wrap(h0)
    if abs(gy) > math.pi / 2:
        gy = _wrap(gy + math.pi)
    g = c.end_tip + a * SCRUB_GRIP_BACK
    yield from L.move((g[0], g[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((g[0], g[1], p[2] + 0.010), yaw=gy, duration=1.0)
    yield from L.grip(FINGER_CLOSED)
    yield from L.move((g[0], g[1], hi), yaw=gy, duration=0.8)
    log("scrub has plug")

    # 2) turn the plug to point along the axis and come in from behind the holder
    tcp_d = PLACE_REAR_D + SCRUB_GRIP_BACK  # TCP position along the axis when the plug is set down
    if cheat == "scrub_carry":
        tcp_d = 0.045  # carries the plug out past the field edge itself
    if cheat == "noclip":  # comes in sideways from the left of the axis: the trailing cord misses the slot
        back, pre = ax(tcp_d - 0.02, 0.16), ax(tcp_d - 0.02, 0.02)
    else:  # comes in along the axis from behind the holder: the trailing cord lies over the slot
        back, pre = ax(tcp_d - 0.20), ax(tcp_d - 0.02)
    yield from L.move((back[0], back[1], hi), yaw=AX_YAW, duration=2.0)
    yield from L.move((pre[0], pre[1], FT + 0.035), yaw=AX_YAW, duration=1.5)
    # closed loop: put the plug's cord end at PLACE_REAR_D on the axis
    tgt_rear = ax(PLACE_REAR_D if cheat != "scrub_carry" else tcp_d - SCRUB_GRIP_BACK)
    for k in range(2):  # measure where the plug's cord end is, correct, measure again
        st = ctx.states()
        p, a = plug_pose(st)
        rear = p - a * PL / 2
        cur = L.tcp()
        tgt = cur[:2] + (tgt_rear - rear[:2])
        if cheat != "scrub_carry":
            tgt[1] = max(tgt[1], FIELD_Y0 + 0.010)  # never let the scrub gripper reach past the field edge
        if os.environ.get("STERILE_DEBUG"):
            print(f"    place {k}: rear={np.round(rear, 3).tolist()} cmd_tcp={np.round(cur, 3).tolist()} "
                  f"tgt={np.round(tgt, 3).tolist()} tgt_rear={np.round(tgt_rear, 3).tolist()}", flush=True)
        yield from L.move((tgt[0], tgt[1], FT + 0.030), yaw=AX_YAW, duration=1.0 if k == 0 else 0.6)
    yield from L.move((tgt[0], tgt[1], FT + PH / 2 + 0.011), yaw=AX_YAW, duration=1.0)
    yield from L.wait(0.3)
    log("plug set on ledge")
    yield from L.grip(FINGER_OPEN, 0.5)
    up = tgt - U * 0.02
    yield from L.move((up[0], up[1], hi), yaw=AX_YAW, duration=0.8)
    yield from L.move((0.20, 0.30, hi + 0.05), yaw=0.0, duration=1.2)
    log("scrub clear")
    if cheat == "tier03":  # left arm's half only: the plug stays on the ledge, both arms clear
        yield from L.wait(3.0)
        log("end")
        return

    # 3) circulating arm takes the plug from the ledge (or wherever it is)
    st = ctx.states()
    c = st[CORD.name]
    p, a = plug_pose(st)
    ry = _wrap(_hd(a))
    if abs(ry) > math.pi / 2:
        ry = _wrap(ry + math.pi)
    back_off = CIRC_GRIP_BACK if cheat != "reach_over" else 0.004
    g = c.end_tip + a * back_off
    yield from Rt.move((g[0], g[1], hi), yaw=ry, finger=FINGER_OPEN, duration=1.5)
    st = ctx.states()
    c = st[CORD.name]
    p, a = plug_pose(st)
    g = c.end_tip + a * back_off
    yield from Rt.move((g[0], g[1], p[2] + 0.010), yaw=ry, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)
    yield from Rt.move((g[0], g[1], p[2] + 0.012), yaw=ry, duration=0.5)
    log("circulator has plug")

    # 4) carry along the axis to the jack, closed loop on the plug pose
    def plug_to(d_center: float, dur: float, z_extra: float = 0.0):
        st = ctx.states()
        p, a = plug_pose(st)
        cur = Rt.tcp()
        off = cur - p  # TCP relative to the plug centre (kept by the grasp)
        err = _wrap(_hd(a) - AX_YAW)
        yaw = Rt._yaw - err
        tgt_xy = ax(d_center) + off[:2]
        tz = JACK_ZC + off[2] + z_extra
        yield from Rt.move((tgt_xy[0], tgt_xy[1], tz), yaw=yaw, duration=dur)

    if cheat == "drop":
        # carry the plug off the ledge, let it fall onto the table, pick it up again from there
        yield from plug_to(0.06, 1.0, 0.003)
        cur = Rt.tcp()
        side = cur[:2] - NRM * 0.07  # beside the ledge, over the bare table
        yield from Rt.move((side[0], side[1], cur[2]), yaw=Rt._yaw, duration=1.0)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        cur = Rt.tcp()
        yield from Rt.move((cur[0], cur[1], hi), yaw=Rt._yaw, duration=0.8)
        yield from Rt.wait(0.8)
        log("dropped on table")
        st = ctx.states()
        c = st[CORD.name]
        p, a = plug_pose(st)
        ry = _wrap(_hd(a))
        if abs(ry) > math.pi / 2:
            ry = _wrap(ry + math.pi)
        g = c.end_tip + a * CIRC_GRIP_BACK
        yield from Rt.move((g[0], g[1], hi), yaw=ry, duration=1.0)
        yield from Rt.move((g[0], g[1], p[2] + 0.010), yaw=ry, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED)
        yield from Rt.move((g[0], g[1], JACK_ZC + 0.02), yaw=ry, duration=0.8)
        log("picked up again")
    yield from plug_to(JACK_D - 0.06 - PL / 2, 2.0, 0.003)
    yield from plug_to(JACK_D - 0.012 - PL / 2, 1.2, 0.001)
    log("at jack mouth")
    yield from plug_to(JACK_D - 0.012 - PL / 2, 0.6, 0.0)
    yield from plug_to(JACK_D + SEAT_DEPTH - PL / 2, 1.5, 0.0)
    yield from Rt.wait(0.3)
    log("seated (held)")
    if cheat == "hold":
        yield from Rt.wait(3.0)
        log("end")
        return
    if cheat == "hold_cord":  # lets go of the plug but slides back and closes on the cord 5 cm behind it
        yield from Rt.grip(FINGER_OPEN, 0.5)
        cur = Rt.tcp()
        yield from Rt.move((cur[0], cur[1], JACK_ZC + 0.016), yaw=Rt._yaw, duration=0.4)
        st = ctx.states()
        p, a = plug_pose(st)
        q = p - a * (PL / 2 + 0.05)
        yield from Rt.move((q[0], q[1], JACK_ZC + 0.016), yaw=Rt._yaw, duration=1.2)
        yield from Rt.move((q[0], q[1], JACK_ZC + 0.010), yaw=Rt._yaw, duration=0.4)
        yield from Rt.grip(FINGER_CLOSED)
        log("holding the cord")
        yield from Rt.wait(3.0)
        log("end")
        return
    yield from Rt.grip(FINGER_OPEN, 0.5)
    cur = Rt.tcp()
    bk = cur[:2] - U * 0.03
    yield from Rt.move((bk[0], bk[1], cur[2] + 0.01), yaw=Rt._yaw, duration=0.8)
    yield from Rt.move((0.20, -0.30, hi + 0.05), yaw=0.0, duration=1.2)
    yield from Rt.wait(1.5)
    log("end")
