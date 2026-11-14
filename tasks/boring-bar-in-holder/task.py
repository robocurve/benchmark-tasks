"""On the station bench, real size (nominal sizes, see layout.py): a bench lathe stand-in. A steel holder block (the tool
post, fixed) carries a 12.4 mm through-bore along the lathe's axis (along y, 45 mm over the bench) and two set screws
on its top face; 240 mm toward the chuck stands a steel workpiece with a real 22 mm blind bore 40 mm deep, in a chuck.
A 12 mm round boring bar (150 mm, 130 g, a flat along its shank, a gold cutting insert at one end) lies on a two-blade
rack in front of the holder, its plain end 15 mm from the holder's face; a 3 mm hex T-handle key (60 g) stands in a block
to the right. Goal: mount the bar in the holder for boring the workpiece's hole to full depth.

The bar and the key are loose mechkit assemblies with real collision: the bar slides in the holder's bore. The set
screws are a rule (setscrew.py, run by ``_screw_rule``, a mechkit on_step callback): the key's tip in a socket, pushed
down and upright, turns the screw with the key at one pitch (1.0 mm) a turn; a screw is tight 0.4 turn past contact with
the bar; the first tight screw fires a mechkit weld of the bar to the holder (released when no screw is tight).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score -E object_jitter=0 ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
Settle:  DEMO_HOLD_ONLY=1.  One arm: DEMO_IDLE_ARM=left (or right).  Counterfactuals (never the submitted demo):
DEMO_VARIANT=<name>, see VARIANTS.  Trace: TASK_TRACE=<file>.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402

import layout as L  # noqa: E402
import mechkit as mk  # noqa: E402  (importing it hooks yam_arena so the assemblies are built as articulations)
import rigstate  # noqa: E402
import setscrew  # noqa: E402
from offline_boring import OfflineBoring  # noqa: E402
from scoring import BAR, HOLDER, KEY, Tracker  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

rigstate.install()

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05  # one control step: the scorer's own 20-step window is the 1 s

INSTRUCTION = "Mount the boring bar in its holder, ready to bore this hole to full depth."


# -- shapes ---------------------------------------------------------------------------------------------------------
def _shape(t, origin=(0.0, 0.0, 0.0)):
    ox, oy, oz = origin
    if t[0] == "box":
        _, c, s, ypr, col = t
        turn = {} if all(abs(v) < 1e-12 for v in ypr) else {"ypr": tuple(float(v) for v in ypr)}
        return mk.box(L.m((c[0] - ox, c[1] - oy, c[2] - oz)), L.m(s), col, **turn)
    _, c, r, h, axis, col = t
    return mk.cylinder(L.m((c[0] - ox, c[1] - oy, c[2] - oz)), r * L.MM, h * L.MM, axis, col)


def _shapes(rows, origin=(0.0, 0.0, 0.0)):
    return tuple(_shape(t, origin) for t in rows)


def _plates(layers, holes_xy, x_c, y0, y1, outer_x, origin):
    """Top-slab layers with square holes at ``holes_xy`` (absolute x, y mm), as mechkit plates about ``origin``."""
    out = []
    yc, yl = (y0 + y1) / 2, y1 - y0
    for z0, z1, size in layers:
        holes = [((hx - x_c) * L.MM, (hy - yc) * L.MM, size * L.MM, size * L.MM) for hx, hy in holes_xy]
        out += mk.plate_with_holes(((x_c - origin[0]) * L.MM, (yc - origin[1]) * L.MM, (z0 + z1) / 2 * L.MM),
                                   (outer_x * L.MM, yl * L.MM), holes, (z1 - z0) * L.MM, L.STEEL)
    return out


LATHE_ORIGIN = (L.X_AX, 0.0, 0.0)
_HOLDER_ROWS, _ = L.holder_rows()
HOLDER_SHAPES = list(_shapes(_HOLDER_ROWS, LATHE_ORIGIN)) + _plates(
    L.SOCKET_STEPS, [(L.X_AX, y) for y in L.SCREW_Y], L.X_AX, L.Y_BACK, L.Y_FACE, L.HOLD_W, LATHE_ORIGIN)
HOLDER_SPEC = mk.assembly(HOLDER, origin=L.world((L.X_AX, 0.0, 0.0)), shapes=HOLDER_SHAPES, color=L.STEEL,
                          tight_fit=True)
CHUCK_SPEC = mk.assembly("chuck", origin=L.world((L.X_AX, 0.0, 0.0)), shapes=_shapes(L.workpiece_rows(), LATHE_ORIGIN),
                         color=L.GREY)
RACK_SPEC = mk.assembly("rack", origin=L.world((L.X_AX, 0.0, 0.0)), shapes=_shapes(L.rack_rows(), LATHE_ORIGIN),
                        color=L.WOOD, tight_fit=True)
STAND_ORIGIN = (L.KEY_XY[0], L.KEY_XY[1], 0.0)
_STAND_SHAPES = list(_shapes(L.stand_rows()))  # stand_rows is already in the stand's own frame (no origin shift)
for _z0, _z1, _s in ((L.STAND_FLOOR, L.STAND_TOP, L.STAND_HOLE),) + tuple(L.STAND_MOUTH):
    _STAND_SHAPES += mk.plate_with_holes((0.0, 0.0, (_z0 + _z1) / 2 * L.MM), (L.STAND_SIZE * L.MM, L.STAND_SIZE * L.MM),
                                         [(0.0, 0.0, _s * L.MM, _s * L.MM)], (_z1 - _z0) * L.MM, L.WOOD)
KEYSTAND_SPEC = mk.assembly("keystand", origin=L.world((L.KEY_XY[0], L.KEY_XY[1], 0.0)), shapes=_STAND_SHAPES,
                            color=L.WOOD, tight_fit=True)
BAR_SPEC = mk.assembly(BAR, origin=L.world((L.X_AX, L.BAR_START_Y, L.Z_AX + 0.1)), yaw=L.BAR_YAW, mass=L.BAR_MASS,
                       shapes=_shapes(L.bar_rows()), color=L.STEEL, friction=L.BAR_FRICTION, tight_fit=True)
KEY_SPEC = mk.assembly(KEY, origin=L.world((L.KEY_XY[0], L.KEY_XY[1], L.STAND_FLOOR + 0.1)), mass=L.KEY_MASS,
                       shapes=_shapes(L.key_rows()), color=L.STEEL, friction=L.KEY_FRICTION, tight_fit=True)
OBJECTS = [HOLDER_SPEC, CHUCK_SPEC, RACK_SPEC, KEYSTAND_SPEC, BAR_SPEC, KEY_SPEC]

WELD = mk.weld(BAR_SPEC, HOLDER_SPEC, name="bar_to_holder")
MODEL = setscrew.ScrewModel()


@mk.on_step
def _screw_rule(states) -> None:
    """In the simulator, at the start of every control step (before the action and the physics)."""
    bar, key = states.get(BAR), states.get(KEY)
    try:
        out = MODEL.update(key.origin, key.rotation(), bar.origin, bar.rotation())
    except (AttributeError, KeyError, TypeError, ValueError):
        out = MODEL.update(np.full(3, np.nan), np.eye(3), np.full(3, np.nan), np.eye(3))
    if out.fire:
        WELD.fire(states)
        print(f"[boring] CLAMP: weld on at episode {mk.episode()} step {mk.step()}: {out.record}", file=sys.stderr,
              flush=True)
    if out.release:
        WELD.release()
        print(f"[boring] clamp released at step {mk.step()}", file=sys.stderr, flush=True)


@mk.on_reset
def _screw_reset() -> None:
    MODEL.reset()
    OFFLINE.reset()


_TRACKER = Tracker()
_EPISODE = {"mechkit": None}


def _shadow(side: str, t: float):
    """Offline only: the TCP rotation of the demo plan for ``side`` at time ``t`` (rigstate carries no wrist angle)."""
    from yam_arena.demo import ArmPlan  # noqa: E402

    plan = _SHADOW.get(side)
    if plan is None:
        wps = plans(os.environ.get("DEMO_VARIANT", "")).get(side)
        if not wps:
            return None
        import yamkin as K

        plan = _SHADOW[side] = ArmPlan(wps, K.BASES["station"][side])
    return plan.target(t)[1]


_SHADOW: dict = {}
def _key_error(tcp):
    """Offline only: the GPU arm lands the key's tip off its command by minus the demo's trim at that screw."""
    y = float(tcp[1]) / L.MM
    for k, sy in enumerate(L.SCREW_Y):
        if abs(y - sy) < 6.0 and k in TRIM_KEY:
            return -TRIM_KEY[k][0] * L.MM, -TRIM_KEY[k][1] * L.MM
    return 0.0, 0.0


OFFLINE = OfflineBoring(BAR_SPEC, KEY_SPEC, _shadow, _key_error)


@rigstate.on_reset
def _new_episode() -> None:
    _TRACKER.reset()
    OFFLINE.reset()


def no_simulator(objects) -> bool:
    return mk.env() is None and getattr(objects.get(BAR), "source", "sim") != "sim"


def _trace(row: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if path:
        try:
            with open(path, "a") as f:
                f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
        except OSError:
            pass


def _pose_fields(objects, arms) -> dict:
    """Trace only (GPU tuning): the bar's plain and cutting ends over the lathe frame (mm), the bar axis's yaw and pitch
    (deg), its offset from the bore axis at the holder's face (mm), the key's tip and shaft tilt, both TCPs (mm over the
    lathe frame) and the gripper openings."""
    out = {}
    try:
        bar = objects[BAR]
        o = np.asarray(bar.origin, dtype=float)
        ax = np.asarray(bar.rotation(), dtype=float)[:, 0]
        ref = np.array([L.X_AX * L.MM, 0.0, L.TOP + L.Z_AX * L.MM])
        half = L.BAR_L / 2 * L.MM
        out["plain_mm"] = [round(float(v), 1) for v in (o - half * ax - ref) / L.MM]
        out["cut_mm"] = [round(float(v), 1) for v in (o + half * ax - ref) / L.MM]
        out["bar_yaw_deg"] = round(math.degrees(math.atan2(-ax[0], ax[1])), 2)
        out["bar_pitch_deg"] = round(math.degrees(math.asin(max(-1.0, min(1.0, ax[2])))), 2)
        if abs(ax[1]) > 1e-6:
            f = (L.Y_FACE * L.MM - o[1]) / ax[1]
            p = o + f * ax - ref
            out["face_err_mm"] = [round(float(p[0]) / L.MM, 2), round(float(p[2]) / L.MM, 2)]
    except Exception:  # noqa: BLE001 - never break scoring for a trace field
        pass
    try:
        key = objects[KEY]
        ko = np.asarray(key.origin, dtype=float)
        kz = np.asarray(key.rotation(), dtype=float)[:, 2]
        out["key_tip_mm"] = [round(float(v), 1) for v in (ko - np.array([L.X_AX * L.MM, 0.0, L.TOP])) / L.MM]
        out["key_tilt_deg"] = round(math.degrees(math.acos(max(-1.0, min(1.0, float(kz[2]))))), 1)
    except Exception:  # noqa: BLE001
        pass
    try:
        for side in ("left", "right"):
            a = arms[side]
            t = np.asarray(a.tcp, dtype=float)
            out[f"tcp_{side}"] = [round(float(t[0]) / L.MM - L.X_AX, 1), round(float(t[1]) / L.MM, 1),
                                  round((float(t[2]) - L.TOP) / L.MM, 1)]
            out[f"open_{side}"] = round(float(a.opening), 2)
    except Exception:  # noqa: BLE001
        pass
    return out


def success(objects) -> bool:
    """Runs every control step: feeds the tracker (and, without a simulator, the offline model that moves the parts)."""
    if _EPISODE["mechkit"] != mk.episode():
        if _EPISODE["mechkit"] is not None:
            _TRACKER.reset()
            OFFLINE.reset()
        _EPISODE["mechkit"] = mk.episode()
    arms = rigstate.arms()
    offline = no_simulator(objects)
    if offline:
        OFFLINE.step(arms, rigstate._STATE["steps"] * L.DT)
        objects = OFFLINE.states(objects, BAR, KEY)
        thread = OFFLINE.model.snapshot()
    else:
        thread = MODEL.snapshot()
    result = _TRACKER.update(objects, arms, thread)
    row = {"episode": rigstate.episode(), "step": _TRACKER.steps, "score": result.score, "success": result.success,
           **result.details}
    row.update(_pose_fields(objects, arms))
    if offline:
        row["offline"] = {"events": [list(map(str, e)) for e in OFFLINE.events[-4:]], "welded": OFFLINE.welded}
    _trace(row)
    return result.success


def score(objects) -> float:
    """The score of the latest scored step (see scoring.py)."""
    return _TRACKER.last.score


# -- the scripted demo ------------------------------------------------------------------------------------------------
# Open loop, millimetres over the bench for the lathe frame (x absolute, y, z over the bench). TCP points (the pad tips
# are 13 mm below). Left arm: closes its two pads into one block (wrist yaw 0: the pads close along y), lowers it behind
# the bar's cutting end and pushes the bar plain end first along the rack into the holder's bore until the cutting end is
# S mm past the face (the rack holds the bar's axis on the bore's axis, so a free bar enters the bore; a gripped bar
# was dragged 2 mm sideways by the pads on the first GPU run and jammed at the mouth), backs off and goes home. Right
# arm: takes the key by its crossbar and waits; once the left has gone home, carries it over screw 1, searches a small
# spiral for the socket, turns the wrist 150 deg clockwise, lifts out, turns back, drops in again (a ratchet), five
# times (the screw goes tight and the bar is welded); the same on screw 2; carries the key back to its stand, lets go
# and goes home. Finger targets: the joint value is minus half the commanded gap.
OPEN = rigstate.FINGER_OPEN
GRIP_PUSH = -0.0005  # 1 mm commanded: the two pads are one block 17 mm across (8 mm each and the 1 mm gap)
PAD_FACE = 4.0  # the block's face on the bar's end from the TCP, along y (GPU run 2: the end face stopped 4.1 to 4.3 mm from the TCP)
GRIP_KEY = -0.0015  # 3 mm commanded on the 8 mm crossbar
HOME = {"left": (0.28, 0.25, 0.12), "right": (0.28, -0.25, 0.12)}  # world TCP points
TIP = 13.0  # pad tips below the TCP
S_DEMO = 46.5  # the stick-out the demo aims at: 1.5 mm inside the band's lower end and 1.5 inside its upper end
S_LONG = 60.0
PUSH_TIP_Z = 44.0  # pad tips 1 mm under the bar's equator (GPU run 2: the TCP held its height to 0.3 mm; the rack blades top out at 39.8)
PUSH_X = 1.7  # mm, demo only: the left TCP landed 1.7 mm short of x 250 on GPU run 2
PUSH_Z = PUSH_TIP_Z + TIP  # TCP height while pushing
PUSH_YAW = 0.0
TAIL1 = 128.0  # mm: where stage 1 leaves the cutting end (the GPU stalled at 118.5)
GRIP_BAR = -0.002  # 4 mm commanded on the 12 mm shank
GRIP_BACK = 20.0  # the grip's distance from the cutting end
GRIP_Z = 44.0 + TIP  # TCP height for the grip: pad tips 1 mm under the equator
YAW_L = -math.pi / 2  # pads close along x, across the shank
PUSH_BEHIND = 9.0  # the block's face this far behind the cutting end before it goes down (the workpiece face is at 240)
TRIM_PUSH_Y = -L.GPU_PUSH_LAG  # mm, demo only: added to the pushing TCP's final y (set from the GPU trace's stick-out)
KEY_TCP_ABOVE_TIP = L.KEY_BAR_Z - 1.5 + TIP  # TCP height over the key's tip when the pads sit 1.5 mm under the crossbar
TIP_IN, TIP_OUT, TIP_CARRY = 60.0, 70.0, 82.0  # key tip heights: in the socket (10 mm), just out, carried
TIP_TOUCH, TIP_PRESS = 72.0, 62.0  # just over the top face; pressed 8 mm under it while searching
SEARCH_REVS, SEARCH_PTS, SEARCH_DT, SEARCH_R = 2, 8, 0.25, (0.5, 3.5)  # mm: the socket search spiral
KEY_WAIT = (60.0, 60.0)  # mm: the right arm waits with the key this far back (-x, -y) from its stand while the left works
TRIM_KEY = {0: (-0.5, 1.8), 1: (0.0, 0.0)}  # mm (x, y) per screw, demo only (GPU run 2: the tip sat 1.4 to 2.6 mm low in y over screw 1)
STROKE_DEG = 150.0
YAW_A = math.radians(130.0)  # wrist yaw at the start of a stroke; a stroke turns it clockwise (the yaw falling)
STROKES = 6
STROKE_S, LIFT_S, BACK_S, LOWER_S = 1.8, 0.5, 1.2, 0.6


def W(x, y, z):
    return (x * L.MM, y * L.MM, L.TOP + z * L.MM)


def _home(side):
    return (HOME[side][0], HOME[side][1], L.TOP + HOME[side][2])


class _Plan:
    def __init__(self, wps=None):
        self.wps = list(wps or [])
        self.t = 0.0

    def add(self, d, pos, finger, label, yaw=0.0):
        self.wps.append(Waypoint(d, pos, finger, label, yaw))
        self.t += d
        return self.t

    @property
    def last(self):
        return self.wps[-1]

    def hold_until(self, t, label="hold"):
        if t > self.t + 1e-9:
            w = self.last
            self.add(t - self.t, w.pos, w.finger, label, w.yaw)


def key_tcp(x, y, tip_z):
    tx, ty = TRIM_KEY.get(0 if abs(y - L.SCREW_Y[0]) < 6.0 else (1 if abs(y - L.SCREW_Y[1]) < 6.0 else -1), (0.0, 0.0))
    return W(x + tx, y + ty, tip_z + KEY_TCP_ABOVE_TIP)


def _socket_entry(p: _Plan, screw_y: float, label: str) -> None:
    """From the carry height over a socket: lower to just over its mouth, press 8 mm under the top face (the tip drops in
    if it is over the hole), then a small spiral so a tip a millimetre or two off finds the 5.2 mm hole, back to the
    centre, to the working depth."""
    x = L.X_AX
    p.add(1.0, key_tcp(x, screw_y, TIP_TOUCH), GRIP_KEY, f"{label}: lower to the socket's mouth", YAW_A)
    p.add(0.8, key_tcp(x, screw_y, TIP_PRESS), GRIP_KEY, f"{label}: press the tip into the socket", YAW_A)
    n = SEARCH_REVS * SEARCH_PTS
    for k in range(1, n + 1):
        r = SEARCH_R[0] + (SEARCH_R[1] - SEARCH_R[0]) * (k - 1) / (n - 1)
        a = 2 * math.pi * k / SEARCH_PTS
        p.add(SEARCH_DT, key_tcp(x + r * math.cos(a), screw_y + r * math.sin(a), TIP_PRESS), GRIP_KEY,
              f"{label}: search spiral", YAW_A)
    p.add(0.4, key_tcp(x, screw_y, TIP_PRESS), GRIP_KEY, f"{label}: back to the socket's axis", YAW_A)
    p.add(0.6, key_tcp(x, screw_y, TIP_IN), GRIP_KEY, f"{label}: to the working depth", YAW_A)


def _stroke_cycles(p: _Plan, screw_y: float, n: int, label: str) -> float:
    """``n`` ratchet cycles at screw_y with the key already in the socket at yaw YAW_A; returns the time the last
    stroke ends."""
    a, b = YAW_A, YAW_A - math.radians(STROKE_DEG)
    t_end = p.t
    for k in range(n):
        p.add(STROKE_S, key_tcp(L.X_AX, screw_y, TIP_IN), GRIP_KEY, f"{label}: turn the key clockwise {STROKE_DEG:.0f} deg", b)
        t_end = p.t
        p.add(LIFT_S, key_tcp(L.X_AX, screw_y, TIP_OUT), GRIP_KEY, f"{label}: lift the key out", b)
        if k < n - 1:
            p.add(BACK_S, key_tcp(L.X_AX, screw_y, TIP_OUT), GRIP_KEY, f"{label}: turn back", a)
            p.add(LOWER_S, key_tcp(L.X_AX, screw_y, TIP_IN), GRIP_KEY, f"{label}: drop the tip in the socket", a)
    return t_end


def plans(variant: str = "") -> dict:
    s = S_LONG if variant == "too_long" else S_DEMO
    two_screws = variant not in ("one_screw", "smoke")
    tail0 = L.BAR_START_Y + L.BAR_L / 2  # the cutting end at spawn
    y_touch = tail0 + PAD_FACE  # TCP y with the block's face on the cutting end
    y_pre = y_touch + PUSH_BEHIND
    y_end = L.Y_FACE + s + PAD_FACE + TRIM_PUSH_Y  # the pad block pushes the cutting end to the stick-out
    px = L.X_AX + PUSH_X
    # ---- the left arm's front half: close the pads, go behind the bar, push it in, then grip it and finish ---------
    lp = _Plan()
    lp.add(1.5, _home("left"), OPEN, "wait", PUSH_YAW)
    lp.add(2.5, W(px, y_pre, PUSH_Z + 60), OPEN, "over the bar's cutting end, behind it, open", PUSH_YAW)
    lp.add(1.0, lp.last.pos, GRIP_PUSH, "close the pads into one block (commanded 1 mm)", PUSH_YAW)
    lp.add(1.5, W(px, y_pre, PUSH_Z), GRIP_PUSH, "down behind the cutting end", PUSH_YAW)
    lp.add(0.5, W(px, y_pre, PUSH_Z), GRIP_PUSH, "settle", PUSH_YAW)
    lp.add(1.0, W(px, y_touch, PUSH_Z), GRIP_PUSH, "up to the cutting end's face", PUSH_YAW)
    lp.add(7.0, W(px, y_end, PUSH_Z), GRIP_PUSH, f"push the bar along the rack into the bore to a {s:.1f} mm stick-out",
           PUSH_YAW)
    lp.add(0.5, lp.last.pos, GRIP_PUSH, "settle", PUSH_YAW)
    # one push all the way (GPU run 5: with the insert at the cutting end the block pushes freely; the grip-and-carry
    # of runs 4 and 5 let the bar slip 10 mm short)
    lp.add(1.0, W(px, y_end + 15.0, PUSH_Z), GRIP_PUSH, "back off from the bar", PUSH_YAW)
    lp.add(1.2, W(px, y_end + 15.0, PUSH_Z + 70), OPEN, "up", PUSH_YAW)
    t_clear = lp.add(2.5, _home("left"), OPEN, "home", PUSH_YAW)
    # ---- the right arm: the key -----------------------------------------------------------------------------------
    rp = _Plan()
    kx, ky = L.KEY_XY
    rp.add(3.0, key_tcp(kx, ky, L.STAND_FLOOR + 60.0), OPEN, "over the key's crossbar, open", 0.0)
    rp.add(1.5, key_tcp(kx, ky, L.STAND_FLOOR), OPEN, "down beside the crossbar", 0.0)
    rp.add(0.5, key_tcp(kx, ky, L.STAND_FLOOR), OPEN, "settle", 0.0)
    rp.add(1.5, key_tcp(kx, ky, L.STAND_FLOOR), GRIP_KEY, "close on the crossbar (commanded 3 mm)", 0.0)
    rp.add(0.5, key_tcp(kx, ky, L.STAND_FLOOR), GRIP_KEY, "settle the grip", 0.0)
    rp.add(1.2, key_tcp(kx, ky, 49.0), GRIP_KEY, "lift the key out of its hole", 0.0)
    rp.add(2.0, key_tcp(kx - KEY_WAIT[0], ky - KEY_WAIT[1], 60.0), GRIP_KEY, "draw the key back out of the left arm's way", 0.0)
    rp.hold_until(t_clear, "hold the key up while the left arm pushes the bar in and backs off")
    rp.add(3.0, key_tcp(L.X_AX, L.SCREW_Y[0], TIP_CARRY), GRIP_KEY, "carry it over screw 1", YAW_A)
    _socket_entry(rp, L.SCREW_Y[0], "screw 1")
    _stroke_cycles(rp, L.SCREW_Y[0], STROKES, "screw 1")
    if two_screws:
        rp.add(0.6, key_tcp(L.X_AX, L.SCREW_Y[0], TIP_CARRY), GRIP_KEY, "key up", YAW_A)
        rp.add(1.5, key_tcp(L.X_AX, L.SCREW_Y[1], TIP_CARRY), GRIP_KEY, "over screw 2", YAW_A)
        _socket_entry(rp, L.SCREW_Y[1], "screw 2")
        _stroke_cycles(rp, L.SCREW_Y[1], STROKES, "screw 2")
        last_y = L.SCREW_Y[1]
    else:
        last_y = L.SCREW_Y[0]
    if variant == "smoke":
        rp.add(1.5, key_tcp(L.X_AX, last_y, TIP_CARRY), GRIP_KEY, "lift the key clear and hold", YAW_A)
    else:
        rp.add(1.0, key_tcp(L.X_AX, last_y, TIP_CARRY), GRIP_KEY, "lift the key clear", YAW_A)
        rp.add(4.0, key_tcp(kx, ky, 45.0), GRIP_KEY, "carry the key back over its stand", 0.0)
        rp.add(2.5, key_tcp(kx, ky, L.STAND_FLOOR + 1.0), GRIP_KEY, "lower it into its hole", 0.0)
        rp.add(0.5, key_tcp(kx, ky, L.STAND_FLOOR + 1.0), GRIP_KEY, "settle", 0.0)
        rp.add(1.5, key_tcp(kx, ky, L.STAND_FLOOR + 1.0), OPEN, "let go (open)", 0.0)
        rp.add(1.5, key_tcp(kx, ky, 60.0), OPEN, "straight up", 0.0)
        rp.add(2.5, _home("right"), OPEN, "home", 0.0)
    lp.hold_until(rp.t, "wait")
    return {"left": lp.wps, "right": rp.wps}


VARIANTS = ("too_long", "flat_down", "one_screw", "smoke")


def demo():
    """Both arms. DEMO_HOLD_ONLY=1: no plan. DEMO_IDLE_ARM=left|right: that arm's plan is dropped.
    DEMO_VARIANT (counterfactual runs for the README, never the submitted demo):
      too_long   the bar is pushed to a 60 mm stick-out (past the 4 x 12 mm rule): 0.5
      flat_down  the bar lies on the rack with its flat down (and its insert facing away from the operator side), the
                 rest as the demo: clamped on the round: 0.5
      one_screw  the demo without screw 2: the bar is clamped by one screw only: 0.25
      smoke      the first GPU job: the bar pushed in at 46.5 mm, the key turns screw 1 tight (the weld fires), the key
                 is lifted clear and held: expect 0.25"""
    if os.environ.get("DEMO_HOLD_ONLY") == "1":
        return {}
    plan = plans(os.environ.get("DEMO_VARIANT", ""))
    plan.pop(os.environ.get("DEMO_IDLE_ARM", ""), None)
    return plan
