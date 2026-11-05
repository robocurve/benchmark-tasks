"""Wire-harness formboard on the station table: a white 1 mm layout sheet (45 x 70 cm) carries a field of 12 steel
routing pins (15 mm diameter, 40 mm tall) in 3 rows x 4 columns at 8 cm pitch, and a 6 mm blue printed route line.
An orange 7 mm wire (50 cm) leaves a fixed grey start-connector fixture at the board's front-left and lies straight
along the front edge of the board towards the right arm, ending in a black 45 x 16 x 12 mm connector plug. A black
mating-connector block stands behind the pin field on the right, its cavity facing the robot. The printed route runs
from the start fixture along the front edge, then turns into the pin field and climbs through it in a two-step
staircase (in at the second column gap, one gap right after the first row, one more gap right after the second row)
and ends at the cavity. The wire must pass every pin on the same side as the printed line, and the plug must be
pushed into the cavity. The shortest routes (along the front to the right-most gap and straight back, or straight
in at the second gap) pass at least one pin on the wrong side.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var PINS_DEMO): correct (default) | nothing | shortest | onejog | hold.
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

INSTRUCTION = ("Route the orange wire on the harness board along the blue printed line: the wire must pass every steel "
               "pin on the same side as the blue line. Then push the wire's plug into the black connector at the end "
               "of the line and let go.")

# -- formboard: 1 mm white layout sheet ------------------------------------------------------------------------------
SHEET_T = 0.001
BT = TOP + SHEET_T  # board top
SHEET_X, SHEET_Y = (0.05, 0.50), (-0.45, 0.25)
WHITE = (0.95, 0.95, 0.93)
BLUE = (0.10, 0.25, 0.75)
STEEL = (0.38, 0.40, 0.44)
INK_T = 0.0003
LINE_W = 0.006
SHEET = Box("formboard", pos=((SHEET_X[0] + SHEET_X[1]) / 2, (SHEET_Y[0] + SHEET_Y[1]) / 2, TOP + SHEET_T / 2),
            size=(SHEET_X[1] - SHEET_X[0], SHEET_Y[1] - SHEET_Y[0], SHEET_T), color=WHITE)

# -- routing pins: 3 rows (x) x 4 columns (y), 8 cm pitch ------------------------------------------------------------
PIN_R, PIN_H = 0.0075, 0.040
ROWS = (0.20, 0.28, 0.36)
COLS = (0.08, 0.00, -0.08, -0.16)
PINS = np.array([(x, y) for x in ROWS for y in COLS])  # pin k = row k // 4 (front to back), column k % 4 (left to right)
PIN_OBJS = [Cylinder(f"pin_{k // 4 + 1}{k % 4 + 1}", pos=(float(p[0]), float(p[1]), BT + PIN_H / 2), radius=PIN_R,
                     height=PIN_H, color=STEEL) for k, p in enumerate(PINS)]

# -- printed route: start fixture exit -> along the front edge -> staircase through the pin field -> connector mouth --
START = np.array([0.13, 0.17])  # wire exit from the start fixture
MOUTH = np.array([0.42, -0.12])  # connector cavity opening (insertion along +x)
ROUTE = np.array([START, (0.13, 0.04), (0.24, 0.04), (0.24, -0.04), (0.32, -0.04), (0.32, -0.12), MOUTH])


def _route_lines() -> list[Box]:
    out = []
    for i, (a, b) in enumerate(zip(ROUTE[:-1], ROUTE[1:])):
        c, d = (a + b) / 2, b - a
        n = float(np.linalg.norm(d))
        horiz = abs(d[0]) > abs(d[1])
        size = (n + LINE_W, LINE_W, INK_T) if horiz else (LINE_W, n + LINE_W, INK_T)
        out.append(Box(f"route_{i}", pos=(float(c[0]), float(c[1]), BT + INK_T / 2), size=size, color=BLUE))
    return out


# -- start-connector fixture and wire -----------------------------------------------------------------------------------
FIX_SIZE = (0.04, 0.03, 0.025)
FIXTURE = Box("start_fixture", pos=(float(START[0]), float(START[1] + 0.006 + FIX_SIZE[1] / 2), BT + FIX_SIZE[2] / 2),
              size=FIX_SIZE, color=(0.45, 0.47, 0.50))
R = 0.0035  # 7 mm wire
WIRE_LEN = 0.50
PL, PW, PH = 0.045, 0.016, 0.012  # plug: length along the wire, width, height
PLUG = Plug(size=(PL, PW, PH), mass=0.015, color=(0.06, 0.06, 0.06))
WIRE_YAW = math.radians(-88.0)
WIRE = cable("wire", start=(float(START[0]), float(START[1]), BT + R + 0.0005), yaw=WIRE_YAW, length=WIRE_LEN,
             radius=R, anchor_start=True, end_plug=PLUG, color=(0.95, 0.45, 0.08), bend_stiffness=0.01,
             bend_damping=0.001)

# -- mating connector: fixed block behind the field, cavity 20 x 15 mm, 25 mm deep, opening towards -x --------------
CAV_W, CAV_H, CAV_D = PW + 0.004, PH + 0.003, 0.025
MOD_D, MOD_W, MOD_H = 0.06, 0.07, 0.03
DARK = (0.10, 0.10, 0.11)


def _connector() -> list[Box]:
    x0, y0 = MOUTH

    def box(tag, u0, u1, v0, v1, z0, z1):
        return Box(f"connector_{tag}", pos=(float(x0 + (u0 + u1) / 2), float(y0 + (v0 + v1) / 2), BT + (z0 + z1) / 2),
                   size=(u1 - u0, v1 - v0, z1 - z0), color=DARK)

    hw, cw = MOD_W / 2, CAV_W / 2
    return [box("back", CAV_D, MOD_D, -hw, hw, 0.0, MOD_H), box("roof", 0.0, CAV_D, -hw, hw, CAV_H, MOD_H),
            box("wall_l", 0.0, CAV_D, cw, hw, 0.0, CAV_H), box("wall_r", 0.0, CAV_D, -hw, -cw, 0.0, CAV_H)]


OBJECTS = [SHEET, *_route_lines(), *PIN_OBJS, FIXTURE, WIRE, *_connector()]

# -- scoring ----------------------------------------------------------------------------------------------------------
SEATED = 0.012  # plug tip at least 12 mm inside the cavity
LAT_MAX = 0.006  # plug centre within 6 mm of the cavity axis sideways
NEAR = 0.04  # partial credit: plug centre within 4 cm of the mouth, in front of it
LOW_Z = BT + 0.025  # every wire point at most 25 mm above the board (pins are 40 mm tall)
RELEASE_DIST = 0.035  # both gripper TCPs at least 3.5 cm from the plug centre
_ENV: dict = {"env": None}


def _install_hooks() -> None:
    """Keep a handle on the env so score() can see where the grippers are."""
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls._task_step, "_pins_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            return base(self, *a, **k)

        _task_step._pins_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    """World positions of both grippers' TCP; [] if unknown (offline)."""
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


def plug_pose(objects) -> tuple[np.ndarray, np.ndarray]:
    """(plug centre, unit axis pointing out through its front end)."""
    st = objects["wire"]
    p = np.asarray(st.end_plug if st.end_plug is not None else np.full(3, np.nan), dtype=float)
    a = p - np.asarray(st.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def tip_depth(objects) -> float:
    """How far the plug's front end reaches past the cavity mouth (m); -1 if the plug is not in line with the cavity
    (centre more than 6 mm off its axis sideways, lifted above it, or pointing more than 30 deg off the axis)."""
    p, a = plug_pose(objects)
    if not np.isfinite(p).all():
        return -1.0
    if abs(p[1] - MOUTH[1]) > LAT_MAX or p[2] > BT + CAV_H + 0.003 or a[0] < math.cos(math.radians(30)):
        return -1.0
    return float(p[0] + PL / 2 - MOUTH[0])


def _dense(poly: np.ndarray, step: float = 0.003) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        k = max(1, int(np.linalg.norm(b - a) / step))
        out.extend(a + (b - a) * (j / k) for j in range(1, k + 1))
    return np.array(out)


def windings(objects) -> np.ndarray:
    """Winding number around each pin of the closed loop: wire (start fixture -> plug centre), straight on to the
    connector mouth, then back along the printed line to the start. 0 = the wire passes that pin on the same side as
    the printed line."""
    st = objects["wire"]
    p, _a = plug_pose(objects)
    wire = np.vstack([st.polyline()[:, :2], p[None, :2]])
    loop = np.vstack([START[None], wire, MOUTH[None], ROUTE[::-1][1:]])
    w = []
    for c in PINS:
        v = loop - c
        ang = np.arctan2(v[:, 1], v[:, 0])
        d = np.diff(ang)
        d = (d + math.pi) % (2 * math.pi) - math.pi
        w.append(d.sum() / (2 * math.pi))
    return np.round(np.array(w), 3)


def analyse(objects) -> dict:
    st = objects["wire"]
    p, _a = plug_pose(objects)
    if not (np.isfinite(st.points).all() and np.isfinite(p).all()):  # simulation blew up: no credit
        return {"over": list(range(len(PINS))), "wrong": list(range(len(PINS))), "depth": -1.0, "near": False,
                "released": False, "low": False}
    pts = _dense(st.polyline())
    over = [k for k, c in enumerate(PINS) if (np.linalg.norm(pts[:, :2] - c, axis=1) < PIN_R).any()]
    w = windings(objects)
    wrong = [k for k in range(len(PINS)) if abs(w[k]) > 0.5]
    dep = tip_depth(objects)
    near = bool(np.isfinite(p).all() and np.linalg.norm(p[:2] - MOUTH) <= NEAR and p[0] <= MOUTH[0] + PL / 2)
    gs = grippers()
    released = all(np.isfinite(p).all() and float(np.linalg.norm(p - g)) >= RELEASE_DIST for g in gs)
    low = bool(st.points[:, 2].max() <= LOW_Z and np.isfinite(st.points).all())
    return {"over": over, "wrong": wrong, "depth": dep, "near": near, "released": released, "low": low}


def score(objects) -> float:
    """0 if: the wire passes any pin on the other side from the printed line (or lies over a pin), any part of the
    wire is more than 25 mm above the board, a gripper is still within 3.5 cm of the plug, or the plug is not within
    4 cm of the connector mouth. Otherwise: plug tip >= 12 mm into the cavity 1.0; tip inside the cavity (>= 0 mm)
    0.5; plug lying within 4 cm in front of the mouth 0.25."""
    r = analyse(objects)
    if r["over"] or r["wrong"] or not r["low"] or not r["released"] or not (r["near"] or r["depth"] >= 0.0):
        return 0.0
    if r["depth"] >= SEATED:
        return 1.0
    if r["depth"] >= 0.0:
        return 0.5
    return 0.25


def success(objects) -> bool:
    """Full score, wire at rest, and both grippers moved at least 10 cm clear of the plug (so the last frames show the
    finished routing unobstructed)."""
    p, _a = plug_pose(objects)
    clear = all(float(np.linalg.norm(p - g)) >= 0.10 for g in grippers())
    return bool(score(objects) >= 0.999 and np.linalg.norm(objects["wire"].lin_vel) < 0.02 and clear)


def distance(objects) -> float:
    """Plug centre to its seated position (m); 1 m while the wire passes a pin on the wrong side."""
    r = analyse(objects)
    if r["over"] or r["wrong"]:
        return 1.0
    p, _a = plug_pose(objects)
    seat = np.array([MOUTH[0] + 0.015 - PL / 2, MOUTH[1]])
    return float(np.linalg.norm(p[:2] - seat)) if np.isfinite(p).all() else 1.0


def progress(objects) -> dict:
    """Stage diagnostic (not the score): fraction of pins on the drawn side, wire on the board, plug near / in the
    cavity (0..1 up to 12 mm), grippers released."""
    r = analyse(objects)
    ok = len(PINS) - len(set(r["wrong"]) | set(r["over"]))
    return {"pins_on_drawn_side": ok / len(PINS), "wire_on_board": float(r["low"]),
            "plug_near_mouth": float(r["near"] or r["depth"] >= 0.0),
            "plug_seated": float(np.clip(r["depth"] / SEATED, 0.0, 1.0)) if r["depth"] > -1 else 0.0,
            "released": float(r["released"])}


# Seeded reset variation for a later runner (the demo runs on the nominal layout): the wire's spawn heading varies by
# +-3 deg (its plug stays in front of the field, clear of the front-row pins by >= 3 cm); the pins, printed line and
# connector are one fixed board and do not move relative to each other.
VARIATION = {"cable:wire": {"dx": 0.0, "dy": 0.0, "yaw": 0.05}}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
# Right arm: pinches the plug 12 mm from its wire end with the gripper turned along +x (wrist camera looking towards
# the connector behind the field), carries it to the entry gap and leads it through the gap centres of the drawn
# staircase just above the board; the trailing wire wraps the inside-corner pins. Left arm: presses the routed wire
# down on the board just inside the entry gap while the right arm pulls the plug through the second step and inserts
# it, so the already-routed part cannot be dragged along. Then both let go and park.

FINGER_PINCH = -0.022
GRASP_BACK = 0.012  # pinch this far from the plug's wire end
GZ = BT + PH + 0.002  # TCP height for pinching / leading the plug
CARRY = BT + 0.030
INSERT_TO = 0.016
PARK = {"left": (0.20, 0.26, TOP + 0.15), "right": (0.20, -0.26, TOP + 0.15)}


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _log(ctx, tag=""):
    st = ctx.states()
    r = analyse(st)
    p, a = plug_pose(st)
    print(f"[t={ctx.t:5.2f}] {tag} score={score(st)} wrong={r['wrong']} over={r['over']} depth_mm="
          f"{r['depth'] * 1000:.1f} near={r['near']} low={r['low']} rel={r['released']} plug={np.round(p, 3).tolist()} "
          f"head={math.degrees(math.atan2(a[1], a[0])):.0f} w={windings(st).tolist()}", flush=True)


def _tcp_for(c: np.ndarray, yaw: float) -> np.ndarray:
    """TCP xy that puts the plug centre at ``c`` when the plug points along ``yaw`` (pinched GRASP_BACK from its rear)."""
    u = np.array([math.cos(yaw), math.sin(yaw)])
    return np.asarray(c, dtype=float) - u * (PL / 2 - GRASP_BACK)


def _pick_plug(ctx, arm):
    from yam_arena.constants import FINGER_CLOSED

    p, a = plug_pose(ctx.states())
    yaw = math.atan2(a[1], a[0])  # ~ -88 deg; turned to 0 after the pinch
    g = _tcp_for(p[:2], yaw)
    yield from arm.move((g[0], g[1], CARRY + 0.03), yaw=yaw, finger=FINGER_PINCH, duration=1.5)
    p, a = plug_pose(ctx.states())
    g = _tcp_for(p[:2], yaw)
    yield from arm.move((g[0], g[1], GZ), yaw=yaw, duration=0.9)
    yield from arm.grip(FINGER_CLOSED, 0.5)
    yield from arm.move((g[0], g[1], CARRY), yaw=yaw, duration=0.6)
    return yaw


def _lead(arm, centres, yaw=0.0, z=GZ, dur=1.2):
    for c in centres:
        t = _tcp_for(np.asarray(c), yaw)
        yield from arm.move((t[0], t[1], z), yaw=yaw, duration=dur)


def _insert(ctx, arm):
    """Line the held plug up on the cavity axis 15 mm in front of the mouth (closed loop), push it in, let go."""
    from yam_arena.constants import FINGER_CLOSED

    yaw = 0.0
    pre = np.array([MOUTH[0] - PL / 2 - 0.015, MOUTH[1]])
    tcp = _tcp_for(pre, yaw)
    yield from arm.move((tcp[0], tcp[1], GZ), yaw=yaw, duration=1.0)
    for _ in range(4):
        yield from arm.wait(0.25)
        p, a = plug_pose(ctx.states())
        dpsi = _wrap(math.atan2(a[1], a[0]))
        e = pre - p[:2]
        if np.linalg.norm(e) < 0.0015 and abs(dpsi) < 0.035:
            break
        if abs(dpsi) >= 0.035:
            yaw -= float(np.clip(dpsi, -0.3, 0.3))
        tcp = tcp + np.clip(e, -0.02, 0.02)
        yield from arm.move((tcp[0], tcp[1], GZ), yaw=yaw, duration=0.5)
    for attempt in range(4):
        p, a = plug_pose(ctx.states())
        tip_out = MOUTH[0] - (p[0] + PL / 2)
        tcp = tcp + np.array([tip_out + INSERT_TO + 0.003, 0.0])
        yield from arm.move((tcp[0], tcp[1], GZ), yaw=yaw, duration=1.4)
        yield from arm.wait(0.3)
        dep = tip_depth(ctx.states())
        if os.environ.get("PINS_DBG"):
            print(f"[insert] attempt {attempt} depth_mm={dep * 1000:.1f} plug={np.round(p, 3).tolist()}", flush=True)
        if dep >= SEATED + 0.002:
            break
        p, a = plug_pose(ctx.states())
        tcp = _tcp_for(p[:2], yaw) - np.array([0.008, 0.0])
        yield from arm.move((tcp[0], tcp[1], GZ + 0.002), yaw=yaw, duration=0.5)
        p, a = plug_pose(ctx.states())
        front = p[:2] + a[:2] * (PL / 2 - 0.010)
        tcp = tcp + np.array([0.0, MOUTH[1] - front[1]])
        yaw -= float(np.clip(_wrap(math.atan2(a[1], a[0])), -0.2, 0.2))
        yield from arm.move((tcp[0], tcp[1], GZ), yaw=yaw, duration=0.5)
    return yaw


def _release(arm, yaw, back=(-0.03, 0.0)):
    yield from arm.grip(FINGER_PINCH, 0.5)
    t = arm.tcp()
    yield from arm.move((t[0] + back[0], t[1] + back[1], CARRY + 0.05), yaw=yaw, duration=0.8)


def _park(arm, side):
    yield from arm.move(PARK[side], yaw=0.0, duration=1.2)


ENTRY = (0.115, 0.04)  # plug centre in front of the entry gap (column gap 1-2)
PRESS = (0.15, 0.08)  # left arm holds the routed wire here (front of the entry gap)
LEAD_YAW = -math.pi / 4  # plug heading while it is led through the field


def _wire_near(objects, xy) -> tuple[np.ndarray, float]:
    """Live wire point nearest ``xy`` (segment centre) and the grip yaw across the wire there."""
    pts = objects["wire"].points
    dist = np.linalg.norm(pts[:, :2] - np.asarray(xy), axis=1)
    clear = np.linalg.norm(pts[:, None, :2] - PINS[None], axis=2).min(axis=1) >= 0.035  # fingers clear of the pins
    i = int(np.argmin(np.where(clear, dist, np.inf)))
    i = min(max(i, 1), len(pts) - 2)
    d = pts[i + 1] - pts[i - 1]
    h = math.atan2(d[1], d[0])
    return pts[i], (h + math.pi / 2) % math.pi - math.pi / 2


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    mode = os.environ.get("PINS_DEMO", "correct")  # test only: correct | nothing | shortest | onejog | hold
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "nothing":
        for _ in range(4):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return

    # 1) right arm pinches the plug and turns it to point along +x (towards the connector)
    yaw0 = yield from _pick_plug(ctx, Rt)
    if mode == "shortest":
        # along the front edge to the right-most gap in front of the connector, straight back and in
        path = [(0.12, -0.12), (0.32, -0.12)]
    elif mode == "onejog":
        # in at the drawn entry gap, straight up to the back row gap, one step right there and in
        path = [ENTRY, (0.32, 0.04), (0.32, -0.04), (0.32, -0.12)]
    else:  # drawn staircase
        path = [ENTRY, (0.24, 0.04), (0.24, -0.04), (0.32, -0.04), (0.32, -0.12)]
    t = _tcp_for(np.asarray(path[0]), LEAD_YAW)
    mid = _tcp_for(np.array([0.11, -0.25]), LEAD_YAW)
    yield from Rt.move((mid[0], mid[1], CARRY), yaw=(yaw0 + LEAD_YAW) / 2, duration=1.2)
    yield from Rt.move((t[0], t[1], CARRY), yaw=LEAD_YAW, duration=1.5)
    yield from Rt.move((t[0], t[1], GZ), yaw=LEAD_YAW, duration=0.6)
    _log(ctx, "at entry")

    # 2) lead the plug through the gap centres of the path, pointing diagonally back-right so the trailing wire
    #    leaves its rear end towards the part already routed
    for c in path[1:]:
        yield from _lead(Rt, [c], yaw=LEAD_YAW)
        _log(ctx, f"plug at {c}")
    # 3) left arm holds the routed wire down on the board just in front of the entry gap, so the insertion pull
    #    cannot drag the routed part along
    hy = 0.0
    if mode != "shortest":
        hp, hy = _wire_near(ctx.states(), PRESS)
        yield from L.move((hp[0], hp[1], CARRY + 0.03), yaw=hy, finger=FINGER_PINCH, duration=1.5)
        hp, hy = _wire_near(ctx.states(), PRESS)
        yield from L.move((hp[0], hp[1], BT + R + 0.009), yaw=hy, duration=0.8)
        yield from L.grip(FINGER_CLOSED, 0.4)
        _log(ctx, f"left holds wire at {np.round(hp, 3).tolist()}")

    # 4) turn the plug square to the connector, insert, let go, park
    sq = _tcp_for(np.asarray(path[-1]), 0.0)
    yield from Rt.move((sq[0], sq[1], GZ), yaw=0.0, duration=1.0)
    _log(ctx, "squared")
    yaw = yield from _insert(ctx, Rt)
    _log(ctx, "inserted")
    if mode == "hold":
        yield from L.wait(2.0)
        _log(ctx, "end (holding)")
        return
    yield from _release(Rt, yaw)
    if mode != "shortest":
        yield from L.grip(FINGER_PINCH, 0.4)
        t = L.tcp()
        yield from L.move((t[0], t[1], CARRY + 0.05), yaw=hy, duration=0.6)
    yield from ctx.both(_park(L, "left"), _park(Rt, "right"))
    for _ in range(2):
        yield from L.wait(1.0)
    _log(ctx, "end")
