"""On the station table, on the robot's left, stands an ECG lead-wire tester (a light grey block, 24 x 6 x 3 cm,
fixed). In front of it, on the table, are three connector seats (pairs of 4 mm rails 3 cm long), 6 cm apart, and
behind each seat, on the tester's top, a round status lamp (26 mm, green when lit). Three identical ECG lead wires
(grey 5 mm cords, 28 cm) each have a dark connector (30 x 20 x 15 mm, 15 g) resting in its seat and run straight
towards the robot's right, across the table, ending in a black snap clip (20 x 15 x 10 mm) that lies on a brass
shorting bar (20 x 11 cm, 2 mm). A lamp is lit while its lead's connector sits in its seat. One lead (a, the one
nearest the robot, x 0.27) has an intermittent break at its strain relief: its lamp goes dark while the cord 5-7 cm
from its connector is bent by more than 30 degrees. A red reject tray (20 x 18 cm inside, 15 mm walls) stands on the
robot's right, nearest the far lead. The robot must flex every lead near its connector, watch the lamps, take the
lead whose lamp went dark out of the tester by its connector, put it in the tray, keep the good leads connected, and
let go. Only the left arm reaches the connector seats; only the right arm reaches the tray.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cables below are simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Three ECG lead wires are connected to this training lead tester, and one of them has an intermittent "
               "break. Find the faulty lead and put it in the red reject tray.")

# -- geometry (m, world frame: x away from the robot, y to its left) -------------------------------------------------
LEADS = ("a", "b", "c")
SEAT_X = {"a": 0.27, "b": 0.33, "c": 0.39}
FAULTY = "a"  # nearest the robot, farthest from the reject tray
SEAT_Y = 0.18  # connector centre
CL, CW, CH = 0.030, 0.020, 0.015  # connector: length along the cord, width, height
SL, SW, SH = 0.020, 0.015, 0.010  # snap clip
R = 0.0025  # 5 mm cord
CORD_LEN = 0.28
EXIT_Y = SEAT_Y - CL / 2  # where the cord leaves the connector
Z0 = TOP + CH / 2 + 0.0005  # connector (and cord) centre height at spawn
GREY_CORD = (0.62, 0.63, 0.64)
DARK = (0.12, 0.12, 0.13)
TESTER_GREY = (0.80, 0.80, 0.78)

TESTER_Y0, TESTER_Y1, TESTER_H = SEAT_Y + CL / 2 + 0.001, SEAT_Y + CL / 2 + 0.061, 0.030
TESTER = Box("tester", pos=(0.33, (TESTER_Y0 + TESTER_Y1) / 2, TOP + TESTER_H / 2),
             size=(0.24, TESTER_Y1 - TESTER_Y0, TESTER_H), color=TESTER_GREY)
LAMP_Y, LAMP_R = (TESTER_Y0 + TESTER_Y1) / 2, 0.013
LAMP_Z = TOP + TESTER_H  # lamps sit on the tester's top
RAIL_H, RAIL_T = 0.003, 0.004
RAIL_OFF = CW / 2 + 0.0015 + RAIL_T / 2


def _seat_parts(k: str) -> list:
    x = SEAT_X[k]
    return [Box(f"rail_{k}_{s}", pos=(x + sg * RAIL_OFF, SEAT_Y, TOP + RAIL_H / 2), size=(RAIL_T, CL, RAIL_H),
                color=DARK) for s, sg in (("n", -1), ("p", 1))] + [
        Cylinder(f"lamp_{k}", pos=(x, LAMP_Y, LAMP_Z + 0.002), radius=LAMP_R, height=0.004, color=(0.18, 0.20, 0.18))]


PLATE_X0, PLATE_X1, PLATE_Y0, PLATE_Y1, PLATE_H = 0.23, 0.43, -0.17, -0.06, 0.002
PLATE = Box("shorting_bar", pos=((PLATE_X0 + PLATE_X1) / 2, (PLATE_Y0 + PLATE_Y1) / 2, TOP + PLATE_H / 2),
            size=(PLATE_X1 - PLATE_X0, PLATE_Y1 - PLATE_Y0, PLATE_H), color=(0.78, 0.62, 0.25))

TRAY_X0, TRAY_X1, TRAY_Y0, TRAY_Y1 = 0.32, 0.52, -0.37, -0.19  # inner footprint
TRAY_WALL_T, TRAY_WALL_H = 0.004, 0.015
RED = (0.80, 0.10, 0.10)


def _tray() -> list[Box]:
    cx, cy = (TRAY_X0 + TRAY_X1) / 2, (TRAY_Y0 + TRAY_Y1) / 2
    w, d, t, h = TRAY_X1 - TRAY_X0, TRAY_Y1 - TRAY_Y0, TRAY_WALL_T, TRAY_WALL_H
    z = TOP + 0.002 + h / 2
    return [
        Box("tray_floor", pos=(cx, cy, TOP + 0.001), size=(w + 2 * t, d + 2 * t, 0.002), color=RED),
        Box("tray_xn", pos=(TRAY_X0 - t / 2, cy, z), size=(t, d + 2 * t, h), color=RED),
        Box("tray_xp", pos=(TRAY_X1 + t / 2, cy, z), size=(t, d + 2 * t, h), color=RED),
        Box("tray_yn", pos=(cx, TRAY_Y0 - t / 2, z), size=(w, t, h), color=RED),
        Box("tray_yp", pos=(cx, TRAY_Y1 + t / 2, z), size=(w, t, h), color=RED),
    ]


def _lead(k: str):
    return cable(f"lead_{k}", start=(SEAT_X[k], EXIT_Y, Z0), yaw=-math.pi / 2, length=CORD_LEN, radius=R,
                 start_plug=Plug(size=(CL, CW, CH), mass=0.015, color=DARK),
                 end_plug=Plug(size=(SL, SW, SH), mass=0.006, color=(0.05, 0.05, 0.05)),
                 color=GREY_CORD, bend_stiffness=0.01, bend_damping=0.001)


CORDS = {k: _lead(k) for k in LEADS}
OBJECTS = [TESTER, *[p for k in LEADS for p in _seat_parts(k)], PLATE, *_tray(), *CORDS.values()]

# -- lamps: a lit green lens (visual only) above each dark lamp, shown or hidden every control step -----------------
LIT = (0.15, 1.0, 0.25)


def _install_lenses() -> None:
    orig = cablekit._spawn_cable
    if getattr(orig, "_wiggle_lenses", False):
        return

    def spawn(prim_path, cfg, *a, **k):
        prim = orig(prim_path, cfg, *a, **k)
        if getattr(cfg, "cable_name", "") != CORDS[LEADS[-1]].name:
            return prim
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_visual_material
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        stage = get_current_stage()
        root = str(prim_path).rsplit("/", 1)[0]
        mat = f"{root}/lens_look"
        look = sim_utils.PreviewSurfaceCfg(diffuse_color=LIT, emissive_color=(0.1, 0.9, 0.2), roughness=0.3)
        look.func(mat, look)
        for k in LEADS:
            path = f"{root}/lens_{k}"
            g = UsdGeom.Cylinder.Define(stage, path)
            g.CreateRadiusAttr(LAMP_R - 0.001)
            g.CreateHeightAttr(0.002)
            g.CreateAxisAttr("Z")
            g.AddTranslateOp().Set(Gf.Vec3d(SEAT_X[k], LAMP_Y, LAMP_Z + 0.0045))
            bind_visual_material(path, mat)
            _LENS[k] = path
        return prim

    spawn._wiggle_lenses = True
    cablekit._spawn_cable = spawn


_LENS: dict[str, str] = {}
_install_lenses()

# -- tester logic and trial history -----------------------------------------------------------------------------------
SEAT_TOL = 0.004  # connector centre within 4 mm of its seat (top view)
SEAT_DZ = 0.005  # ... and within 5 mm of its resting height
SEAT_ANG = math.radians(12)  # ... and its axis within 12 deg of the seat axis
BEND_FAULT = math.radians(30)  # the break opens when the cord 6 cm from the connector is bent more than this
RELEASE_DIST = 0.035
PULL_DIST = 0.01  # a connector leaving its seat by more than 1 cm ...
HAND_DIST = 0.025  # ... must have a gripper TCP within 2.5 cm of its centre (taken out by the connector, not by the cord)
OUT_DIST = 0.03  # "removed": connector >= 3 cm from its seat and snap off the bar


def _ok(v) -> bool:
    return v is not None and bool(np.all(np.isfinite(v)))


def conn_axis(c) -> np.ndarray:
    a = np.asarray(c.start_plug) - c.points[0]
    return a / (np.linalg.norm(a) + 1e-12)


def bend(c) -> float:
    """Angle (rad) between the connector's axis and the cord 5-7 cm from the cord exit."""
    a = conn_axis(c)
    u = c.points[2] - c.points[3]
    u = u / (np.linalg.norm(u) + 1e-12)
    return float(math.acos(float(np.clip(a @ u, -1.0, 1.0))))


def seated(c, k: str) -> bool:
    p = c.start_plug
    if not _ok(p):
        return False
    off = math.hypot(p[0] - SEAT_X[k], p[1] - SEAT_Y)
    ang = math.acos(float(np.clip(conn_axis(c) @ np.array([0.0, 1.0, 0.0]), -1.0, 1.0)))
    return off <= SEAT_TOL and abs(p[2] - Z0) <= SEAT_DZ and ang <= SEAT_ANG


def on_bar(c) -> bool:
    p = c.end_plug
    return _ok(p) and PLATE_X0 <= p[0] <= PLATE_X1 and PLATE_Y0 <= p[1] <= PLATE_Y1 and p[2] < TOP + 0.02


def lamp_lit(objects, k: str) -> bool:
    c = objects[CORDS[k].name]
    if not seated(c, k):
        return False
    return not (k == FAULTY and bend(c) > BEND_FAULT)


class _Episode:
    def __init__(self):
        self.good_dark: set[str] = set()  # good leads whose lamp went dark at some step (latched)
        self.tested: set[str] = set()  # leads flexed past the threshold while seated and on the bar (latched)
        self.max_bend = {k: 0.0 for k in LEADS}
        self.left_seat: set[str] = set()  # leads whose connector has gone more than 1 cm from its seat
        self.cord_pulled: set[str] = set()  # ... with no gripper within 2.5 cm of the connector at that moment


_EP = _Episode()
_ENV: dict = {"env": None}
_LAMP_SHOWN: dict[str, bool] = {}


def reset_episode() -> None:
    global _EP
    _EP = _Episode()


def _track(objects) -> None:
    for k in LEADS:
        c = objects[CORDS[k].name]
        if not _ok(c.points):
            if k != FAULTY:
                _EP.good_dark.add(k)
            continue
        s, b = seated(c, k), bend(c)
        if s:
            _EP.max_bend[k] = max(_EP.max_bend[k], b)
            if b > BEND_FAULT:
                _EP.tested.add(k)
        if k != FAULTY and not lamp_lit(objects, k):
            _EP.good_dark.add(k)
        p = c.start_plug
        if k not in _EP.left_seat and _ok(p) and math.hypot(p[0] - SEAT_X[k], p[1] - SEAT_Y) > PULL_DIST:
            _EP.left_seat.add(k)
            gs = grippers()
            if gs and min(float(np.linalg.norm(g - p)) for g in gs) > HAND_DIST:
                _EP.cord_pulled.add(k)


def _show_lamps(objects) -> None:
    if not _LENS:
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    stage = get_current_stage()
    for k in LEADS:
        on = lamp_lit(objects, k)
        if _LAMP_SHOWN.get(k) is on:
            continue
        img = UsdGeom.Imageable(stage.GetPrimAtPath(_LENS[k]))
        img.MakeVisible() if on else img.MakeInvisible()
        _LAMP_SHOWN[k] = on


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline use)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_wiggle_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            out = orig(self, *a, **k)
            reset_episode()
            return out

        reset._wiggle_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_wiggle_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            from yam_arena.scene import object_states

            states = object_states(self._env, [c.name for c in CORDS.values()])
            _track(states)
            _show_lamps(states)
            return base(self, *a, **k)

        _task_step._wiggle_hooked = True
        cls._task_step = _task_step


_install_hooks()


def grippers() -> list[np.ndarray]:
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def _dense(poly: np.ndarray, step: float = 0.004) -> np.ndarray:
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        n = max(1, int(np.linalg.norm(b - a) / step))
        out += [a + (b - a) * (i / n) for i in range(1, n + 1)]
    return np.asarray(out)


def _lead_pts(c) -> np.ndarray:
    return np.vstack([_dense(c.polyline())] + [p[None] for p in (c.start_plug, c.end_plug) if p is not None])


def released(objects) -> bool:
    pts = np.vstack([_lead_pts(objects[c.name]) for c in CORDS.values()])
    if not np.isfinite(pts).all():
        return False
    return all(float(np.linalg.norm(pts - g, axis=1).min()) >= RELEASE_DIST for g in grippers())


def in_tray(c) -> bool:
    pts = _lead_pts(c)
    m = 0.003
    return bool(np.isfinite(pts).all() and (pts[:, 0] >= TRAY_X0 - m).all() and (pts[:, 0] <= TRAY_X1 + m).all()
                and (pts[:, 1] >= TRAY_Y0 - m).all() and (pts[:, 1] <= TRAY_Y1 + m).all()
                and (pts[:, 2] <= TOP + 0.04).all())


def removed(c, k: str) -> bool:
    p = c.start_plug
    return _ok(p) and math.hypot(p[0] - SEAT_X[k], p[1] - SEAT_Y) >= OUT_DIST and not on_bar(c)


def score(objects) -> float:
    """Score 0 if: a good lead's lamp ever went dark (its connector moved > 4 mm, rose > 5 mm or turned > 12 deg in
    its seat); any connector left its seat (> 1 cm) with no gripper within 2.5 cm of it (pulled out by the cord); not
    every lead was flexed past 30 deg (cord 5-7 cm from the connector vs the connector's axis) while seated; or a
    gripper TCP is within 3.5 cm of any lead at the end. Otherwise: faulty lead entirely in the tray -> 1.0; faulty
    lead out of the tester (connector >= 3 cm from its seat, snap off the bar) but not in the tray -> 0.5; else 0."""
    _track(objects)
    if _EP.good_dark or _EP.cord_pulled or _EP.tested < set(LEADS) or not released(objects):
        return 0.0
    f = objects[CORDS[FAULTY].name]
    if in_tray(f):
        return 1.0
    return 0.5 if removed(f, FAULTY) else 0.0


def success(objects) -> bool:
    return bool(score(objects) >= 1.0 and all(objects[c.name].resting(max_speed=0.02) for c in CORDS.values()))


def distance(objects) -> float:
    if _EP.good_dark:
        return 1.0
    f = objects[CORDS[FAULTY].name]
    p = f.start_plug
    if not _ok(p):
        return 1.0
    c = np.array([(TRAY_X0 + TRAY_X1) / 2, (TRAY_Y0 + TRAY_Y1) / 2])
    return float(np.linalg.norm(p[:2] - c))


def progress(objects) -> dict:
    _track(objects)
    f = objects[CORDS[FAULTY].name]
    return {"good_leads_kept": float(not _EP.good_dark), "no_cord_pull": float(not _EP.cord_pulled),
            "faulty_tested": float(FAULTY in _EP.tested),
            "all_tested": len(_EP.tested) / 3.0, "faulty_removed": float(removed(f, FAULTY) or in_tray(f)),
            "faulty_in_tray": float(in_tray(f)), "released": float(released(objects))}


TWIN_EXTRA = [
    {"name": f"lens_{k}", "kind": "decal", "size_mm": [24, 24], "colour": "green (LED behind a diffuser, lit while the"
     " lead conducts)", "pose": [SEAT_X[k], LAMP_Y, 0.035]} for k in LEADS
]

VARIATION = {
    "cable:lead_a": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},  # connectors sit in fixed seats: no start offset
    "cable:lead_b": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
    "cable:lead_c": {"dx": 0.0, "dy": 0.0, "yaw": 0.0},
}


# -- closed-loop scripted demo (cabledemo.py) ------------------------------------------------------------------------
# Test: the left arm hovers over the lead's connector with its wrist camera on that lead's lamp; the right arm takes
# the cord 20 cm from the connector and pushes it 5 cm towards the tester while lifting 5 cm, so the cord humps and
# bends sharply where it leaves the connector, then lays it back. The lead whose lamp went dark is the faulty one.
# Reject: the left arm lifts that connector out of its seat and sets it down near the midline (the right arm cannot
# reach the seats); the right arm lifts the lead by the middle of its cord, lowers it into the tray, and tucks in any
# part left outside.
FINGER_PINCH = -0.020
GRASP_OPEN = -0.014  # pads ~28 mm apart for the bare cord
FLEX_S = 0.20  # right arm takes the cord this far (m, along it) from the connector's cord exit
FEED, LIFT = 0.05, 0.05
HI = TOP + 0.07
L_PARK = (0.22, 0.26, TOP + 0.12)
R_PARK = (0.22, -0.26, TOP + 0.12)
DROP = (0.27, 0.04)  # where the left arm sets the removed connector down


def _log(ctx, tag=""):
    st = ctx.states()
    info = []
    for k in LEADS:
        c = st[CORDS[k].name]
        p = c.start_plug
        info.append(f"{k}:off={1000 * math.hypot(p[0] - SEAT_X[k], p[1] - SEAT_Y):.1f} "
                    f"bend={math.degrees(bend(c)):.0f} lit={int(lamp_lit(st, k))} bar={int(on_bar(c))} "
                    f"tray={int(in_tray(c))}")
    print(f"[t={ctx.t:6.2f}] {tag:10s} {' | '.join(info)} dark={sorted(_EP.good_dark)} tested={sorted(_EP.tested)} "
          f"pulled={sorted(_EP.cord_pulled)} rel={int(released(st))} score={score(st)}", flush=True)


def _cord_at(c, s: float):
    """Point and heading (rad) on the live lead ``s`` m from the connector's centre (along tips and centres)."""
    poly = np.vstack([c.start_plug[None], c.polyline()]) if c.start_plug is not None else c.polyline()
    d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(poly, axis=0), axis=1))]
    i = int(np.clip(np.searchsorted(d, s), 1, len(poly) - 1))
    t = (s - d[i - 1]) / max(d[i] - d[i - 1], 1e-9)
    p = poly[i - 1] + t * (poly[i] - poly[i - 1])
    v = poly[i] - poly[i - 1]
    return p, math.atan2(v[1], v[0])


def _grip_yaw(h):
    return (h + math.pi / 2) % math.pi - math.pi / 2


def _watch(L, k):
    """Left arm hovers over lead k's connector, wrist camera on its lamp."""
    yield from L.move((SEAT_X[k], SEAT_Y, TOP + 0.075), yaw=math.pi / 2, finger=FINGER_PINCH, duration=1.2)


def _flex(Rt, ctx, k):
    """Right arm flexes lead k at its strain relief; returns True if its lamp went dark while it was flexed."""
    from yam_arena.constants import FINGER_CLOSED

    c = ctx.states()[CORDS[k].name]
    p, h = _cord_at(c, FLEX_S + CL / 2)
    gy = _grip_yaw(h)
    g = np.array([p[0], p[1], p[2] + 0.010])
    yield from Rt.move((g[0], g[1], HI), yaw=gy, finger=GRASP_OPEN, duration=1.2)
    yield from Rt.move(g, yaw=gy, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    dark = False
    for f in np.linspace(0.0, 1.0, 7)[1:]:
        yield from Rt.move((g[0], g[1] + f * FEED, g[2] + f * LIFT), yaw=gy, duration=0.4)
        dark |= not lamp_lit(ctx.states(), k)
    yield from Rt.wait(0.6)
    dark |= not lamp_lit(ctx.states(), k)
    _log(ctx, f"flex_{k}")
    yield from Rt.move(g + np.array([0.0, 0.0, 0.002]), yaw=gy, duration=1.2)
    yield from Rt.grip(GRASP_OPEN, 0.4)
    yield from Rt.move((g[0], g[1], HI), yaw=gy, duration=0.7)
    return dark


def _take_out(L, ctx, k):
    """Left arm lifts lead k's connector out of its seat by the connector body and sets it down at DROP."""
    from yam_arena.constants import FINGER_CLOSED

    p = ctx.states()[CORDS[k].name].start_plug
    gz = TOP + CH + 0.003
    yield from L.move((p[0], p[1], HI), yaw=math.pi / 2, finger=FINGER_PINCH, duration=1.0)
    yield from L.move((p[0], p[1], gz), yaw=math.pi / 2, duration=0.9)
    yield from L.grip(FINGER_CLOSED, 0.6)
    yield from L.move((p[0], p[1], gz + 0.03), yaw=math.pi / 2, duration=0.8)
    yield from L.move((DROP[0], DROP[1], gz + 0.03), yaw=math.pi / 2, duration=2.0)
    yield from L.move((DROP[0], DROP[1], gz + 0.002), yaw=math.pi / 2, duration=0.8)
    yield from L.grip(FINGER_PINCH, 0.5)
    yield from L.move((DROP[0], DROP[1], HI), yaw=math.pi / 2, duration=0.7)
    yield from L.move(L_PARK, yaw=0.0, finger=FINGER_CLOSED, duration=1.2)


def _carry(Rt, ctx, k, dest, *, z_lo=TOP + 0.05, release=True):
    """Right arm lifts lead k by the middle of its cord and lowers it at ``dest``."""
    from yam_arena.constants import FINGER_CLOSED

    c = ctx.states()[CORDS[k].name]
    p, h = _cord_at(c, CL / 2 + CORD_LEN / 2)
    gy = _grip_yaw(h)
    yield from Rt.move((p[0], p[1], HI), yaw=gy, finger=GRASP_OPEN, duration=1.2)
    yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=gy, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED, 0.5)
    yield from Rt.move((p[0], p[1], TOP + 0.15), yaw=gy, duration=1.5)
    yield from Rt.move((dest[0], dest[1], TOP + 0.15), yaw=gy, duration=2.0)
    yield from Rt.wait(0.8)
    yield from Rt.move((dest[0], dest[1], z_lo), yaw=gy, duration=1.5)
    if not release:  # "hold" shortcut: lead lowered into the tray but never let go
        yield from Rt.move((dest[0], dest[1], TOP + 0.02), yaw=gy, duration=1.0)
        return
    yield from Rt.grip(GRASP_OPEN, 0.4)
    yield from Rt.move((dest[0], dest[1], TOP + 0.12), yaw=gy, duration=0.8)


def _tuck(Rt, ctx, k, tries=3):
    """Move any part of lead k lying outside the tray to just inside it."""
    from yam_arena.constants import FINGER_CLOSED

    cx, cy = (TRAY_X0 + TRAY_X1) / 2, (TRAY_Y0 + TRAY_Y1) / 2
    for _ in range(tries):
        c = ctx.states()[CORDS[k].name]
        if in_tray(c):
            return
        pts = np.vstack([p[None] for p in (c.start_plug, c.end_plug)] + [c.points])
        out = np.maximum.reduce([TRAY_X0 + 0.01 - pts[:, 0], pts[:, 0] - TRAY_X1 + 0.01,
                                 TRAY_Y0 + 0.01 - pts[:, 1], pts[:, 1] - TRAY_Y1 + 0.01])
        i = int(np.argmax(out))
        q = pts[i]
        print(f"[tuck] worst idx={i} out={out[i]:.3f} at {np.round(q, 3)} n_out={(out > -0.007).sum()}", flush=True)
        if i < 2:  # a plug: pinch it
            v = (c.points[0] - q) if i == 0 else (c.points[-1] - q)
        else:
            j = min(i - 2, len(c.points) - 2)
            v = c.points[j + 1] - c.points[j]
        gy = _grip_yaw(math.atan2(v[1], v[0]))
        tgt = np.array([np.clip(q[0], TRAY_X0 + 0.03, TRAY_X1 - 0.03), np.clip(q[1], TRAY_Y0 + 0.03, TRAY_Y1 - 0.03)])
        gz = q[2] + (0.004 if i < 2 else 0.010)
        yield from Rt.move((q[0], q[1], HI), yaw=gy, finger=FINGER_PINCH if i < 2 else GRASP_OPEN, duration=1.0)
        yield from Rt.move((q[0], q[1], gz), yaw=gy, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((q[0], q[1], TOP + 0.05), yaw=gy, duration=0.6)
        yield from Rt.move((tgt[0], tgt[1], TOP + 0.05), yaw=gy, duration=1.0)
        yield from Rt.move((tgt[0], tgt[1], TOP + 0.03), yaw=gy, duration=0.5)
        yield from Rt.grip(GRASP_OPEN, 0.4)
        yield from Rt.move((tgt[0], tgt[1], TOP + 0.12), yaw=gy, duration=0.6)
        _log(ctx, "tuck")


def demo_script(ctx):
    from yam_arena.constants import FINGER_CLOSED

    # test only: correct | none | hold | near | pullall | flexonly | notest | cordpull | tier05
    mode = os.environ.get("WIGGLE_DEMO", "correct")
    L, Rt = ctx.left, ctx.right
    _log(ctx, "start")
    if mode == "none":
        for _ in range(4):
            yield from L.wait(1.0)
        _log(ctx, "end")
        return
    if mode == "pullall":  # take every lead out of the tester by its connector, no testing
        for k in LEADS:
            yield from _take_out(L, ctx, k)
        _log(ctx, "end")
        return
    faulty = FAULTY
    if mode != "notest":
        found = []
        for k in LEADS:
            yield from _watch(L, k)
            if (yield from _flex(Rt, ctx, k)):
                found.append(k)
        print(f"[demo] lamp went dark on: {found}", flush=True)
        faulty = found[0] if found else LEADS[0]
    if mode == "near":  # wrong, convenient lead: the one nearest the tray
        faulty = LEADS[-1]
    yield from L.move(L_PARK, yaw=0.0, duration=1.2)
    if mode == "flexonly":
        yield from Rt.move(R_PARK, yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        yield from L.wait(1.0)
        _log(ctx, "end")
        return
    if mode == "cordpull":  # right arm alone: pull the lead out by its cord as close to the connector as it reaches
        c = ctx.states()[CORDS[faulty].name]
        p, h = _cord_at(c, CL / 2 + 0.025)
        gy = _grip_yaw(h)
        yield from Rt.move((p[0], p[1], HI), yaw=gy, finger=GRASP_OPEN, duration=1.2)
        yield from Rt.move((p[0], p[1], p[2] + 0.010), yaw=gy, duration=0.8)
        yield from Rt.grip(FINGER_CLOSED, 0.5)
        yield from Rt.move((p[0], p[1] - 0.10, p[2] + 0.03), yaw=gy, duration=2.5)
        yield from Rt.grip(GRASP_OPEN, 0.4)
        yield from Rt.move((p[0], p[1] - 0.10, HI), yaw=gy, duration=0.6)
        yield from _carry(Rt, ctx, faulty, ((TRAY_X0 + TRAY_X1) / 2, (TRAY_Y0 + TRAY_Y1) / 2))
        yield from _tuck(Rt, ctx, faulty)
        yield from Rt.move(R_PARK, yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
        yield from L.wait(1.0)
        _log(ctx, "end")
        return
    yield from _take_out(L, ctx, faulty)
    _log(ctx, "taken_out")
    if mode == "hold":  # lower the lead into the tray but keep gripping it
        yield from _carry(Rt, ctx, faulty, ((TRAY_X0 + TRAY_X1) / 2, (TRAY_Y0 + TRAY_Y1) / 2), release=False)
        for _ in range(4):
            yield from Rt.wait(1.0)
        _log(ctx, "end")
        return
    if mode == "tier05":  # out of the tester, laid on the table beside the tray
        yield from _carry(Rt, ctx, faulty, (0.22, -0.22), z_lo=TOP + 0.03)
    else:
        yield from _carry(Rt, ctx, faulty, ((TRAY_X0 + TRAY_X1) / 2, (TRAY_Y0 + TRAY_Y1) / 2))
        _log(ctx, "carried")
        yield from _tuck(Rt, ctx, faulty)
    yield from Rt.move(R_PARK, yaw=0.0, finger=FINGER_CLOSED, duration=1.2)
    for _ in range(3):
        yield from L.wait(0.5)
    _log(ctx, "end")
