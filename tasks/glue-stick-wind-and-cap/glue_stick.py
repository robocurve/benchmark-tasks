"""Put the cap back on the glue stick: wind the glue down first.

A Pritt Stick 43 g lies on the table with its glue wound 12 mm out past the rim; its cap lies beside it. The cap only
seats when the glue is wound back into the barrel: the knob turns a screw (here a slide whose target follows the
knob's turn at 6 mm per turn), and glue standing out stops the cap's inner end before its rim reaches the shoulder.
The cap is a snap fit: it resists the last 6 mm with 8 N, clicks home and is held there.

Frame: x forward from the arm bases, y left, z up, metres. Every size and threshold is in layout.py.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
import layout as L  # noqa: E402
from bodykit import Body, BoxShape, CylShape, plate  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
CONTROL_HZ = 20.0

WHITE, RED, GREY, GLUE_C, MARK = (0.93, 0.93, 0.90), (0.80, 0.08, 0.08), (0.75, 0.75, 0.74), (0.98, 0.97, 0.88), (0.98, 0.98, 0.98)
SHOULDER = (L.X, L.STICK_Y, L.AXIS_Z)  # the barrel's origin at the start


def _prism(r: float, y0: float, y1: float, color, n: int = 16, wall: float = 0.002) -> list:
    """A round part along y: a cylinder that is seen, and a ring of n flat staves (outer radius r) that collides. A
    simulated cylinder lying on the table creeps sideways; a 16-sided prism rests on a face."""
    rm = r - wall / 2
    w = 2 * math.pi * rm / n * 1.08
    out = [CylShape((0, (y0 + y1) / 2, 0), r, abs(y1 - y0), color, collide=False, axis="Y")]
    for k in range(n):
        a = 2 * math.pi * k / n
        c, s = math.cos(a), math.sin(a)
        out.append(plate((rm * c, y0, rm * s), (rm * c, y1, rm * s), w, wall, (c, 0.0, s), color))
    return out


def _barrel() -> Body:
    shapes = _prism(L.BARREL_R, 0.0, L.BARREL_L, WHITE, wall=0.0065) + _prism(L.NECK_R, -L.NECK_L, 0.0, GREY)
    shapes += [CylShape((0, 0.0005, 0), L.BARREL_R - 0.001, 0.001, WHITE, axis="Y"),  # the shoulder's face
               CylShape((0, -L.NECK_L + 0.0005, 0), L.NECK_R - 0.001, 0.001, GREY, axis="Y"),  # the rim's face
               BoxShape((0, L.BARREL_L / 2, L.BARREL_R + 0.0002), (0.010, 0.050, 0.0006), RED, collide=False)]
    return Body.build("barrel", pos=SHOULDER, mass=L.BARREL_MASS, shapes=shapes, color=WHITE, friction=L.MU,
                      com=(0.0, L.BARREL_L / 2, 0.0), solver_iterations=32, damping=(0.0, L.ROLL_DAMP))


def _knob() -> Body:
    shapes = _prism(L.KNOB_R, -L.KNOB_L / 2, L.KNOB_L / 2, RED, wall=0.003)
    # a white line across the knob's end and down its side, so its turn shows on camera
    shapes += [BoxShape((0, L.KNOB_L / 2 + 0.0002, 0), (0.002, 0.0006, 2 * L.KNOB_R - 0.002), MARK, collide=False),
               BoxShape((0, 0, L.KNOB_R + 0.0002), (0.002, L.KNOB_L - 0.001, 0.0006), MARK, collide=False)]
    return Body.build("knob", pos=(L.X, L.STICK_Y + L.BARREL_L + L.KNOB_L / 2, L.AXIS_Z), mass=L.KNOB_MASS,
                      shapes=shapes, color=RED, friction=L.MU, solver_iterations=32, ignore=("barrel",))


GLUE_START = (L.X, L.STICK_Y - L.NECK_L - L.GLUE_OUT0, L.AXIS_Z)  # the glue's tip at the start


def _glue() -> Body:
    shapes = [CylShape((0, L.GLUE_L / 2, 0), L.GLUE_R, L.GLUE_L, GLUE_C, axis="Y")]
    return Body.build("glue", pos=GLUE_START, mass=L.GLUE_MASS, shapes=shapes, color=GLUE_C, friction=L.MU,
                      com=(0.0, L.GLUE_L / 2, 0.0), solver_iterations=32, ignore=("barrel", "knob"))


def _cap() -> Body:
    """A cup of 16 wall staves round the y axis and an end disc; origin at the rim's centre, open toward +y."""
    rm = L.CAP_R - L.CAP_WALL / 2
    w = 2 * math.pi * rm / L.CAP_STAVES * 1.08
    shapes = []
    for k in range(L.CAP_STAVES):
        a = 2 * math.pi * k / L.CAP_STAVES
        c, s = math.cos(a), math.sin(a)
        shapes.append(plate((rm * c, 0.0, rm * s), (rm * c, -L.CAP_DEPTH, rm * s), w, L.CAP_WALL, (c, 0.0, s), RED))
    shapes.append(CylShape((0, -L.CAP_DEPTH - L.CAP_END / 2, 0), L.CAP_R, L.CAP_END, RED, axis="Y"))
    return Body.build("cap", pos=(L.CAP_X, L.CAP_Y, TOP + L.CAP_R), mass=L.CAP_MASS, shapes=shapes, color=RED,
                      friction=L.MU, com=(0.0, -0.012, 0.0), solver_iterations=32)


OBJECTS = [_barrel(), _knob(), _glue(), _cap()]
NAMES = [o.name for o in OBJECTS]
LOOSE = tuple(NAMES)

bodykit.hinge("knob_turn", "barrel", "knob", anchor=(L.X, L.STICK_Y + L.BARREL_L, L.AXIS_Z), axis="Y",
              damping=L.KNOB_DAMP)
# the screw: the glue slides along the barrel's axis (+y = into the barrel); the step hook moves the drive's target as
# the knob turns. The drive is stiff and strong (300 N), so the cap cannot push the glue back: only the knob moves it.
bodykit.slider("screw", "barrel", "glue", anchor=GLUE_START, axis="Y",
               limits=(L.GLUE_OUT0 - L.GLUE_MAX, L.GLUE_OUT0 - L.GLUE_MIN), stiffness=1.0e5, damping=40.0,
               target=0.0, max_force=300.0)
# the cap's snap fit: off until the rim is on the axis and within 6 mm of the shoulder; then the cap slides on the axis
# against 8 N (the drive pushes it back out toward 6 mm) until it clicks within 0.7 mm, and is then held home (15 N)
bodykit.thread("fit", "barrel", "cap", anchor0=(0.0, 0.0, 0.0), axis0=(0.0, -1.0, 0.0), anchor1=(0.0, 0.0, 0.0),
               axis1=(0.0, -1.0, 0.0), slide_limits=(0.0, L.FIT_ENGAGE), turn_damping=1.0e-4, slide_stiffness=1.0e4,
               slide_damping=5.0, slide_max_force=L.FIT_PUSH)

# -- episode state ------------------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["GLUE_DEBUG"], "w") if os.environ.get("GLUE_DEBUG") else None


def _blank() -> None:
    start = _EP.get("start")
    _EP.clear()
    _EP.update(step=0, hands=None, report={}, now=None, start=start, turn=0.0, prev=None, out=L.GLUE_OUT0,
               caught=None, clicked=None, pressed=None, max_out=L.GLUE_OUT0)


_blank()


def _fit_force(value: float) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    prim = get_current_stage().GetPrimAtPath(bodykit._THREAD_PATHS["fit"])
    UsdPhysics.DriveAPI(prim, "transX").GetMaxForceAttr().Set(float(value))


def _rearm() -> None:
    bodykit.set_thread_on("fit", False)
    bodykit.set_slide_target("fit", L.FIT_ENGAGE)
    _fit_force(L.FIT_PUSH)
    bodykit.set_slider_drive("screw", target=0.0)


@bodykit.on_reset
def _reset() -> None:
    _blank()


def knob_angle(barrel, knob) -> float:
    """The knob's turn against the barrel about the barrel's axis (+y), rad, in (-pi, pi]."""
    r = barrel.rotation().T @ knob.rotation()
    return math.atan2(r[0, 2], r[0, 0])


def glue_out(barrel, glue) -> float:
    """How far (m) the glue's tip stands past the rim (negative: below it)."""
    return -L.NECK_L - float(barrel.to_local(np.asarray(glue.pos, float))[1])


def cap_on(barrel, cap) -> dict:
    """The cap's rim in the barrel's frame: its gap to the shoulder along the axis, its distance off the axis, the
    angle between the two axes, and where its inner end is along the axis."""
    loc = barrel.to_local(np.asarray(cap.pos, float))
    ya, yb = barrel.rotation()[:, 1], cap.rotation()[:, 1]
    ang = math.degrees(math.acos(max(-1.0, min(1.0, float(ya @ yb)))))
    inner = barrel.to_local(np.asarray(cap.pos, float) + cap.rotation() @ np.array([0.0, -L.CAP_DEPTH, 0.0]))
    return {"gap": -float(loc[1]), "off": float(math.hypot(loc[0], loc[2])), "tilt": ang, "inner_y": float(inner[1]),
            "inner_off": float(math.hypot(inner[0], inner[2]))}


def seated(c: dict) -> bool:
    return c["gap"] <= L.SEATED_GAP and c["off"] <= L.SEATED_OFF and c["tilt"] <= L.SEATED_TILT


def pressing(c: dict, barrel, glue) -> bool:
    """The cap's inner end is at (or past) the glue's tip, with the glue inside the cap."""
    tip_y = float(barrel.to_local(np.asarray(glue.pos, float))[1])
    return c["tilt"] <= 30.0 and c["inner_off"] < L.CAP_IN_R and c["inner_y"] >= tip_y - L.TOUCH and c["gap"] < 0.08


def finger_points(p, r, half: float) -> list:
    """Points on both fingers of a gripper (TCP at p, rotation r, pads half m from the centre line)."""
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> list | None:
    """Finger points of both grippers, or None if any reading is missing or not finite (the release rule then fails)."""
    from bodykit import as_numpy
    from yam_arena import ik, station

    pts = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        pts += finger_points(p, r, half)
    if not pts or not all(np.all(np.isfinite(q)) for q in pts):
        return None
    return pts


def clearance(barrel, cap, pts) -> float:
    """Smallest distance (m) from the points to the stick (a cylinder along its axis, glue and knob included) and to
    the cap (a cylinder along its own axis)."""
    best = float("inf")
    for state, r, y0, y1 in ((barrel, L.BARREL_R, -L.NECK_L - max(0.0, _EP["out"]), L.BARREL_L + L.KNOB_L),
                             (cap, L.CAP_R, -L.CAP_DEPTH - L.CAP_END, 0.0)):
        for p in pts:
            loc = state.to_local(p)
            dr = max(0.0, math.hypot(loc[0], loc[2]) - r)
            dy = max(0.0, y0 - loc[1], loc[1] - y1)
            best = min(best, math.hypot(dr, dy))
    return best


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["now"] = st
    k = _EP["step"]
    b, kn, g, c = st["barrel"], st["knob"], st["glue"], st["cap"]
    try:
        _EP["hands"] = _hand_points(env.unwrapped)
    except Exception as err:  # noqa: BLE001  (fail closed: no reading counts as not released)
        _EP["hands"] = None
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    if _EP.get("start") is None:
        _EP["step"] += 1
        return
    # the screw: the glue's target follows the knob's turn, 6 mm per turn, within the screw's travel
    a = knob_angle(b, kn)
    if _EP["prev"] is not None:
        d = (a - _EP["prev"] + math.pi) % (2 * math.pi) - math.pi
        _EP["turn"] += d
        _EP["out"] = min(L.GLUE_MAX, max(L.GLUE_MIN, _EP["out"] + L.WIND_OUT * L.PITCH * d / (2 * math.pi)))
        bodykit.set_slider_drive("screw", target=L.GLUE_OUT0 - _EP["out"])
    _EP["prev"] = a
    out = glue_out(b, g)
    _EP["max_out"] = max(_EP["max_out"], out)
    # the snap fit
    con = cap_on(b, c)
    if _EP["caught"] is None and 0.0 < con["gap"] <= L.FIT_ENGAGE + 0.0005 and con["off"] <= 0.0015 \
            and con["tilt"] <= 6.0:
        bodykit.set_thread_on("fit", True)
        _EP["caught"] = k
    if _EP["caught"] is not None and _EP["clicked"] is None and con["gap"] <= L.FIT_CLICK:
        bodykit.set_slide_target("fit", 0.0)
        _fit_force(L.FIT_HOLD)
        _EP["clicked"] = k
    if _EP["pressed"] is None and k >= 2 and pressing(con, b, g):
        _EP["pressed"] = (k, round(out, 4), round(con["gap"], 4))
    _EP["step"] += 1
    if _DBG is not None and k % 4 == 0:
        _debug(st, con, out)


# -- scoring --------------------------------------------------------------------------------------------------------
def report(objects) -> dict:
    out: dict = {"score": 0.0}
    if _EP.get("start") is None:
        return out
    b, g, c, kn = objects["barrel"], objects["glue"], objects["cap"], objects["knob"]
    con = cap_on(b, c)
    pts = _EP.get("hands")
    clear = clearance(b, c, pts) if pts is not None else -1.0
    lowest = min(float(b.low[2]), float(kn.low[2])) - TOP
    tip = glue_out(b, g)
    out.update(cap={k: round(v, 4) for k, v in con.items()}, glue_out_m=round(tip, 4), turns=round(_EP["turn"] / (2 * math.pi), 2),
               pressed=_EP["pressed"], caught_at=_EP["caught"], clicked_at=_EP["clicked"],
               stick_lowest_above_table_m=round(lowest, 4), hands_clear_m=round(clear, 3))
    if _EP["pressed"] is not None or tip > 0.0 or lowest > L.ON_TABLE or clear < L.HANDS_CLEAR:
        return out
    out["score"] = L.POINTS["full"] if seated(con) else L.POINTS["wound"]
    return out


def score(objects) -> float:
    """0 if the cap's inner end ever pressed on the glue's tip; 0 unless the glue's tip ends at or below the rim, the
    stick rests on the table (lowest point within 4 mm) and every finger point is 3 cm or more from the stick and the
    cap. Then 1.0 if the cap is seated (rim within 1 mm of the shoulder, on the axis), else 0.5."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return max(0.0, cap_on(objects["barrel"], objects["cap"])["gap"])


def _debug(st, con, out) -> None:
    b, c = st["barrel"], st["cap"]
    rep = _EP.get("report") or {}
    _DBG.write(f"{_EP['step']:4d} barrel=({b.pos[0]:.4f},{b.pos[1]:.4f},{b.pos[2]:.4f}) t={_EP['turn']:.2f} "
               f"set={_EP['out']:.4f} out={out:.4f} cap=({c.pos[0]:.4f},{c.pos[1]:.4f},{c.pos[2]:.4f}) "
               f"gap={con['gap']:.4f} off={con['off']:.4f} ct={con['tilt']:.1f} in={con['inner_y']:.4f} "
               f"caught={_EP['caught']} click={_EP['clicked']} pressed={_EP['pressed']} score={rep.get('score')}\n")
    _DBG.flush()


# -- the fixed start ------------------------------------------------------------------------------------------------
@bodykit.on_reset_env
def _lay_out(emb, seed):
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    base = emb._base
    _rearm()
    for o in OBJECTS:
        obj = base.scene[o.name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.array(o.pos) + np.array([0, 0, 0.0005]), dtype=pose.dtype,
                                   device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor((0.0, 0.0, 0.0, 1.0), dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(max(1.0, getattr(ie, "SETTLE_S", 0.5)) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    st = _scene.object_states(emb._env, NAMES)
    _EP["start"] = {n: (np.asarray(st[n].pos, float).copy(), np.asarray(st[n].quat_xyzw, float).copy()) for n in LOOSE}
    _blank()
    if _DBG is not None:
        _DBG.write("start " + " ".join(f"{n}=({v[0][0]:.4f},{v[0][1]:.4f},{v[0][2]:.4f})"
                                       for n, v in _EP["start"].items()) + "\n")
    return emb._observation()


# -- the scripted demo (reads the stick's and cap's poses from the simulator to plan; moves them only through the
#    grippers, by contact). The stick never leaves the table: the right hand pinches the barrel down on it, the left
#    hand's closed fingertips roll the knob over its top (as a thumb would), then the left hand holds the barrel and
#    the right hand slides the cap along the table onto the neck. ----------------------------------------------------
VARIANT = os.environ.get("DEMO_VARIANT", "")  # "", careless (cap pushed straight on), no_cap (wound, not capped), nothing
TIP = 0.013  # fingertip ahead of the TCP along the tool axis
GRIP_Y = 0.012  # the right hand pinches the barrel 12 mm from the shoulder (the two wrists 8 cm apart)
HOLD_Y = float(os.environ.get("GLUE_HOLD_Y", "0.062"))  # the left hand pinches it 62 mm from the shoulder while the cap goes on
KNOB_TIP_Y = 0.003  # the fingertips press the knob 3 mm beyond its middle, clear of the barrel's end
PRESS = 0.0045  # the fingertips aim this far inside the knob's surface
SWEEP = math.radians(45.0)  # each stroke rolls the fingertips over the knob's top from -45 to +45 degrees
TILT = 0.3  # the tool leans 0.3 of that angle toward the knob's axis (more is out of the wrist's range)
STROKES = 0 if VARIANT == "careless" else int(os.environ.get("GLUE_STROKES", "10"))
F_BARREL = -(L.BARREL_R + 0.010)  # finger opening (per finger, m) round the barrel
F_CAP = -(L.CAP_R + 0.010)
B_HOLD = -(L.BARREL_R - 0.004)  # the left fingers close to 11 mm on the barrel: a full close pushes into its wall
PRESS_CAP = 0.001  # the right hand's closed fingertips press the seated cap 1 mm down while the left hand lets go
CAP_HOLD = -(L.CAP_R - float(os.environ.get("GLUE_CAP_SQ", "0.005")))  # the fingers close to 10.5 mm on the cap: a full close pushes through its 1.5 mm wall
LOOKS = STROKES + 6  # the pinch, the strokes, the hand-over, the cap's three stages, the release
AIM_GAP = L.NECK_L + 0.016  # the cap's rim is lined up 32 mm from the shoulder: 4 mm beyond the standing glue
UP = np.array([0.0, 0.0, 1.0])


def _wp(sec, pos, f, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(float(v) for v in pos), float(f), label, np.asarray(rot, float))


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# the two grippers' housings are wide: straight down, they meet when the hands are 9 cm apart along the stick. Each
# tool leans away from the other hand in the vertical plane along the stick (pads still closing along x)
LEAN_R = math.radians(45.0)  # the right tool points down and toward +y: its housing sits toward -y
LEAN_L = math.radians(20.0)  # the left tool points down and toward -y: its housing sits toward +y


def down_rot() -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return _rx(LEAN_R) @ grip_rotation(math.pi / 2)  # the right hand: pads closing along x


LEAN_HOLD = math.radians(35.0)  # the left tool's lean while it holds the barrel for the cap


def left_down(lean: float = LEAN_L) -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return _rx(-lean) @ grip_rotation(-math.pi / 2)  # the left hand's pinch (pads along x, wrist in range)


def cap_rot() -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return grip_rotation(math.pi / 2)  # the right hand on the cap: straight down (a leaning pinch tips the cap)


def tip_rot(th: float) -> np.ndarray:
    """The left tool, pads closing along x, leaning toward the knob's axis from th rad past its top (positive
    toward +x)."""
    return _ry(TILT * th) @ left_down()


def ready_pose(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, sec: float, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return _wp(sec, p, FINGER_OPEN, label, r)


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


def _pad(plans: dict, keep: str = "hold") -> dict:
    """Make both arms' plans equally long, the shorter one keeping its last pose."""
    t = max(_dur(ws) for ws in plans.values())
    for arm, ws in plans.items():
        gap = t - _dur(ws)
        if gap > 1e-9:
            last = ws[-1] if ws else _EP["last"][arm]
            idle = any(w in last.label for w in ("let go", "back off", "ready pose", "wait"))
            ws.append(Waypoint(gap, last.pos, last.finger, "wait" if idle else keep, last.yaw))
    for arm, ws in plans.items():
        _EP["last"][arm] = ws[-1]
    return plans


def knob_tcp(barrel_pos, barrel_rot, th: float, rho: float) -> np.ndarray:
    """The left TCP with its fingertips rho m from the knob's axis, th rad past its top."""
    axis = np.asarray(barrel_pos, float) + np.asarray(barrel_rot, float) @ np.array(
        [0.0, L.BARREL_L + L.KNOB_L / 2 + KNOB_TIP_Y, 0.0])
    return axis + rho * np.array([math.sin(th), 0.0, math.cos(th)]) - TIP * tip_rot(th)[:, 2]


def _barrel_pinch(barrel_pos, barrel_rot, along: float, rot) -> np.ndarray:
    """The TCP 4 mm back from the barrel's axis along the tool (the pads on the barrel's sides)."""
    return np.asarray(barrel_pos, float) + np.asarray(barrel_rot, float) @ np.array([0.0, along, 0.0]) - \
        0.004 * np.asarray(rot, float)[:, 2]


def stage0(start) -> dict:
    """The right hand pinches the barrel from above where it lies; the left hand comes over the knob."""
    rd = down_rot()
    bp, br = start["barrel"][0], np.eye(3)
    grip = _barrel_pinch(bp, br, GRIP_Y, rd)
    right = [_wp(2.5, grip + 0.06 * UP, F_BARREL, "over the barrel", rd),
             _wp(1.5, grip, F_BARREL, "down round the barrel", rd),
             _wp(0.8, grip, 0.0, "grip the barrel", rd),
             _wp(0.6, grip, 0.0, "hold the stick", rd)]
    rho = L.KNOB_R
    left = [_wp(2.5, knob_tcp(bp, br, -SWEEP, rho + 0.03), 0.0, "toward the knob", tip_rot(-SWEEP)),
            _wp(2.0, knob_tcp(bp, br, -SWEEP, rho + 0.012), 0.0, "over the knob", tip_rot(-SWEEP))]
    return _pad({"right": right, "left": left}, "hold the stick")


def stroke(now, k: int) -> dict:
    """Stroke k: the closed fingertips press on the knob at -55 degrees, roll over its top to +55 (the knob turns
    with them, winding the glue in), lift off and come back over the top. Aimed at the knob where it now is."""
    b = now["barrel"]
    bp, br = np.asarray(b.pos, float), b.rotation()
    rho = L.KNOB_R - PRESS
    left = [_wp(0.3, knob_tcp(bp, br, -SWEEP, rho + 0.012), 0.0, "over the knob", tip_rot(-SWEEP)),
            _wp(0.4, knob_tcp(bp, br, -SWEEP, rho), 0.0, "grip the knob", tip_rot(-SWEEP))]
    n = 10
    for i in range(1, n + 1):
        th = -SWEEP + 2 * SWEEP * i / n
        left.append(_wp(0.18, knob_tcp(bp, br, th, rho), 0.0, "turn the knob", tip_rot(th)))
    left += [_wp(0.3, knob_tcp(bp, br, SWEEP, rho + 0.012), 0.0, "clear of the knob", tip_rot(SWEEP)),
             _wp(0.4, knob_tcp(bp, br, 0.0, rho + 0.020), 0.0, "clear of the knob", tip_rot(0.0)),
             _wp(0.4, knob_tcp(bp, br, -SWEEP, rho + 0.012), 0.0, "over the knob", tip_rot(-SWEEP))]
    return _pad({"left": left, "right": []}, "hold the stick")


def handover(now) -> dict:
    """The left hand leaves the knob and pinches the barrel near it; the right hand lets go of the barrel and rises
    (no_cap: both hands then let go and leave)."""
    rd = down_rot()
    ld = left_down(LEAN_HOLD)
    b = now["barrel"]
    hold = np.asarray(_EP["last"]["right"].pos, float)
    right = [_wp(0.8, hold, F_BARREL, "let go of the barrel", rd),
             _wp(1.2, hold + 0.06 * UP, F_BARREL, "back off up", rd)]
    grip = _barrel_pinch(b.pos, b.rotation(), HOLD_Y, ld)
    left = [_wp(1.0, np.asarray(_EP["last"]["left"].pos, float) + 0.04 * UP, F_BARREL, "off the knob", ld),
            _wp(1.5, grip + 0.05 * UP, F_BARREL, "over the barrel", ld),
            _wp(1.5, grip, F_BARREL, "down round the barrel", ld),
            _wp(0.8, grip, B_HOLD, "grip the barrel", ld),
            _wp(0.4, grip, B_HOLD, "hold the barrel", ld)]
    if VARIANT == "no_cap":
        right.append(park("right", 2.0))
        left = left[:1] + [_wp(1.0, np.asarray(_EP["last"]["left"].pos, float) + 0.08 * UP, FINGER_OPEN,
                                "back off up", ld), park("left", 2.0)]
    return _pad({"right": right, "left": left}, "hold the barrel")


def _cap_target(now, gap: float) -> np.ndarray:
    """The right hand's TCP that puts the cap's rim on the barrel's axis, ``gap`` m from the shoulder (now measured);
    0.5 mm high, where the cap's own axis rests on the table."""
    b, c = now["barrel"], now["cap"]
    loc = b.to_local(np.asarray(c.pos, float))
    shift = b.rotation() @ np.array([-loc[0], -gap - loc[1], -loc[2]]) + np.array([0.0, 0.0, 0.0005])
    last = np.asarray(_EP["last"]["right"].pos, float)
    print(f"gluedemo: cap rim {1000 * loc[0]:.1f}, {1000 * -loc[1]:.1f}, {1000 * loc[2]:.1f} mm (x, gap, z) from the "
          f"shoulder; shift ({1000 * shift[0]:.1f}, {1000 * shift[1]:.1f}, {1000 * shift[2]:.1f}) mm", flush=True)
    return last + shift


def stage_cap(now) -> dict:
    """The right hand pinches the cap where it lies, lifts it 1 mm and brings it along the table to the glue end, its
    rim 32 mm from the shoulder, on the stick's axis."""
    rd = cap_rot()
    c0 = np.asarray(now["cap"].pos, float)
    grip = c0 + np.array([0.0, -(L.CAP_DEPTH + L.CAP_END) / 2 - 0.003, 0.0]) - 0.003 * rd[:, 2]
    b = now["barrel"]
    rim_goal = np.asarray(b.pos, float) + b.rotation() @ np.array([0.0, -AIM_GAP, 0.0]) + np.array([0, 0, 0.001])
    carry = rim_goal + (grip - c0)
    right = [_wp(2.0, grip + 0.06 * UP, F_CAP, "over the cap", rd),
             _wp(1.5, grip, F_CAP, "down round the cap", rd),
             _wp(0.8, grip, CAP_HOLD, "pinch the cap", rd),
             _wp(0.4, grip, CAP_HOLD, "pinch the cap", rd),
             _wp(0.6, np.array([grip[0], grip[1], carry[2]]), CAP_HOLD, "lift the cap", rd),
             _wp(2.0, carry, CAP_HOLD, "bring the cap to the glue end", rd),
             _wp(0.6, carry, CAP_HOLD, "hold the cap", rd)]
    return _pad({"right": right, "left": []}, "hold the barrel")


def stage_line_up(now) -> dict:
    """Line the cap up with the axis from the measured poses, its rim still 32 mm from the shoulder."""
    rd = cap_rot()
    aim = _cap_target(now, AIM_GAP)
    right = [_wp(1.0, aim, CAP_HOLD, "line the cap up", rd), _wp(0.8, aim, CAP_HOLD, "line the cap up", rd)]
    return _pad({"right": right, "left": []}, "hold the barrel")


def stage_push(now) -> dict:
    """Line up once more and push the cap along the axis to 6 mm past the shoulder (it stops at the shoulder, or on the
    glue), hold, let go and leave upward."""
    rd = cap_rot()
    aim = _cap_target(now, AIM_GAP)
    b = now["barrel"]
    push = aim + b.rotation() @ np.array([0.0, AIM_GAP + 0.006, 0.0])
    right = [_wp(0.6, aim, CAP_HOLD, "line the cap up", rd),
             _wp(3.5, push, CAP_HOLD, "push the cap home", rd),
             _wp(1.0, push, CAP_HOLD, "push the cap home", rd),
             _wp(0.8, push, F_CAP, "let go of the cap", rd),
             _wp(1.2, push + 0.06 * UP, F_CAP, "back off up", rd)]
    return _pad({"right": right, "left": []}, "hold the barrel")


def stage_release(now) -> dict:
    """The right hand's closed fingertips press the capped stick down on the table (on the cap's top, as measured);
    the left hand opens fully, waits and lifts straight up; then the right hand lifts and both go back."""
    ld, rd = left_down(LEAN_HOLD), cap_rot()
    c = now["cap"]
    top = np.asarray(c.pos, float) + c.rotation() @ np.array([0.0, -(L.CAP_DEPTH + L.CAP_END) / 2, 0.0])
    top[2] += L.CAP_R + TIP - PRESS_CAP
    hold = np.asarray(_EP["last"]["left"].pos, float)
    right = [_wp(1.5, top + 0.03 * UP, 0.0, "close over the cap", rd),
             _wp(1.0, top, 0.0, "press the stick down", rd),
             _wp(4.6, top, 0.0, "press the stick down", rd),
             _wp(1.2, top + 0.06 * UP, 0.0, "back off up", rd), park("right", 2.0)]
    left = [_wp(2.5, hold, B_HOLD, "hold the barrel", ld),
            _wp(1.0, hold, FINGER_OPEN, "let go of the barrel", ld),
            _wp(0.8, hold, FINGER_OPEN, "let go of the barrel", ld),
            _wp(1.8, hold + 0.08 * UP, FINGER_OPEN, "back off up", ld), park("left", 2.0)]
    return _pad({"left": left, "right": right})


def plan_stage(stage: int, img, module) -> dict:
    if VARIANT == "nothing":
        return {}
    now = _EP["now"]
    if stage == 0:
        start = _EP["start"]
        _EP["last"] = {}
        print(f"gluedemo: variant '{VARIANT or 'full'}'; stick at ({start['barrel'][0][0]:.4f}, "
              f"{start['barrel'][0][1]:.4f}), cap at ({start['cap'][0][0]:.4f}, {start['cap'][0][1]:.4f})", flush=True)
        return stage0(start)
    if stage <= STROKES:
        if stage > 1:
            print(f"gluedemo: after stroke {stage - 1}: {_EP['turn'] / (2 * math.pi):.2f} turns, glue "
                  f"{1000 * _EP['out']:.1f} mm out", flush=True)
        return stroke(now, stage - 1)
    rest = (handover,) if VARIANT == "no_cap" else (handover, stage_cap, stage_line_up, stage_push, stage_release)
    i = stage - STROKES - 1
    return rest[i](now) if i < len(rest) else {}


def demo():
    return {arm: [park(arm, 1.0, "look")] for arm in ("left", "right")}


import gluedemo  # noqa: E402  (the demo's control hooks for yam_scripted)

gluedemo.install()
gluedemo_task = True
