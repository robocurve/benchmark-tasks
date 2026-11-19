"""A closed hard-shell carry-on lying on its back on the table, its built-in TSA zip lock facing the robot.

The zip runs round the case between its two rigid shells; on the side facing the robot it runs straight along the
case, at mid-height. The lock sits in the middle of that run. Two zip sliders are a few centimetres short of the
lock, one on each side, each with a loose puller tab hinged to it. Each end of the lock has a slot that takes one
puller; a spring catch in the slot drops into the hole near the puller's tip when the puller is pushed fully in.
The task: bring both sliders to the lock and seat both pullers in their slots so each catch holds, then let go.

Frame: x forward from the arm bases, y left, z up, metres. The case frame: x into the case (away from the robot at
yaw 0), y along the zip, z up; its origin is the middle of the case.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies and joints are simulated)
from bodykit import Body, BoxShape, CylShape, plate  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
CONTROL_HZ = 20.0
SIDES = ("a", "b")  # a: left of the lock as the robot sees it (+y), b: right (-y)
S = {"a": 1.0, "b": -1.0}  # which way along the case's y each side lies from the lock
ARM = {"a": "left", "b": "right"}
PROBE = os.environ.get("ZIP_PROBE", "")  # "m1": no arm motion; scripted pushes and pulls on the parts (README)
FAIL = os.environ.get("ZIP_FAIL", "")  # a wrong outcome on purpose (README): fixed, short, crooked, holding

# -- sizes (m) -----------------------------------------------------------------------------------------------------------
CASE = (0.35, 0.55, 0.23)  # depth (x, toward the robot is -x), length along the zip (y), height (z)
CASE_MASS = 2.8
F = -CASE[0] / 2  # the near side's surface in the case frame
CASE_Z = TOP + CASE[2] / 2  # the zip runs at the case's mid-height
LOCK_Y, LOCK_X, LOCK_Z = 0.035, 0.018, 0.022  # lock housing: half length along the zip, standout, half height
SLOT_IN, SLOT_OUT = F - 0.008, F - 0.012  # slot: its inner and outer walls (x); 4 mm wide
SLOT_HZ = 0.007  # slot half height: 14 mm
SLOT_BOTTOM = 0.009  # the slot runs from the housing's end (LOCK_Y) in to here: 26 mm deep
MOUTH = 0.004  # the slot's mouth: its first 4 mm are 1.5 mm wider each way (a lead-in step)
SLIDER = (0.008, 0.030, 0.016)  # zip slider: standout (x), length (y), height (z); its middle is the puller's pivot
SLIDER_FRONT = SLIDER[1] / 2  # the slider's front from its pivot; at the end stop it touches the housing
PIV_X = F - 0.010  # the puller tab's mid-plane (x)
TAB_L, TAB_W, TAB_T = 0.035, 0.012, 0.0025  # puller tab: pivot to tip, width (z), thickness
TAB_ROOT = 0.004  # the tab's ring end reaches this far behind the pivot
HOLE = (0.026, 0.031, 0.0025)  # the hole: from, to (along the tab from the pivot), half width (z): 5 x 5 mm
PL_Y = SLOT_BOTTOM + 0.0125  # the catch (plunger) centre along the zip: 21.5 mm from the lock's middle
PL_HALF = 0.002  # plunger: 4 x 4 mm
PL_REST_TIP = F - 0.0085  # the plunger's tip at rest: 3.5 mm into the 4 mm slot
PL_TRAVEL = 0.0035
PL_K, PL_PRE = 500.0, 0.004  # N/m and preload stretch: 2.0 N seated, 3.4 N fully lifted
RAMP_RUN = 0.006  # the catch's ramp rises 3.5 mm over this run along the slot (30 degrees)
LOCK_DEG = 90.0  # the tab's hinge angle (times S) at which it lies flat pointing into its slot
TAB_LIMIT = 90.5  # the tab lies flat on the slider at 90 degrees either way
U_MAX = 0.20  # slider travel away from the stop modelled (m)
U_NOMINAL = 0.05  # where the sliders are authored (their joints' zero)
MASS = {"slider": 0.012, "puller": 0.006, "plunger": 0.003}
ZIP_DAMPING, ZIP_FRICTION = 1.0e4, 3.0  # a drive that holds the slider still up to 3 N: the force to draw it
TAB_DAMPING = 1.0e-5  # N m s/deg: the tab stays where it is left

SHELL = (0.13, 0.30, 0.36)
BLACK = (0.08, 0.08, 0.09)
GREY = (0.35, 0.36, 0.38)
SILVER = (0.78, 0.79, 0.80)
RED = (0.75, 0.12, 0.10)
CASE0 = np.array([0.30 - F, 0.0, CASE_Z])  # the nominal case centre: its near side at x 0.30


def _box(lo, hi, color, collide: bool = True) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _case_shapes() -> list:
    """The shell (one rigid body) with the lock housing and its two slots, in the case frame."""
    sh = [_box((F, -CASE[1] / 2, -CASE[2] / 2), (-F, CASE[1] / 2, CASE[2] / 2), SHELL)]
    # zip: the two tooth rows along the near side (drawn, not simulated) and the carry handle on top
    for z in (-0.002, 0.002):
        sh.append(_box((F - 0.002, -CASE[1] / 2 + 0.04, z - 0.0015), (F, CASE[1] / 2 - 0.04, z + 0.0015), BLACK, False))
    sh.append(_box((-0.03, -0.09, CASE[2] / 2), (0.03, 0.09, CASE[2] / 2 + 0.025), BLACK, False))
    # the housing's middle, with three dials on its face (drawn)
    sh.append(_box((F - LOCK_X, -SLOT_BOTTOM, -LOCK_Z), (F, SLOT_BOTTOM, LOCK_Z), BLACK))
    for k in (-1, 0, 1):
        sh.append(CylShape((F - LOCK_X - 0.001, 0.0, k * 0.011), 0.0045, 0.002, RED, collide=False, axis="X"))
    for s in (1.0, -1.0):
        def yb(a, b):
            return (min(s * a, s * b), max(s * a, s * b))
        ym0, ym1 = yb(SLOT_BOTTOM, LOCK_Y - MOUTH)  # the slot proper
        yq0, yq1 = yb(LOCK_Y - MOUTH, LOCK_Y)  # its mouth
        # inner wall (against the shell), outer wall (toward the robot), top and bottom walls
        sh.append(_box((SLOT_IN, ym0, -LOCK_Z), (F, ym1, LOCK_Z), BLACK))
        sh.append(_box((SLOT_IN + 0.0015, yq0, -LOCK_Z), (F, yq1, LOCK_Z), BLACK))
        for zs in (1.0, -1.0):
            z0, z1 = sorted((zs * SLOT_HZ, zs * LOCK_Z))
            sh.append(_box((SLOT_OUT, ym0, z0), (SLOT_IN, ym1, z1), BLACK))
            z0, z1 = sorted((zs * (SLOT_HZ + 0.0015), zs * LOCK_Z))
            sh.append(_box((SLOT_OUT - 0.0015, yq0, z0), (SLOT_IN + 0.0015, yq1, z1), BLACK))
        # outer wall (toward the robot), with a 5 x 5 mm window for the catch's plunger, and its stepped mouth
        for a, b in ((SLOT_BOTTOM, PL_Y - 0.0025), (PL_Y + 0.0025, LOCK_Y - MOUTH)):
            p0, p1 = yb(a, b)
            sh.append(_box((F - LOCK_X, p0, -LOCK_Z), (SLOT_OUT, p1, LOCK_Z), BLACK))
        w0, w1 = yb(PL_Y - 0.0025, PL_Y + 0.0025)
        for zs in (1.0, -1.0):
            z0, z1 = sorted((zs * 0.0025, zs * LOCK_Z))
            sh.append(_box((F - LOCK_X, w0, z0), (SLOT_OUT, w1, z1), BLACK))
        sh.append(_box((F - LOCK_X, yq0, -LOCK_Z), (SLOT_OUT - 0.0015, yq1, LOCK_Z), BLACK))
        # the mouth's four faces taper (20 degrees) from 1.5 mm wider at the housing's end to the slot's size 4 mm in,
        # so a tip a little off line slides in instead of catching on a step
        y_end, y_in = s * LOCK_Y, s * (LOCK_Y - MOUTH)
        zc, xc, xw = 0.0, (SLOT_OUT + SLOT_IN) / 2, SLOT_IN - SLOT_OUT + 0.003
        for x_end, x_in, nx in ((SLOT_OUT - 0.0015, SLOT_OUT, -1.0), (SLOT_IN + 0.0015, SLOT_IN, 1.0)):
            off = np.array([nx * 0.0005, 0.0, 0.0])
            sh.append(plate(tuple((np.array([x_end, y_end, zc]) + off).tolist()),
                            tuple((np.array([x_in, y_in, zc]) + off).tolist()), 2 * LOCK_Z, 0.001, (nx, 0.0, 0.0), BLACK))
        for zs in (1.0, -1.0):
            off = np.array([0.0, 0.0, zs * 0.0005])
            sh.append(plate(tuple((np.array([xc, y_end, zs * (SLOT_HZ + 0.0015)]) + off).tolist()),
                            tuple((np.array([xc, y_in, zs * SLOT_HZ]) + off).tolist()), xw, 0.001, (0.0, 0.0, zs), BLACK))
    return sh


def _slider_shapes() -> list:
    """A zip slider (origin: the puller's pivot, on the slider's middle)."""
    return [_box((F - SLIDER[0] - PIV_X, -SLIDER[1] / 2, -SLIDER[2] / 2), (F - PIV_X, SLIDER[1] / 2, SLIDER[2] / 2), GREY),
            _box((F - SLIDER[0] - PIV_X - 0.0006, -0.003, -0.004), (F - SLIDER[0] - PIV_X, 0.003, 0.004), SILVER, False)]


def _puller_shapes() -> list:
    """The puller tab at hinge angle 0: it points straight out from the case (-x), origin on the pivot."""
    t = TAB_T / 2
    return [_box((-HOLE[0], -t, -TAB_W / 2), (TAB_ROOT, t, TAB_W / 2), SILVER),
            _box((-HOLE[1], -t, HOLE[2]), (-HOLE[0], t, TAB_W / 2), SILVER),
            _box((-HOLE[1], -t, -TAB_W / 2), (-HOLE[0], t, -HOLE[2]), SILVER),
            _box((-TAB_L, -t, -TAB_W / 2), (-HOLE[1], t, TAB_W / 2), SILVER)]


def _plunger_shapes(s: float) -> list:
    """The catch's plunger at rest (origin: its tip's middle). Flat on the slot-bottom side, so a seated puller cannot
    be drawn back out; a 45 degree ramp on the mouth side, so the puller's tip lifts it on the way in."""
    stem = _box((F - LOCK_X - 0.0005 - PL_REST_TIP, -PL_HALF, -PL_HALF), (SLOT_OUT - PL_REST_TIP, PL_HALF, PL_HALF), BLACK)
    lo, hi = sorted((-s * PL_HALF, 0.0))
    hook = _box((-0.0035, lo, -PL_HALF), (0.0, hi, PL_HALF), BLACK)
    # ramp: its face runs from the tip (x 0, y 0) to the slot's outer wall 3.5 mm toward the mouth
    a = np.array([0.0, 0.0, 0.0])
    b = np.array([-0.0035, s * RAMP_RUN, 0.0])
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([d[1], -d[0], 0.0]) * (-s)  # into the plunger, away from the mouth
    off = 0.0005 * n
    ramp = plate(tuple((a + off).tolist()), tuple((b + off).tolist()), 2 * PL_HALF, 0.001, tuple(n.tolist()), BLACK)
    return [stem, hook, ramp]


def _rz(a: float) -> np.ndarray:
    return np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])


def _zquat(a: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def pivot_local(c: str, u: float) -> np.ndarray:
    """The puller's pivot in the case frame with the slider ``u`` m short of its end stop."""
    return np.array([PIV_X, S[c] * (LOCK_Y + SLIDER_FRONT + u), 0.0])


def plunger_local(c: str) -> np.ndarray:
    return np.array([PL_REST_TIP, S[c] * PL_Y, 0.0])


def tab_dir(theta_deg: float) -> np.ndarray:
    """The tab's direction (pivot to tip) in the case frame at hinge angle ``theta_deg``."""
    t = math.radians(theta_deg)
    return np.array([-math.cos(t), -math.sin(t), 0.0])


def _build() -> list:
    objects = [Body.build("case", pos=tuple(CASE0.tolist()), mass=CASE_MASS, shapes=_case_shapes(), color=SHELL,
                          friction=(0.5, 0.45), solver_iterations=32, com=(0.0, 0.0, 0.0))]
    for c in SIDES:
        s = S[c]
        piv = CASE0 + pivot_local(c, U_NOMINAL)
        objects.append(Body.build(f"slider_{c}", pos=tuple(piv.tolist()), mass=MASS["slider"], shapes=_slider_shapes(),
                                  color=GREY, solver_iterations=32, contact_offset=0.001, ignore=("case",)))
        objects.append(Body.build(f"puller_{c}", pos=tuple(piv.tolist()), mass=MASS["puller"], shapes=_puller_shapes(),
                                  color=SILVER, friction=(0.2, 0.15), solver_iterations=32, contact_offset=0.001,
                                  ignore=(f"slider_{c}",)))
        plp = CASE0 + plunger_local(c)
        objects.append(Body.build(f"plunger_{c}", pos=tuple(plp.tolist()), mass=MASS["plunger"],
                                  shapes=_plunger_shapes(s), color=BLACK, friction=(0.15, 0.1), solver_iterations=32,
                                  contact_offset=0.001, ignore=("case",)))
        lim = (-U_NOMINAL, U_MAX - U_NOMINAL)
        bodykit.slider(f"zip_{c}", "case", f"slider_{c}", anchor=tuple(piv.tolist()), axis="Y",
                       limits=lim if s > 0 else (-lim[1], -lim[0]), stiffness=0.0, damping=ZIP_DAMPING, max_force=ZIP_FRICTION)
        bodykit.hinge(f"tab_{c}", f"slider_{c}", f"puller_{c}", anchor=tuple(piv.tolist()), axis="Z",
                      limits_deg=(-TAB_LIMIT, TAB_LIMIT), damping=TAB_DAMPING)
        bodykit.slider(f"catch_{c}", "case", f"plunger_{c}", anchor=tuple(plp.tolist()), axis="X",
                       limits=(-PL_TRAVEL, 0.0), stiffness=PL_K, damping=0.5, target=PL_PRE, max_force=50.0)
    return objects


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]


# -- reading the mechanism from poses --------------------------------------------------------------------------------
def _yaw(r: np.ndarray) -> float:
    return math.atan2(r[1, 0], r[0, 0])


def joints(st) -> dict:
    """Per side: slider distance from its stop u (m), tab angle (deg, times S: 90 lies flat toward the lock), plunger
    lift (m), tip depth in the slot (m)."""
    case = st["case"]
    rc = case.rotation()
    out = {}
    for c in SIDES:
        s = S[c]
        y = (rc.T @ (np.asarray(st[f"slider_{c}"].pos, float) - case.pos))[1]
        u = s * y - (LOCK_Y + SLIDER_FRONT)
        theta = math.degrees(_yaw(rc.T @ st[f"puller_{c}"].rotation()))
        lift = PL_REST_TIP - (rc.T @ (np.asarray(st[f"plunger_{c}"].pos, float) - case.pos))[0]
        tip = pivot_local(c, u) + TAB_L * tab_dir(theta)
        depth = LOCK_Y - s * tip[1]
        out[c] = dict(u=float(u), theta=float(s * theta), lift=float(lift), depth=float(depth))
    return out


# -- episode state ----------------------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["ZIP_DEBUG"], "w") if os.environ.get("ZIP_DEBUG") else None


def _blank() -> None:
    _EP.clear()
    _EP.update(step=0, hands={}, report={}, probe={}, st=None, start=None)


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def _hand_points(base) -> dict:
    """Points on each gripper's fingers (world), from the arm's own joint readings."""
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = {}
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out[arm] = finger_points(p, r, half)
    return out


def finger_points(p, r, half: float) -> list[np.ndarray]:
    """Points on the two fingers of a gripper whose TCP is at ``p`` with rotation ``r`` and whose pads are each
    ``half`` m from the centre line: pad tips, pad middles and finger roots."""
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["st"] = st
    if _EP["start"] is None:
        _EP["start"] = (np.asarray(st["case"].pos, float).copy(), _yaw(st["case"].rotation()))
    try:
        _EP["hands"] = _hand_points(env.unwrapped)
    except Exception as err:  # noqa: BLE001
        if _DBG is not None and not _EP.get("hand_err"):
            _DBG.write(f"hand points failed: {err!r}\n")
            _EP["hand_err"] = True
    if PROBE == "m1":
        _probe(env, st)
    _EP["step"] += 1
    if _DBG is not None:
        _debug(st)


# -- scoring ----------------------------------------------------------------------------------------------------------
STOP_MM = 2.0  # slider within this of its end stop
IN_MM = 10.0  # puller tip at least this deep in its slot
SEAT_MM = 18.0  # and this deep, with the catch down, for the catch to be in the puller's hole
LIFT_MM = 1.0  # the catch counts as down (in the hole) when lifted less than this
SQUARE_DEG = 10.0  # the tab within this of lying flat along its slot
CLEAR_MM = 30.0  # both grippers at least this far from every slider, puller and catch
MOVED_MM, TILT_DEG = 100.0, 10.0
POINTS = {"at_stop": 0.15, "in_slot": 0.15, "caught": 0.20}


def part_points(st) -> list[np.ndarray]:
    """Points on both sliders, pullers and catches (world)."""
    pts = []
    for c in SIDES:
        sl, pu = st[f"slider_{c}"], st[f"puller_{c}"]
        for y in (-SLIDER[1] / 2, 0.0, SLIDER[1] / 2):
            pts.append(np.asarray(sl.pos, float) + sl.rotation() @ np.array([F - SLIDER[0] / 2 - PIV_X, y, 0.0]))
        for l in (0.0, 0.012, 0.024, TAB_L):
            pts.append(np.asarray(pu.pos, float) + pu.rotation() @ np.array([-l, 0.0, 0.0]))
        pts.append(np.asarray(st[f"plunger_{c}"].pos, float))
    return pts


def clearance_mm(st) -> float | None:
    hands = _EP.get("hands") or {}
    if not hands:
        return None
    tp = np.array(part_points(st))
    best = float("inf")
    for pts in hands.values():
        for p in pts:
            best = min(best, float(np.linalg.norm(tp - p, axis=1).min()))
    return 1000.0 * best


def report(st) -> dict:
    j = joints(st)
    out = {"sides": {}, "clearance_mm": clearance_mm(st)}
    total = 0.0
    for c in SIDES:
        d = j[c]
        square = abs(d["theta"] - LOCK_DEG) <= SQUARE_DEG
        at_stop = d["u"] * 1000.0 <= STOP_MM
        in_slot = square and d["depth"] * 1000.0 >= IN_MM
        caught = in_slot and d["depth"] * 1000.0 >= SEAT_MM and d["lift"] * 1000.0 < LIFT_MM
        pts = POINTS["at_stop"] * at_stop + POINTS["in_slot"] * in_slot + POINTS["caught"] * caught
        out["sides"][c] = dict(u_mm=round(d["u"] * 1000, 2), tab_deg=round(d["theta"], 2),
                               depth_mm=round(d["depth"] * 1000, 2), catch_lift_mm=round(d["lift"] * 1000, 2),
                               at_stop=bool(at_stop), in_slot=bool(in_slot), caught=bool(caught), score=round(pts, 2))
        total += pts
    start = _EP.get("start")
    case = st["case"]
    moved = 0.0 if start is None else 1000.0 * float(np.linalg.norm(np.asarray(case.pos, float)[:2] - start[0][:2]))
    out["case_moved_mm"] = round(moved, 1)
    out["case_tilt_deg"] = round(math.degrees(case.tilt()), 2)
    ok_case = moved <= MOVED_MM and out["case_tilt_deg"] <= TILT_DEG
    clr = out["clearance_mm"]
    out["grippers_clear"] = bool(clr is None or clr >= CLEAR_MM)
    out["score"] = round(min(1.0, total) if ok_case and out["grippers_clear"] else 0.0, 2)
    return out


def score(objects) -> float:
    """Per side: 0.15 slider within 2 mm of its end stop; 0.15 puller lying within 10 degrees of its slot's line with
    its tip at least 10 mm in; 0.20 also at least 18 mm in with the catch down in its hole. Sum of both sides. 0 for
    the trial if the case moved over 100 mm or tipped over 10 degrees, or a gripper ends within 30 mm of a slider,
    puller or catch."""
    rep = report(objects)
    _EP["report"] = rep
    return float(rep["score"])


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    j = joints(objects)
    return float(sum(max(0.0, j[c]["u"]) + max(0.0, 0.020 - j[c]["depth"]) for c in SIDES))


# -- the seeded start -------------------------------------------------------------------------------------------------
VARY_X, VARY_Y, VARY_YAW = 0.03, 0.03, 12.0  # case: m, m, degrees
U_RANGE = (0.03, 0.07)  # each slider starts this far short of its end stop (m), drawn separately
TAB_RANGE = (15.0, 75.0)  # each tab at the start: trailing toward the lock, as left by the last pull (degrees from straight out)


def layout(seed: int | None) -> dict:
    """The start for a seed (None or 0: the nominal layout): case pose and each side's slider and tab."""
    if PROBE == "m1":
        return dict(x=0.0, y=0.0, yaw=0.0, a=(0.015, LOCK_DEG), b=(0.015, LOCK_DEG - 25.0))
    if not seed:
        return dict(x=0.0, y=0.0, yaw=0.0, a=(0.05, 45.0), b=(0.05, 45.0))
    rng = np.random.default_rng(seed)
    out = dict(x=rng.uniform(-VARY_X, VARY_X), y=rng.uniform(-VARY_Y, VARY_Y), yaw=rng.uniform(-VARY_YAW, VARY_YAW))
    for c in SIDES:
        out[c] = (rng.uniform(*U_RANGE), rng.uniform(*TAB_RANGE))
    return out


def poses(lay: dict) -> dict:
    """World poses (position, quaternion x y z w) of every body for a layout."""
    yaw = math.radians(lay["yaw"])
    r = _rz(yaw)
    centre = CASE0 + np.array([lay["x"], lay["y"], 0.0])
    out = {"case": (centre, _zquat(yaw))}
    for c in SIDES:
        u, th = lay[c]
        piv = centre + r @ pivot_local(c, u)
        out[f"slider_{c}"] = (piv, _zquat(yaw))
        out[f"puller_{c}"] = (piv, _zquat(yaw + math.radians(S[c] * th)))
        out[f"plunger_{c}"] = (centre + r @ plunger_local(c), _zquat(yaw))
    return out


@bodykit.on_reset_env
def _lay_out(emb, seed):
    """Put the bodies where this trial's seed says, let them settle, and return the first observation."""
    import torch
    from yam_arena import inspect_embodiment as ie

    base = emb._base
    lay = layout(seed)
    _blank()
    for name in NAMES:
        base.scene[name].permanent_wrench_composer.reset()
    for name, (pos, quat) in poses(lay).items():
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(np.asarray(pos, float), dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(getattr(ie, "SETTLE_S", 0.5) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    _blank()
    _EP["layout"] = lay
    if _DBG is not None:
        _DBG.write(f"seed {seed}: " + " ".join(f"{k}={v}" for k, v in lay.items()) + "\n")
    return emb._observation()


# -- M1 probe: push each slider toward the lock, then pull each puller back out --------------------------------------
PUSH_N, PULL_N = 10.0, 5.0


def _force(env, st, name: str, vec, point=None) -> None:
    import torch

    base = env.unwrapped
    dev = base.device
    obj = base.scene[name]
    obj.permanent_wrench_composer.reset()
    if vec is None:
        return
    p = np.asarray(st[name].pos, float) if point is None else np.asarray(point, float)
    pt = (torch.tensor(p, dtype=torch.float32, device=dev) + base.scene.env_origins[0]).reshape(1, 1, 3)
    f = torch.tensor(np.asarray(vec, float), dtype=torch.float32, device=dev).reshape(1, 1, 3)
    zero = torch.zeros((1, 1, 3), dtype=torch.float32, device=dev)
    obj.permanent_wrench_composer.add_forces_and_torques(f, zero, positions=pt, body_ids=[0], is_global=True)


def _probe(env, st) -> None:
    k, hz, p = _EP["step"], int(CONTROL_HZ), _EP["probe"]
    rc = st["case"].rotation()
    along = {c: rc @ np.array([0.0, -S[c], 0.0]) for c in SIDES}  # toward the lock
    j = joints(st)
    if k == hz:
        p["before"] = {c: dict(j[c]) for c in SIDES}
    if hz <= k < 3 * hz:  # 10 N on each slider toward the lock for 2 s
        for c in SIDES:
            _force(env, st, f"slider_{c}", PUSH_N * along[c])
    elif k == 3 * hz:
        for c in SIDES:
            _force(env, st, f"slider_{c}", None)
    elif k == 4 * hz:
        p["pushed"] = {c: dict(j[c]) for c in SIDES}
    if 4 * hz <= k < 5 * hz:  # 5 N on each puller's root, back out along the slot, for 1 s
        for c in SIDES:
            pu = st[f"puller_{c}"]
            _force(env, st, f"puller_{c}", -PULL_N * along[c], np.asarray(pu.pos, float))
            p.setdefault("pull_min_depth", {})[c] = min(p.get("pull_min_depth", {}).get(c, 1.0), j[c]["depth"])
    elif k == 5 * hz:
        for c in SIDES:
            _force(env, st, f"puller_{c}", None)
    elif k == 6 * hz:
        p["after"] = {c: dict(j[c]) for c in SIDES}
        line = "ZIP PROBE " + " | ".join(
            f"{c} {when}: " + " ".join(f"{q}={v * (1 if q == 'theta' else 1000):.2f}" for q, v in p[when][c].items())
            for when in ("before", "pushed", "after") for c in SIDES)
        line += " | pull min depth mm " + " ".join(f"{c}={v * 1000:.2f}" for c, v in p["pull_min_depth"].items())
        print(line, flush=True)
        if _DBG is not None:
            _DBG.write(line + "\n")


def _debug(st) -> None:
    rep = report(st)
    parts = [f"{_EP['step']:4d}"]
    for c in SIDES:
        d = rep["sides"][c]
        parts.append(f"{c}: u={d['u_mm']:6.2f} tab={d['tab_deg']:7.2f} depth={d['depth_mm']:6.2f} "
                     f"lift={d['catch_lift_mm']:5.2f} s={d['score']:.2f}")
    clr = rep["clearance_mm"]
    parts.append(f"case=({st['case'].pos[0]:.4f},{st['case'].pos[1]:.4f}) moved={rep['case_moved_mm']} "
                 f"tilt={rep['case_tilt_deg']} clr={clr if clr is None else round(clr, 1)} score={rep['score']}")
    _DBG.write(" ".join(parts) + "\n")
    _DBG.flush()


# -- the scripted demo: one side at a time, planned from the simulator's object poses ------------------------------
GRIP_L = 0.024  # the hand pinches the tab this far from its pivot
PAD_REACH = 0.013  # the pad tips are this far past the TCP along the hand's axis
PAD_HALF = 0.0077  # the pads' half width
GRIP_PINCH = -0.0035  # finger target for drawing the slider: well inside the 12 mm tab, a firm pinch
GRIP_LIGHT = -0.0055  # and for swinging the tab: just inside, so the tab can turn between the pads
GRIP_OPEN = -0.016
U_PRE = 0.019  # the slider is drawn to here (m short of its stop): the tab's tip 1 mm into the slot's mouth
OVERPUSH = 0.010  # the push is aimed this far past the end stop
STAGES = ("a", "b")
DRAW_TRIES = 2  # a side whose tab is not lying flat toward the lock after a draw is drawn again, once
LOOKS = 0 if PROBE == "m1" else len(STAGES) * (DRAW_TRIES + 1) + 1
FLAT_DEG, NEAR_U = 84.0, 0.030  # after a draw: the tab within 6 degrees of flat toward the lock, the slider this near


FLIP = {"left": -1.0, "right": 1.0}  # the hand's roll (the same grip turned half a turn), for each arm's wrist range
CLEAR = (0.17, 0.20, 1.00)  # each hand passes here (x, y times the side, z) on its way in and out, above and in front
#                             of the case's near top edge (going straight from the ready pose the hand met it)


PITCH = math.radians(15.0)  # the hand points this far below level (pointing level, it is at the end of its reach)


def hand_rot(yaw: float, arm: str = "left") -> np.ndarray:
    """The hand pointing into the case's near side (case yaw ``yaw``), PITCH below level, fingers closing up and
    down."""
    h = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    z = h * math.cos(PITCH) + np.array([0.0, 0.0, -math.sin(PITCH)])
    x = (h * math.sin(PITCH) + np.array([0.0, 0.0, math.cos(PITCH)])) * FLIP[arm]
    return np.column_stack([x, np.cross(z, x), z])


def clear_pose(arm: str, seconds: float, finger: float, label: str = "clear of the case") -> Waypoint:
    side = 1.0 if arm == "left" else -1.0
    return Waypoint(seconds, (CLEAR[0], side * CLEAR[1], CLEAR[2]), finger, label, hand_rot(0.0, arm))


def ready_pose(arm: str) -> tuple[np.ndarray, np.ndarray]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, seconds: float, finger: float = FINGER_OPEN, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return Waypoint(seconds, tuple(p.tolist()), finger, label, r)


def placeholder_plans() -> dict[str, list[Waypoint]]:
    secs = 7.0 if PROBE == "m1" else 1.0
    return {arm: [park(arm, secs, label="look")] for arm in ("left", "right")}


def demo():
    return placeholder_plans()


def _frame(c: str, case_xy, yaw: float):
    s, arm = S[c], ARM[c]
    centre = np.array([case_xy[0], case_xy[1], CASE_Z])
    r = _rz(yaw)
    rot = hand_rot(yaw, arm)

    def world(p):
        return centre + r @ np.asarray(p, float)

    return s, arm, world, rot, rot[:, 2]


def draw_plan(c: str, case_xy, yaw: float, u0: float, th0: float, fail: str = "", first: bool = True) -> list[Waypoint]:
    """Side ``c`` of a case at ``case_xy`` turned ``yaw`` (rad), its slider ``u0`` m short of the stop and its tab at
    ``th0`` degrees (times S): pinch the tab, swing it flat toward the lock about its pivot, draw the slider along the
    zip by it to just short of the lock, let go and back off."""
    s, arm, world, rot, n_in = _frame(c, case_xy, yaw)

    def grip_tcp(u: float, th: float, back: float = 0.0):
        g = pivot_local(c, u) + GRIP_L * tab_dir(s * th)
        # pad tips 1.5 mm past the tab's mid-plane: the pads cover its edge and stop short of the slider
        return tuple((world(g) - (PAD_REACH - 0.0015 + back) * n_in).tolist())

    out = [clear_pose(arm, 1.5, GRIP_OPEN)] if first else []
    out.append(Waypoint(1.5, grip_tcp(u0, th0, 0.05), GRIP_OPEN, "in front of the puller", rot))
    out.append(Waypoint(1.5, grip_tcp(u0, th0), GRIP_OPEN, "round the puller", rot))
    out.append(Waypoint(0.8, grip_tcp(u0, th0), GRIP_LIGHT, "pinch the puller", rot))
    th_goal = LOCK_DEG if fail != "crooked" else LOCK_DEG - 30.0
    n = max(2, int(math.ceil(abs(th_goal - th0) / 10.0)))
    for k in range(1, n + 1):
        out.append(Waypoint(0.4, grip_tcp(u0, th0 + (th_goal - th0) * k / n), GRIP_LIGHT, "swing the puller flat", rot))
    out.append(Waypoint(0.5, grip_tcp(u0, th_goal), GRIP_PINCH, "pinch the puller", rot))
    n = max(2, int(math.ceil(max(0.0, u0 - U_PRE) / 0.01)))
    for k in range(1, n + 1):
        out.append(Waypoint(0.35, grip_tcp(u0 + (U_PRE - u0) * k / n, th_goal), GRIP_PINCH, "draw the slider", rot))
    out.append(Waypoint(0.6, grip_tcp(U_PRE, th_goal), GRIP_OPEN, "let go", rot))
    out.append(Waypoint(0.8, grip_tcp(U_PRE, th_goal, 0.04), GRIP_OPEN, "back off", rot))
    return out


def push_plan(c: str, case_xy, yaw: float, u0: float, fail: str = "") -> list[Waypoint]:
    """Push the slider's back end toward the lock with the side of the closed fingers, past the end stop, until the
    puller's tip has gone in and the catch has dropped; then back off and go back to the ready pose."""
    s, arm, world, rot, n_in = _frame(c, case_xy, yaw)
    rear = SLIDER_FRONT + SLIDER[1] / 2  # the slider's back end from the stop, plus u
    push_to = -OVERPUSH if fail != "short" else 0.005  # "short": the tip part way in, short of the catch

    def push_tcp(u: float, back: float = 0.0):
        # the fingertips 7.5 mm out from the shell: on the slider's back end, the upper finger clear of the shell
        p = np.array([F - 0.0075, s * (LOCK_Y + rear + u + PAD_HALF + 0.001), 0.0])
        return tuple((world(p) - (PAD_REACH + back) * n_in).tolist())

    u1 = max(u0, 0.0) + 0.004
    out = [Waypoint(1.0, push_tcp(u1, 0.04), FINGER_CLOSED, "behind the slider", rot),
           Waypoint(1.0, push_tcp(u1), FINGER_CLOSED, "down to the slider's back end", rot)]
    n = max(2, int(math.ceil((u1 - push_to) / 0.005)))
    for k in range(1, n + 1):
        out.append(Waypoint(0.4, push_tcp(u1 + (push_to - u1) * k / n), FINGER_CLOSED, "push the puller into the lock",
                            rot))
    out.append(Waypoint(1.5, push_tcp(push_to), FINGER_CLOSED, "hold the push", rot))
    if fail == "holding":
        return out
    out.append(Waypoint(0.6, push_tcp(0.008), FINGER_CLOSED, "ease off", rot))
    out.append(Waypoint(1.0, push_tcp(0.008, 0.05), FINGER_CLOSED, "back off", rot))
    out.append(clear_pose(arm, 1.5, FINGER_CLOSED))
    out.append(park(arm, 1.5))
    out.append(park(arm, 0.5, label="look"))
    return out


def side_plan(c: str, case_xy, yaw: float, u0: float, th0: float, fail: str = "") -> list[Waypoint]:
    """Both parts end to end, as planned from one look (for reach checks; the demo looks again between them)."""
    return draw_plan(c, case_xy, yaw, u0, th0, fail) + push_plan(c, case_xy, yaw, U_PRE, fail)


def plan_stage(stage: int) -> dict[str, list[Waypoint]]:
    """The next stretch of the demo, made from the simulator's poses each time the last stretch has ended: for the
    first side not yet done, a draw (again, once, if the tab is not lying flat toward the lock near the lock), then
    a push. FAIL == "fixed" plans every stretch for the nominal start in the setup text, reading nothing."""
    st = _EP.get("st")
    tries = _EP.setdefault("tries", {c: 0 for c in STAGES})
    done = _EP.setdefault("done", set())
    todo = [c for c in STAGES if c not in done]
    if not todo:
        return {}
    c = todo[0]
    other = "right" if ARM[c] == "left" else "left"
    fail = FAIL if FAIL in ("short", "crooked") or (FAIL == "holding" and c == STAGES[-1]) else ""
    if FAIL == "fixed" or st is None:
        lay = layout(0)
        xy, yaw = (CASE0[0], CASE0[1]), 0.0
        u0, th0 = lay[c]
        if tries[c] == 0:
            plan, what = draw_plan(c, xy, yaw, u0, th0, fail), "draw"
            tries[c] += 1
        else:
            plan, what = push_plan(c, xy, yaw, U_PRE, fail), "push"
            done.add(c)
    else:
        case = st["case"]
        xy, yaw = (float(case.pos[0]), float(case.pos[1])), _yaw(case.rotation())
        j = joints(st)[c]
        u0, th0 = j["u"], j["theta"]
        ready = th0 >= FLAT_DEG - (30.0 if fail == "crooked" else 0.0) and u0 <= NEAR_U
        if tries[c] == 0 or (not ready and tries[c] < DRAW_TRIES):
            plan, what = draw_plan(c, xy, yaw, u0, th0, fail, first=tries[c] == 0), "draw"
            tries[c] += 1
        else:
            plan, what = push_plan(c, xy, yaw, u0, fail), "push"
            done.add(c)
    print(f"zipdemo: side {c} {what}: case ({xy[0]:.4f}, {xy[1]:.4f}) yaw {math.degrees(yaw):+.2f} slider "
          f"{u0 * 1000:.1f} mm tab {th0:+.1f} deg", flush=True)
    return {ARM[c]: plan, other: [park(other, sum(w.duration for w in plan), label="wait")]}


import zipdemo  # noqa: E402  (the demo's control hooks for yam_scripted)

zipdemo.install()
zipdemo_task = True  # tells zipdemo this task uses its hooks
